"""AGV pallet-pocket docking controller (pallet detector approach + AprilTag guidance)."""
import math
from sim.api import (Command, TAG_ID, FX, CAM_X, DOCK_DIST, PALLET_WIDTH,
                     V_MAX, W_MAX)

HALF_PI = math.pi / 2.0
WP_Y = 2.0          # base_link y of pre-dock waypoint on the centre line
BASE_GOAL_G = 0.0   # camera distance error goal handled via DOCK_DIST


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class DockingController:
    def __init__(self):
        self.reset('guidance')

    # ------------------------------------------------------------------
    def reset(self, start_stage):
        self.stage = start_stage
        self.t0 = None
        self.prev_odom = None
        self.pose = None            # base_link pose in pallet frame (x, y, h)
        self.last_fix_t = -1e9
        self.st = 'A'
        self.settle = 0
        self.fixes_in_d = 0
        self.tries = 0
        self.blocked = 0
        self.total_blocked = 0
        # acquisition
        self.last_pal_t = -1e9
        self.last_pal_b = 1.0
        self.blacklist = []
        self.giveup_until = -1e9
        self.no_tag_close = 0

    # ------------------------------------------------------------------
    def _predict(self, od):
        if self.prev_odom is not None and self.pose is not None:
            p = self.prev_odom
            dx, dy = od.x - p.x, od.y - p.y
            df = dx * math.cos(p.yaw) + dy * math.sin(p.yaw)
            dl = -dx * math.sin(p.yaw) + dy * math.cos(p.yaw)
            dyaw = wrap(od.yaw - p.yaw)
            x, y, h = self.pose
            hm = h + dyaw / 2.0
            x += df * math.cos(hm) - dl * math.sin(hm)
            y += df * math.sin(hm) + dl * math.cos(hm)
            self.pose = (x, y, wrap(h + dyaw))
        self.prev_odom = od

    def _fix(self, tag, t):
        yaw = tag.yaw
        h = -HALF_PI + yaw
        cx = tag.x * math.cos(yaw) - tag.z * math.sin(yaw)
        cy = tag.z * math.cos(yaw) + tag.x * math.sin(yaw)
        bx = cx - CAM_X * math.cos(h)
        by = cy - CAM_X * math.sin(h)
        if self.pose is None:
            self.pose = (bx, by, wrap(h))
            first = True
        else:
            a = 0.7 if tag.z < 1.6 else 0.5
            x, y, ph = self.pose
            self.pose = (x + a * (bx - x), y + a * (by - y),
                         wrap(ph + a * wrap(h - ph)))
            first = False
        self.last_fix_t = t
        self.fixes_in_d += 1
        return first

    # ------------------------------------------------------------------
    @staticmethod
    def _sector_min(scan, center, half):
        if scan is None or not scan.ranges:
            return 99.0
        m = 99.0
        inc = scan.angle_increment
        for i, r in enumerate(scan.ranges):
            if r is None or r != r or r < 0.05:
                continue
            a = wrap(scan.angle_min + i * inc - center)
            if abs(a) <= half and r < m:
                m = r
        return m

    def _side_open(self, scan):
        """+1 if left side is more open, -1 if right."""
        if scan is None:
            return 1.0
        l = self._sector_min(scan, math.radians(60), math.radians(30))
        r = self._sector_min(scan, math.radians(-60), math.radians(30))
        return 1.0 if l >= r else -1.0

    def _guard(self, v, w, obs):
        """Slow/stop translation near obstacles."""
        if abs(v) < 1e-3:
            self.blocked = 0
            return v, w
        center = 0.0 if v > 0 else math.pi
        r = self._sector_min(obs.scan, center, math.radians(38))
        scale = clip((r - 0.43) / 0.5, 0.0, 1.0)
        if scale <= 0.0:
            self.blocked += 1
            self.total_blocked += 1
            v = 0.0
            if v == 0.0 and self.blocked > 15:
                w = 0.4 * self._side_open(obs.scan) * (-1.0 if center == 0.0 else 1.0)
            return v, w
        self.blocked = 0
        return v * scale, w

    # ------------------------------------------------------------------
    def _best_pallet(self, obs):
        ox, oy, oyaw = obs.odom.x, obs.odom.y, obs.odom.yaw
        best = None
        for p in obs.pallets:
            if p.score < 0.35 or p.width_px < 5:
                continue
            valid = sorted(d for d in p.depth_samples if 0.3 < d < 8.0)
            wd = FX * PALLET_WIDTH / p.width_px
            if len(valid) >= 3:
                low = valid[:(len(valid) + 1) // 2]
                dep = low[len(low) // 2]
                ratio = dep / wd
                if ratio < 0.7 or ratio > 1.3:
                    continue
                rng = dep
                q = p.score * (1.0 - min(0.5, abs(ratio - 1.0)))
            else:
                if p.score < 0.6:
                    continue
                rng = wd
                q = p.score * 0.5
            b = math.atan((p.u - 320.0) / FX)   # + right
            ang = oyaw - b
            wx = ox + (rng + CAM_X) * math.cos(ang)
            wy = oy + (rng + CAM_X) * math.sin(ang)
            if any((wx - bx) ** 2 + (wy - by) ** 2 < 0.8 ** 2 for bx, by in self.blacklist):
                continue
            if best is None or q > best[0]:
                best = (q, b, rng, wx, wy)
        return best

    def _acquire(self, obs):
        t = obs.t
        best = self._best_pallet(obs) if t >= self.giveup_until else None
        if best is not None:
            q, b, rng, wx, wy = best
            self.last_pal_t = t
            self.last_pal_b = b
            front = self._sector_min(obs.scan, 0.0, math.radians(20))
            if rng < 1.2 or front < 0.85:
                self.no_tag_close += 1
            else:
                self.no_tag_close = 0
            if self.no_tag_close > 8:
                # probably a decoy (or tag unreadable): remember and look elsewhere
                self.blacklist.append((wx, wy))
                self.giveup_until = t + 1.0
                self.no_tag_close = 0
                return Command(0.0, 0.4)
            w = -clip(1.5 * b, -0.5, 0.5)
            if abs(b) > 0.35:
                v = 0.0
            else:
                v = 0.25 * (1.0 - 0.5 * abs(b) / 0.35)
            if rng < 2.0:
                v = min(v, 0.15)
            if rng < 1.4:
                v = min(v, 0.08)
            return Command(v, w)
        if t - self.last_pal_t < 1.0:
            return Command(0.1, 0.0)
        sgn = 1.0 if self.last_pal_b <= 0 else -1.0   # turn toward last seen side
        return Command(0.0, 0.45 * sgn)

    # ------------------------------------------------------------------
    def _decide_final(self):
        if self.pose is None:
            return 'ABORTED'
        x, y, h = self.pose
        cx = x + CAM_X * math.cos(h)
        cy = y + CAM_X * math.sin(h)
        herr = wrap(h + HALF_PI)
        if (abs(cx) <= 0.045 and abs(cy - DOCK_DIST) <= 0.09 and abs(herr) <= math.radians(5)
                and self.t_now - self.last_fix_t < 1.5):
            return 'DOCKED'
        return 'ABORTED'

    def step(self, obs):
        t = obs.t
        self.t_now = t
        if self.t0 is None:
            self.t0 = t
        elapsed = t - self.t0
        self._predict(obs.odom)

        tag = None
        for d in obs.tags:
            if d.id == TAG_ID and (tag is None or d.z < tag.z):
                tag = d
        if tag is not None:
            first = self._fix(tag, t)
            if first:
                x, y, h = self.pose
                self.st = 'B' if (abs(x) < 0.12 and y > 1.7) else 'A'

        limit = 112.0 if self.stage == 'guidance' else 142.0
        if elapsed > limit:
            return Command(0.0, 0.0, self._decide_final())

        if self.pose is None:
            v, w = 0.0, 0.0
            c = self._acquire(obs)
            v, w = self._guard(c.v, c.w, obs)
            if self.total_blocked > 300:
                return Command(0.0, 0.0, 'ABORTED')
            return Command(v, w)

        x, y, h = self.pose
        cx = x + CAM_X * math.cos(h)
        cy = y + CAM_X * math.sin(h)
        e = cx
        g = cy - DOCK_DIST
        herr = wrap(h + HALF_PI)
        v, w = 0.0, 0.0
        st = self.st

        if st == 'A':
            wx, wy = 0.0, WP_Y
            d = math.hypot(wx - x, wy - y)
            if d < 0.12:
                st = 'B'
            else:
                err = wrap(math.atan2(wy - y, wx - x) - h)
                if abs(err) > 0.5:
                    v, w = 0.0, clip(1.5 * err, -W_MAX, W_MAX)
                else:
                    v = min(0.25, 0.5 * d + 0.05) * math.cos(err)
                    w = clip(1.8 * err, -W_MAX, W_MAX)
        if st == 'B':
            err = wrap(-HALF_PI - h)
            if abs(x) > 0.3 or y < 1.5:
                st = 'R' if y < 1.8 else 'A'
            elif abs(err) < 0.05:
                st = 'C'
            else:
                w = clip(1.5 * err, -W_MAX, W_MAX)
                if abs(w) < 0.15:
                    w = 0.15 * (1 if w >= 0 else -1)
                v = 0.0
        if st == 'C':
            if abs(e) > 0.3 or abs(herr) > math.radians(40):
                st = 'R' if y < 1.8 else 'A'
            elif g <= 0.01:
                st = 'D'
                self.settle = 0
                self.fixes_in_d = 0
            else:
                delta = clip(math.atan(2.5 * e), -0.5, 0.5)
                herr_c = wrap(-HALF_PI - delta - h)
                w = clip(2.0 * herr_c, -0.5, 0.5)
                v = clip(0.6 * g, 0.05, 0.22)
                if abs(herr_c) > 0.2:
                    v *= 0.4
        if st == 'D':
            self.settle += 1
            v, w = 0.0, 0.0
            if self.settle >= 6 and (self.fixes_in_d >= 2 or self.settle >= 20):
                ok_pos = abs(g) <= 0.06
                ok_e = abs(e) <= 0.035
                ok_h = abs(herr) <= math.radians(3.5)
                if ok_pos and ok_e and ok_h:
                    self.st = 'D'
                    return Command(0.0, 0.0, 'DOCKED')
                self.tries += 1
                loose = (abs(e) <= 0.05 and abs(herr) <= math.radians(5.0)
                         and abs(g) <= 0.09)
                if self.tries > 3 and loose:
                    return Command(0.0, 0.0, 'DOCKED')
                if ok_pos and ok_e:
                    st = 'H'
                elif g > 0.06 and ok_e and ok_h:
                    st = 'C'
                else:
                    st = 'R'
                self.settle = 0
                self.fixes_in_d = 0
        if st == 'H':
            err = wrap(-HALF_PI - h)
            if abs(err) < 0.02:
                st = 'D'
                self.settle = 0
                self.fixes_in_d = 0
                v, w = 0.0, 0.0
            else:
                w = clip(2.0 * err, -0.4, 0.4)
                if abs(w) < 0.1:
                    w = 0.1 * (1 if w >= 0 else -1)
        if st == 'R':
            if y >= 1.95:
                st = 'B'
                v, w = 0.0, 0.0
            else:
                delta = clip(math.atan(2.5 * e), -0.4, 0.4)
                err = wrap(-HALF_PI + delta - h)
                w = clip(1.5 * err, -0.4, 0.4)
                v = -0.12

        self.st = st
        v, w = self._guard(v, w, obs)
        if self.total_blocked > 300:
            return Command(0.0, 0.0, 'ABORTED')
        return Command(clip(v, -V_MAX, V_MAX), clip(w, -W_MAX, W_MAX))
