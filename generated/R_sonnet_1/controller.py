"""AGV pallet-pocket docking controller (A3 approach, A4 tag localisation, A5 guidance)."""
import math

from sim.api import (Command, TAG_ID, FX, CAM_X, DOCK_DIST, V_MAX, W_MAX,
                     PALLET_WIDTH)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# docking tolerances used for the stop decision (tighter than R-01)
LAT_OK = 0.035
E_OK = 0.05
DN_OK = 0.03


class DockingController:
    def __init__(self):
        self.reset('guidance')

    # ------------------------------------------------------------------
    def reset(self, start_stage):
        self.start_stage = start_stage
        self.t0 = None
        self.limit = 150.0 if start_stage == 'approach' else 120.0
        self.top = 'approach' if start_stage == 'approach' else 'guide'
        # tag estimate in odom frame
        self.T = None            # (x, y)
        self.phi = 0.0           # inward normal angle
        self.last_tag_t = -1e9
        self.n_tag = 0
        # approach
        self.sub = 'search'
        self.P = None
        self.last_pallet_t = -1e9
        self.black = []
        self.wait_t0 = None
        self.stop_rng = 2.0
        self.search_dir = 1.0
        # guide
        self.gmode = 'decide'
        self.S = None
        self.hold_n = 0
        self.hold_fresh = 0
        self.guide_wait_t0 = None
        self.blocked_t = 0.0

    # ------------------------------------------------------------------
    def _cam(self, o):
        return (o.x + CAM_X * math.cos(o.yaw), o.y + CAM_X * math.sin(o.yaw))

    def _update_tag(self, obs):
        o = obs.odom
        det = None
        for d in obs.tags:
            if d.id == TAG_ID and d.z > 0.2:
                if det is None or d.z < det.z:
                    det = d
        if det is None:
            return False
        h = o.yaw
        cx, cy = self._cam(o)
        fx, fy = math.cos(h), math.sin(h)
        rx, ry = math.sin(h), -math.cos(h)
        tx = cx + det.z * fx + det.x * rx
        ty = cy + det.z * fy + det.x * ry
        phi = wrap(h - det.yaw)
        if self.T is None or math.hypot(tx - self.T[0], ty - self.T[1]) > 0.8:
            self.T = (tx, ty)
            self.phi = phi
            self.n_tag = 1
        else:
            a = 0.25 if self.hold_n > 0 else 0.4
            self.T = (self.T[0] + a * (tx - self.T[0]), self.T[1] + a * (ty - self.T[1]))
            self.phi = wrap(self.phi + a * wrap(phi - self.phi))
            self.n_tag += 1
        self.last_tag_t = obs.t
        return True

    # ------------------------------------------------------------------
    # lidar helpers
    def _forward_gap(self, scan, direction):
        """Free distance (beyond the 0.30 m footprint) in the travel direction."""
        if scan is None:
            return 9.9
        gap = 9.9
        a = scan.angle_min
        for r in scan.ranges:
            if r is not None and 0.05 < r < 3.0:
                x = r * math.cos(a)
                y = r * math.sin(a)
                x *= direction
                if x > 0.0 and abs(y) < 0.38:
                    g = x - 0.30
                    if g < gap:
                        gap = g
            a += scan.angle_increment
        return gap

    def _side_pressure(self, scan):
        pl = pr = 0.0
        if scan is None:
            return 0.0, 0.0
        a = scan.angle_min
        lim = math.radians(75)
        for r in scan.ranges:
            if r is not None and 0.05 < r < 1.3 and abs(a) < lim:
                if a > 0:
                    pl += 1.0 / r
                else:
                    pr += 1.0 / r
            a += scan.angle_increment
        return pl, pr

    def _safety(self, obs, v, w, avoid=True):
        scan = obs.scan
        if scan is None or abs(v) < 1e-6:
            return v, w
        d = 1.0 if v > 0 else -1.0
        gap = self._forward_gap(scan, d)
        if gap < 0.12:
            v = 0.0
            self.blocked_t += 0.1
            if avoid and d > 0 and abs(w) < 0.2:
                pl, pr = self._side_pressure(scan)
                w = 0.4 if pl < pr else -0.4
        elif gap < 0.5:
            v *= (gap - 0.12) / 0.38
            if avoid and d > 0:
                pl, pr = self._side_pressure(scan)
                w += -0.4 * (pl - pr) / (pl + pr + 1e-6) * (0.5 - gap) / 0.38
            self.blocked_t = max(0.0, self.blocked_t - 0.05)
        else:
            self.blocked_t = max(0.0, self.blocked_t - 0.05)
        return v, w

    # ------------------------------------------------------------------
    def step(self, obs):
        if self.t0 is None:
            self.t0 = obs.t
        el = obs.t - self.t0
        got_tag = self._update_tag(obs)

        if el > self.limit - 2.0:
            return Command(0.0, 0.0, 'ABORTED')

        if self.top == 'approach' and self.T is not None:
            self.top = 'guide'
            self.gmode = 'decide'

        if self.top == 'guide' and self.T is None:
            # waiting for first tag detection in guidance start
            if self.guide_wait_t0 is None:
                self.guide_wait_t0 = obs.t
            if obs.t - self.guide_wait_t0 > 2.5:
                self.top = 'approach'
                self.sub = 'search'
            else:
                return Command(0.0, 0.0)

        if self.top == 'approach':
            v, w = self._approach(obs)
            v, w = self._safety(obs, v, w)
            return Command(clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX))

        return self._guide(obs, el)

    # ------------------------------------------------------------------
    # A3: pallet search and approach
    def _candidates(self, obs):
        o = obs.odom
        cx, cy = self._cam(o)
        out = []
        for p in obs.pallets:
            if p.score < 0.35:
                continue
            vals = sorted(d for d in p.depth_samples if 0.3 < d < 9.0)
            r_w = FX * PALLET_WIDTH / max(p.width_px, 1.0)
            if len(vals) >= 3:
                r = vals[len(vals) // 4]
                phys = r * p.width_px / FX
                err = abs(phys - PALLET_WIDTH)
                if err > 0.4:
                    continue
            else:
                r = r_w
                err = 0.3
            th = -math.atan((p.u - 320.0) / FX)
            ang = o.yaw + th
            px = cx + r * math.cos(ang)
            py = cy + r * math.sin(ang)
            if any(math.hypot(px - bx, py - by) < 1.0 for bx, by in self.black):
                continue
            out.append((px, py, p.score - err))
        return out

    def _approach(self, obs):
        o = obs.odom
        cands = self._candidates(obs)
        if cands:
            if self.P is not None:
                near = [c for c in cands
                        if math.hypot(c[0] - self.P[0], c[1] - self.P[1]) < 1.3]
                best = min(near, key=lambda c: math.hypot(c[0] - self.P[0], c[1] - self.P[1])) if near else None
                if best is not None:
                    self.P = (0.6 * self.P[0] + 0.4 * best[0], 0.6 * self.P[1] + 0.4 * best[1])
                    self.last_pallet_t = obs.t
            else:
                best = max(cands, key=lambda c: c[2])
                self.P = (best[0], best[1])
                self.last_pallet_t = obs.t
                self.sub = 'track'
        if self.P is None:
            self.sub = 'search'
            return 0.0, 0.5 * self.search_dir
        cx, cy = self._cam(o)
        dx, dy = self.P[0] - cx, self.P[1] - cy
        rng = math.hypot(dx, dy)
        beta = wrap(math.atan2(dy, dx) - o.yaw)
        if self.sub in ('track', 'search'):
            self.sub = 'track'
            if obs.t - self.last_pallet_t > 4.0 and abs(beta) < 0.5 and rng > self.stop_rng:
                self.P = None
                self.sub = 'search'
                return 0.0, 0.5 * self.search_dir
            if rng <= self.stop_rng:
                self.sub = 'wait'
                self.wait_t0 = obs.t
                return 0.0, 0.0
            if abs(beta) > 0.35:
                return 0.0, clamp(1.5 * beta, -0.5, 0.5)
            v = clamp(0.6 * (rng - self.stop_rng) + 0.05, 0.05, V_MAX)
            return v, clamp(1.5 * beta, -0.5, 0.5)
        # wait for the tag
        if obs.t - self.wait_t0 > 3.0:
            if self.stop_rng > 1.4:
                self.stop_rng -= 0.3
                self.sub = 'track'
            else:
                self.black.append(self.P)
                self.P = None
                self.sub = 'search'
                self.stop_rng = 2.0
            return 0.0, 0.0
        # keep the pallet centred while waiting
        if abs(beta) > 0.1:
            return 0.0, clamp(1.2 * beta, -0.3, 0.3)
        return 0.0, 0.0

    # ------------------------------------------------------------------
    # A4/A5: tag-based guidance
    def _guide(self, obs, el):
        o = obs.odom
        cx, cy = self._cam(o)
        nx, ny = math.cos(self.phi), math.sin(self.phi)
        rx, ry = cx - self.T[0], cy - self.T[1]
        dn = -(rx * nx + ry * ny)
        lat = -rx * ny + ry * nx
        e = wrap(o.yaw - self.phi)
        m = self.gmode

        if m == 'decide':
            if abs(lat) > 0.45 or abs(e) > 0.7:
                m = 'stage'
                self._set_stage(dn)
            else:
                m = 'final'

        if m == 'hold':
            ok = (abs(lat) <= LAT_OK + 0.01 and abs(e) <= E_OK + 0.02
                  and abs(dn - DOCK_DIST) <= DN_OK + 0.01)
            if not ok:
                m = 'final'
                self.hold_n = 0
                self.hold_fresh = 0
            else:
                self.hold_n += 1
                if obs.t - self.last_tag_t < 0.15:
                    self.hold_fresh += 1
                still = abs(o.v) < 0.02 and abs(o.w) < 0.03
                if self.hold_n >= 5 and still and (self.hold_fresh >= 2 or self.hold_n > 30):
                    self.gmode = 'hold'
                    return Command(0.0, 0.0, 'DOCKED')
                self.gmode = 'hold'
                return Command(0.0, 0.0)

        if m == 'stage':
            bx, by = o.x, o.y
            dx, dy = self.S[0] - bx, self.S[1] - by
            dist = math.hypot(dx, dy)
            beta = wrap(math.atan2(dy, dx) - o.yaw)
            if dist < 0.12:
                m = 'align'
            else:
                if abs(beta) > 0.6:
                    v, w = 0.0, clamp(1.5 * beta, -W_MAX, W_MAX)
                else:
                    v = min(V_MAX, 0.6 * dist + 0.05) * max(0.0, math.cos(beta))
                    w = clamp(2.0 * beta, -W_MAX, W_MAX)
                v, w = self._safety(obs, v, w)
                self.gmode = m
                return Command(v, w)

        if m == 'align':
            if abs(e) < 0.035:
                m = 'final'
            else:
                w = -clamp(1.2 * e, -W_MAX, W_MAX)
                if abs(w) < 0.08:
                    w = 0.08 * (1 if w > 0 else -1)
                self.gmode = m
                return Command(0.0, w)

        if m == 'retreat':
            if dn >= 1.35:
                m = 'align' if abs(e) > 0.1 else 'final'
            else:
                v, w = self._safety(obs, -0.12, 0.0)
                self.gmode = m
                return Command(v, w)

        # ---- final approach ----
        m = 'final'
        if (abs(lat) > 0.55 or abs(e) > 0.95) and dn > 1.1:
            self.gmode = 'stage'
            self._set_stage(dn)
            return Command(0.0, 0.0)
        dn_err = dn - DOCK_DIST
        if dn_err <= DN_OK:
            if abs(lat) <= LAT_OK and abs(e) <= E_OK and abs(dn_err) <= DN_OK:
                self.gmode = 'hold'
                self.hold_n = 0
                self.hold_fresh = 0
                return Command(0.0, 0.0)
            if dn_err < -DN_OK or abs(lat) > LAT_OK:
                self.gmode = 'retreat'
                return Command(0.0, 0.0)
            self.gmode = 'align'
            return Command(0.0, 0.0)
        k = 3.0 if dn > 1.4 else 1.5
        delta = clamp(math.atan(k * lat), -0.6, 0.6)
        e_h = e + delta
        w = clamp(-2.0 * e_h, -W_MAX, W_MAX)
        if abs(e_h) > 0.9:
            v = 0.0
        else:
            v = clamp(0.6 * dn_err, 0.05, V_MAX) * max(0.2, math.cos(e_h))
            if dn_err < 0.15:
                v = min(v, 0.06)
        v, w = self._safety(obs, v, w)
        self.gmode = 'final'
        return Command(v, w)

    def _set_stage(self, dn):
        dns = clamp(dn, 1.6, 2.4)
        nx, ny = math.cos(self.phi), math.sin(self.phi)
        d = dns + CAM_X
        self.S = (self.T[0] - nx * d, self.T[1] - ny * d)
