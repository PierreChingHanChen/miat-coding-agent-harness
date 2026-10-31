"""AGV pallet-pocket docking controller (A3 approach / A4 localize / A5 guide).

State machine
  SEARCH   rotate, look for pallet candidates (detector)
  TRACK    drive toward best candidate until the AprilTag is seen; if none is
           found near it, blacklist it and search again
  GOTO     reposition to a staging point S on the pocket centre line
           (forward, or reversing when S is behind), tag may be out of view
  FACE     rotate to face the pocket and wait for a tag fix
  CONVERGE Stanley-type lateral law toward the docking pose
  VERIFY   stopped; DOCKED only on fresh, averaged tag measurements
The pose of base_link in the pallet frame is dead-reckoned from odometry and
corrected by every accepted tag detection, so tag drop-outs are tolerated.
Frame F: e = lateral (east), d = distance from pocket face, b = bearing
(clockwise from the inward normal, = -tag yaw).
"""
import math
from sim.api import (Command, TAG_ID, CAM_X, FX, PALLET_WIDTH, DOCK_DIST,
                     V_MAX, W_MAX)

PI = math.pi
INF = float("inf")


def wrap(a):
    return (a + PI) % (2 * PI) - PI


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class DockingController:
    S_DIST = 1.95          # staging point, base_link distance from face [m]

    def reset(self, start_stage):
        self.stage = start_stage
        self.limit = 114.0 if start_stage == 'guidance' else 144.0
        self.t0 = None
        self.prev = None
        self.have_pose = False
        self.pe = self.pd = self.pb = 0.0
        self.fixes = []
        self.last_fix_t = -1e9
        self.gate_rej = 0
        self.mode = 'SEARCH'
        self.mode_t = 0.0
        self.target = None          # dict(x, y, seen)
        self.blacklist = []
        self.rot_acc = 0.0
        self.blocked = 0
        self.escape_until = -1.0
        self.escape_dir = 1.0
        self.goto_n = 0
        self.rev = False
        self.verify_fail = 0
        self.face_aligned_t = None
        self.hold_t = None
        self.last_t = 0.0
        self.close_t = -1e9
        self.inhib_t0 = None
        return None

    # ------------------------------------------------------------------ util
    def _set(self, mode, t):
        self.mode = mode
        self.mode_t = t

    def _cam(self):
        ec = self.pe + CAM_X * math.sin(self.pb)
        dc = self.pd - CAM_X * math.cos(self.pb)
        return ec, dc

    def _odom(self, o):
        if self.prev is not None:
            px, py, pyaw = self.prev
            dx, dy = o.x - px, o.y - py
            dyaw = wrap(o.yaw - pyaw)
            th = pyaw + 0.5 * dyaw
            df = dx * math.cos(th) + dy * math.sin(th)
            dl = -dx * math.sin(th) + dy * math.cos(th)
            self.rot_acc += abs(dyaw)
            if self.have_pose:
                bm = self.pb - 0.5 * dyaw
                self.pe += df * math.sin(bm) - dl * math.cos(bm)
                self.pd -= df * math.cos(bm) + dl * math.sin(bm)
                self.pb = wrap(self.pb - dyaw)
        self.prev = (o.x, o.y, o.yaw)

    def _tag(self, obs, t):
        best = None
        for tg in obs.tags:
            if tg.id == TAG_ID and tg.z > 0.3:
                if best is None or tg.z < best.z:
                    best = tg
        if best is None:
            return False
        yaw = best.yaw
        ce = best.z * math.sin(yaw) - best.x * math.cos(yaw)
        cd = best.x * math.sin(yaw) + best.z * math.cos(yaw)
        if cd < 0.3 or abs(yaw) > 1.2 or math.atan2(abs(ce), cd) > 1.15:
            return False
        bt = -yaw
        be = ce - CAM_X * math.sin(bt)
        bd = cd + CAM_X * math.cos(bt)
        if self.have_pose:
            far = math.hypot(be - self.pe, bd - self.pd) > 0.8 or \
                abs(wrap(bt - self.pb)) > 0.8
            if far and self.gate_rej < 5:
                self.gate_rej += 1
                return False
            if far:
                self.pe, self.pd, self.pb = be, bd, bt
            else:
                a = 0.5
                self.pe += a * (be - self.pe)
                self.pd += a * (bd - self.pd)
                self.pb = wrap(self.pb + a * wrap(bt - self.pb))
        else:
            self.pe, self.pd, self.pb = be, bd, bt
        self.gate_rej = 0
        self.have_pose = True
        self.last_fix_t = t
        self.fixes.append((t, ce, cd, yaw))
        self.fixes = [f for f in self.fixes if t - f[0] < 1.6][-10:]
        return True

    # ------------------------------------------------------------- lidar
    def _corridor(self, scan, forward):
        if scan is None:
            return INF
        m = INF
        a0, inc = scan.angle_min, scan.angle_increment
        rmax = scan.range_max
        for i, r in enumerate(scan.ranges):
            if not (r == r) or r >= rmax or r < 0.05:
                continue
            a = a0 + i * inc
            if not forward:
                a = wrap(a + PI)
            c = math.cos(a)
            if c <= 0.05:
                continue
            if abs(r * math.sin(a)) < 0.40:
                m = min(m, r * c)
        return m

    def _side_clear(self, scan):
        if scan is None:
            return 0.0, 0.0
        L = []
        R = []
        a0, inc = scan.angle_min, scan.angle_increment
        for i, r in enumerate(scan.ranges):
            a = a0 + i * inc
            rr = min(r, 5.0) if r == r else 5.0
            if 0.5 < a < 1.75:
                L.append(rr)
            elif -1.75 < a < -0.5:
                R.append(rr)
        return (sum(L) / max(1, len(L)), sum(R) / max(1, len(R)))

    def _guard(self, v, w, obs, t):
        """Obstacle protection on the commanded translation."""
        if t < self.escape_until:
            self.blocked = 0
            return 0.0, 0.4 * self.escape_dir
        if abs(v) < 1e-3:
            self.blocked = max(0, self.blocked - 1)
            return v, w
        fwd = v > 0
        m = self._corridor(obs.scan, fwd)
        if m < 0.48:
            vv = 0.0
        elif m < 0.9:
            vv = v * (m - 0.48) / 0.42
        else:
            vv = v
        if abs(v) > 0.05 and abs(vv) < 0.02:
            self.blocked += 1
            if self.blocked > 12:
                l, r = self._side_clear(obs.scan)
                self.escape_dir = 1.0 if l >= r else -1.0
                self.escape_until = t + 2.5
                self.blocked = 0
        else:
            self.blocked = max(0, self.blocked - 1)
        return vv, w

    # --------------------------------------------------------- detections
    def _candidates(self, obs):
        out = []
        o = obs.odom
        for p in obs.pallets:
            if p.score < 0.4 or p.width_px < 5:
                continue
            valid = sorted(s for s in p.depth_samples if 0.3 < s < 9.0)
            zw = FX * PALLET_WIDTH * 0.85 / p.width_px
            if len(valid) >= 3:
                z = valid[len(valid) // 4]
            elif valid:
                z = valid[0]
            else:
                z = zw
            cos_est = p.width_px * z / (FX * PALLET_WIDTH)
            ok = 0.5 <= cos_est <= 1.15
            lat_r = z * (p.u - 320.0) / FX
            f = CAM_X + z
            l = -lat_r
            th = o.yaw
            x = o.x + f * math.cos(th) - l * math.sin(th)
            y = o.y + f * math.sin(th) + l * math.cos(th)
            key = p.score + (0.35 if ok else 0.0)
            out.append((key, x, y, z))
        return out

    def _blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < 0.8:
                return True
        return False

    # ----------------------------------------------------------- behaviours
    @staticmethod
    def _turn(eps):
        """w for heading error eps (bearing error, + = need clockwise)."""
        w = -clamp(1.8 * eps, -W_MAX, W_MAX)
        if abs(eps) > 0.02 and abs(w) < 0.12:
            w = 0.12 if w > 0 else -0.12
        return w

    def _search(self, obs, t):
        o = obs.odom
        cands = [c for c in self._candidates(obs)
                 if not self._blacklisted(c[1], c[2])]
        if cands:
            cands.sort(reverse=True)
            _, x, y, z = cands[0]
            self.target = {'x': x, 'y': y, 'seen': t}
            self._set('TRACK', t)
            self.hold_t = None
            return self._track(obs, t)
        if self.rot_acc > 2 * PI * 1.1:
            self.rot_acc = 0.0
            self.blacklist = []
        return 0.0, 0.5

    def _track(self, obs, t):
        o = obs.odom
        tg = self.target
        best = None
        for key, x, y, z in self._candidates(obs):
            dd = math.hypot(x - tg['x'], y - tg['y'])
            if dd < 1.0 and (best is None or dd < best[0]):
                best = (dd, x, y)
        if best is not None:
            tg['x'] += 0.5 * (best[1] - tg['x'])
            tg['y'] += 0.5 * (best[2] - tg['y'])
            tg['seen'] = t
        elif t - tg['seen'] > 1.5:
            self.target = None
            self._set('SEARCH', t)
            return 0.0, 0.5
        dx, dy = tg['x'] - o.x, tg['y'] - o.y
        dist = math.hypot(dx, dy)
        a = wrap(math.atan2(dy, dx) - o.yaw)       # CCW positive
        zc = dist - CAM_X
        if zc <= 1.55:
            if self.hold_t is None:
                self.hold_t = t
            if t - self.hold_t > 1.2:
                self.blacklist.append((tg['x'], tg['y']))
                self.target = None
                self.hold_t = None
                self._set('SEARCH', t)
                return 0.0, 0.5
            return 0.0, clamp(1.5 * a, -0.3, 0.3)
        self.hold_t = None
        w = clamp(1.5 * a, -W_MAX, W_MAX)
        if abs(a) > 0.4:
            v = 0.0
            if abs(w) < 0.15:
                w = 0.15 if w > 0 else -0.15
        else:
            v = 0.26 * max(0.2, math.cos(a))
            if zc < 2.2:
                v = min(v, 0.18)
        return v, w

    def _start_goto(self, t):
        self.goto_n += 1
        self._set('GOTO', t)
        self.rev = False

    def _choose(self, t):
        ec, dc = self._cam()
        b = self.pb
        if self.goto_n >= 4 and abs(ec) < 0.4 and abs(b) < 0.7 and dc > 1.0:
            self._set('CONVERGE', t)
        elif abs(ec) > 0.25 or abs(b) > 0.5:
            self._start_goto(t)
        elif dc < 1.2 and (abs(ec) > 0.08 or abs(b) > 0.15):
            self._start_goto(t)
        else:
            self._set('CONVERGE', t)

    def _goto(self, obs, t):
        de = -self.pe
        dn = self.pd - self.S_DIST
        dist = math.hypot(de, dn)
        if dist < 0.12:
            self._set('FACE', t)
            self.face_aligned_t = None
            return 0.0, 0.0
        bt = math.atan2(de, dn)
        eps = wrap(bt - self.pb)
        # reversing option when the target is behind and the rear is free
        if self.rev:
            if abs(eps) < 1.9:
                self.rev = False
        elif abs(eps) > 2.2 and self._corridor(obs.scan, False) > 1.0:
            self.rev = True
        if self.rev:
            er = wrap(eps - PI)
            w = self._turn(er)
            if abs(er) > 0.35:
                v = 0.0
            else:
                v = -clamp(0.5 * dist, 0.05, 0.25) * max(0.2, math.cos(er))
            return v, w
        w = self._turn(eps)
        if abs(eps) > 0.4:
            v = 0.0
        else:
            v = clamp(0.5 * dist, 0.05, 0.28) * max(0.2, math.cos(eps))
        return v, w

    def _face(self, obs, t):
        eps = wrap(0.0 - self.pb)
        if abs(eps) > 0.04 and self.face_aligned_t is None:
            if t - self.mode_t > 9.0:
                self.face_aligned_t = t
            return 0.0, self._turn(eps)
        if self.face_aligned_t is None:
            self.face_aligned_t = t
        if self.last_fix_t >= self.face_aligned_t:
            self._choose(t)
            return 0.0, 0.0
        if t - self.face_aligned_t > 1.5:
            # nothing seen: tag lost -> re-find with the pallet detector
            self.have_pose = False
            self.blacklist = []
            self.target = None
            self._set('SEARCH', t)
        return 0.0, 0.0

    def _converge(self, obs, t):
        ec, dc = self._cam()
        b = self.pb
        if (dc < 1.25 and abs(ec) > 0.12) or abs(b) > 0.8 or \
                (abs(ec) > 0.3 and dc > 1.4):
            self._start_goto(t)
            return 0.0, 0.0
        if dc - DOCK_DIST <= 0.015:
            self._set('VERIFY', t)
            return 0.0, 0.0
        b_des = -clamp(math.atan(3.0 * ec), -0.45, 0.45)
        eb = wrap(b_des - b)
        w = -clamp(1.6 * eb, -W_MAX, W_MAX)
        v = clamp(0.7 * (dc - DOCK_DIST) + 0.04, 0.04, 0.25)
        v *= max(0.0, 1.0 - abs(eb) / 0.5)
        if dc < 1.2 and abs(ec) > 0.05:
            v *= 0.5
        return v, w

    def _verify(self, obs, t):
        o = obs.odom
        stopped = abs(o.v) < 0.02 and abs(o.w) < 0.03
        fresh = [f for f in self.fixes
                 if f[0] >= self.mode_t + 0.15 and t - f[0] <= 1.0]
        if len(fresh) >= 3 and stopped:
            n = float(len(fresh))
            ce = sum(f[1] for f in fresh) / n
            cd = sum(f[2] for f in fresh) / n
            yw = sum(f[3] for f in fresh) / n
            if abs(ce) <= 0.035 and abs(yw) <= 0.06 and \
                    abs(cd - DOCK_DIST) <= 0.06:
                return 'DOCKED'
            self.verify_fail += 1
            if abs(ce) <= 0.035 and abs(yw) <= 0.06 and cd > DOCK_DIST + 0.06 \
                    and cd < 1.3:
                self._set('CONVERGE', t)
            else:
                self._start_goto(t)
            return 0.0, 0.0
        if t - self.mode_t > 2.5:
            self.verify_fail += 1
            if self.verify_fail >= 3:
                self.have_pose = False
                self.blacklist = []
                self.target = None
                self.verify_fail = 0
                self._set('SEARCH', t)
            else:
                self._start_goto(t)
        return 0.0, 0.0

    # ----------------------------------------------------------------- main
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            t = getattr(obs, 't', self.last_t)
            if self.t0 is not None and t - self.t0 > self.limit:
                return Command(0.0, 0.0, 'ABORTED')
            return Command(0.0, 0.0, 'RUNNING')

    def _step(self, obs):
        t = obs.t
        self.last_t = t
        if self.t0 is None:
            self.t0 = t
            self.mode_t = t
        if t - self.t0 > self.limit:
            return Command(0.0, 0.0, 'ABORTED')
        self._odom(obs.odom)
        got = self._tag(obs, t)
        if got and self.mode in ('SEARCH', 'TRACK'):
            self.blacklist = []
            self.target = None
            self.goto_n = 0
            self.verify_fail = 0
            self._choose(t)

        m = self.mode
        if m == 'SEARCH':
            v, w = self._search(obs, t)
        elif m == 'TRACK':
            v, w = self._track(obs, t)
        elif m == 'GOTO':
            v, w = self._goto(obs, t)
        elif m == 'FACE':
            v, w = self._face(obs, t)
        elif m == 'CONVERGE':
            v, w = self._converge(obs, t)
        elif m == 'VERIFY':
            r = self._verify(obs, t)
            if r == 'DOCKED':
                return Command(0.0, 0.0, 'DOCKED')
            v, w = r
        else:
            v, w = 0.0, 0.0
        v, w = self._guard(v, w, obs, t)
        # R-10 safety supervisor (last stage, overrides everything)
        sc = obs.scan
        if sc is not None:
            for i, r in enumerate(sc.ranges):
                if r == r and r < 0.55 and r != INF and \
                        abs(wrap(sc.angle_min + i * sc.angle_increment)) <= math.radians(30.0):
                    self.close_t = t
                    break
        if t - self.close_t < 1.0:
            if self.inhib_t0 is None:
                self.inhib_t0 = t
            if t - self.inhib_t0 >= 20.0:
                return Command(0.0, 0.0, 'ABORTED')
            v = min(v, 0.0)
        else:
            self.inhib_t0 = None
        return Command(clamp(v, -V_MAX, V_MAX), clamp(w, -W_MAX, W_MAX),
                       'RUNNING')
