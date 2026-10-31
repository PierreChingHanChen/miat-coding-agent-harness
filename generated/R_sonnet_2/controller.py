"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guide)."""
import math
from sim.api import (Command, FX, CAM_X, TAG_ID, DOCK_DIST, V_MAX, W_MAX,
                     PALLET_WIDTH, DT)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


D_GOAL = DOCK_DIST + CAM_X      # base distance from face at docking (1.13)
D_MIN = D_GOAL - 0.08           # never creep deeper than this when tuning
APPROACH_DEPTH = 2.3            # camera-to-pallet depth at which approach stops


class DockingController:
    def reset(self, start_stage):
        self.stage = start_stage
        self.mode = 'GUID' if start_stage == 'guidance' else 'SEARCH'
        self.limit = 120.0 if start_stage == 'guidance' else 150.0
        self.t0 = None
        self.est = None            # [L, d, psi] of base_link in pocket frame
        self.prev_odom = None
        self.last_tag_stamp = None
        self.last_tag_t = -1e9
        self.samples = []
        self.hold_t0 = None
        self.reverse = False
        self.blocked_t = 0.0
        self.last_cmd = (0.0, 0.0)
        # approach state
        self.rej = []              # rejected world bearings
        self.track_u = None
        self.lost_t = 0.0
        self.stop_t = 0.0
        self.search_rot = 0.0
        self.pal_depth = None
        self.wait_t = 0.0

    # ------------------------------------------------------------------
    def step(self, obs):
        t = obs.t
        if self.t0 is None:
            self.t0 = t
        el = t - self.t0
        o = obs.odom
        self._dead_reckon(o)
        fresh = self._ingest_tag(obs)

        if el > self.limit - 3.0:
            if self.est is not None and self._near(self.est, 0.06, 0.1, 0.1):
                return Command(0.0, 0.0, 'DOCKED')
            return Command(0.0, 0.0, 'ABORTED')

        if self.est is not None and self.mode != 'GUID':
            self.mode = 'GUID'
        if self.mode == 'GUID':
            cmd = self._guidance(obs, el, fresh)
        else:
            cmd = self._approach(obs, el)
        if cmd.status == 'RUNNING':
            cmd = self._safety(obs, cmd)
        self.last_cmd = (cmd.v, cmd.w)
        return cmd

    # ------------------------------------------------------------------
    def _dead_reckon(self, o):
        if self.prev_odom is not None and self.est is not None:
            px, py, pyaw = self.prev_odom
            dpsi = _wrap(o.yaw - pyaw)
            ym = pyaw + dpsi / 2
            ds = (o.x - px) * math.cos(ym) + (o.y - py) * math.sin(ym)
            L, d, psi = self.est
            pm = psi + dpsi / 2
            L += -ds * math.sin(pm)
            d -= ds * math.cos(pm)
            psi = _wrap(psi + dpsi)
            self.est = [L, d, psi]
        self.prev_odom = (o.x, o.y, o.yaw)

    @staticmethod
    def _tag_pose(tg):
        yaw = tg.yaw
        Lc = tg.z * math.sin(yaw) - tg.x * math.cos(yaw)
        dc = tg.x * math.sin(yaw) + tg.z * math.cos(yaw)
        return Lc, dc, yaw

    def _ingest_tag(self, obs):
        best = None
        for tg in obs.tags:
            if tg.id != TAG_ID or tg.z < 0.3 or abs(tg.yaw) > 1.3:
                continue
            if best is None or tg.z < best.z:
                best = tg
        if best is None:
            return False
        if self.last_tag_stamp is not None and best.stamp <= self.last_tag_stamp:
            return False
        self.last_tag_stamp = best.stamp
        self.last_tag_t = obs.t
        Lc, dc, yaw = self._tag_pose(best)
        meas = [Lc + CAM_X * math.sin(yaw), dc + CAM_X * math.cos(yaw), yaw]
        if self.est is None:
            self.est = meas
        else:
            a = 0.6
            e = self.est
            self.est = [e[0] + a * (meas[0] - e[0]),
                        e[1] + a * (meas[1] - e[1]),
                        e[2] + a * _wrap(meas[2] - e[2])]
        self.samples.append((Lc, dc, yaw))
        if len(self.samples) > 8:
            self.samples.pop(0)
        return True

    @staticmethod
    def _cam(est):
        L, d, psi = est
        return L - CAM_X * math.sin(psi), d - CAM_X * math.cos(psi), psi

    def _near(self, est, tl, td, ta):
        Lc, dc, psi = self._cam(est)
        return abs(Lc) <= tl and abs(dc - DOCK_DIST) <= td and abs(psi) <= ta

    # ------------------------------------------------------------------
    def _guidance(self, obs, el, fresh):
        o = obs.odom
        if self.est is None:
            if el < 2.0 and self.stage == 'guidance':
                return Command(0.0, 0.0)
            self.mode = 'SEARCH'
            return self._approach(obs, el)

        L, d, psi = self.est
        # ---- stationary verification / docking ----
        if self._near(self.est, 0.05, 0.07, math.radians(4.5)):
            if self.hold_t0 is None:
                self.hold_t0 = obs.t
                self.samples = []
            held = obs.t - self.hold_t0
            n = len(self.samples)
            if n >= 4 and held >= 0.6 and abs(o.v) < 0.03:
                ss = self.samples[-6:]
                Ls = sorted(s[0] for s in ss)
                ds_ = sorted(s[1] for s in ss)
                ys = sorted(s[2] for s in ss)
                m = len(ss) // 2
                Lm, dm, ym = Ls[m], ds_[m], ys[m]
                if (abs(Lm) <= 0.03 and abs(dm - DOCK_DIST) <= 0.07
                        and abs(ym) <= math.radians(3.0)):
                    return Command(0.0, 0.0, 'DOCKED')
                # refine estimate with the averaged measurement and correct
                self.est = [Lm + CAM_X * math.sin(ym), dm + CAM_X * math.cos(ym), ym]
                self.hold_t0 = None
                self.samples = []
                L, d, psi = self.est
            elif held > 3.0 and n < 4:
                if self._near(self.est, 0.03, 0.06, math.radians(3.0)) and abs(o.v) < 0.03:
                    return Command(0.0, 0.0, 'DOCKED')
                self.hold_t0 = None
            else:
                return Command(0.0, 0.0)
        else:
            self.hold_t0 = None

        # ---- reverse / forward mode selection ----
        Lc, dc, _ = self._cam(self.est)
        d_back = D_GOAL + min(0.7, 1.5 * abs(L) + 0.15)
        if not self.reverse:
            if d < D_MIN + 0.0 and abs(Lc) > 0.04:
                self.reverse = True
            elif d < 0.95:
                self.reverse = True
        else:
            if d >= d_back:
                self.reverse = False

        if self.reverse:
            des = -_clamp(math.atan2(L, 0.35), -0.4, 0.4)
            e = _wrap(des - psi)
            w = _clamp(1.5 * e, -W_MAX, W_MAX)
            v = -0.12 if abs(e) < 0.5 else 0.0
            return Command(v, w)

        ell = 0.3 + 0.5 * max(d - D_GOAL, 0.0)
        des = _clamp(math.atan2(L, ell), -1.35, 1.35)
        # near the goal depth the heading should converge to 0 as L -> 0
        e = _wrap(des - psi)
        w = _clamp(1.6 * e, -W_MAX, W_MAX)
        vprof = min(V_MAX, 0.08 + 0.45 * max(d - D_GOAL, 0.0))
        if abs(e) > 0.7:
            v = 0.0
        else:
            v = vprof * max(0.0, math.cos(e)) ** 2
            if abs(e) < 0.4:
                v = max(v, 0.05)
        if d <= D_MIN and abs(Lc) > 0.04:
            v = 0.0
        # tag not seen for long and dead-reckoned pose is near: stay slow
        if obs.t - self.last_tag_t > 1.0:
            v = min(v, 0.12)
        return Command(v, w)

    # ------------------------------------------------------------------
    def _pallet_cands(self, obs):
        out = []
        for p in obs.pallets:
            if p.score < 0.35:
                continue
            valid = sorted(s for s in p.depth_samples if s > 0.2)
            depth = None
            if len(valid) >= 5:
                depth = valid[len(valid) // 4]
                exp = FX * PALLET_WIDTH / depth
                ratio = p.width_px / exp
                if not (0.7 <= ratio <= 1.35):
                    continue
            beta = math.atan((p.u - 320.0) / FX)
            out.append((p, depth, beta))
        return out

    def _approach(self, obs, el):
        o = obs.odom
        cands = self._pallet_cands(obs)
        # drop rejected directions
        good = []
        for c in cands:
            wb = _wrap(o.yaw - c[2])
            if any(abs(_wrap(wb - r)) < 0.2 for r in self.rej):
                continue
            good.append(c)
        if good:
            if self.track_u is not None:
                good.sort(key=lambda c: abs(c[0].u - self.track_u) - 100 * c[0].score)
            else:
                good.sort(key=lambda c: -c[0].score)
            p, depth, beta = good[0]
            self.track_u = p.u
            self.lost_t = 0.0
            self.pal_depth = depth
            self.search_rot = 0.0
            w = _clamp(-1.3 * beta, -W_MAX, W_MAX)
            if depth is None:
                depth = self.pal_depth if self.pal_depth else 3.5
            if abs(beta) > 0.3:
                v = 0.0
            elif depth > APPROACH_DEPTH + 0.1:
                v = _clamp(0.35 * (depth - APPROACH_DEPTH) + 0.06, 0.06, V_MAX)
                self.stop_t = 0.0
                self.wait_t = 0.0
            else:
                # at stopping distance, wait for the tag, then creep, then reject
                self.stop_t += DT
                if self.stop_t < 2.5:
                    v = 0.0
                elif depth > 1.7:
                    v = 0.08
                else:
                    self.rej.append(_wrap(o.yaw - beta))
                    self.track_u = None
                    self.stop_t = 0.0
                    v = 0.0
                    w = 0.4
            return Command(v, w)

        # nothing plausible in view
        self.lost_t += DT
        if self.track_u is not None and self.lost_t < 1.5:
            v = 0.1 if self.stop_t == 0.0 else 0.0
            return Command(v, 0.0)
        self.track_u = None
        self.search_rot += 0.45 * DT
        if self.search_rot > 2 * math.pi + 0.5 and self.rej:
            self.rej = []
            self.search_rot = 0.0
        return Command(0.0, 0.45)

    # ------------------------------------------------------------------
    def _free_dist(self, scan, center):
        if scan is None:
            return 10.0
        rmin = 10.0
        for i, r in enumerate(scan.ranges):
            if not (r > 0.05) or r == float('inf'):
                continue
            rel = _wrap(scan.angle_min + i * scan.angle_increment - center)
            if abs(rel) >= math.pi / 2:
                continue
            if r * abs(math.sin(rel)) <= 0.40:
                rmin = min(rmin, r * math.cos(rel))
        return rmin

    def _safety(self, obs, cmd):
        v, w = cmd.v, cmd.w
        if abs(v) > 1e-3 and obs.scan is not None:
            center = 0.0 if v > 0 else math.pi
            free = self._free_dist(obs.scan, center)
            lim = max(0.0, 0.7 * (free - 0.45))
            if abs(v) > lim:
                v = math.copysign(lim, v)
            if lim <= 0.0:
                self.blocked_t += DT
            else:
                self.blocked_t = 0.0
        else:
            self.blocked_t = 0.0
        if self.blocked_t > 15.0:
            return Command(0.0, 0.0, 'ABORTED')
        return Command(v, w)
