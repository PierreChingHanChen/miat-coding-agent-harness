"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guide).

State machine (all deterministic, one call per tick, no blocking):
  WAIT_TAG  guidance start: wait ~1.2 s for the tag
  SEARCH    rotate in place, collect pallet candidates over a sweep
  TRACK     drive toward a candidate pallet in stages (2.3 -> 1.3 m), waiting
            for the tag at each stage; blacklist the candidate if no tag
  SETTLE    stand still, average tag measurements (tag pose kept in odom frame)
  STAGE     go to a staging point on the pocket centre line (tag may be lost)
  ALIGN     rotate to face the pocket normal
  FINAL     line following with tag (dead reckoning when the tag is lost)
  HOLD      stopped; verify with raw tag detections, then DOCKED
  TWEAK     small in-place heading fix after a failed verification
"""
import math
from sim.api import (Command, FX, CAM_X, DOCK_DIST, TAG_ID, PALLET_WIDTH,
                     V_MAX, W_MAX)

TWO_PI = 2.0 * math.pi
BY_STAGE = 2.0          # base_link distance of staging point from the face


def wrap(a):
    return (a + math.pi) % TWO_PI - math.pi


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class DockingController:
    def __init__(self):
        self.reset('guidance')

    # ------------------------------------------------------------------
    def reset(self, start_stage):
        self.stage0 = start_stage
        self.limit = 120.0 if start_stage == 'guidance' else 150.0
        self.t0 = None
        self.mode = 'INIT'
        self.mode_t = 0.0
        # tag estimate in odom frame
        self.est = False
        self.tx = self.ty = self.phi = 0.0
        self.n = 0
        self.rej = 0
        self.last_tag_t = -1e9
        self.tag_times = []
        self.settle_n = 0
        # pallet / search
        self.blacklist = []
        self.tgt = None
        self.tgt_lost_t = 0.0
        self.dstop = 2.3
        self.reach_t = None
        self.dfilt = None
        self.sweep_acc = 0.0
        self.sweep_last_yaw = None
        self.sweep_best = None
        self.sweeps = 0
        self.guid_track = False
        # docking
        self.by_s = BY_STAGE
        self.retreat = False
        self.no_rev = False
        self.samples = []
        self.attempts = 0
        self.stage_tries = 0
        self.blocked = 0
        self.exc = 0
        self.stop_ticks = 0
        self.last_close_t = -1e9
        self.inhib_since = None

    # ------------------------------------------------------------------
    def _supervisor(self, obs, cmd):
        """R-10 forward-motion inhibit."""
        t = obs.t
        sc = obs.scan
        if sc is not None:
            lim = math.radians(30.0) + 1e-9
            for i, r in enumerate(sc.ranges):
                if r is None or r != r or r in (float('inf'), float('-inf')):
                    continue
                if r < 0.55:
                    a = wrap(sc.angle_min + i * sc.angle_increment)
                    if abs(a) <= lim:
                        self.last_close_t = t
                        break
        if t - self.last_close_t < 1.0:
            if self.inhib_since is None:
                self.inhib_since = t
            if t - self.inhib_since >= 20.0:
                return Command(0.0, 0.0, 'ABORTED')
            if cmd.v > 0.0:
                cmd.v = 0.0
        else:
            self.inhib_since = None
        return cmd

    def step(self, obs):
        try:
            cmd = self._step(obs)
        except Exception:
            self.exc += 1
            if self.exc > 5:
                return Command(0.0, 0.0, 'ABORTED')
            cmd = Command(0.0, 0.0, 'RUNNING')
        try:
            cmd = self._supervisor(obs, cmd)
        except Exception:
            cmd.v = min(cmd.v, 0.0)
        cmd.v = clamp(cmd.v, -V_MAX, V_MAX)
        cmd.w = clamp(cmd.w, -W_MAX, W_MAX)
        if cmd.status != 'RUNNING':
            cmd.v = 0.0
            cmd.w = 0.0
        return cmd

    # ------------------------------------------------------------------
    def _set(self, mode, t):
        self.mode = mode
        self.mode_t = t
        self.stop_ticks = 0

    def _abort(self):
        return Command(0.0, 0.0, 'ABORTED')

    # ---- tag handling ------------------------------------------------
    def _best_tag(self, obs):
        best = None
        for d in obs.tags:
            if d.id != TAG_ID:
                continue
            if not (0.4 < d.z < 5.0):
                continue
            if abs(math.atan2(d.x, d.z)) > math.radians(38):
                continue
            if abs(d.yaw) > math.radians(80):
                continue
            if best is None or d.stamp > best.stamp or \
                    (d.stamp == best.stamp and d.z < best.z):
                best = d
        return best

    def _update_est(self, d, od, t):
        c, s = math.cos(od.yaw), math.sin(od.yaw)
        cx, cy = od.x + CAM_X * c, od.y + CAM_X * s
        tx = cx + d.z * c + d.x * s
        ty = cy + d.z * s - d.x * c
        phi = wrap(od.yaw - d.yaw)
        if not self.est:
            self.tx, self.ty, self.phi = tx, ty, phi
            self.est = True
            self.n = 1
            self.rej = 0
        else:
            err = math.hypot(tx - self.tx, ty - self.ty)
            aerr = abs(wrap(phi - self.phi))
            if self.n >= 3 and (err > 0.5 or aerr > math.radians(25)):
                self.rej += 1
                if self.rej >= 4:
                    self.tx, self.ty, self.phi = tx, ty, phi
                    self.n = 1
                    self.rej = 0
                return False
            self.rej = 0
            self.n += 1
            a = max(1.0 / self.n, 0.3)
            self.tx += a * (tx - self.tx)
            self.ty += a * (ty - self.ty)
            self.phi = wrap(self.phi + a * wrap(phi - self.phi))
        self.last_tag_t = t
        self.tag_times.append(t)
        self.tag_times = [x for x in self.tag_times if t - x <= 1.0]
        self.settle_n += 1
        return True

    def _pose(self, od):
        """Returns (bx_base, by_base, bx_cam, by_cam, heading_err)."""
        ph = self.phi
        eyx, eyy = -math.cos(ph), -math.sin(ph)
        exx, exy = -math.sin(ph), math.cos(ph)
        dx, dy = od.x - self.tx, od.y - self.ty
        bxb = dx * exx + dy * exy
        byb = dx * eyx + dy * eyy
        cx, cy = dx + CAM_X * math.cos(od.yaw), dy + CAM_X * math.sin(od.yaw)
        bxc = cx * exx + cy * exy
        byc = cx * eyx + cy * eyy
        return bxb, byb, bxc, byc, wrap(od.yaw - ph)

    # ---- LiDAR guard -------------------------------------------------
    def _limit(self, scan, v):
        if scan is None or abs(v) < 1e-6:
            return v, False
        sign = 1.0 if v > 0 else -1.0
        xmin = 1e9
        for i, r in enumerate(scan.ranges):
            if r is None or r != r or r >= scan.range_max or r <= 0.0:
                continue
            a = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(a) * sign
            y = r * math.sin(a)
            if x > 0.0 and abs(y) < 0.38 and x < xmin:
                xmin = x
        allow = max(0.0, (xmin - 0.42) * 0.8)
        if allow >= abs(v):
            return v, False
        if allow < 0.02:
            return 0.0, True
        return sign * allow, False

    def _drive(self, obs, v, w):
        v2, blk = self._limit(obs.scan, v)
        if blk:
            self.blocked += 1
        else:
            self.blocked = 0
        return Command(v2, w, 'RUNNING')

    # ---- pallet candidates ------------------------------------------
    def _cands(self, obs):
        od = obs.odom
        c, s = math.cos(od.yaw), math.sin(od.yaw)
        out = []
        for p in obs.pallets:
            if p.score < 0.5 or p.width_px < 20:
                continue
            vals = sorted(d for d in p.depth_samples if d is not None and 0.3 < d < 10.0)
            zw = FX * PALLET_WIDTH / p.width_px
            depth = None
            if len(vals) >= 3:
                depth = vals[int(0.3 * (len(vals) - 1))]
            pen = 0.0
            if depth is None:
                depth = 0.9 * zw
                pen += 0.1
            rho = depth * p.width_px / (FX * PALLET_WIDTH)
            if rho > 1.15 or rho < 0.45:
                pen += 0.35
            eu = math.atan2(p.u - 320.0, FX)
            ang = wrap(od.yaw - eu)
            lat = depth * (p.u - 320.0) / FX
            wx = od.x + CAM_X * c + depth * c + lat * s
            wy = od.y + CAM_X * s + depth * s - lat * c
            skip = False
            for (bx, by) in self.blacklist:
                if math.hypot(wx - bx, wy - by) < 0.9:
                    skip = True
                    break
            if skip:
                continue
            val = p.score - pen
            if self.guid_track:
                val -= 0.8 * abs(eu)
            else:
                val -= 0.2 * max(0.0, abs(depth - 3.3) - 0.7)
            out.append({'val': val, 'ang': ang, 'depth': depth, 'u': p.u,
                        'eu': eu, 'pos': (wx, wy), 'score': p.score})
        out.sort(key=lambda d: -d['val'])
        return out

    # ------------------------------------------------------------------
    def _step(self, obs):
        t = obs.t
        od = obs.odom
        if self.t0 is None:
            self.t0 = t
        el = t - self.t0
        if self.mode == 'INIT':
            if self.stage0 == 'guidance':
                self.guid_track = True
                self._set('WAIT_TAG', t)
            else:
                self._start_search(od, t)

        tag = self._best_tag(obs)
        got = False
        if tag is not None:
            got = self._update_est(tag, od, t)

        # leave search/track modes once the tag is reliably seen
        if self.mode in ('WAIT_TAG', 'SEARCH', 'TRACK') and self.est \
                and len(self.tag_times) >= 2:
            self.settle_n = 0
            self._set('SETTLE', t)

        if el > self.limit - 4.0 and self.mode != 'HOLD':
            return self._abort()
        if el > self.limit - 1.5:
            return self._abort()

        m = self.mode
        if m == 'WAIT_TAG':
            if t - self.mode_t > 1.2:
                self._set('TRACK', t)
                self.tgt = None
            return Command(0.0, 0.0)
        if m == 'SEARCH':
            return self._do_search(obs, t)
        if m == 'TRACK':
            return self._do_track(obs, t)
        if m == 'SETTLE':
            return self._do_settle(obs, t)
        if m == 'STAGE':
            return self._do_stage(obs, t)
        if m == 'ALIGN':
            return self._do_align(obs, t)
        if m == 'FINAL':
            return self._do_final(obs, t)
        if m == 'HOLD':
            return self._do_hold(obs, tag, t)
        if m == 'TWEAK':
            return self._do_tweak(obs, t)
        return self._abort()

    # ---- SEARCH ------------------------------------------------------
    def _start_search(self, od, t):
        self._set('SEARCH', t)
        self.sweep_acc = 0.0
        self.sweep_last_yaw = od.yaw
        self.sweep_best = None
        self.tgt = None

    def _do_search(self, obs, t):
        od = obs.odom
        self.sweep_acc += abs(wrap(od.yaw - self.sweep_last_yaw))
        self.sweep_last_yaw = od.yaw
        cs = self._cands(obs)
        if cs:
            b = cs[0]
            if self.sweep_best is None or b['val'] > self.sweep_best['val']:
                self.sweep_best = b
            if b['val'] >= 0.9 and not self.guid_track:
                return self._pick(b, t)
        if self.sweep_acc >= TWO_PI + 0.1:
            if self.sweep_best is not None:
                return self._pick(self.sweep_best, t)
            self.sweeps += 1
            if self.sweeps >= 2:
                self.blacklist = []
            if self.sweeps >= 5:
                return self._abort()
            self.guid_track = False
            self.sweep_acc = 0.0
            self.sweep_best = None
        return Command(0.0, 0.45)

    def _pick(self, b, t):
        self.tgt = dict(b)
        self.tgt_lost_t = t
        self.dstop = 2.3
        self.reach_t = None
        self.dfilt = None
        self._set('TRACK', t)
        return Command(0.0, 0.0)

    # ---- TRACK -------------------------------------------------------
    def _do_track(self, obs, t):
        od = obs.odom
        if self.tgt is None:
            cs = self._cands(obs)
            if not cs:
                if t - self.mode_t > 1.0:
                    self.guid_track = False
                    self._start_search(od, t)
                return Command(0.0, 0.0)
            return self._pick(cs[0], t)
        cs = self._cands(obs)
        match = None
        bd = 0.25
        for c in cs:
            d = abs(wrap(c['ang'] - self.tgt['ang']))
            if d < bd:
                bd = d
                match = c
        if match is None:
            # lost: rotate toward remembered bearing
            if t - self.tgt_lost_t > 3.0:
                self.guid_track = False
                self._start_search(od, t)
                return Command(0.0, 0.0)
            e = wrap(self.tgt['ang'] - od.yaw)
            w = clamp(1.5 * e, -0.5, 0.5) if abs(e) > 0.05 else 0.0
            return Command(0.0, w)
        self.tgt_lost_t = t
        self.tgt['ang'] = match['ang']
        self.tgt['pos'] = match['pos']
        d = match['depth']
        self.dfilt = d if self.dfilt is None else 0.5 * self.dfilt + 0.5 * d
        eu = match['eu']
        w = clamp(-1.8 * eu, -0.5, 0.5)
        if abs(eu) > 0.15:
            return Command(0.0, w)
        if self.dfilt > self.dstop:
            self.reach_t = None
            v = clamp(0.5 * (self.dfilt - self.dstop) + 0.08, 0.08, 0.25)
            cmd = self._drive(obs, v, w)
            if self.blocked > 20:
                self._blacklist_tgt()
                self._start_search(od, t)
                return Command(0.0, 0.0)
            return cmd
        # reached the viewing distance: wait for the tag
        if self.reach_t is None:
            self.reach_t = t
        if t - self.reach_t > 1.3:
            if self.dstop > 1.35:
                self.dstop = max(1.3, self.dstop - 0.35)
                self.reach_t = None
            else:
                self._blacklist_tgt()
                self._start_search(od, t)
                return Command(0.0, 0.0)
        return Command(0.0, w)

    def _blacklist_tgt(self):
        if self.tgt is not None:
            self.blacklist.append(self.tgt['pos'])
        self.tgt = None
        self.guid_track = False

    # ---- SETTLE ------------------------------------------------------
    def _do_settle(self, obs, t):
        od = obs.odom
        dt = t - self.mode_t
        if not ((self.settle_n >= 6 and dt > 0.5) or dt > 2.0):
            return Command(0.0, 0.0)
        if not self.est:
            self._start_search(od, t)
            return Command(0.0, 0.0)
        bxb, byb, bxc, byc, eh = self._pose(od)
        L = byc - DOCK_DIST
        if abs(bxc) < 0.3 * max(L, 0.0) and abs(eh) < 0.35 and 1.3 < byb < 2.9:
            self._set('FINAL', t)
        else:
            self.by_s = BY_STAGE
            self.retreat = False
            self._set('STAGE', t)
        return Command(0.0, 0.0)

    # ---- STAGE -------------------------------------------------------
    def _do_stage(self, obs, t):
        od = obs.odom
        if t - self.mode_t > 45.0:
            return self._abort()
        ph = self.phi
        sx = self.tx - self.by_s * math.cos(ph)
        sy = self.ty - self.by_s * math.sin(ph)
        dx, dy = sx - od.x, sy - od.y
        dist = math.hypot(dx, dy)
        if dist < 0.10:
            self.settle_n = 0
            self._set('ALIGN', t)
            return Command(0.0, 0.0)
        alpha = math.atan2(dy, dx)
        ea = wrap(alpha - od.yaw)
        reverse = (not self.no_rev) and abs(ea) > math.pi / 2 + 0.1
        if reverse:
            ea = wrap(alpha - (od.yaw + math.pi))
        if abs(ea) > 0.4:
            w = clamp(2.0 * ea, -0.6, 0.6)
            if abs(w) < 0.15:
                w = 0.15 * (1 if w >= 0 else -1)
            return Command(0.0, w)
        v = min(0.25, 0.6 * dist + 0.03) * max(0.2, math.cos(ea))
        if reverse:
            v = -min(v, 0.15)
        w = clamp(2.0 * ea, -0.6, 0.6)
        cmd = self._drive(obs, v, w)
        if self.blocked > 8 and reverse:
            self.no_rev = True
            self.blocked = 0
        elif self.blocked > 40:
            return self._abort()
        return cmd

    # ---- ALIGN -------------------------------------------------------
    def _do_align(self, obs, t):
        od = obs.odom
        dt = t - self.mode_t
        bxb, byb, bxc, byc, eh = self._pose(od)
        if abs(eh) < 0.04 and (self.settle_n >= 3 or dt > 3.0):
            L = byc - DOCK_DIST
            if abs(bxc) < 0.3 * max(L, 0.0) + 0.03 and byc > 1.0:
                self._set('FINAL', t)
                return Command(0.0, 0.0)
            self.stage_tries += 1
            if self.stage_tries > 3:
                return self._abort()
            self.by_s = BY_STAGE
            self.retreat = False
            self._set('STAGE', t)
            return Command(0.0, 0.0)
        if dt > 12.0:
            return self._abort()
        if abs(eh) < 0.04:
            return Command(0.0, 0.0)
        w = clamp(-2.0 * eh, -0.5, 0.5)
        if abs(w) < 0.1:
            w = 0.1 * (1 if w >= 0 else -1)
        return Command(0.0, w)

    # ---- FINAL -------------------------------------------------------
    def _do_final(self, obs, t):
        od = obs.odom
        bxb, byb, bxc, byc, eh = self._pose(od)
        if byc - DOCK_DIST < 0.02:
            self.samples = []
            self.stop_ticks = 0
            self._set('HOLD', t)
            return Command(0.0, 0.0)
        psi = -clamp(math.atan(3.0 * bxc), -0.45, 0.45)
        werr = psi - eh
        w = clamp(2.5 * werr, -0.6, 0.6)
        v = clamp(0.45 * (byc - DOCK_DIST) + 0.04, 0.04, 0.22)
        v *= max(0.25, 1.0 - abs(werr) / 0.5)
        if abs(bxc) > 0.06:
            v = min(v, 0.08)
        if abs(eh) > 0.9:
            return Command(0.0, w)
        cmd = self._drive(obs, v, w)
        if self.blocked > 40:
            return self._abort()
        return cmd

    # ---- HOLD --------------------------------------------------------
    def _do_hold(self, obs, tag, t):
        od = obs.odom
        self.stop_ticks += 1
        dt = t - self.mode_t
        if tag is not None and self.stop_ticks >= 5 and abs(od.v) < 0.03 \
                and abs(od.w) < 0.03:
            y = tag.yaw
            byc = tag.x * math.sin(y) + tag.z * math.cos(y)
            bxc = tag.x * math.cos(y) - tag.z * math.sin(y)
            self.samples.append((bxc, byc, y))
        res = None
        if len(self.samples) >= 4:
            res = self._avg(self.samples[-6:])
        elif dt > 3.0:
            if self.est and self.n >= 3 and t - self.last_tag_t < 20.0:
                _, _, bxc, byc, eh = self._pose(od)
                res = (bxc, byc, eh)
                if not (abs(bxc) < 0.03 and abs(byc - DOCK_DIST) < 0.05
                        and abs(eh) < 0.05):
                    res = (bxc, byc, eh)
            else:
                res = (9.0, 9.0, 9.0)
        if res is None:
            return Command(0.0, 0.0)
        bxc, byc, eh = res
        if abs(bxc) <= 0.035 and abs(byc - DOCK_DIST) <= 0.07 \
                and abs(eh) <= math.radians(3.5) and self.stop_ticks >= 5:
            return Command(0.0, 0.0, 'DOCKED')
        # failed verification
        self.attempts += 1
        if self.attempts > 3:
            return self._abort()
        if len(self.samples) >= 4:
            self._reseed(od, res)
        if abs(bxc) <= 0.045 and abs(byc - DOCK_DIST) <= 0.08 and abs(eh) < 0.3:
            self._set('TWEAK', t)
            return Command(0.0, 0.0)
        self.by_s = 1.75
        self.retreat = True
        self.no_rev = False
        self.stage_tries = 0
        self._set('STAGE', t)
        return Command(0.0, 0.0)

    @staticmethod
    def _avg(s):
        def med(i):
            v = sorted(x[i] for x in s)
            return v[len(v) // 2]
        return (med(0), med(1), med(2))

    def _reseed(self, od, res):
        """Re-initialise the tag estimate from stationary averaged data."""
        bxc, byc, y = res
        ph = wrap(od.yaw - y)
        c, s = math.cos(od.yaw), math.sin(od.yaw)
        cx, cy = od.x + CAM_X * c, od.y + CAM_X * s
        # camera position rel. tag = bxc*ex + byc*ey
        exx, exy = -math.sin(ph), math.cos(ph)
        eyx, eyy = -math.cos(ph), -math.sin(ph)
        self.tx = cx - (bxc * exx + byc * eyx)
        self.ty = cy - (bxc * exy + byc * eyy)
        self.phi = ph
        self.n = 3
        self.rej = 0

    # ---- TWEAK -------------------------------------------------------
    def _do_tweak(self, obs, t):
        od = obs.odom
        _, _, bxc, byc, eh = self._pose(od)
        if abs(eh) < 0.02 or t - self.mode_t > 6.0:
            self.samples = []
            self._set('HOLD', t)
            return Command(0.0, 0.0)
        w = clamp(-2.0 * eh, -0.3, 0.3)
        if abs(w) < 0.08:
            w = 0.08 * (1 if w >= 0 else -1)
        return Command(0.0, w)
