"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guidance)."""
import math
from sim.api import (Command, FX, CAM_X, TAG_ID, DOCK_DIST, V_MAX, W_MAX,
                     PALLET_WIDTH)

TWO_PI = 2.0 * math.pi


def wrap(a):
    while a > math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DockingController:
    SAFE_STOP = 0.44      # base_link-to-obstacle distance where we stop
    SAFE_SLOW = 0.95      # distance where we start slowing
    HALF_W = 0.37         # swept half width checked ahead

    def reset(self, start_stage):
        self.stage = start_stage
        self.t0 = None
        self.mode = 'SEARCH' if start_stage == 'approach' else 'G_WAIT'
        self.limit = 144.0 if start_stage == 'approach' else 112.0
        # tag-frame fusion: F origin O (odom frame), u-axis angle psi
        self.has_f = False
        self.ox = self.oy = self.psi = 0.0
        self.tag_t = -1e9
        self.last_obs_t = 0.0
        self.gstate = None
        self.blocked = 0
        self.final_t = None
        self.still_det = 0
        self.still_ticks = 0
        # approach
        self.hyps = []
        self.rot = 0.0
        self.prev_yaw = None
        self.P = None
        self.wait_t = None
        self.mode_t = 0.0

    # ------------------------------------------------------------ helpers
    def _cam(self, od):
        return (od.x + CAM_X * math.cos(od.yaw), od.y + CAM_X * math.sin(od.yaw))

    def _update_tag(self, obs):
        best = None
        for d in obs.tags:
            if d.id != TAG_ID or d.z < 0.3 or d.z > 6.0:
                continue
            if best is None or d.stamp > best.stamp:
                best = d
        if best is None:
            return None
        od = obs.odom
        x, z, yaw = best.x, best.z, best.yaw
        u = -(z * math.cos(yaw) + x * math.sin(yaw))
        v = x * math.cos(yaw) - z * math.sin(yaw)
        psi = wrap(od.yaw - yaw)
        cx, cy = self._cam(od)
        ox = cx - (u * math.cos(psi) - v * math.sin(psi))
        oy = cy - (u * math.sin(psi) + v * math.cos(psi))
        if not self.has_f:
            self.ox, self.oy, self.psi = ox, oy, psi
            self.has_f = True
        else:
            a = 0.5
            self.psi = wrap(self.psi + a * wrap(psi - self.psi))
            self.ox += a * (ox - self.ox)
            self.oy += a * (oy - self.oy)
        self.tag_t = obs.t
        return best

    def _pose_f(self, od):
        cx, cy = self._cam(od)
        rx, ry = cx - self.ox, cy - self.oy
        c, s = math.cos(self.psi), math.sin(self.psi)
        u = rx * c + ry * s
        v = -rx * s + ry * c
        th = wrap(od.yaw - self.psi)
        return u, v, th

    def _free(self, scan, direction):
        """Distance (from base_link) to the nearest obstacle in the swept corridor."""
        if scan is None:
            return 9.0
        m = 9.0
        a = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            if r == r and r < 8.0 and r > 0.05:
                ang = a if direction > 0 else a - math.pi
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                if x > 0.0 and abs(y) < self.HALF_W and x < m:
                    m = x
            a += inc
        return m

    def _limit_v(self, v, scan):
        if abs(v) < 1e-3:
            return v
        fd = self._free(scan, 1 if v > 0 else -1)
        k = clip((fd - self.SAFE_STOP) / (self.SAFE_SLOW - self.SAFE_STOP), 0.0, 1.0)
        return v * k

    # ------------------------------------------------------------ step
    def step(self, obs):
        if self.t0 is None:
            self.t0 = obs.t
        el = obs.t - self.t0
        od = obs.odom
        if el > self.limit:
            return Command(0.0, 0.0, 'ABORTED')

        tag = self._update_tag(obs)

        if self.mode in ('SEARCH', 'TURN', 'DRIVE', 'WAIT_TAG'):
            if tag is not None and (self.mode != 'SEARCH' or tag.z < 3.0) and tag.z < 3.6:
                self.mode = 'G_WAIT'
                self.gstate = None
        if self.mode == 'SEARCH':
            cmd = self._search(obs)
        elif self.mode in ('TURN', 'DRIVE', 'WAIT_TAG'):
            cmd = self._approach(obs)
        else:
            cmd = self._guidance(obs)

        if cmd.status != 'RUNNING':
            return Command(0.0, 0.0, cmd.status)

        v = cmd.v
        vs = self._limit_v(v, obs.scan)
        if abs(v) > 0.02 and abs(vs) < 0.3 * abs(v):
            self.blocked += 1
        else:
            self.blocked = max(0, self.blocked - 1)
        if self.blocked > 120:
            return Command(0.0, 0.0, 'ABORTED')
        return Command(vs, clip(cmd.w, -W_MAX, W_MAX), 'RUNNING')

    # ------------------------------------------------------------ approach
    def _validate(self, det, obs):
        if det.score < 0.4 or det.width_px < 20:
            return None
        vals = sorted(d for d in det.depth_samples if d and d > 0.2)
        if len(vals) < 5:
            return None
        d = vals[int(0.3 * (len(vals) - 1))]
        wreal = det.width_px * d / FX
        if wreal < 0.7 or wreal > 1.7:
            return None
        b = math.atan2(-(det.u - 320.0), FX)
        radial = d / max(0.3, math.cos(b))
        od = obs.odom
        cx, cy = self._cam(od)
        phi = od.yaw + b
        px = cx + radial * math.cos(phi)
        py = cy + radial * math.sin(phi)
        r0 = math.hypot(px, py)
        if r0 < 2.6 or r0 > 4.6:
            return None
        sc = obs.scan
        if sc is not None and sc.angle_increment > 0:
            exp = (d + CAM_X * math.cos(b)) / max(0.3, math.cos(b))
            ok = False
            n = len(sc.ranges)
            for k in range(n):
                ang = sc.angle_min + k * sc.angle_increment
                if abs(wrap(ang - b)) < math.radians(7) and abs(sc.ranges[k] - exp) < 0.5:
                    ok = True
                    break
            if not ok:
                return None
        return px, py, phi, det.score

    def _search(self, obs):
        od = obs.odom
        if self.prev_yaw is not None:
            self.rot += abs(wrap(od.yaw - self.prev_yaw))
        self.prev_yaw = od.yaw
        for det in obs.pallets:
            r = self._validate(det, obs)
            if r is None:
                continue
            px, py, phi, sc = r
            for h in self.hyps:
                if math.hypot(h['x'] - px, h['y'] - py) < 0.7:
                    h['n'] += 1
                    h['s'] += sc
                    h['x'] += (px - h['x']) / h['n']
                    h['y'] += (py - h['y']) / h['n']
                    break
            else:
                self.hyps.append({'x': px, 'y': py, 'n': 1, 's': sc})
        good = [h for h in self.hyps if h['n'] >= 3]
        if good and self.rot >= TWO_PI * 0.97:
            best = max(good, key=lambda h: h['s'])
            self.P = [best['x'], best['y']]
            self.mode = 'TURN'
            self.mode_t = obs.t
            return Command(0.0, 0.0)
        if self.rot > 3 * TWO_PI:
            return Command(0.0, 0.0, 'ABORTED')
        return Command(0.0, 0.4)

    def _approach(self, obs):
        od = obs.odom
        cx, cy = self._cam(od)
        # refine target using consistent detections
        dirp = math.atan2(self.P[1] - cy, self.P[0] - cx)
        for det in obs.pallets:
            r = self._validate(det, obs)
            if r is None:
                continue
            px, py, phi, sc = r
            if abs(wrap(phi - dirp)) < 0.3 and math.hypot(px - self.P[0], py - self.P[1]) < 0.8:
                self.P[0] += 0.3 * (px - self.P[0])
                self.P[1] += 0.3 * (py - self.P[1])
        dist = math.hypot(self.P[0] - cx, self.P[1] - cy)
        dirp = math.atan2(self.P[1] - cy, self.P[0] - cx)
        e = wrap(dirp - od.yaw)
        if self.mode == 'TURN':
            if abs(e) < 0.06:
                self.mode = 'DRIVE'
                return Command(0.0, 0.0)
            return Command(0.0, clip(1.5 * e, -0.5, 0.5))
        stop_d = 2.0
        rem = dist - stop_d
        if self.mode == 'DRIVE':
            if rem <= 0.0:
                self.mode = 'WAIT_TAG'
                self.wait_t = obs.t
                return Command(0.0, 0.0)
            if abs(e) > 0.5:
                return Command(0.0, clip(1.5 * e, -0.5, 0.5))
            v = clip(0.6 * rem, 0.08, 0.25) * max(0.0, math.cos(e))
            return Command(v, clip(1.5 * e, -0.5, 0.5))
        # WAIT_TAG: tag not yet seen; hold, then creep
        w = clip(1.2 * e, -0.3, 0.3)
        if obs.t - self.wait_t < 1.5:
            return Command(0.0, w)
        if dist > 1.35:
            return Command(0.08, w)
        if obs.t - self.wait_t > 12.0:
            return Command(0.0, 0.0, 'ABORTED')
        return Command(0.0, w)

    # ------------------------------------------------------------ guidance
    def _guidance(self, obs):
        od = obs.odom
        if not self.has_f:
            # no tag yet: hold still, gently scan for the tag
            if obs.t - self.t0 > 20.0:
                return Command(0.0, 0.0, 'ABORTED')
            return Command(0.0, 0.0)
        if self.mode == 'G_WAIT':
            self.mode = 'G'
        u, v, th = self._pose_f(od)
        stale = obs.t - self.tag_t
        gs = self.gstate
        goal_u = -DOCK_DIST
        dxg = goal_u - u

        if gs is None:
            gs = 'RUN' if (abs(v) <= 0.10 and abs(th) <= 0.25) else 'LAT'
            self.gstate = gs

        if gs == 'LAT':
            bu = u - CAM_X * math.cos(th)
            bv = v - CAM_X * math.sin(th)
            wu = min(-1.6, bu + 0.25 * abs(bv))
            rx, ry = wu - bu, -bv
            rho = math.hypot(rx, ry)
            if rho < 0.05 or (rho < 0.12 and abs(bv) < 0.03):
                self.gstate = 'ALIGN'
                return Command(0.0, 0.0)
            ang = math.atan2(ry, rx)
            e = wrap(ang - th)
            rev = False
            if abs(e) > math.radians(110):
                rev = True
                e = wrap(e + math.pi)
            if abs(e) > 0.35:
                return Command(0.0, clip(1.5 * e, -0.55, 0.55))
            sp = clip(0.6 * rho, 0.06, 0.25) * math.cos(e)
            return Command(-sp if rev else sp, clip(1.8 * e, -0.55, 0.55))

        if gs == 'ALIGN':
            if abs(v - CAM_X * math.sin(th)) > 0.25:
                self.gstate = 'LAT'
                return Command(0.0, 0.0)
            if abs(th) < 0.04:
                self.gstate = 'RUN'
                return Command(0.0, 0.0)
            return Command(0.0, clip(1.5 * (-th), -0.55, 0.55))

        # RUN / FINAL
        if abs(v) > 0.30 or abs(th) > 0.7:
            self.gstate = 'LAT'
            self.final_t = None
            return Command(0.0, 0.0)
        thd = clip(-2.0 * v, -0.4, 0.4)
        w = clip(1.8 * (thd - th), -0.5, 0.5)

        if dxg > 0.025 and self.final_t is None:
            if stale > 1.0 and dxg < 0.35:
                # lost tag near the end: hold and wait for it
                if stale > 12.0:
                    return Command(0.0, 0.0, 'ABORTED')
                return Command(0.0, 0.0)
            sp = clip(0.5 * dxg, 0.05, 0.22)
            if abs(thd - th) > 0.3:
                sp = 0.0
            return Command(sp, w)

        # final: stop and verify
        if self.final_t is None:
            self.final_t = obs.t
            self.still_det = 0
            self.still_ticks = 0
            self._last_tag_t = self.tag_t
        if self.tag_t != self._last_tag_t:
            self.still_det += 1
            self._last_tag_t = self.tag_t
        moving = abs(od.v) > 0.02 or abs(od.w) > 0.03
        self.still_ticks = 0 if moving else self.still_ticks + 1
        d_cam = -u
        lat_ok = abs(v) < 0.035
        th_ok = abs(th) < 0.07
        d_ok = abs(d_cam - DOCK_DIST) < 0.08
        if self.still_ticks >= 3 and self.still_det >= 2 and stale < 0.5:
            if lat_ok and th_ok and d_ok:
                return Command(0.0, 0.0, 'DOCKED')
            # needs correction
            self.final_t = None
            if not d_ok and lat_ok and th_ok:
                sp = clip(0.5 * (goal_u - u), -0.1, 0.1)
                return Command(sp, 0.0)
            if not lat_ok:
                self.gstate = 'LAT'
            elif not th_ok:
                self.gstate = 'ALIGN'
            return Command(0.0, 0.0)
        if obs.t - self.final_t > 8.0:
            if stale > 5.0:
                return Command(0.0, 0.0, 'ABORTED')
            self.final_t = None
            if not lat_ok:
                self.gstate = 'LAT'
            elif not th_ok:
                self.gstate = 'ALIGN'
        return Command(0.0, 0.0)
