"""AGV pallet-pocket docking controller (A3 approach, A4 localise, A5 guide).

Pose of the AGV is kept in the pallet frame (tag at origin, +y out of the
pocket face).  Tag detections correct the pose, odometry propagates it while
the tag is not visible.  Deterministic, non-blocking, standard library only.
"""
import math

from sim.api import Command, FX, CAM_X, TAG_ID, PALLET_WIDTH, DT


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DockingController:
    STAGE_CAM = 1.55      # camera-to-face distance of the staging point [m]
    STOP_CAM = 0.87       # stop trigger (camera-to-face) [m]
    REJECT_RNG = 1.45     # reject pallet candidate without tag closer than this
    MAX_RETRIES = 4

    def __init__(self):
        self.reset('approach')

    # ------------------------------------------------------------------
    def reset(self, start_stage):
        self.start_stage = start_stage
        self.deadline = 105.0 if start_stage == 'guidance' else 135.0
        self.t0 = None
        self.pose = None            # base_link pose in pallet frame [x, y, th]
        self.prev_odom = None
        self.last_tag_t = -1e9
        self.tag_now = None
        self.state = 'SEARCH'
        self.retries = 0
        self.creeps = 0
        self.stage_tries = 0
        self.blacklist = []
        self.last_cand = None       # (u, t, bearing)
        self.last_cmd = (0.0, 0.0)
        self.search_rot = 0.0
        self.hop_until = -1e9
        self.blocked_t = 0.0
        self.errs = 0
        self.close_t = -1e9
        self.inhibit_t0 = None
        self.lock_t0 = None
        self.lock_n = 0
        self.align_done_t = None
        self.stop_t0 = None
        self.stop_buf = []

    # ------------------------------------------------------------------
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            self.errs += 1
            return Command(0.0, 0.0, 'RUNNING' if self.errs <= 3 else 'ABORTED')

    # ------------------------------------------------------------------
    def _step(self, obs):
        t = obs.t
        if self.t0 is None:
            self.t0 = t
        self._predict(obs.odom)
        self.tag_now = None
        # R-10 safety supervisor: any finite return within +-30 deg closer than 0.55 m
        if self._front_close(obs.scan):
            self.close_t = t
        inhibit = (t - self.close_t) < 1.0
        if inhibit:
            if self.inhibit_t0 is None:
                self.inhibit_t0 = t
            if t - self.inhibit_t0 >= 20.0:
                return Command(0.0, 0.0, 'ABORTED')
        else:
            self.inhibit_t0 = None
        tg = self._pick_tag(obs)
        if tg is not None:
            meas = self._measure(tg)
            self._update_pose(meas, t)
            self.tag_now = meas

        if (t - self.t0 > self.deadline or self.retries > self.MAX_RETRIES
                or self.blocked_t > 18.0):
            return Command(0.0, 0.0, 'ABORTED')

        if self.state in ('SEARCH', 'CHASE') and self.tag_now is not None:
            self.state = 'LOCK'
            self.lock_t0 = None

        s = self.state
        if s == 'SEARCH':
            r = self._search(t, obs)
        elif s == 'CHASE':
            r = self._chase(t, obs)
        elif s == 'LOCK':
            r = self._lock(t)
        elif s == 'STAGE':
            r = self._stage(t, obs)
        elif s == 'ALIGN':
            r = self._align(t)
        elif s == 'FINAL':
            r = self._final(t, obs)
        else:
            r = self._stop(t, obs)

        if isinstance(r, Command):
            return r
        v, w = r
        if inhibit and v > 0.0:
            v = 0.0
        v = clip(v, -0.30, 0.30)
        w = clip(w, -0.60, 0.60)
        self.last_cmd = (v, w)
        return Command(v, w, 'RUNNING')

    @staticmethod
    def _front_close(scan):
        if scan is None:
            return False
        a = scan.angle_min
        inc = scan.angle_increment
        lim = math.radians(30.0) + 1e-9
        for r in scan.ranges:
            if r is not None and math.isfinite(r) and 0.02 < r < 0.55 \
                    and abs(wrap(a)) <= lim:
                return True
            a += inc
        return False

    # ---------------------------- estimation --------------------------
    def _predict(self, od):
        if self.prev_odom is None:
            self.prev_odom = (od.x, od.y, od.yaw)
            return
        px, py, pyaw = self.prev_odom
        dx, dy = od.x - px, od.y - py
        dyaw = wrap(od.yaw - pyaw)
        m = pyaw + dyaw / 2.0
        dfwd = math.cos(m) * dx + math.sin(m) * dy
        dlat = -math.sin(m) * dx + math.cos(m) * dy
        if self.pose is not None:
            bx, by, bth = self.pose
            h = bth + dyaw / 2.0
            bx += dfwd * math.cos(h) - dlat * math.sin(h)
            by += dfwd * math.sin(h) + dlat * math.cos(h)
            self.pose = [bx, by, wrap(bth + dyaw)]
        self.search_rot += abs(dyaw)
        self.prev_odom = (od.x, od.y, od.yaw)

    @staticmethod
    def _measure(tag):
        x, z, yw = tag.x, tag.z, tag.yaw
        s, c = math.sin(yw), math.cos(yw)
        cxm = -z * s + x * c
        cym = z * c + x * s
        th = wrap(-math.pi / 2.0 + yw)
        return (cxm, cym, th)

    def _pick_tag(self, obs):
        best = None
        for tg in obs.tags:
            try:
                if tg.id != TAG_ID:
                    continue
                if not (math.isfinite(tg.x) and math.isfinite(tg.z) and math.isfinite(tg.yaw)):
                    continue
                if not (0.3 < tg.z < 6.0) or abs(tg.yaw) > 1.4:
                    continue
                cxm, cym, _ = self._measure(tg)
                if cym < 0.3 or math.atan2(abs(cxm), cym) > 1.15:
                    continue
            except Exception:
                continue
            if best is None or tg.z < best.z:
                best = tg
        return best

    def _update_pose(self, meas, t):
        cxm, cym, thm = meas
        bxm = cxm - CAM_X * math.cos(thm)
        bym = cym - CAM_X * math.sin(thm)
        if self.pose is None or math.hypot(bxm - self.pose[0], bym - self.pose[1]) > 0.8:
            self.pose = [bxm, bym, thm]
        else:
            g, gt = 0.4, 0.3
            p = self.pose
            p[0] += g * (bxm - p[0])
            p[1] += g * (bym - p[1])
            p[2] = wrap(p[2] + gt * wrap(thm - p[2]))
        self.last_tag_t = t

    def _cam(self):
        bx, by, bth = self.pose
        return (bx + CAM_X * math.cos(bth), by + CAM_X * math.sin(bth),
                wrap(bth + math.pi / 2.0))

    def _set_pose_from_cam(self, cx, cy, eps):
        th = wrap(eps - math.pi / 2.0)
        self.pose = [cx - CAM_X * math.cos(th), cy - CAM_X * math.sin(th), th]

    # ---------------------------- LiDAR guard -------------------------
    @staticmethod
    def _corridor(scan, reverse=False, half=0.40, reach=1.4):
        if scan is None:
            return reach
        off = math.pi if reverse else 0.0
        best = reach
        a = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            if r is not None and r == r and 0.05 < r < reach + 0.6:
                ang = a - off
                fx = r * math.cos(ang)
                if 0.0 < fx < best and abs(r * math.sin(ang)) < half:
                    best = fx
            a += inc
        return best

    def _guard(self, v, scan):
        if abs(v) < 1e-3:
            self.blocked_t = 0.0
            return 0.0
        clear = self._corridor(scan, reverse=v < 0)
        f = clip((clear - 0.42) / 0.5, 0.0, 1.0)
        if f < 0.05:
            self.blocked_t += DT
        else:
            self.blocked_t = 0.0
        return v * f

    # ---------------------------- pallet detector ---------------------
    @staticmethod
    def _depth(samples):
        v = sorted(d for d in samples
                   if d is not None and d == d and 0.3 < d < 9.0)
        if len(v) < 3:
            return None
        q = v[len(v) // 4]
        cl = [d for d in v if abs(d - q) <= 0.3]
        return cl[len(cl) // 2]

    def _cands(self, obs):
        od = obs.odom
        out = []
        for p in obs.pallets:
            if p.score < 0.45 or p.width_px < 5:
                continue
            b = math.atan((p.u - 320.0) / FX)
            d = self._depth(p.depth_samples)
            if d is not None:
                rng = d / max(0.3, math.cos(b))
                wm = p.width_px * d / FX
                if not (0.45 <= wm <= 1.5):
                    continue
            else:
                rng = clip(FX * PALLET_WIDTH / p.width_px, 0.8, 8.0)
            ang = od.yaw - b
            camx = od.x + CAM_X * math.cos(od.yaw)
            camy = od.y + CAM_X * math.sin(od.yaw)
            wx = camx + rng * math.cos(ang)
            wy = camy + rng * math.sin(ang)
            if any(math.hypot(wx - bx, wy - by) < 0.8 for bx, by in self.blacklist):
                continue
            rank = p.score + (0.15 if d is not None else 0.0)
            out.append({'u': p.u, 'b': b, 'rng': rng, 'rank': rank,
                        'known': d is not None, 'wx': wx, 'wy': wy})
        return out

    def _pick_cand(self, cands, t):
        if self.last_cand is not None and t - self.last_cand[1] < 1.0:
            near = [c for c in cands if abs(c['u'] - self.last_cand[0]) < 100]
            if near:
                return min(near, key=lambda c: abs(c['u'] - self.last_cand[0]))
        return max(cands, key=lambda c: c['rank'])

    # ---------------------------- states ------------------------------
    def _search(self, t, obs):
        if self._cands(obs):
            self.state = 'CHASE'
            return self._chase(t, obs)
        if t < self.hop_until:
            return self._guard(0.22, obs.scan), 0.0
        if self.search_rot > 2.0 * math.pi:
            self.search_rot = 0.0
            self.hop_until = t + 1.8
            self.blacklist = []
        self.blocked_t = 0.0
        return 0.0, 0.45

    def _chase(self, t, obs):
        cands = self._cands(obs)
        if not cands:
            if self.last_cand is None:
                self.state = 'SEARCH'
                return 0.0, 0.0
            dt = t - self.last_cand[1]
            if dt < 0.6:
                return self.last_cmd[0] * 0.5, self.last_cmd[1]
            if dt < 3.0:
                b = self.last_cand[2]
                return 0.0, (-0.4 if b > 0 else 0.4)
            self.state = 'SEARCH'
            self.search_rot = 0.0
            self.last_cand = None
            return 0.0, 0.0
        c = self._pick_cand(cands, t)
        self.last_cand = (c['u'], t, c['b'])
        if c['rng'] < self.REJECT_RNG:
            self.blacklist.append((c['wx'], c['wy']))
            self.state = 'SEARCH'
            self.search_rot = 0.0
            self.last_cand = None
            return 0.0, 0.0
        b = c['b']
        v = 0.25 if c['rng'] > 2.2 else 0.18
        v *= max(0.0, 1.0 - abs(b) / 0.5)
        v = self._guard(v, obs.scan)
        return v, clip(-1.5 * b, -0.5, 0.5)

    def _lock(self, t):
        if self.lock_t0 is None:
            self.lock_t0 = t
            self.lock_n = 0
        if self.tag_now is not None:
            self.lock_n += 1
        if self.lock_n >= 3 or t - self.lock_t0 > 1.2:
            self.lock_t0 = None
            cx, cy, eps = self._cam()
            if abs(cx) < 0.12 and abs(eps) < 0.14 and cy > 0.95:
                self.state = 'FINAL'
            else:
                self.state = 'STAGE'
            self.stage_tries = 0
        return 0.0, 0.0

    def _stage(self, t, obs):
        bx, by, bth = self.pose
        tx, ty = 0.0, self.STAGE_CAM + CAM_X
        dx, dy = tx - bx, ty - by
        dist = math.hypot(dx, dy)
        if dist < 0.10:
            self.state = 'ALIGN'
            self.align_done_t = None
            return 0.0, 0.0
        e = wrap(math.atan2(dy, dx) - bth)
        if abs(e) > 1.9 and dist < 1.3:
            e2 = wrap(e - math.pi)
            v = -min(0.15, 0.4 * dist + 0.04) * max(0.0, 1.0 - abs(e2) / 0.6)
            w = clip(1.8 * e2, -0.6, 0.6)
        else:
            v = min(0.25, 0.5 * dist + 0.05) * max(0.0, 1.0 - abs(e) / 0.6)
            w = clip(1.8 * e, -0.6, 0.6)
        v = self._guard(v, obs.scan)
        return v, w

    def _align(self, t):
        cx, cy, eps = self._cam()
        if self.align_done_t is not None and abs(eps) > 0.07:
            self.align_done_t = None
        if self.align_done_t is None:
            if abs(eps) > 0.035:
                w = clip(-2.0 * eps, -0.5, 0.5)
                if abs(w) < 0.06:
                    w = math.copysign(0.06, w)
                return 0.0, w
            self.align_done_t = t
        dt = t - self.align_done_t
        fresh = (t - self.last_tag_t) < 0.8
        if dt >= 0.6 and (fresh or dt >= 2.0):
            self.align_done_t = None
            lim = 0.05 if self.stage_tries < 3 else 0.10
            if fresh and abs(cx) > lim and self.stage_tries < 4:
                self.stage_tries += 1
                self.state = 'STAGE'
            else:
                self.state = 'FINAL'
        return 0.0, 0.0

    def _final(self, t, obs):
        cx, cy, eps = self._cam()
        if t - self.last_tag_t > 3.0 and cy > 1.0:
            self.retries += 1
            self.stage_tries = 0
            self.state = 'STAGE'
            return 0.0, 0.0
        if cy < 1.05 and (abs(cx) > 0.08 or abs(eps) > 0.17):
            self.retries += 1
            self.stage_tries = 0
            self.state = 'STAGE'
            return 0.0, 0.0
        if cy <= self.STOP_CAM:
            self.state = 'STOP'
            self.stop_t0 = None
            return 0.0, 0.0
        eps_des = clip(-math.atan(3.0 * cx), -0.35, 0.35)
        err = eps_des - eps
        w = clip(2.0 * err, -0.5, 0.5)
        v = clip(0.45 * (cy - self.STOP_CAM) + 0.05, 0.05, 0.22)
        v *= max(0.0, 1.0 - max(0.0, abs(err) - 0.12) / 0.25)
        if abs(cx) > 0.25:
            v *= 0.6
        v = self._guard(v, obs.scan)
        return v, w

    def _stop(self, t, obs):
        od = obs.odom
        if self.stop_t0 is None:
            self.stop_t0 = t
            self.stop_buf = []
        dt = t - self.stop_t0
        moving = abs(od.v) > 0.02 or abs(od.w) > 0.03
        if self.tag_now is not None and dt >= 0.3 and (not moving or dt >= 0.8):
            cxm, cym, thm = self.tag_now
            self.stop_buf.append((cxm, cym, wrap(thm + math.pi / 2.0)))
        n = len(self.stop_buf)
        if n >= 4 or (dt > 2.5 and n >= 2):
            cx = sum(b[0] for b in self.stop_buf) / n
            cy = sum(b[1] for b in self.stop_buf) / n
            eps = sum(b[2] for b in self.stop_buf) / n
            self.stop_t0 = None
            self._set_pose_from_cam(cx, cy, eps)
            if abs(cx) <= 0.035 and abs(eps) <= math.radians(4.0):
                if 0.76 <= cy <= 0.94:
                    return Command(0.0, 0.0, 'DOCKED')
                if cy > 0.94 and self.creeps < 3:
                    self.creeps += 1
                    self.state = 'FINAL'
                    return 0.0, 0.0
            self.retries += 1
            self.stage_tries = 0
            self.state = 'STAGE'
            return 0.0, 0.0
        if dt > 3.5:
            self.stop_t0 = None
            self.retries += 1
            self.stage_tries = 0
            self.state = 'STAGE'
        return 0.0, 0.0
