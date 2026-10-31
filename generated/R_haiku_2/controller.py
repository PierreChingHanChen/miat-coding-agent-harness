"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guidance).

Pose convention: pallet frame, tag at origin, +y out of the pocket face.
Camera pose (cx, cy, psi): psi is the camera heading (CCW positive).
Ideal docking heading is PSI_DOCK = -pi/2 (facing the face).
Heading error e = psi - PSI_DOCK (CCW positive).
"""
import math

from sim.api import (Command, Observation, FX, IMG_W, V_MAX, W_MAX,
                     DOCK_DIST, CAM_X, TAG_ID, DT, PALLET_WIDTH)

HALF_PI = math.pi / 2.0
PSI_DOCK = -HALF_PI

TAG_MIN_Z = 0.6          # ignore partially visible / far tags
TAG_MAX_Z = 4.0
TAG_STALE = 1.0          # s: after this without detection, pose is not trusted
LAT_TOL = 0.04
HEAD_TOL = math.radians(2.5)
DIST_STOP = 0.02         # stop when camera is within this of DOCK_DIST (from above)
APPROACH_HANDOVER = 2.0  # camera-to-pallet distance [m] where approach hands over
PALLET_MIN_SCORE = 0.5
FRONT_ABORT = 0.22       # m, forward obstacle -> abort (footprint radius 0.30)
FRONT_SLOW = 0.6         # m, forward obstacle -> creep
TIME_APPROACH = 145.0    # s, R-03 is 150 s
TIME_GUIDANCE = 115.0    # s, R-03 is 120 s


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clampf(x, lo, hi):
    return max(lo, min(hi, x))


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage: str) -> None:
        self.phase = 'guidance' if start_stage == 'guidance' else 'approach'
        self.limit = TIME_GUIDANCE if self.phase == 'guidance' else TIME_APPROACH
        self.t0 = None
        # tag yaw sign: yaw = s * (psi + pi/2); learned from odometry if possible
        self.sign = 1.0
        self._sign_acc = 0.0
        self._sign_locked = False
        self._prev_det = None           # (stamp, yaw, odom_yaw)
        self._last_stamp = None
        self.pose = None                # (cx, cy, psi)
        self.last_tag_t = -1e9
        self.last_bearing = 0.0
        self.search_ticks = 0
        self.dock_ticks = 0

    # ------------------------------------------------------------------ utils
    def _finish(self, status):
        return Command(0.0, 0.0, status)

    def _update_sign(self, det, odom_yaw):
        if self._sign_locked:
            return
        if self._prev_det is not None:
            _, pyaw, poyaw = self._prev_det
            dodom = wrap(odom_yaw - poyaw)
            dyaw = wrap(det.yaw - pyaw)
            if abs(dodom) > 0.02:
                self._sign_acc += dyaw * dodom
                if abs(self._sign_acc) > 0.1:
                    self.sign = 1.0 if self._sign_acc > 0 else -1.0
                    self._sign_locked = True
        self._prev_det = (det.stamp, det.yaw, odom_yaw)

    def _pose_from_tag(self, det):
        psi = self.sign * det.yaw - HALF_PI
        e = wrap(psi + HALF_PI)
        x, z = det.x, det.z
        cx = x * math.cos(e) - z * math.sin(e)
        cy = x * math.sin(e) + z * math.cos(e)
        return (cx, cy, psi)

    def _predict(self, obs):
        if self.pose is None:
            return
        cx, cy, psi = self.pose
        v, w = obs.odom.v, obs.odom.w
        mid = psi + 0.5 * w * DT
        cx += v * DT * math.cos(mid)
        cy += v * DT * math.sin(mid)
        self.pose = (cx, cy, psi + w * DT)

    def _front_min(self, scan):
        if scan is None or not scan.ranges:
            return None
        best = None
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) > 0.35 or not math.isfinite(r) or r < 0.01:
                continue
            best = r if best is None else min(best, r)
        return best

    def _pallet_target(self, obs):
        """Return (score, bearing [rad, + right], distance to face [m]) or None."""
        best = None
        for p in obs.pallets:
            if p.score < PALLET_MIN_SCORE or p.width_px <= 1.0:
                continue
            depths = sorted(d for d in p.depth_samples if 0.2 < d < 8.0)
            if depths:
                dist = depths[int(0.3 * (len(depths) - 1))]
                wm = p.width_px * dist / FX      # implied pallet width
                if not (0.6 <= wm <= 1.8):
                    continue
            else:
                dist = FX * PALLET_WIDTH / p.width_px
            b = math.atan2(p.u - IMG_W / 2.0, FX)
            if best is None or p.score > best[0]:
                best = (p.score, b, dist)
        return best

    # ---------------------------------------------------------------- stages
    def _approach(self, obs):
        tgt = self._pallet_target(obs)
        if tgt is None:
            self.search_ticks += 1
            d = 1.0 if self.search_ticks < 200 else -1.0
            return Command(0.0, 0.35 * d)
        self.search_ticks = 0
        _, b, dist = tgt
        w = clampf(-1.5 * b, -W_MAX, W_MAX)
        if dist > APPROACH_HANDOVER:
            v = clampf(0.3 * (dist - APPROACH_HANDOVER), 0.08, 0.25)
        else:
            v = 0.05
        v *= max(0.3, math.cos(b))
        return Command(v, w)

    def _search(self, obs):
        tgt = self._pallet_target(obs)
        if tgt is not None:
            _, b, _ = tgt
            return Command(0.0, clampf(-1.0 * b, -0.3, 0.3))
        self.search_ticks += 1
        d = -1.0 if self.last_bearing > 0 else 1.0
        return Command(0.0, 0.3 * d)

    def _dock(self, obs, cx, cy, psi, front):
        e = wrap(psi - PSI_DOCK)                           # heading error
        dist = cy - DOCK_DIST                              # >0: still too far
        lat_ok = abs(cx) <= 0.05
        # Stage 1 (far): steer onto the centre line while driving in.
        # Stage 2 (near): creep only while lateral offset remains, never below -0.08.
        # Stage 3 (stopped): rotate in place to face the face squarely.
        if dist > DIST_STOP:
            self.dock_ticks = 0
            psi_des = math.atan2(-max(dist, 0.15), -cx)
            delta = wrap(psi_des - psi)
            w = clampf(2.0 * delta, -W_MAX, W_MAX)
            v = clampf(0.8 * dist, 0.05, 0.25) * max(0.25, math.cos(delta)) * clampf(1.0 - abs(cx), 0.5, 1.0)
            if front is not None and front < FRONT_SLOW:
                v = min(v, 0.08)
            return Command(v, w)

        if not lat_ok and dist > -0.12:
            self.dock_ticks = 0
            psi_des = math.atan2(-max(dist, 0.15), -cx)
            w = clampf(2.0 * wrap(psi_des - psi), -W_MAX, W_MAX)
            return Command(0.08, w)

        w = clampf(2.0 * wrap(PSI_DOCK - psi), -W_MAX, W_MAX)
        if abs(e) <= HEAD_TOL and lat_ok:
            self.dock_ticks += 1
            if self.dock_ticks >= 3:
                return self._finish('DOCKED')
        else:
            self.dock_ticks = 0
        return Command(0.0, w)

    # ------------------------------------------------------------------ step
    def step(self, obs: Observation) -> Command:
        if self.t0 is None:
            self.t0 = obs.t
        if obs.t - self.t0 > self.limit:
            return self._finish('ABORTED')

        # tag measurement (process each new frame once)
        det = None
        for d in obs.tags:
            if d.id == TAG_ID and TAG_MIN_Z <= d.z <= TAG_MAX_Z:
                if det is None or d.stamp > det.stamp:
                    det = d
        if det is not None and det.stamp != self._last_stamp:
            self._last_stamp = det.stamp
            self._update_sign(det, obs.odom.yaw)
            self.pose = self._pose_from_tag(det)
            self.last_tag_t = obs.t
            self.last_bearing = math.atan2(det.x, det.z)
        else:
            self._predict(obs)

        tag_ok = self.pose is not None and (obs.t - self.last_tag_t) < TAG_STALE

        if self.phase == 'approach' and tag_ok:
            self.phase = 'guidance'
            self.search_ticks = 0

        if self.phase == 'approach':
            cmd = self._approach(obs)
        elif tag_ok:
            cx, cy, psi = self.pose
            cmd = self._dock(obs, cx, cy, psi, self._front_min(obs.scan))
            if cmd.status != 'RUNNING':
                return cmd
        else:
            self.dock_ticks = 0
            cmd = self._search(obs)

        # forward-obstacle safety
        front = self._front_min(obs.scan)
        if front is not None and front < FRONT_ABORT and cmd.v > 0.0:
            return self._finish('ABORTED')

        return Command(clampf(cmd.v, -V_MAX, V_MAX), clampf(cmd.w, -W_MAX, W_MAX), 'RUNNING')
