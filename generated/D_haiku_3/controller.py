"""AGV pallet-pocket docking controller (A3 approach, A4 localise, A5 guidance).

Pose model (pallet frame, pocket-face normal = +y, face on y = 0):
  camera pose P = (px, py), heading theta, optical axis h = (cos th, sin th).
  Tag measurement (x right, z forward, yaw right-positive, 0 = squarely):
      theta = yaw - pi/2
      px    = x*cos(yaw) - z*sin(yaw)
      py    = x*sin(yaw) + z*cos(yaw)      (camera-to-face distance)
  Kinematics for dead reckoning: px' = v cos th, py' = v sin th, th' = w.
Stages: PALLET (drive to pallet candidate, verify tag), TAG (regulate
pose to (0, DOCK_DIST) facing the face), terminal DOCKED / ABORTED.
"""
import math

from sim.api import Command, TAG_ID, DOCK_DIST, FX, DT

TAG_FRESH = 0.35          # s; tag measurement counts as current
TAG_BEARING_MAX = 0.62    # rad; accept tag detections inside this bearing
TAG_BEARING_REPO = 0.50   # rad; beyond this rotate in place to keep tag in FOV (R-06)
YAW_MAX = 1.25            # rad; ignore grazing detections
PAL_SCORE_MIN = 0.45
PAL_VERIFY_DIST = 2.25    # m; closer pallet candidate must show the tag
PAL_MAX_ATTEMPTS = 2      # verification attempts per candidate
PAL_MAX_REJECTS = 6
SEARCH_W = 0.35           # rad/s search rotation
SEARCH_MAX_ANGLE = 4.0 * math.pi
LIDAR_STOP = 0.40         # m; front obstacle stop distance
LIDAR_HALF_SECTOR = 0.35  # rad
BLIND_MAX = 8.0           # s without tag before ABORT when close
BLIND_REACQ = 4.0         # s without tag before re-acquiring when far
DOCK_HOLD_FRESH = 5       # ticks in tolerance (tag fresh)
DOCK_HOLD_BLIND = 15      # ticks in tolerance (dead reckoning)
DOCK_TOL_X = 0.03
DOCK_TOL_Y = 0.07         # well inside the R-01 band 0.85 +/- 0.12
DOCK_TOL_H = math.radians(4.5)   # spec 6 deg; tilt-based steering leaves a small residual
DEADLINE_GUIDANCE = 110.0
DEADLINE_APPROACH = 140.0


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage):
        self.start_stage = start_stage
        self.t0 = None
        self.now = 0.0
        self.deadline = DEADLINE_GUIDANCE if start_stage == 'guidance' else DEADLINE_APPROACH
        self.stage = 'TAG' if start_stage == 'guidance' else 'PALLET'
        self.pose = None            # (px, py, th) camera pose in pallet frame
        self.pose_t = -1e9          # time of last accepted tag measurement
        self.bearing = 0.0          # tag bearing from last measurement (right +)
        self.dock_count = 0
        self.last_odom_yaw = None
        self.search_angle = 0.0
        self.cands = []             # [world_yaw, attempts]
        self.cand = None            # dict: wyaw, b, d, seen
        self.verify_start = None
        self.backoff_until = -1.0
        self.reject_count = 0
        self.blocked_since = None
        self.reacq_count = 0

    # ------------------------------------------------------------------
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 'ABORTED')

    def _stop(self, status):
        return Command(0.0, 0.0, status)

    def _step(self, obs):
        if self.t0 is None:
            self.t0 = obs.t
        self.now = obs.t - self.t0
        odom = obs.odom
        if self.now > self.deadline:
            return self._stop('ABORTED')

        # --- dead reckoning of the camera pose -------------------------
        if self.pose is not None:
            px, py, th = self.pose
            self.pose = (px + odom.v * math.cos(th) * DT,
                         py + odom.v * math.sin(th) * DT,
                         _wrap(th + odom.w * DT))

        # --- search rotation bookkeeping -------------------------------
        if self.last_odom_yaw is not None:
            self.search_angle += abs(_wrap(odom.yaw - self.last_odom_yaw))
        self.last_odom_yaw = odom.yaw

        # --- tag measurement -------------------------------------------
        meas = self._tag_meas(obs.tags)
        if meas is not None:
            mpx, mpy, mth, b = meas
            self.bearing = b
            if self.pose is not None and self.now - self.pose_t < 1.0:
                px, py, th = self.pose
                a = 0.5   # blend: dead reckoning vs fresh measurement
                self.pose = (px + a * (mpx - px), py + a * (mpy - py),
                             _wrap(th + a * _wrap(mth - th)))
            else:
                self.pose = (mpx, mpy, mth)
            self.pose_t = self.now

        if self.stage == 'PALLET':
            if meas is not None and abs(self.bearing) < TAG_BEARING_MAX:
                self.stage = 'TAG'
                self.cand = None
                self.verify_start = None
        if self.stage == 'PALLET':
            return self._pallet_step(obs)
        return self._tag_step(obs)

    # ------------------------------------------------------------------
    def _tag_meas(self, tags):
        best = None
        for d in tags:
            if d.id != TAG_ID or d.z <= 0.05:
                continue
            b = math.atan2(d.x, d.z)
            if abs(b) > TAG_BEARING_MAX or abs(d.yaw) > YAW_MAX:
                continue
            c, s = math.cos(d.yaw), math.sin(d.yaw)
            px = d.x * c - d.z * s
            py = d.x * s + d.z * c
            th = _wrap(d.yaw - math.pi / 2)
            if best is None or abs(b) < abs(best[3]):
                best = (px, py, th, b)
        return best

    def _front_min(self, scan):
        if scan is None:
            return float('inf')
        m = float('inf')
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) <= LIDAR_HALF_SECTOR and math.isfinite(r) and r > 0.05:
                m = min(m, r)
        return m

    def _safe_v(self, v, obs):
        """Block forward motion when an obstacle is in front."""
        if v > 0.0 and self._front_min(obs.scan) < LIDAR_STOP:
            return 0.0
        return v

    @staticmethod
    def _depth_estimate(samples):
        valid = sorted(s for s in samples if 0.3 < s < 8.0)
        if len(valid) < 5:
            return None
        return valid[len(valid) // 4]   # lower quartile: rejects background

    def _is_rejected(self, wyaw):
        for entry in self.cands:
            if abs(_wrap(entry[0] - wyaw)) < 0.25 and entry[1] >= PAL_MAX_ATTEMPTS:
                return True
        return False

    def _find_entry(self, wyaw):
        for entry in self.cands:
            if abs(_wrap(entry[0] - wyaw)) < 0.25:
                return entry
        return None

    # ------------------------------------------------------------------
    def _pallet_step(self, obs):
        odom = obs.odom
        # Pick a new candidate from the pallet detector.
        if self.reject_count > PAL_MAX_REJECTS:
            return self._stop('ABORTED')
        if self.now < self.backoff_until:
            return Command(_clamp(-0.08, -V_BACK, 0.0), 0.0, 'RUNNING')
        best = None
        for p in obs.pallets:
            if p.score < PAL_SCORE_MIN:
                continue
            b = math.atan((p.u - 320.0) / FX)
            wyaw = odom.yaw - b
            if self._is_rejected(wyaw):
                continue
            if best is None or p.score > best[0]:
                best = (p.score, b, wyaw, self._depth_estimate(p.depth_samples))
        if best is not None:
            _, b, wyaw, d = best
            if self.cand is None or abs(_wrap(self.cand['wyaw'] - wyaw)) > 0.25:
                self.verify_start = None
            self.cand = {'wyaw': wyaw, 'b': b, 'd': d, 'seen': self.now}
            if self._find_entry(wyaw) is None:
                self.cands.append([wyaw, 0])

        cand = self.cand
        if cand is not None and self.now - cand['seen'] < 0.6:
            b, d = cand['b'], cand['d']
            w = _clamp(-1.5 * b, -0.5, 0.5)
            if d is None or d > PAL_VERIFY_DIST:
                v = 0.22 if abs(b) < 0.35 else 0.0
                v = self._safe_v(v, obs)
                if self.blocked_since is None and v == 0.0 and abs(b) < 0.35:
                    self.blocked_since = self.now
                elif v > 0.0:
                    self.blocked_since = None
                if self.blocked_since is not None and self.now - self.blocked_since > 4.0:
                    self._reject_candidate(cand)
                    return Command(0.0, 0.0, 'RUNNING')
                return Command(v, w, 'RUNNING')
            # Close to candidate: it must show the tag, otherwise reject.
            if self.verify_start is None:
                self.verify_start = self.now
            if self.now - self.verify_start > 3.0:
                self._reject_candidate(cand)
                self.backoff_until = self.now + 1.5
            return Command(0.0, w, 'RUNNING')

        # No candidate: rotate to search.
        if self.search_angle > SEARCH_MAX_ANGLE:
            return self._stop('ABORTED')
        return Command(0.0, SEARCH_W, 'RUNNING')

    def _reject_candidate(self, cand):
        entry = self._find_entry(cand['wyaw'])
        if entry is None:
            entry = [cand['wyaw'], 0]
            self.cands.append(entry)
        entry[1] += 1
        self.reject_count += 1
        self.cand = None
        self.verify_start = None
        self.blocked_since = None

    # ------------------------------------------------------------------
    def _tag_step(self, obs):
        if self.pose is None:
            self.stage = 'PALLET'
            return self._pallet_step(obs)
        px, py, th = self.pose
        lost = self.now - self.pose_t
        fresh = lost <= TAG_FRESH

        # Docking check (tolerances on the estimated pose).
        ok = (abs(px) <= DOCK_TOL_X and abs(py - DOCK_DIST) <= DOCK_TOL_Y
              and abs(_wrap(th + math.pi / 2)) <= DOCK_TOL_H)
        self.dock_count = self.dock_count + 1 if ok else 0
        need = DOCK_HOLD_FRESH if fresh else DOCK_HOLD_BLIND
        if ok and self.dock_count >= need and (fresh or lost <= 1.5):
            return self._stop('DOCKED')

        # Lost too long: far away -> re-acquire via pallet; close -> bounded blind drive.
        if not fresh:
            if py > 1.3 and lost > BLIND_REACQ:
                if self.reacq_count >= 2:
                    return self._stop('ABORTED')
                self.reacq_count += 1
                self.stage = 'PALLET'
                self.cand = None
                self.verify_start = None
                return self._pallet_step(obs)
            if lost > BLIND_MAX:
                return self._stop('ABORTED')

        if fresh and abs(self.bearing) > TAG_BEARING_REPO:
            # Reposition in place so the tag stays inside the FOV (R-06).
            return Command(0.0, _clamp(-1.5 * self.bearing, -0.5, 0.5), 'RUNNING')

        # Pose regulation to (0, DOCK_DIST) facing the face (th = -pi/2).
        dist = py - DOCK_DIST
        th_t = -math.pi / 2 - math.atan(px / 0.2)   # 0.2 m lookahead to the centre line
        # keep the tag inside the FOV while steering: heading within 0.45 rad of the tag direction
        phi = math.atan2(-py, -px)
        th_t = _clamp(th_t, phi - 0.45, phi + 0.45)
        e = _wrap(th_t - th)
        w = _clamp(2.0 * e, -0.5, 0.5)
        if dist > 0.04:
            v = _clamp(0.9 * dist, 0.05, 0.22)
        elif abs(px) > 0.02:
            # lateral error: only forward motion corrects it (reverse would worsen it)
            v = 0.10 if dist > -0.12 else 0.0
        else:
            v = _clamp(0.9 * dist, -0.03, 0.03)
        if abs(e) > 0.8:
            v = 0.0
        else:
            v *= max(0.0, math.cos(e))
        v = self._safe_v(v, obs)
        return Command(v, w, 'RUNNING')


V_BACK = 0.08
