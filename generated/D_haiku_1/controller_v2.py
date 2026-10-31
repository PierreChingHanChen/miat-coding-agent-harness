"""AGV pallet-pocket docking controller (REQUIREMENTS.md R-01..R-09, CR-1 R-10).

Pose model (pallet frame, camera used as reference point):
  cx  : camera lateral offset from the pocket centre line [m], + = left of the centre line
  cy  : camera-to-pocket-face distance [m]
  dl  : heading error w.r.t. the face normal [rad], + = turned right
  alpha = atan2(cx, cy)  viewing angle from the face normal
  beta  = alpha - dl     tag bearing in the camera image (+ = right)
Tag measurement (x right, z forward, yaw) with dl = YAW_SIGN * yaw:
  cx = x cos(dl) + z sin(dl),  cy = z cos(dl) - x sin(dl),  beta = atan2(x, z)
Motion (v forward, w left-positive): cx' = -v sin(dl), cy' = -v cos(dl), dl' = -w.
Lateral law: alpha' = (v / cy) * beta, so the tag bearing must be used to remove
alpha, and the amount removable per metre is bounded by BETA_MAX. Offsets that
exceed this bound trigger a reposition (rotate 90 deg, drive sideways on odometry,
rotate back), as required by R-06.

R-10 (CR-1) forward-motion inhibit: any finite LiDAR return within +-30 deg of
straight ahead closer than 0.55 m sets the inhibit state; forward speed is zeroed
in the same tick (rotation and reversing pass through). The inhibit is released
after 1.0 s without such a return. Continuous inhibit for 20 s aborts the mission.
"""
import math

from sim.api import (Command, FX, IMG_W, DT, DOCK_DIST, TAG_ID,
                     PALLET_WIDTH, V_MAX, W_MAX)

# ---- tuning -----------------------------------------------------------------
D_TGT = DOCK_DIST                  # camera-to-face distance at docking [m]
YAW_SIGN = -1.0                    # TagDetection.yaw -> right-turn heading error
DOCK_LAT_TOL = 0.03                # stricter than R-01 (0.06)
DOCK_HDG_TOL = math.radians(3.0)   # stricter than R-01 (6 deg)
DOCK_DIST_TOL = 0.04               # R-01 allows 0.12
DOCK_CONFIRM = 3                   # consecutive fresh ticks inside tolerance
FOV_ENTER = math.radians(22.0)     # tag bearing that forces rotate-to-tag (R-06)
FOV_EXIT = math.radians(12.0)
LOSS_LIMIT = 2.5                   # s without tag before dead reckoning is dropped (R-04)
HOLD_MAX_TICKS = 30                # max ticks holding at the docking band without tag
BLOCK_MAX_TICKS = 30               # max ticks blocked by lidar before abort
LIDAR_FWD_STOP = 0.40              # base-to-obstacle distance that stops forward motion
LIDAR_REAR_STOP = 0.45
LIDAR_ROT_STOP = 0.33              # footprint radius 0.30 m + margin
FWD_INHIBIT_DIST = 0.55            # R-10: forward inhibit range [m]
FWD_INHIBIT_ANG = math.radians(30.0)   # R-10: half-angle of the forward sector
FWD_RESUME_T = 1.0                 # R-10: clear time before forward motion resumes [s]
FWD_INHIBIT_ABORT_T = 20.0         # R-10: continuous inhibit before ABORTED [s]
V_FAR = 0.25
V_CREEP = 0.05
K_V = 0.8                          # distance gain [1/s]
K_ALPHA = 4.0                      # viewing-angle gain of the lateral law
BETA_MAX = 0.30                    # max tag bearing used for lateral correction [rad]
K_H = 2.0                          # heading gain [1/s]
K_B = 1.5                          # bearing gain for pallet-candidate centring
W_REPOS = 1.2
REPOS_V = 0.12                     # sideways drive speed during reposition [m/s]
REPOS_MAX_TICKS = 400
PALLET_MIN_SCORE = 0.45
PALLET_STOP_DIST = 2.0             # stop closing on a pallet candidate; tag should show here (R-05)
LOOK_TICKS = 20                    # wait this long for the tag at a candidate before rejecting
APPROACH_CAND_TICKS = 250          # max ticks spent driving toward one candidate
SEARCH_W = 0.35
BLIND_DIST = 1.0                   # guidance start: blind drive toward pallet centre [m]
BLIND_V = 0.12
REJECT_RADIUS = 1.0
GUIDANCE_BUDGET = 110.0            # R-03 limit 120 s
APPROACH_BUDGET = 140.0            # R-03 limit 150 s


def _finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _scan_gates(scan):
    """Return (front_min, rear_min, all_min) distances from valid lidar returns."""
    front = rear = allm = math.inf
    if scan is None:
        return front, rear, allm
    inc = scan.angle_increment
    for i, r in enumerate(scan.ranges):
        if not _finite(r) or r <= 0.02:
            continue
        a = scan.angle_min + i * inc
        if r < allm:
            allm = r
        if abs(a) <= math.radians(20.0) and r < front:
            front = r
        if abs(_wrap(a - math.pi)) <= math.radians(30.0) and r < rear:
            rear = r
    return front, rear, allm


def _lidar_near_front(scan, dist, half_ang):
    """True if any finite return within +-half_ang of straight ahead is closer than dist."""
    inc = scan.angle_increment
    for i, r in enumerate(scan.ranges):
        if not _finite(r) or r >= dist:
            continue
        if abs(_wrap(scan.angle_min + i * inc)) <= half_ang:
            return True
    return False


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage: str) -> None:
        self._guidance = (start_stage == 'guidance')
        self._budget = GUIDANCE_BUDGET if self._guidance else APPROACH_BUDGET
        self._t0 = None
        self._now = 0.0
        self._terminal = None
        self._mode = 'ACQ'
        self._have_pose = False
        self._cx = 0.0
        self._cy = 0.0
        self._dl = 0.0
        self._beta = 0.0
        self._last_tag_t = -math.inf
        self._rotate_only = False
        self._rstate = None            # None | 'ROT1' | 'LAT' | 'ROT2'
        self._rdir = 1.0
        self._rticks = 0
        self._reversing = False
        self._dock_count = 0
        self._hold = 0
        self._blocked = 0
        self._search_dir = 1.0
        self._rot_block = 0
        self._blind = self._guidance
        self._travelled = 0.0
        self._look = 0
        self._approach_ticks = 0
        self._rejects = []
        self._inhib_on = False         # R-10 forward-motion inhibit state
        self._inhib_start = 0.0
        self._last_near_t = -math.inf

    # ---- public -------------------------------------------------------------
    def step(self, obs):
        try:
            cmd = self._step(obs)
        except Exception:
            self._terminal = 'ABORTED'
            return Command(0.0, 0.0, 'ABORTED')
        # R-10: forward motion is inhibited in the same tick; rotation and reversing pass.
        if self._inhib_on and cmd.v > 0.0:
            cmd = Command(0.0, cmd.w, cmd.status)
        return cmd

    # ---- top level ----------------------------------------------------------
    def _finish(self, status):
        self._terminal = status
        return Command(0.0, 0.0, status)

    def _update_inhibit(self, t, scan):
        """R-10 forward-motion inhibit state machine (no scan: hold the current state)."""
        if scan is None:
            return
        if _lidar_near_front(scan, FWD_INHIBIT_DIST, FWD_INHIBIT_ANG):
            self._last_near_t = t
            if not self._inhib_on:
                self._inhib_on = True
                self._inhib_start = t
        elif self._inhib_on and t - self._last_near_t >= FWD_RESUME_T:
            self._inhib_on = False

    def _step(self, obs):
        if self._terminal is not None:
            return Command(0.0, 0.0, self._terminal)
        t = float(obs.t)
        self._now = t
        if self._t0 is None:
            self._t0 = t
        if t - self._t0 > self._budget:
            return self._finish('ABORTED')

        self._update_inhibit(t, obs.scan)
        if self._inhib_on and t - self._inhib_start >= FWD_INHIBIT_ABORT_T:
            return self._finish('ABORTED')

        odom = obs.odom
        v_meas = float(odom.v)
        w_meas = float(odom.w)
        self._travelled += max(0.0, v_meas) * DT

        tag = self._pick_tag(obs.tags)
        pose = self._pose_from_tag(tag) if tag is not None else None
        fresh = pose is not None and self._plausible(pose)
        if fresh:
            self._cx, self._cy, self._dl, self._beta = pose
            self._have_pose = True
            self._last_tag_t = t
        elif self._have_pose:
            self._predict(v_meas, w_meas)
            # the tag is invisible during a sideways reposition by design: do not drop the pose
            if t - self._last_tag_t > LOSS_LIMIT and self._rstate is None:
                self._have_pose = False

        gates = _scan_gates(obs.scan)

        if fresh and self._mode != 'TRACK':
            self._mode = 'TRACK'
            self._dock_count = 0
            self._hold = 0
            self._blocked = 0
            self._rotate_only = False
            self._rstate = None
            self._reset_candidate()

        if self._mode == 'TRACK' and self._have_pose:
            return self._track(fresh, gates)
        self._mode = 'ACQ'
        return self._acquire(obs, gates)

    # ---- tag and pose -----------------------------------------------------------
    @staticmethod
    def _pick_tag(tags):
        for tg in tags or []:
            if (tg.id == TAG_ID and _finite(tg.x) and _finite(tg.z)
                    and _finite(tg.yaw) and tg.z > 0.2):
                return tg
        return None

    @staticmethod
    def _pose_from_tag(tg):
        dl = YAW_SIGN * float(tg.yaw)
        c, s = math.cos(dl), math.sin(dl)
        cx = tg.x * c + tg.z * s
        cy = tg.z * c - tg.x * s
        beta = math.atan2(tg.x, tg.z)
        return cx, cy, dl, beta

    @staticmethod
    def _plausible(pose):
        cx, cy, dl, _ = pose
        return 0.3 < cy < 5.0 and abs(dl) < 1.2 and abs(cx) < 3.0

    def _predict(self, v, w):
        """Dead reckoning of the camera pose from measured odometry speeds."""
        c, s = math.cos(self._dl), math.sin(self._dl)
        d = v * DT
        self._cx -= d * s
        self._cy -= d * c
        self._dl -= w * DT

    def _reset_candidate(self):
        self._look = 0
        self._approach_ticks = 0

    # ---- track (tag visible, or dead-reckoned after a short loss) -------------
    def _track(self, fresh, g):
        cx, cy, dl = self._cx, self._cy, self._dl
        e = cy - D_TGT
        alpha = math.atan2(cx, max(cy, 0.05))

        # 1. docking check on fresh measurements only
        if fresh and abs(cx) <= DOCK_LAT_TOL and abs(dl) <= DOCK_HDG_TOL \
                and abs(e) <= DOCK_DIST_TOL and self._rstate is None:
            self._dock_count += 1
            if self._dock_count >= DOCK_CONFIRM:
                return self._finish('DOCKED')
            return Command(0.0, 0.0, 'RUNNING')
        self._dock_count = 0

        # 2. at the docking band without a tag: hold for a fresh fix (bounded)
        if not fresh and cy <= D_TGT + 0.05 and self._rstate is None:
            self._hold += 1
            if self._hold > HOLD_MAX_TICKS:
                return self._finish('ABORTED')
            return Command(0.0, 0.0, 'RUNNING')
        if fresh:
            self._hold = 0

        # 3. start a reposition if the offset cannot be removed within the FOV bound
        if self._rstate is None:
            if fresh:
                if abs(self._beta) > FOV_ENTER:
                    self._rotate_only = True
                elif abs(self._beta) < FOV_EXIT:
                    self._rotate_only = False
                if self._rotate_only:
                    w = _clamp(-W_REPOS * self._beta, -W_MAX, W_MAX)
                    if g[2] < LIDAR_ROT_STOP:
                        w = 0.0
                    return Command(0.0, w, 'RUNNING')
            lim = 0.12 + BETA_MAX * math.log(max(cy, D_TGT) / D_TGT)
            if abs(alpha) > lim:
                self._rstate = 'ROT1'
                self._rdir = 1.0 if cx > 0.0 else -1.0
                self._rticks = 0

        if self._rstate is not None:
            return self._reposition(g)

        # 4. distance law with hysteresis (no forward/reverse chatter at the band edge)
        if self._reversing:
            if e > -0.03:
                self._reversing = False
        elif e < -0.06:
            self._reversing = True
        if self._reversing:
            v = -V_CREEP
        elif e > DOCK_DIST_TOL:
            v = _clamp(K_V * e, V_CREEP, V_FAR) * max(0.3, math.cos(dl))
        else:
            # inside the band: keep creeping while the lateral error is removed
            v = V_CREEP if abs(cx) > DOCK_LAT_TOL else 0.0

        if v < 0.0 and g[1] < LIDAR_REAR_STOP:
            v = 0.0
        if v > 0.0 and g[0] < LIDAR_FWD_STOP:
            v = 0.0
            self._blocked += 1
            if self._blocked > BLOCK_MAX_TICKS:
                return self._finish('ABORTED')
        else:
            self._blocked = 0

        # 5. lateral law: choose the tag bearing that drives alpha to zero
        sg = 1.0 if v >= 0.0 else -1.0
        beta_des = -sg * _clamp(K_ALPHA * alpha, -BETA_MAX, BETA_MAX)
        dl_des = _clamp(alpha - beta_des, -1.0, 1.0)
        w = _clamp(K_H * (dl - dl_des), -W_MAX, W_MAX)
        return Command(_clamp(v, -V_MAX, V_MAX), w, 'RUNNING')

    def _reposition(self, g):
        """Rotate 90 deg toward the centre line, drive sideways on odometry, rotate back."""
        self._rticks += 1
        if self._rticks > REPOS_MAX_TICKS:
            return self._finish('ABORTED')
        tgt = self._rdir * math.pi / 2.0
        st = self._rstate
        if st == 'ROT1':
            err = self._dl - tgt
            if abs(err) <= math.radians(4.0):
                self._rstate = 'LAT'
            w = _clamp(K_H * err, -W_MAX, W_MAX) if g[2] >= LIDAR_ROT_STOP else 0.0
            return Command(0.0, w, 'RUNNING')
        if st == 'LAT':
            if self._rdir * self._cx <= 0.03:
                self._rstate = 'ROT2'
                return Command(0.0, 0.0, 'RUNNING')
            if g[0] < LIDAR_FWD_STOP:
                return self._finish('ABORTED')          # obstacle in the travel path
            w = _clamp(K_H * (self._dl - tgt), -W_MAX, W_MAX)
            return Command(REPOS_V, w, 'RUNNING')
        # ROT2: face the face normal again
        if abs(self._dl) <= math.radians(3.0):
            self._rstate = None
            self._last_tag_t = self._now
            return Command(0.0, 0.0, 'RUNNING')
        w = _clamp(K_H * self._dl, -W_MAX, W_MAX) if g[2] >= LIDAR_ROT_STOP else 0.0
        return Command(0.0, w, 'RUNNING')

    # ---- acquire (no usable tag pose) -----------------------------------------
    def _acquire(self, obs, g):
        front = g[0]
        cand = self._best_candidate(obs.pallets, obs.odom)
        if cand is None:
            self._reset_candidate()
            if self._blind:
                if self._travelled >= BLIND_DIST or front < LIDAR_FWD_STOP:
                    self._blind = False
                else:
                    return Command(BLIND_V, 0.0, 'RUNNING')
            return self._search(g)

        est, b, d = cand
        self._approach_ticks += 1
        w = _clamp(-K_B * b, -0.5, 0.5)
        if d <= PALLET_STOP_DIST:
            # close to the candidate: centre on it and wait for the tag
            self._look += 1
            if self._look > LOOK_TICKS:
                self._reject(est)
                return Command(0.0, 0.0, 'RUNNING')
            return Command(0.0, w, 'RUNNING')
        if front < LIDAR_FWD_STOP or self._approach_ticks > APPROACH_CAND_TICKS:
            self._reject(est)
            return Command(0.0, 0.0, 'RUNNING')
        self._look = 0
        v = _clamp(0.35 * (d - PALLET_STOP_DIST), 0.05, V_FAR) if abs(b) < 0.25 else 0.0
        return Command(v, w, 'RUNNING')

    def _reject(self, est):
        self._rejects.append(est)
        self._reset_candidate()

    def _search(self, g):
        if g[2] < LIDAR_ROT_STOP:
            if self._rot_block == 0:
                self._search_dir = -self._search_dir
            self._rot_block = (self._rot_block + 1) % 20
            return Command(0.0, 0.0, 'RUNNING')
        self._rot_block = 0
        return Command(0.0, SEARCH_W * self._search_dir, 'RUNNING')

    def _best_candidate(self, pallets, odom):
        best = None
        best_s = -math.inf
        for det in pallets or []:
            if not _finite(det.score) or det.score < PALLET_MIN_SCORE or not _finite(det.u):
                continue
            b = math.atan2(det.u - IMG_W / 2.0, FX)
            d = self._pallet_dist(det)
            if d is None or d > 6.0:
                continue
            ang = float(odom.yaw) - b
            est = (float(odom.x) + d * math.cos(ang), float(odom.y) + d * math.sin(ang))
            if any(math.hypot(est[0] - r[0], est[1] - r[1]) < REJECT_RADIUS
                   for r in self._rejects):
                continue
            s = det.score - 0.3 * abs(b)
            if s > best_s:
                best_s = s
                best = (est, b, d)
        return best

    @staticmethod
    def _pallet_dist(det):
        """Robust distance to the pallet face: low quantile of valid depth samples."""
        valid = sorted(s for s in (det.depth_samples or [])
                       if _finite(s) and 0.2 < s < 8.0)
        if len(valid) >= 3:
            return valid[int(0.25 * (len(valid) - 1))]
        if _finite(det.width_px) and det.width_px > 1.0:
            return PALLET_WIDTH * FX / det.width_px
        return None
