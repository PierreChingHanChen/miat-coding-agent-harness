"""AGV pallet-pocket docking controller (standard library + sim.api only).

Pose model: camera pose (cx, cy, th) in the pallet frame (tag at origin,
face on y = 0, AGV on y > 0 heading -y when aligned, th = -pi/2).
Tag measurement (x, z, yaw) gives th = -pi/2 + yaw and the camera position
cam = -(z*f + x*r). Between detections the pose is dead-reckoned from odom.
"""
import math

from sim.api import Command, TAG_ID, FX, DOCK_DIST, DT, clamp_cmd

HALF_HFOV = math.radians(69.0) / 2.0
PALLET_CX_PX = 320.0
DOCK_D_LO, DOCK_D_HI = -0.05, 0.03   # accepted camera-to-face error [m]
DOCK_TOL_X = 0.03                    # lateral offset [m]
DOCK_TOL_H = math.radians(2.5)       # heading error [rad]
DOCK_TICKS = 3                       # consecutive ticks inside tolerance
TAG_AGE_FAR = 4.0                    # dead-reckoning allowance [s]
TAG_AGE_NEAR = 6.0                   # (camera closer than 1.0 m to face)
APPROACH_TARGET = 1.9                # camera-to-pallet distance for tag search [m]
VERIFY_TIME = 2.0                    # wait for tag at candidate [s]
CAND_MIN_SCORE = 0.55
REJECT_TOL = 0.25                    # heading tolerance for rejected candidates [rad]
SEARCH_RESET_TICKS = 250             # clear rejections after this many search ticks
TIME_LIMIT = {'guidance': 110.0, 'approach': 140.0}
FRONT_HALF = math.radians(20.0)
FRONT_STOP = 0.38                    # no forward motion closer than this [m]
ALL_STOP = 0.25                      # any return closer than this stops the AGV [m]
MAX_BLOCKED_TICKS = 60               # 6 s of emergency stop -> abort


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _depth_estimate(samples):
    """Robust range from raw depth samples: drop zeros, NaN, and background outliers."""
    vals = []
    for s in samples or []:
        try:
            s = float(s)
        except (TypeError, ValueError):
            continue
        if math.isfinite(s) and 0.3 < s < 8.0:
            vals.append(s)
    if len(vals) < 6:
        return None
    med = _median(vals)
    inl = [v for v in vals if abs(v - med) < 0.2]
    if len(inl) < 5:
        return None
    return _median(inl)


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage):
        self.stage = start_stage if start_stage in ('approach', 'guidance') else 'approach'
        self.t0 = None
        self.last_t = None
        self.pose = [0.0, 0.0, -math.pi / 2.0]
        self.pose_valid = False
        self.last_tag_t = None
        self.pm = 'search'          # pallet-mode sub-state: search | near | verify
        self.pm_t = 0.0
        self.pm_lost_t = None
        self.search_ticks = 0
        self.rejected = []          # odom-yaw headings of candidates that had no tag
        self.last_cand_heading = 0.0
        self.dock_count = 0
        self.blocked = 0

    # ------------------------------------------------------------------ API
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 'ABORTED')

    # ------------------------------------------------------------ internals
    def _final(self, status):
        return Command(0.0, 0.0, status)

    def _step(self, obs):
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
            self.last_t = t
        dt = t - self.last_t
        if not (0.0 < dt < 0.5):
            dt = DT
        self.last_t = t
        if t - self.t0 > TIME_LIMIT.get(self.stage, 110.0):
            return self._final('ABORTED')

        # 1. dead-reckoning of the camera pose in the pallet frame
        if self.pose_valid:
            v, w = float(obs.odom.v), float(obs.odom.w)
            th = self.pose[2]
            self.pose[0] += v * dt * math.cos(th)
            self.pose[1] += v * dt * math.sin(th)
            self.pose[2] = _wrap(th + w * dt)

        # 2. tag measurement fusion
        g = self._best_tag(obs)
        if g is not None:
            self._fuse_tag(g)
            self.last_tag_t = t

        # 3. decide between tag-guided alignment and pallet search
        age = (t - self.last_tag_t) if self.last_tag_t is not None else None
        max_age = TAG_AGE_NEAR if self.pose[1] < 1.0 else TAG_AGE_FAR
        if self.pose_valid and (age is None or age > max_age):
            # tag gone too long: forget the pose and re-acquire via the pallet detector
            self.pose_valid = False
            self.pm = 'search'
            self.pm_lost_t = None
        tag_mode = self.pose_valid

        front, allmin = self._scan_mins(obs.scan)

        if tag_mode:
            res = self._align()
            if res == 'DOCKED':
                return self._final('DOCKED')
            cmd = res
        else:
            cmd = self._pallet_mode(obs, t)

        # 4. safety: LiDAR guard
        if allmin is not None and allmin < ALL_STOP:
            self.blocked += 1
            cmd = Command(0.0, 0.0)
        else:
            self.blocked = 0
        if self.blocked > MAX_BLOCKED_TICKS:
            return self._final('ABORTED')
        if cmd.v > 0.0 and front is not None and front < FRONT_STOP:
            cmd = Command(0.0, cmd.w)

        return clamp_cmd(Command(cmd.v, cmd.w, 'RUNNING'))

    def _best_tag(self, obs):
        best = None
        for g in obs.tags or []:
            if g.id != TAG_ID:
                continue
            if not (0.25 <= g.z <= 4.5):
                continue
            if abs(math.atan2(g.x, g.z)) > HALF_HFOV * 1.05:
                continue
            if abs(g.yaw) > 1.2:
                continue
            if best is None or g.z < best.z:
                best = g
        return best

    def _fuse_tag(self, g):
        th_m = -math.pi / 2.0 + g.yaw
        fx, fy = math.cos(th_m), math.sin(th_m)
        rx, ry = math.sin(th_m), -math.cos(th_m)
        mx = -(g.z * fx + g.x * rx)
        my = -(g.z * fy + g.x * ry)
        if not self.pose_valid:
            self.pose = [mx, my, th_m]
            self.pose_valid = True
        else:
            a = 0.5
            self.pose[0] += a * (mx - self.pose[0])
            self.pose[1] += a * (my - self.pose[1])
            self.pose[2] = _wrap(self.pose[2] + a * _wrap(th_m - self.pose[2]))

    def _align(self):
        cx, cy = self.pose[0], self.pose[1]
        herr = _wrap(self.pose[2] + math.pi / 2.0)     # heading error vs face normal
        dd = cy - DOCK_DIST

        if DOCK_D_LO <= dd <= DOCK_D_HI and abs(cx) <= DOCK_TOL_X and abs(herr) <= DOCK_TOL_H:
            self.dock_count += 1
        else:
            self.dock_count = 0
        if self.dock_count >= DOCK_TICKS:
            return 'DOCKED'

        # pure-pursuit style steering onto the pocket centre line
        L = 0.6 if cy > 1.2 else 0.45
        target = -math.pi / 2.0 - math.atan(cx / L)
        herr_t = _wrap(self.pose[2] - target)
        w = _clamp(-1.5 * herr_t, -0.6, 0.6)

        if abs(herr) > 0.6 and cy > 1.0:
            v = 0.0                                    # reposition in place (R-06)
        elif dd < 0.0:
            # too close: back off only when straight and aligned, otherwise hold
            v = -0.05 if (abs(cx) < DOCK_TOL_X and abs(herr) < DOCK_TOL_H) else 0.0
        else:
            v = _clamp(0.8 * dd, 0.0, 0.25) * max(0.3, 1.0 - abs(herr_t) / 0.8)
        return Command(v, w)

    def _candidate(self, obs):
        """Best pallet-like detection not previously rejected -> (bearing, depth)."""
        best, bestk = None, -1e9
        for p in obs.pallets or []:
            if p.score < CAND_MIN_SCORE:
                continue
            b = math.atan2(p.u - PALLET_CX_PX, FX)
            h = obs.odom.yaw + b
            if any(abs(_wrap(h - r)) < REJECT_TOL for r in self.rejected):
                continue
            k = p.score - 0.2 * abs(b)
            if k > bestk:
                best, bestk = (b, _depth_estimate(p.depth_samples)), k
        if best is not None:
            self.last_cand_heading = obs.odom.yaw + best[0]
        return best

    def _pallet_mode(self, obs, t):
        cand = self._candidate(obs)

        if self.pm == 'search':
            self.search_ticks += 1
            if self.search_ticks > SEARCH_RESET_TICKS:
                self.rejected = []
                self.search_ticks = 0
            if cand is not None:
                self.pm = 'near'
                self.pm_lost_t = None
                self.search_ticks = 0
            else:
                return Command(0.0, 0.3)               # rotate in place to scan

        if self.pm == 'near':
            if cand is None:
                if self.pm_lost_t is None:
                    self.pm_lost_t = t
                if t - self.pm_lost_t > 2.0:
                    self.pm = 'search'
                    self.pm_lost_t = None
                return Command(0.0, 0.3)
            self.pm_lost_t = None
            b, d = cand
            w = _clamp(-1.0 * b, -0.5, 0.5)
            if abs(b) > 0.25:
                v = 0.0
            elif d is None:
                v = 0.08
            elif d > APPROACH_TARGET:
                v = _clamp(0.5 * (d - APPROACH_TARGET), 0.05, 0.25)
            else:
                self.pm = 'verify'
                self.pm_t = t
                v = 0.0
            return Command(v, w)

        # verify: stop in front of the candidate and wait for the tag
        if t - self.pm_t > VERIFY_TIME:
            self.rejected.append(self.last_cand_heading)
            self.pm = 'search'
            return Command(0.0, 0.0)
        w = _clamp(-1.0 * cand[0], -0.3, 0.3) if cand is not None else 0.0
        return Command(0.0, w)

    def _scan_mins(self, scan):
        if scan is None or not scan.ranges:
            return None, None
        front, allmin = None, None
        for i, r in enumerate(scan.ranges):
            try:
                r = float(r)
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(r) and r > 0.0):
                continue
            a = scan.angle_min + i * scan.angle_increment
            allmin = r if allmin is None else min(allmin, r)
            if abs(_wrap(a)) <= FRONT_HALF:
                front = r if front is None else min(front, r)
        return front, allmin
