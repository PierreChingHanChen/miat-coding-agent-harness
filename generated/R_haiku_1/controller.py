"""AGV pallet-pocket docking controller (A3 approach, A4 tag pose, A5 guidance).

Frames: pallet frame has the origin at the AprilTag, +y out of the pocket face
toward the AGV. Camera pose in that frame is recovered from the tag detection:
    heading error yaw (0 = facing the face squarely, + = turned left)
    px  = lateral offset of camera from the pocket centre line
    py  = camera-to-face distance
"""
import math

from sim.api import Command, DOCK_DIST, FX, IMG_W, PALLET_WIDTH, TAG_ID, clamp_cmd

V_FWD_MAX = 0.25
V_REV_MAX = 0.08
W_MAX = 0.5
V_CREEP = 0.03
V_LAT = 0.08
TAG_HOLD_S = 1.0           # hold still this long after losing the tag
TAG_LOST_ABORT_S = 15.0    # abort if no usable tag for this long (guidance)
TAG_FIND_S = 10.0          # after approach hand-over, wait this long for the tag
REJECT_S = 10.0            # ignore a pallet candidate that yielded no tag
PALLET_STOP_D = 2.2        # approach stops here, AprilTag guidance takes over
PALLET_MIN_SCORE = 0.5
TAG_HANDOVER_Z = 2.4       # switch to tag guidance when tag is this close
DOCK_TOL_LAT = 0.04
DOCK_TOL_YAW = 0.05
DOCK_TOL_D = 0.06
DOCK_HOLD = 3              # consecutive aligned ticks before DOCKED
SEARCH_W = 0.25
FRONT_MIN = 0.45           # forward motion blocked below this clearance [m]
REAR_MIN = 0.40
SPIN_MIN = 0.36            # rotation blocked if any beam is this close (disc r=0.30)
BLOCK_ABORT_S = 6.0
APPROACH_TIMEOUT = 145.0
GUIDANCE_TIMEOUT = 115.0
PP_FLOOR = 0.2
PP_V = 0.25
PP_YAW = 0.9
FIN_AIM = 0.1


def _sgn(x):
    return 1.0 if x >= 0.0 else -1.0


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _min_range(scan, pred):
    best = float("inf")
    a = scan.angle_min
    for r in scan.ranges:
        if r is not None and r > 0.0 and pred(a) and r < best:
            best = r
        a += scan.angle_increment
    return best


class DockingController:
    def __init__(self):
        self.reset("approach")

    def reset(self, start_stage):
        self.stage = "guidance" if start_stage == "guidance" else "approach"
        self.t0 = None
        self.stage_t = None
        self.from_approach = False
        self.last_tag_t = None
        self.last_tag_x = 0.0
        self.dock_ticks = 0
        self.blocked_t = None
        self.pallet_t = None
        self.pallet_bearing = 0.0
        self.reject_until = -1.0

    # ------------------------------------------------------------------ API
    def step(self, obs):
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
            self.stage_t = t
        el = t - self.t0
        tag = self._tag(obs.tags)
        if tag is not None:
            self.last_tag_t = t
            self.last_tag_x = tag.x

        limit = APPROACH_TIMEOUT if self.stage == "approach" else GUIDANCE_TIMEOUT
        if el > limit:
            return Command(0.0, 0.0, "ABORTED")

        if self.stage == "approach":
            if tag is not None and tag.z <= TAG_HANDOVER_Z:
                self._to_guidance(t)
            else:
                res = self._approach(obs, t)
                if res is not None:
                    return res
                self._to_guidance(t)  # pallet close enough: find the tag

        return self._guidance(obs, t, tag)

    # ------------------------------------------------------------- helpers
    def _to_guidance(self, t):
        self.stage = "guidance"
        self.stage_t = t
        self.from_approach = True
        self.dock_ticks = 0
        self.blocked_t = None

    def _tag(self, tags):
        best = None
        for g in tags or []:
            if g.id != TAG_ID or not (0.1 < g.z < 6.0):
                continue
            if best is None or g.z < best.z:
                best = g
        return best

    @staticmethod
    def _pose(tag):
        """Camera pose in the pallet frame from one tag detection."""
        c, s = math.cos(tag.yaw), math.sin(tag.yaw)
        px = tag.x * c - tag.z * s       # lateral offset from centre line
        py = tag.x * s + tag.z * c       # camera-to-face distance
        return px, py, tag.yaw

    def _pallet_candidate(self, pallets):
        best = None
        for p in pallets or []:
            if p.score < PALLET_MIN_SCORE:
                continue
            ds = sorted(d for d in p.depth_samples if 0.2 < d < 8.0)
            if len(ds) < 8:
                continue
            cluster = [d for d in ds if d <= ds[0] + 0.25]  # nearest surface
            if len(cluster) < 8:
                continue
            dist = _median(cluster)
            exp_w = FX * PALLET_WIDTH / dist
            if p.width_px <= 0.0 or abs(p.width_px - exp_w) > 0.4 * exp_w:
                continue
            bearing = math.atan2(p.u - IMG_W / 2.0, FX)  # + = right
            if best is None or p.score > best[2]:
                best = (bearing, dist, p.score)
        return best

    def _approach(self, obs, t):
        cand = None
        if t >= self.reject_until:
            cand = self._pallet_candidate(obs.pallets)
        if cand is not None:
            bearing, dist, _ = cand
            self.pallet_t = t
            self.pallet_bearing = bearing
            if dist <= PALLET_STOP_D:
                return None
            w = max(-W_MAX, min(W_MAX, -1.2 * bearing))
            v = max(0.0, min(V_FWD_MAX, 0.5 * (dist - PALLET_STOP_D)))
            v *= max(0.0, 1.0 - abs(bearing) / 0.5)
            if v < 0.05 and abs(bearing) < 0.3:
                v = 0.05
            return self._emit(obs, v, w, t)
        if self.pallet_t is not None and t - self.pallet_t < 1.0:
            return self._emit(obs, 0.0, 0.0, t)
        return self._emit(obs, 0.0, -SEARCH_W * _sgn(self.pallet_bearing), t)

    def _guidance(self, obs, t, tag):
        if tag is None:
            if self.from_approach and self.last_tag_t is None and t - self.stage_t > TAG_FIND_S:
                # no tag behind the candidate: treat it as a decoy and search again
                self.stage = "approach"
                self.stage_t = t
                self.from_approach = False
                self.reject_until = t + REJECT_S
                return self._emit(obs, 0.0, 0.0, t)
            ref = self.last_tag_t if self.last_tag_t is not None else self.stage_t
            if t - ref > TAG_LOST_ABORT_S:
                return Command(0.0, 0.0, "ABORTED")
            if self.last_tag_t is not None and t - self.last_tag_t < TAG_HOLD_S:
                return self._emit(obs, 0.0, 0.0, t)
            return self._emit(obs, 0.0, -SEARCH_W * _sgn(self.last_tag_x), t)

        px, py, yaw = self._pose(tag)
        ed = py - DOCK_DIST
        if abs(px) <= DOCK_TOL_LAT and abs(yaw) <= DOCK_TOL_YAW and abs(ed) <= DOCK_TOL_D:
            self.dock_ticks += 1
            if self.dock_ticks >= DOCK_HOLD:
                return Command(0.0, 0.0, "DOCKED")
        else:
            self.dock_ticks = 0

        # pure pursuit toward the docking point (0, DOCK_DIST) in the pallet frame
        r = math.hypot(px, ed)
        if r > 0.25 and ed > 0.02:
            yaw_des = max(-PP_YAW, min(PP_YAW, math.atan2(-px, max(ed, PP_FLOOR))))
            w = max(-W_MAX, min(W_MAX, -2.5 * (yaw - yaw_des)))
            v = PP_V * max(0.2, math.cos(min(abs(yaw - yaw_des), 1.2)))
        else:
            # final alignment: small heading to pull in the lateral error, zero once centred
            if abs(px) > 0.02:
                yaw_des = max(-0.6, min(0.6, -math.atan2(px, FIN_AIM)))
            else:
                yaw_des = 0.0
            w = max(-W_MAX, min(W_MAX, -1.5 * (yaw - yaw_des)))
            v = max(-V_REV_MAX, min(0.1, 0.8 * ed))
            if abs(v) < V_CREEP and (abs(ed) > DOCK_TOL_D or abs(px) > DOCK_TOL_LAT):
                v = V_CREEP if ed >= 0.0 else -V_CREEP
            if abs(px) > 0.02 and ed > -0.12:
                v = max(v, 0.05)
        return self._emit(obs, v, w, t)

    def _emit(self, obs, v, w, t):
        scan = obs.scan
        if scan is not None:
            front = _min_range(scan, lambda a: abs(a) <= 0.35)
            rear = _min_range(scan, lambda a: abs(a) >= 2.6)
            ring = _min_range(scan, lambda a: True)
            if w != 0.0 and ring < SPIN_MIN:
                w = 0.0
            if (v > 0.0 and front < FRONT_MIN) or (v < 0.0 and rear < REAR_MIN):
                v = 0.0
                if self.blocked_t is None:
                    self.blocked_t = t
                elif t - self.blocked_t > BLOCK_ABORT_S:
                    return Command(0.0, 0.0, "ABORTED")
            else:
                self.blocked_t = None
        return clamp_cmd(Command(v, w, "RUNNING"))
