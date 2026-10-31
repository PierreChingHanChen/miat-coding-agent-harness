"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guidance).

Pose model, pallet frame: camera position C = (cx, cy), camera heading phi with
forward vector (sin phi, -cos phi). cy is the camera-to-face distance, cx the lateral
offset from the pocket centre line, phi the heading error w.r.t. the face normal.
A tag detection (x, z, yaw) maps to  cx = x cos(yaw) - z sin(yaw),
cy = x sin(yaw) + z cos(yaw), phi = yaw. Between fixes the pose is dead-reckoned
from odometry anchored at the last fix. Stdlib only, no blocking, no randomness.
"""
import math

from sim.api import Command, TAG_ID, CAM_X, DOCK_DIST, FX, IMG_W, V_MAX, W_MAX

GUIDANCE_LIMIT = 110.0      # s, internal deadline (requirement: 120 s)
APPROACH_LIMIT = 140.0      # s, internal deadline (requirement: 150 s)
PALLET_MIN_SCORE = 0.45
VERIFY_DEPTH = 1.6          # m, pallet closer than this -> expect the tag
VERIFY_TIMEOUT = 6.0        # s, tag must appear near a candidate
CONTACT_R = 0.33            # m, footprint radius 0.30 plus margin
CONTACT_MAX_S = 3.0
FRONT_STOP = 0.45           # m, forward speed blocked inside this range
DOCK_CY = (0.76, 0.94)
DOCK_LAT = 0.03
DOCK_HDG = 0.05
DOCK_TICKS = 5
CARROT = 0.0                # m, steering target short of the docking pose
STEER_K1 = 0.5
STEER_K2 = 2.0
STEER_K3 = -2.0
STEER_KV = 0.8
TAG_BLIND_MAX = 4.0         # s, dead-reckoning limit
SEEK_SPIN = 0.35
REJECT_RADIUS = 1.0
MAX_REJECTED = 30


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DockingController:
    def __init__(self):
        self.reset("approach")

    def reset(self, start_stage):
        self.guidance = (start_stage == "guidance")
        self.limit = GUIDANCE_LIMIT if self.guidance else APPROACH_LIMIT
        self.t0 = None
        self.t = 0.0
        self.state = "SEEK"
        self.anchor = None          # (bx, by, phi0, ox, oy, oyaw0, beta)
        self.last_fix_t = None
        self.dock_ticks = 0
        self.rejected = []
        self.cand_pos = None
        self.verify_t0 = None
        self.lost_since = None
        self.blind_since = None
        self.seek_fwd_until = None
        self.exc = 0

    # ------------------------------------------------------------------ API
    def step(self, obs):
        try:
            cmd = self._step(obs)
            self.exc = 0
        except Exception:
            self.exc += 1
            cmd = self._abort() if self.exc >= 3 else Command(0.0, 0.0, "RUNNING")
        v, w = float(cmd.v), float(cmd.w)
        if not (math.isfinite(v) and math.isfinite(w)):
            v, w = 0.0, 0.0
        return Command(_clip(v, -V_MAX, V_MAX), _clip(w, -W_MAX, W_MAX), cmd.status)

    # ------------------------------------------------------------ top level
    def _step(self, obs):
        if self.state in ("DOCKED", "ABORTED"):
            return Command(0.0, 0.0, self.state)

        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
            if self.guidance:
                self.seek_fwd_until = t + 3.0
        self.t = t
        if t - self.t0 > self.limit:
            return self._abort()

        odom = obs.odom
        front = self._scan_min(obs.scan, 0.87)      # +-50 deg forward sector
        allmin = self._scan_min(obs.scan, math.pi)  # all around

        tag = self._pick_tag(obs.tags)
        if tag is not None and self._fix(tag, odom):
            self.last_fix_t = t
            if self.state != "TAG":
                self._set_state("TAG")
                self.dock_ticks = 0

        v, w = 0.0, 0.0
        if self.state == "TAG":
            r = self._tag_cmd(odom, t)
            if r == "DOCK":
                self._set_state("DOCKED")
                return Command(0.0, 0.0, "DOCKED")
            if r is not None:
                v, w = r
        elif self.state == "PALLET":
            v, w = self._pallet_cmd(obs, odom, front, t)
        else:
            v, w = self._seek_cmd(obs, odom, front, t)

        # safety overlay
        if v > 0.0 and front < FRONT_STOP:
            v = 0.0
        if allmin < CONTACT_R:
            v, w = 0.0, 0.0
            if self.blind_since is None:
                self.blind_since = t
            elif t - self.blind_since > CONTACT_MAX_S:
                return self._abort()
        else:
            self.blind_since = None
        return Command(v, w, "RUNNING")

    def _set_state(self, s):
        self.state = s

    def _abort(self):
        self.state = "ABORTED"
        return Command(0.0, 0.0, "ABORTED")

    # ------------------------------------------------------------- SEEK
    def _seek_cmd(self, obs, odom, front, t):
        if self._best_pallet(obs.pallets, odom) is not None:
            self._set_state("PALLET")
            self.verify_t0 = None
            self.lost_since = None
            return self._pallet_cmd(obs, odom, front, t)
        if self.seek_fwd_until is not None and t < self.seek_fwd_until:
            return (0.15 if front > FRONT_STOP + 0.1 else 0.0), 0.0
        return 0.0, SEEK_SPIN

    # ----------------------------------------------------------- PALLET
    def _pallet_cmd(self, obs, odom, front, t):
        best = self._best_pallet(obs.pallets, odom)
        if best is None:
            if self.verify_t0 is not None:
                if t - self.verify_t0 > VERIFY_TIMEOUT:
                    return self._drop_candidate()
                sign = 1.0 if int((t - self.verify_t0) / 2.5) % 2 == 0 else -1.0
                return 0.0, 0.25 * sign       # sweep for the tag
            if self.lost_since is None:
                self.lost_since = t
            if t - self.lost_since > 1.5:
                self.lost_since = None
                self._set_state("SEEK")
            return 0.0, 0.0

        self.lost_since = None
        th, d, pos = best
        self.cand_pos = pos
        if d is None:
            d = 3.0
        if d <= VERIFY_DEPTH:
            # close to a candidate: the tag must show up, otherwise it is a false pallet
            if self.verify_t0 is None:
                self.verify_t0 = t
            if t - self.verify_t0 > VERIFY_TIMEOUT:
                return self._drop_candidate()
            v = 0.08 if d > 1.3 else 0.0
            return v, _clip(-2.0 * th, -0.5, 0.5)

        self.verify_t0 = None
        if front < 0.5:                      # blocked by something that is not the pallet
            return self._drop_candidate()
        v = 0.25 * _clip(1.0 - abs(th) / 0.35, 0.0, 1.0)
        w = _clip(-2.0 * th, -0.6, 0.6)
        return v, w

    def _drop_candidate(self):
        if self.cand_pos is not None:
            self.rejected.append(self.cand_pos)
            del self.rejected[:-MAX_REJECTED]
        self.cand_pos = None
        self.verify_t0 = None
        self.lost_since = None
        self._set_state("SEEK")
        return 0.0, 0.0

    # -------------------------------------------------------------- TAG
    def _tag_cmd(self, odom, t):
        p = self._pose(odom)
        if p is None or self.last_fix_t is None:
            self._set_state("SEEK")
            return None
        age = t - self.last_fix_t
        if age > TAG_BLIND_MAX:
            self.anchor = None
            self.dock_ticks = 0
            self._set_state("SEEK")
            return None

        bx, by, phi = p                       # base_link pose in pallet frame
        cx = bx + CAM_X * math.sin(phi)       # camera pose (docking check)
        cy = by - CAM_X * math.cos(phi)
        aligned = (abs(cx) <= DOCK_LAT and abs(phi) <= DOCK_HDG
                   and DOCK_CY[0] <= cy <= DOCK_CY[1])
        self.dock_ticks = self.dock_ticks + 1 if aligned else 0
        if self.dock_ticks >= DOCK_TICKS:
            return "DOCK"

        # Steering: polar pose regulation (Aicardi-type) of the base_link towards the
        # docking pose on the centre line (optionally CARROT m short of it). Standard
        # frame: xs = -y (forward), ys = x (left), heading psi = phi.
        xs, ys = -by, bx
        dx = -(DOCK_DIST + CAM_X - CARROT) - xs
        dy = -ys
        rho = math.hypot(dx, dy)
        alpha = _wrap(math.atan2(dy, dx) - phi)
        beta = _wrap(-phi - alpha)
        k1, k2, k3 = STEER_K1, STEER_K2, STEER_K3
        sinc = math.sin(alpha) / alpha if abs(alpha) > 1e-4 else 1.0
        w = k2 * alpha + k1 * sinc * math.cos(alpha) * (alpha + k3 * beta)

        # Speed: Aicardi-type v = K1V * rho * cos(alpha), so the base slows and turns
        # in place when close to the docking pose; floor keeps the final creep alive.
        if rho > 0.03 and math.cos(alpha) > 0.0:
            v = _clip(STEER_KV * rho * math.cos(alpha), 0.05, V_MAX)
        else:
            v = 0.0
        if cy <= 0.92:
            v = min(v, 0.08)
        if age > 1.0:
            v = min(v, 0.08)
        return v, w

    # ------------------------------------------------------ helpers
    def _pick_tag(self, tags):
        best = None
        for d in tags or []:
            if getattr(d, "id", None) != TAG_ID:
                continue
            if best is None or d.z < best.z:
                best = d
        return best

    def _fix(self, tag, odom):
        try:
            x, z, a = float(tag.x), float(tag.z), float(tag.yaw)
        except (TypeError, ValueError):
            return False
        if not (math.isfinite(x) and math.isfinite(z) and math.isfinite(a)):
            return False
        if z <= 0.05 or abs(a) > 1.3:
            return False
        cx = x * math.cos(a) - z * math.sin(a)
        cy = x * math.sin(a) + z * math.cos(a)
        if cy <= 0.05 or cy > 6.0:
            return False
        bx = cx - CAM_X * math.sin(a)         # base_link = camera - CAM_X * forward
        by = cy + CAM_X * math.cos(a)
        beta = (a - math.pi / 2.0) - odom.yaw  # odom-frame -> pallet-frame rotation
        self.anchor = (bx, by, a, odom.x, odom.y, odom.yaw, beta)
        return True

    def _pose(self, odom):
        if self.anchor is None:
            return None
        bx, by, phi0, ox, oy, oyaw0, beta = self.anchor
        dx = odom.x - ox
        dy = odom.y - oy
        cb, sb = math.cos(beta), math.sin(beta)
        Dx = dx * cb - dy * sb
        Dy = dx * sb + dy * cb
        phi = phi0 + _wrap(odom.yaw - oyaw0)
        base_x = bx + Dx
        base_y = by + Dy
        return base_x, base_y, phi

    def _pallet_depth(self, p):
        vals = []
        for d in p.depth_samples or []:
            try:
                d = float(d)
            except (TypeError, ValueError):
                continue
            if math.isfinite(d) and 0.25 < d < 8.0:
                vals.append(d)
        if len(vals) < 3:
            return None
        vals.sort()
        best, bestn = None, 0
        for v in vals:                       # densest cluster, nearest wins ties
            n = sum(1 for u in vals if v <= u <= v + 0.3)
            if n > bestn:
                best, bestn = v, n
        if bestn < 3:
            return None
        cl = sorted(u for u in vals if best <= u <= best + 0.3)
        return cl[len(cl) // 2]

    def _best_pallet(self, pallets, odom):
        best, best_score = None, -1.0
        for p in pallets or []:
            try:
                score = float(p.score)
                u = float(p.u)
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(score) and math.isfinite(u)) or score < PALLET_MIN_SCORE:
                continue
            th = math.atan((u - IMG_W / 2.0) / FX)
            d = self._pallet_depth(p)
            rng = d if d is not None else 2.5
            px = odom.x + rng * math.cos(odom.yaw + th)
            py = odom.y + rng * math.sin(odom.yaw + th)
            if any(math.hypot(px - rx, py - ry) < REJECT_RADIUS for rx, ry in self.rejected):
                continue
            if score > best_score:
                best, best_score = (th, d, (px, py)), score
        return best

    def _scan_min(self, scan, half):
        if scan is None:
            return math.inf
        best = math.inf
        a0 = scan.angle_min
        inc = scan.angle_increment
        for i, r in enumerate(scan.ranges):
            try:
                r = float(r)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(r) or r < 0.12:
                continue
            if abs(_wrap(a0 + i * inc)) <= half and r < best:
                best = r
        return best
