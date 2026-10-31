"""AGV pallet-pocket docking controller (A3 approach, A4 localize, A5 guide).

Two stages:
  pallet  - turn toward the best plausible YOLACT pallet detection and drive
            until the camera is ~2.2 m from the pallet face; creep forward
            until the AprilTag (id 7) is seen.
  tag     - closed-loop guidance on the tag pose. Bearing alpha = atan2(x, z)
            and tag yaw are both driven to zero by w = -(KA*alpha + KY*yaw);
            forward speed regulates the camera-to-face distance
            d_face = r*cos(yaw - alpha) to DOCK_DIST. DOCKED when the camera is
            on the centre line, facing the face, at distance, and stable.
LiDAR is used only as a safety gate on forward motion.
"""
import math

from sim.api import (Command, Observation, FX, DOCK_DIST, TAG_ID, V_MAX, W_MAX,
                     PALLET_WIDTH)

APPROACH_LIMIT = 150.0      # R-03 [s]
GUIDANCE_LIMIT = 120.0
END_MARGIN = 3.0            # return ABORTED this long before the limit

PALLET_SCORE_MIN = 0.5
PALLET_STOP_DIST = 2.2      # R-04: approach stops at ~2.2 m
PALLET_GATE_PX = 150.0      # prefer detections near the last locked column
CREEP_V = 0.05

KA = 1.2                    # bearing gain
KY = 1.0                    # tag yaw gain
KD = 0.8                    # distance gain
SEARCH_W = 0.35

DOCK_TOL_D = 0.04           # |d_face - DOCK_DIST| [m]
DOCK_TOL_LAT = 0.035        # [m]
DOCK_TOL_ANG = math.radians(4.0)
DOCK_HOLD_TICKS = 5
TAG_COAST_TICKS = 5         # keep last command this long when tag drops out
TAG_RECOVER_W = 0.25

FRONT_HALF_ANGLE = math.radians(25.0)
STOP_FRONT = 0.40           # no forward motion closer than this
STOP_ALL = 0.20             # no motion at all closer than this


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DockingController:
    def __init__(self):
        self.start = 'approach'
        self.reset('approach')

    def reset(self, start_stage: str) -> None:
        self.start = 'approach' if start_stage == 'approach' else 'guidance'
        self.stage = 'pallet' if self.start == 'approach' else 'tag'
        self.t0 = None
        self.dock_count = 0
        self.last_cmd = Command(0.0, 0.0)
        self.tag_lost = 0
        self.last_alpha = None
        self.last_yaw = None
        self.last_u = None
        self.last_bearing = None
        self.pallet_lost = 0

    # ------------------------------------------------------------------
    def step(self, obs: Observation) -> Command:
        if self.t0 is None:
            self.t0 = obs.t
        limit = APPROACH_LIMIT if self.start == 'approach' else GUIDANCE_LIMIT
        if obs.t - self.t0 >= limit - END_MARGIN:
            return Command(0.0, 0.0, 'ABORTED')

        tag = None
        for t in obs.tags:
            if t.id == TAG_ID and t.z > 0.05:
                if tag is None or t.z < tag.z:
                    tag = t
        if tag is not None:
            self.stage = 'tag'

        if self.stage == 'tag':
            cmd = self._guidance(obs, tag)
        else:
            cmd = self._approach(obs)
        return self._safety(obs, cmd)

    # ------------------------------------------------------------------
    def _guidance(self, obs, tag):
        if tag is not None:
            self.tag_lost = 0
            x, z, yaw = tag.x, tag.z, tag.yaw
            alpha = math.atan2(x, z)
            r = math.hypot(x, z)
            psi = yaw - alpha
            d_face = r * math.cos(psi)
            d_lat = r * math.sin(psi)
            self.last_alpha, self.last_yaw = alpha, yaw

            w = -(KA * alpha + KY * yaw)
            align = abs(alpha) + abs(yaw)
            v = _clamp(KD * (d_face - DOCK_DIST), -0.05, V_MAX)
            v *= max(0.0, 1.0 - 2.0 * align)

            docked_geom = (abs(d_face - DOCK_DIST) <= DOCK_TOL_D
                           and abs(d_lat) <= DOCK_TOL_LAT
                           and abs(yaw) <= DOCK_TOL_ANG)
            stopped = abs(obs.odom.v) <= 0.05
            if docked_geom and stopped:
                self.dock_count += 1
            else:
                self.dock_count = 0
            if self.dock_count >= DOCK_HOLD_TICKS:
                return Command(0.0, 0.0, 'DOCKED')

            cmd = Command(v, _clamp(w, -W_MAX, W_MAX))
            self.last_cmd = cmd
            return cmd

        # tag not visible
        self.tag_lost += 1
        self.dock_count = 0
        if self.tag_lost <= TAG_COAST_TICKS:
            return Command(self.last_cmd.v * 0.5, self.last_cmd.w)

        # recover by rotating toward the last known tag bearing
        ref = self.last_alpha if self.last_alpha is not None else 0.0
        if abs(ref) < 0.02 and self.last_yaw is not None:
            ref = -self.last_yaw
        direction = -1.0 if ref > 0 else 1.0
        return Command(0.0, TAG_RECOVER_W * direction)

    # ------------------------------------------------------------------
    def _pallet_candidates(self, pallets):
        cands = []
        for p in pallets:
            if p.score < PALLET_SCORE_MIN:
                continue
            valid = sorted(s for s in p.depth_samples if s > 0.2)
            if valid:
                d = valid[len(valid) // 4]          # lower quartile: skip background
            elif p.width_px > 1.0:
                d = FX * PALLET_WIDTH / p.width_px
            else:
                continue
            if d <= 0.3 or d > 8.0:
                continue
            if p.width_px > 1.0:
                ratio = p.width_px / (FX * PALLET_WIDTH / d)
                if not (0.5 <= ratio <= 1.6):      # width must match a 1.2 m face
                    continue
            cands.append((p, d))
        return cands

    def _approach(self, obs):
        cands = self._pallet_candidates(obs.pallets)
        if self.last_u is not None:
            near = [c for c in cands if abs(c[0].u - self.last_u) <= PALLET_GATE_PX]
            if near:
                cands = near
        if not cands:
            self.pallet_lost += 1
            self.last_u = None if self.pallet_lost > 30 else self.last_u
            if self.last_bearing is not None and self.pallet_lost <= 30:
                direction = -1.0 if self.last_bearing > 0 else 1.0
            else:
                direction = 1.0
            return Command(0.0, SEARCH_W * direction)

        p, d = max(cands, key=lambda c: c[0].score)
        self.pallet_lost = 0
        self.last_u = p.u
        bearing = math.atan2(p.u - 320.0, FX)
        self.last_bearing = bearing
        w = _clamp(-1.2 * bearing, -W_MAX, W_MAX)

        if d > PALLET_STOP_DIST:
            v = _clamp(0.6 * (d - PALLET_STOP_DIST), 0.0, V_MAX)
            v *= max(0.0, 1.0 - 2.0 * abs(bearing))
        else:
            v = CREEP_V * max(0.0, 1.0 - 2.0 * abs(bearing))   # wait for the tag
        return Command(v, w)

    # ------------------------------------------------------------------
    def _safety(self, obs, cmd):
        if cmd.status != 'RUNNING':
            return cmd
        v, w = cmd.v, cmd.w
        scan = obs.scan
        if scan is not None and scan.ranges:
            min_front = float('inf')
            min_all = float('inf')
            for i, r in enumerate(scan.ranges):
                if r is None or math.isnan(r) or r <= 0.05 or math.isinf(r):
                    continue
                ang = scan.angle_min + i * scan.angle_increment
                if r < min_all:
                    min_all = r
                if abs(ang) <= FRONT_HALF_ANGLE and r < min_front:
                    min_front = r
            if min_front < STOP_FRONT and v > 0.0:
                v = 0.0
            if min_all < STOP_ALL:
                v = min(v, 0.0)
                w = 0.0
        v = _clamp(v, -V_MAX, V_MAX)
        w = _clamp(w, -W_MAX, W_MAX)
        out = Command(v, w)
        self.last_cmd = out
        return out
