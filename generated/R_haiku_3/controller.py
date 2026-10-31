"""AGV pallet-pocket docking controller.

A3: pallet search and approach with the YOLACT detector (depth-validated).
A4/A5: AprilTag guidance onto the pocket centre line, then DOCKED.

Geometry used (face frame: origin at tag, +Y toward AGV, camera heading psi
measured from +Y toward +X, camera right = -X when facing the face):
  D   = z*cos(yaw) - x*sin(yaw)   camera-to-face distance
  L   = x*cos(yaw) + z*sin(yaw)   camera lateral offset from centre line
  psi = -yaw                      camera heading error w.r.t. face normal
Control: w = K_YAW * (yaw - K_LAT * L / D), v tracks D to DOCK_DIST.
"""
import math

from sim.api import Command, IMG_W, FX, V_MAX, W_MAX, DOCK_DIST, TAG_ID

APPROACH_LIMIT = 150.0
GUIDANCE_LIMIT = 120.0
TIME_MARGIN = 3.0
COLLISION_R = 0.22          # any beam closer than this -> ABORTED
FRONT_STOP = 0.50           # no forward motion below this front clearance
REAR_STOP = 0.35
SEARCH_W = 0.40
TAG_FRESH = 0.35            # s, age limit of a tag detection
TAG_SWITCH_Z = 2.9          # tag range at which approach hands over to guidance
PALLET_HANDOVER_D = 2.0     # camera-to-face distance at which guidance takes over
PALLET_SCORE_MIN = 0.4
PALLET_W_RANGE = (0.8, 1.8)
PALLET_INLIERS_MIN = 10
K_YAW = 2.5
K_LAT = 2.0
K_V = 0.6
CREEP_V = 0.03
LOST_TAG_APPROACH = 6.0
LOST_TAG_GUIDANCE = 8.0
RELAX_AFTER = 20.0


def _finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage):
        self.origin = 'guidance' if start_stage == 'guidance' else 'approach'
        self.mode = 'TAG' if self.origin == 'guidance' else 'SEARCH'
        self.t0 = None
        self.tag_geo = None          # (D, L, yaw, z)
        self.tag_stamp = None
        self.tag_t = -1e9
        self.tag_t0 = 0.0
        self.good = 0
        self.last_side = 1.0
        self.pal = None
        self.pal_t = -1e9
        self.lost_since = None

    # ------------------------------------------------------------------ API
    def step(self, obs):
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
        limit = GUIDANCE_LIMIT if self.origin == 'guidance' else APPROACH_LIMIT
        if t - self.t0 > limit - TIME_MARGIN:
            return Command(0.0, 0.0, 'ABORTED')

        front, rear, nearest = self._lidar(obs.scan)
        if nearest < COLLISION_R:
            return Command(0.0, 0.0, 'ABORTED')

        self._update_tag(obs.tags or [], t)
        pal = self._update_pallet(obs.pallets or [], t)
        tag_fresh = self.tag_geo is not None and (t - self.tag_t) < TAG_FRESH
        tag_close = tag_fresh and self.tag_geo[3] <= TAG_SWITCH_Z

        if self.mode == 'SEARCH':
            if tag_close:
                self._enter_tag(t)
            elif pal is not None:
                self.mode = 'APPROACH'
        if self.mode == 'APPROACH':
            if tag_close or (pal is not None and pal['dist'] <= PALLET_HANDOVER_D):
                self._enter_tag(t)

        if self.mode == 'SEARCH':
            return Command(0.0, SEARCH_W * self.last_side * -1.0, 'RUNNING')
        if self.mode == 'APPROACH':
            return self._approach(pal, t, front)
        return self._guide(t, front, rear, pal, tag_fresh)

    # ------------------------------------------------------------ perception
    def _update_tag(self, tags, t):
        best = None
        for d in tags:
            if getattr(d, 'id', None) != TAG_ID:
                continue
            if not (_finite(d.x) and _finite(d.z) and _finite(d.yaw)) or d.z <= 0.05:
                continue
            if best is None or d.z < best.z:
                best = d
        if best is None:
            return
        stamp = best.stamp if _finite(best.stamp) else t
        if self.tag_stamp is not None and stamp == self.tag_stamp:
            return                      # same detection as last tick
        self.tag_stamp = stamp
        self.tag_t = stamp
        c, s = math.cos(best.yaw), math.sin(best.yaw)
        D = best.z * c - best.x * s
        L = best.x * c + best.z * s
        self.tag_geo = (D, L, best.yaw, best.z)
        self.last_side = 1.0 if best.x >= 0.0 else -1.0
        strict = (abs(D - DOCK_DIST) <= 0.03 and abs(L) <= 0.025
                  and abs(best.yaw) <= math.radians(2.0))
        self.good = self.good + 1 if strict else 0

    def _pallet_cand(self, p):
        samples = [float(s) for s in getattr(p, 'depth_samples', [])
                   if _finite(s) and 0.3 < s < 15.0]
        if len(samples) < 10 or not _finite(p.u) or not _finite(p.width_px):
            return None
        samples.sort()
        med = samples[len(samples) // 2]
        inliers = sum(1 for s in samples if abs(s - med) < 0.12)
        if inliers < PALLET_INLIERS_MIN:
            return None
        width_m = p.width_px * med / FX          # metric width of the pallet
        if not (PALLET_W_RANGE[0] <= width_m <= PALLET_W_RANGE[1]):
            return None
        if p.score < PALLET_SCORE_MIN:
            return None
        return {'u': float(p.u), 'dist': med, 'score': float(p.score),
                'bearing': math.atan2(p.u - IMG_W / 2.0, FX)}

    def _update_pallet(self, pallets, t):
        cands = [c for c in (self._pallet_cand(p) for p in pallets) if c]
        if not cands:
            if self.pal is not None and t - self.pal_t > 1.5:
                self.pal = None
            return None
        chosen = None
        if self.pal is not None and t - self.pal_t < 1.5:
            gated = [c for c in cands
                     if abs(c['u'] - self.pal['u']) < 120.0
                     and abs(c['dist'] - self.pal['dist']) < 0.6]
            if gated:
                chosen = min(gated, key=lambda c: abs(c['u'] - self.pal['u'])
                             + 300.0 * abs(c['dist'] - self.pal['dist']))
        if chosen is None:
            chosen = max(cands, key=lambda c: c['score'])
        self.pal = chosen
        self.pal_t = t
        return chosen

    @staticmethod
    def _lidar(scan):
        front = rear = nearest = math.inf
        if scan is None or not scan.ranges:
            return front, rear, nearest
        for i, r in enumerate(scan.ranges):
            if not _finite(r) or r <= 0.0:
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            nearest = min(nearest, r)
            if abs(a) < 0.45:
                front = min(front, r)
            elif abs(a) > 2.6:
                rear = min(rear, r)
        return front, rear, nearest

    # --------------------------------------------------------------- modes
    def _enter_tag(self, t):
        self.mode = 'TAG'
        self.tag_t0 = t
        self.lost_since = None

    def _approach(self, pal, t, front):
        if pal is None:
            if self.lost_since is None:
                self.lost_since = t
            if t - self.lost_since > 1.5:
                self.mode = 'SEARCH'
                self.lost_since = None
            return Command(0.0, 0.0, 'RUNNING')
        self.lost_since = None
        b = pal['bearing']
        d = pal['dist']
        w = _clamp(-1.6 * b, -W_MAX, W_MAX)
        v = _clamp(K_V * (d - PALLET_HANDOVER_D), 0.05, V_MAX)
        v *= max(0.0, 1.0 - abs(b) / 0.5)
        if front < FRONT_STOP:
            v = 0.0
        return Command(v, w, 'RUNNING')

    def _guide(self, t, front, rear, pal, tag_fresh):
        if tag_fresh:
            self.lost_since = None
            D, L, yaw, _z = self.tag_geo
            eD = D - DOCK_DIST
            relaxed = (abs(eD) <= 0.07 and abs(L) <= 0.04
                       and abs(yaw) <= math.radians(4.0))
            if self.good >= 3 or (relaxed and t - self.tag_t0 > RELAX_AFTER):
                return Command(0.0, 0.0, 'DOCKED')
            w = _clamp(K_YAW * (yaw - K_LAT * L / max(D, 0.4)), -W_MAX, W_MAX)
            v = K_V * eD + (CREEP_V if abs(L) > 0.015 else 0.0)
            v = _clamp(v, -0.05, 0.25)
            v *= max(0.3, math.cos(yaw)) * max(0.3, 1.0 - abs(yaw) / 0.5)
            if v > 0.0 and front < FRONT_STOP:
                v = 0.0
            if v < 0.0 and rear < REAR_STOP:
                v = 0.0
            return Command(v, w, 'RUNNING')

        # tag not visible: fall back to pallet bearing if available
        if pal is not None:
            b = pal['bearing']
            w = _clamp(-1.2 * b, -0.5, 0.5)
            v = 0.08 if pal['dist'] > 1.2 else 0.03
            if abs(b) > 0.4 or front < FRONT_STOP:
                v = 0.0
            return Command(v, w, 'RUNNING')

        if self.lost_since is None:
            self.lost_since = t
        lost = t - self.lost_since
        limit = LOST_TAG_GUIDANCE if self.origin == 'guidance' else LOST_TAG_APPROACH
        if lost > limit:
            if self.origin == 'guidance':
                return Command(0.0, 0.0, 'ABORTED')
            self.mode = 'SEARCH'
            self.lost_since = None
            self.good = 0
            return Command(0.0, 0.0, 'RUNNING')
        if lost < 2.0:
            return Command(0.05 if front >= FRONT_STOP else 0.0, 0.0, 'RUNNING')
        return Command(0.0, -SEARCH_W * self.last_side, 'RUNNING')
