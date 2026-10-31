"""AGV-DockSim: a 2-D surrogate of the Gazebo pallet-docking scene.

World frame: pocket-face centre (AprilTag) at the origin, face normal = +y.
The pallet body occupies x in [-0.6, 0.6], y in [-1.0, 0].  The AGV is a disc of
radius 0.25 m with the camera and LiDAR on its centre, both facing forward.

Sensor models are calibrated to the measurements of the original paper
(Table VI: AprilTag detection range vs viewing angle) and to the published
specifications of the RealSense D435i / RPLIDAR A1.  See calibrate.py.
"""
from __future__ import annotations
import math, time
import numpy as np
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional
from .api import (Observation, Odom, TagDetection, PalletDetection, Scan, Command,
                  clamp_cmd, HFOV, VFOV, FX, IMG_W, DT, TAG_ID, DOCK_DIST, CAM_X)

AGV_R = 0.30
CAM_OFF = CAM_X            # camera mounted at the front of the AGV, on its centre line
LAT_TOL, DIST_TOL, HEAD_TOL = 0.06, 0.12, 6.0   # docking acceptance (R-01)
FP_RATE = 0.12            # per-frame false-positive rate of a pallet-like object in view (calibrated)
TAG_SUBFRAMES = 3         # apriltag_ros runs at ~30 Hz; the 10 Hz controller sees the latest of 3 frames
CAM_H, TAG_H, TAG_SIZE = 0.35, 0.12, 0.10
D_MIN_VERT = (CAM_H - (TAG_H - TAG_SIZE / 2)) / math.tan(VFOV / 2)   # ~0.73 m
PALLET_RECT = (-0.6, 0.6, -1.0, 0.0)   # xmin xmax ymin ymax

# Paper Table VI (symmetric pairs averaged): viewing angle alpha (deg, 0 = frontal)
#   -> (d_stable [P=0.80], d_max [P~0.01])
TAG_TABLE = {0: (2.77, 3.10), 15: (2.525, 2.975), 30: (2.425, 2.765), 45: (2.025, 2.215),
             60: (1.45, 1.65), 70: (0.0, 0.0)}
LN4, LN99 = math.log(4.0), math.log(99.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tag_detect_prob(d: float, alpha_deg: float) -> float:
    a = abs(alpha_deg)
    if a >= 70:
        return 0.0
    keys = sorted(TAG_TABLE)
    for k0, k1 in zip(keys[:-1], keys[1:]):
        if k0 <= a <= k1:
            f = (a - k0) / (k1 - k0)
            ds = TAG_TABLE[k0][0] * (1 - f) + TAG_TABLE[k1][0] * f
            dm = TAG_TABLE[k0][1] * (1 - f) + TAG_TABLE[k1][1] * f
            break
    if dm <= 0.05:
        return 0.0
    s = max((dm - ds) / (LN4 + LN99), 1e-3)
    d50 = ds + LN4 * s
    z = (d - d50) / s
    if z > 50:
        return 0.0
    return 1.0 / (1.0 + math.exp(z))


@dataclass
class Obstacle:
    x: float
    y: float
    r: float
    pallet_like_width: float = 0.0     # >0: the detector may mistake it for a pallet
    t_on: float = -1.0                 # active window (dynamic obstacles)
    t_off: float = 1e9


@dataclass
class Scenario:
    name: str
    start: tuple                       # (x, y, yaw) world
    obstacles: List[Obstacle] = field(default_factory=list)
    start_stage: str = "guidance"      # 'guidance' (A4/A5 only) or 'approach' (A3..A5)
    nav_time: float = 0.0              # time charged for A2 (Nav2 stub) [s]
    max_time: float = 120.0
    dynamic_obstacle: bool = False     # safety scenario: a person steps in front of the AGV
    faults: Dict[str, float] = field(default_factory=dict)


class DockSim:
    def __init__(self, sc: Scenario, seed: int):
        self.sc = sc
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.yaw = sc.start
        self.t = 0.0
        self.ox = self.oy = self.oyaw = 0.0          # odometry
        self.v_meas = self.w_meas = 0.0
        self.obstacles = list(sc.obstacles)
        self.collided = False
        self.collision_with = None
        self.min_clear_dyn = 1e9
        self.dyn = None
        self.f = dict(tag_drop=0.0, tag_outlier=0.02, fp_mult=1.0, slip=1.0,
                      freeze_t0=-1.0, freeze_len=0.0, depth_zero=0.15)
        self.f.update(sc.faults)
        self._last_tags: List[TagDetection] = []
        self.traj = []
        # beam directions for the LiDAR (2 deg)
        self.n_beams = 180
        self.beam_rel = -math.pi + np.arange(self.n_beams) * (2 * math.pi / self.n_beams)
        if sc.dynamic_obstacle:
            self.dyn_t = float(self.rng.uniform(6.0, 14.0))

    # ------------------------------------------------------------------ physics
    def step(self, cmd: Command):
        c = clamp_cmd(cmd)
        sl = self.f["slip"]
        v = c.v * (1 + self.rng.normal(0, 0.03 * sl))
        w = c.w * (1 + self.rng.normal(0, 0.03 * sl)) + self.rng.normal(0, 0.005 * sl)
        self.yaw = wrap(self.yaw + w * DT)
        self.x += v * math.cos(self.yaw) * DT
        self.y += v * math.sin(self.yaw) * DT
        # wheel odometry sees the commanded motion with its own small error
        vo = c.v * (1 + self.rng.normal(0, 0.01))
        wo = c.w * (1 + self.rng.normal(0, 0.01))
        self.oyaw = wrap(self.oyaw + wo * DT)
        self.ox += vo * math.cos(self.oyaw) * DT
        self.oy += vo * math.sin(self.oyaw) * DT
        self.v_meas, self.w_meas = vo, wo
        self.t += DT
        self._spawn_dynamic(c)
        self._check_collision()
        self.traj.append((self.t, self.x, self.y, self.yaw))

    def _spawn_dynamic(self, c):
        if not self.sc.dynamic_obstacle:
            return
        if self.dyn is None and self.t >= self.dyn_t and abs(c.v) > 0.05:
            d = 0.75
            ob = Obstacle(self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw), 0.15,
                          t_on=self.t, t_off=self.t + 4.0)
            self.dyn = ob
            self.obstacles.append(ob)
        if self.dyn is not None and self.dyn.t_on <= self.t <= self.dyn.t_off:
            self.min_clear_dyn = min(self.min_clear_dyn,
                                     math.hypot(self.x - self.dyn.x, self.y - self.dyn.y) - AGV_R - self.dyn.r)

    def _active(self):
        return [o for o in self.obstacles if o.t_on <= self.t <= o.t_off]

    def _check_collision(self):
        x0, x1, y0, y1 = PALLET_RECT
        cx = min(max(self.x, x0), x1); cy = min(max(self.y, y0), y1)
        if math.hypot(self.x - cx, self.y - cy) < AGV_R:
            self.collided, self.collision_with = True, "pallet"
        for o in self._active():
            if math.hypot(self.x - o.x, self.y - o.y) < AGV_R + o.r:
                self.collided, self.collision_with = True, ("dynamic" if o is self.dyn else "obstacle")

    # ------------------------------------------------------------------ sensors
    def observe(self) -> Observation:
        return Observation(t=self.t,
                           odom=Odom(self.ox, self.oy, self.oyaw, self.v_meas, self.w_meas),
                           tags=self._tags(), pallets=self._pallets(), scan=self._scan())

    @property
    def cx(self):
        return self.x + CAM_OFF * math.cos(self.yaw)

    @property
    def cy(self):
        return self.y + CAM_OFF * math.sin(self.yaw)

    def cam_frame(self, px, py):
        """world point -> (x_right, z_forward) in camera frame"""
        dx, dy = px - self.cx, py - self.cy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        z = dx * c + dy * s
        xr = dx * s - dy * c
        return xr, z

    def viewing_angle(self):
        return math.degrees(math.atan2(abs(self.cx), self.cy)) if self.cy > 0 else 180.0

    def tag_visible_geom(self):
        if self.cy <= 0.02:
            return False
        xr, z = self.cam_frame(0.0, 0.0)
        if z <= 0.05:
            return False
        d = math.hypot(xr, z)
        if d < D_MIN_VERT:
            return False
        alpha = math.radians(self.viewing_angle())
        beta = math.atan2(xr, z)
        hw = math.atan2(TAG_SIZE / 2 * math.cos(alpha), d)
        return abs(beta) + hw <= HFOV / 2

    def _tags(self):
        fr0, frl = self.f["freeze_t0"], self.f["freeze_len"]
        if fr0 >= 0 and fr0 <= self.t < fr0 + frl:      # camera driver freeze: stale repeats
            return list(self._last_tags)
        out = []
        if self.tag_visible_geom() and self.rng.random() >= self.f["tag_drop"]:
            xr, z = self.cam_frame(0.0, 0.0)
            d = math.hypot(xr, z)
            p1 = tag_detect_prob(d, self.viewing_angle())
            if self.rng.random() < 1.0 - (1.0 - p1) ** TAG_SUBFRAMES:
                sp = 0.004 + 0.006 * d * d
                yaw_true = wrap(self.yaw + math.pi / 2)        # see api.TagDetection.yaw
                sy = math.radians(0.5 + 1.5 * d)
                yaw_m = yaw_true + self.rng.normal(0, sy)
                if self.rng.random() < self.f["tag_outlier"]:
                    yaw_m += math.radians(self.rng.choice([-1, 1]) * self.rng.uniform(8, 15))
                out.append(TagDetection(TAG_ID, xr + self.rng.normal(0, sp), z + self.rng.normal(0, sp),
                                        wrap(yaw_m), self.t))
        self._last_tags = out
        return out

    def _pallet_like(self, px, py, width_m, true_pallet, alpha_deg):
        xr, z = self.cam_frame(px, py)
        if z <= 0.2:
            return None
        beta = math.atan2(xr, z)
        if abs(beta) > HFOV / 2 - math.radians(2):
            return None
        d = math.hypot(xr, z)
        beta_m = beta + self.rng.normal(0, math.radians(1.0))
        u = IMG_W / 2 + FX * math.tan(beta_m)
        wfac = math.cos(math.radians(min(alpha_deg, 85))) if true_pallet else 1.0
        width_px = FX * width_m * wfac / d * (1 + self.rng.normal(0, 0.05))
        n = 25
        ds = d * (1 + self.rng.normal(0, 0.01, n))
        ds[self.rng.random(n) < self.f["depth_zero"]] = 0.0
        bg = self.rng.random(n) < 0.04
        ds[bg] = d + self.rng.uniform(1.0, 3.0, bg.sum())
        return u, width_px, [float(v) for v in ds], d

    def _pallets(self):
        out = []
        if self.cy > 0.3:
            alpha = self.viewing_angle()
            if alpha < 80:
                cx, cy = 0.0, 0.0
                # type-1 error: at oblique views YOLACT sometimes locks onto the pallet side
                p_side = min(0.35, max(0.0, (alpha - 30.0) / 30.0 * 0.35))
                side = self.rng.random() < p_side
                if side:
                    cx, cy = math.copysign(0.6, self.cx), -0.5
                r = self._pallet_like(cx, cy, 1.2, True, alpha)
                if r and r[3] < 7.0 and self.rng.random() < 0.95:
                    u, wpx, ds, _ = r
                    out.append(PalletDetection(u, wpx, float(self.rng.uniform(0.6, 0.99)), ds, self.t))
        # type-2 error: pallet-like objects (racks, tables)
        for o in self._active():
            if o.pallet_like_width <= 0:
                continue
            r = self._pallet_like(o.x, o.y, o.pallet_like_width, False, 0.0)
            if r and r[3] < 5.0 and self.rng.random() < min(1.0, FP_RATE * self.f["fp_mult"]):
                u, wpx, ds, _ = r
                out.append(PalletDetection(u, wpx, float(self.rng.uniform(0.45, 0.88)), ds, self.t))
        return out

    def _scan(self):
        ang = self.yaw + self.beam_rel
        dx, dy = np.cos(ang), np.sin(ang)
        rng = np.full(self.n_beams, np.inf)
        # circles
        for o in self._active():
            ox, oy = o.x - self.x, o.y - self.y
            b = dx * ox + dy * oy
            c = ox * ox + oy * oy - o.r * o.r
            disc = b * b - c
            ok = disc >= 0
            tt = b - np.sqrt(np.where(ok, disc, 0))
            hit = ok & (tt > 0)
            rng = np.where(hit, np.minimum(rng, tt), rng)
        # pallet rectangle (slab method)
        x0, x1, y0, y1 = PALLET_RECT
        with np.errstate(divide="ignore", invalid="ignore"):
            tx0 = (x0 - self.x) / dx; tx1 = (x1 - self.x) / dx
            ty0 = (y0 - self.y) / dy; ty1 = (y1 - self.y) / dy
            tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
            tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (tmax >= tmin) & (tmax > 0) & (tmin > 0)
        rng = np.where(hit, np.minimum(rng, tmin), rng)
        rng = np.where(rng > 10.0, np.inf, rng + self.rng.normal(0, 0.01, self.n_beams))
        return Scan(-math.pi, 2 * math.pi / self.n_beams, [float(r) for r in rng])

    # ------------------------------------------------------------------ metrics
    def dock_errors(self):
        return dict(lat=abs(self.cx), dist=self.cy - DOCK_DIST,
                    head=math.degrees(abs(wrap(self.yaw + math.pi / 2))))

    def docked_ok(self):
        e = self.dock_errors()
        # EUR-pallet pocket 227 mm vs fork ~115 mm -> lateral clearance ~ +-0.06 m
        return e["lat"] <= LAT_TOL and abs(e["dist"]) <= DIST_TOL and e["head"] <= HEAD_TOL


def run_trial(make_controller: Callable, sc: Scenario, seed: int, keep_traj=False,
              trace_states=False) -> dict:
    sim = DockSim(sc, seed)
    ctrl = make_controller()
    ctrl.reset(start_stage=sc.start_stage)
    status, reason = "RUNNING", "timeout"
    exc = None
    n = int(sc.max_time / DT)
    stop_lat = None
    wall0 = time.perf_counter()
    for k in range(n):
        obs = sim.observe()
        try:
            cmd = ctrl.step(obs)
        except Exception as e:                      # crash = failure of the design
            exc = f"{type(e).__name__}: {e}"
            reason = "exception"
            break
        if cmd is None:
            cmd = Command()
        # safety metric: time from dynamic obstacle entering 0.45 m to |v| < 0.02
        if sim.dyn is not None and sim.dyn.t_on <= sim.t <= sim.dyn.t_off and stop_lat is None:
            dd = math.hypot(sim.x - sim.dyn.x, sim.y - sim.dyn.y)
            if abs(cmd.v) < 0.02:
                stop_lat = sim.t - sim.dyn.t_on
        sim.step(cmd)
        if sim.collided:
            reason = "collision_" + sim.collision_with
            status = "COLLIDED"
            break
        if cmd.status in ("DOCKED", "ABORTED"):
            status = cmd.status
            reason = "reported_" + cmd.status.lower()
            break
    ok = (status == "DOCKED" and sim.docked_ok() and not sim.collided)
    res = dict(success=ok, status=status, reason=reason, t=sim.t + sc.nav_time,
               t_guidance=sim.t, collided=sim.collided, exception=exc,
               x=sim.cx, y=sim.cy, yaw=sim.yaw, **sim.dock_errors(),
               false_dock=(status == "DOCKED" and not ok),
               safe_abort=(status == "ABORTED" and not sim.collided),
               min_clear_dyn=(sim.min_clear_dyn if sc.dynamic_obstacle else None),
               dyn_spawned=(sim.dyn is not None) if sc.dynamic_obstacle else None,
               wall=time.perf_counter() - wall0)
    if keep_traj:
        res["traj"] = sim.traj
    if trace_states and hasattr(ctrl, "trace"):
        res["trace"] = list(ctrl.trace)
    return res


# ---------------------------------------------------------------- scenarios
def guidance_start(theta_deg: float, dist: float) -> tuple:
    """Paper Fig.10: angle around pallet centre (90 = frontal), AGV facing the centre."""
    th = math.radians(theta_deg)
    x, y = dist * math.cos(th), dist * math.sin(th)
    return (x, y, math.atan2(-y, -x))


CLUTTER = [Obstacle(-2.7, 2.9, 0.45, pallet_like_width=2.0),   # rack
           Obstacle(3.0, 3.4, 0.35, pallet_like_width=0.7),    # table
           Obstacle(-1.6, 6.4, 0.45, pallet_like_width=2.0),   # rack behind start
           Obstacle(2.2, 6.0, 0.40, pallet_like_width=1.1),    # pallet-like cart
           Obstacle(1.8, 1.7, 0.25),                           # box (not pallet-like)
           Obstacle(-1.9, 1.2, 0.25)]


def e2e_scenario(scene: str, rng: np.random.Generator, **kw) -> Scenario:
    """Viewpoint reached by Nav2 (A2 stub): 3.2-4.0 m from the pallet, |alpha| <= 15 deg,
    heading = goal heading (+y side of the map) with residual error; the AGV must rotate
    to find the pallet.  Nav time model calibrated so the original design matches the
    paper's mean completion time (67.6 s simple / 69.3 s cluttered)."""
    d = rng.uniform(3.2, 4.0)
    a = math.radians(rng.uniform(-15, 15))   # viewpoint set in front of the pallet (paper Fig. 11)
    x, y = d * math.sin(a), d * math.cos(a)
    yaw = wrap(math.radians(rng.uniform(-60, 60)) + math.atan2(-y, -x) + math.radians(rng.choice([-1, 1]) * rng.uniform(30, 110)))
    if scene == "simple":
        return Scenario("e2e_simple", (x, y, yaw), [], "approach",
                        nav_time=float(rng.normal(NAV_MEAN["simple"], 2.0)), max_time=150.0, **kw)
    return Scenario("e2e_cluttered", (x, y, yaw), list(CLUTTER), "approach",
                    nav_time=float(rng.normal(NAV_MEAN["cluttered"], 3.0)), max_time=150.0, **kw)


NAV_MEAN = {"simple": 21.6, "cluttered": 30.0}   # calibrated: original design total time = paper (67.6 s / 69.3 s)
