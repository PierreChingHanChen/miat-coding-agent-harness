"""AGV pallet-pocket docking controller (A3 approach -> A4 tag localisation -> A5 guidance).

Control logic only; all datapath computations are delegated to the Actions in actions.py.
Every wait is bounded (per-state watchdogs, retry counters and a global budget), every
exception ends the trial as ABORTED with zero velocity, and nothing is random.
"""
import math
import os
import sys

from sim.api import Command, clamp_cmd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import actions as A  # noqa: E402

CONTROLS = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45,
    "confirm_k": 3, "confirm_window_s": 0.5, "confirm_tol_deg": 6.0,
    "track_gate_deg": 8.0, "track_lost_s": 2.0, "d_app": 1.9, "v_app": 0.25,
    "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0, "w_tag": 0.3,
    "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3, "gate_yaw_deg": 12.0,
    "pose_stale_s": 3.0, "near_dist": 1.1, "blind_max_s": 10.0,
    "fov_half_deg": 34.5, "fov_margin_deg": 10.0, "y_pre": 1.65, "repos_extra": 0.6,
    "v_conv": 0.2, "v_final": 0.12, "k_head": 1.6, "k_lat": 2.5, "dock_y": 1.13,
    "x_tol": 0.03, "head_tol_deg": 2.0, "verify_lat": 0.04, "verify_head_deg": 3.5,
    "verify_dist": 0.08, "verify_s": 0.5, "backoff_dist": 0.6, "max_backoff": 3,
    "max_retries": 2,
}

# Watchdogs and limits [s] / [ticks] / [m]
BUDGET = {'guidance': 112.0, 'approach': 142.0}   # trial limits 120 s / 150 s, report earlier
SEARCH_MAX_S = 45.0       # pallet search (rotation) without a confirmed candidate
APPROACH_MAX_S = 60.0     # driving toward a confirmed candidate
LOCATE_MAX_S = 25.0       # tag sweep / localisation before declaring tag failure
GUIDE_MAX_S = 75.0        # all of A5 (converge, reposition, final, verify, backoff)
REACQ_MAX_S = 10.0        # rotation to re-find the tag after a blind manoeuvre
REPOS_MAX_S = 25.0       # reposition manoeuvre (odometry-based)
SENSOR_MAX_S = 3.0        # missing odometry/scan tolerated before abort
SAFE_FRONT = 0.45         # forward stop distance to any LiDAR return [m] (footprint 0.30 + margin)
FRONT_CONE = math.radians(25.0)
BLOCK_TICKS = 20          # 2 s of blocked forward motion before a state-specific reaction
MAX_RESEARCH = 4          # re-entries of the pallet search per trial
MAX_REPOS = 4             # reposition manoeuvres per guidance attempt
GUIDE_STATES = ('converge', 'repos', 'reacq', 'final', 'verify', 'backoff')


def _front_range(scan):
    """Smallest valid LiDAR range inside the forward cone (inf if none)."""
    best = float('inf')
    if scan is None:
        return best
    for i, r in enumerate(scan.ranges):
        if not (0.05 < r < float('inf')):
            continue
        ang = A.wrap(scan.angle_min + i * scan.angle_increment)
        if abs(ang) <= FRONT_CONE and r < best:
            best = r
    return best


class DockingController:
    def __init__(self):
        self.P = dict(CONTROLS)
        self.reset('guidance')

    # ------------------------------------------------------------ lifecycle
    def reset(self, start_stage):
        self.budget = BUDGET['approach' if start_stage == 'approach' else 'guidance']
        self.state = 'search' if start_stage == 'approach' else 'locate'
        self.entered = False
        self.t0 = None
        self.t_state = 0.0
        self.t_guide = None
        self.odom = None
        self.scan = None
        self.hist = []
        self.track = None
        self.pallet_ref = None
        self.blacklist = []
        self.est = None
        self.sweep = None
        self.n_loc = 0
        self.mstate = None
        self.n_bo = 0
        self.bo_ok = True
        self.n_research = 0
        self.n_repos = 0
        self.sensor_miss_t = None
        self.blocked_ticks = 0
        self.status = 'RUNNING'

    def step(self, obs):
        if self.status != 'RUNNING':
            return Command(0.0, 0.0, self.status)
        try:
            cmd = self._tick(obs)
        except Exception:  # unhandled exceptions are failures: stop safely and report
            cmd = self._abort()
        if cmd.status != 'RUNNING':
            self.status = cmd.status
            cmd = Command(0.0, 0.0, cmd.status)
        return clamp_cmd(cmd)

    # ------------------------------------------------------------ helpers
    def _abort(self):
        return Command(0.0, 0.0, 'ABORTED')

    def _dock(self):
        return Command(0.0, 0.0, 'DOCKED')

    def _set(self, state, t):
        self.state = state
        self.t_state = t

    def _enter_locate(self, t):
        _, n_loc, retry_ok, _, _, _, _, _ = A.a40_enter(self.n_loc, None, self.P)
        self.n_loc = n_loc
        if not retry_ok:
            return False
        self.est = None
        self.sweep = None
        self._set('locate', t)
        return True

    def _enter_guide(self, t):
        _, self.mstate, self.n_bo, self.bo_ok, _, _, _ = A.a50_enter()
        if self.t_guide is None:
            self.t_guide = t
        self._set('converge', t)

    def _research(self, t):
        """Return to the pallet search (bounded)."""
        self.n_research += 1
        if self.n_research > MAX_RESEARCH:
            return self._abort()
        self.hist = []
        self.track = None
        self._set('search', t)
        return Command()

    def _reacq(self, t):
        self.mstate = None
        self._set('reacq', t)
        return Command()

    def _backoff(self, t):
        self.mstate = None
        self._set('backoff', t)
        return self._s_backoff(None, t)

    def _blocked(self, cmd):
        """Stop forward motion when a LiDAR return is inside the stop distance."""
        if cmd.v > 0.01 and _front_range(self.scan) < SAFE_FRONT:
            self.blocked_ticks += 1
            return Command(0.0, cmd.w), self.blocked_ticks >= BLOCK_TICKS
        self.blocked_ticks = 0
        return cmd, False

    def _cam_odom(self):
        o = self.odom
        return o.x + A.CAM_X * math.cos(o.yaw), o.y + A.CAM_X * math.sin(o.yaw)

    # ------------------------------------------------------------ tick
    def _tick(self, obs):
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
            self.t_state = t
        if not self.entered:
            self.entered = True
            if self.state == 'locate' and not self._enter_locate(t):
                return self._abort()
        if t - self.t0 > self.budget:
            return self._abort()
        if self.state in GUIDE_STATES and self.t_guide is not None and t - self.t_guide > GUIDE_MAX_S:
            return self._abort()

        if not A.a1_check_sensors(obs.odom, obs.scan):
            if self.sensor_miss_t is None:
                self.sensor_miss_t = t
            if t - self.sensor_miss_t > SENSOR_MAX_S:
                return self._abort()
            return A.a42_hold()
        self.sensor_miss_t = None
        self.odom = obs.odom
        self.scan = obs.scan

        if self.state not in ('search', 'approach'):
            self.est = A.a43_update_pose(self.est, obs.odom, obs.tags, t, self.P)

        handler = getattr(self, '_s_' + self.state)
        return handler(obs, t)

    # ------------------------------------------------------------ A3: approach
    def _s_search(self, obs, t):
        if t - self.t_state > SEARCH_MAX_S:
            return self._abort()
        cands = A.a321_candidates(obs.pallets, obs.odom, self.blacklist, self.P)
        self.hist, confirmed, trk = A.a322_confirm(cands, self.hist, t, self.P)
        if confirmed:
            self.track = trk
            self.blocked_ticks = 0
            self._set('approach', t)
            return Command()
        return A.a31_search_rotate(self.P)

    def _s_approach(self, obs, t):
        if t - self.t_state > APPROACH_MAX_S:
            return self._research(t)
        cands = A.a321_candidates(obs.pallets, obs.odom, self.blacklist, self.P)
        self.track, lost = A.a323_track(cands, self.track, t, self.P)
        if self.track is None:
            return self._research(t)
        cx, cy = self._cam_odom()
        dist = math.hypot(self.track['pos'][0] - cx, self.track['pos'][1] - cy)
        if lost and dist > self.P['d_app']:
            return self._research(t)          # lost far away: search again
        cmd, done = A.a34_approach_cmd(self.track, obs.odom, self.P)
        if done:
            cmd, self.pallet_ref, _ = A.a35_exit(self.track, obs.odom)
            self.hist = []
            if not self._enter_locate(t):
                return self._abort()
            return Command()
        cmd, long_block = self._blocked(cmd)
        if long_block:                        # false object or obstacle in the way
            self.blacklist.append(tuple(self.track['pos']))
            return self._research(t)
        return cmd

    # ------------------------------------------------------------ A4: localise
    def _s_locate(self, obs, t):
        tag_ok, ready, _ = A.a44_pose_status(self.est, t, self.P)
        if ready:
            self._enter_guide(t)
            return Command()
        if t - self.t_state > LOCATE_MAX_S:
            return self._tag_fail(t)
        if tag_ok:
            return A.a42_hold()               # wait for init_meas measurements
        cmd, self.sweep = A.a41_tag_sweep(obs.odom, self.pallet_ref, self.sweep, self.P)
        return cmd

    def _tag_fail(self, t):
        _, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
        self.est = None
        self.sweep = None
        self.pallet_ref = None
        self.hist = []
        self.track = None
        self._set('search', t)               # try another candidate, or re-detect the pallet
        return Command()

    # ------------------------------------------------------------ A5: guidance
    def _s_converge(self, obs, t):
        tag_ok, ready, stale = A.a44_pose_status(self.est, t, self.P)
        if stale:                             # tolerate tag loss; only a stale estimate forces reacquire
            return self._reacq(t)
        cmd, at_predock, miss = A.a53_converge(self.est, self.P)
        if at_predock:
            self.mstate = None
            self._set('final', t)
            return Command()
        if miss:
            return self._backoff(t)
        _, feasible = A.a51_plan(self.est, self.P)
        # a51 is infeasible by construction within 0.15 m of the pre-dock line; there only a
        # large lateral offset justifies a reposition
        near_line = self.est['py'] <= self.P['y_pre'] + 0.15 and abs(self.est['px']) <= 0.25
        if not feasible and not near_line:    # tag would leave the FOV margin: reposition first
            if self.n_repos >= MAX_REPOS:
                return self._abort()
            self.n_repos += 1
            self.mstate = None
            self._set('repos', t)
            return Command()
        cmd, long_block = self._blocked(cmd)
        if long_block:
            return self._backoff(t)
        return cmd

    def _s_repos(self, obs, t):
        # odometry manoeuvre: the tag may be out of view by design, so only the watchdog applies
        if t - self.t_state > REPOS_MAX_S:
            return self._backoff(t)
        cmd, self.mstate, done = A.a52_reposition(self.est, self.mstate, self.P)
        if done:
            self.mstate = None
            self._set('converge', t)
            return Command()
        cmd, long_block = self._blocked(cmd)
        if long_block:
            return self._backoff(t)
        return cmd

    def _s_reacq(self, obs, t):
        if t - self.t_state > REACQ_MAX_S:
            return self._backoff(t)
        cmd, self.mstate = A.a54_reacquire(self.est, self.mstate, t, self.P)
        tag_ok, ready, _ = A.a44_pose_status(self.est, t, self.P)
        if tag_ok and ready:
            self.mstate = None
            self._set('converge', t)
            return Command()
        return cmd

    def _s_final(self, obs, t):
        _, _, stale = A.a44_pose_status(self.est, t, self.P)
        if stale:
            return self._backoff(t)
        cmd, at_dock = A.a55_final_approach(self.est, self.P)
        if at_dock:
            self.mstate = None
            self._set('verify', t)
            return Command()
        cmd, long_block = self._blocked(cmd)
        if long_block:
            return self._backoff(t)
        return cmd

    def _s_verify(self, obs, t):
        cmd, self.mstate, verified, bad, _ = A.a56_verify(self.est, self.mstate, t, self.P)
        if verified:
            tag_ok, _, _ = A.a44_pose_status(self.est, t, self.P)
            if tag_ok:                        # only dock on a fresh tag fix
                return self._dock()
            return self._backoff(t)
        if bad:
            return self._backoff(t)
        return cmd

    def _s_backoff(self, obs, t):
        cmd, self.mstate, self.n_bo, self.bo_ok, done = A.a57_backoff(
            self.odom, self.mstate, self.n_bo, self.P)
        if not self.bo_ok:
            return self._abort()
        if done:
            self.mstate = None
            self._set('reacq', t)
            return Command()
        return cmd
