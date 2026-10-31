"""Docking controller: sequences the datapath Actions of actions.py.

Modes: A3 (approach by pallet detector, optional when starting in guidance),
A4 (tag acquisition and pose estimate), A5 (guidance into the docking position).
Every wait is bounded by a timeout or a retry counter; a global deadline
keeps the trial within 120 s (guidance) / 150 s (approach).
"""
import math
from sim.api import Command, Observation
import actions as A

P_CONTROLS = {
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
P = P_CONTROLS

GUIDANCE_DEADLINE = 110.0   # internal limit, 10 s inside the 120 s requirement
APPROACH_DEADLINE = 140.0   # internal limit, 10 s inside the 150 s requirement
A3_SEARCH_LIMIT = 45.0      # s of pallet search before giving up
A4_LIMIT = 16.0             # s for one tag-acquisition attempt
RECLAIM_LIMIT = 12.0        # s to regain a fresh tag fix
MAX_REPOS = 3               # reposition manoeuvres per guidance run
MAX_BAD_TICKS = 10          # consecutive ticks without sensors before abort
GUARD_DIST = 0.42           # LiDAR stop distance [m] (footprint 0.30 + margin)
GUARD_HALF = math.radians(30.0)


class DockingController:
    def __init__(self):
        self.reset('approach')

    # ------------------------------------------------------------ lifecycle
    def reset(self, start_stage: str) -> None:
        self.guidance = (start_stage == 'guidance')
        self.deadline = GUIDANCE_DEADLINE if self.guidance else APPROACH_DEADLINE
        self.t0 = None
        self.done = None
        self.init_done = False
        self.bad_ticks = 0
        self.blacklist = []
        self.n_loc = 0
        self.retry_ok = True
        self.hist = []
        self.track = None
        self.pallet_ref = None
        self.mode = 'A4' if self.guidance else 'A3'
        self.a4_entered = False
        self.a3_sub = 'SEARCH'
        self.search_t0 = None
        self.est = None
        self.sweep = None
        self.a4_t0 = None
        self.a5_sub = 'PLAN'
        self.mstate = None
        self.n_bo = 0
        self.bo_ok = True
        self.repos_count = 0
        self.reclaim_t0 = None

    def step(self, obs: Observation) -> Command:
        try:
            return self._step(obs)
        except Exception:
            return self._finish('ABORTED')

    # ------------------------------------------------------------- helpers
    def _finish(self, status):
        self.done = status
        return Command(0.0, 0.0, status)

    def _guard(self, cmd, scan):
        """LiDAR motion guard: no forward (backward) motion into an obstacle."""
        if cmd.status != 'RUNNING' or (cmd.v == 0.0):
            return cmd
        if scan is None or not scan.ranges:
            return cmd
        fwd, rear = math.inf, math.inf
        for i, r in enumerate(scan.ranges):
            if r is None or not math.isfinite(r) or r <= 0.01:
                continue
            ang = A.wrap(scan.angle_min + i * scan.angle_increment)
            if abs(ang) <= GUARD_HALF:
                fwd = min(fwd, r)
            if abs(A.wrap(ang - math.pi)) <= GUARD_HALF:
                rear = min(rear, r)
        if cmd.v > 0.0 and fwd < GUARD_DIST:
            return Command(0.0, cmd.w)
        if cmd.v < 0.0 and rear < GUARD_DIST:
            return Command(0.0, cmd.w)
        return cmd

    def _step(self, obs):
        if self.done is not None:
            return Command(0.0, 0.0, self.done)
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 > self.deadline:
            return self._finish('ABORTED')

        ok = A.a1_check_sensors(obs.odom, obs.scan)
        if not ok:
            self.bad_ticks += 1
            if self.bad_ticks > MAX_BAD_TICKS:
                return self._finish('ABORTED')
            return Command()
        self.bad_ticks = 0
        if not self.init_done:
            _, self.init_done = A.a0_init()
            if not A.a2_navigate():
                return self._finish('ABORTED')

        if self.mode == 'A4' and not self.a4_entered:
            self._enter_A4(t)

        if self.mode in ('A4', 'A5'):
            self.est = A.a43_update_pose(self.est, obs.odom, obs.tags, t, P)
            tag_ok, pose_ready, pose_stale = A.a44_pose_status(self.est, t, P)
        else:
            tag_ok = pose_ready = pose_stale = False

        if self.mode == 'A3':
            cmd = self._run_A3(obs, t)
        elif self.mode == 'A4':
            cmd = self._run_A4(obs, t, tag_ok, pose_ready, pose_stale)
        else:
            cmd = self._run_A5(obs, t, tag_ok, pose_ready, pose_stale)

        if cmd.status != 'RUNNING':
            return self._finish(cmd.status)
        return self._guard(cmd, obs.scan)

    # ------------------------------------------------------------ A3 approach
    def _run_A3(self, obs, t):
        odom = obs.odom
        cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
        if self.a3_sub == 'SEARCH':
            if self.search_t0 is None:
                self.search_t0 = t
            if t - self.search_t0 > A3_SEARCH_LIMIT:
                A.a3e_fail()
                return self._finish('ABORTED')
            self.hist, confirmed, trk = A.a322_confirm(cands, self.hist, t, P)
            if not confirmed:
                return A.a31_search_rotate(P)
            self.track = trk
            self.a3_sub = 'TRACK'

        self.track, lost = A.a323_track(cands, self.track, t, P)
        if lost or self.track is None:
            self.track = None
            self.hist = []
            self.a3_sub = 'SEARCH'
            self.search_t0 = t
            return Command()
        cmd, done = A.a34_approach_cmd(self.track, odom, P)
        if done:
            cmd, self.pallet_ref, _ = A.a35_exit(self.track, odom)
            self.mode = 'A4'
            self.a4_entered = False
        return cmd

    # ------------------------------------------------------------ A4 tag
    def _enter_A4(self, t):
        _, self.n_loc, self.retry_ok, self.est, self.sweep, _, _, _ = \
            A.a40_enter(self.n_loc, self.est, P)
        self.a4_t0 = t
        self.a4_entered = True

    def _run_A4(self, obs, t, tag_ok, pose_ready, pose_stale):
        if pose_ready:
            self._enter_A5()
            return Command()
        if self.est is not None and pose_stale:
            self.est = None
        if self.est is not None and tag_ok:
            return A.a42_hold()          # wait for the filter to accumulate fixes
        if t - self.a4_t0 > A4_LIMIT:
            return self._a4_fail()
        cmd, self.sweep = A.a41_tag_sweep(obs.odom, self.pallet_ref, self.sweep, P)
        return cmd

    def _a4_fail(self):
        _, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
        if not self.retry_ok:
            A.a9_abort()
            return self._finish('ABORTED')
        self.mode = 'A3'
        self.a3_sub = 'SEARCH'
        self.search_t0 = None
        self.hist = []
        self.track = None
        self.est = None
        return Command()

    # ------------------------------------------------------------ A5 guidance
    def _enter_A5(self):
        _, self.mstate, self.n_bo, self.bo_ok, _, _, _ = A.a50_enter()
        self.a5_sub = 'PLAN'
        self.repos_count = 0
        self.mode = 'A5'

    def _to_reclaim(self, t):
        self.a5_sub = 'RECLAIM'
        self.reclaim_t0 = t
        self.mstate = None
        return Command()

    def _run_A5(self, obs, t, tag_ok, pose_ready, pose_stale):
        odom = obs.odom
        est = self.est
        sub = self.a5_sub

        if sub == 'PLAN':
            if est is None or pose_stale:
                return self._to_reclaim(t)
            _, feasible = A.a51_plan(est, P)
            if feasible:
                sub = self.a5_sub = 'CONV'
            else:
                self.repos_count += 1
                if self.repos_count > MAX_REPOS:
                    A.a5e_fail()
                    return self._finish('ABORTED')
                self.a5_sub = 'REPOS'
                self.mstate = None
                return Command()

        if sub == 'REPOS':
            if est is None:
                return self._to_reclaim(t)
            cmd, self.mstate, done = A.a52_reposition(est, self.mstate, P)
            if done:
                return self._to_reclaim(t)
            return cmd

        if sub == 'RECLAIM':
            if self.reclaim_t0 is None:
                self.reclaim_t0 = t
            if t - self.reclaim_t0 > RECLAIM_LIMIT:
                A.a5e_fail()
                return self._finish('ABORTED')
            cmd, self.mstate = A.a54_reacquire(est, self.mstate, t, P)
            if tag_ok and est is not None and est['t_meas'] >= self.reclaim_t0:
                self.a5_sub = 'PLAN'
                self.mstate = None
                return Command()
            return cmd

        if sub == 'CONV':
            if pose_stale or est is None:
                return self._to_reclaim(t)
            cmd, at_predock, miss = A.a53_converge(est, P)
            if at_predock:
                self.a5_sub = 'FINAL'
                return Command()
            if miss:
                self.a5_sub = 'BACKOFF'
                self.mstate = None
                return Command()
            return cmd

        if sub == 'FINAL':
            if pose_stale or est is None:
                return self._to_reclaim(t)
            cmd, at_dock = A.a55_final_approach(est, P)
            if at_dock:
                self.a5_sub = 'VERIFY'
                self.mstate = None
                return Command()
            return cmd

        if sub == 'VERIFY':
            cmd, self.mstate, dock_verified, dock_bad, _ = A.a56_verify(est, self.mstate, t, P)
            if dock_verified:
                A.a6_docked()
                return self._finish('DOCKED')
            if dock_bad:
                self.a5_sub = 'BACKOFF'
                self.mstate = None
                return Command()
            return cmd

        if sub == 'BACKOFF':
            cmd, self.mstate, self.n_bo, self.bo_ok, done = \
                A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if done:
                if not self.bo_ok:
                    A.a5e_fail()
                    return self._finish('ABORTED')
                return self._to_reclaim(t)
            return cmd

        return Command()
