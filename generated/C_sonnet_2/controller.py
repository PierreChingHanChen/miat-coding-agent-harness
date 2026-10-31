"""DockingController: A0..A5 state machine built from the Actions in actions.py."""
import math
from sim.api import Command
import actions as A

P = {
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

MAX_LOC = 4            # localisation attempts (own limit, wider than max_retries for clutter)
SEARCH_TIMEOUT = 45.0  # s without a confirmed pallet
BLOCK_TIMEOUT = 3.0    # s blocked by an obstacle while approaching
SWEEP_TIMEOUT = 14.0   # s of tag search per attempt
HOLD_TIMEOUT = 2.5
REPOS_TIMEOUT = 35.0
REACQ_TIMEOUT = 8.0
CONV_TIMEOUT = 40.0
FINAL_TIMEOUT = 40.0
GUARD_FWD = 0.50
GUARD_REAR = 0.45


def _sector_min(scan, centre, half):
    if scan is None:
        return float('inf')
    m = float('inf')
    for i, r in enumerate(scan.ranges):
        if not (r > 0.05) or r == float('inf'):
            continue
        a = A.wrap(scan.angle_min + i * scan.angle_increment - centre)
        if abs(a) <= half and r < m:
            m = r
    return m


class DockingController:
    def __init__(self):
        self.reset('guidance')

    def reset(self, start_stage):
        self.stage = start_stage
        self.t0 = None
        self.limit = 112.0 if start_stage == 'guidance' else 142.0
        self.est = None
        self.track = None
        self.hist = []
        self.blacklist = []
        self.pallet_ref = None
        self.n_loc = 0
        self.sweep = None
        self.mstate = None
        self.n_bo = 0
        self.state = 'INIT'
        self.ts = 0.0           # time the current state was entered
        self.block_t = None
        self.first_attempt = (start_stage == 'guidance')
        self.sens_t0 = None

    # ------------------------------------------------------------------
    def _go(self, state, t):
        self.state = state
        self.ts = t
        self.mstate = None
        self.block_t = None

    def _abort(self):
        c = A.a9_abort()
        return Command(0.0, 0.0, 'ABORTED')

    def _enter_a3(self, t):
        _, self.track, self.hist, _, _, _ = A.a30_enter()
        self._go('A3_SEARCH', t)

    def _enter_a4(self, t, odom):
        _, self.n_loc, retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(self.n_loc, self.est, P)
        if self.n_loc > MAX_LOC:
            return False
        self._go('A4_SWEEP', t)
        return True

    def _enter_a5(self, t):
        _, self.mstate, self.n_bo, _, _, _, _ = A.a50_enter()
        self._go('A5_PLAN', t)

    def _a4_fail(self, t, odom):
        """Tag not found near the approached object: blacklist it and look elsewhere."""
        if self.pallet_ref is not None:
            _, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
        self.pallet_ref = None
        self.first_attempt = False
        if self.n_loc >= MAX_LOC:
            return self._abort()
        self._enter_a3(t)
        return Command()

    # ------------------------------------------------------------------
    def step(self, obs):
        try:
            c = self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 'ABORTED')
        return c

    def _step(self, obs):
        t = obs.t
        odom = obs.odom
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 > self.limit:
            return self._abort()
        # A1 sensor check
        if self.state == 'INIT':
            if self.sens_t0 is None:
                self.sens_t0 = t
            if not A.a1_check_sensors(odom, obs.scan):
                if t - self.sens_t0 > 2.0:
                    return self._abort()
                return Command()
            A.a0_init()
            A.a2_navigate()
            if self.stage == 'guidance':
                self._enter_a4(t, odom)      # first tag look from the start pose
            else:
                self._enter_a3(t)
            return Command()
        if odom is None:
            return Command()

        # continuous pose filter + status
        self.est = A.a43_update_pose(self.est, odom, obs.tags, t, P)
        tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)

        s = self.state
        if s == 'A3_SEARCH' or s == 'A3_TRACK':
            return self._a3(obs, t, odom, ready)
        if s == 'A4_SWEEP' or s == 'A4_HOLD':
            return self._a4(obs, t, odom, tag_ok, ready)
        return self._a5(obs, t, odom, tag_ok, ready, stale)

    # ------------------------------------------------------------------
    def _a3(self, obs, t, odom, ready):
        # a tag fix is ground truth: skip straight to guidance
        if ready and self.est is not None:
            cam = A._cam_pose(self.est)
            if math.hypot(*cam) < 3.0:
                self._enter_a5(t)
                return Command()
        cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
        if self.state == 'A3_SEARCH':
            self.hist, conf, trk = A.a322_confirm(cands, self.hist, t, P)
            if conf:
                self.track = trk
                self._go('A3_TRACK', t)
                return Command()
            if t - self.ts > SEARCH_TIMEOUT:
                A.a3e_fail()
                return self._abort()
            return A.a31_search_rotate(P)
        # TRACK
        self.track, lost = A.a323_track(cands, self.track, t, P)
        if lost:
            self.hist = []
            self._go('A3_SEARCH', t)
            return Command()
        cmd, done = A.a34_approach_cmd(self.track, odom, P)
        if done:
            _, self.pallet_ref, _ = A.a35_exit(self.track, odom)
            if not self._enter_a4(t, odom):
                return self._abort()
            return Command()
        if cmd.v > 0 and _sector_min(obs.scan, 0.0, math.radians(30)) < GUARD_FWD:
            if self.block_t is None:
                self.block_t = t
            if t - self.block_t > BLOCK_TIMEOUT:
                # blocked by an obstacle: give up on this object
                self.blacklist = self.blacklist + [tuple(self.track['pos'])]
                self.track = None
                self.hist = []
                self._go('A3_SEARCH', t)
            return Command(0.0, cmd.w)
        self.block_t = None
        return cmd

    # ------------------------------------------------------------------
    def _a4(self, obs, t, odom, tag_ok, ready):
        if ready:
            self._enter_a5(t)
            return Command()
        if t - self.ts > SWEEP_TIMEOUT and self.state == 'A4_SWEEP' and not tag_ok:
            return self._a4_fail(t, odom)
        if self.state == 'A4_SWEEP':
            if tag_ok and self.est is not None:
                self._go('A4_HOLD', t)
                return A.a42_hold()
            cmd, self.sweep = A.a41_tag_sweep(odom, self.pallet_ref, self.sweep, P)
            return cmd
        # HOLD: accumulate measurements
        if t - self.ts > HOLD_TIMEOUT:
            self.state = 'A4_SWEEP'
            self.ts = t - (SWEEP_TIMEOUT - 6.0)     # leave a bounded extra search window
            return Command()
        return A.a42_hold()

    # ------------------------------------------------------------------
    def _fail5(self, t, odom):
        A.a5e_fail()
        return self._abort()

    def _a5(self, obs, t, odom, tag_ok, ready, stale):
        s = self.state
        est = self.est
        if est is None or (stale and s not in ('A5_REACQ', 'A5_REPOS')):
            if s == 'A5_VERIFY' or s == 'A5_FINAL' or s == 'A5_CONV':
                # lost the tag for too long: try to find it again
                self.mstate = None
                if est is None:
                    return self._fail5(t, odom)
                self._go('A5_REACQ', t)
                return Command()
        if s == 'A5_PLAN':
            _, feas = A.a51_plan(est, P)
            self._go('A5_CONV' if feas else 'A5_REPOS', t)
            return Command()
        if s == 'A5_REPOS':
            if t - self.ts > REPOS_TIMEOUT:
                return self._fail5(t, odom)
            cmd, self.mstate, done = A.a52_reposition(est, self.mstate, P)
            if done:
                self._go('A5_REACQ', t)
                return Command()
            if cmd.v > 0 and _sector_min(obs.scan, 0.0, math.radians(30)) < GUARD_FWD:
                return Command(0.0, cmd.w)
            return cmd
        if s == 'A5_REACQ':
            if ready or (tag_ok and est['n'] >= 2):
                self._go('A5_PLAN', t)
                return Command()
            if t - self.ts > REACQ_TIMEOUT:
                if stale:
                    return self._fail5(t, odom)
                self._go('A5_PLAN', t)
                return Command()
            cmd, self.mstate = A.a54_reacquire(est, self.mstate, t, P)
            return cmd
        if s == 'A5_CONV':
            if t - self.ts > CONV_TIMEOUT:
                return self._fail5(t, odom)
            cmd, at_pre, miss = A.a53_converge(est, P)
            if at_pre:
                self._go('A5_FINAL', t)
                return Command()
            if miss:
                self._go('A5_REPOS', t)
                return Command()
            if cmd.v > 0 and _sector_min(obs.scan, 0.0, math.radians(30)) < GUARD_FWD:
                return Command(0.0, cmd.w)
            return cmd
        if s == 'A5_FINAL':
            if t - self.ts > FINAL_TIMEOUT:
                return self._fail5(t, odom)
            cmd, at_dock = A.a55_final_approach(est, P)
            if at_dock:
                self._go('A5_VERIFY', t)
                return Command()
            if cmd.v > 0 and _sector_min(obs.scan, 0.0, math.radians(20)) < 0.40:
                return Command()
            return cmd
        if s == 'A5_VERIFY':
            # need a fresh fix before declaring success
            if (t - est['t_meas']) > 2.0 and (t - self.ts) < 3.0:
                return Command()
            cmd, self.mstate, ok, bad, arrived = A.a56_verify(est, self.mstate, t, P)
            if (t - est['t_meas']) > 2.5:
                ok, bad = False, (ok or bad)
            if ok:
                A.a6_docked()
                return Command(0.0, 0.0, 'DOCKED')
            if bad:
                self._go('A5_BACKOFF', t)
                return Command()
            return cmd
        if s == 'A5_BACKOFF':
            if _sector_min(obs.scan, math.pi, math.radians(30)) < GUARD_REAR:
                self._go('A5_PLAN', t)
                return Command()
            cmd, self.mstate, self.n_bo, bo_ok, done = A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if done:
                if not bo_ok:
                    return self._fail5(t, odom)
                self._go('A5_REACQ', t)
                return Command()
            if not bo_ok and self.n_bo >= P['max_backoff'] + 1:
                return self._fail5(t, odom)
            return cmd
        return self._abort()
