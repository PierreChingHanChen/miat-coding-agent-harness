"""Docking controller: a state machine whose states are the IDEF0/GRAFCET steps; all computation in actions.py."""
import math
import actions as A
from sim.api import Command

P = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45, "confirm_k": 3,
    "confirm_window_s": 0.5, "confirm_tol_deg": 6.0, "track_gate_deg": 8.0, "track_lost_s": 2.0,
    "d_app": 1.9, "v_app": 0.25, "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0,
    "w_tag": 0.3, "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3, "gate_yaw_deg": 12.0,
    "pose_stale_s": 3.0, "near_dist": 1.1, "blind_max_s": 10.0, "fov_half_deg": 34.5,
    "fov_margin_deg": 10.0, "y_pre": 1.65, "repos_extra": 0.6, "v_conv": 0.2, "v_final": 0.12,
    "k_head": 1.6, "k_lat": 2.5, "dock_y": 1.13, "x_tol": 0.03, "head_tol_deg": 2.0,
    "verify_lat": 0.04, "verify_head_deg": 3.5, "verify_dist": 0.08, "verify_s": 0.5,
    "backoff_dist": 0.6, "max_backoff": 3, "max_retries": 2,
}

SEARCH_TIMEOUT = 24.0     # s per search episode (one full turn is 18 s)
SWEEP_TIMEOUT = 20.0      # s of tag sweep before declaring "no tag here"
REACQ_TIMEOUT = 8.0
FIX_WAIT = 3.0
FRONT_STOP = 0.42


def _sector_min(scan, centre, half):
    if scan is None or not scan.ranges:
        return float('inf')
    m = float('inf')
    for i, r in enumerate(scan.ranges):
        a = A.wrap(scan.angle_min + i * scan.angle_increment - centre)
        if abs(a) <= half and r > 0.05 and r < m:
            m = r
    return m


class DockingController:
    def __init__(self):
        self.reset('approach')

    def reset(self, start_stage):
        self.stage = start_stage
        self.t0 = None
        self.est = None
        self.blacklist = []
        self.pallet_ref = None
        self.n_loc = 0
        self.track = None
        self.hist = []
        self.mstate = None
        self.n_bo = 0
        self.sweep = None
        self.search_fails = 0
        self.s = 'INIT'
        self.s_t0 = 0.0
        self.cleared_bl = False
        self.verify_wait0 = None

    # ------------------------------------------------------------------ helpers
    def _go(self, s, t):
        self.s = s
        self.s_t0 = t

    def _abort(self):
        return Command(*self._zero(A.a9_abort()), status='ABORTED')

    @staticmethod
    def _zero(c):
        return (c.v, c.w)

    def _enter_a3(self, t):
        _, self.track, self.hist, _, _, _ = A.a30_enter()
        self._go('SEARCH', t)

    def _enter_a4(self, t, obs):
        _, self.n_loc, retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(self.n_loc, self.est, P)
        if not retry_ok:
            self._go('FAIL', t)
        else:
            self._go('TAG', t)

    def _enter_a5(self, t):
        _, self.mstate, self.n_bo, _, _, _, _ = A.a50_enter()
        self._go('PLAN', t)

    def _guard(self, cmd, obs, rear=False):
        """LiDAR safety: block motion toward an obstacle inside the footprint margin."""
        if cmd.v > 0 and _sector_min(obs.scan, 0.0, math.radians(35)) < FRONT_STOP:
            return Command(0.0, cmd.w)
        if cmd.v < 0 and _sector_min(obs.scan, math.pi, math.radians(35)) < FRONT_STOP:
            return Command(0.0, cmd.w)
        return cmd

    # ------------------------------------------------------------------ step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, status='ABORTED')

    def _step(self, obs):
        t = obs.t
        if self.t0 is None:
            self.t0 = t
            if not A.a1_check_sensors(obs.odom, obs.scan):
                pass  # tolerate missing scan; odometry is needed
            A.a0_init()
            A.a2_navigate()
            if self.stage == 'guidance':
                self.est = None
                self._enter_a4(t, obs)
            else:
                self._enter_a3(t)
        limit = 120.0 if self.stage == 'guidance' else 150.0
        if t - self.t0 > limit - 3.0 or obs.odom is None:
            return self._abort()

        odom = obs.odom
        # pose estimate runs continuously
        if self.s not in ('SEARCH', 'TRACK') or self.est is not None or obs.tags:
            self.est = A.a43_update_pose(self.est, odom, obs.tags or [], t, P)
        if self.s in ('SEARCH', 'TRACK'):
            # shortcut: tag already reliably visible while approaching
            _, ready, _ = A.a44_pose_status(self.est, t, P)
            if ready and self.est is not None:
                cx, cy = A._cam_pose(self.est['px'], self.est['py'], self.est['yaw'])
                if math.hypot(cx, cy) < 2.9 and self.est['py'] > 0.5:
                    self.pallet_ref = self.pallet_ref or self.pallet_ref
                    self._enter_a5(t)

        for _ in range(6):
            r = self._run(obs, t)
            if r is not None:
                return r
        return Command()

    def _run(self, obs, t):
        s = self.s
        odom = obs.odom
        if s == 'SEARCH':
            cands = A.a321_candidates(obs.pallets or [], odom, self.blacklist, P)
            self.hist, ok, trk = A.a322_confirm(cands, self.hist, t, P)
            if ok:
                self.track = trk
                self._go('TRACK', t)
                return Command(0.0, 0.0)
            if t - self.s_t0 > SEARCH_TIMEOUT:
                self.search_fails += 1
                if self.blacklist and not self.cleared_bl:
                    self.cleared_bl = True
                    self.blacklist = []
                    self._go('SEARCH', t)
                    return Command()
                if self.search_fails >= 2:
                    A.a3e_fail()
                    return self._abort()
                self._go('SEARCH', t)
            cmd = A.a31_search_rotate(P)
            return cmd
        if s == 'TRACK':
            cands = A.a321_candidates(obs.pallets or [], odom, self.blacklist, P)
            self.track, lost = A.a323_track(cands, self.track, t, P)
            if lost:
                self.hist = []
                self.track = None
                self._go('SEARCH', t)
                return None
            cmd, done = A.a34_approach_cmd(self.track, odom, P)
            if done:
                _, self.pallet_ref, _ = A.a35_exit(self.track, odom)
                self._enter_a4(t, obs)
                return None
            return self._guard(cmd, obs)
        if s == 'TAG':
            tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)
            if ready:
                self._enter_a5(t)
                return None
            if self.est is not None and tag_ok:
                return A.a42_hold()
            if t - self.s_t0 > SWEEP_TIMEOUT:
                _, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
                self.pallet_ref = None
                if self.n_loc >= P['max_retries'] + 1 or self.n_loc > P['max_retries']:
                    return self._abort()
                self.est = None
                self._enter_a3(t)
                return None
            cmd, self.sweep = A.a41_tag_sweep(odom, self.pallet_ref, self.sweep, P)
            return cmd
        if s == 'FAIL':
            return self._abort()

        # ------------------------------ A5
        tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)
        if self.est is not None and s in ('REPOS', 'BACKOFF', 'REACQ'):
            stale = (t - self.est['t_meas']) > 25.0   # blind manoeuvres run on odometry
        if self.est is None or stale:
            return self._a5_fail(t, obs)
        est = self.est
        if s == 'PLAN':
            _, feas = A.a51_plan(est, P)
            self.mstate = None
            self._go('CONV' if feas else 'REPOS', t)
            return None
        if s == 'REPOS':
            cmd, self.mstate, done = A.a52_reposition(est, self.mstate, P)
            if done:
                self.mstate = None
                self._go('REACQ', t)
                return Command()
            return self._guard(cmd, obs)
        if s == 'REACQ':
            cmd, self.mstate = A.a54_reacquire(est, self.mstate, t, P)
            aligned = abs(A._cam_to_tag_err(est)) <= math.radians(4.0)
            if aligned and tag_ok:
                self._go('PLAN', t)
                return Command()
            if t - self.s_t0 > REACQ_TIMEOUT:
                return self._a5_fail(t, obs)
            return cmd
        if s == 'CONV':
            cmd, at_pre, miss = A.a53_converge(est, P)
            if miss:
                self._go('BACKOFF', t)
                return Command()
            if at_pre:
                self._go('FINAL', t)
                return None
            return self._guard(cmd, obs)
        if s == 'FINAL':
            cmd, at_dock = A.a55_final_approach(est, P)
            if at_dock:
                self.mstate = None
                self.verify_wait0 = None
                self._go('VERIFY', t)
                return Command()
            return self._guard(cmd, obs)
        if s == 'VERIFY':
            cmd, self.mstate, ok, bad, arrived = A.a56_verify(est, self.mstate, t, P)
            if ok:
                cd = A._cam_dist(est)
                if not tag_ok and cd >= 0.73:
                    # want a fresh tag fix before declaring success
                    if self.verify_wait0 is None:
                        self.verify_wait0 = t
                    if t - self.verify_wait0 < FIX_WAIT:
                        self.mstate = {'own': '56', 't0': t - P['verify_s']}
                        return Command()
                    bad = True
                else:
                    c = A.a6_docked()
                    return Command(c.v, c.w, status='DOCKED')
            if bad:
                self.mstate = None
                self._go('BACKOFF', t)
                return Command()
            return cmd
        if s == 'BACKOFF':
            cmd, self.mstate, self.n_bo, bo_ok, done = A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if not bo_ok and self.mstate is not None and not done:
                pass
            if done:
                self.mstate = None
                if not bo_ok:
                    return self._a5_fail(t, obs)
                self._go('REACQ', t)
                return Command()
            if _sector_min(obs.scan, math.pi, math.radians(35)) < 0.4:
                self.mstate = None
                self._go('REACQ', t)
                return Command()
            return cmd
        return self._abort()

    def _a5_fail(self, t, obs):
        A.a5e_fail()
        self.mstate = None
        self._enter_a4(t, obs)
        if self.s == 'FAIL':
            return self._abort()
        return Command()
