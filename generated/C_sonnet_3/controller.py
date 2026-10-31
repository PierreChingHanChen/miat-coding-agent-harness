"""AGV pallet docking controller: hierarchical state machine built on actions.py."""
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

# per-state time limits [s]
LIMITS = {'SEARCH': 30.0, 'APPROACH': 45.0, 'PROBE': 2.5, 'SWEEP': 14.0, 'CONV': 40.0,
          'REPOS': 35.0, 'REACQ': 6.0, 'FINAL': 40.0, 'VERIFY': 8.0, 'BACKOFF': 12.0}


class DockingController:
    def __init__(self):
        self.reset('guidance')

    def reset(self, start_stage):
        self.stage = start_stage
        self.limit = 112.0 if start_stage == 'guidance' else 142.0
        self.t0 = None
        self.state = 'INIT'
        self.ts = None           # state entry time
        self.est = None
        self.blacklist = []
        self.track = None
        self.hist = []
        self.pallet_ref = None
        self.n_loc = 0
        self.sweep = None
        self.mstate = None
        self.n_bo = 0
        self.from_a5 = False
        self.verify_t = None

    # ---------------------------------------------------------------- helpers
    def _go(self, state, t):
        self.state = state
        self.ts = t
        self.mstate = None

    def _abort(self):
        c = A.a9_abort()
        return Command(0.0, 0.0, 'ABORTED')

    def _guard(self, cmd, scan):
        """LiDAR safety: no forward motion with an obstacle in the front sector."""
        if cmd.v <= 0 or scan is None:
            return cmd
        try:
            m = float('inf')
            for i, r in enumerate(scan.ranges):
                a = A.wrap(scan.angle_min + i * scan.angle_increment)
                if abs(a) <= math.radians(30.0) and r > 0.05:
                    m = min(m, r)
            if m < 0.45:
                return Command(0.0, cmd.w)
        except Exception:
            pass
        return cmd

    # ---------------------------------------------------------------- step
    def step(self, obs):
        try:
            cmd = self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 'ABORTED')
        if cmd.status == 'RUNNING':
            cmd = self._guard(cmd, obs.scan)
        return cmd

    def _step(self, obs):
        t = obs.t
        odom = obs.odom
        if self.t0 is None:
            self.t0 = t
            self.ts = t
        if t - self.t0 > self.limit:
            return self._abort()
        # continuous pose estimation
        self.est = A.a43_update_pose(self.est, odom, obs.tags, t, P)
        tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)
        s = self.state
        if s in LIMITS and t - self.ts > LIMITS[s]:
            return self._timeout(t)

        if s == 'INIT':
            if not A.a1_check_sensors(odom, obs.scan):
                if t - self.t0 > 1.0:
                    return self._abort()
                return A.a0_init()[0]
            A.a2_navigate()
            if self.stage == 'approach':
                self._go('A3_ENTER', t)
            else:
                self._go('PROBE', t)
            return Command()

        if s == 'PROBE':          # guidance: try to see the tag right away
            if ready:
                self._go('A5_ENTER', t)
                return Command()
            if t - self.ts > LIMITS['PROBE'] - 0.1:
                self.est = None
                self._go('A3_ENTER', t)
            return Command()

        # ---------------- A3
        if s == 'A3_ENTER':
            cmd, self.track, self.hist, _, _, _ = A.a30_enter()
            self._go('SEARCH', t)
            return cmd
        if s in ('SEARCH', 'APPROACH') and ready:
            self._go('A5_ENTER', t)
            return Command()
        if s == 'SEARCH':
            cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
            self.hist, conf, trk = A.a322_confirm(cands, self.hist, t, P)
            if conf:
                self.track = trk
                self._go('APPROACH', t)
                return Command()
            return A.a31_search_rotate(P)
        if s == 'APPROACH':
            cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
            self.track, lost = A.a323_track(cands, self.track, t, P)
            if lost:
                self.track = None
                self.hist = []
                self._go('SEARCH', t)
                return Command()
            cmd, done = A.a34_approach_cmd(self.track, odom, P)
            if done:
                _, self.pallet_ref, _ = A.a35_exit(self.track, odom)
                self.from_a5 = False
                self._go('A4_ENTER', t)
                return Command()
            return cmd

        # ---------------- A4
        if s == 'A4_ENTER':
            cmd, self.n_loc, retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(self.n_loc, self.est, P)
            if not retry_ok:
                return self._abort()
            self._go('SWEEP', t)
            return cmd
        if s == 'SWEEP':
            if ready:
                self._go('A5_ENTER', t)
                return A.a42_hold()
            ref = None if self.from_a5 else self.pallet_ref
            cmd, self.sweep = A.a41_tag_sweep(odom, ref, self.sweep, P)
            return cmd

        # ---------------- A5
        if s == 'A5_ENTER':
            cmd, self.mstate, self.n_bo, _, _, _, _ = A.a50_enter()
            self._go('PLAN', t)
            return cmd
        if s in ('PLAN', 'CONV', 'FINAL', 'REPOS', 'REACQ', 'VERIFY', 'BACKOFF') and self.est is None:
            return self._to_a4(t, True)
        if s == 'PLAN':
            e = self.est
            if (e['py'] <= P['y_pre'] + 0.08 and abs(e['px']) <= 0.06 and e['py'] > P['dock_y']
                    and abs(A.wrap(e['yaw'] + math.pi / 2)) <= math.radians(8.0)):
                self._go('FINAL', t)
                return Command()
            cmd, feas = A.a51_plan(e, P)
            self._go('CONV' if feas else 'REPOS', t)
            return cmd
        if s == 'CONV':
            if stale:
                self._go('REACQ', t)
                return Command()
            cmd, at_pre, miss = A.a53_converge(self.est, P)
            if at_pre:
                self._go('FINAL', t)
                return cmd
            if miss:
                self._go('REPOS', t)
                return cmd
            return cmd
        if s == 'REPOS':
            cmd, self.mstate, done = A.a52_reposition(self.est, self.mstate, P)
            if done:
                self._go('REACQ', t)
            return cmd
        if s == 'REACQ':
            if ready:
                self._go('PLAN', t)
                return Command()
            cmd, self.mstate = A.a54_reacquire(self.est, self.mstate, t, P)
            return cmd
        if s == 'FINAL':
            if stale:
                self._go('REACQ', t)
                return Command()
            cmd, at_dock = A.a55_final_approach(self.est, P)
            if at_dock:
                self.verify_t = None
                self._go('VERIFY', t)
            return cmd
        if s == 'VERIFY':
            if self.verify_t is None:
                self.verify_t = t
            cmd, self.mstate, ok, bad, _ = A.a56_verify(self.est, self.mstate, t, P)
            if ok or bad:
                if ok and not tag_ok and t - self.verify_t < 4.0:
                    return cmd          # need a fresh tag fix to confirm
                if ok and tag_ok:
                    A.a6_docked()
                    return Command(0.0, 0.0, 'DOCKED')
                self._go('BACKOFF', t)
            return cmd
        if s == 'BACKOFF':
            if self.mstate is None and self.n_bo >= P['max_backoff']:
                return self._abort()
            cmd, self.mstate, self.n_bo, bo_ok, done = A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if done:
                self._go('REACQ', t)
            return cmd
        return self._abort()

    def _to_a4(self, t, from_a5):
        self.from_a5 = from_a5
        self._go('A4_ENTER', t)
        return Command()

    def _timeout(self, t):
        s = self.state
        if s == 'PROBE':
            self.est = None
            self._go('A3_ENTER', t)
            return Command()
        if s == 'SEARCH':
            A.a3e_fail()
            return self._abort()
        if s == 'APPROACH':          # never converged on a pallet: restart search
            self.track = None
            self.hist = []
            self._go('SEARCH', t)
            return Command()
        if s == 'SWEEP':
            if self.from_a5:
                self.est = None
                self._go('A3_ENTER', t)
                return Command()
            _, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
            self.pallet_ref = None
            self._go('A3_ENTER', t)
            return Command()
        if s == 'REACQ':
            return self._to_a4(t, True)
        A.a5e_fail()
        return self._abort()
