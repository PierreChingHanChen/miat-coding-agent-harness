"""DockingController: sequencing layer over the datapath Actions (actions.py).

State machine: init -> A3 (search, approach) -> A4 (tag localisation) -> A5
(plan, reposition, reacquire, converge, final, verify, backoff) -> DOCKED.
"""
import math
from sim.api import Command, TAG_ID
import actions as A

P = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45,
    "confirm_k": 3, "confirm_window_s": 0.5, "confirm_tol_deg": 6.0,
    "track_gate_deg": 8.0, "track_lost_s": 2.0, "d_app": 1.9, "v_app": 0.25,
    "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0, "w_tag": 0.3,
    "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3, "gate_yaw_deg": 12.0,
    "pose_stale_s": 3.0, "near_dist": 1.1, "blind_max_s": 10.0,
    "fov_half_deg": 34.5, "fov_margin_deg": 10.0, "y_pre": 1.65,
    "repos_extra": 0.6, "v_conv": 0.2, "v_final": 0.12, "k_head": 1.6,
    "k_lat": 2.5, "dock_y": 1.13, "x_tol": 0.03, "head_tol_deg": 2.0,
    "verify_lat": 0.04, "verify_head_deg": 3.5, "verify_dist": 0.08,
    "verify_s": 0.5, "backoff_dist": 0.6, "max_backoff": 3, "max_retries": 2,
}

# controller-level timeouts [s] (R-03)
T_LIMIT = {'guidance': 112.0, 'approach': 142.0}
T_SEARCH = 40.0        # one search episode (two full turns at w_search)
T_APPROACH = 60.0
T_A4_FIRST = 10.0      # first tag look when started in guidance (no pallet ref)
T_A4 = 20.0
T_REPOS = 30.0
T_REACQ = 12.0
T_CONV = 60.0
T_FINAL = 45.0
BLOCK_RANGE = 0.50     # lidar clearance for forward/backward motion [m]
BLOCK_MAX_S = 5.0


def _sector_min(scan, centre, half):
    if scan is None:
        return float('inf')
    m = float('inf')
    for i, r in enumerate(scan.ranges):
        a = A.wrap(scan.angle_min + i * scan.angle_increment - centre)
        if abs(a) <= half and 0.02 < r < m:
            m = r
    return m


class DockingController:
    def __init__(self):
        self.reset('guidance')

    def reset(self, start_stage):
        self.start_stage = start_stage if start_stage in ('approach', 'guidance') else 'guidance'
        self.st = 'init'
        self.t0 = None
        self.t_st = None
        self.final_status = None
        self.blacklist = []
        self.n_loc = 0
        self.est = None
        self.sweep = None
        self.mstate = None
        self.n_bo = 0
        self.track = None
        self.hist = []
        self.ref = None
        self.blocked_t = 0.0
        self.n_repos = 0
        self.cleared_bl = False
        self.first_a4 = (self.start_stage == 'guidance')
        self.last_t = None

    # ------------------------------------------------------------------
    def step(self, obs):
        if self.final_status is not None:
            return Command(0.0, 0.0, self.final_status)
        try:
            cmd = self._step(obs)
        except Exception:
            cmd = None
            self.final_status = 'ABORTED'
            return Command(0.0, 0.0, 'ABORTED')
        if cmd.status == 'RUNNING':
            cmd = Command(float(cmd.v), float(cmd.w), 'RUNNING')
        else:
            self.final_status = cmd.status
        return cmd

    def _abort(self):
        c = A.a9_abort()
        return Command(c.v, c.w, 'ABORTED')

    def _goto(self, st, t):
        self.st = st
        self.t_st = t

    # ------------------------------------------------------------------
    def _guard(self, cmd, obs, dt, rear=False):
        """LiDAR clearance: block motion toward an obstacle (R-02)."""
        if cmd.v == 0.0 or obs.scan is None:
            self.blocked_t = 0.0
            return cmd
        centre = math.pi if (cmd.v < 0) else 0.0
        if _sector_min(obs.scan, centre, math.radians(30.0)) < BLOCK_RANGE:
            self.blocked_t += dt
            return Command(0.0, 0.0)
        self.blocked_t = 0.0
        return cmd

    def _step(self, obs):
        t = obs.t
        odom = obs.odom
        dt = 0.1 if self.last_t is None else max(0.0, min(0.5, t - self.last_t))
        self.last_t = t
        if self.t0 is None:
            self.t0 = t
            self.t_st = t
        if t - self.t0 > T_LIMIT[self.start_stage] or odom is None:
            return self._abort()

        # continuous pose filter while A4/A5 are active
        if self.st.startswith('a4') or self.st.startswith('a5') or self.st in (
                'plan', 'repos', 'reacq', 'converge', 'final', 'verify', 'backoff', 'fail5'):
            self.est = A.a43_update_pose(self.est, odom, obs.tags, t, P)

        st = self.st
        # ---------------------------------------------------------- init
        if st == 'init':
            ok = A.a1_check_sensors(obs.odom, obs.scan)
            if not ok and t - self.t0 < 1.5:
                return A.a0_init()[0]
            A.a2_navigate()
            if self.start_stage == 'guidance':
                self._goto('a4_enter', t)
            else:
                self._goto('a3_enter', t)
            return self._step_again(obs)

        # ---------------------------------------------------------- A3
        if st == 'a3_enter':
            cmd, self.track, self.hist, _, _, _ = A.a30_enter()
            self._goto('search', t)
            return cmd

        if st == 'search':
            cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
            self.hist, conf, trk = A.a322_confirm(cands, self.hist, t, P)
            if conf:
                self.track = trk
                self._goto('approach', t)
                self.blocked_t = 0.0
                return Command()
            if t - self.t_st > T_SEARCH:
                if self.blacklist and not self.cleared_bl:
                    self.cleared_bl = True
                    self.blacklist = []
                    self._goto('a3_enter', t)
                    return Command()
                A.a3e_fail()
                return self._abort()
            return A.a31_search_rotate(P)

        if st == 'approach':
            cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
            self.track, lost = A.a323_track(cands, self.track, t, P)
            if lost or t - self.t_st > T_APPROACH:
                self._goto('a3_enter', t)
                return Command()
            cmd, done = A.a34_approach_cmd(self.track, odom, P)
            if done:
                cmd, self.ref, _ = A.a35_exit(self.track, odom)
                self._goto('a4_enter', t)
                return cmd
            cmd = self._guard(cmd, obs, dt)
            if self.blocked_t > BLOCK_MAX_S:      # something in the way: give up this object
                self.blacklist = list(self.blacklist) + [tuple(self.track['pos'])]
                self._goto('a3_enter', t)
                return Command()
            return cmd

        # ---------------------------------------------------------- A4
        if st == 'a4_enter':
            cmd, self.n_loc, retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(self.n_loc, self.est, P)
            if not (retry_ok or self.n_loc <= P['max_retries'] + 2):
                return self._abort()
            self._goto('a4', t)
            return cmd

        if st == 'a4':
            tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)
            if ready and A.a43_publish(self.est):
                self._goto('a5_enter', t)
                return self._step_again(obs)
            limit = T_A4_FIRST if (self.first_a4 and self.ref is None) else T_A4
            if t - self.t_st > limit:
                cmd, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.ref)
                self.first_a4 = False
                self.ref = None
                self.sweep = None
                self._goto('a3_enter', t)
                return cmd
            if tag_ok or (t - self.t_st < 1.0):
                return A.a42_hold()
            cmd, self.sweep = A.a41_tag_sweep(odom, self.ref, self.sweep, P)
            return cmd

        # ---------------------------------------------------------- A5
        if st == 'a5_enter':
            cmd, self.mstate, self.n_bo, _, _, _, _ = A.a50_enter()
            self.n_repos = 0
            self._goto('plan', t)
            return cmd

        tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)

        if st == 'plan':
            cmd, feasible = A.a51_plan(self.est, P)
            if self.est is None:
                self._goto('fail5', t)
                return cmd
            if not tag_ok:
                self.mstate = None
                self.sweep = None
                self._goto('reacq', t)
                return cmd
            if feasible:
                self._goto('converge', t)
            else:
                self.n_repos += 1
                if self.n_repos > 3:
                    self._goto('fail5', t)
                    return cmd
                self.mstate = None
                self._goto('repos', t)
            return cmd

        if st == 'repos':
            cmd, self.mstate, done = A.a52_reposition(self.est, self.mstate, P)
            if done:
                self.mstate = None
                self.sweep = None
                self._goto('reacq', t)
                return cmd
            if t - self.t_st > T_REPOS:
                self._goto('fail5', t)
                return Command()
            cmd = self._guard(cmd, obs, dt)
            if self.blocked_t > BLOCK_MAX_S:
                self._goto('fail5', t)
                return Command()
            return cmd

        if st == 'reacq':
            if tag_ok and ready:
                self._goto('plan', t)
                return Command()
            el = t - self.t_st
            if el > T_REACQ or self.est is None:
                self._goto('fail5', t)
                return Command()
            if el < 4.0:
                cmd, self.mstate = A.a54_reacquire(self.est, self.mstate, t, P)
                return cmd
            if self.sweep is None:
                # sweep around the direction in which the estimate says the tag is
                cx = self.est['px'] + A.CAM_X * math.cos(self.est['yaw'])
                cy = self.est['py'] + A.CAM_X * math.sin(self.est['yaw'])
                dirp = math.atan2(-cy, -cx)
                dod = odom.yaw + A.wrap(dirp - self.est['yaw'])
                ox = odom.x + A.CAM_X * math.cos(odom.yaw)
                oy = odom.y + A.CAM_X * math.sin(odom.yaw)
                ref = {'pos': (ox + math.cos(dod), oy + math.sin(dod))}
            else:
                ref = {'pos': (odom.x + math.cos(odom.yaw), odom.y + math.sin(odom.yaw))}
            cmd, new_sweep = A.a41_tag_sweep(odom, ref if self.sweep is None else None, self.sweep, P)
            self.sweep = new_sweep
            return cmd

        if st == 'converge':
            if self.est is None or t - self.t_st > T_CONV:
                self._goto('fail5', t)
                return Command()
            if stale:
                self.mstate = None
                self.sweep = None
                self._goto('reacq', t)
                return Command()
            cmd, at_pre, miss = A.a53_converge(self.est, P)
            if at_pre:
                self._goto('final', t)
                return cmd
            if miss:
                self.mstate = None
                self._goto('backoff', t)
                return cmd
            cmd = self._guard(cmd, obs, dt)
            if self.blocked_t > BLOCK_MAX_S:
                self._goto('fail5', t)
                return Command()
            return cmd

        if st == 'final':
            if self.est is None or t - self.t_st > T_FINAL:
                self._goto('fail5', t)
                return Command()
            if stale:
                self.mstate = None
                self.sweep = None
                self._goto('reacq', t)
                return Command()
            cmd, at_dock = A.a55_final_approach(self.est, P)
            if at_dock:
                self.mstate = None
                self._goto('verify', t)
                return Command()
            cmd = self._guard(cmd, obs, dt)
            if self.blocked_t > BLOCK_MAX_S:
                self._goto('fail5', t)
                return Command()
            return cmd

        if st == 'verify':
            el = t - self.t_st
            age = t - self.est['t_meas'] if self.est is not None else 1e9
            if self.mstate is None:
                # let the filter absorb fresh tag fixes while standing still
                if el < 0.6 or (age > 0.35 and el < 2.5):
                    return A.a42_hold()
                if age > 1.0:
                    self.mstate = None
                    self._goto('backoff', t)
                    return Command()
            cmd, self.mstate, verified, bad, _ = A.a56_verify(self.est, self.mstate, t, P)
            if verified:
                c = A.a6_docked()
                return Command(c.v, c.w, 'DOCKED')
            if bad:
                self.mstate = None
                self._goto('backoff', t)
            return cmd

        if st == 'backoff':
            cmd, self.mstate, self.n_bo, bo_ok, done = A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if not bo_ok and not done:
                self.mstate = None
                self._goto('fail5', t)
                return Command()
            if done:
                self.mstate = None
                self.sweep = None
                self._goto('plan', t)
                return cmd
            cmd = self._guard(cmd, obs, dt)
            if self.blocked_t > 2.0:
                self.mstate = None
                self._goto('fail5', t)
                return Command()
            return cmd

        if st == 'fail5':
            cmd, _ = A.a5e_fail()
            # re-localise (bounded by n_loc budget and the global timeout) or give up
            if self.n_loc <= P['max_retries'] + 1:
                self.first_a4 = False
                self.ref = None
                self.sweep = None
                self.mstate = None
                self._goto('a4_enter', t)
                return cmd
            return self._abort()

        return self._abort()

    def _step_again(self, obs):
        return self._step(obs)
