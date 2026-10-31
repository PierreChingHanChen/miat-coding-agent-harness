"""AGV pallet-pocket docking controller (A3 approach, A4 tag localisation, A5 guidance).

Decision logic only; all datapath computations are done by the Action functions in actions.py.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim.api import Command, V_MAX, W_MAX  # noqa: E402
import actions as A  # noqa: E402

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

RUNNING, DOCKED, ABORTED = 'RUNNING', 'DOCKED', 'ABORTED'

SEARCH_TIMEOUT_S = 45.0    # no pallet confirmed
APPROACH_BLOCK_TICKS = 15  # blocked by an obstacle this long -> drop candidate
TAG_TIMEOUT_S = 25.0       # tag localisation attempt
GUIDE_TIMEOUT_S = 90.0     # whole guidance phase
REPOS_TIMEOUT_S = 30.0
REACQ_TIMEOUT_S = 6.0
MAX_SEARCH_RESTARTS = 4
MAX_REPOS = 3
DEADLINE_MARGIN_S = 5.0
FRONT_CLEAR_M = 0.42       # translation guard (footprint radius 0.30)
ROT_CLEAR_M = 0.33         # disc footprint: rotation in place is safe beyond 0.30


class DockingController:
    def __init__(self):
        self.reset('approach')

    # ------------------------------------------------------------ lifecycle
    def reset(self, start_stage: str) -> None:
        self.guidance_start = (start_stage == 'guidance')
        self.limit = 120.0 if self.guidance_start else 150.0
        self.result = RUNNING
        self.t0 = None
        self.state = None
        self.sub = None
        self.state_t0 = 0.0
        self.sub_t0 = 0.0
        self.track = None
        self.hist = []
        self.blacklist = []
        self.pallet_ref = None
        self.est = None
        self.sweep = None
        self.mstate = None
        self.n_loc = 0
        self.n_bo = 0
        self.n_repos = 0
        self.n_search = 0
        self.blocked = 0
        self.init_done = False
        self.t_now = 0.0
        self._pending_start = True

    def step(self, obs) -> Command:
        try:
            cmd = self._step(obs)
        except Exception:  # unhandled errors must not escape: abandon the mission safely
            self.result = ABORTED
            cmd = Command()
        if self.result != RUNNING:
            return Command(0.0, 0.0, self.result)
        return Command(max(-V_MAX, min(V_MAX, float(cmd.v))),
                       max(-W_MAX, min(W_MAX, float(cmd.w))), RUNNING)

    # ------------------------------------------------------------ core
    def _step(self, obs) -> Command:
        if self.result != RUNNING:
            return Command()
        t = float(obs.t)
        if self.t0 is None:
            self.t0 = t
            self.t_now = t
            self.state_t0 = t
            self.sub_t0 = t
            _, self.init_done = A.a0_init()
            self._enter_start(t)
        self.t_now = t
        if t - self.t0 > self.limit - DEADLINE_MARGIN_S:
            return self._abort()

        odom = obs.odom
        if odom is None:
            return self._abort()
        scan = obs.scan
        A.a1_check_sensors(odom, scan)
        pallets = obs.pallets or []
        tags = obs.tags or []

        if self.state == 'SEARCH':
            cmd = self._search(odom, pallets, t)
        elif self.state == 'APPROACH':
            cmd = self._approach(odom, pallets, t)
        elif self.state == 'TAG':
            cmd = self._tag(odom, tags, t)
        elif self.state == 'GUIDE':
            cmd = self._guide(odom, tags, t)
        else:
            return self._abort()

        if self.result != RUNNING:
            return Command()
        return self._guard(cmd, scan)

    def _enter_start(self, t):
        if self.guidance_start:
            self._to_tag(t)          # A4 directly; sweep from current heading
        else:
            self._enter_search(t)    # A3

    def _abort(self):
        self.result = ABORTED
        return A.a9_abort()

    def _docked(self):
        self.result = DOCKED
        return A.a6_docked()

    def _enter_search(self, t):
        cmd, self.track, self.hist, _, _, _ = A.a30_enter()
        self.state = 'SEARCH'
        self.state_t0 = t
        self.blocked = 0
        return cmd

    def _to_tag(self, t):
        cmd, self.n_loc, retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(self.n_loc, None, P)
        if not retry_ok:
            return self._abort()
        self.state = 'TAG'
        self.state_t0 = t
        return cmd

    def _tag_fail(self, t):
        cmd, self.blacklist, _ = A.a4e_blacklist(self.blacklist, {'pos': self.pallet_ref['pos']}
                                                  if self.pallet_ref is not None else None)
        if self.n_loc > P['max_retries'] or self.n_search >= MAX_SEARCH_RESTARTS:
            return self._abort()
        self.n_search += 1
        return self._enter_search(t)

    # ------------------------------------------------------------ A3
    def _search(self, odom, pallets, t):
        if t - self.state_t0 > SEARCH_TIMEOUT_S:
            return self._abort()
        cands = A.a321_candidates(pallets, odom, self.blacklist, P)
        self.hist, confirmed, trk = A.a322_confirm(cands, self.hist, t, P)
        if confirmed:
            self.track = trk
            self.state = 'APPROACH'
            self.state_t0 = t
            self.blocked = 0
            return Command()
        return A.a31_search_rotate(P)

    def _approach(self, odom, pallets, t):
        cands = A.a321_candidates(pallets, odom, self.blacklist, P)
        self.track, lost = A.a323_track(cands, self.track, t, P)
        if lost or self.track is None:
            if self.n_search >= MAX_SEARCH_RESTARTS:
                return self._abort()
            self.n_search += 1
            return self._enter_search(t)
        cmd, done = A.a34_approach_cmd(self.track, odom, P)
        if done:
            cmd, self.pallet_ref, _ = A.a35_exit(self.track, odom)
            return self._to_tag(t)
        return cmd

    # ------------------------------------------------------------ A4
    def _tag(self, odom, tags, t):
        self.est = A.a43_update_pose(self.est, odom, tags, t, P)
        tag_ok, ready, stale = A.a44_pose_status(self.est, t, P)
        if self.est is not None and stale:
            self.est = None
            self.sweep = None
        if ready and A.a43_publish(self.est):
            return self._to_guide(t)
        if t - self.state_t0 > TAG_TIMEOUT_S:
            return self._tag_fail(t)
        if self.est is not None and tag_ok:
            return A.a42_hold()          # accumulate measurements
        cmd, self.sweep = A.a41_tag_sweep(odom, self.pallet_ref, self.sweep, P)
        return cmd

    # ------------------------------------------------------------ A5
    def _to_guide(self, t):
        cmd, self.mstate, _, _, _, _, _ = A.a50_enter()
        self.state = 'GUIDE'
        self.state_t0 = t
        self.n_repos = 0
        self._set_sub('PLAN', t)
        return cmd

    def _set_sub(self, sub, t):
        self.sub = sub
        self.sub_t0 = t
        self.mstate = None

    def _guide_retry(self, t):
        """Lost the pose during guidance: re-localise with the tag (budgeted)."""
        return self._to_tag(t)

    def _enter_repos(self, t):
        self.n_repos += 1
        if self.n_repos > MAX_REPOS:
            return self._guide_retry(t)
        self._set_sub('REPOS', t)
        return Command()

    def _guide(self, odom, tags, t):
        self.est = A.a43_update_pose(self.est, odom, tags, t, P)
        tag_ok, _, stale = A.a44_pose_status(self.est, t, P)
        sub = self.sub
        if sub == 'REPOS':
            # blind odometry manoeuvre (turns can exceed blind_max_s): own timeout applies
            stale = False
        elif sub == 'REACQ':
            stale = self.est is not None and (t - self.est['t_meas']) > P['blind_max_s']
        if self.est is None or stale or t - self.state_t0 > GUIDE_TIMEOUT_S:
            return self._guide_retry(t)
        est = self.est

        if sub == 'PLAN':
            cmd, feasible = A.a51_plan(est, P)
            if feasible:
                self._set_sub('CONV', t)
                return cmd
            return self._enter_repos(t)

        if sub == 'CONV':
            cmd, at_predock, miss = A.a53_converge(est, P)
            if at_predock:
                self._set_sub('FINAL', t)
                return Command()
            if miss:
                return self._enter_repos(t)
            return cmd

        if sub == 'REPOS':
            if t - self.sub_t0 > REPOS_TIMEOUT_S:
                return self._guide_retry(t)
            cmd, self.mstate, done = A.a52_reposition(est, self.mstate, P)
            if done:
                self._set_sub('REACQ', t)
                return Command()
            return cmd

        if sub == 'REACQ':
            cmd, self.mstate = A.a54_reacquire(est, self.mstate, t, P)
            aligned = (cmd.v == 0.0 and cmd.w == 0.0)
            if aligned and tag_ok:
                self._set_sub('PLAN', t)
                return Command()
            if t - self.sub_t0 > REACQ_TIMEOUT_S:
                return self._guide_retry(t)
            return cmd

        if sub == 'FINAL':
            cmd, at_dock = A.a55_final_approach(est, P)
            if at_dock:
                self._set_sub('VERIFY', t)
                return Command()
            return cmd

        if sub == 'VERIFY':
            cmd, self.mstate, verified, bad, _ = A.a56_verify(est, self.mstate, t, P)
            if verified:
                return self._docked()
            if bad:
                return self._set_backoff(t)
            return cmd

        if sub == 'BACKOFF':
            cmd, self.mstate, self.n_bo, bo_ok, done = A.a57_backoff(odom, self.mstate, self.n_bo, P)
            if done:
                return self._to_tag(t)
            return cmd

        return self._abort()

    def _set_backoff(self, t):
        if self.n_bo + 1 >= P['max_backoff']:
            return self._abort()
        self._set_sub('BACKOFF', t)
        self.mstate = None
        return Command()

    # ------------------------------------------------------------ safety guard
    def _guard(self, cmd, scan):
        """Zero motion that would drive into an obstacle seen by the LiDAR (last line of defence)."""
        v, w = float(cmd.v), float(cmd.w)
        if scan is not None and scan.ranges:
            front, rear, allmin = [], [], float('inf')
            for i, r in enumerate(scan.ranges):
                if r is None or not (r > 0.0) or math.isinf(r):
                    continue
                a = scan.angle_min + i * scan.angle_increment
                a = math.atan2(math.sin(a), math.cos(a))
                allmin = min(allmin, r)
                if abs(a) <= math.radians(35.0):
                    front.append(r)
                elif abs(a) >= math.radians(145.0):
                    rear.append(r)
            fmin = min(front) if front else float('inf')
            rmin = min(rear) if rear else float('inf')
            if v > 0.0 and fmin < FRONT_CLEAR_M:
                v = 0.0
                self._blocked(True)
            elif v < 0.0 and rmin < FRONT_CLEAR_M:
                v = 0.0
                self._blocked(True)
            else:
                self._blocked(False)
            if allmin < ROT_CLEAR_M and w != 0.0:
                w = 0.0
        return Command(v, w)

    def _blocked(self, flag):
        if self.state != 'APPROACH':
            return
        if not flag:
            self.blocked = 0
            return
        self.blocked += 1
        if self.blocked > APPROACH_BLOCK_TICKS and self.track is not None:
            self.blacklist = self.blacklist + [tuple(self.track['pos'])]
            self.track = None
            self.blocked = 0
            self.n_search += 1
            if self.n_search > MAX_SEARCH_RESTARTS:
                self.result = ABORTED
                return
            self._enter_search(self.t_now)
