"""AGV pallet-pocket docking controller.

Sequences the datapath Actions of actions.py into the mission:
  INIT -> APP (A3 pallet search/approach) -> TAG (A4 tag acquisition, pose fusion)
       -> PLAN -> CONV -> FINAL -> VERIFY -> DOCKED
with REPOS / REACQ (field-of-view repositioning and re-acquisition), BACK (reverse
manoeuvre after a rejected dock), timeouts and bounded retries. ABORTED is reported
on every unrecoverable path, with v = w = 0.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import actions as A  # noqa: E402
from sim.api import Command, CAM_X, clamp_cmd  # noqa: E402

# Controls (same values as ACTION_CONTRACTS.md)
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

# Mission budgets: R-03 allows 120 s (guidance) / 150 s (approach); keep a margin.
LIMIT_S = {"approach": 140.0, "guidance": 110.0}
PHASE_MAX_S = {"PLAN": 5.0, "APP": 60.0, "TAG": 25.0, "REPOS": 15.0, "REACQ": 8.0,
               "CONV": 30.0, "FINAL": 20.0, "VERIFY": 5.0, "BACK": 8.0}
GUIDE_PHASES = ("PLAN", "REPOS", "REACQ", "CONV", "FINAL", "VERIFY", "BACK")
MAX_REPOS = 4          # re-plans that need a reposition manoeuvre
MAX_RECOVER = 4        # fallbacks to tag re-acquisition during guidance
GUARD_R = 0.40         # LiDAR clearance for commanded motion [m] (footprint radius 0.30)
GUARD_TIMEOUT_S = 2.0
SENSOR_TIMEOUT_S = 2.0
HOLD_MAX_S = 4.0       # max hold on a fresh but under-filtered tag fix


def _cam_xy(x, y, yaw):
    return x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)


def _cam_dist(est):
    cx, cy = _cam_xy(est["px"], est["py"], est["yaw"])
    return math.hypot(cx, cy)


class DockingController:
    def __init__(self):
        self.reset("approach")

    def reset(self, start_stage):
        self.stage = "guidance" if start_stage == "guidance" else "approach"
        self.limit = LIMIT_S[self.stage]
        self.t = 0.0
        self.t0 = None
        self.phase = "INIT"
        self.phase_t0 = 0.0
        self.final = None
        self.est = None
        self.track = None
        self.hist = []
        self.pallet_ref = None
        self.blacklist = []
        self.sweep = None
        self.mstate = None
        self.n_loc = 0
        self.retry_ok = True
        self.n_bo = 0
        self.n_repos = 0
        self.n_recover = 0
        self.guided = False
        self.after_back = "PLAN"
        self.tag_from_guide = False
        self.hold_t0 = None
        self.sensor_t0 = None
        self.guard_t0 = None

    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:  # an unhandled exception is a failure (R-03): fail safe
            self.final = "ABORTED"
            return Command(0.0, 0.0, "ABORTED")

    # ------------------------------------------------------------ core loop
    def _step(self, obs):
        if self.final is not None:
            return Command(0.0, 0.0, self.final)
        self.t = float(obs.t)
        if self.t0 is None:
            self.t0 = self.t
            self.phase_t0 = self.t
        if self.t - self.t0 > self.limit:
            return self._finish("ABORTED")
        if not A.a1_check_sensors(obs.odom, obs.scan):
            if self.sensor_t0 is None:
                self.sensor_t0 = self.t
            if self.t - self.sensor_t0 > SENSOR_TIMEOUT_S:
                return self._finish("ABORTED")
            return Command(0.0, 0.0, "RUNNING")
        self.sensor_t0 = None

        cmd = self._dispatch(obs)
        if self.final is not None:
            return Command(0.0, 0.0, self.final)

        cmd, blocked = self._guard(cmd, obs.scan)
        if blocked:
            if self.phase == "APP":
                # obstacle in the approach path: treat the candidate as false and search again
                if self.track is not None:
                    self.blacklist.append(tuple(self.track["pos"]))
                self.track = None
                self.hist = []
            else:
                if self.guard_t0 is None:
                    self.guard_t0 = self.t
                if self.t - self.guard_t0 > GUARD_TIMEOUT_S:
                    return self._finish("ABORTED")
        else:
            self.guard_t0 = None
        return clamp_cmd(Command(cmd.v, cmd.w, "RUNNING"))

    def _dispatch(self, obs):
        if self.phase == "INIT":
            if self.stage == "guidance":
                return self._enter_tag()
            cmd, self.track, self.hist, _, _, _ = A.a30_enter()
            self._to("APP")
            return cmd

        if self.phase == "APP":
            if self.t - self.phase_t0 > PHASE_MAX_S["APP"]:
                return self._finish("ABORTED")
            return self._app(obs)

        tag_ok, ready, stale = self._monitor(obs)
        ph = self.phase
        if self.t - self.phase_t0 > PHASE_MAX_S[ph]:
            return self._on_timeout(ph)
        if ph == "TAG":
            return self._tag(obs, tag_ok, ready)
        if self.est is None:
            return self._finish("ABORTED")
        if ph == "PLAN":
            return self._plan()
        if ph == "REPOS":
            return self._repos(obs)
        if ph == "REACQ":
            return self._reacq(tag_ok)
        if ph == "CONV":
            return self._conv(obs, stale)
        if ph == "FINAL":
            return self._final(obs, stale)
        if ph == "VERIFY":
            return self._verify(obs)
        if ph == "BACK":
            return self._back(obs)
        return self._finish("ABORTED")

    # ------------------------------------------------------------ helpers
    def _to(self, phase):
        self.phase = phase
        self.phase_t0 = self.t

    def _finish(self, status):
        self.final = status
        return Command(0.0, 0.0, status)

    def _monitor(self, obs):
        self.est = A.a43_update_pose(self.est, obs.odom, obs.tags, self.t, P)
        return A.a44_pose_status(self.est, self.t, P)

    def _guard(self, cmd, scan):
        """Stop translation that would run into a LiDAR return closer than GUARD_R."""
        if scan is None or cmd.v == 0.0:
            return cmd, False
        front, rear = math.inf, math.inf
        for i, r in enumerate(scan.ranges):
            if not (r > 0.02 and math.isfinite(r)):
                continue
            th = scan.angle_min + i * scan.angle_increment
            th = math.atan2(math.sin(th), math.cos(th))
            a = abs(th)
            if a <= math.radians(35.0):
                front = min(front, r)
            elif a >= math.radians(145.0):
                rear = min(rear, r)
        if (cmd.v > 0.0 and front < GUARD_R) or (cmd.v < 0.0 and rear < GUARD_R):
            return Command(0.0, cmd.w), True
        return cmd, False

    # ------------------------------------------------------------ A3 approach
    def _app(self, obs):
        t, odom = self.t, obs.odom
        cands = A.a321_candidates(obs.pallets, odom, self.blacklist, P)
        if self.track is None:
            self.hist, confirmed, tr = A.a322_confirm(cands, self.hist, t, P)
            if not confirmed:
                return A.a31_search_rotate(P)
            self.track = tr
        self.track, lost = A.a323_track(cands, self.track, t, P)
        age = t - self.track["t"]
        cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
        dist = math.hypot(self.track["pos"][0] - cx, self.track["pos"][1] - cy)
        if lost and (age > 6.0 or dist > 2.5):
            # far from the object and out of sight for long: search again
            self.track = None
            self.hist = []
            return A.a31_search_rotate(P)
        cmd, done = A.a34_approach_cmd(self.track, odom, P)
        if done:
            cmd, self.pallet_ref, _ = A.a35_exit(self.track, odom)
            self.track = None
            self.hist = []
            return self._enter_tag()
        return cmd

    # ------------------------------------------------------------ A4 tag
    def _enter_tag(self):
        cmd, self.n_loc, self.retry_ok, self.est, self.sweep, _, _, _ = A.a40_enter(
            self.n_loc, self.est, P)
        self.tag_from_guide = False
        self.hold_t0 = None
        self._to("TAG")
        return cmd

    def _enter_tag_sweep(self):
        """Re-acquire the tag during guidance (est is kept and propagated)."""
        self.n_recover += 1
        if self.n_recover > MAX_RECOVER:
            return self._finish("ABORTED")
        self.sweep = None
        self.hold_t0 = None
        self.tag_from_guide = True
        self._to("TAG")
        return Command()

    def _tag(self, obs, tag_ok, ready):
        if ready:
            if not self.guided:
                _, self.mstate, self.n_bo, _, _, _, _ = A.a50_enter()
                self.guided = True
            self._to("PLAN")
            return Command()
        if tag_ok:
            # fresh detection but not yet enough measurements: hold still
            if self.hold_t0 is None:
                self.hold_t0 = self.t
            if self.t - self.hold_t0 > HOLD_MAX_S:
                self.est = None
                self.hold_t0 = None
            return A.a42_hold()
        self.hold_t0 = None
        cmd, self.sweep = A.a41_tag_sweep(obs.odom, self.pallet_ref, self.sweep, P)
        return cmd

    def _on_timeout(self, ph):
        if ph == "TAG":
            return self._tag_failed()
        if ph == "REACQ":
            return self._enter_tag_sweep()
        return self._finish("ABORTED")

    def _tag_failed(self):
        if self.tag_from_guide:
            return self._finish("ABORTED")
        cmd, self.blacklist, _ = A.a4e_blacklist(self.blacklist, self.pallet_ref)
        if not self.retry_ok:
            return self._finish("ABORTED")
        # no tag near this object: go back and let the detector offer another candidate
        self.track = None
        self.hist = []
        self.est = None
        self.sweep = None
        self._to("APP")
        return cmd

    # ------------------------------------------------------------ A5 guidance
    def _plan(self):
        cmd, feasible = A.a51_plan(self.est, P)
        if feasible:
            self._to("CONV")
            return cmd
        self.n_repos += 1
        if self.n_repos > MAX_REPOS:
            return self._finish("ABORTED")
        self.mstate = None
        self._to("REPOS")
        return cmd

    def _repos(self, obs):
        cmd, self.mstate, done = A.a52_reposition(self.est, self.mstate, P)
        if done:
            self.mstate = None
            self._to("REACQ")
            return Command()
        return cmd

    def _reacq(self, tag_ok):
        cmd, self.mstate = A.a54_reacquire(self.est, self.mstate, self.t, P)
        self.mstate = None
        if tag_ok and cmd.v == 0.0 and cmd.w == 0.0:
            self._to("PLAN")
            return Command()
        return cmd

    def _recover(self):
        """Pose lost while far from the tag: rotate back to it, wait for a fresh fix."""
        self.n_recover += 1
        if self.n_recover > MAX_RECOVER:
            return self._finish("ABORTED")
        self.mstate = None
        self._to("REACQ")
        return Command()

    def _stale_action(self):
        age = self.t - self.est["t_meas"]
        if age > P["blind_max_s"]:
            return self._finish("ABORTED")
        return self._recover()

    def _conv(self, obs, stale):
        if stale:
            return self._stale_action()
        cmd, at_pre, miss = A.a53_converge(self.est, P)
        if at_pre:
            self.mstate = None
            self._to("FINAL")
            return Command()
        if miss:
            return self._backoff(obs, "PLAN")
        return cmd

    def _final(self, obs, stale):
        if stale:
            return self._stale_action()
        cmd, at_dock = A.a55_final_approach(self.est, P)
        if at_dock:
            self.mstate = None
            self._to("VERIFY")
            return Command()
        return cmd

    def _verify(self, obs):
        cmd, self.mstate, dv, db, _ = A.a56_verify(self.est, self.mstate, self.t, P)
        if dv:
            return self._finish("DOCKED")
        if db:
            self.mstate = None
            return self._backoff(obs, "PLAN")
        return cmd

    def _backoff(self, obs, next_phase):
        cmd, self.mstate, self.n_bo, bo_ok, _ = A.a57_backoff(obs.odom, None, self.n_bo, P)
        if not bo_ok:
            return self._finish("ABORTED")
        self.after_back = next_phase
        self._to("BACK")
        return cmd

    def _back(self, obs):
        cmd, self.mstate, self.n_bo, _, done = A.a57_backoff(obs.odom, self.mstate, self.n_bo, P)
        if done:
            self.mstate = None
            self._to(self.after_back)
            return Command()
        return cmd
