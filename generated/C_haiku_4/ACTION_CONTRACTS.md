# Action Contracts (IDEF0 ICOM → function signatures)

Interface contracts of the Action functions (datapath). Implement **every** function below in one module `actions.py`. You must also write the controller that decides which Actions run when (see TASK).

## Conventions that apply to every Action

* One call = one 10 Hz tick. **No loops that wait, no sleep, no threads, no global/module-level mutable state.**
  Everything an action remembers between ticks is passed in and returned explicitly (e.g. `mstate`, `track`, `est`).
* Return values are a tuple in exactly the order given in the signature (a single output is returned bare).
* Only `sim.api` may be imported from the platform (plus `math`). Use `Command(v, w)`; never set `Command.status`
  (status is produced by the controller).
* `P` is the dict of Controls (thresholds) from the model; use the named keys, never hard-code their values.
* Frames: pallet frame = tag at origin, +y out of the pocket face. A **pose** is the AGV *base_link* `(px, py, yaw)` in
  the pallet frame. Bearings measured in the camera are **+ to the right**; angular velocity `w` is **+ to the left**.
  The camera is `CAM_X` m ahead of base_link: `cam = (px + CAM_X cos yaw, py + CAM_X sin yaw)`.
* `est` (pose estimate) is a dict with keys `px, py, yaw, t_meas, n, rej, o` (o = odometry pose `(x, y, yaw)` at the
  last update). `track` is a dict with keys `ba` (absolute bearing in the odometry frame, rad), `d` (m),
  `pos` (x, y in the odometry frame), `t` (time of last association). A candidate is a dict
  `ba, d, b, W, score, pos`.
* Angles are wrapped to (−π, π].

## Controls (P)

```json
{
 "w_search": 0.35,
 "score_min": 0.55,
 "width_min": 0.75,
 "width_max": 1.45,
 "confirm_k": 3,
 "confirm_window_s": 0.5,
 "confirm_tol_deg": 6.0,
 "track_gate_deg": 8.0,
 "track_lost_s": 2.0,
 "d_app": 1.9,
 "v_app": 0.25,
 "k_bear": 1.2,
 "app_bear_tol_deg": 4.0,
 "tag_sweep_deg": 35.0,
 "w_tag": 0.3,
 "init_meas": 3,
 "fuse_gain": 0.35,
 "gate_pos": 0.3,
 "gate_yaw_deg": 12.0,
 "pose_stale_s": 3.0,
 "near_dist": 1.1,
 "blind_max_s": 10.0,
 "fov_half_deg": 34.5,
 "fov_margin_deg": 10.0,
 "y_pre": 1.65,
 "repos_extra": 0.6,
 "v_conv": 0.2,
 "v_final": 0.12,
 "k_head": 1.6,
 "k_lat": 2.5,
 "dock_y": 1.13,
 "x_tol": 0.03,
 "head_tol_deg": 2.0,
 "verify_lat": 0.04,
 "verify_head_deg": 3.5,
 "verify_dist": 0.08,
 "verify_s": 0.5,
 "backoff_dist": 0.6,
 "max_backoff": 3,
 "max_retries": 2
}
```

### `a0_init() -> (cmd, init_done)`

Initial step of the mission. Stop the AGV and report `init_done=True`.

### `a1_check_sensors(odom, scan) -> (sensors_ok)`

`sensors_ok` is True iff an odometry message and a LiDAR scan are present (not None).

### `a2_navigate() -> (navi_ok)`

Placeholder for the unchanged Nav2 subsystem A2 (the viewpoint has already been reached): return True.

### `a6_docked() -> (cmd)`

Final step: stop (`Command()`).

### `a9_abort() -> (cmd)`

Abort step: stop (`Command()`).

### `a30_enter() -> (cmd, track, hist, approach_ok, app_fail, entered)`

Entry of A3. Stop; reset `track=None`, `hist=[]`; `approach_ok=False`, `app_fail=False`; `entered=True`.

### `a31_search_rotate(P) -> (cmd)`

Rotate in place counter-clockwise: `Command(0, P['w_search'])`.

### `a331_bbox_center(det) -> u`

Return the bbox centre column `det.u` as float.

### `a332_pallet_distance(depth_samples) -> (dist, ok)`

Robust range from the 25 depth samples: discard samples ≤ 0.05 m (invalid); if fewer than 5 remain → `(0.0, False)`. Take the median m of the valid samples, keep samples with |s − m| ≤ 0.3 m; if fewer than 3 remain → `(0.0, False)`; else return `(median(kept), True)`. (Median of an even count = mean of the two middle values.)

### `a333_pallet_bearing(u) -> bearing`

Bearing of image column u: `atan((u − IMG_W/2)/FX)` rad, + = right.

### `a321_candidates(pallets, odom, blacklist, P) -> (cands)`

Build pallet candidates from the raw detections. For each detection: skip if `score < P['score_min']`; range `d` via a332 (skip if not ok); bearing `b` via a331/a333; metric width `W = width_px·d/FX`, skip unless `P['width_min'] ≤ W ≤ P['width_max']`; absolute bearing in the odometry frame `ba = wrap(odom.yaw − b)`; camera position `c = odom + CAM_X·(cos odom.yaw, sin odom.yaw)`; object position `pos = c + d·(cos ba, sin ba)`; skip if `pos` lies within 0.8 m of any blacklisted point. Return the list of candidate dicts (`ba, d, b, W, score, pos`).

### `a322_confirm(cands, hist, t, P) -> (hist, pallet_confirmed, track)`

Temporal k-of-window confirmation while searching. Drop history entries older than `P['confirm_window_s']` (keep `t − h.t ≤ window`), append one entry `{'t': t, 'c': cand}` per current candidate. For each current candidate count the number of *distinct timestamps* in the history having a candidate with |wrap(ba diff)| ≤ `confirm_tol_deg` and |d diff| ≤ 0.4 m (the candidate itself counts). If the best count ≥ `P['confirm_k']` → `pallet_confirmed=True` and `track = {'ba','d','pos'` of that candidate, `'t': t}`; else `(hist, False, None)`. Ties → higher score.

### `a323_track(cands, track, t, P) -> (track, track_lost)`

Associate the current candidates with the track. Gate: |wrap(c.ba − track.ba)| ≤ `track_gate_deg` and |c.d − track.d| < 0.6 m; among gated candidates take the smallest angular difference and update `ba ← wrap(ba + 0.5·wrap(c.ba − ba))`, `pos ← midpoint(track.pos, c.pos)`, `d ← c.d`, `t ← t`. `track_lost = (t − track.t) > P['track_lost_s']`. If `track` is None → `(None, True)`.

### `a34_approach_cmd(track, odom, P) -> (cmd, approach_done)`

Drive toward `track.pos`. Camera position from odom (as in a321); `d` = distance camera→pos; `b = wrap(odom.yaw − atan2(pos − cam))` (+ right). If `d ≤ P['d_app']`: stop translating, rotate in place to centre the pallet (`w = sat(−1.5 b, 0.4)`); `approach_done = |b| ≤ app_bear_tol_deg` (and then w = 0). Otherwise `w = sat(−k_bear·b, 0.5)`, `v = v_app` if |b| < 10° else 0.05; `approach_done=False`. If track is None → `(Command(), False)`.

### `a35_exit(track, odom) -> (cmd, pallet_ref, approach_ok)`

Exit of A3: stop; `pallet_ref = {'ba': track.ba, 'pos': track.pos}`; `approach_ok=True`.

### `a3e_fail() -> (cmd, app_fail)`

Error exit of A3: stop; `app_fail=True`.

### `a40_enter(n_loc, est, P) -> (cmd, n_loc, retry_ok, est, sweep, pose_ok, tag_fail, entered)`

Entry of A4: stop; `n_loc ← n_loc+1`; `retry_ok = n_loc ≤ P['max_retries']` (using the incremented value); reset `est=None`, `sweep=None`; `pose_ok=False`, `tag_fail=False`; `entered=True`.

### `a41_tag_sweep(odom, pallet_ref, sweep, P) -> (cmd, sweep)`

Look for the tag. On the first call (`sweep is None`) build the target heading list `[ref, ref+A, ref−A, ref]` with `A = tag_sweep_deg` and `ref` = direction from the camera to `pallet_ref['pos']` (odometry frame) or the current odometry heading if `pallet_ref is None`; index i = 0. Each tick rotate toward `targets[i]` with `w = sat(2·err, P['w_tag'])`; when |err| < 3° output `Command()` and advance i cyclically. Return `(cmd, sweep)` (do not mutate the input dict).

### `a42_tag_to_pose(tag) -> (px, py, yaw)`

Pure geometry: tag (camera frame) → base_link pose in the pallet frame. `yaw = wrap(tag.yaw − π/2)`; camera position `cx = −(z cos yaw + x sin yaw)`, `cy = −(z sin yaw − x cos yaw)`; base = camera − CAM_X·(cos yaw, sin yaw). Returns `(px, py, yaw)`.

### `a43_update_pose(est, odom, tags, t, P) -> (est)`

Continuous action (runs every tick while A4 or A5 is active). Measurements = poses (a42) of tags with `id == TAG_ID`. If `est is None`: initialise from the first measurement `{px,py,yaw, t_meas=t, n=1, rej=0, o=(odom.x,odom.y,odom.yaw)}` or return None if there is none. Otherwise **predict** with the odometry increment since `est['o']` (convert the odometry displacement into the robot frame at the previous odometry heading: forward f, left l; apply them in the pallet frame at est.yaw; add the yaw increment), set `o` to the current odometry. Then for the first measurement: innovation e = z − est; **gate**: accept if |(ex,ey)| ≤ `gate_pos + 0.1·camera distance` and |eyaw| ≤ `gate_yaw_deg`; on accept blend with gain k = 0.5 while `n < init_meas`, else `fuse_gain`; set `t_meas=t`, `n+=1`, `rej=0`. On reject `rej+=1`; after 5 consecutive rejects re-initialise from the measurement (n=1). Return a new dict (never mutate the input).

### `a44_pose_status(est, t, P) -> (tag_ok, pose_ready, pose_stale)`

Continuous status predicates. If est is None → `(False, False, True)`. age = t − t_meas; `tag_ok = age < 0.35`; `pose_ready = tag_ok and n ≥ init_meas`; `pose_stale = (age > pose_stale_s and camera distance > near_dist) or age > blind_max_s` (camera distance = norm of the camera position in the pallet frame).

### `a42_hold() -> (cmd)`

Hold still while the filter accumulates measurements: `Command()`.

### `a43_publish(est) -> (pose_ok)`

`pose_ok = est is not None`.

### `a4e_blacklist(blacklist, pallet_ref) -> (cmd, blacklist, tag_fail)`

Error exit of A4 (no tag near the approached object): stop; append `pallet_ref['pos']` (as a tuple) to a *copy* of the blacklist if pallet_ref is not None; `tag_fail=True`.

### `a50_enter() -> (cmd, mstate, n_bo, bo_ok, arrived_ok, guide_fail, entered)`

Entry of A5: `(Command(), mstate=None, n_bo=0, bo_ok=True, arrived_ok=False, guide_fail=False, entered=True)`.

### `a51_plan(est, P) -> (cmd, feasible)`

Feasibility of a straight path to the pre-dock point (0, y_pre) that keeps the tag in view. Stop. If est None → infeasible. If |px| ≤ 0.05 and |py − y_pre| ≤ 0.15 → feasible. If py ≤ y_pre + 0.15 → infeasible. Else γ = atan2(|px|, py − y_pre); feasible iff γ ≤ β_max − 2°, with **β_max = fov_half_deg − fov_margin_deg**.

### `a52_reposition(est, mstate, P) -> (cmd, mstate, repos_done)`

Multi-tick odometry manoeuvre (state in `mstate`, owner tag `'52'`). On entry (mstate None or not owned): waypoint W = (xw, yw) with `yw = max(py, y_pre + repos_extra)`, `xw = sign(px)·(yw − y_pre)·tan(β_max − 8°)`; phase 'turn1'. turn1: rotate in place toward W (done when heading error < 3°; skip to turn2 if already within 0.06 m of W). drive: `v = 0.2·max(0, cos e)`, `w = sat(1.5 e, 0.5)` toward W until within 0.06 m. turn2: rotate so that the camera faces the tag (direction from camera position to origin); when error < 3° return `(Command(), None, True)`. In-place rotation: `w = sat(1.8 e, 0.5)` if |e| > 0.12 rad else `±0.15`.

### `a53_converge(est, P) -> (cmd, at_predock, predock_miss)`

Drive to the pre-dock point while the tag stays inside the FOV margin. If |px| ≤ 0.05 and py ≤ y_pre + 0.08 → `(Command(), at_predock=True, False)`. If py < y_pre − 0.25 and |px| > 0.10 → `(Command(), False, predock_miss=True)`. Otherwise target y `ty = y_pre` if py > y_pre else `py − 0.3`; desired heading toward (0, ty), then **clamped to within ±β_max of the camera→tag direction**; e = heading error; `v = v_conv·max(0,cos e)·min(1, max(0.3, dist_to_predock/0.4))`, `w = sat(k_head·e, 0.6)`.

### `a54_reacquire(est, mstate, t, P) -> (cmd, mstate)`

After a blind manoeuvre: rotate (in-place rule of a52, max 0.4 rad/s) until the camera points at the tag according to `est` (|error| ≤ 4°), then hold still waiting for a fresh fix. If est is None rotate slowly (`w=0.2`). Returns `(cmd, None)`.

### `a55_final_approach(est, P) -> (cmd, at_dock)`

Centre-line approach. ey = py − dock_y. If ey > 0.01: desired heading `−π/2 − sat(atan(k_lat·px), 30°)`, `v = v_final·min(1, max(0.25, ey/0.25))·max(0, cos e)`, `w = sat(k_head·e, 0.5)`, at_dock False. Else rotate in place to heading −π/2 (rule of a52, max 0.3) until |error| ≤ head_tol_deg, then `(Command(), True)`. Works on the odometry-propagated estimate when the tag is too close to be seen.

### `a56_verify(est, mstate, t, P) -> (cmd, mstate, dock_verified, dock_bad, arrived_ok)`

Settle and verify (owner tag `'56'`, remembers t0). Stop for `verify_s` seconds returning `(Command(), mstate, False, False, False)`. Then ok = |px| ≤ verify_lat and |py − dock_y| ≤ verify_dist and |wrap(yaw + π/2)| ≤ verify_head_deg; return `(Command(), None, ok, not ok, ok)`.

### `a57_backoff(odom, mstate, n_bo, P) -> (cmd, mstate, n_bo, bo_ok, backoff_done)`

Reverse `backoff_dist` along the current heading using odometry (owner `'57'`; on entry store the odometry start point and increment `n_bo`). `bo_ok = n_bo < max_backoff`. While driving `Command(-0.15, 0)`; when the travelled distance ≥ backoff_dist return `(Command(), None, n_bo, bo_ok, True)`.

### `a5e_fail() -> (cmd, guide_fail)`

Error exit of A5: stop; `guide_fail=True`.

## Test vectors

`test_contract.py` checks every function against reference input/output vectors (floats ±1e-3, Commands ±2e-3). All must pass before integration.