# Action list (signatures only)

The controller is synthesized from `model/miat2_grafcet.json` (see `controller_synthesized.py`, read-only). It calls the Action functions below with the inputs and expects the outputs listed in the model's ICOM binding. Implement **every** function in one module `actions.py`. There is no behavioural specification beyond the requirements, the model (step/transition structure, conditions, Controls `P`, initial data values) and the names.

## Conventions

* One call = one 10 Hz tick; no waiting loops, sleep, threads or module-level mutable state. State an action needs across ticks is passed in and returned (it is stored by the controller under the declared output name).
* Return a tuple in exactly the listed output order (a single output is returned bare). `cmd` outputs are `sim.api.Command(v, w)`; never set `Command.status`.
* Conditions used by transitions (e.g. `feasible`, `at_dock`, `pose_stale`) are the boolean outputs of these actions; their meaning follows from the requirements and the model.
* Only `math` and `sim.api` may be used.

## Functions

- `a0_init() -> (cmd, init_done)` — step 0 of G0 (IDEF0 A0)
- `a1_check_sensors(odom, scan) -> (sensors_ok)` — step 1 of G0 (IDEF0 A1)
- `a2_navigate() -> (navi_ok)` — step 2 of G0 (IDEF0 A2)
- `a6_docked() -> (cmd)` — step 6 of G0 (IDEF0 A0)
- `a9_abort() -> (cmd)` — step 9 of G0 (IDEF0 A0)
- `a30_enter() -> (cmd, track, hist, approach_ok, app_fail, entered)` — step 30 of G3 (IDEF0 A3)
- `a31_search_rotate(P) -> (cmd)` — step 31 of G3 (IDEF0 A31,A32)
- `a321_candidates(pallets, odom, blacklist, P) -> (cands)` — step 31 of G3 (IDEF0 A31,A32)
- `a322_confirm(cands, hist, t, P) -> (hist, pallet_confirmed, track)` — step 31 of G3 (IDEF0 A31,A32)
- `a323_track(cands, track, t, P) -> (track, track_lost)` — step 32 of G3 (IDEF0 A32,A34)
- `a34_approach_cmd(track, odom, P) -> (cmd, approach_done)` — step 32 of G3 (IDEF0 A32,A34)
- `a35_exit(track, odom) -> (cmd, pallet_ref, approach_ok)` — step 35 of G3 (IDEF0 A35)
- `a3e_fail() -> (cmd, app_fail)` — step 3E of G3 (IDEF0 A3)
- `a43_update_pose(est, odom, tags, t, P) -> (est)` — G4 continuous action (A4 Localize pocket center)
- `a44_pose_status(est, t, P) -> (tag_ok, pose_ready, pose_stale)` — G4 continuous action (A4 Localize pocket center)
- `a40_enter(n_loc, est, P) -> (cmd, n_loc, retry_ok, est, sweep, pose_ok, tag_fail, entered)` — step 40 of G4 (IDEF0 A4)
- `a41_tag_sweep(odom, pallet_ref, sweep, P) -> (cmd, sweep)` — step 41 of G4 (IDEF0 A41)
- `a42_hold() -> (cmd)` — step 42 of G4 (IDEF0 A42)
- `a43_publish(est) -> (pose_ok)` — step 43 of G4 (IDEF0 A43)
- `a4e_blacklist(blacklist, pallet_ref) -> (cmd, blacklist, tag_fail)` — step 4E of G4 (IDEF0 A4)
- `a50_enter() -> (cmd, mstate, n_bo, bo_ok, arrived_ok, guide_fail, entered)` — step 50 of G5 (IDEF0 A5)
- `a51_plan(est, P) -> (cmd, feasible)` — step 51 of G5 (IDEF0 A51)
- `a52_reposition(est, mstate, P) -> (cmd, mstate, repos_done)` — step 52 of G5 (IDEF0 A52)
- `a53_converge(est, P) -> (cmd, at_predock, predock_miss)` — step 53 of G5 (IDEF0 A53)
- `a54_reacquire(est, mstate, t, P) -> (cmd, mstate)` — step 54 of G5 (IDEF0 A54)
- `a55_final_approach(est, P) -> (cmd, at_dock)` — step 55 of G5 (IDEF0 A55)
- `a56_verify(est, mstate, t, P) -> (cmd, mstate, dock_verified, dock_bad, arrived_ok)` — step 56 of G5 (IDEF0 A56)
- `a57_backoff(odom, mstate, n_bo, P) -> (cmd, mstate, n_bo, bo_ok, backoff_done)` — step 57 of G5 (IDEF0 A57)
- `a5e_fail() -> (cmd, guide_fail)` — step 5E of G5 (IDEF0 A5)
