# AGV Pallet-Pocket Docking Controller — Requirements (Phase 0, frozen)

## 1. Context

A differential-drive AGV must drive to the docking position in front of the fork pockets of a pallet.
Navigation (Nav2) has already brought the AGV to a viewpoint; your controller handles everything after that:

* **A3 – Recognize pallet and approach it**: find the pallet with the YOLACT pallet detector and drive toward it.
* **A4 – Localize pocket centre**: find the AprilTag (family 36h11, 10 cm, id 7) fixed at the centre of the pocket face and estimate the AGV pose relative to it.
* **A5 – Guide AGV into docking position**: bring the AGV onto the pocket centre line, facing the pocket, at the docking distance, and stop.

The platform API (message types, constants) is in `api.py` (module name `sim.api`). It is the **only** interface
available; there is no ROS, no TF library, no map, no other sensor.

### Geometry and frames

* Pallet frame: AprilTag centre at the origin, **+y points out of the pocket face toward the AGV side**, the pocket face
  lies on y = 0 and spans x ∈ [−0.6, 0.6] m; the pallet body occupies y ∈ [−1.0, 0].
* The camera is mounted `CAM_X = 0.28` m ahead of base_link on the AGV centre line, facing forward, at 0.35 m height;
  the tag centre is at 0.12 m height. Horizontal FOV 69°, vertical FOV 42° → the whole tag is visible only when the
  camera is ≥ 0.73 m from it.
* The 2-D LiDAR is at base_link; `Scan.ranges[i]` is at angle `angle_min + i*angle_increment` relative to the AGV
  heading (0 = straight ahead, + = counter-clockwise/left). The AGV footprint is a disc of radius 0.30 m around base_link.
* `Odom` starts at (0, 0, 0) at every trial and drifts slowly (wheel odometry). +w = counter-clockwise.
* `TagDetection`: tag centre in the camera frame (x right, z forward) and `yaw` = angle from the optical axis to the
  tag's inward normal (+ right, 0 when the camera faces the tag squarely). Detections are intermittent.
* `PalletDetection`: bbox centre column `u` (0..639, image centre 320, focal length `FX`), bbox width in px,
  score, and 25 raw depth samples [m] around the bbox centre (0.0 = invalid pixel; some samples hit the background).

### Controller interface (mandatory)

```python
class DockingController:
    def reset(self, start_stage: str) -> None: ...   # 'approach' (start A3) or 'guidance' (start A4/A5)
    def step(self, obs: Observation) -> Command: ...  # called at 10 Hz, must return within one tick (no sleeping/blocking)
```

`Command.v` (m/s, ≤0.30) and `Command.w` (rad/s, ≤0.60) are clipped by the platform. `Command.status` must be
`'RUNNING'`, `'DOCKED'` (mission accomplished, AGV stopped) or `'ABORTED'` (mission abandoned, AGV stopped).
The trial ends as soon as a status other than RUNNING is returned.

## 2. Operating conditions

* `start_stage='guidance'`: the AGV starts 2.2–2.8 m (base_link to tag) from the pallet, at a viewing angle between
  −45° and +45° from the centre line, heading toward the pallet centre. No pallet search has been done.
* `start_stage='approach'`: the AGV starts at a viewpoint 3.2–4.0 m from the pallet, within ±15° of the centre line,
  with an arbitrary heading (the pallet is usually not in view at the start).
* Simple scene: only the pallet. Cluttered scene: racks, tables and carts around the AGV; some of them look like
  pallets to the detector.

## 3. Requirements (each has a measurement method and acceptance criterion)

| ID | Requirement | Verification |
|---|---|---|
| R-01 | **Docking accuracy.** On `DOCKED`: camera lateral offset from the pocket centre line ≤ 0.06 m, camera-to-pocket-face distance 0.85 ± 0.12 m, heading error w.r.t. the face normal ≤ 6°, AGV stopped. | Ground truth at the instant DOCKED is reported. A DOCKED report outside these limits is a **false dock** (safety-relevant failure). |
| R-02 | **No collision** with the pallet or any obstacle. | Simulator contact check. |
| R-03 | **Bounded completion.** The controller must report DOCKED or ABORTED within 120 s (guidance) / 150 s (approach). Every behaviour that waits for a sensor event needs a timeout or a bounded retry; ABORTED implies v = w = 0. Unhandled exceptions are failures. | Timeout without a final status = **hang** failure. |
| R-04 | **Tag-loss tolerance.** Tag detections are intermittent and drop out completely when the camera is closer than 0.73 m or when the tag leaves the 69° FOV. Momentary loss must not make the AGV restart its approach; the final metres may have to be driven without the tag. | Guidance-grid success rate. |
| R-05 | **Detection envelope.** Per-frame tag detection is reliable (≥ 80 %) only within these camera-to-tag distances: 2.77 m frontal, 2.52 m at 15°, 2.42 m at 30°, 2.02 m at 45° (viewing angle measured from the face normal); about 0.3 m farther it drops to ≈ 0, and beyond 60° it is unreliable. When the tag cannot be found from the start position, the controller must use the pallet detector to get closer and try again. | Guidance grid incl. 2.5 m / 2.8 m at ±45°. |
| R-06 | **Field-of-view constraint.** The tag must stay in the camera FOV (with margin) while the AGV converges to the centre line; if the geometry does not allow that, the AGV must first reposition. | Guidance grid at ±30°, ±45°. |
| R-07 | **False pallet detections.** Pallet-like objects (score 0.45–0.88) can be reported; the true pallet scores 0.6–0.99 and its pocket face is 1.2 m wide (apparent width shrinks with cos of the viewing angle). The controller must not get stuck on, collide with, or dock at a false object; if no tag is found near a candidate it must try another candidate. | Cluttered-scene end-to-end success, collisions. |
| R-08 | **Robust ranging.** Depth samples contain invalid zeros and background outliers. | Fault-injection runs (40 % zero pixels). |
| R-09 | **Determinism / no blocking.** One `step()` call per tick, no sleeps, no threads, no randomness without a fixed seed. | Code inspection. |

Performance targets: guidance success ≥ 95 % inside the detection envelope; end-to-end success ≥ 95 % (simple) and
≥ 90 % (cluttered); zero collisions, zero false docks, zero hangs in all fault-injection runs.
