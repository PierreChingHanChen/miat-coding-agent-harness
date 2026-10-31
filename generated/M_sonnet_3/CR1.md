# Change Request CR-1 — Safety supervisor (approved)

Requirement **R-10** is added; all existing requirements remain in force and the docking behaviour must otherwise be unchanged.

**R-10 Forward-motion inhibit.** Whenever any finite LiDAR return within ±30° of straight ahead is closer than
0.55 m to base_link, forward motion must be inhibited (`v ≤ 0`) in the **same** tick; rotation and reversing remain
allowed. Forward motion may resume only after no such return has been seen for 1.0 s. If forward motion has been
inhibited continuously for 20 s, the mission must be aborted (`ABORTED`, v = w = 0).
People may step in front of the AGV during any stage; the AGV must never touch them.

Verification: dynamic-obstacle trials (a person appears 0.75 m in front of the moving AGV for 4 s): zero collisions,
docking still succeeds; plus the full regression suite of R-01…R-09.
