"""Closed-loop self-check of actions.py with the synthesized controller (kinematic toy world)."""
import math
import random
import sys
import types

ROOT = '/workspace/N_haiku_2'
sys.path.insert(0, ROOT)

import actions as A                                   # noqa: E402
from controller_synthesized import State, reset_all, tick  # noqa: E402
from sim.api import Odom, Scan, TagDetection, PalletDetection, Command, FX, CAM_X  # noqa: E402

P = {'w_search': 0.35, 'score_min': 0.55, 'width_min': 0.75, 'width_max': 1.45,
     'confirm_k': 3, 'confirm_window_s': 0.5, 'confirm_tol_deg': 6.0, 'track_gate_deg': 8.0,
     'track_lost_s': 2.0, 'd_app': 1.9, 'v_app': 0.25, 'k_bear': 1.2, 'app_bear_tol_deg': 4.0,
     'tag_sweep_deg': 35.0, 'w_tag': 0.3, 'init_meas': 3, 'fuse_gain': 0.35, 'gate_pos': 0.3,
     'gate_yaw_deg': 12.0, 'pose_stale_s': 3.0, 'near_dist': 1.1, 'blind_max_s': 10.0,
     'fov_half_deg': 34.5, 'fov_margin_deg': 10.0, 'y_pre': 1.65, 'repos_extra': 0.6,
     'v_conv': 0.2, 'v_final': 0.12, 'k_head': 1.6, 'k_lat': 2.5, 'dock_y': 1.13,
     'x_tol': 0.03, 'head_tol_deg': 2.0, 'verify_lat': 0.04, 'verify_head_deg': 3.5,
     'verify_dist': 0.08, 'verify_s': 0.5, 'backoff_dist': 0.6, 'max_backoff': 3,
     'max_retries': 2}

HFOV_HALF = math.radians(34.5)


def run(start, seed, approach, tag_p=0.8, pal_p=0.9, max_t=150.0):
    rng = random.Random(seed)
    bx, by, th = start
    t0 = (bx, by, th)
    odom0 = (0.0, 0.0, 0.0)
    S = State()
    reset_all(S, 0.0)
    C = types.SimpleNamespace(
        t=0.0, odom=None, scan=Scan(-math.pi, math.radians(2), [10.0] * 180),
        tags=[], pallets=[], P=P, init_done=False, start_approach=approach,
        cmd=Command(), status=None, blacklist=[], hist=[], track=None, cands=[],
        n_loc=0, est=None, sweep=None, mstate=None, n_bo=0, pallet_ref=None,
        retry_ok=True, bo_ok=True, pallet_confirmed=False, track_lost=False,
        approach_done=False, approach_ok=False, app_fail=False, entered=False,
        sensors_ok=False, navi_ok=False, pose_ok=False, tag_fail=False,
        tag_ok=False, pose_ready=False, pose_stale=False, feasible=False,
        repos_done=False, at_predock=False, predock_miss=False, at_dock=False,
        dock_verified=False, dock_bad=False, backoff_done=False, arrived_ok=False,
        guide_fail=False, bo_ok_=True)
    ref = (0.0, 0.0, 0.0)
    dt = 0.1
    t = 0.0
    x, y, th_ = bx, by, th
    collided = False
    for k in range(int(max_t / dt)):
        t = k * dt
        # ---- sensors (ground truth -> observation)
        cx = x + CAM_X * math.cos(th_)
        cy = y + CAM_X * math.sin(th_)
        fx_, fy_ = math.cos(th_), math.sin(th_)
        rx_, ry_ = math.sin(th_), -math.cos(th_)
        vx, vy = -cx, -cy
        z = vx * fx_ + vy * fy_
        xr = vx * rx_ + vy * ry_
        bearing = math.atan2(xr, z)
        dist_cam = math.hypot(cx, cy)
        tags = []
        if 0.73 <= cy <= 2.9 and abs(bearing) <= HFOV_HALF and abs(math.atan2(abs(cx), cy)) <= math.radians(60) \
                and rng.random() < tag_p:
            yaw = A._wrap(th_ + math.pi / 2)
            tags.append(TagDetection(7, xr + rng.gauss(0, 0.003), z + rng.gauss(0, 0.005),
                                     yaw + rng.gauss(0, 0.01), t))
        pallets = []
        if dist_cam < 6.0 and abs(bearing) <= HFOV_HALF and rng.random() < pal_p:
            d = dist_cam
            u = 320 + FX * math.tan(bearing)
            samples = [0.0 if rng.random() < 0.4 else d + rng.gauss(0, 0.02) for _ in range(25)]
            pallets.append(PalletDetection(u, FX * 1.2 / d, 0.8, samples, t))
        # ---- odometry (relative to start pose)
        odom_pose = A._compose(A._inv(t0), (x, y, th_))
        C.t = t
        C.odom = Odom(odom_pose[0], odom_pose[1], odom_pose[2], 0.0, 0.0)
        C.tags = tags
        C.pallets = pallets
        tick(S, C, A)
        if C.status:
            break
        c = C.cmd
        v, w = c.v, c.w
        th_ += w * dt
        x += v * dt * math.cos(th_)
        y += v * dt * math.sin(th_)
        if math.hypot(x, y) < 0.1:
            collided = True
    # final truth at end
    cx = x + CAM_X * math.cos(th_)
    cy = y + CAM_X * math.sin(th_)
    e = (th_ + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
    res = {'status': C.status, 'tend': round(t, 1), 'cam_lat': round(cx, 3),
           'cam_dist': round(cy, 3), 'head_deg': round(math.degrees(e), 2),
           'dist_to_tag': round(math.hypot(x, y), 2)}
    res['ok'] = (C.status == 'DOCKED' and abs(cx) <= 0.06 and abs(cy - 0.85) <= 0.12
                 and abs(math.degrees(e)) <= 6.0)
    return res


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'guidance'
    if mode == 'guidance':
        for alpha in (-45, -30, 0, 30, 45):
            for R in (2.2, 2.5, 2.8):
                a = math.radians(alpha)
                bx, by = R * math.sin(a), R * math.cos(a)
                th0 = math.atan2(-by, -bx)
                r = run((bx, by, th0), seed=int(alpha * 10 + R * 100), approach=False)
                print('guid', alpha, R, r)
    else:
        for alpha in (-15, 0, 15):
            for hd in (0, 90, -120, 180):
                a = math.radians(alpha)
                R = 3.6
                bx, by = R * math.sin(a), R * math.cos(a)
                r = run((bx, by, math.radians(hd)), seed=hd + alpha, approach=True)
                print('appr', alpha, hd, r)
