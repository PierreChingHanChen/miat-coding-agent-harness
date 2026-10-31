"""Action functions for the AGV pallet-pocket docking controller (ACTION_LIST.md).

Frames: odometry world frame (x, y, yaw). Pallet frame: tag at origin, +y = normal
out of the pocket face toward the AGV, +x = (N_y, -N_x). Camera = base_link + CAM_X.
Stateful values are passed in and returned; no module-level mutable state.
"""
import math
from sim.api import Command, TAG_ID, CAM_X, DOCK_DIST, FX, W_MAX

_DEFAULTS = dict(
    w_search=0.35, score_min=0.55, width_min=0.75, width_max=1.45,
    confirm_k=3, confirm_window_s=0.5, confirm_tol_deg=6.0, track_gate_deg=8.0,
    track_lost_s=2.0, d_app=1.9, v_app=0.25, k_bear=1.2, app_bear_tol_deg=4.0,
    tag_sweep_deg=35.0, w_tag=0.3, init_meas=3, fuse_gain=0.35, gate_pos=0.3,
    gate_yaw_deg=12.0, pose_stale_s=3.0, near_dist=1.1, blind_max_s=10.0,
    fov_half_deg=34.5, fov_margin_deg=10.0, y_pre=1.65, repos_extra=0.6,
    v_conv=0.2, v_final=0.12, k_head=1.6, k_lat=2.5, dock_y=1.13, x_tol=0.03,
    head_tol_deg=2.0, verify_lat=0.04, verify_head_deg=3.5, verify_dist=0.08,
    verify_s=0.5, backoff_dist=0.6, max_backoff=3, max_retries=2,
)

_IMG_CX = 320.0
_IMG_W = 640.0


def _p(P, k):
    try:
        if P is None:
            return _DEFAULTS[k]
        if isinstance(P, dict):
            return P.get(k, _DEFAULTS[k])
        return getattr(P, k, _DEFAULTS[k])
    except Exception:
        return _DEFAULTS[k]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clip(x, lim):
    return max(-lim, min(lim, x))


def _cam(odom):
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _tag_world(odom, tg):
    """Tag centre and inward normal heading in odometry frame from one detection."""
    fx, fy = math.cos(odom.yaw), math.sin(odom.yaw)
    rx, ry = fy, -fx                       # camera right
    cx, cy = _cam(odom)
    T = (cx + tg.z * fx + tg.x * rx, cy + tg.z * fy + tg.x * ry)
    # normal points from tag toward camera: squarely viewed -> -forward
    nx = math.sin(tg.yaw) * rx - math.cos(tg.yaw) * fx
    ny = math.sin(tg.yaw) * ry - math.cos(tg.yaw) * fy
    return T, math.atan2(ny, nx)


def _pallet_pose(est, odom):
    """Camera position (px, py) and heading phi, error e in pallet frame."""
    cx, cy = _cam(odom)
    dx, dy = cx - est['T'][0], cy - est['T'][1]
    nx, ny = math.cos(est['N']), math.sin(est['N'])
    tx, ty = ny, -nx
    px = dx * tx + dy * ty
    py = dx * nx + dy * ny
    fx, fy = math.cos(odom.yaw), math.sin(odom.yaw)
    phi = math.atan2(fx * nx + fy * ny, fx * tx + fy * ty)
    e = _wrap(phi + math.pi / 2)          # heading error vs facing the face (-y)
    return px, py, phi, e


def _guide(est, P, v_max):
    """Converge/final law: steer toward a point on the centre line ahead of the face."""
    px, py, phi = est['px'], est['py'], est['phi']
    if py > 1.6:
        qy = max(0.5 * py, DOCK_DIST)            # aim at a point ahead on the centre line
        phi_des = math.atan2(-(py - qy), -px)
    else:                                        # close: pure lateral correction, always forward
        phi_des = -math.pi / 2 - math.atan(10.0 * px)
    e = _wrap(phi_des - phi)
    w = _clip(_p(P, 'k_head') * e, W_MAX)
    v = v_max * max(0.0, math.cos(e))
    return v, w


# ---- A0 / A1 / A2 ---------------------------------------------------------
def a0_init():
    return Command(0.0, 0.0), True


def a1_check_sensors(odom, scan):
    ok = odom is not None and scan is not None and len(scan.ranges) > 0
    return bool(ok)


def a2_navigate():
    return True


def a6_docked():
    return Command(0.0, 0.0)


def a9_abort():
    return Command(0.0, 0.0)


# ---- A3: pallet search and approach ---------------------------------------
def a30_enter():
    return Command(0.0, 0.0), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, _p(P, 'w_search'))


def a321_candidates(pallets, odom, blacklist, P):
    out = []
    cx, cy = _cam(odom)
    score_min = _p(P, 'score_min')
    wmin, wmax = _p(P, 'width_min'), _p(P, 'width_max')
    bl = list(blacklist or [])
    for p in pallets or []:
        if p.score < score_min:
            continue
        ds = sorted(d for d in p.depth_samples if math.isfinite(d) and 0.2 < d < 8.0)
        if len(ds) < 3:
            continue
        d = ds[len(ds) // 2]                       # median is robust to zeros/background
        width_m = p.width_px * d / FX
        if not (wmin <= width_m <= wmax):
            continue
        hw = odom.yaw - math.atan2(p.u - _IMG_CX, FX)
        x = cx + d * math.cos(hw)
        y = cy + d * math.sin(hw)
        if any(math.hypot(x - bx, y - by) < 1.0 for bx, by in bl):
            continue
        out.append(dict(x=x, y=y, d=d, hw=hw, score=p.score))
    out.sort(key=lambda c: -c['score'])
    return out


def a322_confirm(cands, hist, t, P):
    window = _p(P, 'confirm_window_s')
    hist = [h for h in (hist or []) if t - h[0] <= window + 1e-9]
    if not cands:
        return hist, False, None
    b = cands[0]
    hist.append((t, b['x'], b['y'], b['hw']))
    tol = math.radians(_p(P, 'confirm_tol_deg'))
    n = sum(1 for (_, hx, hy, hh) in hist
            if math.hypot(hx - b['x'], hy - b['y']) <= 0.5 and abs(_wrap(hh - b['hw'])) <= tol)
    if n >= _p(P, 'confirm_k'):
        return hist, True, dict(b, seen=t)
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, False
    best = None
    for c in cands or []:
        dd = math.hypot(c['x'] - track['x'], c['y'] - track['y'])
        if dd <= 0.6 and (best is None or dd < best[0]):
            best = (dd, c)
    if best is not None:
        return dict(best[1], seen=t), False
    lost = (t - track['seen']) > _p(P, 'track_lost_s')
    return track, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(0.0, 0.0), False
    cx, cy = _cam(odom)
    rem = math.hypot(track['x'] - cx, track['y'] - cy)
    if rem <= _p(P, 'd_app'):
        return Command(0.0, 0.0), True
    ang = math.atan2(track['y'] - cy, track['x'] - cx)
    err = _wrap(ang - odom.yaw)
    w = _clip(_p(P, 'k_bear') * err, W_MAX)
    v = _p(P, 'v_app') * max(0.0, math.cos(err))
    return Command(v, w), False


def a35_exit(track, odom):
    if track is None:
        return Command(0.0, 0.0), None, False
    return Command(0.0, 0.0), (track['x'], track['y']), True


def a3e_fail():
    return Command(0.0, 0.0), True


# ---- A4: pocket-centre localisation ---------------------------------------
def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags or []:
        if tg.id == TAG_ID:
            meas = tg
            break
    e = dict(est) if est else None
    if meas is not None:
        T, N = _tag_world(odom, meas)
        if e is None or e.get('n', 0) == 0:
            e = dict(T=T, N=N, n=1, seen=t, bad=0)
        else:
            dpos = math.hypot(T[0] - e['T'][0], T[1] - e['T'][1])
            dyaw = _wrap(N - e['N'])
            if dpos <= _p(P, 'gate_pos') and abs(dyaw) <= math.radians(_p(P, 'gate_yaw_deg')):
                g = _p(P, 'fuse_gain')
                e['T'] = (e['T'][0] + g * (T[0] - e['T'][0]), e['T'][1] + g * (T[1] - e['T'][1]))
                e['N'] = e['N'] + g * dyaw
                e['n'] += 1
                e['seen'] = t
                e['bad'] = 0
            else:
                e['bad'] = e.get('bad', 0) + 1
                if e['bad'] >= _p(P, 'init_meas'):      # persistent disagreement: re-initialise
                    e = dict(T=T, N=N, n=1, seen=t, bad=0)
    if e is not None and e.get('n', 0) > 0:
        px, py, phi, err = _pallet_pose(e, odom)
        e.update(px=px, py=py, phi=phi, e=err)
    return e


def a44_pose_status(est, t, P):
    if est is None or est.get('n', 0) == 0:
        return False, False, True
    age = t - est['seen']
    tag_ok = age <= 0.3
    pose_ready = est['n'] >= _p(P, 'init_meas')
    stale = age > _p(P, 'blind_max_s') or (age > _p(P, 'pose_stale_s') and est['py'] > _p(P, 'near_dist'))
    return tag_ok, pose_ready, stale


def a40_enter(n_loc, est, P):
    n = (n_loc or 0) + 1
    retry_ok = (n - 1) < _p(P, 'max_retries')
    return Command(0.0, 0.0), n, retry_ok, est, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            c = math.atan2(pallet_ref[1] - odom.y, pallet_ref[0] - odom.x)
        else:
            c = odom.yaw
        sweep = {'c': c, 'dir': 1.0}
    half = math.radians(_p(P, 'tag_sweep_deg'))
    tgt = sweep['c'] + sweep['dir'] * half
    e = _wrap(tgt - odom.yaw)
    if abs(e) < 0.05:
        sweep = dict(sweep, dir=-sweep['dir'])
    wt = _p(P, 'w_tag')
    w = _clip(1.5 * e, wt)
    return Command(0.0, w), sweep


def a42_hold():
    return Command(0.0, 0.0)


def a43_publish(est):
    return bool(est is not None and est.get('n', 0) >= _DEFAULTS['init_meas'])


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref))
    return Command(0.0, 0.0), bl, True


# ---- A5: guidance into docking position -----------------------------------
def a50_enter():
    return Command(0.0, 0.0), {}, 0, True, False, False, True


def a51_plan(est, P):
    fov = math.radians(_p(P, 'fov_half_deg') - _p(P, 'fov_margin_deg'))
    bearing = _wrap(math.atan2(-est['py'], -est['px']) - est['phi'])
    feasible = abs(bearing) <= fov
    return Command(0.0, 0.0), feasible


def a52_reposition(est, mstate, P):
    bearing = _wrap(math.atan2(-est['py'], -est['px']) - est['phi'])
    w = _clip(_p(P, 'k_head') * bearing, W_MAX)
    done = abs(bearing) <= math.radians(10.0)
    if done:
        return Command(0.0, 0.0), mstate, True
    return Command(0.0, w), mstate, False


def a53_converge(est, P):
    v, w = _guide(est, P, _p(P, 'v_conv'))
    px, py = est['px'], est['py']
    at_predock = py <= _p(P, 'y_pre') and abs(px) <= 0.05
    predock_miss = py <= _p(P, 'y_pre') and abs(px) > 0.12
    return Command(v, w), at_predock, predock_miss


def a54_reacquire(est, mstate, t, P):
    return Command(0.0, 0.0), mstate


def a55_final_approach(est, P):
    px, py, e, phi = est['px'], est['py'], est['e'], est['phi']
    xt = _p(P, 'verify_lat')                         # lateral deadband at dock (0.04 m)
    lim = DOCK_DIST + _p(P, 'x_tol')                 # camera-distance limit for stopping
    if py > lim:
        v_max = _p(P, 'v_final') if py > 1.0 else min(_p(P, 'v_final'), 0.06)
        v, w = _guide(est, P, v_max)
    elif abs(px) > xt:
        # short creep to fix lateral error, holding the camera near 0.80 m (inside 0.85 +/- 0.12 m);
        # reverse creep when closer than that, which flips the steering sign
        sgn = 1.0 if py > 0.80 else -1.0
        v = 0.05 * sgn
        w = _clip(_p(P, 'k_head') * _wrap(-math.pi / 2 - sgn * math.atan(10.0 * px) - phi), W_MAX)
    else:
        v = 0.0                                      # stopped: align heading to face normal
        w = _clip(_p(P, 'k_head') * _wrap(-math.pi / 2 - phi), W_MAX)
    at_dock = py <= lim and abs(px) <= xt and abs(e) <= math.radians(_p(P, 'head_tol_deg'))
    return Command(v, w), at_dock


def a56_verify(est, mstate, t, P):
    m = dict(mstate or {})
    px, py, e = est['px'], est['py'], est['e']
    ok = (abs(px) <= _p(P, 'verify_lat') and abs(e) <= math.radians(_p(P, 'verify_head_deg'))
          and abs(py - DOCK_DIST) <= _p(P, 'verify_dist'))
    if ok:
        m.pop('bad_since', None)
        if m.get('ok_since') is None:
            m['ok_since'] = t
        verified = (t - m['ok_since']) >= _p(P, 'verify_s')
        bad = False
    else:
        m.pop('ok_since', None)
        verified = False
        if m.get('bad_since') is None:
            m['bad_since'] = t
        bad = (t - m['bad_since']) >= 1.0
    w = _clip(-_p(P, 'k_head') * e, W_MAX)
    return Command(0.0, w), m, verified, bad, verified


def a57_backoff(odom, mstate, n_bo, P):
    m = dict(mstate or {})
    n = n_bo or 0
    if 'bo0' not in m:
        n += 1
        m['bo0'] = (odom.x, odom.y)
    bo_ok = n <= _p(P, 'max_backoff')
    x0, y0 = m['bo0']
    done = math.hypot(odom.x - x0, odom.y - y0) >= _p(P, 'backoff_dist')
    if done:
        m.pop('bo0', None)
        return Command(0.0, 0.0), m, n, bo_ok, True
    return Command(-0.15, 0.0), m, n, bo_ok, False


def a5e_fail():
    return Command(0.0, 0.0), True
