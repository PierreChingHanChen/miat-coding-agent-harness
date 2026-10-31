"""Action functions for the synthesized AGV pallet-pocket docking controller.

Pose conventions (pallet frame, from REQUIREMENTS.md):
  tag centre at origin, +y out of the pocket face toward the AGV, face on y = 0.
  Base pose (bx, by, th): th = -pi/2 means the AGV faces the pallet.
  Heading error e = th + pi/2 (positive = turned left). Tag yaw == e.
  Camera = base + CAM_X * (cos th, sin th).
  Odometry transform T maps odom frame to pallet frame: base = T o odom.
"""
import math

from sim.api import Command, CAM_X, DOCK_DIST, FX, IMG_W, TAG_ID, V_MAX, W_MAX

HALF_PI = math.pi / 2
ZERO = Command(0.0, 0.0)
LAT_MAX = 0.5   # max base lateral offset [m] for direct convergence without repositioning


# ---------------------------------------------------------------- helpers
def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _cmd(v, w):
    return Command(max(-V_MAX, min(V_MAX, float(v))),
                   max(-W_MAX, min(W_MAX, float(w))))


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def _inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), s * a[0] - c * a[1], _wrap(-a[2]))


def _cam(pb):
    return pb[0] + CAM_X * math.cos(pb[2]), pb[1] + CAM_X * math.sin(pb[2])


def _tag_bearing(pb):
    """Bearing of the tag centre seen from the camera, measured from the heading, + = right."""
    cx, cy = _cam(pb)
    th = pb[2]
    vx, vy = -cx, -cy
    fwd = vx * math.cos(th) + vy * math.sin(th)
    right = vx * math.sin(th) - vy * math.cos(th)
    return math.atan2(right, fwd)


def _tag_to_base(x, z, yaw):
    """Tag detection (camera frame) -> base pose in pallet frame, and camera distance to face."""
    e = yaw
    cxp = x * math.cos(e) - z * math.sin(e)
    cyp = z * math.cos(e) + x * math.sin(e)
    th = -HALF_PI + e
    return (cxp - CAM_X * math.cos(th), cyp - CAM_X * math.sin(th), th), cyp


def _new_est():
    return {'T': None, 'n': 0, 'rej': 0, 't_det': None, 'd_last': None,
            'pb': None, 'fresh': False}


# ---------------------------------------------------------------- G0
def a0_init():
    return ZERO, True


def a1_check_sensors(odom, scan):
    ok = odom is not None and scan is not None and len(scan.ranges) > 0
    return bool(ok)


def a2_navigate():
    return True


def a6_docked():
    return ZERO


def a9_abort():
    return ZERO


# ---------------------------------------------------------------- G3 (approach)
def a30_enter():
    return ZERO, None, [], False, False, True


def a31_search_rotate(P):
    return _cmd(0.0, P['w_search'])


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    for p in pallets or []:
        if p.score < P['score_min']:
            continue
        vals = sorted(z for z in p.depth_samples if 0.2 < z < 8.0)
        if len(vals) < 5:
            continue
        d = vals[len(vals) // 2]
        width = p.width_px * d / FX
        if not (P['width_min'] <= width <= P['width_max']):
            continue
        ang = math.atan2(p.u - IMG_W / 2.0, FX)        # + = right of camera axis
        bw = _wrap(odom.yaw - ang)                      # bearing in odom frame (static objects constant)
        xy = (odom.x + d * math.cos(bw), odom.y + d * math.sin(bw))
        if any(math.hypot(xy[0] - b[0], xy[1] - b[1]) < 0.8 for b in (blacklist or [])):
            continue
        cands.append({'bw': bw, 'xy': xy, 'd': d, 'score': p.score, 'width': width})
    return cands


def a322_confirm(cands, hist, t, P):
    hist = [h for h in (hist or []) if t - h[0] <= P['confirm_window_s'] + 1e-9]
    if not cands:
        return hist, False, None
    best = max(cands, key=lambda c: c['score'])
    hist = hist + [(t, best)]
    tol = math.radians(P['confirm_tol_deg'])
    same = [h for h in hist if abs(_wrap(h[1]['bw'] - best['bw'])) <= tol]
    if len(same) >= P['confirm_k']:
        return [], True, dict(best, t=t)
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    tol = math.radians(P['track_gate_deg'])
    match = [c for c in cands if abs(_wrap(c['bw'] - track['bw'])) <= tol]
    if match:
        c = max(match, key=lambda c: c['score'])
        return dict(c, t=t), False
    lost = (t - track['t']) > P['track_lost_s']
    return track, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return ZERO, False
    dx = track['xy'][0] - odom.x
    dy = track['xy'][1] - odom.y
    if math.hypot(dx, dy) <= P['d_app']:
        return ZERO, True
    err = _wrap(math.atan2(dy, dx) - odom.yaw)
    v = P['v_app'] if abs(err) < math.radians(20.0) else 0.0
    return _cmd(v, P['k_bear'] * err), False


def a35_exit(track, odom):
    return ZERO, (dict(track) if track is not None else None), track is not None


def a3e_fail():
    return ZERO, True


# ---------------------------------------------------------------- G4 (localize)
def a43_update_pose(est, odom, tags, t, P):
    e = dict(est) if est else _new_est()
    o = (odom.x, odom.y, odom.yaw)
    dets = [d for d in (tags or [])
            if d.id == TAG_ID and 0.3 < d.z < 4.0 and abs(d.yaw) < math.radians(60.0)]
    if dets:
        d = max(dets, key=lambda q: q.stamp)
        meas, cyp = _tag_to_base(d.x, d.z, d.yaw)
        accept = True
        pred = None
        if e['T'] is not None and e['n'] > 0:
            pred = _compose(e['T'], o)
            ep = math.hypot(meas[0] - pred[0], meas[1] - pred[1])
            ey = abs(_wrap(meas[2] - pred[2]))
            if ep > P['gate_pos'] or ey > math.radians(P['gate_yaw_deg']):
                e['rej'] += 1
                accept = e['rej'] >= 3          # persistent disagreement: re-initialise
        if accept:
            if pred is None or e['rej'] >= 3:
                e['T'] = _compose(meas, _inv(o))
                e['n'] = 1
            else:
                g = P['fuse_gain']
                tx, ty, tth = e['T']
                e['T'] = (tx + g * (meas[0] - pred[0]),
                          ty + g * (meas[1] - pred[1]),
                          _wrap(tth + g * _wrap(meas[2] - pred[2])))
                e['n'] += 1
            e['rej'] = 0
            e['t_det'] = d.stamp
            e['d_last'] = cyp
    if e['T'] is not None:
        e['pb'] = _compose(e['T'], o)
    e['fresh'] = (e['t_det'] is not None and (t - e['t_det']) <= 0.3
                  and e['n'] >= P['init_meas'])
    return e


def a44_pose_status(est, t, P):
    if not est or est.get('t_det') is None:
        return False, False, True
    age = t - est['t_det']
    tag_ok = age <= 0.3
    pose_ready = bool(est.get('fresh')) and tag_ok
    near = est['d_last'] is not None and est['d_last'] < P['near_dist']
    limit = P['blind_max_s'] if near else P['pose_stale_s']
    pose_stale = est.get('pb') is None or age > limit
    return tag_ok, pose_ready, pose_stale


def a40_enter(n_loc, est, P):
    n = n_loc + 1
    e = dict(est) if est else _new_est()
    e.update(n=0, rej=0, t_det=None, fresh=False)
    return ZERO, n, n <= P['max_retries'], e, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            rx, ry = pallet_ref['xy']
            c = math.atan2(ry - odom.y, rx - odom.x)
        else:
            c = odom.yaw
        sweep = {'c': c, 'dir': 1}
    amp = math.radians(P['tag_sweep_deg'])
    err = _wrap(sweep['c'] + sweep['dir'] * amp - odom.yaw)
    if abs(err) < math.radians(3.0):
        sweep = dict(sweep, dir=-sweep['dir'])
        err = _wrap(sweep['c'] + sweep['dir'] * amp - odom.yaw)
    w = max(-P['w_tag'], min(P['w_tag'], 2.0 * err))
    return _cmd(0.0, w), sweep


def a42_hold():
    return ZERO


def a43_publish(est):
    return bool(est and est.get('fresh'))


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(pallet_ref['xy'])
    return ZERO, bl, True


# ---------------------------------------------------------------- G5 (guide)
def a50_enter():
    mstate = {'bo_start': None, 'vs': None, 'vt': None, 'rt0': None, 'goal': None}
    return ZERO, mstate, 0, True, False, False, True


def a51_plan(est, P):
    pb = est.get('pb') if est else None
    if pb is None:
        return ZERO, False
    # tag must be in the FOV with margin, and the AGV must not be too far off the centre line
    # (larger lateral offsets are first repositioned by a52, which moves onto the centre line)
    feasible = (abs(_tag_bearing(pb)) <= math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
                and abs(pb[0]) <= LAT_MAX)
    return ZERO, bool(feasible)


def a52_reposition(est, mstate, P):
    ms = dict(mstate or {})
    pb = est.get('pb') if est else None
    if pb is None:
        return ZERO, ms, False
    bx, by, th = pb
    if ms.get('goal') is None:
        ms['goal'] = (0.0, by + P['repos_extra'])
    gx, gy = ms['goal']
    dx, dy = gx - bx, gy - by
    if math.hypot(dx, dy) < 0.05:
        return ZERO, ms, True
    err = _wrap(math.atan2(dy, dx) - th)
    v = 0.15 if abs(err) < math.radians(15.0) else 0.0
    return _cmd(v, 1.6 * err), ms, False


def a53_converge(est, P):
    pb = est.get('pb') if est else None
    if pb is None:
        return ZERO, False, False
    bx, by, th = pb
    dx, dy = -bx, P['y_pre'] - by
    dist = math.hypot(dx, dy)
    if dist < 0.06 and abs(_wrap(th + HALF_PI)) < math.radians(3.0):
        return ZERO, True, False
    if by < P['y_pre'] - 0.3:                 # overshot toward the pallet
        return ZERO, False, True
    gh = -HALF_PI if dist < 0.06 else math.atan2(dy, dx)
    # keep the tag inside the camera FOV: heading may deviate from the tag direction by at most lim
    cx, cy = _cam(pb)
    th_tag = math.atan2(-cy, -cx)
    lim = math.radians(P['fov_half_deg'] - P['fov_margin_deg'] - 3.0)
    gh = th_tag + max(-lim, min(lim, _wrap(gh - th_tag)))
    err = _wrap(gh - th)
    v = P['v_conv'] if abs(err) < math.radians(20.0) else 0.0
    return _cmd(v, P['k_head'] * err), False, False


def a54_reacquire(est, mstate, t, P):
    ms = dict(mstate or {})
    if ms.get('rt0') is None:
        ms['rt0'] = t
    pb = est.get('pb') if est else None
    if pb is None:
        return ZERO, ms
    cx, cy = _cam(pb)
    err = _wrap(math.atan2(-cy, -cx) - pb[2])
    return _cmd(0.0, 2.0 * err), ms


def a55_final_approach(est, P):
    pb = est.get('pb') if est else None
    if pb is None:
        return ZERO, False
    bx, by, th = pb
    gh = -HALF_PI - P['k_lat'] * bx          # steer back onto the centre line
    err = _wrap(gh - th)
    v = P['v_final'] if abs(err) < 0.5 else 0.0
    at_dock = (by <= P['dock_y'] + P['x_tol'] and abs(bx) <= 0.03
               and abs(_wrap(th + HALF_PI)) <= math.radians(P['head_tol_deg']))
    if at_dock:
        return ZERO, True
    return _cmd(v, P['k_head'] * err), False


def a56_verify(est, mstate, t, P):
    ms = dict(mstate or {})
    if ms.get('vs') is None:
        ms['vs'] = t
    pb = est.get('pb') if est else None
    tag_ok = est is not None and est.get('t_det') is not None and (t - est['t_det']) <= 0.3
    good = False
    if pb is not None and tag_ok:
        cx, cy = _cam(pb)
        good = (abs(cx) <= P['verify_lat']
                and abs(_wrap(pb[2] + HALF_PI)) <= math.radians(P['verify_head_deg'])
                and abs(cy - DOCK_DIST) <= P['verify_dist'])
    verified = False
    bad = False
    if good:
        if ms.get('vt') is None:
            ms['vt'] = t
        verified = (t - ms['vt']) >= P['verify_s'] - 1e-9
    else:
        ms['vt'] = None
        bad = tag_ok or (t - ms['vs']) > 1.0
    if verified or bad:
        ms['vs'] = None
        ms['vt'] = None
    return ZERO, ms, verified, (bad and not verified), verified


def a57_backoff(odom, mstate, n_bo, P):
    ms = dict(mstate or {})
    if ms.get('bo_start') is None:
        ms['bo_start'] = (odom.x, odom.y)
        n_bo = n_bo + 1
    sx, sy = ms['bo_start']
    done = math.hypot(odom.x - sx, odom.y - sy) >= P['backoff_dist']
    if done:
        ms['bo_start'] = None
    bo_ok = n_bo < P['max_backoff']
    return (ZERO if done else _cmd(-0.15, 0.0)), ms, n_bo, bo_ok, done


def a5e_fail():
    return ZERO, True
