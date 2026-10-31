"""Action functions for the AGV pallet-pocket docking GRAFCET (G0, G3, G4, G5).

Frames: pallet frame (tag centre at origin, +y toward AGV, face on y=0).
Pose estimates are base_link poses (x, y, h) in the pallet frame; the camera
sits CAM_X ahead of base_link along h.
Odometry-frame positions (candidates, track, blacklist) are in the odom frame.
"""
import math

from sim.api import Command, TAG_ID, CAM_X, FX, IMG_W, DOCK_DIST, DT, W_MAX

_D = dict(
    w_search=0.35, score_min=0.55, width_min=0.75, width_max=1.45,
    confirm_k=3, confirm_window_s=0.5, track_lost_s=2.0,
    d_app=1.9, v_app=0.25, k_bear=1.2,
    tag_sweep_deg=35.0, w_tag=0.3, init_meas=3, fuse_gain=0.35,
    gate_pos=0.3, gate_yaw_deg=12.0, pose_stale_s=3.0, near_dist=1.1,
    blind_max_s=10.0, fov_half_deg=34.5, fov_margin_deg=10.0,
    y_pre=1.65, v_conv=0.2, v_final=0.12, k_head=1.6, k_lat=2.5,
    dock_y=1.13, x_tol=0.03, head_tol_deg=2.0,
    verify_lat=0.04, verify_head_deg=3.5, verify_dist=0.08, verify_s=0.5,
    backoff_dist=0.6, max_backoff=3, max_retries=2, va_max_deg=25.0, repos_extra=0.6,
)
_BL_RADIUS = 0.8      # blacklist exclusion radius [m]
_TRACK_GATE = 0.6     # track association radius [m]
_CONFIRM_TOL = 0.3    # hypothesis clustering radius [m]
_TAG_FRESH = 0.25     # tag counted as seen if measured within this age [s]


def _p(P, k):
    if isinstance(P, dict) and k in P:
        return P[k]
    return _D[k]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clip(x, lim):
    return max(-lim, min(lim, x))


def _zero():
    return Command(0.0, 0.0)


def _cam_odom(odom):
    return odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw)


def _cam_base(est):
    return est['x'] + CAM_X * math.cos(est['h']), est['y'] + CAM_X * math.sin(est['h'])


def _tag_alpha(est):
    """Bearing of the tag centre from the camera, relative to heading (+ = left)."""
    cx, cy = _cam_base(est)
    return _wrap(math.atan2(-cy, -cx) - est['h']), math.hypot(cx, cy)


def _view_angle(est):
    """Viewing angle of the camera from the pocket-face normal."""
    cx, cy = _cam_base(est)
    return abs(math.atan2(cx, cy))


def _fov_lim(P):
    return math.radians(_p(P, 'fov_half_deg') - _p(P, 'fov_margin_deg'))


# ---------------------------------------------------------------- G0 actions

def a0_init():
    return _zero(), True


def a1_check_sensors(odom, scan):
    ok = (odom is not None and scan is not None and len(scan.ranges) > 0
          and all(math.isfinite(v) for v in (odom.x, odom.y, odom.yaw)))
    return bool(ok)


def a2_navigate():
    return True


def a6_docked():
    return _zero()


def a9_abort():
    return _zero()


# ---------------------------------------------------------------- G3 actions

def a30_enter():
    return _zero(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, _p(P, 'w_search'))


def _depth(samples):
    v = sorted(s for s in samples if math.isfinite(s) and 0.2 < s < 10.0)
    if len(v) < 5:
        return None
    med = v[len(v) // 2]
    v2 = [s for s in v if abs(s - med) <= 0.15 * med]   # reject background outliers
    if len(v2) < 5:
        return None
    return v2[len(v2) // 2]


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    if not pallets:
        return cands
    cx, cy = _cam_odom(odom)
    for pd in pallets:
        if not (_p(P, 'score_min') <= pd.score <= 1.0):
            continue
        d = _depth(pd.depth_samples)
        if d is None:
            continue
        width_m = pd.width_px * d / FX
        if not (_p(P, 'width_min') <= width_m <= _p(P, 'width_max')):
            continue
        b = -math.atan2(pd.u - IMG_W / 2.0, FX)          # + = left
        ang = odom.yaw + b
        px, py = cx + d * math.cos(ang), cy + d * math.sin(ang)
        if any(math.hypot(px - bx, py - by) < _BL_RADIUS for bx, by in blacklist):
            continue
        cands.append({'x': px, 'y': py, 'd': d, 'score': pd.score})
    return cands


def a322_confirm(cands, hist, t, P):
    win = _p(P, 'confirm_window_s')
    hist = [h for h in hist if t - h[0] <= win + 1e-9]
    if not cands:
        return hist, False, None
    best = max(cands, key=lambda c: c['score'])
    hist = hist + [(t, best['x'], best['y'])]
    n = sum(1 for (_, x, y) in hist
            if math.hypot(x - best['x'], y - best['y']) <= _CONFIRM_TOL)
    if n >= _p(P, 'confirm_k'):
        track = {'x': best['x'], 'y': best['y'], 'stamp': t, 'score': best['score']}
        return [], True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    best, bd = None, _TRACK_GATE
    for c in cands:
        dd = math.hypot(c['x'] - track['x'], c['y'] - track['y'])
        if dd < bd:
            best, bd = c, dd
    if best is not None:
        track = {'x': best['x'], 'y': best['y'], 'stamp': t, 'score': best['score']}
    lost = (t - track['stamp']) > _p(P, 'track_lost_s')
    return track, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return _zero(), False
    cx, cy = _cam_odom(odom)
    dx, dy = track['x'] - cx, track['y'] - cy
    if math.hypot(dx, dy) <= _p(P, 'd_app'):
        return _zero(), True
    b = _wrap(math.atan2(dy, dx) - odom.yaw)
    w = _clip(_p(P, 'k_bear') * b, W_MAX)
    v = _p(P, 'v_app') * max(0.0, math.cos(b))
    return Command(v, w), False


def a35_exit(track, odom):
    return _zero(), track, track is not None


def a3e_fail():
    return _zero(), True


# ---------------------------------------------------------------- G4 actions

def _meas_pose(z, x, yaw):
    """Base_link pose (pallet frame) from a tag detection in the camera frame."""
    hc = -math.pi / 2 - yaw                  # yaw = 0 when facing the tag squarely
    d = (math.cos(hc), math.sin(hc))        # optical axis
    r = (math.sin(hc), -math.cos(hc))       # camera right
    cxp = -(z * d[0] + x * r[0])            # camera position in pallet frame
    cyp = -(z * d[1] + x * r[1])
    return cxp - CAM_X * d[0], cyp - CAM_X * d[1], hc


def _pick_tag(tags):
    tag = None
    for d in tags or []:
        if d.id == TAG_ID and (tag is None or d.stamp > tag.stamp):
            tag = d
    if tag is not None and not (tag.z > 0.3 and abs(tag.yaw) < 1.2):
        tag = None
    return tag


def _predict(est, odom):
    ox, oy, oyaw = est['oref']
    dx, dy = odom.x - ox, odom.y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    bx, by = c * dx + s * dy, -s * dx + c * dy          # displacement in body frame
    h = est['h']
    est['x'] += math.cos(h) * bx - math.sin(h) * by
    est['y'] += math.sin(h) * bx + math.cos(h) * by
    est['h'] = _wrap(h + _wrap(odom.yaw - oyaw))
    est['oref'] = (odom.x, odom.y, odom.yaw)


def _new_est(mx, my, mh, odom, t, stamp):
    return {'x': mx, 'y': my, 'h': mh, 'n': 1, 'rej': 0, 't_tag': t,
            'stamp': stamp, 'oref': (odom.x, odom.y, odom.yaw), 'age': 0.0}


def a43_update_pose(est, odom, tags, t, P):
    tag = _pick_tag(tags)
    if est is not None:
        est = dict(est)
        _predict(est, odom)
    if tag is not None and (est is None or tag.stamp > est['stamp']):
        mx, my, mh = _meas_pose(tag.z, tag.x, tag.yaw)
        if est is None:
            est = _new_est(mx, my, mh, odom, t, tag.stamp)
        else:
            dist = math.hypot(mx - est['x'], my - est['y'])
            dyaw = abs(_wrap(mh - est['h']))
            if dist < _p(P, 'gate_pos') and dyaw < math.radians(_p(P, 'gate_yaw_deg')):
                g = _p(P, 'fuse_gain')
                est['x'] += g * (mx - est['x'])
                est['y'] += g * (my - est['y'])
                est['h'] = _wrap(est['h'] + g * _wrap(mh - est['h']))
                est['n'] += 1
                est['rej'] = 0
                est['t_tag'] = t
                est['stamp'] = tag.stamp
            else:
                est['rej'] += 1
                est['stamp'] = tag.stamp
                if est['rej'] >= 3:                      # persistent disagreement: re-initialise
                    est = _new_est(mx, my, mh, odom, t, tag.stamp)
    if est is None:
        return None
    est['age'] = (t - est['t_tag']) if est['t_tag'] is not None else 1e9
    return est


def a44_pose_status(est, t, P):
    if est is None or est['t_tag'] is None:
        return False, False, True
    age = t - est['t_tag']
    tag_ok = age <= _TAG_FRESH
    pose_ready = est['n'] >= _p(P, 'init_meas') and age <= _p(P, 'pose_stale_s')
    pose_stale = age > _p(P, 'blind_max_s')
    return tag_ok, pose_ready, pose_stale


def a40_enter(n_loc, est, P):
    n = n_loc + 1
    return _zero(), n, n <= _p(P, 'max_retries'), est, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    amp = math.radians(_p(P, 'tag_sweep_deg'))
    if sweep is None:
        if pallet_ref is not None:
            c = math.atan2(pallet_ref['y'] - odom.y, pallet_ref['x'] - odom.x)
        else:
            c = odom.yaw
        sweep = {'c': c, 'side': 1}
    err = _wrap(sweep['c'] + sweep['side'] * amp - odom.yaw)
    if abs(err) < math.radians(3.0):
        sweep = {'c': sweep['c'], 'side': -sweep['side']}
        err = _wrap(sweep['c'] + sweep['side'] * amp - odom.yaw)
    w = _clip(1.5 * err, _p(P, 'w_tag'))
    return Command(0.0, w), sweep


def a42_hold():
    return _zero()


def a43_publish(est):
    return bool(est is not None and est['n'] >= _D['init_meas'])


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist)
    if pallet_ref is not None:
        bl.append((pallet_ref['x'], pallet_ref['y']))
    return _zero(), bl, True


# ---------------------------------------------------------------- G5 actions

def a50_enter():
    return _zero(), {}, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return _zero(), False
    alpha, dist = _tag_alpha(est)
    feasible = (abs(alpha) <= _fov_lim(P) and dist >= _p(P, 'near_dist')
                and _view_angle(est) <= math.radians(_p(P, 'va_max_deg')))
    return _zero(), bool(feasible)


def a52_reposition(est, mstate, P):
    """Drive toward a point on the centre line farther out, so that the tag
    comes back into the FOV at a small viewing angle (R-06)."""
    if est is None:
        return _zero(), mstate, False
    alpha, dist = _tag_alpha(est)
    near = _p(P, 'near_dist')
    done = (dist >= near and abs(alpha) <= 0.8 * _fov_lim(P)
            and _view_angle(est) <= math.radians(0.5 * _p(P, 'va_max_deg')))
    if done:
        return _zero(), mstate, True
    tx, ty = 0.0, _p(P, 'y_pre') + _p(P, 'repos_extra')
    return _steer(est, tx, ty, P), mstate, False


def _align(hd, dist):
    """Blend the heading setpoint toward the pocket-facing heading near the goal
    (continuous, so that no sign chattering occurs at the switching distance)."""
    s = min(1.0, max(0.0, (dist - 0.15) / 0.25))
    return -math.pi / 2 + _wrap(hd + math.pi / 2) * s


def _steer(est, tx, ty, P):
    x, y, h = est['x'], est['y'], est['h']
    dx, dy = tx - x, ty - y
    d = math.hypot(dx, dy)
    th = math.atan2(dy, dx)
    rev = math.hypot(x, y) < math.hypot(tx, ty) - 0.2   # hysteresis band avoids chattering
    hd = _align(th + math.pi if rev else th, d)
    e = _wrap(hd - h)
    w = _clip(_p(P, 'k_head') * e, W_MAX)
    sgn = -1.0 if rev else 1.0
    v = sgn * _p(P, 'v_conv') * max(0.0, math.cos(e)) * min(1.0, d / 0.3)
    return Command(v, w)


def a53_converge(est, P):
    if est is None:
        return _zero(), False, True
    x, y, h = est['x'], est['y'], est['h']
    ty = _p(P, 'y_pre')
    dx, dy = -x, ty - y
    dist = math.hypot(dx, dy)
    miss = est['age'] > _p(P, 'track_lost_s')
    head_err = _wrap(-math.pi / 2 - h)
    if abs(x) < 0.2 and abs(dy) < 0.08 and abs(head_err) < math.radians(5.0):
        return _zero(), True, miss
    th = math.atan2(dy, dx)                              # direction toward pre-dock point
    rev = math.hypot(x, y) < ty - 0.05                   # too close: back away
    hd = _align(th + math.pi if rev else th, dist)
    e = _wrap(hd - h)
    w = _clip(_p(P, 'k_head') * e, W_MAX)
    sgn = -1.0 if rev else 1.0
    v = sgn * _p(P, 'v_conv') * max(0.0, math.cos(e)) * min(1.0, dist / 0.3)
    return Command(v, w), False, miss


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, _p(P, 'w_tag')), mstate
    alpha, _ = _tag_alpha(est)
    cx, cy = _cam_base(est)
    target_h = math.atan2(-cy, -cx)
    err = _wrap(target_h - est['h'])
    if abs(err) >= 0.05:
        w = _clip(err, _p(P, 'w_tag'))
    else:
        w = _p(P, 'w_tag')                  # sweep through the estimated tag bearing
    return Command(0.0, w), mstate


def a55_final_approach(est, P):
    if est is None:
        return _zero(), False
    x, y, h = est['x'], est['y'], est['h']
    hd = -math.pi / 2 - math.atan(_p(P, 'k_lat') * x)
    e = _wrap(hd - h)
    at = (y <= _p(P, 'dock_y') + 0.03 and abs(x) < _p(P, 'x_tol')
          and abs(_wrap(-math.pi / 2 - h)) < math.radians(_p(P, 'head_tol_deg')))
    if at:
        return _zero(), True
    w = _clip(_p(P, 'k_head') * e, W_MAX)
    return Command(_p(P, 'v_final') * max(0.0, math.cos(e)), w), False


def a56_verify(est, mstate, t, P):
    m = dict(mstate or {})
    m.setdefault('ok_n', 0)
    m.setdefault('bad_n', 0)
    need = math.ceil(_p(P, 'verify_s') / DT - 1e-9)
    ok = False
    if est is not None and est['age'] <= _TAG_FRESH:
        cx, cy = _cam_base(est)
        ok = (abs(cx) <= _p(P, 'verify_lat')
              and abs(_wrap(-math.pi / 2 - est['h'])) <= math.radians(_p(P, 'verify_head_deg'))
              and abs(cy - DOCK_DIST) <= _p(P, 'verify_dist'))
    if ok:
        m['ok_n'] += 1
    else:
        m['bad_n'] += 1
    verified = m['ok_n'] >= need
    bad = (not verified) and m['bad_n'] >= need
    if verified or bad:
        m = {}
    return _zero(), m, verified, bad, verified


def a57_backoff(odom, mstate, n_bo, P):
    m = dict(mstate or {})
    if m.get('bo_ref') is None:
        m['bo_ref'] = (odom.x, odom.y)
        n_bo = n_bo + 1
    rx, ry = m['bo_ref']
    moved = math.hypot(odom.x - rx, odom.y - ry)
    bo_ok = n_bo < _p(P, 'max_backoff')
    if moved >= _p(P, 'backoff_dist'):
        m['bo_ref'] = None
        return _zero(), m, n_bo, bo_ok, True
    return Command(-0.15, 0.0), m, n_bo, bo_ok, False


def a5e_fail():
    return _zero(), True
