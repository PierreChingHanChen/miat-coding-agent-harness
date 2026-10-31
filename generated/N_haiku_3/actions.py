"""Action functions for the MIAT-synthesized pallet-pocket docking controller.

Frames
------
Pallet frame: origin at the AprilTag centre, +y points out of the pocket face
toward the AGV, the face lies on y = 0. Camera pose in this frame is stored as
(cx, cy, psi): cy is the camera-to-face distance, cx the lateral position
(+x = left when facing the pallet), psi the camera heading error (CCW positive)
with psi = 0 when the camera looks squarely at the face (heading -y).

Odometry is CCW-positive; the body-frame motion of the camera equals that of
base_link, so dead reckoning is applied directly to the camera pose.
"""
import math

from sim.api import Command, CAM_X, DOCK_DIST, FX, IMG_W, TAG_ID, V_MAX, W_MAX

_DEFAULTS = {
    'w_search': 0.35, 'score_min': 0.55, 'width_min': 0.75, 'width_max': 1.45,
    'confirm_k': 3, 'confirm_window_s': 0.5, 'confirm_tol_deg': 6.0,
    'track_gate_deg': 8.0, 'track_lost_s': 2.0, 'd_app': 1.9, 'v_app': 0.25,
    'k_bear': 1.2, 'app_bear_tol_deg': 4.0, 'tag_sweep_deg': 35.0, 'w_tag': 0.3,
    'init_meas': 3, 'fuse_gain': 0.35, 'gate_pos': 0.3, 'gate_yaw_deg': 12.0,
    'pose_stale_s': 3.0, 'near_dist': 1.1, 'blind_max_s': 10.0,
    'fov_half_deg': 34.5, 'fov_margin_deg': 10.0, 'y_pre': 1.65,
    'repos_extra': 0.6, 'v_conv': 0.2, 'v_final': 0.12, 'k_head': 1.6,
    'k_lat': 2.5, 'dock_y': 1.13, 'x_tol': 0.03, 'head_tol_deg': 2.0,
    'verify_lat': 0.04, 'verify_head_deg': 3.5, 'verify_dist': 0.08,
    'verify_s': 0.5, 'backoff_dist': 0.6, 'max_backoff': 3, 'max_retries': 2,
}

# Tolerances used internally (not part of the model Controls).
TAG_FRESH_S = 0.15        # tag measurement considered current
TAG_CAND_TIMEOUT = 0.6    # candidate-match window for pallet tracking (s)
CAND_RADIUS = 0.8         # blacklist exclusion radius [m]
TRACK_GATE_M = 0.6        # track association radius [m]
CONFIRM_RADIUS = 0.3      # confirmation association radius [m]
BACKOFF_V = -0.12         # reverse speed during backoff [m/s]
REJECT_LIMIT = 5          # consecutive gated-out tag measurements before re-init
VERIFY_BAD_HEAD_DEG = 8.0
VERIFY_BAD_LAT = 0.08
VERIFY_BAD_DIST = 0.15
VERIFY_TIMEOUT_EXTRA = 3.0
PSI_CLAMP = 0.8
LOOK = 0.6
STEER = 'lat'
LAT_GAIN = 4.0
CX_ROOM = 0.5
SLIDE_V = 0.15


def _p(P, k):
    if isinstance(P, dict) and k in P:
        return P[k]
    return _DEFAULTS[k]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _cmd(v=0.0, w=0.0):
    return Command(v=_clamp(float(v), -V_MAX, V_MAX), w=_clamp(float(w), -W_MAX, W_MAX))


def _zero():
    return Command(v=0.0, w=0.0)


def _fwd_left(psi):
    """Unit vectors of the camera heading in the pallet frame: (forward, left)."""
    return (math.sin(psi), -math.cos(psi)), (math.cos(psi), math.sin(psi))


def _tag_bearing(est):
    """Signed angle from the camera optical axis to the tag centre (CCW/left positive)."""
    cx, cy, psi = est['cx'], est['cy'], est['psi']
    F, L = _fwd_left(psi)
    dx, dy = -cx, -cy
    return math.atan2(dx * L[0] + dy * L[1], dx * F[0] + dy * F[1])


# ---------------------------------------------------------------- G0 actions

def a0_init():
    return _zero(), True


def a1_check_sensors(odom, scan):
    ok = odom is not None and scan is not None and len(scan.ranges) > 0
    if ok:
        vals = (odom.x, odom.y, odom.yaw)
        ok = all(math.isfinite(v) for v in vals)
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
    return _cmd(0.0, _p(P, 'w_search'))


def _pallet_depth(samples):
    valid = sorted(s for s in samples if s is not None and 0.2 < s < 8.0)
    if not valid:
        return None
    m = valid[len(valid) // 2]
    inl = sorted(s for s in valid if abs(s - m) <= 0.2)
    if len(inl) >= 3:
        return inl[len(inl) // 2]
    return m


def a321_candidates(pallets, odom, blacklist, P):
    """Pallet-like candidates in the odometry frame (front-face point of each)."""
    cands = []
    if odom is None or not pallets:
        return cands
    smin = _p(P, 'score_min')
    wmin, wmax = _p(P, 'width_min'), _p(P, 'width_max')
    yaw = odom.yaw
    cam_x = odom.x + CAM_X * math.cos(yaw)
    cam_y = odom.y + CAM_X * math.sin(yaw)
    for det in pallets:
        if det.score < smin:
            continue
        d = _pallet_depth(det.depth_samples)
        if d is None:
            continue
        width = det.width_px * d / FX
        if not (wmin <= width <= wmax):
            continue
        beta = -math.atan2(det.u - IMG_W / 2.0, FX)   # bearing, CCW positive
        x = cam_x + d * math.cos(yaw + beta)
        y = cam_y + d * math.sin(yaw + beta)
        if any(math.hypot(x - bx, y - by) < CAND_RADIUS for (bx, by) in (blacklist or [])):
            continue
        cands.append({'x': x, 'y': y, 'd': d, 'score': det.score, 'beta': beta})
    return cands


def a322_confirm(cands, hist, t, P):
    k = int(_p(P, 'confirm_k'))
    win = _p(P, 'confirm_window_s')
    hist = [h for h in (hist or []) if t - h[0] <= 2.0 * win + 1.0]
    for c in cands:
        hist.append((t, c['x'], c['y'], c['score']))
    hist = hist[-200:]
    best, best_n, best_pts = None, 0, None
    for c in cands:
        pts = [h for h in hist
               if t - h[0] <= win + 1e-9 and math.hypot(h[1] - c['x'], h[2] - c['y']) <= CONFIRM_RADIUS]
        if len(pts) > best_n:
            best, best_n, best_pts = c, len(pts), pts
    if best is not None and best_n >= k:
        mx = sum(p[1] for p in best_pts) / best_n
        my = sum(p[2] for p in best_pts) / best_n
        track = {'x': mx, 'y': my, 't_seen': t, 'd': best['d'], 'score': best['score']}
        return hist, True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = TRACK_GATE_M
    best, best_d = None, gate
    for c in cands:
        d = math.hypot(c['x'] - track['x'], c['y'] - track['y'])
        if d < best_d:
            best, best_d = c, d
    if best is not None:
        track = dict(track)
        track['x'] = 0.5 * track['x'] + 0.5 * best['x']
        track['y'] = 0.5 * track['y'] + 0.5 * best['y']
        track['d'] = best['d']
        track['score'] = best['score']
        track['t_seen'] = t
    lost = (t - track['t_seen']) > _p(P, 'track_lost_s')
    return track, bool(lost)


def a34_approach_cmd(track, odom, P):
    if track is None or odom is None:
        return _zero(), False
    dx = track['x'] - odom.x
    dy = track['y'] - odom.y
    c, s = math.cos(odom.yaw), math.sin(odom.yaw)
    fwd = c * dx + s * dy
    left = -s * dx + c * dy
    dist = math.hypot(dx, dy)
    if dist <= _p(P, 'd_app'):
        return _zero(), True
    bearing = math.atan2(left, fwd)
    w = _p(P, 'k_bear') * bearing
    if abs(bearing) > math.radians(60.0):
        v = 0.0
    else:
        v = _p(P, 'v_app') * max(0.0, math.cos(bearing))
    return _cmd(v, w), False


def a35_exit(track, odom):
    ref = dict(track) if track is not None else None
    return _zero(), ref, track is not None


def a3e_fail():
    return _zero(), True


# ---------------------------------------------------------------- G4 / G5 pose

def _propagate(est, odom):
    """Dead-reckon the camera pose with odometry increments since last update."""
    if est is None or odom is None or est.get('odom') is None:
        return est
    ox, oy, oyaw = est['odom']
    dx, dy = odom.x - ox, odom.y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    d_fwd = c * dx + s * dy
    d_left = -s * dx + c * dy
    d_psi = _wrap(odom.yaw - oyaw)
    F, L = _fwd_left(est['psi'])
    est = dict(est)
    est['cx'] += d_fwd * F[0] + d_left * L[0]
    est['cy'] += d_fwd * F[1] + d_left * L[1]
    est['psi'] = _wrap(est['psi'] + d_psi)
    est['odom'] = (odom.x, odom.y, odom.yaw)
    return est


def _new_meas(det):
    """Camera pose (cx, cy, psi) implied by one tag detection.

    The tag (origin) is at camera-frame (x right, z forward); inverting the
    camera-to-tag transform with the measured heading psi = -yaw gives the
    camera position in the pallet frame.
    """
    psi = _wrap(-det.yaw)
    F, L = _fwd_left(psi)
    R = (-L[0], -L[1])
    cx = -(det.x * R[0] + det.z * F[0])
    cy = -(det.x * R[1] + det.z * F[1])
    return (cx, cy, psi)


def _consistent(a, b, P):
    return (math.hypot(a[0] - b[0], a[1] - b[1]) <= _p(P, 'gate_pos')
            and abs(_wrap(a[2] - b[2])) <= math.radians(_p(P, 'gate_yaw_deg')))


def a43_update_pose(est, odom, tags, t, P):
    est = _propagate(est, odom)
    if odom is None:
        return est
    det = None
    for d in (tags or []):
        if d.id == TAG_ID:
            det = d
            break
    if est is None:
        est = {'cx': 0.0, 'cy': 0.0, 'psi': 0.0, 'odom': (odom.x, odom.y, odom.yaw),
               't': t, 't_meas': None, 'n': 0, 'valid': False, 'cand': [], 'rej': 0}
    est['t'] = t
    if det is None:
        return est
    m = _new_meas(det)
    if not est['valid']:
        cand = est['cand']
        if cand and not _consistent(cand[-1], m, P):
            cand = []
        cand = (cand + [m])[-int(_p(P, 'init_meas')):]
        est['cand'] = cand
        if len(cand) >= int(_p(P, 'init_meas')):
            n = len(cand)
            est['cx'] = sum(c[0] for c in cand) / n
            est['cy'] = sum(c[1] for c in cand) / n
            est['psi'] = math.atan2(sum(math.sin(c[2]) for c in cand),
                                    sum(math.cos(c[2]) for c in cand))
            est['valid'] = True
            est['n'] = n
            est['t_meas'] = t
            est['cand'] = []
            est['rej'] = 0
        return est
    pred = (est['cx'], est['cy'], est['psi'])
    if _consistent(pred, m, P):
        g = _p(P, 'fuse_gain')
        est['cx'] = pred[0] + g * (m[0] - pred[0])
        est['cy'] = pred[1] + g * (m[1] - pred[1])
        est['psi'] = _wrap(pred[2] + g * _wrap(m[2] - pred[2]))
        est['t_meas'] = t
        est['n'] += 1
        est['rej'] = 0
    else:
        est['rej'] += 1
        if est['rej'] >= REJECT_LIMIT:
            est['valid'] = False
            est['cand'] = [m]
            est['rej'] = 0
    return est


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age_meas = (t - est['t_meas']) if est.get('t_meas') is not None else float('inf')
    tag_ok = age_meas <= TAG_FRESH_S
    pose_ready = bool(est['valid']) and est['n'] >= int(_p(P, 'init_meas')) and age_meas <= _p(P, 'pose_stale_s')
    near = est['cy'] < _p(P, 'near_dist')
    limit = _p(P, 'blind_max_s') if near else _p(P, 'pose_stale_s')
    pose_stale = (not est['valid']) or age_meas > limit
    return tag_ok, pose_ready, pose_stale


# ---------------------------------------------------------------- G4 actions

def a40_enter(n_loc, est, P):
    n_loc = int(n_loc) + 1
    retry_ok = n_loc <= int(_p(P, 'max_retries'))
    return _zero(), n_loc, retry_ok, est, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if odom is None:
        return _zero(), sweep
    sweep = dict(sweep) if sweep else {}
    if pallet_ref is not None:
        aim = math.atan2(pallet_ref['y'] - odom.y, pallet_ref['x'] - odom.x)
    else:
        if 'aim' not in sweep:
            sweep['aim'] = odom.yaw
        aim = sweep['aim']
    off = _wrap(odom.yaw - aim)
    lim = math.radians(_p(P, 'tag_sweep_deg'))
    if 'dir' not in sweep:
        sweep['dir'] = -1.0 if off > 0 else 1.0
    if off > lim:
        sweep['dir'] = -1.0
    elif off < -lim:
        sweep['dir'] = 1.0
    return _cmd(0.0, _p(P, 'w_tag') * sweep['dir']), sweep


def a42_hold():
    return _zero()


def a43_publish(est):
    return bool(est is not None and est.get('valid') and est.get('n', 0) >= 1)


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append((pallet_ref['x'], pallet_ref['y']))
    return _zero(), bl, True


# ---------------------------------------------------------------- G5 actions

def a50_enter():
    return _zero(), {}, 0, True, False, False, True


def _steer(est, v_max, P):
    """Heading-regulated forward motion toward the centre line (x = 0, heading 0)."""
    cx, cy, psi = est['cx'], est['cy'], est['psi']
    # pure pursuit toward a centre-line point LOOK m ahead of the pre-dock line
    if STEER == 'pp':
        psi_goal = _clamp(math.atan2(-cx, LOOK), -PSI_CLAMP, PSI_CLAMP)
    else:
        psi_goal = _clamp(-math.atan(LAT_GAIN * cx), -PSI_CLAMP, PSI_CLAMP)
    err = _wrap(psi_goal - psi)
    w = _p(P, 'k_head') * err
    v = v_max * max(0.3, math.cos(err))
    return v, w


def _lateral_ok(est, P):
    return (abs(est['cx']) <= _p(P, 'x_tol')
            and abs(est['psi']) <= math.radians(_p(P, 'head_tol_deg')))


def a51_plan(est, P):
    if est is None:
        return _zero(), False
    a = _tag_bearing(est)
    lim = math.radians(_p(P, 'fov_half_deg') - _p(P, 'fov_margin_deg'))
    room = CX_ROOM * max(0.0, est['cy'] - DOCK_DIST)   # lateral error correctable ahead
    feasible = abs(a) <= lim and abs(est['cx']) <= max(room, 0.05)
    return _zero(), bool(feasible)


def a52_reposition(est, mstate, P):
    """Differential-drive reposition: turn to face the centre line sideways,
    slide laterally toward it (cy unchanged), then turn back to face the pallet."""
    ms = dict(mstate or {})
    if est is None:
        return _zero(), ms, False
    cx, psi = est['cx'], est['psi']
    k = _p(P, 'k_head')
    side = -1.0 if cx >= 0 else 1.0           # heading that moves the camera toward x = 0
    psi_side = side * math.pi / 2
    ph = ms.get('ph', 'turn')
    if ph == 'turn':
        err = _wrap(psi_side - psi)
        if abs(err) <= math.radians(3.0):
            ph = 'slide'
        ms['ph'] = ph
        return _cmd(0.0, k * err), ms, False
    if ph == 'slide':
        err = _wrap(psi_side - psi)
        if cx * (-side) <= 0.08:              # reached (or passed) the centre line
            ms['ph'] = 'back'
            return _zero(), ms, False
        return _cmd(SLIDE_V, k * err), ms, False
    err = _wrap(0.0 - psi)
    if abs(err) <= math.radians(3.0):
        return _zero(), {}, True
    return _cmd(0.0, k * err), ms, False


def a53_converge(est, P):
    if est is None:
        return _zero(), False, True
    e = est['cy'] - _p(P, 'y_pre')
    if est['cy'] < DOCK_DIST:           # overshot toward the pocket face
        return _zero(), False, True
    if _lateral_ok(est, P) and e <= 0.15:
        return _zero(), True, False     # pre-dock reached and aligned
    v, w = _steer(est, _p(P, 'v_conv'), P)
    return _cmd(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return _zero(), mstate
    a = _tag_bearing(est)
    return _cmd(0.0, _p(P, 'k_head') * a), mstate


def a55_final_approach(est, P):
    if est is None:
        return _zero(), False
    if est['cy'] - DOCK_DIST <= 0.02:
        return _zero(), True
    v, w = _steer(est, _p(P, 'v_final'), P)
    return _cmd(v, w), False


def a56_verify(est, mstate, t, P):
    ms = dict(mstate or {})
    if ms.get('v0') is None:
        ms = {'v0': t, 'ok_since': None}
    if est is None:
        return _zero(), {}, False, True, False
    tag_ok = est.get('t_meas') is not None and (t - est['t_meas']) <= TAG_FRESH_S
    cx, cy, psi = est['cx'], est['cy'], est['psi']
    e_dist = abs(cy - DOCK_DIST)
    in_tol = (abs(cx) <= _p(P, 'verify_lat') and abs(psi) <= math.radians(_p(P, 'verify_head_deg'))
              and e_dist <= _p(P, 'verify_dist'))
    out_bad = (abs(cx) > VERIFY_BAD_LAT or abs(psi) > math.radians(VERIFY_BAD_HEAD_DEG)
               or e_dist > VERIFY_BAD_DIST)
    verified = False
    bad = False
    if tag_ok and in_tol:
        if ms['ok_since'] is None:
            ms['ok_since'] = t
        if t - ms['ok_since'] >= _p(P, 'verify_s') - 1e-9:
            verified = True
    else:
        ms['ok_since'] = None
        if tag_ok and out_bad:
            bad = True
    if not verified and not bad and (t - ms['v0']) > _p(P, 'verify_s') + VERIFY_TIMEOUT_EXTRA:
        bad = True
    if verified or bad:
        return _zero(), {}, verified, (bad and not verified), verified
    return _zero(), ms, False, False, False


def a57_backoff(odom, mstate, n_bo, P):
    ms = dict(mstate or {})
    n_bo = int(n_bo)
    if odom is None:
        return _zero(), ms, n_bo, n_bo < int(_p(P, 'max_backoff')), True
    if ms.get('bo0') is None:
        ms['bo0'] = (odom.x, odom.y)
        n_bo += 1
    dist = math.hypot(odom.x - ms['bo0'][0], odom.y - ms['bo0'][1])
    bo_ok = n_bo < int(_p(P, 'max_backoff'))
    if dist >= _p(P, 'backoff_dist'):
        return _zero(), {}, n_bo, bo_ok, True
    return _cmd(BACKOFF_V, 0.0), ms, n_bo, bo_ok, False


def a5e_fail():
    return _zero(), True
