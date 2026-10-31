"""Action functions for the AGV pallet-pocket docking controller (MIAT A3-A5).

Implements every function listed in ACTION_CONTRACTS.md. Stateless: anything an
action remembers between ticks is passed in and returned explicitly.
"""
import math

from sim.api import Command, CAM_X, IMG_W, FX, TAG_ID

_HALF_PI = math.pi / 2.0
_TWO_PI = 2.0 * math.pi
_TAG_OK_AGE = 0.35          # [s] pose measurement age for tag_ok
_BLACKLIST_R = 0.8          # [m] candidate-to-blacklist exclusion radius
_CONFIRM_DIST = 0.4         # [m] distance tolerance for k-of-window confirmation
_TRACK_GATE_DIST = 0.6      # [m] distance gate for track association
_INPLACE_DEAD = 0.12        # [rad] in-place rotation rule threshold
_INPLACE_KICK = 0.15        # [rad/s] in-place rotation minimum speed


# ---------------------------------------------------------------- helpers ---

def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, _TWO_PI)
    if a <= 0.0:
        a += _TWO_PI
    return a - math.pi


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _cam_of(x, y, yaw):
    """Camera optical centre for a base_link pose (x, y, yaw)."""
    return (x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw))


def _inplace(e, lim):
    """In-place rotation rule (A52): proportional, with a minimum kick near zero."""
    if abs(e) > _INPLACE_DEAD:
        return _sat(1.8 * e, lim)
    return math.copysign(_INPLACE_KICK, e)


def _init_est(z, odom, t):
    return {'px': z[0], 'py': z[1], 'yaw': z[2], 't_meas': t, 'n': 1, 'rej': 0,
            'o': (odom.x, odom.y, odom.yaw)}


# ------------------------------------------------- G0 / A0 / A1 / A2 / A6 ---

def a0_init():
    return (Command(), True)


def a1_check_sensors(odom, scan):
    return odom is not None and scan is not None


def a2_navigate():
    return True


def a6_docked():
    return Command()


def a9_abort():
    return Command()


# --------------------------------------------------------- A3 (G3) ----------

def a30_enter():
    return (Command(), None, [], False, False, True)


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [s for s in depth_samples if math.isfinite(s) and s > 0.05]
    if len(valid) < 5:
        return (0.0, False)
    m = _median(valid)
    kept = [s for s in valid if abs(s - m) <= 0.3]
    if len(kept) < 3:
        return (0.0, False)
    return (_median(kept), True)


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam_of(odom.x, odom.y, odom.yaw)
    for det in pallets or []:
        if det.score < P['score_min']:
            continue
        d, ok = a332_pallet_distance(det.depth_samples)
        if not ok:
            continue
        b = a333_pallet_bearing(a331_bbox_center(det))
        W = det.width_px * d / FX
        if not (P['width_min'] <= W <= P['width_max']):
            continue
        ba = _wrap(odom.yaw - b)
        px = cx + d * math.cos(ba)
        py = cy + d * math.sin(ba)
        blocked = False
        for bp in blacklist or []:
            if math.hypot(px - bp[0], py - bp[1]) <= _BLACKLIST_R:
                blocked = True
                break
        if blocked:
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': (px, py)})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    tol = math.radians(P['confirm_tol_deg'])
    kept = [h for h in (hist or []) if t - h['t'] <= window + 1e-9]
    new_hist = kept + [{'t': t, 'c': c} for c in cands]
    best = None
    for c in cands:
        stamps = set()
        for h in new_hist:
            cc = h['c']
            if abs(_wrap(cc['ba'] - c['ba'])) <= tol and abs(cc['d'] - c['d']) <= _CONFIRM_DIST:
                stamps.add(h['t'])
        count = len(stamps)
        if best is None or count > best[0] or (count == best[0] and c['score'] > best[1]['score']):
            best = (count, c)
    if best is not None and best[0] >= P['confirm_k']:
        c = best[1]
        return (new_hist, True, {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t})
    return (new_hist, False, None)


def a323_track(cands, track, t, P):
    if track is None:
        return (None, True)
    gate = math.radians(P['track_gate_deg'])
    best = None
    best_diff = None
    for c in cands or []:
        dba = _wrap(c['ba'] - track['ba'])
        if abs(dba) <= gate and abs(c['d'] - track['d']) < _TRACK_GATE_DIST:
            if best is None or abs(dba) < best_diff:
                best, best_diff = c, abs(dba)
    new = dict(track)
    if best is not None:
        new['ba'] = _wrap(track['ba'] + 0.5 * _wrap(best['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2.0,
                      (track['pos'][1] + best['pos'][1]) / 2.0)
        new['d'] = best['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return (new, bool(lost))


def a34_approach_cmd(track, odom, P):
    if track is None:
        return (Command(), False)
    cx, cy = _cam_of(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    dx, dy = px - cx, py - cy
    d = math.hypot(dx, dy)
    b = _wrap(odom.yaw - math.atan2(dy, dx))
    if d <= P['d_app']:
        if abs(b) <= math.radians(P['app_bear_tol_deg']):
            return (Command(0.0, 0.0), True)
        return (Command(0.0, _sat(-1.5 * b, 0.4)), False)
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
    return (Command(v, w), False)


def a35_exit(track, odom):
    ref = None if track is None else {'ba': track['ba'], 'pos': track['pos']}
    return (Command(), ref, True)


def a3e_fail():
    return (Command(), True)


# --------------------------------------------------------- A4 (G4) ----------

def a40_enter(n_loc, est, P):
    n = n_loc + 1
    retry_ok = n <= P['max_retries']
    return (Command(), n, retry_ok, None, None, False, False, True)


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam_of(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        else:
            ref = odom.yaw
        A = math.radians(P['tag_sweep_deg'])
        targets = [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = list(sweep['targets'])
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        return (Command(), {'targets': targets, 'i': (i + 1) % len(targets)})
    return (Command(0.0, _sat(2.0 * err, P['w_tag'])), {'targets': targets, 'i': i})


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - _HALF_PI)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return (cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw)


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in (tags or []) if tg.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        return _init_est(meas[0], odom, t)

    # Predict with odometry increment since the last odometry snapshot.
    x0, y0, yaw0 = est['o']
    dx = odom.x - x0
    dy = odom.y - y0
    dyaw = _wrap(odom.yaw - yaw0)
    f = dx * math.cos(yaw0) + dy * math.sin(yaw0)     # forward in robot frame
    l = -dx * math.sin(yaw0) + dy * math.cos(yaw0)    # left in robot frame
    ye = est['yaw']
    px = est['px'] + f * math.cos(ye) - l * math.sin(ye)
    py = est['py'] + f * math.sin(ye) + l * math.cos(ye)
    yaw = _wrap(ye + dyaw)
    new = dict(est)
    new.update(px=px, py=py, yaw=yaw, o=(odom.x, odom.y, odom.yaw))
    if not meas:
        return new

    zx, zy, zyaw = meas[0]
    ex = zx - px
    ey = zy - py
    eyaw = _wrap(zyaw - yaw)
    cam_d = math.hypot(px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))
    gate_ok = (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cam_d
               and abs(eyaw) <= math.radians(P['gate_yaw_deg']))
    if gate_ok:
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        new.update(px=px + k * ex, py=py + k * ey, yaw=_wrap(yaw + k * eyaw),
                   t_meas=t, n=est['n'] + 1, rej=0)
        return new

    rej = est['rej'] + 1
    if rej >= 5:
        return _init_est(meas[0], odom, t)
    new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return (False, False, True)
    age = t - est['t_meas']
    tag_ok = age < _TAG_OK_AGE
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam_d = math.hypot(est['px'] + CAM_X * math.cos(est['yaw']),
                       est['py'] + CAM_X * math.sin(est['yaw']))
    stale = ((age > P['pose_stale_s'] and cam_d > P['near_dist'])
             or age > P['blind_max_s'])
    return (bool(tag_ok), bool(pose_ready), bool(stale))


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return (Command(), bl, True)


# --------------------------------------------------------- A5 (G5) ----------

def a50_enter():
    return (Command(), None, 0, True, False, False, True)


def a51_plan(est, P):
    if est is None:
        return (Command(), False)
    px, py = est['px'], est['py']
    yp = P['y_pre']
    if abs(px) <= 0.05 and abs(py - yp) <= 0.15:
        feasible = True
    elif py <= yp + 0.15:
        feasible = False
    else:
        gamma = math.atan2(abs(px), py - yp)
        beta_deg = P['fov_half_deg'] - P['fov_margin_deg']
        feasible = gamma <= math.radians(beta_deg - 2.0)
    return (Command(), bool(feasible))


def a52_reposition(est, mstate, P):
    if est is None:
        return (Command(), None, False)
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']

    if mstate is None or mstate.get('owner') != '52':
        yw = max(py, yp + P['repos_extra'])
        beta_deg = P['fov_half_deg'] - P['fov_margin_deg']
        sgn = (px > 0) - (px < 0)
        xw = sgn * (yw - yp) * math.tan(math.radians(beta_deg - 8.0))
        mstate = {'owner': '52', 'phase': 'turn1', 'W': (xw, yw)}

    xw, yw = mstate['W']
    phase = mstate['phase']
    dist = math.hypot(xw - px, yw - py)
    e = _wrap(math.atan2(yw - py, xw - px) - yaw)

    if phase == 'turn1':
        if dist <= 0.06:
            phase = 'turn2'
        elif abs(e) < math.radians(3.0):
            phase = 'drive'
        else:
            return (Command(0.0, _inplace(e, 0.5)),
                    {'owner': '52', 'phase': 'turn1', 'W': (xw, yw)}, False)

    if phase == 'drive':
        if dist <= 0.06:
            phase = 'turn2'
        else:
            v = 0.2 * max(0.0, math.cos(e))
            w = _sat(1.5 * e, 0.5)
            return (Command(v, w),
                    {'owner': '52', 'phase': 'drive', 'W': (xw, yw)}, False)

    # turn2: face the camera toward the tag (origin of pallet frame)
    cx, cy = _cam_of(px, py, yaw)
    e2 = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e2) < math.radians(3.0):
        return (Command(), None, True)
    return (Command(0.0, _inplace(e2, 0.5)),
            {'owner': '52', 'phase': 'turn2', 'W': (xw, yw)}, False)


def a53_converge(est, P):
    if est is None:
        return (Command(), False, False)
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return (Command(), True, False)
    if py < yp - 0.25 and abs(px) > 0.10:
        return (Command(), False, True)

    ty = yp if py > yp else py - 0.3
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam_of(px, py, yaw)
    dir_ct = math.atan2(-cy, -cx)                     # camera -> tag direction
    beta = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    hd = _wrap(dir_ct + _sat(_wrap(hd - dir_ct), beta))
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return (Command(v, w), False, False)


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return (Command(0.0, 0.2), None)
    cx, cy = _cam_of(est['px'], est['py'], est['yaw'])
    e = _wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= math.radians(4.0):
        return (Command(), None)
    return (Command(0.0, _inplace(e, 0.4)), None)


def a55_final_approach(est, P):
    if est is None:
        return (Command(), False)
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        desired = -_HALF_PI - _sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = _wrap(desired - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return (Command(v, w), False)
    e = _wrap(-_HALF_PI - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return (Command(), True)
    return (Command(0.0, _inplace(e, 0.3)), False)


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('owner') != '56':
        mstate = {'owner': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s'] - 1e-9:
        return (Command(), mstate, False, False, False)
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + _HALF_PI)) <= math.radians(P['verify_head_deg']))
    ok = bool(ok)
    return (Command(), None, ok, not ok, ok)


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('owner') != '57':
        n_bo = n_bo + 1
        mstate = {'owner': '57', 'sx': odom.x, 'sy': odom.y}
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - mstate['sx'], odom.y - mstate['sy'])
    if travelled >= P['backoff_dist']:
        return (Command(), None, n_bo, bo_ok, True)
    return (Command(-0.15, 0.0), mstate, n_bo, bo_ok, False)


def a5e_fail():
    return (Command(), True)
