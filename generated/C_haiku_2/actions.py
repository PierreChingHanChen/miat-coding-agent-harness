"""Datapath Action functions for the AGV pallet-pocket docking controller.

Stateless: everything that must persist between ticks is passed in and returned.
Only math and sim.api are used.
"""
import math
from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID

_PI = math.pi


# ---------------------------------------------------------------- helpers
def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _sign(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)


def _cam(px, py, yaw):
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


def _inplace(e, wmax):
    """In-place rotation rule shared by the manoeuvres (w is + to the left)."""
    if abs(e) > 0.12:
        return _sat(1.8 * e, wmax)
    return math.copysign(0.15, e)


def _pose_from_tag(tag):
    yaw = _wrap(tag.yaw - _PI / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return (cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw)


# ---------------------------------------------------------------- A0 / A1 / A2 / A6 / A9
def a0_init():
    return Command(), True


def a1_check_sensors(odom, scan):
    return odom is not None and scan is not None


def a2_navigate():
    return True


def a6_docked():
    return Command()


def a9_abort():
    return Command()


# ---------------------------------------------------------------- A3 Recognise pallet and approach
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [s for s in depth_samples if s > 0.05]
    if len(valid) < 5:
        return 0.0, False
    m = _median(valid)
    kept = [s for s in valid if abs(s - m) <= 0.3]
    if len(kept) < 3:
        return 0.0, False
    return _median(kept), True


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam(odom.x, odom.y, odom.yaw)
    for det in pallets:
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
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bx, pos[1] - by) < 0.8 for (bx, by) in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    win = P['confirm_window_s']
    tol = math.radians(P['confirm_tol_deg'])
    hist = [h for h in hist if t - h['t'] <= win]
    hist = hist + [{'t': t, 'c': c} for c in cands]
    best = None
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best is None or key > best[0]:
            best = (key, c)
    if best is None or best[0][0] < P['confirm_k']:
        return hist, False, None
    c = best[1]
    return hist, True, {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(P['track_gate_deg'])
    best = None
    for c in cands:
        dang = abs(_wrap(c['ba'] - track['ba']))
        if dang <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or dang < best[0]:
                best = (dang, c)
    if best is None:
        lost = (t - track['t']) > P['track_lost_s']
        return dict(track), lost
    c = best[1]
    ba = _wrap(track['ba'] + 0.5 * _wrap(c['ba'] - track['ba']))
    pos = ((track['pos'][0] + c['pos'][0]) / 2, (track['pos'][1] + c['pos'][1]) / 2)
    new = {'ba': ba, 'd': c['d'], 'pos': pos, 't': t}
    return new, (t - new['t']) > P['track_lost_s']


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = _wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        done = abs(b) <= math.radians(P['app_bear_tol_deg'])
        if done:
            return Command(0.0, 0.0), True
        return Command(0.0, _sat(-1.5 * b, 0.4)), False
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ---------------------------------------------------------------- A4 Localise pocket centre (tag)
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    A = math.radians(P['tag_sweep_deg'])
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam(odom.x, odom.y, odom.yaw)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        targets = [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = sweep['targets']
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        return Command(), {'targets': list(targets), 'i': (i + 1) % len(targets)}
    w = _sat(2.0 * err, P['w_tag'])
    return Command(0.0, w), {'targets': list(targets), 'i': i}


def a42_tag_to_pose(tag):
    return _pose_from_tag(tag)


def _new_est(z, odom, t):
    return {'px': z[0], 'py': z[1], 'yaw': z[2], 't_meas': t, 'n': 1, 'rej': 0,
            'o': (odom.x, odom.y, odom.yaw)}


def a43_update_pose(est, odom, tags, t, P):
    z = None
    for tg in tags:
        if tg.id == TAG_ID:
            z = _pose_from_tag(tg)
            break
    if est is None:
        return None if z is None else _new_est(z, odom, t)

    # predict with odometry increment
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    f = c * dx + s * dy          # forward in robot frame at previous heading
    l = -s * dx + c * dy         # left
    yaw = est['yaw']
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    px = est['px'] + f * cy_ - l * sy_
    py = est['py'] + f * sy_ + l * cy_
    nyaw = _wrap(yaw + _wrap(odom.yaw - oyaw))
    new = {'px': px, 'py': py, 'yaw': nyaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'], 'o': (odom.x, odom.y, odom.yaw)}
    if z is None:
        return new

    # update with the first measurement
    ex, ey = z[0] - px, z[1] - py
    eyaw = _wrap(z[2] - nyaw)
    cam_d = math.hypot(z[0] + CAM_X * math.cos(z[2]), z[1] + CAM_X * math.sin(z[2]))
    gate_p = P['gate_pos'] + 0.1 * cam_d
    gate_y = math.radians(P['gate_yaw_deg'])
    if math.hypot(ex, ey) <= gate_p and abs(eyaw) <= gate_y:
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = _wrap(nyaw + k * eyaw)
        new['t_meas'] = t
        new['n'] = est['n'] + 1
        new['rej'] = 0
    else:
        rej = est['rej'] + 1
        if rej >= 5:
            re = _new_est(z, odom, t)
            new.update(px=re['px'], py=re['py'], yaw=re['yaw'], t_meas=t, n=1, rej=0)
        else:
            new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam_d = math.hypot(est['px'] + CAM_X * math.cos(est['yaw']),
                       est['py'] + CAM_X * math.sin(est['yaw']))
    stale = (age > P['pose_stale_s'] and cam_d > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, pose_ready, stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist)
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ---------------------------------------------------------------- A5 Guide into docking position
def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return Command(), False
    px, py = est['px'], est['py']
    yp = P['y_pre']
    if abs(px) <= 0.05 and abs(py - yp) <= 0.15:
        return Command(), True
    if py <= yp + 0.15:
        return Command(), False
    gamma = math.atan2(abs(px), py - yp)
    feasible = gamma <= math.radians(P['fov_half_deg'] - P['fov_margin_deg'] - 2.0)
    return Command(), feasible


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), None, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    bmax = _beta_max(P)
    if mstate is None or mstate.get('own') != '52':
        yw = max(py, yp + P['repos_extra'])
        xw = _sign(px) * (yw - yp) * math.tan(bmax - math.radians(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    wx, wy = mstate['W']
    ph = mstate['ph']

    for _ in range(3):  # bounded phase chaining within one tick
        if ph == 'turn1':
            dist = math.hypot(wx - px, wy - py)
            e = _wrap(math.atan2(wy - py, wx - px) - yaw)
            if dist <= 0.06:
                ph = 'turn2'
                continue
            if abs(e) < math.radians(3.0):
                ph = 'drive'
                continue
            cmd = Command(0.0, _inplace(e, 0.5))
            break
        if ph == 'drive':
            dist = math.hypot(wx - px, wy - py)
            if dist <= 0.06:
                ph = 'turn2'
                continue
            e = _wrap(math.atan2(wy - py, wx - px) - yaw)
            cmd = Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5))
            break
        # turn2: face the tag from the current position
        cx, cy = _cam(px, py, yaw)
        e = _wrap(math.atan2(-cy, -cx) - yaw)
        if abs(e) < math.radians(3.0):
            return Command(), None, True
        cmd = Command(0.0, _inplace(e, 0.5))
        break
    else:
        cmd = Command()
    return cmd, {'own': '52', 'ph': ph, 'W': (wx, wy)}, False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return Command(), True, False
    if py < yp - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = yp if py > yp else py - 0.3
    hd = math.atan2(ty - py, -px)
    cx, cy = _cam(px, py, yaw)
    dirc = math.atan2(-cy, -cx)
    bmax = _beta_max(P)
    delta = max(-bmax, min(bmax, _wrap(hd - dirc)))
    hd = _wrap(dirc + delta)
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _cam(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return Command(0.0, _sat(1.8 * e, 0.4) if abs(e) > 0.12 else math.copysign(0.15, e)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -_PI / 2 - _sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = _wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-_PI / 2 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _inplace(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        return Command(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + _PI / 2)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        n_bo = n_bo + 1
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
    bo_ok = n_bo < P['max_backoff']
    dist = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if dist >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
