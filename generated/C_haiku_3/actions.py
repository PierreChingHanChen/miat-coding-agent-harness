"""Datapath Action functions for the AGV pallet-pocket docking controller.

Implements every Action listed in ACTION_CONTRACTS.md. No module-level mutable
state: everything an action remembers is passed in and returned explicitly.
"""
import math

from sim.api import Command, CAM_X, TAG_ID, IMG_W, FX

_PI = math.pi
_2PI = 2.0 * math.pi
_DEG = math.pi / 180.0


# ---------------------------------------------------------------- helpers
def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + _PI, _2PI)
    if a <= 0.0:
        a += _2PI
    return a - _PI


def _sat(x, m):
    return max(-m, min(m, x))


def _cam_pos(px, py, yaw):
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def _inplace(e, m):
    """In-place rotation rule: proportional above 0.12 rad, else a fixed-rate creep."""
    if abs(e) > 0.12:
        return _sat(1.8 * e, m)
    return math.copysign(0.15, e)


# ------------------------------------------------------------ A0 / A1 / A2
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


# ------------------------------------------------------------------ A3
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
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam_pos(odom.x, odom.y, odom.yaw)
    for det in pallets:
        if det.score < P['score_min']:
            continue
        d, ok = a332_pallet_distance(det.depth_samples)
        if not ok:
            continue
        u = a331_bbox_center(det)
        b = a333_pallet_bearing(u)
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
    window = P['confirm_window_s']
    tol = P['confirm_tol_deg'] * _DEG
    hist = [h for h in hist if t - h['t'] <= window]
    for c in cands:
        hist.append({'t': t, 'c': c})
    best = None
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        count = len(stamps)
        if best is None or count > best[0] or (count == best[0] and c['score'] > best[1]['score']):
            best = (count, c)
    if best is not None and best[0] >= P['confirm_k']:
        c = best[1]
        return hist, True, {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = P['track_gate_deg'] * _DEG
    best = None
    for c in cands:
        dba = abs(_wrap(c['ba'] - track['ba']))
        if dba <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or dba < best[0]:
                best = (dba, c)
    new = dict(track)
    if best is not None:
        c = best[1]
        new['ba'] = _wrap(track['ba'] + 0.5 * _wrap(c['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_pos(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = _wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        w = _sat(-1.5 * b, 0.4)
        done = abs(b) <= P['app_bear_tol_deg'] * _DEG
        if done:
            w = 0.0
        return Command(0.0, w), done
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < 10.0 * _DEG else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ------------------------------------------------------------------ A4
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = P['tag_sweep_deg'] * _DEG
        if pallet_ref is None:
            ref = odom.yaw
        else:
            cx, cy = _cam_pos(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        targets = [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = list(sweep['targets'])
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < 3.0 * _DEG:
        cmd = Command()
        i = (i + 1) % len(targets)
    else:
        cmd = Command(0.0, _sat(2.0 * err, P['w_tag']))
    return cmd, {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - _PI / 2.0)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return (cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw)


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in tags if tg.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        px, py, yaw = meas[0]
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment
    ox, oy, oyaw = est['o']
    dx = odom.x - ox
    dy = odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    psi = est['yaw']
    px = est['px'] + f * math.cos(psi) - l * math.sin(psi)
    py = est['py'] + f * math.sin(psi) + l * math.cos(psi)
    yaw = _wrap(psi + _wrap(odom.yaw - oyaw))
    o = (odom.x, odom.y, odom.yaw)
    n = est['n']
    rej = est['rej']
    t_meas = est['t_meas']

    if meas:
        zx, zy, zyaw = meas[0]
        ex = zx - px
        ey = zy - py
        eyaw = _wrap(zyaw - yaw)
        cam_d = math.hypot(*_cam_pos(zx, zy, zyaw))
        pos_ok = math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cam_d
        yaw_ok = abs(eyaw) <= P['gate_yaw_deg'] * _DEG
        if pos_ok and yaw_ok:
            k = 0.5 if n < P['init_meas'] else P['fuse_gain']
            px += k * ex
            py += k * ey
            yaw = _wrap(yaw + k * eyaw)
            t_meas = t
            n += 1
            rej = 0
        else:
            rej += 1
            if rej >= 5:
                return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
                        'o': o}
    return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t_meas, 'n': n, 'rej': rej, 'o': o}


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam_d = math.hypot(*_cam_pos(est['px'], est['py'], est['yaw']))
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


# ------------------------------------------------------------------ A5
def a50_enter():
    return Command(), None, 0, True, False, False, True


def _beta_max(P):
    return (P['fov_half_deg'] - P['fov_margin_deg']) * _DEG


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
    return Command(), gamma <= _beta_max(P) - 2.0 * _DEG


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    if mstate is None or mstate.get('own') != '52':
        px, py = est['px'], est['py']
        yp = P['y_pre']
        yw = max(py, yp + P['repos_extra'])
        sgn = (px > 0) - (px < 0)
        xw = sgn * (yw - yp) * math.tan(_beta_max(P) - 8.0 * _DEG)
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}

    px, py, yaw = est['px'], est['py'], est['yaw']
    W = mstate['W']
    dist = math.hypot(W[0] - px, W[1] - py)
    e_w = _wrap(math.atan2(W[1] - py, W[0] - px) - yaw)

    # turn1 -> drive -> turn2; a transition falls through to the next phase's action
    ph = mstate['ph']
    if ph == 'turn1':
        if dist <= 0.06:
            ph = 'turn2'
        elif abs(e_w) < 3.0 * _DEG:
            ph = 'drive'
        else:
            return Command(0.0, _inplace(e_w, 0.5)), mstate, False
    if ph == 'drive':
        if dist <= 0.06:
            ph = 'turn2'
        else:
            v = 0.2 * max(0.0, math.cos(e_w))
            w = _sat(1.5 * e_w, 0.5)
            return Command(v, w), {'own': '52', 'ph': 'drive', 'W': W}, False

    # turn2: face the tag from the camera
    mstate = {'own': '52', 'ph': 'turn2', 'W': W}
    cx, cy = _cam_pos(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < 3.0 * _DEG:
        return Command(), None, True
    return Command(0.0, _inplace(e, 0.5)), mstate, False


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
    cx, cy = _cam_pos(px, py, yaw)
    cd = math.atan2(-cy, -cx)
    beta = _beta_max(P)
    dev = _sat(_wrap(hd - cd), beta)
    hd = _wrap(cd + dev)
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam_pos(est['px'], est['py'], est['yaw'])
    e = _wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= 4.0 * _DEG:
        return Command(), None
    return Command(0.0, _sat(_inplace(e, 0.4), 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -_PI / 2.0 - _sat(math.atan(P['k_lat'] * px), 30.0 * _DEG)
        e = _wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-_PI / 2.0 - yaw)
    if abs(e) <= P['head_tol_deg'] * _DEG:
        return Command(), True
    return Command(0.0, _inplace(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        return Command(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s'] - 1e-6:
        return Command(), mstate, False, False, False
    if est is None:
        return Command(), None, False, True, False
    ok = (abs(est['px']) <= P['verify_lat']
          and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(_wrap(est['yaw'] + _PI / 2.0)) <= P['verify_head_deg'] * _DEG)
    return Command(), None, ok, not ok, ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        n_bo = n_bo + 1
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if travelled >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
