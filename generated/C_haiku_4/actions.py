"""Datapath Action functions of the AGV pallet-pocket docking controller (A0..A5).

Stateless: everything an action remembers between ticks is passed in and returned.
Only math and sim.api are used.
"""
import math
from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID


# ---------------------------------------------------------------- helpers
def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def sat(x, lim):
    return max(-lim, min(lim, x))


def _rad(deg):
    return math.radians(deg)


def _beta(P):
    return _rad(P['fov_half_deg'] - P['fov_margin_deg'])


def _cam(px, py, yaw):
    """Camera position in the pallet frame for a base_link pose."""
    return px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    if n % 2:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _inplace(e, wmax):
    """In-place rotation rule (heading error e, + = left/CCW)."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(min(0.15, wmax), e)


def _stop():
    return Command()


# ------------------------------------------------------- A0 / A1 / A2 / A6 / A9
def a0_init():
    return _stop(), True


def a1_check_sensors(odom, scan):
    return odom is not None and scan is not None


def a2_navigate():
    return True


def a6_docked():
    return _stop()


def a9_abort():
    return _stop()


# ------------------------------------------------------------------- A3
def a30_enter():
    return _stop(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [s for s in depth_samples
             if s is not None and math.isfinite(s) and s > 0.05]
    if len(valid) < 5:
        return 0.0, False
    m = _median(valid)
    kept = [s for s in valid if abs(s - m) <= 0.3]
    if len(kept) < 3:
        return 0.0, False
    return _median(kept), True


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cam_x = odom.x + CAM_X * math.cos(odom.yaw)
    cam_y = odom.y + CAM_X * math.sin(odom.yaw)
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
        ba = wrap(odom.yaw - b)
        pos = (cam_x + d * math.cos(ba), cam_y + d * math.sin(ba))
        if any(math.hypot(pos[0] - bx, pos[1] - by) <= 0.8 for (bx, by) in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    win = P['confirm_window_s']
    hist = [h for h in hist if t - h['t'] <= win]
    hist = hist + [{'t': t, 'c': c} for c in cands]
    if not cands:
        return hist, False, None
    tol = _rad(P['confirm_tol_deg'])
    best = None
    for c in cands:
        stamps = set()
        for h in hist:
            cc = h['c']
            if abs(wrap(cc['ba'] - c['ba'])) <= tol and abs(cc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best is None or key > best[0]:
            best = (key, c)
    key, c = best
    if key[0] >= P['confirm_k']:
        track = {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}
        return hist, True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = _rad(P['track_gate_deg'])
    best = None
    for c in cands:
        dba = abs(wrap(c['ba'] - track['ba']))
        if dba <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or dba < best[0]:
                best = (dba, c)
    new = dict(track)
    if best is not None:
        c = best[1]
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(c['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return _stop(), False
    cx, cy = _cam(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        w = sat(-1.5 * b, 0.4)
        done = abs(b) <= _rad(P['app_bear_tol_deg'])
        if done:
            w = 0.0
        return Command(0.0, w), done
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < _rad(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    pallet_ref = None
    if track is not None:
        pallet_ref = {'ba': track['ba'], 'pos': track['pos']}
    return _stop(), pallet_ref, True


def a3e_fail():
    return _stop(), True


# ------------------------------------------------------------------- A4
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return _stop(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = _rad(P['tag_sweep_deg'])
        if pallet_ref is not None:
            cx, cy = _cam(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        else:
            ref = odom.yaw
        targets = [wrap(ref), wrap(ref + A), wrap(ref - A), wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = list(sweep['targets'])
    i = sweep['i'] % len(targets)
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < _rad(3.0):
        cmd = _stop()
        i = (i + 1) % len(targets)
    else:
        cmd = Command(0.0, sat(2.0 * err, P['w_tag']))
    return cmd, {'targets': targets, 'i': i}


def _tag_cam(tag):
    """Camera position in the pallet frame from a tag detection (+ pose yaw)."""
    yaw = wrap(tag.yaw - math.pi / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return cx, cy, yaw


def a42_tag_to_pose(tag):
    cx, cy, yaw = _tag_cam(tag)
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags:
        if tg.id == TAG_ID:
            meas = a42_tag_to_pose(tg)
            meas_cam_dist = math.hypot(*_tag_cam(tg)[:2])
            break

    if est is None:
        if meas is None:
            return None
        px, py, yaw = meas
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = math.cos(oyaw) * dx + math.sin(oyaw) * dy
    l = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
    yaw = est['yaw']
    px = est['px'] + f * math.cos(yaw) - l * math.sin(yaw)
    py = est['py'] + f * math.sin(yaw) + l * math.cos(yaw)
    yaw = wrap(yaw + wrap(odom.yaw - oyaw))
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'], 'o': (odom.x, odom.y, odom.yaw)}

    if meas is None:
        return new

    mx, my, myaw = meas
    ex, ey = mx - px, my - py
    eyaw = wrap(myaw - yaw)
    gate = P['gate_pos'] + 0.1 * meas_cam_dist
    if math.hypot(ex, ey) <= gate and abs(eyaw) <= _rad(P['gate_yaw_deg']):
        k = 0.5 if new['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = wrap(yaw + k * eyaw)
        new['t_meas'] = t
        new['n'] = new['n'] + 1
        new['rej'] = 0
        return new

    new['rej'] = est['rej'] + 1
    if new['rej'] >= 5:
        px, py, yaw = meas
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cx, cy = _cam(est['px'], est['py'], est['yaw'])
    cam_dist = math.hypot(cx, cy)
    pose_stale = ((age > P['pose_stale_s'] and cam_dist > P['near_dist'])
                  or age > P['blind_max_s'])
    return tag_ok, pose_ready, pose_stale


def a42_hold():
    return _stop()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist)
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return _stop(), bl, True


# ------------------------------------------------------------------- A5
def a50_enter():
    return _stop(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return _stop(), False
    ypre = P['y_pre']
    px, py = est['px'], est['py']
    if abs(px) <= 0.05 and abs(py - ypre) <= 0.15:
        return _stop(), True
    if py <= ypre + 0.15:
        return _stop(), False
    gamma = math.atan2(abs(px), py - ypre)
    return _stop(), gamma <= _beta(P) - _rad(2.0)


def a52_reposition(est, mstate, P):
    beta = _beta(P)
    ypre = P['y_pre']
    px, py, yaw = est['px'], est['py'], est['yaw']
    if mstate is None or mstate.get('own') != '52':
        yw = max(py, ypre + P['repos_extra'])
        xw = math.copysign(1.0, px) * (yw - ypre) * math.tan(beta - _rad(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    ph = mstate['ph']
    W = mstate['W']
    for _ in range(4):
        if ph == 'turn1':
            if math.hypot(W[0] - px, W[1] - py) <= 0.06:
                ph = 'turn2'
                continue
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            if abs(e) < _rad(3.0):
                ph = 'drive'
                continue
            return Command(0.0, _inplace(e, 0.5)), {'own': '52', 'ph': ph, 'W': W}, False
        if ph == 'drive':
            dist = math.hypot(W[0] - px, W[1] - py)
            if dist <= 0.06:
                ph = 'turn2'
                continue
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            v = 0.2 * max(0.0, math.cos(e))
            w = sat(1.5 * e, 0.5)
            return Command(v, w), {'own': '52', 'ph': ph, 'W': W}, False
        # turn2: face the tag
        cx, cy = _cam(px, py, yaw)
        e = wrap(math.atan2(-cy, -cx) - yaw)
        if abs(e) < _rad(3.0):
            return _stop(), None, True
        return Command(0.0, _inplace(e, 0.5)), {'own': '52', 'ph': 'turn2', 'W': W}, False
    return _stop(), {'own': '52', 'ph': ph, 'W': W}, False


def a53_converge(est, P):
    if est is None:
        return _stop(), False, False
    ypre = P['y_pre']
    px, py, yaw = est['px'], est['py'], est['yaw']
    if abs(px) <= 0.05 and py <= ypre + 0.08:
        return _stop(), True, False
    if py < ypre - 0.25 and abs(px) > 0.10:
        return _stop(), False, True
    ty = ypre if py > ypre else py - 0.3
    desired = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam(px, py, yaw)
    ref = math.atan2(-cy, -cx)
    beta = _beta(P)
    rel = max(-beta, min(beta, wrap(desired - ref)))
    desired = wrap(ref + rel)
    e = wrap(desired - yaw)
    dist = math.hypot(px, py - ypre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam(est['px'], est['py'], est['yaw'])
    e = wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= _rad(4.0):
        return _stop(), None
    return Command(0.0, _inplace(e, 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return _stop(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        desired = -math.pi / 2 - sat(math.atan(P['k_lat'] * px), _rad(30.0))
        e = wrap(desired - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = wrap(-math.pi / 2 - yaw)
    if abs(e) <= _rad(P['head_tol_deg']):
        return _stop(), True
    return Command(0.0, _inplace(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        return _stop(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s']:
        return _stop(), mstate, False, False, False
    if est is None:
        return _stop(), None, False, True, False
    ok = (abs(est['px']) <= P['verify_lat']
          and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(wrap(est['yaw'] + math.pi / 2)) <= _rad(P['verify_head_deg']))
    return _stop(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        n_bo = n_bo + 1
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
    bo_ok = n_bo < P['max_backoff']
    dist = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if dist >= P['backoff_dist']:
        return _stop(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return _stop(), True
