"""Datapath Actions of the AGV docking controller (pure, tick-based, no hidden state)."""
import math
from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID


def wrap(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def sat(x, m):
    return max(-m, min(m, x))


def _deg(x):
    return math.radians(x)


def _beta_max(P):
    return _deg(P['fov_half_deg'] - P['fov_margin_deg'])


def _cam_odom(odom):
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _cam_pose(px, py, yaw):
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


# ---------------------------------------------------------------- A0..A2, A6, A9
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


# ---------------------------------------------------------------- A3
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = sorted(s for s in depth_samples if s > 0.05)
    if len(valid) < 5:
        return 0.0, False
    m = _median(valid)
    kept = sorted(s for s in valid if abs(s - m) <= 0.3)
    if len(kept) < 3:
        return 0.0, False
    return _median(kept), True


def _median(v):
    n = len(v)
    if n % 2:
        return v[n // 2]
    return 0.5 * (v[n // 2 - 1] + v[n // 2])


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam_odom(odom)
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
        ba = wrap(odom.yaw - b)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - q[0], pos[1] - q[1]) < 0.8 for q in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    hist = [h for h in hist if t - h['t'] <= P['confirm_window_s']]
    for c in cands:
        hist.append({'t': t, 'c': c})
    tol = _deg(P['confirm_tol_deg'])
    best = None
    best_n = 0
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        n = len(stamps)
        if n > best_n or (n == best_n and best is not None and c['score'] > best['score']):
            best, best_n = c, n
    if best is not None and best_n >= P['confirm_k']:
        return hist, True, {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = _deg(P['track_gate_deg'])
    best = None
    bd = None
    for c in cands:
        da = abs(wrap(c['ba'] - track['ba']))
        if da <= gate and abs(c['d'] - track['d']) < 0.6:
            if bd is None or da < bd:
                best, bd = c, da
    new = dict(track)
    if best is not None:
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(best['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2.0, (track['pos'][1] + best['pos'][1]) / 2.0)
        new['d'] = best['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_odom(odom)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        if abs(b) <= _deg(P['app_bear_tol_deg']):
            return Command(0.0, 0.0), True
        return Command(0.0, sat(-1.5 * b, 0.4)), False
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < _deg(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ---------------------------------------------------------------- A4
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    return Command(), n_loc, n_loc <= P['max_retries'], None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam_odom(odom)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        A = _deg(P['tag_sweep_deg'])
        sweep = {'targets': [ref, ref + A, ref - A, ref], 'i': 0}
    targets = sweep['targets']
    i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < _deg(3.0):
        return Command(), {'targets': list(targets), 'i': (i + 1) % len(targets)}
    return Command(0.0, sat(2.0 * err, P['w_tag'])), {'targets': list(targets), 'i': i}


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - math.pi / 2.0)
    x, z = tag.x, tag.z
    cx = -(z * math.cos(yaw) + x * math.sin(yaw))
    cy = -(z * math.sin(yaw) - x * math.cos(yaw))
    return (cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw)


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags:
        if tg.id == TAG_ID:
            meas = a42_tag_to_pose(tg)
            break
    if est is None:
        if meas is None:
            return None
        return {'px': meas[0], 'py': meas[1], 'yaw': meas[2], 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    e = dict(est)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    yw = est['yaw']
    e['px'] = est['px'] + f * math.cos(yw) - l * math.sin(yw)
    e['py'] = est['py'] + f * math.sin(yw) + l * math.cos(yw)
    e['yaw'] = wrap(yw + wrap(odom.yaw - oyaw))
    e['o'] = (odom.x, odom.y, odom.yaw)
    if meas is not None:
        ex, ey, eyaw = meas[0] - e['px'], meas[1] - e['py'], wrap(meas[2] - e['yaw'])
        cxp, cyp = _cam_pose(e['px'], e['py'], e['yaw'])
        cd = math.hypot(cxp, cyp)
        if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cd and abs(eyaw) <= _deg(P['gate_yaw_deg']):
            k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
            e['px'] += k * ex
            e['py'] += k * ey
            e['yaw'] = wrap(e['yaw'] + k * eyaw)
            e['t_meas'] = t
            e['n'] = est['n'] + 1
            e['rej'] = 0
        else:
            e['rej'] = est['rej'] + 1
            if e['rej'] >= 5:
                e = {'px': meas[0], 'py': meas[1], 'yaw': meas[2], 't_meas': t, 'n': 1, 'rej': 0,
                     'o': (odom.x, odom.y, odom.yaw)}
    return e


def _cam_dist(est):
    cx, cy = _cam_pose(est['px'], est['py'], est['yaw'])
    return math.hypot(cx, cy)


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    ready = tag_ok and est['n'] >= P['init_meas']
    stale = (age > P['pose_stale_s'] and _cam_dist(est) > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, ready, stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist) if blacklist else []
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ---------------------------------------------------------------- A5
def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return Command(), False
    px, py = est['px'], est['py']
    if abs(px) <= 0.05 and abs(py - P['y_pre']) <= 0.15:
        return Command(), True
    if py <= P['y_pre'] + 0.15:
        return Command(), False
    gamma = math.atan2(abs(px), py - P['y_pre'])
    return Command(), gamma <= _beta_max(P) - _deg(2.0)


def _rotate_cmd(e, wmax):
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(min(0.15, wmax), e)


def _cam_to_tag_err(est):
    cx, cy = _cam_pose(est['px'], est['py'], est['yaw'])
    return wrap(math.atan2(-cy, -cx) - est['yaw'])


def a52_reposition(est, mstate, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    bm = _beta_max(P)
    if mstate is None or mstate.get('own') != '52':
        yw = max(py, P['y_pre'] + P['repos_extra'])
        s = 1.0 if px >= 0 else -1.0
        xw = s * (yw - P['y_pre']) * math.tan(bm - _deg(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    ph = mstate['ph']
    W = mstate['W']
    dist = math.hypot(W[0] - px, W[1] - py)
    if ph == 'turn1':
        if dist < 0.06:
            ph = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            if abs(e) < _deg(3.0):
                ph = 'drive'
            else:
                return Command(0.0, _rotate_cmd(e, 0.5)), dict(mstate), False
    if ph == 'drive':
        if dist < 0.06:
            ph = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            return (Command(0.2 * max(0.0, math.cos(e)), sat(1.5 * e, 0.5)),
                    {'own': '52', 'ph': 'drive', 'W': W}, False)
    e = _cam_to_tag_err(est)
    if abs(e) < _deg(3.0):
        return Command(), None, True
    return Command(0.0, _rotate_cmd(e, 0.5)), {'own': '52', 'ph': 'turn2', 'W': W}, False


def a53_converge(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return Command(), True, False
    if py < yp - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = yp if py > yp else py - 0.3
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam_pose(px, py, yaw)
    ct = math.atan2(-cy, -cx)
    bm = _beta_max(P)
    hd = ct + sat(wrap(hd - ct), bm)
    e = wrap(hd - yaw)
    dp = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dp / 0.4))
    return Command(v, sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    e = _cam_to_tag_err(est)
    if abs(e) <= _deg(4.0):
        return Command(), None
    return Command(0.0, _rotate_cmd(e, 0.4)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -math.pi / 2 - sat(math.atan(P['k_lat'] * px), _deg(30.0))
        e = wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, sat(P['k_head'] * e, 0.5)), False
    e = wrap(-math.pi / 2 - yaw)
    if abs(e) <= _deg(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _rotate_cmd(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    ok = (abs(est['px']) <= P['verify_lat'] and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(wrap(est['yaw'] + math.pi / 2)) <= _deg(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
        n_bo += 1
    bo_ok = n_bo < P['max_backoff']
    if math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0']) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
