"""Datapath Actions of the AGV docking controller (pure, stateless functions)."""
import math
from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID


def wrap(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def sat(x, lim):
    return max(-lim, min(lim, x))


def _cam_odom(odom):
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _cam_est(est):
    return (est['px'] + CAM_X * math.cos(est['yaw']), est['py'] + CAM_X * math.sin(est['yaw']))


def _rot_inplace(e, wmax):
    """In-place rotation rule: proportional, with minimum creep speed."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(0.15, e) if e != 0 else 0.0


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


def _median(sorted_vals):
    n = len(sorted_vals)
    if n % 2:
        return sorted_vals[n // 2]
    return 0.5 * (sorted_vals[n // 2 - 1] + sorted_vals[n // 2])


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
        b = a333_pallet_bearing(a331_bbox_center(det))
        W = det.width_px * d / FX
        if not (P['width_min'] <= W <= P['width_max']):
            continue
        ba = wrap(odom.yaw - b)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) < 0.8 for bp in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    hist = [h for h in hist if t - h['t'] <= P['confirm_window_s']]
    for c in cands:
        hist.append({'t': t, 'c': c})
    tol = math.radians(P['confirm_tol_deg'])
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
    gate = math.radians(P['track_gate_deg'])
    best = None
    best_diff = None
    for c in cands:
        diff = abs(wrap(c['ba'] - track['ba']))
        if diff <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or diff < best_diff:
                best, best_diff = c, diff
    new = dict(track)
    if best is not None:
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(best['ba'] - track['ba']))
        new['pos'] = (0.5 * (track['pos'][0] + best['pos'][0]), 0.5 * (track['pos'][1] + best['pos'][1]))
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
        if abs(b) <= math.radians(P['app_bear_tol_deg']):
            return Command(0.0, 0.0), True
        return Command(0.0, sat(-1.5 * b, 0.4)), False
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
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
        A = math.radians(P['tag_sweep_deg'])
        sweep = {'targets': [ref, ref + A, ref - A, ref], 'i': 0}
    targets = sweep['targets']
    i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        return Command(), {'targets': list(targets), 'i': (i + 1) % len(targets)}
    return Command(0.0, sat(2.0 * err, P['w_tag'])), {'targets': list(targets), 'i': i}


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - math.pi / 2.0)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags:
        if tg.id == TAG_ID:
            meas = (a42_tag_to_pose(tg), math.hypot(tg.x, tg.z))
            break
    if est is None:
        if meas is None:
            return None
        (px, py, yaw), _ = meas
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    new = dict(est)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    cy_, sy_ = math.cos(est['yaw']), math.sin(est['yaw'])
    new['px'] = est['px'] + f * cy_ - l * sy_
    new['py'] = est['py'] + f * sy_ + l * cy_
    new['yaw'] = wrap(est['yaw'] + wrap(odom.yaw - oyaw))
    new['o'] = (odom.x, odom.y, odom.yaw)
    if meas is not None:
        (mx, my, myaw), dist = meas
        ex, ey, eyaw = mx - new['px'], my - new['py'], wrap(myaw - new['yaw'])
        if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * dist and abs(eyaw) <= math.radians(P['gate_yaw_deg']):
            k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
            new['px'] += k * ex
            new['py'] += k * ey
            new['yaw'] = wrap(new['yaw'] + k * eyaw)
            new['t_meas'] = t
            new['n'] = est['n'] + 1
            new['rej'] = 0
        else:
            new['rej'] = est['rej'] + 1
            if new['rej'] >= 5:
                new.update({'px': mx, 'py': my, 'yaw': myaw, 't_meas': t, 'n': 1, 'rej': 0})
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    ready = tag_ok and est['n'] >= P['init_meas']
    cx, cy = _cam_est(est)
    stale = (age > P['pose_stale_s'] and math.hypot(cx, cy) > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, ready, stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist)
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ---------------------------------------------------------------- A5
def a50_enter():
    return Command(), None, 0, True, False, False, True


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


def a51_plan(est, P):
    stop = Command()
    if est is None:
        return stop, False
    px, py = est['px'], est['py']
    if abs(px) <= 0.05 and abs(py - P['y_pre']) <= 0.15:
        return stop, True
    if py <= P['y_pre'] + 0.15:
        return stop, False
    g = math.atan2(abs(px), py - P['y_pre'])
    return stop, g <= _beta_max(P) - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    if mstate is None or mstate.get('own') != '52':
        bm = _beta_max(P)
        yw = max(est['py'], P['y_pre'] + P['repos_extra'])
        sgn = 1.0 if est['px'] >= 0 else -1.0
        xw = sgn * (yw - P['y_pre']) * math.tan(bm - math.radians(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    ph = mstate['ph']
    W = mstate['W']
    px, py, yaw = est['px'], est['py'], est['yaw']
    for _ in range(4):
        if ph == 'turn1':
            dist = math.hypot(W[0] - px, W[1] - py)
            if dist < 0.06:
                ph = 'turn2'
                continue
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            if abs(e) < math.radians(3.0):
                ph = 'drive'
                continue
            return Command(0.0, _rot_inplace(e, 0.5)), {'own': '52', 'ph': ph, 'W': W}, False
        if ph == 'drive':
            dist = math.hypot(W[0] - px, W[1] - py)
            if dist < 0.06:
                ph = 'turn2'
                continue
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            return (Command(0.2 * max(0.0, math.cos(e)), sat(1.5 * e, 0.5)),
                    {'own': '52', 'ph': ph, 'W': W}, False)
        if ph == 'turn2':
            cx, cy = _cam_est(est)
            e = wrap(math.atan2(-cy, -cx) - yaw)
            if abs(e) < math.radians(3.0):
                return Command(), None, True
            return Command(0.0, _rot_inplace(e, 0.5)), {'own': '52', 'ph': ph, 'W': W}, False
    return Command(), {'own': '52', 'ph': ph, 'W': W}, False


def a53_converge(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    y_pre = P['y_pre']
    if abs(px) <= 0.05 and py <= y_pre + 0.08:
        return Command(), True, False
    if py < y_pre - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = y_pre if py > y_pre else py - 0.3
    des = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam_est(est)
    ct = math.atan2(-cy, -cx)
    bm = _beta_max(P)
    des = wrap(ct + sat(wrap(des - ct), bm))
    e = wrap(des - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    return Command(v, sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam_est(est)
    e = wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return Command(0.0, _rot_inplace(e, 0.4)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -math.pi / 2.0 - sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, sat(P['k_head'] * e, 0.5)), False
    e = wrap(-math.pi / 2.0 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _rot_inplace(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        return Command(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    if est is None:
        return Command(), None, False, True, False
    ok = (abs(est['px']) <= P['verify_lat'] and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(wrap(est['yaw'] + math.pi / 2.0)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, not ok, ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        n_bo += 1
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
        return Command(-0.15, 0.0), mstate, n_bo, n_bo < P['max_backoff'], False
    bo_ok = n_bo < P['max_backoff']
    if math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0']) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
