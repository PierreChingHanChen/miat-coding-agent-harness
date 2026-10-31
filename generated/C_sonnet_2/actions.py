"""Datapath Actions of the AGV docking controller (stateless, one call = one tick)."""
import math
from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID

PI = math.pi


def wrap(a):
    a = math.fmod(a + PI, 2 * PI)
    if a <= 0:
        a += 2 * PI
    return a - PI


def sat(x, m):
    return max(-m, min(m, x))


def _cam_odom(odom):
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _cam_pose(est):
    return (est['px'] + CAM_X * math.cos(est['yaw']), est['py'] + CAM_X * math.sin(est['yaw']))


def _rot(e, wmax):
    """In-place rotation rule."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return 0.15 if e > 0 else -0.15


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


# ---------------- trivial actions ----------------
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


# ---------------- A3 ----------------
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    v = sorted(s for s in depth_samples if s > 0.05)
    if len(v) < 5:
        return 0.0, False
    m = _median(v)
    k = [s for s in v if abs(s - m) <= 0.3]
    if len(k) < 3:
        return 0.0, False
    return _median(sorted(k)), True


def _median(s):
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    out = []
    cx, cy = _cam_odom(odom)
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
        ba = wrap(odom.yaw - b)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) < 0.8 for bp in (blacklist or [])):
            continue
        out.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return out


def a322_confirm(cands, hist, t, P):
    hist = [h for h in (hist or []) if t - h['t'] <= P['confirm_window_s']]
    for c in cands:
        hist.append({'t': t, 'c': c})
    tol = math.radians(P['confirm_tol_deg'])
    best = None
    best_n = 0
    for c in cands:
        ts = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                ts.add(h['t'])
        n = len(ts)
        if n > best_n or (n == best_n and best is not None and c['score'] > best['score']):
            best, best_n = c, n
    if best is not None and best_n >= P['confirm_k']:
        return hist, True, {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(P['track_gate_deg'])
    best, bd = None, None
    for c in cands:
        diff = abs(wrap(c['ba'] - track['ba']))
        if diff <= gate and abs(c['d'] - track['d']) < 0.6:
            if bd is None or diff < bd:
                best, bd = c, diff
    new = dict(track)
    if best is not None:
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(best['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2, (track['pos'][1] + best['pos'][1]) / 2)
        new['d'] = best['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_odom(odom)
    dx, dy = track['pos'][0] - cx, track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = wrap(odom.yaw - math.atan2(dy, dx))
    if d <= P['d_app']:
        if abs(b) <= math.radians(P['app_bear_tol_deg']):
            return Command(), True
        return Command(0.0, sat(-1.5 * b, 0.4)), False
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ---------------- A4 ----------------
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
    i = sweep['i']
    err = wrap(sweep['targets'][i] - odom.yaw)
    if abs(err) < math.radians(3):
        return Command(), {'targets': list(sweep['targets']), 'i': (i + 1) % len(sweep['targets'])}
    return Command(0.0, sat(2 * err, P['w_tag'])), {'targets': list(sweep['targets']), 'i': i}


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - PI / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw


def a43_update_pose(est, odom, tags, t, P):
    metas = [a42_tag_to_pose(g) for g in (tags or []) if g.id == TAG_ID]
    if est is None:
        if not metas:
            return None
        px, py, yaw = metas[0]
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    e = dict(est)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    c, s = math.cos(est['yaw']), math.sin(est['yaw'])
    e['px'] = est['px'] + f * c - l * s
    e['py'] = est['py'] + f * s + l * c
    e['yaw'] = wrap(est['yaw'] + wrap(odom.yaw - oyaw))
    e['o'] = (odom.x, odom.y, odom.yaw)
    if metas:
        zx, zy, zyaw = metas[0]
        ex, ey, eyaw = zx - e['px'], zy - e['py'], wrap(zyaw - e['yaw'])
        cam = _cam_pose(e)
        cd = math.hypot(cam[0], cam[1])
        if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cd and abs(eyaw) <= math.radians(P['gate_yaw_deg']):
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
                e.update({'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0})
    return e


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    ready = tag_ok and est['n'] >= P['init_meas']
    cam = _cam_pose(est)
    cd = math.hypot(cam[0], cam[1])
    stale = (age > P['pose_stale_s'] and cd > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, ready, stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ---------------- A5 ----------------
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
    g = math.atan2(abs(px), py - P['y_pre'])
    return Command(), g <= _beta_max(P) - math.radians(2)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    bm = _beta_max(P)
    px, py, yaw = est['px'], est['py'], est['yaw']
    if not mstate or mstate.get('own') != '52':
        yw = max(py, P['y_pre'] + P['repos_extra'])
        sg = 1.0 if px >= 0 else -1.0
        xw = sg * (yw - P['y_pre']) * math.tan(bm - math.radians(8))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    ph = mstate['ph']
    W = mstate['W']
    dW = math.hypot(W[0] - px, W[1] - py)
    if ph == 'turn1':
        if dW < 0.06:
            ph = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            if abs(e) < math.radians(3):
                ph = 'drive'
            else:
                return Command(0.0, _rot(e, 0.5)), dict(mstate, ph='turn1'), False
    if ph == 'drive':
        if dW < 0.06:
            ph = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            return Command(0.2 * max(0.0, math.cos(e)), sat(1.5 * e, 0.5)), dict(mstate, ph='drive'), False
    cx, cy = _cam_pose(est)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < math.radians(3):
        return Command(), None, True
    return Command(0.0, _rot(e, 0.5)), dict(mstate, ph='turn2'), False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    y_pre = P['y_pre']
    if abs(px) <= 0.05 and py <= y_pre + 0.08:
        return Command(), True, False
    if py < y_pre - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = y_pre if py > y_pre else py - 0.3
    des = math.atan2(ty - py, -px)
    cx, cy = _cam_pose(est)
    ct = math.atan2(-cy, -cx)
    bm = _beta_max(P)
    des = ct + sat(wrap(des - ct), bm)
    e = wrap(des - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    return Command(v, sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam_pose(est)
    e = wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= math.radians(4):
        return Command(), None
    return Command(0.0, _rot(e, 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -PI / 2 - sat(math.atan(P['k_lat'] * px), math.radians(30))
        e = wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, sat(P['k_head'] * e, 0.5)), False
    e = wrap(-PI / 2 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _rot(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if not mstate or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    if est is None:
        return Command(), None, False, True, False
    ok = (abs(est['px']) <= P['verify_lat'] and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(wrap(est['yaw'] + PI / 2)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, not ok, ok


def a57_backoff(odom, mstate, n_bo, P):
    if not mstate or mstate.get('own') != '57':
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
        n_bo += 1
    bo_ok = n_bo < P['max_backoff']
    if math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0']) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
