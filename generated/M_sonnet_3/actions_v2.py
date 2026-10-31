"""Action implementations for the MIAT AGV docking controller."""
import math
from sim.api import (Command, IMG_W, FX, CAM_X, TAG_ID)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def sat(x, m):
    return max(-m, min(m, x))


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


def _cam(px, py, yaw):
    return px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)


def _rotate_cmd(e, wmax):
    if abs(e) > 0.12:
        return Command(0.0, sat(1.8 * e, wmax))
    return Command(0.0, math.copysign(0.15, e) if e != 0 else 0.0)


# ---------------- G0 ----------------
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


# ---------------- G3 ----------------
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


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
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
        cx = odom.x + CAM_X * math.cos(odom.yaw)
        cy = odom.y + CAM_X * math.sin(odom.yaw)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) < 0.8 for bp in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    win = P['confirm_window_s']
    new_hist = [h for h in (hist or []) if t - h['t'] <= win]
    for c in cands:
        new_hist.append({'t': t, 'c': c})
    tol = math.radians(P['confirm_tol_deg'])
    best = None
    best_n = 0
    for c in cands:
        stamps = set()
        for h in new_hist:
            hc = h['c']
            if abs(wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        n = len(stamps)
        if best is None or n > best_n or (n == best_n and c['score'] > best['score']):
            best, best_n = c, n
    if best is not None and best_n >= P['confirm_k']:
        return new_hist, True, {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
    return new_hist, False, None


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
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2, (track['pos'][1] + best['pos'][1]) / 2)
        new['d'] = best['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx = odom.x + CAM_X * math.cos(odom.yaw)
    cy = odom.y + CAM_X * math.sin(odom.yaw)
    dx, dy = track['pos'][0] - cx, track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = wrap(odom.yaw - math.atan2(dy, dx))
    if d <= P['d_app']:
        if abs(b) <= math.radians(P['app_bear_tol_deg']):
            return Command(0.0, 0.0), True
        return Command(0.0, sat(-1.5 * b, 0.4)), False
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    ref = None
    if track is not None:
        ref = {'ba': track['ba'], 'pos': track['pos']}
    return Command(), ref, True


def a3e_fail():
    return Command(), True


# ---------------- G4 ----------------
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    return Command(), n_loc, n_loc <= P['max_retries'], None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx = odom.x + CAM_X * math.cos(odom.yaw)
            cy = odom.y + CAM_X * math.sin(odom.yaw)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        A = math.radians(P['tag_sweep_deg'])
        sweep = {'targets': [ref, ref + A, ref - A, ref], 'i': 0}
    else:
        sweep = {'targets': list(sweep['targets']), 'i': sweep['i']}
    tgt = sweep['targets'][sweep['i']]
    err = wrap(tgt - odom.yaw)
    if abs(err) < math.radians(3.0):
        cmd = Command()
        sweep['i'] = (sweep['i'] + 1) % len(sweep['targets'])
    else:
        cmd = Command(0.0, sat(2 * err, P['w_tag']))
    return cmd, sweep


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - math.pi / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in (tags or []) if tg.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        m = meas[0]
        return {'px': m[0], 'py': m[1], 'yaw': m[2], 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    y = est['yaw']
    new = dict(est)
    new['px'] = est['px'] + f * math.cos(y) - l * math.sin(y)
    new['py'] = est['py'] + f * math.sin(y) + l * math.cos(y)
    new['yaw'] = wrap(y + wrap(odom.yaw - oyaw))
    new['o'] = (odom.x, odom.y, odom.yaw)
    if meas:
        m = meas[0]
        ex, ey = m[0] - new['px'], m[1] - new['py']
        eyaw = wrap(m[2] - new['yaw'])
        cam = _cam(*m)
        cd = math.hypot(*cam)
        if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cd and abs(eyaw) <= math.radians(P['gate_yaw_deg']):
            k = 0.5 if new['n'] < P['init_meas'] else P['fuse_gain']
            new['px'] += k * ex
            new['py'] += k * ey
            new['yaw'] = wrap(new['yaw'] + k * eyaw)
            new['t_meas'] = t
            new['n'] = new['n'] + 1
            new['rej'] = 0
        else:
            new['rej'] = new['rej'] + 1
            if new['rej'] >= 5:
                new.update({'px': m[0], 'py': m[1], 'yaw': m[2], 't_meas': t, 'n': 1, 'rej': 0})
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    ready = tag_ok and est['n'] >= P['init_meas']
    cam = _cam(est['px'], est['py'], est['yaw'])
    cd = math.hypot(*cam)
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


# ---------------- G5 ----------------
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
    return Command(), g <= _beta_max(P) - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    bm = _beta_max(P)
    if not mstate or mstate.get('own') != '52':
        yw = max(py, P['y_pre'] + P['repos_extra'])
        sgn = 1.0 if px >= 0 else -1.0
        xw = sgn * (yw - P['y_pre']) * math.tan(bm - math.radians(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    else:
        mstate = dict(mstate)
    W = mstate['W']
    tol = math.radians(3.0)
    if mstate['ph'] == 'turn1':
        if math.hypot(W[0] - px, W[1] - py) < 0.06:
            mstate['ph'] = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            if abs(e) < tol:
                mstate['ph'] = 'drive'
            else:
                return _rotate_cmd(e, 0.5), mstate, False
    if mstate['ph'] == 'drive':
        if math.hypot(W[0] - px, W[1] - py) < 0.06:
            mstate['ph'] = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            return Command(0.2 * max(0.0, math.cos(e)), sat(1.5 * e, 0.5)), mstate, False
    cx, cy = _cam(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < tol:
        return Command(), None, True
    return _rotate_cmd(e, 0.5), mstate, False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    if abs(px) <= 0.05 and py <= P['y_pre'] + 0.08:
        return Command(), True, False
    if py < P['y_pre'] - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = P['y_pre'] if py > P['y_pre'] else py - 0.3
    des = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam(px, py, yaw)
    ref = math.atan2(-cy, -cx)
    bm = _beta_max(P)
    des = wrap(ref + sat(wrap(des - ref), bm))
    e = wrap(des - yaw)
    dist = math.hypot(px, py - P['y_pre'])
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    return Command(v, sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam(est['px'], est['py'], est['yaw'])
    e = wrap(math.atan2(-cy, -cx) - est['yaw'])
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return _rotate_cmd(e, 0.4), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -math.pi / 2 - sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, sat(P['k_head'] * e, 0.5)), False
    e = wrap(-math.pi / 2 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return _rotate_cmd(e, 0.3), False


def a56_verify(est, mstate, t, P):
    if not mstate or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    if est is None:
        return Command(), None, False, True, False
    ok = (abs(est['px']) <= P['verify_lat'] and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(wrap(est['yaw'] + math.pi / 2)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, not ok, ok


def a57_backoff(odom, mstate, n_bo, P):
    if not mstate or mstate.get('own') != '57':
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
        n_bo = n_bo + 1
    bo_ok = n_bo < P['max_backoff']
    if math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0']) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


# ---------------- GS (CR-1 safety supervisor) ----------------
def as1_obstacle_monitor(scan, sstate, t, P):
    st = dict(sstate) if sstate else {}
    near = False
    if scan is not None:
        sector = math.radians(P['safe_sector_deg'])
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and r < P['safe_dist']:
                if abs(wrap(scan.angle_min + i * scan.angle_increment)) <= sector:
                    near = True
                    break
    if near:
        st['last_near'] = t
    last = st.get('last_near')
    clear = (not near) and (last is None or (t - last) >= P['clear_s'])
    return st, near, clear


def as2_inhibit_forward(cmd):
    return Command(min(cmd.v, 0.0), cmd.w)
