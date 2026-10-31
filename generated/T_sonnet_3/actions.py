"""Action functions for the AGV pallet-pocket docking controller (MIAT flow)."""
import math
from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID


# ----------------------------------------------------------------- helpers
def _wrap(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _cam_odom(odom):
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _cam_pallet(px, py, yaw):
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def _rot_cmd(e, wmax):
    """In-place rotation toward error e (+ = turn left)."""
    if abs(e) > 0.12:
        w = _sat(1.8 * e, wmax)
    else:
        w = 0.15 if e >= 0 else -0.15
    return Command(0.0, w)


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


# ------------------------------------------------------------------- G0
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


# ------------------------------------------------------------------- G3
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = sorted(float(s) for s in depth_samples if s > 0.05)
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
    return math.atan((u - IMG_W / 2) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam_odom(odom)
    for det in (pallets or []):
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
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) < 0.8 for bp in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    new_hist = [h for h in (hist or []) if (t - h['t']) <= P['confirm_window_s']]
    for c in cands:
        new_hist.append({'t': t, 'c': c})
    tol = math.radians(P['confirm_tol_deg'])
    best = None
    best_n = 0
    for c in cands:
        stamps = set()
        for h in new_hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        n = len(stamps)
        if best is None or n > best_n or (n == best_n and c['score'] > best['score']):
            best, best_n = c, n
    if best is not None and best_n >= P['confirm_k']:
        track = {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
        return new_hist, True, track
    return new_hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    new = dict(track)
    gate = math.radians(P['track_gate_deg'])
    best = None
    best_diff = None
    for c in cands:
        diff = abs(_wrap(c['ba'] - track['ba']))
        if diff <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or diff < best_diff:
                best, best_diff = c, diff
    if best is not None:
        new['ba'] = _wrap(track['ba'] + 0.5 * _wrap(best['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2.0,
                      (track['pos'][1] + best['pos'][1]) / 2.0)
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
    b = _wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        done = abs(b) <= math.radians(P['app_bear_tol_deg'])
        w = 0.0 if done else _sat(-1.5 * b, 0.4)
        return Command(0.0, w), done
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    ref = None
    if track is not None:
        ref = {'ba': track['ba'], 'pos': track['pos']}
    return Command(), ref, True


def a3e_fail():
    return Command(), True


# ------------------------------------------------------------------- G4
def a40_enter(n_loc, est, P):
    n = n_loc + 1
    return Command(), n, n <= P['max_retries'], None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam_odom(odom)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        A = math.radians(P['tag_sweep_deg'])
        sweep = {'targets': [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)], 'i': 0}
    targets = list(sweep['targets'])
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        return Command(), {'targets': targets, 'i': (i + 1) % len(targets)}
    return Command(0.0, _sat(2.0 * err, P['w_tag'])), {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - math.pi / 2)
    x, z = tag.x, tag.z
    cx = -(z * math.cos(yaw) + x * math.sin(yaw))
    cy = -(z * math.sin(yaw) - x * math.cos(yaw))
    return cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in (tags or []) if tg.id == TAG_ID]
    z = meas[0] if meas else None

    def fresh(m):
        return {'px': m[0], 'py': m[1], 'yaw': m[2], 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    if est is None:
        return fresh(z) if z is not None else None
    new = dict(est)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    c, s = math.cos(est['yaw']), math.sin(est['yaw'])
    new['px'] = est['px'] + f * c - l * s
    new['py'] = est['py'] + f * s + l * c
    new['yaw'] = _wrap(est['yaw'] + _wrap(odom.yaw - oyaw))
    new['o'] = (odom.x, odom.y, odom.yaw)
    if z is not None:
        ex, ey, eyaw = z[0] - new['px'], z[1] - new['py'], _wrap(z[2] - new['yaw'])
        cam = _cam_pallet(new['px'], new['py'], new['yaw'])
        cd = math.hypot(cam[0], cam[1])
        if (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cd
                and abs(eyaw) <= math.radians(P['gate_yaw_deg'])):
            k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
            new['px'] += k * ex
            new['py'] += k * ey
            new['yaw'] = _wrap(new['yaw'] + k * eyaw)
            new['t_meas'] = t
            new['n'] = est['n'] + 1
            new['rej'] = 0
        else:
            new['rej'] = est['rej'] + 1
            if new['rej'] >= 5:
                return fresh(z)
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam = _cam_pallet(est['px'], est['py'], est['yaw'])
    cd = math.hypot(cam[0], cam[1])
    stale = (age > P['pose_stale_s'] and cd > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, pose_ready, stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist) if blacklist is not None else []
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ------------------------------------------------------------------- G5
def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return Command(), False
    px, py = est['px'], est['py']
    y_pre = P['y_pre']
    if abs(px) <= 0.05 and abs(py - y_pre) <= 0.15:
        return Command(), True
    if py <= y_pre + 0.15:
        return Command(), False
    gamma = math.atan2(abs(px), py - y_pre)
    return Command(), gamma <= _beta_max(P) - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    if not (isinstance(mstate, dict) and mstate.get('owner') == '52'):
        bm = _beta_max(P)
        yw = max(py, P['y_pre'] + P['repos_extra'])
        sgn = 1.0 if px >= 0 else -1.0
        xw = sgn * (yw - P['y_pre']) * math.tan(bm - math.radians(8.0))
        mstate = {'owner': '52', 'phase': 'turn1', 'W': (xw, yw)}
    ms = dict(mstate)
    xw, yw = ms['W']
    for _ in range(4):
        phase = ms['phase']
        dist = math.hypot(xw - px, yw - py)
        if phase == 'turn1':
            if dist < 0.06:
                ms['phase'] = 'turn2'
                continue
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            if abs(e) < math.radians(3.0):
                ms['phase'] = 'drive'
                continue
            return _rot_cmd(e, 0.5), ms, False
        if phase == 'drive':
            if dist < 0.06:
                ms['phase'] = 'turn2'
                continue
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            return Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5)), ms, False
        # turn2
        cam = _cam_pallet(px, py, yaw)
        e = _wrap(math.atan2(-cam[1], -cam[0]) - yaw)
        if abs(e) < math.radians(3.0):
            return Command(), None, True
        return _rot_cmd(e, 0.5), ms, False
    return Command(), ms, False


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
    desired = math.atan2(ty - py, 0.0 - px)
    cam = _cam_pallet(px, py, yaw)
    tagdir = math.atan2(-cam[1], -cam[0])
    bm = _beta_max(P)
    desired = _wrap(tagdir + _sat(_wrap(desired - tagdir), bm))
    e = _wrap(desired - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    return Command(v, _sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cam = _cam_pallet(est['px'], est['py'], est['yaw'])
    e = _wrap(math.atan2(-cam[1], -cam[0]) - est['yaw'])
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return _rot_cmd(e, 0.4), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        desired = -math.pi / 2 - _sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = _wrap(desired - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, _sat(P['k_head'] * e, 0.5)), False
    e = _wrap(-math.pi / 2 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return _rot_cmd(e, 0.3), False


def a56_verify(est, mstate, t, P):
    if not (isinstance(mstate, dict) and mstate.get('owner') == '56'):
        mstate = {'owner': '56', 't0': t}
    if (t - mstate['t0']) < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + math.pi / 2)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if not (isinstance(mstate, dict) and mstate.get('owner') == '57'):
        mstate = {'owner': '57', 'start': (odom.x, odom.y)}
        n_bo = n_bo + 1
    bo_ok = n_bo < P['max_backoff']
    sx, sy = mstate['start']
    if math.hypot(odom.x - sx, odom.y - sy) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
