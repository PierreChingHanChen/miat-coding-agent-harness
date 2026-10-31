"""Action functions for the AGV pallet-pocket docking controller (MIAT)."""
import math
from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID


# ---------------------------------------------------------------- helpers
def _wrap(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def _sat(x, m):
    return max(-m, min(m, x))


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _beta_max(P):
    return math.radians(P['fov_half_deg'] - P['fov_margin_deg'])


def _cam_of(px, py, yaw):
    return px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)


def _rot_cmd(e, wmax):
    """In-place rotation toward error e (+ = left)."""
    if abs(e) > 0.12:
        w = _sat(1.8 * e, wmax)
    else:
        w = math.copysign(0.15, e)
    return Command(0.0, w)


# ---------------------------------------------------------------- G0
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


# ---------------------------------------------------------------- G3
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [float(s) for s in depth_samples if s > 0.05]
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
    cx = odom.x + CAM_X * math.cos(odom.yaw)
    cy = odom.y + CAM_X * math.sin(odom.yaw)
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
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) < 0.8
               for bp in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    new_hist = [h for h in (hist or []) if t - h['t'] <= window]
    for c in cands:
        new_hist.append({'t': t, 'c': c})
    tol = math.radians(P['confirm_tol_deg'])
    best = None
    best_key = None
    for c in cands:
        stamps = set()
        for h in new_hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best_key is None or key > best_key:
            best_key, best = key, c
    if best is not None and best_key[0] >= P['confirm_k']:
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
    cx = odom.x + CAM_X * math.cos(odom.yaw)
    cy = odom.y + CAM_X * math.sin(odom.yaw)
    dx = track['pos'][0] - cx
    dy = track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = _wrap(odom.yaw - math.atan2(dy, dx))
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


# ---------------------------------------------------------------- G4
def a40_enter(n_loc, est, P):
    n = n_loc + 1
    return Command(), n, n <= P['max_retries'], None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx = odom.x + CAM_X * math.cos(odom.yaw)
            cy = odom.y + CAM_X * math.sin(odom.yaw)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        A = math.radians(P['tag_sweep_deg'])
        sweep = {'targets': [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)],
                 'i': 0}
    else:
        sweep = {'targets': list(sweep['targets']), 'i': sweep['i']}
    i = sweep['i']
    err = _wrap(sweep['targets'][i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        sweep['i'] = (i + 1) % len(sweep['targets'])
        return Command(), sweep
    return Command(0.0, _sat(2.0 * err, P['w_tag'])), sweep


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - math.pi / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    return cx - CAM_X * math.cos(yaw), cy - CAM_X * math.sin(yaw), yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags or []:
        if tg.id == TAG_ID:
            meas = (a42_tag_to_pose(tg), math.hypot(tg.x, tg.z))
            break
    o_now = (odom.x, odom.y, odom.yaw)
    if est is None:
        if meas is None:
            return None
        (px, py, yaw), _ = meas
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': o_now}
    new = dict(est)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    yw = est['yaw']
    new['px'] = est['px'] + f * math.cos(yw) - l * math.sin(yw)
    new['py'] = est['py'] + f * math.sin(yw) + l * math.cos(yw)
    new['yaw'] = _wrap(yw + _wrap(odom.yaw - oyaw))
    new['o'] = o_now
    if meas is not None:
        (mx, my, myaw), cdist = meas
        ex, ey = mx - new['px'], my - new['py']
        eyaw = _wrap(myaw - new['yaw'])
        if (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cdist
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
                new.update(px=mx, py=my, yaw=myaw, t_meas=t, n=1, rej=0)
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    ready = tag_ok and est['n'] >= P['init_meas']
    cx, cy = _cam_of(est['px'], est['py'], est['yaw'])
    stale = (age > P['pose_stale_s'] and math.hypot(cx, cy) > P['near_dist']) \
        or age > P['blind_max_s']
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


# ---------------------------------------------------------------- G5
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
    return Command(), gamma <= _beta_max(P) - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    if not mstate or mstate.get('owner') != '52':
        yp = P['y_pre']
        yw = max(py, yp + P['repos_extra'])
        sgn = 1.0 if px >= 0 else -1.0
        xw = sgn * (yw - yp) * math.tan(_beta_max(P) - math.radians(8.0))
        phase = 'turn1'
        if math.hypot(xw - px, yw - py) < 0.06:
            phase = 'turn2'
        mstate = {'owner': '52', 'phase': phase, 'W': (xw, yw)}
    else:
        mstate = dict(mstate)
    xw, yw = mstate['W']
    phase = mstate['phase']
    dist = math.hypot(xw - px, yw - py)
    tol = math.radians(3.0)
    if phase == 'turn1':
        if dist < 0.06:
            mstate['phase'] = 'turn2'
            return Command(), mstate, False
        e = _wrap(math.atan2(yw - py, xw - px) - yaw)
        if abs(e) < tol:
            mstate['phase'] = 'drive'
            return Command(), mstate, False
        return _rot_cmd(e, 0.5), mstate, False
    if phase == 'drive':
        if dist < 0.06:
            mstate['phase'] = 'turn2'
            return Command(), mstate, False
        e = _wrap(math.atan2(yw - py, xw - px) - yaw)
        return Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5)), mstate, False
    # turn2
    cx, cy = _cam_of(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < tol:
        return Command(), None, True
    return _rot_cmd(e, 0.5), mstate, False


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
    desired = math.atan2(ty - py, -px)
    cx, cy = _cam_of(px, py, yaw)
    ctag = math.atan2(-cy, -cx)
    bm = _beta_max(P)
    desired = _wrap(ctag + _sat(_wrap(desired - ctag), bm))
    e = _wrap(desired - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    return Command(v, _sat(P['k_head'] * e, 0.6)), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam_of(est['px'], est['py'], est['yaw'])
    e = _wrap(math.atan2(-cy, -cx) - est['yaw'])
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
    if not mstate or mstate.get('owner') != '56':
        mstate = {'owner': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    ok = (est is not None
          and abs(est['px']) <= P['verify_lat']
          and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(_wrap(est['yaw'] + math.pi / 2)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, not ok, ok


def a57_backoff(odom, mstate, n_bo, P):
    if not mstate or mstate.get('owner') != '57':
        mstate = {'owner': '57', 'start': (odom.x, odom.y)}
        n_bo = n_bo + 1
    else:
        mstate = dict(mstate)
    bo_ok = n_bo < P['max_backoff']
    sx, sy = mstate['start']
    if math.hypot(odom.x - sx, odom.y - sy) >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
