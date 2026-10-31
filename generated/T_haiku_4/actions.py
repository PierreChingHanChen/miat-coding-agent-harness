"""Action functions for the MIAT pallet-pocket docking controller (ACTION_CONTRACTS.md).

Pure functions: everything an action remembers between ticks is passed in and
returned explicitly. Only math and sim.api are used.
"""
import math

from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID

TAG_AGE_OK = 0.35  # tag_ok age limit [s] (contract a44)


# ---------------------------------------------------------------- helpers ---

def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n % 2:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _camera(px, py, yaw):
    """Camera optical centre in the frame of (px, py, yaw)."""
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def _rot(e, cap):
    """In-place rotation rule shared by a52/a54/a55 (returns w)."""
    if abs(e) > 0.12:
        return _sat(1.8 * e, cap)
    return 0.15 if e >= 0 else -0.15


def _sign(x):
    return (x > 0) - (x < 0)


# ------------------------------------------------------- A0 (G0 steps) ------

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


# ------------------------------------------------------- A3 (G3) ------------

def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [s for s in depth_samples
             if s > 0.05 and math.isfinite(s)]
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
    cx, cy = _camera(odom.x, odom.y, odom.yaw)
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
        if any(math.hypot(pos[0] - bp[0], pos[1] - bp[1]) <= 0.8
               for bp in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    win = P['confirm_window_s']
    new_hist = [h for h in (hist or []) if t - h['t'] <= win + 1e-9]
    for c in cands:
        new_hist.append({'t': t, 'c': c})
    if not cands:
        return new_hist, False, None
    tol = math.radians(P['confirm_tol_deg'])
    best, best_key = None, None
    for c in cands:
        stamps = set()
        for h in new_hist:
            hc = h['c']
            if (abs(_wrap(c['ba'] - hc['ba'])) <= tol
                    and abs(c['d'] - hc['d']) <= 0.4):
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best_key is None or key > best_key:
            best, best_key = c, key
    if best_key[0] >= P['confirm_k']:
        track = {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
        return new_hist, True, track
    return new_hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(P['track_gate_deg'])
    best, best_diff = None, None
    for c in cands or []:
        dba = _wrap(c['ba'] - track['ba'])
        if abs(dba) <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or abs(dba) < best_diff:
                best, best_diff = c, abs(dba)
    if best is not None:
        ba = _wrap(track['ba'] + 0.5 * _wrap(best['ba'] - track['ba']))
        pos = ((track['pos'][0] + best['pos'][0]) / 2.0,
               (track['pos'][1] + best['pos'][1]) / 2.0)
        new = {'ba': ba, 'd': best['d'], 'pos': pos, 't': t}
    else:
        new = dict(track)
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _camera(odom.x, odom.y, odom.yaw)
    dx = track['pos'][0] - cx
    dy = track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = _wrap(odom.yaw - math.atan2(dy, dx))
    tol = math.radians(P['app_bear_tol_deg'])
    if d <= P['d_app']:
        done = abs(b) <= tol
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


# ------------------------------------------------------- A4 (G4) ------------

def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = math.radians(P['tag_sweep_deg'])
        if pallet_ref is not None:
            cx, cy = _camera(odom.x, odom.y, odom.yaw)
            ref = math.atan2(pallet_ref['pos'][1] - cy,
                             pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        targets = [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = sweep['targets']
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        cmd = Command()
        new_sweep = {'targets': targets, 'i': (i + 1) % len(targets)}
    else:
        cmd = Command(0.0, _sat(2.0 * err, P['w_tag']))
        new_sweep = {'targets': targets, 'i': i}
    return cmd, new_sweep


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - math.pi / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return (px, py, yaw)


def a43_update_pose(est, odom, tags, t, P):
    meas = [tg for tg in (tags or []) if tg.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        px, py, yaw = a42_tag_to_pose(meas[0])
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1,
                'rej': 0, 'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment (robot frame at previous heading)
    x0, y0, yaw0 = est['o']
    dx = odom.x - x0
    dy = odom.y - y0
    f = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    l = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    dyaw = _wrap(odom.yaw - yaw0)
    ye = est['yaw']
    px = est['px'] + f * math.cos(ye) - l * math.sin(ye)
    py = est['py'] + f * math.sin(ye) + l * math.cos(ye)
    yaw = _wrap(ye + dyaw)
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'],
           'o': (odom.x, odom.y, odom.yaw)}

    if meas:
        m = meas[0]
        zx, zy, zyaw = a42_tag_to_pose(m)
        ex, ey = zx - px, zy - py
        eyaw = _wrap(zyaw - yaw)
        cam_d = math.hypot(m.x, m.z)
        if (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cam_d
                and abs(eyaw) <= math.radians(P['gate_yaw_deg'])):
            k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
            new['px'] = px + k * ex
            new['py'] = py + k * ey
            new['yaw'] = _wrap(yaw + k * eyaw)
            new['t_meas'] = t
            new['n'] = est['n'] + 1
            new['rej'] = 0
        else:
            rej = est['rej'] + 1
            if rej >= 5:
                new.update(px=zx, py=zy, yaw=zyaw, t_meas=t, n=1, rej=0)
            else:
                new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return (False, False, True)
    age = t - est['t_meas']
    tag_ok = age < TAG_AGE_OK
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cx, cy = _camera(est['px'], est['py'], est['yaw'])
    cam_d = math.hypot(cx, cy)
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
    return Command(), bl, True


# ------------------------------------------------------- A5 (G5) ------------

def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return Command(), False
    px, py = est['px'], est['py']
    y_pre = P['y_pre']
    if abs(px) <= 0.05 and abs(py - y_pre) <= 0.15:
        feasible = True
    elif py <= y_pre + 0.15:
        feasible = False
    else:
        gamma = math.atan2(abs(px), py - y_pre)
        beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
        feasible = gamma <= beta_max - math.radians(2.0)
    return Command(), feasible


def a52_reposition(est, mstate, P):
    if est is None:
        # no pose: nothing to plan from; report done so G5 moves on
        return Command(), None, True
    px, py, yaw = est['px'], est['py'], est['yaw']
    y_pre = P['y_pre']
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])

    if not (mstate and mstate.get('owner') == '52'):
        yw = max(py, y_pre + P['repos_extra'])
        xw = _sign(px) * (yw - y_pre) * math.tan(beta_max - math.radians(8.0))
        mstate = {'owner': '52', 'phase': 'turn1', 'W': (xw, yw)}
    xw, yw = mstate['W']
    phase = mstate['phase']

    def st(p):
        return {'owner': '52', 'phase': p, 'W': (xw, yw)}

    dist = math.hypot(xw - px, yw - py)

    if phase == 'turn1':
        if dist <= 0.06:
            phase = 'turn2'
        else:
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            if abs(e) >= math.radians(3.0):
                return Command(0.0, _rot(e, 0.5)), st('turn1'), False
            return Command(), st('drive'), False
    if phase == 'drive':
        if dist > 0.06:
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            v = 0.2 * max(0.0, math.cos(e))
            w = _sat(1.5 * e, 0.5)
            return Command(v, w), st('drive'), False
        phase = 'turn2'
    # turn2: face the camera toward the tag (origin)
    cx, cy = _camera(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < math.radians(3.0):
        return Command(), None, True
    return Command(0.0, _rot(e, 0.5)), st('turn2'), False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    y_pre = P['y_pre']
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])

    if abs(px) <= 0.05 and py <= y_pre + 0.08:
        return Command(), True, False
    if py < y_pre - 0.25 and abs(px) > 0.10:
        return Command(), False, True

    ty = y_pre if py > y_pre else py - 0.3
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _camera(px, py, yaw)
    ct = math.atan2(-cy, -cx)          # camera -> tag direction
    hd = _wrap(ct + _sat(_wrap(hd - ct), beta_max))
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _camera(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return Command(0.0, _sat(1.8 * e, 0.4) if abs(e) > 0.12
                   else (0.15 if e >= 0 else -0.15)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        desired = -math.pi / 2 - _sat(math.atan(P['k_lat'] * px),
                                      math.radians(30.0))
        e = _wrap(desired - yaw)
        v = (P['v_final'] * min(1.0, max(0.25, ey / 0.25))
             * max(0.0, math.cos(e)))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-math.pi / 2 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _rot(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if not (mstate and mstate.get('owner') == '56'):
        mstate = {'owner': '56', 't0': t}
    if (t - mstate['t0']) < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + math.pi / 2))
              <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if not (mstate and mstate.get('owner') == '57'):
        n_bo = n_bo + 1
        mstate = {'owner': '57', 'start': (odom.x, odom.y)}
    bo_ok = n_bo < P['max_backoff']
    sx, sy = mstate['start']
    travelled = math.hypot(odom.x - sx, odom.y - sy)
    if travelled >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
