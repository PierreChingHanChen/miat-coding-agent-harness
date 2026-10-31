"""Action implementations for the pallet-pocket docking controller (ACTION_CONTRACTS.md).

Pure functions: every value an action remembers between ticks is passed in and
returned explicitly. No module-level mutable state, no blocking, no randomness.
"""
import math

from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID

HALF_PI = math.pi / 2.0


# ---------------------------------------------------------------- helpers ----
def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(float(a) + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def sat(x, m):
    return max(-m, min(m, x))


def rad(deg):
    return math.radians(deg)


def in_place_w(e, wmax):
    """In-place rotation rule: proportional, with a minimum creep of 0.15 rad/s."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(0.15, e) if e != 0.0 else 0.0


def cam_pos(px, py, yaw):
    """Camera position in the frame of the pose (x, y)."""
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def cam_pos_odom(odom):
    return cam_pos(odom.x, odom.y, odom.yaw)


def beta_max(P):
    """Maximum bearing of the tag from the optical axis that keeps it in the FOV margin [rad]."""
    return rad(P['fov_half_deg'] - P['fov_margin_deg'])


def median(vals):
    s = sorted(vals)
    n = len(s)
    if n % 2 == 1:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _is_owned(mstate, owner):
    return mstate is not None and mstate.get('own') == owner


# ------------------------------------------------------------- A0 / A6 / A9 --
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


# -------------------------------------------------------------------- A3 -----
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
    m = median(valid)
    kept = [s for s in valid if abs(s - m) <= 0.3]
    if len(kept) < 3:
        return 0.0, False
    return median(kept), True


def a333_pallet_bearing(u):
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = cam_pos_odom(odom)
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
        if any(math.hypot(pos[0] - bx, pos[1] - by) < 0.8 for (bx, by) in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    hist = [h for h in (hist or []) if t - h['t'] <= window + 1e-9]
    for c in cands:
        hist.append({'t': t, 'c': c})

    best = None  # (count, score, cand)
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(hc['ba'] - c['ba'])) <= rad(P['confirm_tol_deg']) and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        count = len(stamps)
        key = (count, c['score'])
        if best is None or key > best[0]:
            best = (key, c)

    if best is not None and best[0][0] >= P['confirm_k']:
        c = best[1]
        track = {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}
        return hist, True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate_ang = rad(P['track_gate_deg'])
    best = None
    for c in cands:
        dba = abs(wrap(c['ba'] - track['ba']))
        if dba <= gate_ang and abs(c['d'] - track['d']) < 0.6:
            if best is None or dba < best[0]:
                best = (dba, c)
    lost_before = (t - track['t']) > P['track_lost_s']
    if best is None:
        return dict(track), lost_before
    c = best[1]
    new = {
        'ba': wrap(track['ba'] + 0.5 * wrap(c['ba'] - track['ba'])),
        'd': c['d'],
        'pos': ((track['pos'][0] + c['pos'][0]) / 2.0, (track['pos'][1] + c['pos'][1]) / 2.0),
        't': t,
    }
    return new, False


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = cam_pos_odom(odom)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        if abs(b) <= rad(P['app_bear_tol_deg']):
            return Command(0.0, 0.0), True
        return Command(0.0, sat(-1.5 * b, 0.4)), False
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < rad(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    pallet_ref = None
    if track is not None:
        pallet_ref = {'ba': track['ba'], 'pos': track['pos']}
    return Command(), pallet_ref, True


def a3e_fail():
    return Command(), True


# -------------------------------------------------------------------- A4 -----
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    A = rad(P['tag_sweep_deg'])
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = cam_pos_odom(odom)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        else:
            ref = odom.yaw
        targets = [ref, wrap(ref + A), wrap(ref - A), ref]
        i = 0
    else:
        targets = list(sweep['targets'])
        i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < rad(3.0):
        cmd = Command()
        i = (i + 1) % len(targets)
    else:
        cmd = Command(0.0, sat(2.0 * err, P['w_tag']))
    return cmd, {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - HALF_PI)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags or []:
        if tg.id == TAG_ID:
            meas = a42_tag_to_pose(tg)
            break

    if est is None:
        if meas is None:
            return None
        return {'px': meas[0], 'py': meas[1], 'yaw': meas[2], 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # --- predict with odometry increment since the last update -------------
    ox, oy, oyaw = est['o']
    dx = odom.x - ox
    dy = odom.y - oy
    dyaw = wrap(odom.yaw - oyaw)
    f = math.cos(oyaw) * dx + math.sin(oyaw) * dy     # forward in previous robot frame
    l = -math.sin(oyaw) * dx + math.cos(oyaw) * dy    # left in previous robot frame
    yaw_e = est['yaw']
    px = est['px'] + f * math.cos(yaw_e) - l * math.sin(yaw_e)
    py = est['py'] + f * math.sin(yaw_e) + l * math.cos(yaw_e)
    yaw = wrap(yaw_e + dyaw)
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'], 'n': est['n'],
           'rej': est.get('rej', 0), 'o': (odom.x, odom.y, odom.yaw)}

    if meas is None:
        return new

    # --- measurement update ------------------------------------------------
    zx, zy, zyaw = meas
    ex = zx - px
    ey = zy - py
    eyaw = wrap(zyaw - yaw)
    zcam = math.hypot(*cam_pos(zx, zy, zyaw))
    gate_ok = (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * zcam and
               abs(eyaw) <= rad(P['gate_yaw_deg']))
    if gate_ok:
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        return {'px': px + k * ex, 'py': py + k * ey, 'yaw': wrap(yaw + k * eyaw),
                't_meas': t, 'n': est['n'] + 1, 'rej': 0, 'o': new['o']}

    rej = est.get('rej', 0) + 1
    if rej >= 5:
        return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': new['o']}
    new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam = math.hypot(*cam_pos(est['px'], est['py'], est['yaw']))
    pose_stale = ((age > P['pose_stale_s'] and cam > P['near_dist']) or
                  age > P['blind_max_s'])
    return tag_ok, pose_ready, pose_stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist) if blacklist is not None else []
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# -------------------------------------------------------------------- A5 -----
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
        feasible = gamma <= beta_max(P) - rad(2.0)
    return Command(), feasible


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), None, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    bmax = beta_max(P)
    if _is_owned(mstate, '52'):
        ms = dict(mstate)
    else:
        yw = max(py, P['y_pre'] + P['repos_extra'])
        xw = math.copysign(1.0, px) * (yw - P['y_pre']) * math.tan(bmax - rad(8.0)) if px != 0.0 else 0.0
        ms = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}

    wx, wy = ms['W']
    dist = math.hypot(wx - px, wy - py)
    to_w = wrap(math.atan2(wy - py, wx - px) - yaw)

    # turn1 -> (skip) turn2 / drive -> turn2: a transition is evaluated in the same tick
    if ms['ph'] == 'turn1':
        if dist < 0.06:
            ms['ph'] = 'turn2'
        elif abs(to_w) < rad(3.0):
            ms['ph'] = 'drive'
        else:
            return Command(0.0, in_place_w(to_w, 0.5)), ms, False

    if ms['ph'] == 'drive':
        if dist <= 0.06:
            ms['ph'] = 'turn2'
        else:
            v = 0.2 * max(0.0, math.cos(to_w))
            w = sat(1.5 * to_w, 0.5)
            return Command(v, w), ms, False

    # turn2: face the tag from the camera position
    cx, cy = cam_pos(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < rad(3.0):
        return Command(), None, True
    return Command(0.0, in_place_w(e, 0.5)), ms, False


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
    h_des = math.atan2(ty - py, -px)
    cx, cy = cam_pos(px, py, yaw)
    th_ct = math.atan2(-cy, -cx)
    bmax = beta_max(P)
    dev = max(-bmax, min(bmax, wrap(h_des - th_ct)))
    h = wrap(th_ct + dev)
    e = wrap(h - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = cam_pos(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= rad(4.0):
        return Command(), None
    return Command(0.0, in_place_w(e, 0.4)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        h_des = -HALF_PI - sat(math.atan(P['k_lat'] * px), rad(30.0))
        e = wrap(h_des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = wrap(-HALF_PI - yaw)
    if abs(e) <= rad(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, sat(in_place_w(e, 0.3), 0.3)), False


def a56_verify(est, mstate, t, P):
    if not _is_owned(mstate, '56'):
        return Command(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    if est is None:
        ok = False
    else:
        ok = (abs(est['px']) <= P['verify_lat'] and
              abs(est['py'] - P['dock_y']) <= P['verify_dist'] and
              abs(wrap(est['yaw'] + HALF_PI)) <= rad(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if not _is_owned(mstate, '57'):
        n_bo = n_bo + 1
        ms = {'own': '57', 'x0': odom.x, 'y0': odom.y}
    else:
        ms = mstate
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - ms['x0'], odom.y - ms['y0'])
    if travelled >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), ms, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True


# ------------------------------------------------- GS (CR-1): safety monitor --
def as1_obstacle_monitor(scan, sstate, t, P):
    """Continuous safety monitor (CR-1, R-10): near return inside the forward sector."""
    st = dict(sstate) if sstate is not None else {}
    near = False
    sector = rad(P['safe_sector_deg'])
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r >= P['safe_dist']:
            continue
        ang = wrap(scan.angle_min + i * scan.angle_increment)
        if abs(ang) <= sector:
            near = True
            break
    if near:
        st['last_near'] = t
    if near:
        path_clear = False
    elif 'last_near' in st:
        path_clear = (t - st['last_near']) >= P['clear_s']
    else:
        path_clear = True
    return st, near, path_clear


def as2_inhibit_forward(cmd):
    """Command override during the safety hold: no forward motion, rotation kept."""
    return Command(min(cmd.v, 0.0), cmd.w)
