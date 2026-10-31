"""Action functions for the AGV pallet-pocket docking controller (MIAT A0..A5).

Stateless: everything an action remembers between ticks is passed in and returned.
Only math and sim.api are used.
"""
import math

from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID

DEG = math.pi / 180.0
TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------- helpers ----
def _wrap(a):
    """Wrap angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _cam_xy(x, y, yaw):
    """Camera optical centre from base_link pose."""
    return x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)


def _rot_cmd(e, wmax):
    """In-place rotation rule (A52): proportional, with a minimum creep rate."""
    if abs(e) > 0.12:
        return _sat(1.8 * e, wmax)
    return math.copysign(0.15, e)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


# ------------------------------------------------------- A0 (G0) actions -----
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


# ------------------------------------------------------- A3 (G3) actions -----
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [float(s) for s in depth_samples if math.isfinite(s) and s > 0.05]
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
    cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
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
        ba = _wrap(odom.yaw - b)
        px = cx + d * math.cos(ba)
        py = cy + d * math.sin(ba)
        if any(math.hypot(px - bp[0], py - bp[1]) <= 0.8 for bp in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': (px, py)})
    return cands


def a322_confirm(cands, hist, t, P):
    hist = [h for h in hist if t - h['t'] <= P['confirm_window_s']]
    hist = hist + [{'t': t, 'c': c} for c in cands]
    if not cands:
        return hist, False, None
    tol = P['confirm_tol_deg'] * DEG
    best = None
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best is None or key > best[0]:
            best = (key, c)
    (count, _), c = best
    if count >= P['confirm_k']:
        track = {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}
        return hist, True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = P['track_gate_deg'] * DEG
    best = None
    for c in cands:
        dba = _wrap(c['ba'] - track['ba'])
        if abs(dba) <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or abs(dba) < abs(best[0]):
                best = (dba, c)
    new = dict(track)
    if best is not None:
        dba, c = best
        new['ba'] = _wrap(track['ba'] + 0.5 * dba)
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    dx, dy = px - cx, py - cy
    d = math.hypot(dx, dy)
    b = _wrap(odom.yaw - math.atan2(dy, dx))
    if d <= P['d_app']:
        w = _sat(-1.5 * b, 0.4)
        done = abs(b) <= P['app_bear_tol_deg'] * DEG
        if done:
            w = 0.0
        return Command(0.0, w), done
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < 10.0 * DEG else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    ref = None if track is None else {'ba': track['ba'], 'pos': track['pos']}
    return Command(), ref, True


def a3e_fail():
    return Command(), True


# ------------------------------------------------------- A4 (G4) actions -----
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = P['tag_sweep_deg'] * DEG
        if pallet_ref is None:
            ref = odom.yaw
        else:
            cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        sweep = {'targets': [ref, ref + A, ref - A, ref], 'i': 0}
    targets = list(sweep['targets'])
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < 3.0 * DEG:
        return Command(), {'targets': targets, 'i': (i + 1) % len(targets)}
    return Command(0.0, _sat(2.0 * err, P['w_tag'])), {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - math.pi / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = None
    for tg in tags:
        if tg.id == TAG_ID:
            meas = a42_tag_to_pose(tg)
            break

    if est is None:
        if meas is None:
            return None
        px, py, yaw = meas
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment since last update
    ox, oy, oyaw = est['o']
    dx = odom.x - ox
    dy = odom.y - oy
    dyaw = _wrap(odom.yaw - oyaw)
    c, s = math.cos(oyaw), math.sin(oyaw)
    f = c * dx + s * dy          # forward in previous robot frame
    l = -s * dx + c * dy         # left in previous robot frame
    h = est['yaw']
    ch, sh = math.cos(h), math.sin(h)
    px = est['px'] + f * ch - l * sh
    py = est['py'] + f * sh + l * ch
    yaw = _wrap(h + dyaw)
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'], 'o': (odom.x, odom.y, odom.yaw)}

    if meas is None:
        return new

    zx, zy, zyaw = meas
    ex = zx - px
    ey = zy - py
    eyaw = _wrap(zyaw - yaw)
    cdist = math.hypot(*_cam_xy(zx, zy, zyaw))
    gate_ok = (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cdist
               and abs(eyaw) <= P['gate_yaw_deg'] * DEG)
    if gate_ok:
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = _wrap(yaw + k * eyaw)
        new['t_meas'] = t
        new['n'] = est['n'] + 1
        new['rej'] = 0
        return new

    rej = est['rej'] + 1
    if rej >= 5:
        return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cdist = math.hypot(*_cam_xy(est['px'], est['py'], est['yaw']))
    pose_stale = ((age > P['pose_stale_s'] and cdist > P['near_dist'])
                  or age > P['blind_max_s'])
    return tag_ok, pose_ready, pose_stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist)
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ------------------------------------------------------- A5 (G5) actions -----
def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return Command(), False
    px, py = est['px'], est['py']
    yp = P['y_pre']
    if abs(px) <= 0.05 and abs(py - yp) <= 0.15:
        feasible = True
    elif py <= yp + 0.15:
        feasible = False
    else:
        gamma = math.atan2(abs(px), py - yp)
        beta_max = P['fov_half_deg'] - P['fov_margin_deg']
        feasible = gamma <= (beta_max - 2.0) * DEG
    return Command(), feasible


def a52_reposition(est, mstate, P):
    """Odometry manoeuvre. Pallet-frame waypoint is mapped into the odometry
    frame once at entry (odom->pallet rigid transform from est and est['o'])."""
    if est is None:
        return Command(), None, False
    ox, oy, oyaw = est['o']          # current odometry pose (set by a43 this tick)

    if mstate is None or mstate.get('owner') != '52':
        px, py, yaw = est['px'], est['py'], est['yaw']
        yp = P['y_pre']
        beta_max = P['fov_half_deg'] - P['fov_margin_deg']
        yw = max(py, yp + P['repos_extra'])
        sx = (px > 0) - (px < 0)
        xw = sx * (yw - yp) * math.tan((beta_max - 8.0) * DEG)
        # odom -> pallet transform: pallet = R(th)*odom + tr
        th = yaw - oyaw
        c, s = math.cos(th), math.sin(th)
        tr = (px - (c * ox - s * oy), py - (s * ox + c * oy))

        def to_odom(X, Y):
            dx, dy = X - tr[0], Y - tr[1]
            return math.cos(-th) * dx - math.sin(-th) * dy, math.sin(-th) * dx + math.cos(-th) * dy

        mstate = {'owner': '52', 'phase': 'turn1',
                  'W': to_odom(xw, yw), 'tag': to_odom(0.0, 0.0)}
    else:
        mstate = dict(mstate)

    W = mstate['W']
    tag = mstate['tag']
    phase = mstate['phase']
    dist_w = math.hypot(W[0] - ox, W[1] - oy)

    if phase == 'turn1':
        if dist_w < 0.06:
            phase = 'turn2'
        else:
            e = _wrap(math.atan2(W[1] - oy, W[0] - ox) - oyaw)
            if abs(e) < 3.0 * DEG:
                phase = 'drive'
            else:
                mstate['phase'] = 'turn1'
                return Command(0.0, _rot_cmd(e, 0.5)), mstate, False

    if phase == 'drive':
        if dist_w < 0.06:
            phase = 'turn2'
        else:
            e = _wrap(math.atan2(W[1] - oy, W[0] - ox) - oyaw)
            mstate['phase'] = 'drive'
            return (Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5)),
                    mstate, False)

    # turn2: face the camera toward the tag
    cx, cy = _cam_xy(ox, oy, oyaw)
    e = _wrap(math.atan2(tag[1] - cy, tag[0] - cx) - oyaw)
    if abs(e) < 3.0 * DEG:
        return Command(), None, True
    mstate['phase'] = 'turn2'
    return Command(0.0, _rot_cmd(e, 0.5)), mstate, False


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
    cx, cy = _cam_xy(px, py, yaw)
    direction = math.atan2(-cy, -cx)
    beta = (P['fov_half_deg'] - P['fov_margin_deg']) * DEG
    dh = _sat(_wrap(hd - direction), beta)
    hd = direction + dh
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _cam_xy(px, py, yaw)
    e = _wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= 4.0 * DEG:
        return Command(), None
    return Command(0.0, _rot_cmd(e, 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -math.pi / 2 - _sat(math.atan(P['k_lat'] * px), 30.0 * DEG)
        e = _wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-math.pi / 2 - yaw)
    if abs(e) <= P['head_tol_deg'] * DEG:
        return Command(), True
    return Command(0.0, _rot_cmd(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('owner') != '56':
        mstate = {'owner': '56', 't0': t}
    if (t - mstate['t0']) < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + math.pi / 2)) <= P['verify_head_deg'] * DEG)
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('owner') != '57':
        n_bo = n_bo + 1
        mstate = {'owner': '57', 'x0': odom.x, 'y0': odom.y}
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if travelled >= P['backoff_dist'] - 1e-9:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
