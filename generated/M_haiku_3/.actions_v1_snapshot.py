"""Action implementations for the AGV pallet-pocket docking controller (MIAT, A0-A5).

Stateless: everything an action remembers between ticks is passed in and returned.
Only math and sim.api are used.
"""
import math

from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID

# ---------------------------------------------------------------- helpers
_PI = math.pi
_TWO_PI = 2.0 * math.pi


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + _PI, _TWO_PI)
    if a <= 0.0:
        a += _TWO_PI
    return a - _PI


def sat(x, m):
    return max(-m, min(m, x))


def _sign(x):
    return 1.0 if x >= 0.0 else -1.0


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2:
        return float(s[mid])
    return 0.5 * (s[mid - 1] + s[mid])


def _cam_xy(odom):
    """Camera position in the odometry frame."""
    return (odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw))


def _in_place(e, k, w_max):
    """In-place rotation rule of A52 / A54 / A55 (proportional, min speed near zero)."""
    if abs(e) > 0.12:
        return sat(k * e, w_max)
    return 0.15 if e >= 0.0 else -0.15


def _pose_cam(est):
    """Camera position in the pallet frame for pose estimate est."""
    return (est['px'] + CAM_X * math.cos(est['yaw']),
            est['py'] + CAM_X * math.sin(est['yaw']))


# ---------------------------------------------------------------- A0 / G0
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


# ---------------------------------------------------------------- A3 / G3
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
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    cx, cy = _cam_xy(odom)
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
        if any(math.hypot(pos[0] - bx, pos[1] - by) < 0.8 for (bx, by) in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    tol = math.radians(P['confirm_tol_deg'])
    hist = [h for h in hist if t - h['t'] <= window]
    for c in cands:
        hist.append({'t': t, 'c': c})
    if not cands:
        return hist, False, None

    best = None
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(c['ba'] - hc['ba'])) <= tol and abs(c['d'] - hc['d']) <= 0.4:
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
    gate = math.radians(P['track_gate_deg'])
    best = None
    for c in cands:
        dba = wrap(c['ba'] - track['ba'])
        if abs(dba) <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or abs(dba) < best[0]:
                best = (abs(dba), dba, c)
    new = dict(track)
    if best is not None:
        _, dba, c = best
        new['ba'] = wrap(track['ba'] + 0.5 * dba)
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    track_lost = (t - new['t']) > P['track_lost_s']
    return new, track_lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_xy(odom)
    dx = track['pos'][0] - cx
    dy = track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = wrap(odom.yaw - math.atan2(dy, dx))
    tol = math.radians(P['app_bear_tol_deg'])
    if d <= P['d_app']:
        approach_done = abs(b) <= tol
        w = 0.0 if approach_done else sat(-1.5 * b, 0.4)
        return Command(0.0, w), approach_done
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    pallet_ref = None
    if track is not None:
        pallet_ref = {'ba': track['ba'], 'pos': track['pos']}
    return Command(), pallet_ref, True


def a3e_fail():
    return Command(), True


# ---------------------------------------------------------------- A4 / G4
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = math.radians(P['tag_sweep_deg'])
        if pallet_ref is not None:
            cx, cy = _cam_xy(odom)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        targets = [ref, ref + A, ref - A, ref]
        sweep = {'targets': targets, 'i': 0}
    else:
        sweep = dict(sweep)
    targets = sweep['targets']
    i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        cmd = Command()
        sweep['i'] = (i + 1) % len(targets)
    else:
        cmd = Command(0.0, sat(2.0 * err, P['w_tag']))
    return cmd, sweep


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - _PI / 2.0)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in tags if tg.id == TAG_ID]

    if est is None:
        if not meas:
            return None
        px, py, yaw = meas[0]
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment since the last update
    x0, y0, yaw0 = est['o']
    dx = odom.x - x0
    dy = odom.y - y0
    f = math.cos(yaw0) * dx + math.sin(yaw0) * dy     # forward in robot frame
    l = -math.sin(yaw0) * dx + math.cos(yaw0) * dy    # left in robot frame
    e_yaw = est['yaw']
    px = est['px'] + f * math.cos(e_yaw) - l * math.sin(e_yaw)
    py = est['py'] + f * math.sin(e_yaw) + l * math.cos(e_yaw)
    yaw = wrap(e_yaw + wrap(odom.yaw - yaw0))
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'],
           'o': (odom.x, odom.y, odom.yaw)}

    if not meas:
        return new

    zx, zy, zyaw = meas[0]
    ex = zx - px
    ey = zy - py
    eyaw = wrap(zyaw - yaw)
    cam_d = math.hypot(px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))
    gate_pos = P['gate_pos'] + 0.1 * cam_d
    gate_yaw = math.radians(P['gate_yaw_deg'])

    if math.hypot(ex, ey) <= gate_pos and abs(eyaw) <= gate_yaw:
        k = 0.5 if new['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = wrap(yaw + k * eyaw)
        new['t_meas'] = t
        new['n'] = new['n'] + 1
        new['rej'] = 0
        return new

    new['rej'] = new['rej'] + 1
    if new['rej'] >= 5:
        return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam_d = math.hypot(*_pose_cam(est))
    pose_stale = ((age > P['pose_stale_s'] and cam_d > P['near_dist'])
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


# ---------------------------------------------------------------- A5 / G5
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
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    return Command(), gamma <= beta_max - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), None, False
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    y_pre = P['y_pre']
    px, py, yaw = est['px'], est['py'], est['yaw']

    if mstate is None or mstate.get('own') != '52':
        yw = max(py, y_pre + P['repos_extra'])
        xw = _sign(px) * (yw - y_pre) * math.tan(beta_max - math.radians(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}

    xw, yw = mstate['W']
    phase = mstate['ph']
    dist_w = math.hypot(xw - px, yw - py)

    if phase == 'turn1':
        if dist_w <= 0.06:
            phase = 'turn2'
        else:
            target = math.atan2(yw - py, xw - px)
            e = wrap(target - yaw)
            if abs(e) < math.radians(3.0):
                phase = 'drive'
            else:
                mstate = dict(mstate, ph='turn1')
                return Command(0.0, _in_place(e, 1.8, 0.5)), mstate, False

    if phase == 'drive':
        if dist_w <= 0.06:
            phase = 'turn2'
        else:
            target = math.atan2(yw - py, xw - px)
            e = wrap(target - yaw)
            v = 0.2 * max(0.0, math.cos(e))
            w = sat(1.5 * e, 0.5)
            mstate = dict(mstate, ph='drive')
            return Command(v, w), mstate, False

    # turn2: face the tag from the camera position
    cx = px + CAM_X * math.cos(yaw)
    cy = py + CAM_X * math.sin(yaw)
    target = math.atan2(-cy, -cx)
    e = wrap(target - yaw)
    if abs(e) < math.radians(3.0):
        return Command(), None, True
    mstate = dict(mstate, ph='turn2')
    return Command(0.0, _in_place(e, 1.8, 0.5)), mstate, False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    y_pre = P['y_pre']
    if abs(px) <= 0.05 and py <= y_pre + 0.08:
        return Command(), True, False
    if py < y_pre - 0.25 and abs(px) > 0.10:
        return Command(), False, True

    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    ty = y_pre if py > y_pre else py - 0.3
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _pose_cam(est)
    ref = math.atan2(-cy, -cx)
    hd = ref + sat(wrap(hd - ref), beta_max)
    e = wrap(hd - yaw)
    dist = math.hypot(px, py - y_pre)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _pose_cam(est)
    ref = math.atan2(-cy, -cx)
    e = wrap(ref - est['yaw'])
    if abs(e) <= math.radians(4.0):
        return Command(), None
    w = sat(1.8 * e, 0.4) if abs(e) > 0.12 else (0.15 if e >= 0.0 else -0.15)
    return Command(0.0, w), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -_PI / 2.0 - sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = wrap(-_PI / 2.0 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _in_place(e, 1.8, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(wrap(est['yaw'] + _PI / 2.0)) <= math.radians(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
        n_bo = n_bo + 1
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if travelled >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True
