"""Action functions of the AGV pallet-pocket docking controller (MIAT A-level).

Implements every function of ACTION_CONTRACTS.md. All memory between ticks is
passed in and returned explicitly; there is no module-level mutable state, no
loop that waits, no sleep and no randomness.  Only `math` and `sim.api` are used.
"""
import math

from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID

PI = math.pi
DEG = math.pi / 180.0


# ---------------------------------------------------------------- helpers ----
def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    r = math.atan2(math.sin(a), math.cos(a))
    if r <= -PI:
        r += 2.0 * PI
    return r


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _sgn(x):
    return (x > 0) - (x < 0)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    m = n // 2
    if n % 2:
        return float(s[m])
    return 0.5 * (s[m - 1] + s[m])


def _cam_xy(x, y, yaw):
    """Camera optical centre for a base_link pose (same frame as the pose)."""
    return (x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw))


def _in_place(e, gain, lim):
    """In-place rotation rule (A52): sat(gain*e, lim), or +-0.15 near zero error."""
    if abs(e) > 0.12:
        return _sat(gain * e, lim)
    return math.copysign(min(0.15, lim), e)


def _bmax(P):
    """Largest bearing (rad) at which the tag stays inside the FOV with margin."""
    return (P['fov_half_deg'] - P['fov_margin_deg']) * DEG


def _tag_dir(px, py, yaw):
    """Direction (rad) from the camera of pose (px,py,yaw) to the tag at the origin."""
    cx, cy = _cam_xy(px, py, yaw)
    return math.atan2(-cy, -cx)


# ------------------------------------------------------- A0 / A1 / A2 / A0 ---
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


# ------------------------------------------------------------- A3 approach --
def a30_enter():
    return Command(), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, P['w_search'])


def a331_bbox_center(det):
    return float(det.u)


def a332_pallet_distance(depth_samples):
    valid = [s for s in depth_samples if s > 0.05 and math.isfinite(s)]
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
    cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
    bl = list(blacklist or [])
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
        px = cx + d * math.cos(ba)
        py = cy + d * math.sin(ba)
        if any(math.hypot(px - bx, py - by) <= 0.8 for (bx, by) in bl):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': (px, py)})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    tol = P['confirm_tol_deg'] * DEG
    hist = [h for h in (hist or []) if t - h['t'] <= window]
    for c in cands or []:
        hist.append({'t': t, 'c': c})

    best, best_key = None, None
    for c in cands or []:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps.add(h['t'])
        key = (len(stamps), c['score'])
        if best_key is None or key > best_key:
            best, best_key = c, key

    if best is not None and best_key[0] >= P['confirm_k']:
        track = {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
        return hist, True, track
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = P['track_gate_deg'] * DEG
    best, best_diff = None, None
    for c in cands or []:
        dba = _wrap(c['ba'] - track['ba'])
        if abs(dba) <= gate and abs(c['d'] - track['d']) < 0.6:
            if best is None or abs(dba) < best_diff:
                best, best_diff = c, abs(dba)
    if best is None:
        new = dict(track)
    else:
        ba = _wrap(track['ba'] + 0.5 * _wrap(best['ba'] - track['ba']))
        pos = ((track['pos'][0] + best['pos'][0]) / 2.0,
               (track['pos'][1] + best['pos'][1]) / 2.0)
        new = {'ba': ba, 'd': best['d'], 'pos': pos, 't': t}
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = _wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        if abs(b) <= P['app_bear_tol_deg'] * DEG:
            return Command(0.0, 0.0), True
        return Command(0.0, _sat(-1.5 * b, 0.4)), False
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < 10 * DEG else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    ref = None if track is None else {'ba': track['ba'], 'pos': track['pos']}
    return Command(), ref, True


def a3e_fail():
    return Command(), True


# ------------------------------------------------------ A4 localisation -----
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    return (Command(), n_loc, n_loc <= P['max_retries'],
            None, None, False, False, True)


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam_xy(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        else:
            ref = odom.yaw
        A = P['tag_sweep_deg'] * DEG
        targets = [ref, ref + A, ref - A, ref]
        i = 0
    else:
        targets = list(sweep['targets'])
        i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < 3 * DEG:
        return Command(), {'targets': targets, 'i': (i + 1) % len(targets)}
    return Command(0.0, _sat(2.0 * err, P['w_tag'])), {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = _wrap(tag.yaw - PI / 2)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def _fresh_est(zpose, t, odom):
    zx, zy, zyaw = zpose
    return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
            'o': (odom.x, odom.y, odom.yaw)}


def a43_update_pose(est, odom, tags, t, P):
    zs = [a42_tag_to_pose(tg) for tg in (tags or []) if tg.id == TAG_ID]

    if est is None:
        return _fresh_est(zs[0], t, odom) if zs else None

    # predict: odometry increment expressed in the robot frame at the previous
    # odometry heading, applied in the pallet frame at the estimated heading
    x0, y0, yaw0 = est['o']
    dx = odom.x - x0
    dy = odom.y - y0
    f = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    l = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    e_yaw = est['yaw']
    px = est['px'] + f * math.cos(e_yaw) - l * math.sin(e_yaw)
    py = est['py'] + f * math.sin(e_yaw) + l * math.cos(e_yaw)
    yaw = _wrap(e_yaw + _wrap(odom.yaw - yaw0))
    n, rej, t_meas = est['n'], est['rej'], est['t_meas']
    o = (odom.x, odom.y, odom.yaw)

    if zs:
        zx, zy, zyaw = zs[0]
        ex, ey = zx - px, zy - py
        eyaw = _wrap(zyaw - yaw)
        cdist = math.hypot(*_cam_xy(zx, zy, zyaw))   # camera distance of the measurement
        if (math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cdist
                and abs(eyaw) <= P['gate_yaw_deg'] * DEG):
            k = 0.5 if n < P['init_meas'] else P['fuse_gain']
            px += k * ex
            py += k * ey
            yaw = _wrap(yaw + k * eyaw)
            t_meas = t
            n += 1
            rej = 0
        else:
            rej += 1
            if rej >= 5:
                new = _fresh_est(zs[0], t, odom)
                return new

    return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t_meas, 'n': n,
            'rej': rej, 'o': o}


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = bool(tag_ok and est['n'] >= P['init_meas'])
    cdist = math.hypot(*_cam_xy(est['px'], est['py'], est['yaw']))
    pose_stale = bool((age > P['pose_stale_s'] and cdist > P['near_dist'])
                      or age > P['blind_max_s'])
    return bool(tag_ok), pose_ready, pose_stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ------------------------------------------------------- A5 guidance --------
def a50_enter():
    return Command(), None, 0, True, False, False, True


def a51_plan(est, P):
    stop = Command()
    if est is None:
        return stop, False
    px, py = est['px'], est['py']
    yp = P['y_pre']
    if abs(px) <= 0.05 and abs(py - yp) <= 0.15:
        return stop, True
    if py <= yp + 0.15:
        return stop, False
    gamma = math.atan2(abs(px), py - yp)
    return stop, gamma <= _bmax(P) - 2 * DEG


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']

    if mstate is None or mstate.get('owner') != '52':
        yw = max(py, yp + P['repos_extra'])
        xw = _sgn(px) * (yw - yp) * math.tan(_bmax(P) - 8 * DEG)
        ms = {'owner': '52', 'phase': 'turn1', 'W': (xw, yw)}
    else:
        ms = dict(mstate)
    xw, yw = ms['W']
    phase = ms['phase']

    if phase == 'turn1':
        if math.hypot(xw - px, yw - py) < 0.06:
            phase = 'turn2'
        else:
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            if abs(e) >= 3 * DEG:
                ms['phase'] = 'turn1'
                return Command(0.0, _in_place(e, 1.8, 0.5)), ms, False
            phase = 'drive'

    if phase == 'drive':
        dist = math.hypot(xw - px, yw - py)
        if dist >= 0.06:
            e = _wrap(math.atan2(yw - py, xw - px) - yaw)
            ms['phase'] = 'drive'
            return (Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5)),
                    ms, False)
        phase = 'turn2'

    # turn2: face the tag from the camera position
    e = _wrap(_tag_dir(px, py, yaw) - yaw)
    if abs(e) < 3 * DEG:
        return Command(), None, True
    ms['phase'] = 'turn2'
    return Command(0.0, _in_place(e, 1.8, 0.5)), ms, False


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
    dir_cam = _tag_dir(px, py, yaw)
    diff = _wrap(hd - dir_cam)
    bm = _bmax(P)
    if abs(diff) > bm:
        hd = _wrap(dir_cam + _sgn(diff) * bm)
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    e = _wrap(_tag_dir(est['px'], est['py'], est['yaw']) - est['yaw'])
    if abs(e) <= 4 * DEG:
        return Command(), None
    return Command(0.0, _in_place(e, 1.8, 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -PI / 2 - _sat(math.atan(P['k_lat'] * px), 30 * DEG)
        e = _wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-PI / 2 - yaw)
    if abs(e) <= P['head_tol_deg'] * DEG:
        return Command(), True
    return Command(0.0, _in_place(e, 1.8, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('owner') != '56':
        mstate = {'owner': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    if est is None:
        ok = False
    else:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(_wrap(est['yaw'] + PI / 2)) <= P['verify_head_deg'] * DEG)
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
