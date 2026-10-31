"""Action implementations for the AGV pallet-pocket docking controller (MIAT, A0-A5).

Stateless: everything that must persist between ticks is passed in and returned.
Only `math` and `sim.api` are used.
"""
import math

from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID


# ---------------------------------------------------------------- helpers
def _wrap(a):
    """Wrap angle to (-pi, pi]."""
    r = (a + math.pi) % (2.0 * math.pi) - math.pi
    if r <= -math.pi:
        r += 2.0 * math.pi
    return r


def _sat(x, m):
    return max(-m, min(m, x))


def _sign(x):
    return math.copysign(1.0, x)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2 == 1:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _cam(x, y, yaw):
    """Camera position of a base_link pose (x, y, yaw)."""
    return (x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw))


def _dir(x0, y0, x1, y1):
    return math.atan2(y1 - y0, x1 - x0)


def _inplace_w(e, wmax):
    """In-place rotation rule shared by several actions."""
    if abs(e) > 0.12:
        return _sat(1.8 * e, wmax)
    return 0.15 if e >= 0.0 else -0.15


# ---------------------------------------------------------------- A0 / A1 / A2 / A6 / A9
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


# ---------------------------------------------------------------- A3 (approach)
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
    return math.atan((u - IMG_W / 2.0) / FX)


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
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
        cx, cy = _cam(odom.x, odom.y, odom.yaw)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - bx, pos[1] - by) < 0.8 for (bx, by) in (blacklist or [])):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W, 'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    tol = math.radians(P['confirm_tol_deg'])
    hist = [h for h in (hist or []) if t - h['t'] <= window]
    hist = hist + [{'t': t, 'c': c} for c in cands]
    if not cands:
        return hist, False, None

    best = None
    best_key = None
    for c in cands:
        stamps = {}
        for h in hist:
            hc = h['c']
            if abs(_wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
                stamps[h['t']] = True
        count = len(stamps)
        key = (count, c['score'])
        if best_key is None or key > best_key:
            best_key = key
            best = c
    if best_key[0] >= P['confirm_k']:
        track = {'ba': best['ba'], 'd': best['d'], 'pos': best['pos'], 't': t}
        return hist, True, track
    return hist, False, None


# ---------------------------------------------------------------- A3 (tracking / approach)
def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(P['track_gate_deg'])
    gated = [c for c in (cands or [])
             if abs(_wrap(c['ba'] - track['ba'])) <= gate and abs(c['d'] - track['d']) < 0.6]
    new = dict(track)
    if gated:
        c = min(gated, key=lambda c: abs(_wrap(c['ba'] - track['ba'])))
        new['ba'] = _wrap(track['ba'] + 0.5 * _wrap(c['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    track_lost = (t - new['t']) > P['track_lost_s']
    return new, track_lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam(odom.x, odom.y, odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = _wrap(odom.yaw - math.atan2(py - cy, px - cx))
    tol = math.radians(P['app_bear_tol_deg'])
    if d <= P['d_app']:
        w = _sat(-1.5 * b, 0.4)
        done = abs(b) <= tol
        if done:
            w = 0.0
        return Command(0.0, w), done
    w = _sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < math.radians(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ---------------------------------------------------------------- A4 (localize tag)
def a40_enter(n_loc, est, P):
    n = n_loc + 1
    retry_ok = n <= P['max_retries']
    return Command(), n, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        A = math.radians(P['tag_sweep_deg'])
        if pallet_ref is not None:
            cx, cy = _cam(odom.x, odom.y, odom.yaw)
            ref = _dir(cx, cy, pallet_ref['pos'][0], pallet_ref['pos'][1])
        else:
            ref = odom.yaw
        targets = [_wrap(ref), _wrap(ref + A), _wrap(ref - A), _wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    targets = sweep['targets']
    i = sweep['i']
    err = _wrap(targets[i] - odom.yaw)
    if abs(err) < math.radians(3.0):
        return Command(), {'targets': targets, 'i': (i + 1) % len(targets)}
    w = _sat(2.0 * err, P['w_tag'])
    return Command(0.0, w), {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    x, z = tag.x, tag.z
    yaw = _wrap(tag.yaw - math.pi / 2.0)
    cx = -(z * math.cos(yaw) + x * math.sin(yaw))
    cy = -(z * math.sin(yaw) - x * math.cos(yaw))
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

    # predict with odometry increment since the last update
    x0, y0, yaw0 = est['o']
    dx = odom.x - x0
    dy = odom.y - y0
    f = math.cos(yaw0) * dx + math.sin(yaw0) * dy     # forward in previous robot frame
    l = -math.sin(yaw0) * dx + math.cos(yaw0) * dy    # left in previous robot frame
    e = est['yaw']
    px = est['px'] + f * math.cos(e) - l * math.sin(e)
    py = est['py'] + f * math.sin(e) + l * math.cos(e)
    yaw = _wrap(e + _wrap(odom.yaw - yaw0))

    n = est['n']
    rej = est['rej']
    t_meas = est['t_meas']
    if meas is not None:
        zx, zy, zyaw = meas
        ex = zx - px
        ey = zy - py
        eyaw = _wrap(zyaw - yaw)
        cx, cy = _cam(px, py, yaw)
        cd = math.hypot(cx, cy)
        if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cd and abs(eyaw) <= math.radians(P['gate_yaw_deg']):
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
                px, py, yaw = zx, zy, zyaw
                t_meas = t
                n = 1
                rej = 0

    return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t_meas, 'n': n, 'rej': rej,
            'o': (odom.x, odom.y, odom.yaw)}


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cx, cy = _cam(est['px'], est['py'], est['yaw'])
    cd = math.hypot(cx, cy)
    pose_stale = (age > P['pose_stale_s'] and cd > P['near_dist']) or age > P['blind_max_s']
    return tag_ok, pose_ready, pose_stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref['pos']))
    return Command(), bl, True


# ---------------------------------------------------------------- A5 (guidance)
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
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    gamma = math.atan2(abs(px), py - yp)
    return Command(), gamma <= beta_max - math.radians(2.0)


def a52_reposition(est, mstate, P):
    if est is None:
        return Command(), mstate, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])

    if mstate is None or mstate.get('own') != '52':
        yw = max(py, yp + P['repos_extra'])
        xw = _sign(px) * (yw - yp) * math.tan(beta_max - math.radians(8.0))
        phase = 'turn1'
        if math.hypot(xw - px, yw - py) <= 0.06:
            phase = 'turn2'
        mstate = {'own': '52', 'ph': phase, 'W': (xw, yw)}

    xw, yw = mstate['W']
    phase = mstate['ph']

    if phase == 'turn1':
        if math.hypot(xw - px, yw - py) <= 0.06:
            phase = 'turn2'
        else:
            e = _wrap(_dir(px, py, xw, yw) - yaw)
            if abs(e) < math.radians(3.0):
                phase = 'drive'
                mstate = {'own': '52', 'ph': phase, 'W': (xw, yw)}
                return Command(), mstate, False
            mstate = {'own': '52', 'ph': phase, 'W': (xw, yw)}
            return Command(0.0, _inplace_w(e, 0.5)), mstate, False

    if phase == 'drive':
        dist = math.hypot(xw - px, yw - py)
        if dist <= 0.06:
            phase = 'turn2'
        else:
            e = _wrap(_dir(px, py, xw, yw) - yaw)
            v = 0.2 * max(0.0, math.cos(e))
            w = _sat(1.5 * e, 0.5)
            mstate = {'own': '52', 'ph': phase, 'W': (xw, yw)}
            return Command(v, w), mstate, False

    # turn2: face the tag (camera -> origin)
    cx, cy = _cam(px, py, yaw)
    e = _wrap(_dir(cx, cy, 0.0, 0.0) - yaw)
    if abs(e) < math.radians(3.0):
        return Command(), None, True
    mstate = {'own': '52', 'ph': 'turn2', 'W': (xw, yw)}
    return Command(0.0, _inplace_w(e, 0.5)), mstate, False


def a53_converge(est, P):
    if est is None:
        return Command(), False, False
    px, py, yaw = est['px'], est['py'], est['yaw']
    yp = P['y_pre']
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return Command(), True, False
    if py < yp - 0.25 and abs(px) > 0.10:
        return Command(), False, True

    beta_max = math.radians(P['fov_half_deg'] - P['fov_margin_deg'])
    ty = yp if py > yp else py - 0.3
    hd = _dir(px, py, 0.0, ty)
    cx, cy = _cam(px, py, yaw)
    ref = _dir(cx, cy, 0.0, 0.0)
    diff = max(-beta_max, min(beta_max, _wrap(hd - ref)))
    hd = _wrap(ref + diff)
    e = _wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = _sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _cam(px, py, yaw)
    e = _wrap(_dir(cx, cy, 0.0, 0.0) - yaw)
    if abs(e) <= math.radians(4.0):
        return Command(), None
    return Command(0.0, _inplace_w(e, 0.4)), None


def a55_final_approach(est, P):
    if est is None:
        return Command(), False
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -math.pi / 2.0 - _sat(math.atan(P['k_lat'] * px), math.radians(30.0))
        e = _wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = _sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = _wrap(-math.pi / 2.0 - yaw)
    if abs(e) <= math.radians(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _inplace_w(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = (est is not None
          and abs(est['px']) <= P['verify_lat']
          and abs(est['py'] - P['dock_y']) <= P['verify_dist']
          and abs(_wrap(est['yaw'] + math.pi / 2.0)) <= math.radians(P['verify_head_deg']))
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
