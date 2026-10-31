"""Action implementations for the AGV pallet-pocket docking controller.

Implements every Action of ACTION_CONTRACTS.md. Stateless: everything that must
persist between ticks is passed in and returned explicitly. Only math and
sim.api are used.
"""
import math

from sim.api import Command, CAM_X, IMG_W, FX, TAG_ID


# ---------------------------------------------------------------- helpers
def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def sat(x, m):
    return max(-m, min(m, x))


def _sign(x):
    return (x > 0) - (x < 0)


def _rot_in_place(e, wmax):
    """In-place rotation rule: proportional above 0.12 rad, else a fixed-rate step."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(0.15, e)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2 == 1:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _cam_pos(x, y, yaw):
    """Camera position from a base_link pose (x, y, yaw)."""
    return (x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw))


def _deg(x):
    return math.radians(x)


# ---------------------------------------------------------- A0 / A1 / A2 / A6 / A9
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


# ------------------------------------------------------------------ A3
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
    cam = _cam_pos(odom.x, odom.y, odom.yaw)
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
        pos = (cam[0] + d * math.cos(ba), cam[1] + d * math.sin(ba))
        if any(math.hypot(pos[0] - bx, pos[1] - by) < 0.8 for (bx, by) in blacklist):
            continue
        cands.append({'ba': ba, 'd': d, 'b': b, 'W': W,
                      'score': det.score, 'pos': pos})
    return cands


def a322_confirm(cands, hist, t, P):
    window = P['confirm_window_s']
    tol = _deg(P['confirm_tol_deg'])
    hist = [h for h in hist if t - h['t'] <= window]
    for c in cands:
        hist.append({'t': t, 'c': c})
    if not cands:
        return hist, False, None

    def count(c):
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(c['ba'] - hc['ba'])) <= tol and abs(c['d'] - hc['d']) <= 0.4:
                stamps.add(h['t'])
        return len(stamps)

    best = None
    for c in cands:
        key = (count(c), c['score'])
        if best is None or key > best[0]:
            best = (key, c)
    (n_best, _), c = best
    if n_best >= P['confirm_k']:
        return hist, True, {'ba': c['ba'], 'd': c['d'], 'pos': c['pos'], 't': t}
    return hist, False, None


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate_ang = _deg(P['track_gate_deg'])
    gated = [c for c in cands
             if abs(wrap(c['ba'] - track['ba'])) <= gate_ang
             and abs(c['d'] - track['d']) < 0.6]
    new = dict(track)
    if gated:
        c = min(gated, key=lambda c: abs(wrap(c['ba'] - track['ba'])))
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(c['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + c['pos'][0]) / 2.0,
                      (track['pos'][1] + c['pos'][1]) / 2.0)
        new['d'] = c['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s'] if not gated else (t - track['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = _cam_pos(odom.x, odom.y, odom.yaw)
    dx = track['pos'][0] - cx
    dy = track['pos'][1] - cy
    d = math.hypot(dx, dy)
    b = wrap(odom.yaw - math.atan2(dy, dx))
    if d <= P['d_app']:
        tol = _deg(P['app_bear_tol_deg'])
        done = abs(b) <= tol
        w = 0.0 if done else sat(-1.5 * b, 0.4)
        return Command(0.0, w), done
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < _deg(10.0) else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ------------------------------------------------------------------ A4
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam_pos(odom.x, odom.y, odom.yaw)
            ref = math.atan2(pallet_ref['pos'][1] - cy, pallet_ref['pos'][0] - cx)
        else:
            ref = odom.yaw
        A = _deg(P['tag_sweep_deg'])
        targets = [wrap(ref), wrap(ref + A), wrap(ref - A), wrap(ref)]
        sweep = {'targets': targets, 'i': 0}
    else:
        sweep = dict(sweep)
    targets = sweep['targets']
    i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < _deg(3.0):
        cmd = Command()
        sweep['i'] = (i + 1) % len(targets)
    else:
        cmd = Command(0.0, sat(2.0 * err, P['w_tag']))
    return cmd, sweep


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - math.pi / 2.0)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = [a42_tag_to_pose(tg) for tg in (tags or []) if tg.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        zx, zy, zyaw = meas[0]
        return {'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # Predict with the odometry increment since the last update.
    ox, oy, oyaw = est['o']
    dx = odom.x - ox
    dy = odom.y - oy
    f = math.cos(oyaw) * dx + math.sin(oyaw) * dy
    l = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
    ye = est['yaw']
    px = est['px'] + f * math.cos(ye) - l * math.sin(ye)
    py = est['py'] + f * math.sin(ye) + l * math.cos(ye)
    yaw = wrap(ye + wrap(odom.yaw - oyaw))
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'],
           'n': est['n'], 'rej': est['rej'],
           'o': (odom.x, odom.y, odom.yaw)}

    if not meas:
        return new

    zx, zy, zyaw = meas[0]
    ex = zx - px
    ey = zy - py
    eyaw = wrap(zyaw - yaw)
    cam_d = math.hypot(zx + CAM_X * math.cos(zyaw), zy + CAM_X * math.sin(zyaw))
    if math.hypot(ex, ey) <= P['gate_pos'] + 0.1 * cam_d and abs(eyaw) <= _deg(P['gate_yaw_deg']):
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = wrap(yaw + k * eyaw)
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
    cam_d = math.hypot(est['px'] + CAM_X * math.cos(est['yaw']),
                       est['py'] + CAM_X * math.sin(est['yaw']))
    pose_stale = ((age > P['pose_stale_s'] and cam_d > P['near_dist'])
                  or age > P['blind_max_s'])
    return tag_ok, pose_ready, pose_stale


def a42_hold():
    return Command()


def a43_publish(est):
    return est is not None


def a4e_blacklist(blacklist, pallet_ref):
    new = list(blacklist) if blacklist is not None else []
    if pallet_ref is not None:
        new.append(tuple(pallet_ref['pos']))
    return Command(), new, True


# ------------------------------------------------------------------ A5
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
    beta_max = _deg(P['fov_half_deg'] - P['fov_margin_deg'])
    return Command(), gamma <= beta_max - _deg(2.0)


def a52_reposition(est, mstate, P):
    yp = P['y_pre']
    beta_max = _deg(P['fov_half_deg'] - P['fov_margin_deg'])
    if mstate is None or mstate.get('own') != '52':
        px, py = est['px'], est['py']
        yw = max(py, yp + P['repos_extra'])
        xw = _sign(px) * (yw - yp) * math.tan(beta_max - _deg(8.0))
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}
    phase = mstate['ph']
    xw, yw = mstate['W']
    px, py, yaw = est['px'], est['py'], est['yaw']
    dist_w = math.hypot(xw - px, yw - py)

    def _state(ph):
        return {'own': '52', 'ph': ph, 'W': (xw, yw)}

    if phase == 'turn1':
        if dist_w < 0.06:
            phase = 'turn2'
        else:
            e = wrap(math.atan2(yw - py, xw - px) - yaw)
            if abs(e) < _deg(3.0):
                phase = 'drive'
            else:
                return Command(0.0, _rot_in_place(e, 0.5)), _state('turn1'), False
    if phase == 'drive':
        if dist_w < 0.06:
            phase = 'turn2'
        else:
            e = wrap(math.atan2(yw - py, xw - px) - yaw)
            v = 0.2 * max(0.0, math.cos(e))
            w = sat(1.5 * e, 0.5)
            return Command(v, w), _state('drive'), False
    # turn2: face the tag
    cx, cy = _cam_pos(px, py, yaw)
    target = math.atan2(-cy, -cx)
    e = wrap(target - yaw)
    if abs(e) < _deg(3.0):
        return Command(), None, True
    return Command(0.0, _rot_in_place(e, 0.5)), _state('turn2'), False


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
    beta_max = _deg(P['fov_half_deg'] - P['fov_margin_deg'])
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam_pos(px, py, yaw)
    ct = math.atan2(-cy, -cx)
    dd = max(-beta_max, min(beta_max, wrap(hd - ct)))
    hd = wrap(ct + dd)
    e = wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _cam_pos(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= _deg(4.0):
        return Command(), None
    return Command(0.0, _rot_in_place(e, 0.4)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        hd = -math.pi / 2.0 - sat(math.atan(P['k_lat'] * px), _deg(30.0))
        e = wrap(hd - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = wrap(-math.pi / 2.0 - yaw)
    if abs(e) <= _deg(P['head_tol_deg']):
        return Command(), True
    return Command(0.0, _rot_in_place(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        mstate = {'own': '56', 't0': t}
    if t - mstate['t0'] < P['verify_s']:
        return Command(), mstate, False, False, False
    if est is None:
        ok = False
    else:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(wrap(est['yaw'] + math.pi / 2.0)) <= _deg(P['verify_head_deg']))
    return Command(), None, ok, (not ok), ok


def a57_backoff(odom, mstate, n_bo, P):
    if mstate is None or mstate.get('own') != '57':
        n_bo = n_bo + 1
        mstate = {'own': '57', 'x0': odom.x, 'y0': odom.y}
    bo_ok = n_bo < P['max_backoff']
    travelled = math.hypot(odom.x - mstate['x0'], odom.y - mstate['y0'])
    if travelled >= P['backoff_dist']:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), mstate, n_bo, bo_ok, False


def a5e_fail():
    return Command(), True


# ------------------------------------------------------------------ GS (CR-1, R-10)
def as1_obstacle_monitor(scan, sstate, t, P):
    near = False
    if scan is not None:
        sector = _deg(P['safe_sector_deg'])
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            ang = wrap(scan.angle_min + i * scan.angle_increment)
            if abs(ang) <= sector and r < P['safe_dist']:
                near = True
                break
    sstate = dict(sstate) if sstate else {}
    if near:
        sstate['last_near'] = t
        return sstate, True, False
    if 'last_near' not in sstate:
        return sstate, False, True
    return sstate, False, (t - sstate['last_near']) >= P['clear_s']


def as2_inhibit_forward(cmd):
    return Command(min(cmd.v, 0.0), cmd.w)
