"""Datapath Actions of the AGV pallet-pocket docking controller (see ACTION_CONTRACTS.md).

No module-level mutable state: everything an action remembers is passed in and returned.
"""
import math

from sim.api import Command, IMG_W, FX, CAM_X, TAG_ID

DEG = math.pi / 180.0


# ---------------------------------------------------------------- helpers
def wrap(a):
    """Wrap angle to (-pi, pi]."""
    r = (a + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if r == -math.pi else r


def sat(x, lim):
    return max(-lim, min(lim, x))


def _cam(px, py, yaw):
    return (px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw))


def _rot(e, wmax):
    """In-place rotation rule: proportional, with a minimum rate near zero error."""
    if abs(e) > 0.12:
        return sat(1.8 * e, wmax)
    return math.copysign(0.15, e)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n % 2:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def _beta_max(P):
    return (P['fov_half_deg'] - P['fov_margin_deg']) * DEG


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


# ---------------------------------------------------------------- A3 approach
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
    cx = odom.x + CAM_X * math.cos(odom.yaw)
    cy = odom.y + CAM_X * math.sin(odom.yaw)
    for det in pallets:
        if det.score < P['score_min']:
            continue
        d, ok = a332_pallet_distance(det.depth_samples)
        if not ok:
            continue
        u = a331_bbox_center(det)
        b = a333_pallet_bearing(u)
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
    tol = P['confirm_tol_deg'] * DEG
    hist = [h for h in hist if t - h['t'] <= window]
    for c in cands:
        hist.append({'t': t, 'c': c})
    best, best_key = None, None
    for c in cands:
        stamps = set()
        for h in hist:
            hc = h['c']
            if abs(wrap(hc['ba'] - c['ba'])) <= tol and abs(hc['d'] - c['d']) <= 0.4:
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
    best, best_dba = None, None
    for c in cands:
        dba = abs(wrap(c['ba'] - track['ba']))
        if dba <= gate and abs(c['d'] - track['d']) < 0.6:
            if best_dba is None or dba < best_dba:
                best, best_dba = c, dba
    new = dict(track)
    if best is not None:
        new['ba'] = wrap(track['ba'] + 0.5 * wrap(best['ba'] - track['ba']))
        new['pos'] = ((track['pos'][0] + best['pos'][0]) / 2.0,
                      (track['pos'][1] + best['pos'][1]) / 2.0)
        new['d'] = best['d']
        new['t'] = t
    lost = (t - new['t']) > P['track_lost_s']
    return new, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw)
    px, py = track['pos']
    d = math.hypot(px - cx, py - cy)
    b = wrap(odom.yaw - math.atan2(py - cy, px - cx))
    if d <= P['d_app']:
        tol = P['app_bear_tol_deg'] * DEG
        done = abs(b) <= tol
        w = 0.0 if done else sat(-1.5 * b, 0.4)
        return Command(0.0, w), done
    w = sat(-P['k_bear'] * b, 0.5)
    v = P['v_app'] if abs(b) < 10.0 * DEG else 0.05
    return Command(v, w), False


def a35_exit(track, odom):
    return Command(), {'ba': track['ba'], 'pos': track['pos']}, True


def a3e_fail():
    return Command(), True


# ---------------------------------------------------------------- A4 localisation
def a40_enter(n_loc, est, P):
    n_loc = n_loc + 1
    retry_ok = n_loc <= P['max_retries']
    return Command(), n_loc, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = _cam(odom.x, odom.y, odom.yaw)
            px, py = pallet_ref['pos']
            ref = math.atan2(py - cy, px - cx)
        else:
            ref = odom.yaw
        A = P['tag_sweep_deg'] * DEG
        targets = [wrap(ref), wrap(ref + A), wrap(ref - A), wrap(ref)]
        i = 0
    else:
        targets = list(sweep['targets'])
        i = sweep['i']
    err = wrap(targets[i] - odom.yaw)
    if abs(err) < 3.0 * DEG:
        return Command(), {'targets': targets, 'i': (i + 1) % len(targets)}
    cmd = Command(0.0, sat(2.0 * err, P['w_tag']))
    return cmd, {'targets': targets, 'i': i}


def a42_tag_to_pose(tag):
    yaw = wrap(tag.yaw - math.pi / 2.0)
    cx = -(tag.z * math.cos(yaw) + tag.x * math.sin(yaw))
    cy = -(tag.z * math.sin(yaw) - tag.x * math.cos(yaw))
    px = cx - CAM_X * math.cos(yaw)
    py = cy - CAM_X * math.sin(yaw)
    return px, py, yaw


def a43_update_pose(est, odom, tags, t, P):
    meas = [z for z in tags if z.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        px, py, yaw = a42_tag_to_pose(meas[0])
        return {'px': px, 'py': py, 'yaw': yaw, 't_meas': t, 'n': 1, 'rej': 0,
                'o': (odom.x, odom.y, odom.yaw)}

    # predict with odometry increment (robot frame at previous odometry heading)
    ox, oy, oyaw = est['o']
    dx, dy = odom.x - ox, odom.y - oy
    dyaw = wrap(odom.yaw - oyaw)
    f = math.cos(oyaw) * dx + math.sin(oyaw) * dy
    l = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
    yaw0 = est['yaw']
    px = est['px'] + f * math.cos(yaw0) - l * math.sin(yaw0)
    py = est['py'] + f * math.sin(yaw0) + l * math.cos(yaw0)
    yaw = wrap(yaw0 + dyaw)
    new = {'px': px, 'py': py, 'yaw': yaw, 't_meas': est['t_meas'], 'n': est['n'],
           'rej': est['rej'], 'o': (odom.x, odom.y, odom.yaw)}

    if not meas:
        return new

    zx, zy, zyaw = a42_tag_to_pose(meas[0])
    ex, ey = zx - px, zy - py
    eyaw = wrap(zyaw - yaw)
    cam_d = math.hypot(*_cam(zx, zy, zyaw))
    gate_pos = P['gate_pos'] + 0.1 * cam_d
    if math.hypot(ex, ey) <= gate_pos and abs(eyaw) <= P['gate_yaw_deg'] * DEG:
        k = 0.5 if est['n'] < P['init_meas'] else P['fuse_gain']
        new['px'] = px + k * ex
        new['py'] = py + k * ey
        new['yaw'] = wrap(yaw + k * eyaw)
        new['t_meas'] = t
        new['n'] = est['n'] + 1
        new['rej'] = 0
    else:
        rej = est['rej'] + 1
        if rej >= 5:
            new.update({'px': zx, 'py': zy, 'yaw': zyaw, 't_meas': t, 'n': 1, 'rej': 0})
        else:
            new['rej'] = rej
    return new


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est['t_meas']
    tag_ok = age < 0.35
    pose_ready = tag_ok and est['n'] >= P['init_meas']
    cam_d = math.hypot(*_cam(est['px'], est['py'], est['yaw']))
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


# ---------------------------------------------------------------- A5 guidance
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
    return Command(), gamma <= _beta_max(P) - 2.0 * DEG


def a52_reposition(est, mstate, P):
    yp = P['y_pre']
    bmax = _beta_max(P)
    if mstate is None or mstate.get('own') != '52':
        px, py = est['px'], est['py']
        yw = max(py, yp + P['repos_extra'])
        sgn = 1.0 if px >= 0 else -1.0
        xw = sgn * (yw - yp) * math.tan(bmax - 8.0 * DEG)
        mstate = {'own': '52', 'ph': 'turn1', 'W': (xw, yw)}

    ph, W = mstate['ph'], mstate['W']
    px, py, yaw = est['px'], est['py'], est['yaw']
    dist = math.hypot(W[0] - px, W[1] - py)

    # phase transitions fall through within the same tick
    if ph == 'turn1' and dist < 0.06:
        ph = 'turn2'
    if ph == 'turn1':
        e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
        if abs(e) < 3.0 * DEG:
            ph = 'drive'
        else:
            return Command(0.0, _rot(e, 0.5)), {'own': '52', 'ph': ph, 'W': W}, False
    if ph == 'drive':
        if dist <= 0.06:
            ph = 'turn2'
        else:
            e = wrap(math.atan2(W[1] - py, W[0] - px) - yaw)
            cmd = Command(0.2 * max(0.0, math.cos(e)), sat(1.5 * e, 0.5))
            return cmd, {'own': '52', 'ph': ph, 'W': W}, False
    # turn2: face the tag from the camera position
    cx, cy = _cam(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < 3.0 * DEG:
        return Command(), None, True
    return Command(0.0, _rot(e, 0.5)), {'own': '52', 'ph': 'turn2', 'W': W}, False


def a53_converge(est, P):
    yp = P['y_pre']
    px, py, yaw = est['px'], est['py'], est['yaw']
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return Command(), True, False
    if py < yp - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    ty = yp if py > yp else py - 0.3
    hd = math.atan2(ty - py, 0.0 - px)
    cx, cy = _cam(px, py, yaw)
    tdir = math.atan2(-cy, -cx)
    bmax = _beta_max(P)
    diff = sat(wrap(hd - tdir), bmax)
    hd = wrap(tdir + diff)
    e = wrap(hd - yaw)
    dist = math.hypot(px, py - yp)
    v = P['v_conv'] * max(0.0, math.cos(e)) * min(1.0, max(0.3, dist / 0.4))
    w = sat(P['k_head'] * e, 0.6)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    if est is None:
        return Command(0.0, 0.2), None
    px, py, yaw = est['px'], est['py'], est['yaw']
    cx, cy = _cam(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) <= 4.0 * DEG:
        return Command(), None
    return Command(0.0, _rot(e, 0.4)), None


def a55_final_approach(est, P):
    px, py, yaw = est['px'], est['py'], est['yaw']
    ey = py - P['dock_y']
    if ey > 0.01:
        des = -math.pi / 2.0 - sat(math.atan(P['k_lat'] * px), 30.0 * DEG)
        e = wrap(des - yaw)
        v = P['v_final'] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        w = sat(P['k_head'] * e, 0.5)
        return Command(v, w), False
    e = wrap(-math.pi / 2.0 - yaw)
    if abs(e) <= P['head_tol_deg'] * DEG:
        return Command(), True
    return Command(0.0, _rot(e, 0.3)), False


def a56_verify(est, mstate, t, P):
    if mstate is None or mstate.get('own') != '56':
        return Command(), {'own': '56', 't0': t}, False, False, False
    if t - mstate['t0'] < P['verify_s'] - 1e-9:
        return Command(), mstate, False, False, False
    ok = False
    if est is not None:
        ok = (abs(est['px']) <= P['verify_lat']
              and abs(est['py'] - P['dock_y']) <= P['verify_dist']
              and abs(wrap(est['yaw'] + math.pi / 2.0)) <= P['verify_head_deg'] * DEG)
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
