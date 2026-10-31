"""Action functions for the AGV pallet-pocket docking controller (MIAT flow).

Only `math` and `sim.api` are used.  No module-level mutable state: everything an
action needs across ticks is passed in and returned.

Frames
------
* odom frame  : as reported by Odom (x, y, yaw), CCW positive.
* pallet frame: tag centre at the origin, +y out of the pocket face.
  The base_link pose in this frame is (bx, by, th); the AGV faces the pocket
  when th = -pi/2.  psi = th + pi/2 is the heading error w.r.t. the face normal
  (psi == tag yaw measurement).
"""
import math
from sim.api import Command, FX, IMG_W, CAM_X, TAG_ID, DOCK_DIST, V_MAX, W_MAX

_DEFAULTS = {
    'w_search': 0.35, 'score_min': 0.55, 'width_min': 0.75, 'width_max': 1.45,
    'confirm_k': 3, 'confirm_window_s': 0.5, 'confirm_tol_deg': 6.0,
    'track_gate_deg': 8.0, 'track_lost_s': 2.0, 'd_app': 1.9, 'v_app': 0.25,
    'k_bear': 1.2, 'app_bear_tol_deg': 4.0, 'tag_sweep_deg': 35.0, 'w_tag': 0.3,
    'init_meas': 3, 'fuse_gain': 0.35, 'gate_pos': 0.3, 'gate_yaw_deg': 12.0,
    'pose_stale_s': 3.0, 'near_dist': 1.1, 'blind_max_s': 10.0,
    'fov_half_deg': 34.5, 'fov_margin_deg': 10.0, 'y_pre': 1.65,
    'repos_extra': 0.6, 'v_conv': 0.2, 'v_final': 0.12, 'k_head': 1.6,
    'k_lat': 2.5, 'dock_y': 1.13, 'x_tol': 0.03, 'head_tol_deg': 2.0,
    'verify_lat': 0.04, 'verify_head_deg': 3.5, 'verify_dist': 0.08,
    'verify_s': 0.5, 'backoff_dist': 0.6, 'max_backoff': 3, 'max_retries': 2,
}


def _p(P, key):
    try:
        if isinstance(P, dict) and key in P:
            return P[key]
        if P is not None and not isinstance(P, dict) and hasattr(P, key):
            return getattr(P, key)
    except Exception:
        pass
    return _DEFAULTS[key]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _cmd(v=0.0, w=0.0):
    return Command(_clamp(float(v), -V_MAX, V_MAX), _clamp(float(w), -W_MAX, W_MAX))


def _zero():
    return Command(0.0, 0.0)


def _fin(*xs):
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in xs)


# ============================================================ G0 (top level)
def a0_init():
    return _zero(), True


def a1_check_sensors(odom, scan):
    try:
        ok = odom is not None and _fin(odom.x, odom.y, odom.yaw)
        if ok and scan is not None:
            ok = len(scan.ranges) > 0
        return bool(ok)
    except Exception:
        return False


def a2_navigate():
    # Nav2 has already brought the AGV to the viewpoint.
    return True


def a6_docked():
    return _zero()


def a9_abort():
    return _zero()


# ============================================================ G3 pallet search / approach
def a30_enter():
    return _zero(), None, [], False, False, True


def a31_search_rotate(P):
    return _cmd(0.0, _p(P, 'w_search'))


def _robust_depth(samples):
    """Nearest well-supported depth cluster (rejects zeros and background hits)."""
    vals = sorted(float(s) for s in (samples or []) if _fin(s) and 0.3 < s < 12.0)
    if len(vals) < 3:
        return None
    clusters = [[vals[0]]]
    for v in vals[1:]:
        if v - clusters[-1][-1] > 0.2:
            clusters.append([v])
        else:
            clusters[-1].append(v)
    need = max(3, int(math.ceil(0.25 * len(vals))))
    good = [c for c in clusters if len(c) >= need]
    if not good:
        good = [max(clusters, key=len)]
        if len(good[0]) < 3:
            return None
    c = good[0]  # nearest supported cluster
    return c[len(c) // 2]


def a321_candidates(pallets, odom, blacklist, P):
    out = []
    smin = _p(P, 'score_min')
    wmin = _p(P, 'width_min')
    wmax = _p(P, 'width_max')
    for d in (pallets or []):
        try:
            if d.score < smin:
                continue
            z = _robust_depth(d.depth_samples)
            if z is None:
                continue
            wm = d.width_px * z / FX
            if not (wmin <= wm <= wmax):
                continue
            right = (d.u - IMG_W / 2.0) * z / FX
            fwd = z + CAM_X
            left = -right
            r = math.hypot(fwd, left)
            b = math.atan2(left, fwd)
            wb = _wrap(odom.yaw + b)
            wx = odom.x + r * math.cos(wb)
            wy = odom.y + r * math.sin(wb)
            skip = False
            for bl in (blacklist or []):
                if math.hypot(wx - bl[0], wy - bl[1]) < 0.8:
                    skip = True
                    break
            if skip:
                continue
            out.append({'x': wx, 'y': wy, 'r': r, 'bear': b, 'wbear': wb,
                        'score': float(d.score), 'width': wm})
        except Exception:
            continue
    out.sort(key=lambda c: -c['score'])
    return out


def a322_confirm(cands, hist, t, P):
    k = int(_p(P, 'confirm_k'))
    win = _p(P, 'confirm_window_s')
    tol = math.radians(_p(P, 'confirm_tol_deg'))
    hist = [h for h in (hist or []) if 0.0 <= t - h[0] <= win + 0.05]
    for c in (cands or []):
        hist.append([t, c['x'], c['y'], c['wbear'], c['r'], c['score']])
    best = None
    best_sc = -1.0
    for c in (cands or []):
        ticks = set()
        scs = []
        for h in hist:
            if abs(_wrap(h[3] - c['wbear'])) <= tol and \
                    math.hypot(h[1] - c['x'], h[2] - c['y']) <= 0.6:
                if h[0] not in ticks:
                    scs.append(h[5])
                ticks.add(h[0])
        if len(ticks) >= k:
            ms = sum(scs) / len(scs)
            if ms > best_sc:
                best_sc = ms
                best = c
    if best is None:
        return hist, False, None
    track = {'x': best['x'], 'y': best['y'], 'r': best['r'], 'wbear': best['wbear'],
             'score': best['score'], 't_seen': t}
    return hist, True, track


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate_deg = math.radians(_p(P, 'track_gate_deg'))
    lost_s = _p(P, 'track_lost_s')
    best = None
    bd = 1e9
    for c in (cands or []):
        gate = 0.4 + c['r'] * math.tan(gate_deg)
        d = math.hypot(c['x'] - track['x'], c['y'] - track['y'])
        if d <= gate and d < bd:
            bd = d
            best = c
    tr = dict(track)
    if best is not None:
        tr['x'] = 0.5 * tr['x'] + 0.5 * best['x']
        tr['y'] = 0.5 * tr['y'] + 0.5 * best['y']
        tr['r'] = best['r']
        tr['wbear'] = best['wbear']
        tr['score'] = best['score']
        tr['t_seen'] = t
    lost = (t - tr['t_seen']) > lost_s
    return tr, lost


def a34_approach_cmd(track, odom, P):
    if track is None:
        return _zero(), False
    dx = track['x'] - odom.x
    dy = track['y'] - odom.y
    r = math.hypot(dx, dy)
    e = _wrap(math.atan2(dy, dx) - odom.yaw)
    d_cam = r - CAM_X
    d_stop = _p(P, 'd_app') - 0.05
    tol = math.radians(_p(P, 'app_bear_tol_deg'))
    if d_cam <= d_stop:
        if abs(e) <= tol:
            return _zero(), True
        return _cmd(0.0, _clamp(_p(P, 'k_bear') * e, -0.4, 0.4)), False
    w = _clamp(_p(P, 'k_bear') * e, -W_MAX, W_MAX)
    v = _p(P, 'v_app') * max(0.0, 1.0 - abs(e) / math.radians(20.0))
    v = min(v, max(0.08, 0.4 * (d_cam - d_stop) + 0.05))
    return _cmd(v, w), False


def a35_exit(track, odom):
    if track is None:
        return _zero(), None, False
    return _zero(), {'x': track['x'], 'y': track['y']}, True


def a3e_fail():
    return _zero(), True


# ============================================================ G4 localisation
def _new_est(odom):
    return {'ok': False, 'bx': 0.0, 'by': 0.0, 'th': 0.0,
            'od': (odom.x, odom.y, odom.yaw), 't_meas': -1e9, 't_now': 0.0,
            'n': 0, 'n_tot': 0, 'stamp': -1e9, 'rej': 0}


def _meas_pose(tag):
    psi = _wrap(tag.yaw)
    cx = -tag.z * math.sin(psi) + tag.x * math.cos(psi)
    cy = tag.z * math.cos(psi) + tag.x * math.sin(psi)
    th = psi - math.pi / 2.0
    bx = cx - CAM_X * math.cos(th)
    by = cy - CAM_X * math.sin(th)
    return bx, by, th


def a43_update_pose(est, odom, tags, t, P):
    if est is None:
        e = _new_est(odom)
    else:
        e = dict(est)
        # dead reckoning with odometry increments
        xp, yp, yawp = e['od']
        dx, dy = odom.x - xp, odom.y - yp
        dth = _wrap(odom.yaw - yawp)
        if e['ok']:
            df = math.cos(yawp) * dx + math.sin(yawp) * dy
            dl = -math.sin(yawp) * dx + math.cos(yawp) * dy
            thm = e['th'] + 0.5 * dth
            e['bx'] += df * math.cos(thm) - dl * math.sin(thm)
            e['by'] += df * math.sin(thm) + dl * math.cos(thm)
            e['th'] = _wrap(e['th'] + dth)
        e['od'] = (odom.x, odom.y, odom.yaw)
    e['t_now'] = t

    # newest unseen tag detection
    tag = None
    for d in (tags or []):
        try:
            if d.id != TAG_ID or not _fin(d.x, d.z, d.yaw):
                continue
            if d.stamp <= e['stamp']:
                continue
            if tag is None or d.stamp > tag.stamp:
                tag = d
        except Exception:
            continue
    if tag is None:
        return e
    e['stamp'] = tag.stamp
    if tag.z < 0.5 or tag.z > 4.0:
        return e
    # viewing angle sanity (unreliable beyond ~60 deg from the face normal)
    psi = _wrap(tag.yaw)
    cxm = -tag.z * math.sin(psi) + tag.x * math.cos(psi)
    cym = tag.z * math.cos(psi) + tag.x * math.sin(psi)
    if cym <= 0.2 or abs(math.atan2(cxm, cym)) > math.radians(66.0):
        return e
    mbx, mby, mth = _meas_pose(tag)

    stationary = abs(odom.v) < 0.01 and abs(odom.w) < 0.02
    if not e['ok']:
        e.update(ok=True, bx=mbx, by=mby, th=mth, n=1, rej=0)
        e['t_meas'] = t
        e['n_tot'] += 1
        return e
    dpos = math.hypot(mbx - e['bx'], mby - e['by'])
    dyaw = abs(_wrap(mth - e['th']))
    if dpos <= _p(P, 'gate_pos') and dyaw <= math.radians(_p(P, 'gate_yaw_deg')):
        e['n'] += 1
        g = max(_p(P, 'fuse_gain'), 1.0 / max(1, e['n']))
        if stationary:
            g = max(g, 0.5)
        e['bx'] += g * (mbx - e['bx'])
        e['by'] += g * (mby - e['by'])
        e['th'] = _wrap(e['th'] + g * _wrap(mth - e['th']))
        e['rej'] = 0
        e['t_meas'] = t
        e['n_tot'] += 1
    else:
        e['rej'] += 1
        if e['rej'] >= 4:
            # persistent disagreement: trust the sensor, restart the estimate
            e.update(bx=mbx, by=mby, th=mth, n=1, rej=0)
            e['t_meas'] = t
            e['n_tot'] += 1
    return e


def _geom(e):
    """camera x, camera y, heading error psi, base th."""
    th = e['th']
    cx = e['bx'] + CAM_X * math.cos(th)
    cy = e['by'] + CAM_X * math.sin(th)
    psi = _wrap(th + math.pi / 2.0)
    return cx, cy, psi, th


def _age(e, t=None):
    tn = e['t_now'] if t is None else t
    return tn - e['t_meas']


def a44_pose_status(est, t, P):
    if est is None or not est.get('ok'):
        return False, False, True
    age = _age(est, t)
    tag_ok = age <= 0.5
    pose_ready = est['n'] >= int(_p(P, 'init_meas')) and age <= _p(P, 'pose_stale_s')
    cx, cy, psi, th = _geom(est)
    limit = _p(P, 'blind_max_s') if cy <= _p(P, 'near_dist') else _p(P, 'pose_stale_s')
    stale = age > limit
    return bool(tag_ok), bool(pose_ready), bool(stale)


def a40_enter(n_loc, est, P):
    n = int(n_loc) + 1
    retry_ok = n <= int(_p(P, 'max_retries'))
    sweep = {'init': False}
    return _zero(), n, bool(retry_ok), None, sweep, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    s = dict(sweep) if sweep else {'init': False}
    if not s.get('init'):
        if pallet_ref:
            c = math.atan2(pallet_ref['y'] - odom.y, pallet_ref['x'] - odom.x)
        else:
            c = odom.yaw
        s.update(init=True, center=c, sgn=1, wait=6)
    if s['wait'] > 0:
        s['wait'] -= 1
        return _zero(), s
    a = math.radians(_p(P, 'tag_sweep_deg'))
    wt = _p(P, 'w_tag')
    target = s['center'] + s['sgn'] * a
    err = _wrap(target - odom.yaw)
    if abs(err) < math.radians(3.0):
        s['sgn'] = -s['sgn']
        target = s['center'] + s['sgn'] * a
        err = _wrap(target - odom.yaw)
    w = math.copysign(min(wt, max(0.1, 1.5 * abs(err))), err)
    return _cmd(0.0, w), s


def a42_hold():
    return _zero()


def a43_publish(est):
    if est is None or not est.get('ok'):
        return False
    return est['n'] >= 3 and _age(est) <= 3.0


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref:
        pt = (pallet_ref['x'], pallet_ref['y'])
        if all(math.hypot(pt[0] - b[0], pt[1] - b[1]) > 0.3 for b in bl):
            bl.append(pt)
    return _zero(), bl, True


# ============================================================ G5 guidance
def a50_enter():
    return _zero(), {}, 0, True, False, False, True


def _tag_bearing(e):
    """bearing of the tag in the camera frame (+ = left)."""
    cx, cy, psi, th = _geom(e)
    return _wrap(math.atan2(-cy, -cx) - th)


def a51_plan(est, P):
    if est is None or not est.get('ok'):
        return _zero(), False
    cx, cy, psi, th = _geom(est)
    if cy < DOCK_DIST - 0.05:
        return _zero(), False
    lx_ok = max(0.10, (cy - DOCK_DIST - 0.25) * math.tan(math.radians(22.0)))
    return _zero(), bool(abs(cx) <= lx_ok)


def a52_reposition(est, mstate, P):
    m = dict(mstate) if mstate else {}
    if est is None or not est.get('ok'):
        return _zero(), m, False
    bx, by, th = est['bx'], est['by'], est['th']
    T = m.get('T')
    if T is None:
        y0 = _p(P, 'y_pre') + _p(P, 'repos_extra')
        T = (0.0, _clamp(by, y0, y0 + 0.2))
        m['T'] = T
    dx, dy = T[0] - bx, T[1] - by
    d = math.hypot(dx, dy)
    if d <= 0.10:
        m['arr'] = True
    if d > 0.10 and not m.get('arr'):
        b = _wrap(math.atan2(dy, dx) - th)
        rev = abs(b) > math.pi / 2.0
        if rev:
            b = _wrap(b - math.pi)
        w = _clamp(1.5 * b, -0.5, 0.5)
        v = _p(P, 'v_conv') * max(0.0, 1.0 - abs(b) / math.radians(35.0))
        v *= min(1.0, d / 0.3 + 0.3)
        return _cmd(-v if rev else v, w), m, False
    al = _tag_bearing(est)
    if abs(al) > math.radians(6.0):
        w = math.copysign(min(0.4, max(0.1, 1.2 * abs(al))), al)
        return _cmd(0.0, w), m, False
    m.pop('T', None)
    m.pop('arr', None)
    return _zero(), m, True


def _funnel(e, P, lim_deg):
    """desired base heading th_des for converging on the centre line, tag kept in FOV."""
    cx, cy, psi, th = _geom(e)
    lim = math.radians(lim_deg)
    psi_des = -_clamp(math.atan(_p(P, 'k_lat') * cx), -lim, lim)
    th_des = psi_des - math.pi / 2.0
    amax = math.radians(_p(P, 'fov_half_deg') - _p(P, 'fov_margin_deg'))
    phi = math.atan2(-cy, -cx)
    dlt = _clamp(_wrap(th_des - phi), -amax, amax)
    th_des = phi + dlt
    return th_des


def a53_converge(est, P):
    if est is None or not est.get('ok'):
        return _zero(), False, False
    cx, cy, psi, th = _geom(est)
    cy_pre = _p(P, 'y_pre') - CAM_X
    aligned = abs(cx) <= 0.07 and abs(psi) <= math.radians(5.0)
    if aligned and cy <= cy_pre + 0.05:
        return _zero(), True, False
    if (not aligned) and cy <= cy_pre - 0.15:
        return _zero(), False, True
    th_des = _funnel(est, P, 25.0)
    err = _wrap(th_des - th)
    w = _clamp(_p(P, 'k_head') * err, -W_MAX, W_MAX)
    if abs(err) > math.radians(35.0):
        v = 0.0
    else:
        v = _p(P, 'v_conv') * max(0.0, 1.0 - abs(err) / math.radians(40.0))
    if cy < cy_pre + 0.1:
        v = min(v, 0.08)
    return _cmd(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    m = dict(mstate) if mstate else {}
    if est is None or not est.get('ok'):
        return _zero(), m
    al = _tag_bearing(est)
    lim = math.radians(_p(P, 'fov_half_deg') - _p(P, 'fov_margin_deg'))
    if abs(al) > 0.5 * lim:
        w = math.copysign(min(0.3, max(0.1, abs(al))), al)
        return _cmd(0.0, w), m
    return _zero(), m


def a55_final_approach(est, P):
    if est is None or not est.get('ok'):
        return _zero(), False
    cx, cy, psi, th = _geom(est)
    if cy <= DOCK_DIST + 0.005:
        return _zero(), True
    th_des = _funnel(est, P, 8.0)
    err = _wrap(th_des - th)
    w = _clamp(_p(P, 'k_head') * err, -W_MAX, W_MAX)
    v = _p(P, 'v_final')
    if abs(err) > math.radians(8.0):
        v = 0.04
    if cy - DOCK_DIST < 0.08:
        v = min(v, 0.06)
    return _cmd(v, w), False


def a56_verify(est, mstate, t, P):
    m = dict(mstate) if mstate else {}
    if est is None or not est.get('ok'):
        return _zero(), m, False, True, False
    if m.get('v_t0') is None:
        m['v_t0'] = t
        m['v_n0'] = est['n_tot']
    el = t - m['v_t0']
    nm = est['n_tot'] - m['v_n0']
    age = t - est['t_meas']
    cx, cy, psi, th = _geom(est)
    good = (abs(cx) <= _p(P, 'verify_lat')
            and abs(psi) <= math.radians(_p(P, 'verify_head_deg'))
            and abs(cy - DOCK_DIST) <= _p(P, 'verify_dist'))
    if el >= _p(P, 'verify_s') - 1e-9 and nm >= 4 and age <= 0.5:
        if good:
            return _zero(), m, True, False, True
        m.pop('v_t0', None)
        m.pop('v_n0', None)
        return _zero(), m, False, True, False
    if el >= 3.0:
        m.pop('v_t0', None)
        m.pop('v_n0', None)
        return _zero(), m, False, True, False
    return _zero(), m, False, False, False


def a57_backoff(odom, mstate, n_bo, P):
    m = dict(mstate) if mstate else {}
    n = int(n_bo)
    if m.get('bo0') is None:
        m['bo0'] = (odom.x, odom.y)
        n += 1
    bo_ok = n < int(_p(P, 'max_backoff'))
    x0, y0 = m['bo0']
    if math.hypot(odom.x - x0, odom.y - y0) >= _p(P, 'backoff_dist'):
        m.pop('bo0', None)
        return _zero(), m, n, bo_ok, True
    return _cmd(-0.15, 0.0), m, n, bo_ok, False


def a5e_fail():
    return _zero(), True
