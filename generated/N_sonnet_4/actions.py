"""Action functions for the MIAT AGV pallet-pocket docking controller.

All functions are pure: state is passed in and returned. Only math and sim.api are used.

Frames
  odom frame   : Odom (x, y, yaw)
  pallet frame : tag at origin, +y out of the pocket face. The AGV pose in this
                 frame is est = {x, y, psi} of base_link; facing the pocket means
                 psi = -pi/2.
"""
import math
from sim.api import Command, FX, CAM_X, TAG_ID, DOCK_DIST, DT, V_MAX, W_MAX, IMG_W

_DEF = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45,
    "confirm_k": 3, "confirm_window_s": 0.5, "confirm_tol_deg": 6.0,
    "track_gate_deg": 8.0, "track_lost_s": 2.0, "d_app": 1.9, "v_app": 0.25,
    "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0, "w_tag": 0.3,
    "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3, "gate_yaw_deg": 12.0,
    "pose_stale_s": 3.0, "near_dist": 1.1, "blind_max_s": 10.0,
    "fov_half_deg": 34.5, "fov_margin_deg": 10.0, "y_pre": 1.65,
    "repos_extra": 0.6, "v_conv": 0.2, "v_final": 0.12, "k_head": 1.6,
    "k_lat": 2.5, "dock_y": 1.13, "x_tol": 0.03, "head_tol_deg": 2.0,
    "verify_lat": 0.04, "verify_head_deg": 3.5, "verify_dist": 0.08,
    "verify_s": 0.5, "backoff_dist": 0.6, "max_backoff": 3, "max_retries": 2,
}
HALF_PI = math.pi / 2.0


def _p(P, k):
    try:
        if P is not None:
            if isinstance(P, dict):
                if k in P:
                    return P[k]
            elif hasattr(P, k):
                return getattr(P, k)
            else:
                try:
                    return P[k]
                except Exception:
                    pass
    except Exception:
        pass
    return _DEF[k]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _stop():
    return Command(0.0, 0.0)


# ---------------------------------------------------------------- G0
def a0_init():
    return Command(0.0, 0.0), True


def a1_check_sensors(odom, scan):
    try:
        if odom is None or scan is None:
            return False
        for v in (odom.x, odom.y, odom.yaw):
            if v is None or not math.isfinite(v):
                return False
        r = scan.ranges
        if r is None or len(r) < 10:
            return False
        valid = 0
        for x in r:
            if x is not None and x == x:
                valid += 1
        return valid >= 10
    except Exception:
        return False


def a2_navigate():
    return True


def a6_docked():
    return Command(0.0, 0.0)


def a9_abort():
    return Command(0.0, 0.0)


# ---------------------------------------------------------------- G3
def a30_enter():
    return Command(0.0, 0.0), None, [], False, False, True


def a31_search_rotate(P):
    return Command(0.0, float(_p(P, "w_search")))


def _robust_depth(samples):
    vals = sorted(float(s) for s in (samples or [])
                  if s is not None and s == s and 0.3 < s < 9.0)
    if len(vals) < 3:
        return None
    best = None
    for i in range(len(vals)):
        grp = [s for s in vals[i:] if s <= vals[i] + 0.25]
        if best is None or len(grp) > len(best):
            best = grp
    if len(best) < 3:
        return None
    return best[len(best) // 2]


def a321_candidates(pallets, odom, blacklist, P):
    cands = []
    if not pallets or odom is None:
        return cands
    smin = _p(P, "score_min")
    wmin, wmax = _p(P, "width_min"), _p(P, "width_max")
    yaw = odom.yaw
    cxw = odom.x + CAM_X * math.cos(yaw)
    cyw = odom.y + CAM_X * math.sin(yaw)
    for d in pallets:
        try:
            if d.score < smin:
                continue
            z = _robust_depth(d.depth_samples)
            if z is None:
                continue
            width = d.width_px * z / FX
            if width < wmin or width > wmax:
                continue
            X = z * (d.u - IMG_W / 2.0) / FX
            wx = cxw + z * math.cos(yaw) + X * math.sin(yaw)
            wy = cyw + z * math.sin(yaw) - X * math.cos(yaw)
            bad = False
            for b in (blacklist or []):
                if math.hypot(wx - b[0], wy - b[1]) < 0.8:
                    bad = True
                    break
            if bad:
                continue
            cands.append({"x": wx, "y": wy, "cam": (cxw, cyw), "rng": z,
                          "brg": math.atan2(wy - cyw, wx - cxw), "score": d.score,
                          "width": width, "stamp": d.stamp})
        except Exception:
            continue
    return cands


def a322_confirm(cands, hist, t, P):
    win = _p(P, "confirm_window_s")
    k = int(_p(P, "confirm_k"))
    tol = math.radians(_p(P, "confirm_tol_deg"))
    new = [h for h in (hist or []) if t - h["t"] <= win + 1e-9]
    for c in cands or []:
        h = dict(c)
        h["t"] = t
        new.append(h)
    best = None
    best_val = -1e9
    for c in cands or []:
        m = [h for h in new if abs(_wrap(h["brg"] - c["brg"])) <= tol
             and math.hypot(h["x"] - c["x"], h["y"] - c["y"]) <= 1.0]
        stamps = set(round(h["t"], 4) for h in m)
        if len(stamps) >= k:
            sc = sum(h["score"] for h in m) / len(m)
            val = sc - 0.5 * max(0.0, c["width"] - 1.25)
            if val > best_val:
                best_val = val
                best = (c, m)
    if best is None:
        return new, False, None
    m = best[1]
    track = {"x": sum(h["x"] for h in m) / len(m),
             "y": sum(h["y"] for h in m) / len(m),
             "t_seen": t, "score": best[0]["score"]}
    return [], True, track


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(_p(P, "track_gate_deg"))
    best = None
    bd = 1e9
    for c in cands or []:
        cam = c["cam"]
        a1 = math.atan2(track["y"] - cam[1], track["x"] - cam[0])
        da = abs(_wrap(a1 - c["brg"]))
        r1 = math.hypot(track["y"] - cam[1], track["x"] - cam[0])
        if da <= gate and abs(r1 - c["rng"]) <= 1.0 and da < bd:
            bd = da
            best = c
    nt = dict(track)
    if best is not None:
        nt["x"] = track["x"] + 0.5 * (best["x"] - track["x"])
        nt["y"] = track["y"] + 0.5 * (best["y"] - track["y"])
        nt["t_seen"] = t
        nt["score"] = best["score"]
    lost = (t - nt["t_seen"]) > _p(P, "track_lost_s")
    return nt, lost


def a34_approach_cmd(track, odom, P):
    if track is None or odom is None:
        return _stop(), False
    cx = odom.x + CAM_X * math.cos(odom.yaw)
    cy = odom.y + CAM_X * math.sin(odom.yaw)
    dx, dy = track["x"] - cx, track["y"] - cy
    dist = math.hypot(dx, dy)
    d_app = _p(P, "d_app")
    if dist <= d_app:
        return _stop(), True
    b = _wrap(math.atan2(dy, dx) - odom.yaw)
    tol = math.radians(_p(P, "app_bear_tol_deg"))
    w = 0.0 if abs(b) < tol * 0.5 else _clamp(_p(P, "k_bear") * b, -W_MAX, W_MAX)
    v = _p(P, "v_app") * max(0.0, 1.0 - abs(b) / 0.5)
    v = min(v, max(0.08, 0.6 * (dist - d_app)))
    return Command(v, w), False


def a35_exit(track, odom):
    ref = None
    if track is not None:
        ref = (track["x"], track["y"])
    return _stop(), ref, True


def a3e_fail():
    return _stop(), True


# ---------------------------------------------------------------- G4 pose
def _cam(x, y, psi):
    """camera lateral/dist, heading error, bearing of tag from camera axis."""
    cx = x + CAM_X * math.cos(psi)
    cy = y + CAM_X * math.sin(psi)
    eps = _wrap(psi + HALF_PI)
    phi = math.atan2(-cy, -cx)
    beta = _wrap(phi - psi)
    return cx, cy, eps, phi, beta


def a43_update_pose(est, odom, tags, t, P):
    if odom is None:
        return est
    # propagate with odometry
    if est is not None:
        o0 = est["o"]
        dx, dy = odom.x - o0[0], odom.y - o0[1]
        dxl = math.cos(o0[2]) * dx + math.sin(o0[2]) * dy
        dyl = -math.sin(o0[2]) * dx + math.cos(o0[2]) * dy
        dth = _wrap(odom.yaw - o0[2])
        pm = est["psi"] + 0.5 * dth
        est = dict(est)
        est["x"] = est["x"] + math.cos(pm) * dxl - math.sin(pm) * dyl
        est["y"] = est["y"] + math.sin(pm) * dxl + math.cos(pm) * dyl
        est["psi"] = _wrap(est["psi"] + dth)
        est["o"] = (odom.x, odom.y, odom.yaw)
    # newest unused tag
    meas = None
    last_stamp = est["stamp"] if est is not None else -1e9
    best_stamp = last_stamp
    for tg in tags or []:
        try:
            if tg.id != TAG_ID or not (0.3 < tg.z < 9.0):
                continue
            if tg.stamp <= best_stamp:
                continue
            best_stamp = tg.stamp
            meas = tg
        except Exception:
            continue
    if meas is None:
        return est
    psi = _wrap(meas.yaw - HALF_PI)
    fx, fy = math.cos(psi), math.sin(psi)
    rx, ry = math.sin(psi), -math.cos(psi)
    camx = -(meas.z * fx + meas.x * rx)
    camy = -(meas.z * fy + meas.x * ry)
    bx, by = camx - CAM_X * fx, camy - CAM_X * fy
    if est is None:
        return {"x": bx, "y": by, "psi": psi, "o": (odom.x, odom.y, odom.yaw),
                "t_tag": t, "n": 1, "n_rej": 0, "stamp": meas.stamp}
    est = dict(est)
    est["stamp"] = meas.stamp
    dpos = math.hypot(bx - est["x"], by - est["y"])
    dyaw = abs(_wrap(psi - est["psi"]))
    if dpos > _p(P, "gate_pos") or dyaw > math.radians(_p(P, "gate_yaw_deg")):
        est["n_rej"] = est.get("n_rej", 0) + 1
        if est["n_rej"] >= 4 or (t - est["t_tag"]) > 2.0:
            est.update({"x": bx, "y": by, "psi": psi, "n": 1, "n_rej": 0, "t_tag": t})
        return est
    g = _p(P, "fuse_gain")
    est["x"] += g * (bx - est["x"])
    est["y"] += g * (by - est["y"])
    est["psi"] = _wrap(est["psi"] + g * _wrap(psi - est["psi"]))
    est["n"] = est["n"] + 1
    est["n_rej"] = 0
    est["t_tag"] = t
    return est


def a44_pose_status(est, t, P):
    if est is None:
        return False, False, True
    age = t - est["t_tag"]
    tag_ok = age <= 0.5
    ready = est["n"] >= int(_p(P, "init_meas")) and age <= 1.0
    cx, cy, _, _, _ = _cam(est["x"], est["y"], est["psi"])
    d = math.hypot(cx, cy)
    lim = _p(P, "blind_max_s") if d <= _p(P, "near_dist") else _p(P, "pose_stale_s")
    return tag_ok, ready, age > lim


# ---------------------------------------------------------------- G4 steps
def a40_enter(n_loc, est, P):
    n = int(n_loc) + 1
    retry_ok = n <= int(_p(P, "max_retries"))
    return _stop(), n, retry_ok, None, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    sw_amp = math.radians(_p(P, "tag_sweep_deg"))
    wt = _p(P, "w_tag")
    if sweep is None or sweep.get("phase", 4) >= 4:
        if pallet_ref is not None:
            c = math.atan2(pallet_ref[1] - odom.y, pallet_ref[0] - odom.x)
        else:
            c = odom.yaw
        sweep = {"phase": 0, "center": c, "dwell": 0}
    sweep = dict(sweep)
    c = sweep["center"]
    ph = sweep["phase"]
    if ph == 0:
        target = c
    elif ph == 1:
        target = c + sw_amp
    elif ph == 2:
        target = c - sw_amp
    else:
        target = c
    err = _wrap(target - odom.yaw)
    if abs(err) < math.radians(2.0):
        if ph == 0:
            sweep["dwell"] += 1
            if sweep["dwell"] >= 8:
                sweep["phase"] = 1
        else:
            sweep["phase"] = ph + 1
        return _stop(), sweep
    w = _clamp(2.0 * err, -wt, wt)
    if abs(w) < 0.08:
        w = 0.08 if err > 0 else -0.08
    return Command(0.0, w), sweep


def a42_hold():
    return _stop()


def a43_publish(est):
    return est is not None and est["n"] >= 1 and all(
        math.isfinite(est[k]) for k in ("x", "y", "psi"))


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        if not any(math.hypot(b[0] - pallet_ref[0], b[1] - pallet_ref[1]) < 0.5 for b in bl):
            bl.append((pallet_ref[0], pallet_ref[1]))
    return _stop(), bl, True


# ---------------------------------------------------------------- G5 guidance law
def _law(x, y, psi, P, v_nom):
    cx, cy, eps, phi, beta = _cam(x, y, psi)
    eps_d = -math.atan(_p(P, "k_lat") * cx)
    eps_d = _clamp(eps_d, -1.4, 1.4)
    psi_d = -HALF_PI + eps_d
    if cy > 0.4:
        lim = math.radians(_p(P, "fov_half_deg") - _p(P, "fov_margin_deg"))
        psi_d = phi + _clamp(_wrap(psi_d - phi), -lim, lim)
    err = _wrap(psi_d - psi)
    w = _clamp(_p(P, "k_head") * err, -W_MAX, W_MAX)
    v = v_nom * max(0.0, 1.0 - abs(err) / 0.6)
    return v, w


def _aligned(x, y, psi, P):
    cx, cy, eps, _, _ = _cam(x, y, psi)
    return abs(cx) <= 2.0 * _p(P, "x_tol") and abs(eps) <= math.radians(3.0 * _p(P, "head_tol_deg") - 1.0)


def _sim_converge(x, y, psi, P, max_steps=330):
    fov = math.radians(_p(P, "fov_half_deg") - 3.0)
    ypre = _p(P, "y_pre")
    for i in range(max_steps):
        if y <= ypre + 0.02:
            return _aligned(x, y, psi, P)
        v, w = _law(x, y, psi, P, _p(P, "v_conv"))
        if _aligned(x, y, psi, P) is False and y - ypre < 0.25:
            v = min(v, 0.08)
        psi = _wrap(psi + w * DT)
        x += v * math.cos(psi) * DT
        y += v * math.sin(psi) * DT
        cx, cy, eps, phi, beta = _cam(x, y, psi)
        if cy > 0.8 and abs(beta) > fov and i > 25:
            return False
    return False


def _env_limit(theta):
    pts = [(0.0, 2.77), (15.0, 2.52), (30.0, 2.42), (45.0, 2.02), (60.0, 1.7)]
    th = math.degrees(abs(theta))
    if th >= 60.0:
        return 1.7
    for (a0, d0), (a1, d1) in zip(pts, pts[1:]):
        if th <= a1:
            return d0 + (d1 - d0) * (th - a0) / (a1 - a0)
    return 1.7


def _staging(x, y, psi, P):
    cx, cy, _, _, _ = _cam(x, y, psi)
    s = 1.0 if cx >= 0 else -1.0
    y0 = _p(P, "y_pre") + _p(P, "repos_extra")
    best = None
    bc = 1e9
    for ys in (y0, y0 + 0.2, y0 + 0.4, y0 + 0.6, y0 + 0.8):
        for xs in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.3):
            camy = ys - CAM_X
            D = math.hypot(xs, camy)
            th = math.atan2(xs, camy)
            if D > _env_limit(th) - 0.3:
                continue
            Sx = s * xs
            phi = math.atan2(-camy, -Sx)
            # place camera on the line: base position when facing tag
            bx = Sx - CAM_X * math.cos(phi) * -1.0
            # base = cam - CAM_X*f ; cam=(Sx, camy)
            bx = Sx - CAM_X * math.cos(phi)
            by = camy - CAM_X * math.sin(phi)
            if not _sim_converge(bx, by, phi, P):
                continue
            cost = math.hypot(bx - x, by - y) + 0.4 * abs(_wrap(phi - psi))
            if cost < bc:
                bc = cost
                best = (bx, by, phi)
    if best is None:
        phi = -HALF_PI
        best = (0.0, y0 + CAM_X + 0.2, phi)
    return best


def _fresh_est(est):
    return est is not None


# ---------------------------------------------------------------- G5 steps
def a50_enter():
    return _stop(), {}, 0, True, False, False, True


def a51_plan(est, P):
    if est is None:
        return _stop(), False
    ok = _sim_converge(est["x"], est["y"], est["psi"], P)
    return _stop(), bool(ok)


def a52_reposition(est, mstate, P):
    ms = dict(mstate or {})
    if est is None:
        return _stop(), ms, False
    x, y, psi = est["x"], est["y"], est["psi"]
    if "rp_S" not in ms:
        S = _staging(x, y, psi, P)
        dx, dy = S[0] - x, S[1] - y
        a = _wrap(math.atan2(dy, dx) - psi)
        ms["rp_S"] = S
        ms["rp_rev"] = abs(a) > HALF_PI
        ms["rp_stage"] = "go"
    S = ms["rp_S"]
    if ms["rp_stage"] == "go":
        dx, dy = S[0] - x, S[1] - y
        dist = math.hypot(dx, dy)
        if dist < 0.08:
            ms["rp_stage"] = "turn"
        else:
            a = _wrap(math.atan2(dy, dx) - psi)
            if ms["rp_rev"]:
                a = _wrap(a - math.pi)
            w = _clamp(1.5 * a, -W_MAX, W_MAX)
            v = 0.2 * max(0.0, math.cos(a)) if abs(a) < 1.0 else 0.0
            v *= min(1.0, dist / 0.4 + 0.2)
            if ms["rp_rev"]:
                v = -v
                w = _clamp(1.5 * a, -W_MAX, W_MAX)
            return Command(v, w), ms, False
    # turn to face the tag
    cx, cy, eps, phi, beta = _cam(x, y, psi)
    err = _wrap(phi - psi)
    if abs(err) <= math.radians(2.5):
        for k in ("rp_S", "rp_rev", "rp_stage", "acq_t0"):
            ms.pop(k, None)
        return _stop(), ms, True
    w = _clamp(1.5 * err, -W_MAX, W_MAX)
    if abs(w) < 0.1:
        w = 0.1 if err > 0 else -0.1
    return Command(0.0, w), ms, False


def a53_converge(est, P):
    if est is None:
        return _stop(), False, False
    x, y, psi = est["x"], est["y"], est["psi"]
    ypre = _p(P, "y_pre")
    al = _aligned(x, y, psi, P)
    if y <= ypre + 0.02 and al:
        return _stop(), True, False
    if y < ypre - 0.1:
        return _stop(), False, True
    v, w = _law(x, y, psi, P, _p(P, "v_conv"))
    if not al and y - ypre < 0.25:
        v = min(v, 0.08)
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    ms = dict(mstate or {})
    if "acq_t0" not in ms:
        ms["acq_t0"] = t
    if est is None:
        return Command(0.0, 0.3), ms
    cx, cy, eps, phi, beta = _cam(est["x"], est["y"], est["psi"])
    dt = t - ms["acq_t0"]
    off = 0.0
    if dt > 1.5:
        amp = math.radians(25.0)
        ph = ((dt - 1.5) % 6.0) / 6.0
        off = amp * (1.0 if ph < 0.25 else -1.0 if ph < 0.75 else 1.0)
        off = amp * math.sin(2 * math.pi * ph)
    err = _wrap(phi + off - est["psi"])
    if abs(err) < math.radians(3.0):
        return _stop(), ms
    w = _clamp(1.5 * err, -0.4, 0.4)
    if abs(w) < 0.08:
        w = 0.08 if err > 0 else -0.08
    return Command(0.0, w), ms


def a55_final_approach(est, P):
    if est is None:
        return _stop(), False
    x, y, psi = est["x"], est["y"], est["psi"]
    dock_y = _p(P, "dock_y")
    rem = y - dock_y
    if rem <= 0.02:
        return _stop(), True
    v, w = _law(x, y, psi, P, _p(P, "v_final"))
    cx, cy, eps, _, _ = _cam(x, y, psi)
    if rem < 0.15 and (abs(cx) > _p(P, "x_tol") or abs(eps) > math.radians(_p(P, "head_tol_deg"))):
        v = min(v, 0.04)
    v = min(v, max(0.04, 1.5 * rem))
    return Command(v, w), False


def a56_verify(est, mstate, t, P):
    ms = dict(mstate or {})
    if "v_enter" not in ms:
        ms["v_enter"] = t
        ms["v_t0"] = None
        ms["b_t0"] = None

    def done(ver, bad):
        for k in ("v_enter", "v_t0", "b_t0"):
            ms.pop(k, None)
        return _stop(), ms, ver, bad, ver

    if est is not None and est["n"] >= int(_p(P, "init_meas")) and t - est["t_tag"] <= 0.35:
        cx, cy, eps, _, _ = _cam(est["x"], est["y"], est["psi"])
        ok = (abs(cx) <= _p(P, "verify_lat")
              and abs(eps) <= math.radians(_p(P, "verify_head_deg"))
              and abs(cy - DOCK_DIST) <= _p(P, "verify_dist"))
        if ok:
            ms["b_t0"] = None
            if ms["v_t0"] is None:
                ms["v_t0"] = t
            if t - ms["v_t0"] >= _p(P, "verify_s") - 1e-9:
                return done(True, False)
        else:
            ms["v_t0"] = None
            if ms["b_t0"] is None:
                ms["b_t0"] = t
            if t - ms["b_t0"] >= 0.6:
                return done(False, True)
    else:
        ms["v_t0"] = None
    if t - ms["v_enter"] >= 4.0:
        return done(False, True)
    return _stop(), ms, False, False, False


def a57_backoff(odom, mstate, n_bo, P):
    ms = dict(mstate or {})
    if "bo_start" not in ms:
        ms["bo_start"] = (odom.x, odom.y)
        n_bo = int(n_bo) + 1
    bo_ok = n_bo < int(_p(P, "max_backoff"))
    s = ms["bo_start"]
    d = math.hypot(odom.x - s[0], odom.y - s[1])
    if d >= _p(P, "backoff_dist"):
        ms.pop("bo_start", None)
        ms.pop("acq_t0", None)
        return _stop(), ms, n_bo, bo_ok, True
    return Command(-0.2, 0.0), ms, n_bo, bo_ok, False


def a5e_fail():
    return _stop(), True
