"""Action functions for the MIAT AGV pallet docking controller.

Pure functions: all cross-tick state is passed in and returned.
Pose convention ("pallet frame"): tag centre at origin, +y out of the pocket
face, estimate = base_link pose (px, py, pth); facing the pocket = -pi/2.
"""
import math
from sim.api import Command, CAM_X, FX, IMG_W, TAG_ID, DOCK_DIST

_DEF = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45,
    "confirm_k": 3, "confirm_window_s": 0.5, "confirm_tol_deg": 6.0,
    "track_gate_deg": 8.0, "track_lost_s": 2.0, "d_app": 1.9, "v_app": 0.25,
    "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0,
    "w_tag": 0.3, "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3,
    "gate_yaw_deg": 12.0, "pose_stale_s": 3.0, "near_dist": 1.1,
    "blind_max_s": 10.0, "fov_half_deg": 34.5, "fov_margin_deg": 10.0,
    "y_pre": 1.65, "repos_extra": 0.6, "v_conv": 0.2, "v_final": 0.12,
    "k_head": 1.6, "k_lat": 2.5, "dock_y": 1.13, "x_tol": 0.03,
    "head_tol_deg": 2.0, "verify_lat": 0.04, "verify_head_deg": 3.5,
    "verify_dist": 0.08, "verify_s": 0.5, "backoff_dist": 0.6,
    "max_backoff": 3, "max_retries": 2,
}

_HALF_PI = math.pi / 2.0


# ----------------------------------------------------------------- helpers
def _cfg(P):
    c = dict(_DEF)
    try:
        if isinstance(P, dict):
            c.update(P)
        elif P is not None:
            c.update(vars(P))
    except Exception:
        pass
    return c


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _cl(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _zero():
    return Command(0.0, 0.0)


def _finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def _xy(ref):
    if ref is None:
        return None
    try:
        if isinstance(ref, dict):
            return (float(ref["x"]), float(ref["y"]))
        return (float(ref[0]), float(ref[1]))
    except Exception:
        return None


def _cam(px, py, pth):
    return (px + CAM_X * math.cos(pth), py + CAM_X * math.sin(pth))


# ----------------------------------------------------------------- G0
def a0_init():
    return _zero(), True


def a1_check_sensors(odom, scan):
    if odom is None or not _finite(odom.x, odom.y, odom.yaw):
        return False
    if scan is not None:
        try:
            if len(scan.ranges) == 0:
                return False
        except Exception:
            return False
    return True


def a2_navigate():
    return True


def a6_docked():
    return _zero()


def a9_abort():
    return _zero()


# ----------------------------------------------------------------- G3
def a30_enter():
    return _zero(), None, [], False, False, True


def a31_search_rotate(P):
    c = _cfg(P)
    return Command(0.0, c["w_search"])


def _robust_depth(samples):
    vals = sorted(d for d in (samples or [])
                  if isinstance(d, (int, float)) and math.isfinite(d) and d > 0.05)
    if len(vals) < 3:
        return None
    best = None
    for i, d0 in enumerate(vals):
        grp = [v for v in vals[i:] if v <= d0 + 0.2]
        if best is None or len(grp) > len(best):
            best = grp
    if len(best) < 3:
        return None
    return best[len(best) // 2]


def a321_candidates(pallets, odom, blacklist, P):
    c = _cfg(P)
    out = []
    if not pallets or odom is None:
        return out
    bl = [_xy(b) for b in (blacklist or [])]
    bl = [b for b in bl if b is not None]
    cy, sy = math.cos(odom.yaw), math.sin(odom.yaw)
    for p in pallets:
        try:
            if p.score < c["score_min"]:
                continue
            z = _robust_depth(p.depth_samples)
            if z is None or z < 0.4 or z > 9.0:
                continue
            width = p.width_px * z / FX
            if not (c["width_min"] <= width <= c["width_max"]):
                continue
            xb = CAM_X + z
            yb = -(p.u - IMG_W / 2.0) / FX * z
            X = odom.x + cy * xb - sy * yb
            Y = odom.y + sy * xb + cy * yb
            if any(math.hypot(X - bx, Y - by) < 0.8 for bx, by in bl):
                continue
            out.append({"x": X, "y": Y, "ox": odom.x, "oy": odom.y,
                        "bw": math.atan2(Y - odom.y, X - odom.x),
                        "dist": math.hypot(X - odom.x, Y - odom.y),
                        "score": p.score, "width": width})
        except Exception:
            continue
    out.sort(key=lambda d: -d["score"])
    return out


def a322_confirm(cands, hist, t, P):
    c = _cfg(P)
    win = c["confirm_window_s"]
    tol = math.radians(c["confirm_tol_deg"])
    h = [e for e in (hist or []) if t - e["t"] <= win + 1e-6]
    for cd in (cands or []):
        e = dict(cd)
        e["t"] = t
        h.append(e)
    best = None
    for e in h:
        grp = [f for f in h
               if abs(_wrap(f["bw"] - e["bw"])) <= tol
               and math.hypot(f["x"] - e["x"], f["y"] - e["y"]) <= 0.8]
        ts = set(round(f["t"], 4) for f in grp)
        if len(ts) >= c["confirm_k"]:
            sc = sum(f["score"] for f in grp) / len(grp)
            key = (sc, len(ts))
            if best is None or key > best[0]:
                best = (key, grp)
    if best is None:
        return h, False, None
    grp = best[1]
    n = len(grp)
    track = {"x": sum(f["x"] for f in grp) / n, "y": sum(f["y"] for f in grp) / n,
             "t_last": t, "score": best[0][0]}
    return [], True, track


def a323_track(cands, track, t, P):
    c = _cfg(P)
    if track is None:
        return None, True
    gate = math.radians(c["track_gate_deg"])
    best = None
    for cd in (cands or []):
        ref = math.atan2(track["y"] - cd["oy"], track["x"] - cd["ox"])
        da = abs(_wrap(cd["bw"] - ref))
        dd = math.hypot(cd["x"] - track["x"], cd["y"] - track["y"])
        if da <= gate and dd <= 1.0 and (best is None or da < best[0]):
            best = (da, cd)
    tr = dict(track)
    if best is not None:
        cd = best[1]
        tr["x"] = 0.6 * tr["x"] + 0.4 * cd["x"]
        tr["y"] = 0.6 * tr["y"] + 0.4 * cd["y"]
        tr["t_last"] = t
    lost = (t - tr["t_last"]) > c["track_lost_s"]
    return tr, lost


def a34_approach_cmd(track, odom, P):
    c = _cfg(P)
    if track is None or odom is None:
        return _zero(), False
    dx, dy = track["x"] - odom.x, track["y"] - odom.y
    dist = math.hypot(dx, dy)
    e = _wrap(math.atan2(dy, dx) - odom.yaw)
    stop = c["d_app"] + CAM_X
    tol = math.radians(c["app_bear_tol_deg"])
    w = _cl(c["k_bear"] * e, -0.5, 0.5)
    if abs(e) > tol and abs(w) < 0.08:
        w = math.copysign(0.08, e)
    if dist <= stop:
        if abs(e) <= tol:
            return _zero(), True
        return Command(0.0, w), False
    v = c["v_app"] * max(0.0, 1.0 - abs(e) / 0.5)
    v = min(v, 0.5 * (dist - stop) + 0.06)
    return Command(v, w), False


def a35_exit(track, odom):
    if track is None:
        return _zero(), None, False
    return _zero(), {"x": track["x"], "y": track["y"]}, True


def a3e_fail():
    return _zero(), True


# ----------------------------------------------------------------- G4 pose
def _meas_pose(tag):
    th = _wrap(-_HALF_PI + tag.yaw)
    fx, fy = math.cos(th), math.sin(th)
    lx, ly = -fy, fx
    camx = -(tag.z * fx - tag.x * lx)
    camy = -(tag.z * fy - tag.x * ly)
    return (camx - CAM_X * fx, camy - CAM_X * fy, th)


def _new_est(m, odom, t, prev=None):
    e = {"px": m[0], "py": m[1], "pth": m[2],
         "ox": odom.x, "oy": odom.y, "oyaw": odom.yaw,
         "n": 1, "t_meas": t, "t_upd": t, "t_prev": None, "rej": 0,
         "n_acc": 1 if prev is None else prev.get("n_acc", 0) + 1,
         "last": None}
    if prev is not None:
        e["t_prev"] = prev.get("t_upd")
    return e


def a43_update_pose(est, odom, tags, t, P):
    c = _cfg(P)
    e = dict(est) if est else None
    if odom is None:
        return est
    if e is not None:
        dxo, dyo = odom.x - e["ox"], odom.y - e["oy"]
        cy, sy = math.cos(e["oyaw"]), math.sin(e["oyaw"])
        db_x = cy * dxo + sy * dyo
        db_y = -sy * dxo + cy * dyo
        dth = _wrap(odom.yaw - e["oyaw"])
        ct, st = math.cos(e["pth"]), math.sin(e["pth"])
        e["px"] += ct * db_x - st * db_y
        e["py"] += st * db_x + ct * db_y
        e["pth"] = _wrap(e["pth"] + dth)
        e["ox"], e["oy"], e["oyaw"] = odom.x, odom.y, odom.yaw
        e["t_prev"] = e["t_upd"]
        e["t_upd"] = t
    gp = c["gate_pos"]
    gy = math.radians(c["gate_yaw_deg"])
    for tag in (tags or []):
        try:
            if tag.id != TAG_ID or not _finite(tag.x, tag.z, tag.yaw):
                continue
            if tag.z < 0.3 or tag.z > 6.0 or abs(tag.yaw) > math.radians(85):
                continue
            key = (round(tag.stamp, 4), round(tag.x, 5), round(tag.z, 5))
            if e is not None and e.get("last") == key:
                continue
            m = _meas_pose(tag)
            cam_y = m[1] + CAM_X * math.sin(m[2])
            if cam_y <= 0.0:
                continue
        except Exception:
            continue
        if e is None:
            e = _new_est(m, odom, t)
            e["last"] = key
            continue
        e["last"] = key
        dpos = math.hypot(m[0] - e["px"], m[1] - e["py"])
        dyaw = abs(_wrap(m[2] - e["pth"]))
        init = e["n"] < c["init_meas"]
        k_pos, k_yaw = (2.0, 2.0) if init else (1.0, 1.0)
        if dpos > gp * k_pos or dyaw > gy * k_yaw:
            e["rej"] += 1
            if init or e["rej"] >= 6:
                e = _new_est(m, odom, t, prev=e)
                e["last"] = key
            continue
        g = 1.0 / (e["n"] + 1.0) if init else c["fuse_gain"]
        e["px"] += g * (m[0] - e["px"])
        e["py"] += g * (m[1] - e["py"])
        e["pth"] = _wrap(e["pth"] + g * _wrap(m[2] - e["pth"]))
        e["n"] += 1
        e["n_acc"] += 1
        e["rej"] = 0
        e["t_meas"] = t
    return e


def a44_pose_status(est, t, P):
    c = _cfg(P)
    if not est:
        return False, False, True
    age = t - est["t_meas"]
    cx, cy = _cam(est["px"], est["py"], est["pth"])
    dist = math.hypot(cx, cy)
    limit = c["blind_max_s"] if dist < c["near_dist"] else c["pose_stale_s"]
    tag_ok = age <= 0.3
    ready = est["n"] >= c["init_meas"] and age <= 1.0
    stale = age > limit
    return tag_ok, ready, stale


# ----------------------------------------------------------------- G4 steps
def a40_enter(n_loc, est, P):
    c = _cfg(P)
    n = (n_loc or 0) + 1
    retry_ok = n <= c["max_retries"]
    keep = None
    tgt = None
    if est:
        gap = None if est.get("t_prev") is None else est["t_upd"] - est["t_prev"]
        if gap is None or gap <= 0.25:
            keep = est
        if est["n"] >= c["init_meas"] or keep is None:
            # remember where the tag is (odom frame) to aim the sweep
            ang = est["oyaw"] - est["pth"]
            ca, sa = math.cos(ang), math.sin(ang)
            tgt = (est["ox"] - (ca * est["px"] - sa * est["py"]),
                   est["oy"] - (sa * est["px"] + ca * est["py"]))
    sweep = {"ph": 0, "y0": None, "tgt": tgt}
    return _zero(), n, retry_ok, keep, sweep, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    c = _cfg(P)
    sw = dict(sweep) if sweep else {"ph": 0, "y0": None, "tgt": None}
    if sw.get("y0") is None:
        sw["y0"] = odom.yaw
    ref = sw.get("tgt") or _xy(pallet_ref)
    center = sw["y0"]
    if ref is not None and math.hypot(ref[0] - odom.x, ref[1] - odom.y) > 0.3:
        center = math.atan2(ref[1] - odom.y, ref[0] - odom.x)
    amp = math.radians(c["tag_sweep_deg"])
    targets = (amp, -amp, 0.0)
    ph = sw.get("ph", 0) % 3
    err = _wrap(center + targets[ph] - odom.yaw)
    if abs(err) <= math.radians(3.0):
        ph = (ph + 1) % 3
        sw["ph"] = ph
        err = _wrap(center + targets[ph] - odom.yaw)
    w = math.copysign(max(0.08, min(c["w_tag"], 1.5 * abs(err))), err)
    return Command(0.0, w), sw


def a42_hold():
    return _zero()


def a43_publish(est):
    return bool(est) and est["n"] >= 1


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    r = _xy(pallet_ref)
    if r is not None:
        known = [_xy(b) for b in bl]
        if not any(k is not None and math.hypot(k[0] - r[0], k[1] - r[1]) < 0.5
                   for k in known):
            bl.append(r)
    return _zero(), bl, True


# ----------------------------------------------------------------- G5 laws
def _law(px, py, pth, c, v_nom, a_max):
    """Line-following law towards the pocket centre line. Returns (v, w)."""
    cx, cy = _cam(px, py, pth)
    lim = math.radians(c["fov_half_deg"] - c["fov_margin_deg"]) - math.radians(4.0)
    beta = math.atan2(cx, max(cy, 0.2))
    a = _cl(math.atan(c["k_lat"] * cx), -a_max, a_max)
    a = _cl(a, beta - lim, beta + lim)
    e = _wrap((-_HALF_PI - a) - pth)
    w = _cl(c["k_head"] * e, -0.5, 0.5)
    v = v_nom * max(0.0, 1.0 - abs(e) / 0.5)
    return v, w


def _predock_ok(cx, cy, eh, c, tol_x):
    return (cy <= c["y_pre"] - CAM_X + 0.02 and abs(cx) <= tol_x
            and abs(eh) <= math.radians(15.0))


def _rollout(px, py, pth, c):
    lim = math.radians(c["fov_half_deg"] - c["fov_margin_deg"])
    cy_pre = c["y_pre"] - CAM_X
    for _ in range(500):
        cx, cy = _cam(px, py, pth)
        eh = _wrap(pth + _HALF_PI)
        if _predock_ok(cx, cy, eh, c, 0.06):
            return True
        if cy < cy_pre - 0.1:
            return False
        beta = math.atan2(cx, max(cy, 0.2))
        phi = -(pth + _HALF_PI) - beta
        if abs(phi) > lim:
            return False
        v, w = _law(px, py, pth, c, c["v_conv"], math.radians(35.0))
        pth = _wrap(pth + w * 0.1)
        px += v * math.cos(pth) * 0.1
        py += v * math.sin(pth) * 0.1
    return False


# ----------------------------------------------------------------- G5 steps
def a50_enter():
    return _zero(), {}, 0, True, False, False, True


def a51_plan(est, P):
    c = _cfg(P)
    if not est:
        return _zero(), False
    return _zero(), _rollout(est["px"], est["py"], est["pth"], c)


def a52_reposition(est, mstate, P):
    c = _cfg(P)
    ms = dict(mstate or {})
    if not est:
        return _zero(), ms, False
    px, py, pth = est["px"], est["py"], est["pth"]
    wx, wy = 0.0, c["y_pre"] + c["repos_extra"]
    ph = ms.get("rp", "go")
    if ph == "go":
        d = math.hypot(wx - px, wy - py)
        if d < 0.1:
            ph = "turn"
        else:
            e = _wrap(math.atan2(wy - py, wx - px) - pth)
            if abs(e) > 0.4:
                w = math.copysign(max(0.15, min(0.5, 1.5 * abs(e))), e)
                ms["rp"] = "go"
                return Command(0.0, w), ms, False
            v = min(c["v_conv"], 0.4 * d + 0.05)
            ms["rp"] = "go"
            return Command(v, _cl(1.5 * e, -0.5, 0.5)), ms, False
    e = _wrap(-_HALF_PI - pth)
    if abs(e) < math.radians(3.0):
        ms.pop("rp", None)
        return _zero(), ms, True
    ms["rp"] = "turn"
    w = math.copysign(max(0.12, min(0.5, 1.5 * abs(e))), e)
    return Command(0.0, w), ms, False


def a53_converge(est, P):
    c = _cfg(P)
    if not est:
        return _zero(), False, False
    px, py, pth = est["px"], est["py"], est["pth"]
    cx, cy = _cam(px, py, pth)
    eh = _wrap(pth + _HALF_PI)
    if _predock_ok(cx, cy, eh, c, 0.08):
        return _zero(), True, False
    if cy < c["y_pre"] - CAM_X - 0.1:
        return _zero(), False, True
    v, w = _law(px, py, pth, c, c["v_conv"], math.radians(35.0))
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    ms = dict(mstate or {})
    if ms.get("ra_last") is None or t - ms["ra_last"] > 0.5:
        ms["ra_t0"] = t
    ms["ra_last"] = t
    if not est:
        return Command(0.0, 0.2), ms
    cx, cy = _cam(est["px"], est["py"], est["pth"])
    phi = _wrap(math.atan2(-cy, -cx) - est["pth"])
    if abs(phi) > 0.06:
        w = math.copysign(max(0.1, min(0.3, 1.2 * abs(phi))), phi)
        return Command(0.0, w), ms
    el = t - ms["ra_t0"]
    w = 0.2 if math.sin(2.0 * math.pi * el / 4.0) >= 0 else -0.2
    return Command(0.0, w), ms


def a55_final_approach(est, P):
    c = _cfg(P)
    if not est:
        return _zero(), False
    px, py, pth = est["px"], est["py"], est["pth"]
    cx, cy = _cam(px, py, pth)
    cy_dock = c["dock_y"] - CAM_X
    eh = _wrap(pth + _HALF_PI)
    if cy <= cy_dock + 0.005:
        if abs(eh) <= math.radians(2.5):
            return _zero(), True
        w = math.copysign(max(0.05, min(0.3, c["k_head"] * abs(eh))), -eh)
        return Command(0.0, w), False
    v_nom = c["v_final"] * _cl(1.0 - (abs(cx) - 0.03) / 0.08, 0.4, 1.0)
    v, w = _law(px, py, pth, c, v_nom, math.radians(15.0))
    v = min(v, 0.7 * (cy - cy_dock) + 0.03)
    return Command(v, w), False


def a56_verify(est, mstate, t, P):
    c = _cfg(P)
    ms = dict(mstate or {})
    if not est:
        return _zero(), ms, False, True, False
    if ms.get("v_last") is None or t - ms["v_last"] > 0.25:
        ms["v_t0"] = t
        ms["v_n0"] = est["n_acc"]
    ms["v_last"] = t
    el = t - ms["v_t0"]
    cx, cy = _cam(est["px"], est["py"], est["pth"])
    eh = _wrap(est["pth"] + _HALF_PI)
    ok = (abs(cx) <= c["verify_lat"]
          and abs(eh) <= math.radians(c["verify_head_deg"])
          and abs(cy - DOCK_DIST) <= c["verify_dist"])
    seen = (est["n_acc"] - ms["v_n0"] >= 1) and (t - est["t_meas"] <= 0.6)
    if el >= c["verify_s"] - 1e-9 and ok and seen:
        ms.pop("v_t0", None)
        ms["v_last"] = None
        return _zero(), ms, True, False, True
    if el >= 0.9:
        ms.pop("v_t0", None)
        ms["v_last"] = None
        return _zero(), ms, False, True, False
    return _zero(), ms, False, False, False


def a57_backoff(odom, mstate, n_bo, P):
    c = _cfg(P)
    ms = dict(mstate or {})
    n = n_bo or 0
    if "bo_x" not in ms:
        ms["bo_x"], ms["bo_y"] = odom.x, odom.y
        n += 1
    bo_ok = n < c["max_backoff"]
    d = math.hypot(odom.x - ms["bo_x"], odom.y - ms["bo_y"])
    if d >= c["backoff_dist"]:
        ms.pop("bo_x", None)
        ms.pop("bo_y", None)
        return _zero(), ms, n, bo_ok, True
    v = -min(0.15, 0.5 * (c["backoff_dist"] - d) + 0.05)
    return Command(v, 0.0), ms, n, bo_ok, False


def a5e_fail():
    return _zero(), True
