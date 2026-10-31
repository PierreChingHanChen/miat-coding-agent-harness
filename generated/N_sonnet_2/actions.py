"""Action functions for the MIAT AGV pallet-docking controller (pure functions, state passed in/out)."""
import math
from sim.api import Command, FX, CAM_X, TAG_ID, DOCK_DIST

_DEF = {
    "w_search": 0.35, "score_min": 0.55, "width_min": 0.75, "width_max": 1.45,
    "confirm_k": 3, "confirm_window_s": 0.5, "confirm_tol_deg": 6.0,
    "track_gate_deg": 8.0, "track_lost_s": 2.0, "d_app": 1.9, "v_app": 0.25,
    "k_bear": 1.2, "app_bear_tol_deg": 4.0, "tag_sweep_deg": 35.0, "w_tag": 0.3,
    "init_meas": 3, "fuse_gain": 0.35, "gate_pos": 0.3, "gate_yaw_deg": 12.0,
    "pose_stale_s": 3.0, "near_dist": 1.1, "blind_max_s": 10.0,
    "fov_half_deg": 34.5, "fov_margin_deg": 10.0, "y_pre": 1.65, "repos_extra": 0.6,
    "v_conv": 0.2, "v_final": 0.12, "k_head": 1.6, "k_lat": 2.5, "dock_y": 1.13,
    "x_tol": 0.03, "head_tol_deg": 2.0, "verify_lat": 0.04, "verify_head_deg": 3.5,
    "verify_dist": 0.08, "verify_s": 0.5, "backoff_dist": 0.6, "max_backoff": 3,
    "max_retries": 2,
}


def _p(P, k):
    try:
        if isinstance(P, dict):
            if P.get(k) is not None:
                return P[k]
        elif P is not None and getattr(P, k, None) is not None:
            return getattr(P, k)
    except Exception:
        pass
    return _DEF[k]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _zero():
    return Command(0.0, 0.0)


def _rad(d):
    return math.radians(d)


# ----------------------------------------------------------------- G0
def a0_init():
    return _zero(), True


def a1_check_sensors(odom, scan):
    try:
        ok = (odom is not None and all(math.isfinite(float(v)) for v in (odom.x, odom.y, odom.yaw))
              and scan is not None and len(scan.ranges) > 0)
    except Exception:
        ok = False
    return bool(ok)


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
    return Command(0.0, float(_p(P, "w_search")))


def _depth(ds):
    v = sorted(d for d in (ds or []) if d is not None and math.isfinite(d) and d > 0.2)
    if len(v) < 3:
        return None
    r0 = v[len(v) // 5]
    sel = [d for d in v if r0 - 0.15 <= d <= r0 + 0.25]
    if len(sel) < 3:
        return None
    return sel[len(sel) // 2]


def a321_candidates(pallets, odom, blacklist, P):
    out = []
    yaw = odom.yaw
    cx = odom.x + CAM_X * math.cos(yaw)
    cy = odom.y + CAM_X * math.sin(yaw)
    for d in (pallets or []):
        try:
            if d.score < _p(P, "score_min"):
                continue
            z = _depth(d.depth_samples)
            if z is None or z > 7.0:
                continue
            wm = d.width_px * z / FX
            if not (_p(P, "width_min") <= wm <= _p(P, "width_max")):
                continue
            dx = (d.u - 320.0) / FX
            px = cx + z * math.cos(yaw) + z * dx * math.sin(yaw)
            py = cy + z * math.sin(yaw) - z * dx * math.cos(yaw)
            bad = False
            for b in (blacklist or []):
                if math.hypot(px - b["x"], py - b["y"]) < 1.0:
                    bad = True
                    break
            if bad:
                continue
            out.append({"x": px, "y": py, "bearing": _wrap(yaw - math.atan(dx)),
                        "range": z, "score": d.score, "width": wm})
        except Exception:
            continue
    out.sort(key=lambda c: -c["score"])
    return out


def a322_confirm(cands, hist, t, P):
    k = int(_p(P, "confirm_k"))
    win = _p(P, "confirm_window_s")
    tol = _rad(_p(P, "confirm_tol_deg"))
    h = [e for e in (hist or []) if 0 <= t - e["t"] <= win + 1e-9]
    for c in (cands or []):
        e = dict(c)
        e["t"] = t
        h.append(e)
    best = None
    for c in (cands or []):
        grp = [e for e in h if abs(_wrap(e["bearing"] - c["bearing"])) <= tol
               and math.hypot(e["x"] - c["x"], e["y"] - c["y"]) <= 0.8]
        ts = set(round(e["t"], 3) for e in grp)
        if len(ts) >= k:
            ms = sum(e["score"] for e in grp) / len(grp)
            if best is None or ms > best[0]:
                best = (ms, c)
    if best is None:
        return h, False, None
    c = best[1]
    track = {"x": c["x"], "y": c["y"], "bearing": c["bearing"], "range": c["range"],
             "score": c["score"], "t_seen": t, "t": t}
    return [], True, track


def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = _rad(_p(P, "track_gate_deg"))
    best = None
    for c in (cands or []):
        d = abs(_wrap(c["bearing"] - track["bearing"]))
        if d <= gate and math.hypot(c["x"] - track["x"], c["y"] - track["y"]) <= 1.0:
            if best is None or d < best[0]:
                best = (d, c)
    tr = dict(track)
    tr["t"] = t
    if best is not None:
        c = best[1]
        tr["x"] = 0.5 * tr["x"] + 0.5 * c["x"]
        tr["y"] = 0.5 * tr["y"] + 0.5 * c["y"]
        tr["bearing"] = c["bearing"]
        tr["range"] = c["range"]
        tr["t_seen"] = t
    lost = (t - tr["t_seen"]) > _p(P, "track_lost_s")
    return tr, bool(lost)


def a34_approach_cmd(track, odom, P):
    if track is None:
        return _zero(), False
    yaw = odom.yaw
    cx = odom.x + CAM_X * math.cos(yaw)
    cy = odom.y + CAM_X * math.sin(yaw)
    dx, dy = track["x"] - cx, track["y"] - cy
    rng = math.hypot(dx, dy)
    err = _wrap(math.atan2(dy, dx) - yaw)
    age = track["t"] - track["t_seen"]
    if rng <= _p(P, "d_app") and age <= 1.0:
        return _zero(), True
    if age > 1.0:
        return Command(0.0, _clamp(_p(P, "k_bear") * err, -0.3, 0.3)), False
    w = _clamp(_p(P, "k_bear") * err, -0.5, 0.5)
    lim = _rad(20.0)
    if abs(err) <= _rad(_p(P, "app_bear_tol_deg")):
        v = _p(P, "v_app")
    elif abs(err) >= lim:
        v = 0.0
    else:
        v = _p(P, "v_app") * (1.0 - abs(err) / lim)
    v = min(v, max(0.05, 0.5 * (rng - _p(P, "d_app")) + 0.05))
    return Command(v, w), False


def a35_exit(track, odom):
    if track is None:
        return _zero(), None, False
    ref = {"x": track["x"], "y": track["y"], "bearing": track["bearing"]}
    return _zero(), ref, True


def a3e_fail():
    return _zero(), True


# ----------------------------------------------------------------- G4
def _new_est(t, t0=None):
    return {"valid": False, "n": 0, "x": 0.0, "y": 0.0, "h": 0.0, "odom": None,
            "t": t, "t0": t if t0 is None else t0, "t_tag": None, "last": None,
            "rej": 0, "rej_pose": None, "v": 0.0, "w": 0.0}


def _meas(tag):
    yaw = tag.yaw
    px = -tag.z * math.sin(yaw) + tag.x * math.cos(yaw)
    py = tag.z * math.cos(yaw) + tag.x * math.sin(yaw)
    h = yaw - math.pi / 2
    return px - CAM_X * math.cos(h), py - CAM_X * math.sin(h), h


def a43_update_pose(est, odom, tags, t, P):
    e = dict(est) if est else _new_est(t)
    # dead reckoning
    if odom is not None:
        cur = (odom.x, odom.y, odom.yaw)
        if e["odom"] is not None and e["n"] > 0:
            ox, oy, oyaw = e["odom"]
            dx, dy = cur[0] - ox, cur[1] - oy
            df = dx * math.cos(oyaw) + dy * math.sin(oyaw)
            dl = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
            dyaw = _wrap(cur[2] - oyaw)
            hm = e["h"] + dyaw / 2.0
            e["x"] += df * math.cos(hm) - dl * math.sin(hm)
            e["y"] += df * math.sin(hm) + dl * math.cos(hm)
            e["h"] = _wrap(e["h"] + dyaw)
        e["odom"] = cur
        e["v"] = odom.v
        e["w"] = odom.w
    e["t"] = t
    init_n = int(_p(P, "init_meas"))
    for tag in sorted(tags or [], key=lambda a: a.stamp):
        if tag.id != TAG_ID:
            continue
        key = (tag.stamp, tag.x, tag.z, tag.yaw)
        if key == e["last"]:
            continue
        e["last"] = key
        if t - tag.stamp > 0.5 or not (0.4 < tag.z < 5.0) or abs(tag.yaw) > _rad(70):
            continue
        mx, my, mh = _meas(tag)
        if e["n"] == 0:
            e["x"], e["y"], e["h"], e["n"] = mx, my, mh, 1
            e["t_tag"] = t
            e["rej"] = 0
        else:
            age = (t - e["t_tag"]) if e["t_tag"] is not None else 0.0
            sc = 1.0 + min(2.0, age / 3.0)
            dpos = math.hypot(mx - e["x"], my - e["y"])
            dh = abs(_wrap(mh - e["h"]))
            if dpos <= _p(P, "gate_pos") * sc and dh <= _rad(_p(P, "gate_yaw_deg")) * sc:
                g = max(_p(P, "fuse_gain"), 1.0 / (e["n"] + 1)) if e["n"] < init_n else _p(P, "fuse_gain")
                e["x"] += g * (mx - e["x"])
                e["y"] += g * (my - e["y"])
                e["h"] = _wrap(e["h"] + g * _wrap(mh - e["h"]))
                e["n"] = min(e["n"] + 1, 1000)
                e["t_tag"] = t
                e["rej"] = 0
                e["rej_pose"] = None
            else:
                rp = e["rej_pose"]
                if rp is not None and math.hypot(mx - rp[0], my - rp[1]) <= 0.3 and abs(_wrap(mh - rp[2])) <= _rad(12):
                    e["rej"] += 1
                else:
                    e["rej"] = 1
                e["rej_pose"] = (mx, my, mh)
                if e["rej"] >= 3:
                    e["x"], e["y"], e["h"] = mx, my, mh
                    e["n"] = max(1, init_n) if e["n"] >= init_n else 1
                    e["t_tag"] = t
                    e["rej"] = 0
                    e["rej_pose"] = None
    e["valid"] = e["n"] >= init_n
    return e


def _deadline(e):
    t0 = e.get("t0", 0.0)
    if t0 < 5.0:
        return 98.0
    if t0 < 100.0:
        return 128.0
    return None


def a44_pose_status(est, t, P):
    if not est or est.get("t_tag") is None:
        return False, False, True
    age = t - est["t_tag"]
    tag_ok = age <= 0.6
    ready = bool(est["valid"]) and age <= 1.0
    cx = est["x"] + CAM_X * math.cos(est["h"])
    cy = est["y"] + CAM_X * math.sin(est["h"])
    lim = _p(P, "blind_max_s") if math.hypot(cx, cy) < _p(P, "near_dist") else _p(P, "pose_stale_s")
    stale = (not est["valid"]) or age > lim
    dl = _deadline(est)
    if dl is not None and t >= dl:
        stale = True
    return bool(tag_ok), bool(ready), bool(stale)


def a40_enter(n_loc, est, P):
    n = int(n_loc or 0) + 1
    e0 = est or _new_est(0.0)
    t = e0.get("t", 0.0)
    ne = _new_est(t, e0.get("t0", t))
    ne["odom"] = e0.get("odom")
    ne["last"] = e0.get("last")
    retry = n <= int(_p(P, "max_retries"))
    dl = _deadline(ne)
    if dl is not None and dl - t < 40.0:
        retry = False
    return _zero(), n, bool(retry), ne, None, False, False, True


def a41_tag_sweep(odom, pallet_ref, sweep, P):
    S = _rad(_p(P, "tag_sweep_deg"))
    wt = _p(P, "w_tag")
    yaw = odom.yaw
    sw = dict(sweep) if sweep else None
    if sw is None:
        c = yaw
        if pallet_ref:
            cx = odom.x + CAM_X * math.cos(yaw)
            cy = odom.y + CAM_X * math.sin(yaw)
            dx, dy = pallet_ref["x"] - cx, pallet_ref["y"] - cy
            if math.hypot(dx, dy) > 0.3:
                c = math.atan2(dy, dx)
        sw = {"c": c, "phase": 0}
    offs = [0.0, S, -S, 0.0]
    for _ in range(4):
        ph = sw["phase"]
        err = _wrap(sw["c"] + offs[ph] - yaw)
        if abs(err) < _rad(3.0):
            sw["phase"] = 1 if ph == 3 else ph + 1
            continue
        w = math.copysign(_clamp(abs(err) * 2.0, 0.12, wt), err)
        return Command(0.0, w), sw
    return _zero(), sw


def a42_hold():
    return _zero()


def a43_publish(est):
    if not est or not est.get("valid"):
        return False
    cy = est["y"] + CAM_X * math.sin(est["h"])
    cx = est["x"] + CAM_X * math.cos(est["h"])
    return bool(0.3 < cy < 6.0 and abs(cx) < 5.0)


def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref:
        if not any(math.hypot(pallet_ref["x"] - b["x"], pallet_ref["y"] - b["y"]) < 0.5 for b in bl):
            bl.append({"x": pallet_ref["x"], "y": pallet_ref["y"]})
    return _zero(), bl, True


# ----------------------------------------------------------------- G5
def _cam(est):
    h = est["h"]
    return est["x"] + CAM_X * math.cos(h), est["y"] + CAM_X * math.sin(h)


def _tag_az(x, y, h):
    cx, cy = x + CAM_X * math.cos(h), y + CAM_X * math.sin(h)
    return _wrap(math.atan2(-cy, -cx) - h), math.hypot(cx, cy)


def _law(x, y, h, P):
    kl, kh, vc = _p(P, "k_lat"), _p(P, "k_head"), _p(P, "v_conv")
    ypre = _p(P, "y_pre")
    xc = x + CAM_X * math.cos(h)
    delta = _clamp(math.atan(kl * xc), -_rad(30), _rad(30))
    e = _wrap(-math.pi / 2 - delta - h)
    w = _clamp(kh * e, -0.5, 0.5)
    vb = vc if y > ypre + 0.5 else 0.10 + (vc - 0.10) * max(0.0, (y - ypre) / 0.5)
    v = vb * max(0.0, 1.0 - abs(e) / _rad(25))
    return v, w, xc, _wrap(h + math.pi / 2)


def _aligned(y, xc, eh, P):
    ypre, dy = _p(P, "y_pre"), _p(P, "dock_y")
    f = _clamp((ypre - y) / (ypre - (dy + 0.2)), 0.0, 1.0)
    sc = 1.0 + 0.7 * f
    return abs(xc) <= _p(P, "x_tol") * sc and abs(eh) <= _rad(_p(P, "head_tol_deg")) * sc


def _sim_ok(x, y, h, P):
    lim = _rad(_p(P, "fov_half_deg") - _p(P, "fov_margin_deg"))
    a0, _r = _tag_az(x, y, h)
    if abs(a0) > _rad(_p(P, "fov_half_deg")):
        return False
    ypre, dy = _p(P, "y_pre"), _p(P, "dock_y")
    for _ in range(500):
        v, w, xc, eh = _law(x, y, h, P)
        if y <= ypre + 0.1 and _aligned(y, xc, eh, P):
            return True
        if y < dy + 0.2:
            return False
        if v > 0.0:
            a, r = _tag_az(x, y, h)
            if r > 0.9 and abs(a) > lim:
                return False
        x += v * 0.1 * math.cos(h)
        y += v * 0.1 * math.sin(h)
        h = _wrap(h + w * 0.1)
    return False


def a50_enter():
    return _zero(), {}, 0, True, False, False, True


def a51_plan(est, P):
    if not est or not est.get("valid"):
        return _zero(), False
    return _zero(), bool(_sim_ok(est["x"], est["y"], est["h"], P))


def a52_reposition(est, mstate, P):
    m = dict(mstate or {})
    if m.get("phase") != "repos":
        m = {"phase": "repos", "sub": "turn1"}
    wx, wy = 0.0, _p(P, "y_pre") + _p(P, "repos_extra")
    x, y, h = est["x"], est["y"], est["h"]
    dx, dy = wx - x, wy - y
    dist = math.hypot(dx, dy)
    sub = m["sub"]
    if sub == "turn1":
        if dist < 0.08:
            m["sub"] = sub = "turn2"
        else:
            err = _wrap(math.atan2(dy, dx) - h)
            if abs(err) < _rad(4):
                m["sub"] = sub = "drive"
            else:
                return Command(0.0, math.copysign(_clamp(abs(err) * 1.5, 0.15, 0.5), err)), m, False
    if sub == "drive":
        if dist < 0.06:
            m["sub"] = sub = "turn2"
        else:
            err = _wrap(math.atan2(dy, dx) - h)
            if abs(err) > _rad(30):
                m["sub"] = "turn1"
                return _zero(), m, False
            v = min(0.2, max(0.06, dist))
            return Command(v * max(0.0, 1 - abs(err) / _rad(30)), _clamp(1.5 * err, -0.5, 0.5)), m, False
    err = _wrap(-math.pi / 2 - h)
    if abs(err) < _rad(2.0):
        return _zero(), m, True
    return Command(0.0, math.copysign(_clamp(abs(err) * 1.5, 0.12, 0.5), err)), m, False


def a53_converge(est, P):
    if not est:
        return _zero(), False, False
    x, y, h = est["x"], est["y"], est["h"]
    v, w, xc, eh = _law(x, y, h, P)
    ypre, dy = _p(P, "y_pre"), _p(P, "dock_y")
    al = _aligned(y, xc, eh, P)
    if y <= ypre + 0.1 and al:
        return _zero(), True, False
    if y < dy + 0.2:
        return _zero(), False, True
    return Command(v, w), False, False


def a54_reacquire(est, mstate, t, P):
    m = dict(mstate or {})
    if m.get("phase") != "reacq":
        m = {"phase": "reacq", "t0": t}
    if not est:
        return _zero(), m
    age = (t - est["t_tag"]) if est.get("t_tag") is not None else 1e9
    if age <= 0.6:
        return _zero(), m
    a, _r = _tag_az(est["x"], est["y"], est["h"])
    if abs(a) > _rad(3.0):
        return Command(0.0, math.copysign(_clamp(abs(a) * 1.5, 0.1, 0.3), a)), m
    if t - m["t0"] > 2.0:
        return Command(0.0, 0.3 * math.sin(2 * math.pi * (t - m["t0"] - 2.0) / 6.0)), m
    return _zero(), m


def a55_final_approach(est, P):
    if not est:
        return _zero(), False
    x, y, h = est["x"], est["y"], est["h"]
    cx, cy = _cam(est)
    err = cy - DOCK_DIST
    if abs(err) <= 0.02:
        return _zero(), True
    kl, kh = _p(P, "k_lat"), _p(P, "k_head")
    if err > 0:
        v = min(_p(P, "v_final"), max(0.04, 1.5 * err))
        delta = _clamp(math.atan(kl * cx), -_rad(15), _rad(15))
    else:
        v = -0.04
        delta = 0.0
    w = _clamp(kh * _wrap(-math.pi / 2 - delta - h), -0.4, 0.4)
    return Command(v, w), False


def a56_verify(est, mstate, t, P):
    m = dict(mstate or {})
    if m.get("phase") != "verify":
        m = {"phase": "verify", "t0": t, "ok_t0": None}
    if not est:
        return _zero(), m, False, True, False
    cx, cy = _cam(est)
    eh = _wrap(est["h"] + math.pi / 2)
    age = (t - est["t_tag"]) if est.get("t_tag") is not None else 1e9
    ok = (est["valid"] and age <= 0.35 and abs(est.get("v", 0.0)) < 0.03 and abs(est.get("w", 0.0)) < 0.05
          and abs(cx) <= _p(P, "verify_lat") and abs(eh) <= _rad(_p(P, "verify_head_deg"))
          and abs(cy - DOCK_DIST) <= _p(P, "verify_dist"))
    ver = False
    if ok:
        if m["ok_t0"] is None:
            m["ok_t0"] = t
        if t - m["ok_t0"] >= _p(P, "verify_s") - 1e-9:
            ver = True
    else:
        m["ok_t0"] = None
    bad = (not ver) and (t - m["t0"] >= 4.0)
    return _zero(), m, ver, bad, ver


def a57_backoff(odom, mstate, n_bo, P):
    m = dict(mstate or {})
    n = int(n_bo or 0)
    if m.get("phase") != "backoff":
        n += 1
        m = {"phase": "backoff", "x0": odom.x, "y0": odom.y}
    bo_ok = n < int(_p(P, "max_backoff"))
    d = math.hypot(odom.x - m["x0"], odom.y - m["y0"])
    if d >= _p(P, "backoff_dist"):
        return _zero(), m, n, bo_ok, True
    return Command(-0.15, 0.0), m, n, bo_ok, False


def a5e_fail():
    return _zero(), True
