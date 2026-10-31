"""MIAT 2.0 reference Action/Datapath functions (generated in this session by an
LLM under the interface contracts of contracts/ACTION_CONTRACTS.md, then reviewed
with the four white-box checks and the unit tests in tests/test_actions.py).

Rules imposed by the contract:
  * pure functions: every piece of state is passed in and returned explicitly
  * no hidden globals, no sleeps/blocking loops: one call = one 10 Hz tick
  * only the platform API in sim/api.py may be used
  * frames: pallet frame = AprilTag at origin, +y out of the pocket face;
    'pose' = AGV base_link (px, py, yaw) in that frame; bearings are + to the RIGHT
"""
import math
from sim.api import Command, FX, IMG_W, CAM_X, TAG_ID


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _sat(x, lim):
    return max(-lim, min(lim, x))


def _median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _cam(px, py, yaw):
    return px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)


# ============================ G0 =============================================
def a0_init():
    return Command(), True

def a1_check_sensors(odom, scan):
    return odom is not None and scan is not None

def a2_navigate():
    return True          # Nav2 (unchanged subsystem A2) — charged by the simulator

def a6_docked():
    return Command()

def a9_abort():
    return Command()


# ============================ A3 ==============================================
def a30_enter():
    return Command(), None, [], False, False, True

def a31_search_rotate(P):
    return Command(0.0, P["w_search"])

def a331_bbox_center(det):
    return float(det.u)

def a332_pallet_distance(depth_samples):
    """median of valid depth pixels after rejecting background outliers -> (dist [m], ok)"""
    valid = [d for d in depth_samples if d > 0.05]
    if len(valid) < 5:
        return 0.0, False
    m = _median(valid)
    inl = [d for d in valid if abs(d - m) <= 0.3]
    if len(inl) < 3:
        return 0.0, False
    return _median(inl), True

def a333_pallet_bearing(u):
    """bbox column -> bearing [rad], + = right of the optical axis"""
    return math.atan((u - IMG_W / 2) / FX)

def a321_candidates(pallets, odom, blacklist, P):
    out = []
    cx, cy = odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw)
    for det in pallets:
        if det.score < P["score_min"]:
            continue
        d, ok = a332_pallet_distance(det.depth_samples)
        if not ok:
            continue
        b = a333_pallet_bearing(a331_bbox_center(det))
        W = det.width_px * d / FX
        if not (P["width_min"] <= W <= P["width_max"]):
            continue
        ba = wrap(odom.yaw - b)
        pos = (cx + d * math.cos(ba), cy + d * math.sin(ba))
        if any(math.hypot(pos[0] - q[0], pos[1] - q[1]) < 0.8 for q in (blacklist or [])):
            continue
        out.append(dict(ba=ba, d=d, b=b, W=W, score=det.score, pos=pos))
    return out

def a322_confirm(cands, hist, t, P):
    """k-of-window temporal confirmation in the odometry frame"""
    hist = [h for h in (hist or []) if t - h["t"] <= P["confirm_window_s"] + 1e-6]
    hist = hist + [dict(t=t, c=c) for c in cands]
    tol = math.radians(P["confirm_tol_deg"])
    best, best_n = None, 0
    for c in cands:
        frames = {round(h["t"], 3) for h in hist
                  if abs(wrap(h["c"]["ba"] - c["ba"])) <= tol and abs(h["c"]["d"] - c["d"]) <= 0.4}
        if len(frames) > best_n or (len(frames) == best_n and best is not None and c["score"] > best["score"]):
            best, best_n = c, len(frames)
    if best is not None and best_n >= P["confirm_k"]:
        return hist, True, dict(ba=best["ba"], d=best["d"], pos=best["pos"], t=t)
    return hist, False, None

def a323_track(cands, track, t, P):
    if track is None:
        return None, True
    gate = math.radians(P["track_gate_deg"])
    m = [c for c in cands if abs(wrap(c["ba"] - track["ba"])) <= gate and abs(c["d"] - track["d"]) < 0.6]
    if m:
        c = min(m, key=lambda c: abs(wrap(c["ba"] - track["ba"])))
        ba = wrap(track["ba"] + 0.5 * wrap(c["ba"] - track["ba"]))
        pos = (0.5 * (track["pos"][0] + c["pos"][0]), 0.5 * (track["pos"][1] + c["pos"][1]))
        track = dict(ba=ba, d=c["d"], pos=pos, t=t)
    return track, (t - track["t"]) > P["track_lost_s"]

def a34_approach_cmd(track, odom, P):
    if track is None:
        return Command(), False
    cx, cy = odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw)
    dx, dy = track["pos"][0] - cx, track["pos"][1] - cy
    d = math.hypot(dx, dy)
    b = wrap(odom.yaw - math.atan2(dy, dx))          # + right
    w = _sat(-P["k_bear"] * b, 0.5)
    if d <= P["d_app"]:
        done = abs(b) <= math.radians(P["app_bear_tol_deg"])
        return Command(0.0, 0.0 if done else _sat(-1.5 * b, 0.4)), done
    v = P["v_app"] if abs(b) < math.radians(10) else 0.05
    return Command(v, w), False

def a35_exit(track, odom):
    return Command(), dict(ba=track["ba"], pos=track["pos"]), True

def a3e_fail():
    return Command(), True


# ============================ A4 ==============================================
def a40_enter(n_loc, est, P):
    n = n_loc + 1
    return Command(), n, n <= P["max_retries"], None, None, False, False, True

def a41_tag_sweep(odom, pallet_ref, sweep, P):
    """look toward the remembered pallet, then sweep +-tag_sweep_deg around it"""
    if sweep is None:
        if pallet_ref is not None:
            cx, cy = odom.x + CAM_X * math.cos(odom.yaw), odom.y + CAM_X * math.sin(odom.yaw)
            ref = math.atan2(pallet_ref["pos"][1] - cy, pallet_ref["pos"][0] - cx)
        else:
            ref = odom.yaw
        a = math.radians(P["tag_sweep_deg"])
        sweep = dict(targets=[ref, ref + a, ref - a, ref], i=0)
    sweep = dict(sweep)
    tgt = sweep["targets"][sweep["i"]]
    e = wrap(tgt - odom.yaw)
    if abs(e) < math.radians(3):
        sweep["i"] = (sweep["i"] + 1) % len(sweep["targets"])
        return Command(), sweep
    return Command(0.0, _sat(2.0 * e, P["w_tag"])), sweep

def a42_tag_to_pose(tag):
    """AprilTag in camera frame -> AGV base_link pose (px, py, yaw) in the pallet frame"""
    yaw = wrap(tag.yaw - math.pi / 2)
    c, s = math.cos(yaw), math.sin(yaw)
    px = -(tag.z * c + tag.x * s) - CAM_X * c
    py = -(tag.z * s - tag.x * c) - CAM_X * s
    return px, py, yaw

def a43_update_pose(est, odom, tags, t, P):
    """odometry-propagated, outlier-gated complementary filter of the AGV pose"""
    meas = [a42_tag_to_pose(g) for g in tags if g.id == TAG_ID]
    if est is None:
        if not meas:
            return None
        px, py, yaw = meas[0]
        return dict(px=px, py=py, yaw=yaw, t_meas=t, n=1, rej=0, o=(odom.x, odom.y, odom.yaw))
    e = dict(est)
    ox, oy, oyaw = e["o"]
    dx, dy = odom.x - ox, odom.y - oy
    f = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    l = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    e["px"] += f * math.cos(e["yaw"]) - l * math.sin(e["yaw"])
    e["py"] += f * math.sin(e["yaw"]) + l * math.cos(e["yaw"])
    e["yaw"] = wrap(e["yaw"] + wrap(odom.yaw - oyaw))
    e["o"] = (odom.x, odom.y, odom.yaw)
    for z in meas[:1]:
        ex, ey, eyaw = z[0] - e["px"], z[1] - e["py"], wrap(z[2] - e["yaw"])
        dist = math.hypot(*_cam(e["px"], e["py"], e["yaw"]))
        if math.hypot(ex, ey) <= P["gate_pos"] + 0.1 * dist and abs(eyaw) <= math.radians(P["gate_yaw_deg"]):
            k = 0.5 if e["n"] < P["init_meas"] else P["fuse_gain"]
            e["px"] += k * ex; e["py"] += k * ey; e["yaw"] = wrap(e["yaw"] + k * eyaw)
            e["t_meas"], e["n"], e["rej"] = t, e["n"] + 1, 0
        else:
            e["rej"] += 1
            if e["rej"] >= 5:
                e.update(px=z[0], py=z[1], yaw=z[2], t_meas=t, n=1, rej=0)
    return e

def a44_pose_status(est, t, P):
    """-> (tag_ok: fresh tag fix, pose_ready: filter initialised, pose_stale)"""
    if est is None:
        return False, False, True
    age = t - est["t_meas"]
    fresh = age < 0.35
    dist = math.hypot(*_cam(est["px"], est["py"], est["yaw"]))
    stale = (age > P["pose_stale_s"] and dist > P["near_dist"]) or age > P["blind_max_s"]
    return fresh, fresh and est["n"] >= P["init_meas"], stale

def a42_hold():
    return Command()

def a43_publish(est):
    return est is not None

def a4e_blacklist(blacklist, pallet_ref):
    bl = list(blacklist or [])
    if pallet_ref is not None:
        bl.append(tuple(pallet_ref["pos"]))
    return Command(), bl, True


# ============================ A5 ==============================================
def _beta_max(P):
    return math.radians(P["fov_half_deg"] - P["fov_margin_deg"])

def a50_enter():
    return Command(), None, 0, True, False, False, True

def a51_plan(est, P):
    """feasible <=> a straight path to the pre-dock point keeps the tag inside the FOV margin"""
    if est is None:
        return Command(), False
    px, py = est["px"], est["py"]
    if abs(px) <= 0.05 and abs(py - P["y_pre"]) <= 0.15:
        return Command(), True
    if py <= P["y_pre"] + 0.15:
        return Command(), False
    gamma = math.atan2(abs(px), py - P["y_pre"])
    return Command(), gamma <= _beta_max(P) - math.radians(2)

def _rotate(err, w=0.5):
    return Command(0.0, _sat(1.8 * err, w) if abs(err) > 0.12 else math.copysign(0.15, err))

def a52_reposition(est, ms, P):
    """odometry maneuver to a waypoint from which the pre-dock path is feasible"""
    px, py, yaw = est["px"], est["py"], est["yaw"]
    if ms is None or ms.get("own") != "52":
        yw = max(py, P["y_pre"] + P["repos_extra"])
        xw = math.copysign((yw - P["y_pre"]) * math.tan(_beta_max(P) - math.radians(8)), px if px else 1.0)
        ms = dict(own="52", ph="turn1", W=(xw, yw))
    ms = dict(ms)
    xw, yw = ms["W"]
    dist = math.hypot(xw - px, yw - py)
    if ms["ph"] == "turn1":
        if dist < 0.06:
            ms["ph"] = "turn2"
        else:
            e = wrap(math.atan2(yw - py, xw - px) - yaw)
            if abs(e) < math.radians(3):
                ms["ph"] = "drive"
            else:
                return _rotate(e), ms, False
    if ms["ph"] == "drive":
        if dist < 0.06:
            ms["ph"] = "turn2"
        else:
            e = wrap(math.atan2(yw - py, xw - px) - yaw)
            return Command(0.2 * max(0.0, math.cos(e)), _sat(1.5 * e, 0.5)), ms, False
    cx, cy = _cam(px, py, yaw)
    e = wrap(math.atan2(-cy, -cx) - yaw)
    if abs(e) < math.radians(3):
        return Command(), None, True
    return _rotate(e), ms, False

def a54_reacquire(est, ms, t, P):
    """turn the camera toward the tag (from the propagated pose) and wait for a fresh fix"""
    if est is None:
        return Command(0.0, 0.2), None
    cx, cy = _cam(est["px"], est["py"], est["yaw"])
    e = wrap(math.atan2(-cy, -cx) - est["yaw"])
    if abs(e) > math.radians(4):
        return _rotate(e, 0.4), None
    return Command(), None

def a53_converge(est, P):
    """drive to the pre-dock point on the centre line while holding the tag inside the FOV margin"""
    px, py, yaw = est["px"], est["py"], est["yaw"]
    yp = P["y_pre"]
    if abs(px) <= 0.05 and py <= yp + 0.08:
        return Command(), True, False
    if py < yp - 0.25 and abs(px) > 0.10:
        return Command(), False, True
    cx, cy = _cam(px, py, yaw)
    psi_tag = math.atan2(-cy, -cx)
    ty = yp if py > yp else py - 0.3
    psi_d = math.atan2(ty - py, -px)
    bm = _beta_max(P)
    psi_d = psi_tag + _sat(wrap(psi_d - psi_tag), bm)
    e = wrap(psi_d - yaw)
    d = math.hypot(px, py - yp)
    v = P["v_conv"] * max(0.0, math.cos(e)) * min(1.0, max(0.3, d / 0.4))
    return Command(v, _sat(P["k_head"] * e, 0.6)), False, False

def a55_final_approach(est, P):
    """centre-line approach to the docking distance; continues on odometry when the tag is too close"""
    px, py, yaw = est["px"], est["py"], est["yaw"]
    ey = py - P["dock_y"]
    if ey > 0.01:
        psi_d = -math.pi / 2 - _sat(math.atan(P["k_lat"] * px), math.radians(30))
        e = wrap(psi_d - yaw)
        v = P["v_final"] * min(1.0, max(0.25, ey / 0.25)) * max(0.0, math.cos(e))
        return Command(v, _sat(P["k_head"] * e, 0.5)), False
    e = wrap(-math.pi / 2 - yaw)
    if abs(e) > math.radians(P["head_tol_deg"]):
        return _rotate(e, 0.3), False
    return Command(), True

def a56_verify(est, ms, t, P):
    if ms is None or ms.get("own") != "56":
        ms = dict(own="56", t0=t)
    if t - ms["t0"] < P["verify_s"]:
        return Command(), ms, False, False, False
    ok = (abs(est["px"]) <= P["verify_lat"] and abs(est["py"] - P["dock_y"]) <= P["verify_dist"]
          and abs(wrap(est["yaw"] + math.pi / 2)) <= math.radians(P["verify_head_deg"]))
    return Command(), None, ok, not ok, ok

def a57_backoff(odom, ms, n_bo, P):
    if ms is None or ms.get("own") != "57":
        ms = dict(own="57", x0=odom.x, y0=odom.y)
        n_bo = n_bo + 1
    bo_ok = n_bo < P["max_backoff"]
    if math.hypot(odom.x - ms["x0"], odom.y - ms["y0"]) >= P["backoff_dist"]:
        return Command(), None, n_bo, bo_ok, True
    return Command(-0.15, 0.0), ms, n_bo, bo_ok, False

def a5e_fail():
    return Command(), True


# ============================ CR-1: safety supervisor (A6) ===================
def as1_obstacle_monitor(scan, sstate, t, P):
    near = False
    if scan is not None:
        lim = math.radians(P["safe_sector_deg"])
        for i, r in enumerate(scan.ranges):
            a = wrap(scan.angle_min + i * scan.angle_increment)
            if abs(a) <= lim and math.isfinite(r) and r < P["safe_dist"]:
                near = True
                break
    s = dict(sstate or {})
    if near:
        s["last_near"] = t
    last = s.get("last_near")
    clear = (not near) and (last is None or t - last >= P["clear_s"])
    return s, near, clear

def as2_inhibit_forward(cmd):
    return Command(min(cmd.v, 0.0), cmd.w)
