"""Re-implementation of the ORIGINAL design's action functions, following the
paper (Sec. III-C/D) and the thesis (Sec. 3.3-3.5).  Numeric thresholds that the
paper does not report were chosen to be physically plausible (see controls in
original_grafcet.json).  Velocity commands are 'held' (ROS cmd_vel semantics of
the Gazebo diff-drive plugin): an action that does not publish leaves the last
command in force."""
import math
from sim.api import Command, FX, IMG_W, DT, CAM_X


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tag_to_pose(tag):
    """AprilTag (camera frame) -> AGV base_link pose (px, py, yaw) in the pallet frame (via TF)."""
    yaw = wrap(tag.yaw - math.pi / 2)
    c, s = math.cos(yaw), math.sin(yaw)
    px = -(tag.z * c + tag.x * s) - CAM_X * c
    py = -(tag.z * s - tag.x * c) - CAM_X * s
    return (px, py, yaw)


# ---------------- G0 / A1 / A2 (Nav2 stub) -----------------------------------
def a0_init():
    return True, Command()

def a1_init_sensors(odom, scan):
    return odom is not None and scan is not None

def a21_acquire_scan(scan):
    return scan is not None

def a22_build_map():
    return True            # pre-built Gmapping map (paper Sec. V-D)

def a23_navigate():
    return True            # Nav2 drive to viewpoint: charged as nav_time by the simulator

def a24_exit():
    return True


# ---------------- G3: recognize pallet and approach ---------------------------
def a31_rotate(P):
    return Command(0.0, P["w_search"]), True

def a32_recognize(pallets, P):
    c = [d for d in pallets if d.score >= P["score_min"]]
    if not c:
        return None, False
    return max(c, key=lambda d: d.score), True

def a331_center(det):
    return (det.u, True) if det is not None else (IMG_W / 2, False)

def a332_distance(det):
    s = sorted(det.depth_samples)
    return s[len(s) // 2]          # median of n sampled pixels around P (thesis Sec. 3.3.1)

def a333_bearing(u):
    return math.atan((u - IMG_W / 2) / FX)

def a334_exit():
    return True

def a34_approach(dist, ang, P):
    if dist <= P["approach_thr"]:
        return Command(), True
    return Command(P["v_pallet"], -P["k_ang"] * ang), False

def a35_exit():
    return Command()


# ---------------- G4: localize pocket centre ----------------------------------
def a41_rotate(P):
    return Command(0.0, P["w_search"]), True

def a42_detect_tag(tags):
    return (tags[0], True) if tags else (None, False)

def a43_publish(tag):
    return tag_to_pose(tag), Command()

def a44_exit():
    return True


# ---------------- G5: pocket guidance -----------------------------------------
def a50_enter():
    return Command(), {}

def a51_searching(tags, m51, t, P):
    """rotate to search for the tag; hold still for search_hold s after the last sighting"""
    m51 = dict(m51 or {})
    if not tags:
        if t - m51.get("seen", -1e9) < P["search_hold"]:
            return Command(), False, False, None, m51
        return Command(0.0, P["w_search"]), False, False, None, m51
    m51["seen"] = t
    pose = tag_to_pose(tags[0])
    alpha = math.degrees(math.atan2(abs(pose[0]), pose[1]))
    return Command(), True, alpha <= P["angle_ok_deg"], pose, m51


def _rotate_to(odom, target_odom_yaw, w=0.4):
    e = wrap(target_odom_yaw - odom.yaw)
    if abs(e) < math.radians(2):
        return Command(), True
    return Command(0.0, max(-w, min(w, 1.5 * e)) if abs(e) > 0.15 else math.copysign(0.12, e)), False


def a52_angle_adjust(pose, odom, ms, P):
    """turn adj_turn_deg toward the pocket's perpendicular bisector, drive adj_dist (blocking)"""
    ms = dict(ms or {})
    if ms.get("ph") is None:
        px, py, yaw = pose
        psi_tag = math.atan2(-py, -px)
        sign = -1.0 if px > 0 else 1.0
        tgt_world = psi_tag + sign * math.radians(P["adj_turn_deg"])
        ms.update(ph="turn", tgt=wrap(odom.yaw + (tgt_world - yaw)))
    if ms["ph"] == "turn":
        cmd, done = _rotate_to(odom, ms["tgt"])
        if done:
            ms.update(ph="drive", x0=odom.x, y0=odom.y)
        return cmd, ms, False
    if ms["ph"] == "drive":
        if math.hypot(odom.x - ms["x0"], odom.y - ms["y0"]) >= P["adj_dist"]:
            return Command(), {}, True
        return Command(0.2, 0.0), ms, False
    return Command(), {}, True


def a53_rotate_to_line(tags, odom, ms, P):
    """rotate toward the centre line while keeping the tag in view (blocking)"""
    ms = dict(ms or {})
    if ms.get("ph") is None:
        if not tags:
            return Command(), {}, False, None, True
        tg = tags[0]
        px, py, yaw = tag_to_pose(tg)
        bearing = math.atan2(tg.x, tg.z)            # tag bearing seen by the camera (+ right)
        psi_tag = wrap(yaw - bearing)               # world direction camera -> tag
        lx, ly = 0.0, py - P["line_lookahead"]
        psi_d = math.atan2(ly - py, lx - px)
        lim = math.radians(P["align_bearing_max_deg"])
        dpsi = max(-lim, min(lim, wrap(psi_d - psi_tag)))
        ms.update(ph="rot", tgt=wrap(odom.yaw + (psi_tag + dpsi - yaw)))
    cmd, done = _rotate_to(odom, ms["tgt"])
    if done:
        inview = bool(tags)
        pose = tag_to_pose(tags[0]) if tags else None
        return Command(), {}, inview, pose, True
    return cmd, ms, False, None, False


def a54_approaching(tags, odom, ms, P):
    """drive forward step_dist, then report tag_in_view / theta_ok (blocking)"""
    ms = dict(ms or {})
    if ms.get("ph") is None:
        ms.update(ph="drv", x0=odom.x, y0=odom.y, last=None, miss=0, first=None)
    w = 0.0
    if tags:
        ms["last"] = tag_to_pose(tags[0]); ms["miss"] = 0
        b = math.atan2(tags[0].x, tags[0].z)
        if ms["first"] is None:
            ms["first"] = ms["last"]; ms["b0"] = b
        w = -1.5 * (b - ms["b0"])         # continuous heading correction: hold the tag bearing
    else:
        ms["miss"] += 1
    if math.hypot(odom.x - ms["x0"], odom.y - ms["y0"]) >= P["step_dist"]:
        inview = ms["miss"] < 3 and ms["last"] is not None
        pose = ms["last"]
        crossed = ms["first"] is not None and pose is not None and ms["first"][0] * pose[0] < 0
        th = inview and (abs(pose[0]) < P["theta_tol"] or crossed)
        return Command(0.0, 0.0), {}, inview, th, pose, True
    return Command(P["v_app"], w), ms, False, False, ms["last"], False


def a55_check_distance(tags, pose, odom, ms, P):
    """compare distance with threshold; turn to face the pocket centre (blocking)"""
    ms = dict(ms or {})
    if ms.get("ph") is None:
        p = tag_to_pose(tags[0]) if tags else pose
        if p is None:
            return Command(), {}, False, True
        bearing = math.atan2(tags[0].x, tags[0].z) if tags else 0.0
        get = p[1] <= P["dock_thr"]
        if not get and abs(bearing) < math.radians(4):
            return Command(), {}, False, True
        ms.update(ph="face", get=get, tgt=wrap(odom.yaw - bearing))
    cmd, done = _rotate_to(odom, ms["tgt"])
    if done:
        return Command(), {}, ms["get"], True
    return cmd, ms, False, False


def a56_stop():
    return Command(), True
