"""Quick kinematic mock (self-check only): ideal geometry, intermittent tag, drifting odometry."""
import math, random, sys
from sim.api import Observation, Odom, Scan, TagDetection, PalletDetection, Command, CAM_X, FX, IMG_W, TAG_ID, DT
import controller

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

def segs():
    return [((-0.6, 0.0), (0.6, 0.0)), ((-0.6, 0.0), (-0.6, -1.0)),
            ((0.6, 0.0), (0.6, -1.0)), ((-0.6, -1.0), (0.6, -1.0))]

def raycast(px, py, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best = 10.0
    for (x1, y1), (x2, y2) in segs():
        ex, ey = x2 - x1, y2 - y1
        den = dx * (-ey) - dy * (-ex)
        if abs(den) < 1e-12: continue
        qx, qy = x1 - px, y1 - py
        t = (qx * (-ey) - qy * (-ex)) / den
        s = (dx * qy - dy * qx) / den
        if t > 0 and 0 <= s <= 1 and t < best: best = t
    return best

def run(stage, seed, rng_noise=0.9):
    rng = random.Random(seed)
    if stage == 'guidance':
        r = rng.uniform(2.2, 2.8); th = math.radians(rng.uniform(-45, 45))
        px, py = r * math.sin(th), r * math.cos(th)
        yaw = math.atan2(-py, -px)
    else:
        r = rng.uniform(3.2, 4.0); th = math.radians(rng.uniform(-15, 15))
        px, py = r * math.sin(th), r * math.cos(th)
        yaw = rng.uniform(-math.pi, math.pi)
    ox = oy = oyaw = 0.0
    ctrl = controller.DockingController(); ctrl.reset(stage)
    t = 0.0; status = None; collided = False; cmd = Command()
    for k in range(1600):
        cx, cy = px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)
        dist = math.hypot(cx, cy)
        th_view = math.degrees(math.atan2(abs(cx), cy)) if cy > 0 else 90.0
        rel = wrap(math.atan2(-cy, -cx) - yaw)
        tags = []
        lim = 2.77 - 0.75 * min(th_view, 45) / 45.0
        if 0.73 <= dist <= lim and abs(math.degrees(rel)) <= 34.5 and th_view <= 60 and rng.random() < rng_noise:
            x = -cx * math.sin(yaw) + cy * math.cos(yaw)
            z = -cx * math.cos(yaw) - cy * math.sin(yaw)
            tags.append(TagDetection(TAG_ID, x + rng.gauss(0, 0.003), z, wrap(yaw + math.pi / 2), t))
        pallets = []
        if cy > 0.5 and abs(math.degrees(rel)) <= 34.5 and dist <= 4.0 and rng.random() < 0.95:
            ang = math.atan2(-cx, cy)  # bearing of face centre, left positive
            u = IMG_W / 2 - FX * math.tan(-rel * 0 + wrap(math.atan2(-cx, cy)) * 0 + (-(math.atan2(-cx, cy))) * -1 * -1) if False else IMG_W / 2 - FX * math.tan(math.atan2(-cx, cy))
            width = 1.2 * FX / math.hypot(cx, cy)
            depth = [math.hypot(cx, cy) + rng.gauss(0, 0.01) if rng.random() > 0.4 else 0.0 for _ in range(25)]
            pallets.append(PalletDetection(u, width, 0.8, depth, t))
        ranges = [raycast(px, py, wrap(-math.pi + i * 2 * math.pi / 180) + yaw) for i in range(180)]
        obs = Observation(t, Odom(ox, oy, oyaw, cmd.v, cmd.w), tags, pallets,
                          Scan(-math.pi, 2 * math.pi / 180, ranges, 10.0))
        cmd = ctrl.step(obs)
        if cmd.status != 'RUNNING':
            status = cmd.status
            break
        v = max(-0.3, min(0.3, cmd.v)); w = max(-0.6, min(0.6, cmd.w))
        # true motion
        yaw_m = yaw + 0.5 * w * DT
        px += v * math.cos(yaw_m) * DT; py += v * math.sin(yaw_m) * DT; yaw = wrap(yaw + w * DT)
        # odometry with drift
        ov, ow = v * 1.01, w * 0.995
        om = oyaw + 0.5 * ow * DT
        ox += ov * math.cos(om) * DT; oy += ov * math.sin(om) * DT; oyaw = wrap(oyaw + ow * DT)
        if (px - 0.0) ** 2 + 0 >= 0 and -0.6 - 0.3 < px < 0.6 + 0.3 and -1.0 - 0.3 < py < 0.3:
            # coarse disc-vs-body check
            qx = min(max(px, -0.6), 0.6); qy = min(max(py, -1.0), 0.0)
            if math.hypot(px - qx, py - qy) < 0.30: collided = True
        t += DT
    cx, cy = px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)
    truth = None
    if status == 'DOCKED':
        truth = (abs(cx) <= 0.06 and abs(cy - 0.85) <= 0.12 and abs(wrap(yaw + math.pi / 2)) <= math.radians(6))
    return status, truth, collided, round(t, 1), round(r, 2)

if __name__ == '__main__':
    stage = sys.argv[1]; n = int(sys.argv[2])
    res = [run(stage, s) for s in range(n)]
    from collections import Counter
    print(stage, Counter((s, tr, c) for s, tr, c, _, _ in res))
    bad = [(i, x) for i, x in enumerate(res) if x[0] != 'DOCKED' or x[1] is not True or x[2]]
    print('non-ok seeds', bad[:10])
    print('mean time', round(sum(x[3] for x in res) / len(res), 1), 'max time', max(x[3] for x in res))
