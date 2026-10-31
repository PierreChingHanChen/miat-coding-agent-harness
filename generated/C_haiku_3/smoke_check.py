"""Kinematic smoke check for controller.py (not the platform simulator).
Usage: python3 smoke_check.py
Checks: tag-pose convention round trip, then closed-loop runs in guidance/approach starts.
"""
import math, os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sim.api import Command, Observation, Odom, Scan, TagDetection, PalletDetection, CAM_X, FX, IMG_W, TAG_ID, DT
import actions as A
from controller import DockingController

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

def cam_of(px, py, yaw): return px + CAM_X * math.cos(yaw), py + CAM_X * math.sin(yaw)

# ---- convention round trip: pose -> tag measurement -> pose
for _ in range(200):
    px, py, yaw = random.uniform(-1, 1), random.uniform(0.9, 4), random.uniform(-math.pi, math.pi)
    cx, cy = cam_of(px, py, yaw)
    vx, vy = -cx, -cy
    z = vx * math.cos(yaw) + vy * math.sin(yaw)
    x = vx * math.sin(yaw) - vy * math.cos(yaw)
    tag = TagDetection(TAG_ID, x, z, wrap(yaw + math.pi / 2), 0.0)
    qx, qy, qyaw = A.a42_tag_to_pose(tag)
    assert abs(qx - px) < 1e-9 and abs(qy - py) < 1e-9 and abs(wrap(qyaw - yaw)) < 1e-9, (px, py, yaw, qx, qy, qyaw)
print("tag pose convention round trip: OK")


class World:
    def __init__(self, base, heading, cluttered, rng):
        self.x, self.y, self.h = base[0], base[1], heading
        self.x0, self.y0, self.h0 = base[0], base[1], heading
        self.rng = rng
        self.t = 0.0
        self.cluttered = cluttered
        self.decoy = (-1.6, 2.6) if cluttered else None
        self.contact = False
        self.status = None
        self.dodx = 0.0

    def odom(self):
        # odometry: pose expressed in the start frame, with small translational drift
        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(-self.h0), math.sin(-self.h0)
        ox, oy = (c * dx - s * dy) * 1.01, (s * dx + c * dy) * 1.01
        return Odom(ox, oy, wrap(self.h - self.h0), 0.0, 0.0)

    def observe(self):
        rng = self.rng
        cx, cy = cam_of(self.x, self.y, self.h)
        tags = []
        vx, vy = -cx, -cy
        z = vx * math.cos(self.h) + vy * math.sin(self.h)
        x = vx * math.sin(self.h) - vy * math.cos(self.h)
        ang = math.atan2(x, z)
        p = 0.92 if z <= 2.5 else (0.5 if z <= 3.0 else 0.0)
        if z >= 0.73 and abs(ang) <= math.radians(34.5) and rng.random() < p:
            tags.append(TagDetection(TAG_ID, x + rng.gauss(0, 0.004), z + rng.gauss(0, 0.01),
                                     wrap(self.h + math.pi / 2 + rng.gauss(0, 0.005)), self.t))
        pallets = []
        objs = [((0.0, 0.0), 1.2, 0.9)]
        if self.decoy:
            objs.append((self.decoy, 1.0, 0.7))
        for (ox, oy), width, score in objs:
            dx, dy = ox - cx, oy - cy
            d = math.hypot(dx, dy)
            b = -wrap(math.atan2(dy, dx) - self.h)
            if d > 6.0 or abs(b) > math.radians(34.5):
                continue
            u = IMG_W / 2 + FX * math.tan(b)
            samples = []
            for _ in range(25):
                r = rng.random()
                if r < 0.2:
                    samples.append(0.0)
                elif r < 0.25:
                    samples.append(d + 1.5)
                else:
                    samples.append(d + rng.gauss(0, 0.01))
            pallets.append(PalletDetection(u, width * FX / d, score, samples, self.t))
        ranges = []
        for i in range(180):
            th = -math.pi + i * math.radians(2)
            ranges.append(self._ray(self.x, self.y, self.h + th))
        return Observation(self.t, self.odom(), tags, pallets, Scan(-math.pi, math.radians(2), ranges, 10.0))

    def _boxes(self):
        bx = [(-0.6, -1.0, 0.6, 0.0)]
        if self.decoy:
            dx, dy = self.decoy
            bx.append((dx - 0.5, dy - 0.5, dx + 0.5, dy + 0.5))
        return bx

    def _ray(self, x, y, th):
        best = 10.0
        dx, dy = math.cos(th), math.sin(th)
        for (x0, y0, x1, y1) in self._boxes():
            tmin, tmax = 0.0, 10.0
            ok = True
            for o, d, lo, hi in ((x, dx, x0, x1), (y, dy, y0, y1)):
                if abs(d) < 1e-12:
                    if o < lo or o > hi:
                        ok = False
                        break
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    tmin = max(tmin, min(t1, t2))
                    tmax = min(tmax, max(t1, t2))
            if ok and tmin <= tmax and tmin > 0.0:
                best = min(best, tmin)
        return best

    def clearance(self):
        # distance of the base disc (radius 0.3) to the pallet body / decoy
        m = 10.0
        for (x0, y0, x1, y1) in self._boxes():
            qx = min(max(self.x, x0), x1)
            qy = min(max(self.y, y0), y1)
            m = min(m, math.hypot(self.x - qx, self.y - qy))
        return m

    def step(self, cmd):
        self.x += cmd.v * math.cos(self.h) * DT
        self.y += cmd.v * math.sin(self.h) * DT
        self.h = wrap(self.h + cmd.w * DT)
        self.t += DT
        if self.clearance() < 0.30:
            self.contact = True

    def truth(self):
        cx, cy = cam_of(self.x, self.y, self.h)
        return abs(cx), abs(cy - 0.0) - 0.0, cy, wrap(self.h + math.pi / 2)


def run(stage, base, heading, cluttered, seed, max_t):
    rng = random.Random(seed)
    w = World(base, heading, cluttered, rng)
    c = DockingController()
    c.reset(stage)
    ticks = 0
    st = 'RUNNING'
    while w.t < max_t:
        obs = w.observe()
        cmd = c.step(obs)
        ticks += 1
        st = cmd.status
        if st != 'RUNNING':
            break
        w.step(cmd)
    cx, cy = cam_of(w.x, w.y, w.h)
    lat = cx
    dist = cy  # camera distance to the face (face at y=0, AGV on y>0)
    head = wrap(w.h + math.pi / 2)
    ok_dock = st == 'DOCKED' and abs(lat) <= 0.06 and abs(dist - 0.85) <= 0.12 and abs(head) <= math.radians(6) and abs(cmd.v) < 1e-9
    return st, ticks, ok_dock, w.contact, (round(lat, 3), round(cy, 3), round(math.degrees(head), 1)), round(w.t, 1)


if __name__ == '__main__':
    results = []
    rng0 = random.Random(7)
    cases = []
    for seed in range(12):
        ang = math.radians(rng0.uniform(-45, 45))
        dist = rng0.uniform(2.2, 2.8)
        bx, by = -dist * math.sin(ang) * 0 + 0.0, 0.0
        cam_dir = (math.sin(ang), math.cos(ang))
        base = (dist * math.sin(ang), dist * math.cos(ang) - 0.0)
        head = math.atan2(-base[1], -base[0])
        cases.append(('guidance', base, head, False, seed, 120))
    for seed in range(12):
        dist = rng0.uniform(3.2, 4.0)
        off = math.radians(rng0.uniform(-15, 15))
        base = (dist * math.sin(off), dist * math.cos(off))
        head = rng0.uniform(-math.pi, math.pi)
        cases.append(('approach', base, head, False, 100 + seed, 150))
    for seed in range(8):
        dist = rng0.uniform(3.2, 4.0)
        off = math.radians(rng0.uniform(-15, 15))
        base = (dist * math.sin(off), dist * math.cos(off))
        head = rng0.uniform(-math.pi, math.pi)
        cases.append(('approach', base, head, True, 200 + seed, 150))
    n_ok = 0
    for case in cases:
        stage, base, head, clut, seed, mt = case
        st, ticks, ok, contact, pose, tend = run(stage, base, head, clut, seed, mt)
        n_ok += ok
        print(f"{stage:9s} clutter={int(clut)} seed={seed:3d} status={st:8s} ok={ok!s:5s} contact={contact!s:5s} "
              f"lat/dist/head(deg)={pose} t={tend}s")
    print(f"smoke runs: {n_ok}/{len(cases)} docked within tolerance")
