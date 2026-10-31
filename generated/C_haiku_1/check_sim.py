"""Minimal kinematic self-check for controller.py (not the platform simulator).
Usage: python3 check_sim.py [n_trials]"""
import math
import random
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sim.api import Observation, Odom, Scan, TagDetection, PalletDetection, Command, \
    CAM_X, FX, IMG_W, TAG_ID, DT, V_MAX, W_MAX
import controller

PI = math.pi
BOXES = [(-0.6, 0.6, -1.0, 0.0)]          # pallet body (pocket face on y = 0)
FALSE_BOXES = []                          # optional clutter


def wrap(a):
    return (a + PI) % (2 * PI) - PI


def ray_box(ox, oy, dx, dy, box):
    xmin, xmax, ymin, ymax = box
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin > 0 else None


class World:
    def __init__(self, stage, rng, clutter):
        self.rng = rng
        self.boxes = list(BOXES)
        if clutter:
            self.boxes.append((-2.0, -1.2, 1.6, 2.4))   # rack-like obstacle (false pallet)
            self.boxes.append((0.9, 1.4, 2.5, 3.2))     # table-like obstacle
        self.false_pallet = clutter
        while True:   # resample until the start pose is clear of every obstacle
            if stage == 'guidance':
                r = rng.uniform(2.2, 2.8)
                th = math.radians(rng.uniform(-45, 45))
                x, y = r * math.sin(th), r * math.cos(th)
                yaw = math.atan2(-y, -x)
            else:
                r = rng.uniform(3.2, 4.0)
                th = math.radians(rng.uniform(-15, 15))
                x, y = r * math.sin(th), r * math.cos(th)
                yaw = rng.uniform(-PI, PI)
            if all(not (b[0] - 0.6 < x < b[1] + 0.6 and b[2] - 0.6 < y < b[3] + 0.6) for b in self.boxes):
                break
        self.p = [x, y, yaw]
        self.p0 = list(self.p)
        self.t = 0.0
        self.collided = False

    def odom(self):
        x, y, yaw = self.p
        dx, dy = x - self.p0[0], y - self.p0[1]
        c, s = math.cos(-self.p0[2]), math.sin(-self.p0[2])
        return Odom(c * dx - s * dy, s * dx + c * dy, wrap(yaw - self.p0[2]), 0.0, 0.0)

    def cam(self):
        x, y, yaw = self.p
        return x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw), yaw

    def tag_det(self):
        cx, cy, yaw = self.cam()
        h = (math.cos(yaw), math.sin(yaw))
        right = (math.sin(yaw), -math.cos(yaw))
        dx, dy = -cx, -cy
        z = dx * h[0] + dy * h[1]
        x = dx * right[0] + dy * right[1]
        dist = math.hypot(dx, dy)
        view = math.degrees(math.atan2(-(cx), -(cy)))  # not used precisely
        if z < 0.73 or dist > 2.9:
            return None
        bearing = math.degrees(math.atan2(x, z))
        if abs(bearing) > 34.5:
            return None
        # angle from face normal (+y) to camera direction
        ang_n = math.degrees(abs(math.atan2(cx, cy)))
        if ang_n > 60:
            return None
        p_det = 0.9 if dist < 2.5 else max(0.0, 0.9 - (dist - 2.5) * 3)
        if self.rng.random() > p_det:
            return None
        tyaw = wrap(yaw + PI / 2)
        return TagDetection(TAG_ID, x + self.rng.gauss(0, 0.005), z + self.rng.gauss(0, 0.005),
                            tyaw, self.t)

    def pallet_det(self):
        cx, cy, yaw = self.cam()
        h = (math.cos(yaw), math.sin(yaw))
        right = (math.sin(yaw), -math.cos(yaw))
        dx, dy = -cx, -cy
        z = dx * h[0] + dy * h[1]
        x = dx * right[0] + dy * right[1]
        dist = math.hypot(dx, dy)
        if z < 0.5 or dist > 4.5 or math.degrees(abs(math.atan2(x, z))) > 34:
            return []
        if self.rng.random() < 0.15:
            return []
        u = IMG_W / 2 + FX * math.tan(math.atan2(x, z))
        view = abs(math.atan2(cx, cy))
        width = FX * 1.2 * math.cos(view) / dist
        samples = []
        for _ in range(25):
            if self.rng.random() < 0.2:
                samples.append(0.0)
            else:
                samples.append(dist + self.rng.gauss(0, 0.02))
        dets = [PalletDetection(u, width, 0.8 + self.rng.uniform(-0.1, 0.1), samples, self.t)]
        if self.false_pallet:   # rack-like false detection at the first obstacle centre
            fx, fy = -1.6, 2.0
            fdx, fdy = fx - cx, fy - cy
            fz = fdx * h[0] + fdy * h[1]
            fxc = fdx * right[0] + fdy * right[1]
            fd = math.hypot(fdx, fdy)
            if fz > 0.5 and fd < 4.5 and math.degrees(abs(math.atan2(fxc, fz))) < 34:
                fu = IMG_W / 2 + FX * math.tan(math.atan2(fxc, fz))
                dets.append(PalletDetection(fu, FX * 1.0 / fd, 0.6,
                                            [fd + self.rng.gauss(0, 0.02)] * 25, self.t))
        return dets

    def scan(self):
        x, y, yaw = self.p
        ranges = []
        for i in range(180):
            a = -PI + i * math.radians(2)
            d = 10.0
            for b in self.boxes:
                r = ray_box(x, y, math.cos(yaw + a), math.sin(yaw + a), b)
                if r is not None and r < d:
                    d = r
            ranges.append(d)
        return Scan(-PI, math.radians(2), ranges, 10.0)

    def step(self, cmd):
        v = max(-V_MAX, min(V_MAX, cmd.v))
        w = max(-W_MAX, min(W_MAX, cmd.w))
        x, y, yaw = self.p
        yaw2 = yaw + w * DT / 2
        self.p = [x + v * DT * math.cos(yaw2), y + v * DT * math.sin(yaw2), wrap(yaw + w * DT)]
        x, y = self.p[0], self.p[1]
        for b in self.boxes:
            if b[0] - 0.30 < x < b[1] + 0.30 and b[2] - 0.30 < y < b[3] + 0.30:
                self.collided = True
        self.t += DT

    def truth_ok(self):
        cx, cy, yaw = self.cam()
        return (abs(cx) <= 0.06 and abs(cy - 0.85) <= 0.12
                and abs(wrap(yaw + PI / 2)) <= math.radians(6))


def run_trial(stage, seed, clutter=False):
    rng = random.Random(seed)
    w = World(stage, rng, clutter)
    c = controller.DockingController()
    c.reset(stage)
    budget = 120.0 if stage == 'guidance' else 150.0
    status, cmd, ticks = 'RUNNING', Command(), 0
    while status == 'RUNNING' and w.t < budget + 5:
        odom = w.odom()
        tags = [d for d in [w.tag_det()] if d is not None]
        pallets = w.pallet_det()
        obs = Observation(t=round(w.t, 3), odom=odom, tags=tags, pallets=pallets, scan=w.scan())
        cmd = c.step(obs)
        ticks += 1
        status = cmd.status
        if status == 'RUNNING':
            w.step(cmd)
        else:
            break
    if status == 'RUNNING':
        return 'HANG', w, ticks, c
    if status == 'DOCKED':
        ok = w.truth_ok()
        return ('DOCK_OK' if ok else 'FALSE_DOCK'), w, ticks, c
    return 'ABORTED', w, ticks, c


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    for stage, clutter in (('guidance', False), ('approach', False), ('approach', True), ('guidance', True)):
        res = {}
        coll = 0
        for s in range(n):
            out, w, ticks, c = run_trial(stage, 1000 + s, clutter)
            res[out] = res.get(out, 0) + 1
            coll += w.collided
        print(f"{stage:9s} clutter={clutter!s:5s} {res} collisions={coll}")


if __name__ == '__main__':
    main()
