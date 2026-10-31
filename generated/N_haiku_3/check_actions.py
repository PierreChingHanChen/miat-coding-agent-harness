"""Self-check: run the synthesized controller with actions.py on a simple kinematic model.

World (pallet) frame: tag at origin, face on y=0, AGV on +y side.
Base pose (bx, by, yawb): forward = (cos yawb, sin yawb). Camera = base + CAM_X * forward.
Camera heading error psi = yawb + pi/2 (0 when facing the face squarely).
"""
import json
import math
import random
import sys
import types

sys.path.insert(0, '/workspace/N_haiku_3')
import actions as A
import controller_synthesized as CS
from sim.api import (CAM_X, DOCK_DIST, FX, IMG_W, TAG_ID, DT, Observation, Odom, Scan,
                     TagDetection, PalletDetection, clamp_cmd)

MODEL = json.load(open('/workspace/N_haiku_3/model/miat2_grafcet.json'))
P = MODEL['controls']
HFOV_HALF = math.radians(34.5)


TRUTH = []


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def run_trial(stage, bx, by, yawb, rng, clutter=False, max_t=160.0, zero_frac=0.0):
    S = CS.State()
    global LAST
    LAST = S
    C = types.SimpleNamespace()
    d = MODEL['data']
    for k, v in d.items():
        setattr(C, k, json.loads(json.dumps(v)))
    C.P = P
    C.start_approach = stage == 'approach'
    C.start_guidance = stage == 'guidance'
    C.blacklist = []
    C.hist = []
    C.cmd = None
    C.status = 'RUNNING'
    C.n_loc = 0
    C.n_bo = 0
    C.init_done = False
    C.est = None
    C.mstate = {}
    # sim state
    odx, ody, odyaw = 0.0, 0.0, 0.0      # odometry (drifting)
    t = 0.0
    S.trace = []
    CS.reset_G0(S, t)
    if stage == 'guidance':
        pass
    rng_local = rng
    collided = False
    # clutter: false pallet placed at a fixed spot in world frame
    decoy = (-1.8, 1.2) if clutter else None
    while t < max_t:
        cam_x = bx + CAM_X * math.cos(yawb)
        cam_y = by + CAM_X * math.sin(yawb)
        psi = wrap(yawb + math.pi / 2)
        TRUTH.append((round(t,1), round(cam_x,3), round(cam_y,3), round(psi,3)))
        f = (math.cos(yawb), math.sin(yawb))
        r = (math.sin(yawb), -math.cos(yawb))
        # tag detection
        tags, pallets = [], []
        dist_cam = math.hypot(cam_x, cam_y)
        if cam_y > 0:
            xc = -(cam_x * r[0] + cam_y * r[1]) * -1.0
            xc = (0 - cam_x) * r[0] + (0 - cam_y) * r[1]
            zc = (0 - cam_x) * f[0] + (0 - cam_y) * f[1]
            bearing = math.atan2(xc, zc)
            view = math.atan2(abs(cam_x), cam_y)
            if 0.73 <= zc <= 2.8 and abs(bearing) <= HFOV_HALF and view <= math.radians(60) \
                    and rng_local.random() < (0.9 if zc < 2.5 else 0.6):
                tags.append(TagDetection(id=TAG_ID, x=xc, z=zc, yaw=-psi, stamp=t))
            if abs(bearing) <= HFOV_HALF and zc < 4.5:
                u = IMG_W / 2 + FX * math.tan(bearing)
                wpx = 1.2 * FX * max(0.1, math.cos(view)) / zc
                samples = []
                for _ in range(25):
                    s = zc + rng_local.uniform(-0.02, 0.02)
                    if rng_local.random() < zero_frac:
                        s = 0.0
                    samples.append(s)
                pallets.append(PalletDetection(u=u, width_px=wpx, score=rng_local.uniform(0.7, 0.95), depth_samples=samples, stamp=t))
        if decoy is not None and cam_y > 0:
            dx, dy = decoy[0] - cam_x, decoy[1] - cam_y
            zc = dx * f[0] + dy * f[1]
            xc = dx * r[0] + dy * r[1]
            bearing = math.atan2(xc, zc)
            if zc > 0.5 and zc < 4.5 and abs(bearing) <= HFOV_HALF:
                u = IMG_W / 2 + FX * math.tan(bearing)
                pallets.append(PalletDetection(u=u, width_px=1.0 * FX / zc, score=0.7,
                                               depth_samples=[zc] * 25, stamp=t))
        scan = Scan(angle_min=-math.pi, angle_increment=math.radians(2), ranges=[10.0] * 180)
        obs = Observation(t=t, odom=Odom(odx, ody, odyaw, 0.0, 0.0), tags=tags, pallets=pallets, scan=scan)
        C.t = t
        C.odom = obs.odom
        C.tags = tags
        C.pallets = pallets
        C.scan = scan
        C.sensors_ok = C.sensors_ok
        CS.tick(S, C, A)
        cmd = clamp_cmd(C.cmd) if C.cmd is not None else None
        if C.status != 'RUNNING':
            ok = (abs(cam_x) <= 0.06 and abs(cam_y - DOCK_DIST) <= 0.12 and abs(psi) <= math.radians(6))
            return {'status': C.status, 'ok': ok, 't': t, 'lat': cam_x, 'dist': cam_y,
                    'head_deg': math.degrees(psi), 'collided': collided}
        v, w = cmd.v, cmd.w
        # collision check against pallet body x in [-0.6,0.6], y in [-1,0] with 0.3 m radius
        px = min(max(bx, -0.6), 0.6)
        py = min(max(by, -1.0), 0.0)
        if math.hypot(bx - px, by - py) < 0.3:
            collided = True
        # integrate truth and odometry (with mild drift)
        yawb += w * DT
        bx += v * math.cos(yawb) * DT
        by += v * math.sin(yawb) * DT
        odyaw += (w * DT) * 1.003
        odx += v * math.cos(odyaw) * DT
        ody += v * math.sin(odyaw) * DT
        t += DT
    return {'status': 'HANG', 'ok': False, 't': t, 'collided': collided}


def main(n=30, seed=1):
    rng = random.Random(seed)
    results = {'guidance': [], 'approach': []}
    for i in range(n):
        # guidance start
        dist = rng.uniform(2.2, 2.8)
        th = math.radians(rng.uniform(-45, 45))
        bxg, byg = dist * math.sin(th), dist * math.cos(th)
        yawg = math.atan2(-byg, -bxg) + rng.uniform(-0.05, 0.05)
        clutter = (i % 3 == 2)
        rg = run_trial('guidance', bxg, byg, yawg, random.Random(seed * 1000 + i), clutter=clutter)
        results['guidance'].append(rg)
        # approach start
        dist2 = rng.uniform(3.2, 4.0)
        th2 = math.radians(rng.uniform(-15, 15))
        bxa, bya = (dist2 - CAM_X) * math.sin(th2), (dist2 - CAM_X) * math.cos(th2)
        yawa = rng.uniform(-math.pi, math.pi)
        ra = run_trial('approach', bxa, bya, yawa, random.Random(seed * 2000 + i), clutter=clutter)
        results['approach'].append(ra)
    for k, rs in results.items():
        nok = sum(1 for r in rs if r['ok'] and r['status'] == 'DOCKED')
        false_dock = sum(1 for r in rs if r['status'] == 'DOCKED' and not r['ok'])
        hangs = sum(1 for r in rs if r['status'] == 'HANG')
        coll = sum(1 for r in rs if r['collided'])
        print(f"{k}: docked_ok={nok}/{len(rs)} false_dock={false_dock} hang={hangs} collided={coll}")
    bad = [r for rs in results.values() for r in rs if not r['ok']]
    for r in bad[:8]:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 20)
