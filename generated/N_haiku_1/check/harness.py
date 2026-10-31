import sys, json, math, random
ROOT = '/workspace/N_haiku_1'
sys.path.insert(0, ROOT)
import actions as A
import controller_synthesized as K
from sim.api import Odom, Observation, TagDetection, PalletDetection, Scan, FX, CAM_X, TAG_ID, DT, DOCK_DIST

M = json.load(open(ROOT + '/model/miat2_grafcet.json'))

TRUTH = []

class Ctx:
    pass

def make_ctx(stage):
    C = Ctx()
    C.P = dict(M['controls'])
    for k, v in M['data'].items():
        setattr(C, k, json.loads(json.dumps(v)))
    C.cmd = None; C.status = 'RUNNING'
    C.start_approach = (stage == 'approach')
    C.start_guidance = (stage == 'guidance')
    C.t = 0.0
    C.odom = Odom(0, 0, 0, 0, 0); C.tags = []; C.pallets = []; C.scan = Scan(-math.pi, math.radians(2), [5.0]*360)
    return C

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

def run(stage, h0, base0, seed=0, pallet_fp=(0.0, 0.0), max_t=160.0, noise=0.0):
    rnd = random.Random(seed)
    # true base pose in pallet frame
    bx, by, h = base0[0], base0[1], h0
    odom0 = (bx, by, h)                    # odom frame origin = start pose
    S = K.State(); C = make_ctx(stage); K.reset_all(S, 0.0)
    t = 0.0; minc = 9
    while t < max_t:
        # sensors from true pose
        cx, cy = bx + CAM_X*math.cos(h), by + CAM_X*math.sin(h)
        d = (math.cos(h), math.sin(h)); r = (math.sin(h), -math.cos(h))
        z = -(cx*d[0] + cy*d[1]); x = -(cx*r[0] + cy*r[1])
        # T-C = (-cx,-cy) = z*d + x*r
        z = (-cx)*d[0] + (-cy)*d[1]; x = (-cx)*r[0] + (-cy)*r[1]
        ang = math.atan2(x, z)
        nrm = (0.0, 1.0)
        yaw = math.atan2(nrm[0]*r[0]+nrm[1]*r[1], -(nrm[0]*d[0]+nrm[1]*d[1]))
        dist = math.hypot(cx, cy)
        vis = (0.73 <= z <= 2.77 and abs(ang) <= math.radians(34.5)
               and cy > 0 and math.degrees(math.acos(max(-1,min(1,cy/dist)))) <= 60)
        tags = []
        if vis and rnd.random() < 0.95:
            tags = [TagDetection(TAG_ID, x + rnd.gauss(0, noise), z, yaw, t)]
        pallets = []
        if abs(ang) <= math.radians(34.5) and cy > 0 and dist < 4.0:
            u = 320 + FX*math.tan(ang)
            width = 1.2*max(0.0, cy/dist)*FX/dist
            depth = dist
            samples = [depth if rnd.random() > 0.3 else 0.0 for _ in range(25)]
            pallets = [PalletDetection(u, width, 0.85, samples, t)]
        import os
        if os.environ.get('DECOY'):
            dpx, dpy = float(os.environ['DECOY'].split(',')[0]), float(os.environ['DECOY'].split(',')[1])
            vx, vy = dpx-cx, dpy-cy
            zz = vx*d[0]+vy*d[1]; xx = vx*r[0]+vy*r[1]; dd = math.hypot(vx, vy)
            aa = math.atan2(xx, zz)
            if zz > 0 and abs(aa) <= math.radians(34.5) and dd < 4.0:
                samples = [dd if rnd.random() > 0.3 else 0.0 for _ in range(25)]
                pallets.append(PalletDetection(320 + FX*math.tan(aa), 1.2*FX/dd*0.7, 0.7, samples, t))
        ox, oy, oh = bx - odom0[0], by - odom0[1], h - odom0[2]
        c0, s0 = math.cos(-odom0[2]), math.sin(-odom0[2])
        # odom = pose expressed in start frame
        px, py = bx - odom0[0], by - odom0[1]
        o_x = c0*px - s0*py; o_y = s0*px + c0*py
        C.odom = Odom(o_x, o_y, wrap(h - odom0[2]), 0, 0)
        C.tags = tags; C.pallets = pallets; C.t = t
        TRUTH.append((round(t,2), round(bx,3), round(by,3), round(math.degrees(h),1)))
        K.tick(S, C, A)
        st = C.status
        if st != 'RUNNING':
            break
        v, w = C.cmd.v, C.cmd.w
        v = max(-0.3, min(0.3, v)); w = max(-0.6, min(0.6, w))
        bx += v*DT*math.cos(h); by += v*DT*math.sin(h); h = wrap(h + w*DT)
        # collision with pallet body (x in [-0.6,0.6], y in [-1,0]) with footprint 0.3
        if -0.6-0.3 < bx < 0.6+0.3 and -1.0-0.3 < by < 0.3:
            minc = min(minc, by)
        t += DT
    cx, cy = bx + CAM_X*math.cos(h), by + CAM_X*math.sin(h)
    ok = None
    if st == 'DOCKED':
        ok = (abs(cx) <= 0.06 and abs(cy-DOCK_DIST) <= 0.12 and abs(wrap(h + math.pi/2)) <= math.radians(6))
    return dict(status=st, t=round(t,1), cam=(round(cx,3), round(cy,3)), head_deg=round(math.degrees(wrap(h+math.pi/2)),2), false_dock=(st=='DOCKED' and not ok), trace=S.trace[-4:], collide_min=minc)

if __name__ == '__main__':
    stage = sys.argv[1]
    for seed in range(int(sys.argv[2]) if len(sys.argv) > 2 else 3):
        rnd = random.Random(100+seed)
        if stage == 'guidance':
            dist = rnd.uniform(2.2, 2.8); ang = math.radians(rnd.uniform(-45, 45))
            # tag at origin; base at (dist*sin(ang), dist*cos(ang)) minus CAM offset approx
            bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
            h = wrap(math.atan2(-by, -bx))
        else:
            dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
            bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
            h = rnd.uniform(-math.pi, math.pi)
        print(seed, run(stage, h, (bx, by), seed=seed, noise=0.003))
