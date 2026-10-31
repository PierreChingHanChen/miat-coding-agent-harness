import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
orig_tick = K.tick
seed = int(sys.argv[1]); lo, hi = float(sys.argv[2]), float(sys.argv[3]); stage = sys.argv[4]
log = {'last': -1}
def tick(S, C, A):
    orig_tick(S, C, A)
    if lo <= C.t <= hi and C.t - log['last'] >= 0.5:
        log['last'] = C.t
        st = [k for k,v in S.X.items() if v and k not in ('30',)]
        est = getattr(C, 'est', None)
        tr = C.tags[0] if C.tags else None
        print(round(C.t,1), st, 'cmd', round(C.cmd.v,2), round(C.cmd.w,2), 'odom', round(C.odom.x,2), round(C.odom.y,2), round(math.degrees(C.odom.yaw),1),
              'tag', None if tr is None else (round(tr.x,2), round(tr.z,2)), 'track', C.track and (round(C.track['x'],2), round(C.track['y'],2)),
              'est', None if est is None else (round(est['x'],2), round(est['y'],2), round(math.degrees(est['h']),1), round(est['age'],2), est['n']))
K.tick = tick
rnd = random.Random(int(sys.argv[5]) if len(sys.argv)>5 else 100+seed)
if stage == 'guidance':
    dist = rnd.uniform(2.2, 2.8); ang = math.radians(rnd.uniform(-45, 45))
    bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
    h = wrap(math.atan2(-by, -bx))
else:
    dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
    bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
    h = rnd.uniform(-math.pi, math.pi)
res = run(stage, h, (bx, by), seed=seed, noise=0.003)
print(res)
print(TRUTH[:3], TRUTH[50:53], TRUTH[260:263])
for row in TRUTH:
    if 109.9 <= row[0] <= 111.0: print('TRUTH', row)
