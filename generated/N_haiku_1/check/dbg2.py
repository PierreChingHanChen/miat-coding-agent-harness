import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
import harness as H
from harness import *
orig_tick = K.tick
log = {'last': -1}
def tick(S, C, A):
    orig_tick(S, C, A)
    if int(C.t*10) % 10 == 0 and C.t - log['last'] >= 1.0 and C.t > 60:
        log['last'] = C.t
        st = [k for k,v in S.X.items() if v]
        est = C.est
        print(round(C.t,1), st, 'cmd', round(C.cmd.v,3), round(C.cmd.w,3), 'est', None if est is None else (round(est['x'],2), round(est['y'],2), round(math.degrees(est['h']),1), round(est['age'],2), est['n']))
K.tick = tick
rnd = random.Random(101)
dist = rnd.uniform(2.2, 2.8); ang = math.radians(rnd.uniform(-45, 45))
bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
h = wrap(math.atan2(-by, -bx))
print(run('guidance', h, (bx, by), seed=1, noise=0.003))
