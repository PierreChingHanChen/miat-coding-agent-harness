import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
import harness
rnd = random.Random(100)
dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
h = rnd.uniform(-math.pi, math.pi)
orig = A.a321_candidates
cnt = {'n':0}
def wrapped(pallets, odom, bl, P):
    c = orig(pallets, odom, bl, P)
    if pallets and cnt['n'] < 8:
        cnt['n'] += 1
        print('pallets', len(pallets), 'cands', c, 'odom', odom, 'u', pallets[0].u, 'w', pallets[0].width_px)
    return c
A.a321_candidates = wrapped
print(run('approach', h, (bx,by), seed=0, noise=0.003))
