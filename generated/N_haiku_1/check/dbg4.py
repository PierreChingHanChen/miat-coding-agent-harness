import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
rnd = random.Random(101)
dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
h0 = rnd.uniform(-math.pi, math.pi)
c, s = math.cos(-h0), math.sin(-h0)
tag_odom = (c*(0-bx) - s*(0-by), s*(0-bx) + c*(0-by))
print('true tag in odom frame', [round(v,2) for v in tag_odom], 'start base', bx, by, 'h0', h0)
orig = A.a322_confirm
def w(cands, hist, t, P):
    r = orig(cands, hist, t, P)
    if cands: print('t', round(t,2), 'cand', [(round(c_['x'],2), round(c_['y'],2), round(c_['d'],2)) for c_ in cands], 'confirmed', r[1], r[2] and (round(r[2]['x'],2), round(r[2]['y'],2)))
    return r
A.a322_confirm = w
print(run('approach', h0, (bx,by), seed=0, noise=0.003, max_t=3))
