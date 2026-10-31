import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
for seed in range(30):
    rnd = random.Random(500+seed)
    dist = rnd.uniform(2.2, 2.8); ang = math.radians(rnd.uniform(-45, 45))
    bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
    h = wrap(math.atan2(-by, -bx))
    r = run('guidance', h, (bx, by), seed=seed, noise=0.003)
    if r['status'] != 'DOCKED':
        print(seed, round(math.degrees(ang)), round(dist,2), r['trace'][-3:], r['cam'], r['head_deg'])
