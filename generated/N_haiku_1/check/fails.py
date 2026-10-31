import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
stage='approach'
for seed in range(12):
    rnd = random.Random(500+seed)
    dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
    bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
    h = rnd.uniform(-math.pi, math.pi)
    r = run(stage, h, (bx, by), seed=seed, noise=0.003)
    if r['status'] != 'DOCKED':
        print(seed, round(math.degrees(h)), r['trace'][-4:], r['cam'], r['head_deg'])
