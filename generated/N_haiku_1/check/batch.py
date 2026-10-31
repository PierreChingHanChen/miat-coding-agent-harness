import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1/check")
from harness import *
stage = sys.argv[1]; n = int(sys.argv[2]); noise = float(sys.argv[3])
res = []
for seed in range(n):
    rnd = random.Random(500+seed)
    if stage == 'guidance':
        dist = rnd.uniform(2.2, 2.8); ang = math.radians(rnd.uniform(-45, 45))
        bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
        h = wrap(math.atan2(-by, -bx))
    else:
        dist = rnd.uniform(3.2, 4.0); ang = math.radians(rnd.uniform(-15, 15))
        bx, by = dist*math.sin(ang), dist*math.cos(ang) + CAM_X
        h = rnd.uniform(-math.pi, math.pi)
    r = run(stage, h, (bx, by), seed=seed, noise=noise)
    res.append(r)
from collections import Counter
print(stage, 'noise', noise, Counter(r['status'] for r in res), 'false_dock', sum(r['false_dock'] for r in res), 'max_t', max(r['t'] for r in res))
print([ (round(r['t']), r['status'][0]) for r in res if r['status'] != 'DOCKED'])
