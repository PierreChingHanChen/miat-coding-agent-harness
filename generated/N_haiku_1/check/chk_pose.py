import sys, math, random
sys.path.insert(0, "/workspace/N_haiku_1")
import actions as A
from sim.api import CAM_X
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
rnd = random.Random(1)
worst = 0
for _ in range(2000):
    bx, by, h = rnd.uniform(-1,1), rnd.uniform(0.9,3), rnd.uniform(-math.pi, math.pi)
    cx, cy = bx + CAM_X*math.cos(h), by + CAM_X*math.sin(h)
    d = (math.cos(h), math.sin(h)); r = (math.sin(h), -math.cos(h))
    z = (-cx)*d[0] + (-cy)*d[1]; x = (-cx)*r[0] + (-cy)*r[1]
    yaw = math.atan2(0*r[0]+1*r[1], -(0*d[0]+1*d[1]))
    mx, my, mh = A._meas_pose(z, x, yaw)
    err = math.hypot(mx-bx, my-by) + abs(wrap(mh-h))
    worst = max(worst, err)
print('worst pose error', worst)
