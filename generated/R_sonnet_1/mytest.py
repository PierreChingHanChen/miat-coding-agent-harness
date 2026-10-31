import math, random
from sim.api import *
from controller import DockingController
def run(dist, ang, seed):
    random.seed(seed)
    # pallet frame: tag origin, +y out. AGV base
    bx = dist*math.sin(ang); by = dist*math.cos(ang)
    h = math.atan2(-by, -bx)
    # odom = world offset by start
    c = DockingController(); c.reset('guidance')
    t=0; v=w=0
    for k in range(1500):
        cx=bx+CAM_X*math.cos(h); cy=by+CAM_X*math.sin(h)
        dx,dy=-cx,-cy
        z=dx*math.cos(h)+dy*math.sin(h); x=dx*math.sin(h)-dy*math.cos(h)
        yaw=h-(-math.pi/2)
        tags=[]
        if z>0.5 and abs(math.atan2(x,z))<0.55 and random.random()<0.6:
            tags=[TagDetection(7,x+random.gauss(0,.01),z+random.gauss(0,.02),yaw+random.gauss(0,.03),t)]
        # odom frame = start frame
        s0=math.atan2(-by,-bx)
        ox=(bx-bx)*0; 
        # odom: relative to start pose
        rx,ry=bx0=bx,by
        o=Odom(0,0,0,v,w)
        # compute odom pose as transform of true pose
        h0=math.atan2(-(dist*math.cos(ang)),-(dist*math.sin(ang)))
        sx,sy=dist*math.sin(ang),dist*math.cos(ang)
        ddx,ddy=bx-sx,by-sy
        o=Odom(ddx*math.cos(h0)+ddy*math.sin(h0),-ddx*math.sin(h0)+ddy*math.cos(h0),h-h0,v,w)
        cmd=c.step(Observation(t,o,tags,[],None))
        if cmd.status!='RUNNING':
            cx=bx+CAM_X*math.cos(h); cy=by+CAM_X*math.sin(h)
            lat=cx; dn=cy; e=math.degrees(wrap(h+math.pi/2))
            return cmd.status,round(t,1),round(lat,3),round(dn,3),round(e,2)
        cl=clamp_cmd(cmd); v,w=cl.v,cl.w
        bx+=v*math.cos(h)*DT; by+=v*math.sin(h)*DT; h+=w*DT; t+=DT
    return 'timeout'
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
for d in (2.2,2.5,2.8):
    for a in (-45,-20,0,10,30,45):
        print(d,a,run(d,math.radians(a),1))
