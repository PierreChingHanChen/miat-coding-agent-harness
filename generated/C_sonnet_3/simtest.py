import math, random, sys
from sim.api import *
import controller, importlib
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
def run(stage, seed, ang_deg, dist, clutter=False):
    rnd=random.Random(seed)
    # pallet frame; AGV base pose
    a=math.radians(ang_deg)
    cam=(dist*math.sin(a), dist*math.cos(a))   # base start
    px,py=cam
    yaw=math.atan2(-py,-px)  # heading to tag
    if stage=='approach':
        dist=rnd.uniform(3.2,4.0); a=math.radians(rnd.uniform(-15,15))
        px,py=dist*math.sin(a),dist*math.cos(a); yaw=rnd.uniform(-math.pi,math.pi)
    sx,sy,syaw=px,py,yaw
    fake=(rnd.uniform(-2,2),rnd.uniform(1.5,3.5)) if clutter else None
    c=controller.DockingController(); c.reset(stage)
    v=w=0.0; t=0.0; ox=oy=oyaw=0.0
    for k in range(1600):
        # odom
        cs,sn=math.cos(syaw),math.sin(syaw)
        dx,dy=px-sx,py-sy
        ox=(dx*cs+dy*sn); oy=(-dx*sn+dy*cs); oyaw=wrap(yaw-syaw)
        odom=Odom(ox,oy,oyaw,v,w)
        camx,camy=px+CAM_X*math.cos(yaw),py+CAM_X*math.sin(yaw)
        tags=[]
        dd=math.hypot(camx,camy)
        # bearing of tag from camera (+right)
        brg=wrap(yaw-math.atan2(-camy,-camx))
        view=abs(math.degrees(math.atan2(camx,camy)))
        env={0:2.77,15:2.52,30:2.42,45:2.02}
        lim=2.77-(2.77-2.02)*min(view,45)/45
        if dd>=0.73 and abs(math.degrees(brg))<34.5 and dd<lim and rnd.random()<0.85 and view<60:
            ty=yaw+math.pi/2
            f=(math.cos(yaw),math.sin(yaw)); r=(math.sin(yaw),-math.cos(yaw))
            z=-(camx*f[0]+camy*f[1]); x=-(camx*r[0]+camy*r[1])
            tags.append(TagDetection(7,x+rnd.gauss(0,.005),z+rnd.gauss(0,.01),wrap(ty+rnd.gauss(0,.01)),t))
        pallets=[]
        def det(wx,wy,width,score):
            ddx,ddy=wx-camx,wy-camy
            d=math.hypot(ddx,ddy); b=wrap(yaw-math.atan2(ddy,ddx))
            if abs(b)<math.radians(34) and d<6 and rnd.random()<0.9:
                u=320+FX*math.tan(b)
                ds=[d if rnd.random()>0.4 else 0.0 for _ in range(25)]
                ds=[x+rnd.gauss(0,.02) if x>0 else 0 for x in ds]
                pallets.append(PalletDetection(u,width*FX/d,score,ds,t))
        vw=math.cos(math.radians(view)) if False else 1
        det(0,0,1.2*max(0.5,abs(math.cos(math.radians(view)))),rnd.uniform(.6,.99))
        if fake: det(fake[0],fake[1],1.2,rnd.uniform(.45,.88))
        # scan: pallet body ahead
        ranges=[float('inf')]*180
        scan=Scan(-math.pi,math.radians(2),ranges)
        obs=Observation(t,odom,tags,pallets,scan)
        cmd=c.step(obs)
        if cmd.status!='RUNNING':
            camd=abs(camy); lat=camx
            hd=abs(math.degrees(wrap(yaw+math.pi/2)))
            ok=cmd.status=='DOCKED' and abs(camx)<=.06 and abs(camy-.85)<=.12 and hd<=6
            return cmd.status,ok,t,(round(camx,3),round(camy,3),round(hd,1))
        cmd=clamp_cmd(cmd)
        v=cmd.v+rnd.gauss(0,0.003)*(cmd.v!=0); w=cmd.w
        yaw=wrap(yaw+w*DT); px+=v*DT*math.cos(yaw); py+=v*DT*math.sin(yaw)
        if abs(px)<0.6+0.3 and py<0.3 : return 'COLLISION',False,t,(px,py)
        t+=DT
    return 'HANG',False,t,None
if __name__=='__main__':
    res=[]
    for stage in ('guidance','approach'):
        for ang in ([-45,-30,-15,0,15,30,45] if stage=='guidance' else [0]):
            for dist in ((2.2,2.5,2.8) if stage=='guidance' else (0,)):
                for seed in range(4):
                    r=run(stage,seed,ang,dist)
                    res.append(r[1]); 
                    if not r[1]: print(stage,ang,dist,seed,r)
    print(sum(res),len(res))
