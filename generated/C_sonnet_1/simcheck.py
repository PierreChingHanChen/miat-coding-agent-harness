import math, random, sys
from sim.api import *
from controller import DockingController
import actions as A
wrap=A.wrap
def run(stage, dist, view_deg, seed, zeros=0.0, clutter=False, verbose=False):
    rnd=random.Random(seed)
    # pallet frame; AGV start
    va=math.radians(view_deg)
    cx0=dist*math.sin(va); cy0=dist*math.cos(va)
    if stage=='guidance':
        yaw=math.atan2(-cy0,-cx0)   # heading toward tag
        # base from camera
        x=cx0-CAM_X*math.cos(yaw); y=cy0-CAM_X*math.sin(yaw)
    else:
        x=cx0; y=cy0; yaw=rnd.uniform(-math.pi,math.pi)
    ox=oy=oyaw=0.0
    fake=[(2.0,-0.5+0.0)] if clutter else []
    ctl=DockingController(); ctl.reset(stage)
    t=0.0; v=w=0.0
    lim=120 if stage=='guidance' else 150
    while t<lim+5:
        # observation
        cx=x+CAM_X*math.cos(yaw); cy=y+CAM_X*math.sin(yaw)
        tags=[]
        rx,ry=-cx,-cy; dd=math.hypot(rx,ry)
        f=(math.cos(yaw),math.sin(yaw)); rt=(math.sin(yaw),-math.cos(yaw))
        z=rx*f[0]+ry*f[1]; xr=rx*rt[0]+ry*rt[1]
        view=abs(math.degrees(math.atan2(cx,cy)))
        env=2.77-(2.77-2.02)*min(view,45)/45
        if z>0.73 and abs(math.atan2(xr,z))<math.radians(34.5) and dd<env and rnd.random()<0.85:
            tags.append(TagDetection(7,xr+rnd.gauss(0,0.01),z+rnd.gauss(0,0.01),wrap(yaw+math.pi/2)+rnd.gauss(0,0.01),t))
        pals=[]
        objs=[((0,0),True)]+[(p,False) for p in fake]
        for (px,py),true in objs:
            rx,ry=px-cx,py-cy; d=math.hypot(rx,ry)
            z=rx*f[0]+ry*f[1]; xr=rx*rt[0]+ry*rt[1]
            if z>0.5 and abs(math.atan2(xr,z))<math.radians(30) and d<5:
                vw=abs(math.degrees(math.atan2(cx-px,cy-py))) if true else 0
                if true and vw>55: continue
                wpx=1.2*(math.cos(math.radians(vw)) if true else 1)*FX/d
                ds=[d if rnd.random()>zeros else 0.0 for _ in range(25)]
                pals.append(PalletDetection(320+FX*xr/z,wpx,rnd.uniform(.6,.99) if true else rnd.uniform(.5,.85),ds,t))
        # scan
        n=60; rngs=[]
        for i in range(n):
            a=-math.pi+i*math.radians(6); ang=yaw+a
            best=float('inf')
            # pallet body rectangle x[-.6,.6], y[-1,0]
            for k in range(1,50):
                r=k*0.05
                px=x+r*math.cos(ang); py=y+r*math.sin(ang)
                if -0.6<=px<=0.6 and -1.0<=py<=0: best=r;break
                if any(math.hypot(px-fx,py-fy)<0.3 for fx,fy in fake): best=r;break
            rngs.append(best)
        obs=Observation(t,Odom(ox,oy,oyaw,v,w),tags,pals,Scan(-math.pi,math.radians(6),rngs))
        c=ctl.step(obs)
        if verbose and int(t*10)%20==0: print(round(t,1),ctl.st,round(x,2),round(y,2),round(yaw,2),round(c.v,2),round(c.w,2))
        if c.status!='RUNNING':
            camx=x+CAM_X*math.cos(yaw);camy=y+CAM_X*math.sin(yaw)
            hd=math.degrees(wrap(yaw+math.pi/2))
            col=(-0.6-0.3<=x<=0.9 and -1.3<=y<=0.3)
            ok=c.status=='DOCKED' and abs(camx)<=0.06 and abs(camy-0.85)<=0.12 and abs(hd)<=6
            return c.status,ok,round(t,1),round(camx,3),round(camy,3),round(hd,1)
        cv=max(-.3,min(.3,c.v)); cw=max(-.6,min(.6,c.w))
        # motion with small odom drift/noise
        for _ in range(1):
            dx=cv*math.cos(yaw)*DT; dy=cv*math.sin(yaw)*DT; dth=cw*DT
            x+=dx;y+=dy;yaw=wrap(yaw+dth)
            od=cv*DT; ox+=od*math.cos(oyaw+dth/2)*1.0; oy+=od*math.sin(oyaw+dth/2); oyaw=wrap(oyaw+dth*1.01)
        v,w=cv,cw
        if abs(x)<0.9 and -1.3<y<0.3 and (min(abs(x)-0.6,0)>-1) and (-0.6-0.3<x<0.6+0.3 and -1.3<y<0.3):
            return 'COLLISION',False,round(t,1),x,y,0
        t+=DT
    return 'HANG',False,t,0,0,0
if __name__=='__main__':
    res=[]
    for stage,dists in (('guidance',[2.2,2.5,2.8]),('approach',[3.2,3.6,4.0])):
        for vd in (-45,-30,-15,0,15,30,45) if stage=='guidance' else (-15,0,15):
            for d in dists:
                r=run(stage,d,vd,hash((stage,d,vd))%1000)
                res.append(r); print(stage,d,vd,r)
    print(sum(1 for r in res if r[1]),len(res))
