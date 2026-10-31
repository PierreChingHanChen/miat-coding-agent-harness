import math, random, sys
sys.path.insert(0,'.')
from sim.api import *
import controller as C

def wrap(a): return math.atan2(math.sin(a),math.cos(a))

def raycast(px,py,ang):
    # rectangle x[-.6,.6], y[-1,0]
    best=float('inf')
    dx,dy=math.cos(ang),math.sin(ang)
    for r in [i*0.05 for i in range(1,200)]:
        x,y=px+r*dx,py+r*dy
        if -0.6<=x<=0.6 and -1<=y<=0: return r
    return best

def run(stage,seed,ang_deg=None,dist=None,faults=False,maxt=160):
    rnd=random.Random(seed)
    if stage=='guidance':
        d=dist or rnd.uniform(2.2,2.8); a=math.radians(ang_deg if ang_deg is not None else rnd.uniform(-45,45))
        cx0,cy0=d*math.sin(a),d*math.cos(a)  # camera pos
        yaw=math.atan2(-cy0,-cx0)
        px,py=cx0-CAM_X*math.cos(yaw),cy0-CAM_X*math.sin(yaw)
    else:
        d=rnd.uniform(3.2,4.0); a=math.radians(rnd.uniform(-15,15))
        px,py=d*math.sin(a),d*math.cos(a); yaw=rnd.uniform(-math.pi,math.pi)
    st=(px,py,yaw); ox=oy=oyaw=0.0
    ctl=C.DockingController(); ctl.reset(stage)
    t=0.0; v=w=0.0
    lim=120 if stage=='guidance' else 150
    while t<lim+5:
        # observation
        cam=(px+CAM_X*math.cos(yaw),py+CAM_X*math.sin(yaw))
        rx,ry=-cam[0],-cam[1]
        zf=rx*math.cos(yaw)+ry*math.sin(yaw); xr=rx*math.sin(yaw)-ry*math.cos(yaw)
        tags=[]
        dist_t=math.hypot(rx,ry)
        view=math.degrees(abs(wrap(math.atan2(cam[0],cam[1]))))
        env=2.77-(2.77-2.02)*min(view,45)/45
        if zf>0 and dist_t>=0.73 and abs(math.atan2(xr,zf))<math.radians(34.5) and dist_t<env and view<60 and rnd.random()<0.85:
            tags.append(TagDetection(7,xr+rnd.gauss(0,0.01),zf+rnd.gauss(0,0.01),wrap(yaw+math.pi/2)+rnd.gauss(0,0.01),t))
        pals=[]
        if zf>0.3 and abs(math.atan2(xr,zf))<math.radians(34.5) and cam[1]>0 and rnd.random()<0.9:
            b=math.atan2(xr,zf); dd=math.hypot(rx,ry)
            ds=[0.0 if (faults and rnd.random()<0.4) or rnd.random()<0.1 else dd+rnd.gauss(0,0.03) for _ in range(25)]
            ds[0]=3.0
            wpx=1.2*abs(math.cos(math.atan2(cam[0],cam[1])))*FX/dd
            pals.append(PalletDetection(320+math.tan(b)*FX,wpx,rnd.uniform(0.7,0.95),ds,t))
        ranges=[raycast(px,py,yaw+(-math.pi+i*math.radians(2))) for i in range(180)]
        ranges=[r if r<10 else float('inf') for r in ranges]
        obs=Observation(t,Odom(ox,oy,oyaw,v,w),tags,pals,Scan(-math.pi,math.radians(2),ranges))
        cmd=ctl.step(obs)
        if cmd.status!='RUNNING':
            camx,camy=px+CAM_X*math.cos(yaw),py+CAM_X*math.sin(yaw)
            lat=camx; dist_f=camy; head=math.degrees(abs(wrap(yaw+math.pi/2)))
            good = cmd.status=='DOCKED' and abs(lat)<=0.06 and abs(dist_f-0.85)<=0.12 and head<=6
            return cmd.status,good,(round(lat,3),round(dist_f,3),round(head,1)),round(t,1)
        c=clamp_cmd(cmd); v,w=c.v,c.w
        # integrate (true), odometry w/ small drift
        dt=DT
        nx=px+v*math.cos(yaw)*dt; ny=py+v*math.sin(yaw)*dt; nyaw=wrap(yaw+w*dt)
        # collision
        if -0.9<=nx<=0.9 and -1.3<=ny<=0.3:
            qx=max(-.6,min(.6,nx)); qy=max(-1,min(0,ny))
            if math.hypot(nx-qx,ny-qy)<0.30: return 'COLLISION',False,(round(nx,2),round(ny,2)),round(t,1)
        px,py,yaw=nx,ny,nyaw
        ds=v*dt*(1+0.01); dyw=w*dt*(1+0.01)
        oyaw=wrap(oyaw+dyw); ox+=ds*math.cos(oyaw); oy+=ds*math.sin(oyaw)
        t+=dt
    return 'HANG',False,None,t

if __name__=='__main__':
    res={}
    for stage in ['guidance','approach']:
        ok=0;n=0;fails=[]
        for seed in range(24):
            r=run(stage,seed,faults=(seed%4==0)); n+=1; ok+=r[1]
            if not r[1]: fails.append((seed,r))
        print(stage,ok,n,fails[:8])
    for a in (-45,-30,0,30,45):
        for d in (2.2,2.5,2.8):
            r=run('guidance',1,a,d); print(a,d,r)
