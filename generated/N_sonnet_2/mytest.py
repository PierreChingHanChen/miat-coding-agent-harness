import math, random, json, sys
from types import SimpleNamespace as NS
from sim.api import *
import controller_synthesized as CS
import actions as A
M = json.load(open('model/miat2_grafcet.json'))
wrap=lambda a: math.atan2(math.sin(a),math.cos(a))

def run(mode, seed, r0, ang_deg, clutter=False, zero_frac=0.4, hdg=None, verbose=False):
    rnd=random.Random(seed)
    ang=math.radians(ang_deg)
    # pallet frame: AGV base at distance r0 (base->tag) at viewing angle ang from +y axis
    bx=r0*math.sin(ang); by=r0*math.cos(ang)
    if mode=='guidance':
        bh=math.atan2(-by,-bx)
    else:
        bh=rnd.uniform(-math.pi,math.pi)
        if hdg is not None: bh=hdg
    x,y,h=bx,by,bh
    ox,oy,oyaw=0.0,0.0,0.0
    objs=[]
    if clutter:
        objs.append((rnd.uniform(-2.5,2.5), rnd.uniform(2.5,4.5), rnd.uniform(0,6.28)))
    P=M['controls']
    S=CS.State(); C=NS(**{k:v for k,v in M['data'].items()})
    C.P=P; C.start_approach=(mode=='approach'); C.start_guidance=(mode=='guidance')
    C.cmd=Command(); C.status='RUNNING'; C.scan=Scan(-math.pi,math.radians(2),[5.0]*180)
    t=0.0; CS.reset_all(S,t)
    v=w=0.0; vm=wm=0.0
    drift=rnd.uniform(-0.02,0.02)
    while t<160:
        # observation
        tags=[]; pallets=[]
        cx=x+CAM_X*math.cos(h); cy=y+CAM_X*math.sin(h)
        # tag
        dx,dy=-cx,-cy
        r=math.hypot(dx,dy)
        az=wrap(math.atan2(dy,dx)-h)
        view=math.degrees(abs(math.atan2(cx,cy)))
        env=2.77-(2.77-2.02)*min(view,45)/45 if view<=45 else 2.0
        if view>60: env=0
        if r>0.73 and abs(az)<math.radians(34.5) and r<env and cy>0 and rnd.random()<0.85:
            # camera-frame
            f=(math.cos(h),math.sin(h)); l=(-math.sin(h),math.cos(h))
            z=dx*f[0]+dy*f[1]; xl=dx*l[0]+dy*l[1]
            yaw=h+math.pi/2
            sg=0.01*z
            tags.append(TagDetection(7,-xl+rnd.gauss(0,sg),z+rnd.gauss(0,sg),yaw+rnd.gauss(0,math.radians(1.0)*z/1.5),t))
        def pal(px,py,pn,score):
            ex,ey=px-cx,py-cy
            rr=math.hypot(ex,ey)
            a=wrap(math.atan2(ey,ex)-h)
            if abs(a)>math.radians(30) or rr>7 or rr<0.5: return
            zz=rr*math.cos(a)
            u=320+FX*math.tan(-a)
            view=abs(wrap(math.atan2(ey,ex)-pn))  # angle between LOS and face normal(pointing into object)
            wpx=FX*1.2*abs(math.cos(wrap(math.atan2(-ey,-ex)-pn)))/zz
            ds=[]
            for i in range(25):
                q=rnd.random()
                if q<zero_frac: ds.append(0.0)
                elif q<zero_frac+0.15: ds.append(zz+rnd.uniform(0.5,2))
                else: ds.append(zz+rnd.gauss(0,0.01))
            pallets.append(PalletDetection(u,wpx,score,ds,t))
        if rnd.random()<0.9:
            pal(0,0,math.pi/2*-1 if False else -math.pi/2, rnd.uniform(0.6,0.99))
        for (ox2,oy2,_) in objs:
            if rnd.random()<0.8: pal(ox2,oy2,-math.pi/2,rnd.uniform(0.45,0.88))
        C.t=t; C.odom=Odom(ox,oy,oyaw,vm,wm); C.tags=tags; C.pallets=pallets
        CS.tick(S,C,A)
        cmd=clamp_cmd(C.cmd)
        if C.status!='RUNNING':
            cx=x+CAM_X*math.cos(h); cy=y+CAM_X*math.sin(h)
            lat=cx; dist=cy; he=math.degrees(abs(wrap(h+math.pi/2)))
            good=abs(lat)<=0.06 and abs(dist-0.85)<=0.12 and he<=6
            return C.status,good,t,(lat,dist,he),S.trace if verbose else None
        # dynamics
        v,w=cmd.v,cmd.w
        nx=x+v*DT*math.cos(h+w*DT/2); ny=y+v*DT*math.sin(h+w*DT/2); nh=wrap(h+w*DT)
        if ny<0.3 and abs(nx)<0.9: return 'COLLIDE',False,t,None,None
        # odom
        d=v*DT*(1+drift); dth=w*DT*(1+drift)
        ox+=d*math.cos(oyaw+dth/2); oy+=d*math.sin(oyaw+dth/2); oyaw=wrap(oyaw+dth)
        x,y,h=nx,ny,nh; vm,wm=v,w
        t+=DT
    return 'HANG',False,t,None,None

if __name__=='__main__':
    mode=sys.argv[1]
    res={}
    n=0
    for seed in range(int(sys.argv[2])):
        rnd=random.Random(1000+seed)
        if mode=='guidance':
            r0=rnd.uniform(2.2,2.8); ang=rnd.uniform(-45,45)
        else:
            r0=rnd.uniform(3.2,4.0); ang=rnd.uniform(-15,15)
        st,good,t,info,tr=run(mode,seed,r0,ang,clutter=len(sys.argv)>3)
        key=(st,good)
        res.setdefault(key,[]).append((seed,round(r0,2),round(ang),round(t,1)))
    for k,v in res.items(): print(k,len(v),v[:8])
