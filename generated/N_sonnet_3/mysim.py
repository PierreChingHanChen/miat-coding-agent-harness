import math, random, json, sys
import controller_synthesized as cs
import actions as A
from sim.api import *
P=json.load(open('model/miat2_grafcet.json'))['controls']
D=json.load(open('model/miat2_grafcet.json'))['data']
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
class C: pass
def run(stage, dist, view_deg, seed, clutter=False, zero_frac=0.4):
    rnd=random.Random(seed)
    # true pose in pallet frame
    if stage=='guidance':
        a=math.radians(view_deg); bx=dist*math.sin(a); by=dist*math.cos(a)
        th=math.atan2(-by,-bx)
    else:
        a=math.radians(view_deg); bx=dist*math.sin(a); by=dist*math.cos(a)
        th=rnd.uniform(-math.pi,math.pi)
    x,y=bx,by
    x0,y0,th0=x,y,th
    fakes=[(2.5,1.0,0.8)] if clutter else []  # fake pallet pos in pallet frame
    S=cs.State(); cs.reset_all(S,0.0)
    c=C()
    for k,v in D.items(): setattr(c,k,json.loads(json.dumps(v)))
    c.P=P; c.start_approach=(stage=='approach'); c.start_guidance=not c.start_approach
    c.scan=Scan(-math.pi,math.radians(2),[5.0]*180)
    t=0.0; v=w=0
    for i in range(1500):
        t=i*0.1
        # odom in start frame
        dx,dy=x-x0,y-y0
        ox=math.cos(-th0)*dx-math.sin(-th0)*dy; oy=math.sin(-th0)*dx+math.cos(-th0)*dy
        c.odom=Odom(ox+rnd.gauss(0,0.0005)*0,oy,wrap(th-th0),v,w)
        c.t=t
        camx=x+CAM_X*math.cos(th); camy=y+CAM_X*math.sin(th)
        tags=[]; pals=[]
        # tag
        rel=(-camx,-camy)
        f=(math.cos(th),math.sin(th)); l=(-f[1],f[0])
        z=rel[0]*f[0]+rel[1]*f[1]; xr=-(rel[0]*l[0]+rel[1]*l[1])
        dd=math.hypot(*rel)
        if z>0:
            bearing=math.atan2(xr,z)
            view=abs(math.degrees(math.atan2(camx,camy)))
            env={0:2.77,15:2.52,30:2.42,45:2.02}
            lim=2.77-(2.77-2.02)*min(view,45)/45
            if dd>=0.73 and abs(bearing)<math.radians(34.5) and dd<=lim and camy>0 and rnd.random()<0.8:
                yaw=wrap(th+math.pi/2)
                tags.append(TagDetection(7,xr+rnd.gauss(0,0.008),z+rnd.gauss(0,0.02),yaw+rnd.gauss(0,math.radians(1.5)),t))
        def pal(px_,py_,w_m,score):
            relx,rely=px_-x,py_-y
            fz=(relx-0)*0
            # in base frame
            xb=math.cos(th)*relx+math.sin(th)*rely; yb=-math.sin(th)*relx+math.cos(th)*rely
            zc=xb-CAM_X
            if zc<0.5: return None
            if abs(math.atan2(-yb,zc))>math.radians(34.5): return None
            u=320+FX*(-yb)/zc
            wpx=w_m*FX/zc
            ds=[0.0 if rnd.random()<zero_frac else (zc+rnd.uniform(-.02,.02) if rnd.random()<0.8 else zc+rnd.uniform(0.5,2)) for _ in range(25)]
            return PalletDetection(u,wpx,score,ds,t)
        if rnd.random()<0.9:
            p=pal(0,0,1.2*abs(math.cos(math.atan2(camx,camy))),rnd.uniform(0.6,0.99))
            if p: pals.append(p)
        for fx_,fy_,s in fakes:
            p=pal(fx_,fy_,1.15,rnd.uniform(.45,.88))
            if p and rnd.random()<0.9: pals.append(p)
        c.tags=tags; c.pallets=pals
        cs.tick(S,c,A)
        st=getattr(c,'status','RUNNING') if hasattr(c,'status') else 'RUNNING'
        cmd=c.cmd
        v=max(-.3,min(.3,cmd.v)); w=max(-.6,min(.6,cmd.w))
        if st!='RUNNING' and st in('DOCKED','ABORTED'): 
            v=w=0
            camx=x+CAM_X*math.cos(th); camy=y+CAM_X*math.sin(th)
            return st,t,(camx,camy,math.degrees(wrap(th+math.pi/2))),S.trace
        th=wrap(th+w*0.1); x+=v*math.cos(th)*0.1; y+=v*math.sin(th)*0.1
        if -0.9<x<0.9 and -1.3<y<0.3: return 'COLLISION',t,(x,y),S.trace
    return 'HANG',t,None,S.trace
if __name__=='__main__':
    for stage,dist,views in (('guidance',2.5,[0,15,30,45,-30,-45]),('guidance',2.2,[0,30,-45]),('approach',3.6,[0,10,-15])):
        for vd in views:
            for sd in range(3):
                r=run(stage,dist,vd,sd,clutter=(sd==2))
                tr=[x[1] for x in r[3]]
                print(stage,dist,vd,sd,r[0],round(r[1],1),r[2] and tuple(round(q,3) for q in r[2]))
