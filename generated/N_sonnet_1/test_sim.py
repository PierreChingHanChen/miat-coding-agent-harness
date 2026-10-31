import math, json, random, sys, types
sys.path.insert(0,'.')
from sim.api import *
import controller_synthesized as CS
import actions as A

M=json.load(open('model/miat2_grafcet.json'))
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
ENV={0:2.77,15:2.52,30:2.42,45:2.02}
def env(va):
    va=abs(math.degrees(va))
    if va>60: return 0
    ks=sorted(ENV)
    if va<=0: return ENV[0]
    for a,b in zip(ks,ks[1:]):
        if va<=b: return ENV[a]+(ENV[b]-ENV[a])*(va-a)/(b-a)
    return ENV[45]-(va-45)*0.1

def run(stage, ang_deg, dist, seed, clutter=False, zero_frac=0.2, verbose=False, hdg_off=0.0):
    rnd=random.Random(seed)
    # pallet frame; robot at viewing angle ang from face normal at distance (tag->base)
    a=math.radians(ang_deg)
    px=dist*math.sin(a); py=dist*math.cos(a)
    # heading toward tag
    th=math.atan2(-py,-px)+hdg_off
    ox=oy=oyaw=0.0
    S=CS.State(); C=types.SimpleNamespace(**{k:(list(v) if isinstance(v,list) else v) for k,v in M['data'].items()})
    C.P=M['controls']; C.start_approach = stage=='approach'; C.start_guidance= not C.start_approach
    C.status='RUNNING'; C.cmd=Command()
    CS.reset_all(S,0.0)
    # false objects (pallet frame): list of (x,y)
    fakes=[(px+1.5,py-1.0+0.0),(-2.5,2.0)] if clutter else []
    fakes=[(f[0],f[1]) for f in fakes]
    t=0.0; vcur=wcur=0.0; collided=False
    odom_ov=0.0
    limit=150 if stage=='approach' else 120
    while t<limit:
        # observation
        cx=px+CAM_X*math.cos(th); cy=py+CAM_X*math.sin(th)
        fx,fy=math.cos(th),math.sin(th); lx,ly=-math.sin(th),math.cos(th)
        tags=[]
        dx,dy=-cx,-cy
        z=dx*fx+dy*fy; left=dx*lx+dy*ly; xr=-left
        psi=wrap(th+math.pi/2)
        if z>0.73:
            alpha=abs(math.atan2(xr,z))
            va=abs(math.atan2(cx,cy)) if cy>0 else 3
            dd=math.hypot(dx,dy)
            if alpha<math.radians(34.5) and dd<=env(va) and rnd.random()<0.85:
                nz=z+rnd.gauss(0,0.01*z); nx=xr+rnd.gauss(0,0.008*z); ny=psi+math.radians(rnd.gauss(0,1.0))
                if rnd.random()<0.02: ny+=math.radians(25)
                tags.append(TagDetection(TAG_ID,nx,nz,ny,t))
        pallets=[]
        def det(ox_,oy_,score,width,is_true):
            ddx,ddy=ox_-cx,oy_-cy
            zz=ddx*fx+ddy*fy; lf=ddx*lx+ddy*ly
            if zz<0.5 or zz>7: return
            u=320-lf/zz*FX
            if not(30<u<610): return
            if is_true:
                vang=abs(math.atan2(cx-ox_,cy-oy_))
                wpx=FX*width*math.cos(vang)/zz
            else: wpx=FX*width/zz
            ds=[]
            for i in range(25):
                r=rnd.random()
                if r<zero_frac: ds.append(0.0)
                elif r<zero_frac+0.2: ds.append(zz+rnd.uniform(0.8,3.0))
                else: ds.append(zz+rnd.gauss(0,0.03))
            pallets.append(PalletDetection(u+rnd.gauss(0,3),wpx,score,ds,t))
        if rnd.random()<0.9: det(0,0,rnd.uniform(0.6,0.99),1.2,True)
        for f in fakes:
            if rnd.random()<0.9: det(f[0],f[1],rnd.uniform(0.45,0.88),1.2,False)
        obs_odom=Odom(ox,oy,oyaw,vcur,wcur)
        sc=Scan(-math.pi,math.radians(2),[5.0]*180)
        C.t=t;C.odom=obs_odom;C.tags=tags;C.pallets=pallets;C.scan=sc
        C.cmd=Command()
        try:
            CS.tick(S,C,A)
        except Exception as e:
            import traceback; traceback.print_exc(); return dict(res='EXC',t=t)
        c=clamp_cmd(C.cmd)
        if C.status!='RUNNING':
            cx=px+CAM_X*math.cos(th); cy=py+CAM_X*math.sin(th)
            psi=wrap(th+math.pi/2)
            ok=abs(cx)<=0.06 and abs(cy-0.85)<=0.12 and abs(psi)<=math.radians(6)
            return dict(res=C.status,t=round(t,1),lat=round(cx,3),dist=round(cy,3),head=round(math.degrees(psi),1),ok=ok,coll=collided,trace=[x[1] for x in S.trace] if verbose else None)
        # integrate
        v,w=c.v,c.w
        dt=DT
        thm=th+w*dt/2
        px+=v*math.cos(thm)*dt; py+=v*math.sin(thm)*dt; th=wrap(th+w*dt)
        oyaw=wrap(oyaw+w*dt*1.002); ox+=v*math.cos(oyaw)*dt*1.01; oy+=v*math.sin(oyaw)*dt*1.01
        vcur,wcur=v,w
        # collision with pallet body
        qx=min(max(px,-0.6),0.6); qy=min(max(py,-1.0),0.0)
        if math.hypot(px-qx,py-qy)<0.30: collided=True
        t+=dt
    return dict(res='HANG',t=t,trace=[x[1] for x in S.trace])

if __name__=='__main__':
    stage=sys.argv[1]; clutter=len(sys.argv)>2 and sys.argv[2]=='c'
    tot=0;good=0
    for ang in ([-45,-30,-15,0,15,30,45] if stage=='guidance' else [-15,-5,0,5,15]):
        for dist in ([2.2,2.5,2.8] if stage=='guidance' else [3.2,3.6,4.0]):
            for seed in range(3):
                hd = 0.0
                if stage=='approach':
                    # start at viewpoint, arbitrary heading
                    pass
                r=run(stage,ang,dist,seed,clutter, hdg_off=(random.Random(seed*7+int(ang)).uniform(-math.pi,math.pi) if stage=='approach' else 0.0))
                tot+=1; g=(r['res']=='DOCKED' and r['ok'] and not r.get('coll'))
                good+=g
                if not g: print(ang,dist,seed,r)
    print(good,'/',tot)
