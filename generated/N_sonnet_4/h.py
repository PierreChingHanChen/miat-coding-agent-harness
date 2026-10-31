import math, random, sys, types
sys.path.insert(0,'.')
import controller_synthesized as CS, actions as A, json
from sim.api import *
M=json.load(open('model/miat2_grafcet.json'))
def run(theta, dist, start, seed, noise=1.0, zero=0.4, clutter=False, head0=None, verbose=False):
    rnd=random.Random(seed)
    th=math.radians(theta)
    if start=='guidance':
        cam=(dist*math.sin(th), dist*math.cos(th)); psi=math.atan2(-cam[1],-cam[0])
        X=cam[0]-CAM_X*math.cos(psi); Y=cam[1]-CAM_X*math.sin(psi)
    else:
        X=dist*math.sin(th);Y=dist*math.cos(th); psi=rnd.uniform(-math.pi,math.pi)
    st=(X,Y,psi)
    ox=oy=oy_=0.0; oyaw=0.0
    false_objs=[(2.0,3.0),( -2.5,2.5)] if clutter else []
    S=CS.State(); C=types.SimpleNamespace(**M['data']); C.P=M['controls']
    C.start_approach=(start=='approach'); C.start_guidance=not C.start_approach
    CS.reset_all(S,0.0)
    C.status='RUNNING'; C.cmd=Command(0,0)
    vlast=wlast=0
    T=0
    while T<150:
        # truth
        cx=X+CAM_X*math.cos(psi); cy=Y+CAM_X*math.sin(psi)
        tags=[]
        rel=(-cx,-cy); z=rel[0]*math.cos(psi)+rel[1]*math.sin(psi); xr=rel[0]*math.sin(psi)-rel[1]*math.cos(psi)
        if z>0.73:
            ang=abs(math.atan2(xr,z)); view=abs(math.atan2(cx,cy)); D=math.hypot(cx,cy)
            lim=2.77 if view<math.radians(15) else 2.42 if view<math.radians(35) else 2.02
            if ang<math.radians(34.5) and D<lim and cy>0 and rnd.random()<0.8 and view<math.radians(60):
                yaw=psi+math.pi/2
                tags.append(TagDetection(7,xr+rnd.gauss(0,.01*noise*z),z+rnd.gauss(0,.01*noise*z),yaw+rnd.gauss(0,math.radians(1.5)*noise),T))
        pals=[]
        objs=[((0,0),True)]+[(p,False) for p in false_objs]
        for (px,py),true in objs:
            rx=px-cx;ry=py-cy
            zz=rx*math.cos(psi)+ry*math.sin(psi); xx=rx*math.sin(psi)-ry*math.cos(psi)
            if zz>0.5 and abs(math.atan2(xx,zz))<math.radians(34.5) and rnd.random()<0.9:
                view=abs(math.atan2(cx-px,cy-py)) if true else 0.3
                wpx=FX*1.2*math.cos(view)/zz
                ds=[]
                for i in range(25):
                    r=rnd.random()
                    ds.append(0.0 if r<zero else (zz+rnd.uniform(1,3) if r<zero+.15 else zz+rnd.gauss(0,.03)))
                sc=rnd.uniform(.6,.99) if true else rnd.uniform(.45,.88)
                pals.append(PalletDetection(320+FX*xx/zz,wpx,sc,ds,T))
        obs=types.SimpleNamespace()
        C.t=T; C.odom=Odom(ox,oy,oyaw,vlast,wlast); C.tags=tags; C.pallets=pals; C.scan=Scan(-math.pi,math.radians(2),[3.0]*180)
        CS.tick(S,C,A)
        if C.status!='RUNNING': break
        c=clamp_cmd(C.cmd); v,w=c.v,c.w
        # integrate
        psi+=w*DT; X+=v*math.cos(psi)*DT; Y+=v*math.sin(psi)*DT
        oyaw+=w*DT*1.002; ox+=v*math.cos(oyaw)*DT; oy+=v*math.sin(oyaw)*DT
        vlast,wlast=v,w
        if math.hypot(X,Y-0)<0.3 or (abs(X)<0.9 and -1.0<Y<0.3): return 'COLLISION',T,None
        T=round(T+DT,3)
    if C.status=='DOCKED':
        cx=X+CAM_X*math.cos(psi); cy=Y+CAM_X*math.sin(psi)
        e=abs(cx); h=abs(math.degrees(_w(psi+math.pi/2))); d=cy
        ok= e<=.06 and abs(d-.85)<=.12 and h<=6
        return ('OK' if ok else 'FALSE_DOCK'),T,(round(cx,3),round(d,3),round(h,1)),[x[1] for x in S.trace] if verbose else None
    return C.status if C.status!='RUNNING' else 'HANG',T,None
def _w(a): return math.atan2(math.sin(a),math.cos(a))
if __name__=='__main__':
    import collections
    res=collections.Counter(); bad=[]
    for th in (-45,-30,-15,0,15,30,45):
        for d in (2.2,2.5,2.8):
            for s in range(3):
                r=run(th,d,'guidance',s)
                res[r[0]]+=1
                if r[0]!='OK': bad.append((th,d,s,r))
    print(res); 
    for b in bad: print(b)
    res=collections.Counter(); bad=[]
    for th in (-15,0,15):
        for d in (3.2,3.6,4.0):
            for s in range(4):
                r=run(th,d,'approach',s)
                res[r[0]]+=1
                if r[0]!='OK': bad.append((th,d,s,r))
    print(res)
    for b in bad: print(b)
