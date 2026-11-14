import io,sys,math
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import numpy as np
from collections import deque
mask=np.load('_work/mask_%s.npy'%sys.argv[2])
RES=0.1; x0,y0=0.30,0.30
def disk(r):
    n=int(r)+1; o=[]
    for a in range(-n,n+1):
        for b in range(-n,n+1):
            if a*a+b*b<=r*r: o.append((a,b))
    return o
def erode(m,off):
    out=np.ones_like(m)
    for (a,b) in off:
        s=np.zeros_like(m)
        ys=slice(max(0,a),m.shape[0]+min(0,a)); xs=slice(max(0,b),m.shape[1]+min(0,b))
        yd=slice(max(0,-a),m.shape[0]+min(0,-a)); xd=slice(max(0,-b),m.shape[1]+min(0,-b))
        s[yd,xd]=m[ys,xs]
        out&=s
    return out
def dilate(m,off):
    out=np.zeros_like(m)
    for (a,b) in off:
        s=np.zeros_like(m)
        ys=slice(max(0,a),m.shape[0]+min(0,a)); xs=slice(max(0,b),m.shape[1]+min(0,b))
        yd=slice(max(0,-a),m.shape[0]+min(0,-a)); xd=slice(max(0,-b),m.shape[1]+min(0,-b))
        s[yd,xd]=m[ys,xs]
        out|=s
    return out
off=disk(1.45)
er=erode(mask,off); op=dilate(er,off)
sl=mask&~op
print("copper",int(mask.sum()),"sliver px",int(sl.sum()))
NY,NX=sl.shape
lab=np.zeros((NY,NX),dtype=np.int32); cur=0; out=[]
for j in range(NY):
    row=np.nonzero(sl[j])[0]
    for i in row:
        if lab[j,i]: continue
        cur+=1; q=deque([(j,i)]); lab[j,i]=cur; pix=[]
        while q:
            a,b=q.popleft(); pix.append((a,b))
            for da in (-1,0,1):
                for db in (-1,0,1):
                    na,nb=a+da,b+db
                    if 0<=na<NY and 0<=nb<NX and sl[na,nb] and lab[na,nb]==0:
                        lab[na,nb]=cur; q.append((na,nb))
        if len(pix)>=6:
            js=[p[0] for p in pix]; iis=[p[1] for p in pix]
            w=(max(iis)-min(iis)+1)*RES; h=(max(js)-min(js)+1)*RES
            out.append((len(pix),round(x0+min(iis)*RES,2),round(x0+max(iis)*RES,2),
                        round(y0+min(js)*RES,2),round(y0+max(js)*RES,2),round(w,2),round(h,2)))
out.sort(reverse=True)
out=[o for o in out if max(o[5],o[6])/max(1e-9,min(o[5],o[6]))>=1.8 and max(o[5],o[6])>=0.3]
print("elongated clusters:",len(out))
for o in out[:25]: print("   px=%d x=%.2f..%.2f y=%.2f..%.2f box=%.2fx%.2f"%o)
