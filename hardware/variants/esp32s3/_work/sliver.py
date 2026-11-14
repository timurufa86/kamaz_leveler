import io,sys,math
sys.path.insert(0,'_work')
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import numpy as np, pcbnew
MM=pcbnew.ToMM
SRC=sys.argv[1]; LAY={'F':pcbnew.F_Cu,'B':pcbnew.B_Cu}[sys.argv[2]]
RES=0.1
b=pcbnew.LoadBoard(SRC)
layset=pcbnew.LSET(); layset.AddLayer(LAY)
poly=pcbnew.SHAPE_POLY_SET()
for z in b.Zones():
    if z.GetIsRuleArea(): continue
    if not z.IsOnLayer(LAY): continue
    ps=z.GetFilledPolysList(LAY)
    poly.Append(ps)
pass
bb=poly.BBox()
x0=MM(bb.GetLeft())-0.2; x1=MM(bb.GetRight())+0.2; y0=MM(bb.GetTop())-0.2; y1=MM(bb.GetBottom())+0.2
NX=int((x1-x0)/RES)+1; NY=int((y1-y0)/RES)+1
print("raster %dx%d"%(NX,NY),"bbox %.2f..%.2f / %.2f..%.2f"%(x0,x1,y0,y1))
mask=np.zeros((NY,NX),dtype=bool)
edges=[]
for oi in range(poly.OutlineCount()):
    o=poly.Outline(oi)
    n=o.PointCount()
    pts=[(MM(o.CPoint(k).x),MM(o.CPoint(k).y)) for k in range(n)]
    for k in range(n):
        edges.append((pts[k],pts[(k+1)%n]))
print("edges",len(edges))
# scanline even-odd fill
ys=y0+np.arange(NY)*RES
xe=[e[0][0] for e in edges]; ye=[e[0][1] for e in edges]
xa=[e[1][0] for e in edges]; ya=[e[1][1] for e in edges]
import array
for j in range(NY):
    y=ys[j]
    xs=[]
    for k in range(len(edges)):
        yy0=ye[k]; yy1=ya[k]
        if (yy0<=y<yy1) or (yy1<=y<yy0):
            t=(y-yy0)/(yy1-yy0)
            xs.append(xe[k]+t*(xa[k]-xe[k]))
    if not xs: continue
    xs.sort()
    for q in range(0,len(xs)-1,2):
        a=int((xs[q]-x0)/RES+0.5); c=int((xs[q+1]-x0)/RES+0.5)
        if c<0 or a>=NX: continue
        a=max(0,a); c=min(NX-1,c)
        if c>=a: mask[j,a:c+1]=True
np.save('_work/mask_%s.npy'%sys.argv[2],mask)
print("copper px",int(mask.sum()))
# chamfer distance to nearest non-copper
INF=1e6
d=np.where(mask,INF,0.0)
c=RES
for it in range(64):
    n=d.copy()
    n[1:,:]=np.minimum(n[1:,:],d[:-1,:]+c)
    n[:-1,:]=np.minimum(n[:-1,:],d[1:,:]+c)
    n[:,1:]=np.minimum(n[:,1:],d[:,:-1]+c)
    n[:,:-1]=np.minimum(n[:,:-1],d[:,1:]+c)
    n[1:,1:]=np.minimum(n[1:,1:],d[:-1,:-1]+c*1.4142)
    n[:-1,:-1]=np.minimum(n[:-1,:-1],d[1:,1:]+c*1.4142)
    n[1:,:-1]=np.minimum(n[1:,:-1],d[:-1,1:]+c*1.4142)
    n[:-1,1:]=np.minimum(n[:-1,1:],d[1:,:-1]+c*1.4142)
    if np.array_equal(n,d): break
    d=n
width=2*d
print("max halfwidth %.2f"%d.max())
# thin regions: width < 0.30 and connected area
thin=(mask)&(width<0.30)
lab=np.zeros((NY,NX),dtype=np.int32); cur=0; out=[]
from collections import deque
for j in range(NY):
    for i in range(NX):
        if thin[j,i] and lab[j,i]==0:
            cur+=1; q=deque([(j,i)]); lab[j,i]=cur; pix=[]
            while q:
                a0,b0=q.popleft(); pix.append((a0,b0))
                for da in (-1,0,1):
                    for db in (-1,0,1):
                        na,nb=a0+da,b0+db
                        if 0<=na<NY and 0<=nb<NX and thin[na,nb] and lab[na,nb]==0:
                            lab[na,nb]=cur; q.append((na,nb))
            if len(pix)>=30:
                js=[p[0] for p in pix]; iis=[p[1] for p in pix]
                out.append((len(pix),x0+min(iis)*RES,x0+max(iis)*RES,y0+min(js)*RES,y0+max(js)*RES,
                            float(width[thin].max() if False else max(width[a,b_] for a,b_ in pix))))
out.sort(reverse=True)
print("thin clusters (>=30 px):",len(out))
for o in out[:15]:
    print("   px=%d x=%.2f..%.2f y=%.2f..%.2f maxw=%.2f"%o)
