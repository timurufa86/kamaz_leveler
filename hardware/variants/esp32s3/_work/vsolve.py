import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
from collections import deque

BRD = sys.argv[1]
X0, Y0, X1, Y1 = [float(v) for v in sys.argv[2:6]]
CELL = float(sys.argv[6])
CLR = float(sys.argv[7])
VIA_D = float(sys.argv[8])
NET = sys.argv[9]
Fx, Fy = float(sys.argv[10]), float(sys.argv[11])
Bx, By = float(sys.argv[12]), float(sys.argv[13])
TW = float(sys.argv[14]) if len(sys.argv) > 14 else 0.15

b = pcbnew.LoadBoard(BRD)
FC, BC = pcbnew.F_Cu, pcbnew.B_Cu

def d_seg(px,py,x1,y1,x2,y2):
    dx,dy=x2-x1,y2-y1; L2=dx*dx+dy*dy
    if L2==0: return math.hypot(px-x1,py-y1)
    t=((px-x1)*dx+(py-y1)*dy)/L2
    t=max(0.0,min(1.0,t))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))
def d_rect(px,py,x0,y0,x1,y1):
    return math.hypot(max(x0-px,0.0,px-x1), max(y0-py,0.0,py-y1))

segs  = {FC: [], BC: []}
rects = {FC: [], BC: []}
circs = {FC: [], BC: []}
for t in b.GetTracks():
    if t.GetNetname() == NET: continue
    s,e = t.GetStart(), t.GetEnd()
    sx,sy,ex,ey = s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6
    if isinstance(t, pcbnew.PCB_VIA):
        r = t.GetWidth(FC)/2e6
        for L in (FC,BC): circs[L].append((sx,sy,r))
    else:
        L = t.GetLayer()
        if L in segs: segs[L].append((sx,sy,ex,ey,t.GetWidth()/2e6))
for fp in b.GetFootprints():
    for pd in fp.Pads():
        if pd.GetNetname() == NET: continue
        p = pd.GetPosition(); qx,qy = p.x/1e6, p.y/1e6
        sz = pd.GetSize(); hw,hh = sz.x/2e6, sz.y/2e6
        ang = pd.GetOrientationDegrees() % 180.0
        if abs(ang-90.0) < 0.01: hw,hh = hh,hw
        ls = [L for L in (FC,BC) if pd.IsOnLayer(L) or pd.GetAttribute()==pcbnew.PAD_ATTRIB_PTH]
        ok = abs(ang)<0.01 or abs(ang-90.0)<0.01 or abs(ang-180.0)<0.01
        for L in ls:
            if ok: rects[L].append((qx-hw,qy-hh,qx+hw,qy+hh))
            else: circs[L].append((qx,qy,math.hypot(hw,hh)))

NX = int(round((X1-X0)/CELL)); NY = int(round((Y1-Y0)/CELL))
def gx(i): return X0 + i*CELL
def gy(j): return Y0 + j*CELL

def blocked(L, x, y, r):
    for (a,b2,c,d,hw) in segs[L]:
        if d_seg(x,y,a,b2,c,d) < hw + CLR + r: return True
    for (a,b2,c,d) in rects[L]:
        if d_rect(x,y,a,b2,c,d) < CLR + r: return True
    for (a,b2,rr) in circs[L]:
        if math.hypot(x-a,y-b2) < rr + CLR + r: return True
    return False

HT = TW/2.0; HV = VIA_D/2.0
freeF = [[False]*NX for _ in range(NY)]
freeB = [[False]*NX for _ in range(NY)]
freeV = [[False]*NX for _ in range(NY)]
for j in range(NY):
    y = gy(j)
    for i in range(NX):
        x = gx(i)
        f = not blocked(FC, x, y, HT)
        k = not blocked(BC, x, y, HT)
        freeF[j][i] = f; freeB[j][i] = k
        freeV[j][i] = (not blocked(FC, x, y, HV)) and (not blocked(BC, x, y, HV))

def flood(free, sx, sy):
    i = int(round((sx-X0)/CELL)); j = int(round((sy-Y0)/CELL))
    seen = [[False]*NX for _ in range(NY)]
    if not (0 <= i < NX and 0 <= j < NY) or not free[j][i]:
        print('  seed not free at (%d,%d)' % (i, j)); return seen
    dq = deque([(i,j)]); seen[j][i] = True
    while dq:
        a,b3 = dq.popleft()
        for da,db in ((1,0),(-1,0),(0,1),(0,-1)):
            na,nb = a+da, b3+db
            if 0<=na<NX and 0<=nb<NY and free[nb][na] and not seen[nb][na]:
                seen[nb][na]=True; dq.append((na,nb))
    return seen
rF = flood(freeF, Fx, Fy)
rB = flood(freeB, Bx, By)
print('freeF %d reachF %d ; freeB %d reachB %d' % (sum(map(sum,freeF)), sum(map(sum,rF)), sum(map(sum,freeB)), sum(map(sum,rB))))
cands = []
for j in range(NY):
    for i in range(NX):
        if freeV[j][i] and rF[j][i] and rB[j][i]:
            cands.append((math.hypot(gx(i)-Bx, gy(j)-By), gx(i), gy(j)))
cands.sort()
print('via candidates (F-reach & B-reach & via-free): %d' % len(cands))
for c in cands[:12]:
    print('   (%.2f,%.2f)  d_from_B %.3f' % (c[1], c[2], c[0]))
# where do the regions come closest?
best = (1e9, None, None)
for j in range(NY):
    for i in range(NX):
        if rF[j][i] and not rB[j][i]:
            for dj in (-2,-1,0,1,2):
                for di in (-2,-1,0,1,2):
                    nj,nk = j+dj, i+di
                    if 0<=nj<NY and 0<=nk<NX and rB[nj][nk]:
                        dd = math.hypot(gx(i)-gx(nk), gy(j)-gy(nj))
                        if dd < best[0]: best = (dd,(gx(i),gy(j)),(gx(nk),gy(nj)))
print('closest F-reach <-> B-reach: %.3f mm  F(%.2f,%.2f) B(%.2f,%.2f)' % (best[0], best[1][0],best[1][1],best[2][0],best[2][1]) if best[1] else 'no F/B proximity')
rul = []
for i in range(NX):
    t = int(round(gx(i)*10))
    rul.append('|' if t % 50 == 0 else (str((t//10)%10) if t % 10 == 0 else ' '))
print('        ' + ''.join(rul))
for j in range(NY-1, -1, -1):
    row = []
    for i in range(NX):
        if rF[j][i] and rB[j][i]:
            row.append('X' if freeV[j][i] else 'o')
        elif rF[j][i]:
            row.append('F')
        elif rB[j][i]:
            row.append('B')
        elif freeV[j][i]:
            row.append('v')
        elif not freeF[j][i] and not freeB[j][i]:
            row.append('#')
        elif not freeF[j][i]:
            row.append('f')
        elif not freeB[j][i]:
            row.append('b')
        else:
            row.append('.')
    print('%7.2f %s' % (gy(j), ''.join(row)))
