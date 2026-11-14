import io,sys,math,heapq,json
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass
import pcbnew
MM=pcbnew.ToMM; FM=pcbnew.FromMM
FC=pcbnew.F_Cu; BC=pcbnew.B_Cu
W=0.15; HW=W/2.0; CLR=0.15; EPS=0.008
VIA_D=0.45; VIA_DR=0.20; VIA_R=VIA_D/2.0
BOARD_MIN=(0.55,0.55); BOARD_MAX=(103.45,53.45)
ANT_X=95.0          # forbid new copper right of this (antenna end of the module)
CELL=1.0

def v2(x,y): return pcbnew.VECTOR2I(int(round(x*1e6)),int(round(y*1e6)))

def pt_seg(px,py,x1,y1,x2,y2):
    dx=x2-x1; dy=y2-y1; L2=dx*dx+dy*dy
    if L2==0.0: return math.hypot(px-x1,py-y1)
    t=((px-x1)*dx+(py-y1)*dy)/L2
    if t<0:t=0.0
    elif t>1:t=1.0
    return math.hypot(px-(x1+t*dx),py-(y1+t*dy))

def _cross(ax,ay,bx,by): return ax*by-ay*bx

def seg_seg(x1,y1,x2,y2,x3,y3,x4,y4):
    d1=_cross(x2-x1,y2-y1,x3-x1,y3-y1); d2=_cross(x2-x1,y2-y1,x4-x1,y4-y1)
    d3=_cross(x4-x3,y4-y3,x1-x3,y1-y3); d4=_cross(x4-x3,y4-y3,x2-x3,y2-y3)
    if ((d1>0 and d2<0) or (d1<0 and d2>0)) and ((d3>0 and d4<0) or (d3<0 and d4>0)):
        return 0.0
    return min(pt_seg(x1,y1,x3,y3,x4,y4),pt_seg(x2,y2,x3,y3,x4,y4),
               pt_seg(x3,y3,x1,y1,x2,y2),pt_seg(x4,y4,x1,y1,x2,y2))

def seg_rect(x1,y1,x2,y2,cx,cy,hx,hy,ang):
    ca=math.cos(-ang); sa=math.sin(-ang)
    def tr(px,py):
        dx=px-cx; dy=py-cy
        return (dx*ca-dy*sa, dx*sa+dy*ca)
    ax,ay=tr(x1,y1); bx,by=tr(x2,y2)
    # inside test: midpoint / endpoints
    def inside(px,py): return -hx<=px<=hx and -hy<=py<=hy
    if inside(ax,ay) or inside(bx,by): return 0.0
    # proper segment-AABB intersection test via clipping (Liang-Barsky)
    t0=0.0; t1=1.0; dx=bx-ax; dy=by-ay; ok=True
    for p,q in ((-dx,ax+hx),(dx,hx-ax),(-dy,ay+hy),(dy,hy-ay)):
        if abs(p)<1e-15:
            if q<0: ok=False; break
        else:
            r=q/p
            if p<0:
                if r>t1: ok=False; break
                if r>t0: t0=r
            else:
                if r<t0: ok=False; break
                if r<t1: t1=r
    if ok and t0<=t1: return 0.0
    best=1e18
    for (ex1,ey1,ex2,ey2) in ((-hx,-hy,hx,-hy),(hx,-hy,hx,hy),(hx,hy,-hx,hy),(-hx,hy,-hx,-hy)):
        d=seg_seg(ax,ay,bx,by,ex1,ey1,ex2,ey2)
        if d<best: best=d
    return best

class Obs:
    __slots__=('kind','net','lay','x1','y1','x2','y2','r','cx','cy','hx','hy','ang','tag','bx0','by0','bx1','by1','inf')
    def __init__(self,kind,net,lay,**kw):
        self.kind=kind; self.net=net; self.lay=lay; self.inf=0.0
        for k,v in kw.items(): setattr(self,k,v)

class Index:
    def __init__(self):
        self.grid={}
        self.maxr=0.5
    def add(self,o):
        r=0.0
        if o.kind=='cap':
            r=o.r; self.maxr=max(self.maxr,r)
            x0=min(o.x1,o.x2)-r; x1=max(o.x1,o.x2)+r; y0=min(o.y1,o.y2)-r; y1=max(o.y1,o.y2)+r
        else:
            hd=math.hypot(o.hx,o.hy)+o.inf; self.maxr=max(self.maxr,hd)
            x0=o.cx-hd; x1=o.cx+hd; y0=o.cy-hd; y1=o.cy+hd
        o.bx0,o.by0,o.bx1,o.by1=x0,y0,x1,y1
        i0=int(math.floor(x0/CELL)); i1=int(math.floor(x1/CELL))
        j0=int(math.floor(y0/CELL)); j1=int(math.floor(y1/CELL))
        for lay in o.lay:
            for i in range(i0,i1+1):
                for j in range(j0,j1+1):
                    self.grid.setdefault((lay,i,j),[]).append(o)
    def remove(self,objs):
        dead=set(id(o) for o in objs)
        for k in list(self.grid.keys()):
            lst=self.grid[k]
            nl=[o for o in lst if id(o) not in dead]
            if len(nl)!=len(lst):
                if nl: self.grid[k]=nl
                else: del self.grid[k]
    def near(self,x0,y0,x1,y1,lay,R):
        R=R+self.maxr
        i0=int(math.floor((x0-R)/CELL)); i1=int(math.floor((x1+R)/CELL))
        j0=int(math.floor((y0-R)/CELL)); j1=int(math.floor((y1+R)/CELL))
        seen=set(); out=[]
        g=self.grid
        for i in range(i0,i1+1):
            for j in range(j0,j1+1):
                lst=g.get((lay,i,j))
                if lst:
                    for o in lst:
                        if id(o) not in seen: seen.add(id(o)); out.append(o)
        return out

def dist_to(o,x1,y1,x2,y2):
    if o.kind=='cap':
        d=seg_seg(x1,y1,x2,y2,o.x1,o.y1,o.x2,o.y2)
        return d-o.r
    return seg_rect(x1,y1,x2,y2,o.cx,o.cy,o.hx,o.hy,o.ang)-o.inf

HOLES=[]
def via_clear2(idx,x,y,net):
    need=VIA_R+CLR+EPS
    for lay in (FC,BC):
        for o in idx.near(x,y,x,y,lay,need):
            if o.net==net: continue
            if dist_to(o,x,y,x,y)<need: return False
    for (hx,hy,hr) in HOLES:
        if math.hypot(x-hx,y-hy) < hr+0.25+VIA_DR/2.0+0.02: return False
    return True

SAMENET_SEP=0.30
SAMENET_ON=False
def _shared(o,x1,y1,x2,y2,tol=0.16):
    for (ax,ay) in ((x1,y1),(x2,y2)):
        for (bx,by) in ((o.x1,o.y1),(o.x2,o.y2)):
            if abs(ax-bx)<tol and abs(ay-by)<tol: return True
    return False

def seg_clear(idx,lay,net,x1,y1,x2,y2,extra=0.0):
    """True if a track of half-width HW on `lay` of net `net` keeps clearance."""
    need=HW+CLR+EPS+extra
    for o in idx.near(min(x1,x2),min(y1,y2),max(x1,x2),max(y1,y2),lay,need):
        if o.net==net:
            if o.kind!='cap' or not SAMENET_ON: continue
            if _shared(o,x1,y1,x2,y2): continue
            if dist_to(o,x1,y1,x2,y2)<HW+SAMENET_SEP+EPS: return False
            continue
        if dist_to(o,x1,y1,x2,y2)<need: return False
    return True

def via_clear(idx,x,y,net):
    need=VIA_R+CLR+EPS
    for lay in (FC,BC):
        for o in idx.near(x,y,x,y,lay,need):
            if o.net==net: continue
            if dist_to(o,x,y,x,y)<need: return False
    return True

def inside_board(x,y,m=0.0):
    return BOARD_MIN[0]-m<=x<=BOARD_MAX[0]+m and BOARD_MIN[1]-m<=y<=BOARD_MAX[1]+m

def poly_ok(idx,lay,net,pts,extra=0.0,allow_ant=False):
    for (x,y) in pts:
        if not inside_board(x,y): return False
        if (not allow_ant) and x>ANT_X: return False
    for k in range(len(pts)-1):
        x1,y1=pts[k]; x2,y2=pts[k+1]
        if math.hypot(x2-x1,y2-y1)<1e-9: continue
        if not seg_clear(idx,lay,net,x1,y1,x2,y2,extra): return False
    return True

# ---------------- shapes ----------------
def _dedupe(pts):
    out=[pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0]-out[-1][0],p[1]-out[-1][1])>1e-7: out.append(p)
    return out

def _shapes(p1,p2):
    x1,y1=p1; x2,y2=p2
    dx=x2-x1; dy=y2-y1
    out=[]
    if abs(dx)<1e-9 or abs(dy)<1e-9:
        out.append([p1,p2])
    else:
        out.append([p1,p2])
        out.append([p1,(x1,y2),p2])
        out.append([p1,(x2,y1),p2])
        adx=1.0 if dx>0 else -1.0; ady=1.0 if dy>0 else -1.0
        if abs(dx)>=abs(dy):
            out.append([p1,(x1+adx*abs(dy),y2),p2])
            out.append([p1,(x2-adx*abs(dy),y1),p2])
        else:
            out.append([p1,(x2,y1+ady*abs(dx)),p2])
            out.append([p1,(x1,y2-ady*abs(dx)),p2])
    return [_dedupe(s) for s in out]

def shape_scan(idx,lay,net,p1,p2,term_pts,extra=0.0,step=0.25,win=14.0):
    """axis Z scans + basic shapes; returns list of ok candidates sorted by (n, len)."""
    cands=[]
    def tryp(pts):
        pts=_dedupe(pts)
        if len(pts)<2: return
        if not poly_ok_t(idx,lay,net,pts,term_pts,extra): return
        L=sum(math.hypot(pts[k+1][0]-pts[k][0],pts[k+1][1]-pts[k][1]) for k in range(len(pts)-1))
        cands.append((len(pts)-1,L,pts))
    for s in _shapes(p1,p2): tryp(s)
    x1,y1=p1; x2,y2=p2
    lo=min(y1,y2)-win; hi=max(y1,y2)+win
    n=int((hi-lo)/step)+1
    for k in range(n+1):
        ym=round(lo+k*step,4)
        if abs(ym-y1)<1e-9 or abs(ym-y2)<1e-9: continue
        tryp([p1,(x1,ym),(x2,ym),p2])
    lo=min(x1,x2)-win; hi=max(x1,x2)+win
    n=int((hi-lo)/step)+1
    for k in range(n+1):
        xm=round(lo+k*step,4)
        if abs(xm-x1)<1e-9 or abs(xm-x2)<1e-9: continue
        tryp([p1,(xm,y1),(xm,y2),p2])
    cands.sort(key=lambda t:(t[0],t[1]))
    return cands

def poly_ok_t(idx,lay,net,pts,term_pts,extra=0.0):
    for (x,y) in pts:
        if not inside_board(x,y): return False
        if x>ANT_X:
            if not any(abs(x-tx)<0.02 and abs(y-ty)<0.02 for (tx,ty) in term_pts): return False
    for k in range(len(pts)-1):
        x1,y1=pts[k]; x2,y2=pts[k+1]
        if math.hypot(x2-x1,y2-y1)<1e-9: continue
        if not seg_clear(idx,lay,net,x1,y1,x2,y2,extra): return False
    return True

def merge_collinear(pts):
    out=[pts[0]]
    for p in pts[1:]:
        if len(out)>=1 and math.hypot(p[0]-out[-1][0],p[1]-out[-1][1])<1e-7: continue
        out.append(p)
    k=1
    while k<len(out)-1:
        x0,y0=out[k-1]; x1,y1=out[k]; x2,y2=out[k+1]
        a1=math.atan2(y1-y0,x1-x0); a2=math.atan2(y2-y1,x2-x1)
        d=abs((a1-a2+math.pi)%(2*math.pi)-math.pi)
        if d<1e-6:
            out.pop(k)
        else: k+=1
    return out

# ---------------- A* fallback ----------------
def astar(idx,net,p1,l1,p2,l2,cell=0.2,margin=7.0,via_cost=8.0,turn_cost=0.35,maxnodes=400000):
    x0=min(p1[0],p2[0])-margin; x1=max(p1[0],p2[0])+margin
    y0=min(p1[1],p2[1])-margin; y1=max(p1[1],p2[1])+margin
    x0=max(x0,BOARD_MIN[0]); y0=max(y0,BOARD_MIN[1])
    x1=min(x1,BOARD_MAX[0]); y1=min(y1,BOARD_MAX[1])
    i0=int(math.floor(x0/cell)); j0=int(math.floor(y0/cell))
    NX=int((x1-x0)/cell)+2; NY=int((y1-y0)/cell)+2
    def cx(i): return round((i0+i)*cell,6)
    def cy(j): return round((j0+j)*cell,6)
    def cix(x): return int(round(x/cell))-i0
    def cjy(y): return int(round(y/cell))-j0
    memo={}
    def blocked(lay,i,j):
        k=(lay,i,j)
        v=memo.get(k)
        if v is None:
            x=cx(i); y=cy(j)
            if not inside_board(x,y) or x>ANT_X: v=True
            else: v=not seg_clear(idx,lay,net,x,y,x,y,0.0)
            memo[k]=v
        return v
    vmem={}
    def vblocked(i,j):
        k=(i,j); v=vmem.get(k)
        if v is None:
            v=not via_clear(idx,cx(i),cy(j),net); vmem[k]=v
        return v
    def free_pt(lay,x,y):
        return (not blocked(lay,cix(x),cjy(y)))
    si,sj=cix(p1[0]),cjy(p1[1]); ti,tj=cix(p2[0]),cjy(p2[1])
    if not (0<=si<NX and 0<=sj<NY and 0<=ti<NX and 0<=tj<NY):
        return None
    memo[(l1,si,sj)]=False; memo[(l2,ti,tj)]=False
    DIRS=[(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
    def h(i,j): return math.hypot(cx(i)-p2[0],cy(j)-p2[1])
    start=(l1,si,sj,-1)
    best={start:0.0}; prev={}
    pq=[(h(si,sj),0.0,l1,si,sj,-1)]
    found=None; n=0
    while pq:
        f,g,lay,i,j,d=heapq.heappop(pq)
        key=(lay,i,j,d)
        if g>best.get(key,1e18)+1e-12: continue
        n+=1
        if n>maxnodes: break
        if lay==l2 and i==ti and j==tj: found=key; break
        for nd,(di,dj) in enumerate(DIRS):
            ni,nj=i+di,j+dj
            if not (0<=ni<NX and 0<=nj<NY): continue
            if blocked(lay,ni,nj): continue
            step=math.hypot(di,dj)*cell
            cost=g+step+(0.0 if d==-1 or d==nd else turn_cost)
            nk=(lay,ni,nj,nd)
            if cost<best.get(nk,1e18)-1e-12:
                best[nk]=cost; prev[nk]=(key,'M')
                heapq.heappush(pq,(cost+h(ni,nj),cost,lay,ni,nj,nd))
        if not vblocked(i,j):
            ol=BC if lay==FC else FC
            cost=g+via_cost
            nk=(ol,i,j,-1)
            if cost<best.get(nk,1e18)-1e-12:
                best[nk]=cost; prev[nk]=(key,'V')
                heapq.heappush(pq,(cost+h(i,j),cost,ol,i,j,-1))
    if found is None: return None
    raw=[]; k=found
    while k in prev:
        p,kind=prev[k]; raw.append((k,kind)); k=p
    raw.append((k,None)); raw.reverse()
    return [(cx(i),cy(j),lay,kind) for (lay,i,j,d),kind in raw]

def simplify_run(idx,net,lay,pts,step=1.6,extra=0.0):
    """greedy longest-shortcut smoothing of a same-layer polyline"""
    pts=_dedupe(pts)
    if len(pts)<=2: return pts
    i=0; out=[pts[0]]
    n=len(pts)
    while i<n-1:
        # if the original first steps are short (near a pad), allow leaving them
        j=n-1; chosen=None
        while j>i+1:
            if seg_clear(idx,lay,net,pts[i][0],pts[i][1],pts[j][0],pts[j][1],extra):
                chosen=j; break
            j-=1
        if chosen is None: chosen=i+1
        out.append(pts[chosen]); i=chosen
    return merge_collinear(out)
