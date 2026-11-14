import io,sys,os,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
PX,PY=102.33,38.26
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
fpads=[];racks=[];rsilk=[]
for fp in b.GetFootprints():
    if 'ESP32-S3-DevKitC-1_Socket' in str(fp.GetFPID()): continue
    for p in fp.Pads():
        fpads.append((fp.GetReference(),p.GetNumber(),p.GetPosition().x/IU,p.GetPosition().y/IU,max(p.GetSize().x,p.GetSize().y)/2/IU))
    bb=fp.GetBoundingBox(); rsilk.append((fp.GetReference(),bb.GetX()/IU,bb.GetY()/IU,(bb.GetX()+bb.GetWidth())/IU,(bb.GetY()+bb.GetHeight())/IU))
for t in b.GetTracks(): racks.append((t.GetStart().x/IU,t.GetStart().y/IU,t.GetEnd().x/IU,t.GetEnd().y/IU))
def seg_rect_intersect(x1,y1,x2,y2,rx0,ry0,rx1,ry1):
    if max(x1,x2)<rx0 or min(x1,x2)>rx1 or max(y1,y2)<ry0 or min(y1,y2)>ry1: return False
    if (rx0<=x1<=rx1 and ry0<=y1<=ry1) or (rx0<=x2<=rx1 and ry0<=y2<=ry1): return True
    def cross(ax,ay,bx,by): return ax*by-ay*bx
    def segs(p,q,r,s):
        d1=cross(q[0]-p[0],q[1]-p[1],r[0]-p[0],r[1]-p[1]); d2=cross(q[0]-p[0],q[1]-p[1],s[0]-p[0],s[1]-p[1])
        d3=cross(s[0]-r[0],s[1]-r[1],p[0]-r[0],p[1]-r[1]); d4=cross(s[0]-r[0],s[1]-r[1],q[0]-r[0],q[1]-r[1])
        return ((d1>0)!=(d2>0)) and ((d3>0)!=(d4>0))
    E=[((rx0,ry0),(rx1,ry0)),((rx1,ry0),(rx1,ry1)),((rx1,ry1),(rx0,ry1)),((rx0,ry1),(rx0,ry0))]
    return any(segs((x1,y1),(x2,y2),e[0],e[1]) for e in E)
def scan(path,label):
    d,fn=os.path.split(os.path.abspath(path))
    fp=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0])
    items=[it for it in fp.GraphicalItems()]
    pads_bad=[];trk_bad=[];cmp_bad=[]
    for it in items:
        if 'Silk' not in it.GetLayerName(): continue
        bb=it.GetBoundingBox()
        x0=bb.GetX()/IU+PX; y0=bb.GetY()/IU+PY; x1=x0+bb.GetWidth()/IU; y1=y0+bb.GetHeight()/IU
        for ref,num,px,py,pr in fpads:
            dx=max(x0-px,0,px-x1); dy=max(y0-py,0,py-y1)
            if math.hypot(dx,dy)<pr: pads_bad.append((ref,num))
        for sx1,sy1,sx2,sy2 in racks:
            if seg_rect_intersect(sx1,sy1,sx2,sy2,x0,y0,x1,y1): trk_bad.append(1)
        for ref,rx0,ry0,rx1_,ry1_ in rsilk:
            if not (x1<rx0 or x0>rx1_ or y1<ry0 or y0>ry1_): cmp_bad.append(ref)
    print('%-14s foreign pads touched=%-3d tracks crossed=%-4d other-fp boxes touched=%-3d'%(
        label,len(pads_bad),len(trk_bad),len(set(cmp_bad))))
    return set(pads_bad)
a=scan('_work/BACKUP_U7_socket.kicad_mod','BEFORE')
c=scan('kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod','AFTER')
print('newly touched foreign pads:',sorted(c-a)[:12],'count',len(c-a))
print('pads touched only before:',sorted(a-c)[:12])
