import io,sys,os,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
def seg_pt(ax,ay,bx,by,px,py):
    dx,dy=bx-ax,by-ay; L=dx*dx+dy*dy
    t=0.0 if L==0 else max(0.0,min(1.0,((px-ax)*dx+(py-ay)*dy)/L))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))
def polys_of(it):
    ps=pcbnew.SHAPE_POLY_SET()
    try: it.TransformShapeToPolygon(ps, it.GetLayer(), 0, 5000, pcbnew.ERROR_OUTSIDE)
    except Exception: return None
    chains=[ps.GetOutline(i) for i in range(ps.OutlineCount())]
    return chains
def min_dist(chains,px,py):
    best=1e9; inside=False
    for c in chains:
        if c.PointCount()<1: continue
        try:
            if c.PointInside(pcbnew.VECTOR2I(int(px*IU),int(py*IU))): inside=True
        except Exception: pass
        n=c.PointCount()
        for i in range(n):
            a=c.CPoint(i); b=c.CPoint((i+1)%n)
            d=seg_pt(a.x/IU,a.y/IU,b.x/IU,b.y/IU,px,py)
            if d<best: best=d
    if inside: return 0.0
    return best
def run(path,label,show=False):
    d,fn=os.path.split(os.path.abspath(path))
    fp=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0])
    if fp is None: print(label,'LOAD FAIL'); return
    items=[(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),fp.Reference().GetLayerName())]
    pads=[(pd.GetNumber(),pd.GetPosition().x/IU,pd.GetPosition().y/IU,pd.GetSize().x/IU/2.0) for pd in fp.Pads()]
    bad=[]; n=0; empty=0
    for it,l in items:
        if 'Silk' not in l: continue
        n+=1
        ch=polys_of(it)
        if not ch or sum(c.PointCount() for c in ch)==0:
            empty+=1; continue
        worst=(1e9,None)
        for num,px,py,pr in pads:
            dd=min_dist(ch,px,py)
            if dd<worst[0]: worst=(dd,num,pr)
        if worst[0] < worst[2]:
            bad.append((round(worst[0]-worst[2],4), getattr(it,'GetText',lambda:'shape')(), worst[1]))
    print('%-28s silk=%-3d polys_ok=%-3d empty=%-3d  overlaps=%d'%(label,n,n-empty,empty,len(bad)))
    if show or bad:
        for b in sorted(bad)[:50]: print('   ',b)
    return bad
run('kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod','TARGET 2x22',True)
run('../../kicad/kamaz_leveler_hw.pretty/ESP32-DevKitC_Socket_2x19_P2.54mm.kicad_mod','BASE 2x19')
run('../../kicad/kamaz_leveler_hw.pretty/ESP32-DevKitC_Socket_2x15_P2.54mm.kicad_mod','BASE 2x15')
