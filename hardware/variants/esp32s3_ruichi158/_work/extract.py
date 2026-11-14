import io,sys,json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

b = pcbnew.LoadBoard("kamaz_leveler_ruichi158.kicad_pcb")

def bb_of(items):
    x0=y0=x1=y1=None
    for it in items:
        bb = it.GetBoundingBox()
        a,bx = bb.GetX()/1e6, bb.GetY()/1e6
        c,d = (bb.GetX()+bb.GetWidth())/1e6, (bb.GetY()+bb.GetHeight())/1e6
        if x0 is None: x0,y0,x1,y1=a,bx,c,d
        else: x0=min(x0,a); y0=min(y0,bx); x1=max(x1,c); y1=max(y1,d)
    if x0 is None: return None
    return [round(x0,4),round(y0,4),round(x1,4),round(y1,4)]

out=[]
for fp in b.GetFootprints():
    ref=fp.GetReference()
    fp.SetPosition(pcbnew.VECTOR2I(0,0))
    fp.SetOrientationDegrees(0)
    pads=[p for p in fp.Pads()]
    padbb=bb_of(pads)
    fab=[g for g in fp.GraphicalItems() if g.GetLayer()==pcbnew.F_Fab]
    silk=[g for g in fp.GraphicalItems() if g.GetLayer()==pcbnew.F_SilkS]
    fabbb=bb_of(fab); silkbb=bb_of(silk)
    crt=None
    try:
        cy=fp.GetCourtyard(pcbnew.F_CrtYd)
        if cy.OutlineCount()>0:
            cbb=cy.BBox()
            crt=[round(cbb.GetX()/1e6,4),round(cbb.GetY()/1e6,4),round((cbb.GetX()+cbb.GetWidth())/1e6,4),round((cbb.GetY()+cbb.GetHeight())/1e6,4)]
    except Exception as e:
        crt=None
    padt=set(p.GetAttribute() for p in pads)
    nets={}
    for p in pads:
        n=p.GetNetname()
        if n: nets.setdefault(n,[]).append(p.GetNumber())
    out.append(dict(ref=ref, fp=str(fp.GetFPID().GetLibItemName()), pads=padbb, fab=fabbb, silk=silkbb, crt=crt,
                    npads=len(pads), tht=any(p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH for p in pads), nets=nets))
json.dump(out, open("_work/geom.json","w"), ensure_ascii=False, indent=0)
print("dumped", len(out))
# summary of sizes
def wh(r):
    bx = r['crt'] or r['fab'] or r['pads']
    return (bx[2]-bx[0], bx[3]-bx[1])
rows=sorted(out, key=lambda r: -wh(r)[0]*wh(r)[1])
tot=0
for r in rows:
    w,h=wh(r); a=(w+0.3)*(h+0.3); tot+=a
print(f"{'ref':6s} {'fp':50s} {'crt?':5s} {'W':>7s} {'H':>7s} {'area':>8s}")
for r in rows:
    w,h=wh(r)
    print(f"{r['ref']:6s} {r['fp'][:50]:50s} {str(r['crt'] is not None):5s} {w:7.2f} {h:7.2f} {w*h:8.1f}")
print("TOTAL area incl 0.3 gap:", round(tot,0))
