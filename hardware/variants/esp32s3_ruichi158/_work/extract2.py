import io,sys,json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard("kamaz_leveler_ruichi158.kicad_pcb")

def bb_of(items):
    x0=y0=x1=y1=None
    for it in items:
        bb = it.GetBoundingBox()
        a,by = bb.GetX()/1e6, bb.GetY()/1e6
        c,d = (bb.GetX()+bb.GetWidth())/1e6, (bb.GetY()+bb.GetHeight())/1e6
        if x0 is None: x0,y0,x1,y1=a,by,c,d
        else: x0=min(x0,a); y0=min(y0,by); x1=max(x1,c); y1=max(y1,d)
    if x0 is None: return None
    return [round(x0,4),round(y0,4),round(x1,4),round(y1,4)]

out=[]
for fp in b.GetFootprints():
    ref=fp.GetReference()
    side = "F" if fp.GetLayer()==pcbnew.F_Cu else "B"
    fp.SetPosition(pcbnew.VECTOR2I(0,0))
    fp.SetOrientationDegrees(0)
    pads=[p for p in fp.Pads()]
    padbb=bb_of(pads)
    fabL = pcbnew.F_Fab if side=="F" else pcbnew.B_Fab
    silkL= pcbnew.F_SilkS if side=="F" else pcbnew.B_SilkS
    crtL = pcbnew.F_CrtYd if side=="F" else pcbnew.B_CrtYd
    fab=[g for g in fp.GraphicalItems() if g.GetLayer()==fabL]
    silk=[g for g in fp.GraphicalItems() if g.GetLayer()==silkL]
    fabbb=bb_of(fab); silkbb=bb_of(silk)
    crt=None
    try:
        cy=fp.GetCourtyard(crtL)
        if cy.OutlineCount()>0:
            cbb=cy.BBox()
            crt=[round(cbb.GetX()/1e6,4),round(cbb.GetY()/1e6,4),round((cbb.GetX()+cbb.GetWidth())/1e6,4),round((cbb.GetY()+cbb.GetHeight())/1e6,4)]
    except Exception:
        crt=None
    nets={}
    for p in pads:
        n=p.GetNetname()
        if n: nets.setdefault(n,[]).append(p.GetNumber())
    out.append(dict(ref=ref, fp=str(fp.GetFPID().GetLibItemName()), side=side, pads=padbb, fab=fabbb, silk=silkbb, crt=crt,
                    npads=len(pads), tht=any(p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH for p in pads), nets=nets))
json.dump(out, open("_work/geom2.json","w"), ensure_ascii=False, indent=0)

def wh(r):
    bx = r['crt'] or (r['fab'] if r['fab'] and r['fab']!=r['pads'] else r['pads'])
    return (round(bx[2]-bx[0],3), round(bx[3]-bx[1],3))
for side in ("F","B"):
    rows=[r for r in out if r['side']==side]
    rows.sort(key=lambda r: -(wh(r)[0]*wh(r)[1]))
    tot=sum((wh(r)[0]+0.4)*(wh(r)[1]+0.4) for r in rows)
    print(f"=== side {side}: {len(rows)} parts, area incl 0.4 gap = {tot:.0f} mm2")
    for r in rows:
        w,h=wh(r)
        print(f"  {r['ref']:5s} {r['fp'][:34]:34s} {w:7.2f} x {h:6.2f}  crt={'Y' if r['crt'] else 'N'} tht={'Y' if r['tht'] else '-'}")
