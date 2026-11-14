import io,sys,json
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
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
    fp.SetPosition(pcbnew.VECTOR2I(0,0)); fp.SetOrientationDegrees(0)
    pads=list(fp.Pads())
    padbb=bb_of(pads)
    fabL = pcbnew.F_Fab if side=="F" else pcbnew.B_Fab
    crtL = pcbnew.F_CrtYd if side=="F" else pcbnew.B_CrtYd
    fabbb=bb_of([g for g in fp.GraphicalItems() if g.GetLayer()==fabL])
    crt=None
    try:
        cy=fp.GetCourtyard(crtL)
        if cy.OutlineCount()>0:
            cbb=cy.BBox()
            crt=[round(cbb.GetX()/1e6,4),round(cbb.GetY()/1e6,4),round((cbb.GetX()+cbb.GetWidth())/1e6,4),round((cbb.GetY()+cbb.GetHeight())/1e6,4)]
    except Exception: crt=None
    thru=[]
    for p in pads:
        a = p.GetAttribute()
        if a in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
            pb=p.GetBoundingBox()
            thru.append([round(pb.GetX()/1e6,4),round(pb.GetY()/1e6,4),round((pb.GetX()+pb.GetWidth())/1e6,4),round((pb.GetY()+pb.GetHeight())/1e6,4)])
    nets={}
    for p in pads:
        n=p.GetNetname()
        if n: nets.setdefault(n,[]).append(p.GetNumber())
    out.append(dict(ref=ref, fp=str(fp.GetFPID().GetLibItemName()), side=side, pads=padbb, fab=fabbb, crt=crt,
                    npads=len(pads), thru=thru, nets=nets))
json.dump(out, open("_work/geom3.json","w"), ensure_ascii=False, indent=0)

G={r['ref']:r for r in out}
bad=[]
for r in out:
    if not r['crt'] or not r['pads']: continue
    c,p = r['crt'], r['pads']
    if p[0] < c[0]-0.001 or p[1] < c[1]-0.001 or p[2] > c[2]+0.001 or p[3] > c[3]+0.001:
        bad.append((r['ref'], r['pads'], r['crt']))
print('pads outside courtyard:', len(bad))
for x in bad: print('  ', x)
print('parts with through pads:', [r['ref'] for r in out if r['thru']])
print('total through pads:', sum(len(r['thru']) for r in out))
