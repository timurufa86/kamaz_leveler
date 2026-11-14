import io,sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard("kamaz_leveler_ruichi158.kicad_pcb")
print("board bbox:", b.ComputeBoundingBox().GetX()/1e6, b.ComputeBoundingBox().GetY()/1e6, b.ComputeBoundingBox().GetWidth()/1e6, b.ComputeBoundingBox().GetHeight()/1e6)
rows=[]
for fp in b.GetFootprints():
    ref=fp.GetReference()
    p=fp.GetPosition()
    ang=fp.GetOrientationDegrees()
    try:
        cy=fp.GetCourtyard(pcbnew.F_CrtYd)
        bb=cy.BBox()
        cw=bb.GetWidth()/1e6; ch=bb.GetHeight()/1e6
        ccx=(bb.GetX()+bb.GetWidth()/2)/1e6; ccy=(bb.GetY()+bb.GetHeight()/2)/1e6
        cx0=bb.GetX()/1e6; cy0=bb.GetY()/1e6; cx1=(bb.GetX()+bb.GetWidth())/1e6; cy1=(bb.GetY()+bb.GetHeight())/1e6
    except Exception as e:
        cw=ch=0; ccx=ccy=0; cx0=cy0=cx1=cy1=0
    fpbb=fp.GetBoundingBox(True, False)
    rows.append((ref, fp.GetFPID().GetLibItemName(), p.x/1e6, p.y/1e6, ang, cw, ch, cx0,cy0,cx1,cy1, fp.GetLayerName()))
rows.sort(key=lambda r:(r[1],r[0]))
print("ref\tfp\tx\ty\trot\tcw\tch\tcrt_minx\tcrt_miny\tcrt_maxx\tcrt_maxy\tlayer")
for r in rows:
    print("\t".join(str(round(v,3)) if isinstance(v,float) else str(v) for v in r))
print("total", len(rows))
