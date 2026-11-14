import io,sys
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/zout.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
for i,z in enumerate(b.Zones()):
    if z.GetNetname()!='GND': continue
    ol=z.Outline()
    print("zone",i,z.GetNetname(),"shapes",ol.OutlineCount(),"verts",ol.TotalVertices(),"fill polys check")
    for oi in range(ol.OutlineCount()):
        o=ol.Outline(oi)
        pts=[(round(MM(o.CPoint(k).x),2),round(MM(o.CPoint(k).y),2)) for k in range(o.PointCount())]
        print(f"  outline {oi} pts={len(pts)} first={pts[:8]}")
        if len(pts)<=60: print("   all:",pts)
    # also: does outline cover antenna region?
    for pt in ((100.0,20.0),(103.2,20.0),(100.0,5.0),(96.0,30.0)):
        v=pcbnew.VECTOR2I(int(pt[0]*1e6),int(pt[1]*1e6))
        print("   outline contains",pt,ol.Contains(v))
