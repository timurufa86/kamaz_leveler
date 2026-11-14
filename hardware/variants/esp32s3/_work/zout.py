import io,sys
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/zout.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
for i,z in enumerate(b.Zones()):
    ol=z.Outline()
    print("zone",i,z.GetNetname(),"outlines",ol.OutlineCount(),"total_verts",ol.TotalVertices())
    for oi in range(ol.OutlineCount()):
        o=ol.Outline(oi)
        print("  outline",oi,"pts",o.PointCount())
        if o.PointCount()<=40:
            print("   ", [(round(MM(o.CPoint(k).x),2),round(MM(o.CPoint(k).y),2)) for k in range(o.PointCount())])
        for hi in range(o.HoleCount()):
            h=o.Hole(hi)
            print("   hole",hi,"pts",h.PointCount())
            if h.PointCount()<=40:
                print("    ", [(round(MM(h.CPoint(k).x),2),round(MM(h.CPoint(k).y),2)) for k in range(h.PointCount())])
