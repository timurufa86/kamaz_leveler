import io,sys
sys.path.insert(0,'_work')
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
SRC=sys.argv[1]; DST=sys.argv[2]
b=pcbnew.LoadBoard(SRC)
print("zones before:",b.GetAreaCount())
for i,z in enumerate(b.Zones()):
    print("  zone",i,z.GetNetname(),"fill outlines F/B:",
          z.GetFilledPolysList(pcbnew.F_Cu).OutlineCount() if z.IsOnLayer(pcbnew.F_Cu) else '-',
          z.GetFilledPolysList(pcbnew.B_Cu).OutlineCount() if z.IsOnLayer(pcbnew.B_Cu) else '-')
f=pcbnew.ZONE_FILLER(b)
ok=f.Fill(b.Zones())
print("fill ok:",ok)
b.BuildConnectivity()
pcbnew.SaveBoard(DST,b)
b2=pcbnew.LoadBoard(DST)
tot=0
for i,z in enumerate(b2.Zones()):
    for lay in (pcbnew.F_Cu,pcbnew.B_Cu):
        if z.IsOnLayer(lay):
            ps=z.GetFilledPolysList(lay)
            print("  after zone",i,z.GetNetname(),b2.GetLayerName(lay),"outlines",ps.OutlineCount(),"verts",ps.TotalVertices())
print("saved",DST)
