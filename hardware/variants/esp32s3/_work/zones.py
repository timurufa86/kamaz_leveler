import io,sys
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace'); sys.stderr=open('_work/zones.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
for i,z in enumerate(b.Zones()):
    lays=[b.GetLayerName(l) for l in (pcbnew.F_Cu,pcbnew.B_Cu) if z.IsOnLayer(l)]
    print(f"zone{i} net={z.GetNetname()!r} layers={lays} minw={MM(z.GetMinThickness())} prio={z.GetAssignedPriority()} rulearea={z.GetIsRuleArea()} fp={z.GetZoneName()!r}")
    for lay in (pcbnew.F_Cu,pcbnew.B_Cu):
        if not z.IsOnLayer(lay): continue
        ps=z.GetFilledPolysList(lay)
        bb=ps.BBox(); ol=z.Outline(); ob=ol.BBox()
        print(f"   {b.GetLayerName(lay)} fill outlines={ps.OutlineCount()} verts={ps.TotalVertices()} bbox x={MM(bb.GetLeft()):.2f}..{MM(bb.GetRight()):.2f} y={MM(bb.GetTop()):.2f}..{MM(bb.GetBottom()):.2f}")
        print(f"   {b.GetLayerName(lay)} outl bbox x={MM(ob.GetLeft()):.2f}..{MM(ob.GetRight()):.2f} y={MM(ob.GetTop()):.2f}..{MM(ob.GetBottom()):.2f}")
