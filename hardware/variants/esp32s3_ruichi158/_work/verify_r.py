import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

p = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb"
b = pcbnew.LoadBoard(p)

# контур
edges = [d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]
xmin = min(TOMM(d.GetBoundingBox().GetLeft()) for d in edges)
xmax = max(TOMM(d.GetBoundingBox().GetRight()) for d in edges)
ymin = min(TOMM(d.GetBoundingBox().GetTop()) for d in edges)
ymax = max(TOMM(d.GetBoundingBox().GetBottom()) for d in edges)
print(f"Edge.Cuts: n={len(edges)} bbox x[{xmin:.3f}..{xmax:.3f}] y[{ymin:.3f}..{ymax:.3f}] -> {xmax-xmin:.2f} x {ymax-ymin:.2f}")

# дуги пазов
for d in edges:
    if d.GetShape() == pcbnew.SHAPE_T_ARC and abs(TOMM(d.GetRadius()) - 3.75) < 0.01:
        c = d.GetCenter()
        print(f"  паз-дуга R3.75 центр ({TOMM(c.x):.2f};{TOMM(c.y):.2f})")

# компоненты
fps = list(b.GetFootprints())
print(f"футпринтов: {len(fps)}")
u7 = [f for f in fps if f.GetReference() == "U7"][0]
pos = u7.GetPosition()
print(f"U7: ({TOMM(pos.x):.3f};{TOMM(pos.y):.3f}) rot={u7.GetOrientationDegrees():.0f} падов={u7.GetPadCount()}")

# разъёмы
for ref in ("J1","J2","J3","J4","J5","J6","J7","J8"):
    f = [x for x in fps if x.GetReference() == ref][0]
    pp = f.GetPosition()
    print(f"  {ref}: ({TOMM(pp.x):.3f};{TOMM(pp.y):.3f}) rot={f.GetOrientationDegrees():.0f}")

# медь
tracks = list(b.GetTracks())
zones = [z for z in b.Zones()] if hasattr(b, "Zones") else []
print(f"медь: tracks={len(tracks)} (vias={sum(1 for t in tracks if t.Type()==pcbnew.PCB_VIA_T)})")

# пады/цепи
pads = sum(f.GetPadCount() for f in fps)
nets = {f.GetReference(): set() for f in fps}
allnets = set()
for f in fps:
    for p_ in f.Pads():
        n = p_.GetNetname()
        if n and not n.startswith("unconnected") and n != "":
            allnets.add(n)
print(f"падов: {pads}, имён цепей (непустых): {len(allnets)}")
