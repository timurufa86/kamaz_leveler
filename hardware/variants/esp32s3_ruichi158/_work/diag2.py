import io, sys, math
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb")

print("=== U7 все пады ===")
u7 = [f for f in b.GetFootprints() if f.GetReference() == "U7"][0]
for p in sorted(u7.Pads(), key=lambda p: int(p.GetNumber())):
    pos = p.GetPosition()
    print(f"  pad {p.GetNumber():>2}: ({TOMM(pos.x):.2f};{TOMM(pos.y):.2f}) {p.GetNetname()}")

print("\n=== зоны: полигоны ===")
for z in b.Zones():
    lay = pcbnew.F_Cu if z.GetLayer() == pcbnew.F_Cu else pcbnew.B_Cu
    pl = z.GetFilledPolysList(lay)
    name = z.GetZoneName()
    print(f"  {name:<12} {z.GetNetname():<6} {'F' if lay==pcbnew.F_Cu else 'B'} outlines={pl.OutlineCount()}")
    for i in range(pl.OutlineCount()):
        out = pl.Outline(i)
        xs = [TOMM(out.CPoint(j).x) for j in range(out.PointCount())]
        ys = [TOMM(out.CPoint(j).y) for j in range(out.PointCount())]
        if xs:
            print(f"      island {i}: x[{min(xs):.1f}..{max(xs):.1f}] y[{min(ys):.1f}..{max(ys):.1f}] pts={out.PointCount()}")

# препятствия в окрестности кандидатов
print("\n=== препятствия возле кандидатов ===")
def near(px, py, r=2.0):
    out = []
    for t in b.GetTracks():
        s, e = t.GetStart(), t.GetEnd()
        if t.Type() == pcbnew.PCB_VIA_T:
            if abs(TOMM(s.x)-px) < r and abs(TOMM(s.y)-py) < r:
                out.append(f"VIA {t.GetNetname()} ({TOMM(s.x):.2f};{TOMM(s.y):.2f})")
        else:
            # distance from segment to point, rough
            x1,y1,x2,y2 = TOMM(s.x),TOMM(s.y),TOMM(e.x),TOMM(e.y)
            dx,dy = x2-x1,y2-y1
            L2 = dx*dx+dy*dy
            tt = max(0, min(1, ((px-x1)*dx+(py-y1)*dy)/L2)) if L2>0 else 0
            qx,qy = x1+tt*dx, y1+tt*dy
            if math.hypot(px-qx, py-qy) < r:
                out.append(f"TRK {t.GetNetname()} ({x1:.2f};{y1:.2f})-({x2:.2f};{y2:.2f})")
    for f in b.GetFootprints():
        for p in f.Pads():
            pos = p.GetPosition()
            sz = p.GetSize()
            if abs(TOMM(pos.x)-px) < r and abs(TOMM(pos.y)-py) < r:
                out.append(f"PAD {f.GetReference()}.{p.GetNumber()} {p.GetNetname()} ({TOMM(pos.x):.2f};{TOMM(pos.y):.2f})")
    return out

for name, px, py in [("U7.23 колонка", 11.43, 37.5), ("BUT3 колонка x=-20.6", -20.6, 16.0),
                     ("BUT4 колонка x=-21.5", -21.5, 18.0), ("+3V3 via кандидат", -27.2, 0.5),
                     ("+12V мост", -24.5, -13.5)]:
    print(f"  {name}:")
    for o in near(px, py):
        print(f"      {o}")
