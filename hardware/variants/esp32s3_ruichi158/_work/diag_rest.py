import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb")

print("=== U7 pad 23 ===")
u7 = [f for f in b.GetFootprints() if f.GetReference() == "U7"][0]
for p in u7.Pads():
    if p.GetNumber() == "23":
        pos = p.GetPosition(); sz = p.GetSize()
        print(f"  pad23: ({TOMM(pos.x):.3f};{TOMM(pos.y):.3f}) size {TOMM(sz.x):.2f}x{TOMM(sz.y):.2f} layers {p.GetLayerSet().FmtHex()} net {p.GetNetname()}")

print("\n=== nets /BUT3 /BUT4 ===")
for net in ("/BUT3", "/BUT4"):
    print(f"--- {net} ---")
    for t in b.GetTracks():
        if t.GetNetname() == net and t.Type() == pcbnew.PCB_TRACE_T:
            s, e = t.GetStart(), t.GetEnd()
            lay = "F" if t.GetLayer() == pcbnew.F_Cu else "B"
            print(f"  TRACK {lay} ({TOMM(s.x):.3f};{TOMM(s.y):.3f}) -> ({TOMM(e.x):.3f};{TOMM(e.y):.3f}) w={TOMM(t.GetWidth()):.3f}")
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net:
                pos = p.GetPosition()
                print(f"  PAD {f.GetReference()}.{p.GetNumber()} ({TOMM(pos.x):.3f};{TOMM(pos.y):.3f})")

print("\n=== zones ===")
for z in b.Zones():
    lay = "F" if z.GetLayer() == pcbnew.F_Cu else "B"
    bb = z.GetBoundingBox()
    print(f"  {z.GetZoneName():<12} {z.GetNetname():<6} {lay} bbox ({TOMM(bb.GetLeft()):.1f};{TOMM(bb.GetTop()):.1f})-({TOMM(bb.GetRight()):.1f};{TOMM(bb.GetBottom()):.1f})")
    # отдельные полигоны
    polys = z.GetFilledPolysList()
    print(f"      filled polys: {polys.OutlineCount()}")

print("\n=== +12V pads ===")
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNetname() == "+12V":
            pos = p.GetPosition()
            print(f"  {f.GetReference()}.{p.GetNumber()} ({TOMM(pos.x):.3f};{TOMM(pos.y):.3f}) {('F' if p.GetLayerSet().Contains(pcbnew.F_Cu) else '')}{('B' if p.GetLayerSet().Contains(pcbnew.B_Cu) else '')}")
