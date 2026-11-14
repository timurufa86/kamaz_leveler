import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

A = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
B = r"C:\my\for cursor\kamaz_leveler\hardware\kicad\kamaz_leveler_hw.kicad_pcb"


def load(p):
    b = pcbnew.LoadBoard(p)
    pos = {}
    for f in b.GetFootprints():
        p_ = f.GetPosition()
        pos[f.GetReference()] = (round(TOMM(p_.x), 3), round(TOMM(p_.y), 3), f.GetFPID().GetLibItemName())
    npth = []
    for f in b.GetFootprints():
        for pad in f.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                np = pad.GetPosition()
                npth.append((f.GetReference(), round(TOMM(np.x), 2), round(TOMM(np.y), 2),
                             round(TOMM(pad.GetDrillSize().x), 2)))
    return b, pos, npth


ba, pa, na = load(A)
bb, pb, nb = load(B)
print("A refs:", len(pa), "BASE refs:", len(pb))
print("only in A:", sorted(set(pa) - set(pb)))
print("only in BASE:", sorted(set(pb) - set(pa)))
moved = []
for r in sorted(set(pa) & set(pb)):
    if pa[r][:2] != pb[r][:2]:
        d = ((pa[r][0] - pb[r][0]) ** 2 + (pa[r][1] - pb[r][1]) ** 2) ** 0.5
        moved.append((r, pb[r][:2], pa[r][:2], round(d, 3)))
print("moved/common:", len(moved))
for m in moved[:15]:
    print("  ", m)
print("A  NPTH:", len(na), sorted(na, key=lambda t: t[0]))
print("BASE NPTH:", len(nb), sorted(nb, key=lambda t: t[0]))
# pad-level diff on shared refs: pad count and sizes
da = {r: [len(f.Pads()) for f in ba.GetFootprints() if f.GetReference() == r][0] for r in pa}
db = {r: [len(f.Pads()) for f in bb.GetFootprints() if f.GetReference() == r][0] for r in pb}
diffpads = [(r, db[r], da[r]) for r in sorted(set(pa) & set(pb)) if da[r] != db[r]]
print("pads differ:", diffpads)
