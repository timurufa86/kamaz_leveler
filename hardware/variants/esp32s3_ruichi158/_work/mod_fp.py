import sys
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
b = pcbnew.LoadBoard(MAIN)

# --- collect references BEFORE any modification (SWIG safety)
FPS = {}
for fp in b.GetFootprints():
    FPS[fp.GetReference()] = fp
print('footprints collected:', len(FPS))



def reshape(ref, lib, half, sx, sy):
    fp = FPS[ref]
    pads = [p for p in fp.Pads()]
    if len(pads) != 2:
        print('%s: %d pads, skip' % (ref, len(pads)))
        return
    p1, p2 = pads
    # keep the polarity: put the new pad 1 on the same side as the old one
    r1 = p1.GetFPRelativePosition()
    r2 = p2.GetFPRelativePosition()
    s1 = 1.0 if (r1.x if abs(r1.x) > abs(r1.y) else r1.y) >= 0 else -1.0
    for p, sign in ((p1, s1), (p2, -s1)):
        p.SetFPRelativePosition(pcbnew.VECTOR2I(int(round(sign * half * 1e6)), 0))
        p.SetSize(pcbnew.VECTOR2I(int(round(sx * 1e6)), int(round(sy * 1e6))))
        p.SetShape(pcbnew.PAD_SHAPE_ROUNDRECT)
        p.SetRoundRectRadiusRatio(0.25)
        p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        p.SetDrillSize(pcbnew.VECTOR2I(0, 0))
        ls = pcbnew.LSET()
        ls.AddLayer(pcbnew.F_Cu)
        ls.AddLayer(pcbnew.F_Paste)
        ls.AddLayer(pcbnew.F_Mask)
        p.SetLayerSet(ls)
        p.SetOrientation(fp.GetOrientation())
    fp.SetFPID(pcbnew.LIB_ID('Diode_SMD', lib))
    fp.SetPosition(fp.GetPosition())
    print('%s -> %s:%s pads +/-%.2f %.2fx%.2f' % (ref, 'Diode_SMD', lib, half, sx, sy))


reshape('D1', 'D_SMB', 2.15, 2.5, 2.3)
for ref in ('D5', 'D6', 'D7', 'D8', 'D9', 'D10'):
    reshape(ref, 'D_SOD-123F', 1.4, 1.1, 1.1)

if 'F1' in FPS:
    b.Remove(FPS['F1'])
    print('F1 removed')
b.Save(MAIN)
print('saved', MAIN)
