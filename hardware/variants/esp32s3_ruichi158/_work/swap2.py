import sys
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
TMP = os.path.join(ROOT, '_work', '_tmp_fp.kicad_pcb')
LIB = r'C:\Program Files\KiCad\10.0\share\kicad\footprints\Diode_SMD.pretty'
OLD = {'D1': 'kamaz_leveler:D_DO-201AD', 'D5': 'Diode_SMD:D_SMA', 'D6': 'Diode_SMD:D_SMA',
       'D7': 'Diode_SMD:D_SMA', 'D8': 'Diode_SMD:D_SMA', 'D9': 'Diode_SMD:D_SMA',
       'D10': 'Diode_SMD:D_SMA'}

b = pcbnew.LoadBoard(MAIN)

# --- 1. rename VIN_FUSED -> +24V_IN so the board matches the merged schematic net
ni = b.GetNetInfo()
src = None
for i in range(ni.GetNetCount()):
    n = ni.GetNetItem(i)
    if n and n.GetNetname() == 'VIN_FUSED':
        src = n
print('VIN_FUSED found:', src is not None)
if src is not None:
    src.SetNetname('+24V_IN')
    print('renamed VIN_FUSED -> +24V_IN')

FPS = {fp.GetReference(): fp for fp in b.GetFootprints()}
print('footprints', len(FPS))

for ref in ('D1', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10'):
    old = FPS[ref]
    pos, ori, flipped = old.GetPosition(), old.GetOrientation(), old.IsFlipped()
    ref_txt, val = old.GetReference(), old.GetValue()
    netmap = {p.GetNumber(): p.GetNetCode() for p in old.Pads()}
    libname = 'D_SMB' if ref == 'D1' else 'D_SOD-123F'
    newfp = pcbnew.FootprintLoad(LIB, libname)
    newfp.SetPosition(pos)
    newfp.SetOrientation(ori)
    if flipped:
        newfp.Flip(pos, False)
    newfp.SetReference(ref_txt)
    newfp.SetValue(val)
    newfp.SetFPID(pcbnew.LIB_ID('Diode_SMD', libname))
    got = 0
    for p in newfp.Pads():
        nc = netmap.get(p.GetNumber())
        if nc:
            p.SetNetCode(nc)
            got += 1
    b.Add(newfp)
    print('%s -> %s nets %d/%d (pad1=%s)' % (ref, libname, got, len(netmap), val))
b.Save(TMP)

b = pcbnew.LoadBoard(TMP)
kill = []
for fp in b.GetFootprints():
    r = fp.GetReference()
    li = fp.GetFPID()
    f = str(li.GetLibNickname()) + ':' + str(li.GetLibItemName())
    if r in OLD and f.startswith(OLD[r]):
        kill.append(fp)
    elif r == 'F1':
        kill.append(fp)
print('removing:', [fp.GetReference() for fp in kill])
for fp in kill:
    b.Remove(fp)
b.Save(MAIN)
os.remove(TMP)
print('saved', MAIN, 'footprints now', len(list(b.GetFootprints())))
