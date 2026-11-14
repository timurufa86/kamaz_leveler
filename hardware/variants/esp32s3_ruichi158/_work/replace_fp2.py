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
FPS = {fp.GetReference(): fp for fp in b.GetFootprints()}
print('collected', len(FPS))

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
        if p.GetNumber() in netmap and netmap[p.GetNumber()]:
            p.SetNetCode(netmap[p.GetNumber()])
            got += 1
    b.Add(newfp)
    print('%s -> %s (%s) nets %d/%d' % (ref, libname, 'flipped' if flipped else 'top', got, len(netmap)))
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
print('removing:', [(fp.GetReference(), str(fp.GetFPID().GetLibNickname())+':'+str(fp.GetFPID().GetLibItemName())) for fp in kill])
for fp in kill:
    b.Remove(fp)
b.Save(MAIN)
os.remove(TMP)
print('saved', MAIN)
