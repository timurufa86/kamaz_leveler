import sys
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
LIB = r'C:\Program Files\KiCad\10.0\share\kicad\footprints\Diode_SMD.pretty'
b = pcbnew.LoadBoard(MAIN)

FPS = {}
for fp in b.GetFootprints():
    FPS[fp.GetReference()] = fp
print('collected', len(FPS))


def replace(ref, libname):
    old = FPS[ref]
    pos = old.GetPosition()
    ori = old.GetOrientation()
    flipped = old.IsFlipped()
    ref_txt = old.GetReference()
    val = old.GetValue()
    netmap = {}
    for p in old.Pads():
        netmap[p.GetNumber()] = (p.GetNetCode(), p.GetNetname())
    newfp = pcbnew.FootprintLoad(LIB, libname)
    if newfp is None:
        print('%s: cannot load %s' % (ref, libname))
        return False
    newfp.SetPosition(pos)
    newfp.SetOrientation(ori)
    if flipped:
        newfp.Flip(pos, False)
    newfp.SetReference(ref_txt)
    newfp.SetValue(val)
    newfp.SetFPID(pcbnew.LIB_ID('Diode_SMD', libname))
    got = 0
    for p in newfp.Pads():
        if p.GetNumber() in netmap:
            nc = netmap[p.GetNumber()][0]
            p.SetNetCode(nc)
            got += 1
    b.Add(newfp)
    FPS[ref] = None
    print('%s -> %s (%s) nets %d/%d' % (ref, libname, 'flipped' if flipped else 'top', got, len(netmap)))
    return True


for ref in ('D1', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10'):
    replace(ref, 'D_SMB' if ref == 'D1' else 'D_SOD-123F')

# remove the old footprints after all the new ones are in
TMP = os.path.join(ROOT, '_work', '_tmp_fp.kicad_pcb')
b.Save(TMP)
b = pcbnew.LoadBoard(TMP)
kill = [fp for fp in b.GetFootprints() if fp.GetReference() in
        ('D1', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10') and fp.GetFPID().GetLibItemName() !=
        ('D_SMB' if fp.GetReference() == 'D1' else 'D_SOD-123F')]
print('old footprints to remove:', len(kill))
for fp in kill:
    b.Remove(fp)
b.Save(MAIN)
os.remove(TMP)
print('saved', MAIN)
