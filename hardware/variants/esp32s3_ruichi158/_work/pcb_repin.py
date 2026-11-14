import sys
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
DST = os.path.join(ROOT, '_work', '_repin.kicad_pcb')

EXP = {4: '/BUT1', 5: '/BUT2', 6: '/BUT3', 7: '/BUT4', 8: '/BUT5',
       9: '/ST_U4_12', 10: '/ST_U4_34', 11: None,
       13: '/VLV_IN1', 14: '/VLV_IN2', 15: '/VLV_IN3', 16: '/VLV_IN4',
       17: '/VLV_IN5', 18: '/VLV_IN6', 19: '/ST_U3_12', 20: '/ST_U3_34'}
STALE = sorted(set(v for v in EXP.values() if v))

b = pcbnew.LoadBoard(SRC)
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
u6 = [f for f in b.GetFootprints() if f.GetReference() == 'U6'][0]
print('U6 pads:', u6.GetPadCount())
for p in u6.Pads():
    n = int(p.GetNumber())
    if n not in EXP:
        continue
    nm = EXP[n]
    old = p.GetNetname()
    code = nmap[nm] if nm else 0
    p.SetNetCode(code)
    print('  pad %-2d %-12s -> %s' % (n, old, nm if nm else '<no net>'))
n = 0
for t in list(b.GetTracks()):
    if t.GetNetname() in STALE:
        b.Remove(t)
        n += 1
print('ripped stale tracks:', n, 'for nets', STALE)
b.Save(DST)
print('saved', DST)
