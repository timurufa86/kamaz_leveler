import sys
import math
import os
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass
from rr import Model
from go import route_net, width_of

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
ANT = (-13.0, 34.0, 13.0, 42.75)
NETS = ['/BUT1', '/BUT2', '/BUT5', '/ST_U3_12', '+5V']

# --- 1. move C21 left by 2.5 mm (clears U6's left escape corridor)
b = pcbnew.LoadBoard(MAIN)
c21 = [f for f in b.GetFootprints() if f.GetReference() == 'C21'][0]
p0 = c21.GetPosition()
c21.SetPosition(pcbnew.VECTOR2I(p0.x - int(2.5e6), p0.y))
print('MOVE C21 (%.3f,%.3f) -> (%.3f,%.3f)' % (p0.x / 1e6, p0.y / 1e6,
                                               c21.GetPosition().x / 1e6, c21.GetPosition().y / 1e6))
# --- 2. drop target nets' copper and the +3V3 tracks inside the antenna strip
n = 0
nz = 0
for t in list(b.GetTracks()):
    nm = t.GetNetname()
    if nm in NETS:
        b.Remove(t)
        n += 1
        continue
    if t.Type() == pcbnew.PCB_VIA_T or nm != '+3V3':
        continue
    x1, y1 = t.GetStart().x / 1e6, t.GetStart().y / 1e6
    x2, y2 = t.GetEnd().x / 1e6, t.GetEnd().y / 1e6
    h = t.GetWidth() / 2e6
    if (max(x1, x2) + h > ANT[0] and min(x1, x2) - h < ANT[2] and
            max(y1, y2) + h > ANT[1] and min(y1, y2) - h < ANT[3]):
        b.Remove(t)
        nz += 1
b.Save(MAIN)
print('+3V3 strip tracks dropped:', nz)
print('ripped tracks:', n)
