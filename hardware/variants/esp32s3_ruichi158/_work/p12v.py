import io
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
from go import route_net

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'routed.kicad_pcb')
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'routed12.kicad_pcb')
BOX = (-24.5, -13.5, 1.5, 0.5)          # +12V pour box (B.Cu)
ANCHOR = (-3.5, -6.0)

m = Model(SRC)
m.build()
allp = m.netpads('+12V')
inside = [p for p in allp if BOX[0] <= p['x'] <= BOX[2] and BOX[1] <= p['y'] <= BOX[3]]
outside = [p for p in allp if p not in inside]
print('+12V pads: %d inside pour box, %d outside' % (len(inside), len(outside)))
fake = dict(x=ANCHOR[0], y=ANCHOR[1], hx=0.35, hy=0.35, r=0.35, c=1.0, s=0.0,
            net='+12V', ref='ZZ12', num='1', onF=False, onB=True, drill=0.0)
subset = outside + [fake]
_orig = m.netpads


def netpads(nm):
    if nm == '+12V':
        return subset
    return _orig(nm)


m.netpads = netpads
log = []
segs, vias = None, None
for w in (0.6, 0.5, 0.4):
    segs, vias = route_net(m, '+12V', w / 2.0, log, margin=12.0, via_cost=1.0, use_hub=False)
    if segs:
        print('routed +12V at w=%.2f: %d segs, %d vias, len %.1f' %
              (w, len(segs), len(vias), sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)))
        break
if not segs:
    print('FAILED:', log[-6:])
    sys.exit(3)
b = m.b
nc = None
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetNetname() == '+12V':
            nc = p.GetNetCode()
            break
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
for (x1, y1, x2, y2, h, L) in segs:
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
    s.SetWidth(int(round(2 * h * 1e6)))
    s.SetLayer(LAY[int(L)])
    s.SetNetCode(nc)
    b.Add(s)
for (x, y) in vias:
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(rr.VIA_D * 1e6)))
    v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(nc)
    b.Add(v)
b.Save(DST)
print('saved', DST)
