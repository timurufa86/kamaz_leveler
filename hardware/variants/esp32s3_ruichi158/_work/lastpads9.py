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
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')

# 1. create the net the schematic expects for the freed U6 pin 11
b = pcbnew.LoadBoard(MAIN)
u6 = [f for f in b.GetFootprints() if f.GetReference() == 'U6'][0]
p11 = [p for p in u6.Pads() if p.GetNumber() == '11'][0]
NAME = 'unconnected-(U6-IO0_7-Pad11)'
if not p11.GetNetname():
    print('net U6.11 already handled')
# 2. pair-connect the pads still unconnected
TARGETS = [('U4','1'),('U6','22'),('D16','1'),('U5','3'),('U6','16'),('U6','17'),('U3','4'),('R12','1')]
m = Model(MAIN)
m.build()
b = m.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
byref = {(p['ref'], p['num']): p for p in m.pads}
log = []
fixed = 0
for (ref, num) in TARGETS:
    p = byref.get((ref, num))
    if p is None:
        print('no pad', ref, num)
        continue
    net = p['net']
    others = [q for q in m.netpads(net) if q is not p]
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    ok = False
    for q in others[:4]:
        orig = m.netpads
        m.netpads = (lambda nm, _s=[p, q], _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for w in (0.25, 0.2, 0.15):
                segs, vias = route_net(m, net, w / 2.0, log, margin=20.0, via_cost=0.4, use_hub=False)
                if segs:
                    break
        finally:
            m.netpads = orig
        if segs:
            m.add_copper(segs, vias)
            nc = nmap[net]
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
            print('%-4s.%-3s %-11s -> %s.%-3s : %d segs %d vias' %
                  (ref, num, net, q['ref'], q['num'], len(segs), len(vias)), flush=True)
            fixed += 1
            ok = True
            break
    if not ok:
        print('%-4s.%-3s %-11s FAILED' % (ref, num, net), flush=True)
print('fixed', fixed)
b.Save(MAIN)
print('saved', MAIN)
