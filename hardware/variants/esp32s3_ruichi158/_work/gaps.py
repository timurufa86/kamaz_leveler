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
log = []


def add_seg(net, x1, y1, x2, y2, w, L):
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
    s.SetWidth(int(round(w * 1e6)))
    s.SetLayer(LWR[L])
    s.SetNetCode(nmap[net])
    b.Add(s)
    m.add_copper([(x1, y1, x2, y2, w / 2e6, L)], [])


LWR = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}

# --- a. close the track->pad gaps flagged by DRC: route pad -> its own net's copper
GAPS = [('U6', '6', '/BUT3'), ('U6', '7', '/BUT4'), ('U6', '19', '/ST_U3_12'),
        ('U7', '23', 'GND')]
byref = {(p['ref'], p['num']): p for p in m.pads}
for (ref, num, net) in GAPS:
    p = byref[(ref, num)]
    others = [q for q in m.netpads(net) if q is not p]
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    ok = False
    for q in others[:3]:
        orig = m.netpads
        m.netpads = (lambda nm, _s=[p, q], _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for w in (0.25, 0.2, 0.15):
                segs, vias = route_net(m, net, w / 2.0, log, margin=12.0, via_cost=0.3, use_hub=False)
                if segs:
                    break
        finally:
            m.netpads = orig
        if segs:
            for (x1, y1, x2, y2, h, L) in segs:
                add_seg(net, x1, y1, x2, y2, 2 * h, int(L))
            for (x, y) in vias:
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
                v.SetWidth(int(round(rr.VIA_D * 1e6)))
                v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNetCode(nmap[net])
                b.Add(v)
                m.add_copper([], [(x, y)])
            print('%-4s.%-3s %-11s -> %s.%-3s %d segs %d vias' %
                  (ref, num, net, q['ref'], q['num'], len(segs), len(vias)), flush=True)
            ok = True
            break
    if not ok:
        print('%-4s.%-3s %-11s FAILED' % (ref, num, net), flush=True)

# --- b. stitch the +3V3 pours (F/B) with a via inside the pour box
for (x, y) in [(-10.0, 5.0), (5.0, 20.0)]:
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(rr.VIA_D * 1e6)))
    v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(nmap['+3V3'])
    b.Add(v)
    print('+3V3 stitch via at (%.1f,%.1f)' % (x, y), flush=True)

b.Save(MAIN)
print('saved', MAIN)
