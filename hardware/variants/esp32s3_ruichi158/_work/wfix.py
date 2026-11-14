import sys
import math
import os
import json
import time
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
OUT = os.path.join(ROOT, '_work', 'geoW.json')
BUNDLE = ['/BUT2']

# --- 1. move R19 left 2.0 mm (its /BUT5 pad sits in U6's column exhaust)


# --- 2. one pass for all six nets
t0 = time.time()
m = Model(MAIN)
m.build()
b = m.b
LAY = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
segs_of, vias_of = {}, {}
for t in b.GetTracks():
    nm = t.GetNetname()
    segs_of.setdefault(nm, [])
    vias_of.setdefault(nm, [])
    if t.Type() == pcbnew.PCB_VIA_T:
        q = t.GetPosition()
        vias_of[nm].append((q.x / 1e6, q.y / 1e6))
    else:
        s0, e0 = t.GetStart(), t.GetEnd()
        segs_of[nm].append((s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6,
                            t.GetWidth() / 2e6, LAY[t.GetLayer()]))
rorder = sorted(segs_of.keys(), key=lambda n: -len(segs_of[n]))
log = []


def rebuild(skip):
    m.reset_pen()
    for n in rorder:
        if n in skip:
            continue
        m.add_copper(segs_of[n], vias_of[n])


# free the column: rip up to 2 nets nearest to U6.6 that are not in the bundle
p6 = [q for q in m.pads if q['ref'] == 'U6' and q['num'] == '6'][0]


def prox(n):
    best = 1e6
    for s in segs_of.get(n, []):
        mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
        d = math.hypot(mx - p6['x'], my - p6['y'])
        if d < best:
            best = d
    return best


cand = sorted([n for n in rorder if n not in BUNDLE and segs_of.get(n)], key=prox)
rips = []
print('rip:', rips, flush=True)

ok_nets = []
for net in BUNDLE:
    got = None
    for w in (0.15, 0.2):
        rebuild(set(rips) | {net})
        segs, vias = route_net(m, net, w / 2.0, log, margin=16.0, via_cost=0.6, use_hub=False)
        if segs:
            got = (w, segs, vias)
            break
    if got:
        w, segs, vias = got
        segs_of[net] = segs
        vias_of[net] = vias
        m.add_copper(segs, vias)
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        pads = [q for q in m.netpads(net)]
        print('%-11s w%.2f segs %2d vias %2d len %6.2f' % (net, w, len(segs), len(vias), L), flush=True)
        ok_nets.append(net)
    else:
        print('%-11s UNRESOLVED' % net, flush=True)

for n in rips:
    w = max(0.15, min(width_of(n), 0.4))
    rebuild({n})
    segs, vias = route_net(m, n, w / 2.0, log, margin=14.0, via_cost=1.2)
    if segs:
        segs_of[n] = segs
        vias_of[n] = vias
        m.add_copper(segs, vias)
        print('re-route %-10s w%.2f segs %2d vias %2d' % (n, w, len(segs), len(vias)), flush=True)
    else:
        print('re-route %-10s FAILED' % n, flush=True)

json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in rorder},
           'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in rorder}},
          open(OUT, 'w'))
print('dumped', OUT, '(%.0fs)' % (time.time() - t0))
