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
OUT = os.path.join(ROOT, '_work', 'geoB134.json')
BUNDLE = ['/BUT1', '/BUT3', '/BUT4']
ANTBOX = []

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


def rebuild(skip, forbid=None):
    m.reset_pen()
    for n in rorder:
        if n in skip:
            continue
        m.add_copper(segs_of[n], vias_of[n])
    m.forbid = list(forbid or [])


def try_route(net, widths, forbid=None, skip=()):
    for w in widths:
        rebuild(set(skip) | {net}, forbid)
        segs, vias = route_net(m, net, w / 2.0, log, margin=16.0, via_cost=0.7, use_hub=False)
        if segs:
            return w, segs, vias
    return None, None, None


# --- 1. free the U4/L4 zone: rip up to 2 nets whose copper sits nearest U4.5
p = [q for q in m.pads if q['ref'] == 'U6' and q['num'] == '6'][0]


def prox(n):
    best = 1e6
    for s in segs_of.get(n, []):
        mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
        d = math.hypot(mx - p['x'], my - p['y'])
        if d < best:
            best = d
    return best


cand = sorted([n for n in rorder if n not in BUNDLE and segs_of.get(n)],
              key=lambda n: prox(n))
rips = []
print('rip for U4 zone:', rips, 'dist',

      [round(prox(n), 2) for n in rips], flush=True)

# --- 2. route the bundle in one pass (each stays in the field for the next)
done = []
for net in BUNDLE:
    w, segs, vias = try_route(net, [0.15, 0.2, 0.25], skip=rips)
    if segs:
        segs_of[net] = segs
        vias_of[net] = vias
        m.add_copper(segs, vias)
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        print('%-12s w%.2f segs %2d vias %2d len %6.2f' % (net, w, len(segs), len(vias), L), flush=True)
        done.append(net)
    else:
        print('%-12s UNRESOLVED' % net, flush=True)

# --- 3. re-route the ripped nets
for n in rips:
    w, segs, vias = try_route(n, [max(0.15, min(width_of(n), 0.4))])
    if segs:
        segs_of[n] = segs
        vias_of[n] = vias
        m.add_copper(segs, vias)
        print('re-route %-10s w%.2f segs %2d vias %2d' % (n, w, len(segs), len(vias)), flush=True)
    else:
        print('re-route %-10s FAILED' % n, flush=True)

# --- 4. +3V3 with a hard forbid over the whole antenna strip
w, segs, vias = None, None, None
if segs:
    segs_of['+3V3'] = segs
    vias_of['+3V3'] = vias
    L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
    print('+3V3 w%.2f segs %2d vias %2d len %6.2f (strip forbidden)' % (w, len(segs), len(vias), L), flush=True)
else:
    print('+3V3 UNRESOLVED with strip forbidden', flush=True)

json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in rorder},
           'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in rorder}},
          open(OUT, 'w'))
print('dumped', OUT, '(%.0fs)' % (time.time() - t0))
