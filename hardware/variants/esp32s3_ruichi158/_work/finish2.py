import io
import sys
import json
import math
import os
import time
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
sys.path.insert(0, WORK)
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass
from rr import Model, NX, NY, CELL, ix, iy
from go import route_net, width_of

SRC = os.path.join(ROOT, 'routed.kicad_pcb')
GEO = os.path.join(WORK, 'routed_geo.json')

t0 = time.time()
m = Model(SRC)
m.build()
b = m.b
LAY = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
segs_of = {}
vias_of = {}
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
if os.path.exists(GEO):
    g = json.load(open(GEO, encoding='utf-8'))
    for n, ss in g['segs'].items():
        segs_of[n] = [(float(s[0]), float(s[1]), float(s[2]), float(s[3]),
                       float(s[4]), int(float(s[5]))) for s in ss]
        vias_of[n] = [tuple(float(x) for x in v) for v in g['vias'].get(n, [])]
    print('merged saved geometry from routed_geo.json (%d nets)' % len(g['segs']))
rorder = sorted(segs_of.keys(), key=lambda n: -len(segs_of[n]))
wused = {n: round(2 * segs_of[n][0][4], 3) for n in rorder if segs_of[n]}

FAILED = sys.argv[1].split(',') if len(sys.argv) > 1 else ['ST_U4_34', 'I2C_SDA', '+12V']
netnames = set(p['net'] for p in m.pads)
FAILED = [(f if f in netnames else '/' + f) for f in FAILED]
FAILED = [f for f in FAILED if f in netnames]
print('targets:', FAILED)

log = []
still = []
for f in FAILED:
    fp = m.netpads(f)
    fx0 = min(p['x'] for p in fp) - 4.0
    fx1 = max(p['x'] for p in fp) + 4.0
    fy0 = min(p['y'] for p in fp) - 4.0
    fy1 = max(p['y'] for p in fp) + 4.0

    def prox(n):
        ss = segs_of[n]
        if not ss:
            return 1e6
        best = 1e6
        for s in ss:
            mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            for p in fp:
                d = math.hypot(mx - p['x'], my - p['y'])
                if d < best:
                    best = d
        return best

    cand = []
    for n in rorder:
        if n == f:
            continue
        for (x1, y1, x2, y2, h, L) in segs_of[n]:
            if (max(x1, x2) + h > fx0 and min(x1, x2) - h < fx1 and
                    max(y1, y2) + h > fy0 and min(y1, y2) - h < fy1):
                cand.append(n)
                break
    cand.sort(key=prox)
    done = False
    ks = [0, 1, 2, 3, 5, 8, 12, 20, 35, 60, len(cand)]
    for k in ks:
        rip = set(cand[:k])
        for wtry in (min(width_of(f), 0.3), 0.2, 0.15):
            m.reset_pen()
            for n in rorder:
                if n in rip:
                    continue
                m.add_copper(segs_of[n], vias_of[n])
            segs_f, vias_f = route_net(m, f, wtry / 2.0, log, margin=10.0,
                                       via_cost=1.2, use_hub=False)
            if segs_f is None:
                continue
            m.add_copper(segs_f, vias_f)
            ok2 = True
            newgeo = {}
            for n in [x for x in rorder if x in rip]:
                s2, v2 = route_net(m, n, wused.get(n, 0.15) / 2.0, log, margin=10.0, via_cost=2.0)
                if s2 is None:
                    ok2 = False
                    break
                m.add_copper(s2, v2)
                newgeo[n] = (s2, v2)
            if not ok2:
                continue
            for n, gp in newgeo.items():
                segs_of[n] = gp[0]
                vias_of[n] = gp[1]
            segs_of[f] = segs_f
            vias_of[f] = vias_f
            if f not in rorder:
                rorder.append(f)
            L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs_f)
            print('%-12s SOLVED k=%d w=%.2f dsegs %d vias %d len %.1f (%.0fs)' %
                  (f, k, wtry, len(segs_f), len(vias_f), L, time.time() - t0), flush=True)
            done = True
            break
        if done:
            break
    if not done:
        still.append(f)
        print('%-12s UNRESOLVED' % f, flush=True)
    json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in rorder},
               'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in rorder},
               'failed': still},
              open(GEO, 'w'))
for a in log[-15:]:
    print('   ', a)
print('unresolved:', still)
print('saved', GEO, '(%.0fs)' % (time.time() - t0))
