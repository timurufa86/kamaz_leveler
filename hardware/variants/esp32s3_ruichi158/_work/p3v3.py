import io
import sys
import json
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
from rr import Model, NX, NY, CELL, ix, iy
from go import access_cells, trim_to_pad, sd_pad

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = os.path.join(ROOT, 'routed2.kicad_pcb')
DST = os.path.join(ROOT, 'routed3.kicad_pcb')
RECT = (-27.0, 1.5, -12.5, 27.5)
HW = 0.2

m = Model(SRC)
m.build()
pads = m.netpads('+3V3')
inside = [p for p in pads if RECT[0] <= p['x'] <= RECT[2] and RECT[1] <= p['y'] <= RECT[3]]
outside = [p for p in pads if p not in inside]
print('+3V3 pads: %d inside rect, %d outside' % (len(inside), len(outside)))
for p in outside:
    print('   outside %s.%s (%.2f,%.2f)' % (p['ref'], p['num'], p['x'], p['y']))

pens = m.pen_for_net('+3V3')
blkF, blkB, blkV = m.pen_masks(pens, HW)
blk = [blkF, blkB]
# seeds: access cells of every inside pad (they are joined by the +3V3 pour)
seeds = []
for p in inside:
    Ls = [0, 1] if (p['onF'] and p['onB']) else ([0] if p['onF'] else [1])
    for L in Ls:
        for (x, y) in access_cells(m, p, HW, L, blk[L]):
            seeds.append((x, y, L, 0.35))
print('seeds', len(seeds))

segs = []
vias = []
log = []
for p in outside:
    Ls = [0, 1] if (p['onF'] and p['onB']) else ([0] if p['onF'] else [1])
    goals = []
    for L in Ls:
        for (x, y) in access_cells(m, p, HW, L, blk[L]):
            goals.append((L, ix(x), iy(y)))
    if not goals:
        print('no access', p['ref'], p['num'])
        continue
    pts = m.astar(seeds, goals, (p['x'], p['y']), HW, pens, margin=8.0, via_cost=1.5)
    if pts is None:
        pts = m.astar(seeds, goals, (p['x'], p['y']), HW, pens, margin=90.0, via_cost=1.5, maxexp=900000)
    if pts is None:
        print('NO ROUTE', p['ref'], p['num'])
        continue
    runs = []
    cur = []
    for (x, y, L, via) in pts:
        if cur and cur[-1][2] != L:
            runs.append(cur)
            cur = []
        cur.append((x, y, L))
    if cur:
        runs.append(cur)
    runs[-1].append((p['x'], p['y'], runs[-1][-1][2]))
    ns = []
    for k, run in enumerate(runs):
        pl = rr.collinear_merge([(q[0], q[1]) for q in run])
        pl = rr.smooth(pl, m, HW, run[0][2], '+3V3')
        if k == len(runs) - 1 and len(pl) > 1:
            pl[-1] = trim_to_pad(p, pl[-1][0], pl[-1][1], pl[-2][0], pl[-2][1])
        for t in range(len(pl) - 1):
            (ax, ay), (bx, by) = pl[t], pl[t + 1]
            if math.hypot(bx - ax, by - ay) < 1e-4:
                continue
            ns.append((ax, ay, bx, by, HW, run[0][2]))
        if k + 1 < len(runs):
            vias.append((run[-1][0], run[-1][1]))
    # commit into the model so the next pad sees it
    m.add_copper(ns, vias[-1:] if False else [])
    segs += ns
    L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in ns)
    print('   %s.%s -> %d segs %.1f mm' % (p['ref'], p['num'], len(ns), L))

print('total +3V3 stubs: %d segs, %d vias' % (len(segs), len(vias)))
b = m.b
nc = None
for fp in b.GetFootprints():
    for q in fp.Pads():
        if q.GetNetname() == '+3V3':
            nc = q.GetNetCode()
            break
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
for (x1, y1, x2, y2, h, L) in segs:
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
    s.SetWidth(int(round(2 * h * 1e6)))
    s.SetLayer(LAY[L])
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
