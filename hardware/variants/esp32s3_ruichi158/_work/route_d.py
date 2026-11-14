import sys
import math
import os
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
SRC = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
DST = os.path.join(ROOT, '_work', 'geoD.json')
ORDER = ['+24V', '/VALVE_BUB1', '/VALVE_BUB2', '/VALVE_BUB3', '/VALVE_BUB4', '/VALVE_DEFL', '/VALVE_INFL', '+12V']

t0 = time.time()
m = Model(SRC)
m.build()
b = m.b
LAY = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
LWR = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
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
netnames = set(p['net'] for p in m.pads)
log = []


def allsegs(exclude):
    out = []
    for n in rorder:
        if n == exclude:
            continue
        for s in segs_of[n]:
            out.append((n, s[0], s[1], s[2], s[3], s[4], int(s[5])))
    return out


def _dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def conflicts(segs, net):
    out = set()
    lst = allsegs(net)
    for (x1, y1, x2, y2, h, L) in segs:
        L = int(L)
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
        P = [(x1 + (x2 - x1) * t / n, y1 + (y2 - y1) * t / n) for t in range(n + 1)]
        mx0, mx1 = min(x1, x2) - h - 1.0, max(x1, x2) + h + 1.0
        my0, my1 = min(y1, y2) - h - 1.0, max(y1, y2) + h + 1.0
        for (sn, a, bb, c, e, hh, LL) in lst:
            if LL != L:
                continue
            if max(a, c) + hh < mx0 or min(a, c) - hh > mx1 or max(bb, e) + hh < my0 or min(bb, e) - hh > my1:
                continue
            lim = hh + h + rr.CLR - 0.01
            if min(_dseg(px, py, a, bb, c, e) for (px, py) in P) < lim:
                out.add(sn)
        for q in m.pads:
            if q['net'] == net or not (q['onF'] if L == 0 else q['onB']):
                continue
            if q['x'] + q['hx'] + h + rr.CLR < mx0 or q['x'] - q['hx'] - h - rr.CLR > mx1:
                continue
            if q['y'] + q['hy'] + h + rr.CLR < my0 or q['y'] - q['hy'] - h - rr.CLR > my1:
                continue
            for (px, py) in P:
                u = q['c'] * (px - q['x']) + q['s'] * (py - q['y'])
                v = -q['s'] * (px - q['x']) + q['c'] * (py - q['y'])
                qx = abs(u) - (q['hx'] - q['r'])
                qy = abs(v) - (q['hy'] - q['r'])
                sd = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - q['r']
                if sd < rr.CLR + h - 0.01:
                    if q['net'] != '__none__':
                        out.add(q['net'])
                    break
    return out


solved, failed = [], []
for net in ORDER:
    if net not in netnames:
        print('skip %s (no such net)' % net)
        continue
    fp = m.netpads(net)
    if len(fp) < 2:
        continue
    w0 = width_of(net)

    def prox(n):
        best = 1e6
        for s in segs_of.get(n, []):
            mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            for p in fp:
                d = math.hypot(mx - p['x'], my - p['y'])
                if d < best:
                    best = d
        return best
    cand = sorted([n for n in rorder if n != net and segs_of.get(n)], key=prox)
    got = None
    for wtry in ([w0] if w0 <= 0.3 else [w0, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15]):
        C = set()
        k_idx = 0
        chunks = [0, 1, 2, 3, 5, 8, 13]
        segs = None
        for it in range(len(chunks) + 3):
            m.reset_pen()
            for n in rorder:
                if n in C or n == net:
                    continue
                m.add_copper(segs_of.get(n, []), vias_of.get(n, []))
            segs, vias = route_net(m, net, wtry / 2.0, log, margin=14.0, via_cost=0.6, use_hub=False)
            if not segs:
                if k_idx >= len(chunks):
                    break
                add = chunks[k_idx]
                k_idx += 1
                take = [n for n in cand if n not in C][:max(1, add)]
                if not take:
                    break
                C |= set(take)
                continue
            conf = conflicts(segs, net) - C
            if not conf:
                break
            C |= conf
        if not segs:
            continue
        m.add_copper(segs, vias)
        newgeo = {}
        ok2 = True
        for n in [x for x in rorder if x in C]:
            s2, v2 = route_net(m, n, max(0.15, min(width_of(n), 0.3)) / 2.0, log,
                               margin=14.0, via_cost=1.2)
            if not s2:
                ok2 = False
                break
            m.add_copper(s2, v2)
            newgeo[n] = (s2, v2)
        if not ok2:
            continue
        for n, gp in newgeo.items():
            segs_of[n] = gp[0]
            vias_of[n] = gp[1]
        got = (wtry, segs, vias, len(C))
        break
    if got:
        wtry, segs, vias, nc = got
        segs_of[net] = segs
        vias_of[net] = vias
        if net not in rorder:
            rorder.append(net)
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        print('%-12s w%.2f segs %2d vias %2d len %6.2f  rip %d  (%.0fs)' %
              (net, wtry, len(segs), len(vias), L, nc, time.time() - t0), flush=True)
        solved.append(net)
    else:
        print('%-12s UNRESOLVED' % net, flush=True)
        failed.append(net)
print('solved %d / failed %d' % (len(solved), len(failed)))

import json
json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in rorder},
           'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in rorder}},
          open(DST, 'w'))
print('geometry dumped to', DST)
