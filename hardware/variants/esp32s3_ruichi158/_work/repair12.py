import io
import sys
import json
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
from rr import Model, NX, NY, CELL, ix, iy
from go import route_net, width_of

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
SRC = os.path.join(ROOT, 'routed.kicad_pcb')
GEO = os.path.join(WORK, 'routed_geo.json')
TARGETS = sys.argv[1].split(',') if len(sys.argv) > 1 else ['I2C_SDA', '+12V']

t0 = time.time()
m = Model(SRC)
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
if os.path.exists(GEO):
    g = json.load(open(GEO, encoding='utf-8'))
    for n, ss in g['segs'].items():
        segs_of[n] = [(float(s[0]), float(s[1]), float(s[2]), float(s[3]),
                       float(s[4]), int(float(s[5]))) for s in ss]
        vias_of[n] = [tuple(float(x) for x in v) for v in g['vias'].get(n, [])]
    print('merged geometry for %d nets' % len(g['segs']))
rorder = sorted(segs_of.keys(), key=lambda n: -len(segs_of[n]))
wused = {n: round(2 * segs_of[n][0][4], 3) for n in rorder if segs_of[n]}

BOX = (-24.5, -13.5, 1.5, 0.5)
ANCHOR = (-3.5, -6.0)
_allp = m.netpads('+12V')
_in = [p for p in _allp if BOX[0] <= p['x'] <= BOX[2] and BOX[1] <= p['y'] <= BOX[3]]
_out = [p for p in _allp if p not in _in]
_fake = dict(x=ANCHOR[0], y=ANCHOR[1], hx=0.35, hy=0.35, r=0.35, c=1.0, s=0.0,
             net='+12V', ref='ZZ12', num='1', onF=False, onB=True, drill=0.0)
_subset = _out + [_fake]
_orig_np = m.netpads
def _np(nm):
    if nm == '+12V':
        return _subset
    return _orig_np(nm)
m.netpads = _np
print('+12V subset: %d outside + anchor (pour covers %d)' % (len(_out), len(_in)), flush=True)

netnames = set(p['net'] for p in m.pads)
TARGETS = [(t if t in netnames else '/' + t) for t in TARGETS]
TARGETS = [t for t in TARGETS if t in netnames]
print('targets:', TARGETS)


def allsegs(exclude):
    lst = []
    for n in rorder:
        if n == exclude:
            continue
        for s in segs_of[n]:
            lst.append((n, s[0], s[1], s[2], s[3], s[4], int(s[5])))
    return lst


def conflicts(segs, net):
    """foreign nets whose copper violates clearance with these segments"""
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
            dmin = 9e9
            for (px, py) in P:
                d = _dseg(px, py, a, bb, c, e)
                if d < dmin:
                    dmin = d
                    if dmin < lim:
                        break
            if dmin < lim:
                out.add(sn)
        for q in m.pads:
            if q['net'] == net:
                continue
            if not (q['onF'] if L == 0 else q['onB']):
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


def _dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


log = []
still = []
for f in TARGETS:
    fp = m.netpads(f)
    try:
        w0 = width_of(f)
    except Exception:
        w0 = 0.2
    done = False
    def prox(n):
        best = 1e6
        for s in segs_of[n]:
            mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            for p in fp:
                d = math.hypot(mx - p['x'], my - p['y'])
                if d < best:
                    best = d
        return best
    cand = [n for n in rorder if n != f and segs_of[n]]
    cand.sort(key=prox)
    for wtry in ([w0] if w0 <= 0.3 else [w0, 0.5, 0.3]):
        C = set()
        k_idx = 0
        chunks = [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
        for it in range(len(chunks) + 4):
            m.reset_pen()
            for n in rorder:
                if n in C or n == f:
                    continue
                m.add_copper(segs_of[n], vias_of[n])
            segs_f, vias_f = route_net(m, f, wtry / 2.0, log, margin=12.0,
                                       via_cost=1.2, use_hub=False)
            if segs_f is None:
                if k_idx >= len(chunks):
                    break
                add = chunks[k_idx]
                k_idx += 1
                got = [n for n in cand if n not in C][:max(1, add)]
                if not got:
                    break
                C |= set(got)
                print('   %s it%d: route fail, rip +%d (total %d)' % (f, it, len(got), len(C)), flush=True)
                continue
            conf = conflicts(segs_f, f) - C
            if not conf:
                break
            print('   %s it%d: +%d conflicters %s' % (f, it, len(conf), sorted(conf)[:6]), flush=True)
            C |= conf
        if segs_f is None:
            continue
        m.add_copper(segs_f, vias_f)
        ok2 = True
        newgeo = {}
        for n in [x for x in rorder if x in C]:
            s2, v2 = route_net(m, n, wused.get(n, 0.15) / 2.0, log, margin=12.0, via_cost=2.0)
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
        print('%-12s SOLVED w=%.2f rip=%d segs=%d vias=%d len=%.1f (%.0fs)' %
              (f, wtry, len(C), len(segs_f), len(vias_f), L, time.time() - t0), flush=True)
        done = True
        break
    if not done:
        still.append(f)
        print('%-12s UNRESOLVED' % f, flush=True)
    json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in rorder},
               'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in rorder},
               'failed': still}, open(GEO, 'w'))
print('unresolved:', still)
print('saved', GEO, '(%.0fs)' % (time.time() - t0))
