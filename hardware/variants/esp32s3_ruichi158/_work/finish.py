"""Finish pass: INT_MCU / SPEED_SIG / +3V3 re-routes + local rip-up for boxed pads."""
import re
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
DST = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '_work', 'fin1.kicad_pcb')
REPORT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, '_work', 'drc_final.txt')

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


def rebuild(skip=(), forbid=None):
    m.reset_pen()
    for n in rorder:
        if n in skip:
            continue
        m.add_copper(segs_of[n], vias_of[n])
    m.forbid = list(forbid or [])


def do_net(net, widths, forbid=None, rips=(), margin=10.0):
    rebuild(skip=set(rips) | {net}, forbid=forbid)
    for w in widths:
        segs, vias = route_net(m, net, w / 2.0, log, margin=margin, via_cost=1.0, use_hub=False)
        if segs:
            return w, segs, vias
    return None, None, None


def commit(net, segs, vias, append=False):
    if append:
        segs_of[net] = list(segs_of.get(net, [])) + list(segs)
        vias_of[net] = list(vias_of.get(net, [])) + list(vias)
    else:
        segs_of[net] = segs
        vias_of[net] = vias
    if net not in rorder:
        rorder.append(net)
    m.add_copper(segs, vias)


done = []
# A. /INT_MCU tight
for w in (0.15, 0.2):
    w1, segs, vias = do_net('/INT_MCU', [w])
    if segs:
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        if L <= 30.0:
            commit('/INT_MCU', segs, vias)
            done.append(('/INT_MCU', w, len(segs), len(vias), L))
            break
        print('  INT_MCU len %.2f > 30, retry' % L)

# B. /SPEED_SIG around U11.2 real copper (size rect 1.475x0.9 at 21.383,8.045)
forb = None
w2, segs, vias = do_net('/SPEED_SIG', [0.15, 0.2], forbid=forb)
if segs:
    commit('/SPEED_SIG', segs, vias)
    done.append(('/SPEED_SIG', w2, len(segs), len(vias),
                 sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)))
else:
    print('  SPEED_SIG failed')

# C. /+3V3 with the antenna corridor narrowed -> transit leaves the strip
w3, segs, vias = None, None, None
if segs:
    commit('/+3V3', segs, vias)
    done.append(('/+3V3', w3, len(segs), len(vias),
                 sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)))
else:
    print('  +3V3 failed')

for d in done:
    print('%-12s w%.2f segs %3d vias %2d len %6.2f' % d, flush=True)

# D. boxed pads: local rip-up + pair connect
txt = open(REPORT, encoding='utf-8', errors='replace').read()
todo, seen = [], set()
for m0 in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    key = (m0.group(3), m0.group(1), m0.group(2))
    if key not in seen:
        seen.add(key)
        todo.append(key)
byref = {(p['ref'], p['num']): p for p in m.pads}
fixed = 0
covered = set()
for (ref, num, net) in sorted(todo):
    if (ref, num) in covered:
        continue
    p = byref.get((ref, num))
    if p is None or net != p['net']:
        continue
    others = [q for q in m.netpads(net) if q is not p]
    if not others:
        continue
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    # geometric blockers: nets with a track passing within 1.6 mm of the pad
    blockers = []
    for n in rorder:
        if n == net:
            continue
        best = 1e9
        for (x1, y1, x2, y2, h, L) in segs_of[n]:
            if abs((x1 + x2) / 2 - p['x']) > 6 or abs((y1 + y2) / 2 - p['y']) > 6:
                continue
            dx, dy = x2 - x1, y2 - y1
            l2 = dx * dx + dy * dy
            t2 = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((p['x'] - x1) * dx + (p['y'] - y1) * dy) / l2))
            d = math.hypot(p['x'] - (x1 + t2 * dx), p['y'] - (y1 + t2 * dy)) - h
            if d < best:
                best = d
        if best < 1.6:
            blockers.append((best, n))
    blockers.sort()
    ok = False
    for q in others[:3]:
        sub = [p, q]
        orig = m.netpads
        m.netpads = (lambda nm, _s=sub, _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for kb in (0,):
                rip = [n for (_d, n) in blockers[:kb]]
                w4, segs, vias = do_net(net, [0.2, 0.15], rips=rip)
                if segs:
                    break
        finally:
            m.netpads = orig
        if segs:
            commit(net, segs, vias, append=True)
            nc = nmap = None
            print('%-4s.%-3s %-11s -> %s.%-3s  %d segs %d vias  (rips %s)' %
                  (ref, num, net, q['ref'], q['num'], len(segs), len(vias), rip), flush=True)
            fixed += 1
            covered.add((ref, num))
            covered.add((q['ref'], q['num']))
            ok = True
            break
    if not ok:
        print('%-4s.%-3s %-11s FAILED' % (ref, num, net), flush=True)
print('pads fixed: %d' % fixed)

# write board
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
for t in list(b.GetTracks()):
    b.Remove(t)
n = 0
for net in rorder:
    if net not in nmap:
        continue
    nc = nmap[net]
    for (x1, y1, x2, y2, h, L) in segs_of[net]:
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        s.SetWidth(int(round(2 * h * 1e6)))
        s.SetLayer(LWR[int(L)])
        s.SetNetCode(nc)
        b.Add(s)
        n += 1
    for (x, y) in vias_of[net]:
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(rr.VIA_D * 1e6)))
        v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(nc)
        b.Add(v)
        n += 1
b.Save(DST)
print('saved %s (%d items)' % (DST, n))
for a in log[-8:]:
    print('   ', a)
