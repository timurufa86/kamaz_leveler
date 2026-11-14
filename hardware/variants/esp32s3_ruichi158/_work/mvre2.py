"""Last-mile repair: route each unconnected pad to its nearest same-net pad (2-pad net)."""
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
REPORT = sys.argv[1]
SRC = sys.argv[2]
DST = sys.argv[3]

import io as _io
txt = open(REPORT, encoding='utf-8', errors='replace').read()
# drop +3V3 tracks inside the moved-pads window (the +3V3 pour covers it)
nz = 0
for t in list(b.GetTracks()):
    if t.Type() == pcbnew.PCB_VIA_T or t.GetNetname() != '+3V3':
        continue
    x1, y1 = t.GetStart().x / 1e6, t.GetStart().y / 1e6
    x2, y2 = t.GetEnd().x / 1e6, t.GetEnd().y / 1e6
    if max(x1, x2) > -18.6 and min(x1, x2) < -14.4 and max(y1, y2) > 4.4 and min(y1, y2) < 10.1:
        b.Remove(t)
        nz += 1
print('dropped +3V3 tracks in the moved window:', nz, flush=True)
EXTRACT = [('R33', '2'), ('R34', '2'), ('U6', '10'), ('U6', '11'), ('U3', '4'), ('U3', '8')]
todo = []
seen = set()
for m0 in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    key = (m0.group(3), m0.group(1), m0.group(2))
    if key not in seen:
        seen.add(key)
        todo.append(key)
for (r_, n_) in EXTRACT if False else []:
    pass
print('unconnected pads: %d' % len(todo))

m = Model(SRC)
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
for (ref, num, net) in sorted(todo) + [(r_, n_, None) for (r_, n_) in EXTRACT]:
    p = byref.get((ref, num))
    if p is None or (net is not None and net != p['net']):
        continue
    others = [q for q in m.netpads(net) if q is not p]
    if not others:
        continue
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    ok = False
    rorder = [n for n in set(t.GetNetname() for t in b.GetTracks()) if n and n != net]
    for q in others[:5]:
        sub = [p, q]
        orig = m.netpads
        m.netpads = (lambda nm, _s=sub, _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for C in ([],):
                m.reset_pen()
                rip = set(C)
                for t in list(b.GetTracks()):
                    nm2 = t.GetNetname()
                    if nm2 in rip or nm2 == net:
                        continue
                    if t.Type() == pcbnew.PCB_VIA_T:
                        q2 = t.GetPosition()
                        m.add_copper([], [(q2.x / 1e6, q2.y / 1e6)])
                    else:
                        s0, e0 = t.GetStart(), t.GetEnd()
                        m.add_copper([(s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6,
                                       t.GetWidth() / 2e6, 0 if t.GetLayer() == pcbnew.F_Cu else 1)], [])
                for w in (0.25, 0.2, 0.15):
                    segs, vias = route_net(m, net, w / 2.0, log, margin=20.0, via_cost=0.4, use_hub=False)
                    if segs:
                        break
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
            m.reset_pen()
            for t in b.GetTracks():
                if t.Type() == pcbnew.PCB_VIA_T:
                    q3 = t.GetPosition()
                    m.add_copper([], [(q3.x / 1e6, q3.y / 1e6)])
                else:
                    s1, e1 = t.GetStart(), t.GetEnd()
                    m.add_copper([(s1.x / 1e6, s1.y / 1e6, e1.x / 1e6, e1.y / 1e6,
                                   t.GetWidth() / 2e6, 0 if t.GetLayer() == pcbnew.F_Cu else 1)], [])
            print('%-4s.%-3s %-10s -> %s.%-3s : %d segs %d vias' %
                  (ref, num, net, q['ref'], q['num'], len(segs), len(vias)), flush=True)
            fixed += 1
            ok = True
            break
    if not ok:
        print('%-4s.%-3s %-10s FAILED' % (ref, num, net), flush=True)
print('fixed %d pads' % fixed)
b.Save(DST)
print('saved', DST)
