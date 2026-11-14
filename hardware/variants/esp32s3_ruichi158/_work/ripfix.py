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
from go import route_net, width_of

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
REPORT = os.path.join(ROOT, '_work', 'drc_G.txt')
TARGETS = [('U6', '4'), ('U6', '8'), ('U6', '9'), ('U6', '10'), ('U6', '19'), ('U6', '20'),
           ('R30', '2'), ('D9', '2'), ('U7', '21')]

m = Model(MAIN)
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
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
byref = {(p['ref'], p['num']): p for p in m.pads}
log = []
fixed = 0


def prox(n, p):
    best = 1e6
    for s in segs_of.get(n, []):
        mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
        d = math.hypot(mx - p['x'], my - p['y'])
        if d < best:
            best = d
    return best


for (ref, num) in TARGETS:
    p = byref.get((ref, num))
    if p is None:
        continue
    net = p['net']
    others = [q for q in m.netpads(net) if q is not p]
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    if not others:
        continue
    cand = sorted([n for n in rorder if n != net and segs_of.get(n)], key=lambda n: prox(n, p))
    done = False
    for kb in (0, 1, 2, 4, 6):
        rip = set(cand[:kb])
        snap = {n: (list(segs_of[n]), list(vias_of[n])) for n in rip}
        q = others[0]
        orig = m.netpads
        m.netpads = (lambda nm, _s=[p, q], _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for w in (0.25, 0.2, 0.15):
                m.reset_pen()
                for n in rorder:
                    if n in rip or n == net:
                        continue
                    m.add_copper(segs_of[n], vias_of[n])
                segs, vias = route_net(m, net, w / 2.0, log, margin=20.0, via_cost=0.4, use_hub=False)
                if segs:
                    break
        finally:
            m.netpads = orig
        if not segs:
            continue
        m.add_copper(segs, vias)
        ok2 = True
        for n in [x for x in rorder if x in rip]:
            s2, v2 = route_net(m, n, max(0.15, min(width_of(n), 0.3)) / 2.0, log,
                               margin=16.0, via_cost=1.2)
            if not s2:
                ok2 = False
                break
            m.add_copper(s2, v2)
            segs_of[n] = s2
            vias_of[n] = v2
        if not ok2:
            for n, (s0_, v0_) in snap.items():
                segs_of[n] = s0_
                vias_of[n] = v0_
            continue
        segs_of[net] = list(segs_of.get(net, [])) + list(segs)
        vias_of[net] = list(vias_of.get(net, [])) + list(vias)
        print('%-4s.%-3s %-11s -> %s.%-2s %d segs %d vias (rip %d)' %
              (ref, num, net, q['ref'], q['num'], len(segs), len(vias), kb), flush=True)
        fixed += 1
        done = True
        break
    if not done:
        print('%-4s.%-3s %-11s FAILED' % (ref, num, net), flush=True)
print('fixed', fixed)

# write board
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
b.Save(MAIN)
print('saved (%d items)' % n)
