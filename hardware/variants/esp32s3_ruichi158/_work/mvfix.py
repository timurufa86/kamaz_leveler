import re
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
REPORT = sys.argv[1]
SRC = sys.argv[2]
DST = sys.argv[3]
TMP = os.path.join(ROOT, '_work', '_tmp_mv.kicad_pcb')

# --- 1. drop +3V3 tracks inside the moved-pads window (the +3V3 pour covers it)
b = pcbnew.LoadBoard(SRC)
nz = 0
for t in list(b.GetTracks()):
    if t.Type() == pcbnew.PCB_VIA_T or t.GetNetname() != '+3V3':
        continue
    x1, y1 = t.GetStart().x / 1e6, t.GetStart().y / 1e6
    x2, y2 = t.GetEnd().x / 1e6, t.GetEnd().y / 1e6
    if max(x1, x2) > -18.6 and min(x1, x2) < -14.4 and max(y1, y2) > 4.4 and min(y1, y2) < 10.1:
        b.Remove(t)
        nz += 1
print('dropped +3V3 tracks in moved window: %d' % nz, flush=True)
b.Save(TMP)

# --- 2. pair-connect the pads
m = Model(TMP)
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
txt = open(REPORT, encoding='utf-8', errors='replace').read()
want = set()
for m0 in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    want.add((m0.group(3), m0.group(1)))
for extra in (('R33', '2'), ('R34', '2'), ('U6', '10'), ('U6', '11'), ('U3', '4'), ('U3', '8')):
    want.add(extra)
print('pads to connect: %d' % len(want), flush=True)

log = []
fixed = 0
covered = set()
for (ref, num) in sorted(want):
    if (ref, num) in covered:
        continue
    p = byref.get((ref, num))
    if p is None:
        continue
    net = p['net']
    others = [q for q in m.netpads(net) if q is not p]
    if not others:
        continue
    others.sort(key=lambda q: math.hypot(q['x'] - p['x'], q['y'] - p['y']))
    ok = False
    for q in others[:5]:
        orig = m.netpads
        m.netpads = (lambda nm, _s=[p, q], _o=orig: _s if nm == net else _o(nm))
        segs = None
        try:
            for w in (0.25, 0.2, 0.15):
                segs, vias = route_net(m, net, w / 2.0, log, margin=20.0, via_cost=0.4, use_hub=False)
                if segs:
                    break
        finally:
            m.netpads = orig
        if segs:
            m.add_copper(segs, vias)
            nc = nmap[net]
            for (x1, y1, x2, y2, h, L) in segs:
                s2 = pcbnew.PCB_TRACK(b)
                s2.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
                s2.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
                s2.SetWidth(int(round(2 * h * 1e6)))
                s2.SetLayer(LAY[int(L)])
                s2.SetNetCode(nc)
                b.Add(s2)
            for (x, y) in vias:
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
                v.SetWidth(int(round(rr.VIA_D * 1e6)))
                v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNetCode(nc)
                b.Add(v)
            print('%-4s.%-3s %-11s -> %s.%-3s : %d segs %d vias' %
                  (ref, num, net, q['ref'], q['num'], len(segs), len(vias)), flush=True)
            fixed += 1
            covered.add((ref, num))
            covered.add((q['ref'], q['num']))
            ok = True
            break
    if not ok:
        print('%-4s.%-3s %-11s FAILED' % (ref, num, net), flush=True)
print('fixed %d pads' % fixed)
b.Save(DST)
print('saved', DST)
