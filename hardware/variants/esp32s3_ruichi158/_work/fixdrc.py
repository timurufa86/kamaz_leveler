"""Repair DRC violations: rip the offending net, forbid the violation area, re-route."""
import io
import re
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
from rr import Model
from go import route_net, width_of

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
REPORT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WORK, 'drc.txt')
SRC = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
DST = sys.argv[3] if len(sys.argv) > 3 else os.path.join(ROOT, 'fixed.kicad_pcb')
BAD = ('clearance', 'shorting_items', 'hole_to_hole', 'solder_mask_bridge',
       'track_dangling', 'via_dangling', 'track_not_centered_on_via')

txt = open(REPORT, encoding='utf-8', errors='replace').read().split('\n')
issues = []
cur = None
for ln in txt:
    m = re.match(r'^\[(\w+)\]', ln)
    if m:
        cur = {'type': m.group(1), 'pts': [], 'nets': []}
        issues.append(cur)
        continue
    if cur is None:
        continue
    m = re.match(r'\s*@\(([-\d.]+) mm, ([-\d.]+) mm\): (.+)$', ln)
    if m:
        cur['pts'].append((float(m.group(1)), float(m.group(2))))
        d = m.group(3)
        for nm in re.findall(r'\[([^\]]+)\]', d):
            cur['nets'].append(nm)
        if 'Zone' in d:
            m2 = re.search(r"Zone '([^']+)'", d)
            if m2:
                cur['nets'].append('__zone__')
print('parsed issues: %d' % len(issues))
per = {}
for it in issues:
    if it['type'] not in BAD:
        continue
    nets = [n for n in it['nets'] if n and not n.startswith('unconnected')]
    for n in nets:
        per.setdefault(n, []).append(it)
print('nets with violations: %s' % {k: len(v) for k, v in per.items()})

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
rorder = sorted(segs_of.keys(), key=lambda n: -len(segs_of[n]))
wused = {n: round(2 * segs_of[n][0][4], 3) for n in rorder if segs_of[n]}

# choose nets to re-route: skip pour nets
targets = [n for n in per if n in segs_of and n not in ('GND', '+3V3', '+5V', '+12V', '+24V')]
targets.sort(key=lambda n: len(m.netpads(n)))
print('re-route targets:', targets)

log = []
changed = 0
for net in targets:
    pads = m.netpads(net)
    forb = []
    for it in per[net]:
        for (x, y) in it['pts']:
            r = 0.55 if it['type'] in ('clearance', 'shorting_items') else 0.4
            forb.append((x - r, y - r, x + r, y + r))
    # keep forbid boxes away from own pads
    cleaned = []
    for (x0, y0, x1, y1) in forb:
        if any(x0 - 0.3 < p['x'] < x1 + 0.3 and y0 - 0.3 < p['y'] < y1 + 0.3 for p in pads):
            continue
        cleaned.append((x0, y0, x1, y1))
    ok = False
    for w in (min(width_of(net), 0.3), 0.2, 0.15):
        m.reset_pen()
        for n in rorder:
            if n == net:
                continue
            m.add_copper(segs_of[n], vias_of[n])
        m.forbid = cleaned
        segs, vias = route_net(m, net, w / 2.0, log, margin=12.0, via_cost=1.2, use_hub=False)
        if segs is None:
            continue
        m.forbid = []
        m.add_copper(segs, vias)
        segs_of[net] = segs
        vias_of[net] = vias
        print('%-12s re-routed w=%.2f segs %d vias %d (forbid %d)' %
              (net, w, len(segs), len(vias), len(cleaned)), flush=True)
        ok = True
        changed += 1
        break
    if not ok:
        print('%-12s could not re-route' % net, flush=True)
print('re-routed %d nets' % changed)

# write out
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
for t in list(b.GetTracks()):
    b.Remove(t)
L2 = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
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
        s.SetLayer(L2[int(L)])
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
