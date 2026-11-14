"""Stitch pours of the same net across layers with vias placed where both layers are poured."""
import sys
import math
import os
import numpy as np
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass
from rr import Model, NX, NY, ix, iy, cx, cy

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = sys.argv[1]
DST = sys.argv[2]
SPEC = sys.argv[3] if len(sys.argv) > 3 else 'GND:4.0,0:0:0;+3V3:4.0,0:0:0'

m = Model(SRC)
m.build()
b = m.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()

fills = {}
for z in b.Zones():
    nm = z.GetNetname()
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        if ps.OutlineCount():
            fills.setdefault(nm, {}).setdefault(0 if lay == pcbnew.F_Cu else 1, []).append(ps)

total = 0
for spec in SPEC.split(';'):
    parts = spec.split(':')
    net = parts[0]
    step = float(parts[1].split(',')[0])
    if net not in fills or 0 not in fills[net] or 1 not in fills[net]:
        print('skip %s (no pour on both layers)' % net)
        continue
    pens = m.pen_for_net(net)
    blkF, blkB, blkV = m.pen_masks(pens, 0.1)
    placed = []
    y = -41.0
    while y < 42.0:
        x = -28.5
        while x < 29.0:
            i, j = ix(x), iy(y)
            if 0 <= i < NX and 0 <= j < NY and not blkV[j, i] and not m.antmask[j, i]:
                v = pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6)))
                if any(ps.Contains(v) for ps in fills[net][0]) and \
                        any(ps.Contains(v) for ps in fills[net][1]) and \
                        m.via_clear(x, y, net, pens):
                    vi = pcbnew.PCB_VIA(b)
                    vi.SetPosition(v)
                    vi.SetWidth(int(round(rr.VIA_D * 1e6)))
                    vi.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                    vi.SetViaType(pcbnew.VIATYPE_THROUGH)
                    vi.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                    vi.SetNetCode(nmap[net])
                    b.Add(vi)
                    m.add_copper([], [(x, y)])
                    placed.append((x, y))
                    total += 1
            x += step
        y += step
    print('%s: %d stitching vias' % (net, len(placed)))
print('total %d' % total)
b.Save(DST)
print('saved', DST)
