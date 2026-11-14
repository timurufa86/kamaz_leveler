import io
import sys
import math
import os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
sys.path.insert(0, WORK)
import numpy as np
import pcbnew
import rr
from rr import Model, NX, NY, ix, iy

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'stitched.kicad_pcb')
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'zoned.kicad_pcb')
IU = 1e6


def mm(v):
    return int(round(v * IU))


model = Model(SRC)
model.build()

b = model.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()

def arc(cx, cy, r, a0, a1, n=18):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]


P = []
P += [(-29.45, -33.3), (-27.0, -33.3)]
P += arc(-27.0, -29.0, 4.3, -90, 90)
P += [(-29.45, -24.7), (-29.45, 24.7), (-27.0, 24.7)]
P += arc(-27.0, 29.0, 4.3, -90, 90)
P += [(-29.45, 33.3), (-29.45, 39.75)]
P += arc(-27.0, 39.75, 2.45, 180, 90, 10)
P += [(-13.0, 42.2), (-13.0, 35.7), (13.0, 35.7), (13.0, 42.2), (27.0, 42.2)]
P += arc(27.0, 39.75, 2.45, 90, 0, 10)
P += [(29.45, 39.2), (29.45, 33.3), (27.0, 33.3)]
P += arc(27.0, 29.0, 4.3, 90, 270)
P += [(29.45, 24.7), (29.45, -24.7), (27.0, -24.7)]
P += arc(27.0, -29.0, 4.3, 90, -90)
P += [(29.45, -33.3), (29.45, -39.75)]
P += arc(27.0, -39.75, 2.45, 180, 270, 10)
P += [(-27.0, -42.2)]
P += arc(-27.0, -39.75, 2.45, 270, 360, 10)
poly = pcbnew.SHAPE_POLY_SET()
oi = poly.NewOutline()
for (x, y) in P:
    poly.Append(mm(x), mm(y), oi)
print('GND outline: %d points, area %.1f mm2' % (len(P), poly.Area() / IU / IU))

PLUS3 = (-27.0, 1.5, -12.5, 27.5)
PLUS5 = (-28.4, -25.0, -13.0, 1.0)


def add_zone(net, layer, prio, outline=None, rect=None, minth=0.25, clr=0.15):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNetCode(nmap[net])
    z.SetLocalClearance(mm(clr))
    z.SetMinThickness(mm(minth))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetAssignedPriority(prio)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    if outline is not None:
        z.SetOutline(outline)
    else:
        p = pcbnew.SHAPE_POLY_SET()
        oi = p.NewOutline()
        for (x, y) in ((rect[0], rect[1]), (rect[2], rect[1]), (rect[2], rect[3]), (rect[0], rect[3])):
            p.Append(mm(x), mm(y), oi)
        z.SetOutline(p)
    z.SetZoneName('%s_%s' % (net.replace('+', 'p'), b.GetLayerName(layer)))
    b.Add(z)
    print('zone added: %s net=%s layer=%s' % (net, z.GetNetname(), b.GetLayerName(z.GetLayer())), flush=True)


add_zone('GND', pcbnew.F_Cu, 0, outline=poly)

b.BuildConnectivity()
filler = pcbnew.ZONE_FILLER(b)
filler.Fill(b.Zones(), False)
b.BuildConnectivity()
tot = 0
for z in b.Zones():
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        n = 0
        for k in range(ps.OutlineCount()):
            n += ps.Outline(k).PointCount()
            for h in range(ps.HoleCount(k)):
                n += ps.Hole(k, h).PointCount()
        print('zone %-6s %-4s polys %3d verts %5d area %8.1f' %
              (z.GetNetname(), b.GetLayerName(lay), ps.OutlineCount(), n, ps.Area() / IU / IU))
        tot += n
print('total zone vertices', tot)
b.Save(DST)
print('saved', DST)
