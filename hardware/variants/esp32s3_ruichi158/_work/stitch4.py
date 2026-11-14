"""Connect every unconnected pad of pour nets to the LARGEST filled polygon of the other layer."""
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
from rr import Model, NX, NY, ix, iy
from go import trim_to_pad

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
REPORT = sys.argv[1]
SRC = sys.argv[2]
DST = sys.argv[3]
NETS = set(x.strip() for x in (sys.argv[4].split(',') if len(sys.argv) > 4 else ['GND', '+3V3', '+12V']))

txt = open(REPORT, encoding='utf-8', errors='replace').read()
todo = set()
for m in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    num, net, ref = m.group(1), m.group(2), m.group(3)
    if net in NETS:
        todo.add((ref, num))
print('pads to connect: %d' % len(todo))

m = Model(SRC)
m.build()
b = m.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()

# largest filled polygon per (net, layer)
big = {}
for z in b.Zones():
    nm = z.GetNetname()
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        if ps.OutlineCount() == 0:
            continue
        areas = [(ps.Outline(k).Area(), k) for k in range(ps.OutlineCount())]
        areas.sort(reverse=True)
        cand = pcbnew.SHAPE_POLY_SET()
        cand.AddOutline(ps.Outline(areas[0][1]))
        L = 0 if lay == pcbnew.F_Cu else 1
        if (nm, L) not in big or cand.Area() > big[(nm, L)][1].Area():
            big[(nm, L)] = (L, cand)
print('largest pours: %s' % {(k[0], k[1]): round(v[1].Area() / 1e12, 1) for k, v in big.items()})

LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
byref = {(p['ref'], p['num']): p for p in m.pads}
done = 0
fail = []
for (ref, num) in sorted(todo):
    p = byref.get((ref, num))
    if p is None:
        continue
    net = p['net']
    pens = m.pen_for_net(net)
    blkF, blkB, blkV = m.pen_masks(pens, 0.1)
    blk = [blkF, blkB]
    ok = False
    for L in (0, 1):
        if ok:
            break
        if not (p['onF'] if L == 0 else p['onB']):
            continue
        if (net, L) not in big:
            continue
        ps = big[(net, L)][1]
        for r in (0.5, 0.7, 0.9, 1.1, 1.4, 1.7, 2.1, 2.6, 3.2, 4.0, 5.0, 6.5):
            for t in range(48):
                a = 2 * math.pi * t / 48.0
                vx = p['x'] + r * math.cos(a)
                vy = p['y'] + r * math.sin(a)
                i, j = ix(vx), iy(vy)
                if not (0 <= i < NX and 0 <= j < NY):
                    continue
                if blk[L][j, i] or blkV[j, i] or m.antmask[j, i]:
                    continue
                if not ps.Contains(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6)))):
                    continue
                if not m.seg_clear(p['x'], p['y'], vx, vy, 0.1, L, net):
                    continue
                if not m.via_clear(vx, vy, net, pens):
                    continue
                qx, qy = trim_to_pad(p, p['x'], p['y'], vx, vy, 0.10)
                s = pcbnew.PCB_TRACK(b)
                s.SetStart(pcbnew.VECTOR2I(int(round(qx * 1e6)), int(round(qy * 1e6))))
                s.SetEnd(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                s.SetWidth(int(round(0.2 * 1e6)))
                s.SetLayer(LAY[L])
                s.SetNetCode(nmap[net])
                b.Add(s)
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                v.SetWidth(int(round(rr.VIA_D * 1e6)))
                v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNetCode(nmap[net])
                b.Add(v)
                m.add_copper([(qx, qy, vx, vy, 0.1, L)], [(vx, vy)])
                done += 1
                ok = True
                break
            if ok:
                break
    if not ok:
        fail.append('%s.%s(%s)' % (ref, num, net))
print('connected %d pads; failed %d: %s' % (done, len(fail), fail[:12]))
b.Save(DST)
print('saved', DST)
