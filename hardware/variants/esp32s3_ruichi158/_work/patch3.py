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
from rr import Model, NX, NY, ix, iy, cx, cy
from go import trim_to_pad, access_cells

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
REPORT = sys.argv[1]
SRC = sys.argv[2]
DST = sys.argv[3]
NETS = None if (len(sys.argv) > 4 and sys.argv[4] == 'ALL') else set(x.strip() for x in (sys.argv[4].split(',') if len(sys.argv) > 4 else ['GND', '+3V3', '+12V']))

txt = open(REPORT, encoding='utf-8', errors='replace').read()
todo = set()
for m in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    num, net, ref = m.group(1), m.group(2), m.group(3)
    if NETS is None or net in NETS:
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
copper = {}
segs_of = {}
vias_of = {}
rorder = []
for t in b.GetTracks():
    nm = t.GetNetname()
    if nm not in segs_of:
        segs_of[nm] = []
        vias_of[nm] = []
        rorder.append(nm)
    if t.Type() == pcbnew.PCB_VIA_T:
        q = t.GetPosition()
        vias_of[nm].append((q.x / 1e6, q.y / 1e6))
        copper.setdefault(nm, []).append((q.x / 1e6, q.y / 1e6, q.x / 1e6, q.y / 1e6, 0, 0))
        copper.setdefault(nm, []).append((q.x / 1e6, q.y / 1e6, q.x / 1e6, q.y / 1e6, 1, 0))
        continue
    s0, e0 = t.GetStart(), t.GetEnd()
    L2 = 0 if t.GetLayer() == pcbnew.F_Cu else 1
    segs_of[nm].append((s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6, t.GetWidth() / 2e6, L2))
    copper.setdefault(nm, []).append((s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6, L2, 0))
byref = {(p['ref'], p['num']): p for p in m.pads}
done = 0
fail = []
for (ref, num) in sorted(todo):
    p = byref.get((ref, num))
    if p is None:
        continue
    net = p['net']
    save_pens = m.pen
    m.reset_pen()
    for n2 in rorder:
        if n2 != net and n2 in segs_of:
            m.add_copper(segs_of[n2], vias_of[n2])
    pens = m.pen_for_net(net)
    m.pen = save_pens
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
        blk = [blkF, blkB]
        acc = access_cells(m, p, 0.075, L, blk[L])
        if not acc:
            continue
        goals = []
        for gy in range(iy(p['y']) - 90, iy(p['y']) + 91, 3):
            for gx in range(ix(p['x']) - 90, ix(p['x']) + 91, 3):
                if not (0 <= gx < NX and 0 <= gy < NY):
                    continue
                if blk[L][gy][gx]:
                    continue
                if ps.Contains(pcbnew.VECTOR2I(int(round(cx(gx) * 1e6)), int(round(cy(gy) * 1e6)))):
                    goals.append((L, gx, gy))
        if not goals:
            continue
        for (x1, y1, x2, y2, L2, _z) in copper.get(net, []):
            if L2 != L:
                continue
            ln = math.hypot(x2 - x1, y2 - y1)
            n = max(1, int(ln / 0.3))
            if abs((x1 + x2) / 2 - p['x']) > 14 or abs((y1 + y2) / 2 - p['y']) > 14:
                continue
            for t2 in range(n + 1):
                f = t2 / n
                gx, gy = ix(x1 + (x2 - x1) * f), iy(y1 + (y2 - y1) * f)
                if 0 <= gx < NX and 0 <= gy < NY and not blk[L][gy][gx]:
                    goals.append((L, gx, gy))
        for q in m.pads:
            if q['net'] != net or q is p:
                continue
            if not (q['onF'] if L == 0 else q['onB']):
                continue
            acc2 = access_cells(m, q, 0.075, L, blk[L])
            for (x, y) in acc2:
                goals.append((L, ix(x), iy(y)))
        seeds = [(x, y, L, math.hypot(x - p['x'], y - p['y'])) for (x, y) in acc]
        pts = m.astar(seeds, goals, goals and (cx(goals[0][1]), cy(goals[0][2])) or (p['x'], p['y']),
                      0.075, pens, margin=12.0, via_cost=1.0)
        if pts is None:
            continue
        runs = []
        cur = []
        for (x, y, L2, via) in pts:
            if cur and cur[-1][2] != L2:
                runs.append(cur)
                cur = []
            cur.append((x, y, L2))
        if cur:
            runs.append(cur)
        runs[0].insert(0, (p['x'], p['y'], runs[0][0][2]))
        for k, run in enumerate(runs):
            pl = rr.collinear_merge([(q[0], q[1]) for q in run])
            pl = rr.smooth(pl, m, 0.075, run[0][2], net)
            if k == 0 and len(pl) > 1:
                pl[0] = trim_to_pad(p, pl[0][0], pl[0][1], pl[1][0], pl[1][1])
            for t in range(len(pl) - 1):
                (ax, ay), (bx, by) = pl[t], pl[t + 1]
                if math.hypot(bx - ax, by - ay) < 1e-4:
                    continue
                s2 = pcbnew.PCB_TRACK(b)
                s2.SetStart(pcbnew.VECTOR2I(int(round(ax * 1e6)), int(round(ay * 1e6))))
                s2.SetEnd(pcbnew.VECTOR2I(int(round(bx * 1e6)), int(round(by * 1e6))))
                s2.SetWidth(int(round(0.15 * 1e6)))
                s2.SetLayer(LAY[run[0][2]])
                s2.SetNetCode(nmap[net])
                b.Add(s2)
            if k + 1 < len(runs):
                vx2, vy2 = run[-1][0], run[-1][1]
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(round(vx2 * 1e6)), int(round(vy2 * 1e6))))
                v.SetWidth(int(round(rr.VIA_D * 1e6)))
                v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNetCode(nmap[net])
                b.Add(v)
        ok = True
        done += 1
    if not ok:
        fail.append('%s.%s(%s)' % (ref, num, net))
print('connected %d pads; failed %d: %s' % (done, len(fail), fail[:12]))
b.Save(DST)
print('saved', DST)
