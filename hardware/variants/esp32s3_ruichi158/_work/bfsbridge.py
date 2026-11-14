import sys
import math
import os
import heapq
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

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
m = Model(MAIN)
m.build()
b = m.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
LWR = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
NX, NY = rr.NX, rr.NY


def islands(net, lay):
    out = []
    for z in b.Zones():
        if z.GetNetname() != net or z.GetLayer() != lay:
            continue
        ps = z.GetFilledPolysList(lay)
        for i in range(ps.OutlineCount()):
            ch = ps.Outline(i)
            pts = [(ch.CPoint(k).x / 1e6, ch.CPoint(k).y / 1e6) for k in range(ch.PointCount())]
            if len(pts) > 3:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                out.append({'pts': pts, 'area': (max(xs) - min(xs)) * (max(ys) - min(ys))})
    return sorted(out, key=lambda d: -d['area'])


def bfs(a, c, hw, layers=(0, 1), win=14.0):
    """2-layer BFS from a to c on the model grid; returns [(x,y,layer),...] or None"""
    x0, x1 = min(a[0], c[0]) - win, max(a[0], c[0]) + win
    y0, y1 = min(a[1], c[1]) - win, max(a[1], c[1]) + win
    si, sj = rr.ix(a[0]), rr.iy(a[1])
    gi, gj = rr.ix(c[0]), rr.iy(c[1])
    thr = -(rr.CLR + hw)
    VIA_R = rr.VIA_R
    thrv = -(rr.CLR + VIA_R)
    INF = 1e18
    dist = {}
    pq = []
    start = (si, sj, layers[0])
    dist[start] = 0.0
    heapq.heappush(pq, (0.0, start))
    prev = {}
    DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while pq:
        d, (i, j, L) = heapq.heappop(pq)
        if d > dist.get((i, j, L), INF) + 1e-9:
            continue
        if abs(i - gi) <= 1 and abs(j - gj) <= 1:
            path = [(i, j, L)]
            k = (i, j, L)
            while k in prev:
                k = prev[k]
                path.append(k)
            return [(p[0] * rr.CELL + rr.X0, p[1] * rr.CELL + rr.Y0, p[2]) for p in reversed(path)]
        for (di, dj) in DIRS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < NX and 0 <= nj < NY):
                continue
            x, y = ni * rr.CELL + rr.X0, nj * rr.CELL + rr.Y0
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            if m.pen[L][nj, ni] > thr:
                continue
            nd = d + math.hypot(di, dj) * rr.CELL
            key = (ni, nj, L)
            if nd < dist.get(key, INF):
                dist[key] = nd
                prev[key] = (i, j, L)
                heapq.heappush(pq, (nd, key))
        if len(layers) > 1:
            L2 = [l for l in (0, 1) if l != L][0]
            if m.pen[0][j, i] < thrv and m.pen[1][j, i] < thrv and m.penhole[j, i] < 0:
                key = (i, j, L2)
                nd = d + 1.2
                if nd < dist.get(key, INF):
                    dist[key] = nd
                    prev[key] = (i, j, L)
                    heapq.heappush(pq, (nd, key))
    return None


def emit(net, path, w):
    """add segments along the path; vias where the layer changes"""
    n = 0
    for k in range(len(path) - 1):
        x1, y1, L1 = path[k]
        x2, y2, L2 = path[k + 1]
        if L1 != L2:
            v = pcbnew.PCB_VIA(b)
            v.SetPosition(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
            v.SetWidth(int(round(rr.VIA_D * 1e6)))
            v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNetCode(nmap[net])
            b.Add(v)
            m.add_copper([], [(x1, y1)])
            n += 1
            continue
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        s.SetWidth(int(round(w * 1e6)))
        s.SetLayer(LWR[L1])
        s.SetNetCode(nmap[net])
        b.Add(s)
        m.add_copper([(x1, y1, x2, y2, w / 2e6, L1)], [])
        n += 1
    return n


def bridge(net, lay_a, lay_b, w, name):
    A = islands(net, lay_a)
    B = islands(net, lay_b) if lay_b != lay_a else A
    if not A or not B:
        print('%s: no islands' % name)
        return False
    pairs = []
    for ia in range(min(3, len(A))):
        for ib in range(min(5, len(B))):
            if ib == ia and lay_a == lay_b:
                continue
            for pa in A[ia]['pts'][::2]:
                for pb in B[ib]['pts'][::2]:
                    d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                    if 0.8 < d < 14.0:
                        pairs.append((d, pa, pb))
    pairs.sort()
    if not pairs:
        print('%s: no island pair' % name)
        return False
    PCB2RR0 = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
    ra0, rb0 = PCB2RR0[lay_a], PCB2RR0[lay_b]
    lay0 = (ra0, rb0) if ra0 != rb0 else (ra0, 1 - ra0)
    got = None
    seen = set()
    for (d, pa, pb) in pairs:
        k = (round(pa[0], 1), round(pa[1], 1), round(pb[0], 1), round(pb[1], 1))
        if k in seen:
            continue
        seen.add(k)
        if len(seen) > 8:
            break
        path = bfs(pa, pb, w / 2.0, lay0)
        if path:
            got = (d, pa, pb, path)
            break
    if not got:
        print('%s: BFS found no corridor in 8 candidate pairs' % name)
        return False
    d, pa, pb, path = got
    PCB2RR = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
    ra, rb = PCB2RR[lay_a], PCB2RR[lay_b]
    n = emit(net, path, w)
    print('%s: bridge %.2f mm -> %d objects, start (%.2f,%.2f) end (%.2f,%.2f)' %
          (name, d, n, pa[0], pa[1], pb[0], pb[1]), flush=True)
    return True


bridge('+12V', pcbnew.B_Cu, pcbnew.B_Cu, 0.6, '+12V_B')
bridge('+3V3', pcbnew.F_Cu, pcbnew.B_Cu, 0.4, '+3V3_FB')
bridge('GND', pcbnew.F_Cu, pcbnew.F_Cu, 0.15, 'GND_F')
b.Save(MAIN)
print('saved', MAIN)
