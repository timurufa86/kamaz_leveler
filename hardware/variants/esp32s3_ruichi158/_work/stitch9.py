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


def polys(netname, layer):
    out = []
    for z in b.Zones():
        if z.GetNetname() != netname or z.GetLayer() != layer:
            continue
        ps = z.GetFilledPolysList(layer)
        for i in range(ps.OutlineCount()):
            ch = ps.Outline(i)
            pts = [(ch.CPoint(k).x / 1e6, ch.CPoint(k).y / 1e6) for k in range(ch.PointCount())]
            if len(pts) > 3:
                out.append(pts)
    return out


def inside(pts, x, y):
    c = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            c = not c
    return c


def free(px, py, hw):
    i, j = rr.ix(px), rr.iy(py)
    if not (0 <= i < rr.NX and 0 <= j < rr.NY):
        return False
    t = -(rr.CLR + hw)
    return m.pen[0][j, i] < t and m.pen[1][j, i] < t


def add_via(net, x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(rr.VIA_D * 1e6)))
    v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(nmap[net])
    b.Add(v)
    m.add_copper([], [(x, y)])
    print('via %-6s at (%.2f,%.2f)' % (net, x, y), flush=True)


def pick(netf, netb, ppf, ppb, take_all=False):
    got = []
    for pts in ppf:
        bx = [p[0] for p in pts]
        by = [p[1] for p in pts]
        found = None
        y = min(by) + 0.5
        while y < max(by) and found is None:
            x = min(bx) + 0.5
            while x < max(bx):
                if inside(pts, x, y) and (not ppb or any(inside(q, x, y) for q in ppb)) and free(x, y, rr.VIA_R + 0.03):
                    found = (x, y)
                    break
                x += 0.5
            y += 0.5
        if found:
            add_via(netf, found[0], found[1])
            got.append(found)
            if not take_all:
                break
    return got


# 1. +3V3 F<->B : a via inside BOTH filled polygons
pf3 = polys('+3V3', pcbnew.F_Cu)
pb3 = polys('+3V3', pcbnew.B_Cu)
print('+3V3 islands F/B:', len(pf3), len(pb3), flush=True)
pick('+3V3', '+3V3', pf3, pb3)

# 2. GND F islands -> one via each (they join through the B pour)
pfg = polys('GND', pcbnew.F_Cu)
print('GND F islands:', len(pfg), flush=True)
pick('GND', 'GND', pfg, None, take_all=True)

# 3. +12V islands: straight bridge between the closest pair whose line is clear
p12 = sorted(polys('+12V', pcbnew.B_Cu), key=len, reverse=True)[:4]
print('+12V islands:', [len(p) for p in p12], flush=True)
best = None
for i in range(len(p12)):
    for j in range(i + 1, len(p12)):
        for a in p12[i][::1]:
            for c in p12[j][::1]:
                d = math.hypot(a[0] - c[0], a[1] - c[1])
                if d > 6.0 or (best and d >= best[0]):
                    continue
                n = max(2, int(d / 0.05) + 1)
                ok = True
                for k in range(n + 1):
                    fx = a[0] + (c[0] - a[0]) * k / n
                    fy = a[1] + (c[1] - a[1]) * k / n
                    ii, jj = rr.ix(fx), rr.iy(fy)
                    if not (0 <= ii < rr.NX and 0 <= jj < rr.NY) or m.pen[1][jj, ii] > -(rr.CLR + 0.30):
                        ok = False
                        break
                if ok:
                    best = (d, a, c)
if best:
    d, a, c = best
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(a[0] * 1e6)), int(round(a[1] * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(c[0] * 1e6)), int(round(c[1] * 1e6))))
    s.SetWidth(int(round(0.6e6)))
    s.SetLayer(pcbnew.B_Cu)
    s.SetNetCode(nmap['+12V'])
    b.Add(s)
    print('+12V bridge %.2f mm (%.2f,%.2f)->(%.2f,%.2f)' % (d, a[0], a[1], c[0], c[1]), flush=True)
else:
    print('+12V bridge: no clear straight path found', flush=True)

b.Save(MAIN)
print('saved', MAIN)
