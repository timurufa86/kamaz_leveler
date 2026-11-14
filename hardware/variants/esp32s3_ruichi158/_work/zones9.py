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
    return out


def pen(lay, x, y):
    i, j = rr.ix(x), rr.iy(y)
    if not (0 <= i < rr.NX and 0 <= j < rr.NY):
        return 1e9
    return m.pen[lay][j, i]


def clear_line(lay, a, c, hw, step=0.05):
    d = math.hypot(a[0] - c[0], a[1] - c[1])
    n = max(2, int(d / step) + 1)
    thr = -(rr.CLR + hw + 0.02)
    for k in range(n + 1):
        x = a[0] + (c[0] - a[0]) * k / n
        y = a[1] + (c[1] - a[1]) * k / n
        if pen(lay, x, y) > thr:
            return False
    return True


def add_track(net, lay, a, c, w):
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(a[0] * 1e6)), int(round(a[1] * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(c[0] * 1e6)), int(round(c[1] * 1e6))))
    s.SetWidth(int(round(w * 1e6)))
    s.SetLayer({0: pcbnew.F_Cu, 1: pcbnew.B_Cu}[lay])
    s.SetNetCode(nmap[net])
    b.Add(s)
    m.add_copper([(a[0], a[1], c[0], c[1], w / 2e6, lay)], [])


def bridge(net, lay, w, maxarea=500.0, limit=6):
    isl = sorted(islands(net, lay), key=lambda d: -d['area'])
    if len(isl) < 2:
        return 0
    main = isl[0]
    n = 0
    for other in isl[1:]:
        if n >= limit:
            break
        if other['area'] < 0.8:
            continue
        best = None
        for a in other['pts']:
            for c in main['pts']:
                d = math.hypot(a[0] - c[0], a[1] - c[1])
                if d > 8.0 or (best and d >= best[0]):
                    continue
                if clear_line(lay, a, c, w / 2.0):
                    best = (d, a, c)
        if best:
            d, a, c = best
            add_track(net, lay, a, c, w)
            print('  bridge %-6s L%d %.2f mm (%.2f,%.2f)->(%.2f,%.2f)' % (net, lay, d, a[0], a[1], c[0], c[1]), flush=True)
            n += 1
    return n


print('GND F bridges:', bridge('GND', 0, 0.15), flush=True)
print('+12V B bridges:', bridge('+12V', 1, 0.6), flush=True)

# +3V3 F<->B : via inside the two LARGEST islands, free of other copper
fF = sorted(islands('+3V3', 0), key=lambda d: -d['area'])
fB = sorted(islands('+3V3', 1), key=lambda d: -d['area'])


def inside(pts, x, y):
    c = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            c = not c
    return c


placed = None
for pf in fF[:3]:
    for pb in fB[:3]:
        pts = []
        for k in range(len(pf['pts'])):
            pts.append(pf['pts'][k])
        pts.sort()
        for (x, y) in pts:
            if inside(pb['pts'], x, y) and pen(0, x, y) < -(rr.CLR + rr.VIA_R + 0.05) and \
               pen(1, x, y) < -(rr.CLR + rr.VIA_R + 0.05):
                placed = (x, y)
                break
        if placed:
            break
    if placed:
        break
if placed:
    x, y = placed
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(rr.VIA_D * 1e6)))
    v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(nmap['+3V3'])
    b.Add(v)
    print('+3V3 stitching via at (%.2f,%.2f)' % (x, y), flush=True)
else:
    print('+3V3: no common point found', flush=True)

b.Save(MAIN)
print('saved', MAIN)
