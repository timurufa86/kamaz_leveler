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
LWR = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}


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


def add_track(net, lay, a, c, w):
    s = pcbnew.PCB_TRACK(b)
    s.SetStart(pcbnew.VECTOR2I(int(round(a[0] * 1e6)), int(round(a[1] * 1e6))))
    s.SetEnd(pcbnew.VECTOR2I(int(round(c[0] * 1e6)), int(round(c[1] * 1e6))))
    s.SetWidth(int(round(w * 1e6)))
    s.SetLayer(LWR[lay])
    s.SetNetCode(nmap[net])
    b.Add(s)
    m.add_copper([(a[0], a[1], c[0], c[1], w / 2e6, lay)], [])


def viahop(net, lay_a, lay_b, w, name):
    """via inside island A (lay_a) -> track on lay_b -> point inside island B (lay_b)"""
    A = islands(net, lay_a)
    B = islands(net, lay_b) if lay_b != lay_a else A
    if not A or not B:
        print('%s: no islands' % name)
        return False
    best = None
    for ia_i in range(min(4, len(A))):
        ia = A[ia_i]
        for pa in ia['pts']:
            if pen(lay_a, pa[0], pa[1]) > -(rr.CLR + rr.VIA_R + 0.05):
                continue
            for ib_i in range(min(6, len(B))):
                if ib_i == ia_i and lay_b == lay_a:
                    continue
                for pb in B[ib_i]['pts']:
                    d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                    if d < 0.5 or d > 12.0 or (best and d >= best[0]):
                        continue
                    if clear_line(lay_b, pa, pb, w / 2.0):
                        best = (d, pa, pb)
            if best:
                break
        if best:
            break
    if not best:
        print('%s: no via-hop corridor' % name)
        return False
    d, pa, pb = best
    add_via(net, pa[0], pa[1])
    add_track(net, lay_b, pa, pb, w)
    print('%s: via-hop %.2f mm  via(%.2f,%.2f) -> (%.2f,%.2f) on L%d' %
          (name, d, pa[0], pa[1], pb[0], pb[1], lay_b), flush=True)
    return True


viahop('GND', 0, 1, 0.15, 'GND_F')
viahop('+12V', 1, 0, 0.6, '+12V_B')
viahop('+3V3', 0, 1, 0.4, '+3V3_FB')

b.Save(MAIN)
print('saved', MAIN)
