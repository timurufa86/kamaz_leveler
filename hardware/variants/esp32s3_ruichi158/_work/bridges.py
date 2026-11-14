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
log = []


def islands(netname, layer):
    out = []
    for z in b.Zones():
        if z.GetNetname() != netname or z.GetLayer() != layer:
            continue
        ps = z.GetFilledPolysList(layer)
        try:
            n = ps.OutlineCount()
        except Exception:
            n = 0
        for i in range(n):
            pts = []
            try:
                chain = ps.Outline(i)
                for k in range(chain.PointCount()):
                    q = chain.CPoint(k)
                    pts.append((q.x / 1e6, q.y / 1e6))
            except Exception:
                continue
            if len(pts) > 2:
                out.append(pts)
    return out


# --- (b) bridge the +12V islands on B.Cu with one 0.6 mm track
isl = sorted(islands('+12V', pcbnew.B_Cu), key=len, reverse=True)
print('+12V B.Cu islands:', [len(p) for p in isl][:6], flush=True)
if len(isl) >= 2:
    best = None
    for i in range(min(6, len(isl))):
        for j in range(i + 1, min(6, len(isl))):
            for a in isl[i][::2]:
                for c in isl[j][::2]:
                    d = math.hypot(a[0] - c[0], a[1] - c[1])
                    if best is None or d < best[0]:
                        best = (d, a, c)
    if best and best[0] < 25.0:
        d, a, c = best
        print('+12V bridge %.2f mm: (%.2f,%.2f) -> (%.2f,%.2f)' % (d, a[0], a[1], c[0], c[1]), flush=True)
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(int(round(a[0] * 1e6)), int(round(a[1] * 1e6))))
        s.SetEnd(pcbnew.VECTOR2I(int(round(c[0] * 1e6)), int(round(c[1] * 1e6))))
        s.SetWidth(int(round(0.6e6)))
        s.SetLayer(pcbnew.B_Cu)
        s.SetNetCode(nmap['+12V'])
        b.Add(s)
        m.add_copper([(a[0], a[1], c[0], c[1], 0.3, 1)], [])

# --- (c) +3V3 F<->B stitching via inside both fills, clear of other copper
fF = islands('+3V3', pcbnew.F_Cu)
fB = islands('+3V3', pcbnew.B_Cu)
placed = None
if fF and fB:
    xs = [p[0] for p in fF[0]]
    ys = [p[1] for p in fF[0]]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    cand = []
    penF, penB = m.pen[0], m.pen[1]
    thr = -(rr.CLR + rr.VIA_R + 0.05)
    step = 1.0
    y = y0
    while y < y1 and len(cand) < 40:
        x = x0
        while x < x1:
            i, j = rr.ix(x), rr.iy(y)
            if 0 <= i < rr.NX and 0 <= j < rr.NY:
                if penF[j, i] < thr and penB[j, i] < thr:
                    cand.append((x, y))
            x += step
        y += step
    cand.sort(key=lambda t: (abs(t[0] - 0) + abs(t[1] - 15)))
    print('free via candidates inside +3V3 box:', len(cand), cand[:4], flush=True)
    if cand:
        x, y = cand[0]
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(rr.VIA_D * 1e6)))
        v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(nmap['+3V3'])
        b.Add(v)
        m.add_copper([], [(x, y)])
        placed = (x, y)
        print('+3V3 stitch via at (%.2f,%.2f)' % (x, y), flush=True)

b.Save(MAIN)
print('saved', MAIN)
