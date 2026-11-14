import io
import sys
import os
import math
import json
import hashlib
import collections
import statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
from pcbnew import ToMM as M

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
BAK = os.path.join(ROOT, '_work', 'BACKUP_before_route.kicad_pcb')
ANT = (-13.0, 36.0, 13.0, 42.75)
SLOTS = [(27.0, 29.0), (-27.0, 29.0), (27.0, -29.0), (-27.0, -29.0)]
RS = 3.75


def d_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def d_arc(px, py, cx_, cy_):
    a = math.atan2(py - cy_, px - cx_)
    return math.hypot(px - (cx_ + RS * math.cos(a)), py - (cy_ + RS * math.sin(a)))


b = pcbnew.LoadBoard(PATH)
fps = list(b.GetFootprints())
pads = [p for f in fps for p in f.Pads()]
tracks = list(b.GetTracks())
segs = [t for t in tracks if t.Type() == pcbnew.PCB_TRACE_T]
vias = [t for t in tracks if t.Type() == pcbnew.PCB_VIA_T]
lens = sorted(M(t.GetLength()) for t in segs)
print('footprints %d  pads %d  tracks %d  vias %d' % (len(fps), len(pads), len(segs), len(vias)))
if lens:
    print('segments %d  copper %.1f mm  median %.3f  p10 %.3f  p90 %.3f  min %.3f max %.2f' %
          (len(lens), sum(lens), statistics.median(lens), lens[len(lens) // 10],
           lens[9 * len(lens) // 10], lens[0], lens[-1]))
per = collections.defaultdict(lambda: [0, 0, 0.0])
for t in segs:
    e = per[t.GetNetname()]
    e[0] += 1
    e[2] += M(t.GetLength())
for v in vias:
    per[v.GetNetname()][1] += 1
mx = max(per.items(), key=lambda kv: kv[1][0])
print('max segments per net: %s %d' % (mx[0], mx[1][0]))
print('| цепь | пад | отрезков | переходов | длина, мм |')
PAD = {'/DISP_BL': 4, '/DISP_CS': 5, '/DISP_DC': 9, '/DISP_RES': 10, '/DISP_SCLK': 11,
       '/SPEED_PULSE': 12, '/DISP_MOSI': 17, '/INT_MCU': 19, '/VLV_EN': 20, '/PRES_SW': 32,
       '/I2C_SCL': 39, '/I2C_SDA': 40}
for n in ('/DISP_BL', '/DISP_CS', '/DISP_DC', '/DISP_RES', '/DISP_SCLK', '/SPEED_PULSE',
          '/DISP_MOSI', '/INT_MCU', '/VLV_EN', '/PRES_SW', '/I2C_SCL', '/I2C_SDA'):
    e = per[n]
    print('| %s | %d | %d | %d | %.3f |' % (n, PAD[n], e[0], e[1], e[2]))
print('| +3V3_ESP | 1-2 | %d | %d | %.3f |' % tuple(per['+3V3_ESP']))

# antenna strip
def box_hit(x1, y1, x2, y2, h):
    i0, i1, j0, j1 = ANT
    for s in range(21):
        f = s / 20.0
        x, y = x1 + (x2 - x1) * f, y1 + (y2 - y1) * f
        if x - h < i1 and x + h > i0 and y - h < j1 and y + h > j0:
            if i0 <= x <= i1 and j0 <= y <= j1:
                return True
    return False


ant_t = [t for t in segs if box_hit(M(t.GetStart().x), M(t.GetStart().y),
                                    M(t.GetEnd().x), M(t.GetEnd().y), M(t.GetWidth()) / 2)]
ant_v = [v for v in vias if ANT[0] <= M(v.GetPosition().x) <= ANT[2] and ANT[1] <= M(v.GetPosition().y) <= ANT[3]]
ant_p = [p for p in pads if ANT[0] <= M(p.GetPosition().x) <= ANT[2] and ANT[1] <= M(p.GetPosition().y) <= ANT[3]]
zverts = 0
zverts_ant = 0
for z in b.Zones():
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        for k in range(ps.OutlineCount()):
            ch = ps.Outline(k)
            for t in range(ch.PointCount()):
                zverts += 1
                x, y = M(ch.CPoint(t).x), M(ch.CPoint(t).y)
                if ANT[0] <= x <= ANT[2] and ANT[1] <= y <= ANT[3]:
                    zverts_ant += 1
            for hh in range(ps.HoleCount(k)):
                ch = ps.Hole(k, hh)
                for t in range(ch.PointCount()):
                    zverts += 1
                    x, y = M(ch.CPoint(t).x), M(ch.CPoint(t).y)
                    if ANT[0] <= x <= ANT[2] and ANT[1] <= y <= ANT[3]:
                        zverts_ant += 1
print('antenna strip: tracks %d (len %.2f) vias %d fill_vertices %d  pads_inside %d %s' %
      (len(ant_t), sum(M(t.GetLength()) for t in ant_t), len(ant_v), zverts_ant, len(ant_p),
       sorted(set('%s.%s' % (p.GetParentFootprint().GetReference(), p.GetNumber()) for p in ant_p))))

# slots
worst = 1e9
what = ''
for (sx, sy) in SLOTS:
    for t in segs:
        x1, y1 = M(t.GetStart().x), M(t.GetStart().y)
        x2, y2 = M(t.GetEnd().x), M(t.GetEnd().y)
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
        for s in range(n + 1):
            f = s / n
            d = d_arc(x1 + (x2 - x1) * f, y1 + (y2 - y1) * f, sx, sy) - M(t.GetWidth()) / 2
            if d < worst:
                worst = d
                what = 'track %s' % t.GetNetname()
    for v in vias:
        d = d_arc(M(v.GetPosition().x), M(v.GetPosition().y), sx, sy) - M(v.GetWidth()) / 2
        if d < worst:
            worst = d
            what = 'via %s' % v.GetNetname()
    for p in pads:
        cx_, cy_ = M(p.GetPosition().x), M(p.GetPosition().y)
        r = math.hypot(M(p.GetSize().x) / 2, M(p.GetSize().y) / 2)
        d = d_arc(cx_, cy_, sx, sy) - r
        if d < worst:
            worst = d
            what = 'pad %s.%s' % (p.GetParentFootprint().GetReference(), p.GetNumber())
    for z in b.Zones():
        for lay in z.GetLayerSet().Seq():
            ps = z.GetFilledPolysList(lay)
            for k in range(ps.OutlineCount()):
                ch = ps.Outline(k)
                for t in range(ch.PointCount()):
                    d = d_arc(M(ch.CPoint(t).x), M(ch.CPoint(t).y), sx, sy)
                    if d < worst:
                        worst = d
                        what = 'zone %s' % z.GetNetname()
print('slots: min copper distance to arc %.3f mm (%s)' % (worst, what))

# footprint moves
b0 = pcbnew.LoadBoard(BAK)
pos0 = {f.GetReference(): (M(f.GetPosition().x), M(f.GetPosition().y), f.GetOrientationDegrees()) for f in b0.GetFootprints()}
moved = []
for f in fps:
    p = (M(f.GetPosition().x), M(f.GetPosition().y), f.GetOrientationDegrees())
    q = pos0.get(f.GetReference())
    if q and (abs(p[0] - q[0]) > 1e-4 or abs(p[1] - q[1]) > 1e-4 or abs(p[2] - q[2]) > 1e-4):
        moved.append((f.GetReference(), q, p))
print('moved footprints: %d %s' % (len(moved), moved))
print('zone fill vertices total %d' % zverts)
for z in b.Zones():
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        print('  zone %-6s %-4s polys %2d' % (z.GetNetname(), b.GetLayerName(lay), ps.OutlineCount()))
print('sha256 %s %s' % (hashlib.sha256(open(PATH, 'rb').read()).hexdigest(), os.path.basename(PATH)))
print('sha256 %s %s' % (hashlib.sha256(open(BAK, 'rb').read()).hexdigest(), os.path.basename(BAK)))
