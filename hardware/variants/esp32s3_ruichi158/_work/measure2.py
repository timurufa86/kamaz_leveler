import io
import sys
import os
import math
import hashlib
import collections
import statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
from pcbnew import ToMM as M

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'zoned.kicad_pcb')
BAK = os.path.join(ROOT, '_work', 'BACKUP_before_route.kicad_pcb')
ANT = (-13.0, 36.0, 13.0, 42.75)
SLOTS = [(27.0, 29.0), (-27.0, 29.0), (27.0, -29.0), (-27.0, -29.0)]
RS = 3.75


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
print('footprints %d pads %d segs %d vias %d' % (len(fps), len(pads), len(segs), len(vias)))
if lens:
    print('copper %.1f mm  median %.3f  p10 %.3f  p90 %.3f  min %.3f  max %.2f' %
          (sum(lens), statistics.median(lens), lens[len(lens) // 10],
           lens[9 * len(lens) // 10], lens[0], lens[-1]))
per = collections.defaultdict(lambda: [0, 0, 0.0, 0.0])
for t in segs:
    e = per[t.GetNetname()]
    e[0] += 1
    e[2] += M(t.GetLength())
    e[3] = max(e[3], M(t.GetWidth()))
for v in vias:
    per[v.GetNetname()][1] += 1
mx = max(per.items(), key=lambda kv: kv[1][0])
print('max segments/net: %s = %d' % (mx[0], mx[1][0]))
PAD = {'/DISP_BL': 4, '/DISP_CS': 5, '/DISP_DC': 9, '/DISP_RES': 10, '/DISP_SCLK': 11,
       '/SPEED_PULSE': 12, '/DISP_MOSI': 17, '/INT_MCU': 19, '/VLV_EN': 20, '/PRES_SW': 32,
       '/I2C_SCL': 39, '/I2C_SDA': 40}
for n in ('/DISP_BL', '/DISP_CS', '/DISP_DC', '/DISP_RES', '/DISP_SCLK', '/SPEED_PULSE',
          '/DISP_MOSI', '/INT_MCU', '/VLV_EN', '/PRES_SW', '/I2C_SCL', '/I2C_SDA'):
    e = per[n]
    print('| %s | %d | %d | %d | %.3f | w%.2f' % (n, PAD[n], e[0], e[1], e[2], e[3]))
print('| +3V3_ESP | 1-2 | %d | %d | %.3f |' % tuple(per['+3V3_ESP'][:3]))

ant_t = []
for t in segs:
    x1, y1 = M(t.GetStart().x), M(t.GetStart().y)
    x2, y2 = M(t.GetEnd().x), M(t.GetEnd().y)
    h = M(t.GetWidth()) / 2
    if max(x1, x2) + h > ANT[0] and min(x1, x2) - h < ANT[2] and \
            max(y1, y2) + h > ANT[1] and min(y1, y2) - h < ANT[3]:
        ant_t.append(t)
ant_v = [v for v in vias if ANT[0] <= M(v.GetPosition().x) <= ANT[2] and ANT[1] <= M(v.GetPosition().y) <= ANT[3]]
ant_p = [p for p in pads if ANT[0] <= M(p.GetPosition().x) <= ANT[2] and ANT[1] <= M(p.GetPosition().y) <= ANT[3]]
zverts = 0
zant = 0
zp = 0
for z in b.Zones():
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        zp = ps.OutlineCount()
        for k in range(ps.OutlineCount()):
            for ch in [ps.Outline(k)] + [ps.Hole(k, hh) for hh in range(ps.HoleCount(k))]:
                for i in range(ch.PointCount()):
                    zverts += 1
                    x, y = M(ch.CPoint(i).x), M(ch.CPoint(i).y)
                    if ANT[0] <= x <= ANT[2] and ANT[1] <= y <= ANT[3]:
                        zant += 1
print('antenna strip: tracks %d vias %d fill_verts %d pads_inside %d %s' %
      (len(ant_t), len(ant_v), zant, len(ant_p),
       sorted(set('%s.%s' % (p.GetParentFootprint().GetReference(), p.GetNumber()) for p in ant_p))))
print('antenna track detail:', [(t.GetNetname(), round(M(t.GetLength()), 2)) for t in ant_t])

worst = 1e9
what = ''
for (sx, sy) in SLOTS:
    for t in segs:
        x1, y1 = M(t.GetStart().x), M(t.GetStart().y)
        x2, y2 = M(t.GetEnd().x), M(t.GetEnd().y)
        if max(abs(x1 - sx), abs(x2 - sx)) > 8 or max(abs(y1 - sy), abs(y2 - sy)) > 8:
            continue
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
        for k in range(n + 1):
            f = k / n
            d = d_arc(x1 + (x2 - x1) * f, y1 + (y2 - y1) * f, sx, sy) - M(t.GetWidth()) / 2
            if d < worst:
                worst, what = d, 'track ' + t.GetNetname()
    for v in vias:
        x, y = M(v.GetPosition().x), M(v.GetPosition().y)
        if abs(x - sx) > 8 or abs(y - sy) > 8:
            continue
        d = d_arc(x, y, sx, sy) - M(v.GetWidth(pcbnew.F_Cu)) / 2
        if d < worst:
            worst, what = d, 'via ' + v.GetNetname()
    for z in b.Zones():
        for lay in z.GetLayerSet().Seq():
            ps = z.GetFilledPolysList(lay)
            for k in range(ps.OutlineCount()):
                ch = ps.Outline(k)
                for i in range(ch.PointCount()):
                    x, y = M(ch.CPoint(i).x), M(ch.CPoint(i).y)
                    if abs(x - sx) > 8 or abs(y - sy) > 8:
                        continue
                    d = d_arc(x, y, sx, sy)
                    if d < worst:
                        worst, what = d, 'zone ' + z.GetNetname()
    for p in pads:
        x, y = M(p.GetPosition().x), M(p.GetPosition().y)
        if abs(x - sx) > 8 or abs(y - sy) > 8:
            continue
        r = math.hypot(M(p.GetSize().x), M(p.GetSize().y)) / 2
        d = d_arc(x, y, sx, sy) - r
        if d < worst:
            worst, what = d, 'pad %s.%s' % (p.GetParentFootprint().GetReference(), p.GetNumber())
print('slots: min copper-to-arc %.3f mm (%s)' % (worst, what), flush=True)

b0 = pcbnew.LoadBoard(BAK)
pos0 = {f.GetReference(): (M(f.GetPosition().x), M(f.GetPosition().y), f.GetOrientationDegrees())
        for f in b0.GetFootprints()}
moved = []
for f in fps:
    p = (M(f.GetPosition().x), M(f.GetPosition().y), f.GetOrientationDegrees())
    q = pos0.get(f.GetReference())
    if q and (abs(p[0] - q[0]) > 1e-4 or abs(p[1] - q[1]) > 1e-4 or abs(p[2] - q[2]) > 1e-4):
        moved.append(f.GetReference())
print('moved footprints: %d %s' % (len(moved), moved))
print('zones:')
for z in b.Zones():
    for lay in z.GetLayerSet().Seq():
        ps = z.GetFilledPolysList(lay)
        print('  %-6s %-4s polys %d verts %d' % (z.GetNetname(), b.GetLayerName(lay),
                                                 ps.OutlineCount(), zverts))
print('zone verts total %d (antenna %d), fill polygons %d' % (zverts, zant, zp))
print('sha256 %s  %s' % (hashlib.sha256(open(PATH, 'rb').read()).hexdigest(), os.path.basename(PATH)))
print('sha256 %s  %s' % (hashlib.sha256(open(BAK, 'rb').read()).hexdigest(), os.path.basename(BAK)))
