import io
import sys
import json
import math
import os
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
from rr import Model
import statistics

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
GEO = os.path.join(WORK, 'routed_geo.json')
SRC = os.path.join(ROOT, 'routed5.kicad_pcb')

g = json.load(open(GEO, encoding='utf-8'))
m = Model(SRC)
m.build()
segs_of = {n: [(float(s[0]), float(s[1]), float(s[2]), float(s[3]), float(s[4]), int(float(s[5])))
               for s in ss] for n, ss in g['segs'].items()}
vias_of = {n: [tuple(float(x) for x in v) for v in g['vias'].get(n, [])] for n in segs_of}

before = sum(len(v) for v in segs_of.values())


def key(x, y):
    return (round(x, 3), round(y, 3))


total_merged = 0
for net, segs in segs_of.items():
    hw = segs[0][4] if segs else 0.075
    changed = True
    it = 0
    while changed and it < 60:
        changed = False
        it += 1
        # vertex -> list of (index, which)
        V = {}
        for i, (x1, y1, x2, y2, h, L) in enumerate(segs):
            V.setdefault(key(x1, y1), []).append((i, 0))
            V.setdefault(key(x2, y2), []).append((i, 1))
        for v, lst in V.items():
            if len(lst) != 2:
                continue
            (i1, w1), (i2, w2) = lst
            if i1 == i2:
                continue
            s1, s2 = segs[i1], segs[i2]
            if s1[5] != s2[5]:
                continue
            p1 = (s1[2], s1[3]) if w1 == 0 else (s1[0], s1[1])
            p2 = (s2[2], s2[3]) if w2 == 0 else (s2[0], s2[1])
            a = math.hypot(s1[2] - s1[0], s1[3] - s1[1])
            b = math.hypot(s2[2] - s2[0], s2[3] - s2[1])
            if a < 1e-6 or b < 1e-6:
                continue
            if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < 1e-6:
                continue
            if not m.seg_clear(p1[0], p1[1], p2[0], p2[1], hw, s1[5], net):
                continue
            segs[i1] = (p1[0], p1[1], p2[0], p2[1], s1[4], s1[5])
            del segs[i2]
            total_merged += 1
            changed = True
            break
    segs_of[net] = segs

after = sum(len(v) for v in segs_of.values())
print('segments %d -> %d (merged %d)' % (before, after, total_merged))
lens = sorted(math.hypot(s[2] - s[0], s[3] - s[1]) for v in segs_of.values() for s in v)
print('median %.3f  p10 %.3f  p90 %.3f' % (statistics.median(lens), lens[len(lens) // 10],
                                            lens[9 * len(lens) // 10]))
for n in sorted(segs_of, key=lambda x: -len(segs_of[x]))[:6]:
    print('  %-12s %d segs' % (n, len(segs_of[n])))
json.dump({'segs': {n: [[float(v) for v in q] for q in segs_of[n]] for n in segs_of},
           'vias': {n: [[float(v) for v in q] for q in vias_of[n]] for n in segs_of},
           'failed': g.get('failed', [])}, open(GEO, 'w'))
print('saved', GEO)
