import pcbnew
import collections
import math
import json

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = ROOT + r'\kamaz_leveler_ruichi158.kicad_pcb'
b = pcbnew.LoadBoard(MAIN)
pos, half, kind, fixed = {}, {}, {}, set()
pads = collections.defaultdict(set)
for f in b.GetFootprints():
    r = f.GetReference()
    p = f.GetPosition()
    pos[r] = [p.x / 1e6, p.y / 1e6]
    bb = f.GetBoundingBox(False, False)
    half[r] = [bb.GetWidth() / 2e6 + 0.25, bb.GetHeight() / 2e6 + 0.25]
    kind[r] = str(f.GetFPID().GetLibItemName())
    if r.startswith('J') or r == 'U7':
        fixed.add(r)
    for pd in f.Pads():
        nm = str(pd.GetNetname())
        if nm and nm != 'GND':
            pads[nm].add(r)
nets = [[r for r in rs if r in pos] for rs in pads.values() if len(rs) > 1]
XMIN, XMAX, YMIN, YMAX = -29.0, 29.0, -41.5, 41.5


def hpwl(ps):
    t = 0.0
    for rs in nets:
        xs = [ps[r][0] for r in rs]
        ys = [ps[r][1] for r in rs]
        t += (max(xs) - min(xs)) + (max(ys) - min(ys))
    return t


cur = {k: list(v) for k, v in pos.items()}
base = hpwl(cur)


def overlaps(ps):
    bad = 0
    rs = list(ps)
    for i in range(len(rs)):
        for j in range(i + 1, len(rs)):
            a, c = rs[i], rs[j]
            if a == c:
                continue
            dx = abs(ps[a][0] - ps[c][0])
            dy = abs(ps[a][1] - ps[c][1])
            if dx < half[a][0] + half[c][0] and dy < half[a][1] + half[c][1]:
                bad += 1
    return bad


print('base HPWL %.1f  overlaps %d' % (base, overlaps(cur)), flush=True)
for it in range(300):
    # attraction to the net centroid
    for r in cur:
        if r in fixed:
            continue
        cx = cy = 0.0
        w = 0
        for rs in nets:
            if r in rs:
                o = [q for q in rs if q != r]
                if o:
                    cx += sum(cur[q][0] for q in o) / len(o)
                    cy += sum(cur[q][1] for q in o) / len(o)
                    w += 1
        if not w:
            continue
        k = 0.35 if it < 150 else 0.15
        cur[r][0] += k * (cx / w - cur[r][0])
        cur[r][1] += k * (cy / w - cur[r][1])
        cur[r][0] = min(XMAX, max(XMIN, cur[r][0]))
        cur[r][1] = min(YMAX, max(YMIN, cur[r][1]))
    # repulsion between any overlapping pair
    rs = list(cur)
    for i in range(len(rs)):
        for j in range(i + 1, len(rs)):
            a, c = rs[i], rs[j]
            if a in fixed and c in fixed:
                continue
            dx = cur[a][0] - cur[c][0]
            dy = cur[a][1] - cur[c][1]
            ox = half[a][0] + half[c][0] - abs(dx)
            oy = half[a][1] + half[c][1] - abs(dy)
            if ox > 0 and oy > 0:
                if ox < oy:
                    s = (1 if dx >= 0 else -1) * ox * 0.5
                    if a not in fixed:
                        cur[a][0] += s
                    if c not in fixed:
                        cur[c][0] -= s
                else:
                    s = (1 if dy >= 0 else -1) * oy * 0.5
                    if a not in fixed:
                        cur[a][1] += s
                    if c not in fixed:
                        cur[c][1] -= s
    if it % 50 == 0:
        print('  it %3d HPWL %.1f overlaps %d' % (it, hpwl(cur), overlaps(cur)), flush=True)

print('final HPWL %.1f (base %.1f, gain %.1f%%)  overlaps %d' %
      (hpwl(cur), base, 100 * (base - hpwl(cur)) / base, overlaps(cur)), flush=True)
json.dump({r: [round(cur[r][0], 2), round(cur[r][1], 2)] for r in cur},
          open(ROOT + r'\_work\place_v2.json', 'w'))
print('saved _work/place_v2.json')
