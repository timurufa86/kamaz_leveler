import io, sys, json, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
WORK = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work'
sys.path.insert(0, WORK)
import pcbnew

m = None
routes = json.load(open(WORK + r'\routes.json', encoding='utf-8'))
b = pcbnew.LoadBoard(r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb')
from pcbnew import ToMM as M
pads = {}
for f in b.GetFootprints():
    for p in f.Pads():
        pads.setdefault(p.GetNetname(), []).append((M(p.GetPosition().x), M(p.GetPosition().y),
                                                    M(p.GetSize().x) / 2, M(p.GetSize().y) / 2,
                                                    f.GetReference() + '.' + p.GetNumber(),
                                                    p.IsOnLayer(pcbnew.F_Cu), p.IsOnLayer(pcbnew.B_Cu)))
LAY = {0: 'F', 1: 'B'}


def inside(pad, x, y, depth=0.0):
    px, py, hx, hy = pad[0], pad[1], pad[2], pad[3]
    qx = abs(x - px) - hx
    qy = abs(y - py) - hy
    d = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
    return d <= -depth


def dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    t = 0 if L2 < 1e-12 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


bad = 0
for (net, segs, vias) in routes:
    for i, s in enumerate(segs):
        x1, y1, x2, y2, h, L = s
        for (ex, ey) in ((x1, y1), (x2, y2)):
            if any(inside(p, ex, ey) and (p[5] if L == 0 else p[6]) for p in pads.get(net, [])):
                continue
            if any(math.hypot(ex - vx, ey - vy) < 1e-6 for v in vias for (vx, vy) in [v]):
                continue
            ok = False
            for j, t in enumerate(segs):
                if j == i or t[5] != L:
                    continue
                if dseg(ex, ey, t[0], t[1], t[2], t[3]) < 0.01:
                    ok = True
                    break
            if ok:
                continue
            bad += 1
            near = min(((dseg(ex, ey, t[0], t[1], t[2], t[3]), t) for j, t in enumerate(segs)
                        if j != i and t[5] == L), default=(99, None))
            print('DANGLING %-12s seg%d %s layer %s end(%.3f,%.3f) len %.3f near=%.3f' %
                  (net, i, 'start' if (ex == x1 and ey == y1) else 'end', LAY[L], ex, ey,
                   math.hypot(x2 - x1, y2 - y1), near[0]))
print('total dangling', bad)
