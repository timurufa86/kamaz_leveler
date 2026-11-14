import io, sys, math, shutil, heapq
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"
shutil.copy(ROOT + r"\_work\BACKUP_r8z.kicad_pcb", MAIN)   # чистое 0/4 состояние агента

b = pcbnew.LoadBoard(MAIN)


def netcode(name):
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == name:
                return p.GetNetCode()
    raise RuntimeError("net not found: " + name)


def add_track(net, pts, layer, w=0.15):
    nc = netcode(net)
    assert nc is not None
    n = 0
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        t.SetWidth(int(round(w * 1e6)))
        t.SetLayer(layer)
        t.SetNetCode(nc)
        b.Add(t)
        n += 1
    return n


def add_via(net, x, y):
    nc = netcode(net)
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(0.45 * 1e6)))
    v.SetDrill(int(round(0.2 * 1e6)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(nc)
    b.Add(v)


def zone_polys(layer, net):
    out = []
    for z in b.Zones():
        if z.GetLayer() != layer or z.GetNetname() != net:
            continue
        pl = z.GetFilledPolysList(layer)
        for i in range(pl.OutlineCount()):
            o = pl.Outline(i)
            pts = [(TOMM(o.CPoint(j).x), TOMM(o.CPoint(j).y)) for j in range(o.PointCount())]
            if len(pts) >= 3:
                out.append(pts)
    return out


def in_poly(x, y, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# 1. +3V3 F↔B: переход в сплошной меди обоих слоёв, с корректным LayerPair
F3 = zone_polys(pcbnew.F_Cu, "+3V3")
B3 = zone_polys(pcbnew.B_Cu, "+3V3")
done = False
for pf in F3:
    for pb in B3:
        ox0, ox1 = max(min(p[0] for p in pf), min(p[0] for p in pb)), min(max(p[0] for p in pf), max(p[0] for p in pb))
        oy0, oy1 = max(min(p[1] for p in pf), min(p[1] for p in pb)), min(max(p[1] for p in pf), max(p[1] for p in pb))
        if ox1 < ox0 or oy1 < oy0:
            continue
        for xx in [round(ox0 + i * 0.25, 2) for i in range(int((ox1 - ox0) / 0.25) + 1)]:
            for yy in [round(oy0 + i * 0.25, 2) for i in range(int((oy1 - oy0) / 0.25) + 1)]:
                if in_poly(xx, yy, pf) and in_poly(xx, yy, pb):
                    add_via("+3V3", xx, yy)
                    print(f"+3V3 via в ({xx};{yy})")
                    done = True
                    break
            if done:
                break
        if done:
            break
    if done:
        break
if not done:
    print("+3V3 via не найдена")

# 2. GND_F мостик: пара островов с минимальным зазором, мост с ПРОВЕРКОЙ сети
FG = zone_polys(pcbnew.F_Cu, "GND")
print(f"GND_F островов: {len(FG)}")
done = False
best = None
for i in range(len(FG)):
    for j in range(i + 1, len(FG)):
        for pi in FG[i][::5]:
            for pj in FG[j][::5]:
                d = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
                if best is None or d < best[0]:
                    best = (d, pi, pj)
if best and best[0] < 3.0:
    d, a, c = best
    add_track("GND", [a, c], pcbnew.F_Cu, 0.15)
    print(f"GND_F мостик ({a[0]:.2f};{a[1]:.2f})->({c[0]:.2f};{c[1]:.2f}) d={d:.2f}")
    done = True
if not done:
    print("GND_F мостик не найден")

# 3. +12V мост: BFS по B.Cu между двумя крупнейшими островами
P12 = zone_polys(pcbnew.B_Cu, "+12V")
print(f"+12V островов: {len(P12)}")
if len(P12) >= 2:
    P12.sort(key=len, reverse=True)
    A, C = P12[0], P12[1]
    box = (-26.5, -18.5, 3.0, 3.5)
    x0, y0, x1, y1 = box
    step = 0.2
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    starts = [A[k] for k in range(0, len(A), 3) if x0 <= A[k][0] <= x1 and y0 <= A[k][1] <= y1]
    def cidx(x, y):
        return int(round((x - x0) / step)), int(round((y - y0) / step))
    dist = [[1e9] * ny for _ in range(nx)]
    prev = [[None] * ny for _ in range(nx)]
    q = []
    for s in starts:
        si, sj = cidx(*s)
        dist[si][sj] = 0
        heapq.heappush(q, (0, si, sj))
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    found = None
    while q:
        d, ci, cj = heapq.heappop(q)
        if d > dist[ci][cj]:
            continue
        xx, yy = x0 + ci * step, y0 + cj * step
        if in_poly(xx, yy, C):
            found = (ci, cj)
            break
        for di, dj in dirs:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if dist[ni][nj] < 1e8:
                continue
            nxm, nym = x0 + ni * step, y0 + nj * step
            if not (in_poly(nxm, nym, A) or in_poly(nxm, nym, C)):
                continue
            nd = d + math.hypot(di, dj) * step
            dist[ni][nj] = nd
            prev[ni][nj] = (ci, cj)
            heapq.heappush(q, (nd, ni, nj))
    if found:
        path = [(x0 + found[0] * step, y0 + found[1] * step)]
        ci, cj = found
        while prev[ci][cj] is not None:
            ci, cj = prev[ci][cj]
            path.append((x0 + ci * step, y0 + cj * step))
        path.reverse()
        add_track("+12V", path, pcbnew.B_Cu, 0.6)
        print(f"+12V мост: {len(path)} точек")
    else:
        print("+12V мост не найден")

pcbnew.SaveBoard(MAIN, b)
print("сохранено")
