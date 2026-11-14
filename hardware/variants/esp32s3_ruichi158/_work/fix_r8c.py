import io, sys, math, shutil, heapq
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"

shutil.copy(ROOT + r"\_work\BACKUP_r8.kicad_pcb", MAIN)   # 0/6 состояние
b = pcbnew.LoadBoard(MAIN)


def netcode(name):
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == name:
                return p.GetNetCode()
    return None


def add_track(net, pts, layer, w=0.15):
    nc = netcode(net)
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        t.SetWidth(int(round(w * 1e6)))
        t.SetLayer(layer)
        t.SetNetCode(nc)
        b.Add(t)


# 1. U7.23 GND (проверено)
add_track("GND", [(11.43, 40.71), (13.30, 40.71), (13.30, 34.00)], pcbnew.F_Cu)
print("U7.23 GND добавлен")

# модель препятствий
m = rr.Model(MAIN)
m.build()
tracks = m.tracks          # per layer: list of (x1,y1,x2,y2,h)
pads = m.pads
CLR = rr.CLR                # 0.127?


def blocked(x, y, net):
    # foreign tracks (обе стороны) — дистанция от центра клетки до отрезка
    hw = 0.075
    for L in (0, 1):
        for (x1, y1, x2, y2, h) in tracks[L]:
            dx, dy = x2 - x1, y2 - y1
            l2 = dx * dx + dy * dy
            tt = 0 if l2 < 1e-12 else max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / l2))
            if math.hypot(x - (x1 + tt * dx), y - (y1 + tt * dy)) < h + hw + CLR - 0.01:
                return True
    # foreign pads
    for q in pads:
        if q['net'] == net:
            continue
        u = q['c'] * (x - q['x']) + q['s'] * (y - q['y'])
        v = -q['s'] * (x - q['x']) + q['c'] * (y - q['y'])
        lim = CLR + hw + 0.02
        if abs(u) < q['hx'] + lim and abs(v) < q['hy'] + lim:
            return True
    return False


def in_zone_foreign(x, y, layer, net):
    for z in b.Zones():
        if z.GetLayer() != layer or z.GetNetname() == net:
            continue
        pl = z.GetFilledPolysList(layer)
        for i in range(pl.OutlineCount()):
            out = pl.Outline(i)
            pts = [(TOMM(out.CPoint(j).x), TOMM(out.CPoint(j).y)) for j in range(out.PointCount())]
            if len(pts) < 3:
                continue
            x0 = min(p[0] for p in pts) - 0.3
            x1 = max(p[0] for p in pts) + 0.3
            y0 = min(p[1] for p in pts) - 0.3
            y1 = max(p[1] for p in pts) + 0.3
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            inside = False
            j = len(pts) - 1
            for k in range(len(pts)):
                xi, yi = pts[k]
                xj, yj = pts[j]
                if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                    inside = not inside
                j = k
            if inside:
                return True
    return False


def bfs_connect(net, start, goal, box, layer=pcbnew.F_Cu, step=0.1):
    x0, y0, x1, y1 = box
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    gx, gy = goal
    def cidx(x, y):
        return int(round((x - x0) / step)), int(round((y - y0) / step))
    si, sj = cidx(*start)
    gi, gj = cidx(gx, gy)
    if not (0 <= si < nx and 0 <= sj < ny):
        return None
    # цель: рядом с падом (внутри достижимости пада)
    gpad = None
    for q in pads:
        if q['net'] == net:
            u = q['c'] * (gx - q['x']) + q['s'] * (gy - q['y'])
            v = -q['s'] * (gx - q['x']) + q['c'] * (gy - q['y'])
            if abs(u) < q['hx'] + 0.1 and abs(v) < q['hy'] + 0.1:
                gpad = q
                break
    if gpad is None:
        return None
    dist = [[1e9] * ny for _ in range(nx)]
    prev = [[None] * ny for _ in range(nx)]
    dist[si][sj] = 0
    q = [(0, si, sj)]
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    found = None
    while q:
        d, ci, cj = heapq.heappop(q)
        if d > dist[ci][cj]:
            continue
        x, y = x0 + ci * step, y0 + cj * step
        if gpad is not None:
            u = gpad['c'] * (x - gpad['x']) + gpad['s'] * (y - gpad['y'])
            v = -gpad['s'] * (x - gpad['x']) + gpad['c'] * (y - gpad['y'])
            if abs(u) < gpad['hx'] + 0.05 and abs(v) < gpad['hy'] + 0.05:
                found = (ci, cj)
                break
        for di, dj in dirs:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if dist[ni][nj] < 1e8:
                continue
            nxm, nym = x0 + ni * step, y0 + nj * step
            if blocked(nxm, nym, net):
                continue
            nd = d + math.hypot(di, dj) * step
            dist[ni][nj] = nd
            prev[ni][nj] = (ci, cj)
            heapq.heappush(q, (nd, ni, nj))
    if found is None:
        return None
    path = [(x0 + found[0] * step, y0 + found[1] * step)]
    ci, cj = found
    while (ci, cj) != (si, sj):
        ci, cj = prev[ci][cj]
        path.append((x0 + ci * step, y0 + cj * step))
    path.reverse()
    # упрощение: жадное line-of-sight
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if not any(blocked(x0 + k * step, y0 + k * step, net) for k in []):
                pass
            j -= 1
        out.append(path[j])
        i = j
    return out


def simplify(path, net, layer):
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            x1, y1 = path[i]
            x2, y2 = path[j]
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.1) + 1)
            ok = True
            for k in range(1, n):
                x = x1 + (x2 - x1) * k / n
                y = y1 + (y2 - y1) * k / n
                if blocked(x, y, net):
                    ok = False
                    break
            if ok:
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


for net, start, goal, box in [
    ("/BUT3", (-16.30, 18.95), (-20.117, 13.87), (-21.6, 12.8, -15.4, 20.2)),
    ("/BUT4", (-21.80, 23.05), (-20.117, 14.52), (-23.0, 13.5, -19.4, 24.5)),
]:
    path = bfs_connect(net, start, goal, box)
    if path:
        path = simplify(path, net, pcbnew.F_Cu)
        add_track(net, path, pcbnew.F_Cu)
        print(f"{net}: путь из {len(path)} точек")
    else:
        print(f"{net}: путь НЕ найден")

pcbnew.SaveBoard(MAIN, b)
print("сохранено")
