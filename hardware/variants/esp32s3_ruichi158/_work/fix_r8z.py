import io, sys, math, shutil, heapq
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"
shutil.copy(MAIN, ROOT + r"\_work\BACKUP_r8z.kicad_pcb")

b = pcbnew.LoadBoard(MAIN)
m = rr.Model(MAIN)
m.build()
tracks = m.tracks
pads = m.pads
CLR = rr.CLR


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


def add_via(net, x, y):
    nc = netcode(net)
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(0.45 * 1e6)))
    v.SetDrill(int(round(0.2 * 1e6)))
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


def solid(pts, x, y, r=0.15):
    for dx in (-r, 0, r):
        for dy in (-r, 0, r):
            if not in_poly(x + dx, y + dy, pts):
                return False
    return True


def foreign_near(x, y, net, r=0.5, L=None):
    for li in ((0, 1) if L is None else (L,)):
        for (x1, y1, x2, y2, h) in tracks[li]:
            dx, dy = x2 - x1, y2 - y1
            l2 = dx * dx + dy * dy
            tt = 0 if l2 < 1e-12 else max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / l2))
            if math.hypot(x - (x1 + tt * dx), y - (y1 + tt * dy)) < h + r:
                return True
    for q in pads:
        if q['net'] == net:
            continue
        on = True if L is None else (q['onF'] if L == 0 else q['onB'])
        if not on:
            continue
        u = q['c'] * (x - q['x']) + q['s'] * (y - q['y'])
        v = -q['s'] * (x - q['x']) + q['c'] * (y - q['y'])
        if abs(u) < q['hx'] + r and abs(v) < q['hy'] + r:
            return True
    return False


# --- 1. +3V3 F↔B via ---
F3 = zone_polys(pcbnew.F_Cu, "+3V3")
B3 = zone_polys(pcbnew.B_Cu, "+3V3")
print(f"+3V3: F островов {len(F3)}, B островов {len(B3)}")
via_done = False
for pf in F3:
    xf0, xf1 = min(p[0] for p in pf), max(p[0] for p in pf)
    yf0, yf1 = min(p[1] for p in pf), max(p[1] for p in pf)
    for pb in B3:
        xb0, xb1 = min(p[0] for p in pb), max(p[0] for p in pb)
        yb0, yb1 = min(p[1] for p in pb), max(p[1] for p in pb)
        ox0, ox1 = max(xf0, xb0), min(xf1, xb1)
        oy0, oy1 = max(yf0, yb0), min(yf1, yb1)
        if ox1 < ox0 or oy1 < oy0:
            continue
        for xx in [ox0 + i * 0.2 for i in range(int((ox1 - ox0) / 0.2) + 1)]:
            for yy in [oy0 + i * 0.2 for i in range(int((oy1 - oy0) / 0.2) + 1)]:
                if solid(pf, xx, yy) and solid(pb, xx, yy) and not foreign_near(xx, yy, "+3V3", 0.5):
                    add_via("+3V3", xx, yy)
                    print(f"+3V3 via в ({xx:.2f};{yy:.2f})")
                    via_done = True
                    break
            if via_done:
                break
        if via_done:
            break
    if via_done:
        break
if not via_done:
    print("+3V3 via НЕ найдена")

# --- 2. GND_F мостик между островами ---
FG = zone_polys(pcbnew.F_Cu, "GND")
print(f"GND_F островов: {len(FG)}")
# найти пару островов с зазором < 2 мм и соединить мостиком
bridge_done = False
for i in range(len(FG)):
    for j in range(i + 1, len(FG)):
        # ищем ближайшие точки
        best = None
        for pi in FG[i][::7]:
            for pj in FG[j][::7]:
                d = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
                if best is None or d < best[0]:
                    best = (d, pi, pj)
        d, a, bpt = best
        if d < 2.5:
            # проверить коридор
            n = max(2, int(d / 0.05) + 1)
            ok = True
            for k in range(n + 1):
                x = a[0] + (bpt[0] - a[0]) * k / n
                y = a[1] + (bpt[1] - a[1]) * k / n
                if foreign_near(x, y, "GND", 0.2, L=0):
                    ok = False
                    break
            if ok:
                add_track("GND", [a, bpt], pcbnew.F_Cu, 0.15)
                print(f"GND_F мостик: ({a[0]:.2f};{a[1]:.2f}) -> ({bpt[0]:.2f};{bpt[1]:.2f}) d={d:.2f}")
                bridge_done = True
                break
    if bridge_done:
        break
if not bridge_done:
    print("GND_F мостик не найден")

# --- 3. +12V мост: BFS по B.Cu между островами ---
P12 = zone_polys(pcbnew.B_Cu, "+12V")
print(f"+12V B островов: {len(P12)}")


def in_any(x, y, polys):
    return any(in_poly(x, y, p) for p in polys)


# клетка считается допустимой для моста: внутри одного из островов ИЛИ вне чужой меди
def cell_ok(x, y):
    if foreign_near(x, y, "+12V", 0.35, L=1):
        return False
    return True


bridge_done = False
if len(P12) >= 2:
    # берём два крупнейших острова
    P12.sort(key=len, reverse=True)
    A, Bp = P12[0], P12[1]
    box = (-26.0, -20.0, 2.0, 5.0)
    x0, y0, x1, y1 = box
    step = 0.2
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    # старты: точки A у границы к B, цели: точки B
    starts = [A[k] for k in range(0, len(A), 5) if x0 <= A[k][0] <= x1 and y0 <= A[k][1] <= y1]
    goals = [Bp[k] for k in range(0, len(Bp), 5) if x0 <= Bp[k][0] <= x1 and y0 <= Bp[k][1] <= y1]
    if starts and goals:
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
            if in_poly(xx, yy, Bp):
                found = (ci, cj)
                break
            for di, dj in dirs:
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < nx and 0 <= nj < ny):
                    continue
                if dist[ni][nj] < 1e8:
                    continue
                nxm, nym = x0 + ni * step, y0 + nj * step
                # допустимо: внутри острова A, или в любом месте без чужой меди
                if not (in_poly(nxm, nym, A) or cell_ok(nxm, nym)):
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
            bridge_done = True
        else:
            print("+12V мост не найден")

pcbnew.SaveBoard(MAIN, b)
print("сохранено")
