import io, sys, math, shutil, heapq
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"
shutil.copy(ROOT + r"\_work\BACKUP_r8.kicad_pcb", MAIN)
b = pcbnew.LoadBoard(MAIN)


def netcode(name):
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == name:
                return p.GetNetCode()
    return None


def add(net, pathF, pathB, vias):
    nc = netcode(net)
    for pts, layer in ((pathF, pcbnew.F_Cu), (pathB, pcbnew.B_Cu)):
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
            t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
            t.SetWidth(int(round(0.15 * 1e6)))
            t.SetLayer(layer)
            t.SetNetCode(nc)
            b.Add(t)
    for (x, y) in vias:
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(0.45 * 1e6)))
        v.SetDrill(int(round(0.2 * 1e6)))
        v.SetNetCode(nc)
        b.Add(v)


# U7.23
def add_simple(net, pts, layer, w=0.15):
    nc = netcode(net)
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        t.SetWidth(int(round(w * 1e6)))
        t.SetLayer(layer)
        t.SetNetCode(nc)
        b.Add(t)


add_simple("GND", [(11.43, 40.71), (13.30, 40.71), (13.30, 34.00)], pcbnew.F_Cu)

m = rr.Model(MAIN)
m.build()
tracks = m.tracks
pads = m.pads
CLR = rr.CLR
hw = 0.075


def blocked(x, y, L, net):
    for (x1, y1, x2, y2, h) in tracks[L]:
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        tt = 0 if l2 < 1e-12 else max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / l2))
        if math.hypot(x - (x1 + tt * dx), y - (y1 + tt * dy)) < h + hw + CLR - 0.01:
            return True
    for q in pads:
        if q['net'] == net:
            continue
        on = q['onF'] if L == 0 else q['onB']
        if not on:
            continue
        u = q['c'] * (x - q['x']) + q['s'] * (y - q['y'])
        v = -q['s'] * (x - q['x']) + q['c'] * (y - q['y'])
        lim = CLR + hw + 0.02
        if abs(u) < q['hx'] + lim and abs(v) < q['hy'] + lim:
            return True
    return False


def find_path(net, start, goal, box, step=0.1):
    x0, y0, x1, y1 = box
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    gpad = None
    for q in pads:
        if q['net'] == net:
            u = q['c'] * (goal[0] - q['x']) + q['s'] * (goal[1] - q['y'])
            v = -q['s'] * (goal[0] - q['x']) + q['c'] * (goal[1] - q['y'])
            if abs(u) < q['hx'] + 0.1 and abs(v) < q['hy'] + 0.1:
                gpad = q
                break
    if gpad is None:
        return None

    def cell(x, y):
        return int(round((x - x0) / step)), int(round((y - y0) / step))
    si, sj = cell(*start)
    dist = [[[1e9] * ny for _ in range(nx)] for _ in range(2)]
    prev = [[[None] * ny for _ in range(nx)] for _ in range(2)]
    dist[0][si][sj] = 0
    q = [(0, 0, si, sj)]
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    found = None
    while q:
        d, L, ci, cj = heapq.heappop(q)
        if d > dist[L][ci][cj]:
            continue
        x, y = x0 + ci * step, y0 + cj * step
        u = gpad['c'] * (x - gpad['x']) + gpad['s'] * (y - gpad['y'])
        v = -gpad['s'] * (x - gpad['x']) + gpad['c'] * (y - gpad['y'])
        if L == 0 and abs(u) < gpad['hx'] + 0.05 and abs(v) < gpad['hy'] + 0.05:
            found = (L, ci, cj)
            break
        for di, dj in dirs:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            nxm, nym = x0 + ni * step, y0 + nj * step
            if blocked(nxm, nym, L, net):
                continue
            nd = d + math.hypot(di, dj) * step
            if nd < dist[L][ni][nj]:
                dist[L][ni][nj] = nd
                prev[L][ni][nj] = (L, ci, cj)
                heapq.heappush(q, (nd, L, ni, nj))
        # via переход
        if L == 0 and not blocked(x, y, 1, net):
            if d + 0.8 < dist[1][ci][cj]:
                dist[1][ci][cj] = d + 0.8
                prev[1][ci][cj] = (L, ci, cj)
                heapq.heappush(q, (d + 0.8, 1, ci, cj))
        elif L == 1 and not blocked(x, y, 0, net):
            if d + 0.8 < dist[0][ci][cj]:
                dist[0][ci][cj] = d + 0.8
                prev[0][ci][cj] = (L, ci, cj)
                heapq.heappush(q, (d + 0.8, 0, ci, cj))
    if found is None:
        return None
    path = []
    L, ci, cj = found
    while True:
        path.append((L, x0 + ci * step, y0 + cj * step))
        p = prev[L][ci][cj]
        if p is None:
            break
        L, ci, cj = p
    path.reverse()
    return path


def simplify_path(path, net, step=0.1):
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            L0, x1, y1 = path[i]
            L1, x2, y2 = path[j]
            if L0 != L1:
                j -= 1
                continue
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
            ok = True
            for k in range(1, n):
                x = x1 + (x2 - x1) * k / n
                y = y1 + (y2 - y1) * k / n
                if blocked(x, y, L0, net):
                    ok = False
                    break
            if ok:
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


for net, start, goal, box in [
    ("/BUT3", (-16.30, 18.95), (-20.117, 13.87), (-22.0, 12.8, -15.0, 20.2)),
    ("/BUT4", (-21.80, 23.05), (-20.117, 14.52), (-23.5, 13.5, -19.0, 24.5)),
]:
    path = find_path(net, start, goal, box)
    if path:
        path = simplify_path(path, net)
        pathF = [(x, y) for (L, x, y) in path if L == 0]
        pathB = [(x, y) for (L, x, y) in path if L == 1]
        # vias на переходах слоя
        vias = []
        for k in range(len(path) - 1):
            if path[k][0] != path[k + 1][0]:
                vias.append((path[k][1], path[k][2]))
        add(net, pathF, pathB, vias)
        print(f"{net}: путь найден, F точек {len(pathF)}, B {len(pathB)}, vias {len(vias)}")
    else:
        print(f"{net}: путь НЕ найден")

pcbnew.SaveBoard(MAIN, b)
print("сохранено")
