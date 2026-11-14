import sys, json, math, heapq
import pcbnew

spec = json.load(open(sys.argv[1], encoding='utf-8'))
board_path = spec['board']
NET   = spec['net']
START = spec['start']; END = spec['end']          # [x, y, 'F.Cu']
CELL  = spec.get('cell', 0.1)
CLR   = spec.get('clearance', 0.15)
TW    = spec.get('track_w', 0.15)
VIA_D = spec.get('via_d', 0.45); VIA_DR = spec.get('via_drill', 0.2)
MARGIN= spec.get('margin', 2.0)
IGN   = set(spec.get('ignore_nets', []))
TURN  = spec.get('turn_cost', 0.8)
VIAC  = spec.get('via_cost', 3.0)
BOXES = spec.get('forbid_boxes', [])

b = pcbnew.LoadBoard(board_path)
L = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}

if 'bounds' in spec:
    X0, Y0, X1, Y1 = spec['bounds']
else:
    X0 = min(START[0], END[0]) - MARGIN; X1 = max(START[0], END[0]) + MARGIN
    Y0 = min(START[1], END[1]) - MARGIN; Y1 = max(START[1], END[1]) + MARGIN
i0 = int(math.floor(X0/CELL)); j0 = int(math.floor(Y0/CELL))
NX = int(math.ceil((X1-X0)/CELL)) + 2
NY = int(math.ceil((Y1-Y0)/CELL)) + 2
def cx(i): return (i0+i)*CELL
def cy(j): return (j0+j)*CELL

obs_tracks = {pcbnew.F_Cu: [], pcbnew.B_Cu: []}
obs_pads   = {pcbnew.F_Cu: [], pcbnew.B_Cu: []}
obs_vias   = []
for t in b.GetTracks():
    nm = t.GetNetname()
    if nm == NET or nm in IGN: continue
    s, e = t.GetStart(), t.GetEnd()
    sx, sy, ex, ey = s.x/1e6, s.y/1e6, e.x/1e6, e.y/1e6
    if t.GetClass() == 'PCB_VIA':
        obs_vias.append((sx, sy, t.GetWidth(t.TopLayer())/2e6))
        for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
            obs_tracks[lay].append((sx, sy, sx, sy, t.GetWidth(t.TopLayer())/2e6))
    else:
        obs_tracks[t.GetLayer()].append((sx, sy, ex, ey, t.GetWidth()/2e6))
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm == NET or nm in IGN: continue
        pp = p.GetPosition(); sz = p.GetSize()
        hw, hh = sz.x/2e6, sz.y/2e6
        pth = p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
        ang = p.GetOrientationDegrees() % 180.0
        qx, qy = pp.x/1e6, pp.y/1e6
        if abs(ang - 90.0) < 0.01: hw, hh = hh, hw
        ang_ok = (abs(ang) < 0.01 or abs(ang - 90.0) < 0.01 or abs(ang - 180.0) < 0.01)
        for lay, ok in ((pcbnew.F_Cu, p.IsOnLayer(pcbnew.F_Cu)), (pcbnew.B_Cu, p.IsOnLayer(pcbnew.B_Cu))):
            if ok or pth:
                if ang_ok:
                    obs_pads[lay].append((qx-hw, qy-hh, qx+hw, qy+hh))
                else:
                    rr = math.hypot(hw, hh)
                    obs_tracks[lay].append((qx, qy, qx, qy, rr))

zone_polys = {pcbnew.F_Cu: [], pcbnew.B_Cu: []}
for z in b.Zones():
    for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
        try:
            if not z.IsOnLayer(lay): continue
            ps = z.GetFilledPolysList(lay)
        except Exception:
            continue
        if ps is not None and ps.OutlineCount() > 0:
            zone_polys[lay].append(ps)
print('zone fill polys F=%d B=%d' % (len(zone_polys[pcbnew.F_Cu]), len(zone_polys[pcbnew.B_Cu])))


def d_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1
    L2 = dx*dx + dy*dy
    if L2 == 0: return math.hypot(px-x1, py-y1)
    t = ((px-x1)*dx + (py-y1)*dy)/L2
    if t < 0: t = 0.0
    elif t > 1: t = 1.0
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))
def d_rect(px, py, x0, y0, x1, y1):
    return math.hypot(max(x0-px, 0, px-x1), max(y0-py, 0, py-y1))

# rasterise
BLK_T = {pcbnew.F_Cu: [[False]*NX for _ in range(NY)], pcbnew.B_Cu: [[False]*NX for _ in range(NY)]}
BLK_V = [[False]*NX for _ in range(NY)]
own_hw = TW/2.0
for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
    for (x1, y1, x2, y2, hw) in obs_tracks[lay]:
        reach = hw + CLR + own_hw + CELL
        ax0 = int(math.floor((min(x1,x2)-reach)/CELL))-i0; ax1 = int(math.ceil((max(x1,x2)+reach)/CELL))-i0
        ay0 = int(math.floor((min(y1,y2)-reach)/CELL))-j0; ay1 = int(math.ceil((max(y1,y2)+reach)/CELL))-j0
        for j in range(max(0,ay0), min(NY, ay1+1)):
            py = cy(j)
            for i in range(max(0,ax0), min(NX, ax1+1)):
                if d_seg(cx(i), py, x1, y1, x2, y2) < hw + CLR + own_hw:
                    BLK_T[lay][j][i] = True
for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
    for (x1, y1, x2, y2, hw) in obs_tracks[lay]:
        reach = hw + CLR + VIA_D/2.0 + CELL
        ax0 = int(math.floor((min(x1,x2)-reach)/CELL))-i0; ax1 = int(math.ceil((max(x1,x2)+reach)/CELL))-i0
        ay0 = int(math.floor((min(y1,y2)-reach)/CELL))-j0; ay1 = int(math.ceil((max(y1,y2)+reach)/CELL))-j0
        for j in range(max(0,ay0), min(NY, ay1+1)):
            py = cy(j)
            for i in range(max(0,ax0), min(NX, ax1+1)):
                if d_seg(cx(i), py, x1, y1, x2, y2) < hw + CLR + VIA_D/2.0:
                    BLK_V[j][i] = True

for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
    for (px0, py0, px1, py1) in obs_pads[lay]:
        reach = CLR + own_hw + CELL
        ax0 = int(math.floor((px0-reach)/CELL))-i0; ax1 = int(math.ceil((px1+reach)/CELL))-i0
        ay0 = int(math.floor((py0-reach)/CELL))-j0; ay1 = int(math.ceil((py1+reach)/CELL))-j0
        for j in range(max(0,ay0), min(NY, ay1+1)):
            py = cy(j)
            for i in range(max(0,ax0), min(NX, ax1+1)):
                if d_rect(cx(i), py, px0, py0, px1, py1) < CLR + own_hw:
                    BLK_T[lay][j][i] = True
for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
    for (px0, py0, px1, py1) in obs_pads[lay]:
        reach = CLR + VIA_D/2.0 + CELL
        ax0 = int(math.floor((px0-reach)/CELL))-i0; ax1 = int(math.ceil((px1+reach)/CELL))-i0
        ay0 = int(math.floor((py0-reach)/CELL))-j0; ay1 = int(math.ceil((py1+reach)/CELL))-j0
        for j in range(max(0,ay0), min(NY, ay1+1)):
            py = cy(j)
            for i in range(max(0,ax0), min(NX, ax1+1)):
                if d_rect(cx(i), py, px0, py0, px1, py1) < CLR + VIA_D/2.0:
                    BLK_V[j][i] = True

# zone fill as obstacle (foreign net copper)
OFFT = [(0,0), (0.30,0), (-0.30,0), (0,0.30), (0,-0.30)]
OFFV = [(0,0), (0.45,0), (-0.45,0), (0,0.45), (0,-0.45)]
if spec.get('zones_block', True):
    for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
        for ps in zone_polys[lay]:
            for j in range(NY):
                py = cy(j)
                for i in range(NX):
                    if BLK_T[lay][j][i]: continue
                    px = cx(i)
                    if any(ps.Contains(pcbnew.VECTOR2I(int((px+ox)*1e6), int((py+oy)*1e6))) for (ox, oy) in OFFT):
                        BLK_T[lay][j][i] = True
        for ps in zone_polys[lay]:
            for j in range(NY):
                py = cy(j)
                for i in range(NX):
                    if BLK_V[j][i]: continue
                    px = cx(i)
                    if any(ps.Contains(pcbnew.VECTOR2I(int((px+ox)*1e6), int((py+oy)*1e6))) for (ox, oy) in OFFV):
                        BLK_V[j][i] = True

# board edge containment
poly = pcbnew.SHAPE_POLY_SET()
b.GetBoardPolygonOutlines(poly, True)
def inside(x, y, m):
    for ox, oy in ((0,0), (m,0), (-m,0), (0,m), (0,-m)):
        if not poly.Contains(pcbnew.VECTOR2I(int((x+ox)*1e6), int((y+oy)*1e6))):
            return False
    return True
bb = poly.BBox()
bx0, by0, bx1, by1 = bb.GetX()/1e6, bb.GetY()/1e6, bb.GetRight()/1e6, bb.GetBottom()/1e6
for j in range(NY):
    for i in range(NX):
        x, y = cx(i), cy(j)
        if x < bx0+0.5 or x > bx1-0.5 or y < by0+0.5 or y > by1-0.5:
            bad = True
        elif x < bx0+1.2 or x > bx1-1.2 or y < by0+1.2 or y > by1-1.2:
            bad = not inside(x, y, 0.4)
        else:
            bad = False
        if bad:
            BLK_T[pcbnew.F_Cu][j][i] = True; BLK_T[pcbnew.B_Cu][j][i] = True; BLK_V[j][i] = True
for bx in BOXES:
    for j in range(NY):
        for i in range(NX):
            if bx[0] <= cx(i) <= bx[2] and bx[1] <= cy(j) <= bx[3]:
                BLK_T[pcbnew.F_Cu][j][i] = True; BLK_T[pcbnew.B_Cu][j][i] = True; BLK_V[j][i] = True

if spec.get('dump_grid'):
    gw = open(spec['dump_grid'], 'w', encoding='utf-8')
    gw.write('bounds %.2f,%.2f..%.2f,%.2f cell %.3f  F=#  B=+  both=X  free=.\n' % (X0, Y0, X1, Y1, CELL))
    for j in range(NY-1, -1, -1):
        row = ''
        for i in range(NX):
            f = BLK_T[pcbnew.F_Cu][j][i]; k = BLK_T[pcbnew.B_Cu][j][i]
            row += 'X' if (f and k) else ('#' if f else ('+' if k else '.'))
        gw.write('%7.2f %s\n' % (cy(j), row))
    gw.close()

def cellof(x, y):
    return (int(round(x/CELL))-i0, int(round(y/CELL))-j0)
si, sj = cellof(START[0], START[1]); ti, tj = cellof(END[0], END[1])
sl = L[START[2]]; tl = L[END[2]]
print('grid %dx%d  start %s(%d,%d) end %s(%d,%d)' % (NX, NY, START[2], si, sj, END[2], ti, tj))
print('blk at start T=%s V=%s ; at end T=%s V=%s' % (BLK_T[sl][sj][si], BLK_V[sj][si], BLK_T[tl][tj][ti], BLK_V[tj][ti]))

from collections import deque
reach = set(); dq = deque()
if not BLK_T[sl][sj][si]:
    reach.add((sl, si, sj)); dq.append((sl, si, sj))
while dq:
    lay, i, j = dq.popleft()
    if not BLK_V[j][i]:
        ol = pcbnew.B_Cu if lay == pcbnew.F_Cu else pcbnew.F_Cu
        if (ol, i, j) not in reach: reach.add((ol, i, j)); dq.append((ol, i, j))
    for di, dj in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
        ni, nj = i+di, j+dj
        if 0 <= ni < NX and 0 <= nj < NY and not BLK_T[lay][nj][ni] and (lay, ni, nj) not in reach:
            reach.add((lay, ni, nj)); dq.append((lay, ni, nj))
reach2 = set()
if not BLK_T[tl][tj][ti]:
    reach2.add((tl, ti, tj)); dq = deque([(tl, ti, tj)])
    while dq:
        lay, i, j = dq.popleft()
        if not BLK_V[j][i]:
            ol = pcbnew.B_Cu if lay == pcbnew.F_Cu else pcbnew.F_Cu
            if (ol, i, j) not in reach2: reach2.add((ol, i, j)); dq.append((ol, i, j))
        for di, dj in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            ni, nj = i+di, j+dj
            if 0 <= ni < NX and 0 <= nj < NY and not BLK_T[lay][nj][ni] and (lay, ni, nj) not in reach2:
                reach2.add((lay, ni, nj)); dq.append((lay, ni, nj))
def bboxof(S, tag):
    for lay, nm in ((pcbnew.F_Cu, 'F'), (pcbnew.B_Cu, 'B')):
        c = [(i, j) for (l, i, j) in S if l == lay]
        if c:
            print('  %s layer %s: %d cells  x %.2f..%.2f  y %.2f..%.2f' % (tag, nm, len(c), cx(min(p[0] for p in c)), cx(max(p[0] for p in c)), cy(min(p[1] for p in c)), cy(max(p[1] for p in c))))
print('reachable cells %d ; from-end cells %d ; connected: %s' % (len(reach), len(reach2), (tl, ti, tj) in reach))
bboxof(reach, 'A'); bboxof(reach2, 'B')
if reach2 and (tl, ti, tj) not in reach:
    INF = 10**9
    distB = [[INF]*NX for _ in range(NY)]; srcB = [[None]*NX for _ in range(NY)]
    q = deque()
    for (lay, i, j) in reach2:
        if distB[j][i] > 0: distB[j][i] = 0; srcB[j][i] = (i, j); q.append((i, j))
    while q:
        i, j = q.popleft()
        for di2, dj2 in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            ni, nj = i+di2, j+dj2
            if 0 <= ni < NX and 0 <= nj < NY and distB[nj][ni] == INF:
                distB[nj][ni] = distB[j][i] + 1; srcB[nj][ni] = srcB[j][i]; q.append((ni, nj))
    bestpair = (1e18, None, None)
    for (lay, i, j) in reach:
        d = distB[j][i]
        if d < bestpair[0]:
            bestpair = (d, (i, j, lay), (srcB[j][i][0], srcB[j][i][1], lay))
    print('closest A-B: dist %.2f mm  A=(%.2f,%.2f)  B=(%.2f,%.2f)' % (bestpair[0]*CELL, cx(bestpair[1][0]), cy(bestpair[1][1]), cx(bestpair[2][0]), cy(bestpair[2][1])))
if spec.get('dump_reach'):
    gw = open(spec['dump_reach'], 'w', encoding='utf-8')
    for j in range(NY-1, -1, -1):
        row = ''
        for i in range(NX):
            if (pcbnew.F_Cu, i, j) in reach or (pcbnew.B_Cu, i, j) in reach: row += 'o'
            else:
                f = BLK_T[pcbnew.F_Cu][j][i]; k = BLK_T[pcbnew.B_Cu][j][i]
                row += 'X' if (f and k) else ('#' if f else ('+' if k else '.'))
        gw.write('%7.2f %s\n' % (cy(j), row))
    gw.close()
    print('wrote reach map', spec['dump_reach'])

DIRS = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
def h(i, j):
    return math.hypot(cx(i)-cx(ti), cy(j)-cy(tj))

start = (sl, si, sj, -1)
best = {(sl, si, sj, -1): 0.0}
pq = [(h(si,sj), 0.0, sl, si, sj, -1)]
prev = {}
found = None
while pq:
    f, g, lay, i, j, d = heapq.heappop(pq)
    key = (lay, i, j, d)
    if g > best.get(key, 1e18) + 1e-12: continue
    if lay == tl and i == ti and j == tj:
        found = key; break
    for nd, (di, dj) in enumerate(DIRS):
        ni, nj = i+di, j+dj
        if not (0 <= ni < NX and 0 <= nj < NY): continue
        if BLK_T[lay][nj][ni]: continue
        step = math.hypot(di, dj)*CELL
        cost = g + step + (0.0 if d == -1 or d == nd else TURN)
        nk = (lay, ni, nj, nd)
        if cost < best.get(nk, 1e18) - 1e-12:
            best[nk] = cost; prev[nk] = (key, 'M')
            heapq.heappush(pq, (cost + h(ni,nj), cost, lay, ni, nj, nd))
    if not BLK_V[j][i]:
        ol = pcbnew.B_Cu if lay == pcbnew.F_Cu else pcbnew.F_Cu
        cost = g + VIAC
        nk = (ol, i, j, -1)
        if cost < best.get(nk, 1e18) - 1e-12:
            best[nk] = cost; prev[nk] = (key, 'V')
            heapq.heappush(pq, (cost + h(i,j), cost, ol, i, j, -1))
if not found:
    print('NO ROUTE'); sys.exit(3)
path = []
k = found
while k in prev:
    p, kind = prev[k]
    path.append((k, kind)); k = p
path.append((k, None)); path.reverse()
print('cost %.3f  nodes %d' % (best[found], len(path)))
out = []
for (lay, i, j, d), kind in path:
    out.append({'x': round(cx(i), 4), 'y': round(cy(j), 4), 'l': 'F.Cu' if lay == pcbnew.F_Cu else 'B.Cu', 'via': kind == 'V'})
json.dump(out, open(spec['out'], 'w', encoding='utf-8'), indent=0)
print('wrote', spec['out'], len(out), 'points')
