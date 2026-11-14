"""Routing engine for kamaz_leveler_ruichi158 (2 layers, 0.1 mm grid, A*).

Model: numpy "penetration" fields per layer (max over foreign copper of halfsize-distance),
static fields for board boundary / slot arcs, exact geometric checks for final geometry.
"""
import math
import heapq
import numpy as np
import pcbnew

CELL = 0.1
X0, Y0 = -30.0, -42.75
NX, NY = 601, 856
NCELL = NX * NY

CLR = 0.170          # project min_clearance 0.127 / netclass 0.15 + margin
EDGE_REQ = 0.22      # min_copper_edge_clearance 0.2 + margin
SLOT_REQ = 0.52      # task requirement 0.5 for the 4 R3.75 slots
VIA_D = 0.45
VIA_R = VIA_D / 2.0
VIA_DRILL = 0.2
MAXHW = 0.40         # widest track half width (0.8 mm)
BIGNEG = -10.0

ANT = (-13.0, 36.0, 13.0, 42.75)   # antenna keepout strip (x0,y0,x1,y1)
# forced corridors inside the strip: U7 pads 1-2 (+3V3_ESP) and pad 23 (GND)
ANT_HOLES = [(-11.75, 37.4, -11.11, 41.5), (11.18, 34.0, 11.70, 41.6)]


def ix(x):
    return int(round((x - X0) / CELL))


def iy(y):
    return int(round((y - Y0) / CELL))


def cx(i):
    return X0 + i * CELL


def cy(j):
    return Y0 + j * CELL


def dseg(X, Y, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return np.hypot(X - x1, Y - y1)
    t = ((X - x1) * dx + (Y - y1) * dy) / L2
    np.clip(t, 0.0, 1.0, out=t)
    return np.hypot(X - (x1 + t * dx), Y - (y1 + t * dy))


def _win(x, y, R):
    i0 = max(0, ix(x - R))
    i1 = min(NX - 1, ix(x + R))
    j0 = max(0, iy(y - R))
    j1 = min(NY - 1, iy(y + R))
    return i0, i1, j0, j1


def add_pad_field(pen, pad):
    """pen = max(pen, -signed_distance) for a rounded-rect pad."""
    hx, hy, r = pad['hx'], pad['hy'], pad['r']
    R = max(hx, hy) + CLR + MAXHW + 2 * CELL
    i0, i1, j0, j1 = _win(pad['x'], pad['y'], R)
    if i1 < i0 or j1 < j0:
        return
    xs = X0 + np.arange(i0, i1 + 1) * CELL - pad['x']
    ys = Y0 + np.arange(j0, j1 + 1) * CELL - pad['y']
    U = pad['c'] * xs[None, :] + pad['s'] * ys[:, None]
    V = -pad['s'] * xs[None, :] + pad['c'] * ys[:, None]
    qx = np.abs(U) - (hx - r)
    qy = np.abs(V) - (hy - r)
    d = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0)
    val = (r - d).astype(np.float32)
    np.maximum(pen[j0:j1 + 1, i0:i1 + 1], val, out=pen[j0:j1 + 1, i0:i1 + 1])


def add_seg_field(pen, x1, y1, x2, y2, h):
    R = h + CLR + MAXHW + 2 * CELL
    i0, i1, j0, j1 = _win((x1 + x2) / 2, (y1 + y2) / 2, math.hypot(x2 - x1, y2 - y1) / 2 + R)
    if i1 < i0 or j1 < j0:
        return
    X = (X0 + np.arange(i0, i1 + 1) * CELL)[None, :]
    Y = (Y0 + np.arange(j0, j1 + 1) * CELL)[:, None]
    d = dseg(X, Y, x1, y1, x2, y2)
    val = (h - d).astype(np.float32)
    np.maximum(pen[j0:j1 + 1, i0:i1 + 1], val, out=pen[j0:j1 + 1, i0:i1 + 1])


def add_dist_field(fld, x1, y1, x2, y2, R):
    i0, i1, j0, j1 = _win((x1 + x2) / 2, (y1 + y2) / 2, math.hypot(x2 - x1, y2 - y1) / 2 + R)
    if i1 < i0 or j1 < j0:
        return
    X = (X0 + np.arange(i0, i1 + 1) * CELL)[None, :]
    Y = (Y0 + np.arange(j0, j1 + 1) * CELL)[:, None]
    d = dseg(X, Y, x1, y1, x2, y2).astype(np.float32)
    np.minimum(fld[j0:j1 + 1, i0:i1 + 1], d, out=fld[j0:j1 + 1, i0:i1 + 1])


def min_dist_pt(P, S):
    """P: (n,2) points, S: (m,4) segments -> (n,) min distance."""
    x1, y1, x2, y2 = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
    dx = x2 - x1
    dy = y2 - y1
    L2 = dx * dx + dy * dy
    L2 = np.where(L2 < 1e-12, 1e-12, L2)
    px = P[:, 0:1]
    py = P[:, 1:2]
    t = ((px - x1) * dx + (py - y1) * dy) / L2
    np.clip(t, 0.0, 1.0, out=t)
    d = np.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
    return d.min(axis=1)


class Model:
    def __init__(self, path):
        self.b = pcbnew.LoadBoard(path)
        self.pads = []
        self.nets = {}
        self.tracks = [[], []]   # (x1,y1,x2,y2,h) per layer
        self.vias = []           # (x,y)
        self._read()

    def _read(self):
        b = self.b
        for fp in b.GetFootprints():
            for p in fp.Pads():
                nm = p.GetNetname()
                if not nm:
                    nm = '__none__'
                pos = p.GetPosition()
                sz = p.GetSize()
                hx, hy = sz.x / 2e6, sz.y / 2e6
                ang = math.radians(p.GetOrientationDegrees())
                sh = p.GetShape()
                if sh in (pcbnew.PAD_SHAPE_CIRCLE, pcbnew.PAD_SHAPE_OVAL):
                    r = min(hx, hy)
                elif sh == pcbnew.PAD_SHAPE_ROUNDRECT:
                    r = min(p.GetRoundRectCornerRadius() / 1e6, min(hx, hy))
                else:
                    r = 0.0
                if sh in (pcbnew.PAD_SHAPE_TRAPEZOID, pcbnew.PAD_SHAPE_CUSTOM,
                          pcbnew.PAD_SHAPE_CHAMFERED_RECT):
                    rr_ = math.hypot(hx, hy)
                    hx = hy = r = rr_
                bb = p.GetBoundingBox()
                bbx, bby = bb.GetWidth() / 2e6, bb.GetHeight() / 2e6
                ex = hx * abs(math.cos(ang)) + hy * abs(math.sin(ang))
                ey = hx * abs(math.sin(ang)) + hy * abs(math.cos(ang))
                if bbx > ex + 0.03 or bby > ey + 0.03:
                    hx, hy, r, ang = bbx, bby, 0.0, 0.0
                    cbc = bb.Centre()
                    pos = pcbnew.VECTOR2I(cbc.x, cbc.y)
                self.pads.append(dict(
                    x=pos.x / 1e6, y=pos.y / 1e6, hx=hx, hy=hy, r=r,
                    c=math.cos(ang), s=math.sin(ang), net=nm,
                    ref=fp.GetReference(), num=p.GetNumber(),
                    onF=p.IsOnLayer(pcbnew.F_Cu), onB=p.IsOnLayer(pcbnew.B_Cu),
                    drill=p.GetDrillSize().x / 1e6))
                if nm != '__none__':
                    self.nets[nm] = p.GetNetCode()
        self.vias = []
        for t in b.GetTracks():
            nm = t.GetNetname()
            if t.Type() == pcbnew.PCB_VIA_T:
                q = t.GetPosition()
                self.vias.append((q.x / 1e6, q.y / 1e6))
            else:
                s0, e0 = t.GetStart(), t.GetEnd()
                L = 0 if t.GetLayer() == pcbnew.F_Cu else 1
                self.tracks[L].append((s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6,
                                       t.GetWidth() / 2e6))

    def netpads(self, nm):
        return [p for p in self.pads if p['net'] == nm]

    def reset_pen(self):
        self.pen = [np.full((NY, NX), BIGNEG, np.float32), np.full((NY, NX), BIGNEG, np.float32)]
        for pd in self.pads:
            if pd['onF']:
                add_pad_field(self.pen[0], pd)
            if pd['onB']:
                add_pad_field(self.pen[1], pd)
        self.tracks = [[], []]
        self.vias = []

    def build(self):
        tr = [list(self.tracks[0]), list(self.tracks[1])]
        vi = list(self.vias)
        self.reset_pen()
        for L in (0, 1):
            for (x1, y1, x2, y2, h) in tr[L]:
                add_seg_field(self.pen[L], x1, y1, x2, y2, h)
        for (vx, vy) in vi:
            add_seg_field(self.pen[0], vx, vy, vx, vy, VIA_R)
            add_seg_field(self.pen[1], vx, vy, vx, vy, VIA_R)
        self.tracks = tr
        self.vias = vi
        poly = pcbnew.SHAPE_POLY_SET()
        self.b.GetBoardPolygonOutlines(poly, True)
        bsegs = []
        for k in range(poly.OutlineCount()):
            ch = poly.Outline(k)
            n = ch.PointCount()
            for t in range(n):
                a = ch.CPoint(t)
                c = ch.CPoint((t + 1) % n)
                bsegs.append((a.x / 1e6, a.y / 1e6, c.x / 1e6, c.y / 1e6))
        self.bA = np.array(bsegs, float)
        ssegs = []
        self.slot_c = [(27.0, 29.0), (-27.0, 29.0), (27.0, -29.0), (-27.0, -29.0)]
        for (sx, sy) in self.slot_c:
            n = 48
            pts = [(sx + 3.75 * math.cos(2 * math.pi * t / n), sy + 3.75 * math.sin(2 * math.pi * t / n))
                   for t in range(n + 1)]
            for t in range(n):
                ssegs.append((pts[t][0], pts[t][1], pts[t + 1][0], pts[t + 1][1]))
        self.sA = np.array(ssegs, float)
        self.dedge = np.full((NY, NX), 99.0, np.float32)
        for (x1, y1, x2, y2) in bsegs:
            add_dist_field(self.dedge, x1, y1, x2, y2, 1.2)
        self.dslot = np.full((NY, NX), 99.0, np.float32)
        for (x1, y1, x2, y2) in ssegs:
            add_dist_field(self.dslot, x1, y1, x2, y2, 1.2)
        self.penhole = np.full((NY, NX), BIGNEG, np.float32)
        for pd in self.pads:
            if pd['drill'] > 0:
                hh = pd['drill'] / 2.0 + 0.25 + VIA_DRILL / 2.0
                add_seg_field(self.penhole, pd['x'], pd['y'], pd['x'], pd['y'], hh)
        self.forbid = []
        self.antmask = np.zeros((NY, NX), bool)
        self.antmask[iy(ANT[1]):iy(ANT[3]) + 1, ix(ANT[0]):ix(ANT[2]) + 1] = True
        for (hx0, hy0, hx1, hy1) in ANT_HOLES:
            self.antmask[iy(hy0):iy(hy1) + 1, ix(hx0):ix(hx1) + 1] = False

    def blocks(self, hw, extra=None):
        t = -(CLR + hw)
        out = []
        for L in (0, 1):
            m = self.pen[L] > t
            m |= self.dedge < (EDGE_REQ + hw)
            m |= self.dslot < (SLOT_REQ + hw)
            m |= self.antmask
            out.append(m)
        mv = (self.pen[0] > -(CLR + VIA_R)) | (self.pen[1] > -(CLR + VIA_R))
        mv |= self.dedge < (EDGE_REQ + VIA_R)
        mv |= self.dslot < (SLOT_REQ + VIA_R)
        mv |= self.antmask
        return out[0], out[1], mv

    def pen_for_net(self, netname):
        pen = [self.pen[0].copy(), self.pen[1].copy()]
        for pd in self.pads:
            if pd['net'] != netname:
                continue
            for L, on in ((0, pd['onF']), (1, pd['onB'])):
                if not on:
                    continue
                R = max(pd['hx'], pd['hy']) + CLR + MAXHW + 3 * CELL
                i0, i1, j0, j1 = _win(pd['x'], pd['y'], R)
                if i1 < i0 or j1 < j0:
                    continue
                w = pen[L][j0:j1 + 1, i0:i1 + 1]
                w[...] = BIGNEG
                x0w, x1w, y0w, y1w = cx(i0), cx(i1), cy(j0), cy(j1)
                for q in self.pads:
                    if q['net'] == netname:
                        continue
                    if not (q['onF'] if L == 0 else q['onB']):
                        continue
                    e = max(q['hx'], q['hy']) + CLR + MAXHW
                    if (q['x'] + e < x0w or q['x'] - e > x1w or
                            q['y'] + e < y0w or q['y'] - e > y1w):
                        continue
                    add_pad_field(pen[L], q)
                for (sx1, sy1, sx2, sy2, h) in self.tracks[L]:
                    e = h + CLR + MAXHW
                    if (max(sx1, sx2) + e < x0w or min(sx1, sx2) - e > x1w or
                            max(sy1, sy2) + e < y0w or min(sy1, sy2) - e > y1w):
                        continue
                    add_seg_field(pen[L], sx1, sy1, sx2, sy2, h)
                for (vx, vy) in self.vias:
                    add_seg_field(pen[L], vx, vy, vx, vy, VIA_R)
        return pen

    # ---------- exact checks ----------
    def seg_clear(self, x1, y1, x2, y2, hw, L, netname):
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
        t = np.linspace(0.0, 1.0, n)
        P = np.column_stack((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        if (min_dist_pt(P, self.bA) < EDGE_REQ + hw).any():
            return False
        if (min_dist_pt(P, self.sA) < SLOT_REQ + hw).any():
            return False
        X = P[:, 0]
        Y = P[:, 1]
        ins = (X > ANT[0]) & (X < ANT[2]) & (Y > ANT[1]) & (Y < ANT[3])
        if ins.any():
            ok = np.zeros(n, bool)
            for (hx0, hy0, hx1, hy1) in ANT_HOLES:
                ok |= (X > hx0) & (X < hx1) & (Y > hy0) & (Y < hy1)
            if (ins & ~ok).any():
                return False
        for q in self.pads:
            if q['net'] == netname:
                continue
            if not (q['onF'] if L == 0 else q['onB']):
                continue
            if (q['x'] + q['hx'] + hw + CLR < X.min() or q['x'] - q['hx'] - hw - CLR > X.max() or
                    q['y'] + q['hy'] + hw + CLR < Y.min() or q['y'] - q['hy'] - hw - CLR > Y.max()):
                continue
            u = q['c'] * (X - q['x']) + q['s'] * (Y - q['y'])
            v = -q['s'] * (X - q['x']) + q['c'] * (Y - q['y'])
            qx = np.abs(u) - (q['hx'] - q['r'])
            qy = np.abs(v) - (q['hy'] - q['r'])
            sd = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - q['r']
            if sd.min() < CLR + hw:
                return False
        for (a, b_, c, e, h) in self.tracks[L]:
            if (max(a, c) + h + hw + CLR < X.min() or min(a, c) - h - hw - CLR > X.max() or
                    max(b_, e) + h + hw + CLR < Y.min() or min(b_, e) - h - hw - CLR > Y.max()):
                continue
            lim = h + CLR + hw
            if dseg(P[:, 0:1], P[:, 1:2], a, b_, c, e).min() < lim:
                return False
            # exact: min over the 4 point-to-segment distances
            if (dseg(np.array([x1]), np.array([y1]), a, b_, c, e)[0] < lim or
                    dseg(np.array([x2]), np.array([y2]), a, b_, c, e)[0] < lim or
                    dseg(np.array([a]), np.array([b_]), x1, y1, x2, y2)[0] < lim or
                    dseg(np.array([c]), np.array([e]), x1, y1, x2, y2)[0] < lim):
                return False
        for (vx, vy) in self.vias:
            if (vx + VIA_R + hw + CLR < X.min() or vx - VIA_R - hw - CLR > X.max() or
                    vy + VIA_R + hw + CLR < Y.min() or vy - VIA_R - hw - CLR > Y.max()):
                continue
            if min_dist_pt(P, np.array([[vx, vy, vx, vy]])).min() < VIA_R + CLR + hw:
                return False
        return True

    def via_clear(self, x, y, netname, pens=None):
        pen = pens if pens is not None else self.pen
        for L in (0, 1):
            if pen[L][iy(y), ix(x)] > -(CLR + VIA_R):
                return False
        if self.dedge[iy(y), ix(x)] < EDGE_REQ + VIA_R:
            return False
        if self.dslot[iy(y), ix(x)] < SLOT_REQ + VIA_R:
            return False
        if self.antmask[iy(y), ix(x)]:
            return False
        return self.seg_clear(x, y, x, y, VIA_R, 0, netname) and \
            self.seg_clear(x, y, x, y, VIA_R, 1, netname)

    def add_copper(self, segs, vias):
        for (x1, y1, x2, y2, h, L) in segs:
            add_seg_field(self.pen[L], x1, y1, x2, y2, h)
            self.tracks[L].append((x1, y1, x2, y2, h))
        for (x, y) in vias:
            add_seg_field(self.pen[0], x, y, x, y, VIA_R)
            add_seg_field(self.pen[1], x, y, x, y, VIA_R)
            self.vias.append((x, y))

    # ---------- A* ----------
    def astar(self, seeds, goalset, gc, hw, pens, margin=3.0, via_cost=4.0,
              ownpen=None, maxexp=300000):
        blkF, blkB, blkV = self.pen_masks(pens, hw)
        xs = [s[0] for s in seeds] + [gc[0]]
        ys = [s[1] for s in seeds] + [gc[1]]
        i0 = max(0, ix(min(xs) - margin))
        i1 = min(NX - 1, ix(max(xs) + margin))
        j0 = max(0, iy(min(ys) - margin))
        j1 = min(NY - 1, iy(max(ys) + margin))
        W = i1 - i0 + 1
        H = j1 - j0 + 1
        BW = [blkF[j0:j1 + 1, i0:i1 + 1].tobytes(), blkB[j0:j1 + 1, i0:i1 + 1].tobytes()]
        BV = blkV[j0:j1 + 1, i0:i1 + 1].tobytes()
        OW = None
        if ownpen is not None:
            OW = ownpen[j0:j1 + 1, i0:i1 + 1].tobytes()
        gset = set()
        for (gl, gi, gj) in goalset:
            if i0 <= gi <= i1 and j0 <= gj <= j1:
                gset.add((gl, (gj - j0) * W + (gi - i0)))
        if not gset:
            return None
        DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
        C1 = CELL
        C2 = CELL * math.sqrt(2)
        DLEN = (C1, C2, C1, C2, C1, C2, C1, C2)
        T45 = 0.35
        T90 = 1.20
        INF = 1e18
        best = {}
        pq = []
        gx, gy = gc
        for (sx, sy, L, g) in seeds:
            si = ix(sx) - i0
            sj = iy(sy) - j0
            if not (0 <= si < W and 0 <= sj < H):
                continue
            k = (L, sj * W + si, -1)
            if g < best.get(k, INF):
                best[k] = g
                heapq.heappush(pq, (g + math.hypot(sx - gx, sy - gy), g, L, sj * W + si, -1))
        prev = {}
        found = None
        nexp = 0
        while pq:
            f, g, L, idx, d = heapq.heappop(pq)
            if g > best.get((L, idx, d), INF) + 1e-12:
                continue
            if (L, idx) in gset:
                found = (L, idx, d)
                break
            nexp += 1
            if nexp > maxexp:
                break
            j, i = divmod(idx, W)
            BL = BW[L]
            for nd in range(8):
                di, dj = DIRS[nd]
                ni = i + di
                nj = j + dj
                if ni < 0 or ni >= W or nj < 0 or nj >= H:
                    continue
                nidx = nj * W + ni
                if BL[nidx]:
                    continue
                c = g + DLEN[nd]
                if d >= 0 and nd != d:
                    dd = nd - d
                    if dd < 0:
                        dd = -dd
                    if dd > 4:
                        dd = 8 - dd
                    c += T45 if dd == 1 else T90
                if OW is not None and OW[nidx]:
                    c += 0.30
                nk = (L, nidx, nd)
                if c < best.get(nk, INF) - 1e-12:
                    best[nk] = c
                    prev[nk] = (L, idx, d, False)
                    nx = X0 + (ni + i0) * CELL
                    ny = Y0 + (nj + j0) * CELL
                    heapq.heappush(pq, (c + math.hypot(nx - gx, ny - gy), c, L, nidx, nd))
            if not BV[idx]:
                c = g + via_cost
                nk = (1 - L, idx, -1)
                if c < best.get(nk, INF) - 1e-12:
                    best[nk] = c
                    prev[nk] = (L, idx, d, True)
                    heapq.heappush(pq, (c + math.hypot(cx(i + i0) - gx, cy(j + j0) - gy), c, 1 - L, idx, -1))
        if found is None:
            return None
        path = []
        k = found
        while True:
            path.append(k[:2])
            p = prev.get(k)
            if p is None:
                break
            k = (p[0], p[1], p[2])
        path.reverse()
        pts = []
        for (L, idx) in path:
            j, i = divmod(idx, W)
            x = X0 + (i + i0) * CELL
            y = Y0 + (j + j0) * CELL
            if pts and pts[-1][2] == L:
                pts.append((x, y, L, False))
            else:
                if pts:
                    pts.append((pts[-1][0], pts[-1][1], pts[-1][2], True))
                pts.append((x, y, L, False))
        return pts

    def pen_masks(self, pens, hw):
        t = -(CLR + hw)
        blkF = pens[0] > t
        blkB = pens[1] > t
        for m in (blkF, blkB):
            m |= self.dedge < (EDGE_REQ + hw)
            m |= self.dslot < (SLOT_REQ + hw)
            m |= self.antmask
        blkV = (pens[0] > -(CLR + VIA_R)) | (pens[1] > -(CLR + VIA_R))
        blkV |= self.dedge < (EDGE_REQ + VIA_R)
        blkV |= self.dslot < (SLOT_REQ + VIA_R)
        blkV |= self.antmask
        blkV |= self.penhole > 0.0
        if self.forbid:
            fm = np.zeros((NY, NX), bool)
            for (x0, y0, x1, y1) in self.forbid:
                fm[max(0, iy(y0)):iy(y1) + 1, max(0, ix(x0)):ix(x1) + 1] = True
            blkF |= fm
            blkB |= fm
            blkV |= fm
        return blkF, blkB, blkV


def collinear_merge(pts):
    out = []
    for p in pts:
        if out and abs(out[-1][0] - p[0]) < 1e-9 and abs(out[-1][1] - p[1]) < 1e-9:
            continue
        out.append(p)
    if len(out) < 3:
        return out
    keep = [out[0]]
    for k in range(1, len(out) - 1):
        a, b, c = keep[-1], out[k], out[k + 1]
        x1, y1 = b[0] - a[0], b[1] - a[1]
        x2, y2 = c[0] - b[0], c[1] - b[1]
        cr = x1 * y2 - y1 * x2
        dot = x1 * x2 + y1 * y2
        if abs(cr) < 1e-7 and dot > 0:
            continue
        keep.append(b)
    keep.append(out[-1])
    return keep


def smooth(pts, model, hw, L, netname):
    if len(pts) <= 2:
        return pts[:]
    out = [pts[0]]
    a = 0
    n = len(pts)
    while a < n - 1:
        b = n - 1
        while b > a + 1:
            if model.seg_clear(pts[a][0], pts[a][1], pts[b][0], pts[b][1], hw, L, netname):
                break
            b -= 1
        out.append(pts[b])
        a = b
    return out
