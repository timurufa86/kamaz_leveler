import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

# usage: grid.py board X0 Y0 X1 Y1 CELL TW CLR VIA_D ignore_nets zones(0/1)
BRD = sys.argv[1]
X0, Y0, X1, Y1 = [float(v) for v in sys.argv[2:6]]
CELL = float(sys.argv[6])
TW = float(sys.argv[7])
CLR = float(sys.argv[8])
VIA_D = float(sys.argv[9])
IGN = set(a.strip() for a in sys.argv[10].split(',') if a.strip())
ZONES = int(sys.argv[11])
HIDEZONE = int(sys.argv[12]) if len(sys.argv) > 12 else 0

b = pcbnew.LoadBoard(BRD)
FC, BC = pcbnew.F_Cu, pcbnew.B_Cu
layname = {FC: 'F.Cu', BC: 'B.Cu'}

def d_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1
    L2 = dx*dx+dy*dy
    if L2 == 0: return math.hypot(px-x1, py-y1)
    t = ((px-x1)*dx + (py-y1)*dy)/L2
    if t < 0: t = 0.0
    elif t > 1: t = 1.0
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))

def d_rect(px, py, x0, y0, x1, y1):
    return math.hypot(max(x0-px, 0.0, px-x1), max(y0-py, 0.0, py-y1))

class Obst:
    __slots__ = ('kind','layers','x1','y1','x2','y2','hw','r','net')
    def __init__(self, kind, layers, x1, y1, x2=0.0, y2=0.0, hw=0.0, r=0.0, net=''):
        self.kind=kind; self.layers=layers; self.x1=x1; self.y1=y1; self.x2=x2; self.y2=y2
        self.hw=hw; self.r=r; self.net=net
    def dist(self, px, py):
        k = self.kind
        if k == 'seg':
            return d_seg(px, py, self.x1, self.y1, self.x2, self.y2) - self.hw
        if k == 'circ':
            return math.hypot(px-self.x1, py-self.y1) - self.r
        if k == 'rect':
            return d_rect(px, py, self.x1, self.y1, self.x2, self.y2)
        if k == 'rr':   # roundrect: inner rect + corner radius
            return d_rect(px, py, self.x1, self.y1, self.x2, self.y2) - self.r
        if k == 'cap':  # capsule
            return d_seg(px, py, self.x1, self.y1, self.x2, self.y2) - self.r
        return 1e9

obst = {FC: [], BC: []}

for t in b.GetTracks():
    nm = t.GetNetname()
    if nm in IGN: continue
    if isinstance(t, pcbnew.PCB_VIA):
        p = t.GetPosition(); x = p.x/1e6; y = p.y/1e6
        r = t.GetWidth(FC)/2e6
        for L in (FC, BC):
            obst[L].append(Obst('circ', L, x, y, r=r, net=nm))
    else:
        L = t.GetLayer()
        if L not in (FC, BC): continue
        s, e = t.GetStart(), t.GetEnd()
        obst[L].append(Obst('seg', L, s.x/1e6, s.y/1e6, e.x/1e6, e.y/1e6, hw=t.GetWidth()/2e6, net=nm))

for fp in b.GetFootprints():
    ref = fp.GetReference()
    for pd in fp.Pads():
        nm = pd.GetNetname()
        if nm in IGN: continue
        p = pd.GetPosition(); px = p.x/1e6; py = p.y/1e6
        sz = pd.GetSize(); w = sz.x/1e6; h = sz.y/1e6
        ang = pd.GetOrientationDegrees()
        shp = pd.GetShape()
        ls = [L for L in (FC, BC) if pd.IsOnLayer(L) or pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
        if not ls: continue
        o = None
        if abs(((ang+180) % 180) - 0) > 0.01 and abs(((ang+180) % 180) - 180) > 0.01:
            # rotated 90 -> swap
            if abs(((ang % 180) - 90)) < 0.01:
                w, h = h, w
            else:
                o = None
        if shp == pcbnew.PAD_SHAPE_CIRCLE:
            o = Obst('circ', ls, px, py, r=max(w,h)/2, net=nm)
        elif shp == pcbnew.PAD_SHAPE_OVAL:
            if w >= h:
                hw = h/2
                o = Obst('cap', ls, px-(w-h)/2, py, px+(w-h)/2, py, r=hw, net=nm)
            else:
                hw = w/2
                o = Obst('cap', ls, px, py-(h-w)/2, px, py+(h-w)/2, r=hw, net=nm)
        elif shp == pcbnew.PAD_SHAPE_ROUNDRECT:
            try:
                rc = pd.GetRoundRectCornerRadius()/1e6
            except Exception:
                rc = min(w, h)*0.25
            o = Obst('rr', ls, px-(w/2-rc), py-(h/2-rc), px+(w/2-rc), py+(h/2-rc), r=rc, net=nm)
        else:
            o = Obst('rect', ls, px-w/2, py-h/2, px+w/2, py+h/2, net=nm)
        if o is not None and (abs(ang) < 0.01 or abs(ang-180) < 0.01 or abs(ang-90) < 0.01 or abs(ang-270) < 0.01):
            for L in ls: obst[L].append(o)
        else:
            # rotated: approximate with bounding circle of pad + guard
            r = math.hypot(w, h)/2
            for L in ls: obst[L].append(Obst('circ', L, px, py, r=r, net=nm))

# zone fill polys
def zone_contains(polys, x, y):
    for ps in polys:
        if ps.Contains(pcbnew.VECTOR2I(int(round(x*1e6)), int(round(y*1e6)))):
            return True
    return False

zp = {FC: [], BC: []}
znets = {FC: set(), BC: set()}
if ZONES:
    for z in b.Zones():
        for L in (FC, BC):
            try:
                if not z.IsOnLayer(L): continue
                ps = z.GetFilledPolysList(L)
            except Exception:
                continue
            if ps is not None and ps.OutlineCount() > 0:
                zp[L].append(ps); znets[L].add(z.GetNetname())

NX = int(round((X1-X0)/CELL))
NY = int(round((Y1-Y0)/CELL))
HWT = TW/2.0
HWV = VIA_D/2.0

def build(lay, own_hw):
    m = [['.' for _ in range(NX)] for _ in range(NY)]
    ob = obst[lay]
    # bucket by row for speed
    for j in range(NY):
        py = Y0 + (j+0.5)*CELL
        for i in range(NX):
            px = X0 + (i+0.5)*CELL
            if i % 1: pass
            if HIDEZONE == 0 and ZONES and znets[lay] and lay in znets and False:
                pass
            blocked = False
            for o in ob:
                reach = own_hw + CLR
                if o.kind == 'seg':
                    if d_seg(px, py, o.x1, o.y1, o.x2, o.y2) < o.hw + reach: blocked = True; break
                elif o.kind == 'circ':
                    if math.hypot(px-o.x1, py-o.y1) < o.r + reach: blocked = True; break
                elif o.kind == 'rect' or o.kind == 'rr':
                    if d_rect(px, py, o.x1, o.y1, o.x2, o.y2) - (o.r if o.kind=='rr' else 0) < reach: blocked = True; break
                elif o.kind == 'cap':
                    if d_seg(px, py, o.x1, o.y1, o.x2, o.y2) - o.r < reach: blocked = True; break
            if not blocked and ZONES and znets[lay]:
                if zone_contains(zp[lay], px, py):
                    m[j][i] = ','
                    continue
            m[j][i] = '#' if blocked else '.'
    return m

def show(title, m, extra=None):
    print('--- %s   x from %.2f step %.2f' % (title, X0 + 0.5*CELL, CELL))
    rul = []
    for i in range(NX):
        xv = X0 + (i+0.5)*CELL
        if abs(xv/5.0 - round(xv/5.0)) < 0.02: rul.append('|')
        elif abs(xv - round(xv)) < 0.02: rul.append(str(int(round(xv)) % 10))
        else: rul.append(' ')
    print('       ' + ''.join(rul))
    for j in range(NY):
        y = Y0 + (j+0.5)*CELL
        line = ''.join(str(c) if isinstance(c,str) else ('V' if c else '-') for c in m[j])
        if extra: line += '   ' + ''.join(extra[j])
        print('%6.2f %s' % (y, line))

mf = build(FC, HWT)
mb = build(BC, HWT)
mv = [[False]*NX for _ in range(NY)]
for j in range(NY):
    py = Y0 + (j+0.5)*CELL
    for i in range(NX):
        px = X0 + (i+0.5)*CELL
        ok = True
        for lay in (FC, BC):
            for o in obst[lay]:
                reach = HWV + CLR
                if o.kind == 'seg':
                    if d_seg(px, py, o.x1, o.y1, o.x2, o.y2) < o.hw + reach: ok = False; break
                elif o.kind == 'circ':
                    if math.hypot(px-o.x1, py-o.y1) < o.r + reach: ok = False; break
                elif o.kind in ('rect','rr'):
                    if d_rect(px, py, o.x1, o.y1, o.x2, o.y2) - (o.r if o.kind=='rr' else 0) < reach: ok = False; break
                elif o.kind == 'cap':
                    if d_seg(px, py, o.x1, o.y1, o.x2, o.y2) - o.r < reach: ok = False; break
            if not ok: break
        mv[j][i] = ok
show('F.Cu track sw=%.2f' % TW, mf)
show('B.Cu track sw=%.2f' % TW, mb)
show('VIA d=%.2f both layers' % VIA_D, mv)
print('LEGEND: . free  # blocked  , zone-fill copper (passable if refilled)')
