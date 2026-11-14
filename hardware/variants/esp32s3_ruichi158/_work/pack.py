import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
import numpy as np
from lib import GEOM, local_box, rot_pt, gbox, W, H, BW, BH, RC, SLOT_R, inside

# ---------------- board raster ----------------
STEP = 0.1
X0, Y0 = -30.0, -42.75
NX, NY = int(round(W/STEP)), int(round(H/STEP))          # 600 x 855
EDGE_M = 0.30          # body margin from board edge
SLOT_M = 0.50          # slot keepout margin
ANT_Y  = 42.75 - 5.0   # antenna band lower bound  (band: y >= 37.75 ? )
# module geometry (filled in below)
MOD = {}

def cell(x, y):
    return int(round((x - X0)/STEP)), int(round((y - Y0)/STEP))

def base_mask(side):
    xs = X0 + (np.arange(NX) + 0.5)*STEP
    ys = Y0 + (np.arange(NY) + 0.5)*STEP
    X, Y = np.meshgrid(xs, ys)
    ok = (np.abs(X) <= BW - EDGE_M) & (np.abs(Y) <= BH - EDGE_M)
    # rounded corners
    for cx in (-27.0, 27.0):
        for cy in (-39.75, 39.75):
            m = (np.abs(X) > 27.0) & (np.abs(Y) > 39.75) & (np.sign(X) == np.sign(cx)) & (np.sign(Y) == np.sign(cy))
            ok &= ~m | (((X-cx)**2 + (Y-cy)**2) <= (RC - EDGE_M)**2)
    # slots (semicircle keepout, full disk)
    for sx in (-27.0, 27.0):
        for sy in (-29.0, 29.0):
            ok &= ((X-sx)**2 + (Y-sy)**2) >= (SLOT_R + SLOT_M)**2
    # antenna band clear on both sides
    ok &= ~((X >= MOD['x0']-0.1) & (X <= MOD['x1']+0.1) & (Y >= MOD['y1']-5.0-0.05))
    if side == 'F':
        # module courtyard fenced off for every other footprint
        ok &= ~((X >= MOD['x0']-0.1) & (X <= MOD['x1']+0.1) & (Y >= MOD['y0']-0.1) & (Y <= MOD['y1']+0.1))
    return ~ok          # True = blocked

def prefix(occ):
    p = np.zeros((NY+1, NX+1), dtype=np.int32)
    p[1:, 1:] = np.cumsum(np.cumsum(occ.astype(np.int32), axis=0), axis=1)
    return p

def rect_sum(p, i0, j0, i1, j1):
    if i1 < i0 or j1 < j0: return 1
    return int(p[j1+1, i1+1] - p[j0, i1+1] - p[j1+1, i0] + p[j0, i0])

def mark(occ, bx, pad=0.0):
    x0,y0,x1,y1 = bx
    i0,j0 = cell(x0-pad, y0-pad); i1,j1 = cell(x1+pad, y1+pad)
    i0=max(i0,0); j0=max(j0,0); i1=min(i1,NX-1); j1=min(j1,NY-1)
    occ[j0:j1+1, i0:i1+1] = 1

# ---------------- part geometry ----------------
def box_size(ref, deg):
    b = local_box(ref)
    w,h = b[2]-b[0], b[3]-b[1]
    return (h,w) if deg % 180 else (w,h)

def bbox_for(ref, cx, cy, deg):
    w,h = box_size(ref, deg)
    return (cx-w/2, cy-h/2, cx+w/2, cy+h/2)

# ---------------- rules data ----------------
GLOBAL_NETS = {'GND','+3V3','+5V','+12V','+24V'}
TALL = {'L2','F1','L4','L3','C2','C28','C6','C13','C18','D1'}   # > 5 mm high parts

POWER = set('J1 J2 F1 Q1 L4 L2 L3 D1 D2 D3 C2 C28 C4 C5 C7 C6 C10 C11 C12 C30 C13 C14 U1 U2 C1 C3 R1 R2 R3 R4 R5 R6 R9 R10'.split())
# 3V3 regulator group + 8V sensor supply group: belong with logic / near module

# band limits (y) per class
BAND = {'POWER': (-42.45, -14.25), 'LOGIC': (-13.5, 27.5)}

PINNED = {   # ref: (cx, cy, rot)  -- courtyard/box centre
    'U7': (0.0, 10.7575, 270),
    'J1': (9.0,   -40.835, 90),
    'J2': (-11.0, -40.835, 90),
    'J3': (-15.5, 34.485, 180),
    'J4': (-19.6, 37.025, 180),
    'J5': (20.0,  31.945, 180),
    'J6': (15.6,  37.025, 180),
    'J7': (27.885, 8.0, 0),
    'J8': (27.885, -5.0, 0),
}

def main():
    refs = [r for r in GEOM if not r.startswith('H')]
    # module box at pinned position
    b = local_box('U7')
    cx, cy, deg = PINNED['U7']
    MOD.update(dict(zip(('x0','y0','x1','y1'), bbox_for('U7', cx, cy, deg))))

    occ = {'F': base_mask('F'), 'B': base_mask('B')}
    pref = {s: prefix(occ[s]) for s in occ}
    placed = {}          # ref -> (cx,cy,deg,side,box)
    nets = {r: set(GEOM[r]['nets']) for r in refs}

    THRU = {r: GEOM[r].get('thru', []) for r in GEOM}

    def do_place(ref, cx, cy, deg):
        side = GEOM[ref]['side']
        other = 'B' if side == 'F' else 'F'
        bx = bbox_for(ref, cx, cy, deg)
        mark(occ[side], bx, pad=0.25)
        pref[side] = prefix(occ[side])
        b0 = local_box(ref)
        lc = ((b0[0]+b0[2])/2, (b0[1]+b0[3])/2)
        for pb in THRU.get(ref, ()):
            mark(occ[other], gbox(pb, lc, (cx, cy), deg), pad=0.30)
        pref[other] = prefix(occ[other])
        placed[ref] = (round(cx,3), round(cy,3), deg, side, bx)

    # 1. pinned
    for ref,(cx,cy,deg) in PINNED.items():
        do_place(ref, cx, cy, deg)

    # 2. pack the rest
    rest = [r for r in refs if r not in PINNED]
    rest.sort(key=lambda r: -(lambda b: (b[2]-b[0])*(b[3]-b[1]))(local_box(r)))

    OWNERS = ['U1','U2','U3','U4','U5','U6','U8','U11','U12','J1','J2','J3','J4','J5','J6','J7','J8']
    ORIG = json.load(open('_work/orig_pos.json'))
    RAILOWN = json.load(open('_work/owner.json'))
    OWN = {}
    for _r in refs:
        if _r in ORIG:
            px, py = ORIG[_r]; bd = 1e9; bo = None
            for o in OWNERS:
                if o == _r or o not in ORIG: continue
                d = math.hypot(px-ORIG[o][0], py-ORIG[o][1])
                if d < bd: bd, bo = d, o
            OWN[_r] = bo

    def anchor(ref):
        acc = []
        for n in nets[ref] - GLOBAL_NETS:
            share = [p for p in placed if p != ref and n in nets.get(p, ())]
            w = 1.0/len(share) if share else 0
            for p in share:
                acc.append((placed[p][0], placed[p][1], w))
        if acc:
            tw = sum(a[2] for a in acc)
            return (sum(a[0]*a[2] for a in acc)/tw, sum(a[1]*a[2] for a in acc)/tw)
        o = RAILOWN.get(ref) or OWN.get(ref)
        if o and o in placed:
            return (placed[o][0], placed[o][1])
        return (0.0, -35.0) if ref in POWER else (0.0, 5.0)

    fails = []
    for ref in rest:
        cls = 'POWER' if ref in POWER else 'LOGIC'
        ylo, yhi = BAND[cls]
        if ref in TALL and GEOM[ref]['side'] == 'B':
            yhi = min(yhi, -21.3) if cls=='POWER' else min(yhi, 27.0)
        ax, ay = anchor(ref)
        best = None
        for relax in (0.0, 4.0, 9.0, 15.0, 22.0):
            lo, hi = ylo - (relax if ay < 0 else 0), yhi + relax
            for deg in (0, 90) if max(box_size(ref,0)) <= 8 else (0, 90):
                w,h = box_size(ref, deg)
                if w > 59.0 or h > 40.0: continue
                xs = np.arange(-29.4 + w/2, 29.4 - w/2 + 1e-9, 0.25)
                ys = np.arange(lo + h/2, min(hi, BH-0.35) - h/2 + 1e-9, 0.25)
                if len(xs) == 0 or len(ys) == 0: continue
                Xg, Yg = np.meshgrid(xs, ys)
                cost = np.hypot(Xg-ax, Yg-ay) + (0.4 if deg else 0.0)
                order = np.argsort(cost, axis=None)
                side = GEOM[ref]['side']
                P = pref[side]
                for k in order[:25000]:
                    cx_ = float(Xg.flat[k]); cy_ = float(Yg.flat[k])
                    x0,y0 = cell(cx_-w/2, cy_-h/2); x1,y1 = cell(cx_+w/2, cy_+h/2)
                    if rect_sum(P, x0,y0,x1,y1) == 0:
                        best = (cx_, cy_, deg); break
                if best: break
            if best: break
        if not best:
            # last resort: plain first-fit sweep over the whole side, bottom-up
            side = GEOM[ref]['side']; P = pref[side]
            for deg in (0, 90):
                w,h = box_size(ref, deg)
                for cy_ in np.arange(-BH+0.45+h/2, BH-0.45-h/2+1e-9, 0.25):
                    for cx_ in np.arange(-29.4+w/2, 29.4-w/2+1e-9, 0.25):
                        x0,y0 = cell(cx_-w/2, cy_-h/2); x1,y1 = cell(cx_+w/2, cy_+h/2)
                        if rect_sum(P, x0,y0,x1,y1) == 0:
                            best = (float(cx_), float(cy_), deg); break
                    if best: break
                if best: break
        if not best:
            fails.append(ref); print('FAIL', ref); continue
        do_place(ref, *best)

    print('placed', len(placed), 'failed', fails)
    json.dump({k:[v[0],v[1],v[2],v[3]] for k,v in placed.items()}, open('_work/placements.json','w'), indent=0)
    # report
    for cls in ('POWER','LOGIC'):
        ys = [v[1] for k,v in placed.items() if k!='U7' and (('POWER' if k in POWER else 'LOGIC')==cls)]
        print(cls, 'n=', len(ys), 'y range', round(min(ys),2), round(max(ys),2))

if __name__ == '__main__':
    main()
