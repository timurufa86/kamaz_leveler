import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
from lib import GEOM, local_box, rot_pt, gbox, inside, SLOT_R

P = json.load(open('_work/placements.json'))
def boxof(ref, cx, cy, deg):
    b = local_box(ref); lc = ((b[0]+b[2])/2, (b[1]+b[3])/2)
    return gbox(b, lc, (cx, cy), deg)
B = {r: boxof(r, *P[r][:3]) for r in P}
THRU = {r: GEOM[r].get('thru', []) for r in GEOM}
CONN = {'J1','J2','J3','J4','J5','J6','J7','J8'}
def ov(a,b,g=0.0):
    return not (a[2]+g<=b[0] or b[2]+g<=a[0] or a[3]+g<=b[1] or b[3]+g<=a[1])
def pads_of(ref, cx, cy, deg):
    b = local_box(ref); lc = ((b[0]+b[2])/2, (b[1]+b[3])/2)
    return [gbox(pb, lc, (cx, cy), deg) for pb in THRU.get(ref, ())]

MOD = B['U7']
def ok(ref, cx, cy, deg, skip_self=True):
    bx = boxof(ref, cx, cy, deg)
    side = P[ref][3]
    m = 0.10 if ref in CONN else (0.05 if ref=='U7' else 0.30)
    for i in range(9):
        t=i/8
        for pt in ((bx[0]+(bx[2]-bx[0])*t, bx[1]),(bx[0]+(bx[2]-bx[0])*t, bx[3]),(bx[0],bx[1]+(bx[3]-bx[1])*t),(bx[2],bx[1]+(bx[3]-bx[1])*t)):
            if not inside(pt[0],pt[1],m): return False
    for sx in (-27.,27.):
        for sy in (-29.,29.):
            nx=min(max(sx,bx[0]),bx[2]); ny=min(max(sy,bx[1]),bx[3])
            if math.hypot(nx-sx, ny-sy) < SLOT_R+0.5: return False
    if side=='F' and ref!='U7' and ov(bx, MOD): return False
    mypads = pads_of(ref, cx, cy, deg)
    for r2 in B:
        if r2 == ref: continue
        if P[r2][3] == side:
            if ov(bx, B[r2]): return False
        else:
            for pb in pads_of(r2, *P[r2][:3]):
                for pa in mypads:
                    if ov(pa, pb): return False
                if ov(bx, pb): return False
    return True

POWER='J1 J2 F1 Q1 L4 L2 L3 D1 D2 D3 C2 C28 C4 C5 C7 C6 C10 C11 C12 C30 C13 C14 U1 U2 C1 C3 R1 R2 R3 R4 R5 R6 R9 R10'.split()
moved=[]
for ref in POWER:
    cx,cy,deg,side = P[ref]
    b = B[ref]
    if b[1] <= -14.25: continue
    need = -14.25 - b[1]
    done=False
    for k in range(1, 81):
        dy = -0.05*k
        if dy > -need - 2.5: break
        if ok(ref, cx, cy+dy, deg):
            P[ref][1] = round(cy+dy,3); B[ref]=boxof(ref, cx, cy+dy, deg)
            moved.append((ref, round(dy,2))); done=True; break
    if not done: moved.append((ref,'FAILED'))
print('nudged down:', moved)
json.dump(P, open('_work/placements.json','w'), indent=0)
