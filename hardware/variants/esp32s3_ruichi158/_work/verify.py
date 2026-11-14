import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
from lib import GEOM, local_box, inside, W, H, BW, BH, RC, SLOT_R

P = json.load(open('_work/placements.json'))
def box(ref):
    cx, cy, deg, side = P[ref]
    b = local_box(ref)
    lcx, lcy = (b[0]+b[2])/2, (b[1]+b[3])/2
    a = math.radians(deg); c,s = math.cos(a), math.sin(a)
    pts=[]
    for x in (b[0],b[2]):
        for y in (b[1],b[3]):
            px, py = x-lcx, y-lcy
            pts.append((cx + px*c + py*s, cy - px*s + py*c))
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    return (round(min(xs),3), round(min(ys),3), round(max(xs),3), round(max(ys),3))
B = {r: box(r) for r in P}
CONN = ['J1','J2','J3','J4','J5','J6','J7','J8']
MOD = B['U7']

def ov(a,b,gap=0.0):
    return not (a[2]+gap<=b[0] or b[2]+gap<=a[0] or a[3]+gap<=b[1] or b[3]+gap<=a[1])

print('=== same-side courtyard overlaps (gap < 0.30) ===')
bad=0; mind=99
for r1 in P:
    for r2 in P:
        if r1>=r2: continue
        if P[r1][3]!=P[r2][3]: continue
        a,b = B[r1],B[r2]
        if ov(a,b):
            d = 0.0; bad+=1
            print(f'  OVERLAP {r1} / {r2}  {a} {b}')
        else:
            dx = max(a[0]-b[2], b[0]-a[2]); dy = max(a[1]-b[3], b[1]-a[3])
            d = max(dx,dy); mind=min(mind,d)
            if d < 0.30: print(f'  tight {d:.3f} {r1}/{r2}')
print('overlaps:', bad, 'min gap between same-side courtyards:', round(mind,3))

print('=== containment ===')
for r in sorted(P):
    b=B[r]; m = 0.10 if r in CONN else (0.05 if r=='U7' else 0.30)
    if not all(inside(x,y,m) for x in (b[0],b[1],b[2]) for y in ()) : pass
    ok = True
    for i in range(7):
        t=i/6
        for pt in ((b[0]+(b[2]-b[0])*t,b[1]),(b[0]+(b[2]-b[0])*t,b[3]),(b[0],b[1]+(b[3]-b[1])*t),(b[2],b[1]+(b[3]-b[1])*t)):
            if not inside(pt[0],pt[1],m): ok=False
    if not ok: print('  OUTSIDE/too close:', r, b, 'margin', m)

print('=== antenna band (module end y=%.2f, band y>=%.2f) ==='%(MOD[3], MOD[3]-5))
for r in sorted(P):
    if r=='U7': continue
    b=B[r]
    if b[3] > MOD[3]-5.0 and b[2] > MOD[0] and b[0] < MOD[2]:
        print('  IN ANTENNA BAND:', r, b)
print('=== module fence (F side, same side only) ===')
for r in sorted(P):
    if P[r][3]!='F' or r=='U7': continue
    if ov(B[r], MOD): print('  OVERLAPS MODULE:', r, B[r])
print('=== slot keepout (r=4.25) ===')
for r in sorted(P):
    b=B[r]
    for sx in (-27.,27.):
        for sy in (-29.,29.):
            nx = min(max(sx,b[0]),b[2]); ny=min(max(sy,b[1]),b[3])
            d = math.hypot(nx-sx, ny-sy)
            if d < SLOT_R+0.5: print(f'  SLOT VIOLATION {r} d={d:.2f} box={b}')
print('=== ascii map (1 mm/char, F top / B bottom) ===')
for side in ('F','B'):
    grid=[['.']*60 for _ in range(86)]
    for r in P:
        if P[r][3]!=side: continue
        b=B[r]
        ch = '#' if r=='U7' else (r[0] if r[0] in 'UJ' else ('c' if r[0]=='C' else ('d' if r[0]=='D' else ('r' if r[0]=='R' else ('l' if r[0]=='L' else 'x')))))
        for j in range(max(0,int(b[1]+42.75)), min(85,int(b[3]+42.75)+1)):
            for i in range(max(0,int(b[0]+30)), min(59,int(b[2]+30)+1)):
                if grid[85-j][i]=='.': grid[85-j][i]=ch
    print('--- side',side)
    for row in grid: print(''.join(row))
