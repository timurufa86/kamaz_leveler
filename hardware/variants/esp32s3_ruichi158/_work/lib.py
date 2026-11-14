import io,sys,json,math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if True:
    pass
import pcbnew

GEOM = {r['ref']: r for r in json.load(open("_work/geom3.json"))}

def local_box(ref):
    """local (rot0, origin at 0,0) occupancy box: courtyard, else fab, else pads"""
    g = GEOM[ref]
    b = g['crt'] or (g['fab'] if (g['fab'] and g['fab'] != g['pads']) else g['pads'])
    return b

def rot_pt(x, y, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x*c + y*s, -x*s + y*c)

def box_at(ref, px, py, deg):
    """global bbox of the local occupancy box for footprint placed at (px,py,deg)"""
    x0,y0,x1,y1 = local_box(ref)
    pts = [rot_pt(x,y,deg) for x in (x0,x1) for y in (y0,y1)]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return (round(px+min(xs),3), round(py+min(ys),3), round(px+max(xs),3), round(py+max(ys),3))

def gbox(localb, lc, C, deg):
    x0,y0,x1,y1 = localb
    lcx,lcy = lc
    pts=[rot_pt(x-lcx, y-lcy, deg) for x in (x0,x1) for y in (y0,y1)]
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    return (round(C[0]+min(xs),3), round(C[1]+min(ys),3), round(C[0]+max(xs),3), round(C[1]+max(ys),3))

def dims(ref, deg):
    x0,y0,x1,y1 = local_box(ref)
    if deg % 180 == 0: return (round(x1-x0,3), round(y1-y0,3))
    return (round(y1-y0,3), round(x1-x0,3))

# board model ------------------------------------------------------------
W, H, RC = 60.0, 85.5, 3.0
BW, BH = W/2, H/2                      # 30, 42.75
SLOT_Y = 29.0; SLOT_R = 3.75
CORNER = [(27,-39.75),(-27,-39.75),(-27,39.75),(27,39.75)]

def inside(x, y, m=0.0):
    """point inside board pocket, inset by margin m (board edge = -m inside)"""
    if not (-BW+m <= x <= BW-m and -BH+m <= y <= BH-m): return False
    cx = -27.0 if x < 0 else 27.0
    cy = -39.75 if y < 0 else 39.75
    if abs(x) > 27.0 and abs(y) > 39.75:
        if (x-cx)**2 + (y-cy)**2 > (RC-m)**2: return False
    # slot keepout: semicircle centre (+-27, +-29) radius 3.75, plus clearance 0.5
    for sx in (-27.0, 27.0):
        for sy in (-29.0, 29.0):
            if (x-sx)**2 + (y-sy)**2 < (SLOT_R+0.5)**2: return False
    return True

def box_inside(bx, m=0.0, n=6):
    x0,y0,x1,y1 = bx
    pts = []
    for i in range(n+1):
        t = i/n
        pts += [(x0+(x1-x0)*t, y0), (x0+(x1-x0)*t, y1), (x0, y0+(y1-y0)*t), (x1, y0+(y1-y0)*t)]
    return all(inside(x,y,m) for x,y in pts)

def overlap(a, b, gap=0.0):
    return not (a[2]+gap <= b[0] or b[2]+gap <= a[0] or a[3]+gap <= b[1] or b[3]+gap <= a[1])
