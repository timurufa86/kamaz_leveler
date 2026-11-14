import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
import pcbnew
from lib import inside, SLOT_R, RC, BW, BH

PCB="kamaz_leveler_ruichi158.kicad_pcb"
b = pcbnew.LoadBoard(PCB)
R = {}

# ---------- outline ----------
edges=[]
for d in b.GetDrawings():
    if d.GetLayer()!=pcbnew.Edge_Cuts: continue
    st=d.GetShape()
    if st==pcbnew.SHAPE_T_SEGMENT:
        s,e=d.GetStart(),d.GetEnd()
        edges.append(('line',(s.x/1e6,s.y/1e6),(e.x/1e6,e.y/1e6)))
    elif st==pcbnew.SHAPE_T_ARC:
        s,m,e=d.GetStart(),d.GetArcMid(),d.GetEnd()
        c=d.GetCenter(); rad=d.GetRadius()/1e6
        ax,ay=s.x/1e6,s.y/1e6; bx,by=m.x/1e6,m.y/1e6; cx,cy=e.x/1e6,e.y/1e6
        edges.append(('arc',(ax,ay),(cx,cy),'c',(c.x/1e6,c.y/1e6),'r',rad,'mid',(bx,by)))
xs=[];ys=[]
for e in edges:
    for p in (e[1],e[2]):
        xs.append(p[0]); ys.append(p[1])
xs=[round(v,4) for v in xs]; ys=[round(v,4) for v in ys]
print('EDGE.CUTS items:', len(edges), ' lines:', sum(1 for e in edges if e[0]=='line'), ' arcs:', sum(1 for e in edges if e[0]=='arc'))
print('bbox x: %.4f .. %.4f  (W=%.4f)' % (min(xs),max(xs),max(xs)-min(xs)))
print('bbox y: %.4f .. %.4f  (H=%.4f)' % (min(ys),max(ys),max(ys)-min(ys)))
print('--- arcs ---')
for e in edges:
    if e[0]=="arc":
        print('  center=(%.4f, %.4f) R=%.4f  start=(%.3f,%.3f) mid=(%.3f,%.3f) end=(%.3f,%.3f)' %
              (e[4][0],e[4][1],e[6],e[1][0],e[1][1],e[8][0],e[8][1],e[2][0],e[2][1]))
# connectivity: every endpoint must appear exactly twice
from collections import Counter
cnt=Counter()
for e in edges:
    for p in (e[1],e[2]): cnt[(round(p[0],3),round(p[1],3))]+=1
bad=[k for k,v in cnt.items() if v!=2]
print('outline vertex check: endpoints used twice?', 'OK' if not bad else bad)

# ---------- footprints ----------
fps=list(b.GetFootprints())
print('footprints:', len(fps), ' tracks:', len(b.GetTracks()), ' zones:', len(b.Zones()),
      ' vias:', sum(1 for t in b.GetTracks() if t.Type()==pcbnew.PCB_VIA_T))
rows=[]
for fp in fps:
    ref=fp.GetReference(); p=fp.GetPosition()
    side='F' if fp.GetLayer()==pcbnew.F_Cu else 'B'
    crtL = pcbnew.F_CrtYd if side=='F' else pcbnew.B_CrtYd
    cy=fp.GetCourtyard(crtL); bb=cy.BBox()
    box=(bb.GetX()/1e6, bb.GetY()/1e6, (bb.GetX()+bb.GetWidth())/1e6, (bb.GetY()+bb.GetHeight())/1e6)
    pads=[(pp.GetBoundingBox().GetX()/1e6, pp.GetBoundingBox().GetY()/1e6,
           (pp.GetBoundingBox().GetX()+pp.GetBoundingBox().GetWidth())/1e6,
           (pp.GetBoundingBox().GetY()+pp.GetBoundingBox().GetHeight())/1e6) for pp in fp.Pads()]
    rows.append(dict(ref=ref, fp=str(fp.GetFPID().GetLibItemName()), x=p.x/1e6, y=p.y/1e6,
                     rot=fp.GetOrientationDegrees(), side=side, box=box, pads=pads,
                     h=box[3]-box[1]))
rows.sort(key=lambda r:r['ref'])
json.dump(rows, open('_work/board_rows.json','w'), indent=0)

CONN={'J1','J2','J3','J4','J5','J6','J7','J8'}
def ov(a,bb):
    return not (a[2]<=bb[0] or bb[2]<=a[0] or a[3]<=bb[1] or bb[3]<=a[1])
def gap(a,bb):
    dx=max(a[0]-bb[2], bb[0]-a[2]); dy=max(a[1]-bb[3], bb[1]-a[3])
    return round(max(dx,dy),3)

# containment
out=[]
for r in rows:
    m = 0.10 if r['ref'] in CONN else (0.05 if r['ref']=='U7' else 0.30)
    for pt in [(x,y) for x in (r['box'][0],r['box'][2]) for y in (r['box'][1],r['box'][3])]:
        if not inside(pt[0],pt[1],m): out.append((r['ref'],pt,m)); break
print('containment violations (box corners, margin per class):', len(out), out[:5])

# same-side courtyard overlaps
ovl=[]; ming=99
for i in range(len(rows)):
    for j in range(i+1,len(rows)):
        a,bb=rows[i],rows[j]
        if a['side']!=bb['side']: continue
        if ov(a['box'],bb['box']): ovl.append((a['ref'],bb['ref']))
        else:
            g=gap(a['box'],bb['box'])
            if g<0.30: ming=min(ming,g)
print('same-side courtyard overlaps:', len(ovl), ovl[:5], ' min gap:', ming)

# cross-side copper conflicts (any pad of one part vs any pad of other, opposite side)
cross=[]
for i in range(len(rows)):
    for j in range(i+1,len(rows)):
        a,bb=rows[i],rows[j]
        if a['side']==bb['side']: continue
        for pa in a['pads']:
            for pb in bb['pads']:
                if ov(pa,pb): cross.append((a['ref'],bb['ref']))
print('cross-side pad overlaps (copper on both layers):', len(cross), cross[:5])

# slot keepout with bodies AND pads
sl=[]
for r in rows:
    for sx in (-27.,27.):
        for sy in (-29.,29.):
            for bx in [r['box']]+r['pads']:
                nx=min(max(sx,bx[0]),bx[2]); ny=min(max(sy,bx[1]),bx[3])
                d=math.hypot(nx-sx,ny-sy)
                if d < SLOT_R+0.5: sl.append((r['ref'],round(d,3)))
print('slot-keepout violations (R3.75+0.5):', len(set(sl)), sorted(set(sl))[:8])

# antenna band
mod=[r for r in rows if r['ref']=='U7'][0]
band_lo = mod['box'][3]-5.0
inb=[r['ref'] for r in rows if r['ref']!='U7' and r['box'][3]>band_lo and r['box'][0]<mod['box'][2] and r['box'][2]>mod['box'][0]]
print('U7 box:', [round(v,3) for v in mod['box']], ' rot:', mod['rot'])
print('antenna band y >= %.3f, x in [%.3f,%.3f]: footprints inside: %d' % (band_lo, mod['box'][0], mod['box'][2], len(inb)), inb)

print('\n=== CONNECTORS ===')
for ref in ('J1','J2','J3','J4','J5','J6','J7','J8'):
    r=[r for r in rows if r['ref']==ref][0]
    print('  %-3s x=%7.3f y=%7.3f rot=%6.1f box=[%7.3f,%7.3f .. %7.3f,%7.3f]' % (ref,r['x'],r['y'],r['rot'],*r['box']))
print('\n=== U7 ===')
r=mod
print('  x=%7.3f y=%7.3f rot=%6.1f box=[%.3f,%.3f .. %.3f,%.3f]' % (r['x'],r['y'],r['rot'],*r['box']))

print('\n=== y distribution ===')
for lo,hi in ((-42.75,-28.5),(-28.5,-14.25),(-14.25,0),(0,14.25),(14.25,28.5),(28.5,42.75)):
    n=[r['ref'] for r in rows if lo<=r['box'][1] and r['box'][3]<=hi]
    print('   y %.2f..%.2f : %d parts' % (lo,hi,len(n)))
