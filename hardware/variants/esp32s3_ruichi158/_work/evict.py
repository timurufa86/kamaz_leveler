import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
import numpy as np
import pack
from lib import local_box, gbox

P = json.load(open('_work/placements.json'))
pack.MOD.update(dict(zip(('x0','y0','x1','y1'), pack.bbox_for('U7', *P['U7'][:3]))))
THRU = {r: pack.GEOM[r].get('thru', []) for r in pack.GEOM}

def build(skip=()):
    occ = {'F': pack.base_mask('F'), 'B': pack.base_mask('B')}
    for ref,(cx,cy,deg,side) in P.items():
        if ref in skip: continue
        other = 'B' if side=='F' else 'F'
        pack.mark(occ[side], pack.bbox_for(ref,cx,cy,deg), pad=0.25)
        b0 = local_box(ref); lc = ((b0[0]+b0[2])/2,(b0[1]+b0[3])/2)
        for pb in THRU.get(ref,()):
            pack.mark(occ[other], gbox(pb, lc, (cx,cy), deg), pad=0.30)
    return occ

def find_slot(occ, w, h, ylo, yhi, step=0.25):
    Pp = pack.prefix(occ["F"])
    out=[]
    for cy in np.arange(ylo+h/2, yhi-h/2, step):
        for cx in np.arange(-29.4+w/2, 29.4-w/2, step):
            i0,j0 = pack.cell(cx-w/2, cy-h/2); i1,j1 = pack.cell(cx+w/2, cy+h/2)
            if pack.rect_sum(Pp,i0,j0,i1,j1)==0: out.append((round(cx,2),round(cy,2)))
    return out

occ = build()
print('D2 slot in bottom third (none):', len(find_slot(occ, 9.89, 6.79, -42.4, -14.3)) )
# candidates to evict: F side, small, fully inside bottom third
cands=[]
for ref,(cx,cy,deg,side) in P.items():
    if side!='F' or ref in ('U7','J1','J2'): continue
    b = pack.bbox_for(ref,cx,cy,deg)
    if b[3] <= -14.3 and (b[2]-b[0])*(b[3]-b[1]) < 22: cands.append((ref,b))
print('eviction candidates:', len(cands))
for ref,b in sorted(cands, key=lambda t:(t[1][2]-t[1][0])*(t[1][3]-t[1][1])):
    occ2 = build(skip={ref})
    slots = find_slot(occ2, 9.89, 6.79, -42.4, -14.3)
    if not slots:
        continue
    # try to relocate the evicted part above the line
    w = b[2]-b[0]; h = b[3]-b[1]
    cy0 = P[ref][1]; deg = P[ref][2]
    new = find_slot(occ2, w, h, -14.2, 27.0, 0.25)
    if not new: 
        continue
    print('candidate', ref, 'unblocks', len(slots), 'slots; relocation options', len(new))
    # apply: move ref to the spot closest to its old position
    best = min(new, key=lambda s: (s[0]-P[ref][0])**2 + (s[1]-cy0)**2)
    P[ref][0], P[ref][1] = best
    slot = min(slots, key=lambda s: -(s[1]))
    P['D2'] = [slot[0], slot[1], 0, 'F']
    json.dump(P, open('_work/placements.json','w'), indent=0)
    print('  moved', ref, '->', best, '; D2 ->', slot)
    break
else:
    print('no single-part eviction works')
