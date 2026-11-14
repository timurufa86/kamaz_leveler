import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard('../esp32s3/kamaz_leveler_s3.kicad_pcb')
OWNS = set('U1 U2 U3 U4 U5 U6 U8 U11 U12 J1 J2 J3 J4 J5 J6 J7 J8 U7'.split())
RAIL = {'+3V3','+5V','+12V','+24V'}
pads = []   # (ref, net, x, y)
for fp in b.GetFootprints():
    r = fp.GetReference()
    if r.startswith('H'): continue
    for p in fp.Pads():
        n = p.GetNetname()
        if not n: continue
        pos = p.GetPosition()
        pads.append((r, n, pos.x/1e6, pos.y/1e6))
owner = {}
detail = {}
for fp in b.GetFootprints():
    r = fp.GetReference()
    if r.startswith('H') or r in OWNS: continue
    best = None; bd = 1e9
    mine = [(n,x,y) for (rr,n,x,y) in pads if rr == r and n in RAIL]
    for (n,x,y) in mine:
        for (rr,nn,xx,yy) in pads:
            if nn != n or rr not in OWNS: continue
            d = math.hypot(x-xx, y-yy)
            if d < bd: bd, best = d, rr
    owner[r] = best
    detail[r] = (best, round(bd,1))
json.dump(owner, open('_work/owner.json','w'), indent=0)
from collections import Counter
print('owner assignment:', Counter(owner.values()).most_common())
print('samples:', {k:detail[k] for k in ('C18','C21','C22','C29','C35','C15','C4','C5','C7','C11','C12','C30','C13','C14','C16','C20','C17','C19','C1','C3','C10','R23','R11','R29') if k in detail})
