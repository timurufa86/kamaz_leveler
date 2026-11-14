import io,sys,statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
tr=list(b.GetTracks())
segs=[t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]
vias=[t for t in tr if t.Type()==pcbnew.PCB_VIA_T]
L=sorted(MM(s.GetLength()) for s in segs)
n=len(L)
print("segs",n,"total",round(sum(L),2),"median",round(statistics.median(L),3))
bins=[(0,0.15),(0.15,0.3),(0.3,0.5),(0.5,0.8),(0.8,1.5),(1.5,3),(3,10),(10,1e9)]
for a,c in bins:
    k=sum(1 for x in L if a<=x<c)
    print(f"  [{a},{c}): {k} ({100*k/n:.1f}%)")
# vias
print("vias",len(vias))
from collections import Counter
print("via sizes",Counter((MM(v.GetWidth()),MM(v.GetDrill())) for v in vias).most_common(5))
print("track widths",Counter(MM(s.GetWidth()) for s in segs).most_common(8))
# net-level
per={}
for s in segs:
    d=per.setdefault(s.GetNetname(),[0,0.0])
    d[0]+=1; d[1]+=MM(s.GetLength())
rows=sorted(per.items(),key=lambda kv:-kv[1][0])
print("nets with tracks:",len(rows))
for k,(c,l) in rows[:25]:
    print(f"  {k:<22} segs={c:<5} len={l:8.2f}")
# short-seg concentration
short=[s for s in segs if MM(s.GetLength())<0.8]
c=Counter(s.GetNetname() for s in short)
print("segments<0.8:",len(short),"top nets:",c.most_common(10))
# zone
zx,zy0,zy1=40.73,13.84,39.83
inz=[s for s in segs if min(MM(s.GetStart().x),MM(s.GetEnd().x))>=zx-0.01 and min(MM(s.GetStart().y),MM(s.GetEnd().y))>=zy0-0.01 and max(MM(s.GetStart().y),MM(s.GetEnd().y))<=zy1+0.01]
zl=sorted(MM(s.GetLength()) for s in inz)
print("zone segs",len(inz),"median",round(statistics.median(zl),3),"cu",round(sum(zl),2))
cx=Counter(s.GetNetname() for s in inz)
print("zone nets:",cx.most_common(15))
