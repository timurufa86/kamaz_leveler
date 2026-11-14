import io,sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
fps=list(b.GetFootprints())
u7=[f for f in fps if f.GetPadCount()==44]
print("44-pad fps:",[(f.GetReference(),f.GetFPID().GetLibItemName()) for f in u7])
u7=u7[0]
print("U7 ref",u7.GetReference(),"pos",MM(u7.GetPosition().x),MM(u7.GetPosition().y))
rows={}
for p in u7.Pads():
    num=p.GetNumber()
    rows[num]=(MM(p.GetPosition().x),MM(p.GetPosition().y),p.GetNetname(),p.GetAttribute(),p.GetShape(),MM(p.GetSize().x),MM(p.GetSize().y))
for n in sorted(rows,key=lambda s:int(s)):
    x,y,net,attr,shape,w,h=rows[n]
    print(f"{n:>3} ({x:8.3f};{y:7.3f}) net={net!r:<16} shape={shape} {w:.2f}x{h:.2f}")
print("footprints",len(fps),"pads",sum(f.GetPadCount() for f in fps))
tr=list(b.GetTracks())
from collections import Counter
print("segs",len([t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]),"vias",len([t for t in tr if t.Type()==pcbnew.PCB_VIA_T]))
print("zones",b.GetAreaCount())
print("nets",len(b.GetNetInfo().NetsByNetcode()))
