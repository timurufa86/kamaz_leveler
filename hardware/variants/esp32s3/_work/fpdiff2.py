import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
lib=pcbnew.FootprintLoad('kamaz_leveler_hw.pretty','ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm')
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
u7=[f for f in b.GetFootprints() if f.GetReference()=='U7'][0]
def psig(p):
    return (p.GetNumber(),round(MM(p.GetPosition().x),4),round(MM(p.GetPosition().y),4),
            round(MM(p.GetSize().x),4),round(MM(p.GetSize().y),4),int(p.GetShape()),
            round(MM(p.GetDrillSize().x),4),round(MM(p.GetDrillSize().y),4),
            int(p.GetAttribute()),round(p.GetOrientationDegrees(),2),
            tuple(sorted(b.GetLayerName(l) for l in p.GetLayerSet().Seq() if l<=pcbnew.B_Cu)))
bp={p.GetNumber():psig(p) for p in u7.Pads()}
lp={p.GetNumber():psig(p) for p in lib.Pads()}
d=0
for n in sorted(bp,key=lambda x:int(x)):
    if n not in lp: print("ONLY BOARD pad",n); d+=1
    elif bp[n]!=lp[n]:
        d+=1; print("DIFF pad",n); print("   board",bp[n]); print("   lib  ",lp[n])
print("pad diffs:",d)
def isig(gi):
    t=type(gi).__name__
    s=[t,b.GetLayerName(gi.GetLayer())]
    if hasattr(gi,'GetText'): s.append(gi.GetText())
    try:
        p=gi.GetPosition(); s.append((round(MM(p.x),4),round(MM(p.y),4)))
    except Exception: pass
    if hasattr(gi,'GetTextSize'):
        try: s.append((round(MM(gi.GetTextSize().x),3),round(MM(gi.GetTextSize().y),3)))
        except Exception: pass
    if hasattr(gi,'GetTextThickness'):
        try: s.append(round(MM(gi.GetTextThickness()),3))
        except Exception: pass
    if hasattr(gi,'GetHorizJustify'):
        try: s.append(int(gi.GetHorizJustify()))
        except Exception: pass
    if hasattr(gi,'GetVertJustify'):
        try: s.append(int(gi.GetVertJustify()))
        except Exception: pass
    if hasattr(gi,'GetStart') and t!='PCB_TEXT':
        try:
            a=gi.GetStart(); c=gi.GetEnd()
            s.append((round(MM(a.x),4),round(MM(a.y),4),round(MM(c.x),4),round(MM(c.y),4)))
        except Exception: pass
    return tuple(str(x) for x in s)
bi=[isig(g) for g in u7.GraphicalItems()]
li=[isig(g) for g in lib.GraphicalItems()]
print("board item sigs:",len(bi),"lib:",len(li))
from collections import Counter
cb=Counter(bi); cl=Counter(li)
print("--- only in board:")
for k,v in (cb-cl).items(): print("   ",v,"x",k)
print("--- only in lib:")
for k,v in (cl-cb).items(): print("   ",v,"x",k)
print("--- identical:",sum((cb&cl).values()))
