import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
LIBDIR='kamaz_leveler_hw.pretty'; NAME='ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm'
lib=pcbnew.FootprintLoad(LIBDIR,NAME)
print("lib loaded:",lib is not None,"pads",lib.GetPadCount(),"pos",MM(lib.GetPosition().x),MM(lib.GetPosition().y),
      "orient",lib.GetOrientationDegrees())
print("lib items:",len(list(lib.GraphicalItems())))
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
u7=[f for f in b.GetFootprints() if f.GetReference()=='U7'][0]
print("board U7 pads",u7.GetPadCount(),"pos",MM(u7.GetPosition().x),MM(u7.GetPosition().y),"orient",u7.GetOrientationDegrees())
print("board items:",len(list(u7.GraphicalItems())))
print("path board:",u7.GetPath().AsString() if hasattr(u7.GetPath(),'AsString') else u7.GetPath())
print("path lib  :",lib.GetPath().AsString() if hasattr(lib.GetPath(),'AsString') else lib.GetPath())
print("fields board: ref=%r val=%r fp=%r ds=%r desc=%r"%(u7.GetReference(),u7.GetValue(),u7.GetFPID().GetLibItemName(),
      u7.GetFieldText(pcbnew.FIELD_T_DATASHEET) if hasattr(pcbnew,'FIELD_T_DATASHEET') else '?', u7.GetFieldText(pcbnew.FIELD_T_DESCRIPTION) if hasattr(pcbnew,'FIELD_T_DESCRIPTION') else '?'))
print()
print("=== PAD DIFF (board vs lib, by number) ===")
def psig(p):
    return (p.GetNumber(),round(MM(p.GetPosition().x),4),round(MM(p.GetPosition().y),4),
            round(MM(p.GetSize().x),4),round(MM(p.GetSize().y),4),p.GetShape(),
            round(MM(p.GetDrillSize().x),4),round(MM(p.GetDrillSize().y),4),
            p.GetAttribute(),round(p.GetOrientationDegrees(),2),
            str(sorted(b.GetLayerName(l) for l in p.GetLayerSet().Seq() if l<=pcbnew.B_Cu)))
bp={p.GetNumber():psig(p) for p in u7.Pads()}
lp={p.GetNumber():psig(p) for p in lib.Pads()}
print("board pad numbers:",len(bp),"lib:",len(lp))
diff=0
for n in sorted(bp,key=lambda x:int(x)):
    if n not in lp: print("  ONLY IN BOARD pad",n); diff+=1; continue
    if bp[n]!=lp[n]:
        diff+=1; print("  DIFF pad",n); print("    board",bp[n]); print("    lib  ",lp[n])
for n in lp:
    if n not in bp: print("  ONLY IN LIB pad",n); diff+=1
print("pad diffs:",diff)
