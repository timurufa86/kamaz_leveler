import sys,math,hashlib
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
u7=[f for f in b.GetFootprints() if f.GetReference()=='U7'][0]
FREE={'6','7','8','15','16','18'}
pads={p.GetNumber():p for p in u7.Pads()}
print("--- freed pads: net / copper contact")
for n in sorted(FREE,key=int):
    p=pads[n]; pos=p.GetPosition()
    print("  pad %2s net=%-30s"%(n,p.GetNetname()))
touched=set()
for t in b.GetTracks():
    nm=t.GetNetname()
    if nm.startswith('unconnected-'): touched.add(nm)
print("tracks on any unconnected-* net:",len([t for t in b.GetTracks() if t.GetNetname().startswith('unconnected-')]))
print("--- power pads")
for n in ('1','2','21','22','23','43','44'):
    print("  pad %2s %s"%(n,pads[n].GetNetname()))
print("--- nets in file (text count)")
txt=open('kamaz_leveler_s3.kicad_pcb',encoding='utf-8').read()
print("  (net N \"name\") declarations:",txt.count('\n\t(net '))
tr=list(b.GetTracks())
seg=[t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]
via=[t for t in tr if t.Type()==pcbnew.PCB_VIA_T]
print("--- counters: fps=%d pads=%d u7pads=%d segs=%d vias=%d zones=%d"%(len(list(b.GetFootprints())),
   sum(f.GetPadCount() for f in b.GetFootprints()),u7.GetPadCount(),len(seg),len(via),b.GetAreaCount()))
L=sorted(MM(s.GetLength()) for s in seg)
import statistics
print("--- copper=%.2f median=%.4f p10=%.4f p90=%.4f"%(sum(L),statistics.median(L),L[len(L)//10],L[9*len(L)//10]))
print("--- /INT_MCU detail")
ns=[s for s in seg if s.GetNetname()=='/INT_MCU']; nv=[v for v in via if v.GetNetname()=='/INT_MCU']
print("   segs=%d vias=%d len=%.3f"%(len(ns),len(nv),sum(MM(s.GetLength()) for s in ns)))
for s in ns:
    print("     %s (%.2f,%.2f)-(%.2f,%.2f) %.2f"%(b.GetLayerName(s.GetLayer()),MM(s.GetStart().x),MM(s.GetStart().y),MM(s.GetEnd().x),MM(s.GetEnd().y),MM(s.GetLength())))
print("--- sha256")
for p in ('_work/BACKUP_pcb_before_repin.kicad_pcb','kamaz_leveler_s3.kicad_pcb'):
    print("  ",hashlib.sha256(open(p,'rb').read()).hexdigest(),p)
