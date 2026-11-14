import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
NET=sys.argv[1]
b=pcbnew.LoadBoard('_work/s3.kicad_pcb')
n=0
for t in list(b.GetTracks()):
    if t.Type()==pcbnew.PCB_TRACE_T and t.GetLayer()==pcbnew.B_Cu and t.GetNetname()==NET:
        b.Remove(t); n+=1
pcbnew.SaveBoard(sys.argv[2],b)
print("del",n,"of",NET)
