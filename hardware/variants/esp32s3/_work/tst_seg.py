import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
ax,ay,bx,by=[float(v) for v in sys.argv[1:5]]
b=pcbnew.LoadBoard('_work/s3.kicad_pcb')
n=0
for t in list(b.GetTracks()):
    if t.Type()!=pcbnew.PCB_TRACE_T: continue
    p1=t.GetStart(); p2=t.GetEnd()
    P=(round(MM(p1.x),2),round(MM(p1.y),2)); Q=(round(MM(p2.x),2),round(MM(p2.y),2))
    if (abs(P[0]-ax)<0.02 and abs(P[1]-ay)<0.02 and abs(Q[0]-bx)<0.02 and abs(Q[1]-by)<0.02) or \
       (abs(P[0]-bx)<0.02 and abs(P[1]-by)<0.02 and abs(Q[0]-ax)<0.02 and abs(Q[1]-ay)<0.02):
        print("del",P,Q,t.GetNetname()); b.Remove(t); n+=1
pcbnew.SaveBoard(sys.argv[5],b)
print("removed",n)
