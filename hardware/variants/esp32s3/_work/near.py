import io,sys,math
sys.path.insert(0,'_work')
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard('_work/s3.kicad_pcb')
tgt=[(42.4656,44.7411,'C18.2 GND'),(51.5300,15.4000,'U7.43 GND')]
for (tx,ty,nm) in tgt:
    print("==",nm,(tx,ty))
    for t in b.GetTracks():
        if t.Type()==pcbnew.PCB_VIA_T:
            p=t.GetPosition(); d=math.hypot(MM(p.x)-tx,MM(p.y)-ty)
            if d<4.0: print("   VIA %-12s (%.2f,%.2f) d=%.2f"%(t.GetNetname(),MM(p.x),MM(p.y),d))
        else:
            p1=t.GetStart(); p2=t.GetEnd()
            x1,y1,x2,y2=MM(p1.x),MM(p1.y),MM(p2.x),MM(p2.y)
            dx,dy=x2-x1,y2-y1; L2=dx*dx+dy*dy
            if L2==0: d=math.hypot(tx-x1,ty-y1)
            else:
                u=max(0,min(1,((tx-x1)*dx+(ty-y1)*dy)/L2)); d=math.hypot(tx-(x1+u*dx),ty-(y1+u*dy))
            if d<3.0: print("   TRK %-12s %s (%.2f,%.2f)-(%.2f,%.2f) d=%.2f"%(t.GetNetname(),b.GetLayerName(t.GetLayer()),x1,y1,x2,y2,d))
