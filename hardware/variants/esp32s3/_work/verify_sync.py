import sys,math,statistics
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
A=pcbnew.LoadBoard('_work/BACKUP_pcb_before_fpsync2.kicad_pcb')
Bp=pcbnew.LoadBoard(sys.argv[1])
def getsilk(board):
    out=[]
    for lay in (pcbnew.F_SilkS,pcbnew.B_SilkS):
        for d in board.GetDrawings():
            if d.GetLayer()==lay: out.append(('B',d))
    for f in board.GetFootprints():
        for g in f.GraphicalItems():
            if g.GetLayer() in (pcbnew.F_SilkS,pcbnew.B_SilkS): out.append((f.GetReference(),g))
    return out
def metrics(board):
    edges=[d for d in board.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts]
    xmin=min(MM(d.GetBoundingBox().GetLeft()) for d in edges); xmax=max(MM(d.GetBoundingBox().GetRight()) for d in edges)
    ymin=min(MM(d.GetBoundingBox().GetTop()) for d in edges); ymax=max(MM(d.GetBoundingBox().GetBottom()) for d in edges)
    silk=getsilk(board)
    out=0; mx=-1e9
    for who,g in silk:
        bb=g.GetBoundingBox()
        r=MM(bb.GetRight()); mx=max(mx,r)
        if r>xmax+0.001 or MM(bb.GetLeft())<xmin-0.001 or MM(bb.GetTop())<ymin-0.001 or MM(bb.GetBottom())>ymax+0.001: out+=1
    u7=[f for f in board.GetFootprints() if f.GetReference()=='U7'][0]
    cy=u7.GetCourtyard(pcbnew.F_CrtYd)
    cyx=MM(cy.BBox().GetRight()) if cy.OutlineCount() else float('nan')
    cybb=u7.GetBoundingBox(False,False)
    tr=list(board.GetTracks())
    seg=[t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]; via=[t for t in tr if t.Type()==pcbnew.PCB_VIA_T]
    L=sorted(MM(s.GetLength()) for s in seg)
    zx0,zy0,zy1=40.73,13.84,39.83
    z=[s for s in seg if min(MM(s.GetStart().x),MM(s.GetEnd().x))>=zx0-0.005 and max(MM(s.GetStart().x),MM(s.GetEnd().x))<=104.19+0.005
        and min(MM(s.GetStart().y),MM(s.GetEnd().y))>=zy0-0.005 and max(MM(s.GetStart().y),MM(s.GetEnd().y))<=zy1+0.005]
    return dict(edge=(round(xmin,3),round(xmax,3),round(ymin,3),round(ymax,3)),
                silk_out=out,silk_maxx=round(mx,4),crtyd_maxx=round(cyx,4),
                fp_bbox_maxx=round(MM(cybb.GetRight()),4),
                fp=len(list(board.GetFootprints())),pads=sum(f.GetPadCount() for f in board.GetFootprints()),
                segs=len(seg),vias=len(via),zones=board.GetAreaCount(),
                copper=round(sum(L),2),median=round(statistics.median(L),4),zone=len(z))
print("BEFORE:",metrics(A))
print("AFTER :",metrics(Bp))
def padsig(board):
    u7=[f for f in board.GetFootprints() if f.GetReference()=='U7'][0]
    d={}
    for p in u7.Pads():
        d[p.GetNumber()]=(round(MM(p.GetPosition().x),4),round(MM(p.GetPosition().y),4),
                          round(MM(p.GetSize().x),4),round(MM(p.GetSize().y),4),int(p.GetShape()),
                          round(MM(p.GetDrillSize().x),4),int(p.GetAttribute()),p.GetNetname())
    return d
pa=padsig(A); pb=padsig(Bp)
diff=[k for k in pa if pa[k]!=pb.get(k)]
print("pad signature diffs:",len(diff),diff[:5])
u7b=[f for f in Bp.GetFootprints() if f.GetReference()=='U7'][0]
print("U7 after: pads=%d items=%d pos=(%.3f,%.3f) ang=%.1f"%(u7b.GetPadCount(),len(list(u7b.GraphicalItems())),
      MM(u7b.GetPosition().x),MM(u7b.GetPosition().y),u7b.GetOrientationDegrees()))
pads={p.GetNumber():p.GetNetname() for p in u7b.Pads()}
EXP={'/DISP_BL':'4','/DISP_CS':'5','/DISP_DC':'9','/DISP_RES':'10','/DISP_SCLK':'11','/SPEED_PULSE':'12',
     '/DISP_MOSI':'17','/INT_MCU':'19','/VLV_EN':'20','/PRES_SW':'32','/I2C_SCL':'39','/I2C_SDA':'40'}
ok=sum(1 for net,pad in EXP.items() if [n for n,v in pads.items() if v==net]==[pad])
print("pad map 12 pairs: %d/12"%ok)
for n in ('1','2','6','7','8','15','16','18','21','22','23','43','44'):
    print("   pad %-3s %s"%(n,pads[n]))
