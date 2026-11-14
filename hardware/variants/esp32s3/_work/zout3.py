import io,sys
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
z=[z for z in b.Zones() if z.GetNetname()=='GND'][0]
o=z.Outline().Outline(0)
pts=[(round(MM(o.CPoint(k).x),2),round(MM(o.CPoint(k).y),2)) for k in range(o.PointCount())]
print("n=",len(pts))
for i in range(0,len(pts),1): print(i,pts[i])
