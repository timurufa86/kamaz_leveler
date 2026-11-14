import io,sys,os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
V=pcbnew.VECTOR2I
p='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
d,fn=os.path.split(os.path.abspath(p))
fp=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0])
items=[(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),fp.Reference().GetLayerName())]
pads=[(pd.GetNumber(),pd.GetPosition(),pd.GetSize().x/IU/2.0) for pd in fp.Pads()]
rows=[]
for it,l in items:
    if 'Silk' not in l: continue
    ps=pcbnew.SHAPE_POLY_SET()
    it.TransformShapeToPolygon(ps, it.GetLayer(), 0, 5000, pcbnew.ERROR_OUTSIDE)
    bb=ps.BBox()
    np=ps.FullPointCount()
    for num,pos,pr in pads:
        px,py=pos.x/IU,pos.y/IU
        dx=max(bb.GetX()/IU-px,0.0,px-(bb.GetX()+bb.GetWidth())/IU)
        dy=max(bb.GetY()/IU-py,0.0,py-(bb.GetY()+bb.GetHeight())/IU)
        bboxd=(dx*dx+dy*dy)**0.5
        if bboxd>pr+0.30: continue
        cont=ps.Contains(pos); col=ps.Collide(pos,int(pr*IU))
        rows.append((getattr(it,'GetText',lambda:'shape')(),num,np,round(bboxd,3),round(pr,3),cont,col))
print('candidate pairs (bbox close):',len(rows),' poly-collide hits:',sum(1 for r in rows if r[6]),' contains:',sum(1 for r in rows if r[5]))
for r in rows[:25]: print('  text=%-12s pad=%-3s pts=%-4d bboxdist=%-6s R=%-5s inside=%-5s collide=%s'%r)
