import io,sys,os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
p='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
d,fn=os.path.split(os.path.abspath(p))
fp=pcbnew.FootprintLoad(d, os.path.splitext(fn)[0])
items=[(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),'ref')]
pads=[(pd.GetNumber(),pd.GetPosition(),pd.GetSize().x/2.0) for pd in fp.Pads()]
print('n items',len(items))
viol=[]
for it,l in items:
    if 'Silk' not in l: continue
    poly=pcbnew.SHAPE_POLY_SET()
    try:
        it.TransformShapeToPolygon(poly, it.GetLayer(), 0, 5000, pcbnew.ERROR_OUTSIDE)
    except Exception as e:
        print('poly fail',type(it).__name__,e); continue
    for num,pos,pr in pads:
        hit=False
        for method in ('a','b'):
            try:
                hit=poly.Collide(pos, int((pr)*IU))
                break
            except Exception as e:
                hit=None
        if hit:
            viol.append((type(it).__name__, getattr(it,'GetText',lambda:'—')(), num, round(pr,3)))
print('POLY-COLLIDE violations:', len(viol))
for v in viol: print('  ',v)
