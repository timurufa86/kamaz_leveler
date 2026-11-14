import io,sys,os,re,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
SRC='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
OUT='_work/test_justify.kicad_mod'
t=open(SRC,encoding='utf-8').read()
def fix(m):
    line=m.group(0)
    x=float(m.group(1)); y=float(m.group(2))
    just='left' if abs(y-1.02)<1e-6 else 'right'
    if 'justify' in line: return line
    return line.replace('(thickness %s)))'%m.group(3), '(thickness %s)) (justify %s))'%(m.group(3),just))
pat=re.compile(r'\(fp_text user "[^"]*" \(at (-?[\d.]+) (-?[\d.]+) 90\) \(layer "F.SilkS"\) \(effects \(font \(size [\d.]+ [\d.]+\) \(thickness ([\d.]+)\)\)\)\)')
t2,n=pat.subn(fix,t)
print('labels patched:',n)
open(OUT,'w',encoding='utf-8',newline='').write(t2)
d,fn=os.path.split(os.path.abspath(OUT))
fp=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0]); assert fp is not None
pads=[(pd.GetNumber(),pd.GetPosition().x/IU,pd.GetPosition().y/IU,pd.GetSize().x/IU/2) for pd in fp.Pads()]
items=[(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),fp.Reference().GetLayerName())]
bad=[];n2=0;minm=9e9
ymin=9e9;ymax=-9e9;xmax=-9e9
for it,l in items:
    if 'Silk' not in l: continue
    n2+=1
    b=it.GetBoundingBox(); x1=b.GetX()/IU;y1=b.GetY()/IU;x2=x1+b.GetWidth()/IU;y2=y1+b.GetHeight()/IU
    ymin=min(ymin,y1);ymax=max(ymax,y2);xmax=max(xmax,x2)
    w=(it.GetTextThickness() if isinstance(it,(pcbnew.PCB_TEXT,pcbnew.PCB_FIELD)) else it.GetWidth())/IU
    for num,px,py,pr in pads:
        dx=max(x1-px,0,px-x2); dy=max(y1-py,0,py-y2)
        m=math.hypot(dx,dy)-(pr+w/2)
        if m<minm: minm=m
        if m<0: bad.append((round(m,4),getattr(it,'GetText',lambda:'shape')(),num))
print('silk items',n2,'min margin %.4f'%minm,'overlaps',len(bad))
print('bad sample',sorted(bad)[:8])
print('silk bbox local x max %.4f  y[%.4f..%.4f]  -> world x max %.4f, y[%.4f..%.4f]'%(xmax,ymin,ymax,102.33+xmax,38.26+ymin,38.26+ymax))
