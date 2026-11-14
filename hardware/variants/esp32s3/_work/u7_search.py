import io,sys,os,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6; PX,PY=102.33,38.26
FP='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
V=pcbnew.VECTOR2I
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
foreign=[]
for fp in b.GetFootprints():
    if fp.GetReference()=='U7': continue
    for p in fp.Pads():
        foreign.append((fp.GetReference()+'.'+p.GetNumber(),p.GetPosition().x/IU,p.GetPosition().y/IU,
                        max(p.GetSize().x,p.GetSize().y)/2.0/IU))
TARGET=('C35.1','C35.2','R38.2')
d,fn=os.path.split(os.path.abspath(FP))
LAY=fp_lay=None
def load():
    f=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0]); assert f is not None; return f
def measure(f):
    own=1e9; tgt={}; tot=0
    pads=[(p.GetPosition().x/IU,p.GetPosition().y/IU,p.GetSize().x/IU/2) for p in f.Pads()]
    for it in f.GraphicalItems():
        if 'Silk' not in it.GetLayerName(): continue
        bb=it.GetBoundingBox()
        x0=bb.GetX()/IU+PX; y0=bb.GetY()/IU+PY; x1=x0+bb.GetWidth()/IU; y1=y0+bb.GetHeight()/IU
        w=(it.GetTextThickness() if hasattr(it,'GetTextThickness') else it.GetWidth())/IU
        for px,py,pr in pads:
            dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
            own=min(own, math.hypot(dx,dy)-(pr+w/2))
        for ref,px,py,pr in foreign:
            dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
            m=math.hypot(dx,dy)-pr
            if m<0: tot+=1
            if ref in TARGET: tgt[ref]=min(tgt.get(ref,9),m)
    return own,max(tgt.values()) if tgt else 9e9,tot,tgt
def find(f):
    out=[]
    for it in f.GraphicalItems():
        if 'Silk' not in it.GetLayerName() or not hasattr(it,'GetText'): continue
        p=it.GetPosition()
        if abs(p.y/IU-1.02)<0.001 and (abs(p.x/IU+40.64)<0.01 or abs(p.x/IU+53.34)<0.01):
            out.append((it.GetText(),it))
    return out
print('%-6s %-9s %-9s %-6s %s'%('font','own_min','tgt_worst','fgn_tot','per-target'))
f0=load()
for f in (0.90,0.80,0.75,0.70,0.68,0.65,0.62,0.60,0.58,0.55):
    fp=load()
    for name,it in find(fp):
        it.SetTextSize(V(round(f*IU),round(f*IU)))
    own,tw,tot,tgt=measure(fp)
    print('%-6.2f %-9.4f %-9.4f %-6d %s'%(f,own,tw,tot,{k:round(v,3) for k,v in tgt.items()}))
