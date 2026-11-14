import io,sys,os,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6; PX,PY=102.33,38.26
FP='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
b=pcbnew.LoadBoard('kamaz_leveler_s3.kicad_pcb')
fpads=[]
for fp in b.GetFootprints():
    if fp.GetReference()=='U7':
        continue
    for p in fp.Pads():
        r=max(p.GetSize().x,p.GetSize().y)/2.0/IU
        fpads.append((fp.GetReference(),p.GetNumber(),p.GetPosition().x/IU,p.GetPosition().y/IU,r))
def scan(path):
    d,fn=os.path.split(os.path.abspath(path))
    f=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0])
    assert f is not None
    hits=[]
    for it in f.GraphicalItems():
        if 'Silk' not in it.GetLayerName():
            continue
        bb=it.GetBoundingBox()
        x0=bb.GetX()/IU+PX; y0=bb.GetY()/IU+PY
        x1=x0+bb.GetWidth()/IU; y1=y0+bb.GetHeight()/IU
        txt=it.GetText() if hasattr(it,'GetText') else 'shape'
        for ref,num,px,py,pr in fpads:
            dx=max(x0-px,0.0,px-x1); dy=max(y0-py,0.0,py-y1)
            dist=math.hypot(dx,dy)
            if dist<pr:
                hits.append((round(dist-pr,4),txt,ref+'.'+num,round(dist,4),round(pr,4),
                             round(x0,2),round(y0,2),round(x1,2),round(y1,2)))
    return hits
h=scan(FP)
print('CURRENT foreign contacts:',len(h))
for r in sorted(h):
    print('  margin=%7.4f text=%-8s pad=%-8s dist=%6.4f R=%5.3f  bbox world x[%.2f..%.2f] y[%.2f..%.2f]'%(r[0],r[1],r[2],r[3],r[4],r[5],r[7],r[6],r[8]))
for fp in b.GetFootprints():
    if fp.GetReference() in ('C35','R38'):
        print(fp.GetReference(),[(p.GetNumber(),round(p.GetPosition().x/IU,2),round(p.GetPosition().y/IU,2),round(p.GetSize().x/IU,2),round(p.GetSize().y/IU,2)) for p in fp.Pads()])
