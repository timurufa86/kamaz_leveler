import io,sys,os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
LIB=os.path.abspath('kamaz_leveler_hw.pretty')
NAME='ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm'

def load(path=None):
    fp=pcbnew.FootprintLoad(LIB,NAME) if path is None else pcbnew.FootprintLoad(os.path.dirname(path), os.path.splitext(os.path.basename(path))[0])
    if fp is None: raise SystemExit('load failed')
    return fp

def items(fp):
    out=[]
    for it in fp.GraphicalItems():
        try: l=it.GetLayerName()
        except Exception: l=str(it.GetLayer())
        out.append((l,it))
    for t in [fp.Reference(), fp.Value()]:
        out.append((t.GetLayerName(),t))
    for t in fp.GetProperties() if False else []:
        pass
    return out

def bbox_local(fp, it):
    bb=it.GetBoundingBox()
    px,py=fp.GetPosition().x/IU, fp.GetPosition().y/IU
    x1=bb.GetX()/IU-px; y1=bb.GetY()/IU-py
    return x1,y1,x1+bb.GetWidth()/IU,y1+bb.GetHeight()/IU

def report(fp, title):
    print('=====', title)
    print('pads:', len(fp.Pads()))
    agg={}
    its=items(fp)
    print('graphic+text items:', len(its))
    for l,it in its:
        x1,y1,x2,y2=bbox_local(fp,it)
        a=agg.setdefault(l,[1e9,1e9,-1e9,-1e9,0])
        a[0]=min(a[0],x1);a[1]=min(a[1],y1);a[2]=max(a[2],x2);a[3]=max(a[3],y2);a[4]+=1
        it._bbox=(x1,y1,x2,y2)
    for l,a in sorted(agg.items()):
        print('  %-14s n=%-3d x[%9.4f .. %9.4f] y[%9.4f .. %9.4f]'%(l,a[4],a[0],a[2],a[1],a[3]))
    return agg, its

fp=load()
agg,its=report(fp,'LIB (before)')
print('--- F.SilkS items by xmax ---')
for l,it in sorted([x for x in its if 'Silk' in x[0]], key=lambda t:-t[1]._bbox[2]):
    print('   %-22s %s  bbox=%s'%(type(it).__name__, getattr(it,'GetText',lambda:'')() if hasattr(it,'GetText') else '', tuple(round(v,3) for v in it._bbox)))
