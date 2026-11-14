import io,sys,os,hashlib,re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
PLACE=(102.33,38.26)
LIM_X=104.05; LIM_Y0=0.0; LIM_Y1=54.05

def fp_load(path):
    d,fn=os.path.split(path); n=os.path.splitext(fn)[0]
    fp=pcbnew.FootprintLoad(os.path.abspath(d), n)
    assert fp is not None, 'load failed '+path
    return fp

def rel(fp, bb):
    px,py=fp.GetPosition().x/IU, fp.GetPosition().y/IU
    x1=bb.GetX()/IU-px; y1=bb.GetY()/IU-py
    return x1,y1,x1+bb.GetWidth()/IU,y1+bb.GetHeight()/IU

def all_items(fp):
    out=[('shape',it,it.GetLayerName()) for it in fp.GraphicalItems()]
    out.append(('ref',fp.Reference(),fp.Reference().GetLayerName()))
    out.append(('val',fp.Value(),fp.Value().GetLayerName()))
    return out

def silkname(l): return l in ('F.SilkS','F.Silkscreen','B.SilkS','B.Silkscreen')

def shape_of(it,w):
    try:
        s=it.GetEffectiveShape()
        if s is not None: return s
    except Exception: pass
    return None

def dist_shape_point(s,p):
    try: return s.Distance(p)/IU
    except Exception: return None

def report(path,title):
    print('='*70); print(title, path)
    raw=open(path,'rb').read()
    print('sha256', hashlib.sha256(raw).hexdigest())
    fp=fp_load(path)
    # --- name
    rawtxt=raw.decode('utf-8')
    m=re.match(r'\s*\(footprint\s+"([^"]+)"', rawtxt)
    print('footprint name:', m.group(1) if m else '??')
    bal=rawtxt.count('(')-rawtxt.count(')')
    print('paren balance (open-close):', bal, 'quotes even:', rawtxt.count('"')%2==0)
    # --- pads
    pads=[]
    for p in fp.Pads():
        pos=p.GetPosition(); sz=p.GetSize(); dr=p.GetDrillSize()
        pads.append((p.GetNumber(),
                     round(pos.x/IU-fp.GetPosition().x/IU,3), round(pos.y/IU-fp.GetPosition().y/IU,3),
                     round(sz.x/IU,3), round(sz.y/IU,3),
                     round(dr.x/IU,3), round(dr.y/IU,3),
                     p.GetLayerSet().FmtHex()))
    print('pads:', len(pads))
    # --- layer bboxes (body-only for Fab: exclude hidden value field)
    agg={}
    for kind,it,l in all_items(fp):
        if kind=='val' and it.IsVisible() is False:  # hidden
            pass
        x1,y1,x2,y2=rel(fp,it.GetBoundingBox())
        a=agg.setdefault(l,[1e9,1e9,-1e9,-1e9,0])
        a[0]=min(a[0],x1);a[1]=min(a[1],y1);a[2]=max(a[2],x2);a[3]=max(a[3],y2);a[4]+=1
    for l in sorted(agg):
        a=agg[l]
        print('  %-14s n=%-3d local x[%9.4f .. %9.4f] y[%9.4f .. %9.4f]'%(l,a[4],a[0],a[2],a[1],a[3]))
        if silkname(l) or 'CrtYd' in l or 'Courtyard' in l:
            print('      -> world x[%9.4f .. %9.4f] y[%9.4f .. %9.4f]  (limits x<=%.2f, y %.2f..%.2f)'%(
                PLACE[0]+a[0],PLACE[0]+a[2],PLACE[1]+a[1],PLACE[1]+a[3],LIM_X,LIM_Y0,LIM_Y1))
    # --- silk vs pads
    padcir=[]
    for p in fp.Pads():
        r=p.GetSize().x/IU/2.0
        padcir.append((p.GetNumber(), p.GetPosition(), r))
    print('--- silk vs pads ---')
    tot=0; worst=(9e9,None)
    viol=[]
    for kind,it,l in all_items(fp):
        if not silkname(l): continue
        w=0.0
        try: w=it.GetWidth()/IU if hasattr(it,'GetWidth') else 0.0
        except Exception: pass
        if kind in ('ref','val'): 
            try: w=it.GetTextThickness()/IU
            except Exception: w=0.0
        s=shape_of(it,w)
        for num,pos,pr in padcir:
            tot+=1
            need=pr+w/2.0
            d=None
            if s is not None: d=dist_shape_point(s,pos)
            if d is None:
                x1,y1,x2,y2=rel(fp,it.GetBoundingBox())
                cx,cy=pos.x/IU,pos.y/IU
                dx=max(x1-cx,0,cx-x2); dy=max(y1-cy,0,cy-y2)
                d=(dx*dx+dy*dy)**0.5
            margin=d-need
            if margin<worst[0]: worst=(margin,(kind,getattr(it,'GetText',lambda:'—')(),num,d,need))
            if margin<0: viol.append((round(margin,4),kind,getattr(it,'GetText',lambda:'—')(),num,round(d,4),round(need,4)))
    print('  pairs checked:', tot)
    print('  min margin (d - (Rpad+w/2)):', round(worst[0],4), worst[1])
    print('  violations:', len(viol))
    for v in sorted(viol)[:12]: print('   ',v)
    return fp,pads,agg

if __name__=='__main__':
    report(sys.argv[1] if len(sys.argv)>1 else 'kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod','BEFORE/AFTER')
