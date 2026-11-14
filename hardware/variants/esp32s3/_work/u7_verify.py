import io,sys,os,re,hashlib,math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
PX,PY=102.33,38.26
CUR='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
BAK='_work/BACKUP_U7_socket.kicad_mod'

def load(p):
    d,fn=os.path.split(os.path.abspath(p))
    fp=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0]); assert fp is not None,p
    return fp
def items(fp):
    return [(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),fp.Reference().GetLayerName())]
def bb_local(fp,it):
    b=it.GetBoundingBox(); px=fp.GetPosition().x/IU; py=fp.GetPosition().y/IU
    x1=b.GetX()/IU-px; y1=b.GetY()/IU-py
    return x1,y1,x1+b.GetWidth()/IU,y1+b.GetHeight()/IU
def bbox_layer(fp,layer):
    a=[1e9,1e9,-1e9,-1e9,0]
    for it,l in items(fp):
        if layer not in l: continue
        x1,y1,x2,y2=bb_local(fp,it)
        a[0]=min(a[0],x1);a[1]=min(a[1],y1);a[2]=max(a[2],x2);a[3]=max(a[3],y2);a[4]+=1
    return a
def silk_pad(fp):
    pads=[(pd.GetNumber(),pd.GetPosition().x/IU,pd.GetPosition().y/IU,pd.GetSize().x/IU/2) for pd in fp.Pads()]
    res=[]; npair=0
    for it,l in items(fp):
        if 'Silk' not in l: continue
        if isinstance(it,pcbnew.PCB_TEXT) or isinstance(it,pcbnew.PCB_FIELD):
            w=it.GetTextThickness()/IU
        else:
            w=it.GetWidth()/IU
        x1,y1,x2,y2=bb_local(fp,it)
        worst=None
        for num,px,py,pr in pads:
            npair+=1
            dx=max(x1-px,0,px-x2); dy=max(y1-py,0,py-y2)
            d=math.hypot(dx,dy); m=d-(pr+w/2)
            if worst is None or m<worst[0]: worst=(m,num,d,pr+w/2)
        res.append((round(worst[0],4),getattr(it,'GetText',lambda:'shape')(),worst[1],round(worst[2],4),round(worst[3],4)))
    return res,npair

fpA=load(BAK); fpB=load(CUR)
print('--- pads ---')
pa=[(p.GetNumber(),round(p.GetPosition().x/IU,3),round(p.GetPosition().y/IU,3),round(p.GetSize().x/IU,3),round(p.GetSize().y/IU,3),round(p.GetDrillSize().x/IU,3),p.GetLayerSet().FmtHex()) for p in fpA.Pads()]
pb=[(p.GetNumber(),round(p.GetPosition().x/IU,3),round(p.GetPosition().y/IU,3),round(p.GetSize().x/IU,3),round(p.GetSize().y/IU,3),round(p.GetDrillSize().x/IU,3),p.GetLayerSet().FmtHex()) for p in fpB.Pads()]
print('pads before/after:',len(pa),len(pb),' mismatched tuples:',sum(1 for a,b in zip(pa,pb) if a!=b))
rawA=[x for x in open(BAK,encoding='utf-8') if x.startswith('  (pad ')]
rawB=[x for x in open(CUR,encoding='utf-8') if x.startswith('  (pad ')]
print('raw pad lines:',len(rawA),len(rawB),'byte-identical:',rawA==rawB)
print('--- file validity ---')
tA=open(BAK,encoding='utf-8').read(); tB=open(CUR,encoding='utf-8').read()
print('parens A/B:',tA.count('(')-tA.count(')'), tB.count('(')-tB.count(')'),
      '| name A:',re.match(r'\s*\(footprint\s+"([^"]+)"',tA).group(1),
      '| name B:',re.match(r'\s*\(footprint\s+"([^"]+)"',tB).group(1),
      '| quotes even:',tB.count('"')%2==0)
print('--- layer bboxes (local mm) & world ---')
for lay in ('F.Fab','F.Courtyard','F.Silkscreen'):
    a=bbox_layer(fpA,lay); b=bbox_layer(fpB,lay)
    print('%-13s BEFORE n=%-2d x[%8.4f..%8.4f] y[%8.4f..%8.4f]   AFTER n=%-2d x[%8.4f..%8.4f] y[%8.4f..%8.4f]'%(
        lay,a[4],a[0],a[2],a[1],a[3],b[4],b[0],b[2],b[1],b[3]))
    print('              world max x BEFORE %.4f  AFTER %.4f   (limit %.2f)'%(PX+a[2],PX+b[2],104.05))
fab=bbox_layer(fpB,'F.Fab'); cy=bbox_layer(fpB,'F.Courtyard'); sil=bbox_layer(fpB,'F.Silkscreen')
print('--- courtyard vs fab clearance (after) ---')
for side,(fx,cx) in {'xmin':(fab[0],cy[0]),'xmax':(fab[2],cy[2]),'ymin':(fab[1],cy[1]),'ymax':(fab[3],cy[3])}.items():
    sgn=1 if side in ('xmax','ymax') else -1
    print('   %s: fab %.4f courtyard %.4f clearance %.4f'%(side,fx,cx,sgn*(cx-fx)))
print('   need: courtyard max local x <= %.4f ; fab max +0.25 = %.4f'%(1.72, fab[2]+0.25))
print('--- silk vs pads (bbox-distance minus (R+w/2)) ---')
ra,npa=silk_pad(fpA); rb,npb=silk_pad(fpB)
print('BEFORE pairs=%d items=%d min margin=%.4f  overlaps(margin<0)=%d'%(npa,len(ra),min(r[0] for r in ra),sum(1 for r in ra if r[0]<0)))
print('AFTER  pairs=%d items=%d min margin=%.4f  overlaps(margin<0)=%d'%(npb,len(rb),min(r[0] for r in rb),sum(1 for r in rb if r[0]<0)))
print('AFTER worst 6:',sorted(rb)[:6])
print('moved-item margins (after):',[r for r in rb if r[1] in ('ANTENNA END','J1','J3')])
print('--- sha256 ---')
for p in (BAK,CUR): print(' ',hashlib.sha256(open(p,'rb').read()).hexdigest(), p)
