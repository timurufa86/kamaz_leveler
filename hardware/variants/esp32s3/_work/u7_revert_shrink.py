import io,sys,hashlib,re,os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6; PX,PY=102.33,38.26
P='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
BAK='_work/BACKUP_U7_socket_v4.kicad_mod'
t=open(P,encoding='utf-8').read()
EDITS=[
 ('  (fp_text user "11" (at -40.64 1.02 90) (layer "F.SilkS") (effects (font (size 0.58 0.58) (thickness 0.12)) (justify right)))',
  '  (fp_text user "11" (at -40.64 1.02 90) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.12)) (justify right)))'),
 ('  (fp_text user "G" (at -53.34 1.02 90) (layer "F.SilkS") (effects (font (size 0.58 0.58) (thickness 0.12)) (justify right)))',
  '  (fp_text user "G" (at -53.34 1.02 90) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.12)) (justify right)))'),
]
for old,new in EDITS:
    assert t.count(old)==1, (t.count(old), old[:60])
    t=t.replace(old,new)
open(P,'w',encoding='utf-8',newline='').write(t)
print('reverted 2 lines; parens',t.count('(')-t.count(')'),'| quotes even',t.count(chr(34))%2==0)
print('sha256 before(v4)',hashlib.sha256(open(BAK,'rb').read()).hexdigest())
print('sha256 after     ',hashlib.sha256(open(P,'rb').read()).hexdigest())

# --- min font over all fp_text
L=t.split('\n'); mn=(999,None,None)
for i,l in enumerate(L,1):
    if '(fp_text' not in l: continue
    m=re.search(r'\(size ([\d.]+) ([\d.]+)\)',l)
    if m:
        v=float(m.group(1))
        if v<mn[0]: mn=(v,i,l.strip()[:70])
print('min fp_text size: %.2f at line %d'%(mn[0],mn[1]))
# --- pcbnew checks
def load(path):
    d,fn=os.path.split(os.path.abspath(path))
    f=pcbnew.FootprintLoad(d,os.path.splitext(fn)[0]); assert f is not None,path
    return f
def bbox(f,layer):
    a=[1e9,1e9,-1e9,-1e9,0]
    its=list(f.GraphicalItems())+[f.Reference(),f.Value()]
    for it in its:
        ln=it.GetLayerName()
        if layer not in ln: continue
        b=it.GetBoundingBox(); px=f.GetPosition().x/IU; py=f.GetPosition().y/IU
        x1=b.GetX()/IU-px; y1=b.GetY()/IU-py
        x2=x1+b.GetWidth()/IU; y2=y1+b.GetHeight()/IU
        a[0]=min(a[0],x1);a[1]=min(a[1],y1);a[2]=max(a[2],x2);a[3]=max(a[3],y2);a[4]+=1
    return a
fa=load(BAK); fb=load(P)
for lay in ('F.CrtYd','F.SilkS'):
    A=bbox(fa,lay); B=bbox(fb,lay)
    print('%-8s BEFORE n=%-2d x[%9.4f..%9.4f] y[%9.4f..%9.4f] | AFTER n=%-2d x[%9.4f..%9.4f] y[%9.4f..%9.4f] | world xmax %.4f'%(
        lay,A[4],A[0],A[2],A[1],A[3],B[4],B[0],B[2],B[1],B[3],PX+B[2]))
silkgraph=len([it for it in fb.GraphicalItems() if 'Silk' in it.GetLayerName()])
print('silk graphical primitives:',silkgraph,' (+REF field on silk ->',silkgraph+1,')')
pa=[(p.GetNumber(),round(p.GetPosition().x/IU,3),round(p.GetPosition().y/IU,3),round(p.GetSize().x/IU,3),round(p.GetSize().y/IU,3),round(p.GetDrillSize().x/IU,3),p.GetLayerSet().FmtHex()) for p in fa.Pads()]
pb=[(p.GetNumber(),round(p.GetPosition().x/IU,3),round(p.GetPosition().y/IU,3),round(p.GetSize().x/IU,3),round(p.GetSize().y/IU,3),round(p.GetDrillSize().x/IU,3),p.GetLayerSet().FmtHex()) for p in fb.Pads()]
print('pads %d/%d mismatched %d | raw pad lines identical: %s'%(len(pa),len(pb),
      sum(1 for a,b in zip(pa,pb) if a!=b),
      [x for x in open(BAK,encoding='utf-8') if x.startswith('  (pad ')]==[x for x in open(P,encoding='utf-8') if x.startswith('  (pad ')]))
print('name:',re.match(r'\s*\(footprint\s+"([^"]+)"',t).group(1),'| min size >=0.8:',mn[0]>=0.8)
