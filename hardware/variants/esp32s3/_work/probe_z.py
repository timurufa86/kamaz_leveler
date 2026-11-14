import io,sys
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/probe_z.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
zf={}
for i,z in enumerate(b.Zones()):
    for lay in (pcbnew.F_Cu,pcbnew.B_Cu):
        if z.IsOnLayer(lay):
            zf.setdefault(lay,[]).append((z.GetNetname(),z.GetFilledPolysList(lay)))
def cov(lay,x,y):
    v=pcbnew.VECTOR2I(int(x*1e6),int(y*1e6))
    for nm,ps in zf.get(lay,[]):
        if ps.Contains(v): return nm
    return None
def runs(lay,y,x0,x1,st=0.5):
    out=[];cur=None;s=None;x=x0
    while x<=x1+1e-9:
        c=cov(lay,x,y)
        if c!=cur:
            if cur is not None: out.append((cur,round(s,2),round(x,2)))
            cur=c;s=x
        x=round(x+st,6)
    out.append((cur,round(s,2),round(x1,2)))
    return out
for y in (37.0,38.26,39.2,39.6,40.0,40.5,41.0,42.0,44.0,15.4,14.5,20.0,26.0,34.0):
    for lay,nm in ((pcbnew.F_Cu,'F'),(pcbnew.B_Cu,'B')):
        print(f"y={y:6.2f} {nm}: ", [(c,a,bb) for c,a,bb in runs(lay,y,46.0,104.0) if c is None or True][:14])
