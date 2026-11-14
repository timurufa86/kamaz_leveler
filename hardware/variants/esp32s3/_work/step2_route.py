import io,sys,math,json
sys.path.insert(0,'_work')
import rtlib as R
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
FC=R.FC; BC=R.BC
SRC=sys.argv[1] if len(sys.argv)>1 else '_work/s1.kicad_pcb'
DST=sys.argv[2] if len(sys.argv)>2 else '_work/s2.kicad_pcb'
THERMS=[float(x) for x in (sys.argv[4].split(',') if len(sys.argv)>4 else ['0.5','0.3','0.15','0.0'])]

NETS=["/SPEED_PULSE","/INT_MCU","/VLV_EN","/DISP_DC","/DISP_RES","/DISP_SCLK",
      "/DISP_MOSI","/DISP_BL","/DISP_CS","/PRES_SW","/I2C_SCL","/I2C_SDA"]
b=pcbnew.LoadBoard(SRC)
R.index=R.Index(); idx=R.index
gnd_obs=[]; pads={}
def set_therm(t):
    for o in gnd_obs: o.inf=t
for f in b.GetFootprints():
    for p in f.Pads():
        nm=p.GetNetname() or ('__npth_%s_%s'%(f.GetReference(),p.GetNumber()))
        sz=p.GetSize(); ang=math.radians(p.GetOrientationDegrees()); pos=p.GetPosition()
        lays=[FC,BC] if p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH else [l for l in (FC,BC) if p.IsOnLayer(l)]
        o=R.Obs('rect',nm,lays,cx=MM(pos.x),cy=MM(pos.y),hx=MM(sz.x)/2.0,hy=MM(sz.y)/2.0,
                ang=ang,x1=0,y1=0,x2=0,y2=0,r=0,tag=None)
        if nm in ('GND','+3V3'): gnd_obs.append(o)
        idx.add(o)
        if p.GetDrillSize().x>0:
            R.HOLES.append((MM(pos.x),MM(pos.y),MM(p.GetDrillSize().x)/2.0))
        if nm in NETS:
            pads.setdefault(nm,[]).append({'ref':f.GetReference(),'pad':p.GetNumber(),
                'x':MM(p.GetPosition().x),'y':MM(p.GetPosition().y),'lays':set(lays)})
for t in b.GetTracks():
    nm=t.GetNetname()
    if t.Type()==pcbnew.PCB_VIA_T:
        pos=t.GetPosition(); w=MM(t.GetWidth(t.TopLayer()))
        for l in (FC,BC):
            idx.add(R.Obs('cap',nm,[l],x1=MM(pos.x),y1=MM(pos.y),x2=MM(pos.x),y2=MM(pos.y),
                          r=w/2.0,cx=0,cy=0,hx=0,hy=0,ang=0,tag=None))
    else:
        p1=t.GetStart(); p2=t.GetEnd(); w=MM(t.GetWidth()); lay=t.GetLayer()
        idx.add(R.Obs('cap',nm,[lay],x1=MM(p1.x),y1=MM(p1.y),x2=MM(p2.x),y2=MM(p2.y),r=w/2.0,
                      cx=0,cy=0,hx=0,hy=0,ang=0,tag=None))
# forbid the small +3V3 F.Cu pour rectangle so it stays connected
idx.add(R.Obs('rect','__zone3v3',[FC],cx=81.0,cy=46.9,hx=1.6,hy=2.5,ang=0.0,
              x1=0,y1=0,x2=0,y2=0,r=0,tag=None))
print("obstacles:",sum(len(v) for v in idx.grid.values()),"maxr=%.2f holes=%d"%(idx.maxr,len(R.HOLES)))

CREATED=[]
def emit(netname,lay,pts):
    ni=b.FindNet(netname); n=0
    for k in range(len(pts)-1):
        x1,y1=pts[k]; x2,y2=pts[k+1]
        if math.hypot(x2-x1,y2-y1)<1e-9: continue
        t=pcbnew.PCB_TRACK(b)
        t.SetStart(R.v2(x1,y1)); t.SetEnd(R.v2(x2,y2))
        t.SetWidth(pcbnew.FromMM(R.W)); t.SetLayer(lay); t.SetNet(ni)
        b.Add(t); n+=1
        o=R.Obs('cap',netname,[lay],x1=x1,y1=y1,x2=x2,y2=y2,r=R.HW,cx=0,cy=0,hx=0,hy=0,ang=0,tag=None)
        idx.add(o); CREATED.append((t,o))
    return n

def emit_via(netname,x,y):
    ni=b.FindNet(netname)
    v=pcbnew.PCB_VIA(b)
    v.SetPosition(R.v2(x,y)); v.SetWidth(pcbnew.FromMM(R.VIA_D)); v.SetDrill(pcbnew.FromMM(R.VIA_DR))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(FC,BC); v.SetNet(ni)
    b.Add(v); CREATED.append((v,None))
    for l in (FC,BC):
        o=R.Obs('cap',netname,[l],x1=x,y1=y,x2=x,y2=y,r=R.VIA_R,cx=0,cy=0,hx=0,hy=0,ang=0,tag=None)
        idx.add(o); CREATED.append((None,o))
    return v

def rollback(mark):
    while len(CREATED)>mark:
        obj,o=CREATED.pop()
        if o is not None: idx.remove([o])
        if obj is not None:
            try: b.Remove(obj)
            except Exception: pass

def new_segs(mark):
    out=[]
    for (obj,o) in CREATED[mark:]:
        if obj is not None and obj.Type()==pcbnew.PCB_TRACE_T:
            p1=obj.GetStart(); p2=obj.GetEnd()
            out.append((obj.GetNetname(),obj.GetLayer(),MM(p1.x),MM(p1.y),MM(p2.x),MM(p2.y)))
    return out

def mst(terms):
    n=len(terms); INF=1e18
    used=[False]*n; dist=[INF]*n; par=[-1]*n; dist[0]=0; edges=[]
    for _ in range(n):
        u=-1
        for i in range(n):
            if not used[i] and (u<0 or dist[i]<dist[u]): u=i
        used[u]=True
        if par[u]>=0: edges.append((par[u],u))
        for v in range(n):
            if used[v]: continue
            d=math.hypot(terms[u]['x']-terms[v]['x'],terms[u]['y']-terms[v]['y'])
            if not (terms[u]['lays'] & terms[v]['lays']): d+=8.0
            if d<dist[v]: dist[v]=d; par[v]=u
    return edges

def plan_once(net,A,Bt,p1,p2,tpts):
    common=[l for l in (FC,BC) if l in A['lays'] and l in Bt['lays']]
    for lay in common:
        c=R.shape_scan(idx,lay,net,p1,p2,tpts)
        if c: return ('direct',lay,c[0])
    best=None
    for l1 in A['lays']:
        for l2 in Bt['lays']:
            if l1==l2: continue
            dx=p1[0]-p2[0]; dy=p1[1]-p2[1]; L=math.hypot(dx,dy)
            if L<1e-6: continue
            ux,uy=dx/L,dy/L
            for d in (1.0,1.5,2.2,3.0,4.0,5.5,7.0,9.0):
                V=(round(p2[0]+ux*d,4),round(p2[1]+uy*d,4))
                if not R.via_clear2(idx,V[0],V[1],net): continue
                c1=R.shape_scan(idx,l1,net,p1,V,tpts,win=10.0)
                if not c1: continue
                c2=R.shape_scan(idx,l2,net,V,p2,tpts,win=10.0)
                if not c2: continue
                tot=(c1[0][0]+c2[0][0]+1,c1[0][1]+c2[0][1])
                if best is None or tot<best[0]: best=(tot,l1,l2,V,c1[0][2],c2[0][2])
    if best: return ('cross',best)
    path=None; bc=None
    for l1 in sorted(A['lays']):
        for l2 in sorted(Bt['lays']):
            p=R.astar(idx,net,p1,l1,p2,l2)
            if p is None: continue
            nv=sum(1 for q in p if q[3]=='V'); c=(nv,)
            if bc is None or c<bc: bc=c; path=p
    if path: return ('astar',path)
    return None

def do_plan(net,A,Bt,p1,p2,tpts,plan):
    kind=plan[0]
    if kind=='direct':
        nseg,ln,pts=plan[2]; emit(net,plan[1],pts); return (nseg,0,ln)
    if kind=='cross':
        tot,l1,l2,V,s1,s2=plan[1]
        emit(net,l1,s1); emit_via(net,V[0],V[1]); emit(net,l2,s2)
        return (tot[0],1,tot[1])
    path=plan[1]
    runs=[]; cur=[(path[0][0],path[0][1])]; curlay=path[0][2]
    for (x,y,lay,k) in path[1:]:
        if k=='V': runs.append((curlay,cur)); cur=[(x,y)]; curlay=lay
        else: cur.append((x,y))
    runs.append((curlay,cur))
    tot=0; nseg=0
    for lay,pts in runs:
        sp=R.simplify_run(idx,net,lay,pts)
        nseg+=emit(net,lay,sp)
        tot+=sum(math.hypot(sp[k+1][0]-sp[k][0],sp[k+1][1]-sp[k][1]) for k in range(len(sp)-1))
    nv=0
    for (x,y,lay,k) in path:
        if k=='V': emit_via(net,x,y); nv+=1
    return (nseg,nv,tot)

fail=[]; newtracks=[]
for net in NETS:
    terms=pads.get(net,[])
    if len(terms)<2:
        print(net,"SKIP terminals=%d"%len(terms)); continue
    tpts=[(t['x'],t['y']) for t in terms]
    edges=mst(terms)
    edges.sort(key=lambda e: math.hypot(terms[e[0]]['x']-terms[e[1]]['x'],terms[e[0]]['y']-terms[e[1]]['y']))
    print("== %s  %d pads, %d edges"%(net,len(terms),len(edges)))
    for (iu,iv) in edges:
        A=terms[iu]; Bt=terms[iv]
        p1=(A['x'],A['y']); p2=(Bt['x'],Bt['y'])
        got=False
        for attempt in (1,0):
            R.SAMENET_ON=bool(attempt)
            plan=None; used_t=None
            for th in THERMS:
                set_therm(th)
                plan=plan_once(net,A,Bt,p1,p2,tpts)
                if plan: used_t=th; break
            if not plan: continue
            nseg,nv,ln=do_plan(net,A,Bt,p1,p2,tpts,plan)
            print("   %s.%s -> %s.%s  %-6s nseg=%d nvia=%d len=%.2f  therm=%.2f%s"%(
                  A['ref'],A['pad'],Bt['ref'],Bt['pad'],plan[0],nseg,nv,ln,used_t,
                  '  [sep]' if attempt==1 else '  [noseP]'))
            got=True; break
        if not got:
            print("   !! FAILED %s.%s -> %s.%s"%(A['ref'],A['pad'],Bt['ref'],Bt['pad']))
            fail.append((net,A['ref'],A['pad'],Bt['ref'],Bt['pad'])); continue
set_therm(0.0)
R.SAMENET_ON=False
# --- verification of every new segment against exact clearance ---
bad=0; newlist=[]
# recompute the new tracks: those added after load
for t in b.GetTracks():
    if t.Type()!=pcbnew.PCB_TRACE_T: continue
    nm=t.GetNetname()
    if nm not in NETS: continue
    p1=t.GetStart(); p2=t.GetEnd()
    x1,y1,x2,y2=MM(p1.x),MM(p1.y),MM(p2.x),MM(p2.y)
    if not R.seg_clear(idx,t.GetLayer(),nm,x1,y1,x2,y2):
        bad+=1
        if bad<=8: print("   VERIFY FAIL %s %s (%.3f,%.3f)-(%.3f,%.3f)"%(nm,b.GetLayerName(t.GetLayer()),x1,y1,x2,y2))
print("self-check: %d/%d new segments violate clearance (approximate, GND pads un-inflated)"%(
      bad,len([t for t in b.GetTracks() if t.Type()==pcbnew.PCB_TRACE_T and t.GetNetname() in NETS])))
b.BuildConnectivity()
pcbnew.SaveBoard(DST,b)
print("FAILS:",fail)
print("saved",DST)
