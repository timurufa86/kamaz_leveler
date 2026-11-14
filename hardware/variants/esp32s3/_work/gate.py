import io,sys,json,statistics
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/gate.err','w')
import pcbnew
MM=pcbnew.ToMM
path=sys.argv[1] if len(sys.argv)>1 else r"kamaz_leveler_s3.kicad_pcb"
TAG=sys.argv[2] if len(sys.argv)>2 else "cur"
b=pcbnew.LoadBoard(path)
fps=list(b.GetFootprints())
allpads=[(f,p) for f in fps for p in f.Pads()]
u7=[f for f in fps if f.GetReference()=="U7"][0]
tr=list(b.GetTracks())
segs=[t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]
vias=[t for t in tr if t.Type()==pcbnew.PCB_VIA_T]
L=sorted(MM(s.GetLength()) for s in segs)
def med(a): return statistics.median(a) if a else 0.0
R={}
R['footprints']=len(fps); R['pads']=len(allpads); R['u7_pads']=u7.GetPadCount()
R['segs']=len(segs); R['vias']=len(vias); R['zones']=b.GetAreaCount()
R['copper']=round(sum(L),2); R['median']=round(med(L),4)
R['p10']=round(L[len(L)//10],4) if L else 0; R['p90']=round(L[9*len(L)//10],4) if L else 0
R['nets']=len([n for n in b.GetNetInfo().NetsByNetcode() if n!=0])
# zone
ZX0,ZX1,ZY0,ZY1=40.73,104.19,13.84,39.83
zs=[s for s in segs if min(MM(s.GetStart().x),MM(s.GetEnd().x))>=ZX0-0.005
    and max(MM(s.GetStart().x),MM(s.GetEnd().x))<=ZX1+0.005
    and min(MM(s.GetStart().y),MM(s.GetEnd().y))>=ZY0-0.005
    and max(MM(s.GetStart().y),MM(s.GetEnd().y))<=ZY1+0.005]
zl=sorted(MM(s.GetLength()) for s in zs)
R['zone_segs']=len(zs); R['zone_median']=round(med(zl),4); R['zone_copper']=round(sum(zl),2)
# antenna strip
AX0,AX1,AY0,AY1=103.00,104.05,15.40,38.26
def seg_in(x1,y1,x2,y2):
    return not (max(x1,x2)<AX0 or min(x1,x2)>AX1 or max(y1,y2)<AY0 or min(y1,y2)>AY1)
ant=0
for s in segs:
    p1,p2=s.GetStart(),s.GetEnd()
    if seg_in(MM(p1.x),MM(p1.y),MM(p2.x),MM(p2.y)): ant+=1
av=0
for v in vias:
    p=v.GetPosition()
    if AX0<=MM(p.x)<=AX1 and AY0<=MM(p.y)<=AY1: av+=1
zv=0
for z in b.Zones():
    for lay in (pcbnew.F_Cu,pcbnew.In1_Cu,pcbnew.In2_Cu,pcbnew.In3_Cu,pcbnew.In4_Cu,pcbnew.In5_Cu,pcbnew.In6_Cu,pcbnew.In7_Cu,pcbnew.In8_Cu,pcbnew.In9_Cu,pcbnew.In10_Cu,pcbnew.In11_Cu,pcbnew.In12_Cu,pcbnew.In13_Cu,pcbnew.In14_Cu,pcbnew.In15_Cu,pcbnew.In16_Cu,pcbnew.In17_Cu,pcbnew.In18_Cu,pcbnew.In19_Cu,pcbnew.In20_Cu,pcbnew.In21_Cu,pcbnew.In22_Cu,pcbnew.In23_Cu,pcbnew.In24_Cu,pcbnew.In25_Cu,pcbnew.In26_Cu,pcbnew.In27_Cu,pcbnew.In28_Cu,pcbnew.In29_Cu,pcbnew.In30_Cu,pcbnew.B_Cu):
        try:
            if not z.IsOnLayer(lay): continue
            ps=z.GetFilledPolysList(lay)
        except Exception: continue
        if ps is None: continue
        for oi in range(ps.OutlineCount()):
            o=ps.Outline(oi)
            for k in range(o.PointCount()):
                p=o.CPoint(k)
                if AX0<=MM(p.x)<=AX1 and AY0<=MM(p.y)<=AY1: zv+=1
R['ant_segs']=ant; R['ant_vias']=av; R['ant_fill_verts']=zv
# per-net
def netstat(nm):
    ns=[s for s in segs if s.GetNetname()==nm]
    nv=[v for v in vias if v.GetNetname()==nm]
    return len(ns),len(nv),round(sum(MM(s.GetLength()) for s in ns),3)
R['nets_desc']={}
for nm in ["/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE","/DISP_MOSI",
            "/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"]:
    R['nets_desc'][nm]=netstat(nm)
# U7 pad map
pm={}
for p in u7.Pads():
    pm[p.GetNumber()]=(round(MM(p.GetPosition().x),3),round(MM(p.GetPosition().y),3),p.GetNetname())
R['u7']=pm
# J/H pos
fix={}
for f in fps:
    r=f.GetReference()
    if r[0] in "JH" and r[1:].isdigit():
        fix[r]=(round(MM(f.GetPosition().x),3),round(MM(f.GetPosition().y),3))
R['fixed']=dict(sorted(fix.items()))
json.dump(R,open('_work/gate_%s.json'%TAG,'w'),ensure_ascii=False,indent=1)
print("=== %s (%s)"%(TAG,path))
print("fp=%d pads=%d u7pads=%d segs=%d vias=%d zones=%d nets=%d copper=%.2f median=%.4f p10=%.4f p90=%.4f"%(
  R['footprints'],R['pads'],R['u7_pads'],R['segs'],R['vias'],R['zones'],R['nets'],R['copper'],R['median'],R['p10'],R['p90']))
print("zone: segs=%d median=%.4f copper=%.2f"%(R['zone_segs'],R['zone_median'],R['zone_copper']))
print("antenna strip: segs=%d vias=%d fill_verts=%d"%(R['ant_segs'],R['ant_vias'],R['ant_fill_verts']))
for nm in R['nets_desc']:
    s,v,l=R['nets_desc'][nm]; print("  %-14s segs=%-4d vias=%-3d len=%8.3f"%(nm,s,v,l))
print("U7 pads:")
for n in sorted(pm,key=lambda x:int(x)):
    x,y,net=pm[n]; print("  %2s (%8.3f;%7.3f) %s"%(n,x,y,net))
print("fixed:",R['fixed'])
