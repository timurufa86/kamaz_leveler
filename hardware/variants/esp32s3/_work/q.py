import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
px, py = float(sys.argv[1]), float(sys.argv[2])
IGN = set(a.strip() for a in sys.argv[3].split(',') if a.strip()) if len(sys.argv) > 3 else set()
def d_seg(px,py,x1,y1,x2,y2):
    dx,dy=x2-x1,y2-y1; L2=dx*dx+dy*dy
    if L2==0: return math.hypot(px-x1,py-y1)
    t=((px-x1)*dx+(py-y1)*dy)/L2; t=max(0.0,min(1.0,t))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))
def d_rect(px,py,x0,y0,x1,y1):
    return math.hypot(max(x0-px,0,px-x1), max(y0-py,0,py-y1))
print("query point", px, py, "ignore", IGN)
for t in b.GetTracks():
    nm=t.GetNetname()
    if nm in IGN: continue
    s,e=t.GetStart(),t.GetEnd()
    sx,sy,ex,ey=s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6
    if isinstance(t, pcbnew.PCB_VIA):
        r=t.GetWidth(pcbnew.F_Cu)/2e6
        d=math.hypot(px-sx,py-sy)-r
        if d < 0.30:
            print('VIA  %-12s (%7.3f,%7.3f) d=%.3f dist=%.3f' % (nm,sx,sy,r*2,d))
    else:
        d=d_seg(px,py,sx,sy,ex,ey)-t.GetWidth()/2e6
        if d < 0.30:
            print('TRK  %-12s %-5s (%7.3f,%7.3f)->(%7.3f,%7.3f) w=%.3f dist=%.3f' % (nm,b.GetLayerName(t.GetLayer()),sx,sy,ex,ey,t.GetWidth()/1e6,d))
for fp in b.GetFootprints():
    for pd in fp.Pads():
        nm=pd.GetNetname()
        if nm in IGN: continue
        p=pd.GetPosition(); cx,cy=p.x/1e6,p.y/1e6
        sz=pd.GetSize(); w,h=sz.x/1e6,sz.y/1e6
        d=d_rect(px,py,cx-w/2,cy-h/2,cx+w/2,cy+h/2)
        if d < 0.30:
            ls=[b.GetLayerName(L) for L in (pcbnew.F_Cu,pcbnew.B_Cu) if pd.IsOnLayer(L) or pd.GetAttribute()==pcbnew.PAD_ATTRIB_PTH]
            print('PAD  %-12s %-5s %-4s (%7.3f,%7.3f) size=(%.2f,%.2f) dist=%.3f %s' % (nm,fp.GetReference(),pd.GetNumber(),cx,cy,w,h,d,ls))
for z in b.Zones():
    lay = 'F' if z.IsOnLayer(pcbnew.F_Cu) else 'B'
    try:
        ps=z.GetFilledPolysList(pcbnew.F_Cu if z.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu)
        if ps is not None and ps.OutlineCount()>0 and ps.Contains(pcbnew.VECTOR2I(int(px*1e6),int(py*1e6))):
            print('ZONE %s net=%s layer=%s contains point' % (z.GetZoneName(), z.GetNetname(), lay))
    except Exception as ex:
        pass
