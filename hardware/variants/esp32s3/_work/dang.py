import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
uuids = sys.argv[1:]
byid = {}
for t in b.GetTracks():
    try: byid[t.m_Uuid.AsString()] = t
    except Exception: pass
FC, BC = pcbnew.F_Cu, pcbnew.B_Cu

def d_seg(px,py,x1,y1,x2,y2):
    dx,dy=x2-x1,y2-y1; L2=dx*dx+dy*dy
    if L2==0: return math.hypot(px-x1,py-y1)
    t=((px-x1)*dx+(py-y1)*dy)/L2; t=max(0.0,min(1.0,t))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))

for u in uuids:
    t = byid.get(u)
    print('='*70)
    if t is None:
        print('uuid not found', u); continue
    lay = b.GetLayerName(t.GetLayer())
    s,e = t.GetStart(), t.GetEnd()
    sx,sy,ex,ey = s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6
    print('TRACK %s %s (%7.3f,%7.3f)->(%7.3f,%7.3f) w=%.3f len=%.4f' % (t.GetNetname(), lay, sx,sy,ex,ey, t.GetWidth()/1e6, math.hypot(ex-sx,ey-sy)))
    for tag,(px,py) in (('A',(sx,sy)),('B',(ex,ey))):
        print(' endpoint %s (%.3f,%.3f)' % (tag,px,py))
        for o in b.GetTracks():
            if o is t: continue
            if o.GetNetname() != t.GetNetname(): continue
            os_,oe = o.GetStart(), o.GetEnd()
            ox1,oy1,ox2,oy2 = os_.x/1e6,os_.y/1e6,oe.x/1e6,oe.y/1e6
            isvia = isinstance(o, pcbnew.PCB_VIA)
            if isvia:
                if abs(ox1-px) < 1e-6 and abs(oy1-py) < 1e-6:
                    print('   VIA at this point d=%.2f' % (o.GetWidth(FC)/1e6))
                elif math.hypot(ox1-px, oy1-py) < o.GetWidth(FC)/2e6 + 0.076:
                    print('   VIA nearby (%.3f,%.3f) dist %.3f' % (ox1,oy1,math.hypot(ox1-px,oy1-py)))
                continue
            if o.GetLayer() != t.GetLayer(): 
                # check same-net other-layer touch (only meaningful for vias)
                pass
            dist = d_seg(px,py,ox1,oy1,ox2,oy2)
            if dist < 1e-6:
                print('   touches %s (%7.3f,%7.3f)->(%7.3f,%7.3f) w=%.2f %s' % (b.GetLayerName(o.GetLayer()), ox1,oy1,ox2,oy2, o.GetWidth()/1e6, o.m_Uuid.AsString()[:8]))
        for fp in b.GetFootprints():
            for pd in fp.Pads():
                if pd.GetNetname() != t.GetNetname(): continue
                p = pd.GetPosition()
                if not (pd.IsOnLayer(t.GetLayer()) or pd.GetAttribute()==pcbnew.PAD_ATTRIB_PTH): continue
                sz = pd.GetSize()
                dx = max(abs(p.x/1e6-px)-(sz.x/2e6), 0); dy = max(abs(p.y/1e6-py)-(sz.y/2e6), 0)
                if math.hypot(dx,dy) < 1e-6:
                    print('   touches PAD %s.%s (%.3f,%.3f)' % (fp.GetReference(), pd.GetNumber(), p.x/1e6, p.y/1e6))
    # same-net zone fill presence at both ends
    for z in b.Zones():
        if z.GetNetname() != t.GetNetname(): continue
        for L in (FC,BC):
            try:
                if not z.IsOnLayer(L): continue
                ps = z.GetFilledPolysList(L)
            except Exception:
                continue
            if ps is None or ps.OutlineCount()==0: continue
            for tag,(px,py) in (('A',(sx,sy)),('B',(ex,ey))):
                if ps.Contains(pcbnew.VECTOR2I(int(round(px*1e6)),int(round(py*1e6)))):
                    print('   zone(%s) covers %s' % (b.GetLayerName(L), tag))
