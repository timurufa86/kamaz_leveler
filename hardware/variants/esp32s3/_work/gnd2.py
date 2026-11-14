import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
BRD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
b = pcbnew.LoadBoard(BRD)
FU, BU = pcbnew.F_Cu, pcbnew.B_Cu

def d_seg(px,py,x1,y1,x2,y2):
    dx,dy=x2-x1,y2-y1; L2=dx*dx+dy*dy
    if L2==0: return math.hypot(px-x1,py-y1)
    t=((px-x1)*dx+(py-y1)*dy)/L2; t=max(0.0,min(1.0,t))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))
def d_rect(px,py,x0,y0,x1,y1):
    return math.hypot(max(x0-px,0.0,px-x1), max(y0-py,0.0,py-y1))

gnd_trk = {FU: [], BC if False else BU: []}
for t in b.GetTracks():
    if t.GetNetname() != 'GND': continue
    s,e = t.GetStart(), t.GetEnd()
    x1,y1,x2,y2 = s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6
    if isinstance(t, pcbnew.PCB_VIA):
        for L in (FU,BU): gnd_trk[L].append(('c', x1,y1, t.GetWidth(FU)/2e6))
    else:
        gnd_trk[t.GetLayer()].append(('s', x1,y1,x2,y2, t.GetWidth()/2e6))
gnd_pads = []
for fp in b.GetFootprints():
    for pd in fp.Pads():
        if pd.GetNetname() != 'GND': continue
        p = pd.GetPosition(); sz = pd.GetSize()
        ls = [L for L in (FU,BU) if pd.IsOnLayer(L) or pd.GetAttribute()==pcbnew.PAD_ATTRIB_PTH]
        gnd_pads.append((p.x/1e6, p.y/1e6, sz.x/2e6, sz.y/2e6, ls))
gnd_vias = []
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname()=='GND':
        p = t.GetPosition(); gnd_vias.append((p.x/1e6, p.y/1e6))

polys = {FU: [], BU: []}
for z in b.Zones():
    for L in (FU,BU):
        try:
            if not z.IsOnLayer(L): continue
            ps = z.GetFilledPolysList(L)
        except Exception: continue
        if ps is not None and ps.OutlineCount()>0: polys[L].append((z.GetNetname(), ps))

print('=== GND zone fill islands (per outline)')
for L,nm in ((FU,'F.Cu'),(BU,'B.Cu')):
    for (znet, ps) in polys[L]:
        if znet != 'GND': 
            print('  (%s zone net=%s outlines=%d - not GND)' % (nm, znet, ps.OutlineCount())); continue
        for i in range(ps.OutlineCount()):
            poly = ps.Outline(i)
            area = abs(poly.Area())/1e12
            nv = 0; npa = 0
            for (x,y) in gnd_vias:
                if poly.PointInside(pcbnew.VECTOR2I(int(round(x*1e6)),int(round(y*1e6)))): nv += 1
            for (x,y,hw,hh,ls) in gnd_pads:
                if L in ls and poly.PointInside(pcbnew.VECTOR2I(int(round(x*1e6)),int(round(y*1e6)))): npa += 1
            bb = poly.BBox()
            print('  %s island#%d area=%8.2f bbox %6.2f,%6.2f..%6.2f,%6.2f gnd_vias=%d gnd_pads=%d' % (nm,i,area,bb.GetX()/1e6,bb.GetY()/1e6,bb.GetRight()/1e6,bb.GetBottom()/1e6,nv,npa))

def zone_covers(L, x, y):
    for (znet, ps) in polys[L]:
        if znet != 'GND': continue
        if ps.Contains(pcbnew.VECTOR2I(int(round(x*1e6)),int(round(y*1e6)))): return True
    return False

print('=== GND via support (each via must touch GND copper on each layer: zone fill OR track/pad)')
orph = []
for (x,y) in gnd_vias:
    sup = {}
    for L in (FU,BU):
        ok = zone_covers(L,x,y)
        if not ok:
            for it in gnd_trk[L]:
                if it[0]=='c':
                    if math.hypot(x-it[1], y-it[2]) < 0.6: ok=True; break
                else:
                    if d_seg(x,y,it[1],it[2],it[3],it[4]) < 0.6: ok=True; break
        if not ok:
            for (px,py,hw,hh,ls) in gnd_pads:
                if L in ls and d_rect(x,y,px-hw,py-hh,px+hw,py+hh) < 0.6: ok=True; break
        sup[L] = ok
    if not (sup[FU] and sup[BU]):
        orph.append((x,y,sup[FU],sup[BU]))
print('gnd vias total=%d  not-supported-on-both-layers=%d' % (len(gnd_vias), len(orph)))
for r in orph[:20]: print('   (%.3f,%.3f) F=%s B=%s' % r)
