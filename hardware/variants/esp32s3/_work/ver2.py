import sys, io, math, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

NEW = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
OLD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work\BACKUP_session_start.kicad_pcb"
FU, BU = pcbnew.F_Cu, pcbnew.B_Cu
AX0, AY0, AX1, AY1 = 96.50, 17.40, 104.05, 36.60

def stats(path, tag):
    b = pcbnew.LoadBoard(path)
    seg = 0; via = 0
    for t in b.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA): via += 1
        else: seg += 1
    fps = list(b.GetFootprints())
    print('%s: segments=%d vias=%d footprints=%d zones=%d' % (tag, seg, via, len(fps), b.GetAreaCount()))
    return b

bn = stats(NEW, 'NEW')
bo = stats(OLD, 'OLD')

def inbox(x, y, m=0.0):
    return AX0 - m <= x <= AX1 + m and AY0 - m <= y <= AY1 + m
cnt = {'trk':0,'via':0,'fp':0,'pad':0}
for t in bn.GetTracks():
    s, e = t.GetStart(), t.GetEnd()
    x1,y1,x2,y2 = s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6
    mx, my = (x1+x2)/2, (y1+y2)/2
    if inbox(x1,y1) or inbox(x2,y2) or inbox(mx,my):
        if isinstance(t, pcbnew.PCB_VIA): cnt['via'] += 1
        else: cnt['trk'] += 1
for fp in bn.GetFootprints():
    p = fp.GetPosition()
    if inbox(p.x/1e6, p.y/1e6): cnt['fp'] += 1
    for pd in fp.Pads():
        pp = pd.GetPosition()
        if inbox(pp.x/1e6, pp.y/1e6): cnt['pad'] += 1
zone_hits = []
for z in bn.Zones():
    for L in (FU, BU):
        try:
            if not z.IsOnLayer(L): continue
            ps = z.GetFilledPolysList(L)
        except Exception: continue
        if ps is None or ps.OutlineCount()==0: continue
        hit = False
        yy = AY0 + 0.5
        while yy <= AY1 and not hit:
            xx = AX0 + 0.5
            while xx <= AX1:
                if ps.Contains(pcbnew.VECTOR2I(int(round(xx*1e6)), int(round(yy*1e6)))):
                    hit = True; break
                xx += 1.0
            yy += 1.0
        if hit: zone_hits.append(bn.GetLayerName(L))
print('ANTENNA BOX %.2f..%.2f x %.2f..%.2f : tracks=%d vias=%d footprints=%d pads=%d zone_fill_inside=%s' % (AX0,AX1,AY0,AY1,cnt['trk'],cnt['via'],cnt['fp'],cnt['pad'],zone_hits if zone_hits else 'none'))

def u7(path):
    b = pcbnew.LoadBoard(path)
    d = {}
    for fp in b.GetFootprints():
        if fp.GetReference() != 'U7': continue
        for pd in fp.Pads():
            d[pd.GetNumber()] = pd.GetNetname()
    return d
u_new = u7(NEW); u_old = u7(OLD)
diff = {k: (u_old.get(k), u_new.get(k)) for k in set(u_old) | set(u_new) if u_old.get(k) != u_new.get(k)}
real = sorted(k for k,v in u_new.items() if v and not v.startswith('unconnected'))
print('U7 pads=%d real_net_pads=%d placeholder=%d DIFF=%s' % (len(u_new), len(real), len(u_new)-len(real), diff if diff else 'NONE'))
print('U7 real: ' + ', '.join('%s=%s' % (k, u_new[k]) for k in real))

def outline(path):
    b = pcbnew.LoadBoard(path)
    poly = pcbnew.SHAPE_POLY_SET()
    b.GetBoardPolygonOutlines(poly, True)
    bb = poly.BBox()
    return (poly.OutlineCount(), round(poly.Area()/1e12,2), round(bb.GetX()/1e6,3), round(bb.GetY()/1e6,3), round(bb.GetRight()/1e6,3), round(bb.GetBottom()/1e6,3))
o1, o0 = outline(NEW), outline(OLD)
print('OUTLINE new=%s old=%s same=%s' % (o1, o0, o1==o0))

def holes(path):
    b = pcbnew.LoadBoard(path)
    r = []
    for fp in b.GetFootprints():
        ref = fp.GetReference()
        if ref in ('H1','H2','H3','H4'):
            p = fp.GetPosition()
            r.append((ref, round(p.x/1e6,3), round(p.y/1e6,3)))
    return sorted(r)
h1, h0 = holes(NEW), holes(OLD)
print('HOLES %s same=%s' % (h1, h1 == h0))

def refs(path):
    b = pcbnew.LoadBoard(path)
    return sorted(fp.GetReference() for fp in b.GetFootprints())
r1, r0 = refs(NEW), refs(OLD)
print('FOOTPRINT REFS same=%s (new=%d old=%d)' % (r1 == r0, len(r1), len(r0)))
if r1 != r0:
    print('  only_new=%s only_old=%s' % (sorted(set(r1)-set(r0)), sorted(set(r0)-set(r1))))
