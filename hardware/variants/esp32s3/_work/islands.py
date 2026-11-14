import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
BRD = sys.argv[1] if len(sys.argv) > 1 else r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
b = pcbnew.LoadBoard(BRD)
FU, BU = pcbnew.F_Cu, pcbnew.B_Cu

gnd_pads = []
gnd_vias = []
for fp in b.GetFootprints():
    for pd in fp.Pads():
        if pd.GetNetname() != 'GND':
            continue
        p = pd.GetPosition()
        ls = [L for L in (FU, BU) if pd.IsOnLayer(L) or pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
        gnd_pads.append((fp.GetReference(), pd.GetNumber(), p.x/1e6, p.y/1e6, ls, pd.GetAttribute()==pcbnew.PAD_ATTRIB_PTH))
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() == 'GND':
        p = t.GetPosition()
        gnd_vias.append((p.x/1e6, p.y/1e6))

polys = {FU: [], BU: []}
for z in b.Zones():
    for L in (FU, BU):
        try:
            if not z.IsOnLayer(L): continue
            ps = z.GetFilledPolysList(L)
        except Exception:
            continue
        if ps is not None and ps.OutlineCount() > 0:
            polys[L].append((z.GetNetname(), ps))

def contains(ps, x, y):
    return ps.Contains(pcbnew.VECTOR2I(int(round(x*1e6)), int(round(y*1e6))))

print('=== ZONE FILL ISLANDS')
tot_islands = 0
small = 0
for L, nm in ((FU, 'F.Cu'), (BU, 'B.Cu')):
    for (znet, ps) in polys[L]:
        for i in range(ps.OutlineCount()):
            poly = ps.Outline(i)
            bb = poly.BBox()
            area = abs(poly.Area())/1e12
            nv = sum(1 for (x, y) in gnd_vias if contains(ps, x, y))
            npad = sum(1 for r in gnd_pads if (L in r[4]) and contains(ps, r[2], r[3]))
            tot_islands += 1
            if area < 2.0: small += 1
            print('%s net=%-6s island#%d area=%8.3f mm2  bbox %7.2f,%7.2f..%7.2f,%7.2f  gndvias_in=%d  gndpads_in=%d' % (
                nm, znet, i, area, bb.GetX()/1e6, bb.GetY()/1e6, bb.GetRight()/1e6, bb.GetBottom()/1e6, nv, npad))
print('total islands', tot_islands, ' small(<2mm2):', small)

print('=== GND VIAS: land on both layers?')
bad = []
for (x, y) in gnd_vias:
    f = any(contains(ps, x, y) for (_, ps) in polys[FU])
    k = any(contains(ps, x, y) for (_, ps) in polys[BU])
    if not (f and k):
        bad.append((x, y, f, k))
print('gnd vias', len(gnd_vias), ' without fill on both layers:', len(bad))
for r in bad[:30]:
    print('   via (%.3f,%.3f) F=%s B=%s' % r)
