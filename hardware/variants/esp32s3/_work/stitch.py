import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
FU, BU = pcbnew.F_Cu, pcbnew.B_Cu
polys = {}
for L in (FU,BU):
    polys[L] = []
for z in b.Zones():
    for L in (FU,BU):
        try:
            if not z.IsOnLayer(L): continue
            ps = z.GetFilledPolysList(L)
        except Exception: continue
        if ps is not None and ps.OutlineCount()>0 and z.GetNetname()=='GND': polys[L].append(ps)
def cov(L,x,y):
    return any(ps.Contains(pcbnew.VECTOR2I(int(round(x*1e6)),int(round(y*1e6)))) for ps in polys[L])
print('GND stitching-grid vias (x 54..64, y 42..52) on main pour:')
for t in b.GetTracks():
    if not isinstance(t, pcbnew.PCB_VIA): continue
    if t.GetNetname() != 'GND': continue
    p = t.GetPosition(); x,y = p.x/1e6, p.y/1e6
    if 54 <= x <= 64 and 42 <= y <= 52:
        print('  (%6.2f,%6.2f) d=%.2f  F-fill=%s B-fill=%s' % (x,y,t.GetWidth(FU)/1e6, cov(FU,x,y), cov(BU,x,y)))
# J5 pad 1
for fp in b.GetFootprints():
    if fp.GetReference() != 'J5': continue
    for pd in fp.Pads():
        p = pd.GetPosition()
        print('J5 pad %s net=%s at (%.3f,%.3f) F-fill=%s B-fill=%s' % (pd.GetNumber(), pd.GetNetname(), p.x/1e6, p.y/1e6, cov(FU,p.x/1e6,p.y/1e6), cov(BU,p.x/1e6,p.y/1e6)))
