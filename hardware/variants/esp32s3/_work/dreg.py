import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

BRD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
x0, x1, y0, y1 = [float(v) for v in sys.argv[1:5]]
b = pcbnew.LoadBoard(BRD)
IU = 1e6
def mm(v): return v / IU
def inr(p): return x0 <= mm(p.x) <= x1 and y0 <= mm(p.y) <= y1

print("=== TRACKS/VIAS")
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA):
        p = t.GetPosition()
        if inr(p):
            print("VIA %-12s %-5s (%8.3f,%8.3f) d=%.3f w=%.3f %s..%s %s" % (
                t.GetNetname(), b.GetLayerName(t.GetLayer()), mm(p.x), mm(p.y),
                mm(t.GetDrillValue()), mm(t.GetWidth(pcbnew.F_Cu)),
                b.GetLayerName(t.TopLayer()), b.GetLayerName(t.BottomLayer()), t.m_Uuid.AsString()))
    else:
        s, e = t.GetStart(), t.GetEnd()
        if inr(s) or inr(e):
            print("TRK %-12s %-5s (%8.3f,%8.3f)->(%8.3f,%8.3f) w=%.3f %s" % (
                t.GetNetname(), b.GetLayerName(t.GetLayer()), mm(s.x), mm(s.y), mm(e.x), mm(e.y),
                mm(t.GetWidth()), t.m_Uuid.AsString()))

print("=== PADS")
for fp in b.GetFootprints():
    ref = fp.GetReference()
    for pd in fp.Pads():
        p = pd.GetPosition()
        if inr(p):
            ls = [b.GetLayerName(L) for L in range(0, 40) if pd.IsOnLayer(L)]
            print("PAD %-5s %-3s %-12s (%8.3f,%8.3f) size=(%.3f,%.3f) shape=%s attr=%s drill=%.3f L=%s" % (
                ref, pd.GetNumber(), pd.GetNetname(), mm(p.x), mm(p.y),
                mm(pd.GetSize().x), mm(pd.GetSize().y), pd.GetShape(), pd.GetAttribute(),
                mm(pd.GetDrillSize().x), ",".join(ls)))
