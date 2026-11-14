import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

BRD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
x0, x1, y0, y1 = [float(v) for v in sys.argv[1:5]]

b = pcbnew.LoadBoard(BRD)
IU = 1e6

def mm(v):
    return v / IU

def inreg(p):
    return x0 <= mm(p.x) <= x1 and y0 <= mm(p.y) <= y1

print("LAYERNAMES", b.GetLayerName(0), b.GetLayerName(31), b.GetLayerName(32))

trk = []
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA):
        p = t.GetPosition()
        if inreg(p):
            trk.append(("VIA", t.GetNetname(), b.GetLayerName(t.GetLayer()),
                        (round(mm(p.x),3), round(mm(p.y),3)),
                        (round(mm(p.GetDrillValue()),3), round(mm(t.GetWidth()),3)),
                        t.m_Uuid.AsString() if hasattr(t,'m_Uuid') else ''))
    else:
        s, e = t.GetStart(), t.GetEnd()
        if inreg(s) or inreg(e):
            trk.append(("TRK", t.GetNetname(), b.GetLayerName(t.GetLayer()),
                        (round(mm(s.x),3), round(mm(s.y),3)),
                        (round(mm(e.x),3), round(mm(e.y),3)),
                        round(mm(t.GetWidth()),3),
                        t.m_Uuid.AsString() if hasattr(t,'m_Uuid') else ''))
for r in trk:
    print(r)

print("=== PADS")
for fp in b.GetFootprints():
    ref = fp.GetReference()
    for pd in fp.Pads():
        p = pd.GetPosition()
        if inreg(p):
            ls = []
            for L in range(0,32):
                if pd.IsOnLayer(L):
                    ls.append(b.GetLayerName(L))
            print(("PAD", ref, pd.GetNumber(), pd.GetNetname(),
                   (round(mm(p.x),3), round(mm(p.y),3)),
                   (round(mm(pd.GetSize().x),3), round(mm(pd.GetSize().y),3)),
                   pd.GetShape(), pd.GetAttribute(), ls,
                   round(mm(pd.GetDrillSize().x),3)))
