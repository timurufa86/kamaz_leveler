import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
net = sys.argv[1]
rows = []
for t in b.GetTracks():
    if t.GetNetname() != net: continue
    s, e = t.GetStart(), t.GetEnd()
    if isinstance(t, pcbnew.PCB_VIA):
        rows.append((round(s.x/1e6,3), round(s.y/1e6,3), 'VIA', 'd=%.2f/%.2f' % (t.GetWidth(pcbnew.F_Cu)/1e6, t.GetDrillValue()/1e6), '', t.m_Uuid.AsString()))
    else:
        rows.append((round(s.x/1e6,3), round(s.y/1e6,3), b.GetLayerName(t.GetLayer()),
                     '->(%.3f,%.3f) w=%.2f' % (e.x/1e6, e.y/1e6, t.GetWidth()/1e6), '', t.m_Uuid.AsString()))
rows.sort()
for r in rows:
    print('%-12s %8.3f %8.3f %s %s' % (r[2], r[0], r[1], r[3], r[5]))
print('--- pads on net', net)
for fp in b.GetFootprints():
    for pd in fp.Pads():
        if pd.GetNetname() != net: continue
        p = pd.GetPosition()
        ls = [b.GetLayerName(L) for L in (pcbnew.F_Cu, pcbnew.B_Cu) if pd.IsOnLayer(L) or pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
        print('PAD %-5s %-3s (%8.3f,%8.3f) size=(%.2f,%.2f) %s' % (fp.GetReference(), pd.GetNumber(), p.x/1e6, p.y/1e6, pd.GetSize().x/1e6, pd.GetSize().y/1e6, ls))
