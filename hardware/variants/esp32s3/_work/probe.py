import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
for i in range(0,42):
    print(i, b.GetLayerName(i))
print("F_Cu const", pcbnew.F_Cu, "B_Cu", pcbnew.B_Cu)
via = None
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA):
        via = t
        break
print("VIA ATTRS", [a for a in dir(via) if 'rill' in a or 'Width' in a or 'Diameter' in a or 'Layer' in a])
print("via drill", via.GetDrillValue(), "width", via.GetWidth(), "toplayer", via.TopLayer(), "botlayer", via.BottomLayer())
