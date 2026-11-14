import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb")


def add_zone(netname, layer):
    net = b.FindNet(netname)
    if net is None:
        print("нет сети", netname)
        return
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetLocalClearance(int(0.3e6))
    z.SetMinThickness(int(0.25e6))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_INHERITED)
    outline = pcbnew.SHAPE_POLY_SET()
    outline.NewOutline()
    for x, y in ((-29.0, -41.5), (29.0, -41.5), (29.0, 41.5), (-29.0, 41.5)):
        outline.Append(int(x * 1e6), int(y * 1e6))
    z.SetOutline(outline)
    b.Add(z)
    print("добавлена зона", netname, "слой", "B.Cu" if layer == pcbnew.B_Cu else "F.Cu")


add_zone("GND", pcbnew.B_Cu)
add_zone("+3V3", pcbnew.F_Cu)

pcbnew.SaveBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work\zonetest.kicad_pcb", b)
print("сохранено _work/zonetest.kicad_pcb, зон:", sum(1 for _ in b.Zones()))
