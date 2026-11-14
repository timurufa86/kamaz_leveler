import sys
import json
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
MAIN = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
GEO = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '_work', 'geo15.json')

geo = json.load(open(GEO, encoding='utf-8'))
b = pcbnew.LoadBoard(MAIN)
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
for t in list(b.GetTracks()):
    b.Remove(t)
n = 0
for net, segs in geo['segs'].items():
    if not segs or net not in nmap:
        continue
    nc = nmap[net]
    for (x1, y1, x2, y2, h, L) in segs:
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        s.SetWidth(int(round(2 * h * 1e6)))
        s.SetLayer(LAY[int(L)])
        s.SetNetCode(nc)
        b.Add(s)
        n += 1
    for (x, y) in geo['vias'].get(net, []):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(0.45 * 1e6)))
        v.SetDrill(int(round(0.2 * 1e6)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(nc)
        b.Add(v)
        n += 1
b.Save(MAIN)
print('board rebuilt from %s: %d items' % (GEO, n))
