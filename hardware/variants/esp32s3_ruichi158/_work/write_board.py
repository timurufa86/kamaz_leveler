import io
import sys
import json
import os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
WORK = os.path.join(ROOT, '_work')
import pcbnew

geo = json.load(open(os.path.join(WORK, 'routed_geo.json'), encoding='utf-8'))
SRC = os.path.join(WORK, 'BACKUP_run2.kicad_pcb')
DST = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'routed2.kicad_pcb')

b = pcbnew.LoadBoard(SRC)
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
nseg = nvia = 0
missing = []
for net, segs in geo['segs'].items():
    if not segs:
        continue
    if net not in nmap:
        missing.append(net)
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
        nseg += 1
    for (x, y) in geo['vias'].get(net, []):
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(0.45 * 1e6)))
        v.SetDrill(int(round(0.2 * 1e6)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(nc)
        b.Add(v)
        nvia += 1
b.Save(DST)
print('saved %s : %d tracks %d vias ; missing nets %s ; failed %s' %
      (DST, nseg, nvia, missing, geo.get('failed')))
