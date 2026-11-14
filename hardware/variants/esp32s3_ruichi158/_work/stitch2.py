"""Place a via + short stub next to every pad reported as unconnected (pour misses)."""
import re
import sys
import math
import os
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass
from rr import Model, NX, NY, ix, iy
from go import trim_to_pad

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
REPORT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '_work', 'drc_b.txt')
SRC = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
DST = sys.argv[3] if len(sys.argv) > 3 else os.path.join(ROOT, 'stitched.kicad_pcb')
NETS = set(x.strip() for x in (sys.argv[4].split(',') if len(sys.argv) > 4 else ['GND', '+3V3', '+12V']))

txt = open(REPORT, encoding='utf-8', errors='replace').read()
todo = set()
for m in re.finditer(r"(?i)(?:pth |smd )?pad (\S+) \[([^\]]+)\] of (\S+)", txt):
    num, net, ref = m.group(1), m.group(2), m.group(3)
    if net in NETS:
        todo.add((ref, num))
print('pads to stitch: %d' % len(todo))

m = Model(SRC)
m.build()
b = m.b
nmap = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
byref = {}
for p in m.pads:
    byref[(p['ref'], p['num'])] = p

done = 0
fail = []
for (ref, num) in sorted(todo):
    p = byref.get((ref, num))
    if p is None:
        continue
    net = p['net']
    pens = m.pen_for_net(net)
    blkF, blkB, blkV = m.pen_masks(pens, 0.1)
    blk = [blkF, blkB]
    ok = False
    for L in (1, 0):
        if ok:
            break
        if not (p['onB'] if L == 1 else p['onF']):
            continue
        for r in (0.5, 0.62, 0.75, 0.9, 1.05, 1.2, 1.4):
            for t in range(24):
                a = 2 * math.pi * t / 24.0
                vx = p['x'] + r * math.cos(a)
                vy = p['y'] + r * math.sin(a)
                i, j = ix(vx), iy(vy)
                if not (0 <= i < NX and 0 <= j < NY):
                    continue
                if blk[L][j, i] or blkV[j, i] or m.antmask[j, i]:
                    continue
                if not m.seg_clear(p['x'], p['y'], vx, vy, 0.1, L, net):
                    continue
                if not m.via_clear(vx, vy, net, pens):
                    continue
                qx, qy = trim_to_pad(p, p['x'], p['y'], vx, vy, 0.10)
                s = pcbnew.PCB_TRACK(b)
                s.SetStart(pcbnew.VECTOR2I(int(round(qx * 1e6)), int(round(qy * 1e6))))
                s.SetEnd(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                s.SetWidth(int(round(0.2 * 1e6)))
                s.SetLayer(LAY[L])
                s.SetNetCode(nmap[net])
                b.Add(s)
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                v.SetWidth(int(round(rr.VIA_D * 1e6)))
                v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNetCode(nmap[net])
                b.Add(v)
                m.add_copper([(qx, qy, vx, vy, 0.1, L)], [(vx, vy)])
                done += 1
                ok = True
                break
            if ok:
                break
    if not ok:
        fail.append((ref, num, net))
print('stitched %d pads, failed %d %s' % (done, len(fail), fail[:10]))
b.Save(DST)
print('saved', DST)
