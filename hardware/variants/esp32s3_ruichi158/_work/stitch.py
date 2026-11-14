import io
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
from go import access_cells, trim_to_pad

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = os.path.join(ROOT, 'routed.kicad_pcb')
DST = os.path.join(ROOT, 'stitched.kicad_pcb')
NETS = (sys.argv[1].split(',') if len(sys.argv) > 1 else ['GND'])

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
nvia = nstub = 0
for net in NETS:
    net = net if net in nmap else '/' + net
    pads = m.netpads(net)
    pens = m.pen_for_net(net)
    blkF, blkB, blkV = m.pen_masks(pens, 0.1)
    blk = [blkF, blkB]
    for p in pads:
        if p['onF'] and not p['onB']:
            continue          # SMD on F.Cu: covered by the F.Cu pour
        done = False
        for L in (1, 0):
            if done:
                break
            if not (p['onB'] if L == 1 else p['onF']):
                continue
            for r in (0.55, 0.7, 0.85, 1.0, 1.2):
                for t in range(24):
                    a = 2 * math.pi * t / 24.0
                    vx = p['x'] + r * math.cos(a)
                    vy = p['y'] + r * math.sin(a)
                    i, j = ix(vx), iy(vy)
                    if not (0 <= i < NX and 0 <= j < NY):
                        continue
                    if blk[L][j, i] or blkV[j, i]:
                        continue
                    if not m.seg_clear(p['x'], p['y'], vx, vy, 0.1, L, net):
                        continue
                    if not m.via_clear(vx, vy, net, pens):
                        continue
                    # add stub + via
                    qx, qy = trim_to_pad(p, p['x'], p['y'], vx, vy, 0.10)
                    s = pcbnew.PCB_TRACK(b)
                    s.SetStart(pcbnew.VECTOR2I(int(round(qx * 1e6)), int(round(qy * 1e6))))
                    s.SetEnd(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                    s.SetWidth(int(round(0.2 * 1e6)))
                    s.SetLayer(LAY[L])
                    s.SetNetCode(nmap[net])
                    b.Add(s)
                    nstub += 1
                    v = pcbnew.PCB_VIA(b)
                    v.SetPosition(pcbnew.VECTOR2I(int(round(vx * 1e6)), int(round(vy * 1e6))))
                    v.SetWidth(int(round(rr.VIA_D * 1e6)))
                    v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
                    v.SetViaType(pcbnew.VIATYPE_THROUGH)
                    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                    v.SetNetCode(nmap[net])
                    b.Add(v)
                    nvia += 1
                    m.add_copper([(qx, qy, vx, vy, 0.1, L)], [(vx, vy)])
                    done = True
                    break
                if done:
                    break
        if not done:
            print('  no stitch for %s.%s (%.2f,%.2f)' % (p['ref'], p['num'], p['x'], p['y']))
print('%s: %d stubs, %d vias' % (NETS, nstub, nvia))
b.Save(DST)
print('saved', DST)
