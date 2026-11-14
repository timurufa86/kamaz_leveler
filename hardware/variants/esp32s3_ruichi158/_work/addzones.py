"""Add un-filled zone blocks as text (S-expression) to the board.
Fill is done afterwards by:  kicad-cli pcb drc --refill-zones --save-board ...
"""
import sys
import math
import uuid
import os

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'routed.kicad_pcb')
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'zoned.kicad_pcb')

ANT = (-13.0, 34.0, 13.0, 42.75)   # antenna keepout (new spec: y from 34)
INS = 0.55


def arc(cx, cy, r, a0, a1, n=18):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]


# board outline inset by INS (clockwise list, consistent arc joins)
W = 30.0 - INS               # 29.45
H = 42.75 - INS              # 42.2
CX, CY = 27.0, 39.75         # corner arc centres
RC = 3.0 - INS               # 2.45
RS = 3.75 + INS              # 4.30 (slot arcs)
P = [(-W, -CY)]
P += [(-W, -32.75 - INS), (-27.0, -32.75 - INS)]
P += arc(-27.0, -29.0, RS, -90, 90)
P += [(-W, -25.25 + INS), (-W, 25.25 - INS), (-27.0, 25.25 - INS)]
P += arc(-27.0, 29.0, RS, -90, 90)
P += [(-W, 32.75 + INS), (-W, CY)]
P += arc(-27.0, CY, RC, 180, 90, 10)
P += [(-13.0 - 0.2, H), (-13.0 - 0.2, ANT[1]), (13.0 + 0.2, ANT[1]), (13.0 + 0.2, H), (CX, H)]
P += arc(27.0, CY, RC, 90, 0, 10)
P += [(W, 32.75 + INS), (27.0, 33.3)]
P += arc(27.0, 29.0, RS, 90, 270)
P += [(W, 25.25 - INS), (W, -25.25 + INS), (27.0, -24.7)]
P += arc(27.0, -29.0, RS, 90, -90)
P += [(W, -32.75 - INS), (W, -CY)]
P += arc(27.0, -CY, RC, 0, -90, 10)
P += [(-27.0, -H)]
P += arc(-27.0, -CY, RC, 270, 180, 10)

R3V3 = (-27.2, 0.5, 18.0, 30.0)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def zone(net, layer, pts, prio, name, clear=0.15):
    u = uuid.uuid4()
    out = []
    out.append('\t(zone')
    out.append('\t\t(net "%s")' % net)
    out.append('\t\t(layer "%s")' % layer)
    out.append('\t\t(uuid "%s")' % u)
    out.append('\t\t(name "%s")' % name)
    out.append('\t\t(hatch edge 0.5)')
    out.append('\t\t(priority %d)' % prio)
    out.append('\t\t(connect_pads yes')
    out.append('\t\t\t(clearance %s)' % clear)
    out.append('\t\t)')
    out.append('\t\t(min_thickness 0.2)')
    out.append('\t\t(fill yes')
    out.append('\t\t\t(thermal_gap 0.5)')
    out.append('\t\t\t(thermal_bridge_width 0.5)')
    out.append('\t\t\t(island_removal_mode 0)')
    out.append('\t\t)')
    out.append('\t\t(polygon')
    out.append('\t\t\t(pts')
    line = '\t\t\t\t'
    for i, (x, y) in enumerate(pts):
        tok = '(xy %s %s) ' % (round(x, 3), round(y, 3))
        if len(line) + len(tok) > 110:
            out.append(line.rstrip())
            line = '\t\t\t\t'
        line += tok
    if line.strip():
        out.append(line.rstrip())
    out.append('\t\t\t)')
    out.append('\t\t)')
    out.append('\t)')
    return '\n'.join(out)


blocks = []
blocks.append(zone('GND', 'F.Cu', P, 0, 'GND_F.Cu'))
blocks.append(zone('GND', 'B.Cu', P, 0, 'GND_B.Cu'))
blocks.append(zone('+3V3', 'F.Cu', rect(*R3V3), 1, 'P3V3_F.Cu'))
blocks.append(zone('+3V3', 'B.Cu', rect(*R3V3), 1, 'P3V3_B.Cu'))
blocks.append(zone('+12V', 'B.Cu', rect(-24.5, -13.5, 1.5, 0.5), 2, 'P12V_B.Cu'))

s = open(SRC, encoding='utf-8').read()
# strip existing top-level zone blocks
out = []
i = 0
while True:
    k = s.find('\n\t(zone', i)
    if k < 0:
        out.append(s[i:])
        break
    out.append(s[i:k])
    j = k + 1
    d = 0
    while j < len(s):
        if s[j] == '(':
            d += 1
        elif s[j] == ')':
            d -= 1
            if d == 0:
                break
        j += 1
    i = j + 1
s = ''.join(out)
tail = s.rstrip()
assert tail.endswith(')')
s2 = tail[:-1].rstrip('\n') + '\n' + '\n'.join(blocks) + '\n)\n'
open(DST, 'w', encoding='utf-8').write(s2)
print('wrote %s : %d zones, GND outline %d points' % (DST, len(blocks), len(P)))
