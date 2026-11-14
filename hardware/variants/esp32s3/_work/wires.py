import sys
lines = open(sys.argv[1], encoding='utf-8', newline='').read().splitlines(keepends=True)
def strip(l): return l.strip()
PINS = {}
for k in range(15):
    PINS[(round(72.39, 2), round(251.46 + 2.54*k, 2))] = 'L%d' % (k+1)
    PINS[(round(102.87, 2), round(251.46 + 2.54*k, 2))] = 'R%d' % (k+16)
out = sys.argv[2]
f = open(out, 'w', encoding='utf-8')
def W(s): f.write(s + '\n'); f.flush()
n = 0
for i, l in enumerate(lines):
    if strip(l).startswith('(wire'):
        pts = []
        j = i
        while j < len(lines) and not (strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t')):
            if strip(lines[j]).startswith('(xy '):
                t = strip(lines[j])[4:].rstrip(')').split()
                pts.append((round(float(t[0]), 2), round(float(t[1]), 2)))
            j += 1
        for p in pts:
            if p in PINS:
                W('WIRE at pin %-4s %s  line %d  pts=%s' % (PINS[p], i+1, len(pts), pts))
                n += 1
W('wires touching U7 pins: %d' % n)
f.close()
