import sys
lines = open(sys.argv[1], encoding='utf-8', newline='').read().splitlines(keepends=True)
def strip(l): return l.strip()
TARGETS = [(66.04, 284.48), (53.34, 287.02), (110.49, 284.48), (121.92, 287.02), (63.5, 251.46)]
mx = 0
import re
for l in lines:
    m = re.search(r'\(property "Reference" "#PWR(\d+)"', l)
    if m: mx = max(mx, int(m.group(1)))
f = open(sys.argv[2], 'w', encoding='utf-8')
def W(s): f.write(s + '\n'); f.flush()
W('max #PWR = %d' % mx)
blocks = []
i = 0
while i < len(lines):
    if strip(lines[i]) == '(symbol' and lines[i].startswith('\t') and not lines[i].startswith('\t\t'):
        j = i + 1
        while j < len(lines) and not (strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t')):
            j += 1
        at = None; ref = None; lid = None
        for k in range(i, j+1):
            t = strip(lines[k])
            if t.startswith('(at ') and at is None: at = t[4:].rstrip(')').split()
            if t.startswith('(property "Reference"') and ref is None: ref = t
            if t.startswith('(lib_id '): lid = t
        if at:
            p = (round(float(at[0]), 2), round(float(at[1]), 2))
            if p in TARGETS:
                W('#### block at %s  %s  %s  lines %d-%d' % (p, ref, lid, i+1, j+1))
                for k in range(i, j+1): W(lines[k].rstrip('\r\n'))
        i = j + 1
    else:
        i += 1
f.close()
