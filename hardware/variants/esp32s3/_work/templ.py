import sys
lines = open(sys.argv[1], encoding='utf-8', newline='').read().splitlines(keepends=True)
def strip(l): return l.strip()
PINS = [(72.39, 251.46 + 2.54*k) for k in range(15)] + [(102.87, 251.46 + 2.54*k) for k in range(15)]
out = open(sys.argv[2], 'w', encoding='utf-8')
def W(s): out.write(s + '\n')
# find blocks at indent 1 starting with (symbol  OR (label OR (no_connect
i = 0
seen = {}
while i < len(lines):
    s = strip(lines[i])
    tag = None
    for t in ('(symbol', '(label', '(no_connect', '(global_label', '(hierarchical_label'):
        if s == t or s.startswith(t + ' '):
            tag = t; break
    if tag is None:
        i += 1; continue
    if not (lines[i].startswith('\t') and not lines[i].startswith('\t\t')):
        i += 1; continue
    j = i + 1
    while j < len(lines):
        if strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t'):
            break
        j += 1
    if tag == '(symbol':
        at = None; ref = None
        for k in range(i, j + 1):
            if strip(lines[k]).startswith('(at ') and at is None:
                at = strip(lines[k])[4:].rstrip(')').strip()
            if strip(lines[k]).startswith('(property "Reference"'):
                ref = strip(lines[k])
        if at:
            parts = at.split()
            ax, ay = float(parts[0]), float(parts[1])
            if any(abs(ax-px) < 1e-6 and abs(ay-py) < 1e-6 for px, py in PINS):
                W('### SYMBOL at %s  %s' % (at, ref))
                for k in range(i, j + 1): W(lines[k].rstrip('\r\n'))
                seen[at] = (i, j)
    i = j + 1
# sample label + no_connect blocks
cnt = 0
for i in range(len(lines)):
    if strip(lines[i]).startswith('(label ') and cnt < 2:
        j = i
        while not (strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t')): j += 1
        W('### LABEL'); 
        for k in range(i, j+1): W(lines[k].rstrip('\r\n'))
        cnt += 1
cnt = 0
for i in range(len(lines)):
    if strip(lines[i]) == '(no_connect' and cnt < 1:
        j = i
        while not (strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t')): j += 1
        W('### NO_CONNECT')
        for k in range(i, j+1): W(lines[k].rstrip('\r\n'))
        cnt += 1
# R43
for i in range(len(lines)):
    if '(property "Reference" "R43"' in lines[i]:
        # walk back to block start
        s = i
        while not (lines[s].startswith('\t( symbol') or (lines[s].startswith('\t') and not lines[s].startswith('\t\t') and strip(lines[s]) == '(symbol')):
            s -= 1
        j = s + 1
        while not (strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t')): j += 1
        W('### R43 BLOCK lines %d-%d' % (s+1, j+1))
        for k in range(s, min(s+30, j+1)): W(lines[k].rstrip('\r\n'))
        break
out.close()
