import sys, io, json, uuid, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)

src, dst, opsjson = sys.argv[1], sys.argv[2], sys.argv[3]
ops = json.loads(opsjson)
text = open(src, encoding='utf-8', newline='').read()
lines = text.splitlines(keepends=True)
print('lines:', len(lines), 'CRLF:', text.count('\r\n'))

def strip(l): return l.strip()

def lat(l):
    s = l
    if not s.startswith('(at '): return None
    s = s[4:].rstrip(')').strip()
    parts = s.split()
    if len(parts) < 2: return None
    try: return [float(parts[0]), float(parts[1]), parts[2] if len(parts) > 2 else None]
    except ValueError: return None

def fmt(v):
    s = ('%.4f' % float(v)).rstrip('0').rstrip('.')
    return s if s else '0'

fails = 0

# ---------- 1) label moves ----------
for name, ox, oy, nx, ny, angle, justify in ops.get('moves', []):
    found = -1
    for i in range(len(lines) - 1):
        if strip(lines[i]) == '(label "%s"' % name:
            a = lat(strip(lines[i+1]))
            if a and abs(a[0]-ox) < 1e-6 and abs(a[1]-oy) < 1e-6:
                found = i; break
    if found < 0:
        print('!! LABEL NOT FOUND', name, ox, oy); fails += 1; continue
    print('label %-12s %-24s -> at %s %s %s' % (name, strip(lines[found+1]), fmt(nx), fmt(ny), angle))
    lines[found+1] = '\t\t(at %s %s %s\r\n' % (fmt(nx), fmt(ny), angle)
    for j in range(found+2, min(found+16, len(lines))):
        if strip(lines[j]).startswith('(justify '):
            if strip(lines[j]) != '(justify %s bottom' % justify:
                print('    justify %s -> %s' % (strip(lines[j]), '(justify %s bottom' % justify))
                lines[j] = '\t\t\t(justify %s bottom\r\n' % justify
            break

# ---------- 2) no_connect removal ----------
def block_end(i):
    j = i + 1
    while j < len(lines):
        if strip(lines[j]) == ')' and lines[j].startswith('\t') and not lines[j].startswith('\t\t'):
            return j
        j += 1
    raise SystemExit('unterminated block at %d' % i)

for x, y in ops.get('rm_nc', []):
    hit = -1
    for i in range(len(lines) - 1):
        if strip(lines[i]) == '(no_connect':
            a = lat(strip(lines[i+1]))
            if a and abs(a[0]-x) < 1e-6 and abs(a[1]-y) < 1e-6:
                hit = i; break
    if hit < 0:
        print('!! NC NOT FOUND', x, y); fails += 1; continue
    j = block_end(hit)
    print('remove no_connect at %s %s (lines %d-%d)' % (fmt(x), fmt(y), hit+1, j+1))
    del lines[hit:j+1]

# ---------- 3) no_connect addition ----------
if ops.get('add_nc'):
    tmpl = None
    for i in range(len(lines)):
        if strip(lines[i]) == '(no_connect':
            tmpl = lines[i:block_end(i)+1]; last = block_end(i); break
    assert tmpl, 'no template'
    ins = []
    for x, y in ops['add_nc']:
        for l in tmpl:
            if strip(l).startswith('(at '): ins.append('\t\t(at %s %s\r\n' % (fmt(x), fmt(y)))
            elif strip(l).startswith('(uuid '): ins.append('\t\t(uuid "%s"\r\n' % uuid.uuid4())
            else: ins.append(l)
        print('add no_connect at %s %s' % (fmt(x), fmt(y)))
    # insert after the LAST existing no_connect block, recomputed
    lastend = None
    for i in range(len(lines)):
        if strip(lines[i]) == '(no_connect':
            lastend = block_end(i)
    lines[lastend+1:lastend+1] = ins

# ---------- 4) property value edits on symbol instances ----------
props = ops.get('props', [])
if props:
    # collect instance blocks at indent 1 tab starting with '(symbol'
    blocks = []
    i = 0
    while i < len(lines):
        if lines[i].rstrip('\r\n') == '\t(symbol':
            j = block_end(i)
            blocks.append((i, j)); i = j + 1
        else:
            i += 1
    print('symbol instance blocks found:', len(blocks))
    for ref, pname, newval in props:
        done = False
        for (s, e) in blocks:
            body = lines[s:e+1]
            if not any('(property "Reference" "%s"' % ref in l for l in body):
                continue
            for k in range(s, e+1):
                m = re.match(r'^(\t+)\(property "%s" "' % re.escape(pname), lines[k].rstrip('\r\n'))
                if m:
                    # replace whole value part: keep indent, set empty
                    lines[k] = '%s(property "%s" "%s"\r\n' % (m.group(1), pname, newval)
                    print('prop %s.%s -> %r (was %r)' % (ref, pname, newval, m.group(0).split('" "')[-1]))
                    done = True
                    break
            break
        if not done:
            print('!! PROP NOT FOUND', ref, pname); fails += 1

if fails:
    print('FAILURES:', fails); sys.exit(5)
open(dst, 'w', encoding='utf-8', newline='').write(''.join(lines))
print('written', dst)
