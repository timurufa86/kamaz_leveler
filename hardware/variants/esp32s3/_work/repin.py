import io, sys, re, uuid
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

SCH = 'kamaz_leveler_s3.kicad_sch'
DRY = '--apply' not in sys.argv
src = open(SCH, encoding='utf-8').read()

# ---------- collect top-level items with char offsets ----------
def scan_items(text):
    items = []
    depth = 0
    start = None
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n:
                if text[i] == '\\':
                    i += 2
                    continue
                if text[i] == '"':
                    break
                i += 1
            i += 1
            continue
        if c == '(':
            depth += 1
            if depth == 2:
                start = i
        elif c == ')':
            if depth == 2 and start is not None:
                items.append((start, i + 1, text[start:i + 1]))
                start = None
            depth -= 1
        i += 1
    return items

items = scan_items(src)
print("top-level items:", len(items))

def fnum(v):
    s = ('%.4f' % v).rstrip('0').rstrip('.')
    return s

AT = re.compile(r'\(at\s+([\-0-9.]+)\s+([\-0-9.]+)(?:\s+([\-0-9.]+))?\s*\)')

# ---------- plan ----------
# name, cur_x, cur_y, new_x, new_y, side
MOVES = [
    ("DISP_BL",    72.39, 275.59,  72.39, 247.65, 'L'),
    ("DISP_CS",    72.39, 278.13,  72.39, 250.19, 'L'),
    ("DISP_DC",    72.39, 285.75,  72.39, 260.35, 'L'),
    ("DISP_RES",   72.39, 288.29,  72.39, 262.89, 'L'),
    ("DISP_SCLK",  72.39, 283.21,  72.39, 265.43, 'L'),
    ("SPEED_PULSE",72.39, 252.73,  72.39, 267.97, 'L'),
    ("INT_MCU",    72.39, 260.35,  72.39, 285.75, 'L'),
    ("VLV_EN",     72.39, 257.81,  72.39, 288.29, 'L'),
    ("PRES_SW",    72.39, 255.27, 102.87, 262.89, 'R'),
    ("I2C_SCL",    72.39, 250.19, 102.87, 280.67, 'R'),
    ("I2C_SDA",    72.39, 247.65, 102.87, 283.21, 'R'),
]
NC_REMOVE = [(72.39, 262.89), (72.39, 265.43), (72.39, 267.97),
             (102.87, 262.89), (102.87, 280.67), (102.87, 283.21)]
NC_ADD = [(72.39, 252.73), (72.39, 255.27), (72.39, 257.81),
          (72.39, 275.59), (72.39, 278.13), (72.39, 283.21)]

# index existing items
labels, ncs, last_nc_end = {}, {}, None
for s, e, txt in items:
    head = txt[:60]
    m = re.match(r'\(label\s+"([^"]+)"', head)
    if m:
        a = AT.search(txt)
        key = (m.group(1), round(float(a.group(1)), 2), round(float(a.group(2)), 2))
        labels.setdefault(key, []).append((s, e, txt))
    elif txt.startswith('(no_connect'):
        a = AT.search(txt)
        key = (round(float(a.group(1)), 2), round(float(a.group(2)), 2))
        ncs.setdefault(key, []).append((s, e, txt))
        last_nc_end = max(last_nc_end or 0, e)

# ---------- build edits ----------
edits = []   # (start, end, replacement, kind, note)
problems = []

for name, cx, cy, nx, ny, side in MOVES:
    key = (name, round(cx, 2), round(cy, 2))
    hits = labels.get(key, [])
    if len(hits) != 1:
        problems.append("label %s at (%s,%s): %d matches" % (name, cx, cy, len(hits)))
        continue
    s, e, txt = hits[0]
    a = AT.search(txt)
    rot = 180 if side == 'L' else 0
    old_at = a.group(0)
    new_at = "(at %s %s %d)" % (fnum(nx), fnum(ny), rot)
    if rot == 0:
        if '(justify right bottom)' not in txt:
            problems.append("label %s: justify not found as expected" % name)
        newtxt = txt.replace(old_at, new_at).replace('(justify right bottom)', '(justify left bottom)')
    else:
        newtxt = txt.replace(old_at, new_at)
    edits.append((s, e, newtxt, 'label-move', "%s (%s,%s)->(%s,%s)" % (name, cx, cy, nx, ny)))

for (x, y) in NC_REMOVE:
    key = (round(x, 2), round(y, 2))
    hits = ncs.get(key, [])
    if len(hits) != 1:
        problems.append("no_connect at (%s,%s): %d matches" % (x, y, len(hits)))
        continue
    s, e, txt = hits[0]
    # also swallow following newline+indent so we do not leave a blank line
    end = e
    while end < len(src) and src[end] in ' \t':
        end += 1
    if end < len(src) and src[end] == '\n':
        end += 1
    edits.append((s, end, '', 'nc-remove', "(%s,%s)" % (x, y)))

nc_blocks = []
for (x, y) in NC_ADD:
    if (round(x, 2), round(y, 2)) in ncs:
        problems.append("no_connect already exists at (%s,%s) - refusing duplicate" % (x, y))
    nc_blocks.append('\t(no_connect\n\t\t(at %s %s\n\t\t)\n\t\t(uuid "%s"\n\t\t)\n\t)\n'
                     % (fnum(x), fnum(y), uuid.uuid4()))

print("\n--- plan ---")
for ed in edits:
    print("  %-10s %s" % (ed[3], ed[4]))
print("  nc-add     %d blocks at %s" % (len(NC_ADD), NC_ADD))
nmv = len([e for e in edits if e[3] == 'label-move'])
ncr = len([e for e in edits if e[3] == 'nc-remove'])
print("  total operations: %d (label moves %d + nc add %d + nc remove %d)" % (
    nmv + len(nc_blocks) + ncr, nmv, len(nc_blocks), ncr))

if problems:
    print("\n!!! PROBLEMS:")
    for p in problems:
        print("  ", p)
    sys.exit(2)

# ---------- apply ----------
edits.sort(key=lambda t: t[0], reverse=True)
out = src
for s, e, rep, kind, note in edits:
    out = out[:s] + rep + out[e:]

# append no_connect blocks after the last no_connect item (offsets shifted by edits)
if nc_blocks:
    # recompute insertion point in the new text: end of last no_connect item
    it2 = scan_items(out)
    ins = 0
    for s, e, txt in it2:
        if txt.startswith('(no_connect'):
            ins = max(ins, e)
    if ins == 0:
        ins = out.rstrip().rfind(')')
    pad = '\n' if not out[:ins].endswith('\n') else ''
    out = out[:ins] + pad + ''.join(nc_blocks)[:-1] + out[ins:]

if DRY:
    print("\nDRY RUN - nothing written. Use --apply")
    sys.exit(0)

open(SCH, 'w', encoding='utf-8', newline='\n').write(out)
print("\nWROTE", SCH, len(out), "bytes")
