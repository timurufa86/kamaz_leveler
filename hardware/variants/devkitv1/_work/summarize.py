import sys, io, json, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)

path = sys.argv[1]
d = json.load(open(path, encoding='utf-8'))
print('top keys:', list(d.keys()))
for sect in ('violations','unconnected_items','schematic_parity'):
    items = d.get(sect) or []
    print('--- %s: %d' % (sect, len(items)))
    c = collections.Counter()
    for it in items:
        c[(it.get('type'), it.get('severity'))] += 1
    for k, v in sorted(c.items()):
        print('    %-45s %-12s %d' % (k[0], k[1], v))
if (d.get('schematic_parity') or []) or len(sys.argv) > 2:
    print('=== schematic_parity detail ===')
    for it in (d.get('schematic_parity') or []):
        desc = it.get('description','')
        print(' -', it.get('type'), '|', it.get('severity'), '|', desc)
        for ii in it.get('items', []):
            print('      item:', json.dumps(ii, ensure_ascii=False))
