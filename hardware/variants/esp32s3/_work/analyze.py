import sys, io, json, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew

d = json.load(open(sys.argv[1], encoding='utf-8'))
print('=== DRC SECTIONS ===')
for sect in ('violations', 'unconnected_items', 'schematic_parity'):
    items = d.get(sect) or []
    print('%-18s : %d' % (sect, len(items)))
    c = collections.Counter((i.get('type'), i.get('severity')) for i in items)
    for (t, s), v in sorted(c.items()):
        print('      %-40s %-9s %d' % (t, s, v))

print()
print('=== VIOLATIONS (detail) ===')
for it in (d.get('violations') or []):
    ii = it.get('items') or []
    pos = ii[0].get('pos') if ii else None
    print('%-24s %-8s %s' % (it.get('type'), it.get('severity'), it.get('description')))
    for x in ii:
        print('     item %-14s %s' % (x.get('description'), x.get('pos')))
print()
print('=== UNCONNECTED (detail) ===')
for it in (d.get('unconnected_items') or []):
    ii = it.get('items') or []
    print('%-24s %-8s %s' % (it.get('type'), it.get('severity'), it.get('description')))
    for x in ii:
        print('     item %-14s %s' % (x.get('description'), x.get('pos')))

print()
print('=== SCHEMATIC PARITY (detail) ===')
c2 = collections.Counter()
for it in (d.get('schematic_parity') or []):
    ii = it.get('items') or []
    desc = ii[0].get('description') if ii else ''
    c2[it.get('type')] += 1
    print('%-32s | %-26s | %s' % (it.get('type'), desc, it.get('description')))
print(c2)
