import sys, io, json, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open(sys.argv[1], encoding='utf-8'))
print('source:', d.get('source'))
print('kicad_version:', d.get('kicad_version'))
print('ignored_checks:', d.get('ignored_checks'))
print('included_severities:', d.get('included_severities'))
for sect in ('violations', 'unconnected_items', 'schematic_parity'):
    items = d.get(sect) or []
    print('%-18s : %d items' % (sect, len(items)))
    c = collections.Counter((i.get('type'), i.get('severity')) for i in items)
    for (t, s), v in sorted(c.items()):
        print('      %-34s %-8s %d' % (t, s, v))
print('--- every remaining schematic_parity item ---')
for it in (d.get('schematic_parity') or []):
    items = it.get('items', [])
    desc = items[0].get('description') if items else ''
    print('   %-18s | %-8s | %-22s | %s' % (it.get('type'), it.get('severity'), desc, it.get('description')))
