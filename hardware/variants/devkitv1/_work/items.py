import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open(sys.argv[1], encoding='utf-8'))
for it in (d.get('schematic_parity') or []):
    items = it.get('items') or []
    pad = items[0].get('description') if items else ''
    print('%-30s | %-14s | %s' % (it.get('type'), pad, it.get('description')))
