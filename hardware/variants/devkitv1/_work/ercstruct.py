import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open(sys.argv[1], encoding='utf-8'))
sh = d['sheets']
print('sheet count:', len(sh))
for s in sh:
    print('sheet keys:', list(s.keys()))
    print('  path:', s.get('path'))
    viol = s.get('violations') or []
    print('  violations:', len(viol))
    for v in viol[:10]:
        print('   ', v.get('type'), v.get('severity'), v.get('description'))
print('total violations:', sum(len(s.get('violations') or []) for s in sh))
