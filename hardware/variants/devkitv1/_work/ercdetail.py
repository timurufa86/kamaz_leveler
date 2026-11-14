import json, io, sys, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open(sys.argv[1], encoding='utf-8'))
print('keys:', list(d.keys()))
for k in ('violations', 'unconnected_items', 'schematic_parity', 'sheets'):
    v = d.get(k)
    if isinstance(v, list):
        print('%-18s : %d' % (k, len(v)))
    elif v is not None:
        print('%-18s : %r' % (k, v))
for k in ('included_severities', 'ignored_checks'):
    print('%-18s : %s' % (k, d.get(k)))
