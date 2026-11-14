import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
a = json.load(open(sys.argv[1], encoding='utf-8'))
b = json.load(open(sys.argv[2], encoding='utf-8'))
print('pads before/after:', len(a), len(b))
assert set(a) == set(b), 'pad set changed!'
diff = [(k, a[k], b[k]) for k in a if a[k] != b[k]]
print('changed pad nets: %d' % len(diff))
for k, x, y in sorted(diff):
    print('   %-8s %r -> %r' % (k, x, y))
same = sum(1 for k in a if a[k] == b[k])
print('unchanged pads: %d/%d' % (same, len(a)))
