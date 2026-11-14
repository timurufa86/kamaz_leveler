import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
sys.path.insert(0, sys.argv[1])
from sexpr import parse, kids, kid, head, find_all

root = parse(open(sys.argv[2], encoding='utf-8').read())[0]
print('root head:', head(root))
syms = kids(root, 'symbol')
for s in syms:
    name = s[1]
    pins = find_all(s, 'pin')
    print('%-40s pins=%d' % (name, len(pins)))
    if 'DevKit' in name:
        for p in pins:
            num = kid(p, 'number'); nm = kid(p, 'name')
            at = kid(p, 'at')
            print('    %-4s %-22s %-14s at %s' % (num[1] if num else '?', (nm[1] if nm else '?'), p[1] if p else '?', ' '.join(at[1:]) if at else '?'))
