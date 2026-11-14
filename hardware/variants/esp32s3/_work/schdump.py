import sys, io, math
sys.path.insert(0, sys.argv[1])
from sexpr import parse, kids, kid, head, find_all
out = open(sys.argv[3], 'w', encoding='utf-8')
def W(s): out.write(s + '\n'); out.flush()
root = parse(open(sys.argv[2], encoding='utf-8').read())[0]
libs = kid(root, 'lib_symbols')
symdefs = {}
for s in kids(libs, 'symbol'):
    symdefs[s[1]] = s
W('lib symbols: %d' % len(symdefs))

inst = None
for s in kids(root, 'symbol'):
    p = kid(s, 'property')
    ref = None
    for pp in kids(s, 'property'):
        if pp[1] == 'Reference': ref = pp[2]
    if ref == 'U7': inst = s
assert inst, 'U7 not found'
lid = kid(inst, 'lib_id')[1]
at = kid(inst, 'at')
mir = kid(inst, 'mirror')
W('U7 lib_id=%s at=%s mirror=%s unit=%s' % (lid, ' '.join(at[1:]), mir[1] if mir else '-', kid(inst,'unit')[1] if kid(inst,'unit') else '?'))
for pp in kids(inst, 'property'):
    W('   prop %-14s = %r' % (pp[1], pp[2] if len(pp) > 2 else ''))
name = lid.split(':')[-1] + '_1'
sd = symdefs.get(name) or symdefs.get(lid.split(':')[-1])
W('symbol def %s pins=%d' % (name, len(find_all(sd, 'pin'))))
IX, IY, IROT = float(at[1]), float(at[2]), (float(at[3]) if len(at) > 3 else 0.0)
W('instance x=%.3f y=%.3f rot=%s' % (IX, IY, IROT))
def xform(px, py):
    a = math.radians(IROT)
    rx = px*math.cos(a) - py*math.sin(a)
    ry = px*math.sin(a) + py*math.cos(a)
    return (IX + rx, IY - ry)
pins = []
for p in find_all(sd, 'pin'):
    num = kid(p, 'number')[1]; nm = kid(p, 'name')[1]
    pat = kid(p, 'at'); px, py, pang = float(pat[1]), float(pat[2]), float(pat[3])
    ax, ay = xform(px, py)
    pins.append((int(num), nm, ax, ay, p[1], pang))
pins.sort()
labels = []
for l in kids(root, 'label'):
    a = kid(l, 'at')
    labels.append((l[1], float(a[1]), float(a[2]), float(a[3]) if len(a) > 3 else 0.0))
ncs = []
for l in kids(root, 'no_connect'):
    a = kid(l, 'at'); ncs.append((float(a[1]), float(a[2])))
W('')
W('%-4s %-8s %-16s %-10s %-10s %s' % ('num', 'name', 'type', 'abs_x', 'abs_y', 'attached'))
for num, nm, ax, ay, typ, pang in pins:
    hit = [l for l in labels if abs(l[1]-ax) < 1e-3 and abs(l[2]-ay) < 1e-3]
    nhit = [n for n in ncs if abs(n[0]-ax) < 1e-3 and abs(n[1]-ay) < 1e-3]
    tag = ('LABEL ' + ','.join('%s@(%.2f,%.2f,%g)' % (h[0], h[1], h[2], h[3]) for h in hit)) if hit else ('NC' if nhit else '')
    W('%-4d %-8s %-16s %-10.3f %-10.3f %s' % (num, nm, typ, ax, ay, tag))
W('')
W('all labels (%d):' % len(labels))
for l in sorted(labels, key=lambda r: (r[2], r[1])):
    W('   %-14s at %.3f,%.3f %g' % l)
W('')
W('all no_connect (%d):' % len(ncs))
for n in sorted(ncs, key=lambda r: (r[1], r[0])):
    W('   at %.3f,%.3f' % n)
W('')
W('symbol instances:')
for s in kids(root, 'symbol'):
    ref = None
    for pp in kids(s, 'property'):
        if pp[1] == 'Reference': ref = pp[2]
    a = kid(s, 'at')
    W('   %-6s %-32s at %s' % (ref, kid(s, 'lib_id')[1], ' '.join(a[1:]) if a else '?'))
out.close()
