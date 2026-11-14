import sys
sys.path.insert(0, '_work')
from sexpr import *
import pcbnew

netfile, boardfile, schfile = sys.argv[1], sys.argv[2], sys.argv[3]
root = parse(open(netfile, encoding='utf-8').read())[0]
sch = {}
for n in kids(kid(root, 'nets'), 'net'):
    name = kid(n, 'name')[1]
    for node in kids(n, 'node'):
        if kid(node, 'ref')[1] == 'U7':
            sch[kid(node, 'pin')[1]] = name

sroot = parse(open(schfile, encoding='utf-8').read())[0]
libsym = [s for s in kids(kid(sroot, 'lib_symbols'), 'symbol') if s[1] == 'ESP32-DevKitC_30P_1'][0]
pname = {}
for p in find_all(libsym, 'pin'):
    pname[kid(p, 'number')[1]] = kid(p, 'name')[1]

b = pcbnew.LoadBoard(boardfile)
u7 = [f for f in b.GetFootprints() if f.GetReference() == 'U7'][0]
bd = dict((pad.GetNumber(), pad.GetNetname()) for pad in u7.Pads())

print('%-4s %-7s %-32s %-32s %s' % ('pad', 'pin', 'SCHEMATIC netlist', 'BOARD pad net', 'verdict'))
mism = 0
for num in sorted(sch, key=lambda x: int(x)):
    s = sch[num]; t = bd.get(num, '<missing>')
    ok = 'MATCH' if s == t else '>>> MISMATCH'
    if s != t: mism += 1
    print('%-4s %-7s %-32s %-32s %s' % (num, pname.get(num, '?'), s, t, ok))
print('pads compared: %d   mismatches: %d   board pads for U7: %d' % (len(sch), mism, len(bd)))
