import sys, json
sys.path.insert(0, sys.argv[1])
from sexpr import parse, kids, kid
import pcbnew
netfile, boardfile = sys.argv[2], sys.argv[3]
root = parse(open(netfile, encoding='utf-8').read())[0]
nets = kid(root, 'nets')
sch = {}
nall = kids(nets, 'net')
for n in nall:
    nm = kid(n, 'name')[1]
    for node in kids(n, 'node'):
        if kid(node, 'ref')[1] == 'U7':
            sch[kid(node, 'pin')[1]] = nm
b = pcbnew.LoadBoard(boardfile)
u7 = [f for f in b.GetFootprints() if f.GetReference() == 'U7'][0]
bd = dict((p.GetNumber(), p.GetNetname()) for p in u7.Pads())
print('netlist nets: %d ; U7 pins in netlist: %d ; board U7 pads: %d' % (len(nall), len(sch), len(bd)))
print('%-4s %-34s %-34s %s' % ('pad', 'SCHEMATIC netlist', 'BOARD pad net', 'verdict'))
mism = 0; missing = 0
for n in range(1, 45):
    s = sch.get(str(n), '<no pin in schematic>'); t = bd.get(str(n), '<missing>')
    if s.startswith('unconnected-'): s = '(no-connect flag) ' + str(n)
    ok = (s == t)
    if not ok: mism += 1
    if s.startswith('<no pin'): missing += 1
    print('%-4d %-34s %-34s %s' % (n, s, t, 'MATCH' if ok else '>>> MISMATCH'))
print('pads matching: %d/44   mismatches: %d   pads with no schematic pin: %d' % (44-mism, mism, missing))
