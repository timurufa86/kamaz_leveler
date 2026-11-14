import sys, io, json
sys.path.insert(0, '_work')
from sexpr import *

netfile, ref = sys.argv[1], sys.argv[2]
root = parse(open(netfile, encoding='utf-8').read())[0]
nets = kid(root,'nets')
padnet = {}
for n in kids(nets,'net'):
    name = kid(n,'name')[1]
    for node in kids(n,'node'):
        if kid(node,'ref')[1] == ref:
            padnet.setdefault(kid(node,'pin')[1], []).append(name)
pads = sorted(padnet, key=lambda x:int(x)) if padnet else []
print('nets in netlist:', len(kids(nets,'net')))
print('pads of %s present in netlist: %d -> %s' % (ref, len(padnet), pads))
for num in sorted(padnet, key=lambda x: int(x)):
    print('  pad %-3s -> %s' % (num, ' | '.join(padnet[num])))
