import sys, io, json
sys.path.insert(0, '_work')
from sexpr import *
root = parse(open(sys.argv[1], encoding='utf-8').read())[0]
nets = kid(root, 'nets')
m = {}
for n in kids(nets, 'net'):
    name = kid(n, 'name')[1]
    for node in kids(n, 'node'):
        if kid(node, 'ref')[1] == 'U7':
            m[kid(node, 'pin')[1]] = name
json.dump(m, open(sys.argv[2], 'w', encoding='utf-8'), indent=1)
print('U7 pad->schematic-net entries:', len(m))
print(json.dumps(m, indent=1, ensure_ascii=False))
