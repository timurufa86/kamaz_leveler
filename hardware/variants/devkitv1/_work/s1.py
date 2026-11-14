import sys
sys.path.insert(0, '_work')
from sexpr import *
text = open('kamaz_leveler_devkitv1.kicad_sch', encoding='utf-8').read()
root = parse(text)[0]
print('root head', head(root))
kinds = {}
for c in root:
    if isinstance(c, list): kinds[head(c)] = kinds.get(head(c),0)+1
print(json.dumps(kinds, indent=1))
libsyms = kid(root, 'lib_symbols')
print('lib_symbols count', len(kids(libsyms,'symbol')))
for s in kids(libsyms,'symbol'):
    print('  LIB', s[1], 'pins', len(find_all(s,'pin')))
syms = kids(root,'symbol')
print('symbol instances', len(syms))
