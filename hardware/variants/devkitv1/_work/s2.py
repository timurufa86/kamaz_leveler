import sys
sys.path.insert(0, '_work')
from sexpr import *
root = parse(open('kamaz_leveler_devkitv1.kicad_sch', encoding='utf-8').read())[0]
for s in kids(root,'symbol'):
    props = {}
    for p in kids(s,'property'):
        props[p[1]] = p[2]
    ref = None
    inst = kid(s,'instances')
    if inst:
        for pr in find_all(inst,'reference'):
            ref = pr[1]; break
    if props.get('Reference') == 'U7' or ref == 'U7':
        print('FOUND lib_id', s[1], 'uuid', kid(s,'uuid')[1])
        print('at', kid(s,'at')[1:], 'mirror', kid(s,'mirror'), 'unit', kid(s,'unit'))
        for k,v in props.items(): print('   prop', k, '=', repr(v))
        print('pins in instance:', [ (p[1], kid(p,'uuid')[1]) for p in kids(s,'pin')][:40])
        break
