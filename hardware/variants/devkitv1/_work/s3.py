import sys
sys.path.insert(0, '_work')
from sexpr import *
root = parse(open('kamaz_leveler_devkitv1.kicad_sch', encoding='utf-8').read())[0]
libs = kid(root,'lib_symbols')
for s in kids(libs,'symbol'):
    if s[1].startswith('ESP32-DevKitC_30P_1'):
        print('LIB SYMBOL', s[1])
        for p in find_all(s,'pin'):
            num = kid(p,'number')
            nam = kid(p,'name')
            at = kid(p,'at')
            print('  num=%-4s name=%-10s at=%s len=%s type=%s orient=%s' % (
                num[1] if num else '?', nam[1] if nam else '?', at[1:4], kid(p,'length')[1] if kid(p,'length') else '?',
                p[1], p[2] if len(p)>2 else '?'))
