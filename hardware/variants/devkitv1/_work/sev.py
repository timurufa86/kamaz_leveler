import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open(sys.argv[1], encoding='utf-8'))
rs = d['board']['design_settings']['rule_severities']
ign = sorted(k for k,v in rs.items() if v == 'ignore')
print('ignored count:', len(ign))
for k in ign: print('   ', k)
for k in ('net_conflict','footprint_symbol_field_mismatch','extra_footprint','missing_footprint','footprint_symbol_mismatch'):
    print('%-32s = %s' % (k, rs.get(k)))
