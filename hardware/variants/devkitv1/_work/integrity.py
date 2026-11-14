import json, io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
d = json.load(open('kamaz_leveler_devkitv1.kicad_pro', encoding='utf-8'))
ds = d['board']['design_settings']
rs = ds['rule_severities']
ign = sorted(k for k, v in rs.items() if v == 'ignore')
print('net_settings.classes (%d):' % len(d['net_settings']['classes']))
for c in d['net_settings']['classes']:
    print('   ', c.get('name'), 'clearance=', c.get('clearance_mm'), 'track_width=', c.get('track_width_mm'), 'via_dia=', c.get('via_diameter_mm'), 'via_drill=', c.get('via_drill_mm'))
print('rule_severities total:', len(rs), '| ignore count:', len(ign))
for k in ign: print('   IGNORE', k)
print('drc_exclusions:', ds.get('drc_exclusions'))
print('netclass_assignments:', len(d['net_settings'].get('netclass_assignments') or {}))
print('netclass_patterns:', [(p.get('netclass'), p.get('pattern')) for p in (d['net_settings'].get('netclass_patterns') or [])])
