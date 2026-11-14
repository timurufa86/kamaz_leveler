import json, sys, hashlib
p = sys.argv[1]
d = json.load(open(p, encoding='utf-8'))
bd = d.get('board', d)
ncs = bd.get('netclasses') or {}
print('netclasses: %d' % len(ncs))
for k, v in ncs.items():
    print('   %-22s track %.3f/%.3f  via %.3f/%.3f' % (k, v.get('track_width', 0)*1, v.get('clearance', 0), v.get('via_diameter', 0), v.get('via_drill', 0)))
print('netclass_patterns: %d' % len(bd.get('netclass_patterns') or []))
drc = bd.get('drc') or d.get('drc') or {}
sev = bd.get('rule_severities') or {}
print('rule_severities: %d entries' % len(sev))
import collections
c = collections.Counter(sev.values())
print('   ', dict(c))
ign = sorted(k for k, v in sev.items() if v == 'ignore')
print('   ignore waivers (%d): %s' % (len(ign), ', '.join(ign)))
print('drc_exclusions: %d' % len((bd.get('drc_exclusions') or d.get('drc_exclusions') or [])))
print('meta filename: %s' % (bd.get('meta', {}) or {}).get('filename', '?'))
