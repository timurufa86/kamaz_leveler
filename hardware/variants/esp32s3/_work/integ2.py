import json, sys, collections
d = json.load(open(sys.argv[1], encoding='utf-8'))
found = {}
def walk(o, path=''):
    if isinstance(o, dict):
        for k, v in o.items():
            if k in ('classes', 'rule_severities', 'drc_exclusions', 'netclass_patterns', 'custom_rules'):
                found.setdefault(k, []).append((path + '/' + k, v))
            walk(v, path + '/' + k)
    elif isinstance(o, list):
        for i, v in enumerate(o):
            walk(v, path + '[%d]' % i)
walk(d)
for k in ('classes', 'netclass_patterns', 'rule_severities', 'drc_exclusions', 'custom_rules'):
    for (p, v) in found.get(k, []):
        if k == 'classes' and isinstance(v, list):
            print('netclasses: %d  -> %s' % (len(v), ', '.join('%s(%s/%s via %s/%s)' % (c.get('name'), c.get('track_width'), c.get('clearance'), c.get('via_diameter'), c.get('via_drill')) for c in v)))
        elif k == 'rule_severities' and isinstance(v, dict):
            cnt = collections.Counter(v.values())
            print('rule_severities: %d entries %s' % (len(v), dict(cnt)))
            ign = sorted(x for x, y in v.items() if y == 'ignore')
            print('   ignore waivers (%d): %s' % (len(ign), ', '.join(ign)))
        elif isinstance(v, list):
            print('%s: %d' % (k, len(v)))
        elif isinstance(v, str):
            print('%s: %r' % (k, v))
