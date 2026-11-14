import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
p = sys.argv[1]
d = json.load(open(p, encoding='utf-8'))
def pt(it):
    return it.get('pos',{})
print("== counts: viol=%d unc=%d parity=%d" % (len(d.get('violations',[])), len(d.get('unconnected_items',[])), len(d.get('schematic_parity',[]))))
sev = {}
for v in d.get('violations',[]):
    sev[v.get('severity')] = sev.get(v.get('severity'),0)+1
print("severity:", sev)
for i,v in enumerate(d.get('violations',[])):
    items = v.get('items',[])
    parts=[]
    for it in items:
        parts.append("%s:%s" % (it.get('description','')[:40], it.get('pos',{})))
    print("V%d [%s] %s | %s" % (i, v.get('severity'), v.get('type'), " || ".join(parts)))
print()
for i,v in enumerate(d.get('unconnected_items',[])):
    items = v.get('items',[])
    parts=[]
    for it in items:
        parts.append("%s @%s" % (it.get('description','')[:60], it.get('pos',{})))
    print("U%d [%s] %s | %s" % (i, v.get('severity'), v.get('type'), " || ".join(parts)))
print()
for i,v in enumerate(d.get('schematic_parity',[])):
    items = v.get('items',[])
    print("P%d %s | %s" % (i, v.get('type'), " || ".join(str(it.get('description'))[:70] for it in items)))
