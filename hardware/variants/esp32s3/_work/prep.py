import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
# prep.py route.json out.json [startx starty]  -> emits tracks polylines + vias
pts = json.load(open(sys.argv[1], encoding='utf-8'))
out = {'add_tracks': [], 'add_vias': []}
n = len(pts)
if len(sys.argv) >= 5:
    pts[0]['x'] = float(sys.argv[3]); pts[0]['y'] = float(sys.argv[4])
# walk: collect per-layer polylines split at vias
i = 0
while i < n:
    layer = pts[i]['l']
    poly = [(pts[i]['x'], pts[i]['y'])]
    j = i + 1
    while j < n and not pts[j].get('via') and pts[j]['l'] == layer:
        poly.append((pts[j]['x'], pts[j]['y']))
        j += 1
    # simplify collinear
    simp = [poly[0]]
    for k in range(1, len(poly)-1):
        ax, ay = poly[k][0]-poly[k-1][0], poly[k][1]-poly[k-1][1]
        bx, by = poly[k+1][0]-poly[k][0], poly[k+1][1]-poly[k][1]
        if abs(ax*by - ay*bx) > 1e-9:
            simp.append(poly[k])
    if len(poly) > 1: simp.append(poly[-1])
    if len(simp) >= 2:
        out['add_tracks'].append({'layer': layer, 'pts': [[round(a,4), round(b,4)] for a,b in simp]})
    if j < n and pts[j].get('via'):
        out['add_vias'].append({'x': round(pts[j]['x'],4), 'y': round(pts[j]['y'],4)})
        i = j
    else:
        i = j
json.dump(out, open(sys.argv[2], 'w', encoding='utf-8'), indent=1)
ns = sum(len(t['pts'])-1 for t in out['add_tracks'])
tot = sum(((t['pts'][k+1][0]-t['pts'][k][0])**2+(t['pts'][k+1][1]-t['pts'][k][1])**2)**0.5 for t in out['add_tracks'] for k in range(len(t['pts'])-1))
print('polylines', len(out['add_tracks']), 'segments', ns, 'vias', len(out['add_vias']), 'len %.3f' % tot)
for t in out['add_tracks']:
    print(' ', t['layer'], ' '.join('(%.2f,%.2f)' % (p[0],p[1]) for p in t['pts']))
for v in out['add_vias']:
    print('  VIA (%.3f,%.3f)' % (v['x'], v['y']))
