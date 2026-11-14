import sys, json
pts = json.load(open(sys.argv[1], encoding='utf-8'))
net = sys.argv[2]; w = float(sys.argv[3]); out = sys.argv[4]
d = float(sys.argv[5]) if len(sys.argv) > 5 else 0.45
drill = float(sys.argv[6]) if len(sys.argv) > 6 else 0.2
# drop collinear intermediate points (keeps segments long)
keep = [pts[0]]
for i in range(1, len(pts)-1):
    a, b2, c = keep[-1], pts[i], pts[i+1]
    if b2['l'] != a['l'] or b2['l'] != c['l']:
        keep.append(b2); continue
    cr = (b2['x']-a['x'])*(c['y']-b2['y']) - (b2['y']-a['y'])*(c['x']-b2['x'])
    if abs(cr) > 1e-9: keep.append(b2)
keep.append(pts[-1])
tracks, vias = [], []
run = [keep[0]]
for p in keep[1:]:
    if p['via']:
        if len(run) > 1:
            for k in range(len(run)-1):
                tracks.append([net, run[k]['l'], w, run[k]['x'], run[k]['y'], run[k+1]['x'], run[k+1]['y']])
        vias.append([net, p['x'], p['y'], d, drill])
        run = [p]
    elif p['l'] != run[-1]['l']:
        # layer change without explicit via flag -> place a via here
        vias.append([net, run[-1]['x'], run[-1]['y'], d, drill])
        if len(run) > 1:
            for k in range(len(run)-1):
                tracks.append([net, run[k]['l'], w, run[k]['x'], run[k]['y'], run[k+1]['x'], run[k+1]['y']])
        run = [p]
    else:
        run.append(p)
if len(run) > 1:
    for k in range(len(run)-1):
        tracks.append([net, run[k]['l'], w, run[k]['x'], run[k]['y'], run[k+1]['x'], run[k+1]['y']])
json.dump({'add_tracks': tracks, 'add_vias': vias}, open(out, 'w', encoding='utf-8'), indent=0)
print('points %d -> %d kept; tracks %d vias %d' % (len(pts), len(keep), len(tracks), len(vias)))
for t in tracks: print('  TRK', t)
for v in vias: print('  VIA', v)
