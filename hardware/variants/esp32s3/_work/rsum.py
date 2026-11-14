import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
pts = json.load(open(sys.argv[1], encoding='utf-8'))
# collapse into segments: consecutive points with same direction and layer
segs = []
i = 0
n = len(pts)
tot = 0.0
while i < n:
    p = pts[i]
    if p.get('via') and i > 0:
        segs.append(('VIA', p['l'], p['x'], p['y']))
        i += 1
        continue
    # find run
    j = i + 1
    while j < n and not pts[j].get('via') and pts[j]['l'] == p['l']:
        # direction check
        dx1 = pts[j]['x'] - pts[j-1]['x']; dy1 = pts[j]['y'] - pts[j-1]['y']
        if j > i+1:
            dx0 = pts[j-1]['x'] - pts[j-2]['x']; dy0 = pts[j-1]['y'] - pts[j-2]['y']
            if abs(dx1*dy0 - dy1*dx0) > 1e-9: break
        j += 1
    segs.append(('SEG', p['l'], p['x'], p['y'], pts[j-1]['x'], pts[j-1]['y']))
    i = j
for s in segs:
    if s[0] == 'VIA':
        print('VIA        %s (%7.3f,%7.3f)' % (s[1], s[2], s[3]))
    else:
        L = ((s[4]-s[2])**2 + (s[5]-s[3])**2)**0.5
        tot += L
        print('SEG %-5s (%7.3f,%7.3f)->(%7.3f,%7.3f)  len %.3f' % (s[1], s[2], s[3], s[4], s[5], L))
print('total track length %.3f mm, %d segments, %d vias' % (tot, sum(1 for s in segs if s[0]=='SEG'), sum(1 for s in segs if s[0]=='VIA')))
