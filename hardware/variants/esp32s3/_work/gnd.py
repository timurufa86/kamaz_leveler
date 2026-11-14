import sys, math, collections
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
def M(v): return v/1e6
def dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1; L2 = dx*dx+dy*dy
    if L2 == 0: return math.hypot(px-x1, py-y1)
    t = max(0.0, min(1.0, ((px-x1)*dx + (py-y1)*dy)/L2))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))
def drect(px, py, x0, y0, x1, y1):
    return math.hypot(max(x0-px, 0, px-x1), max(y0-py, 0, py-y1))
nodes = []   # ('t', layer, x1,y1,x2,y2, hw) | ('v', x,y,r) | ('p', layer, cx,cy,hw,hh) | ('z', poly)
for t in b.GetTracks():
    if t.GetNetname() != 'GND': continue
    s, e = t.GetStart(), t.GetEnd()
    if t.GetClass() == 'PCB_VIA':
        nodes.append(('v', M(s.x), M(s.y), M(t.GetWidth(t.TopLayer()))/2))
    else:
        nodes.append(('t', t.GetLayer(), M(s.x), M(s.y), M(e.x), M(e.y), M(t.GetWidth())/2))
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetNetname() != 'GND': continue
        pp = p.GetPosition(); sz = p.GetSize()
        lay = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        nodes.append(('p', lay, M(pp.x), M(pp.y), M(sz.x)/2, M(sz.y)/2))
zoff = len(nodes)
zinfo = []
for z in b.Zones():
    for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
        try:
            if not z.IsOnLayer(lay): continue
            ps = z.GetFilledPolysList(lay)
        except Exception:
            continue
        if ps is not None and ps.OutlineCount() > 0:
            nodes.append(('z', ps, lay)); zinfo.append((lay, ps.OutlineCount()))
print('GND items: %d copper + %d zone poly-sets (outlines %s)' % (zoff, len(nodes)-zoff, zinfo))
par = list(range(len(nodes)))
def find(a):
    while par[a] != a: par[a] = par[par[a]]; a = par[a]
    return a
def uni(a, c):
    ra, rc = find(a), find(c)
    if ra != rc: par[ra] = rc

def pts_of(n):
    k = n[0]
    if k == 't': return [((n[2]+n[4])/2, (n[3]+n[5])/2), (n[2], n[3]), (n[4], n[5])]
    if k == 'v': return [(n[1], n[2])]
    return [(n[2], n[3])]

def touch(a, c):
    ka, kc = a[0], c[0]
    if ka == 'z' and kc == 'z': return False
    if ka == 'z' or kc == 'z':
        z, o = (a, c) if ka == 'z' else (c, a)
        ps, lay = z[1], z[2]
        if o[0] == 't' and o[1] != lay: return False
        if o[0] == 'p' and o[1] != lay: return False
        for (x, y) in pts_of(o):
            if ps.Contains(pcbnew.VECTOR2I(int(x*1e6), int(y*1e6))): return True
        return False
    if ka == 't' and kc == 't':
        if a[1] != c[1]: return False
        tol = a[6] + c[6]
        return dseg(c[2], c[3], a[2], a[3], a[4], a[5]) <= tol or dseg(c[4], c[5], a[2], a[3], a[4], a[5]) <= tol
    if ka == 't' and kc == 'v':
        return dseg(c[1], c[2], a[2], a[3], a[4], a[5]) <= a[6] + c[3]
    if ka == 'v' and kc == 't':
        return dseg(a[1], a[2], c[2], c[3], c[4], c[5]) <= c[6] + a[3]
    if ka == 't' and kc == 'p':
        if a[1] != c[1]: return False
        return drect(c[2], c[3], 0, 0, 0, 0) is not None and (drect(c[2], c[3], a[2]-c[4], a[3]-c[5], a[2]+c[4], a[3]+c[5]) <= a[6] or drect(c[2], c[3], a[4]-c[4], a[5]-c[5], a[4]+c[4], a[5]+c[5]) <= a[6] or dseg(c[2], c[3], a[2], a[3], a[4], a[5]) <= a[6] + min(c[4], c[5]))
    if ka == 'p' and kc == 't':
        return touch(c, a)
    if ka == 'v' and kc == 'v':
        return math.hypot(a[1]-c[1], a[2]-c[2]) <= a[3] + c[3]
    if ka == 'v' and kc == 'p':
        return drect(a[1], a[2], c[2]-c[4], c[3]-c[5], c[2]+c[4], c[3]+c[5]) <= a[3]
    if ka == 'p' and kc == 'v':
        return touch(c, a)
    if ka == 'p' and kc == 'p':
        return drect(a[2], a[3], c[2]-c[4], c[3]-c[5], c[2]+c[4], c[3]+c[5]) == 0
    return False

n = len(nodes); edges = 0
for i in range(n):
    for j in range(i+1, n):
        if touch(nodes[i], nodes[j]):
            uni(i, j); edges += 1
comp = collections.Counter(find(i) for i in range(n))
sizes = sorted(comp.values(), reverse=True)
print('edges %d ; GND clusters = %d ; sizes %s' % (edges, len(comp), sizes[:8]))
