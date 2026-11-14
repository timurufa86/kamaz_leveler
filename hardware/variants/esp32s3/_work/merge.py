import sys, io, json, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew

src, dst = sys.argv[1], sys.argv[2]

def stats(b):
    ts = [t for t in b.GetTracks() if t.GetClass() == 'PCB_TRACK']
    vs = [t for t in b.GetTracks() if t.GetClass() == 'PCB_VIA']
    L = [ (t.GetStart()-t.GetEnd()).EuclideanNorm()/1e6 for t in ts ]
    L.sort()
    print('  segs %d vias %d total %.1f mm median %.3f mm' % (len(ts), len(vs), sum(L), L[len(L)//2] if L else 0))
    return len(ts)

b = pcbnew.LoadBoard(src)
print('BEFORE'); n_before = stats(b)

# --- collect items ---
trs = []
for t in b.GetTracks():
    if t.GetClass() == 'PCB_TRACK':
        trs.append(t)

def pt(v): return (v.x, v.y)
via_pts = set()
for t in b.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        via_pts.add(pt(t.GetPosition()))

# same-net same-layer incidence at every endpoint (any width) -> junction detector
inc = collections.defaultdict(list)
for i, t in enumerate(trs):
    k_nl = (t.GetNetCode(), t.GetLayer())
    for e in (0, 1):
        p = pt(t.GetStart() if e == 0 else t.GetEnd())
        inc[(k_nl, p)].append(i)

def cross(a, b):
    return a[0]*b[1] - a[1]*b[0]

# segment endpoints
ends = [ (pt(t.GetStart()), pt(t.GetEnd())) for t in trs ]

# mergeable junction: exactly 2 same-net same-layer tracks, no via, and the two
# segments continue in the same direction (collinear, dot > 0)
merge_ok = set()
for (k_nl, p), lst in inc.items():
    if len(lst) != 2: continue
    if p in via_pts: continue
    i, j = lst
    if i == j: continue
    if trs[i].GetWidth() != trs[j].GetWidth(): continue   # keep width breakpoints
    a0, a1 = ends[i]; b0, b1 = ends[j]
    # orient both away from p
    A = a1 if a0 == p else a0
    B = b1 if b0 == p else b0
    v1 = (A[0]-p[0], A[1]-p[1])
    v2 = (B[0]-p[0], B[1]-p[1])
    if v1 == (0,0) or v2 == (0,0): continue
    if cross(v1, v2) != 0: continue
    if v1[0]*v2[0] + v1[1]*v2[1] >= 0: continue   # both vectors point AWAY from p: straight run needs dot < 0
    merge_ok.add((k_nl, p))

# union-find over segments joined at mergeable points
par = list(range(len(trs)))
def find(x):
    while par[x] != x:
        par[x] = par[par[x]]; x = par[x]
    return x
def uni(a, c):
    ra, rc = find(a), find(c)
    if ra != rc: par[ra] = rc
for (k_nl, p) in merge_ok:
    lst = inc[(k_nl, p)]
    uni(lst[0], lst[1])

comp = collections.defaultdict(list)
for i in range(len(trs)): comp[find(i)].append(i)

def fmt_pt(p): return (p[0], p[1])

new_items = []
del_items = []
runs = 0
for root, idxs in comp.items():
    if len(idxs) < 2: continue
    # find the two endpoints of the chain: points inside idxs that are NOT mergeable
    inner_pts = collections.Counter()
    for i in idxs:
        for p in ends[i]: inner_pts[p] += 1
    open_ends = [p for p in inner_pts if (trs[idxs[0]].GetNetCode(), trs[idxs[0]].GetLayer()) and ((trs[idxs[0]].GetNetCode(), trs[idxs[0]].GetLayer()), p) not in merge_ok]
    if len(open_ends) != 2:
        continue   # cycle or ambiguous -> leave untouched
    # sanity: all same net/layer/width
    ncs = set((trs[i].GetNetCode(), trs[i].GetLayer(), trs[i].GetWidth()) for i in idxs)
    if len(ncs) != 1: continue
    nc, lay, w = ncs.pop()
    total = sum((trs[i].GetStart()-trs[i].GetEnd()).EuclideanNorm() for i in idxs)
    d = (pcbnew.VECTOR2I(open_ends[0][0], open_ends[0][1]) - pcbnew.VECTOR2I(open_ends[1][0], open_ends[1][1])).EuclideanNorm()
    if abs(d - total) > 2:      # not a straight run (shouldn't happen)
        continue
    nt = pcbnew.PCB_TRACK(b)
    nt.SetStart(pcbnew.VECTOR2I(*open_ends[0]))
    nt.SetEnd(pcbnew.VECTOR2I(*open_ends[1]))
    nt.SetWidth(w); nt.SetLayer(lay); nt.SetNetCode(nc)
    nt.SetLocked(trs[idxs[0]].IsLocked())
    new_items.append(nt)
    del_items.extend(trs[i] for i in idxs)
    runs += 1

print('runs merged: %d   segments replaced: %d -> %d new' % (runs, len(del_items), len(new_items)))
for t in del_items: b.Remove(t)
for t in new_items: b.Add(t)

# purge any zero-length
zero = [t for t in b.GetTracks() if t.GetClass()=='PCB_TRACK' and t.GetStart()==t.GetEnd()]
print('zero-length after merge:', len(zero))
for t in zero: b.Remove(t)

print('AFTER'); n_after = stats(b)
pcbnew.SaveBoard(dst, b)
print('saved', dst, n_before, '->', n_after)
