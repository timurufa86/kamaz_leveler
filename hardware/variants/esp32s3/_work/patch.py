import sys, io, json, math
import pcbnew

spec = json.load(open(sys.argv[1], encoding='utf-8'))
path = spec['board']
b = pcbnew.LoadBoard(path)
E = 1e-6
def same(a, c): return abs(a[0]-c[0]) < E and abs(a[1]-c[1]) < E

def netcode(name):
    n = b.FindNet(name)
    if n is None or n.GetNetCode() <= 0:
        n = pcbnew.NETINFO_ITEM(b, name); b.Add(n)
    return n.GetNetCode()

lid = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}

print('--- deletions ---')
alltracks = list(b.GetTracks())
pending = []
for d in spec.get('del_tracks', []):
    nm, lay, x1, y1, x2, y2 = d
    hit = None
    for t in alltracks:
        if t.GetClass() != 'PCB_TRACK': continue
        if t.GetNetname() != nm or t.GetLayer() != lid[lay]: continue
        s, e = t.GetStart(), t.GetEnd()
        a = (s.x/1e6, s.y/1e6); c = (e.x/1e6, e.y/1e6)
        if (same(a, (x1,y1)) and same(c, (x2,y2))) or (same(a, (x2,y2)) and same(c, (x1,y1))):
            hit = t; break
    if hit is None:
        print('  !! NOT FOUND %s %s (%s,%s)-(%s,%s)' % (nm, lay, x1,y1,x2,y2)); sys.exit(4)
    print('  del %s %s (%.3f,%.3f)->(%.3f,%.3f)' % (nm, lay, x1,y1,x2,y2))
    pending.append(hit)
for t in pending:
    b.Remove(t)

print('--- additions ---')
for a in spec.get('add_tracks', []):
    nm, lay, w, x1, y1, x2, y2 = a
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(int(round(x1*1e6)), int(round(y1*1e6))))
    t.SetEnd(pcbnew.VECTOR2I(int(round(x2*1e6)), int(round(y2*1e6))))
    t.SetWidth(int(round(w*1e6))); t.SetLayer(lid[lay]); t.SetNetCode(netcode(nm))
    b.Add(t)
    print('  add %s %s w=%.3f (%.3f,%.3f)->(%.3f,%.3f)' % (nm, lay, w, x1,y1,x2,y2))
for v in spec.get('add_vias', []):
    nm, x, y, d, drill = v
    vi = pcbnew.PCB_VIA(b)
    vi.SetPosition(pcbnew.VECTOR2I(int(round(x*1e6)), int(round(y*1e6))))
    vi.SetWidth(int(round(d*1e6))); vi.SetDrill(int(round(drill*1e6)))
    vi.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); vi.SetNetCode(netcode(nm))
    b.Add(vi)
    print('  via %s at (%.3f,%.3f) d=%.3f/%.3f' % (nm, x, y, d, drill))

if spec.get('refill'):
    zs = b.Zones()
    filler = pcbnew.ZONE_FILLER(b)
    filler.Fill(zs)
    print('  refilled %d zones' % len(list(zs)))

if spec.get('out'):
    pcbnew.SaveBoard(spec['out'], b)
    print('saved', spec['out'])
else:
    print('dry run, nothing saved')
