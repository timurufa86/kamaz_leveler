import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew

spec = json.load(open(sys.argv[1], encoding='utf-8'))
BRD = spec.get('board', r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
b = pcbnew.LoadBoard(BRD)
netmap = {}
for t in b.GetTracks():
    nm = t.GetNetname()
    netmap.setdefault(nm, t.GetNetCode())
for fp in b.GetFootprints():
    for pd in fp.Pads():
        nm = pd.GetNetname()
        if nm and nm not in netmap:
            netmap[nm] = pd.GetNetCode()
for z in b.Zones():
    nm = z.GetNetname()
    if nm and nm not in netmap:
        netmap[nm] = z.GetNetCode()

L = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu, 'In1.Cu': pcbnew.In1_Cu}

def mm(v): return int(round(v * 1e6))

# deletions
kill = set(spec.get('delete_uuids', []))
filters = spec.get('delete_filter', [])
if kill or filters:
    removed_u = 0; removed_f = 0
    for t in list(b.GetTracks()):
        try:
            u = t.m_Uuid.AsString()
        except Exception:
            u = ''
        if u in kill:
            b.Remove(t); removed_u += 1; continue
        isvia = isinstance(t, pcbnew.PCB_VIA)
        for f in filters:
            if t.GetNetname() != f['net']: continue
            if f.get('layer'):
                if isvia: continue
                if b.GetLayerName(t.GetLayer()) != f['layer']: continue
            s, e = t.GetStart(), t.GetEnd()
            x1, y1, x2, y2 = s.x/1e6, s.y/1e6, e.x/1e6, e.y/1e6
            bx = f['box']
            if bx[0] <= x1 <= bx[2] and bx[1] <= y1 <= bx[3] and bx[0] <= x2 <= bx[2] and bx[1] <= y2 <= bx[3]:
                b.Remove(t); removed_f += 1
                break
    print('deleted by uuid %d / %d, by filter %d' % (removed_u, len(kill), removed_f))

# add tracks
for tr in spec.get('add_tracks', []):
    net = tr['net']; layer = L[tr['layer']]; w = tr.get('width', 0.15)
    pts = tr['pts']
    for k in range(len(pts) - 1):
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(mm(pts[k][0]), mm(pts[k][1])))
        s.SetEnd(pcbnew.VECTOR2I(mm(pts[k+1][0]), mm(pts[k+1][1])))
        s.SetWidth(mm(w))
        s.SetLayer(layer)
        s.SetNetCode(netmap[net])
        b.Add(s)
for v in spec.get('add_vias', []):
    vi = pcbnew.PCB_VIA(b)
    vi.SetPosition(pcbnew.VECTOR2I(mm(v['x']), mm(v['y'])))
    vi.SetWidth(mm(v.get('dia', 0.45)))
    vi.SetDrill(mm(v.get('drill', 0.2)))
    vi.SetViaType(pcbnew.VIATYPE_THROUGH)
    vi.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    vi.SetNetCode(netmap[v['net']])
    b.Add(vi)

# also allow width changes: list of {uuid, width}
for wc in spec.get('set_width', []):
    for t in b.GetTracks():
        try:
            if t.m_Uuid.AsString() == wc['uuid']:
                if isinstance(t, pcbnew.PCB_VIA):
                    t.SetWidth(mm(wc['width']))
                else:
                    t.SetWidth(mm(wc['width']))
        except Exception:
            pass

# net reassign not supported

if spec.get('refill', True):
    filler = pcbnew.ZONE_FILLER(b)
    filler.Fill(b.Zones())
    print('zones refilled')

if spec.get('save', True):
    b.Save(BRD)
    print('saved', BRD)
