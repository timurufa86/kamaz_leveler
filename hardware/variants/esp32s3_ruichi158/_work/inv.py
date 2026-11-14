import io, sys, json, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard('kamaz_leveler_ruichi158.kicad_pcb')
fps = list(b.GetFootprints())
pads = []
for f in fps:
    for p in f.Pads():
        pads.append(p)
print('footprints', len(fps), 'pads', len(pads), 'tracks', len(list(b.GetTracks())), 'zones', len(b.Zones()))
nets = collections.Counter()
for p in pads:
    nets[p.GetNetname()] += 1
print('nets', len([n for n in nets if n]))
for n, c in sorted(nets.items(), key=lambda kv: -kv[1]):
    print('  %-24s %d' % (n or '<none>', c))
nc = {}
for c in b.GetDesignSettings().GetNetClasses().NetClasses():
    try:
        nc[c] = b.GetDesignSettings().GetNetClasses().GetNetClass(c)
    except Exception:
        pass
print('netclass names:', list(nc.keys()))
