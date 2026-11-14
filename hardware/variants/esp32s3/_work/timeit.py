import sys, time
f = open(sys.argv[2], 'w', encoding='utf-8')
def W(s):
    f.write('%.2f %s\n' % (time.perf_counter()-T0, s)); f.flush()
T0 = time.perf_counter()
import pcbnew
W('import done')
b = pcbnew.LoadBoard(sys.argv[1])
W('loaded')
tracks = list(b.GetTracks())
W('tracks listed %d' % len(tracks))
a = time.perf_counter()
n = 0
for t in tracks:
    t.GetStart(); t.GetEnd(); t.GetNetname(); n += 1
W('loop1 4 calls x%d in %.2fs' % (n, time.perf_counter()-a))
a = time.perf_counter()
n = 0
for t in tracks:
    b.GetLayerName(t.GetLayer()); n += 1
W('loop2 GetLayerName x%d in %.2fs' % (n, time.perf_counter()-a))
a = time.perf_counter()
fps = list(b.GetFootprints())
n = 0
for fp in fps:
    for p in fp.Pads():
        p.GetPosition(); p.GetNetname(); n += 1
W('loop3 pads x%d in %.2fs' % (n, time.perf_counter()-a))
a = time.perf_counter()
n = 0
for t in tracks:
    if t.GetClass() == 'PCB_VIA': n += 1
W('loop4 class x%d in %.2fs' % (len(tracks), time.perf_counter()-a))
f.close()
