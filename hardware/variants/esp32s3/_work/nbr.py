import sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
X, Y, R = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
only = sys.argv[5] if len(sys.argv) > 5 else None
def M(v): return v/1e6
def near(x, y): return abs(M(x)-X) <= R and abs(M(y)-Y) <= R
print('--- site %.3f,%.3f r=%.2f ---' % (X, Y, R))
for t in b.GetTracks():
    s, e = t.GetStart(), t.GetEnd()
    if not (near(s.x, s.y) or near(e.x, e.y)): continue
    nm = t.GetNetname()
    if only and nm != only: continue
    lay = b.GetLayerName(t.GetLayer())
    if t.GetClass() == 'PCB_VIA':
        print('VIA  %-14s %.3f,%.3f d=%.3f drill=%.3f' % (nm, M(s.x), M(s.y), M(t.GetWidth()), M(t.GetDrillValue())))
    else:
        L = math.hypot(M(e.x)-M(s.x), M(e.y)-M(s.y))
        print('TRK  %-14s %-5s w=%.3f L=%.4f  (%.3f,%.3f)->(%.3f,%.3f)' % (nm, lay, M(t.GetWidth()), L, M(s.x), M(s.y), M(e.x), M(e.y)))
for f in b.GetFootprints():
    for p in f.Pads():
        pp = p.GetPosition()
        if near(pp.x, pp.y):
            print('PAD  %-14s %-4s %s %-8s %.3f,%.3f size %.3f/%.3f' % (p.GetNetname(), p.GetNumber(), f.GetReference(), p.GetLayerName(), M(pp.x), M(pp.y), M(p.GetSize().x), M(p.GetSize().y)))
