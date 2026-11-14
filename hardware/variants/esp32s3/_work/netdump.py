import sys, math
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
out = open(sys.argv[2], 'w', encoding='utf-8')
def W(s): out.write(s + '\n'); out.flush()
def M(v): return v/1e6
want = set(a.strip() for a in sys.argv[3:] if a.strip())
for nm in sorted(want):
    W('=== NET %s ===' % nm)
    for t in b.GetTracks():
        if t.GetNetname() != nm: continue
        s, e = t.GetStart(), t.GetEnd()
        if t.GetClass() == 'PCB_VIA':
            W('  VIA %-5s at %.3f,%.3f d=%.3f drill=%.3f' % (b.GetLayerName(t.TopLayer()), M(s.x), M(s.y), M(t.GetWidth(t.TopLayer())), M(t.GetDrillValue())))
        else:
            L = math.hypot(M(e.x)-M(s.x), M(e.y)-M(s.y))
            W('  TRK %-5s w=%.3f L=%.4f (%.3f,%.3f)->(%.3f,%.3f)' % (b.GetLayerName(t.GetLayer()), M(t.GetWidth()), L, M(s.x), M(s.y), M(e.x), M(e.y)))
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == nm:
                pp = p.GetPosition()
                W('  PAD %-5s %-4s %-4s at %.3f,%.3f sz %.3f/%.3f drill %.3f' % (fp.GetReference(), p.GetNumber(), b.GetLayerName(p.GetLayer()), M(pp.x), M(pp.y), M(p.GetSize().x), M(p.GetSize().y), M(p.GetDrillSize().x)))
out.close()
