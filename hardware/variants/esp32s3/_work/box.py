import sys, math
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
X0, Y0, X1, Y1 = [float(a) for a in sys.argv[3:7]]
out = open(sys.argv[2], 'w', encoding='utf-8')
def W(s): out.write(s + '\n'); out.flush()
def M(v): return v/1e6
def inbox(x, y): return X0-0.01 <= x <= X1+0.01 and Y0-0.01 <= y <= Y1+0.01
for t in b.GetTracks():
    s, e = t.GetStart(), t.GetEnd()
    sx, sy, ex, ey = M(s.x), M(s.y), M(e.x), M(e.y)
    if not (inbox(sx, sy) or inbox(ex, ey)): continue
    if t.GetClass() == 'PCB_VIA':
        W('VIA %-14s at %.3f,%.3f d=%.3f' % (t.GetNetname(), sx, sy, M(t.GetWidth(t.TopLayer()))))
    else:
        W('TRK %-14s %-5s w=%.3f (%.3f,%.3f)->(%.3f,%.3f)' % (t.GetNetname(), b.GetLayerName(t.GetLayer()), M(t.GetWidth()), sx, sy, ex, ey))
for fp in b.GetFootprints():
    for p in fp.Pads():
        pp = p.GetPosition()
        if inbox(M(pp.x), M(pp.y)):
            W('PAD %-14s %-4s %-4s at %.3f,%.3f sz %.2f/%.2f' % (p.GetNetname(), p.GetNumber(), fp.GetReference(), M(pp.x), M(pp.y), M(p.GetSize().x), M(p.GetSize().y)))
out.close()
