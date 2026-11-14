import sys, io, math
import pcbnew

board = sys.argv[1]; out = sys.argv[2]
f = open(out, 'w', encoding='utf-8')
def W(s):
    f.write(s + '\n'); f.flush()

b = pcbnew.LoadBoard(board)
W('footprints %d' % len(list(b.GetFootprints())))
tracks = list(b.GetTracks())
W('items %d' % len(tracks))
def M(v): return v/1e6

SITES = [('INT_dang', 41.5, 42.25, 3.0), ('INT_R37', 59.17, 40.09, 2.0),
         ('SCLK_g1', 60.9, 39.8, 2.0), ('SCLK_g2', 66.8, 50.25, 2.0),
         ('5V_a', 89.3, 41.2, 1.6), ('5V_b', 90.6, 41.1, 1.6),
         ('PRES', 80.4, 17.1, 1.6), ('BL', 71.1, 40.6, 1.6)]
for name, X, Y, R in SITES:
    W('')
    W('=== %s  (%.3f,%.3f) r=%.1f ===' % (name, X, Y, R))
    for t in tracks:
        s, e = t.GetStart(), t.GetEnd()
        if not (abs(M(s.x)-X) <= R and abs(M(s.y)-Y) <= R) and not (abs(M(e.x)-X) <= R and abs(M(e.y)-Y) <= R):
            continue
        nm = t.GetNetname(); lay = b.GetLayerName(t.GetLayer())
        if t.GetClass() == 'PCB_VIA':
            W('  VIA  %-14s %-5s at %.3f,%.3f d=%.3f drill=%.3f' % (nm, lay, M(s.x), M(s.y), M(t.GetWidth(t.TopLayer())), M(t.GetDrillValue())))
        else:
            L = math.hypot(M(e.x)-M(s.x), M(e.y)-M(s.y))
            W('  TRK  %-14s %-5s w=%.3f L=%.4f (%.3f,%.3f)->(%.3f,%.3f)' % (nm, lay, M(t.GetWidth()), L, M(s.x), M(s.y), M(e.x), M(e.y)))
    for fp in b.GetFootprints():
        for p in fp.Pads():
            pp = p.GetPosition()
            if abs(M(pp.x)-X) <= R and abs(M(pp.y)-Y) <= R:
                W('  PAD  %-14s %-4s %-5s %-8s %.3f,%.3f sz %.3f/%.3f' % (p.GetNetname(), p.GetNumber(), fp.GetReference(), b.GetLayerName(p.GetLayer()), M(pp.x), M(pp.y), M(p.GetSize().x), M(p.GetSize().y)))
W('')
W('=== nets of interest: item counts + bbox ===')
import collections
cnt = collections.Counter(); bb = {}
for t in tracks:
    nm = t.GetNetname()
    cnt[(nm, b.GetLayerName(t.GetLayer()))] += 1
    s, e = t.GetStart(), t.GetEnd()
    for v in (s, e):
        k = (nm, b.GetLayerName(t.GetLayer()))
        if k not in bb: bb[k] = [M(v.x), M(v.y), M(v.x), M(v.y)]
        r = bb[k]; r[0]=min(r[0],M(v.x)); r[1]=min(r[1],M(v.y)); r[2]=max(r[2],M(v.x)); r[3]=max(r[3],M(v.y))
for k in sorted(cnt):
    if any(x in k[0] for x in ('INT_MCU','DISP_SCLK','DISP_BL','PRES_SW','DISP_RES','DISP_CS','DISP_MOSI','VLV_EN','I2C_SDA')):
        W('  %-16s %-5s n=%-5d bbox %.2f,%.2f - %.2f,%.2f' % (k[0], k[1], cnt[k], *bb[k]))
f.close()
