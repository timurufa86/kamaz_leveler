import sys, math
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
out = open(sys.argv[2], 'w', encoding='utf-8')
def W(s): out.write(s + '\n'); out.flush()
layer_name = sys.argv[3]
X0, Y0, X1, Y1 = [float(a) for a in sys.argv[4:8]]
CELL = float(sys.argv[8])
IGN = set(a.strip() for a in sys.argv[9].split(',') if a.strip())
CLR = 0.15
lay = b.GetLayerID(layer_name)

segs = []   # (x1,y1,x2,y2,halfw)
for t in b.GetTracks():
    if t.GetNetname() in IGN: continue
    s, e = t.GetStart(), t.GetEnd()
    sx, sy, ex, ey = s.x/1e6, s.y/1e6, e.x/1e6, e.y/1e6
    if t.GetClass() == 'PCB_VIA':
        top = t.TopLayer(); bot = t.BottomLayer()
        # through vias span all layers
        segs.append((sx, sy, sx, sy, t.GetWidth(top)/2e6))
    elif t.GetLayer() == lay:
        segs.append((sx, sy, ex, ey, t.GetWidth()/2e6))
rects = []  # pads / fps on this layer
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetNetname() in IGN: continue
        if not p.IsOnLayer(lay) and p.GetAttribute() != pcbnew.PAD_ATTRIB_PTH: continue
        pp = p.GetPosition(); sz = p.GetSize()
        hw, hh = sz.x/2e6, sz.y/2e6
        rects.append((pp.x/1e6-hw, pp.y/1e6-hh, pp.x/1e6+hw, pp.y/1e6+hh, max(hw,hh)))
for fp in b.GetFootprints():
    if fp.GetReference() in IGN: continue
    for p in fp.Pads():
        if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH: continue
print_holes = []
# board edges
import pcbnew as _p

def d_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1
    L = dx*dx + dy*dy
    if L == 0: return math.hypot(px-x1, py-y1)
    t = ((px-x1)*dx + (py-y1)*dy)/L
    t = max(0.0, min(1.0, t))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))

def d_rect(px, py, x0, y0, x1, y1):
    dx = max(x0-px, 0, px-x1); dy = max(y0-py, 0, py-y1)
    return math.hypot(dx, dy)

nx = int(round((X1-X0)/CELL)); ny = int(round((Y1-Y0)/CELL))
W('layer %s  box %.2f,%.2f .. %.2f,%.2f  cell %.3f  (%d x %d)' % (layer_name, X0, Y0, X1, Y1, CELL, nx, ny))
W('ignore nets: %s' % sorted(IGN))
hdr = '      ' + ''.join(str(int((X0+i*CELL)) % 10) for i in range(nx))
W(hdr)
for j in range(ny):
    y = Y0 + (j+0.5)*CELL
    row = []
    for i in range(nx):
        x = X0 + (i+0.5)*CELL
        blocked = False
        for (x1, y1, x2, y2, hw) in segs:
            if d_seg(x, y, x1, y1, x2, y2) < hw + CLR + CELL*0.35: blocked = True; break
        if not blocked:
            for (rx0, ry0, rx1, ry1, hw) in rects:
                if d_rect(x, y, rx0, ry0, rx1, ry1) < CLR + CELL*0.35: blocked = True; break
        row.append('#' if blocked else '.')
    W('%6.2f %s' % (y, ''.join(row)))
out.close()
