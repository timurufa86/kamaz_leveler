import io, sys, math, shutil
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"

b = pcbnew.LoadBoard(MAIN)


def point_in_poly(x, y, pts):
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def in_zone(x, y, layer, net):
    for z in b.Zones():
        if z.GetLayer() != layer or z.GetNetname() != net:
            continue
        pl = z.GetFilledPolysList(layer)
        for i in range(pl.OutlineCount()):
            out = pl.Outline(i)
            pts = [(TOMM(out.CPoint(j).x), TOMM(out.CPoint(j).y)) for j in range(out.PointCount())]
            if len(pts) >= 3 and point_in_poly(x, y, pts):
                return True
    return False


def netcode(name):
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == name:
                return p.GetNetCode()
    return None


def add_track(net, x1, y1, x2, y2, layer, w=0.15):
    nc = netcode(net)
    if nc is None:
        print("нет сети", net)
        return
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
    t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
    t.SetWidth(int(round(w * 1e6)))
    t.SetLayer(layer)
    t.SetNetCode(nc)
    b.Add(t)


def add_via(net, x, y):
    nc = netcode(net)
    if nc is None:
        print("нет сети", net)
        return
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
    v.SetWidth(int(round(0.45 * 1e6)))
    v.SetDrill(int(round(0.2 * 1e6)))
    v.SetNetCode(nc)
    b.Add(v)


def foreign_near(x, y, net, r=0.6):
    for t in b.GetTracks():
        if t.GetNetname() == net:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            p = t.GetPosition()
            if math.hypot(TOMM(p.x)-x, TOMM(p.y)-y) < r:
                return True
        else:
            s, e = t.GetStart(), t.GetEnd()
            x1,y1,x2,y2 = TOMM(s.x),TOMM(s.y),TOMM(e.x),TOMM(e.y)
            dx,dy=x2-x1,y2-y1
            L2=dx*dx+dy*dy
            tt = max(0,min(1,((x-x1)*dx+(y-y1)*dy)/L2)) if L2>0 else 0
            if math.hypot(x-(x1+tt*dx), y-(y1+tt*dy)) < r:
                return True
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net:
                continue
            pos=p.GetPosition()
            if math.hypot(TOMM(pos.x)-x, TOMM(pos.y)-y) < r:
                return True
    return False


# 1. U7.23 GND
add_track("GND", 11.43, 40.71, 13.30, 40.71, pcbnew.F_Cu)
add_track("GND", 13.30, 40.71, 13.30, 34.00, pcbnew.F_Cu)
print("U7.23 GND отвод добавлен")

# 2. +3V3 via
via_ok = None
for xx in [i*0.5 for i in range(int(-26.0/0.5), int(-17.5/0.5)+1)]:
    for yy in [i*0.5 for i in range(int(1.5/0.5), int(12.5/0.5)+1)]:
        if in_zone(xx, yy, pcbnew.F_Cu, "+3V3") and in_zone(xx, yy, pcbnew.B_Cu, "+3V3"):
            if not foreign_near(xx, yy, "+3V3", 0.5):
                via_ok = (xx, yy)
                break
    if via_ok:
        break
if via_ok:
    add_via("+3V3", *via_ok)
    print("+3V3 via в", via_ok)
else:
    print("+3V3 via НЕ найдена")

# 3. +12V мост
bridge = None
for yy in [i*0.25 for i in range(int(-13.0/0.25), int(-6.0/0.25)+1)]:
    x0, x1 = -16.0, -12.9
    if in_zone(x0, yy, pcbnew.B_Cu, "+12V") and in_zone(x1, yy, pcbnew.B_Cu, "+12V"):
        ok = True
        for xx in [i*0.25 for i in range(int(x0/0.25), int(x1/0.25)+1)]:
            if foreign_near(xx, yy, "+12V", 0.3):
                ok = False
                break
        if ok:
            bridge = (yy, x0, x1)
            break
if bridge:
    yy, x0, x1 = bridge
    add_track("+12V", x0, yy, x1, yy, pcbnew.B_Cu, 0.6)
    print("+12V мост на y=", yy)
else:
    print("+12V мост не найден")

pcbnew.SaveBoard(MAIN, b)
print("сохранено")
