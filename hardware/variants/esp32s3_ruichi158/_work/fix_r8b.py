import io, sys, math, shutil
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr, go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

TOMM = pcbnew.ToMM
ROOT = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158"
MAIN = ROOT + r"\kamaz_leveler_ruichi158.kicad_pcb"

b = pcbnew.LoadBoard(MAIN)

# диагностика via +3V3
print("=== vias возле (-25,9.5) ===")
for t in b.GetTracks():
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        if math.hypot(TOMM(p.x) - (-25.0), TOMM(p.y) - 9.5) < 3:
            print(f"  VIA {t.GetNetname()} ({TOMM(p.x):.2f};{TOMM(p.y):.2f}) w={TOMM(t.GetWidth()):.2f} drill={TOMM(t.GetDrillValue()):.2f}")

# удаляем старую медь /BUT3 и /BUT4
removed = 0
for t in list(b.GetTracks()):
    if t.GetNetname() in ("/BUT3", "/BUT4"):
        b.Remove(t)
        removed += 1
print(f"удалено старой меди /BUT3//BUT4: {removed}")

pcbnew.SaveBoard(MAIN, b)

# модель без старой меди
m = rr.Model(MAIN)
m.build()

def add_net(net):
    log = []
    w = 0.15
    segs, vias = go.route_net(m, net, w / 2.0, log, margin=10.0, via_cost=0.6, use_hub=False)
    print(f"{net}: segs={len(segs)} vias={len(vias)}")
    if not segs and not vias:
        return False
    nc = None
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net:
                nc = p.GetNetCode()
                break
        if nc is not None:
            break
    for (x1, y1, x2, y2, h, L) in segs:
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        t.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        t.SetWidth(int(round(2 * h * 1e6)))
        t.SetLayer(pcbnew.F_Cu if int(L) == 0 else pcbnew.B_Cu)
        t.SetNetCode(nc)
        b.Add(t)
    for (x, y) in vias:
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(rr.VIA_D * 1e6)))
        v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
        v.SetNetCode(nc)
        b.Add(v)
    return True

ok3 = add_net("/BUT3")
ok4 = add_net("/BUT4")
print(f"/BUT3 {ok3}, /BUT4 {ok4}")

pcbnew.SaveBoard(MAIN, b)
print("сохранено после BUT3/BUT4")
