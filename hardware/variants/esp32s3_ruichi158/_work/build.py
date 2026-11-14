import sys, json, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0,'_work')
import pcbnew
from lib import local_box, rot_pt

PCB = "kamaz_leveler_ruichi158.kicad_pcb"
P = json.load(open('_work/placements.json'))
b = pcbnew.LoadBoard(PCB)

# ---- 1..3 done by _work/strip.py (text surgery)
print('footprints on load:', len(list(b.GetFootprints())), 'tracks:', len(b.GetTracks()), 'zones:', len(b.Zones()))
FPS = {fp.GetReference(): fp for fp in b.GetFootprints()}
print('edge items on load:', sum(1 for d in b.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts))

# ---- 4. new outline
W = pcbnew.FromMM(0.05)
def seg(x1,y1,x2,y2):
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
    s.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x2), pcbnew.FromMM(y2)))
    s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(W); b.Add(s)
def arc(x1,y1,xm,ym,x2,y2):
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_ARC)
    s.SetArcGeometry(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)),
                     pcbnew.VECTOR2I(pcbnew.FromMM(xm), pcbnew.FromMM(ym)),
                     pcbnew.VECTOR2I(pcbnew.FromMM(x2), pcbnew.FromMM(y2)))
    s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(W); b.Add(s)

S = [(-27,-42.75, 27,-42.75), (30,-39.75, 30,-32.75), (27,42.75, -27,42.75), (-30,39.75, -30,32.75),
     (-27,32.75, -30,32.75), (-27,25.25, -30,25.25), (27,25.25, 30,25.25), (27,32.75, 30,32.75),
     (27,-25.25, 30,-25.25), (27,-32.75, 30,-32.75), (-27,-25.25, -30,-25.25), (-27,-32.75, -30,-32.75),
     (-30,-32.75, -30,-39.75), (-30,25.25, -30,-25.25), (30,32.75, 30,39.75), (30,-25.25, 30,25.25)]
C = 3.0
k = C - C/math.sqrt(2)          # 0.8787 sagitta
A = [(-27,42.75, -27-2.1213203,39.75+2.1213203, -30,39.75),
     (30,39.75, 27+2.1213203,39.75+2.1213203, 27,42.75),
     (27,-42.75, 27+2.1213203,-39.75-2.1213203, 30,-39.75),
     (-30,-39.75, -27-2.1213203,-39.75-2.1213203, -27,-42.75),
     (-27,25.25, -23.25,29, -27,32.75),
     (27,32.75, 23.25,29, 27,25.25),
     (-27,-32.75, -23.25,-29, -27,-25.25),
     (27,-25.25, 23.25,-29, 27,-32.75)]
for s in S: seg(*s)
for a in A: arc(*a)
print('outline: segments', len(S), 'arcs', len(A))

# ---- 5. apply placement
for ref,(cx,cy,deg,side) in P.items():
    fp = FPS.get(ref)
    if fp is None: print('missing', ref); continue
    b0 = local_box(ref)
    lcx, lcy = (b0[0]+b0[2])/2, (b0[1]+b0[3])/2
    rx, ry = rot_pt(lcx, lcy, deg)
    fp.SetOrientationDegrees(deg)
    fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(cx-rx), pcbnew.FromMM(cy-ry)))

b.BuildListOfNets()
pcbnew.SaveBoard(PCB, b)
print('saved. footprints:', len(list(b.GetFootprints())), 'tracks:', len(list(b.GetTracks())), 'zones:', len(list(b.Zones())))
