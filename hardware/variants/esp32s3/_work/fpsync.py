import sys,math
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
SRC=sys.argv[1]; DST=sys.argv[2]
LIBDIR='kamaz_leveler_hw.pretty'; NAME='ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm'
b=pcbnew.LoadBoard(SRC)
u7=[f for f in b.GetFootprints() if f.GetReference()=='U7'][0]
pos=(u7.GetPosition().x,u7.GetPosition().y); ang=u7.GetOrientationDegrees()
nets={p.GetNumber():p.GetNetname() for p in u7.Pads()}
print("U7 before: pads=%d items=%d pos=(%.3f,%.3f) ang=%.1f fpid=%s"%(
    u7.GetPadCount(),len(list(u7.GraphicalItems())),MM(pos[0]),MM(pos[1]),ang,str(u7.GetFPID().GetLibNickname())+"/"+str(u7.GetFPID().GetLibItemName())))
lib=pcbnew.FootprintLoad(LIBDIR,NAME)
print("lib: pads=%d items=%d"%(lib.GetPadCount(),len(list(lib.GraphicalItems()))))
u7.SetPosition(pcbnew.VECTOR2I(0,0))
keep=[]
for p in list(u7.Pads()):
    u7.Remove(p); keep.append(p)
for g in list(u7.GraphicalItems()):
    u7.Remove(g); keep.append(g)
print("after clear: pads=%d items=%d"%(u7.GetPadCount(),len(list(u7.GraphicalItems()))))
lp=list(lib.Pads()); lg=list(lib.GraphicalItems())
for p in lp:
    lib.Remove(p); u7.Add(p)
for g in lg:
    lib.Remove(g); u7.Add(g)
print("after fill: pads=%d items=%d"%(u7.GetPadCount(),len(list(u7.GraphicalItems()))))
miss=[]
for p in u7.Pads():
    nm=nets.get(p.GetNumber())
    if nm is None: miss.append(p.GetNumber()); continue
    ni=b.FindNet(nm)
    if ni is None: b.Add(pcbnew.NETINFO_ITEM(b,nm)); ni=b.FindNet(nm)
    p.SetNet(ni)
print("pads w/o old net:",miss)
u7.SetPosition(pcbnew.VECTOR2I(pos[0],pos[1]))
u7.SetOrientationDegrees(ang)
b.BuildConnectivity()
pcbnew.SaveBoard(DST,b)
print("saved",DST)
