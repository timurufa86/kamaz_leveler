import sys, io, time
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
t0=time.time()
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
print("loaded", round(time.time()-t0,2), "s", flush=True)
nv=0; nt=0
for t in b.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA):
        nv+=1
        if nv==1:
            v=t
    else:
        nt+=1
print("tracks",nt,"vias",nv, flush=True)
cands = ['GetDrillValue','GetDrill','GetViaType','GetWidth','GetPosition','TopLayer','BottomLayer','GetLayer','GetNetname']
for c in cands:
    print(c, hasattr(v,c), flush=True)
print("drill", v.GetDrillValue(), "width", v.GetWidth(), "top", v.TopLayer(), "bot", v.BottomLayer(), flush=True)
