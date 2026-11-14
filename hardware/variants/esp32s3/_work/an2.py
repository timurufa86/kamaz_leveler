import io,sys,statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = open('_work/an2.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
TARGET={"/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE",
        "/DISP_MOSI","/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"}
tr=list(b.GetTracks())
segs=[t for t in tr if t.Type()==pcbnew.PCB_TRACE_T]
vias=[t for t in tr if t.Type()==pcbnew.PCB_VIA_T]
oth=[s for s in segs if s.GetNetname() not in TARGET]
tgt=[s for s in segs if s.GetNetname() in TARGET]
ov=[v for v in vias if v.GetNetname() not in TARGET]
tv=[v for v in vias if v.GetNetname() in TARGET]
def st(name,arr):
    L=sorted(MM(s.GetLength()) for s in arr)
    if not L: print(name,"empty"); return
    print(f"{name}: n={len(L)} total={sum(L):.2f} median={statistics.median(L):.3f} "
          f"ge08={sum(1 for x in L if x>=0.8)} ({100*sum(1 for x in L if x>=0.8)/len(L):.1f}%) "
          f"p10={L[len(L)//10]:.3f} p90={L[9*len(L)//10]:.3f}")
st("OTHER",oth); st("TARGET12",tgt)
print("vias other",len(ov),"target",len(tv))
print("total cu", round(sum(MM(s.GetLength()) for s in segs),2))
# if target12 replaced by N segments all >=0.8 with length sum S
n0=len(oth); g0=sum(1 for s in oth if MM(s.GetLength())>=0.8)
print("baseline others: n",n0,"ge08",g0)
for add in (200,250,300,350,400,500):
    tot=n0+add
    # assume all added >=0.8
    need=tot//2+1
    print(f"  if 12nets become {add} segs (all>=0.8): median-seg-count check need {need} ge08, have {g0+add} -> {'OK' if g0+add>=need else 'FAIL'}")
