import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
MM=pcbnew.ToMM
NETS={"/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE",
      "/DISP_MOSI","/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"}
mode=sys.argv[1]; layer=sys.argv[2]; dst=sys.argv[3]
b=pcbnew.LoadBoard('_work/s3.kicad_pcb')
n=0
for t in list(b.GetTracks()):
    if t.GetNetname() not in NETS: continue
    if layer=='B' and (t.Type()!=pcbnew.PCB_TRACE_T or t.GetLayer()!=pcbnew.B_Cu): continue
    if layer=='F' and (t.Type()!=pcbnew.PCB_TRACE_T or t.GetLayer()!=pcbnew.F_Cu): continue
    b.Remove(t); n+=1
print("removed",n)
pcbnew.SaveBoard(dst,b)
