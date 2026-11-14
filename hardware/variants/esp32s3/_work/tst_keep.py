import sys
try: sys.stdout.reconfigure(encoding='utf-8',errors='replace')
except Exception: pass
import pcbnew
KEEP=set(sys.argv[1].split(','))
LAY={'F':pcbnew.F_Cu,'B':pcbnew.B_Cu}[sys.argv[2]]
b=pcbnew.LoadBoard('_work/s3.kicad_pcb')
n=0
for t in list(b.GetTracks()):
    if t.Type()!=pcbnew.PCB_TRACE_T or t.GetLayer()!=LAY: continue
    if t.GetNetname() in KEEP: continue
    if not t.GetNetname().startswith('/'): continue
    if t.GetNetname() in ("/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE",
                          "/DISP_MOSI","/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"):
        b.Remove(t); n+=1
print("removed",n,"kept nets",sorted(KEEP))
pcbnew.SaveBoard(sys.argv[3],b)
