import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
segs = [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T]
vias = [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]

NETS = ["/DISP_BL", "/DISP_CS", "/DISP_DC", "/DISP_RES", "/DISP_SCLK", "/SPEED_PULSE",
        "/DISP_MOSI", "/INT_MCU", "/VLV_EN", "/PRES_SW", "/I2C_SCL", "/I2C_SDA"]
PAD = {"/DISP_BL": 4, "/DISP_CS": 5, "/DISP_DC": 9, "/DISP_RES": 10, "/DISP_SCLK": 11,
       "/SPEED_PULSE": 12, "/DISP_MOSI": 17, "/INT_MCU": 19, "/VLV_EN": 20,
       "/PRES_SW": 32, "/I2C_SCL": 39, "/I2C_SDA": 40}

print("| цепь | пад | отрезков | переходов | длина, мм |")
print("|---|---|---|---|---|")
tot = 0
for n in NETS:
    s = [x for x in segs if x.GetNetname() == n]
    v = [x for x in vias if x.GetNetname() == n]
    L = sum(TOMM(x.GetLength()) for x in s)
    tot += L
    print(f"| {n} | {PAD[n]} | {len(s)} | {len(v)} | {L:.3f} |")
print(f"\nсумма по 12 сигналам: {tot:.2f} мм")
print(f"вся плата: отрезков {len(segs)}, переходов {len(vias)}, медь {sum(TOMM(t.GetLength()) for t in segs):.2f} мм")
