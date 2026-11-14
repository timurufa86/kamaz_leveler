import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")
names = set()
for t in b.GetTracks():
    names.add(t.GetNetname())
print('net names with SCL/INT:', sorted(n for n in names if 'SCL' in n or 'INT' in n or 'DC' in n)[:20])
print('total names', len(names))
print('has I2C_SCLK?', '/I2C_SCLK' in names)
