# -*- coding: utf-8 -*-
"""Production export for variant A (esp32s3) - mirrors hardware/scripts/export_prod.py."""
import io, os, subprocess, sys
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
KC = r'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
HW = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3'
PCB = os.path.join(HW, 'kamaz_leveler_s3.kicad_pcb')
SCH = os.path.join(HW, 'kamaz_leveler_s3.kicad_sch')
OUT = os.path.join(HW, 'production')
for sub in ('gerber', 'drill'):
    os.makedirs(os.path.join(OUT, sub), exist_ok=True)
LAYERS = 'F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts'
def run(args):
    r = subprocess.run(args, capture_output=True, text=True, encoding='utf-8', errors='replace')
    ok = r.returncode == 0
    print(('OK   ' if ok else 'FAIL ') + ' '.join(a for a in args[2:4]))
    if not ok:
        print('    ', (r.stderr or r.stdout)[-500:])
    return ok
run([KC,'pcb','export','gerbers','-o',os.path.join(OUT,'gerber'),'--layers',LAYERS,'--no-protel-ext',PCB])
run([KC,'pcb','export','drill','-o',os.path.join(OUT,'drill'),'--format','excellon',
     '--drill-origin','absolute','--excellon-units','mm','--excellon-separate-th',
     '--generate-map','--map-format','pdf',PCB])
run([KC,'pcb','export','pos','-o',os.path.join(OUT,'pos.csv'),'--format','csv','--units','mm',
     '--side','both','--use-drill-file-origin',PCB])
run([KC,'pcb','export','step','-o',os.path.join(OUT,'board.step'),PCB])
run([KC,'pcb','export','pdf','-o',os.path.join(OUT,'board.pdf'),'--layers',LAYERS,PCB])
run([KC,'pcb','export','stats','-o',os.path.join(OUT,'board_stats.txt'),PCB])
run([KC,'sch','export','pdf','-o',os.path.join(OUT,'schematic.pdf'),SCH])
run([KC,'sch','export','bom','--fields','Reference,Value,Footprint','--labels','Reference,Value,Footprint','--group-by','Value,Footprint','-o',os.path.join(OUT,'BOM.csv'),SCH])
print('--- files ---')
n=0
for root,dirs,files in os.walk(OUT):
    for f in sorted(files):
        p=os.path.join(root,f); n+=1
        print('%-46s %9d' % (os.path.relpath(p,OUT), os.path.getsize(p)))
print('total files:',n)
