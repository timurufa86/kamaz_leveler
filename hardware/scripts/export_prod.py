# -*- coding: utf-8 -*-
"""Production export for the Kamaz Leveler board (2 layers, user placement)."""
import io
import os
import subprocess
import sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
KC = r'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
HW = r'C:\my\for cursor\kamaz_leveler\hardware\kicad'
PCB = os.path.join(HW, 'kamaz_leveler_hw.kicad_pcb')
SCH = os.path.join(HW, 'kamaz_leveler_hw.kicad_sch')
OUT = r'C:\my\for cursor\kamaz_leveler\hardware\production'
for sub in ('gerber', 'drill'):
    os.makedirs(os.path.join(OUT, sub), exist_ok=True)

LAYERS2 = 'F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts'


def run(args):
    r = subprocess.run(args, capture_output=True, text=True, encoding='utf-8', errors='replace')
    ok = r.returncode == 0
    print(('OK  ' if ok else 'FAIL') + ' ' + os.path.basename(args[2] if len(args) > 2 else args[0]))
    if not ok:
        print('   ', (r.stderr or r.stdout)[-400:])
    return ok


run([KC, 'pcb', 'export', 'gerbers', '-o', os.path.join(OUT, 'gerber'),
     '--layers', LAYERS2, '--no-protel-ext', PCB])
run([KC, 'pcb', 'export', 'drill', '-o', os.path.join(OUT, 'drill'), '--format', 'excellon',
     '--drill-origin', 'absolute', '--excellon-units', 'mm', '--excellon-separate-th',
     '--generate-map', '--map-format', 'pdf', PCB])
run([KC, 'pcb', 'export', 'pos', '-o', os.path.join(OUT, 'pos.csv'), '--format', 'csv',
     '--units', 'mm', '--side', 'both', '--use-drill-file-origin', PCB])
run([KC, 'pcb', 'export', 'step', '-o', os.path.join(OUT, 'board.step'), PCB])
run([KC, 'pcb', 'export', 'pdf', '-o', os.path.join(OUT, 'board.pdf'), '--layers', LAYERS2, PCB])
run([KC, 'pcb', 'export', 'stats', '-o', os.path.join(OUT, 'board_stats.txt'), PCB])
run([KC, 'sch', 'export', 'pdf', '-o', os.path.join(OUT, 'schematic.pdf'), SCH])
run([KC, 'sch', 'export', 'bom', '-o', os.path.join(OUT, 'BOM.csv'), SCH])
print('\n--- files ---')
for root, dirs, files in os.walk(OUT):
    for f in sorted(files):
        p = os.path.join(root, f)
        print('%-42s %9d' % (os.path.relpath(p, OUT), os.path.getsize(p)))
