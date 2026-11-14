import io, sys, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
WORK = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work'
sys.path.insert(0, WORK)
import numpy as np
import rr
from rr import Model, ix, iy, cx, cy

m = Model(r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb')
m.build()
# manually commit a fake foreign track identical to a DISP_SCLK segment and check the field
x1, y1, x2, y2, h = 3.75, 34.45, 14.05, 34.45, 0.125
m.add_copper([(x1, y1, x2, y2, h, 0)], [])
for x in (4.0, 8.0, 14.0, 14.5):
    i, j = ix(x), iy(34.45)
    print('x=%.2f y=34.45 penF=%.3f blocked(F,0.125)=%s' % (x, m.pen[0][j, i], m.pen[0][j, i] > -(rr.CLR + h)))
pens = m.pen_for_net('/I2C_SCL')
for x in (4.0, 8.0, 14.0):
    i, j = ix(x), iy(34.45)
    print('pens x=%.2f penF=%.3f' % (x, pens[0][j, i]))
print('seg_clear across it:', m.seg_clear(10.0, 32.0, 10.0, 36.0, 0.125, 0, '/I2C_SCL'))
print('seg_clear far away:', m.seg_clear(3.0, 20.0, 3.0, 24.0, 0.125, 0, '/I2C_SCL'))
