import sys, math
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr, go
from rr import Model
m = Model(r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\routed.kicad_pcb')
m.build()
for ref, num, L in (('D3', '1', 0), ('C2', '1', 0), ('U11', '2', 0)):
    p = [q for q in m.pads if q['ref'] == ref and q['num'] == num][0]
    print('%s.%s x=%.3f y=%.3f hx=%.3f hy=%.3f r=%.3f ang_deg=%.1f onF=%s onB=%s shape-drill=%.2f' %
          (ref, num, p['x'], p['y'], p['hx'], p['hy'], p['r'], math.degrees(math.atan2(p['s'], p['c'])),
           p['onF'], p['onB'], p['drill']))
# tracks of a net near a point
for net, cx, cy in (('/VALVE_BUB2', -21.5, -12.85), ('/+8V_SENS', 23.4, 8.25), ('/VALVE_BUB4', 12.9, -12.0)):
    print('==', net)
    for L in (0, 1):
        for (x1, y1, x2, y2, h) in m.tracks[L]:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            if math.hypot(mx - cx, my - cy) < 3.0:
                print('   L%d  %.3f,%.3f -> %.3f,%.3f h=%.3f len=%.2f' %
                      (L, x1, y1, x2, y2, h, math.hypot(x2 - x1, y2 - y1)))
