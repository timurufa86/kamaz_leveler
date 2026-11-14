import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
src = open('kamaz_leveler_s3.kicad_sch', encoding='utf-8').read()
wires = []
for m in re.finditer(r'\(wire\s*\n\s*\(pts\s*\n\s*\(xy\s+([\-0-9.]+)\s+([\-0-9.]+)\s*\n\s*\)\s*\n\s*\(xy\s+([\-0-9.]+)\s+([\-0-9.]+)', src):
    wires.append(tuple(round(float(g), 2) for g in m.groups()))
print("wires parsed:", len(wires))
SX, SY = 87.63, 266.70
order = [(1,'3V3',-15.24,26.67),(2,'3V3',-15.24,24.13),(3,'RST',-15.24,21.59),(4,'IO4',-15.24,19.05),
(5,'IO5',-15.24,16.51),(6,'IO6',-15.24,13.97),(7,'IO7',-15.24,11.43),(8,'IO15',-15.24,8.89),
(9,'IO16',-15.24,6.35),(10,'IO17',-15.24,3.81),(11,'IO18',-15.24,1.27),(12,'IO8',-15.24,-1.27),
(13,'IO3',-15.24,-3.81),(14,'IO46',-15.24,-6.35),(15,'IO9',-15.24,-8.89),(16,'IO10',-15.24,-11.43),
(17,'IO11',-15.24,-13.97),(18,'IO12',-15.24,-16.51),(19,'IO13',-15.24,-19.05),(20,'IO14',-15.24,-21.59),
(21,'5V',-15.24,-24.13),(22,'GND',-15.24,-26.67),
(23,'GND',15.24,26.67),(24,'IO43',15.24,24.13),(25,'IO44',15.24,21.59),(26,'IO1',15.24,19.05),
(27,'IO2',15.24,16.51),(28,'IO42',15.24,13.97),(29,'IO41',15.24,11.43),(30,'IO40',15.24,8.89),
(31,'IO39',15.24,6.35),(32,'IO38',15.24,3.81),(33,'IO37',15.24,1.27),(34,'IO36',15.24,-1.27),
(35,'IO35',15.24,-3.81),(36,'IO0',15.24,-6.35),(37,'IO45',15.24,-8.89),(38,'IO48',15.24,-11.43),
(39,'IO47',15.24,-13.97),(40,'IO21',15.24,-16.51),(41,'IO20',15.24,-19.05),(42,'IO19',15.24,-21.59),
(43,'GND',15.24,-24.13),(44,'GND',15.24,-26.67)]
affected = [4,5,6,7,8,9,10,11,12,15,16,17,18,19,20,32,39,40]
for num, name, x, y in order:
    px, py = round(SX + x, 2), round(SY - y, 2)
    wh = [w for w in wires if (abs(w[0]-px)<0.01 and abs(w[1]-py)<0.01) or (abs(w[2]-px)<0.01 and abs(w[3]-py)<0.01)]
    # also wires passing through the point (collinear)
    through = []
    for w in wires:
        x1,y1,x2,y2 = w
        if abs(x1-px)<0.01 and abs(x2-px)<0.01 and min(y1,y2)-0.01 <= py <= max(y1,y2)+0.01: through.append(w)
        elif abs(y1-py)<0.01 and abs(y2-py)<0.01 and min(x1,x2)-0.01 <= px <= max(x1,x2)+0.01: through.append(w)
    mark = 'AFFECTED' if num in affected else ''
    if wh or through:
        print("pin %2d %-5s (%6.2f,%6.2f) endpoint_wires=%s through=%s %s" % (num,name,px,py,wh,through,mark))
print("--- done (only pins with wires listed) ---")
