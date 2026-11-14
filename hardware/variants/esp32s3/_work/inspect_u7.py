import io, sys, re, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

src = open('kamaz_leveler_s3.kicad_sch', encoding='utf-8').read()
lines = src.split('\n')

SX, SY = 87.63, 266.70
# pin sheet coords
pins = {}
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
for num, name, x, y in order:
    pins[num] = (name, round(SX + x, 2), round(SY - y, 2))
bypos = {}
for num, (name, x, y) in pins.items():
    bypos.setdefault((x, y), []).append(num)

def scan(kind, pattern):
    out = []
    for m in re.finditer(pattern, src):
        blk = src[m.start():m.start() + 600]
        at = re.search(r'\(at\s+([\-0-9.]+)\s+([\-0-9.]+)', blk)
        if not at:
            continue
        out.append((kind, m.group(1), float(at.group(1)), float(at.group(2))))
    return out

items = []
items += scan('label', r'\(label\s+"([^"]+)"')
items += scan('glabel', r'\(global_label\s+"([^"]+)"')
items += scan('hlabel', r'\(hierarchical_label\s+"([^"]+)"')

ncs = []
for m in re.finditer(r'\(no_connect\s*\n\s*\(at\s+([\-0-9.]+)\s+([\-0-9.]+)', src):
    ncs.append((float(m.group(1)), float(m.group(2))))

wires = []
for m in re.finditer(r'\(wire\s*\n\s*\(pts\s*\n\s*\(xy\s+([\-0-9.]+)\s+([\-0-9.]+)\)\s*\n\s*\(xy\s+([\-0-9.]+)\s+([\-0-9.]+)\)', src):
    wires.append(tuple(round(float(g), 2) for g in m.groups()))

print("total labels/glabels/hlabels:", len(items), " no_connects:", len(ncs), " wires:", len(wires))
print("\n=== U7 pin  1..22 (left, x=72.39) : attached items ===")
for num in list(range(1, 23)) + list(range(23, 45)):
    name, x, y = pins[num]
    hits = [(k, n) for k, n, lx, ly in items if abs(lx - x) < 0.01 and abs(ly - y) < 0.01]
    nc = any(abs(nx - x) < 0.01 and abs(ny - y) < 0.01 for nx, ny in ncs)
    wh = [w for w in wires if any(abs(w[i] - x) < 0.01 and abs(w[i+1] - y) < 0.01 for i in (0, 2))]
    print("pin %2d %-5s (%6.2f,%6.2f) labels=%s nc=%s wires=%d %s" % (
        num, name, x, y, hits, nc, len(wh), wh if wh else ''))
json.dump({'pins': pins, 'ncs': ncs, 'wires': wires}, open('_work/sch_geom.json', 'w'), indent=1)
