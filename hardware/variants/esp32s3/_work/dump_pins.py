import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
txt = open('kamaz_leveler.kicad_sym', encoding='utf-8').read().split('\n')
start = None
for k, l in enumerate(txt):
    if l.strip() == '(symbol "ESP32-S3-DevKitC-1_N16R8"':
        start = k
        break
i = start
while not txt[i].strip().startswith('(symbol "ESP32-S3-DevKitC-1_N16R8_0_1"'):
    i += 1
seg = '\n'.join(txt[i:])
jx = seg.find('\n\t\t)\n', 10)
if jx > 0:
    seg = seg[:jx]
pat = re.compile(r'\(pin\s+(\w+)\s+(\w+)\s*\n\s*\(at\s+([\-0-9.]+)\s+([\-0-9.]+)\s+(\d+)\s*\n.*?\(name\s+"([^"]*)"', re.S)
pat2 = re.compile(r'\(number\s+"([^"]*)"')
pins = []
for m in pat.finditer(seg):
    tail = seg[m.end():m.end() + 400]
    n = pat2.search(tail)
    pins.append((n.group(1) if n else '?', m.group(6), float(m.group(3)), float(m.group(4)), int(m.group(5))))
print("pins:", len(pins))
SX, SY = 87.63, 266.70
for num, name, x, y, rot in pins:
    print("pin %2s %-6s sym=(%s,%s) rot=%d sheet=(%.2f,%.2f)" % (num, name, x, y, rot, SX + x, SY - y))
