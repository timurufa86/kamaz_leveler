import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

fn = sys.argv[1] if len(sys.argv) > 1 else '_work/nl_before.net'
lines = open(fn, encoding='utf-8').read().split('\n')

nets = []
cur = None
idx = None
for i, raw in enumerate(lines):
    l = raw.strip()
    if l == '(net':
        cur = {'name': None, 'nodes': []}
        nets.append(cur)
        idx = i
        continue
    if cur is not None:
        nm = re.match(r'\(name "([^"]+)"', l)
        if nm and cur['name'] is None:
            cur['name'] = nm.group(1)
        r = re.match(r'\(ref "([^"]+)"', l)
        if r:
            cur['_lastref'] = r.group(1)
        pm = re.match(r'\(pin "([^"]+)"', l)
        if pm and '_lastref' in cur:
            cur['nodes'].append((cur['_lastref'], pm.group(1)))
            del cur['_lastref']

print("FILE:", fn, "| nets:", len(nets))

targets = ['/DISP_BL','/DISP_CS','/DISP_DC','/DISP_RES','/DISP_SCLK','/DISP_MOSI',
           '/SPEED_PULSE','/INT_MCU','/VLV_EN','/PRES_SW','/I2C_SCL','/I2C_SDA']
u7 = {}
for n in nets:
    for ref, pin in n['nodes']:
        if ref == 'U7':
            u7[pin] = n['name']

print("\n=== U7 pin -> net ===")
for p in sorted(u7, key=int):
    print("  pin %-2s -> %s" % (p, u7[p]))

print("\n=== 12 target nets -> U7 pins ===")
for t in targets:
    hits = sorted([p for p, n in u7.items() if n == t], key=int)
    print("  %-13s -> %s" % (t, hits if hits else 'NONE'))

print("\n=== power on U7 ===")
for p in ['1','2','21','22','23','43','44']:
    print("  pin %-2s -> %s" % (p, u7.get(p, 'NONE')))

print("\n=== totals ===")
print("  nets:", len(nets))
comps = re.findall(r'\(comp\n\s*\(ref "([^"]+)"', open(fn, encoding='utf-8').read())
print("  comps:", len(comps), "unique:", len(set(comps)))
