import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def parse(fn):
    lines = open(fn, encoding='utf-8').read().split('\n')
    nets = []
    cur = None
    for raw in lines:
        l = raw.strip()
        if l == '(net':
            cur = {'name': None, 'nodes': []}
            nets.append(cur)
            continue
        if cur is not None:
            nm = re.match(r'\(name "([^"]+)"', l)
            if nm and cur['name'] is None:
                cur['name'] = nm.group(1)
            r = re.match(r'\(ref "([^"]+)"', l)
            if r:
                cur['_ref'] = r.group(1)
            pm = re.match(r'\(pin "([^"]+)"', l)
            if pm and '_ref' in cur:
                cur['nodes'].append((cur['_ref'], pm.group(1)))
                del cur['_ref']
    return nets

before = parse('_work/nl_before.net')
after = parse('_work/nl_after.net')

SPEC = [('/DISP_BL', '4', 'IO4'), ('/DISP_CS', '5', 'IO5'), ('/DISP_DC', '9', 'IO16'),
        ('/DISP_RES', '10', 'IO17'), ('/DISP_SCLK', '11', 'IO18'), ('/SPEED_PULSE', '12', 'IO8'),
        ('/DISP_MOSI', '17', 'IO11'), ('/INT_MCU', '19', 'IO13'), ('/VLV_EN', '20', 'IO14'),
        ('/PRES_SW', '32', 'IO38'), ('/I2C_SCL', '39', 'IO47'), ('/I2C_SDA', '40', 'IO21')]

def u7pins(nets):
    d = {}
    for n in nets:
        for ref, pin in n['nodes']:
            if ref == 'U7':
                d.setdefault(n['name'], []).append(pin)
    return d

b, a = u7pins(before), u7pins(after)
print("nets before/after: %d / %d" % (len(before), len(after)))
print("U7 nets before/after: %d / %d" % (len(b), len(a)))
print()
print("=== SPEC CHECK: net -> U7 pad ===")
fails = 0
for net, pad, io in SPEC:
    got = a.get(net, [])
    ok = got == [pad]
    if not ok:
        fails += 1
    print("  %-13s spec pad %-2s  netlist pad %-8s %s   (was %s)" % (
        net, pad, got if got else 'MISSING', 'OK ' if ok else 'FAIL', b.get(net)))
print("\nSPEC FAILURES:", fails)

print("\n=== power on U7 ===")
for net, pads in sorted(a.items()):
    if net in ('+3V3_ESP', '+5V', 'GND'):
        print("  %-10s -> %s" % (net, sorted(pads, key=int)))

print("\n=== per-net node diff (12 signal nets) ===")
bn = {n['name']: n['nodes'] for n in before}
an = {n['name']: n['nodes'] for n in after}
specnames = [s[0] for s in SPEC]
for net in specnames:
    sb = set(bn.get(net, []))
    sa = set(an.get(net, []))
    print("  %-13s nodes %d->%d  only_before=%s  only_after=%s" % (
        net, len(sb), len(sa), sorted(sb - sa), sorted(sa - sb)))

print("\n=== nets only in before / only in after ===")
print("  only_before:", sorted(set(bn) - set(an)))
print("  only_after :", sorted(set(an) - set(bn)))

print("\n=== freed pads 6,7,8,15,16,18 state after ===")
inv = {}
for n in after:
    for ref, pin in n['nodes']:
        if ref == 'U7':
            inv[pin] = n['name']
for p in ['6', '7', '8', '15', '16', '18', '3']:
    nm = inv.get(p, 'NOT-IN-NETLIST')
    print("  pad %-2s -> %s%s" % (p, nm, '   [no-connect]' if 'unconnected' in nm or nm == 'NOT-IN-NETLIST' else '   <-- STILL CONNECTED'))

print("\n=== all 12 net NAMES present byte-exact? ===")
raw = open('kamaz_leveler_s3.kicad_sch', encoding='utf-8').read()
for net in specnames:
    short = net.lstrip('/')
    print("  %-13s in-file label count: %d" % (net, len(re.findall(r'\(label "%s"' % re.escape(short), raw))))
