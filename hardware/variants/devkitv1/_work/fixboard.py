import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew

path = sys.argv[1]
schmap = json.load(open(sys.argv[2], encoding='utf-8'))     # {"1": "unconnected-(...)"} expected nets for U7
apply_ = (len(sys.argv) > 3 and sys.argv[3] == 'apply')

FIELDS = {
    'C15': {'Description': 'Unpolarized capacitor'},
    'C16': {'Description': 'Unpolarized capacitor'},
    'U8':  {'Description': '1A Low Dropout regulator, positive, 3.3V fixed output, SOT-223'},
    'U7':  {'Datasheet': 'https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html'},
}

b = pcbnew.LoadBoard(path)
u7 = [f for f in b.GetFootprints() if f.GetReference() == 'U7'][0]
print('U7 footprint:', u7.GetFPID().GetLibItemName(), 'nets on board:', b.GetNetCount())

plan_nets = []
refuse = 0
for pad in u7.Pads():
    num = pad.GetNumber()
    want = schmap.get(num, '')
    cur = pad.GetNetname()
    if cur == want:
        continue
    if cur == '' and want.startswith('unconnected-('):
        plan_nets.append((num, cur, want, pad))
    else:
        print('  REFUSE pad %s: board=%r schematic=%r' % (num, cur, want))
        refuse += 1
print('--- U7 pad net assignments planned: %d' % len(plan_nets))
for num, cur, want, _ in plan_nets:
    print('   pad %-3s %r -> %r' % (num, cur, want))

plan_fields = []
for fp in b.GetFootprints():
    ref = fp.GetReference()
    if ref not in FIELDS:
        continue
    for f in fp.GetFields():
        n = f.GetName()
        if n in FIELDS[ref]:
            cur = f.GetText()
            want = FIELDS[ref][n]
            if cur != want:
                plan_fields.append((fp, f, ref, n, cur, want))
print('--- footprint fields planned: %d' % len(plan_fields))
for _, _, ref, n, cur, want in plan_fields:
    print('   %-4s %-12s %r -> %r' % (ref, n, cur, want))

if refuse:
    print('ABORT: unsafe changes present'); sys.exit(2)

if not apply_:
    print('DRY RUN - nothing written')
    sys.exit(0)

for num, cur, want, pad in plan_nets:
    net = b.FindNet(want)
    if net is None or net.GetNetCode() <= 0:
        net = pcbnew.NETINFO_ITEM(b, want)
        b.Add(net)
    pad.SetNet(net)
for fp, f, ref, n, cur, want in plan_fields:
    f.SetText(want)

pcbnew.SaveBoard(path, b)
print('SAVED', path)

# verify by reload
b2 = pcbnew.LoadBoard(path)
u7b = [f for f in b2.GetFootprints() if f.GetReference() == 'U7'][0]
ok = 0
for pad in u7b.Pads():
    num = pad.GetNumber()
    if pad.GetNetname() == schmap.get(num, ''):
        ok += 1
    else:
        print('  MISMATCH after save: pad', num, repr(pad.GetNetname()), 'want', repr(schmap.get(num,'')))
print('verify: %d/30 U7 pads now agree with the schematic netlist' % ok)
print('nets after:', b2.GetNetCount(), 'footprints:', len(list(b2.GetFootprints())))
