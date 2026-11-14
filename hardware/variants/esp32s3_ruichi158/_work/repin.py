"""Repin U6 (PCAL9555A) labels in the schematic: position-anchored text swap."""
import re
import sys
import uuid
import shutil

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SCH = ROOT + r'\kamaz_leveler_ruichi158.kicad_sch'

# pin -> new net name (None = leave unconnected with a no_connect flag)
NEW = {
    4: 'BUT1', 5: 'BUT2', 6: 'BUT3', 7: 'BUT4', 8: 'BUT5',
    9: 'ST_U4_12', 10: 'ST_U4_34', 11: None,
    13: 'VLV_IN1', 14: 'VLV_IN2', 15: 'VLV_IN3', 16: 'VLV_IN4', 17: 'VLV_IN5', 18: 'VLV_IN6',
    19: 'ST_U3_12', 20: 'ST_U3_34',
}
# absolute pin positions (from the symbol definition, rot 0, no mirror)
PIN = {
    4: (482.60, 229.87), 5: (482.60, 232.41), 6: (482.60, 234.95), 7: (482.60, 237.49),
    8: (482.60, 240.03), 9: (482.60, 242.57), 10: (482.60, 245.11), 11: (482.60, 247.65),
    13: (482.60, 252.73), 14: (482.60, 255.27), 15: (482.60, 257.81), 16: (482.60, 260.35),
    17: (482.60, 262.89), 18: (482.60, 265.43), 19: (482.60, 267.97), 20: (482.60, 270.51),
}

s = open(SCH, encoding='utf-8').read()
orig = s


def label_at(x, y):
    """find (label "NAME" ... (at X Y ...)) block at the exact position"""
    pat = re.compile(r'\(label "([^"]+)"\s*\n\s*\(at\s+%07.4g\s+%07.4g\s' % (x, y))
    for m in re.finditer(r'\(label "([^"]+)"\s*\n\s*\(at ([-0-9.]+) ([-0-9.]+)', s):
        if abs(float(m.group(2)) - x) < 1e-6 and abs(float(m.group(3)) - y) < 1e-6:
            return m.group(1), m.start(), m.end()
    return None, None, None


ADD = []
changes = []
for pin, net in sorted(NEW.items()):
    x, y = PIN[pin]
    name, st, en = label_at(x, y)
    if name is None:
        print('  pin %-2d (no label) -> add %s' % (pin, net))
        ADD.append((pin, x, y, net))
        continue
    changes.append((pin, name, net, st, en))
print('found labels: %d' % len(changes))
for c in changes:
    print('  pin %-2d %-10s -> %s' % (c[0], c[1], c[2]))

# apply: replace names (longest-first edits from the end so offsets stay valid)
edits = []
for (pin, old, new, st, en) in changes:
    if new is None:
        # find the whole label block for deletion
        d = 0
        for j in range(st, len(s)):
            if s[j] == '(':
                d += 1
            elif s[j] == ')':
                d -= 1
                if d == 0:
                    break
        edits.append((st, j + 1, ''))
        continue
    if old == new:
        continue
    # locate the name literal inside this label block
    m = re.compile(r'\(label "([^"]+)"').match(s, st)
    assert m and m.group(1) == old, (pin, old, m.group(1) if m else None)
    edits.append((m.start(1) - 1, m.end(1) + 1, '"%s"' % new))

edits.sort(key=lambda t: -t[0])
for (a, b, txt) in edits:
    s = s[:a] + txt + s[b:]
print('label edits applied: %d' % len(edits))

for (pin, x, y, net) in ADD:
    blk = ('\t(label "%s"\n\t\t(at %g %g 0\n\t\t)\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27\n'
           '\t\t\t\t)\n\t\t\t)\n\t\t\t(justify left bottom\n\t\t\t)\n\t\t)\n'
           '\t\t(uuid "%s"\n\t\t)\n\t)\n') % (net, x, y, uuid.uuid4())
    anchor = s.rfind('\n)')
    s = s[:anchor + 1] + blk + s[anchor + 1:]
    print('label added at pin %d: %s' % (pin, net))

# no_connect on the freed pin
nc = '\t(no_connect\n\t\t(at 482.6 247.65)\n\t\t(uuid "%s")\n\t)\n' % uuid.uuid4()
anchor = s.rfind('\n)')
s = s[:anchor + 1] + nc + s[anchor + 1:]
print('no_connect added at (482.6, 247.65)')

open(SCH, 'w', encoding='utf-8').write(s)
print('saved schematic, delta %d bytes' % (len(s) - len(orig)))
