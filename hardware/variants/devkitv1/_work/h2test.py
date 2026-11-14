import sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
src = 'kamaz_leveler_devkitv1.kicad_sch'
dst = '_work/h2test.kicad_sch'
text = open(src, encoding='utf-8', newline='').read()
lines = text.splitlines(keepends=True)
# locate lib symbol ESP32-DevKitC_30P_1 block range
start = None
for i,l in enumerate(lines):
    if l.strip() == '(symbol "ESP32-DevKitC_30P_1"':
        start = i; break
print('lib start', start)
# find pin blocks within, change type for numbers 1 and 3
i = start
changed = 0
while i < len(lines):
    m = re.match(r'^\t+\t+\(pin (\w+) line', lines[i])
    if m:
        # find number
        j = i
        num = None
        while j < len(lines) and j < i+20:
            mm = re.match(r'^\s*\(number "([^"]+)"', lines[j])
            if mm: num = mm.group(1); break
            j += 1
        if num in ('1','3'):
            lines[i] = lines[i].replace('(pin %s line' % m.group(1), '(pin no_connect line')
            changed += 1
            print('pin', num, 'type', m.group(1), '-> no_connect')
    i += 1
print('changed', changed)
open(dst,'w',encoding='utf-8',newline='').write(''.join(lines))
