import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
lines = open('kamaz_leveler_devkitv1.kicad_pcb', encoding='utf-8').read().splitlines()
start = next(i for i,l in enumerate(lines) if l.startswith('\t(footprint "kamaz_leveler:ESP32-DevKitV1-TypeC'))
end = start
while lines[end] != '\t)':
    end += 1
blk = lines[start:end+1]
cur = None
out = []
for l in blk:
    m = re.match(r'^\t\t\(pad "([^"]+)"', l)
    if m:
        cur = m.group(1)
        out.append([cur, '<none>'])
    m2 = re.match(r'^\t\t\t\(net (?:(\d+) )?"([^"]*)"\)', l)
    if m2 and out:
        out[-1][1] = m2.group(2)
for p, n in out:
    print('  pad %-3s net=%r' % (p, n))
