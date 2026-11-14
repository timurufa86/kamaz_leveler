import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
f = sys.argv[1]
want = set(sys.argv[2].split(','))
lines = open(f, encoding='utf-8', newline='').read().splitlines()
i = 0
while i < len(lines):
    if lines[i] == '\t(symbol':
        j = i + 1
        props = {}
        while j < len(lines) and lines[j] != '\t)':
            mm = re.match(r'^\t\t\(property "([^"]+)" "([^"]*)"', lines[j])
            if mm: props[mm.group(1)] = mm.group(2)
            j += 1
        r = props.get('Reference')
        if r in want:
            print('%-5s Description=%r Datasheet=%r' % (r, props.get('Description'), props.get('Datasheet')))
        i = j + 1
    else:
        i += 1
