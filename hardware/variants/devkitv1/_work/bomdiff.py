import io, sys, difflib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
def norm(p):
    t = open(p, encoding='utf-8-sig', newline='').read()
    t = t.replace('\r\n', '\n').replace('\r', '\n')
    return t.strip().split('\n')
a = norm('BOM.csv')
b = norm(sys.argv[1])
print('BOM.csv rows:', len(a), ' regen rows:', len(b))
if a == b:
    print('IDENTICAL (byte-equal after newline normalisation)')
else:
    d = list(difflib.unified_diff(a, b, 'BOM.csv', 'regen', lineterm='', n=0))
    print('DIFF lines:', len(d))
    for l in d[:30]:
        print('  ', l)
