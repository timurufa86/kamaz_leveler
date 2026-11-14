import sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
f = sys.argv[1]; X0,Y0,X1,Y1 = [float(v) for v in sys.argv[2:6]]
lines = open(f, encoding='utf-8').read().splitlines()
print(lines[0])
m = re.search(r'bounds ([-\d.]+),([-\d.]+)\.\.([-\d.]+),([-\d.]+) cell ([-\d.]+)', lines[0])
bx0, by0, cell = float(m.group(1)), float(m.group(2)), float(m.group(5))
data = {}
for L in lines[1:]:
    p = L.split()
    if len(p) != 2: continue
    data[round(float(p[0]),3)] = p[1]
def iof(x): return int(round((x - bx0)/cell))
i0 = iof(X0); i1 = iof(X1)
rul = []
for i in range(i0, i1+1):
    t = int(round((bx0 + i*cell)*10))
    if t % 50 == 0: rul.append('|')
    elif t % 10 == 0: rul.append(str((t//10) % 10))
    else: rul.append(' ')
print('        ' + ''.join(rul))
print('        x=%.2f .. %.2f' % (bx0+i0*cell, bx0+i1*cell))
keys = sorted(data.keys(), reverse=True)
for key in keys:
    if key > Y1 + 1e-6 or key < Y0 - 1e-6: continue
    print('%7.2f %s' % (key, data[key][i0:i1+1]))
