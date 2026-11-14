import sys
sys.path.insert(0, '_work')
from sexpr import *

root = parse(open('kamaz_leveler_devkitv1.kicad_sch', encoding='utf-8').read())[0]
libs = kid(root,'lib_symbols')
libsym = None
for s in kids(libs,'symbol'):
    if s[1] == 'ESP32-DevKitC_30P_1': libsym = s
inst = None
for s in kids(root,'symbol'):
    for p in kids(s,'property'):
        if p[1]=='Reference' and p[2]=='U7': inst = s
sx,sy,srot = [float(x) for x in kid(inst,'at')[1:4]]
print('inst at', sx, sy, srot, 'mirror', kid(inst,'mirror'))

def fmt(v):
    return ('%.4f' % v).rstrip('0').rstrip('.')

pins = {}
for p in find_all(libsym,'pin'):
    num = kid(p,'number')[1]; nam = kid(p,'name')[1]
    px,py,po = [float(x) for x in kid(p,'at')[1:4]]
    # rot 0, no mirror
    X = sx + px; Y = sy - py
    pins[num] = (nam, round(X,3), round(Y,3))

def key(x,y): return (round(x,3), round(y,3))
pos = {}
for n,(nam,X,Y) in pins.items(): pos[key(X,Y)] = n

# collect items at pin coords
labels = []
for kind in ('label','global_label','hierarchical_label'):
    for l in kids(root,kind):
        at = kid(l,'at'); x=float(at[1]); y=float(at[2])
        labels.append((kind, l[1], round(x,3), round(y,3)))
ncs = set()
for l in kids(root,'no_connect'):
    at = kid(l,'at'); ncs.add((round(float(at[1]),3), round(float(at[2]),3)))
wires = []
for w in kids(root,'wire'):
    pts = kid(w,'pts')
    xy = kids(pts,'xy')
    wires.append(((round(float(xy[0][1]),3), round(float(xy[0][2]),3)), (round(float(xy[1][1]),3), round(float(xy[1][2]),3))))

lab_at = {}
for kind,name,x,y in labels:
    lab_at.setdefault((x,y), []).append((kind,name))
wire_at = {}
for a,b in wires:
    wire_at.setdefault(a, []).append(b)
    wire_at.setdefault(b, []).append(a)

print('%-4s %-6s %-9s %-9s %-22s %s' % ('pad','name','x','y','labels','wires/no_connect'))
for n in sorted(pins, key=lambda k:int(k)):
    nam,X,Y = pins[n]
    k = key(X,Y)
    ls = lab_at.get(k, [])
    wsz = wire_at.get(k, [])
    nc = 'NC' if k in ncs else ''
    print('%-4s %-6s %-9s %-9s %-22s %s %s' % (n, nam, fmt(X), fmt(Y), ','.join('%s:%s'%(a,b) for a,b in ls), 'wires=%d'%len(wsz), nc))
