import io, sys, re

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

path = r"C:\Users\Asus\Downloads\PCB_60x85.5_RUICHI_15-8.dxf"
txt = open(path, encoding="cp1251", errors="replace").read()
lines = [l.rstrip("\r") for l in txt.split("\n")]

# найти блок ENTITIES (последний) .. ENDSEC
start = 0
for i, l in enumerate(lines):
    if l.strip() == "ENTITIES":
        start = i
ents_block = lines[start + 1:]
end = next(i for i, l in enumerate(ents_block) if l.strip() == "ENDSEC")
ents_block = ents_block[:end]

# пары
def groups(block):
    g = {}
    i = 0
    while i < len(block) - 1:
        c = block[i].strip()
        v = block[i + 1].strip()
        if c == "0":
            yield g
            g = {"type": v}
        else:
            g.setdefault(c, []).append(v)
        i += 2
    yield g

res = []
for g in groups(ents_block):
    if "type" in g and g["type"] in ("LINE", "ARC", "CIRCLE", "LWPOLYLINE", "TEXT", "MTEXT", "INSERT"):
        res.append(g)

print(f"сущностей: {len(res)}\n")
def f(g, k, i=0):
    try:
        return float(g[k][i])
    except Exception:
        return None

for g in res:
    t = g["type"]
    lay = (g.get("8") or ["?"])[0]
    if t == "LINE":
        print(f"LINE слой={lay} ({f(g,'10'):.3f},{f(g,'20'):.3f}) -> ({f(g,'11'):.3f},{f(g,'21'):.3f})")
    elif t in ("ARC", "CIRCLE"):
        print(f"{t} слой={lay} центр({f(g,'10'):.3f},{f(g,'20'):.3f}) R={f(g,'40'):.3f}"
              + (f" углы {f(g,'50')}..{f(g,'51')}" if t == "ARC" else ""))
    else:
        print(f"{t} слой={lay}")
