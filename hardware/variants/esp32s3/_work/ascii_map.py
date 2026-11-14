import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")

COLS, ROWS = 52, 27          # 2 мм на клетку: 104 x 54
g = [[" "] * COLS for _ in range(ROWS)]


def put(x, y, ch):
    c, r = int(TOMM(x) / 2), int(TOMM(y) / 2)
    if 0 <= c < COLS and 0 <= r < ROWS:
        g[r][c] = ch


# контур
for c in range(COLS):
    g[0][c] = "-" if 0 < c < COLS - 1 else "+"
    g[ROWS - 1][c] = "-" if 0 < c < COLS - 1 else "+"
for r in range(ROWS):
    g[r][0] = "|"
    g[r][COLS - 1] = "|"

# гнёзда сокета (44)
u7 = max(b.GetFootprints(), key=lambda f: f.GetPadCount())
rows = {}
for p in u7.Pads():
    rows.setdefault(round(TOMM(p.GetPosition().y)), []).append(TOMM(p.GetPosition().x))
for y, xs in rows.items():
    for c in range(int(min(xs) / 2), int(max(xs) / 2) + 1):
        if 0 < c < COLS - 1 and 0 < int(y / 2) < ROWS - 1:
            g[int(y / 2)][c] = "="

# отверстия и разъёмы/микросхемы
for f in b.GetFootprints():
    ref = f.GetReference()
    pos = f.GetPosition()
    if ref in ("H1", "H2", "H3", "H4"):
        put(pos.x, pos.y, "O")
    elif ref.startswith("J"):
        put(pos.x, pos.y, ref[1])
    elif ref in ("U5", "U6", "U8", "U1", "U2"):
        put(pos.x, pos.y, "U")

# координатная линейка
ruler = "   " + "".join(str((c * 2) // 10 % 10) if c % 5 == 0 else " " for c in range(COLS))
print(ruler)
for i, row in enumerate(g):
    mark = f"{i*2:>2} " if i % 5 == 0 else "   "
    print(mark + "".join(row))
print()
print("обозначения:  = гнездо сокета (44)   O монтажное отверстие NPTH D2,2   U5/U6/U8 микросхемы")
print("              1=J1 24В   2=J2 клапаны   3=J3   4=J4   5=J5 дисплей   6=J6 I2C   7=J7 скорость   8=J8 давление")
