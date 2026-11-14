import io, sys, math

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# контур: прямоугольник 60 x 85.5, углы R3, 4 боковых полукруглых паза R3.75 (центры y=±29)
COLS, ROWS = 30, 43           # 2 мм/клетка
g = [["."] * COLS for _ in range(ROWS)]


def C(x, y):
    c = int((x + 30) / 2 + 0.5)
    r = int((y + 42.75) / 2 + 0.5)
    return c, r


def mark(x, y, ch="#"):
    c, r = C(x, y)
    if 0 <= c < COLS and 0 <= r < ROWS:
        g[r][c] = ch


# верхний/нижний края и вертикальные серединные
for x in [i * 0.5 - 30 for i in range(121)]:
    if abs(x) <= 27:
        mark(x, 42.75); mark(x, -42.75)
    if abs(x) <= 30:
        pass
for y in [i * 0.5 - 42.75 for i in range(172)]:
    if 25.25 <= abs(y) <= 39.75 or abs(y) <= 25.25:
        mark(-30, y); mark(30, y)

# углы R3
for cx, cy in ((-27, 39.75), (27, 39.75), (27, -39.75), (-27, -39.75)):
    for a in range(361):
        rad = math.radians(a)
        mark(cx + 3 * math.cos(rad), cy + 3 * math.sin(rad))

# пазы: полукруги R3.75 внутрь, центры (±27, ±29), от 25.25 до 32.75 (слева/справа)
for cx, cy in ((-27, 29), (27, 29), (-27, -29), (27, -29)):
    # дуга идёт внутрь (для левых — вправо, для правых — влево)
    inward = 1 if cx < 0 else -1
    for a in range(-90, 91):
        rad = math.radians(a)
        mark(cx - inward * 3.75 * math.cos(rad), cy + 3.75 * math.sin(rad))
    # касательные от x=±27 к x=±30 на y=25.25 и 32.75
    y0, y1 = 25.25, 32.75
    for yy in (y0, y1):
        xa, xb = sorted([-30, -27]) if cx < 0 else sorted([30, 27])
        x = xa
        while x <= xb:
            mark(x, yy); x += 0.5

print("  x:  -30      -20      -10        0        10       20       30")
for i, row in enumerate(g):
    y = 42.75 - i * 2
    mark = f"{y:5.1f} " if i % 5 == 0 else "      "
    print(mark + "".join(row))
