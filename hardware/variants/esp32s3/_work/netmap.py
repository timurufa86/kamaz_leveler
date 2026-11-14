import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

A = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
b = pcbnew.LoadBoard(A)
u7 = max(b.GetFootprints(), key=lambda f: f.GetPadCount())
u7pads = {p.GetNumber(): (round(TOMM(p.GetPosition().x), 2), round(TOMM(p.GetPosition().y), 2), p.GetNetname())
          for p in u7.Pads()}

NETS = ["/I2C_SDA", "/I2C_SCL", "/DISP_MOSI", "/DISP_SCLK", "/DISP_CS", "/DISP_DC",
        "/DISP_RES", "/DISP_BL", "/VLV_EN", "/INT_MCU", "/PRES_SW", "/SPEED_PULSE"]

print("=== U7 pixel map: pad -> GPIO/net (row A y=38.26, row B y=15.40) ===")
for n in sorted(u7pads, key=lambda s: int(s)):
    x, y, net = u7pads[n]
    print(f"  pad {n:>2}: ({x:7.2f};{y:6.2f}) {net}")

print("\n=== other pads per signal net (excluding U7) ===")
for net in NETS:
    ends = []
    for f in b.GetFootprints():
        if f.GetReference() == "U7":
            continue
        for p in f.Pads():
            if p.GetNetname() == net:
                pp = p.GetPosition()
                ends.append(f"{f.GetReference()}.{p.GetNumber()}({TOMM(pp.x):.1f};{TOMM(pp.y):.1f})")
    print(f"  {net:<14} -> {' '.join(ends) if ends else '(нет других падов)'}")

print("\n=== key parts ===")
for ref in ("J5", "J3", "J4", "J6", "J7", "J8", "U6", "U12", "U5", "R37", "R29", "R30", "U8", "C35"):
    fp = [f for f in b.GetFootprints() if f.GetReference() == ref]
    if fp:
        f = fp[0]
        p = f.GetPosition()
        print(f"  {ref:<4} {f.GetValue():<28} ({TOMM(p.x):7.2f};{TOMM(p.y):6.2f}) rot={f.GetOrientationDegrees():.0f}")
