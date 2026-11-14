import io, re, sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

EXPECT = {
    "/DISP_BL": ("4", "IO4"), "/DISP_CS": ("5", "IO5"), "/DISP_DC": ("9", "IO16"),
    "/DISP_RES": ("10", "IO17"), "/DISP_SCLK": ("11", "IO18"), "/DISP_MOSI": ("17", "IO11"),
    "/SPEED_PULSE": ("12", "IO8"), "/INT_MCU": ("19", "IO13"), "/VLV_EN": ("20", "IO14"),
    "/PRES_SW": ("32", "IO38"), "/I2C_SCL": ("39", "IO47"), "/I2C_SDA": ("40", "IO21"),
}

txt = open(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work\verify.net", encoding="utf-8").read()
start = txt.index("\n\t(nets")
body = txt[start:]

# header positions
hdr = [(m.start(), m.group(1), m.group(2)) for m in
       re.finditer(r'\(net\s*\n\s*\(code "(\d+)"\)\s*\n\s*\(name "([^"]*)"\)', body)]
hdr.append((len(body), None, None))

found, nets = {}, 0
for k in range(len(hdr) - 1):
    s, code, name = hdr[k]
    blk = body[s:hdr[k + 1][0]]
    nets += 1
    for node in re.finditer(r'\(node\s*\n\s*\(ref "([^"]+)"\)\s*\n\s*\(pin "([^"]+)"\)(?:\s*\n\s*\(pinfunction "([^"]*)"\))?', blk):
        if node.group(1) == "U7":
            found.setdefault(name, []).append((node.group(2), node.group(3)))

print(f"nets parsed: {nets}")
ok = 0
for net, (pad, func) in EXPECT.items():
    got = found.get(net, [])
    good = len(got) == 1 and got[0][0] == pad and (got[0][1] or "").split("_")[0] == func
    ok += good
    print(f"  {'PASS' if good else 'FAIL'} {net:<14} expect {pad}/{func:<5} got {got}")

print("\nU7 power:")
for net in ("+3V3_ESP", "+5V", "GND"):
    print(f"  {net:<10} {sorted((p for p, _ in found.get(net, [])), key=int)}")

allpins = sorted({p for lst in found.values() for p, _ in lst}, key=int)
print(f"\nU7 pins carrying a net: {len(allpins)} -> {allpins}")
print(f"RESULT: {ok}/{len(EXPECT)} pairs match the spec")
