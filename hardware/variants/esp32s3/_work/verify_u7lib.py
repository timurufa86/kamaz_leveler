import io, sys, re
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

LIB = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_hw.pretty"
NAME = "ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm"
OLD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work\BACKUP_U7_socket.kicad_mod"


def bbox_of(path_or_fp, is_file=False):
    fp = pcbnew.FootprintLoad(LIB, NAME) if not is_file else None
    if is_file:
        # load the backup by temporarily pointing at its own dir/name
        import os, shutil, tempfile
        d = tempfile.mkdtemp()
        p = os.path.join(d, NAME + ".kicad_mod")
        shutil.copy(path_or_fp, p)
        fp = pcbnew.FootprintLoad(d, NAME)
    res = {}
    for layer, tag in ((pcbnew.F_CrtYd, "CrtYd"), (pcbnew.F_SilkS, "SilkS"), (pcbnew.F_Fab, "Fab")):
        xs0 = ys0 = 1e9
        xs1 = ys1 = -1e9
        n = 0
        for it in fp.GraphicalItems():
            if it.GetLayer() != layer:
                continue
            bb = it.GetBoundingBox()
            xs0 = min(xs0, TOMM(bb.GetLeft())); xs1 = max(xs1, TOMM(bb.GetRight()))
            ys0 = min(ys0, TOMM(bb.GetTop())); ys1 = max(ys1, TOMM(bb.GetBottom()))
            n += 1
        res[tag] = (n, round(xs0, 4), round(xs1, 4), round(ys0, 4), round(ys1, 4))
    res["pads"] = len(list(fp.Pads()))
    return res


def padlines(path):
    txt = open(path, encoding="utf-8").read()
    return re.findall(r'\(pad "[^"]*"[\s\S]*?\n\t\t\)', txt)


new = bbox_of(None)
old = bbox_of(OLD, is_file=True)
for tag in ("CrtYd", "SilkS", "Fab"):
    print(f"{tag:6} OLD n={old[tag][0]:<3} x[{old[tag][1]:>9}..{old[tag][2]:>8}] y[{old[tag][3]:>9}..{old[tag][4]:>8}]")
    print(f"{tag:6} NEW n={new[tag][0]:<3} x[{new[tag][1]:>9}..{new[tag][2]:>8}] y[{new[tag][3]:>9}..{new[tag][4]:>8}]")
print(f"pads OLD={old['pads']} NEW={new['pads']}")

cur = rf"{LIB}\{NAME}.kicad_mod"
po, pn = padlines(OLD), padlines(cur)
print(f"pad blocks OLD={len(po)} NEW={len(pn)} identical={sorted(po) == sorted(pn)}")

# world-space check with the real board placement
PX, PY = 102.33, 38.26
for tag in ("CrtYd", "SilkS", "Fab"):
    print(f"{tag:6} world max x: OLD {PX + old[tag][2]:.4f}  NEW {PX + new[tag][2]:.4f}")

txt = open(cur, encoding="utf-8").read()
print("brackets:", txt.count("(") - txt.count(")"), "| name kept:", f'(footprint "{NAME}"' in txt)
