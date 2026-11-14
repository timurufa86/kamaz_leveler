import io, os, sys, shutil, tempfile
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

LIB = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_hw.pretty"
NAME = "ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm"
OLD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work\BACKUP_U7_socket.kicad_mod"


def padsig(libdir):
    fp = pcbnew.FootprintLoad(libdir, NAME)
    out = {}
    for p in fp.Pads():
        pos = p.GetPosition()
        sz = p.GetSize()
        dr = p.GetDrillSize()
        out[p.GetNumber()] = (
            round(TOMM(pos.x), 4), round(TOMM(pos.y), 4),
            round(TOMM(sz.x), 4), round(TOMM(sz.y), 4),
            round(TOMM(dr.x), 4), round(TOMM(dr.y), 4),
            int(p.GetShape()), int(p.GetAttribute()), p.GetLayerSet().FmtHex(),
        )
    return out


d = tempfile.mkdtemp()
shutil.copy(OLD, os.path.join(d, NAME + ".kicad_mod"))
old = padsig(d)
new = padsig(LIB)

print(f"pads: old={len(old)} new={len(new)}")
diff = [k for k in sorted(old, key=lambda s: (len(s), s)) if old.get(k) != new.get(k)]
print(f"differing pads: {len(diff)} -> {diff}")
if diff:
    for k in diff[:5]:
        print(f"  pin {k}: old={old.get(k)}")
        print(f"  pin {k}: new={new.get(k)}")
nogeom = all(old[k][:6] == new[k][:6] for k in old if k in new)
print(f"all pad geometry identical: {nogeom}")
print("sample NEW pad:", sorted(new.items(), key=lambda kv: int(kv[0]))[0])
