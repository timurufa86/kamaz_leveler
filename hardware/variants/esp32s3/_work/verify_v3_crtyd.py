import io, sys, os, re, shutil, tempfile, hashlib
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

BASE = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3"
LIB = os.path.join(BASE, "kamaz_leveler_hw.pretty")
NAME = "ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm"
CUR = os.path.join(LIB, NAME + ".kicad_mod")
OLD = os.path.join(BASE, "_work", "BACKUP_U7_socket_v3.kicad_mod")
PX, PY = 102.33, 38.26


def load(path, is_cur):
    if is_cur:
        return pcbnew.FootprintLoad(LIB, NAME)
    d = tempfile.mkdtemp()
    p = os.path.join(d, NAME + ".kicad_mod")
    shutil.copy(path, p)
    return pcbnew.FootprintLoad(d, NAME)


def bb(fp, layer):
    xs0 = ys0 = 1e9
    xs1 = ys1 = -1e9
    n = 0
    for it in fp.GraphicalItems():
        if it.GetLayer() != layer:
            continue
        b = it.GetBoundingBox()
        xs0 = min(xs0, TOMM(b.GetLeft())); xs1 = max(xs1, TOMM(b.GetRight()))
        ys0 = min(ys0, TOMM(b.GetTop()));  ys1 = max(ys1, TOMM(b.GetBottom()))
        n += 1
    return (n, round(xs0, 4), round(xs1, 4), round(ys0, 4), round(ys1, 4))


def layerstr(p):
    try:
        return p.GetLayerSet().FmtHex()
    except Exception:
        try:
            return ",".join(sorted(str(x) for x in p.GetLayerSet().Seq()))
        except Exception:
            return "?"


def pads(fp):
    out = []
    for p in fp.Pads():
        pos = p.GetPosition(); sz = p.GetSize(); dr = p.GetDrillSize()
        out.append((p.GetNumber(),
                    round(TOMM(pos.x), 4), round(TOMM(pos.y), 4),
                    round(TOMM(sz.x), 4), round(TOMM(sz.y), 4),
                    round(TOMM(dr.x), 4), round(TOMM(dr.y), 4),
                    layerstr(p)))
    return out


def crtyd_ends(path):
    txt = open(path, encoding="utf-8").read()
    segs = []
    for m in re.finditer(r'\(fp_line \(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\)[^\n]*F\.CrtYd', txt):
        segs.append(tuple(float(v) for v in m.groups()))
    return segs


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


old_fp = load(OLD, False)
new_fp = load(CUR, True)

print("=== BBOX (pcbnew GetBoundingBox, mm) ===")
for tag, layer in (("CrtYd", pcbnew.F_CrtYd), ("SilkS", pcbnew.F_SilkS), ("Fab", pcbnew.F_Fab)):
    o = bb(old_fp, layer); n = bb(new_fp, layer)
    print(f"{tag:6} OLD n={o[0]:<3} x[{o[1]:>9.4f}..{o[2]:>8.4f}] y[{o[3]:>9.4f}..{o[4]:>8.4f}]")
    print(f"{tag:6} NEW n={n[0]:<3} x[{n[1]:>9.4f}..{n[2]:>8.4f}] y[{n[3]:>9.4f}..{n[4]:>8.4f}]")

oc = bb(old_fp, pcbnew.F_CrtYd); nc = bb(new_fp, pcbnew.F_CrtYd)
of = bb(new_fp, pcbnew.F_Fab); ns = bb(new_fp, pcbnew.F_SilkS)

print("\n=== CRITERIA ===")
exp_old = (-61.5750, 1.8450, -24.4050, 1.5450)
exp_new = (-61.5750, 1.6700, -24.4050, 1.5450)
got_old = (oc[1], oc[2], oc[3], oc[4]); got_new = (nc[1], nc[2], nc[3], nc[4])
print(f"1 CrtYd OLD {got_old} match={all(abs(a-b) <= 0.001 for a, b in zip(got_old, exp_old))}")
print(f"1 CrtYd NEW {got_new} match={all(abs(a-b) <= 0.001 for a, b in zip(got_new, exp_new))}")
print(f"2 world max x CrtYd = {PX + nc[2]:.4f}  expect 104.0000  ok={abs(PX + nc[2] - 104.0) <= 0.001}")
print(f"3 gap CrtYd-Fab antenna end = {nc[2] - of[2]:.4f}  expect 0.0500  ok={abs(nc[2] - of[2] - 0.05) <= 0.001}")
print(f"3 contain L={nc[1] <= of[1] + 0.001} R={of[2] <= nc[2] + 0.001} T={nc[3] <= of[3] + 0.001} B={of[4] <= nc[4] + 0.001}")
print(f"   Fab bbox x[{of[1]:.4f}..{of[2]:.4f}] y[{of[3]:.4f}..{of[4]:.4f}]")

print("\n=== 4 CLOSURE (parsed from file text) ===")
for lbl, path in (("OLD", OLD), ("NEW", CUR)):
    segs = crtyd_ends(path)
    print(f"{lbl} segments={len(segs)}")
    for s in segs:
        print(f"   ({s[0]:.3f},{s[1]:.3f})->({s[2]:.3f},{s[3]:.3f})")
    if len(segs) == 4:
        closed = True
        for i in range(len(segs)):
            a = segs[i]; b = segs[(i + 1) % len(segs)]
            d = max(abs(a[2] - b[0]), abs(a[3] - b[1]))
            print(f"   pair {i}->{i+1}: dist={d:.4f} ok={d <= 0.001}")
            closed = closed and d <= 0.001
        print(f"{lbl} closed={closed}")

print("\n=== 5 PADS ===")
po, pn = pads(old_fp), pads(new_fp)
print(f"pads OLD={len(po)} NEW={len(pn)}")
diff = [i for i in range(max(len(po), len(pn)))
        if i >= len(po) or i >= len(pn) or po[i] != pn[i]]
print(f"pad diffs = {len(diff)}")
for i in diff[:10]:
    print("  ", po[i] if i < len(po) else None, "|", pn[i] if i < len(pn) else None)
sz = set((p[3], p[4], p[5], p[6]) for p in pn)
print(f"pad size/drill set NEW = {sz}")

print("\n=== 6 SILK ===")
print(f"Silk bounds equal OLD vs NEW: {bb(old_fp, pcbnew.F_SilkS) == bb(new_fp, pcbnew.F_SilkS)}")
print(f"Silk count {bb(old_fp, pcbnew.F_SilkS)[0]} == {bb(new_fp, pcbnew.F_SilkS)[0]}: {bb(old_fp, pcbnew.F_SilkS)[0] == bb(new_fp, pcbnew.F_SilkS)[0]}")
print(f"Silk x max NEW = {ns[2]:.4f} (expect 1.6300) world = {PX + ns[2]:.4f}")

print("\n=== 7 FILE VALIDITY ===")
txt = open(CUR, encoding="utf-8").read()
print(f"brackets balance = {txt.count('(') - txt.count(')')}")
_nm = '(footprint "' + NAME + '"'
print("name kept = %s" % (_nm in txt))
print(f"load ok = {new_fp is not None and len(list(new_fp.Pads())) == 44}")
print(f"fp ref name = {new_fp.GetFPID().GetLibItemName()}")

print("\n=== HASHES ===")
print(f"OLD sha256 {sha(OLD)}  {os.path.getsize(OLD)}B")
print(f"NEW sha256 {sha(CUR)}  {os.path.getsize(CUR)}B")
ot = open(OLD, encoding="utf-8").read().splitlines()
nt = txt.splitlines()
print(f"lines OLD={len(ot)} NEW={len(nt)} changed_line_count={sum(1 for a, b in zip(ot, nt) if a != b)}")
