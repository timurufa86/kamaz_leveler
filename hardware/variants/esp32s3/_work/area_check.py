import io, sys
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
TOMM = pcbnew.ToMM

b = pcbnew.LoadBoard(r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb")

def layer_bbox(fp, layer):
    xs, ys = [], []
    for it in fp.GraphicalItems():
        if it.GetLayer() == layer:
            bb = it.GetBoundingBox()
            xs += [TOMM(bb.GetLeft()), TOMM(bb.GetRight())]
            ys += [TOMM(bb.GetTop()), TOMM(bb.GetBottom())]
    if not xs:
        return None
    return (max(xs)-min(xs)) * (max(ys)-min(ys))

rows = []
body_tot = 0
crt_tot = 0
for f in b.GetFootprints():
    if f.GetReference() in ("H1", "H2", "H3", "H4"):
        continue
    body = layer_bbox(f, pcbnew.F_Fab)
    crt = layer_bbox(f, pcbnew.F_CrtYd) or body or 0
    if body is None:
        body = crt
    body_tot += body
    crt_tot += crt
    rows.append((f.GetReference(), f.GetValue(), body, crt))

n = len(rows)
rows.sort(key=lambda r: -r[2])
print(f"компонентов: {n}")
print(f"сумма площадей тел (F.Fab bbox): {body_tot:.1f} мм²")
print(f"сумма площадей дворов (F.CrtYd bbox): {crt_tot:.1f} мм²")
print(f"полезная площадь платы: 4944 мм²")
print(f"доля тел от платы: {100*body_tot/4944:.1f}%   |   дворов: {100*crt_tot/4944:.1f}%")
print()
print("топ-15 по площади тела:")
for ref, val, body, crt in rows[:15]:
    print(f"  {ref:<5} {val:<32} тело {body:8.1f}  двор {crt:8.1f}")
