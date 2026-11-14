import io, sys, statistics
import pcbnew

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
MM = pcbnew.FromMM
TOMM = pcbnew.ToMM

A = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
B = r"C:\my\for cursor\kamaz_leveler\hardware\kicad\kamaz_leveler_hw.kicad_pcb"


def stats(path, tag):
    b = pcbnew.LoadBoard(path)
    fps = list(b.GetFootprints())
    pads = sum(f.GetPadCount() for f in fps)
    tracks = list(b.GetTracks())
    segs = [t for t in tracks if t.Type() == pcbnew.PCB_TRACE_T]
    vias = [t for t in tracks if t.Type() == pcbnew.PCB_VIA_T]
    lens = sorted(TOMM(s.GetLength()) for s in segs)
    med = statistics.median(lens) if lens else 0
    print(f"[{tag}] footprints={len(fps)} pads={pads} segs={len(segs)} vias={len(vias)} "
          f"copper={sum(lens):.2f}mm median={med:.3f} p10={lens[len(lens)//10]:.3f} p90={lens[9*len(lens)//10]:.3f}")

    # module socket = max pad count footprint
    u7 = max(fps, key=lambda f: f.GetPadCount())
    cy = u7.GetCourtyard(pcbnew.F_CrtYd)
    bb = u7.GetBoundingBox(False, False)
    bbx = u7.GetBoundingBox(False, False)
    print(f"[{tag}] U7={u7.GetReference()} pads={u7.GetPadCount()} pos=({TOMM(u7.GetPosition().x):.3f};{TOMM(u7.GetPosition().y):.3f}) "
          f"bbox x={TOMM(bb.GetLeft()):.2f}..{TOMM(bb.GetRight()):.2f} y={TOMM(bb.GetTop()):.2f}..{TOMM(bb.GetBottom()):.2f}")
    if cy.OutlineCount() > 0:
        cbb = cy.BBox()
        print(f"[{tag}] U7 courtyard x={TOMM(cbb.GetLeft()):.2f}..{TOMM(cbb.GetRight()):.2f} "
              f"y={TOMM(cbb.GetTop()):.2f}..{TOMM(cbb.GetBottom()):.2f}")

    # board outline
    edges = [d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]
    xmin = min(TOMM(d.GetBoundingBox().GetLeft()) for d in edges)
    xmax = max(TOMM(d.GetBoundingBox().GetRight()) for d in edges)
    ymin = min(TOMM(d.GetBoundingBox().GetTop()) for d in edges)
    ymax = max(TOMM(d.GetBoundingBox().GetBottom()) for d in edges)
    print(f"[{tag}] edge bbox x={xmin:.2f}..{xmax:.2f} y={ymin:.2f}..{ymax:.2f}")

    # silkscreen items outside board
    out = 0
    maxsilkx = -1e9
    for layer in (pcbnew.F_SilkS, pcbnew.B_SilkS):
        for d in b.GetDrawings():
            if d.GetLayer() != layer:
                continue
            dbb = d.GetBoundingBox()
            r = TOMM(dbb.GetRight())
            maxsilkx = max(maxsilkx, r)
            if r > xmax + 0.001 or TOMM(dbb.GetLeft()) < xmin - 0.001 or TOMM(dbb.GetTop()) < ymin - 0.001 or TOMM(dbb.GetBottom()) > ymax + 0.001:
                out += 1
    # footprint silk too
    fpout = 0
    for f in fps:
        for d in f.GraphicalItems():
            if d.GetLayer() not in (pcbnew.F_SilkS, pcbnew.B_SilkS):
                continue
            dbb = d.GetBoundingBox()
            r = TOMM(dbb.GetRight())
            maxsilkx = max(maxsilkx, r)
            if r > xmax + 0.001:
                fpout += 1
    print(f"[{tag}] silk: drawings_outside_board={out} fp_silk_right_of_edge={fpout} maxsilk_x={maxsilkx:.2f}")

    # mounting holes
    hs = {}
    for f in fps:
        if f.GetReference() in ("H1", "H2", "H3", "H4"):
            hs[f.GetReference()] = (round(TOMM(f.GetPosition().x), 3), round(TOMM(f.GetPosition().y), 3))
    print(f"[{tag}] holes={hs}")

    # nets of interest
    for net in ("/INT_MCU", "/VLV_EN", "/DISP_RES"):
        ns = [s for s in segs if s.GetNetname() == net]
        nv = [v for v in vias if v.GetNetname() == net]
        L = sum(TOMM(s.GetLength()) for s in ns)
        fc = sum(1 for s in ns if s.GetLayer() == pcbnew.F_Cu)
        print(f"[{tag}] {net}: segs={len(ns)} (F.Cu={fc}) vias={len(nv)} len={L:.2f}mm")

    # module zone: segments fully inside x>=xmin_of_module, y band of socket rows
    zx = TOMM(bb.GetLeft()) if cy.OutlineCount() == 0 else TOMM(cy.BBox().GetLeft())
    zy0 = TOMM(cy.BBox().GetTop()) if cy.OutlineCount() else TOMM(bb.GetTop())
    zy1 = TOMM(cy.BBox().GetBottom()) if cy.OutlineCount() else TOMM(bb.GetBottom())
    inz = 0
    zl = []
    for s in segs:
        p1, p2 = s.GetStart(), s.GetEnd()
        xs = [TOMM(p1.x), TOMM(p2.x)]
        ys = [TOMM(p1.y), TOMM(p2.y)]
        if min(xs) >= zx - 0.01 and min(ys) >= zy0 - 0.01 and max(ys) <= zy1 + 0.01:
            inz += 1
            zl.append(TOMM(s.GetLength()))
    zl.sort()
    print(f"[{tag}] module zone (x>={zx:.2f}, y {zy0:.2f}..{zy1:.2f}): segs={inz} "
          f"({100.0*inz/len(segs):.1f}%) median={statistics.median(zl) if zl else 0:.3f} "
          f"copper={sum(zl):.1f}mm")
    print()


stats(A, "A")
stats(B, "BASE")
