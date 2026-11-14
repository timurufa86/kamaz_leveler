# D1 (1.5KE36CA) relocation study — KAMAZ pneumatic-suspension controller

**Status:** read-only placement study. Board snapshot analysed: `hardware/kicad/kamaz_leveler_hw.kicad_pcb`,
sha256 `753333837051157abb25d34121459bd03949900e5e692ad0c93beea0cf9275ff` (1 011 513 B), copied once at the
start of the study into C:\my\sch3\run_now\w4_C6\snapshot.kicad_pcb. The live project file was re-hashed at the
end of the study and is **byte-identical** — no project file was modified. All scratch material is in
C:\my\sch3\run_now\w4_C6\.

---

## 0. VERDICT — yes, D1 can be placed legally

**A legal, fully connectable home for D1 exists. The previous "infeasible" verdict was wrong.**

Four placements satisfy every hard rule *and* have both nets routed to existing copper by an actual
two-layer search (not by assertion). The best two (Z and X) are in the **left margin corridor x ≈ 4.0 mm**,
i.e. exactly the strip the previous attempt called the only courtyard-free corridor and then rejected —
because it demanded a 3.2 mm pad (needs 3.50 mm of copper-free width) instead of the 2.0 mm pad
(needs 2.30 mm) that a DO-201AD lead actually warrants.

Measured summary (C = clearance to other nets' copper, both layers, real copper polygons):

| # | pos (mm) | rot | pad1 (+24V) | pad2 (GND) | min C pad1 | min C pad2 | pad->edge | hole-hole | D1 courtyard vs J2/U7 | board courtyard pairs | +24V run | GND run |
|---|----------|-----|-------------|------------|-----------|-----------|----------|-----------|-----------------------|----------------------|----------|---------|
| **Z** | (4.000, 1.500) | 270 | (4.000, 1.500) | (4.000, 16.740) | **3.898** | **0.511** | 0.505 / 3.005 | 3.105 | no / no | **34** (was 36) | 9.71 mm, 0 via | 14.44 mm, 2 via |
| **X** | (4.000, 16.700) | 90 | (4.000, 16.700) | (4.000, 1.460) | **0.551** | **3.927** | 3.005 / 0.465* | 3.124 | no / no | **34** (was 36) | 4.60 mm, 1 via | 14.31 mm, 1 via |
| **W** | (21.100, 2.700) | 180 | (21.100, 2.700) | (5.860, 2.700) | **0.237** | **1.735** | 1.705 / 1.705 | 1.765 | no / no | 36 (was 36) | **0.85 mm, 0 via** | 12.01 mm, 1 via |
| **H** | (6.500, 2.700) | 0 | (6.500, 2.700) | (21.740, 2.700) | **1.282** | **0.236** | 1.705 / 1.705 | 1.130 | no / no | 37 (was 36) | 6.81 mm, 0 via | **1.60 mm, 0 via** |

`*` X pad 2 edge = 0.465 mm — above the 0.30 mm hard rule but 0.035 mm below the board's own
copper_edge_clearance = 0.5 mm setting, so KiCad DRC will raise one edge-clearance marker. See section 7.
All four: pth_inside_courtyard = **none**, no hole-to-hole conflict, D1 body fully on-board with
nothing under it.

Pad proposal for all four: **round pads 2.00 mm dia, drill 1.30 mm -> annular ring 0.35 mm** (both pads;
pad 1 changed from roundrect to circle). Legal against min_through_hole_diameter 0.2 (board setting 0.30)
and min_via_annular_width 0.1.

---

## 1. Why the current placement is illegal (baseline, measured)

| item | measured |
|---|---|
| footprint origin | (49.000, 44.000), rot -90 |
| pad 1 (+24V) | (49.000, 44.000), 3.20 dia / drill 1.60, annular 0.80 |
| pad 2 (GND) | (49.000, **59.240**), 3.20 dia / drill 1.60 |
| board outline | rectangle x 0..83, y 0..58 (single closed loop, area 4814 mm2, **no cut-outs**) |
| pad 2 hole edge | **1.240 mm outside Edge.Cuts** -> not manufacturable |
| pad 2 copper edge | 2.240 mm outside Edge.Cuts |
| D1 courtyard | x[46.16..51.84] y[42.16..**61.09**] -> 3.09 mm off-board |
| off-board copper | 3 GND segments, w 0.25: (47.00,57.24)->(49.00,59.24) on B.Cu, (49.00,59.24)->(67.99,59.24) and (67.99,59.24)->(75.13,52.10) on F.Cu |

**Important, and not fixable by moving D1 alone:** that off-board GND chain is 3 segments / ~33 mm of
copper outside the outline and it does **not** hang solely on D1 — it also ties x = 47.00 (J2 area) to
x = 75.13. Deleting D1's old pad removes one endpoint only; **those three segments must be re-routed
inside the board as a separate piece of work.** A D1 move alone does not make the board manufacturable.

## 2. Rules actually in force (read from the project, not assumed)

| rule | value | source |
|---|---|---|
| min_clearance | 0.127 | .kicad_pro |
| netclass clearance: Default / +24V | 0.150 / 0.127 | .kicad_pro net_settings |
| min_hole_to_hole | 0.250 (net-agnostic) | .kicad_pro |
| min_hole_clearance | 0.250 | .kicad_pro |
| min_through_hole_diameter | 0.20 (board's m_MinThroughDrill = 0.30) | .kicad_pro / board |
| min_via_diameter / min_via_annular_width | 0.45 / 0.10 | .kicad_pro |
| min_copper_edge_clearance | 0.20 (.kicad_pro) — **0.50 in the board's design settings** | both |
| copper layers | 2 (F.Cu, B.Cu) | board header |
| zones | **0** (all copper is pads / 2464 tracks / 254 vias) | board |
| DRU | only 2 local rules (U1/U2 exposed-pad clearance 0.05) | .kicad_dru |

D1's pads on +24V are in the +24V netclass (0.127); everything else it can touch is Default (0.150).
**The governing clearance for a D1 pad is therefore 0.150 mm.** This study searched and verified at
0.200 mm wherever the geometry allowed, and reports the true measured number.

## 3. THE PAD-SIZE QUESTION (answered with numbers)

D1's pads are 3.20 mm dia with 1.60 mm drill. A 1.5KE36CA in DO-201AD has a 1.2-1.3 mm lead, so
3.20/1.60 is roughly 2x larger than needed.

| pad dia / drill | annular | radial copper use (r) | copper-free width needed (2*(r + 0.15 clr)) |
|---|---|---|---|
| 3.20 / 1.60 (today) | 0.80 | 1.60 | **3.50 mm** |
| 2.20 / 1.50 | 0.35 | 1.10 | **2.50 mm** |
| **2.00 / 1.30 (proposed)** | **0.35** | **1.00** | **2.30 mm** |

### The left-margin corridor, re-measured

Previous claim: the only courtyard-free corridors are x 0.405..6.195 at y 0.4..19.6 and y 37.1..57.6,
where /BUT4 (x = 2.20) and /BUT2 (x = 5.76) leave **3.31 mm**, "needing 3.654 mm".

**Refuted — with the 2.0 mm pad it fits with room to spare.** Measured on this snapshot:

* copper-free lane for a D1 pad centre in that corridor: **x 3.6 .. 4.4 mm** (verified at 0.20 mm
  clearance; x 3.4 .. 4.4 mm at the 0.15 mm rule) => the copper walls sit at **x ~ 2.45 and x ~ 5.60**,
  i.e. **~3.15 mm of usable width** (agrees with the earlier 3.31 mm estimate).
* required with the old pad: 3.20 + 2*0.15 = **3.50 mm > 3.15 mm -> does not fit** (this is the only
  part of the old analysis that was right).
* required with a 2.00 mm pad: 2.00 + 2*0.15 = **2.30 mm <= 3.15 mm -> fits, 0.85 mm spare**
  (0.75 mm spare at this study's 0.20 mm design target).
* 2.20 mm pad: 2.50 mm <= 3.15 mm -> also fits in principle, **but** the 2.2 mm geometry was not
  re-verified cell-by-cell; use 2.0 mm.
* the corridor contains **no components at all** (only vias/tracks of other nets), so D1's courtyard
  (5.70 mm wide) also lands on empty board — candidates X and Z produce **zero** courtyard overlaps.

So the failing quantity was the pad, not the corridor: **3.50 mm demanded vs 2.30 mm needed.**

## 4. Method (why this is exhaustive, not a 4730-point sample)

1. Snapshot copy taken once; every measurement below is from that file.
2. All copper reconstructed as exact shapes: every pad via Pad.GetEffectivePolygon(F_Cu/B_Cu)
   (3746 polygons), all 2464 tracks as capsules, all 254 vias as discs; 340 holes catalogued
   net-agnostically.
3. **Full-board scan**: the pad-centre plane is rasterised at 0.2 mm (415 x 290 = 120 350 cells) x 4
   rotations = **481 400 pad-1 positions**. Two clearance fields are built (distance to every net !=
   +24V, and to every net != GND, per layer, min over layers), then
   legal(pad1) AND legal(pad2 = pad1 + 15.24 mm along the axis) is intersected with edge,
   hole-to-hole, body-on-board and courtyard constraints. This covers the *entire* board.
4. **Exact re-verification** of every survivor on a live pcbnew board with D1 actually moved and its
   pads set to 2.00/1.30 circle, so the reported gaps come from the real pad polygons
   (segment-segment distances), not from the raster.
5. **Connectability by search, not by hope**: multi-source Dijkstra over both layers (0.2 mm grid,
   8-neighbour, via = 8-cell penalty, via legality incl. 0.30 drill hole-to-hole), seeded from all
   existing copper of the net and terminated on the pad. Target width +24V 0.80 mm (netclass), GND
   0.25 mm; if 0.80 fails the search retries 0.50 and 0.30 before declaring failure.
6. Mechanical check: D1 body = 9.5 x 5.2 mm rectangle centred between the pads along the pad axis;
   it must be on-board and no other F-side footprint may sit under it.

## 5. Ranked candidates (all numbers measured, all rules checked)

### Rank 1 — Z: (4.000, 1.500), rot 270   [recommended]

| item | value |
|---|---|
| pad 1 (+24V) | **(4.000, 1.500)** |
| pad 2 (GND) | **(4.000, 16.740)** |
| pads | 2.00 circle / drill 1.30 / **annular 0.35** (2.20/1.50 also fits here: gap stays >= 0.41) |
| clearance pad 1 | **3.898 mm** (min over F.Cu+B.Cu; nearest other-net copper: B.Cu /BUT5 track) |
| clearance pad 2 | **0.511 mm** (F.Cu /+24V_IN track — the unfused-input side; 0.36 mm above the 0.15 rule) |
| pad -> Edge.Cuts | pad 1 **0.505 mm**, pad 2 3.005 mm (both >= 0.30 hard rule **and** >= 0.50 board setting) |
| hole-to-hole (net-agnostic) | **3.105 mm** to nearest via (rule 0.25) |
| pth_inside_courtyard | **none** |
| D1 courtyard | x[1.15..6.85] y[-0.35..18.59]; overlaps J2: **NO**, U7: **NO**, any footprint: **NO** |
| board courtyard pairs | **34** with D1 here (baseline 36: D1's old J2+U7 pair disappears, nothing added) — strictly better than today under either way the 39 waiver is counted |
| D1 body | x[1.40..6.60] y[4.37..13.87], on-board, nothing under it |
| verified legal window | x 3.6..4.4, y 1.2..1.8 (~0.8 x 0.6 mm) — use the exact coordinate |

**Connection sketch**
* **+24V** — 9.71 mm, **0 vias**, F.Cu, 0.80 mm wide:
  pad1(4.000,1.500) -> (8.70, 6.10) -> (11.90, 6.10);
  **tie-in: existing +24V track at (11.90, 6.10)** (the C4/C5/C1 +24V cluster feed).
* **GND** — 14.44 mm, **2 vias**, 0.25 mm wide:
  pad2(4.000,16.740) -> F.Cu (5.10,17.70) -> F.Cu (8.30,17.70) -> VIA(8.30,17.70) ->
  B.Cu (10.30,19.70) -> B.Cu (10.30,21.30) -> VIA(10.30,21.30) -> F.Cu (8.10,21.30);
  **tie-in: GND pad R16.2 at (8.10, 21.30)**.

**Difficulty / risk: LOW-MEDIUM.** Zero new courtyard overlaps, all clearances above rule *and* above the
board's 0.5 mm edge setting, no pth-in-courtyard, via-free +24V. Only real risk: the tight legal window
(+-0.4 mm x, +-0.3 mm y) — place the exact coordinate, do not round to a 1 mm grid. GND takes 2 vias and
a 14 mm detour.

---

### Rank 2 — X: (4.000, 16.700), rot 90

| item | value |
|---|---|
| pad 1 (+24V) | **(4.000, 16.700)** |
| pad 2 (GND) | **(4.000, 1.460)** |
| pads | 2.00 circle / drill 1.30 / **annular 0.35** (2.20/1.50 also fits: 0.451 left) |
| clearance pad 1 | **0.551 mm** (F.Cu /+24V_IN track) |
| clearance pad 2 | **3.927 mm** (B.Cu /BUT5 track) |
| pad -> Edge.Cuts | pad 1 3.005 mm, pad 2 **0.465 mm** (>= 0.30 hard rule; 0.035 below the board's 0.5 setting) |
| hole-to-hole | **3.124 mm** |
| pth_inside_courtyard | **none** |
| D1 courtyard | x[1.15..6.85] y[-0.39..18.55]; J2 **NO**, U7 **NO**, any footprint **NO** |
| board courtyard pairs | **34** (strictly better than today) |
| D1 body | x[1.40..6.60] y[4.33..13.83], on-board, nothing under it |
| verified legal window | x 3.6..4.4, y 16.4..17.0 (~0.8 x 0.6 mm) |

**Connection sketch**
* **+24V** — 4.60 mm, **1 via**, 0.80 mm: pad1(4.000,16.700) -> F.Cu (7.10,16.70) -> VIA -> B.Cu;
  **tie-in: existing +24V B.Cu track at (7.10, 16.70)** — the (7.04,12.27)-(7.04,17.54) left-side riser.
* **GND** — 14.31 mm, **1 via**, 0.25 mm: pad2(4.000,1.460) -> F.Cu (7.70,5.10) -> F.Cu (13.50,5.10)
  -> VIA(13.50,5.10) -> B.Cu (13.90,5.30) -> B.Cu (14.90,6.10); **tie-in: GND pad C4.2 at (14.90, 6.10)**.

**Difficulty / risk: LOW-MEDIUM.** Same corridor quality as Z, shortest +24V stub of the two (4.6 mm, and
it lands on the +24V riser that already feeds the left side). One edge-clearance marker (pad 2 at
0.465 mm vs the board's 0.5 mm setting) — acceptable under the stated 0.30 mm rule; shifting the part
0.05 mm (y = 16.75) clears it at the cost of 0.05 mm pad-2 clearance.

---

### Rank 3 — W: (21.100, 2.700), rot 180

| item | value |
|---|---|
| pad 1 (+24V) | **(21.100, 2.700)** |
| pad 2 (GND) | **(5.860, 2.700)** |
| pads | 2.00 circle / drill 1.30 / **annular 0.35** (do **not** go to 2.20 here: 0.137 mm would break the rule) |
| clearance pad 1 | **0.237 mm** (B.Cu /BUT5 track) — 0.087 mm margin over the 0.15 rule |
| clearance pad 2 | **1.735 mm** (B.Cu /BUT5 track) |
| pad -> Edge.Cuts | 1.705 mm / 1.705 mm |
| hole-to-hole | **1.765 mm** |
| pth_inside_courtyard | **none** |
| D1 courtyard | x[4.01..22.95] y[-0.15..5.55]; J2 **NO**, U7 **NO**; overlaps **C4, J5** |
| board courtyard pairs | **36** (= today's 36; D1's 2 old pairs replaced by 2 new) |
| D1 body | x[8.73..18.23] y[0.10..5.30], on-board, **nothing under it** (0.10 mm from the top edge) |
| verified legal window | x 20.2..22.0, y **2.6..2.8** (~1.8 x 0.2 mm — tightest of the four) |

**Connection sketch**
* **+24V** — **0.85 mm, 0 vias**, 0.80 mm: pad1(21.100,2.700) -> F.Cu (20.50,2.10);
  **tie-in: existing +24V track at (20.50, 2.10)** — the (21.46,0.63)-(18.09,4.00) trunk segment.
  Essentially a direct connection; best surge node in the whole candidate set.
* **GND** — 12.01 mm, **1 via**, 0.25 mm: pad2(5.860,2.700) -> F.Cu (8.30,5.10) -> F.Cu (13.50,5.10)
  -> VIA(13.50,5.10) -> B.Cu (13.90,5.30) -> B.Cu (14.90,6.10); **tie-in: GND pad C4.2 at (14.90, 6.10)**.

**Difficulty / risk: MEDIUM.** Electrically the best (+24V 0.85 mm away), both pads far from the edge,
body clear — but the legal window is only 0.2 mm thick in y, pad 1 sits 0.237 mm from a /BUT5 track
(rule-compliant, thin), and it adds 2 courtyard overlaps (total stays at 36, so the 39 waiver is safe).

---

### Rank 4 — H: (6.500, 2.700), rot 0

| item | value |
|---|---|
| pad 1 (+24V) | **(6.500, 2.700)** |
| pad 2 (GND) | **(21.740, 2.700)** |
| pads | 2.00 circle / drill 1.30 / **annular 0.35** (not 2.20: 0.136 would remain) |
| clearance pad 1 | **1.282 mm** (B.Cu /BUT5 track) |
| clearance pad 2 | **0.236 mm** (B.Cu /BUT5 track) |
| pad -> Edge.Cuts | 1.705 mm / 1.705 mm |
| hole-to-hole | **1.130 mm** |
| pth_inside_courtyard | **none** |
| D1 courtyard | x[4.65..23.59] y[-0.15..5.55]; J2 **NO**, U7 **NO**; overlaps **C4, C5, J5** |
| board courtyard pairs | **37** (2 below the 39 waiver on my count; see caveat in section 7) |
| D1 body | x[9.37..18.87] y[0.10..5.30], on-board, **nothing under it**, flush with the top edge |
| verified legal window | x 6.4..7.4, y 2.6..2.8 |

**Connection sketch**
* **+24V** — 6.81 mm, **0 vias**, 0.80 mm: pad1(6.500,2.700) -> F.Cu (9.90,6.10) -> F.Cu (11.90,6.10);
  **tie-in: existing +24V track at (11.90, 6.10)**.
* **GND** — **1.60 mm, 0 vias**, 0.25 mm: pad2(21.740,2.700) -> F.Cu (23.30,2.70);
  **tie-in: GND pad J5.1 at (23.30, 2.70)**.

**Difficulty / risk: MEDIUM.** Cheapest routing of all (both nets via-free, 8.4 mm total), but three
courtyard overlaps (C4, C5, J5 — no part under the body, so courtyard margin only), the body sits
0.10 mm from the top edge, and the y-window is 0.2 mm.

---

## 6. Rejected placements and the proof (why the space is exhausted, not unexplored)

The scan enumerated **every** position; these near-misses look attractive and fail:

| candidate | why it fails (measured) |
|---|---|
| (16.700, 52.700) rot 180 — 3.3 mm from J1, gaps 0.746/2.737, edge 0.465, no pth issue | **+24V is unreachable.** Flood fill from pad 1 at 0.80 / 0.50 / 0.30 mm width covers a sealed pocket of ~2500 cells/layer, bbox x[0.4..15.8] y[33.6..57.4], with **0** +24V copper cells inside it. GND is fine (16.84 mm, 1 via). The whole bottom-left (everything near J1) is isolated from +24V by U5's pad row and the via wall at y ~ 30-37. |
| (5.300, 54.100) rot 0 / (3.700, 56.100) rot 90 / (4.100, 39.900) rot 270 / (4.000, 55.200) rot 90 | same sealed pocket -> **+24V unreachable (0 goal cells inside the reachable set)**. Otherwise excellent (e.g. (4.0,55.2): gaps 2.708/0.553, zero courtyard overlaps, 3 vias-free) — usable **only if +24V is deliberately extended into the bottom-left** (section 8, option B). |
| right-hand margin x = 81.70 (rot 90/270, y 19.7..56.1) | pad copper 0.374 mm from B.Cu /I2C_SCL and, on exact polygons, pad 1 **overlaps** it (-0.013 mm at (81.70,48.00)) -> short. Also only 0.300 mm to Edge.Cuts, and the 5.2 mm body would hang 1.35 mm over the board edge. |
| (78.800, 32.900) rot 270 / (78.800, 48.000) rot 90 — beside C28 and the +24V spine, very attractive electrically | with real circular pads, pad 1 gap becomes **0.065 mm** and **-0.013 mm** (short) to /INT_MCU resp. /I2C_SCL. Also 4 courtyard overlaps (D17, C32, C7, U12) and D17's pads sit directly under the body. |
| (21.700, 3.100) rot 270 (interim #3) | copper legal (gaps 0.455/0.938, +24V only 1.61 mm away) **but the body lands on C5.1, C5.2 and J4.1** (F side) and the courtyard swallows F1.2, J4.1, J4.2, J5.1 (pth_inside_courtyard x4) -> assembly clash. |
| (22.300, 18.900) rot 90 | body covers **J4.1 and J4.2** (a header); courtyard swallows 5 PTH pads -> clash. |
| (2.100, 5.700) rot 0 / (18.100, 7.300) rot 180 / (20.700, 54.100) rot 180 | body covers C4.1 / gap 0.138 mm / gap 0.124 mm (< 0.15 rule). |
| x = 0.4..1.3 strip, y 5..55 | pad would be < 0.30 mm from Edge.Cuts. |
| anything with x 44.1..66.9 and y 19.6..55.2 (under U7) | U7 courtyard is x[42.81..68.19] y[18.33..57.24] and is excluded by construction (D1 is 5.2 mm tall > the 5 mm allowed under the module). |

## 7. Caveat on the 39-courtyard waiver (read before committing)

My independent count of overlapping footprint-courtyard **pairs** on the untouched snapshot is **36**,
while the documented waiver says **39**; I could not reproduce 39 exactly (KiCad may split a pair into
several markers, or count hole/courtyard interactions separately). Consequences:

* **Z and X reduce** the count to 34 (D1's old J2 and U7 pairs disappear, nothing is added) -> safe under
  either counting.
* **W** keeps it at 36 (+3 worst case 39) -> safe, exactly at the limit if KiCad adds 3.
* **H** raises it to 37 (worst case 40) -> **verify with the project's own DRC before committing**; if it
  breaches 39, use W, X or Z.

Also: the board's copper_edge_clearance is **0.5 mm** in the .kicad_pcb design settings even though the
.kicad_pro rule says 0.2. Under the strict 0.5 reading, X's pad 2 (0.465 mm) will produce one marker;
Z, W and H are all >= 0.5.

## 8. How to implement (clean way to shrink the pads)

Recommended: **local library copy**, not an in-place board edit.

1. Add hardware/kicad/kamaz_leveler_hw.pretty/D_DO-201AD_P15.24mm_Horizontal_2mm.kicad_mod — a copy of
   the stock footprint with pad 1 changed from roundrect 3.2/1.6 to circle 2.0/1.3 and pad 2 from
   circle 3.2/1.6 to circle 2.0/1.3. The library nickname "kamaz_leveler" is **already registered** in
   fp-lib-table, so no table edit is needed.
2. Point D1 at kamaz_leveler:D_DO-201AD_P15.24mm_Horizontal_2mm.
3. Cost / consequences:
   * the footprint no longer tracks KiCad's stock DO-201AD library (accepted: the pad size is a
     deliberate choice and 3.2/1.6 is oversized for a 1.3 mm lead);
   * the **schematic** symbol D1 still carries footprint Diode_THT:D_DO-201AD_P15.24mm_Horizontal —
     update that field too, otherwise "Update PCB from Schematic" silently reverts the pad change;
   * lib_footprint_mismatch stays clean, because the board footprint then *matches* its library.
4. Cheaper alternative (what this study did in memory): edit the two pads directly in the board. Cost:
   lib_footprint_mismatch markers on both pads and the change is destroyed by any future "update
   footprints from library". Acceptable only with a documented DRC exclusion.
5. Then: delete the old D1 stubs — F.Cu (49.66,44.00)-(49.00,44.00) (+24V) and the B.Cu/F.Cu GND chain
   listed in section 1 — place D1 at the chosen coordinate/rotation, and route the two stubs per
   section 5.
6. Do section 1's off-board GND re-route **in the same pass**, otherwise the board stays unmanufacturable
   for a reason unrelated to D1.

Optional (electrically better, more work) — **option B**: keep the TVS near J1 by first extending the
+24V net down the left margin (a 0.4-0.5 mm B.Cu trace from the C2 / via (6.52, 24.94) area through
x ~ 1.5-2.5 up to y ~ 50). That unlocks the P-family placements at (4.0,55.2) / (3.7,56.1) /
(16.7,52.7), which are otherwise perfect (zero courtyard overlaps, 3.3 mm from J1). That is a net-change
decision, not a D1 placement decision, so it is out of scope here.

## 9. Risks noticed and routed

| risk | disposition |
|---|---|
| Off-board GND chain (section 1) is not D1-specific and survives a D1 move | raised as a **separate required fix**; do not close the D1 task on the strength of this study alone |
| My courtyard-pair count (36) != documented waiver (39) | quantified in section 7; Z/X can only reduce it, H is the only candidate that could breach it -> verify with project DRC, fall back to W/X/Z |
| Board copper_edge_clearance 0.5 mm contradicts .kicad_pro 0.2 mm | flagged; X's pad 2 at 0.465 mm is the only candidate affected, and the 0.30 mm hard rule is still met |
| Legal windows are 0.2-0.8 mm wide; a "roughly here" placement will fail DRC | each candidate gives the exact coordinate and its verified window; do not round to a 1 mm grid |
| 1.30 mm drill is tight for a 1.2-1.3 mm DO-201AD lead | 2.20 mm pad / 1.50 mm drill is also legal for **Z and X** (gap stays >= 0.41 mm); it is **not** possible for W/H (0.136 mm would remain) |
| Routing was searched at a 0.2 mm grid with 0.2 mm clearance margin | paths are feasibility proofs, not fab-ready copper; the human router must re-verify against DRC. Both nets exist; tie-in points and via positions are given |
