# ROUTE PLAN — the 9 unrouted nets (KAMAZ pneumatic-suspension controller)

Read-only analysis. No project file was modified by this work.

## 0. Snapshot analysed

| item | value |
|---|---|
| file | hardware/kicad/kamaz_leveler_hw.kicad_pcb, copied to C:\my\sch3\run_now\w1_C3\snapshot.kicad_pcb |
| size | 963,511 bytes |
| SHA-256 | 5f3021fe6c2752a59c5b006e64a9708436e8b5f443c3a20f0ba0c1190a981dff |
| taken | start of this session, before any analysis |
| board | 83.1 x 58.1 mm outline bbox (0..83 / 0..58 nominally), 2 copper layers (F.Cu, B.Cu), **0 copper zones**, 2,409 tracks, 117 footprints |

The on-disk board was already **958,763 bytes** when this plan was written — other agents are
editing it. Every number below comes from the snapshot above, not from the live file. Re-run the
geometry dump before acting on these coordinates.

## 1. Model, parameters and occupancy numbers

Raster model, two layers, 0.20 mm pitch (416 x 291 cells per layer, KiCad frame, Y grows down).

* Obstacles = every copper item of the *other* nets: track segments (capsule), vias (0.8/0.4), and
  pads as exact Minkowski sums (circle / capsule for oval+roundrect / rectangle).
* Track aperture = clearance + track width/2; via aperture = clearance + via diameter/2.
* Clearance margin assumed **0.20 mm** (netclass Default), track **0.25 mm**, via **0.80/0.40 mm** —
  taken from kamaz_leveler_hw.kicad_pro, net_settings.classes[0]. All 9 nets are on the Default
  class. Board limits: min via 0.5, min track 0.2, copper-edge clearance 0.5.
* Board-edge keepout 0.30 mm inside Edge.Cuts (brief constraint); H1..H4 keepout 1.46 mm radius.
* A* on 8 neighbours (45 and 90 degree moves), layer change cost 2 mm, **a via is allowed only where
  a 0.8 mm via aperture is clear on both layers** (and analogously for 0.6 / 0.5 mm in the sweeps).
* Every produced path was re-checked cell by cell against the masks: 0 illegal track cells,
  0 illegal vias in all reported results.

Board-wide occupancy at 0.20 mm clearance / 0.25 mm track, all 2,409 tracks and all pads as obstacles:

| layer | cells that admit a 0.25 mm track centreline |
|---|---|
| F.Cu | **41.5 %** |
| B.Cu | **37.0 %** |
| both layers + 0.8 mm via aperture | **13.4 %** |

Local occupancy inside each net's conflict box (endpoint bbox grown by 2 mm), same parameters —
this is the number the verdicts lean on:

| net | box (mm) | F free | B free | cells admitting a via (0.80 / 0.60 / 0.50) | direct distance |
|---|---|---|---|---|---|
| /BUT1 | 41.4 x 39.3 | 30.4 % | 33.1 % | 6.60 / 8.69 / 9.96 % | 51.45 |
| /VIN_FUSED | 31.6 x 14.4 | 35.3 % | 30.5 % | 8.41 / 10.09 / 11.76 % | 29.48 |
| /VLV_IN2 | 4.8 x 9.3 | 15.7 % | 22.2 % | **0.00** / 0.24 / 0.24 % | 5.38 |
| /VLV_IN3 | 5.4 x 8.0 | 12.6 % | 19.6 % | **0.00** / 0.08 / 0.08 % | 4.30 |
| /VLV_IN5 | 6.7 x 5.5 | 13.7 % | 15.9 % | **0.00 / 0.00** / 0.10 % | 3.12 |
| /ST_U3_34 | 40.4 x 4.7 | 49.5 % | 37.8 % | 11.61 / 15.57 / 17.06 % | 36.39 |
| /PRES_IN | 23.4 x 25.8 | 35.2 % | 29.6 % | 3.68 / 6.25 / 7.76 % | 29.17 |
| /VLV_DRV1 | 12.3 x 17.2 | 28.3 % | 30.8 % | 1.69 / 3.62 / 4.79 % | 15.56 |
| /VLV_DRV4 | 16.3 x 14.4 | 34.6 % | 31.6 % | 1.96 / 4.12 / 5.47 % | 16.08 |

The via column is the whole story: the U6 to U5 cluster has **no cell at all** where a 0.8 mm via
fits on both layers, and /VLV_IN5 has none even at 0.6 mm.

## 2. Result summary (measured, not estimated)

"FAIL" = no continuous legal path exists for that net at those parameters.

| net | 0.20 clr / 0.25 track / **0.80 via** (netclass) | 0.20 / 0.25 / 0.60 via | 0.15 / 0.20 / 0.60 via | minimum setting that closes it | difficulty |
|---|---|---|---|---|---|
| /ST_U3_34 | **PASS 78.55 mm, 6 vias** | PASS 51.58 mm, 6 vias | PASS 51.61 mm, 6 vias | netclass (0.8 via) | **easy-medium** |
| /PRES_IN | **PASS 69.90 mm, 8 vias** | PASS 55.27 mm, 6 vias | PASS 51.67 mm, 4 vias | netclass (0.8 via) | **medium** |
| /VLV_DRV4 | FAIL | FAIL | PASS 53.94 mm, 9 vias | 0.15 clr **plus** 0.6 via | hard |
| /VLV_DRV1 | FAIL | FAIL | PASS 55.03 mm, 3 vias | 0.15 clr **plus** 0.6 via | hard |
| /BUT1 | FAIL | PASS 119.84 mm, 9 vias (pathological) | PASS 111.96 mm, 7 vias | 0.6 via at 0.20 clr | **hard — no practical corridor** |
| /VIN_FUSED | FAIL | FAIL | FAIL | 0.125 clr (95 mm) | **hard — not closable** |
| /VLV_IN2 | FAIL | FAIL | FAIL | 0.10 clr + 0.5 via (8.24 mm) | **not closable** |
| /VLV_IN3 | FAIL | FAIL | FAIL | 0.10 clr + 0.5 via (8.04 mm) | **not closable** |
| /VLV_IN5 | FAIL | FAIL | FAIL | nothing down to 0.05 clr / 0.5 via | **not closable** |

**The brief's premise is inverted.** /ST_U3_34 and /PRES_IN — two of the four nets called hard — are
the only two that route at the *netclass* clearance and via size. The genuinely hard ones are
/VIN_FUSED (needs clearance <= 0.125 mm, and even then an 81-95 mm detour for a 29.5 mm net) and the
three /VLV_IN* stubs (no via-capable cell anywhere between U6 and U5).

### One physical corridor, three claimants

/ST_U3_34, /PRES_IN and /BUT1 want the same copper lanes:

* lane N: y = 34.6..36.8, x 43.0..56.8 — measured as a 0.25 mm centreline band with 0.20 mm to
  spare on each side, i.e. **one track only**, not two;
* lane V: x = 57.8..61.0, y 22.0..35.0 — also single-lane, and /PRES_IN and /BUT1 both need it.

At the netclass setting only /ST_U3_34 can be given an alternative (it detours around the bottom
edge at y = 55..57, 78.55 mm). So the simultaneous feasible plan at netclass parameters is
**/ST_U3_34 via the bottom edge + /PRES_IN via lanes N and V**; /BUT1 has nowhere to go. Any plan
that puts /BUT1 through lane V steals it from /PRES_IN, and /PRES_IN then has no corridor.

Coordinates below are quantised to the 0.20 mm analysis grid; they must be re-entered and
re-verified in KiCad (they are corridors, not a finished board).

---

## 3. Per-net plans

### 3.1 /BUT1 — 9.62,38.31 (U6.15, F.Cu) to 47.04,3.00 (J3.2, F.Cu)

Two islands: U6.15 alone in the bottom-left pocket, and one long body from J3.2 through
(55.29,9.07)/(58.38,17.65) to C23.1 (60.85,17.55) and R29.2 (56.68,6.85). Straight distance 51.5 mm.

**Corridor on the current placement at netclass: NO.** The flood from U6.15 reaches only 7,850
cells (the bottom-left pocket); only 6.6 % of its conflict box can host a via. It needs a 0.6 mm via
to escape at all, and even then the only path is 111.96 mm (0.15 clr) / 119.84 mm (0.20 clr) —
2.3x the direct distance, around the bottom edge and up lane V, which /PRES_IN also needs.

Waypoints (0.20 clr / 0.25 track / 0.60 via, the shortest legal variant, 119.84 mm, 9 vias):

| # | layer | waypoint / via |
|---|---|---|
| 1 | F.Cu | start U6.15 (9.62,38.31) -> (9.60,39.20) -> (8.20,40.60) -> (7.40,41.80) -> (7.20,42.60) -> (7.20,51.80) |
| 2 | via | **(7.20,51.80)** 0.60 / 0.30 |
| 3 | B.Cu | (7.20,51.80) -> (12.40,56.80) -> (13.00,57.20) -> (28.80,57.20) -> (29.20,56.80) -> (42.40,56.80) -> (42.60,57.00) -> (44.00,57.00) -> (45.40,55.60) -> (47.80,53.00) |
| 4 | via | **(47.80,53.00)** |
| 5 | F.Cu | (47.80,53.00) -> (49.40,53.00) -> (53.20,49.20) -> (53.20,48.40) |
| 6 | via | **(53.20,48.40)** |
| 7 | B.Cu | (53.20,48.40) -> (53.20,42.40) |
| 8 | via | **(53.20,42.40)** |
| 9 | F.Cu | (53.20,42.40) -> (53.20,41.80) -> (56.00,39.00) |
| 10 | via | **(56.00,39.00)** |
| 11 | B.Cu | (56.00,39.00) -> (56.00,36.20) |
| 12 | via | **(56.00,36.20)** |
| 13 | F.Cu | (56.00,36.20) -> (56.80,35.80) -> (58.00,35.00) |
| 14 | via | **(58.00,35.00)** |
| 15 | B.Cu | (58.00,35.00) -> (58.00,31.80) -> (59.80,30.00) -> (60.00,28.40) -> (60.20,28.20) |
| 16 | via | **(60.20,28.20)** |
| 17 | F.Cu | (60.20,28.20) -> (61.00,27.20) -> (61.00,23.80) -> (59.20,22.00) |
| 18 | via | **(59.20,22.00)** |
| 19 | B.Cu | (59.20,22.00) -> (59.20,18.60) -> (58.80,18.20) -> joins the existing /BUT1 B.Cu at via (58.38,17.65) |

Must avoid: the whole J1/J2 pad row at y = 56.00 (1.70 mm pads on 2.54 mm pitch, 0.84 mm gaps —
impassable at 0.2 clearance, which is why the track hugs y = 57.00-57.20, only 0.30-0.35 mm inside
Edge.Cuts); C14 (9.75,53.35); C19 (13.29,51.57); U3 (29.00,48.00) and its 0.19 mm fan-out;
D7 (31.20,48.22); C13 (31.85,52.10); R25 (71.75,47.73); U7 pad column at x = 66.94; +24V 0.80 mm
B.Cu at x = 63.39; and /PRES_IN's corridor once /PRES_IN is routed.

**Difficulty: hard, no practical corridor.** The bottom-left pocket is sealed by the U6 QFN pad
fence, U2/D5/D14 and 0.19-0.25 mm fan-out copper; the only escape is a 0.6 mm via, and the remaining
route is 2.3x the direct length through a lane another net needs.

### 3.2 /VIN_FUSED — 52.30,25.40 (L4.1, F.Cu) to 24.70,15.04 (F1.2, F.Cu + B.Cu pads)

The net is almost entirely unrouted: L4.1, the two F1 pad-2 sites at (21.30,15.04) / (24.70,15.04)
and one 0.80 mm F.Cu track between them. Straight distance 29.5 mm across the DC-DC area.

**Corridor on the current placement at netclass: NO.** Measured threshold: no path at 0.15 mm
clearance with a 0.6 mm via; a path appears only at **clearance <= 0.125 mm**, 95.0 mm long
(88.2 mm at 0.10, 81.4 mm at 0.05). The flood from L4.1 covers 50k+ cells but never reaches F1.2:
the F1.2 pad cage is closed to within 0.13 mm on the north (the +5V F.Cu track at y = 12.88, 0.50 mm
wide, is 0.52 mm from the pad edge where 0.65 mm is needed) and to within 0.21 mm on the south (the
/VLV_DRV6 track at y = 22.68 and the /VLV_EN track at y = 23.14 are 0.46 mm apart centre to centre).

Only route found (0.125 mm clearance, not legal at netclass): from L4.1 west/south into the free
pocket at x 20..36, y 16.8..22.1, then north-west to F1.2 — the pocket is entered only through the
0.46 mm slot between /VLV_DRV6 (y = 22.68) and /VLV_EN (y = 23.14), or the 0.52 mm slot at y = 12.88.

Must avoid: L3.1 pad (/SW2, 3.50 x 10.20 mm at 29.23,16.52 — the largest obstacle in the corridor);
F1's own 2.78 mm pad sites; /+24V_IN 0.80 mm F.Cu (13.69,21.18)-(19.71,21.18) and its via
(19.71,21.18); /I2C_SCL B.Cu at x = 18.96; /GND B.Cu at x = 17.69; +3V3 B.Cu at x = 16.71;
/SW2 F.Cu (25.42,18.25)-(29.23,16.52); +12V 0.60 mm B.Cu at x = 26.99; +3V3 B.Cu at x = 26.23 and
26.49; U4 (33.19,21.15) and its fan-out.

**Difficulty: hard — not closable under the stated constraints.** F1 cannot be reached without
violating the 0.2 mm netclass clearance, and the route even then is 3x the direct distance.

### 3.3 /ST_U3_34 — 31.91,39.20 (existing via, F.Cu) to 68.29,38.51 (existing via, B.Cu)

**Corridor on the current placement: YES, at netclass parameters.** 78.55 mm, 6 vias (direct 36.39 mm;
the 2.2x factor is the bottom-edge detour that keeps clear of lanes N and V). With 0.6 mm vias the
same net closes in 51.58 mm through lane N — but that lane is then unavailable to /PRES_IN.

Recommended (netclass, 0.80 mm vias, does not compete for lane N), 78.55 mm, 6 vias:

| # | layer | waypoint / via |
|---|---|---|
| 1 | F.Cu | existing via (31.91,39.20) -> (32.00,39.60) |
| 2 | via | **(32.00,39.60)** 0.80 / 0.40 |
| 3 | B.Cu | (32.00,39.60) -> (32.00,47.40) -> (29.40,50.00) |
| 4 | via | **(29.40,50.00)** |
| 5 | F.Cu | (29.40,50.00) -> (30.60,51.20) -> (30.80,53.80) -> (30.40,54.20) |
| 6 | via | **(30.40,54.20)** |
| 7 | B.Cu | (30.40,54.20) -> (32.80,55.20) -> (40.80,55.20) -> (42.60,57.00) -> (44.00,57.00) -> (45.40,55.60) -> (47.40,52.80) |
| 8 | via | **(47.40,52.80)** |
| 9 | F.Cu | (47.40,52.80) -> (49.60,52.80) -> (53.20,49.20) -> (53.40,48.60) |
| 10 | via | **(53.40,48.60)** |
| 11 | B.Cu | (53.40,48.60) -> (53.40,43.40) -> (53.60,43.20) |
| 12 | via | **(53.60,43.20)** |
| 13 | F.Cu | (53.60,43.20) -> (58.00,38.80) -> (58.40,38.60) -> (67.60,38.60) -> existing via (68.29,38.51) |

Short variant (0.20 clr / 0.20 track / 0.60 via, 51.58 mm, 6 vias, uses lane N):
(32.00,39.20) -> (35.80,37.60) [via] -> B (38.80,34.60) [via] -> F (42.00,34.60) -> (43.40,35.80)
-> (44.80,36.00) [via] -> B (49.80,36.00) -> (50.60,36.80) [via] -> F (55.40,36.80) -> (56.00,36.20)
[via] -> B (57.60,37.60) [via] -> F (65.40,37.60) -> (66.20,38.40) -> (67.60,38.40).

Must avoid: **U7's right-hand pad column at x = 66.94 (rows y = 37.43 and y = 39.97)** — the corridor
squeezes between those two pads at y = 38.60, 0.32 mm to copper; D12 (38.89,38.94); C22 (31.87,33.49);
U3 (29.00,48.00) and its fan-out; C12 (24.64,49.42); D10 (39.22,45.21); D8 (39.24,50.40);
R26 (38.82,41.08); R24 (40.91,39.26); D6/D9 (49.25 / 56.65, 32.15); the J2 pad row at y = 56.00
(corridor runs at y = 55.20-57.00); /VALVE_DEFL 0.80 mm B.Cu (62.24,56.00)-(52.56,46.32); and the
existing /ST_U3_34 B.Cu near R34.2 (70.90,31.72).

**Difficulty: easy-medium.** A netclass-legal corridor exists; the only real constraint is that it
must not use lane N (needed by /PRES_IN) unless /PRES_IN is rerouted.

### 3.4 /PRES_IN — 40.09,36.88 (R42.1, B.Cu) to 59.51,15.12 (existing via, B.Cu)

**Corridor on the current placement: YES, at netclass parameters.** 69.90 mm, 8 vias (direct 29.17 mm).
With 0.6 mm vias: 55.27 mm, 6 vias; at 0.15 clr: 51.67 mm, 4 vias.

Recommended (netclass, 0.80 mm vias), 69.90 mm, 8 vias:

| # | layer | waypoint / via |
|---|---|---|
| 1 | B.Cu | R42.1 (40.09,36.88) -> (41.00,35.60) -> (41.00,34.20) |
| 2 | via | **(41.00,34.20)** 0.80 / 0.40 |
| 3 | F.Cu | (41.00,34.20) -> (43.20,35.80) -> (43.60,36.00) -> (44.80,36.20) |
| 4 | via | **(44.80,36.20)** |
| 5 | B.Cu | (44.80,36.20) -> (45.60,36.00) -> (49.60,36.00) -> (50.60,37.00) |
| 6 | via | **(50.60,37.00)** |
| 7 | F.Cu | (50.60,37.00) -> (55.60,37.00) -> (57.40,35.80) -> (58.00,35.20) |
| 8 | via | **(58.00,35.20)** |
| 9 | B.Cu | (58.00,35.20) -> (58.00,31.60) -> (59.00,30.60) |
| 10 | via | **(59.00,30.60)** |
| 11 | F.Cu | (59.00,30.60) -> (61.60,32.80) -> (64.00,32.80) -> (64.40,33.20) |
| 12 | via | **(64.40,33.20)** |
| 13 | B.Cu | (64.40,33.20) -> (64.40,30.00) |
| 14 | via | **(64.40,30.00)** |
| 15 | F.Cu | (64.40,30.00) -> (61.20,27.40) -> (61.00,23.80) -> (59.20,22.00) |
| 16 | via | **(59.20,22.00)** |
| 17 | B.Cu | (59.20,22.00) -> (59.20,17.20) -> (59.00,16.20) -> (59.40,15.80) -> existing via (59.51,15.12) |

Short variant (0.20 / 0.25 / 0.60 via, 55.27 mm, 6 vias): (40.09,36.88) -> (41.00,35.60) ->
(41.00,34.20) [via] -> F (43.20,35.80) -> (44.80,36.00) [via] -> B (49.80,36.00) -> (50.60,36.80)
[via] -> F (55.80,36.80) -> (58.00,35.00) [via] -> B (58.00,31.80) -> (60.00,28.40) -> (60.20,28.20)
[via] -> F (61.00,27.20) -> (61.00,23.80) -> (59.20,22.00) [via] -> B (59.20,17.20) -> (59.40,15.80).

Must avoid: **U7's right column at x = 66.94** (the y = 32.80-33.20 hop passes 0.29 mm from U7.7/12);
C33 (41.91,36.96) and R42/R41 (41.00, 36.88 / 35.03) — the start sits in a 0.25 mm channel between
them; C35 (40.54,26.78); R38 (40.75,26.72); R23 (41.27,30.17); C30 (39.70,30.12); D11 (39.02,32.37);
D18 (38.10,36.46); R36 (39.10,32.87); +24V 0.80 mm B.Cu at x = 63.39 (y 27.43-36.56); /DISP_BL F.Cu
(57.52,19.28)-(64.09,25.86); C23 (60.85,16.60); C25 (60.38,12.91); R32 (62.16,14.77); R21
(65.93,12.48); U11 (64.66,16.00).

**Difficulty: medium.** Netclass-legal, but it owns lanes N and V uniquely; if /BUT1 is routed,
/PRES_IN loses its corridor.

### 3.5 /VLV_DRV1 — 25.82,43.13 (U3.3, B.Cu) to 17.52,29.97 (R11.1) and via 17.07,28.65

Islands: U3.3 alone, and the long body from R11.1 through (15.31,25.71), (11.05,23.06), (7.61,26.50)
to U5.18 (4.35,27.26). Missing link 15.6 mm.

**Corridor at netclass: NO** (1.69 % of its box can host a 0.80 mm via). Closes only at
**0.15 clr + 0.60 via**: 55.03 mm, 3 vias (3.5x direct).

| # | layer | waypoint / via |
|---|---|---|
| 1 | B.Cu | U3.3 (25.82,43.13) -> (25.80,42.60) -> (25.20,42.00) |
| 2 | via | **(25.20,42.00)** 0.60 / 0.30 |
| 3 | F.Cu | (25.20,42.00) -> (23.40,42.00) -> (22.20,42.60) -> (18.20,42.60) -> (17.00,43.80) -> (17.00,48.80) -> (16.40,49.40) -> (16.00,49.60) -> (12.60,49.60) -> (11.80,48.80) |
| 4 | via | **(11.80,48.80)** |
| 5 | B.Cu | (11.80,48.80) -> (7.80,44.80) -> (7.60,44.00) -> (7.40,41.80) -> (3.00,37.40) -> (3.00,36.00) -> (3.40,35.60) -> (5.20,35.60) -> (5.80,34.40) -> (5.60,34.20) |
| 6 | via | **(5.60,34.20)** |
| 7 | F.Cu | (5.60,34.20) -> (5.20,33.60) -> (5.20,29.40) -> (6.80,27.80) -> joins the existing /VLV_DRV1 F.Cu at (6.90,27.20) |

Must avoid: C20 (22.86,42.26); R22 (16.85,44.97); **C6.2 GND pad 2.56 x 3.57 at (15.45,47.55)** —
the corridor threads between C6 and the J1 pads; C19 (13.29,51.57); C14 (9.75,53.35); H3 (10.70,43.00);
U5 pads .13-.17 on B.Cu at x = 4.35; C2 (7.77,26.64) GND pad 3.50 x 2.50; R16 (6.77,20.76); the board
edge (x = 3.00 is 0.30 mm inside Edge.Cuts — the tightest point of the whole plan).

**Difficulty: hard.** Not routable at netclass; needs 0.15 mm clearance, a 0.6 mm via, a 3.5x detour
around the left edge, and it passes 0.30 mm from Edge.Cuts.

### 3.6 /VLV_DRV4 — 33.45,43.13 (U3.9, B.Cu) to 21.15,32.78 (R14.1) and via 7.36,31.62

Islands: the U3.9 side (reaching R14.1 at 21.15,32.78) and U5.15 (4.35,31.07) -> (7.36,31.62).
Missing link 16.1 mm.

**Corridor at netclass: NO** (1.96 % via-capable). Closes at **0.15 clr + 0.60 via**: 53.94 mm, 9 vias.

| # | layer | waypoint / via |
|---|---|---|
| 1 | B.Cu | U3.9 (33.45,43.13) -> (33.00,43.20) |
| 2 | via | **(33.00,43.20)** 0.60 / 0.30 |
| 3 | F.Cu | (33.00,43.20) -> (31.40,43.20) -> (29.40,45.20) -> (27.20,45.20) -> (27.00,45.00) |
| 4 | via | **(27.00,45.00)** |
| 5 | B.Cu | (27.00,45.00) -> (26.00,45.00) -> (25.20,44.20) |
| 6 | via | **(25.20,44.20)** |
| 7 | F.Cu | (25.20,44.20) -> (23.40,42.40) -> (22.20,42.60) -> (20.80,42.60) -> (20.60,42.80) |
| 8 | via | **(20.60,42.80)** |
| 9 | B.Cu | (20.60,42.80) -> (18.00,40.20) |
| 10 | via | **(18.00,40.20)** |
| 11 | F.Cu | (18.00,40.20) -> (16.40,38.60) |
| 12 | via | **(16.40,38.60)** |
| 13 | B.Cu | (16.40,38.60) -> (16.80,38.00) -> (16.80,36.20) -> (15.80,35.20) |
| 14 | via | **(15.80,35.20)** |
| 15 | F.Cu | (15.80,35.20) -> (14.20,36.40) -> (12.60,36.40) -> (11.40,36.60) |
| 16 | via | **(11.40,36.60)** |
| 17 | B.Cu | (11.40,36.60) -> (7.60,33.00) -> existing via (7.36,31.62) |

Must avoid: C20 (22.86,42.26); D4 (16.83,44.42); R22 (16.85,44.97); /BUT4 F.Cu (11.57,37.61)-(8.82,34.86);
U6.13/.14/.15 pads at y = 38.31 (x 8.32-9.62); /VLV_DRV2 B.Cu (8.19,30.89)-(11.54,34.25);
/VLV_DRV3 B.Cu (7.53,29.04)-(12.36,29.16); U5.15/.16 B.Cu pads at x = 4.35; R14 (22.06,32.78);
R13 (17.34,31.91).

**Difficulty: hard.** Nine vias in 54 mm through the most congested corner of the board, and
0.15 mm clearance required.

### 3.7 /VLV_IN2 — 12.87,32.58 (U6.5, F.Cu) to 13.65,27.26 (U5.3, B.Cu)

**Corridor at netclass: NO. Not closable at any sane setting** — the bridge appears only at clearance
0.10 mm with a 0.50 mm via, and its via-capable share is **0.00 %** at 0.80 and 0.24 % at 0.60.
Best path found (illegal at netclass), 8.24 mm at 0.10 clr / 0.20 track / 0.50 via:

| # | layer | waypoint / via |
|---|---|---|
| 1 | F.Cu | U6.5 (12.87,32.58) -> (12.80,30.80) -> (15.20,28.40) |
| 2 | via | **(15.20,28.40)** 0.50 / 0.25 |
| 3 | B.Cu | (15.20,28.40) -> (14.60,27.60) -> U5.3 (13.65,27.26) |

Must avoid (this is why it fails): U6's own pad fence — 0.40 mm wide bars on 0.65 mm pitch, i.e.
0.25 mm gaps, impassable even at 0.05 mm clearance with a 0.20 mm track; /VLV_IN6 0.80 mm B.Cu
(13.65,32.34)-(11.03,32.34) and its via (10.29,31.22); /VLV_IN4 0.80 mm B.Cu (11.40,31.32)-(12.18,30.53)
and its via (11.40,31.32); /GND F.Cu (13.76,33.71)-(8.69,33.71); /VLV_DRV2 B.Cu (8.19,30.89)-(11.54,34.25);
/VLV_IN1 0.80 mm F.Cu (13.52,31.39)-(15.97,28.95) with via (15.97,28.21); /VLV_DRV3 B.Cu at
y = 29.04 / 29.16; /ST_U4_34 F.Cu (15.42,27.44)-(12.73,30.14); /ST_U3_12 F.Cu (12.88,29.10)-(15.16,26.82);
R11.2 GND pad (15.70,29.97).

**Difficulty: not closable.** The pad must cross a 2.4 mm band holding /VLV_IN6, /VLV_IN4, /GND,
/VLV_DRV2 and /VLV_IN1 copper, and there is no 0.60 mm via site anywhere in the 8 mm box (0.24 % of
cells, all of them outside the pocket).

### 3.8 /VLV_IN3 — 12.22,32.58 (U6.6, F.Cu) to 13.65,28.53 (U5.4, B.Cu)

Same verdict as /VLV_IN2: **not closable** (0.00 % via-capable at 0.80, 0.08 % at 0.60). Best
illegal path 8.04 mm at 0.10 clr / 0.50 via:

| # | layer | waypoint / via |
|---|---|---|
| 1 | F.Cu | U6.6 (12.22,32.58) -> (12.20,31.40) -> (15.20,28.40) |
| 2 | via | **(15.20,28.40)** 0.50 / 0.25 |
| 3 | B.Cu | (15.20,28.40) -> (14.60,28.40) -> U5.4 (13.65,28.53) |

Must avoid: the same set as /VLV_IN2, plus U6.5 (12.87) and U6.7 (11.57) either side of it, and
U5.5 (/VLV_IN4) directly north of the target pad at (13.65,29.80).

### 3.9 /VLV_IN5 — 10.92,32.58 (U6.8, F.Cu) to 13.65,31.07 (U5.6, B.Cu)

**Corridor at netclass: NO. Not closable at all** — no path exists at 0.15, 0.125, 0.10, 0.075 or
0.05 mm clearance with 0.5-0.8 mm vias. The flood from U6.8 reaches **10 cells**; the pad is boxed:

* north (y < 31.84, the pad tip): /VLV_IN6 via (10.29,31.22) and /VLV_IN4 via (11.40,31.32) are
  1.11 mm apart — a 0.80 mm via pair needs 1.45 mm, so there is **no** slot;
* east/west: U6.7 (11.57) and U6.9 (10.27) — 0.25 mm gaps;
* south (y > 33.32, the pad tip): /GND F.Cu (13.76,33.71)-(8.69,33.71) leaves 0.265 mm where
  0.325 mm is needed — short by **0.06 mm**;
* via-capable cell share in the box: 0.00 % at both 0.80 and 0.60 mm vias.

Any one of these makes it routable (none is a component move):
1. move the /GND F.Cu track from y = 33.71 to y >= 34.30 over x 9.5..12.0 (frees a 0.63 mm slot;
   then a via near (11.0,33.60) and a B.Cu run east to U5.6), or
2. move the /VLV_IN4 via from (11.40,31.32) to (12.20,31.30) and swing its 0.40 mm F.Cu stub with it
   (pair separation becomes 1.91 mm > 1.45 mm), or
3. accept 0.30 mm clearance locally (not proposed — the board is already at 0.2).

**Difficulty: not closable.** Fully enclosed pad: the enclosure is 0.06 mm short on the south side
and via-size limited on the north side.

---

## 4. The four "hard" nets — explicit verdicts

| net | corridor on the current placement? | coordinates | component move needed? |
|---|---|---|---|
| /ST_U3_34 | **YES at netclass** | §3.3, 78.55 mm / 6 vias (bottom-edge detour); or 51.58 mm via lane N with 0.6 mm vias | **no** |
| /PRES_IN | **YES at netclass** | §3.4, 69.90 mm / 8 vias | **no** |
| /BUT1 | **NO at netclass.** Exists only with 0.6 mm vias, 111.96-119.84 mm, and it collides with /PRES_IN in lane V | §3.1 | **no move helps** (see §5) |
| /VIN_FUSED | **NO** — needs clearance <= 0.125 mm, then 81-95 mm for a 29.5 mm net | none at netclass | **no move helps** (see §5) |

So **2 of the 4 have a netclass-legal corridor on today's placement and need no relocation at all,
and the other 2 cannot be fixed by relocating one component.** The binding constraints are the
0.20 mm netclass clearance, the 0.80 mm via, and the 0.8 mm-wide power copper already routed through
both channels.

## 5. Component moves proposed: **0 of the allowed 4**

Screen: for each net, every footprint whose origin lies in the conflict window had its pads removed
from the obstacle model, and the nominal search was re-run. Tracks of that part were left in place,
which is more conservative than a real move + re-route.

| net | parts screened | parts whose removal opens a netclass corridor |
|---|---|---|
| /VIN_FUSED | 10 (L3, U4, D3, C35, R38, R23, C30, D11, R15, R36 window) | none |
| /VLV_IN2 | 5 (U5, U6, C2, R13, R11 window) | none |
| /VLV_IN3 | 5 | none |
| /VLV_IN5 | 5 | none |
| /BUT1 | 14 (U2, D5, C22, C20, R10, R9, C10, U3, D7, D12, C12 and others) | none |

A stronger test: removing **all seven** named blockers around F1 together (the +5V track at y = 12.88,
the /+24V_IN 0.8 mm track and its via, the /VLV_DRV6 track, the /VLV_EN track, the /GND B.Cu at
x = 17.69, the /I2C_SCL B.Cu at x = 18.96) still does **not** open /VIN_FUSED at 0.20 mm clearance —
the 0.13 mm deficit is distributed, not caused by one part. Eight named blockers around U6.8 removed
together also do not open /VLV_IN5.

Candidates examined and rejected (for the record; 4 or fewer kept, here 0):

| candidate | why rejected |
|---|---|
| U6 (11.90,35.44) | its top pad row *is* the wall; shifting it +-2 mm moves the row into the /VLV_DRV2 diagonal or under the /GND track, and 24 routed nets would need re-routing |
| U5 (9.00,30.44, 1.27 mm pitch) | any shift >= 1.27 mm displaces all 20 pads by a full pitch; ~14 nets re-routed |
| F1 (24.70,24.96) | 10.4 mm long; the only free space is the x 36-40 DC-DC block, and /+24V_IN would need a 12 mm re-route through it |
| L3 (32.53,16.52) | 3.50 x 10.20 mm pads; moving it +-4 mm in Y fills more of the y 16-22 pocket than it frees |

**What actually fixes the hard nets** (copper-level, no relocation):

1. Set the Default netclass clearance 0.20 -> 0.15 mm and the Default via 0.80/0.40 -> 0.60/0.30 (the
   +12V, +24V and +3V3 classes on this board already use 0.6/0.3). That alone closes /VLV_DRV1
   (55.03 mm) and /VLV_DRV4 (53.94 mm) and shortens /ST_U3_34 to 51.61 mm and /PRES_IN to 51.67 mm.
   It does **not** close /VIN_FUSED or any /VLV_IN*.
2. /VIN_FUSED needs a deliberate decision: either ~0.125 mm clearance (not recommended for a 24 V
   rail), or move the +5V F.Cu track at y = 12.88 north by 0.5 mm **and** separate /VLV_DRV6 (y = 22.68)
   from /VLV_EN (y = 23.14) by a further 0.5 mm **and** re-route L3's /SW2 copper — three separate
   re-routes, each insufficient alone.
3. /VLV_IN5 needs 0.06 mm of extra room south of the U6 top row (item 1 or 2 in §3.9).

## 6. Caveats

* Model, not DRC: the plan proves *geometric* corridors under the stated aperture model. Pads are
  Minkowski sums (circle / capsule / rectangle); real KiCad rounded-rect corner radii and custom pad
  shapes can differ by a few hundredths of a millimetre. Re-check with kicad-cli pcb drc after routing.
* The board changed on disk during this analysis (963,511 -> 958,763 bytes); the plan is against the
  snapshot in §0.
* Tracks already belonging to a net are free space for that net (correct, same net) and obstacles for
  every other net.
* Endpoints that are pads were taken at their pad centre, so corridors start at the pad rather than at
  a preferred escape direction.
