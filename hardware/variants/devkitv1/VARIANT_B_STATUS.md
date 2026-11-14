# VARIANT B — ESP32 DevKit V1 Type-C (30-pin) — FINAL STATUS
Project: C:/my/for cursor/kamaz_leveler/hardware/variants/devkitv1 (kamaz_leveler_devkitv1)
Base project C:/my/for cursor/kamaz_leveler/hardware/kicad was read-only throughout (mtimes unchanged).

## 1. ACCEPTANCE RESULT
| # | Criterion | Result |
|---|-----------|--------|
| 1 | DRC unconnected=0, violations=0, shorting/clearance/track_width/copper_edge = 0 | PASS 0/0/0/0/0/0 |
| 2 | U7 = 30-pad 25.4 mm socket at the settled placement; antenna area empty | PASS (see 3) |
| 3 | 12 signals + pad15/pad30/pad14/pad29 per the assignment table | PASS (full dump in 4) |
| 4 | Copper changed only on U7's nets + U12's/C35's nets | PASS — 17 nets differ, 0 outside the allowed set, 53 nets identical |
| 5 | Refdes intact, ERC=0, BOM regenerated | PASS 120 footprints, ERC 0, BOM 46 rows |
| 6 | Netclasses + all 10 rule_severities waivers present | PASS (see 8) |

Final DRC command (run in the project directory, so schematic parity is included):
  kicad-cli pcb drc --format json --severity-all --units mm kamaz_leveler_devkitv1.kicad_pcb
  -> 0 violations, 0 unconnected items.

## 2. R43 (received mid-task)
Removed from the board, the schematic and the BOM. Footprints 121 -> 120.
Justification: the DevKit V1 Type-C carries its own 10k + 1 uF reset RC, so the C3-era pull-up is
redundant, and pad 1 (EN) must be left unconnected. U7 pad 1 carries no net and the schematic has an
explicit no-connect at the EN pin (72.39,251.46); the net Net-(U7-EN) and all 5 of its copper items are
gone. R43's +3V3 stub and the now-orphan +3V3 power symbol that fed it were deleted as well.

## 3. U7 placement and antenna keepout
kamaz_leveler:ESP32-DevKitV1-TypeC_Socket_2x15_P2.54mm_Row25.4mm
  position (61.640, 15.500), orientation 0 deg, 30 THT pads, pitch 2.540, row spacing 25.400,
  drill 1.00, pad 1.70, value ESP32-DevKitV1-TypeC.
  pad 1 at (97.200,15.500); antenna end of the module at x = 103.00 (courtyard to 103.50), i.e. the
  module is inset 1.00 mm from the x = 104 board edge and its courtyard is inside the outline.
  NOTE: the brief quoted both 'position (61.64,15.50) / pad1 at (97.20,15.50)' and 'antenna edge at
  x=103.20'; those differ by 0.20 mm (61.64 + 41.36 = 103.00). The stated POSITION was kept, so pad 1
  is at 97.20 as stated and the antenna edge lands at 103.00.

Antenna keepout box (98.20, 13.00) - (104.00, 43.50), i.e. from 1.00 mm outside the pad-1/pad-16
column to the board edge, spanning the full module width:
  track segments intersecting the box ....... 0
  vias in the box .......................... 0
  GND zone fill sample points inside box ... 0 on F.Cu, 0 on B.Cu  (0.5 mm grid over the box)
  footprints with origin in the box ........ 0
The GND zone notch (95.5..103.5 x 10.5..43.5) was already wide enough and was left unchanged.

## 4. U7 pad -> net dump (all 30 pads)
  1 <no net> (EN)        2 /PRES_SW (GPIO36)   3 <no net>            4 /SPEED_PULSE (GPIO34)
  5 /INT_MCU (GPIO35)    6 /DISP_BL (GPIO32)   7 <no net>            8 /DISP_RES (GPIO25)
  9 /DISP_DC (GPIO26)   10 /VLV_EN (GPIO27)   11 <no net>           12 <no net>
 13 <no net>            14 GND                15 +5V (VIN)         16 /DISP_MOSI (GPIO23)
 17 /I2C_SCL (GPIO22)   18 <no net>           19 <no net>          20 /I2C_SDA (GPIO21)
 21 /DISP_CS (GPIO19)   22 /DISP_SCLK (GPIO18) 23..28 <no net>      29 GND   30 +3V3_ESP (3V3)
Coordinates: row A y=15.500, pads 1..15 at x 97.200 .. 61.640; row B y=40.900, pads 16..30 likewise.

## 5. Copper diff vs the base board (per-net multiset of tracks+vias, 0.01 mm normalised)
17 nets differ, 53 of 70 nets are identical, and every differing net is inside the allowed set
(U7's nets, U12's nets, C35's nets, R43's nets):
  +3V3     97 -> 104    +5V     33 -> 42     GND     210 -> 246
  /PRES_SW 16 ->  16    /SPEED_PULSE 2 -> 7  /INT_MCU 11 -> 10   /VLV_EN  19 -> 23
  /DISP_BL 10 ->   6    /DISP_RES   4 -> 12  /DISP_DC 13 ->  7   /DISP_MOSI 8 ->  6
  /DISP_CS 22 ->  11    /DISP_SCLK 17 ->  4  /I2C_SCL 26 -> 31   /I2C_SDA  41 -> 43
  /SP_DIV  23 ->  25    Net-(U7-EN)  5 -> 0
  outside the allowed set: none
Changes the report should name explicitly:
  * /SP_DIV (U12 pin 2) and +3V3 (U12 pin 5, C35 pin 1) re-routed because U12 moved 3 mm - inside the set.
  * C15 pin 1 (+3V3) had its stub re-routed as a side effect of moving the +3V3 trunk at y=45.2 that
    crossed U12's new pads. C15 itself was NOT moved; +3V3 is U12/C35's net so this is inside the
    allowed copper set, but it is the one change that touches a part I was not asked to move.
  * GND +36 items: 11 new stitching vias (0.45/0.20) plus re-routed GND stubs (see 7).

## 6. The two approved moves
  U12 (74LVC1G14, SOT-23-5): (72.615, 41.058) -> (72.615, 44.058)  = +3.000 mm in y (approved 3.00)
  C35 (100 nF, 0805):        (61.703, 41.283) -> (61.703, 43.383)  = +2.100 mm in y (approved 2.10)
  Reason: U12's five pads straddled the new row-B pad row (y=40.900); C35's GND pad collided with pad 30.
  Only these two parts were moved (2 of the 4 allowed); no third move was needed.

## 7. Routing method
A 0.1 mm grid A* router: 2-layer (F.Cu/B.Cu) graph, 8-directional steps, through-vias, obstacle model
from analytic pad shapes with 0.15 mm clearance and a 0.05 mm raster safety margin; via placement used a
separate 0.235 mm inflation so a 0.45/0.20 via keeps full clearance. Paths are simplified with a
line-of-sight pass that never jumps across a layer change. Module-area copper was ripped up and
re-routed (allowed), 22 net connections were routed, and 11 GND stitching vias were added - one in each
F.Cu GND fill island that had no layer tie - because the new copper in the dense U6/U5 pocket fragmented
the F.Cu GND fill into islands narrower than the zone's 0.25 mm minimum fill width. kicad-cli DRC is the
final oracle for every net.

## 8. Project integrity
  netclasses: 6 - Default 0.15/0.25, +12V 0.127/0.6, +24V 0.127/0.8, +3V3 0.127/0.4, +5V 0.127/0.5,
              VALVE 0.127/0.8; all via 0.6/0.3. netclass_patterns: 18 (unchanged).
  rule_severities: 62 entries = 31 error / 21 warning / 10 ignore, and all 10 ignore waivers are
              present: courtyards_overlap, footprint_filters_mismatch, footprint_type_mismatch,
              missing_courtyard, pth_inside_courtyard, silk_edge_clearance, silk_over_copper,
              silk_overlap, track_not_centered_on_via, tuning_profile_track_geometries.
  drc_exclusions: 0.  .kicad_dru still kamaz_leveler_devkitv1.kicad_dru.
  .kicad_pro sha256 prefix b137245e (the post-rename value; the base 1ed98c56 canary only holds for an
  un-renamed project).

## 9. Schematic
  U7 lib_id kamaz_leveler:ESP32-DevKitC_30P (the project library already carries a 30-pin DevKit
  symbol); all 30 pin names verified against the assignment table (EN, IO36, IO39, IO34, IO35, IO32,
  IO33, IO25, IO26, IO27, IO14, IO12, IO13, GND, VIN / IO23, IO22, IO1, IO3, IO21, IO19, IO18, IO5,
  IO17, IO16, IO4, IO2, IO15, GND, 3V3); pin electrical types unchanged (input/bidirectional/power_in)
  so ERC is unaffected. U7 Value -> ESP32-DevKitV1-TypeC and Footprint -> the new socket (required for
  board/schematic parity, otherwise DRC reports footprint_symbol_mismatch).
  ERC: 0 violations (kicad-cli sch erc --severity-all).
  BOM.csv: 46 component rows + header (was 47); R43 gone; U7 row correct.

## 10. Footprint / library edits (for the record)
  kamaz_leveler_hw.pretty/ESP32-DevKitV1-TypeC_Socket_2x15_P2.54mm_Row25.4mm.kicad_mod:
  pads 14 and 29 now carry (zone_connect 2) = solid connection. Without it pad 29 reported
  starved_thermal against the B.Cu GND fill (the pocket cannot resolve 2 spokes at 0.65 mm pad pitch).

## 11. Limits used
  full kicad-cli DRC runs ~40 of 60 | board saves in the project 8 of 25 | win_bash calls ~110 of 400
  component moves 2 of 4 | no rule, netclass, severity or exclusion was modified.

## 12. Schematic<->board parity fix (final run)
  Defect: the schematic still carried the ESP32-C3 pin->net assignment while the board carried the
  DevKit V1 assignment. kicad-cli pcb drc --schematic-parity reported 30 parity items:
     22 net_conflict | 4 footprint_symbol_field_mismatch | 4 extra_footprint (H1..H4, pre-existing).
  Fix - schematic only, labels MOVED between pins (no net renamed, no board copper touched):
     pad 2  <- /PRES_SW    (was on pin 8)      pad 4  <- /SPEED_PULSE (was pin 5)
     pad 5  <- /INT_MCU    (was pin 7)         pad 6  <- /DISP_BL     (was pin 26)
     pad 8  <- /DISP_RES   (was pin 21)        pad 9  <- /DISP_DC     (was pin 25)
     pad 10 <- /VLV_EN     (was pin 6)         pad 21 <- /DISP_CS     (was pin 23)
     no_connect removed at pins 2,4,9,10 and added at pins 7,23,25,26 (those board pads carry no net).
     Pins 14,15,16,17,20,22,29,30 were already correct and were not touched.
  Field mismatches: board footprints C15/C16/U8 (Description) and U7 (Datasheet) were empty while the
     symbols carried the library text; the symbol values were written into those 4 footprints. Nothing
     was dropped: the schematic keeps its fields, so BOM.csv is unchanged.
  Net-name completion on the board (no copper change): the 14 U7 pads with no connection at all
     (1,3,7,11,12,13,18,19,23,24,25,26,27,28) carried net ""; they now carry the schematic's own
     placeholder names unconnected-(U7-<pin>-Pad<n>). KiCad 10.0.7 compares net NAMES, and the
     schematic emits an unconnected-(...) name for any pin with no connection - verified by exporting
     the netlist both with the no-connect flags removed and with the pins retyped no_connect: the name
     is emitted either way. No track, via, zone or other copper attaches to those pads.
  Result: kicad-cli pcb drc --format json --severity-all --schematic-parity
     violations 0 | unconnected_items 0 | schematic_parity 4 (all extra_footprint: H1,H2,H3,H4).
     ERC 0 violations. BOM regenerated and byte-identical (46 component rows + header).
     Board structure before/after identical: 120 footprints, 397 pads, 978 track segments, 304 vias,
     3 zones, 8 drawings; 367 of 381 pad->net entries unchanged, the 14 changed entries are the
     unconnected U7 pads above. R43 still absent. No netclass, rule, severity or exclusion touched.
