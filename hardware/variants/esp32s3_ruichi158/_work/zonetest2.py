import io, sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

SRC = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb"
DST = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work\zonetest2.kicad_pcb"

zone_bcu_gnd = """	(zone
		(net "GND")
		(layer "B.Cu")
		(uuid "00000000-0000-4000-8000-000000000001")
		(name "GND_B.Cu_test")
		(hatch edge 0.5)
		(connect_pads yes
			(clearance 0.3)
		)
		(min_thickness 0.25)
		(fill yes
			(thermal_gap 0.5)
			(thermal_bridge_width 0.5)
			(island_removal_mode 0)
		)
		(polygon
			(pts
				(xy -28.8 -41.4) (xy 28.8 -41.4) (xy 28.8 41.4) (xy -28.8 41.4)
			)
		)
	)"""

zone_fcu_3v3 = zone_bcu_gnd.replace('"GND"', '"+3V3"').replace('"B.Cu"', '"F.Cu"') \
    .replace("GND_B.Cu_test", "+3V3_F.Cu_test").replace("00000000-0000-4000-8000-000000000001",
                                                       "00000000-0000-4000-8000-000000000002")

txt = open(SRC, encoding="utf-8").read()
i = txt.rstrip().rfind("\n)")
assert i > 0, "не найден финальный )"
insert = zone_bcu_gnd + "\n" + zone_fcu_3v3 + "\n"
txt = txt[:i + 1] + insert + txt[i + 1:]
open(DST, "w", encoding="utf-8", newline="\n").write(txt)
print("добавлено 2 зоны (B.Cu GND, F.Cu +3V3), файл:", DST)
print("вхождений (zone:", txt.count("(zone"))
