set -e
PY="C:/Program Files/KiCad/10.0/bin/python.exe"
cd "C:/my/for cursor/kamaz_leveler/hardware/variants/esp32s3_ruichi158"
cp -p ../esp32s3/kamaz_leveler_s3.kicad_pcb kamaz_leveler_ruichi158.kicad_pcb
"$PY" _work/strip.py
"$PY" _work/build.py 2>&1 | grep -v "memory leak" | grep -v "^ewo"
