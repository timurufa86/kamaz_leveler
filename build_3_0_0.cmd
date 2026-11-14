@echo off
REM ============================================================================
REM  Build script for the NEW line: 3.0.0 (ESP32-S3, v3 board / PCAL9555A).
REM  Counterpart of build.cmd, which builds the OLD 2.0.0 line (ESP32 classic).
REM
REM  Usage:
REM    build_3_0_0.cmd         ^<-- compile kamaz_3_0_0_fr_adafruiti_OTA_NM
REM    build_3_0_0.cmd COM5    ^<-- compile + upload to a board
REM
REM  The S3 board has 16MB flash and uses a CUSTOM partition table
REM  (%SKETCH%\partitions.csv): two OTA app slots of 2MB each, ~11.9MB LittleFS.
REM  The stock PartitionScheme=default_8MB has a single app slot, so OTA is
REM  impossible there — do not switch back to it.
REM  FlashSize=16M and PSRAM=opi must match the physical chip.
REM ============================================================================

setlocal

set SKETCH=kamaz_3_0_0_fr_adafruiti_OTA_NM
REM Sketch folder 3_0_0; VERSION string = 3.0.0
set FQBN=esp32:esp32:esp32s3:PartitionScheme=custom,FlashSize=16M,PSRAM=opi
set CONFIG=arduino-cli.yaml
set LINKFLAGS=-Wl,--allow-multiple-definition

echo === Compiling %SKETCH% (%FQBN%) ===
arduino-cli compile --config-file %CONFIG% -b %FQBN% --warnings none --export-binaries ^
  --build-property compiler.c.elf.extra_flags=%LINKFLAGS% %SKETCH%
if errorlevel 1 (
  echo [BUILD] FAILED
  exit /b 1
)

echo [BUILD] OK

if not "%~1"=="" (
  echo === Uploading to %~1 ===
  arduino-cli upload --config-file %CONFIG% -b %FQBN% -p %~1 %SKETCH%
  if errorlevel 1 (
    echo [UPLOAD] FAILED
    exit /b 1
  )
  echo [UPLOAD] OK
)

endlocal
