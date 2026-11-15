@echo off
REM ============================================================================
REM  Build script for Kamaz-Leveler firmware (Windows / arduino-cli).
REM
REM  Uses the project-local arduino-cli.yaml, which points the sketchbook at
REM  C:\my so that libraries are resolved from C:\my\libraries.
REM
REM  This script builds the OLD line only (2.0.0, ESP32 classic, v2 board).
REM  The NEW line (3.0.0, ESP32-S3, v3 board) has its own build_3_2_1.cmd.
REM
REM  Usage:
REM    build.cmd         ^<-- compile kamaz_2_0_0_fr_adafruiti_OTA_NM
REM    build.cmd COM5    ^<-- compile + upload to a board
REM
REM  The board uses a CUSTOM 4MB partition table (see
REM  %SKETCH%\partitions.csv) so the OTA app slots are 1.75MB each instead
REM  of the stock 1.25MB. FlashSize=4M must match the physical chip.
REM ============================================================================

setlocal

set SKETCH=kamaz_2_0_0_fr_adafruiti_OTA_NM
REM Sketch folder 2_0_0; VERSION string = 2.0.0
set FQBN=esp32:esp32:esp32:PartitionScheme=custom,FlashSize=4M
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
