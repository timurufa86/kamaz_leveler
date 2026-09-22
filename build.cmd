@echo off
REM ============================================================================
REM  Build script for Kamaz-Leveler firmware (Windows / arduino-cli).
REM
REM  Uses the project-local arduino-cli.yaml, which points the sketchbook at
REM  C:\my so that libraries are resolved from C:\my\libraries.
REM
REM  Usage:
REM    build.cmd              ^<-- compile the current 8.5.9 sketch
REM    build.cmd upload COM5  ^<-- compile + upload to a board
REM
REM  The board uses a CUSTOM 16MB partition table (see
REM  %SKETCH%\partitions.csv) so the OTA app slots are 6.25MB each instead
REM  of the stock 1.25MB. FlashSize=16M must match the physical chip.
REM ============================================================================

setlocal

set SKETCH=kamaz_8_5_9_fr_adafruiti_OTA_NM
set FQBN=esp32:esp32:esp32:PartitionScheme=custom,FlashSize=16M
set CONFIG=arduino-cli.yaml

echo === Compiling %SKETCH% (%FQBN%) ===
arduino-cli compile --config-file %CONFIG% -b %FQBN% --warnings none --export-binaries %SKETCH%
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
