@echo off
REM ============================================================================
REM  Build script for Kamaz-Leveler firmware (Windows / arduino-cli).
REM
REM  Uses the project-local arduino-cli.yaml, which points the sketchbook at
REM  C:\my so that libraries are resolved from C:\my\libraries.
REM
REM  Usage:
REM    build.cmd            ^<-- compile the current 8.5.8 sketch
REM    build.cmd upload COM5 ^<-- compile + upload to a board
REM ============================================================================

setlocal

set SKETCH=kamaz_8_5_8_fr_adafruiti_OTA_NM
set FQBN=esp32:esp32:esp32
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
