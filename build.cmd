@echo off
REM ============================================================================
REM  Build script for Kamaz-Leveler firmware (Windows / arduino-cli).
REM
REM  Uses the project-local arduino-cli.yaml, which points the sketchbook at
REM  C:\my so that libraries are resolved from C:\my\libraries.
REM
REM  Usage:
REM    build.cmd              ^<-- compile the current 8.7.0 sketch
REM    build.cmd upload COM5  ^<-- compile + upload to a board
REM
REM  The board uses a CUSTOM 16MB partition table (see
REM  %SKETCH%\partitions.csv) so the OTA app slots are 6.25MB each instead
REM  of the stock 1.25MB. FlashSize=16M must match the physical chip.
REM
REM  --build-property compiler.c.elf.extra_flags=...
REM    GEM 1.8.1 компилируется целиком (включая GEM_u8g2.cpp), из-за чего
REM    в сборку попадает библиотека U8g2, а её u8g2_fonts.c дублирует
REM    символы шрифтов из U8g2_for_Adafruit_GFX. Флаг разрешает дубликаты:
REM    линковщик оставляет первое определение (набор шрифтов Adafruit-GFX).
REM ============================================================================

setlocal

set SKETCH=kamaz_8_7_0_fr_adafruiti_OTA_NM
set FQBN=esp32:esp32:esp32:PartitionScheme=custom,FlashSize=16M
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
