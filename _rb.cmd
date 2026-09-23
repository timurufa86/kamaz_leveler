@echo off
REM Временный запуск сборки 8.9.0 через планировщик задач (не зависит от терминала).
cd /d "c:\my\for cursor\kamaz_leveler"
call build.cmd > build_8_9_0.log 2>&1
echo BUILD_EXIT=%ERRORLEVEL% >> build_8_9_0.log
