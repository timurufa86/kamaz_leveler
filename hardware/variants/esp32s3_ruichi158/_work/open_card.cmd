@echo off
start "" "C:\Program Files\Google\Chrome\Application\chrome.exe" ^
 --remote-debugging-port=9222 ^
 --user-data-dir="C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work\chrome-profile" ^
 --no-first-run --no-default-browser-check --disable-features=Translate ^
 "file:///C:/my/for%%20cursor/kamaz_leveler/hardware/variants/esp32s3_ruichi158/plots/board_card.html"
