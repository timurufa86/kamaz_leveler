# -*- coding: utf-8 -*-
"""Non-resetting COM4 serial logger for OTA test."""
import serial
import time
import sys
from datetime import datetime

PORT = "COM4"
BAUD = 115200
LOG_PATH = r"C:\my\for cursor\kamaz_leveler\_ota_serial_9_0_11.txt"

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.2
ser.dsrdtr = False
ser.rtscts = False
# Avoid ESP32 auto-reset on open
ser.dtr = False
ser.rts = False
ser.open()
# Keep lines quiet after open
time.sleep(0.05)
ser.dtr = False
ser.rts = False

print(f"OPEN {PORT} {BAUD} (no DTR/RTS reset) -> {LOG_PATH}", flush=True)
with open(LOG_PATH, "a", encoding="utf-8", errors="replace") as log:
    log.write(f"\n=== serial {PORT} {BAUD} started {datetime.now().isoformat()} (no reset) ===\n")
    log.flush()
    buf = b""
    while True:
        try:
            chunk = ser.read(4096)
        except serial.SerialException as e:
            print(f"SERIAL ERR: {e}", flush=True)
            break
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                out = f"[{stamp}] {text}"
                print(out, flush=True)
                log.write(out + "\n")
                log.flush()
        else:
            time.sleep(0.05)
