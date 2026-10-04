#!/usr/bin/env python3
"""Serial OTA harness driver for Kamaz-Leveler."""
import serial, time, sys, datetime, re

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
LOG = r"C:\my\for cursor\kamaz_leveler\_serial_live.log"

ser = serial.Serial(port="COM4", baudrate=115200, timeout=0.2)
ser.dtr = False
ser.rts = False
time.sleep(0.3)
ser.rts = True
time.sleep(0.05)
ser.rts = False
time.sleep(1.0)
ser.reset_input_buffer()

buf = ""
open(LOG, "w", encoding="utf-8").write(f"=== OTA driver {datetime.datetime.now().isoformat()} ===\n")
print(f"[{datetime.datetime.now():%H:%M:%S}] driver open+reset", flush=True)


def pump(seconds=0.0, until=None, timeout=120.0, clear=False):
    global buf
    if clear:
        buf = ""
    t0 = time.time()
    deadline = t0 + (timeout if until is not None else seconds)
    while time.time() < deadline:
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if chunk:
            text = chunk.decode("utf-8", errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            with open(LOG, "a", encoding="utf-8") as log:
                log.write(text)
            buf += text
            if until is not None and until.search(buf):
                return True
        else:
            time.sleep(0.05)
    return False


def send_wait(cmd: str, expect_rx: str, retries=8, gap=1.5):
    """Send until firmware echoes [OTA-CMD] rx: <cmd> (TX flood drops RX)."""
    pat = re.compile(re.escape(f"[OTA-CMD] rx: {expect_rx}"))
    for attempt in range(1, retries + 1):
        # brief quiet: drain RX first
        pump(0.4, clear=True)
        print(f"\n>>> TX[{attempt}]: {cmd!r}", flush=True)
        ser.write((cmd + "\n").encode("ascii"))
        ser.flush()
        if pump(until=pat, timeout=gap + 2.0, clear=False):
            print(f"[driver] got echo for {expect_rx}", flush=True)
            return True
        time.sleep(gap)
    print(f"[driver] NO echo for {expect_rx}", flush=True)
    return False


pump(until=re.compile(r"OTA-CMD\] ready"), timeout=60)
pump(until=re.compile(r"STA подключен|STA connected"), timeout=60)
print("\n>>> settle 20s after WiFi", flush=True)
pump(20.0)

if not send_wait("OTA HEAP", "OTA HEAP"):
    sys.exit(2)
pump(1.0)

if not send_wait("OTA VER", "OTA VER"):
    sys.exit(2)
pump(2.0)

# LIST optional — иногда TLS к api.github.com падает; INSTALL сам тянет список.
if send_wait("OTA LIST", "OTA LIST", retries=3, gap=2.0):
    ok = pump(until=re.compile(r"OTA-CMD\] LIST (ok|fail)"), timeout=150, clear=False)
    print(f"\n[driver] LIST matched={ok}", flush=True)
else:
    print("[driver] LIST echo skipped — INSTALL will fetch", flush=True)

if not send_wait("OTA INSTALL 0", "OTA INSTALL 0", retries=10, gap=2.0):
    sys.exit(4)

ok = pump(
    until=re.compile(
        r"esp_ota_begin OK|esp_ota_begin fail|OTA begin fail|INSTALL fail|set_boot|chunked\(FS\)|сбой записи|SHA"
    ),
    timeout=120,
    clear=True,
)
print(f"\n[driver] early matched={ok}", flush=True)
ok2 = pump(
    until=re.compile(
        r"set_boot|INSTALL fail|OTA begin fail|сбой записи|SHA не|обрыв потока|таймаут|Версия: V "
    ),
    timeout=1200,
    clear=False,
)
print(f"\n[driver] finish matched={ok2}", flush=True)
pump(30.0)
print(f"\n[{datetime.datetime.now():%H:%M:%S}] driver done", flush=True)
ser.close()
