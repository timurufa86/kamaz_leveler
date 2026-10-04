#!/usr/bin/env python3
"""Drive embedded OTA serial harness: wait RUNNING, send cmds with echo retry."""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
CMDS = sys.argv[2:] if len(sys.argv) > 2 else ["PING", "LIST"]


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ser = serial.Serial(PORT, 115200, timeout=0.2)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.3)
    buf = ""

    def pump(until: re.Pattern | None, timeout: float, clear: bool = False) -> bool:
        nonlocal buf
        if clear:
            buf = ""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                text = chunk.decode("utf-8", "replace")
                sys.stdout.write(text)
                sys.stdout.flush()
                buf += text
                if until and until.search(buf):
                    return True
            else:
                time.sleep(0.04)
        return False

    def send(cmd: str, done: re.Pattern, done_timeout: float) -> bool:
        """Retry until device echoes rx, then wait for completion pattern."""
        echo = re.compile(r"\[OTA-CMD\] rx: OTA " + re.escape(cmd), re.I)
        for attempt in range(1, 12):
            line = f"OTA {cmd}\n"
            print(f"\n>>> send '{line.strip()}' try {attempt}", flush=True)
            # Do NOT reset_input_buffer — races with device TX and can drop sync.
            for _ in range(3):
                ser.write(line.encode("ascii"))
                ser.flush()
                time.sleep(0.15)
            if not pump(echo, timeout=8.0, clear=True):
                print("[host] no echo, retry", flush=True)
                time.sleep(1.0)
                continue
            print("[host] echo ok, wait done…", flush=True)
            if pump(done, timeout=done_timeout, clear=False):
                return True
            print("[host] done-pattern timeout", flush=True)
            return False
        return False

    print(f"[host] open {PORT}", flush=True)
    pump(re.compile(r"OTA-CMD\] ready|готов к работе"), timeout=90, clear=True)
    t_end = time.time() + 50
    while time.time() < t_end:
        pump(None, timeout=1.0, clear=False)
        if "Система готова" in buf or "Changed from 1 to 2" in buf:
            break
    time.sleep(4)

    rc = 0
    for cmd in CMDS:
        cmd = cmd.upper()
        if cmd == "PING":
            ok = send("PING", re.compile(r"\[OTA-PING\] (OK|HTTPS GET zen|FAIL)"), 120)
        elif cmd == "LIST":
            ok = send("LIST", re.compile(r"LIST (ok|fail)"), 180)
        elif cmd.startswith("INSTALL"):
            ok = send(
                cmd,
                re.compile(r"INSTALL fail|OK set_boot|SHA-256 mismatch|esp_ota_end fail"),
                900,
            )
        elif cmd == "SELFTEST":
            ok = send(
                "SELFTEST",
                re.compile(r"SELFTEST FAIL|OK set_boot|SHA-256 mismatch|LIST fail|обрыв"),
                1500,
            )
        elif cmd == "HEAP":
            ok = send("HEAP", re.compile(r"OTA-HEAP"), 15)
        elif cmd == "VER":
            ok = send("VER", re.compile(r"VERSION="), 15)
        else:
            ok = send(cmd, re.compile(r"OTA-CMD"), 30)
        if not ok:
            print(f"[host] FAIL {cmd}", flush=True)
            rc = 1
            break
        print(f"[host] done {cmd}", flush=True)

    ser.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
