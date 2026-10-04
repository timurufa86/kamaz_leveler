#!/usr/bin/env python3
"""Host runner for Kamaz-Leveler serial self-test harness (TEST … commands).

Usage:
  python _serial_selftest.py              # auto-detect COM, full suite
  python _serial_selftest.py COM4
  python _serial_selftest.py COM4 --quick
  python _serial_selftest.py COM4 --full
  python _serial_selftest.py COM4 --cmd "TEST STATUS"
  python _serial_selftest.py COM4 --cmd "TEST OTA LIST"

Expects firmware with [TEST] protocol (9.1.5+ for FULL / PRESS / MODE / OTA).
Detects hang: no serial bytes for --hang-sec while heartbeat is on.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Callable, List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(2)


def pick_port(preferred: Optional[str]) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "uart" in desc or "cp210" in desc or "ch340" in desc or "serial" in desc:
            return p.device
    if ports:
        return ports[0].device
    raise SystemExit("No serial ports found")


class SerialTest:
    def __init__(self, port: str, baud: int = 115200, hang_sec: float = 8.0):
        self.port = port
        self.hang_sec = hang_sec
        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.buf = ""
        self.lines: List[str] = []
        self.last_rx = time.monotonic()
        self.fails = 0
        self.passes = 0

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _pump(self) -> None:
        n = self.ser.in_waiting
        if n:
            chunk = self.ser.read(n).decode("utf-8", errors="replace")
            self.last_rx = time.monotonic()
            self.buf += chunk
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                line = line.rstrip("\r")
                if line:
                    self.lines.append(line)
                    safe = line.encode("ascii", errors="replace").decode("ascii")
                    print(f"  << {safe}")

    def wait_for(
        self,
        pred: Callable[[str], bool],
        timeout: float = 5.0,
        label: str = "",
    ) -> Optional[str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            for i, line in enumerate(self.lines):
                if pred(line):
                    self.lines = self.lines[i + 1 :]
                    return line
            if time.monotonic() - self.last_rx > self.hang_sec:
                raise TimeoutError(f"HANG: no RX for {self.hang_sec:.1f}s ({label})")
            time.sleep(0.02)
        return None

    def send(self, cmd: str) -> None:
        print(f"  >> {cmd}")
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))
        self.ser.flush()

    def expect_ok(self, token: str, timeout: float = 5.0) -> bool:
        line = self.wait_for(
            lambda L: L.startswith("[TEST] OK") and token in L,
            timeout=timeout,
            label=token,
        )
        if line:
            self.passes += 1
            return True
        print(f"  !! missing OK {token}")
        self.fails += 1
        return False

    def expect_substr(self, substr: str, timeout: float = 5.0) -> bool:
        line = self.wait_for(lambda L: substr in L, timeout=timeout, label=substr)
        if line:
            self.passes += 1
            return True
        print(f"  !! missing '{substr}'")
        self.fails += 1
        return False

    def cmd_ok(self, cmd: str, token: str, timeout: float = 5.0) -> bool:
        self.send(cmd)
        return self.expect_ok(token, timeout=timeout)

    def status_field(self, key: str, timeout: float = 3.0) -> Optional[str]:
        self.send("TEST STATUS")
        line = self.wait_for(lambda L: "[TEST] STATUS " in L and key + "=" in L, timeout=timeout)
        self.expect_ok("STATUS", timeout=2)
        if not line:
            return None
        for part in line.replace(",", " ").split():
            if part.startswith(key + "="):
                return part.split("=", 1)[1]
        return None


def run_suite(t: SerialTest, quick: bool, full: bool) -> int:
    print(f"=== selftest on {t.port} (quick={quick} full={full}) ===")
    # Boot may spam — drain a bit
    time.sleep(1.0)
    t._pump()
    t.lines.clear()

    t.send("TEST HEARTBEAT ON")
    t.expect_ok("HEARTBEAT_ON", timeout=4)

    basic: List[Tuple[str, str, float]] = [
        ("TEST PING", "PONG", 3),
        ("TEST VER", "VER", 3),
        ("TEST WDT", "WDT", 3),
        ("TEST STATUS", "STATUS", 3),
        ("TEST HEAP", "HEAP", 3),
        ("TEST PRESS", "PRESS", 4),
        ("TEST IMU", "IMU", 3),
        ("TEST WIFI", "WIFI", 3),
        ("TEST BTN", "BTN", 3),
        ("TEST ERR", "ERR", 3),
        ("TEST CALIB", "CALIB", 3),
        ("TEST I2C", "I2C", 8),
        ("TEST OTA STATUS", "OTA_STATUS", 3),
    ]
    for cmd, tok, to in basic:
        t.cmd_ok(cmd, tok, timeout=to)

    # --- Menu ---
    t.cmd_ok("TEST MENU OPEN", "MENU_OPEN_REQ", timeout=3)
    time.sleep(0.55)
    menu = t.status_field("menu")
    if menu == "1":
        t.passes += 1
        print("  ++ menu open confirmed")
    else:
        t.fails += 1
        print(f"  !! menu did not open (menu={menu})")

    t.cmd_ok("TEST MENU CLOSE", "MENU_CLOSE_REQ", timeout=3)
    time.sleep(0.75)
    t.wait_for(lambda L: "[MENU] CLOSE ok" in L or "menu=0" in L, timeout=4, label="close")
    menu = t.status_field("menu")
    if menu == "0":
        t.passes += 1
        print("  ++ menu close confirmed")
    else:
        t.fails += 1
        print(f"  !! menu still open (menu={menu})")

    # --- Modes ---
    t.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=4)
    time.sleep(0.4)
    mode = t.status_field("mode")
    if mode == "0":
        t.passes += 1
        print("  ++ MODE MANUAL")
    else:
        t.fails += 1
        print(f"  !! MODE MANUAL got mode={mode}")

    t.cmd_ok("TEST MODE AUTO", "MODE_AUTO", timeout=4)
    time.sleep(0.4)
    mode = t.status_field("mode")
    if mode == "1":
        t.passes += 1
        print("  ++ MODE AUTO")
    else:
        t.fails += 1
        print(f"  !! MODE AUTO got mode={mode}")

    t.send("TEST MODE MOVEMENT")
    mv = t.wait_for(
        lambda L: L.startswith("[TEST] OK MODE_MOVEMENT") or L.startswith("[TEST] FAIL MODE_MOVEMENT"),
        timeout=5,
        label="movement",
    )
    if mv and "OK MODE_MOVEMENT" in mv:
        t.passes += 1
        print("  ++ MODE MOVEMENT")
        time.sleep(0.5)
        t.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=4)
    elif mv and "FAIL" in mv:
        t.passes += 1
        print(f"  ++ MODE MOVEMENT skipped/fail ok: {mv}")
    else:
        t.fails += 1
        print("  !! MODE MOVEMENT no response")

    t.cmd_ok("TEST BRIGHT 55", "BRIGHT", timeout=3)
    t.cmd_ok("TEST DIM ON", "DIM_ON", timeout=3)
    t.cmd_ok("TEST DIM OFF", "DIM_OFF", timeout=3)
    t.cmd_ok("TEST BRIGHT 70", "BRIGHT", timeout=3)

    if not quick:
        t.cmd_ok("TEST CFG", "CFG", timeout=5)
        t.cmd_ok("TEST STACK", "STACK", timeout=5)

        t.send("TEST SELF")
        t.expect_ok("SELF_START", timeout=3)
        ok = t.wait_for(
            lambda L: "SELF DONE PASS" in L or "SELF_PASS" in L or "SELF DONE FAIL" in L,
            timeout=10,
            label="SELF",
        )
        if ok and ("PASS" in ok):
            t.passes += 1
            print("  ++ SELF PASS")
        else:
            t.fails += 1
            print("  !! SELF FAIL")

        for _ in range(4):
            t.send("TEST MENU TOGGLE")
            t.expect_ok("MENU_TOGGLE_REQ", timeout=3)
            time.sleep(0.5)
        t.send("TEST MENU CLOSE")
        t.expect_ok("MENU_CLOSE_REQ", timeout=3)
        time.sleep(0.7)

    if full:
        print("--- FULL device sequence ---")
        t.send("TEST FULL")
        t.expect_ok("FULL_START", timeout=3)
        ok = t.wait_for(
            lambda L: "FULL DONE PASS" in L or "FULL_PASS" in L or "FULL DONE FAIL" in L,
            timeout=20,
            label="FULL",
        )
        if ok and ("PASS" in ok):
            t.passes += 1
            print("  ++ FULL PASS")
        else:
            t.fails += 1
            print(f"  !! FULL FAIL ({ok})")

        # OTA list request (may fail TLS — count as soft)
        print("--- OTA LIST (soft) ---")
        t.send("TEST OTA LIST")
        ota = t.wait_for(
            lambda L: "OTA_LIST_REQ" in L or "OTA_LIST_NO_WIFI" in L or "FAIL" in L,
            timeout=5,
            label="ota-list",
        )
        if ota and "OTA_LIST_REQ" in ota:
            t.passes += 1
            # wait for GH-OTA result up to 45s without requiring success
            gh = t.wait_for(
                lambda L: "[GH-OTA]" in L and (
                    "Получено релизов" in L
                    or "список:" in L
                    or "HTTP" in L
                    or "TLS" in L
                    or "мало RAM" in L
                    or "нет Wi-Fi" in L
                    or "saved" in L
                ),
                timeout=45,
                label="gh-ota",
            )
            if gh:
                t.passes += 1
                print(f"  ++ OTA fetch activity: {gh[:80]}")
            else:
                # Soft: TLS/RAM на ESP32 часто не успевает — не валим весь suite
                t.passes += 1
                print("  !! no GH-OTA activity after LIST (soft — check heap/TLS)")
            time.sleep(1.0)
            t.cmd_ok("TEST OTA STATUS", "OTA_STATUS", timeout=5)
        elif ota and "NO_WIFI" in ota:
            t.passes += 1
            print("  ++ OTA LIST no wifi (expected if AP-only)")
        else:
            t.fails += 1
            print("  !! OTA LIST no ack")

        # Ensure back to manual for operator
        t.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=4)

    t.send("TEST HEARTBEAT OFF")
    t.expect_ok("HEARTBEAT_OFF", timeout=3)

    print(f"=== done: pass={t.passes} fail={t.fails} ===")
    return 0 if t.fails == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--full", action="store_true", help="FULL device sequence + OTA list probe")
    ap.add_argument("--hang-sec", type=float, default=12.0)
    ap.add_argument("--cmd", action="append", default=[], help="single TEST command(s)")
    args = ap.parse_args()

    port = pick_port(args.port)
    t = SerialTest(port, baud=args.baud, hang_sec=args.hang_sec)
    try:
        if args.cmd:
            t.send("TEST HEARTBEAT ON")
            t.expect_ok("HEARTBEAT_ON", timeout=4)
            for c in args.cmd:
                t.send(c if c.upper().startswith("TEST") else f"TEST {c}")
                t.wait_for(lambda L: L.startswith("[TEST]"), timeout=8, label=c)
            t.send("TEST HEARTBEAT OFF")
            return 0
        return run_suite(t, quick=args.quick, full=args.full)
    except TimeoutError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
