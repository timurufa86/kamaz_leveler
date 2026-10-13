#!/usr/bin/env python3
"""Live motion-detection monitor for Kamaz-Leveler (firmware 10.10.6+).

Streams [MOT] lines from the device and prints a compact live view + summary.

Usage:
  python _motion_monitor.py COM4
  python _motion_monitor.py COM4 --sec 120
  python _motion_monitor.py COM4 --sec 180 --period 100 --log motion.csv
  python _motion_monitor.py COM4 --gyro 40          # live thr = gyro×8
  python _motion_monitor.py COM4 --duration 15 --settle 20

Serial commands used:
  TEST MOT / MOT CFG / MOT STREAM / MOT STATS / MOT RESET / MOT SET ...
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(2)

# Current firmware [MOT] line (10.10.6+):
# [MOT] t=.. mot=.. motSt=0x.. gyro=.. gdlt=.. thr=.. busy=..
#   bump=.. bdlt=.. bthr=.. bbusy=.. lin=.. ldlt=.. lthr=.. lbusy=..
#   hold=.. was=.. mode=.. age=.. enter=.. settleLeft=.. exit=..
#   pulse=.. need=.. int=0x..
MOT_RE = re.compile(
    r"\[MOT\] t=(\d+) mot=(\d) motSt=0x([0-9A-Fa-f]+) "
    r"gyro=([-\d.]+) gdlt=([-\d.]+) thr=([-\d.]+) busy=(\d) "
    r"bump=([-\d.]+) bdlt=([-\d.]+) bthr=([-\d.]+) bbusy=(\d) "
    r"lin=([-\d.]+) ldlt=([-\d.]+) lthr=([-\d.]+) lbusy=(\d) "
    r"hold=(\d) was=(\d) mode=(\w+) "
    r"age=([-\d.]+) enter=([-\d.]+) settleLeft=([-\d.]+) exit=([-\d.]+) "
    r"pulse=(\d+) need=(\d+) int=0x([0-9A-Fa-f]+)"
)
EDGE_RE = re.compile(r"\[IMU\] >>> ДВИЖЕНИЕ|\[MOVEMENT\] Длительное движение")


def pick_port(preferred: Optional[str]) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "ch340" in desc or "cp210" in desc or "usb" in desc or "uart" in desc:
            return p.device
    if ports:
        return ports[0].device
    raise SystemExit("No serial ports found")


@dataclass
class Sample:
    t_ms: int
    mot: int
    mot_st: int
    gyro: float
    gdlt: float
    thr: float
    busy: int
    bump: float
    bdlt: float
    bthr: float
    bbusy: int
    lin: float
    ldlt: float
    lthr: float
    lbusy: int
    hold: int
    was: int
    mode: str
    age: float
    enter: float
    settle_left: float
    exit_s: float
    pulse_ms: int
    need_ms: int
    int_status: int


@dataclass
class Stats:
    samples: List[Sample] = field(default_factory=list)
    hold_edges: int = 0
    mode_enters: int = 0
    mot_pulses: int = 0
    gyro_busy: int = 0
    bump_busy: int = 0
    lin_busy: int = 0
    _prev_hold: Optional[int] = None
    _prev_mode: Optional[str] = None

    def add(self, s: Sample) -> None:
        if self._prev_hold is not None and s.hold == 1 and self._prev_hold == 0:
            self.hold_edges += 1
        if self._prev_mode is not None and s.mode == "MOVEMENT" and self._prev_mode != "MOVEMENT":
            self.mode_enters += 1
        self._prev_hold = s.hold
        self._prev_mode = s.mode
        if s.mot:
            self.mot_pulses += 1
        if s.busy:
            self.gyro_busy += 1
        if s.bbusy:
            self.bump_busy += 1
        if s.lbusy:
            self.lin_busy += 1
        self.samples.append(s)

    def summary(self) -> str:
        n = len(self.samples) or 1
        gdlts = [s.gdlt for s in self.samples if s.gdlt >= 0]
        bdlts = [s.bdlt for s in self.samples if s.bdlt >= 0]
        ldlts = [s.ldlt for s in self.samples if s.ldlt >= 0]
        thr = self.samples[-1].thr if self.samples else 0

        def pct(xs: List[float]) -> str:
            if not xs:
                return "-"
            xs = sorted(xs)
            p95 = xs[int(0.95 * (len(xs) - 1))]
            return f"min/avg/p95/max={xs[0]:.0f}/{sum(xs)/len(xs):.0f}/{p95:.0f}/{xs[-1]:.0f}"

        return (
            f"n={len(self.samples)} holdEdges={self.hold_edges} modeEnter={self.mode_enters} "
            f"mot%={100 * self.mot_pulses / n:.1f} gBusy%={100 * self.gyro_busy / n:.1f} "
            f"bBusy%={100 * self.bump_busy / n:.1f} lBusy%={100 * self.lin_busy / n:.1f} "
            f"hold%={100 * sum(1 for s in self.samples if s.hold) / n:.1f} thr={thr:.0f} "
            f"gdlt[{pct(gdlts)}] bdlt[{pct(bdlts)}] ldlt[{pct(ldlts)}]"
        )


class MotLink:
    def __init__(self, port: str, baud: int = 115200) -> None:
        # Avoid DTR/RTS pulse (ESP32 reboot) when opening the port.
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.2
        self.ser.dsrdtr = False
        self.ser.rtscts = False
        try:
            self.ser.setDTR(False)
            self.ser.setRTS(False)
        except Exception:
            pass
        self.ser.open()
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.buf = ""

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, cmd: str) -> None:
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))
        self.ser.flush()

    def read_lines(self) -> List[str]:
        chunk = self.ser.read(4096)
        if chunk:
            self.buf += chunk.decode("utf-8", errors="replace")
        lines: List[str] = []
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                lines.append(line)
        return lines

    def wait_ok(self, token: str, timeout: float = 4.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            for line in self.read_lines():
                if f"[TEST] OK {token}" in line or token in line:
                    return True
            time.sleep(0.05)
        return False


def parse_mot(line: str) -> Optional[Sample]:
    m = MOT_RE.search(line)
    if not m:
        return None
    return Sample(
        t_ms=int(m.group(1)),
        mot=int(m.group(2)),
        mot_st=int(m.group(3), 16),
        gyro=float(m.group(4)),
        gdlt=float(m.group(5)),
        thr=float(m.group(6)),
        busy=int(m.group(7)),
        bump=float(m.group(8)),
        bdlt=float(m.group(9)),
        bthr=float(m.group(10)),
        bbusy=int(m.group(11)),
        lin=float(m.group(12)),
        ldlt=float(m.group(13)),
        lthr=float(m.group(14)),
        lbusy=int(m.group(15)),
        hold=int(m.group(16)),
        was=int(m.group(17)),
        mode=m.group(18),
        age=float(m.group(19)),
        enter=float(m.group(20)),
        settle_left=float(m.group(21)),
        exit_s=float(m.group(22)),
        pulse_ms=int(m.group(23)),
        need_ms=int(m.group(24)),
        int_status=int(m.group(25), 16),
    )


def fmt_row(s: Sample) -> str:
    flags = (
        ("M" if s.mot else ".")
        + ("G" if s.busy else ".")
        + ("B" if s.bbusy else ".")
        + ("L" if s.lbusy else ".")
        + ("H" if s.hold else ".")
        + ("W" if s.was else ".")
    )
    enter = f"{s.enter:5.1f}" if s.enter >= 0 else "   --"
    exit_s = f"{s.exit_s:5.1f}" if s.exit_s >= 0 else "   --"
    pulse_pct = (100.0 * s.pulse_ms / s.need_ms) if s.need_ms > 0 else 0.0
    return (
        f"{s.t_ms/1000:8.1f} {flags:6} "
        f"gd={s.gdlt:5.0f}/{s.thr:<5.0f} bd={s.bdlt:5.0f}/{s.bthr:<5.0f} "
        f"ld={s.ldlt:5.0f}/{s.lthr:<5.0f} {s.mode:8} "
        f"age={s.age:5.1f} enter={enter} pulse={s.pulse_ms}/{s.need_ms}({pulse_pct:4.0f}%) "
        f"exit={exit_s}"
    )


CSV_HEADER = [
    "t_ms", "mot", "mot_st", "gyro", "gdlt", "thr", "busy",
    "bump", "bdlt", "bthr", "bbusy", "lin", "ldlt", "lthr", "lbusy",
    "hold", "was", "mode", "age", "enter", "settle_left", "exit",
    "pulse_ms", "need_ms", "int",
]


def sample_to_csv_row(s: Sample) -> list:
    return [
        s.t_ms, s.mot, f"0x{s.mot_st:02X}",
        f"{s.gyro:.1f}", f"{s.gdlt:.1f}", f"{s.thr:.1f}", s.busy,
        f"{s.bump:.1f}", f"{s.bdlt:.1f}", f"{s.bthr:.1f}", s.bbusy,
        f"{s.lin:.1f}", f"{s.ldlt:.1f}", f"{s.lthr:.1f}", s.lbusy,
        s.hold, s.was, s.mode,
        f"{s.age:.2f}", f"{s.enter:.2f}", f"{s.settle_left:.2f}", f"{s.exit_s:.2f}",
        s.pulse_ms, s.need_ms, f"0x{s.int_status:02X}",
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description="Kamaz motion detector live monitor")
    ap.add_argument("port", nargs="?", default=None, help="COM port (default: auto)")
    ap.add_argument("--sec", type=int, default=60, help="stream duration seconds")
    ap.add_argument("--period", type=int, default=100, help="device log period ms")
    ap.add_argument("--gyro", type=int, default=None, help="set gyro menu threshold (thr=gyro*8)")
    ap.add_argument("--bump", type=int, default=None, help="set bump menu threshold")
    ap.add_argument("--lin", type=int, default=None, help="set linAcc delta threshold")
    ap.add_argument("--det", type=int, default=None, help="set MOT_THR (20..255)")
    ap.add_argument("--duration", type=int, default=None, help="seconds before MOVEMENT mode")
    ap.add_argument("--settle", type=int, default=None, help="hold/settle seconds")
    ap.add_argument("--log", type=str, default=None, help="CSV log path")
    ap.add_argument("--quiet", action="store_true", help="less raw serial noise")
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"Port {port} — motion monitor {args.sec}s", flush=True)
    link = MotLink(port)
    stats = Stats()
    csv_file = None
    csv_writer = None

    try:
        link.send("TEST PING")
        if not link.wait_ok("PONG", 3):
            print("!! no PONG — is firmware running with TEST harness?", flush=True)
            return 1

        link.send("TEST VER")
        link.wait_ok("VER", 2)

        for key, val in (
            ("gyro", args.gyro),
            ("bump", args.bump),
            ("lin", args.lin),
            ("det", args.det),
            ("duration", args.duration),
            ("settle", args.settle),
        ):
            if val is not None:
                link.send(f"TEST MOT SET {key} {val}")
                link.wait_ok("MOT_SET", 2)
        link.send(f"TEST MOT SET period {args.period}")
        link.wait_ok("MOT_SET", 2)

        link.send("TEST MOT RESET")
        link.wait_ok("MOT_RESET", 2)
        link.send("TEST MOT CFG")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            for line in link.read_lines():
                if line.startswith("[TEST] MOT"):
                    print(line, flush=True)

        if args.log:
            csv_file = open(args.log, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(CSV_HEADER)

        print(
            "time_s  flags  gd/thr  bd/bthr  ld/lthr  mode     age   enter pulse%  exit",
            flush=True,
        )
        print("-" * 100, flush=True)
        print("Flags: M=MOT G=gyroBusy B=bumpBusy L=linBusy H=hold W=wasMoving", flush=True)
        print("Drive / park now… Ctrl+C to stop early", flush=True)

        link.send(f"TEST MOT STREAM {args.sec}")
        link.wait_ok("MOT_STREAM", 3)

        deadline = time.monotonic() + args.sec + 3
        last_print = 0.0
        last_line_at = time.monotonic()
        reboot_n = 0
        while time.monotonic() < deadline:
            try:
                lines = link.read_lines()
            except Exception as e:
                print(f"!! serial error: {e} — waiting reconnect…", flush=True)
                time.sleep(1.0)
                try:
                    link.ser.close()
                except Exception:
                    pass
                try:
                    link = MotLink(port)
                    print("!! reconnected", flush=True)
                except Exception as e2:
                    print(f"!! reconnect fail: {e2}", flush=True)
                continue

            if lines:
                last_line_at = time.monotonic()

            for line in lines:
                if "[RESET] reason=" in line or ("Версия: V " in line) or (
                    "Kamaz" in line and "FreeRTOS" in line
                ):
                    reboot_n += 1
                    print(f"*** REBOOT/BOOT #{reboot_n}: {line}", flush=True)
                    try:
                        link.send(f"TEST MOT STREAM {max(30, int(deadline - time.monotonic()))}")
                    except Exception:
                        pass
                if "STREAM end" in line:
                    deadline = time.monotonic()
                s = parse_mot(line)
                if s:
                    stats.add(s)
                    if csv_writer:
                        csv_writer.writerow(sample_to_csv_row(s))
                        csv_file.flush()
                    now = time.monotonic()
                    edge = s.mot or s.busy or s.bbusy or s.lbusy
                    if args.period >= 200 or now - last_print >= 0.2 or edge:
                        print(fmt_row(s), flush=True)
                        last_print = now
                elif EDGE_RE.search(line):
                    print(f"*** {line}", flush=True)
                elif not args.quiet and (
                    line.startswith("[IMU]")
                    or line.startswith("[MOVEMENT]")
                    or line.startswith("[TEST] MOT")
                    or line.startswith("[RESET]")
                    or line.startswith("[WDT]")
                ):
                    print(line, flush=True)

            if time.monotonic() - last_line_at > 8.0:
                print("!! serial quiet >8s (possible reboot/USB) — probing PING", flush=True)
                try:
                    link.send("TEST PING")
                except Exception:
                    pass
                last_line_at = time.monotonic()
            time.sleep(0.02)

        if reboot_n:
            print(f"NOTE: observed {reboot_n} boot/reset marker(s) during session", flush=True)

        link.send("TEST MOT STATS")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            for line in link.read_lines():
                if "MOT_STATS" in line or line.startswith("[TEST] MOT"):
                    print(line, flush=True)
            time.sleep(0.05)

        print("-" * 100, flush=True)
        print("SUMMARY", stats.summary(), flush=True)
        if args.log:
            print(f"CSV saved: {args.log}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        print("SUMMARY", stats.summary(), flush=True)
        return 0
    finally:
        try:
            link.send("TEST MOT STREAM 1")
        except Exception:
            pass
        if csv_file:
            csv_file.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
