#!/usr/bin/env python3
"""2-hour serial soak test: heap/lag/mode flips for Kamaz-Leveler.

Usage:
  python _stability_soak.py COM4
  python _stability_soak.py COM4 --hours 2 --log soak_log.txt

Every sample interval: STATUS + HEAP (+ periodic STACK/IMU/STATS).
Every mode interval: toggle MANUAL <-> AUTO.
Detects: hang (no RX), heap decline, slow STATUS round-trip.
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, TextIO

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(2)


STATUS_RE = re.compile(
    r"\[TEST\] STATUS menu=\d+ .* mode=(\d+) mpu=(\d+) heap=(\d+) minHeap=(\d+) maxBlk=(\d+) uptime=(\d+)s"
)
HEAP_RE = re.compile(r"\[TEST\] HEAP free=(\d+) min=(\d+)|Free heap: (\d+)|\[MEM\] Free heap: (\d+)")
HB_RE = re.compile(r"\[TEST\] HB |\[HB\]|heartbeat", re.I)


def pick_port(preferred: Optional[str]) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(x in desc for x in ("usb", "uart", "cp210", "ch340", "serial")):
            return p.device
    if not ports:
        raise SystemExit("No serial ports")
    return ports[0].device


class Soak:
    def __init__(self, port: str, log_path: Path, hang_sec: float = 15.0):
        self.port = port
        self.hang_sec = hang_sec
        self.log_path = log_path
        self.fp: TextIO = log_path.open("w", encoding="utf-8", buffering=1)
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.buf = ""
        self.lines: List[str] = []
        self.last_rx = time.monotonic()
        self.samples = 0
        self.mode_flips = 0
        self.hangs = 0
        self.slow_rtt = 0
        self.heap_start: Optional[int] = None
        self.heap_min_seen: Optional[int] = None
        self.heap_last: Optional[int] = None
        self.min_heap_fw: Optional[int] = None
        self.max_blk_min: Optional[int] = None
        self.rtt_max_ms = 0.0
        self.errors: List[str] = []

    def log(self, msg: str) -> None:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line, flush=True)
        self.fp.write(line + "\n")

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
        self.fp.close()

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

    def wait(self, pred, timeout: float) -> Optional[str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            for i, line in enumerate(self.lines):
                if pred(line):
                    self.lines = self.lines[i + 1 :]
                    return line
            if time.monotonic() - self.last_rx > self.hang_sec:
                raise TimeoutError(f"HANG: no RX for {self.hang_sec:.0f}s")
            time.sleep(0.02)
        return None

    def send(self, cmd: str) -> None:
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))
        self.ser.flush()

    def cmd_ok(self, cmd: str, token: str, timeout: float = 6.0) -> bool:
        t0 = time.monotonic()
        self.send(cmd)
        line = self.wait(lambda L: L.startswith("[TEST] OK") and token in L, timeout)
        rtt = (time.monotonic() - t0) * 1000.0
        if rtt > self.rtt_max_ms:
            self.rtt_max_ms = rtt
        if rtt > 1500:
            self.slow_rtt += 1
            self.log(f"WARN slow RTT {rtt:.0f}ms cmd={cmd}")
        return line is not None

    def sample_status(self) -> Optional[dict]:
        self.lines.clear()
        t0 = time.monotonic()
        self.send("TEST STATUS")
        line = self.wait(lambda L: "[TEST] STATUS menu=" in L, timeout=8.0)
        self.wait(lambda L: "[TEST] OK STATUS" in L, timeout=3.0)
        rtt = (time.monotonic() - t0) * 1000.0
        if rtt > self.rtt_max_ms:
            self.rtt_max_ms = rtt
        if rtt > 1500:
            self.slow_rtt += 1
            self.log(f"WARN slow STATUS RTT {rtt:.0f}ms")
        if not line:
            return None
        m = STATUS_RE.search(line)
        if not m:
            self.log(f"WARN unparsed STATUS: {line[:160]}")
            return None
        d = {
            "mode": int(m.group(1)),
            "mpu": int(m.group(2)),
            "heap": int(m.group(3)),
            "minHeap": int(m.group(4)),
            "maxBlk": int(m.group(5)),
            "uptime": int(m.group(6)),
            "rtt_ms": rtt,
        }
        self.heap_last = d["heap"]
        if self.heap_start is None:
            self.heap_start = d["heap"]
        if self.heap_min_seen is None or d["heap"] < self.heap_min_seen:
            self.heap_min_seen = d["heap"]
        if self.min_heap_fw is None or d["minHeap"] < self.min_heap_fw:
            self.min_heap_fw = d["minHeap"]
        if self.max_blk_min is None or d["maxBlk"] < self.max_blk_min:
            self.max_blk_min = d["maxBlk"]
        self.samples += 1
        return d

    def flip_mode(self, to_auto: bool) -> bool:
        if to_auto:
            ok = self.cmd_ok("TEST MODE AUTO", "MODE_AUTO", timeout=5)
        else:
            ok = self.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=5)
        if ok:
            self.mode_flips += 1
        return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--sample-sec", type=float, default=30.0, help="STATUS period")
    ap.add_argument("--mode-sec", type=float, default=180.0, help="mode toggle period")
    ap.add_argument("--imu-sec", type=float, default=300.0, help="IMU STATS period")
    ap.add_argument("--stack-sec", type=float, default=600.0, help="STACK dump period")
    ap.add_argument("--log", default="", help="log file path")
    ap.add_argument("--hang-sec", type=float, default=20.0)
    args = ap.parse_args()

    port = pick_port(args.port)
    log_path = Path(args.log) if args.log else Path(
        f"soak_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    duration = args.hours * 3600.0

    s = Soak(port, log_path, hang_sec=args.hang_sec)
    s.log(f"=== SOAK START port={port} hours={args.hours} log={log_path} ===")
    try:
        # wait for device
        for i in range(15):
            s.lines.clear()
            if s.cmd_ok("TEST PING", "PONG", timeout=2.5):
                break
            time.sleep(1)
        else:
            s.log("FAIL: no PONG")
            return 1

        s.cmd_ok("TEST HEARTBEAT ON", "HEARTBEAT_ON", timeout=3)
        s.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=4)

        t0 = time.monotonic()
        next_sample = t0
        next_mode = t0 + args.mode_sec
        next_imu = t0 + args.imu_sec
        next_stack = t0 + 60.0  # first stack soon
        want_auto = True
        last_summary = t0

        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= duration:
                break

            # hang watchdog even between samples
            s._pump()
            if now - s.last_rx > args.hang_sec:
                s.hangs += 1
                msg = f"HANG detected (no RX {args.hang_sec:.0f}s) at t={elapsed/60:.1f}min"
                s.log(msg)
                s.errors.append(msg)
                # try recover
                try:
                    s.ser.reset_input_buffer()
                    s.cmd_ok("TEST PING", "PONG", timeout=5)
                except Exception as e:
                    s.log(f"recover failed: {e}")
                    break

            if now >= next_sample:
                try:
                    d = s.sample_status()
                    if d:
                        drop = (s.heap_start or d["heap"]) - d["heap"]
                        s.log(
                            f"SAMPLE t={elapsed/60:6.1f}m up={d['uptime']:5d}s mode={d['mode']} "
                            f"heap={d['heap']} minFw={d['minHeap']} maxBlk={d['maxBlk']} "
                            f"drop={drop} rtt={d['rtt_ms']:.0f}ms mpu={d['mpu']}"
                        )
                        if d["heap"] < 25000:
                            s.log(f"WARN low heap {d['heap']}")
                        if d["maxBlk"] < 12000:
                            s.log(f"WARN low maxBlk {d['maxBlk']}")
                    else:
                        s.log("WARN STATUS failed")
                except TimeoutError as e:
                    s.hangs += 1
                    s.log(str(e))
                    s.errors.append(str(e))
                next_sample = now + args.sample_sec

            if now >= next_mode:
                try:
                    name = "AUTO" if want_auto else "MANUAL"
                    ok = s.flip_mode(want_auto)
                    s.log(f"MODE -> {name} ok={ok} flips={s.mode_flips}")
                    want_auto = not want_auto
                except TimeoutError as e:
                    s.hangs += 1
                    s.log(f"MODE flip hang: {e}")
                    s.errors.append(str(e))
                next_mode = now + args.mode_sec

            if now >= next_imu:
                try:
                    s.send("TEST IMU STATS 3")
                    s.wait(lambda L: "IMU_STATS_START" in L, timeout=3)
                    st = s.wait(lambda L: "[IMU] STATS n=" in L, timeout=12)
                    s.wait(lambda L: "[TEST] OK IMU_STATS" in L, timeout=3)
                    if st:
                        s.log(f"IMU {st}")
                    else:
                        s.log("WARN IMU STATS missing")
                except TimeoutError as e:
                    s.hangs += 1
                    s.log(f"IMU hang: {e}")
                    s.errors.append(str(e))
                next_imu = now + args.imu_sec

            if now >= next_stack:
                try:
                    s.send("TEST STACK")
                    # collect a few stack lines
                    deadline = time.monotonic() + 4
                    got = []
                    while time.monotonic() < deadline:
                        s._pump()
                        while s.lines:
                            line = s.lines.pop(0)
                            if "stack=" in line or "Free heap" in line or "[TEST] OK STACK" in line:
                                got.append(line)
                            if "[TEST] OK STACK" in line:
                                deadline = 0
                        time.sleep(0.02)
                    for line in got:
                        s.log(f"STACK {line}")
                except Exception as e:
                    s.log(f"STACK err: {e}")
                next_stack = now + args.stack_sec

            if now - last_summary >= 600:
                s.log(
                    f"SUMMARY elapsed={elapsed/60:.1f}m samples={s.samples} flips={s.mode_flips} "
                    f"hangs={s.hangs} slowRTT={s.slow_rtt} heapNow={s.heap_last} "
                    f"heapMinSeen={s.heap_min_seen} minHeapFw={s.min_heap_fw} "
                    f"maxBlkMin={s.max_blk_min} rttMax={s.rtt_max_ms:.0f}ms"
                )
                last_summary = now

            time.sleep(0.2)

        # final
        s.cmd_ok("TEST MODE MANUAL", "MODE_MANUAL", timeout=5)
        d = s.sample_status()
        s.log("=== SOAK DONE ===")
        s.log(
            f"duration_h={args.hours} samples={s.samples} mode_flips={s.mode_flips} "
            f"hangs={s.hangs} slowRTT={s.slow_rtt}"
        )
        if s.heap_start is not None and s.heap_last is not None:
            s.log(
                f"heap start={s.heap_start} end={s.heap_last} "
                f"delta={s.heap_last - s.heap_start} minSeen={s.heap_min_seen} "
                f"minHeapFw={s.min_heap_fw} maxBlkMin={s.max_blk_min}"
            )
        s.log(f"rttMax={s.rtt_max_ms:.0f}ms errors={len(s.errors)}")
        for e in s.errors[:20]:
            s.log(f"ERR {e}")
        # heuristic verdict
        leak = 0
        if s.heap_start and s.heap_last:
            leak = s.heap_start - s.heap_last
        if s.hangs > 0:
            verdict = "FAIL (hangs)"
        elif leak > 15000:
            verdict = "SUSPECT heap leak"
        elif s.slow_rtt > 10:
            verdict = "SUSPECT lag (slow RTT)"
        elif s.min_heap_fw is not None and s.min_heap_fw < 20000:
            verdict = "SUSPECT low minHeap"
        else:
            verdict = "OK"
        s.log(f"VERDICT {verdict}")
        print(f"\nLog: {log_path.resolve()}")
        return 0 if verdict == "OK" else 2
    except KeyboardInterrupt:
        s.log("INTERRUPTED")
        return 130
    finally:
        s.close()


if __name__ == "__main__":
    # fix typo guard in case of edit mistakes
    sys.exit(main())
