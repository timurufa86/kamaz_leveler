#!/usr/bin/env python3
"""MPU filter tuner for Kamaz-Leveler (firmware 9.2.0+).

Uses TEST IMU STREAM / STATS / PRESET / SET / SAVE over serial.

Usage:
  python _mpu_tune.py COM4                 # compare presets, apply best, save
  python _mpu_tune.py COM4 --stats 5       # only measure current config
  python _mpu_tune.py COM4 --preset BAL --save
  python _mpu_tune.py COM4 --stream 3
  python _mpu_tune.py COM4 --set poll 20 --set ema 0.28 --stats 4 --save
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(2)


STATS_RE = re.compile(
    r"\[IMU\] STATS n=(\d+) hz=([\d.]+) std=([\d.]+),([\d.]+) "
    r"pp=([\d.]+),([\d.]+) spikes=(\d+) maxStep=([\d.]+) "
    r"mean=([-\d.]+),([-\d.]+)"
)
STREAM_RE = re.compile(
    r"\[IMU\] S (\d+) raw=([-\d.]+),([-\d.]+) out=([-\d.]+),([-\d.]+) n=(\d+)"
)
CFG_RE = re.compile(
    r"\[TEST\] IMU_CFG mea=([\d.]+) est=([\d.]+) q=([\d.]+) poll=(\d+) fifo=(\d+) "
    r"ema=([\d.]+)/([\d.]+) thr=([\d.]+) slew=([\d.]+) redraw=([\d.]+) det=(\d+)"
)


def pick_port(preferred: Optional[str]) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "uart" in desc or "cp210" in desc or "ch340" in desc:
            return p.device
    if ports:
        return ports[0].device
    raise SystemExit("No serial ports found")


@dataclass
class Stats:
    n: int
    hz: float
    std_x: float
    std_y: float
    pp_x: float
    pp_y: float
    spikes: int
    max_step: float
    mean_x: float
    mean_y: float

    @property
    def std_max(self) -> float:
        return max(self.std_x, self.std_y)

    @property
    def pp_max(self) -> float:
        return max(self.pp_x, self.pp_y)

    def score(self) -> float:
        """Lower is better: noise + spikes; strongly prefer >=35 Hz."""
        rate_pen = 0.0
        if self.hz < 35.0:
            rate_pen = (35.0 - self.hz) * 0.15
        return (
            self.std_max * 12.0
            + self.pp_max * 2.0
            + self.spikes * 0.5
            + self.max_step * 3.0
            + rate_pen
        )


class MpuLink:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.buf = ""
        self.lines: List[str] = []

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _pump(self) -> None:
        n = self.ser.in_waiting
        if n:
            chunk = self.ser.read(n).decode("utf-8", errors="replace")
            self.buf += chunk
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                line = line.rstrip("\r")
                if line:
                    self.lines.append(line)
                    safe = line.encode("ascii", errors="replace").decode("ascii")
                    print(f"  << {safe}")

    def wait(self, pred, timeout: float = 8.0) -> Optional[str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            for i, line in enumerate(self.lines):
                if pred(line):
                    self.lines = self.lines[i + 1 :]
                    return line
            time.sleep(0.02)
        return None

    def send(self, cmd: str) -> None:
        print(f"  >> {cmd}")
        self.ser.write((cmd.strip() + "\n").encode("utf-8"))
        self.ser.flush()

    def cmd_ok(self, cmd: str, token: str, timeout: float = 6.0) -> bool:
        self.send(cmd)
        line = self.wait(lambda L: L.startswith("[TEST] OK") and token in L, timeout)
        return line is not None

    def read_cfg(self) -> Optional[Dict[str, float]]:
        self.send("TEST IMU CFG")
        line = self.wait(lambda L: "[TEST] IMU_CFG " in L, timeout=4.0)
        self.wait(lambda L: "[TEST] OK IMU_CFG" in L, timeout=2.0)
        if not line:
            return None
        m = CFG_RE.search(line)
        if not m:
            return None
        keys = [
            "mea", "est", "q", "poll", "fifo", "ema", "spike", "thr", "slew", "redraw", "det"
        ]
        vals = [float(x) for x in m.groups()]
        return dict(zip(keys, vals))

    def measure(self, sec: int = 5) -> Optional[Stats]:
        self.lines.clear()
        self.send(f"TEST IMU STATS {sec}")
        self.wait(lambda L: "IMU_STATS_START" in L, timeout=3.0)
        line = self.wait(lambda L: "[IMU] STATS n=" in L, timeout=sec + 8.0)
        self.wait(lambda L: "[TEST] OK IMU_STATS" in L, timeout=3.0)
        if not line:
            return None
        m = STATS_RE.search(line)
        if not m:
            return None
        g = m.groups()
        return Stats(
            n=int(g[0]),
            hz=float(g[1]),
            std_x=float(g[2]),
            std_y=float(g[3]),
            pp_x=float(g[4]),
            pp_y=float(g[5]),
            spikes=int(g[6]),
            max_step=float(g[7]),
            mean_x=float(g[8]),
            mean_y=float(g[9]),
        )

    def stream(self, sec: int = 3) -> Tuple[int, float]:
        """Return (sample_count, max_|out-raw| proxy via peak |step| of out)."""
        self.lines.clear()
        self.send(f"TEST IMU STREAM {sec}")
        self.wait(lambda L: "IMU_STREAM" in L, timeout=3.0)
        deadline = time.monotonic() + sec + 2.0
        samples = 0
        max_step = 0.0
        prev = None
        while time.monotonic() < deadline:
            self._pump()
            while self.lines:
                line = self.lines.pop(0)
                m = STREAM_RE.search(line)
                if not m:
                    if "STREAM end" in line:
                        deadline = 0
                    continue
                samples += 1
                ox, oy = float(m.group(4)), float(m.group(5))
                if prev is not None:
                    step = max(abs(ox - prev[0]), abs(oy - prev[1]))
                    if step > max_step:
                        max_step = step
                prev = (ox, oy)
            time.sleep(0.01)
        return samples, max_step


def print_stats(label: str, s: Stats) -> None:
    print(
        f"=== {label}: hz={s.hz:.1f} std={s.std_x:.4f}/{s.std_y:.4f} "
        f"pp={s.pp_x:.3f}/{s.pp_y:.3f} spikes={s.spikes} maxStep={s.max_step:.3f} "
        f"score={s.score():.3f}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Tune Kamaz MPU filters via serial")
    ap.add_argument("port", nargs="?", help="COM port")
    ap.add_argument("--stats", type=int, default=0, help="measure current N seconds")
    ap.add_argument("--stream", type=int, default=0, help="stream N seconds")
    ap.add_argument("--preset", choices=["BAL", "SMOOTH", "FAST"], help="apply preset")
    ap.add_argument("--set", nargs=2, action="append", metavar=("KEY", "VAL"), help="IMU SET")
    ap.add_argument("--save", action="store_true", help="TEST IMU SAVE")
    ap.add_argument("--tune", action="store_true", default=False, help="compare presets and pick best")
    ap.add_argument("--sec", type=int, default=4, help="seconds per preset measure")
    args = ap.parse_args()

    # default action: full tune if nothing else requested
    do_tune = args.tune or (
        not args.stats and not args.stream and not args.preset and not args.set
    )

    port = pick_port(args.port)
    print(f"=== MPU tune on {port} ===")
    link = MpuLink(port)
    try:
        # Boot spam after flash — wait until TEST harness answers
        ok = False
        for _ in range(12):
            link.lines.clear()
            if link.cmd_ok("TEST PING", "PONG", timeout=2.5):
                ok = True
                break
            time.sleep(1.0)
        if not ok:
            print("!! no PONG — is 9.2.0+ flashed and awake?")
            return 1

        cfg = link.read_cfg()
        if cfg:
            print("current:", cfg)

        if args.set:
            for key, val in args.set:
                if not link.cmd_ok(f"TEST IMU SET {key} {val}", "IMU_SET", timeout=4):
                    print(f"!! SET {key} failed")
                    return 1

        if args.preset:
            if not link.cmd_ok(f"TEST IMU PRESET {args.preset}", "IMU_PRESET", timeout=4):
                print("!! PRESET failed")
                return 1

        if args.stream:
            n, mx = link.stream(args.stream)
            print(f"stream samples={n} max_out_step={mx:.3f}")

        if args.stats:
            s = link.measure(args.stats)
            if not s:
                print("!! STATS failed")
                return 1
            print_stats("current", s)

        if do_tune:
            results: List[Tuple[str, Stats]] = []
            for name in ("BAL", "SMOOTH", "FAST"):
                print(f"\n--- preset {name} ---")
                if not link.cmd_ok(f"TEST IMU PRESET {name}", "IMU_PRESET", timeout=4):
                    print(f"!! preset {name} failed")
                    continue
                time.sleep(0.4)  # settle after filter reset
                s = link.measure(args.sec)
                if not s:
                    print(f"!! stats {name} failed")
                    continue
                print_stats(name, s)
                results.append((name, s))

            if not results:
                print("!! no successful measurements")
                return 1

            best_name, best = min(results, key=lambda t: t[1].score())
            print(f"\n>>> BEST preset: {best_name} (score={best.score():.3f})")
            # Prefer BAL if within 8% of best — better responsiveness
            bal = next((s for n, s in results if n == "BAL"), None)
            if bal and bal.score() <= best.score() * 1.08 and best_name != "BAL":
                best_name, best = "BAL", bal
                print(f">>> prefer BAL (within 8%): score={best.score():.3f}")

            link.cmd_ok(f"TEST IMU PRESET {best_name}", "IMU_PRESET", timeout=4)

            # Fine nudge: if spikes high, lower ema a bit / raise mea
            if best.spikes > 3 or best.max_step > 0.35:
                print(">>> fine: quieter (mea+1, ema-0.04, slew-2)")
                link.cmd_ok("TEST IMU SET mea 7", "IMU_SET", timeout=3)
                link.cmd_ok("TEST IMU SET ema 0.21", "IMU_SET", timeout=3)
                link.cmd_ok("TEST IMU SET slew 10", "IMU_SET", timeout=3)
                time.sleep(0.3)
                s2 = link.measure(args.sec)
                if s2:
                    print_stats("fine", s2)
                    if s2.score() > best.score() * 1.15:
                        print(">>> fine worse — revert to preset")
                        link.cmd_ok(f"TEST IMU PRESET {best_name}", "IMU_PRESET", timeout=4)

            args.save = True

        if args.save:
            if not link.cmd_ok("TEST IMU SAVE", "IMU_SAVE", timeout=8):
                print("!! SAVE failed")
                return 1
            print("saved to LittleFS")
            cfg = link.read_cfg()
            if cfg:
                print("final:", cfg)

        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
