#!/usr/bin/env python3
"""Serial MOT stream monitor for Kamaz-Leveler 10.4.0+ (gyro/bump/lin Δ EMA)."""
from __future__ import annotations

import argparse
import csv
import re
import sys
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

# 10.4.0: gyro= raw gdlt=Δ thr= busy= bump= bdlt=Δ ...
MOT_RE = re.compile(
    r"\[MOT\] t=(\d+) mot=(\d+) gyro=([\-\d.]+)(?: gdlt=([\-\d.]+))? thr=([\-\d.]+) busy=(\d+) "
    r"(?:bump=([\-\d.]+)(?: bdlt=([\-\d.]+))? bthr=([\-\d.]+) bbusy=(\d+) )?"
    r"lin=([\-\d.]+)(?: ldlt=([\-\d.]+))? lthr=([\-\d.]+) lbusy=(\d+) hold=(\d+) was=(\d+) mode=(\S+)"
)


def main() -> int:
    ap = argparse.ArgumentParser(description="Log TEST MOT STREAM lines")
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-o", "--out", default="motion.csv")
    ap.add_argument("-s", "--stream-sec", type=int, default=60)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(f"TEST MOT STREAM {args.stream_sec}\n".encode())
    print(f"Streaming {args.stream_sec}s from {args.port} → {args.out}")

    n = 0
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_ms",
                "mot",
                "gyro",
                "gdlt",
                "thr",
                "busy",
                "bump",
                "bdlt",
                "bthr",
                "bbusy",
                "lin",
                "ldlt",
                "lthr",
                "lbusy",
                "hold",
                "was",
                "mode",
            ]
        )
        deadline = time.time() + args.stream_sec + 5
        buf = ""
        while time.time() < deadline:
            chunk = ser.read(512).decode("utf-8", errors="ignore")
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                print(line)
                m = MOT_RE.search(line)
                if m:
                    g = [x if x is not None else "" for x in m.groups()]
                    w.writerow(g)
                    n += 1
                    f.flush()
                if "STREAM end" in line:
                    deadline = time.time()
                    break

    ser.close()
    print(f"Wrote {n} rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
