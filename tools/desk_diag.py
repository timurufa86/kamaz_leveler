#!/usr/bin/env python3
"""Diagnose false MOT triggers while device sits on desk (10.4.0+ gdlt/bdlt)."""
from __future__ import annotations

import re
import statistics as st
import sys
import time

import serial

MOT_RE = re.compile(
    r"mot=(\d+) motSt=0x([0-9A-Fa-f]+) gyro=([\-\d.]+) gdlt=([\-\d.]+) thr=(\d+) busy=(\d+) "
    r"bump=([\-\d.]+) bdlt=([\-\d.]+) bthr=(\d+) bbusy=(\d+) lin=([\-\d.]+) ldlt=([\-\d.]+) "
    r"lthr=(\d+) lbusy=(\d+) hold=(\d+) was=(\d+) mode=(\S+) age=([\-\d.]+) "
    r"enter=([\-\d.]+) settleLeft=([\-\d.]+)"
)


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    sec = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    ser = serial.Serial(port, 115200, timeout=0.25)
    time.sleep(0.6)
    ser.reset_input_buffer()
    ser.write(b"TEST MOT CFG\n")
    time.sleep(0.4)
    print(ser.read(1500).decode("utf-8", errors="replace"))

    ser.reset_input_buffer()
    ser.write(f"TEST MOT STREAM {sec}\n".encode())
    deadline = time.time() + sec + 4
    rows: list[dict] = []
    buf = ""
    while time.time() < deadline:
        chunk = ser.read(2048).decode("utf-8", errors="ignore")
        if not chunk:
            continue
        buf += chunk
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            m = MOT_RE.search(line)
            if m:
                rows.append(
                    {
                        "mot": int(m[1]),
                        "gyro": float(m[3]),
                        "gdlt": float(m[4]),
                        "thr": int(m[5]),
                        "busy": int(m[6]),
                        "bump": float(m[7]),
                        "bdlt": float(m[8]),
                        "bthr": int(m[9]),
                        "bbusy": int(m[10]),
                        "ldlt": float(m[12]),
                        "lthr": int(m[13]),
                        "lbusy": int(m[14]),
                        "hold": int(m[15]),
                        "mode": m[17],
                        "enter": float(m[19]),
                    }
                )
            if "STREAM end" in line:
                deadline = time.time()

    print(f"samples={len(rows)}")
    if not rows:
        return 1
    for k in ["gyro", "gdlt", "bump", "bdlt", "ldlt"]:
        xs = sorted(r[k] for r in rows)
        p95 = xs[int(0.95 * (len(xs) - 1))]
        print(f"{k}: min={min(xs):.0f} avg={st.mean(xs):.1f} max={max(xs):.0f} p95={p95:.0f}")
    n = len(rows)
    print(
        f"mot%={100 * sum(r['mot'] for r in rows) / n:.1f} "
        f"busy%={100 * sum(r['busy'] for r in rows) / n:.1f} "
        f"bbusy%={100 * sum(r['bbusy'] for r in rows) / n:.1f} "
        f"lbusy%={100 * sum(r['lbusy'] for r in rows) / n:.1f} "
        f"hold%={100 * sum(r['hold'] for r in rows) / n:.1f}"
    )
    modes: dict[str, int] = {}
    for r in rows:
        modes[r["mode"]] = modes.get(r["mode"], 0) + 1
    print("modes", modes)
    spikes = [r for r in rows if r["busy"] or r["bbusy"] or r["lbusy"] or r["mot"]]
    print(f"trigger_lines={len(spikes)}")
    for r in spikes[:15]:
        print(
            f"  gdlt={r['gdlt']:.0f}/{r['thr']} busy={r['busy']} "
            f"bdlt={r['bdlt']:.0f}/{r['bthr']} bbusy={r['bbusy']} "
            f"ldlt={r['ldlt']:.0f}/{r['lthr']} lbusy={r['lbusy']} "
            f"hold={r['hold']} mode={r['mode']} enter={r['enter']:.1f}"
        )
    if len(spikes) > 15:
        print(f"  ... +{len(spikes) - 15} more")
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
