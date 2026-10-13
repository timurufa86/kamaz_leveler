#!/usr/bin/env python3
"""Capture / analyze raw [DMP] stream to find STAT vs DRIVE features.

Usage:
  python _dmp_motion_discover.py COM4 capture --label idle_on --sec 90 --log runs/dmp_idle.csv
  python _dmp_motion_discover.py COM4 capture --label drive --sec 120 --log runs/dmp_drive.csv
  python _dmp_motion_discover.py analyze --idle runs/dmp_idle.csv --drive runs/dmp_drive.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from _motion_monitor import MotLink, pick_port

DMP_RE = re.compile(
    r"\[DMP\] t=(\d+) nPkt=(\d+) int=0x([0-9A-Fa-f]+) mot=(\d) motSt=0x([0-9A-Fa-f]+) "
    r"gx=([-0-9.]+) gy=([-0-9.]+) gz=([-0-9.]+) gRms=([-0-9.]+) "
    r"ax=([-0-9.]+) ay=([-0-9.]+) az=([-0-9.]+) aRms=([-0-9.]+) "
    r"bx=([-0-9.]+) by=([-0-9.]+) bz=([-0-9.]+) "
    r"yaw=([-0-9.]+) pitch=([-0-9.]+) roll=([-0-9.]+) "
    r"gEma=([-0-9.]+) bEma=([-0-9.]+) lEma=([-0-9.]+) "
    r"gdlt=([-0-9.]+) bdlt=([-0-9.]+) ldlt=([-0-9.]+) "
    r"hold=(\d) was=(\d) pulse=(\d+) need=(\d+) mode=(\w+)"
)

CSV_FIELDS = [
    "t_ms", "nPkt", "int", "mot", "mot_st",
    "gx", "gy", "gz", "gRms",
    "ax", "ay", "az", "aRms",
    "bx", "by", "bz",
    "yaw", "pitch", "roll",
    "gEma", "bEma", "lEma", "gdlt", "bdlt", "ldlt",
    "hold", "was", "pulse", "need", "mode",
    # derived for analysis
    "abs_gx", "abs_gy", "abs_gz",
    "abs_ax", "abs_ay", "abs_az",
    "abs_bx", "abs_by", "abs_bz",
]

FEATURE_KEYS = [
    "gRms", "abs_gx", "abs_gy", "abs_gz",
    "aRms", "abs_ax", "abs_ay", "abs_az",
    "abs_bx", "abs_by", "abs_bz",
    "gdlt", "bdlt", "ldlt",
    "gEma", "bEma", "lEma",
    "mot", "hold",
]


def parse_dmp(line: str) -> Optional[Dict]:
    m = DMP_RE.search(line)
    if not m:
        return None
    g = m.groups()
    gx, gy, gz = float(g[5]), float(g[6]), float(g[7])
    ax, ay, az = float(g[9]), float(g[10]), float(g[11])
    bx, by, bz = float(g[13]), float(g[14]), float(g[15])
    return {
        "t_ms": int(g[0]),
        "nPkt": int(g[1]),
        "int": int(g[2], 16),
        "mot": int(g[3]),
        "mot_st": int(g[4], 16),
        "gx": gx, "gy": gy, "gz": gz, "gRms": float(g[8]),
        "ax": ax, "ay": ay, "az": az, "aRms": float(g[12]),
        "bx": bx, "by": by, "bz": bz,
        "yaw": float(g[16]), "pitch": float(g[17]), "roll": float(g[18]),
        "gEma": float(g[19]), "bEma": float(g[20]), "lEma": float(g[21]),
        "gdlt": float(g[22]), "bdlt": float(g[23]), "ldlt": float(g[24]),
        "hold": int(g[25]), "was": int(g[26]),
        "pulse": int(g[27]), "need": int(g[28]), "mode": g[29],
        "abs_gx": abs(gx), "abs_gy": abs(gy), "abs_gz": abs(gz),
        "abs_ax": abs(ax), "abs_ay": abs(ay), "abs_az": abs(az),
        "abs_bx": abs(bx), "abs_by": abs(by), "abs_bz": abs(bz),
    }


def percentile(xs: List[float], p: float) -> float:
    if not xs:
        return 0.0
    ys = sorted(xs)
    if len(ys) == 1:
        return ys[0]
    k = (len(ys) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return ys[int(k)]
    return ys[f] * (c - k) + ys[c] * (k - f)


def load_csv(path: str) -> List[Dict]:
    rows: List[Dict] = []
    with open(path, newline="", encoding="utf-8") as f:
        for d in csv.DictReader(f):
            row = {}
            for k in CSV_FIELDS:
                if k == "mode":
                    row[k] = d.get(k, "")
                elif k in d and d[k] != "":
                    if k in ("t_ms", "nPkt", "int", "mot", "mot_st", "hold", "was", "pulse", "need"):
                        row[k] = int(float(d[k]))
                    else:
                        row[k] = float(d[k])
            rows.append(row)
    return rows


def col(rows: List[Dict], key: str) -> List[float]:
    out = []
    for r in rows:
        if key not in r:
            continue
        v = r[key]
        if isinstance(v, (int, float)) and not (isinstance(v, float) and math.isnan(v)):
            out.append(float(v))
    return out


def stats_col(xs: List[float]) -> Dict[str, float]:
    n = len(xs) or 1
    return {
        "n": float(len(xs)),
        "mean": sum(xs) / n if xs else 0.0,
        "p50": percentile(xs, 0.50),
        "p95": percentile(xs, 0.95),
        "p99": percentile(xs, 0.99),
        "max": max(xs) if xs else 0.0,
    }


def capture(port: str, sec: int, period: int, log: str, label: str) -> int:
    link = MotLink(port)
    Path(log).parent.mkdir(parents=True, exist_ok=True)
    n = 0
    try:
        link.send("TEST PING")
        if not link.wait_ok("PONG", 4):
            print("!! no PONG", flush=True)
            return 1
        link.send("TEST VER")
        link.wait_ok("VER", 2)
        link.send(f"TEST DMP SET period {period}")
        link.wait_ok("DMP_SET", 2)
        print(f"=== DMP CAPTURE label={label} {sec}s -> {log} ===", flush=True)
        link.send(f"TEST DMP STREAM {sec}")
        if not link.wait_ok("DMP_STREAM", 3):
            print("!! no DMP_STREAM (need firmware 10.10.7+)", flush=True)
            return 1

        with open(log, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
            w.writeheader()
            deadline = time.monotonic() + sec + 3
            last_print = 0.0
            while time.monotonic() < deadline:
                for line in link.read_lines():
                    if "STREAM end" in line and "[DMP]" in line:
                        deadline = time.monotonic()
                    row = parse_dmp(line)
                    if not row:
                        continue
                    w.writerow(row)
                    f.flush()
                    n += 1
                    now = time.monotonic()
                    if now - last_print >= 1.0:
                        print(
                            f"  t={row['t_ms']/1000:.1f} gRms={row['gRms']:.0f} "
                            f"aRms={row['aRms']:.0f} gdlt={row['gdlt']:.0f} "
                            f"ldlt={row['ldlt']:.0f} hold={row['hold']} mode={row['mode']}",
                            flush=True,
                        )
                        last_print = now
                time.sleep(0.02)
        print(f"DONE n={n} CSV={log}", flush=True)
        if n < 10:
            print("!! too few samples", flush=True)
            return 2
        # quick summary
        rows = load_csv(log)
        for key in ("gRms", "aRms", "gdlt", "bdlt", "ldlt", "abs_gx", "abs_gy", "abs_gz"):
            st = stats_col(col(rows, key))
            print(
                f"  {key}: mean={st['mean']:.1f} p95={st['p95']:.1f} p99={st['p99']:.1f} max={st['max']:.1f}",
                flush=True,
            )
        return 0
    finally:
        try:
            link.send("TEST DMP STREAM 1")
        except Exception:
            pass
        link.close()


def analyze(idle_path: str, drive_path: str) -> int:
    idle = load_csv(idle_path)
    drive = load_csv(drive_path)
    if len(idle) < 10 or len(drive) < 10:
        print("!! need more samples in both CSVs", flush=True)
        return 2
    print(f"idle_on n={len(idle)}  drive n={len(drive)}", flush=True)
    ranked: List[Tuple[float, str, Dict]] = []
    for key in FEATURE_KEYS:
        si = stats_col(col(idle, key))
        sd = stats_col(col(drive, key))
        thr = si["p95"]
        # avoid thr=0 making everything pass
        if thr < 1e-6:
            thr = max(si["p99"], 1.0)
        drive_vals = col(drive, key)
        idle_vals = col(idle, key)
        drive_above = sum(1 for v in drive_vals if v > thr) / (len(drive_vals) or 1)
        idle_above = sum(1 for v in idle_vals if v > thr) / (len(idle_vals) or 1)
        sep = drive_above - idle_above  # want high
        ratio = (sd["p95"] / si["p95"]) if si["p95"] > 1e-6 else (sd["p95"] if sd["p95"] > 0 else 0.0)
        score = sep * 2.0 + min(ratio, 20.0) * 0.05
        ranked.append(
            (
                score,
                key,
                {
                    "idle_p95": si["p95"],
                    "drive_p95": sd["p95"],
                    "ratio": ratio,
                    "drive_above_idle_p95": drive_above,
                    "idle_above_idle_p95": idle_above,
                    "sep": sep,
                    "suggest_thr": max(si["p95"] * 1.3, si["p99"]),
                },
            )
        )
    ranked.sort(key=lambda x: x[0], reverse=True)
    print("--- RANKED FEATURES (best first) ---", flush=True)
    for i, (score, key, m) in enumerate(ranked[:12], 1):
        print(
            f"{i:2}. {key:8} score={score:.3f} "
            f"idle_p95={m['idle_p95']:.1f} drive_p95={m['drive_p95']:.1f} "
            f"ratio={m['ratio']:.2f} drive%>idle_p95={m['drive_above_idle_p95']*100:.1f}% "
            f"idle%>p95={m['idle_above_idle_p95']*100:.1f}% "
            f"suggest_thr~{m['suggest_thr']:.0f}",
            flush=True,
        )
    top = [r for r in ranked if r[0] > 0.3][:2]
    if not top:
        top = ranked[:2]
    print("--- PICK (top 1-2) ---", flush=True)
    for score, key, m in top:
        print(
            f"  USE {key}: thr~{m['suggest_thr']:.0f} "
            f"(idle_p95={m['idle_p95']:.1f}, drive_above={m['drive_above_idle_p95']*100:.0f}%)",
            flush=True,
        )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="DMP motion feature discovery")
    sub = ap.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture")
    cap.add_argument("port", nargs="?", default=None)
    cap.add_argument("--label", required=True, choices=("idle_on", "drive", "other"))
    cap.add_argument("--sec", type=int, default=90)
    cap.add_argument("--period", type=int, default=100)
    cap.add_argument("--log", type=str, default=None)

    an = sub.add_parser("analyze")
    an.add_argument("--idle", required=True)
    an.add_argument("--drive", required=True)

    args = ap.parse_args()
    if args.cmd == "capture":
        port = pick_port(args.port)
        print(f"Port {port}", flush=True)
        log = args.log
        if not log:
            os.makedirs("runs", exist_ok=True)
            log = f"runs/dmp_{args.label}_{int(time.time())}.csv"
        return capture(port, args.sec, args.period, log, args.label)
    return analyze(args.idle, args.drive)


if __name__ == "__main__":
    sys.exit(main())
