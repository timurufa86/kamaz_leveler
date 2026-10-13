#!/usr/bin/env python3
"""On-vehicle STAT vs MOVEMENT capture, suggest thresholds, apply via TEST MOT SET.

Phases:
  stat    - park / idle (expect hold~0, no MOVEMENT enter)
  drive   - driving (expect MOVEMENT enter, measure busy%)
  session - guided STOY then EXAY (Enter between phases)
  suggest - from one or two CSVs (stat required; drive optional)
  apply   - push suggested or explicit SET values; optional SAVE

Usage:
  python _stat_motion_drive.py COM4 --phase session
  python _stat_motion_drive.py COM4 --phase stat --sec 90 --log runs/stat.csv
  python _stat_motion_drive.py COM4 --phase drive --sec 120 --log runs/drive.csv
  python _stat_motion_drive.py --phase suggest --stat-log runs/stat.csv --drive-log runs/drive.csv
  python _stat_motion_drive.py COM4 --phase apply --gyro 12 --bump 10 --lin 800 --duration 25 --save
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Reuse monitor helpers
from _motion_monitor import (
    CSV_HEADER,
    MotLink,
    Sample,
    Stats,
    fmt_row,
    parse_mot,
    pick_port,
    sample_to_csv_row,
)


SETTLE_RE = re.compile(r"settleLeft=([0-9.]+)")


def banner(msg: str) -> None:
    line = "=" * 60
    print(f"\n{line}\n{msg}\n{line}\n", flush=True)


def wait_operator(prompt: str, countdown_s: int = 15) -> None:
    """Wait for Enter; if stdin is not a TTY, use a countdown so the driver can prepare."""
    banner(prompt)
    if sys.stdin is not None and sys.stdin.isatty():
        try:
            input(">>> Press ENTER when ready... ")
            return
        except EOFError:
            print("(EOF on stdin) falling back to countdown", flush=True)
    print(f"(no TTY) starting in {countdown_s}s - get ready now", flush=True)
    for left in range(countdown_s, 0, -1):
        print(f"  ... {left}", flush=True)
        time.sleep(1.0)


def wait_settle_clear(port: str, timeout_s: float = 60.0) -> None:
    """Block until MOT settleLeft ~ 0 (hold afterglow finished)."""
    link = MotLink(port)
    try:
        link.send("TEST PING")
        if not link.wait_ok("PONG", 4):
            raise SystemExit("!! no PONG")
        print("Waiting settleLeft~0 ...", flush=True)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            link.send("TEST MOT")
            t1 = time.monotonic()
            while time.monotonic() - t1 < 0.8:
                for line in link.read_lines():
                    if not line.startswith("[TEST] MOT"):
                        continue
                    if "settleLeft=" not in line:
                        continue
                    m = SETTLE_RE.search(line)
                    if not m:
                        continue
                    left = float(m.group(1))
                    hold = "hold=1" in line
                    print(f"  settleLeft={left:.1f} hold={'1' if hold else '0'}", flush=True)
                    if left <= 0.5 and not hold:
                        print("SETTLE CLEAR", flush=True)
                        return
            time.sleep(1.5)
        print("!! settle timeout - continuing anyway", flush=True)
    finally:
        link.close()


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


def load_csv(path: str) -> List[Sample]:
    samples: List[Sample] = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                samples.append(
                    Sample(
                        t_ms=int(row["t_ms"]),
                        mot=int(row["mot"]),
                        mot_st=int(row.get("mot_st", "0"), 0),
                        gyro=float(row["gyro"]),
                        gdlt=float(row["gdlt"]),
                        thr=float(row["thr"]),
                        busy=int(row["busy"]),
                        bump=float(row["bump"]),
                        bdlt=float(row["bdlt"]),
                        bthr=float(row["bthr"]),
                        bbusy=int(row["bbusy"]),
                        lin=float(row["lin"]),
                        ldlt=float(row["ldlt"]),
                        lthr=float(row["lthr"]),
                        lbusy=int(row["lbusy"]),
                        hold=int(row["hold"]),
                        was=int(row["was"]),
                        mode=row["mode"],
                        age=float(row["age"]),
                        enter=float(row["enter"]),
                        settle_left=float(row["settle_left"]),
                        exit_s=float(row["exit"]),
                        pulse_ms=int(float(row.get("pulse_ms", 0))),
                        need_ms=int(float(row.get("need_ms", 1))),
                        int_status=int(row.get("int", "0"), 0),
                    )
                )
            except (KeyError, ValueError) as e:
                print(f"!! skip row: {e}", file=sys.stderr)
    return samples


@dataclass
class Suggest:
    gyro_menu: int  # 1..25 → thr = *8
    bump_menu: int
    lin_thr: int
    duration_sec: int
    notes: List[str]


def analyze(samples: List[Sample], label: str) -> Dict[str, float]:
    n = len(samples) or 1
    gdlts = [s.gdlt for s in samples if s.gdlt >= 0]
    bdlts = [s.bdlt for s in samples if s.bdlt >= 0]
    ldlts = [s.ldlt for s in samples if s.ldlt >= 0]
    hold_n = sum(1 for s in samples if s.hold)
    mode_enter = 0
    prev = None
    t_enter_ms: Optional[int] = None
    t0 = samples[0].t_ms if samples else 0
    for s in samples:
        if prev is not None and s.mode == "MOVEMENT" and prev != "MOVEMENT":
            mode_enter += 1
            if t_enter_ms is None:
                t_enter_ms = s.t_ms - t0
        prev = s.mode
    out = {
        "n": float(len(samples)),
        "hold_pct": 100.0 * hold_n / n,
        "g_busy_pct": 100.0 * sum(1 for s in samples if s.busy) / n,
        "b_busy_pct": 100.0 * sum(1 for s in samples if s.bbusy) / n,
        "l_busy_pct": 100.0 * sum(1 for s in samples if s.lbusy) / n,
        "mot_pct": 100.0 * sum(1 for s in samples if s.mot) / n,
        "mode_enter": float(mode_enter),
        "gdlt_p95": percentile(gdlts, 0.95),
        "bdlt_p95": percentile(bdlts, 0.95),
        "ldlt_p95": percentile(ldlts, 0.95),
        "gdlt_p99": percentile(gdlts, 0.99),
        "bdlt_p99": percentile(bdlts, 0.99),
        "ldlt_p99": percentile(ldlts, 0.99),
        "enter_s": (t_enter_ms / 1000.0) if t_enter_ms is not None else -1.0,
    }
    print(
        f"[{label}] n={int(out['n'])} hold%={out['hold_pct']:.1f} "
        f"gBusy%={out['g_busy_pct']:.1f} bBusy%={out['b_busy_pct']:.1f} "
        f"lBusy%={out['l_busy_pct']:.1f} mot%={out['mot_pct']:.1f} "
        f"modeEnter={int(out['mode_enter'])} enter_s={out['enter_s']:.1f} "
        f"p95 g/b/lD={out['gdlt_p95']:.0f}/{out['bdlt_p95']:.0f}/{out['ldlt_p95']:.0f}",
        flush=True,
    )
    return out


def suggest_from(stat: Dict[str, float], drive: Optional[Dict[str, float]]) -> Suggest:
    notes: List[str] = []
    # menu gyro/bump: thr_device = menu * 8; want thr > p95_STAT * 1.3
    def menu_from_delta(p95: float, lo: int = 1, hi: int = 25) -> int:
        need = p95 * 1.3
        menu = int(math.ceil(need / 8.0))
        return max(lo, min(hi, menu))

    gyro_m = menu_from_delta(stat["gdlt_p95"])
    bump_m = menu_from_delta(stat["bdlt_p95"])
    lin = int(math.ceil(stat["ldlt_p95"] * 1.3))
    lin = max(100, min(3000, lin))
    # snap lin to common steps if close
    for step in (200, 500, 1000, 1500, 2000):
        if abs(lin - step) < 80:
            lin = step
            break

    duration = 30
    if drive and drive["n"] > 10:
        # If drive entered MOVEMENT, use that time (floor 20). Else estimate from busy%.
        if drive["enter_s"] >= 0:
            duration = max(20, min(300, int(math.ceil(drive["enter_s"] * 0.9))))
            notes.append(f"duration from drive enter_s={drive['enter_s']:.1f}")
        else:
            busy = max(drive["g_busy_pct"], drive["b_busy_pct"], drive["l_busy_pct"]) / 100.0
            if busy > 0.15:
                # need ~duration seconds of busy accum; wall time ≈ duration / busy
                # keep duration moderate when busy is high
                duration = 25 if busy > 0.4 else 35
                notes.append(f"no MOVEMENT enter in drive log; busy~{busy:.0%}, duration={duration}")
            else:
                notes.append("drive log looks quiet - check that vehicle was moving")
                duration = 30
    else:
        notes.append("no drive log - duration left at 30")

    if stat["hold_pct"] > 5:
        notes.append(f"STAT hold%={stat['hold_pct']:.1f} high - raise thresholds further if needed")
    if stat["mode_enter"] > 0:
        notes.append("STAT saw MOVEMENT enter - thresholds too low")

    return Suggest(gyro_menu=gyro_m, bump_menu=bump_m, lin_thr=lin, duration_sec=duration, notes=notes)


def run_capture(port: str, sec: int, period: int, log: Optional[str], phase: str) -> Stats:
    link = MotLink(port)
    stats = Stats()
    csv_file = None
    csv_writer = None
    try:
        link.send("TEST PING")
        if not link.wait_ok("PONG", 4):
            raise SystemExit("!! no PONG")
        link.send("TEST VER")
        link.wait_ok("VER", 2)
        link.send(f"TEST MOT SET period {period}")
        link.wait_ok("MOT_SET", 2)
        link.send("TEST MOT RESET")
        link.wait_ok("MOT_RESET", 2)
        link.send("TEST MOT CFG")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.2:
            for line in link.read_lines():
                if line.startswith("[TEST] MOT"):
                    print(line, flush=True)

        if log:
            Path(log).parent.mkdir(parents=True, exist_ok=True)
            csv_file = open(log, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(CSV_HEADER)

        print(f"=== PHASE {phase.upper()} {sec}s - start ===", flush=True)
        if phase == "stat":
            print("STOY: stay PARKED. Do not move the vehicle.", flush=True)
        else:
            print("EXAY: keep DRIVING. Wait for mode=MOVEMENT.", flush=True)

        link.send(f"TEST MOT STREAM {sec}")
        link.wait_ok("MOT_STREAM", 3)
        deadline = time.monotonic() + sec + 3
        last_print = 0.0
        prev_mode: Optional[str] = None
        while time.monotonic() < deadline:
            for line in link.read_lines():
                if "STREAM end" in line:
                    deadline = time.monotonic()
                s = parse_mot(line)
                if s:
                    if prev_mode is not None and s.mode == "MOVEMENT" and prev_mode != "MOVEMENT":
                        print("*** MOVEMENT ENTER ***", flush=True)
                    prev_mode = s.mode
                    stats.add(s)
                    if csv_writer:
                        csv_writer.writerow(sample_to_csv_row(s))
                        csv_file.flush()
                    now = time.monotonic()
                    if now - last_print >= 0.5 or s.hold or (s.mode == "MOVEMENT"):
                        print(fmt_row(s), flush=True)
                        last_print = now
                elif "ДВИЖЕНИЕ" in line or line.startswith("[MOVEMENT]"):
                    print(f"*** {line}", flush=True)
            time.sleep(0.02)

        print("SUMMARY", stats.summary(), flush=True)
        if log:
            print(f"CSV: {log}", flush=True)
        analyze(stats.samples, phase)
        return stats
    finally:
        try:
            link.send("TEST MOT STREAM 1")
        except Exception:
            pass
        if csv_file:
            csv_file.close()
        link.close()


def print_suggest(sug: Suggest, port: str = "COM4") -> None:
    print("--- SUGGEST ---", flush=True)
    print(f"  TEST MOT SET gyro {sug.gyro_menu}     # thr={sug.gyro_menu * 8}", flush=True)
    print(f"  TEST MOT SET bump {sug.bump_menu}     # thr={sug.bump_menu * 8}", flush=True)
    print(f"  TEST MOT SET lin {sug.lin_thr}", flush=True)
    print(f"  TEST MOT SET duration {sug.duration_sec}", flush=True)
    for n in sug.notes:
        print(f"  note: {n}", flush=True)
    print(
        f"Apply:\n  python _stat_motion_drive.py {port} --phase apply "
        f"--gyro {sug.gyro_menu} --bump {sug.bump_menu} --lin {sug.lin_thr} "
        f"--duration {sug.duration_sec} --save",
        flush=True,
    )


def run_session(
    port: str,
    sec_stat: int,
    sec_drive: int,
    period: int,
    countdown_s: int,
) -> int:
    os.makedirs("runs", exist_ok=True)
    stat_log = "runs/session_stat.csv"
    drive_log = "runs/session_drive.csv"

    banner("SESSION: guided STAT then DRIVE\nI will tell you when to STOY and when to EXAY.")
    wait_settle_clear(port)

    wait_operator(
        f">>> STOY (STAT).\n"
        f"Stay PARKED. Engine as in real use.\n"
        f"Do NOT move for {sec_stat}s after start.\n"
        f"Enter = start STAT capture ({sec_stat}s)",
        countdown_s=countdown_s,
    )
    run_capture(port, sec_stat, period, stat_log, "stat")
    st = analyze(load_csv(stat_log), "stat")

    wait_operator(
        f">>> EXAY (DRIVE).\n"
        f"Start driving NOW, keep moving ~{sec_drive}s.\n"
        f"Need mode=MOVEMENT (pulse accum >= duration).\n"
        f"Enter = start DRIVE capture ({sec_drive}s)",
        countdown_s=countdown_s,
    )
    run_capture(port, sec_drive, period, drive_log, "drive")
    dr = analyze(load_csv(drive_log), "drive")

    sug = suggest_from(st, dr)
    print_suggest(sug, port=port)

    stat_ok = st["hold_pct"] <= 5.0 and st["mode_enter"] <= 0
    drive_ok = dr["mode_enter"] > 0 and dr["enter_s"] >= 0
    if stat_ok and drive_ok:
        wait_operator(
            "STAT clean + DRIVE entered MOVEMENT.\n"
            "Enter = APPLY suggest + MOT SAVE\n"
            "(or Ctrl+C to skip)",
            countdown_s=countdown_s,
        )
        apply_settings(
            port,
            sug.gyro_menu,
            sug.bump_menu,
            sug.lin_thr,
            sug.duration_sec,
            None,
            save=True,
        )
        print("Applied + SAVE done. Re-check STAT/DRIVE on vehicle if needed.", flush=True)
    else:
        banner(
            "NO APPLY: vehicle data not valid for auto-tune.\n"
            f"  STAT ok={stat_ok} (need hold%~0, modeEnter=0)\n"
            f"  DRIVE ok={drive_ok} (need MOVEMENT enter)\n"
            "Defaults unchanged. Re-run --phase session on the car."
        )
    return 0


def apply_settings(
    port: str,
    gyro: Optional[int],
    bump: Optional[int],
    lin: Optional[int],
    duration: Optional[int],
    settle: Optional[int],
    save: bool,
) -> None:
    link = MotLink(port)
    try:
        link.send("TEST PING")
        if not link.wait_ok("PONG", 4):
            raise SystemExit("!! no PONG")
        for key, val in (
            ("gyro", gyro),
            ("bump", bump),
            ("lin", lin),
            ("duration", duration),
            ("settle", settle),
        ):
            if val is not None:
                link.send(f"TEST MOT SET {key} {val}")
                ok = link.wait_ok("MOT_SET", 3)
                print(f"SET {key}={val} -> {'OK' if ok else 'FAIL'}", flush=True)
        if save:
            link.send("TEST MOT SAVE")
            ok = link.wait_ok("MOT_SAVE", 5)
            print(f"SAVE -> {'OK' if ok else 'FAIL'}", flush=True)
        link.send("TEST MOT CFG")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            for line in link.read_lines():
                if line.startswith("[TEST] MOT"):
                    print(line, flush=True)
            time.sleep(0.05)
    finally:
        link.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="STAT/DRIVE motion tuning for Kamaz-Leveler")
    ap.add_argument("port", nargs="?", default=None, help="COM port")
    ap.add_argument(
        "--phase",
        choices=("stat", "drive", "session", "suggest", "apply"),
        required=True,
        help="capture phase or suggest/apply",
    )
    ap.add_argument("--sec", type=int, default=90, help="capture seconds (stat/drive)")
    ap.add_argument("--sec-stat", type=int, default=90, help="STAT seconds for session")
    ap.add_argument("--sec-drive", type=int, default=120, help="DRIVE seconds for session")
    ap.add_argument(
        "--countdown",
        type=int,
        default=15,
        help="seconds to wait when stdin has no TTY (session prompts)",
    )
    ap.add_argument("--period", type=int, default=100, help="MOT stream period ms")
    ap.add_argument("--log", type=str, default=None, help="CSV path for capture")
    ap.add_argument("--stat-log", type=str, default=None, help="STAT CSV for suggest")
    ap.add_argument("--drive-log", type=str, default=None, help="DRIVE CSV for suggest")
    ap.add_argument("--gyro", type=int, default=None)
    ap.add_argument("--bump", type=int, default=None)
    ap.add_argument("--lin", type=int, default=None)
    ap.add_argument("--duration", type=int, default=None)
    ap.add_argument("--settle", type=int, default=None)
    ap.add_argument("--save", action="store_true", help="MOT SAVE after apply")
    args = ap.parse_args()

    if args.phase in ("stat", "drive", "session", "apply"):
        port = pick_port(args.port)
        print(f"Port {port}", flush=True)
    else:
        port = args.port or "COM4"

    if args.phase == "session":
        return run_session(port, args.sec_stat, args.sec_drive, args.period, args.countdown)

    if args.phase in ("stat", "drive"):
        log = args.log
        if not log:
            os.makedirs("runs", exist_ok=True)
            log = f"runs/{args.phase}_{int(time.time())}.csv"
        run_capture(port, args.sec, args.period, log, args.phase)
        return 0

    if args.phase == "suggest":
        if not args.stat_log:
            print("!! --stat-log required", file=sys.stderr)
            return 2
        st = analyze(load_csv(args.stat_log), "stat")
        dr = analyze(load_csv(args.drive_log), "drive") if args.drive_log else None
        sug = suggest_from(st, dr)
        print_suggest(sug, port=port)
        return 0

    # apply
    if all(v is None for v in (args.gyro, args.bump, args.lin, args.duration, args.settle)):
        print("!! apply needs at least one of --gyro/--bump/--lin/--duration/--settle", file=sys.stderr)
        return 2
    apply_settings(port, args.gyro, args.bump, args.lin, args.duration, args.settle, args.save)
    return 0


if __name__ == "__main__":
    sys.exit(main())
