#!/usr/bin/env python3
"""Two-phase MOT calibration: idle then shake (full or --phase step-by-step)."""
from __future__ import annotations

import argparse
import json
import re
import statistics as st
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

SESSION_PATH = Path(__file__).with_name("mot_calib_session.json")

MOT_RE = re.compile(
    r"mot=(\d+) motSt=0x([0-9A-Fa-f]+) gyro=([\-\d.]+) gdlt=([\-\d.]+) thr=\d+ busy=(\d+) "
    r"bump=([\-\d.]+) bdlt=([\-\d.]+) bthr=\d+ bbusy=(\d+) lin=([\-\d.]+) ldlt=([\-\d.]+) "
    r"lthr=\d+ lbusy=(\d+) hold=(\d+)"
)


def parse_mot(line: str) -> dict | None:
    m = MOT_RE.search(line)
    if not m:
        return None
    return {
        "mot": int(m.group(1)),
        "motSt": int(m.group(2), 16),
        "gyro": float(m.group(3)),
        "gdlt": float(m.group(4)),
        "busy": int(m.group(5)),
        "bump": float(m.group(6)),
        "bdlt": float(m.group(7)),
        "bbusy": int(m.group(8)),
        "ldlt": float(m.group(10)),
        "lbusy": int(m.group(11)),
        "hold": int(m.group(12)),
    }


def open_port(port: str) -> serial.Serial:
    ser = serial.Serial(port, 115200, timeout=0.25)
    time.sleep(0.8)
    return ser


def collect(ser: serial.Serial, sec: int, label: str) -> list[dict]:
    ser.reset_input_buffer()
    ser.write(f"TEST MOT STREAM {sec}\n".encode())
    deadline = time.time() + sec + 4
    rows: list[dict] = []
    buf = ""
    while time.time() < deadline:
        chunk = ser.read(1024).decode("utf-8", errors="ignore")
        if not chunk:
            continue
        buf += chunk
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            if "[MOT]" in line and "t=" in line:
                r = parse_mot(line)
                if r:
                    rows.append(r)
            if "STREAM end" in line:
                deadline = time.time()
    print(f"--- {label}: samples={len(rows)} ---")
    return rows


def stats(rows: list[dict], key: str) -> dict | None:
    xs = [r[key] for r in rows if r[key] >= 0]
    if not xs:
        return None
    xs_sorted = sorted(xs)
    p95 = xs_sorted[int(0.95 * (len(xs_sorted) - 1))] if len(xs_sorted) > 1 else xs_sorted[0]
    return {"min": min(xs), "avg": st.mean(xs), "max": max(xs), "p95": p95}


def wait_calm(ser: serial.Serial, max_sec: int = 40) -> None:
    print(f"Жду покоя (hold=0), до {max_sec} с — датчик НЕ трясти...")
    deadline = time.time() + max_sec
    calm_streak = 0
    while time.time() < deadline:
        ser.reset_input_buffer()
        ser.write(b"TEST MOT STREAM 2\n")
        end = time.time() + 3.0
        buf = ""
        last_hold = None
        while time.time() < end:
            chunk = ser.read(1024).decode("utf-8", errors="ignore")
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                r = parse_mot(line)
                if r:
                    last_hold = r["hold"]
                    if r["hold"] == 0:
                        calm_streak += 1
                    else:
                        calm_streak = 0
                if "STREAM end" in line:
                    end = time.time()
        if last_hold is not None:
            print(f"  hold={last_hold} (нужно 3 подряд hold=0)")
        if calm_streak >= 3:
            print("Покой достигнут.\n")
            return
    print("Таймаут ожидания покоя — baseline может быть грязным.\n")


def load_session() -> dict:
    if SESSION_PATH.is_file():
        return json.loads(SESSION_PATH.read_text(encoding="utf-8"))
    return {"port": "COM4", "idle": [], "shake": []}


def save_session(data: dict) -> None:
    SESSION_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def print_block(name: str, rows: list[dict]) -> None:
    print(f"\n{name} n={len(rows)}")
    for k in ["gyro", "gdlt", "bump", "bdlt", "ldlt"]:
        s = stats(rows, k)
        if s:
            print(
                f"  {k}: min={s['min']:.0f} avg={s['avg']:.1f} "
                f"max={s['max']:.0f} p95={s['p95']:.0f}"
            )
    if rows:
        n = len(rows)
        print(
            f"  mot%={100*sum(r['mot'] for r in rows)/n:.1f} "
            f"busy%={100*sum(r['busy'] for r in rows)/n:.1f} "
            f"bbusy%={100*sum(r['bbusy'] for r in rows)/n:.1f} "
            f"lbusy%={100*sum(r['lbusy'] for r in rows)/n:.1f} "
            f"hold%={100*sum(r['hold'] for r in rows)/n:.1f}"
        )


def print_suggestions(idle: list[dict], shake: list[dict]) -> None:
    # thr сравнивается с Δ (gdlt/bdlt/ldlt), не с сырым уровнем
    si = stats(idle, "gdlt")
    sb_i = stats(idle, "bdlt")
    sl_i = stats(idle, "ldlt")
    ss_g = stats(shake, "gdlt")
    ss_b = stats(shake, "bdlt")
    ss_l = stats(shake, "ldlt")

    if not si or not ss_g:
        print("Недостаточно данных для калибровки.")
        return
    gyro_menu = max(10, min(200, int((si["p95"] * 1.5) / 8) + 2))
    bump_menu = max(10, min(200, int((sb_i["p95"] * 1.5) / 8) + 2)) if sb_i else 40
    lin_thr = max(100, min(3000, int(sl_i["p95"] * 2.5))) if sl_i else 400
    print("\n=== SUGGESTED CALIB (idle Δ p95 + margin) ===")
    print(f"  TEST MOT SET gyro {gyro_menu}   # thr~{gyro_menu * 8:.0f}")
    print(f"  TEST MOT SET bump {bump_menu}   # thr~{bump_menu * 8:.0f}")
    print(f"  TEST MOT SET lin {lin_thr}")
    if sb_i and ss_b and sl_i and ss_l:
        print(
            f"  separation shake/idle Δ p95: "
            f"gdlt x{ss_g['p95']/max(si['p95'],1):.1f} "
            f"bdlt x{ss_b['p95']/max(sb_i['p95'],1):.1f} "
            f"ldlt x{ss_l['p95']/max(sl_i['p95'],1):.1f}"
        )


def phase_reset(port: str) -> int:
    print(f"[ЭТАП 1/4] Сброс MOT + ожидание покоя на {port}\n")
    ser = open_port(port)
    ser.reset_input_buffer()
    ser.write(b"TEST MOT RESET\n")
    time.sleep(0.2)
    ser.write(b"TEST VER\n")
    time.sleep(0.3)
    print(ser.read(800).decode("utf-8", errors="ignore"))
    wait_calm(ser)
    ser.close()
    data = load_session()
    data["port"] = port
    data["idle"] = []
    data["shake"] = []
    save_session(data)
    print("Этап 1 готов. Напишите «готов» — запущу 5 с отсчёт и фазу ПОКОЙ.")
    return 0


def phase_idle(port: str, sec: int, countdown: int) -> int:
    print(f"[ЭТАП 2/4] ПОКОЙ {sec} с на {port}\n")
    for i in range(countdown, 0, -1):
        print(f"  {i}... не двигать датчик")
        time.sleep(1)
    print("\n>>> СЕЙЧАС: ДЕРЖИТЕ В ПОКОЕ <<<\n")
    ser = open_port(port)
    idle = collect(ser, sec, "IDLE")
    ser.close()
    data = load_session()
    data["idle"] = idle
    save_session(data)
    print_block("IDLE", idle)
    print("\nЭтап 2 готов. Напишите «готов» — запущу фазу ТРЯСКА.")
    return 0


def phase_shake(port: str, sec: int) -> int:
    print(f"[ЭТАП 3/4] ТРЯСКА {sec} с на {port}\n")
    print(">>> СЕЙЧАС: ТРЯСИТЕ ДАТЧИК / КОРПУС <<<\n")
    ser = open_port(port)
    shake = collect(ser, sec, "SHAKE")
    ser.write(b"TEST MOT CFG\n")
    time.sleep(0.4)
    print(ser.read(1500).decode("utf-8", errors="ignore"))
    ser.close()
    data = load_session()
    data["shake"] = shake
    save_session(data)
    print_block("SHAKE", shake)
    print("\nЭтап 3 готов. Напишите «готов» — выведу итог и пороги.")
    return 0


def phase_report() -> int:
    print("[ЭТАП 4/4] Итог\n")
    data = load_session()
    idle = data.get("idle") or []
    shake = data.get("shake") or []
    print_block("IDLE", idle)
    print_block("SHAKE", shake)
    print_suggestions(idle, shake)
    return 0


def run_full(port: str, idle_sec: int, shake_sec: int) -> int:
    phase_reset(port)
    phase_idle(port, idle_sec, 5)
    phase_shake(port, shake_sec)
    return phase_report()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    ap = argparse.ArgumentParser(description="MOT idle/shake calibration")
    ap.add_argument("port", nargs="?", default="COM4")
    ap.add_argument("idle_sec", nargs="?", type=int, default=15)
    ap.add_argument("shake_sec", nargs="?", type=int, default=15)
    ap.add_argument(
        "--phase",
        choices=("reset", "idle", "shake", "report", "full"),
        default="full",
        help="reset|idle|shake|report — по шагам; full — всё сразу",
    )
    ap.add_argument("--countdown", type=int, default=5, help="секунд перед фазой idle")
    args = ap.parse_args()

    if args.phase == "full":
        return run_full(args.port, args.idle_sec, args.shake_sec)
    if args.phase == "reset":
        return phase_reset(args.port)
    if args.phase == "idle":
        return phase_idle(args.port, args.idle_sec, args.countdown)
    if args.phase == "shake":
        return phase_shake(args.port, args.shake_sec)
    return phase_report()


if __name__ == "__main__":
    raise SystemExit(main())
