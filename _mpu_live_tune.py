#!/usr/bin/env python3
"""One-shot live MPU capture + tune while user rotates the board."""
from __future__ import annotations

import math
import statistics
import sys
import time

from _mpu_tune import STREAM_RE, MpuLink


def corr(a, b):
    n = len(a)
    ma = sum(a) / n
    mb = sum(b) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x - ma) ** 2 for x in a)) or 1e-9
    db = math.sqrt(sum((y - mb) ** 2 for y in b)) or 1e-9
    return num / (da * db)


def main() -> int:
    link = MpuLink("COM4")
    ok = False
    for _ in range(8):
        link.lines.clear()
        if link.cmd_ok("TEST PING", "PONG", timeout=2):
            ok = True
            break
        time.sleep(0.4)
    if not ok:
        print("!! no PONG")
        return 1

    print("CFG", link.read_cfg(), flush=True)
    for i in (3, 2, 1):
        print(f"{i}...", flush=True)
        time.sleep(1)
    print("===== START — KRUTITE 20s =====", flush=True)

    sec = 20
    link.lines.clear()
    link.send(f"TEST IMU STREAM {sec}")
    link.wait(lambda L: "IMU_STREAM" in L, timeout=3)
    deadline = time.monotonic() + sec + 4
    samples = []
    t0 = time.monotonic()
    last = 0.0
    while time.monotonic() < deadline:
        link._pump()
        while link.lines:
            line = link.lines.pop(0)
            m = STREAM_RE.search(line)
            if m:
                samples.append(
                    (
                        int(m.group(1)),
                        float(m.group(2)),
                        float(m.group(3)),
                        float(m.group(4)),
                        float(m.group(5)),
                    )
                )
            elif "STREAM end" in line:
                deadline = 0
        el = time.monotonic() - t0
        if el - last >= 4 and samples:
            rx = [s[1] for s in samples]
            ry = [s[2] for s in samples]
            pp = max(max(rx) - min(rx), max(ry) - min(ry))
            print(f"  {el:.0f}s n={len(samples)} raw_pp={pp:.2f}", flush=True)
            last = el
        time.sleep(0.003)

    print(f"DONE n={len(samples)}", flush=True)
    if len(samples) < 40:
        print("FAIL few samples")
        link.close()
        return 1

    rx = [s[1] for s in samples]
    ry = [s[2] for s in samples]
    ox = [s[3] for s in samples]
    oy = [s[4] for s in samples]
    raw_pp = max(max(rx) - min(rx), max(ry) - min(ry))
    out_pp = max(max(ox) - min(ox), max(oy) - min(oy))
    print(f"raw_pp={raw_pp:.2f} out_pp={out_pp:.2f}")
    print(f"X=[{min(rx):.1f}..{max(rx):.1f}] Y=[{min(ry):.1f}..{max(ry):.1f}]")
    print(f"outX=[{min(ox):.1f}..{max(ox):.1f}] outY=[{min(oy):.1f}..{max(oy):.1f}]")
    if raw_pp < 3.0:
        print("!! NET DVIZHENIYA — ne menyayu")
        link.close()
        return 0

    raw_steps = []
    out_steps = []
    lag = []
    dts = []
    for i in range(1, len(samples)):
        t0s, rx0, ry0, ox0, oy0 = samples[i - 1]
        t1, rx1, ry1, ox1, oy1 = samples[i]
        dts.append(max(t1 - t0s, 1))
        rs = max(abs(rx1 - rx0), abs(ry1 - ry0))
        os_ = max(abs(ox1 - ox0), abs(oy1 - oy0))
        raw_steps.append(rs)
        out_steps.append(os_)
        lag.append(max(abs(ox1 - rx1), abs(oy1 - ry1)))

    hz = 1000.0 / statistics.mean(dts)
    phantom = sum(1 for rs, os_ in zip(raw_steps, out_steps) if os_ > 0.5 and rs < 0.1)
    hard = sum(1 for s in out_steps if s > 1.2)
    still = [out_steps[i] for i, rs in enumerate(raw_steps) if rs <= 0.03] or [0]
    motion = [i for i, rs in enumerate(raw_steps) if rs > 0.1]
    mlag = [lag[i] for i in motion] or lag

    rxd = [samples[i][1] - samples[i - 1][1] for i in range(1, len(samples))]
    oxd = [samples[i][3] - samples[i - 1][3] for i in range(1, len(samples))]
    ryd = [samples[i][2] - samples[i - 1][2] for i in range(1, len(samples))]
    oyd = [samples[i][4] - samples[i - 1][4] for i in range(1, len(samples))]
    cx, cy = corr(rxd, oxd), corr(ryd, oyd)
    lag_m = statistics.mean(mlag)
    still_m = statistics.mean(still)
    cm = (cx + cy) / 2
    track = out_pp / raw_pp if raw_pp > 0.01 else 0

    print(f"hz={hz:.1f} phantom={phantom} hard={hard} still={still_m:.4f}")
    print(f"motion_lag={lag_m:.3f} p95={sorted(mlag)[int(0.95 * len(mlag))]:.3f}")
    print(f"corr={cx:.3f}/{cy:.3f} track_ratio={track:.2f}")
    print(f"max_raw_step={max(raw_steps):.3f} max_out_step={max(out_steps):.3f}")

    need_faster = lag_m > 0.5 or cm < 0.78 or track < 0.75
    need_quieter = phantom > 10 or still_m > 0.04 or hard > 15
    # "still" during continuous tilting is noisy — ignore if strong motion present
    if raw_pp > 15:
        need_quieter = phantom > 10 or hard > 20
    print(f"need_faster={need_faster} need_quieter={need_quieter}")

    cfg = link.read_cfg()
    mea, est, q = cfg["mea"], cfg["est"], cfg["q"]
    poll, fifo = int(cfg["poll"]), int(cfg["fifo"])
    ema, spike, thr, slew = cfg["ema"], cfg["spike"], cfg["thr"], cfg["slew"]

    if need_faster:
        # Never lower slew when lagging — that was the main bug
        mea = max(3.0, mea - 1.0)
        ema = min(0.55, ema + 0.08)
        slew = max(slew, 60.0)
        slew = min(120.0, max(slew, 90.0))
        poll = 15
        fifo = max(1, min(fifo, 2))
        q = min(0.020, q + 0.004)
        thr = max(thr, 2.0)
        spike = max(spike, 0.12)
        print("action: faster (keep high slew)")
    elif need_quieter:
        mea = min(12, mea + 1)
        ema = max(0.18, ema - 0.03)
        # only trim slew if lag already low
        if lag_m < 0.4:
            slew = max(12, slew - 2)
        fifo = min(4, fifo + 1)
        print("action: quieter")
    else:
        print("action: keep")

    for k, v in [
        ("mea", f"{mea:.1f}"),
        ("est", f"{est:.1f}"),
        ("q", f"{q:.3f}"),
        ("poll", str(poll)),
        ("fifo", str(fifo)),
        ("ema", f"{ema:.2f}"),
        ("spike", f"{spike:.2f}"),
        ("thr", f"{thr:.1f}"),
        ("slew", f"{slew:.1f}"),
    ]:
        link.cmd_ok(f"TEST IMU SET {k} {v}", "IMU_SET", 3)
    link.cmd_ok("TEST IMU SAVE", "IMU_SAVE", 8)
    print("SAVED", link.read_cfg())
    print('Dlya 2-go passa snova napishite "poehali".')
    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
