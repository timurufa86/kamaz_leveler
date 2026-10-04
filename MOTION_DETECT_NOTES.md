# Motion detection findings (road tests 2026-10-01 … 2026-10-02)

## Firmware 10.4.0 (current)
- DMP: **MotionApps v6.12**, FIFO **28B** (quat+accel+gyro)
- Busy = MOT `0x40`/`motSt` **OR** `|gyro−EMA| ≥ menu×8` **OR** `|bump−EMA| ≥ menu×8` **OR** `|lin−EMA| ≥ linThr`
- EMA alpha **0.05** on gyro / bump / lin — constant engine idle baselines out
- STREAM: `gdlt=` / `bdlt=` / `ldlt=` (deltas compared to thr)
- Entry: sum of **busy pulse time** ≥ `durationSec` (settle afterglow not counted)
- Gyro/bump/lin spikes > **2000** ignored (DMP glitch clamp)
- Serial: `TEST MOT STREAM`, `TEST MOT SET`, `TEST MOT SAVE`
- Tools: `tools/mot_calib_test.py`, `tools/desk_diag.py`

## Firmware 10.2.0–10.3.3 (historical)
- Spike = MOT `0x40` **OR** absolute gyro RMS ≥ menu×8 **OR** linAcc Δ ≥ thr
- Absolute gyro/bump falsely entered MOVEMENT on idle with engine running (continuous rumble)
- 10.3.3: pulse-accum duration (desk settle fix); still false on diesel idle if rumble > thr

## Setup (historical 10.1.26)
- Firmware: 10.1.26
- Gyro menu threshold default was 50 → thr = 50×8 = **400** (too high for this cabin)
- Working test thr: menu **10** → thr = **80**
- Hardware MOT bit (0x40): essentially **never** fired in tests (mot% ≈ 0)
- Detection that worked: DMP FIFO gyro RMS only

## Idle (standing)
| Session | gyro avg | gyro max | holdEdges | modeEnter | notes |
|---------|----------|----------|-----------|-----------|-------|
| motion_idle.csv | ~2 | 48–85 | 1 | 0 | rare spike 85 re-armed hold; engine/cabin |
| motion_idle2.csv | ~0 | 0 / often -1 | 0 | 0 | clean; FIFO empty (gyro=-1) common |

Idle signature:
- gyro typically **0…5** (or **-1** = no FIFO packet this tick)
- occasional spikes **15…85** on idle with engine → false hold if thr≤80 (pre-EMA)
- With 10.4.0 EMA, steady idle rumble should show low `gdlt`/`bdlt` after ~1–2 s
- int often 0x01 or 0x03, no MOT pulse

## Driving
| Session | gyro peaks | holdEdges | modeEnter | notes |
|---------|------------|-----------|-----------|-------|
| motion_lowthr.csv | max ~112 | 1 | 1 | entered MOVEMENT; busy% only ~0.4 |
| motion_drive2.csv | peak ~447, glitch 11k–44k | 1 | 1 | after enter often gyro=-1 but settle kept hold; then exit |

Drive signature:
- Real motion spikes: **~100…450** (short), not continuous high RMS
- busy% very low (<<1%) — rely on **settle hold timer**, not sustained busy
- After MOVEMENT entry, FIFO often empty (gyro=-1) while hold continues via settle
- Wild outliers (thousands) = DMP glitches — must clamp/ignore

## Problems with pre-10.2.0 logic
1. Default thr 400 never sees road (drive looks like idle at 7–26)
2. thr 80 works for entry but idle spikes ~85 can false-trigger
3. Detection is impulse-based; long quiet gaps between spikes while still moving
4. gyro=-1 (no packet) treated as “not busy” but should not clear activity by itself
5. MOT hardware unused in practice with DMP
6. No clamp on absurd RMS (glitches look like mega-motion)
7. No accel/linear channel for smooth straight-line motion

## Tuning (after 10.4.0)
- Calibrate on **idle engine** STREAM: look at `gdlt`/`bdlt`/`ldlt` p95, not raw gyro
- Defaults: gyro/bump menu **25** (thr 200), lin **500**, settle **10**, duration **30**
- Persist with `TEST MOT SAVE` after `TEST MOT SET`

## Logs
- motion.csv / motion_lowthr.csv / motion_idle.csv / motion_drive2.csv / motion_idle2.csv
- motion_drive3.csv — **no detection**: 240s all MANUAL, gyro mostly -1/empty FIFO, holdEdges=0 modeEnter=0 (thr=80). Either very smooth segment / no road vib reaching DMP, or FIFO starved — needs accel channel or raw gyro path.
