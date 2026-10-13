#!/usr/bin/env python3
"""Button inject self-test for Kamaz-Leveler (TEST BTN …).

Usage:
  python tools/btn_test.py COM4
  python tools/btn_test.py          # auto-detect port

Sends TEST BTN SELF (dry-run pad logic + sticky KN1 + menu pair check).
Exit 0 on BTN_SELF_PASS.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from _serial_selftest import SerialTest, pick_port  # noqa: E402


def run_btn(port: str) -> int:
    t = SerialTest(port)
    try:
        time.sleep(0.5)
        t.send("TEST PING")
        t.expect_ok("PONG", timeout=3)
        t.send("TEST VER")
        t.expect_ok("VER", timeout=3)
        t.send("TEST BTN CLEAR")
        t.expect_ok("BTN_CLEAR", timeout=3)
        t.send("TEST BTN SELF")
        ok = t.wait_for(
            lambda L: "BTN_SELF_PASS" in L
            or "BTN_SELF_FAIL" in L
            or "BTN_SELF DONE PASS" in L
            or "BTN_SELF DONE FAIL" in L,
            timeout=8,
            label="BTN_SELF",
        )
        if ok and "PASS" in ok:
            print("=== BTN SELF PASS ===")
            return 0
        print(f"=== BTN SELF FAIL ({ok}) ===")
        return 1
    finally:
        t.send("TEST BTN CLEAR")
        t.send("TEST BTN DRY OFF")
        t.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=None)
    args = ap.parse_args()
    port = pick_port(args.port)
    print(f"=== btn_test on {port} ===")
    return run_btn(port)


if __name__ == "__main__":
    raise SystemExit(main())
