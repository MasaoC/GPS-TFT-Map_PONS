#!/usr/bin/env python3
# ============================================================
# File    : tools/repons/test/run_serial_tests.py
# Project : PONS v7.1 / v7.2 Re:PONS
# Role    : Re:PONS の本体とのやりとり（serial.js）の試験。いまは塊ごとの取り込み（PONS:GETRANGE。2026-11-25〜）だけ。
#           serial_stubs.js → ../serial.js → serial_range_test.js の順に jsc で読む（模擬の本体と仮の時計。本体もブラウザも要らない）。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026-11-25
# ============================================================
# 使い方: python3 tools/repons/test/run_serial_tests.py
# ★ jsc（macOS の JavaScriptCore）が無い PC では飛ばす（飛ばしたと書く）。
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPONS = HERE.parent
JSC = '/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc'


def main():
    if not os.path.exists(JSC):
        print('skip: jsc が無い（macOS の JavaScriptCore）。serial.js の試験は飛ばした')
        return 0
    files = [HERE / 'serial_stubs.js', REPONS / 'serial.js', HERE / 'serial_range_test.js']
    r = subprocess.run([JSC] + [str(f) for f in files], capture_output=True, text=True, timeout=600)
    out = (r.stdout or '') + (r.stderr or '')
    print(out.rstrip())
    if r.returncode != 0 or 'ALL PASS' not in out:
        print('NG: serial.js の試験が通らない')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
