#!/usr/bin/env python3
# ============================================================
# File    : tools/check_release.py
# Project : PONS v7 / v7.1
# Role    : リリース前・コミット前の一括チェックスクリプト。
#           1) settings.h の RELEASE / RELEASE_GNSS / デバッグフラグ / BUILDVERSION 検査
#           2) FLASH 焼き込みアセット（音声 wav_data・案内画像 guide_data）の CRC 整合性検査
#           3) ホスト単体テスト一括実行（attitude_test / wire2_test / vario_test / dbd_test / test_eskf.py）
#              ※ Arduino IDE がビルド時に tools/ ごとコピーして重くならないよう、実行後に make clean する
#
# 使い方:
#   python3 tools/check_release.py
# ============================================================
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS_H = os.path.join(ROOT, "settings.h")


def check_settings_h():
    print("=== [1/3] settings.h リリース設定チェック ===")
    with open(SETTINGS_H, "r", encoding="utf-8") as f:
        lines = f.readlines()

    defines = {}
    warnings = []
    for lineno, raw in enumerate(lines, 1):
        line = raw.split("//")[0].strip()
        m_def = re.match(r"^#\s*define\s+([A-Za-z0-9_]+)(?:\s+(.*))?$", line)
        if m_def:
            defines[m_def.group(1)] = (m_def.group(2) or "").strip()
        m_warn = re.match(r"^#\s*warning\s+(.*)$", line)
        if m_warn and "NOT RELEASE" not in m_warn.group(1):
            warnings.append((lineno, m_warn.group(1)))

    ok = True
    version = defines.get("BUILDVERSION", "?")
    builddate = defines.get("BUILDDATE", "?")
    print(f"  BUILDVERSION = {version}, BUILDDATE = {builddate}")

    if "RELEASE" not in defines:
        print("  [NG] #define RELEASE が無効です（USB シリアル待ちで実機が起動しません）")
        ok = False
    else:
        print("  [OK] #define RELEASE 有効")

    if "RELEASE_GNSS" not in defines:
        print("  [NG] #define RELEASE_GNSS が無効です")
        ok = False
    else:
        print("  [OK] #define RELEASE_GNSS 有効")

    forbidden = [
        k for k in defines
        if k.startswith("DEBUG_GNSS_SIM_") or k in ("DEBUG_ESKF", "DEBUG_GBX_NMEA")
    ]
    if forbidden:
        print(f"  [NG] デバッグマクロが有効のままです: {', '.join(forbidden)}")
        ok = False

    if defines.get("GNSS_RF_LOG_RAW", "0") != "0":
        print(f"  [NG] GNSS_RF_LOG_RAW が {defines.get('GNSS_RF_LOG_RAW')} です（リリース時は 0）")
        ok = False

    for lineno, w in warnings:
        print(f"  [WARN] settings.h:{lineno}: #warning {w}")

    return ok


def run_step(title, cmd, cwd=ROOT):
    print(f"  -> {title}")
    res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if res.returncode != 0:
        print(res.stdout)
        print(f"  [NG] {title} 失敗 (exit={res.returncode})")
        return False
    print(f"  [OK] {title}")
    return True


def run_make_test(name, rel_dir):
    test_dir = os.path.join(ROOT, rel_dir)
    ok = run_step(name, ["make"], cwd=test_dir)
    subprocess.run(["make", "clean"], cwd=test_dir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return ok


def main():
    all_ok = True

    if not check_settings_h():
        all_ok = False

    print("\n=== [2/3] FLASH アセット整合性チェック ===")
    if not run_step("gen_wav_flash.py --check", [sys.executable, "tools/gen_wav_flash.py", "--check"]):
        all_ok = False
    if not run_step("gen_guide_flash.py --check", [sys.executable, "tools/gen_guide_flash.py", "--check"]):
        all_ok = False

    print("\n=== [3/3] ホスト単体テスト一括実行 ===")
    for name, rel_dir in [
        ("attitude_test (C++ ESKF)", "tools/attitude_test"),
        ("wire2_test (C++ PONS Link Wire2 / Beacon)", "tools/wire2_test"),
        ("vario_test (C++ バリオ KF / 加速度ガード / MS5611 reinit)", "tools/vario_test"),
        ("dbd_test (C++ GNSS DBD 保存・復元・時刻注入)", "tools/dbd_test"),
    ]:
        if not run_make_test(name, rel_dir):
            all_ok = False

    if not run_step("test_eskf.py (Python ESKF 二重実装テスト)", [sys.executable, "tools/imulog/test_eskf.py"]):
        all_ok = False

    print("\n============================================================")
    if all_ok:
        print("結果: PASS — リリース前チェック全項目合格")
        return 0
    else:
        print("結果: FAIL — 失敗した項目を確認してください")
        return 1


if __name__ == "__main__":
    sys.exit(main())
