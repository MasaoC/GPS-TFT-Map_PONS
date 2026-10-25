// ============================================================
// File    : readiness.cpp
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 本番前チェック（v7 の readiness の v3 版）。設定画面の右上に READY / NOT READY を出す。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
// ★ v7 と同じく、既存の判定（gps.cpp・power.cpp・mysd.cpp）を読むだけで、条件を書き写さない。
//   v3 には音が無いので「準備完了」の読み上げは無く、設定画面に出すだけ。
#include "readiness.h"
#include "settings.h"
#include "gps.h"
#include "power.h"
#include "mysd.h"

static void add_item(char* missing, size_t n, const char* item) {
  const size_t len = strlen(missing);
  snprintf(missing + len, n - len, "%s%s", len ? " " : "", item);
}

bool readiness_check(char* missing, size_t n) {
  missing[0] = '\0';
  // v7 と同じ: 3D 測位・gnssFixOK・水平精度（NAV-PVT の hAcc）READY_HACC_MAX_M 以内
  const float hacc = get_gps_hacc_m();
  const bool gnss_ok = get_gps_fix3d() && hacc >= 0.0f && hacc <= READY_HACC_MAX_M;
  if (!gnss_ok) add_item(missing, n, "GNSS");
  // デモ・リプレイは「仮想の位置」。どちらかのままだと実際の位置で誘導しない（同時には使わない）
  if (replay_active()) add_item(missing, n, "REPLAY");
  else if (get_demo_biwako()) add_item(missing, n, "DEMO");
  if (usb_power_present()) add_item(missing, n, "USB");
  if (power_fault_active() || power_battery_percent() < READY_BATT_MIN_PERCENT) add_item(missing, n, "BATT");
  if (!good_sd()) add_item(missing, n, "SD");
  return missing[0] == '\0';
}
