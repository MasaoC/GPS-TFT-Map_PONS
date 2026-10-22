// ============================================================
// File    : readiness.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 本番前チェック。飛ぶ前に確かめる項目がそろったら、地上で止まっているときに
//           1 度だけ「準備完了」と読み上げる。そろっていなければ何が足りないかを
//           設定画面の右上に出す（描画は display_tft.cpp の draw_setting_mode）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/22
// ============================================================
//
// ■ なぜ
//   飛ぶ前に見る項目（GNSS の精度・姿勢推定・較正・設定の赤丸・電池・SD・無線）は、
//   それぞれ別の画面と別の音にある。台の上でそれを 1 つずつ見て回らなくても済むように、
//   全部そろったら声で知らせる。足りないものは設定画面の右上に短い名前で並べる。
//
// ■ 決めごと
//   ★ **ここは判定だけ。** 描画は draw_setting_mode()（CLAUDE.md「警報を描画関数の中に書かない」）。
//   ★ **既にある判定を写さない。** 設定画面の丸の色（menu_settings[].iconColor）、
//     地上の姿勢チェック（ground_*_warn_active）、送信前チェック（link_preflight_*）を
//     そのまま読む。条件を書き写すと、片方だけ直されたときに食い違う。
//   ★ 読み上げは 1 回だけ。欠けた状態が READY_REARM_MS 続いたら、もう一度言えるようにする
//     （何かを直したあとにまた言ってほしい。一方、境目でそろったり欠けたりするたびに
//     言わせるとうるさい）。
//   ★ 飛行中は言わない（地上で止まっているときだけ）。着陸しても、飛行中にそろったままなら
//     言い直さない（言い直すのは欠けてから戻ったときだけなので）。
//   ★ 「準備完了」は**足りないものが無い**という意味で、機体の安全を保証するものではない。
#include "readiness.h"
#include "settings.h"
#include "gnss.h"
#include "imu.h"
#include "attitude.h"
#include "link.h"
#include "mysd.h"
#include "power.h"
#include "display_tft.h"   // get_input_voltage / battery_percent / COLOR_RED / SETTING_*
#include "src/button.h"    // menu_settings（設定画面の丸の色）

// 地上の姿勢チェックが発報中か（GPS_TFT_map.ino の ground_check_tick が毎周回立て直す）。
extern bool ground_roll_warn_active;
extern bool ground_pitch_warn_active;

static bool     s_applicable = false;
static uint16_t s_missing    = 0;

bool     readiness_applicable() { return s_applicable; }
uint16_t readiness_missing()    { return s_missing; }

// 設定画面に赤丸があるか。IMU と無線の行は、この下で別の項目（ESKF・CALIB・LINK）として
// 見るので数えない（同じ理由を 2 つの名前で出さないため）。
static bool any_setting_red() {
  for (int i = 0; i < setting_size; i++) {
    if (menu_settings[i].iconColor == nullptr) continue;
    const int id = menu_settings[i].id;
    if (id == SETTING_IMU_DETAIL || id == SETTING_WIRELESS) continue;
    if (menu_settings[i].iconColor() == COLOR_RED) return true;
  }
  return false;
}

static uint16_t evaluate() {
  uint16_t m = 0;

  // GNSS（**自機の**受信機。受信モードは対象外なのでミラーは通らない）
  double lat, lon, gs, trk;
  if (!get_gnss_connection() || !gnss_get_own_fix(lat, lon, gs, trk) ||
      get_gnss_fixtype() < 3 || !get_gnss_fixok() ||
      get_gnss_hacc_mm() > (uint32_t)(READY_HACC_MAX_M * 1000.0f))
    m |= RDY_GNSS;

  // IMU と姿勢。IMU を載せていない機体（GNSS ナビだけの機体）は対象外。
  // 姿勢の機能を設定で OFF にしているときも対象外（使わないと決めたもので止めない）。
  if (get_imu_ok()) {
    if (!get_imu_alive()) m |= RDY_IMU;
    if (attitude_get_rpy_enabled()) {
      if (!attitude_ready()) m |= RDY_ESKF;
      // ★ 「較正した日から日付をまたいでいる」（attitude_calib_date_stale）は**入れないこと。**
      //   あれは推測（付けっぱなしで日をまたぐこともある）なので、ground_check_tick は
      //   1 度言うだけにしている。ここに入れると、正しい較正のまま台に上がった機体が
      //   NOT READY になり、「止めるために台の上で APPLY して、正しかった較正を台の姿勢で
      //   上書きする」ことを促してしまう（GPS_TFT_map.ino の該当コメント）。
      //   入れるのは確定している「マウントから外された」だけ。
      if (attitude_needs_apply()) m |= RDY_CALIB;
      if (ground_roll_warn_active || ground_pitch_warn_active) m |= RDY_ATT;
    }
  }

  if (any_setting_red()) m |= RDY_SET;

  // 電源。USB がつながっている間は電池の残量が測れない（v7 は VSYS、v7.1 の SW OFF は 0V）。
  if (usb_power_present() || power_sw_off_on_usb()) {
    m |= RDY_USB;
  } else if (power_fault_active() ||
             battery_percent(get_input_voltage()) < READY_BATT_MIN_PERCENT) {
    m |= RDY_BATT;
  }

  if (!good_sd()) m |= RDY_SD;

  // 無線。送信モードのときだけ（OFF は「使わない」と決めたもの、受信モードは対象外）。
  // ★ 雑音が高い（NOISY / BURST）は止めない。送信前チェック自体が「送信は止めない」作りで、
  //   チャンネルを変えるかは人の判断（docs/pons_link.md）。止めるのは、送信機が 2 台・
  //   同じチャンネルに別のグループ・モジュール無応答のように、**混信して読み違える**ものだけ。
  if (link_get_mode() == LINK_MODE_TX) {
    const uint8_t f = link_preflight_flags();
    if (!link_module_alive() || link_preflight_state() == LINK_PF_RUNNING ||
        (f & (LINK_PFF_PONS_SAME | LINK_PFF_PONS_OTHER | LINK_PFF_SKIPPED)))
      m |= RDY_LINK;
  }
  return m;
}

void readiness_format_missing(uint16_t missing, char* buf, size_t n) {
  static const struct { uint16_t bit; const char* name; } kItems[] = {
    { RDY_GNSS, "GNSS" }, { RDY_IMU, "IMU" },   { RDY_ESKF, "ESKF" }, { RDY_CALIB, "CALIB" },
    { RDY_ATT,  "ATT"  }, { RDY_SET, "SET" },   { RDY_USB,  "USB"  }, { RDY_BATT,  "BATT"  },
    { RDY_SD,   "SD"   }, { RDY_LINK, "LINK" },
  };
  if (n == 0) return;
  buf[0] = '\0';
  size_t len = 0;
  for (const auto& it : kItems) {
    if (!(missing & it.bit)) continue;
    const int w = snprintf(buf + len, n - len, len ? " %s" : "%s", it.name);
    if (w < 0 || (size_t)w >= n - len) break;   // 入りきらない分は落とす（途中で切れた名前は残さない）
    len += (size_t)w;
  }
  buf[len] = '\0';
}

void readiness_tick() {
  static uint32_t last_ms        = 0;
  static uint32_t ready_since    = 0;   // そろった（かつ止まっている）状態が始まった時刻
  static uint32_t notready_since = 0;   // 欠けた状態が始まった時刻
  static bool     announced      = false;
  const uint32_t now = millis();
  if (now - last_ms < 1000) return;
  last_ms = now;

  s_applicable = !getReplayMode() && !is_demo_active() && link_get_mode() != LINK_MODE_RX;
  if (!s_applicable) {
    s_missing   = 0;
    ready_since = 0;
    return;
  }
  s_missing = evaluate();

  if (s_missing == 0) {
    notready_since = 0;
    // 地上で止まっているときだけ言う（自機の GNSS。evaluate() が fix を確かめている）
    double lat, lon, gs, trk;
    const bool still = gnss_get_own_fix(lat, lon, gs, trk) && gs <= READY_STILL_MPS;
    if (!still) { ready_since = 0; return; }
    if (ready_since == 0) ready_since = now;
    if (!announced && now - ready_since >= READY_HOLD_MS) {
      announced = true;
      // 「準備完了。」優先度 2（無線モードの読み上げと同じ）。
      enqueueTask(createPlayWavTask("wav/ready.wav", 2));
      enqueueTask(createLogSdTask("READY: all preflight checks passed"));
    }
  } else {
    ready_since = 0;
    if (notready_since == 0) notready_since = now;
    if (announced && now - notready_since >= READY_REARM_MS) {
      announced = false;   // 欠けたまま続いたので、直ったらもう一度言う
      char b[64];
      readiness_format_missing(s_missing, b, sizeof b);
      enqueueTask(createLogSdfTask("READY lost: %s", b));
    }
  }
}
