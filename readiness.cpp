// ============================================================
// File    : readiness.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 本番前チェック（v7 の 0.994 の v6 版。無線の項目は無い）。飛ぶ前に確かめる項目が
//           そろったら、地上で止まっているときに 1 度だけ「準備完了」と読み上げる。
//           そろっていなければ何が足りないかを設定画面の右上に出す
//           （描画は display_tft.cpp の draw_setting_mode）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//
// ■ なぜ
//   飛ぶ前に見る項目（GPS の精度・姿勢推定・較正・設定の赤丸・電池・SD）は、
//   それぞれ別の画面と別の音にある。台の上でそれを 1 つずつ見て回らなくても済むように、
//   全部そろったら声で知らせる。足りないものは設定画面の右上に短い名前で並べる。
//
// ■ 決めごと
//   ★ **ここは判定だけ。** 描画は draw_setting_mode()（警報を描画関数の中に書かない）。
//   ★ **既にある判定を写さない。** 設定画面の丸の色（menu_settings[].iconColor）、
//     地上の姿勢チェック（ground_*_warn_active）をそのまま読む。条件を書き写すと、
//     片方だけ直されたときに食い違う。
//   ★ 読み上げは 1 回だけ。欠けた状態が READY_REARM_MS 続いたら、もう一度言えるようにする。
//   ★ 飛行中は言わない（地上で止まっているときだけ）。
//   ★ 「準備完了」は**足りないものが無い**という意味で、機体の安全を保証するものではない。
//   ★ v6 は BNO085 が載っていない機体の方が多い。載っていない機体では IMU・ESKF・CALIB・ATT を見ない。
#include "readiness.h"
#include "settings.h"
#include "gps.h"
#include "imu.h"
#include "attitude.h"
#include "mysd.h"
#include "power.h"
#include "display_tft.h"   // get_input_voltage / battery_percent / COLOR_RED / SETTING_*
#include "button.h"        // menu_settings（設定画面の丸の色）

// 地上の姿勢チェックが発報中か（GPS_TFT_map.ino の地上チェックが毎周回立て直す）。
extern bool ground_roll_warn_active;
extern bool ground_pitch_warn_active;

static bool     s_applicable = false;
static uint16_t s_missing    = 0;

bool     readiness_applicable() { return s_applicable; }
uint16_t readiness_missing()    { return s_missing; }

// 設定画面に赤丸があるか。IMU の行は、この下で別の項目（ESKF・CALIB）として
// 見るので数えない（同じ理由を 2 つの名前で出さないため）。
static bool any_setting_red() {
  for (int i = 0; i < setting_size; i++) {
    if (menu_settings[i].iconColor == nullptr) continue;
    if (menu_settings[i].id == SETTING_IMU_DETAIL) continue;
    if (menu_settings[i].iconColor() == COLOR_RED) return true;
  }
  return false;
}

static uint16_t evaluate() {
  uint16_t m = 0;

  if (!get_gps_connection() || !get_gps_fix() || get_gps_fixtype() < 3 ||
      !get_gps_gnssFixOK() ||
      get_gps_hacc_mm() > (uint32_t)(READY_HACC_MAX_M * 1000.0f))
    m |= RDY_GNSS;

  // IMU と姿勢。BNO085 を載せていない機体は対象外。
  // 姿勢の機能を設定で OFF にしているときも対象外（使わないと決めたもので止めない）。
  // ★ get_imu_ok() ではなく get_imu_present()。v6 の get_imu_ok() は通信が途絶えると false に
  //   落ちるので、それで分岐すると **BNO085 が止まった機体で IMU の項目を丸ごと見なくなり、
  //   READY になってしまう**（v7 の get_imu_ok() は「載っているか」で意味が違う）。
  if (get_imu_present()) {
    if (!get_imu_alive()) m |= RDY_IMU;
    if (attitude_get_rpy_enabled()) {
      if (!attitude_ready()) m |= RDY_ESKF;
      if (attitude_needs_apply()) m |= RDY_CALIB;
      if (ground_roll_warn_active || ground_pitch_warn_active) m |= RDY_ATT;
    }
  }

  if (any_setting_red()) m |= RDY_SET;

  // 電源。USB がつながっている間は電池の残量が測れない（測れるのは VSYS＝USB 側の電圧）。
  if (usb_power_present()) {
    m |= RDY_USB;
  } else if (power_fault_active() ||
             battery_percent(get_input_voltage()) < READY_BATT_MIN_PERCENT) {
    m |= RDY_BATT;
  }

  if (!good_sd()) m |= RDY_SD;
  return m;
}

void readiness_format_missing(uint16_t missing, char* buf, size_t n) {
  static const struct { uint16_t bit; const char* name; } kItems[] = {
    { RDY_GNSS, "GNSS" }, { RDY_IMU, "IMU" },   { RDY_ESKF, "ESKF" }, { RDY_CALIB, "CALIB" },
    { RDY_ATT,  "ATT"  }, { RDY_SET, "SET" },   { RDY_USB,  "USB"  }, { RDY_BATT,  "BATT"  },
    { RDY_SD,   "SD"   },
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

  s_applicable = !getReplayMode() && !is_demo_active();
  if (!s_applicable) {
    s_missing   = 0;
    ready_since = 0;
    return;
  }
  s_missing = evaluate();

  if (s_missing == 0) {
    notready_since = 0;
    // 地上で止まっているときだけ言う（evaluate() が fix を確かめている）
    const bool still = get_gps_fix() && get_gps_mps() <= READY_STILL_MPS;
    if (!still) { ready_since = 0; return; }
    if (ready_since == 0) ready_since = now;
    if (!announced && now - ready_since >= READY_HOLD_MS) {
      announced = true;
      // 「準備完了。」SD が無いときは鳴らない（SD が無ければそもそも SD の項目で欠ける）。
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
