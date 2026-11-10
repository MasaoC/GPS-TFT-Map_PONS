// ============================================================
// File    : power.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告、地図の充電ポップアップの判断（2026-11-08〜）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/10
// ============================================================
//
// ■ 何を見張るか（docs/pons_power.md）
//   2026-10-15、v7 の充電 IC が電池につながったまま内部で壊れ、
//   電池 → 充電 IC → VBUS → D3 → VSYS と回って SW で切れず、画面は「USB」になった。
//   本物の USB なら電圧は 4.3V 以上あるので、「USB に見えるのに電圧が低い」が
//   続いたら異常として、画面・音・ログ・無線で知らせる。判定値は settings.h。
//
// ■ 高温での充電
//   電池の仕様は「充電 0〜45°C」。MCP73831 には温度を入れる端子が無く、夏の車内や日なたでも
//   充電を続ける。USB がつながっている間に MS5611（本体基板）の温度が CHARGE_HOT_C を
//   超え続けたら、地図に出して読み上げる。止める手段（充電 IC の PROG を切る GPIO）は無いので、
//   USB を抜いてもらう。
//
// ■ 充電ポップアップ（2026-11-08〜。v7.1 の 1.01 から移した。中身は v7 で出せるものだけ）
//   v7 は電池の電圧を USB 中に測れない（GPIO40 は VSYS で、USB 中は USB の電圧になる）。充電 IC の
//   STAT も LED にしかつながっていない。なので v7.1 のような**残量と残り時間は出せない**。
//   出すのは USB をつないでからの時間・GNSS の時計の電池（BT1）の目安・「側面の赤い LED が消えたら満充電」だけ。
//   出す条件は v7.1 と同じ（USB・異常なし・高温なし・地上で止まっている・自機の表示・ボタンで隠していない）。
//
// ★ 警報はここ（loop() の最上位）で鳴らす。描画は display_tft.cpp が
//   power_fault_active() を見て描く（CLAUDE.md「警報を描画関数の中に書かない」）。
#include "power.h"
#include "settings.h"
#include "mysd.h"
#include "airdata.h"      // get_airdata_temperature()（高温での充電）
#include "gnss.h"         // gnss_get_own_fix()・gnss_rtc_alive_on_boot()・リプレイ／デモ中か（充電ポップアップ）
#include "link.h"         // link_mirror_active()（充電ポップアップ）
#include "rtc_backup.h"   // GNSS のバックアップ電池の推定（power_tick から回す・充電ポップアップの 2 行目）

#define POWER_SAMPLE_MS 250UL

static bool     s_fault       = false;
static uint32_t s_cond_since  = 0;     // 異常の条件が成り立ち始めた時刻（0 = 成り立っていない）
static uint32_t s_last_alert  = 0;
static uint32_t s_last_sample = 0;
static bool     s_usb_pin     = false; // USB_DETECT の生の値
static float    s_volt        = 0.0f;  // VSYS（生の平均。ピーク追跡は通さない）
static bool     s_hot         = false; // 高温での充電（CHARGE_HOT_HOLD_MS 続いて確定）
static uint32_t s_hot_since   = 0;
static uint32_t s_hot_alert   = 0;
static float    s_hot_temp    = 0.0f;  // 判定に使った温度 [°C]（表示・ログ用）
static uint32_t s_usb_connected_ms = 0;      // USB をつないだ時刻 [ms]（0 = USB なし）
static bool     s_moving      = false;       // 自機が動いている（充電ポップアップを出さない）
static uint32_t s_still_since = 0;           // 移動中に止まり始めた時刻（0 = まだ止まっていない）
static uint32_t s_chg_popup_snooze_since = 0; // 充電ポップアップをボタンで隠した時刻（0 = 隠していない）

// ADC の生の平均を電圧にする。ピーク追跡（get_input_voltage）は高い値を
// 数秒保持するので、ここでは使わない。
static float read_volt(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return BATTERY_MULTIPLYER(sum / 4.0f);
}

static void sample() {
  s_usb_pin = digitalRead(USB_DETECT);
  s_volt    = read_volt(BATTERY_PIN);
}

// 異常の条件（1 回の測定ぶん）。
static bool fault_condition() {
  return s_usb_pin && s_volt < POWER_FAULT_VSYS_V;
}

bool power_fault_active() { return s_fault; }

bool usb_power_present() {
  if (s_fault) return false;
  // 異常が確定するまで（最長 POWER_FAULT_HOLD_MS）は従来どおり USB 扱い。
  // 1 回の測定で揺らすと「USB」表示がちらつくため。
  return s_usb_pin;
}

bool  power_charge_hot_active() { return s_hot; }
float power_charge_hot_temp()   { return s_hot_temp; }

void power_format_status(char* buf, size_t n) {
  const char* hot = s_hot ? " hot" : "";
  char bk[16] = "";
  if (rtc_backup_known()) snprintf(bk, sizeof(bk), " bk=%.1fh", rtc_backup_hours());
  snprintf(buf, n, "usb=%d%s%s", usb_power_present() ? 1 : 0, hot, bk);
}

void power_setup() {
  pinMode(USB_DETECT, INPUT);
  sample();
}


// 高温での充電（上の ■）。USB がつながっている間だけ見る。
// ★ 温度は get_airdata_temperature()（MS5611 の実測。リプレイでも差し替わらない）。
// ★ 電源経路の異常のあいだは見ない（usb_power_present() が false になる）。あちらが優先。
// ★ ミラー中・リプレイ中・デモ中も出す。**自機が熱い**という話なので、電源経路の異常と同じく
//   「ミラー中に自機の状態を混ぜない」の例外（docs/pons_power.md）。
static void charge_hot_tick(uint32_t now) {
  const bool watch = usb_power_present() && get_airdata_ok();
  const float t = watch ? get_airdata_temperature() : 0.0f;
  if (!watch) {
    if (s_hot) {
      const char* why = usb_power_present() ? "temperature sensor lost"
                      : (s_fault ? "power fault" : "USB removed");
      enqueueTask(createLogSdfTask("CHARGE HOT cleared (%s)", why));
    }
    s_hot = false;
    s_hot_since = 0;
    return;
  }
  s_hot_temp = t;
  if (!s_hot) {
    if (t < CHARGE_HOT_C) { s_hot_since = 0; return; }
    if (s_hot_since == 0) { s_hot_since = now ? now : 1; return; }       // 0 は「まだ」の印
    if (now - s_hot_since < CHARGE_HOT_HOLD_MS) return;
    s_hot = true;
    s_hot_alert = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT: %.1fC (limit %.0fC while charging)", t, CHARGE_HOT_C));
  } else if (t <= CHARGE_HOT_CLEAR_C) {
    s_hot = false;
    s_hot_since = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT cleared: %.1fC", t));
    return;
  }
  if (s_hot_alert == 0 || now - s_hot_alert >= CHARGE_HOT_ALERT_MS) {
    s_hot_alert = now;
    // 「充電中の温度が高すぎます。USBを外して、涼しい場所で冷ましてください。」
    // ★ 最低音量 60: 電池の仕様外の充電で、発火の恐れにつながるので電源経路の異常と同じ扱い
    //   （docs/pons_sound.md §5）。
    enqueueTask(createPlayWavTask("wav/charge_hot.wav", 3, 60));
  }
}

// 地図が自機の状態を映しているか。ミラー中は機体、リプレイ中は過去のログ、デモ中は
// 仮想機体を映しているので、自機の充電の話は重ねない。
static bool showing_own_state() {
  return !getReplayMode() && !link_mirror_active() && !is_demo_active();
}

// 移動中か。**自機の** GNSS（ミラーを通さない）の対地速度で見る。v7.1 の update_moving と同じ。
// ★ fix が無いときは静止とみなす（屋内で充電している場面が本命なので、fix 待ちで黙らせない）。
// ★ GNSS が抜けると値が凍ったまま fix が残るので、接続も見る。
// ★ 止まってもすぐには「静止」に戻さない（CHG_POPUP_STILL_MS）。信号待ちのたびに出たり消えたりしない。
static void update_moving(uint32_t now) {
  double lat, lon, gs, trk;
  const bool fix = get_gnss_connection() && gnss_get_own_fix(lat, lon, gs, trk);
  if (fix && gs > CHG_POPUP_MOVING_MPS) {
    s_moving      = true;
    s_still_since = 0;
  } else if (s_moving) {
    if (s_still_since == 0) s_still_since = now ? now : 1;
    else if (now - s_still_since >= CHG_POPUP_STILL_MS) s_moving = false;
  }
}

bool power_charge_popup_active() {
  const uint32_t now = millis();
  if (s_chg_popup_snooze_since != 0 && now - s_chg_popup_snooze_since < CHG_POPUP_SNOOZE_MS) return false;
  if (!usb_power_present()) return false;   // 電源経路の異常のあいだも false になる
  if (s_fault || s_hot) return false;
  if (s_moving || !showing_own_state()) return false;
  return true;
}

void power_charge_popup_snooze() {
  s_chg_popup_snooze_since = millis() ? millis() : 1;
}

uint32_t power_charging_elapsed_sec() {
  if (!usb_power_present() || s_usb_connected_ms == 0) return 0;
  return (millis() - s_usb_connected_ms) / 1000UL;
}

// GNSS のバックアップ電池（BT1 = MS621FE）が「あと何時間もつか」の推定（充電ポップアップの 2 行目 "Fast GPS if OFF < 1.5 days"）。
// 推定は rtc_backup.cpp（v7.1 と同じ。通電した時間で足し、電源を切っていた時間で引く。起動時の時計の生死で引き戻す）。
const char* power_rtc_backup_status_str() {
  static char buf[32];   // 1 行まるごと（最長 26 文字）
  rtc_backup_format(buf, sizeof(buf));
  return buf;
}

void power_tick() {
  const uint32_t now = millis();
  if (now - s_last_sample < POWER_SAMPLE_MS) return;
  s_last_sample = now;
  sample();
  // USB をつないでからの時間（充電ポップアップ）
  if (usb_power_present()) {
    if (s_usb_connected_ms == 0) s_usb_connected_ms = now ? now : 1;
  } else {
    s_usb_connected_ms = 0;
  }
  if (!getReplayMode()) update_moving(now);   // リプレイ中の速度は過去のログなので見ない
  rtc_backup_tick();       // GNSS のバックアップ電池の推定（rtc_backup.cpp）
  charge_hot_tick(now);

  if (!s_fault) {
    if (now < POWER_FAULT_ARM_MS || !fault_condition()) {
      s_cond_since = 0;
    } else if (s_cond_since == 0) {
      s_cond_since = now ? now : 1;   // 0 は「まだ」の印
    } else if (now - s_cond_since >= POWER_FAULT_HOLD_MS) {
      s_fault = true;
      s_last_alert = 0;
      enqueueTask(createLogSdfTask("POWER FAULT: USB_DETECT=1 but VSYS=%.2fV", s_volt));
    }
  }

  if (s_fault && (s_last_alert == 0 || now - s_last_alert >= POWER_FAULT_ALERT_MS)) {
    s_last_alert = now;
    // 「電源回路に異常があります。すぐにスイッチを切って、電池とUSBを外してください。」
    // ★ 文言は v7 と v7.1 の両方で正しい手順にしてある（v7 は電池を抜くしかなく、v7.1 は
    //   SW で切れる。画面の POWER_FAULT_ACTION_TEXT は基板ごとに違うが、音声はその和）。
    // ★ 最低音量 60 は電池警告だけの約束だが、これは**発火の恐れ**なので例外にする
    //   （音量 0 の機体でも聞こえないと意味が無い）。docs/pons_sound.md §5。
    enqueueTask(createPlayWavTask("wav/power_fault.wav", 4, 60));
  }
}
