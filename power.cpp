// ============================================================
// File    : power.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報。
//           v7.1 では「SW を OFF にしたまま USB で充電している」の検出と読み上げも。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/20
// ============================================================
//
// ■ 何を見張るか（docs/pons_power.md）
//   2026-10-15、v7 の充電 IC が電池につながったまま内部で壊れ、
//   電池 → 充電 IC → VBUS → D3 → VSYS と回って SW で切れず、画面は「USB」になった。
//   本物の USB なら電圧は 4.3V 以上あるので、「USB に見えるのに電圧が低い」が
//   続いたら異常として、画面・音・ログ・無線で知らせる。判定値は settings.h。
//
// ■ SW を OFF にしたまま USB で充電している（v7.1 のみ）
//   v7.1 は充電 IC が SW（ロードスイッチ Q3）の後ろにあるので、SW OFF のままだと電池へは
//   Q3 のボディダイオード経由になり、満充電の手前で充電が終わる。本体は USB で起動し、
//   充電 LED も消えるので見た目では分からない。SW4 の OFF 側の接点が GPIO40 を GND に
//   落とすので、「本体が動いているのに GPIO40 ≒ 0V」で分かる。地図に注意を出し、1 度だけ読み上げる。
//
// ★ 警報はここ（loop() の最上位）で鳴らす。描画は display_tft.cpp が
//   power_fault_active() を見て描く（CLAUDE.md「警報を描画関数の中に書かない」）。
#include "power.h"
#include "settings.h"
#include "mysd.h"
#include "gnss.h"         // gnss_get_own_fix()（移動中の判定）・リプレイ／デモ中か
#include "link.h"         // link_mirror_active()

#define POWER_SAMPLE_MS 250UL

static bool     s_fault       = false;
static uint32_t s_cond_since  = 0;     // 異常の条件が成り立ち始めた時刻（0 = 成り立っていない）
static uint32_t s_last_alert  = 0;
static uint32_t s_last_sample = 0;
static bool     s_usb_pin     = false; // v7: USB_DETECT の生の値
static float    s_volt        = 0.0f;  // v7: VSYS / v7.1: VBUS（生の平均。ピーク追跡は通さない）
#if PONS_BOARD == PONS_BOARD_V71
static float    s_batt_v      = 0.0f;  // GPIO40（BATPWR）の生の平均。SW OFF なら SW4 が GND に落とすので 0V
static bool     s_swoff       = false; // SW OFF のまま USB（CHARGE_SW_OFF_HOLD_MS 続いて確定）
static uint32_t s_swoff_since = 0;
static bool     s_swoff_said  = false; // この回の SW OFF をもう読み上げたか
static bool     s_moving      = false;
static uint32_t s_still_since = 0;     // 移動中に止まり始めた時刻（0 = まだ止まっていない）
#endif

// ADC の生の平均を電圧にする。ピーク追跡（get_input_voltage）は高い値を
// 数秒保持するので、ここでは使わない。
static float read_volt(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return BATTERY_MULTIPLYER(sum / 4.0f);
}

static void sample() {
#if PONS_BOARD == PONS_BOARD_V71
  s_volt   = read_volt(USB_VBUS_ADC_PIN);
  s_batt_v = read_volt(BATTERY_PIN);
#else
  s_usb_pin = digitalRead(USB_DETECT);
  s_volt    = read_volt(BATTERY_PIN);
#endif
}

// 異常の条件（1 回の測定ぶん）。
static bool fault_condition() {
#if PONS_BOARD == PONS_BOARD_V71
  // ★ SW OFF のあいだは判定しない。電池は Q3 で切り離されていて（ボディダイオードも
  //   電池 → 充電 IC の向きには流さない）、壊れた充電 IC が電池で VBUS を持ち上げることが
  //   できない。このとき VBUS が 2.0〜4.3V なのは**弱い USB**（細いケーブル・へたった
  //   モバイルバッテリー）で、異常と言うと誤報になる。
  if (power_sw_off_on_usb()) return false;
  return s_volt >= POWER_FAULT_VBUS_MIN_V && s_volt < USB_VBUS_PRESENT_V;
#else
  return s_usb_pin && s_volt < POWER_FAULT_VSYS_V;
#endif
}

bool power_fault_active() { return s_fault; }

bool usb_power_present() {
  if (s_fault) return false;
#if PONS_BOARD == PONS_BOARD_V71
  return s_volt >= USB_VBUS_PRESENT_V;
#else
  // 異常が確定するまで（最長 POWER_FAULT_HOLD_MS）は従来どおり USB 扱い。
  // 1 回の測定で揺らすと「USB」表示がちらつくため。
  return s_usb_pin;
#endif
}

bool power_sw_off_on_usb() {
#if PONS_BOARD == PONS_BOARD_V71
  // ★ **USB の判定（usb_power_present()）を条件に入れないこと。** 本体が動いているのに
  //   電池が 0V に見えるなら、それだけで SW OFF と言い切れる（SW ON なら GPIO40 は電池の
  //   電圧で、保護回路付きの 1s は 2.5V を切らない。SW OFF なら電源は USB しかない）。
  //   USB を条件に入れると、細いケーブルで VBUS が一瞬 4.3V を割っただけで外れ、
  //   その瞬間に 0V が本物の電池電圧として電池低下の警告・無線・CSV へ流れる。
  // ★ 確定（s_swoff）を待たずに今の測定で答える。確定までの 2 秒間も 0V を本物として
  //   扱わないようにするため。
  return s_batt_v < CHARGE_SW_OFF_BATT_V;
#else
  return false;
#endif
}

#if PONS_BOARD == PONS_BOARD_V71
// 地図が自機の状態を映しているか。ミラー中は機体、リプレイ中は過去のログ、デモ中は
// 仮想機体を映しているので、自機の充電の話は重ねない。
// （replay_voltage_active() はリプレイ中しか true にならないので getReplayMode() で足りる）
static bool showing_own_state() {
  return !getReplayMode() && !link_mirror_active() && !is_demo_active();
}

// 移動中か。**自機の** GNSS（ミラーを通さない）の対地速度で見る。
// ★ fix が無いときは静止とみなす（屋内で充電している場面が本命なので、fix 待ちで黙らせない）。
// ★ GNSS が抜けると stored_* が凍ったまま fix が残るので、接続も見る。
// ★ 止まってもすぐには「静止」に戻さない（CHARGE_SW_STILL_MS）。信号待ちのたびに出たり消えたりしない。
static void update_moving(uint32_t now) {
  double lat, lon, gs, trk;
  const bool fix = get_gnss_connection() && gnss_get_own_fix(lat, lon, gs, trk);
  if (fix && gs > CHARGE_SW_MOVING_MPS) {
    s_moving      = true;
    s_still_since = 0;
  } else if (s_moving) {
    if (s_still_since == 0) s_still_since = now;
    else if (now - s_still_since >= CHARGE_SW_STILL_MS) s_moving = false;
  }
}
#endif

bool power_sw_off_notice_active() {
#if PONS_BOARD == PONS_BOARD_V71
  // !s_fault: 地図の段を電源経路の異常と共用しているので、重ねない（settings.h の NOTICE_Y_*）。
  return s_swoff && !s_fault && !s_moving && showing_own_state();
#else
  return false;
#endif
}

void power_format_status(char* buf, size_t n) {
#if PONS_BOARD == PONS_BOARD_V71
  snprintf(buf, n, "usb=%d vbus=%.2fV%s", usb_power_present() ? 1 : 0, s_volt,
           power_sw_off_on_usb() ? " sw=off" : "");
#else
  snprintf(buf, n, "usb=%d", usb_power_present() ? 1 : 0);
#endif
}

void power_setup() {
#if PONS_BOARD != PONS_BOARD_V71
  pinMode(USB_DETECT, INPUT);
#endif
  sample();
}

#if PONS_BOARD == PONS_BOARD_V71
// SW を OFF にしたまま USB で充電している、の確定と読み上げ。
// ★ 読み上げは**この回に 1 度だけ**。一晩充電する場面で繰り返すとうるさい。画面には出し続ける。
// ★ 読み上げたあとに SW が入ったら charge_sw_on.wav で知らせる。同じ排他グループなので、
//   割り込まれて pending に残っていた「切れています」もここで捨てられる
//   （残すと SW を入れたあとに「切れています」と嘘を喋る。docs/pons_sound.md §3）。
static void sw_off_tick(uint32_t now) {
  if (!getReplayMode()) update_moving(now);   // リプレイ中の速度は過去のログなので見ない

  if (!power_sw_off_on_usb()) {
    s_swoff_since = 0;
    if (s_swoff) {
      // ここへ来るのは電池電圧が読めるようになったとき＝SW が入ったときだけ
      // （SW OFF のまま USB を抜けば本体ごと止まる）。
      s_swoff = false;
      enqueueTask(createLogSdfTask("CHARGE: SW ON (batt=%.2fV)", s_batt_v));
      if (s_swoff_said)
        enqueueTask(createPlayWavTask("wav/charge_sw_on.wav", 1, 0, WAV_EXCL_CHARGE_SW));
    }
    s_swoff_said = false;
    return;
  }
  if (!s_swoff) {
    if (s_swoff_since == 0) { s_swoff_since = now; return; }
    if (now - s_swoff_since < CHARGE_SW_OFF_HOLD_MS) return;
    s_swoff = true;
    enqueueTask(createLogSdTask("CHARGE: SW OFF on USB (battery cut off by Q3, will not charge fully)"));
  }
  // 優先度は最低（1）、最低音量なし。画面に出ているので、音量 0 の機体まで鳴らす話ではない。
  if (!s_swoff_said && now >= CHARGE_SW_OFF_ARM_MS && power_sw_off_notice_active()) {
    s_swoff_said = true;
    enqueueTask(createPlayWavTask("wav/charge_sw_off.wav", 1, 0, WAV_EXCL_CHARGE_SW));
  }
}
#endif

void power_tick() {
  const uint32_t now = millis();
  if (now - s_last_sample < POWER_SAMPLE_MS) return;
  s_last_sample = now;
  sample();
#if PONS_BOARD == PONS_BOARD_V71
  sw_off_tick(now);
#endif

  if (!s_fault) {
    if (now < POWER_FAULT_ARM_MS || !fault_condition()) {
      s_cond_since = 0;
    } else if (s_cond_since == 0) {
      s_cond_since = now;
    } else if (now - s_cond_since >= POWER_FAULT_HOLD_MS) {
      s_fault = true;
      s_last_alert = 0;
#if PONS_BOARD == PONS_BOARD_V71
      enqueueTask(createLogSdfTask("POWER FAULT: VBUS=%.2fV (not USB, not 0)", s_volt));
#else
      enqueueTask(createLogSdfTask("POWER FAULT: USB_DETECT=1 but VSYS=%.2fV", s_volt));
#endif
    }
  }

  if (s_fault && (s_last_alert == 0 || now - s_last_alert >= POWER_FAULT_ALERT_MS)) {
    s_last_alert = now;
    // 「電源回路に異常があります。すぐにスイッチを切って、電池とUSBを外してください。」
    // ★ 文言は v7 と v7.1 の両方で正しい手順にしてある（v7 は電池を抜くしかなく、v7.1 は
    //   SW で切れる。画面の POWER_FAULT_ACTION_TEXT は基板ごとに違うが、音声はその和）。
    // ★ macOS の音声合成で作った**仮の音声**。録音したら同じ名前で wav/ に置き、
    //   tools/clean_wav8.py → tools/gen_wav_flash.py を回す。
    // ★ 最低音量 60 は電池警告だけの約束だが、これは**発火の恐れ**なので例外にする
    //   （音量 0 の機体でも聞こえないと意味が無い）。docs/pons_sound.md §5。
    enqueueTask(createPlayWavTask("wav/power_fault.wav", 4, 60));
  }
}
