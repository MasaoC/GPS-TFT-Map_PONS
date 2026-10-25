// ============================================================
// File    : power.cpp
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告（v7 の 0.993 / 0.994 の v5 版。温度は CPU 内蔵のセンサー）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//
// ■ 何を見張るか
//   2026-10-15、v7 の充電 IC（MCP73831）が電池につながったまま内部で壊れ、
//   電池 → 充電 IC → VBUS → VSYS と回って SW で切れず、画面は「USB」になった。
//   v5 の電源回路も同じ考え方（充電 IC の VBAT は電源スイッチの手前、VDD は VBUS。
//   USB 検出は Pico 2 の GPIO24、VSYS は Pico 2 基板上の 1/3 分圧で GPIO29）。
//   本物の USB なら VSYS は約 4.7V あるので、「USB に見えるのに電圧が低い」が
//   続いたら異常として、画面・音・ログで知らせる。判定値は settings.h。
//
// ■ 高温での充電
//   電池の仕様は「充電 0〜45°C」。MCP73831 には温度を入れる端子が無く、夏の車内や日なたでも
//   充電を続ける。USB がつながっている間に CPU 温度（RP2350 内蔵の温度センサー）が
//   CHARGE_HOT_C を超え続けたら、地図に出して読み上げる。止める手段は無いので、USB を抜いてもらう。
//   ★ v5 には温度センサーが無い。CPU 温度は電池の温度そのものではなく、マイコン自身の発熱で
//     周囲より高めに出る（＝早めに警告する側）。目安として使う（v3 の 0.503 と同じ考え方）。
//
// ★ 警報はここ（loop() の最上位）で鳴らす。描画は display_tft.cpp が
//   power_fault_active() を見て描く（警報を描画関数の中に書かない）。
// ★ 音声は本体 FLASH に内蔵している（SD が無くても鳴る）。
#include "power.h"
#include "settings.h"
#include "mysd.h"

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
static float    s_cpu_temp    = NAN;   // CPU 温度 [°C]（平滑化済み。NAN = まだ測っていない／読み違い）
static uint32_t s_last_temp   = 0;

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
float power_cpu_temp()          { return s_cpu_temp; }

void power_setup() {
  pinMode(USB_DETECT, INPUT);
  sample();
}

// 音声を鳴らす（FLASH 内蔵なので SD の状態は見ない）。
static void play_alert(const char* wav, int priority) {
  enqueueTask(createPlayWavTask(wav, priority, 60));
}

// CPU 温度を測る（上の ■）。USB の有無に関係なく測る（設定画面の表示にも使うため）。
static void sample_cpu_temp(uint32_t now) {
  if (s_last_temp != 0 && now - s_last_temp < CPU_TEMP_SAMPLE_MS) return;
  s_last_temp = now;
  const float t = analogReadTemp(ADC_VREF_V);
  // ★ ADC_VREF_V が 1% ずれると約 4°C ずれる（温度センサーの 0.7V 付近を 1.7mV/°C で読むため）。
  //   ありえない値は読み違いなので、使わずに「分からない」にする（警告もしない）。
  if (t < CPU_TEMP_MIN_C || t > CPU_TEMP_MAX_C) {
    s_cpu_temp = NAN;
    return;
  }
  // 1 回ごとのばらつき（±1°C ほど）で境目を出入りしないよう、軽くならす。
  s_cpu_temp = isnan(s_cpu_temp) ? t : s_cpu_temp + (t - s_cpu_temp) * 0.3f;
}

// 高温での充電（上の ■）。USB がつながっている間だけ見る。
// ★ 温度は CPU 温度（リプレイでも差し替わらない）。測れていないときは判定しない。
// ★ 電源経路の異常のあいだは見ない（usb_power_present() が false になる）。あちらが優先。
// ★ リプレイ中・デモ中も出す。**自機が熱い**という話で、画面に出している数字の話ではない。
static void charge_hot_tick(uint32_t now) {
  const bool watch = usb_power_present() && !isnan(s_cpu_temp);
  const float t = watch ? s_cpu_temp : 0.0f;
  if (!watch) {
    if (s_hot) {
      const char* why = usb_power_present() ? "CPU temperature unreadable"
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
    if (s_hot_since == 0) { s_hot_since = now; return; }
    if (now - s_hot_since < CHARGE_HOT_HOLD_MS) return;
    s_hot = true;
    s_hot_alert = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT: CPU %.1fC (limit %.0fC while charging)", t, CHARGE_HOT_C));
  } else if (t <= CHARGE_HOT_CLEAR_C) {
    s_hot = false;
    s_hot_since = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT cleared: CPU %.1fC", t));
    return;
  }
  if (s_hot_alert == 0 || now - s_hot_alert >= CHARGE_HOT_ALERT_MS) {
    s_hot_alert = now;
    // 「充電中の温度が高すぎます。USBを外して、涼しい場所で冷ましてください。」
    // ★ 最低音量 60: 電池の仕様外の充電で、発火の恐れにつながるので電源経路の異常と同じ扱い。
    play_alert("wav/charge_hot.wav", 3);
  }
}

void power_tick() {
  const uint32_t now = millis();
  if (now - s_last_sample < POWER_SAMPLE_MS) return;
  s_last_sample = now;
  sample();
  sample_cpu_temp(now);
  charge_hot_tick(now);

  if (!s_fault) {
    if (now < POWER_FAULT_ARM_MS || !fault_condition()) {
      s_cond_since = 0;
    } else if (s_cond_since == 0) {
      s_cond_since = now;
    } else if (now - s_cond_since >= POWER_FAULT_HOLD_MS) {
      s_fault = true;
      s_last_alert = 0;
      enqueueTask(createLogSdfTask("POWER FAULT: USB_DETECT=1 but VSYS=%.2fV", s_volt));
    }
  }

  if (s_fault && (s_last_alert == 0 || now - s_last_alert >= POWER_FAULT_ALERT_MS)) {
    s_last_alert = now;
    // 「電源回路に異常があります。すぐにスイッチを切って、電池とUSBを外してください。」
    // ★ 最低音量 60 は電池警告だけの約束だが、これは**発火の恐れ**なので例外にする
    //   （音量 0 の機体でも聞こえないと意味が無い）。
    play_alert("wav/power_fault.wav", 4);
  }
}
