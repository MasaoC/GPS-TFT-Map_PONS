// ============================================================
// File    : power.cpp
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。電源経路の異常（充電 IC の故障）の検出、USB 判定、
//           電池の状態と残量、高温での充電の警告。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//
// ■ 何を見張るか
//   2026-10-15、v7 の充電 IC（MCP73831）が壊れて電池から VBUS へ逆流し、発熱した（README 冒頭）。
//   v3 では充電 IC が SW の後ろにあるので、壊れると SW ON の間「電池 → 充電 IC → VBUS → Pico の D1
//   → VSYS」と流れる。本物の USB なら VSYS は 4.4V 以上あり、逆流なら電池電圧（4.2V 以下）より低い。
//   「VBUS があるのに VSYS が低い」が続いたら異常として、画面とログで知らせる。判定値は settings.h。
//
// ■ v3 で使える信号（どちらも Pico 基板に作り込まれていて、v3 基板と干渉しない）
//   GPIO24 : VBUS を 5.6k/10k で分圧したデジタル入力。逆流の VBUS（3V 前後）では High/Low が際どい。
//   GPIO29 : VSYS を 200k/100k で 1/3 にした ADC。v3 基板は VSYS を使っていないので VBUS − D1 になり、
//            USB が無ければほぼ 0V。そこで「GPIO24 が High、または VSYS が 2V 以上」を VBUS ありとみなす。
//   GPIO26 : 電池（SW の後ろの VCC）を 1/11 に分圧した ADC。換算は BATTERY_MULTIPLYER。
//
// ■ 電池の表示
//   以前は display_tft.cpp の描画関数の中で判定していて、地図を描き直したときしか更新されず、
//   下降の時定数も描画の回数で変わっていた。判定はここ（loop() の最上位）に集め、描画は結果を読むだけにした。
//
// ★ v3 には音が無いので、知らせるのは画面だけ。描画は display_tft.cpp が下の関数を見て描く
//   （ここに描画を書かない）。
#include "power.h"
#include "settings.h"
#include "mysd.h"

#define POWER_SAMPLE_MS 250UL
#define SWING_SAMPLES   8      // BATOFF の振れを見る幅（POWER_SAMPLE_MS × 8 = 2 秒）

static bool     s_fault       = false;
static uint32_t s_pin_since   = 0;     // GPIO24 High ＆ VSYS が低い、が成り立ち始めた時刻（0 = 成り立っていない）
static uint32_t s_vsys_since  = 0;     // VBUS あり（GPIO24 か VSYS）＆ VSYS が低い、が成り立ち始めた時刻
static uint32_t s_last_sample = 0;
static uint32_t s_last_log    = 0;
static bool     s_logged      = false;
static bool     s_vbus_pin    = false; // GPIO24 の生の値
static int      s_vsys_adc    = 0;
static float    s_vsys        = 0.0f;
static int      s_bat_adc     = 0;

static float    s_bat_disp    = 0.0f;  // 表示用の電池電圧（0 = まだ測っていない）
static uint32_t s_bat_peak_ms = 0;     // 表示用の電圧を最後に引き上げた時刻
static uint32_t s_bat_last_ms = 0;
static int      s_swing[SWING_SAMPLES];
static int      s_swing_n     = 0;     // s_swing に入っている数
static int      s_swing_i     = 0;
static uint32_t s_last_swing  = 0;     // 最後に振れを見た時刻（0 = 見ていない）

static bool     s_hot         = false;
static uint32_t s_hot_since   = 0;
static uint32_t s_last_temp   = 0;
static float    s_cpu_temp    = NAN;

// 平均した ADC 値。チャンネルを切り替えた直後の 1 回は前のチャンネルの影響が残りうるので捨てる。
static int read_adc_avg(uint8_t pin) {
  analogRead(pin);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return (int)(sum / 8);
}

static void sample() {
  s_vbus_pin = digitalRead(VBUS_DETECT_PIN);
  s_vsys_adc = read_adc_avg(VSYS_ADC_PIN);
  s_vsys     = VSYS_MULTIPLYER(s_vsys_adc);
  s_bat_adc  = read_adc_avg(BATTERY_PIN);
}

// 条件が hold_ms 続いたら true。since は成り立ち始めた時刻（0 = 成り立っていない）。
static bool held(bool cond, uint32_t& since, uint32_t now, uint32_t hold_ms) {
  if (!cond) { since = 0; return false; }
  if (since == 0) { since = now; return false; }
  return now - since >= hold_ms;
}

bool power_fault_active() { return s_fault; }

bool usb_power_present() {
  return !s_fault && s_vbus_pin && s_vsys >= POWER_FAULT_VSYS_V;
}

// 表示用の電池電圧。上がるときは即追従、下がるときは BATTERY_PEAK_HOLD_MS 待ってから
// BATTERY_DECAY_TAU_MS の時定数で寄せる（v7 と同じ。GPS 受信などの負荷の谷を拾わない）。
static void update_battery(uint32_t now) {
  const float v = BATTERY_MULTIPLYER(s_bat_adc);
  if (s_bat_disp == 0.0f || v >= s_bat_disp) {
    s_bat_disp = v;
    s_bat_peak_ms = now;
  } else if (now - s_bat_peak_ms > BATTERY_PEAK_HOLD_MS) {
    const float dt = (float)(now - s_bat_last_ms);
    s_bat_disp += (v - s_bat_disp) * (dt / ((float)BATTERY_DECAY_TAU_MS + dt));
  }
  s_bat_last_ms = now;

  // SW OFF のまま USB をつなぐと、充電 IC が電池を探して VCC が上下に振れる。直近 2 秒の幅で見る。
  s_swing[s_swing_i] = s_bat_adc;
  s_swing_i = (s_swing_i + 1) % SWING_SAMPLES;
  if (s_swing_n < SWING_SAMPLES) s_swing_n++;
  int lo = s_swing[0], hi = s_swing[0];
  for (int i = 1; i < s_swing_n; i++) {
    if (s_swing[i] < lo) lo = s_swing[i];
    if (s_swing[i] > hi) hi = s_swing[i];
  }
  if (usb_power_present() && hi - lo > BATOFF_SWING_COUNTS) s_last_swing = now;
}

BattState power_battery_state() {
  if (s_fault) return BATT_FAULT;
  if (usb_power_present()) {
    const bool swinging = s_last_swing != 0 && millis() - s_last_swing < BATOFF_HOLD_MS;
    return swinging ? BATT_OFF : BATT_CHARGING;
  }
  if (s_bat_disp < BAT_LOW_VOLTAGE) return BATT_LOW;
  return BATT_NORMAL;
}

float power_battery_volts() { return s_bat_disp; }

int power_battery_percent() {
  return constrain((int)((s_bat_disp - BAT_ZERO_VOLTAGE) /
                         (BAT_FULL_VOLTAGE - BAT_ZERO_VOLTAGE) * 100.0f), 0, 100);
}

bool  power_charge_hot_active() { return s_hot; }
float power_cpu_temp()          { return s_cpu_temp; }

// 高温での充電。USB がつながっている間だけ見る。電源経路の異常のあいだは見ない（あちらが優先）。
static void charge_hot_tick(uint32_t now) {
  if (s_last_temp == 0 || now - s_last_temp >= CPU_TEMP_SAMPLE_MS) {
    s_last_temp = now;
    const float t = analogReadTemp(ADC_VREF_V);
    // ★ ADC_VREF_V が 1% ずれると約 4°C ずれる（温度センサーの 0.7V 付近を 1.7mV/°C で読むため）。
    //   ありえない値は ADC_VREF_V が違う証拠なので、使わずに「分からない」にする（警告もしない）。
    if (t < CPU_TEMP_MIN_C || t > CPU_TEMP_MAX_C) {
      s_cpu_temp = NAN;
    } else {
      // 1 回の読み取りは ±1°C ほど揺れるので軽くならす
      s_cpu_temp = isnan(s_cpu_temp) ? t : s_cpu_temp + (t - s_cpu_temp) * 0.3f;
    }
  }
  if (!usb_power_present()) {
    if (s_hot) enqueueLogSdf("CHARGE HOT cleared (%s)", s_fault ? "power fault" : "USB removed");
    s_hot = false;
    s_hot_since = 0;
    return;
  }
  if (!s_hot) {
    if (held(!isnan(s_cpu_temp) && s_cpu_temp >= CHARGE_HOT_C, s_hot_since, now, CHARGE_HOT_HOLD_MS)) {
      s_hot = true;
      enqueueLogSdf("CHARGE HOT: CPU %.1fC (limit %.0fC while charging)", s_cpu_temp, CHARGE_HOT_C);
    }
  } else if (isnan(s_cpu_temp) || s_cpu_temp <= CHARGE_HOT_CLEAR_C) {
    s_hot = false;
    s_hot_since = 0;
    enqueueLogSdf("CHARGE HOT cleared: CPU %.1fC", s_cpu_temp);
  }
}

void power_format_status(char* buf, size_t n) {
  snprintf(buf, n, "VBUS:%d VSYS:%.2fV(%d) BAT:%.2fV(%d)",
           s_vbus_pin ? 1 : 0, s_vsys, s_vsys_adc, BATTERY_MULTIPLYER(s_bat_adc), s_bat_adc);
}

void power_setup() {
  pinMode(VBUS_DETECT_PIN, INPUT);  // プル無し（Pico 基板の 10k で下がっている）
  sample();
  update_battery(millis());
}

void power_tick() {
  const uint32_t now = millis();
  if (now - s_last_sample < POWER_SAMPLE_MS) return;
  s_last_sample = now;
  sample();

  if (!s_fault) {
    const bool armed = now >= POWER_FAULT_ARM_MS;
    const bool low   = s_vsys < POWER_FAULT_VSYS_V;
    // GPIO24 で VBUS が見えている場合（3 秒）と、VSYS だけで見えている場合（10 秒）に分ける。
    // 後者は USB を抜いた直後に VSYS がゆっくり下がる間も成り立つので、長く待つ。
    const bool by_pin  = held(armed && low && s_vbus_pin, s_pin_since, now, POWER_FAULT_HOLD_MS);
    const bool by_vsys = held(armed && low && (s_vbus_pin || s_vsys >= POWER_VBUS_SEEN_VSYS_V),
                              s_vsys_since, now, POWER_FAULT_HOLD_VSYS_MS);
    if (by_pin || by_vsys) {
      s_fault = true;
      char st[64];
      power_format_status(st, sizeof(st));
      Serial.print("POWER FAULT: ");
      Serial.println(st);
      enqueueLogSdf("POWER FAULT: %s", st);
    }
  }

  update_battery(now);
  charge_hot_tick(now);

  // ADC_VREF_V の確かめと、後から経緯を追うための記録（USB をつないで VSYS が 4.7V 前後なら正しい）。
  // 最初は起動 10 秒後、以降 POWER_LOG_MS ごと。
  if ((!s_logged && now >= 10000UL) || (s_logged && now - s_last_log >= POWER_LOG_MS)) {
    s_logged = true;
    s_last_log = now;
    char st[64];
    power_format_status(st, sizeof(st));
    Serial.print("PWR ");
    Serial.println(st);
    enqueueLogSdf("PWR %s disp=%.2fV %d%% usb=%d CPU%.1fC%s%s", st, s_bat_disp,
                                 power_battery_percent(), usb_power_present() ? 1 : 0, s_cpu_temp,
                                 s_hot ? " hot" : "", s_fault ? " FAULT" : "");
  }
}
