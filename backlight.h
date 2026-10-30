// ============================================================
// File    : backlight.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : v7.1 の TFT バックライト。輝度スイッチと照度センサー（VEML7700）で
//           PWM の点灯率を決める。電池低下のときは 2 段階で点滅させて電池をもたせる。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/29
// ============================================================
#ifndef BACKLIGHT_H
#define BACKLIGHT_H

#include <Arduino.h>

// 電池低下時の省電力点滅モード（判定は GPS_TFT_map.ino の batt_alarm_tick）。
enum BacklightSaver : uint8_t {
  BL_SAVER_OFF  = 0,  // 常時点灯
  BL_SAVER_LOW  = 1,  // 第 1 段階（BAT_LOW_VOLTAGE 以下）: BL_SAVER_ON_MS 点灯 / BL_SAVER_OFF_MS 消灯
  BL_SAVER_CRIT = 2,  // 第 2 段階（BAT_CRIT_VOLTAGE 以下）: BL_SAVER_ON_MS 点灯 / BL_SAVER_CRIT_OFF_MS 消灯
};

void backlight_setup();      // setup_tft() より前に呼ぶ（PWM を設定するまで消灯しているため）
void backlight_als_setup();  // airdata_wire_begin() の後に呼ぶ（VEML7700 は i2c0）
void backlight_tick();       // loop() から毎回呼ぶ（内部で間引く）
// 電池低下の点滅モードを切り替える。段階が変わったときは点灯の側から周期を始める。
void backlight_set_saver(BacklightSaver mode);

// ログ用（例: " bl=35% lux=12"）。
void backlight_format_status(char* buf, size_t n);

// 飛行 CSV 用。読めていないときは負の値（CSV では空欄）。
float backlight_get_lux();       // いまの照度 [lx]。センサーが応答していないときは -1
int   backlight_get_duty_pct();  // いまの点灯率 [%]

#endif
