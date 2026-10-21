// ============================================================
// File    : backlight.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : v7.1 の TFT バックライト。輝度スイッチと照度センサー（VEML7700）で
//           PWM の点灯率を決める。v7 では何もしない（スイッチで直接切り替える）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/19
// ============================================================
#ifndef BACKLIGHT_H
#define BACKLIGHT_H

#include <Arduino.h>

void backlight_setup();      // setup_tft() より前に呼ぶ（PWM を設定するまで消灯しているため）
void backlight_als_setup();  // airdata_wire_begin() の後に呼ぶ（VEML7700 は i2c0）
void backlight_tick();       // loop() から毎回呼ぶ（内部で間引く）

// ログ用（例: " bl=35% lux=12"）。v7 では空文字。
void backlight_format_status(char* buf, size_t n);

// 飛行 CSV 用。読めていないとき・v7 では負の値（CSV では空欄）。
float backlight_get_lux();       // いまの照度 [lx]。センサーが応答していないときは -1
int   backlight_get_duty_pct();  // いまの点灯率 [%]

#endif
