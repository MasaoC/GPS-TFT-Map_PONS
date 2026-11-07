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
  BL_SAVER_CRIT = 2,  // 第 2 段階（BAT_CRIT_VOLTAGE 以下）: BL_SAVER_CRIT_ON_MS 点灯 / BL_SAVER_CRIT_OFF_MS 消灯
};

// 輝度スイッチ Low のときの照度→点灯率の曲線（設定画面の「AUTO Brightness」。SD の auto_brightness）。
// 値は settings.h の BL_AUTO_* / BL_ECO_*。
enum BacklightAutoMode : uint8_t {
  BL_AUTO_ECO    = 0,  // 控えめ（既定）。1000 lx で 50%、20000 lx で 70%
  BL_AUTO_BRIGHT = 1,  // 1000 lx 以上で最大
};
BacklightAutoMode backlight_get_auto_mode();
void backlight_set_auto_mode(BacklightAutoMode m);   // 次の backlight_tick() ですぐに効かせる

void backlight_setup();      // setup_tft() より前に呼ぶ（PWM を設定するまで消灯しているため）
void backlight_als_setup();  // airdata_wire_begin() の後に呼ぶ（VEML7700 は i2c0）
void backlight_tick();       // loop() から毎回呼ぶ（内部で間引く）
// 電池低下の点滅モードを切り替える。段階が変わったときは点灯の側から周期を始める。
void backlight_set_saver(BacklightSaver mode);

// 待機中自動減光中か。
bool backlight_is_dimmed();
// ユーザー操作（ボタン等）や活動発生時に呼び、減光解除・ディミングタイマーをリセットする。
void backlight_notify_activity();

// ログ用（例: " bl=35% lux=12"）。
void backlight_format_status(char* buf, size_t n);

// 飛行 CSV 用。読めていないときは負の値（CSV では空欄）。
float backlight_get_lux();       // いまの照度 [lx]。センサーが応答していないときは -1
int   backlight_get_duty_pct();  // いまの点灯率 [%]

// 画面用（起動画面の VEML 表示・設定画面のフッター）。
bool  backlight_als_ok();        // VEML7700 が応答しているか
// いまの点灯率をどの決まりで決めているか: "SW BRIGHT"（輝度スイッチ High）/ "IDLE DIM"（待機中の自動減光）/
// "NO SENSOR"（照度センサーが一度も読めていない）/ "AUTO"（照度で自動）/ "LAST LUX"（読めなくなり最後の照度のまま）
const char* backlight_mode_label();
BacklightSaver backlight_get_saver();   // 電池低下の点滅の段階

#endif
