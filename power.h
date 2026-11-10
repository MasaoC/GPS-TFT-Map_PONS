// ============================================================
// File    : power.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告、地図の充電ポップアップの判断。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/10
// ============================================================
#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

void power_setup();   // setup() で analogReadResolution(12) の後に呼ぶ
void power_tick();    // loop() の最上位で毎回呼ぶ（内部で間引く）

// 本物の USB から給電されているか。
// ★ USB の有無はこれだけで判定する。digitalRead(USB_DETECT) を直接書かないこと
//   （2026-10-15、壊れた充電 IC が VBUS を持ち上げて「USB」と誤判定し、
//   電池低下警告まで止めていた）。
bool usb_power_present();

// 電源経路の異常（再起動まで消えない）。
bool power_fault_active();

// USB がつながっている間に本体基板（MS5611）が CHARGE_HOT_C を超え続けている（電池の仕様は
// 充電 0〜45°C）。地図の注意と読み上げは電源経路の異常と同じく、ミラー中・リプレイ中も出す。
bool  power_charge_hot_active();
float power_charge_hot_temp();   // 判定に使っている温度 [°C]

// ---- 地図の充電ポップアップ（2026-11-08〜。判断はここ、描画は display_tft.cpp の draw_charge_popup）----
// ★ v7 は USB 中に電池の電圧を測れないので、残量と残り時間は出さない（power.cpp の ■）。
bool        power_charge_popup_active();    // USB・異常なし・高温なし・地上で止まっている・自機の表示・隠していない
void        power_charge_popup_snooze();    // ボタン操作で CHG_POPUP_SNOOZE_MS だけ隠す
uint32_t    power_charging_elapsed_sec();   // USB をつないでからの秒数（USB なしは 0）（2026-11-10 から画面では使っていない）
const char* power_rtc_backup_status_str();  // 充電ポップアップの 2 行目 "Fast GPS if OFF < 1.5 days"（BT1 の残り。rtc_backup.cpp）

// ログ用の短い文字列（例: "usb=1" / "usb=1 vbus=4.95V"）。
void power_format_status(char* buf, size_t n);

#endif
