// ============================================================
// File    : power.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告（v7 の 0.993 / 0.994 の v5 版。温度は CPU 内蔵のセンサー）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

void power_setup();   // setup() で analogReadResolution(12) の後に呼ぶ
void power_tick();    // loop() の最上位で毎回呼ぶ（内部で間引く）

// 本物の USB から給電されているか。
// ★ USB の有無はこれだけで判定する。digitalRead(USB_DETECT) を直接書かないこと
//   （2026-10-15、v7 で壊れた充電 IC が VBUS を持ち上げて「USB」と誤判定し、
//   電池低下警告まで止めていた。v5 も同じ考え方の電源回路なので同じことが起こる）。
bool usb_power_present();

// 電源経路の異常（再起動まで消えない）。
bool power_fault_active();

// USB がつながっている間に CPU 温度が CHARGE_HOT_C を超え続けている（電池の仕様は
// 充電 0〜45°C）。地図の注意と読み上げは電源経路の異常と同じく、リプレイ中・デモ中も出す。
bool  power_charge_hot_active();
float power_charge_hot_temp();   // 判定に使っている温度 [°C]
// CPU 温度 [°C]（2 秒ごとに測って平滑化したもの）。NAN = まだ測っていない／読み違い。
// ★ analogReadTemp() を直接呼ばないこと（1 回 1ms ほど止まる。表示もこれを使う）。
float power_cpu_temp();

#endif
