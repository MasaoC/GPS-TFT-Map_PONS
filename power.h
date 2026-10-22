// ============================================================
// File    : power.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告。
//           v7.1 では「SW を OFF にしたまま USB で充電している」の検出と読み上げも。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/22
// ============================================================
#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

void power_setup();   // setup() で analogReadResolution(12) の後に呼ぶ
void power_tick();    // loop() の最上位で毎回呼ぶ（内部で間引く）

// 本物の USB から給電されているか。
// ★ USB の有無はこれだけで判定する。VBUS の ADC（USB_VBUS_ADC_PIN）を直接読まないこと
//   （2026-10-15、壊れた充電 IC が VBUS を持ち上げて「USB」と誤判定し、
//   電池低下警告まで止めていた）。
bool usb_power_present();

// 電源経路の異常（再起動まで消えない）。
bool power_fault_active();

// v7.1: SW を OFF にしたまま USB で動いているか（電池は Q3 で切り離されている）。
// 判定は「GPIO40 が 0V に見える」だけ（SW OFF なら電源は USB しかないので、USB の判定は使わない）。
// ★ このあいだ GPIO40 は SW で GND に落ちているので、**電池電圧は読めない**
//   （get_input_voltage() は 0V へ向かって下がる）。電池電圧を表示・送信・記録・警告する
//   箇所はこれを見て「不明」として扱うこと。0V を本物として扱うと、残量 0% 表示・
//   電池低下の誤報（自機と受信機）・CSV の 0.00V になる。
bool power_sw_off_on_usb();

// 「SW を入れて充電して」の注意を地図に出すか。
// 移動中・ミラー中・リプレイ中・デモ中・電源経路の異常・高温の充電のあいだは false。
bool power_sw_off_notice_active();

// USB がつながっている間に本体基板（MS5611）が CHARGE_HOT_C を超え続けている（電池の仕様は
// 充電 0〜45°C）。地図の注意と読み上げは電源経路の異常と同じく、ミラー中・リプレイ中も出す。
bool  power_charge_hot_active();
float power_charge_hot_temp();   // 判定に使っている温度 [°C]

// ログ用の短い文字列（例: "usb=1" / "usb=1 vbus=4.95V"）。
void power_format_status(char* buf, size_t n);

#endif
