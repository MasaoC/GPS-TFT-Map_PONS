// ============================================================
// File    : power.h
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。電源経路の異常（充電 IC の故障）の検出、USB 判定、
//           電池の状態と残量、高温での充電の警告。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

void power_setup();   // setup() で呼ぶ
void power_tick();    // loop() の最上位で毎回呼ぶ（内部で間引く）

// 電源経路の異常（再起動まで消えない）。
bool power_fault_active();

// 本物の USB から給電されているか（異常のあいだは false）。
bool usb_power_present();

// 電池の状態。表示（display_tft.cpp）と本番前チェック（readiness.cpp）はこれだけを見る。
enum BattState {
  BATT_NORMAL,    // 電池で動いている
  BATT_LOW,       // 電池で動いていて BAT_LOW_VOLTAGE 未満
  BATT_CHARGING,  // USB があり、電池もつながっている（SW ON）
  BATT_OFF,       // USB があるが SW OFF（電池が切り離されていて充電されない）
  BATT_FAULT,     // 電源経路の異常
};
BattState power_battery_state();
float power_battery_volts();     // 表示用の電圧（上がるときは即追従・下がるときはゆっくり）
int   power_battery_percent();   // BAT_ZERO_VOLTAGE〜BAT_FULL_VOLTAGE の線形（v7 と同じ）

// 高温での充電（USB がつながっている間、RP2040 のチップ温度が CHARGE_HOT_C を超え続けている）。
bool  power_charge_hot_active();
float power_cpu_temp();          // RP2040 のチップ温度 [°C]（ADC_VREF_V で換算）

// 表示・ログ用の短い文字列（例: "VBUS:1 VSYS:4.78V(2105) BAT:4.11V(527)"）。
void power_format_status(char* buf, size_t n);

#endif
