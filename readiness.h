// ============================================================
// File    : readiness.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 本番前チェック（v7 の 0.994 の v6 版。無線の項目は無い）。飛ぶ前に確かめる項目が
//           そろったら、地上で止まっているときに 1 度だけ「準備完了」と読み上げる。
//           そろっていなければ何が足りないかを設定画面の右上に出す
//           （描画は display_tft.cpp の draw_setting_mode）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
#ifndef READINESS_H
#define READINESS_H

#include <Arduino.h>

// 足りない項目（ビット）。並びは設定画面に出す順。判定値は settings.h の READY_*。
#define RDY_GNSS   0x0001  // 3D fix・gnssFixOK・hAcc ≤ READY_HACC_MAX_M
#define RDY_IMU    0x0002  // BNO085 は載っているのに応答しない
#define RDY_ESKF   0x0004  // 姿勢推定がまだ収束していない
#define RDY_CALIB  0x0008  // 較正のやり直しが必要（マウントから外された）
#define RDY_ATT    0x0010  // 地上で姿勢が申告値（SET ROLL / SET PITCH）からずれ続けている
#define RDY_SET    0x0020  // 設定画面に赤丸がある（IMU の行は ESKF・CALIB で見る）
#define RDY_USB    0x0040  // USB がつながっている
#define RDY_BATT   0x0080  // 電池残量が READY_BATT_MIN_PERCENT 未満・電源経路の異常
#define RDY_SD     0x0100  // SD カードが使えない

void     readiness_tick();         // loop() の最上位から毎回呼ぶ（内部で 1 秒に 1 回）
// 判定の対象か。リプレイ中・デモ中は false（どちらも「この機体がこれから飛ぶ」話ではない）。
bool     readiness_applicable();
uint16_t readiness_missing();      // 足りない項目（RDY_*）。0 ならそろっている
// 足りない項目を "GNSS ESKF SD" のような短い文字列にする。
void     readiness_format_missing(uint16_t missing, char* buf, size_t n);

#endif
