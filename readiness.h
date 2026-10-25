// ============================================================
// File    : readiness.h
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 本番前チェック（v7 の readiness の v3 版）。設定画面の右上に READY / NOT READY を出す。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
#ifndef READINESS_H
#define READINESS_H

#include <Arduino.h>

// そろっていれば true。欠けている項目を空白区切りで missing に入れる（例: "GNSS USB"）。
//   GNSS : 3D 測位・gnssFixOK でない、または水平精度（NAV-PVT の hAcc）が READY_HACC_MAX_M を超えている
//   DEMO : デモのまま
//   REPLAY : SD の飛行 CSV を再生している（デモと同じく実際の位置を使っていない）
//   USB  : USB がつながっている
//   BATT : 電池残量が READY_BATT_MIN_PERCENT 未満、または電源経路の異常
//   SD   : SD カードが使えない
bool readiness_check(char* missing, size_t n);

#endif
