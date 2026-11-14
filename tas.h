// ============================================================
// File    : tas.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 推定 TAS（対気速度）の表示の判断。ヘッダー右の大きい枠に何を出すか
//           （真方位 / 平均ピッチ / 推定 TAS）を、設定・地上と飛行中・表示の供給元
//           （実センサー / リプレイ / 無線ミラー / デモ）から 1 か所で決める。
//           仕様は docs/pons_tas.md。推定 TAS の計算そのものは attitude.cpp（attitude_get_tas）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================
//
// ★ 判断はここ、描画は display_tft.cpp。描画の中で設定や地上判定を見ないこと
//   （CLAUDE.md「警報を描画関数の中に書かない」と同じ理由。描画は tas_header() を読むだけ）。
//
#ifndef TAS_H
#define TAS_H

#include <Arduino.h>

// 大きい枠に出しているもの。
// ★ **値を変えないこと。** 飛行 CSV の hdr 列にこの数字をそのまま書いていて、
//   リプレイは記録された値で当時の画面を出す（過去の CSV の意味が変わってしまう）。
enum TasHdr : uint8_t {
  HDR_TT         = 0,   // 真方位（平均ピッチが無い・姿勢機能 OFF・IMU 無し）
  HDR_PAVG       = 1,   // 平均ピッチ
  HDR_TAS        = 2,   // 推定 TAS（青い箱に数字）
  HDR_TAS_CLAMP  = 3,   // 推定 TAS が上下限に張り付いている（灰色の箱。推定ではなく端の値）
  HDR_TAS_GROUND = 4,   // TAS を選んでいるが地上（青い箱の数字に横線。2026-11-07 までは「-.-」）
};
#define HDR_COUNT 5

static inline bool tas_hdr_is_tas(TasHdr h) {
  return h == HDR_TAS || h == HDR_TAS_CLAMP || h == HDR_TAS_GROUND;
}

// ---- 設定（IMU/ESKF 画面 1 ページ目の「Header: Pavg / TAS」。settings.txt の header_tas）----
// この機体の大きい枠を TAS にするか。受信機（ボート）のミラー中は自分のこれではなく、
// 送信機の設定（無線で受けたビット）に従う。
void tas_setting_set(bool tas);
bool tas_setting_get();

// loop() の最上位から毎回呼ぶ。地上と飛行中の切り替え（settings.h の TAS_*）。
void tas_tick();

// 地上（HDR_TAS_GROUND）のまま、飛び出す側の条件（対地速度 TAS_AIR_GS_MPS 以上）が続いているか（2026-11-14〜）。
// GROUND の札はこれが true になった時点で外す。線は今までどおり TAS_SWITCH_HOLD_MS 続いてから外す
// （札が 10 秒残ると「いつ消えるのか」と気になるため。線は平均ピッチの窓が飛行中の値だけになるまで残す意味がある）。
// リプレイ中は再生中の対地速度から決める（札は記録に無い）。
bool tas_leaving_ground();

// いま大きい枠に出すもの（供給元を解決済み）。描画・CSV の hdr 列・受信ログはこれを読む。
TasHdr tas_header();
// TAS の箱（HDR_TAS / HDR_TAS_CLAMP / HDR_TAS_GROUND）に出す推定 TAS [m/s]。
float  tas_header_value();
// 大きい枠の平均ピッチ（HDR_PAVG）と、TAS の箱の横に小さく残す平均ピッチ [度]。
// 供給元を解決済み（ミラー中は送信機、リプレイ中は記録値）。無ければ false。
bool   tas_header_pitch_avg(float &avg);

// ---- 自機の値（無線で送る・飛行 CSV に書く）----
// 送信機の設定（Pavg / TAS）にかかわらず、出せる限り出す。姿勢機能 OFF では出さない（平均ピッチと同じ）。
bool tas_local(float &mps, bool &clamped);

#endif // TAS_H
