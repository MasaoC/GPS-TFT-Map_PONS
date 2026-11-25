// ============================================================
// File    : live.h
// Project : PONS v7.1 / v7.2
// Role    : ライブ配信。受信機が受けたテレメトリ・画面の判断・受信機の状態を USB へ行で流し、
//           Re:PONS（tools/repons/live.html）が機体の画面を描き直して OBS で配信する
//           （docs/pons_live_stream_plan.md）。**受信機だけの機能。**
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/24
// ============================================================
//
// ★ 守ること（設計書 §1）
//   ・受信機の本来の仕事（ミラー表示・音・received/ の記録・無線の受け方・描き直しの時刻）を変えない。
//     PONS:CONNECT は使わない（PC 接続の画面に入ると received/ の記録が止まる）。
//   ・**Core0 を USB で止めない。** arduino-pico の Serial.write() は、PC がポートを開いたまま読まないと
//     送信バッファ（256 バイト）が空くまで最大 1 秒待つ。LIVE の行は RAM のリングに積み、
//     Serial.availableForWrite() に行がまるごと入るときだけ書く。入らなければ次の周回へ回し、
//     リングが溢れたら古い行から捨てて数える（60 秒ログの LIVE 行の dropped=）。
//     **LIVE の行を Serial.print で直接書かないこと。**
//   ・配信にボートの値を機体の値として出さない。判断の行（LN）はミラー中だけ。
//     リプレイ中・デモ中（link_rf_suppressed()）は状態の行（LS）だけ。
//   ・判断は本体の 1 か所で。PC（Re:PONS）は届いた判断を描くだけ。
//
// ★ 行の形は docs/pons_webserial_protocol.md §4 (9)。変えたら LIVE_FMT_VER を上げ、Re:PONS の live.js も直す。
#ifndef LIVE_H
#define LIVE_H

#include <Arduino.h>
#include "lora_link/link_proto.h"

// 行の形の版（PONS:LIVE,OK の 4 つめ）。Re:PONS は合わなければ描かない。
#define LIVE_FMT_VER 1

// "PONS:LIVE," の後ろ（"ON" / "OFF"）。.ino の handle_serial_command() から呼ぶ。
// ★ ファイルの転送中・PC 接続の画面の扱いは .ino が先に済ませてから呼ぶ（転送中は BUSY、PC 接続は抜ける）。
void live_command(const char* arg);
// 止める（PONS:CONNECT を受けた・受信モードでなくなった など）。動いていなければ何もしない。
// why は PONS:LIVE,OFF,<why> と log.txt に出る（"CMD" / "CONNECT" / "MODE" / "PORT"）。
void live_stop(const char* why);
bool live_active();

// loop() の最上位で毎回（受信機の状態の行・止める条件・リングを書けるだけ書く・目的地と地図の一覧を少しずつ）。
void live_tick();
// 自分のグループのテレメトリを受けたとき（link.cpp の link_poll_radio()。received/ に書くのと同じ瞬間・同じ値）。
// known は回転枠 8 つのうち受けたもの（LinkW2Rx の slot_seen）。
void live_on_rx_frame(const LinkTelem* t, uint8_t slot, uint8_t known);
// loop() で画面を描いた後（.ino の draw_current_screen() の直後）。新しいフレームの後なら判断の行（LN）を作る。
void live_after_screen();

// 受信機の TFT の LIVE の印（display_tft.cpp の draw_link_icons()）。0 = 出さない / 1 = 緑（流している）/ 2 = 赤（捨てている）
uint8_t live_mark_state();
// 60 秒ログ（.ino の periodic_log_tick()）。動いている間・この 1 分に動いていたときだけ 1 行。
void live_periodic_log();

#endif // LIVE_H
