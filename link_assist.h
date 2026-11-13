// ============================================================
// File    : link_assist.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS の時刻の助け合い（LoRa）。BT1 が切れて GNSS が時刻を知らない機体・ボートへ、時刻を持つ
//           仲間が精度つきで時刻を渡す（2026-11-13〜）。GNSS へ入れるのは時刻だけ（gnss_dbd_give_time）。
//           v7 と v7.1 で同じ中身。計画と決めごとは docs/pons_ttff_assist_plan.md §4。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/13
// ============================================================
//
// ■ 誰が誰を助けるか
//   ・ボート（未 fix）← 機体のテレメトリ（回転枠 0 の日付と時 ＋ 毎回の 0.1 秒時刻）。新しい送信は無い。
//                       機体が fix していなくても、GNSS の時計（RTC）の時刻が精度つきで載っていれば使う（2026-11-13〜）
//   ・機体（未 fix）  ← 時刻を持つボートの提供ビーコン（fix していなくても RTC の時刻を精度つきで）。
//                       機体はテレメトリの status に LINK_ST_TIME_WANTED を立て、窓の間だけ送信の合間も mode 0 で聞く
//   ・ボート同士      ← 困っているボートの要求ビーコンに、時刻を持つボートが提供ビーコンで答える
// ■ 守ること（CLAUDE.md・計画書 §1.2）
//   ・上り（ボート → 機体）で変えられるのは「GNSS へ時刻を 1 回入れる」ことだけ。機体の操作・設定・表示・航法は変えない
//   ・機体の 1Hz のテレメトリはタイミングも中身も変えない（status に時刻のビットを足すだけ）
//   ・ビーコンはテレメトリと別の印。ミラー・NO SIGNAL・DUP SENDER・GROUP X FOUND の判定に混ぜない
//   ・別のグループのビーコン・テレメトリの時刻は使わない。リプレイ中・デモ中は送らない・使わない
//   ・settings.txt の link_assist:OFF で全部止まる
//
#ifndef LINK_ASSIST_H
#define LINK_ASSIST_H

#include <Arduino.h>
#include "lora_link/link_wire2.h"

// settings.txt の link_assist（1 = ON / 0 = OFF。既定 ON）。mysd.cpp が読み書きする
extern uint8_t link_assist_setting;

// ---- link.cpp から ----
void lassist_loop();                                  // link_loop() から毎回（チャンネル調査の間は呼ばれない）
// 受けたビーコン（印 0x58。形と CRC は確かめ済み）。どの役でも渡してよい（使うかはこちらで決める）
void lassist_on_beacon(const uint8_t* p, int16_t rssi);
// 受信機（ボート）が受けた自グループのテレメトリ（p は電波の 32 バイト、st は link_w2_decode() を通したあとの状態）
void lassist_on_telem(const uint8_t* p, const LinkW2Rx* st, int16_t rssi);
// 送信機（機体）: テレメトリの status に LINK_ST_TIME_WANTED を立てるか
bool lassist_time_wanted();
// 送信機（機体）: テレメトリの status に添える自分の時刻の確からしさ（LINK_ST_TIME_COARSE / LINK_ST_TIME_BAD。よければ 0）
uint16_t lassist_time_status_bits();
// 送信機（機体）: 送信の合間も E220 を mode 0 にしておくか（窓の間だけ。提供ビーコンを聞くため）
bool lassist_keep_awake();

#endif // LINK_ASSIST_H
