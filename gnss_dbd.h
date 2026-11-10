// ============================================================
// File    : gnss_dbd.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS の航法データベース（UBX-MGA-DBD）を SD に写しておき、時計の電池（BT1）が切れていた起動で
//           GNSS へ戻す（2026-11-09〜）。v7 と v7.1 で同じ中身。仕組みは gnss_dbd.cpp の冒頭。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/09
// ============================================================
#ifndef GNSS_DBD_H
#define GNSS_DBD_H

#include <Arduino.h>

#define GNSS_DBD_MAGIC "PONSDBD1"   // ファイルの先頭 8 バイト（形を変えたら数字を上げる）

// SD のファイル（GNSS_DBD_FILE）の頭。64 バイト。後ろに UBX-MGA-DBD のフレーム（B5 62 13 80 … CK_A CK_B）を
// 受けた順にそのまま並べる。戻すときは先頭から 1 通ずつ GNSS へ書くだけで済む。
struct __attribute__((packed)) GnssDbdHead {
  char     magic[8];      // GNSS_DBD_MAGIC
  uint32_t utc;           // 吸い出した時刻（UTC、1970-01-01 からの秒）
  uint16_t count;         // フレームの数（吸い出しの最後の MGA-ACK が言った数と合っていたもの）
  uint16_t reserved0;
  uint32_t bytes;         // フレームの合計の大きさ [B]
  uint8_t  gnss_id[6];    // GNSS チップの固有 ID（UBX-SEC-UNIQID）。違う GNSS には戻さない
  uint8_t  reserved1[2];
  char     board_id[24];  // 本体の固有 ID（16 進の文字列。rtc_backup.txt と同じ）。違う本体の SD なら読まない
  uint8_t  reserved2[8];
  uint16_t data_crc;      // フレーム全体の CRC16（link_crc16）
  uint16_t head_crc;      // ここより前の 62 バイトの CRC16
};
static_assert(sizeof(GnssDbdHead) == 64, "GnssDbdHead は 64 バイト");

// ---- Core0（gnss.cpp・.ino から）----
void gnss_dbd_tick();        // gnss_loop() から毎回。送るのも少しずつで、待たない
void gnss_dbd_on_ubx(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len);  // 受けた UBX をすべて渡す（要るものだけ拾う）
void gnss_dbd_on_setup();    // gnss_setup_impl() の頭。吸い出し・戻しの途中なら打ち切る（ポートを開き直すため）
void gnss_dbd_after_setup(); // gnss_setup_impl() で設定を送り終えたあと。GNSS の固有 ID と版をポールする
// ★ GNSS の UART へ書く前に必ず呼ぶ。DBD のフレームを書きかけていたら書き切る（最大約 45ms。待つ間も IMU を引き取る）。
//   途中に別のコマンドが割り込むと、両方とも GNSS に捨てられる。gnss.cpp の ubx_send_packet() と
//   NAV-SAT のレート切替、.ino のデバッグ中継は呼んである。
void gnss_dbd_finish_tx();
// 起動後最初の fix のログ（"GNSS FIXED ... after boot"）に添える文字列。例 "rtc=lost dbd=40/52 resend=52/52 time=12.3s"
void gnss_dbd_fix_tag(char* buf, size_t n);
// USB シリアルの "PONS:DBD,<arg>"（机の上の試験。SAVE / COLD / COLDONLY / HOT / STATUS）
void gnss_dbd_command(const char* arg);

// ---- Core1（mysd.cpp の gnss_dbd_save_sd / gnss_dbd_load_sd から）----
uint8_t* gnss_dbd_buffer_core1();                          // RAM（GNSS_DBD_BUF_BYTES）。Core0 が手放している間だけ触る
bool gnss_dbd_head_for_save_core1(GnssDbdHead* h);        // 書くよう頼まれていれば頭（CRC と本体 ID はまだ）を返す
void gnss_dbd_saved_core1(bool ok, const char* why, uint32_t ms);   // 書き終えた（why は文字列リテラル。ms は書くのにかかった時間）
void gnss_dbd_loaded_core1(bool ok, const GnssDbdHead* h, const char* why);   // 読み終えた（ok なら RAM にフレーム）
// フレームの並びが壊れていないか（各フレームの印・長さ・Fletcher のチェックサムと、数・合計の大きさ）
bool gnss_dbd_frames_ok(const uint8_t* b, uint32_t bytes, uint16_t count);

#endif // GNSS_DBD_H
