// ============================================================
// File    : gnss_dbd.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS の航法データベース（UBX-MGA-DBD）を SD に写しておき、時計の電池（BT1）が切れていた起動で
//           GNSS へ戻す（2026-11-09〜）。外から来た時刻（PC。のちに LoRa）を GNSS へ入れる口もここ（2026-11-10〜）。
//           v7 と v7.1 で同じ中身。仕組みは gnss_dbd.cpp の冒頭。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/10
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
// USB シリアルの "PONS:DBD,<arg>"（机の上の試験。SAVE / COLD / COLDONLY / HOT / STATUS、COLD,TIME / COLDONLY,TIME）
void gnss_dbd_command(const char* arg);

// ---- 外から GNSS へ時刻を入れる（2026-11-10〜。docs/pons_ttff_assist_plan.md §3.6）----
// BT1 が切れて GNSS が時刻を知らない間に、ほかから来た時刻を UBX-MGA-INI-TIME_UTC で入れる。戻した DBD が
// 「時刻が無いので使えない」と断られずに済み（断られていた分はすぐ入れ直す）、予測軌道（ANA）もすぐ使える。
// 入れるのは GNSS が時刻を知らないときだけ（知っていれば GNSS の時刻のほうが正確）。判断はぜんぶ gnss_dbd.cpp の中。
// ★ **時刻の出どころを足すときも、ここへ渡すだけにする**（GNSS へ書く・いつ入れるか・入れ直しの判断を呼び出し側に書かない）。
//   いまの入口は USB シリアルの PONS:TIME（PC。Flight Viewer が接続のたびに送る）。LoRa で仲間からもらう時刻
//   （計画書 §4）も、受けた側の確かめ（§4.6 の 1.）を済ませてから GNSS_TIME_SRC_LORA で渡す。
enum GnssTimeSrc : uint8_t {
  GNSS_TIME_SRC_PC   = 0,   // USB シリアルの PONS:TIME
  GNSS_TIME_SRC_LORA = 1,   // 仲間の機体・ボート（link_assist.cpp。2026-11-13〜）
  GNSS_TIME_SRC_SELF = 2,   // 机の上の試験（COLD,TIME。リセット前に本体が覚えた時刻を、外から来た時刻の代わりに入れる）
};
// utc_ms: UTC（1970-01-01 からのミリ秒。うるう秒は数えない＝UNIX 時刻。JavaScript の Date.now() と同じ）
// at_ms : その時刻だった millis()（受けた瞬間。無線なら電波に乗っていた時間を引いた値）
// 返り値（文字列リテラル。PONS:TIME の返事にそのまま使う）:
//   "SENT"        GNSS へ入れた（結果は log.txt の "GNSS TIME (pc): ... -> accepted|refused|no answer"）
//   "LATER"       まだ入れられない（起動直後で時計の生死が分からない・GNSS がつながっていない など）。
//                 覚えておいて、入れられるようになったら入れる
//   "NOT_NEEDED"  GNSS は時刻を知っている
//   "ALREADY"     この GNSS には入れて受け取られた（NAV-PVT に時刻が出るのを待っている）
//   "GAVE_UP"     GNSS_TIME_MAX_TRIES 回入れて受け取られなかった（GNSS がリセットするまで入れない）
//   "BAD_TIME"    ありえない時刻（ビルドした日より前・2100 年以降・SD の DBD を吸い出した時刻より前）
//   "REPLAY"      リプレイ中・抜けた直後
//   "TEST"        机の上の試験（COLD / COLDONLY / HOT）の最中。比べる素の値を汚さない
//   "DISABLED"    GNSS_DBD_ENABLE が 0
// acc_ms: GNSS へ伝える精度 [ms]。0 なら出どころの既定（GNSS_TIME_ACC_PC_MS / GNSS_TIME_ACC_LORA_MS）。
//         LoRa で fix の前の時刻（送り主の GNSS の時計＝RTC）を渡すとき、送り主の精度に合わせて広げる（2026-11-13〜）
const char* gnss_dbd_give_time(uint64_t utc_ms, uint32_t at_ms, GnssTimeSrc src, uint32_t acc_ms = 0);
// USB シリアルの "PONS:TIME,<UNIX 時刻 ms>"。rx_ms は行を受けた millis()。返事は "PONS:TIME,OK,SENT" など
void gnss_dbd_time_command(const char* arg, uint32_t rx_ms);
// いま外から時刻をもらう値打ちがあるか（2026-11-13〜。LoRa の助け合い（link_assist.cpp）が、聞く窓・要求を
// 開けておくかに使う）。判断は gnss_dbd_give_time() と同じ time_block() から出すので、食い違わない。
//   GNSS_TIME_WANT_UNKNOWN  まだ分からない（起動後最初の NAV-PVT の前。リプレイ中・抜けた直後）
//   GNSS_TIME_WANT_YES      ほしい（GNSS が時刻を知らず、入れて受け取られてもいない）
//   GNSS_TIME_WANT_NO       要らない（GNSS が知っている・入れて受け取られた・上限まで入れた・机の上の試験中・無効）
enum GnssTimeWant : uint8_t { GNSS_TIME_WANT_UNKNOWN = 0, GNSS_TIME_WANT_YES, GNSS_TIME_WANT_NO };
GnssTimeWant gnss_dbd_time_want();

// ---- Core1（mysd.cpp の gnss_dbd_save_sd / gnss_dbd_load_sd から）----
uint8_t* gnss_dbd_buffer_core1();                          // RAM（GNSS_DBD_BUF_BYTES）。Core0 が手放している間だけ触る
bool gnss_dbd_head_for_save_core1(GnssDbdHead* h);        // 書くよう頼まれていれば頭（CRC と本体 ID はまだ）を返す
void gnss_dbd_saved_core1(bool ok, const char* why, uint32_t ms);   // 書き終えた（why は文字列リテラル。ms は書くのにかかった時間）
void gnss_dbd_loaded_core1(bool ok, const GnssDbdHead* h, const char* why);   // 読み終えた（ok なら RAM にフレーム）
// フレームの並びが壊れていないか（各フレームの印・長さ・Fletcher のチェックサムと、数・合計の大きさ）
bool gnss_dbd_frames_ok(const uint8_t* b, uint32_t bytes, uint16_t count);

#endif // GNSS_DBD_H
