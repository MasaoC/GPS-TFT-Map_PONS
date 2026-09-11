// ============================================================
// File    : e220.h
// Project : PONS v7 — PONS Link
// Role    : E220-900T22S(JP)（920MHz LoRa）の下位ドライバ。
//           UART の叩き方・モード制御・レジスタ設定だけを持ち、
//           テレメトリの意味は一切知らない（それは link.cpp の仕事）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/11
// ============================================================
//
// ■ 配線（docs/pons_link.md §2）
//     RP2350 GPIO8  (UART1 TX) → E220 RXD
//     RP2350 GPIO9  (UART1 RX) ← E220 TXD
//     RP2350 GPIO30            ← E220 AUX（状態通知）
//     RP2350 GPIO35 or 41      → E220 M0+M1（結線して 1 本で駆動）
//
//   M0/M1 に使う GPIO は BNO085 のバス構成と排他共用なので、
//   settings.h の IMU_BUS_SPI に追従させてある（下の E220_MODE_PIN）。
//   基板の 0Ω（R50/R52）も同じ組み合わせで実装すること。
//
// ■ モードは 2 つしか使わない
//     mode 0 (LOW)  : 通常送受信。受信待機 8.2mA
//     mode 3 (HIGH) : Config/DeepSleep。2.5µA
//   WOR は使わないので M0/M1 を分ける必要が無い（データシート 5.3）。
//
// ■ 送信機は寝かせる、受信機は寝かせない
//   受信機を寝かせるにはスロット同期が要るが、常時受信でも 8.2mA しか
//   食わないので当面やらない。送信機は UART へ流し込んだ直後に
//   mode 3 へ落とすだけでよい（高速スリープ）。→ docs/pons_link.md §3
//
#ifndef E220_H
#define E220_H

#include <Arduino.h>
#include <stdint.h>
#include "settings.h"

// ---- ピン ----
#define E220_AUX_PIN     30      // 状態通知（Low = 処理中 / High = アイドル）
#define E220_TX_PIN       8      // RP2350 → E220 RXD
#define E220_RX_PIN       9      // RP2350 ← E220 TXD

// M0+M1。BNO085 が SPI なら I2C1_SCL(35) が余り、I2C なら SPI1_CS(41) が余る。
#ifdef IMU_BUS_SPI
  #define E220_MODE_PIN  35
#else
  #define E220_MODE_PIN  41
#endif

// ---- ラジオ設定 ----
// radio_profile → 拡散率。SF を上げるほど 2.5dB ずつ強くなるが、
// 送信時間（＝送信電流）が倍々に増える。既定は SF8。
#define E220_SF_MIN       7
#define E220_SF_MAX      11
#define E220_BW_KHZ     500      // CH0-12 を使う前提で固定（docs/pons_link.md §2）
#define E220_CH_MAX      12      // 923.2MHz。ここまでが休止 50ms 固定の帯域

uint8_t e220_profile_to_sf(uint8_t profile);   // 0→SF7 … 4→SF11

// ---- ライフサイクル ----
// UART を開き、モジュールへ設定を書き、読み返して照合する。
// 失敗しても戻ってくる（e220_alive() が false になるだけ）。
void e220_setup(uint8_t ch, uint8_t profile);

// 設定画面からチャンネル/プロファイルが変わったときに呼ぶ。
// mode 3 へ入って書き直し、mode 0 へ戻す。
//
// ★ persist で書込みコマンドを選ぶ。
//     true  … 0xC0（不揮発）。**書換え寿命がある**ので起動時の 1 回だけ。
//     false … 0xC2（RAM のみ）。設定画面で値を回している間はこちら。
//   設定の正本は SD の settings.txt なので、RAM に書くだけでも
//   次回起動時に同じ値が復元される。CH を 0→12 と回すだけで 13 回
//   フラッシュに書くのは、モジュールの寿命を無駄に削るだけになる。
bool e220_reconfigure(uint8_t ch, uint8_t profile, bool persist);

// モジュールが生きているか。次の 2 つを **両方** 満たすときだけ true。
//   ・起動時（または設定変更時）の書き込み→読み返しの照合が通っている
//   ・直近の問い合わせに返事がある（連続して無応答なら false へ落ちる）
// ★ 後者が要る。照合は e220_setup() / e220_reconfigure() でしか行われないので、
//   それだけだと **飛行中にコネクタが抜けても true のまま**になり、
//   送信機は「送れているつもり」で飛び続ける（上りが無いので気づけない）。
//   返事の有無は e220_read_noise() と e220_recv() が内部で数えている。
//   送信機は寝ている時間が長く自然な機会が無いので、link.cpp が定期的に叩く。
bool e220_alive();

// ---- 送受信 ----
// n バイトを 1 パケットとしてブロードキャスト送信する。
//   ★ write() はハード FIFO が空くまでスピンする（arduino-pico 4.5.1 で確認済。
//     送信側にソフトバッファは無く、setFIFOSize() が効くのは受信側だけ）。
//     115200bps で 68 バイトなら約 3.1ms ブロックする。ブロックするおかげで
//     バイト列に隙間ができず、E220 にパケットを分割されない（§4）。
bool e220_send(const uint8_t* payload, uint8_t n);

// 受信バイトを取り込み、完全な 1 パケットが揃ったら true。
// 毎ループ呼ぶこと。frame_len バイトのペイロードと RSSI 1 バイトを想定する。
bool e220_recv(uint8_t* out, uint8_t frame_len, int16_t* rssi_dbm);

// ---- 省電力 ----
void e220_sleep();     // mode 3 へ。送信中なら送信完了後に入る
void e220_wake();      // mode 0 へ。復帰は 1ms（データシート 5.3）

// ---- 診断 ----
// 環境ノイズ [dBm]。距離を決めているのはこれ（docs/pons_link.md §8）。
// mode 0 で呼ぶこと（mode 3 では常に false）。
// ★ **生存確認を兼ねる。** モジュールへの唯一の往復なので、返事の有無が
//   そのまま e220_alive() に効く。値が要らない場面で叩いてもよい。
//   応答待ちは生きていれば数 ms。無応答が続く間は待ちを 30ms へ詰めるので、
//   繰り返し呼んでも Core0 を長く止めない。
bool e220_read_noise(int16_t* noise_dbm, int16_t* last_rssi_dbm);

#endif // E220_H
