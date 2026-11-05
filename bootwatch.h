// ============================================================
// File    : bootwatch.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 起動の見張り。起動の途中で固まったらチップを再起動し、止まった段階を log.txt に残す。
//           あわせて、水晶の立ち上がりを待ってから 150MHz に上げる（BOOTWATCH_XOSC_SETTLE_MS）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/06
// ============================================================
//
// 水晶の立ち上がり待ちが要る理由は docs/hardware.md §2「起動時に固まる件」。
//
// ■ 何をするか（bootwatch.cpp）
//   - pico-sdk の起動処理の最初（main() より前）で watchdog（BOOTWATCH_WDT_MS）を掛け、
//     "BOOT PONS" の後（bootwatch_boot_done_core0）で外す。その間に固まれば自分で再起動し、
//     次の起動で "BOOT WATCH: restarted by WATCHDOG timeout" と止まった段階を出す。
//   - 段階は Core0・Core1 とも目印を置く（下の BootWatchStage）。main() より前の段階（1〜8・40〜46）は
//     pico-sdk の runtime_init_clocks() を写しに置き換えて入れている。
//   - Core1 は setup1() で Core0 の tft.begin() を待ち、待ちが切れたら Core0 の段階・割り込みの生死・
//     HardFault の PC を書いて再起動する（BOOTWATCH_AUTO_RESTART）。
//   - SW2 で起こした起動では "BOOT WATCH: SW2 reset ..." を 1 行出す（前回が起動処理の途中で
//     止まっていたか。SW2 では SRAM も watchdog の記録も消えるので、残る 1 bit だけで判断する）。
//   - 固まらなかった起動でも、PLL のロックが外れていれば "BOOT WATCH: PLL lost lock" を出す。
#pragma once
#include <Arduino.h>

// 段階（ログに番号と名前が出る）。6 は欠番（以前の「各クロックの設定」。40〜46 に分けた）。
enum BootWatchStage : uint8_t {
  BW_STAGE_NONE        = 0,
  // Core0、main() より前（pico-sdk の起動処理）
  BW_STAGE_RT_START    = 1,    // 起動処理の最初（クロックより前）
  BW_STAGE_CLK_XOSC    = 2,    // 水晶の発振が安定するのを待っている（xosc_init と立ち上がり待ち）
  BW_STAGE_CLK_SWITCH  = 3,    // clk_sys / clk_ref を元のクロックへ切り替えている
  BW_STAGE_CLK_PLL_SYS = 4,    // PLL_SYS のロック待ち
  BW_STAGE_CLK_PLL_USB = 5,    // PLL_USB のロック待ち
  BW_STAGE_CLK_REF     = 40,   // clk_ref を水晶へ切り替えている
  BW_STAGE_CLK_SYS     = 41,   // clk_sys を PLL_SYS（150MHz）へ切り替えている（ここで CPU が 150MHz になる）
  BW_STAGE_CLK_USB     = 42,   // clk_usb の設定（ここからは 150MHz で動いている）
  BW_STAGE_CLK_ADC     = 43,   // clk_adc の設定
  BW_STAGE_CLK_PERI    = 44,   // clk_peri の設定
  BW_STAGE_CLK_HSTX    = 45,   // clk_hstx の設定
  BW_STAGE_CLK_TICKS   = 46,   // tick の発生器を動かしている
  BW_STAGE_RT_CLOCKS   = 7,    // クロックが済んで、残りの起動処理
  BW_STAGE_RT_DONE     = 8,    // 起動処理の最後（この後 C++ のコンストラクタ、rp2040.begin()）
  // Core0、main() と setup()
  BW_STAGE_MAIN        = 9,    // main() の initVariant()（USB を起こす前）
  BW_STAGE_SETUP       = 10,   // setup() に入った
  BW_STAGE_SERIAL      = 11,   // Serial.begin() の後
  BW_STAGE_PINS        = 12,   // ユーザー LED・ADC・SD 検出ピンの設定の後
  BW_STAGE_POWER       = 13,   // power_setup()（ADC で VBUS・電池を読む）の後
  BW_STAGE_BACKLIGHT   = 14,   // backlight_setup() の後
  BW_STAGE_TFT         = 15,   // setup_tft()（tft.begin()）の後
  // Core1（setup1）
  BW_STAGE1_SETUP1     = 20,   // setup1() に入った
  BW_STAGE1_LATCH      = 21,   // power_latch_init_core1()（EEPROM の読み込み）の後
  BW_STAGE1_DEST       = 22,   // init_destinations() の後
  BW_STAGE1_SOUND      = 23,   // setup_sound() の後
  BW_STAGE1_WAIT       = 24,   // tft_pio_ready を待っている（bootwatch_wait_core0 の中）
  BW_STAGE1_SD         = 25,   // setup_sd() の最中
  BW_STAGE1_SD_DONE    = 26,   // setup_sd() の後
  BW_STAGE1_DONE       = 27,   // setup1() の見張りを抜けた
};

void bootwatch_stage(BootWatchStage stage);    // Core0
void bootwatch_stage1(BootWatchStage stage);   // Core1

// Core1（setup1）。*ready が立つまで wait_ms 待つ（以前の tft_pio_ready の待ちと同じ）。
void bootwatch_wait_core0(volatile bool* ready, uint32_t wait_ms);
// Core1（setup1）。setup_sd() の後に呼ぶ。待ちが切れていたら Core0 が止まったままかを確かめ、
// 止まっていれば log.txt に書いて、設定なら再起動する（戻ってこない）。
void bootwatch_report_core1(volatile bool* ready);

// Core0（setup()）。"BOOT PONS" の後で呼ぶ。watchdog を外し、前回の様子をログに出す。
void bootwatch_boot_done_core0();
