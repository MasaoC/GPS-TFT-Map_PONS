// ============================================================
// File    : power.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告、過放電（3.0V 未満）の本体フラッシュ記録と次回起動ロック。
//           v7.1 では「SW を OFF にしたまま USB で充電している」の検出と読み上げも。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/29
// ============================================================
#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

// 本体フラッシュ（EEPROM 領域）に残す過放電ラッチの記録（32 バイト固定）。
// ★ PONSv7（BNO085 搭載）の ImuCalRec がオフセット 0..11 を使っているため、
//   cherry-pick しても衝突しないよう BATT_LATCH_EE_ADDR（64）に置く。
// ★ PONSv7 への cherry-pick 注意:
//   v7 では Core0 の ImuCalRec（オフセット 0..11）と Core1 の過放電ラッチ（オフセット 64..95）が
//   同一 EEPROM（EEPROM.commit()）を共有する。両方のコアが同時に commit() すると
//   arduino-pico の idleOtherCore 機構によってデッドロック（フリーズ）する恐れがある。
//   v7 へ取り込む際は commit() をどちらか単一のコア（Core1 推奨）に集約するか mutex で保護すること。
#define BATT_LATCH_FLAG_MOVING    0x01  // 検知時に移動中（フライト中）だった
#define BATT_LATCH_FLAG_JST_VALID 0x02  // date_yyyymmdd / time_hhmmss が有効

// ---- 過放電警告トーン・音声の共通定義 ----
#define BATT_OVERDISCHARGE_TONE_FREQ      2637
#define BATT_OVERDISCHARGE_TONE_DUR_MS    150
#define BATT_OVERDISCHARGE_TONE_COUNT     4
#define BATT_OVERDISCHARGE_TONE_PRIORITY  4
#define BATT_OVERDISCHARGE_WAV_FILE       "wav/batt_overdischarge.wav"

struct BattLatchRecord {
  uint32_t magic;          // BATT_LATCH_EE_MAGIC ('PBAT')
  uint8_t  version;        // BATT_LATCH_EE_VERSION (= 1)
  uint8_t  latched;        // 1 = 過放電ラッチ中、0 = 正常（解除済み含む）
  uint8_t  flags;          // BATT_LATCH_FLAG_*
  uint8_t  reserved;
  uint16_t min_volt_mv;    // ラッチ確定時の電池電圧 [mV]（例: 2970 = 2.97V）
  uint16_t event_count;    // 累計ラッチ発生回数（解除しても維持）
  uint16_t reset_count;    // SD 裏コマンドによる累計解除回数
  uint16_t reserved2;
  uint32_t date_yyyymmdd;  // 発生日 JST（例: 20261029。未測位なら 0）
  uint32_t time_hhmmss;    // 発生時刻 JST（例: 142305。未測位なら 0）
  uint32_t uptime_sec;     // 発生時の起動からの経過秒数（millis() / 1000）
  uint16_t reserved3;
  uint16_t crc16;          // 先頭 30 バイトの CRC-16-CCITT
};
static_assert(sizeof(BattLatchRecord) == 32, "BattLatchRecord must be 32 bytes");

void power_setup();   // setup() で analogReadResolution(12) の後に呼ぶ
void power_tick();    // loop() の最上位で毎回呼ぶ（内部で間引く）

// ---- 過放電（3.0V 未満）ラッチの Core1 専用操作 ----
// ★ EEPROM の読み書きと commit() は Core1 だけで行う。SDIO 転送との衝突を避け、
//   かつ SD カードが無い・壊れている機体でも確実に本体フラッシュへ記録するため。
void power_latch_init_core1();     // setup1() の mutex_init 直後に 1 度だけ呼ぶ
void power_latch_commit_core1();   // TASK_LATCH_BATT_OVERDISCHARGE で Core1 から呼ぶ
// setup_sd() で裏コマンド（batt_unlock.txt）が見つかったときに Core1 から呼ぶ。
// 戻り値: ラッチ中（latched==1）から解除したら true、元から未ラッチなら false。
bool power_latch_unlock_core1(uint16_t* out_events, uint16_t* out_resets, uint16_t* out_min_mv);

// ---- 過放電ラッチの参照・警告／ロック制御（Core0 から呼ぶ）----
bool  power_latch_ready();                  // Core1 の初期フラッシュ読込が終わったか
bool  power_overdischarge_latched();        // 過去または今回の起動でラッチが立っているか
bool  power_overdischarge_boot_latched();   // 前回までの起動でラッチ済みか（setup() の警告画面判定用）
// ラッチ中に外部 USB から給電されているか（VBUS >= 4.3V / SW OFF+USB / 垂れた弱い USB 給電）。
// ★ s_fault 確定後も実際の VBUS 給電があれば true を返す（接触不良後の 5V 給電でロックが外れない）。
// true のあいだ警告画面はボタンで抜けられない完全ロックになり、false（電池単体）ならボタンで進める。
bool  power_overdischarge_usb_active();
// 通常動作中（loop()）に USB が挿されて完全ロック画面へ入るべきか。
// ★ フライト中（電池単体）は常に false。電池単体で起動警告を抜けた後や、同一セッションで 3.0V を割った後に
//   USB を挿された場合、BATT_LATCH_USB_HOLD_MS 継続で true になる。
bool  power_overdischarge_usb_lock_due();
void  power_overdischarge_screen_enter();   // 全画面警告／ロックに入ったときに呼ぶ（警告トーン＋音声の開始）
void  power_overdischarge_screen_exit();    // 全画面警告／ロックを抜けるときに呼ぶ
bool  power_take_overdischarge_cleared();   // SD 裏コマンドで解除された通知を 1 度だけ受け取る
void  power_get_overdischarge_record(BattLatchRecord* out);
// 警告画面の診断表示用（生の VBUS 電圧 [V]）。★ USB の有無判定には使わないこと（下の usb_power_present を使う）。
float power_get_vbus_voltage();

// 本物の USB から給電されているか。
// ★ USB の有無はこれだけで判定する。VBUS の ADC（USB_VBUS_ADC_PIN）を直接読まないこと
//   （2026-10-15、壊れた充電 IC が VBUS を持ち上げて「USB」と誤判定し、
//   電池低下警告まで止めていた）。
bool usb_power_present();

// 電源経路の異常（再起動まで消えない）。
bool power_fault_active();

// v7.1: SW を OFF にしたまま USB で動いているか（電池は Q3 で切り離されている）。
// 判定は「GPIO40 が 0V に見える」だけ（SW OFF なら電源は USB しかないので、USB の判定は使わない）。
// ★ このあいだ GPIO40 は SW で GND に落ちているので、**電池電圧は読めない**
//   （get_input_voltage() は 0V へ向かって下がる）。電池電圧を表示・送信・記録・警告する
//   箇所はこれを見て「不明」として扱うこと。0V を本物として扱うと、残量 0% 表示・
//   電池低下の誤報（自機と受信機）・CSV の 0.00V になる。
bool power_sw_off_on_usb();

// 「SW を入れて充電して」の注意を地図に出すか。
// 移動中・ミラー中・リプレイ中・デモ中・電源経路の異常・高温の充電のあいだは false。
bool power_sw_off_notice_active();

// USB がつながっている間に本体基板（MS5611）が CHARGE_HOT_C を超え続けている（電池の仕様は
// 充電 0〜45°C）。地図の注意と読み上げは電源経路の異常と同じく、ミラー中・リプレイ中も出す。
bool  power_charge_hot_active();
float power_charge_hot_temp();   // 判定に使っている温度 [°C]

// ---- USB 充電中の残量(%)・残り時間推定（docs/pons_charge_estimation.md）----
// 完全ステートレス物理モデル:
// BATPWR の測定ノード（F1 と Q3/Q4 の中間点）の物理特性に基づき、真の開放電圧と残量を推定する。
// ★ タイマーや状態変数は一切使わず、瞬時電圧から純粋関数で計算するため、途中挿しや再起動でも即座に正しい値を返す。
float power_estimated_cell_voltage();    // 推定セル開放電圧 [V] (CC: V-75mV / CV: 2V-4.20V)
int   power_charging_percent();          // 充電中残量 [%] (0〜100)
int   power_charging_remaining_min();    // 満充電までの推定残り時間 [分] (0=FULL)

// ログ用の短い文字列（例: "usb=1" / "usb=1 vbus=4.95V"）。
void power_format_status(char* buf, size_t n);

#endif
