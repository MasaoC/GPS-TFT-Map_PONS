// ============================================================
#pragma once
// File    : settings.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GPS/TFT種別選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/11
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// リリース時
#define RELEASE

#define BUILDDATE 20260323
#define BUILDVERSION "0.912"
#define VERSION_TEXT "Version 6"




//----------GPS---------
//#define DEBUG_NMEA
//#define QUECTEL_GPS
//#define MEADIATEK_GPS
#define UBLOX_GPS

//GPSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GPS_SIM_BIWAKO         //琵琶湖固定座標
  //#define DEBUG_GPS_SIM_SAPPORO         //札幌固定座標
  //#define DEBUG_GPS_SIM_SHISHI         //しし固定座標
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換


//---------TFT----------
// TFTを選択
  #define TFT_USE_ST7789    //Tested well. Change User Setting at TFT_eSPI
  //#define TFT_USE_ILI9341   //Tested well.  Change User Setting at TFT_eSPI
  //#define TFT_USE_ST7735      //Not supported anymore.  Screen size too small.

#define VERTICAL_FLIP

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 1050


#define MAX_TRACK_CORDS 500



// =====Hardware Settings =====

// Hardware Ver6
#define SW_PUSH 35  //30(v6 proto)
#define BATTERY_PIN 40 //A0
#define TFT_BL  -1
#define GPS_SERIAL Serial1
#define GPS_TX 0
#define GPS_RX 1
#define USB_DETECT 31
#define SD_CS_PIN -1
#define RP_CLK_GPIO 2 // Set to CLK GPIO
#define RP_CMD_GPIO 3 // Set to CMD GPIO
#define RP_DAT0_GPIO 4 // Set to DAT0 GPIO. DAT1..3 must be consecutively connected. DAT1=5, DAT2=6, DAT3=7
#define SD_CS_SPI_PIN 7 // SPI フォールバック時の CS ピン（DAT3 = GPIO7）
#define SD_DETECT 8

#define BATTERY_MULTIPLYER(adr) (0.00238423334*adr) //VSYS 1/4098*3.3*(151/51)=0.00238423334
#define BAT_LOW_VOLTAGE 3.5
#define BAT_ZERO_VOLTAGE 3.4
#define PIN_PWMTONE 38
#define PIN_AMP_SD 39 //アンプシャットダウン(HIGHでON)
#define USERLED_PIN 34 //ユーザーLED（エラー表示用。エラー時 HIGH）
#define SIN_VOLUME 0.15f // WAVファイルの音量と合わせるために、Sin wave音の音量は下げられていますが、ここで調整可能です。0〜1.0f

// =====追加設定項目====
// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// HDOP 不確かさ円 設定
// GPS fix があっても HDOP が悪い場合、飛行機マークの代わりに青い不確かさ円を表示する。
#define HDOP_THRESHOLD         5.0f  // この値以上で不確かさ円モードに切替え（HDOP 5 = Moderate/Poor 境界）
#define GPS_BASE_ACCURACY_M    5.0f  // HDOP=1 時の基準精度 [m]（95% 信頼円相当 ≈ 2σ）
#define HDOP_MIN_CIRCLE_RADIUS 2     // 輪郭円を描画する最小半径 [px]（未満はテキスト表示に切替え）
#define HDOP_CENTER_DOT_RADIUS 3     // 中心位置を示す塗りつぶし小円の半径 [px]



//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GPSDETAIL 3
#define MODE_MAPLIST 4
#define MODE_SDDETAIL 5
#define MODE_VARIODETAIL 6


#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GPS)
    #warning NOT RELEASE!
  #endif
#endif


//デバッグ用 print マクロ
#define PRINTREVERSEDATE_NUM 10
#ifndef RELEASE
  #define DEBUG_P(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt);
  #define DEBUG_PN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt,num);
  #define DEBUGW_P(date,txt)  Serial.print(txt);
  #define DEBUG_PLN(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt);
  #define DEBUG_PNLN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt,num);
  #define DEBUGW_PLN(date,txt)  Serial.println(txt);
#else
  #define DEBUG_P(date,txt)  
  #define DEBUG_PN(date,txt,num)  
  #define DEBUGW_P(date,txt)  
  #define DEBUG_PLN(date,txt)
  #define DEBUG_PNLN(date,txt,num)   
  #define DEBUGW_PLN(date,txt)  
#endif

// ============================================================
// スタック残量計測マクロ
//
// Core0: リンカシンボル __StackLimit（スタック下限）と現在 SP の差 = 残りバイト数
// Core1: core1_separate_stack=true のため別ヒープから確保。
//        setup1() 先頭でキャプチャした _core1_base_sp との差 = 使用済みバイト数
//
// ※ 5 秒に 1 回だけ出力するのでシリアルが流れすぎない
// ============================================================
extern volatile uint32_t _core1_base_sp;  // GPS_TFT_map.ino で定義


#ifndef RELEASE
  #define DEBUG_STACK_C0(label) do { \
    static unsigned long _t0_; \
    if (millis() - _t0_ >= 5000) { \
      _t0_ = millis(); \
      Serial.print("[C0 stack free:" label "]="); \
      Serial.println((int)rp2040.getFreeStack()); \
    } \
  } while(0)

  #define DEBUG_STACK_C1(label) do { \
    static unsigned long _t1_; \
    if (millis() - _t1_ >= 5000) { \
      _t1_ = millis(); \
      uint32_t _sp_; \
      asm volatile ("mov %0, sp" : "=r" (_sp_)); \
      Serial.print("[C1 stack used:" label "]="); \
      Serial.println((int)(_core1_base_sp > _sp_ ? _core1_base_sp - _sp_ : 0)); \
    } \
  } while(0)

  // ---- 処理時間計測ユーティリティ（BNO085配置コア決定用） ----
  // 使い方: TIMING_START(名前) → 重い処理 → TIMING_END(統計変数, 名前)
  // 30秒ごとに TIMING_REPORT で max/avg/count を Serial 出力する。
  // 閾値: BNO085 H_INT 応答リミット = 10ms = 10000 us
  struct TimingStat {
    uint32_t max_us;   // ワーストケース（マイクロ秒）
    uint32_t sum_us;   // 累計（平均計算用。オーバーフロー対策として count も管理）
    uint32_t count;    // 呼び出し回数
    const char* label; // 識別ラベル文字列
  };
  #define TSTAT_INIT(label_str) { 0, 0, 0, (label_str) }

  // TIMING_START: ローカル変数にタイムスタンプを記録する
  #define TIMING_START(var)  uint32_t _ts_##var = time_us_32()

  // TIMING_END: 経過時間を計算して統計変数を更新する
  #define TIMING_END(stat, var) do { \
    uint32_t _te_ = time_us_32() - _ts_##var; \
    if (_te_ > (stat).max_us) (stat).max_us = _te_; \
    (stat).sum_us += _te_; \
    (stat).count++; \
  } while(0)

  // TIMING_REPORT: 統計を Serial に出力する（count > 0 の時のみ）
  #define TIMING_REPORT(stat) do { \
    if ((stat).count > 0) { \
      Serial.print("[TIME] "); Serial.print((stat).label); \
      Serial.print(" max="); Serial.print((stat).max_us); \
      Serial.print("us avg="); Serial.print((stat).sum_us / (stat).count); \
      Serial.print("us n="); Serial.println((stat).count); \
    } \
  } while(0)

#else
  #define DEBUG_STACK_C0(label)
  #define DEBUG_STACK_C1(label)
  // RELEASE ビルド: 計測マクロはすべて空にしてオーバーヘッドゼロにする
  #define TIMING_START(var)
  #define TIMING_END(stat, var)
  #define TIMING_REPORT(stat)
#endif

// ============================================================
// BNO085 IMU (GY-BNO080) ハードウェア設定
// ============================================================
// MS5611 と I2C バスを共用する（i2c0, GPIO32=SDA, GPIO33=SCL, 100kHz）。
// Core0 で imu_update() を実行する。
// H_INT を使わない場合は -1 のまま（ポーリングモード, 15ms 間隔, バリオ用途で十分）。
// 接続する場合はピン番号を設定（割り込みドリブンモード, 応答遅延 <1ms）。
#define IMU_INT_PIN   -1   // BNO085 H_INT ピン（未使用: -1 → ポーリング）
#define IMU_RST_PIN   46   // BNO085 NRST ピン（GPIO46）
#define IMU_I2C_ADDR  0x4B // GY-BNO080 の I2C アドレス（PS1=HIGH → 0x4B）

// ============================================================
// Kalman フィルター チューニングパラメーター
// ============================================================
//
// VSI = x[1] (速度) は以下の2ステップで更新される:
//
//   ① predict (50Hz, IMU 加速度):
//       x[1] += (a_k - x[2]) * dt
//       P[1][1] += KF_Q_VEL    ← 速度の不確かさを毎ステップ KF_Q_VEL だけ増やす
//
//   ② update (50Hz, 気圧高度):
//       K[1] = P[1][0] / (P[0][0] + KF_R)
//       x[1] += K[1] * (z_baro - x[0])   ← 気圧ノイズが VSI に乗る経路
//
// 静止時に VSI が揺れる → K[1] が大きすぎる → 対策:
//   ・KF_R を大きく   : K[1] の分母が増え、気圧ノイズの影響が減る
//   ・KF_Q_VEL を小さく: P[1][0] が小さく保たれ K[1] が減る
//
// トレードオフ: 値を大きく/小さくするほど静止ノイズは減るが、
//              上昇・下降への追従がわずかに遅くなる（目安 0.5〜2 秒）。
//
// KF_Q_VEL  : 速度プロセスノイズ
//   大きい → P[1][1] の成長が速く K[1] が大きくなる → 気圧変化に速く追従するが揺れやすい
//   小さい → 速度状態が変化しにくく静止ノイズが減る
//
// KF_Q_BIAS : バイアスプロセスノイズ
//   大きい → 加速度バイアスが速く変化することを許容する
//   通常は小さい値のまま変える必要はない
//
// KF_R      : 気圧高度観測ノイズ [m²]
//   大きい → K[1] が小さくなり気圧ノイズの影響が減る（IMU 主体）
//   小さい → 気圧を強く信頼するため気圧ノイズが VSI に直接乗る

#define KF_Q_VEL    0.05f   // 速度プロセスノイズ  (旧 0.5 → 1/10。静止ノイズ抑制)
#define KF_Q_BIAS   0.005f  // バイアスプロセスノイズ（変更なし）
#define KF_R        4.0f    // 気圧高度観測ノイズ [m²] (旧 0.5 → 8倍。気圧ノイズを薄める)