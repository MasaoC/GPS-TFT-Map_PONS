// ============================================================
#pragma once
// File    : settings.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GPS/TFT種別選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/23
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// リリース時
#define RELEASE

#define BUILDDATE 20260326
#define BUILDVERSION "0.915"
#define VERSION_TEXT "Version 6"




//----------GPS---------
//#define DEBUG_GBX_NMEA
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
#define SIN_VOLUME 0.15f  // Sin波の振幅倍率（0〜1.0f）。WAVと音量を合わせるため小さめにしてあるが、バリオが小さいと感じる場合は上げる。0.5fで±254、1.0fで±508（±512ヘッドルーム）。
#define VARIO_VOL_SCALE 3

// =====追加設定項目====
// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// hAcc 不確かさ円 設定
// GPS fix があっても hAcc（NAV-PVT 水平精度推定）が大きい場合、飛行機マークの代わりに青い不確かさ円を表示する。
// gnssFixOK=false の場合は常に不確かさ円を表示する。
#define HACC_THRESHOLD_M       10.0f // この値（m）以上で不確かさ円モードに切替え（旧 HDOP=2×5m 相当）
#define HDOP_MIN_CIRCLE_RADIUS 4     // 輪郭円を描画する最小半径 [px]（未満はテキスト表示に切替え）
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

#define KF_Q_VEL    0.02f   // 速度プロセスノイズ  (旧 0.05 → 1/2.5。VSIふらつきをさらに抑制)
#define KF_Q_BIAS   0.005f  // バイアスプロセスノイズ（変更なし）
#define KF_R       12.0f    // 気圧高度観測ノイズ [m²] (旧 4.0 → 3倍。気圧ノイズのVSI影響を低減)

// ============================================================
// 水平加速度による速度プロセスノイズ動的増幅（imu.cpp kf_predict で使用）
// ============================================================
// 水平加速度が大きいと BNO085 の重力ベクトル推定がずれ、
// 地球座標系の鉛直加速度 a_k に誤差が混入する。
// predict ステップで速度プロセスノイズを下式で増幅することで、
// IMU 加速度への依存を自動的に弱め、気圧・GNSS 観測の重みを上げる。
//
//   q_vel_eff = KF_Q_VEL + KF_HORIZ_ACCEL_GAIN × horiz_accel²
//
// KF_HORIZ_ACCEL_GAIN : 増幅ゲイン [s²/m²]
//   0    → 従来どおり（水平加速度による補正なし）
//   大きい → 少しの水平加速度でも IMU の影響を早く落とす
//
// 参考: gain=0.01 のときの q_vel_eff
//   horiz 0 m/s² → 0.050 (×1.0)
//   horiz 2 m/s² → 0.090 (×1.8)
//   horiz 4 m/s² → 0.210 (×4.2)
//   horiz 8 m/s² → 0.690 (×13.8)
#define KF_HORIZ_ACCEL_GAIN  0.01f

// ============================================================
// GNSS高度補正パラメーター（imu_kalman_gnss_update で使用）
// ============================================================
// GNSS高度は気圧高度より誤差が大きいが、長期ドリフトのない絶対基準として使える。
// 気圧基準（ground_alt_abs）をゆっくり修正することで、高度をGNSS基準に近づける。
// Vertical speedへの影響は補正レート（最大 GNSS_MAX_DELTA_M m/s）に留まり無視できる。
//
// GNSS_VACC_MAX_M        : vAcc がこれ以上の時は補正しない（カットオフ）[m]
// GNSS_INIT_SAMPLES      : 起動地MSL高度を平均する初期化サンプル数
// GNSS_CORRECT_RATE      : 気圧基準補正ゲイン（イノベーション×quality に掛ける比率）
//                          vAcc が小さい（高精度）ほど quality が高く補正が速くなる。
// GNSS_MAX_DELTA_M       : 1更新あたりの最大補正量 [m]（バリオへの影響上限）
// GNSS_OFFSET_UPDATE_RATE: gnss_kf_offset（起動地MSL高度）の長期更新ゲイン。
//                          MSL絶対高度がGNSSに収束する速さを決める。quality × rate が実効値。
//                          rate=0.005, quality=0.8 → α=0.004/s → 半減期約170秒（約3分）
#define GNSS_VACC_MAX_M        10.0f  // 垂直精度カットオフ [m]（vAcc < 10m の時のみ補正）
#define GNSS_INIT_SAMPLES      10     // 初期化サンプル数
#define GNSS_CORRECT_RATE      0.02f  // 気圧基準補正ゲイン（旧 0.005 → 4倍に増速）
#define GNSS_MAX_DELTA_M       0.05f  // 1回あたりの最大補正量 [m]（旧 0.02 → 2.5倍）
#define GNSS_OFFSET_UPDATE_RATE 0.005f // MSL絶対基準の長期収束ゲイン
//
// ============================================================
// GNSS VSI Kalman 速度観測パラメーター（imu_kalman_gnss_vel_update で使用）
// ============================================================
// GNSS velD（上昇正）を KF の速度観測（H=[0,1,0]）として取り込む。
// 観測ノイズ R_vel は sAcc²（速度精度の二乗）× R_SCALE を使用。
// ゲートは sAcc のみで判断する（vAcc=垂直位置精度 は速度品質の指標として不適切なため使用しない）。
//
// GNSS_VSI_SACC_MAX_MPS: sAcc がこの値以上なら速度観測更新をスキップ [m/s]
//                        sAcc = NAV-PVT Speed Accuracy Estimate（速度精度 1-sigma）。
//                        この値未満では R_vel = sAcc² × R_SCALE の連続曲線が機能する。
// GNSS_VSI_R_SCALE     : 観測ノイズ R_vel の倍率（1.0 = sAcc² そのまま）
//                        大きくするほど GNSS VSI の影響が弱まり、気圧・IMU 主体になる。
//                        K[1] ≈ P/(P+R) なので R を 16 倍にするとゲインが大幅に減少する。
//
// 参考: sAcc² × R_SCALE のカルマンゲイン K[1] ≈ P/(P+R) への影響（P≈0.2 の場合）
//   sAcc=0.1 → R=0.16  K≈0.56（有効に補正）
//   sAcc=0.2 → R=0.64  K≈0.24（緩やかに補正）
//   sAcc=0.3 → [GNSS_VSI_SACC_MAX_MPS でスキップ]
//
#define GNSS_VSI_SACC_MAX_MPS  0.3f  // 速度精度ハードゲート [m/s]（以上はスキップ）
#define GNSS_VSI_R_SCALE      16.0f  // R_vel 倍率（sAcc 小さい時のみ有効に機能させる）

// ============================================================
// 高度表示設定
// ============================================================
// 当デバイスの GPS は UBX NAV-PVT の hMSL（EGM96 ジオイド基準）を使用する。
// hMSL は日本の標高（T.P.=東京湾平均海面 基準）とほぼ一致する（差は ±数十cm 程度）。
//
// ※注意: GPS の「楕円体高（WGS84）」とは異なる。
//   楕円体高 = hMSL + ジオイド高 N（日本では N ≈ +36〜38m）
//   例: 関西で 0m 標高 → hMSL ≈ 0m、楕円体高 ≈ +37m
//   当 GPS は既に hMSL を出力済みなので、-37m 補正は不要。
//
// ELEVATION_GEOID_OFFSET_M: B.S.L. 高度 = KF_MSL - この値 [m]
//   表示ラベルは "B.S.L."（琵琶湖基準水位面、Biwa Standard Level）。
//   B.S.L. 0m = T.P.（日本の標高） +84.371m（瀬田川洗堰基準・国土交通省設定値）。
//   → 琵琶湖面で B.S.L. ≈ 0m、湖面上 100m 飛行時に B.S.L. ≈ 100m と表示される。
//   標高（T.P.）に戻したい場合は 0.0f に変更する。
#define ELEVATION_GEOID_OFFSET_M  84.371f  // B.S.L.基準補正 [m]（T.P.84.371m = 琵琶湖面）