// ============================================================
#pragma once
// File    : settings.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GPS/TFT種別選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// リリース時
#define RELEASE
//#define DEBUG_ESKF

#define BUILDDATE 20260820
#define BUILDVERSION "0.941"
#define VERSION_TEXT "Version 6"




//----------GPS---------
//#define DEBUG_GBX_NMEA
//#define QUECTEL_GPS
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

// センサー値を眺めるための画面（VARIO 詳細）の強制リフレッシュ間隔 [ms]。
// 地図画面は GPS 更新(2Hz)でも再描画されるので 1050ms で足りるが、
// VARIO 詳細には他の再描画トリガーが無く、この間隔がそのまま表示更新レートになる
// （従来は 1050ms = 約 0.95Hz しか出ていなかった）。
//
// 200ms = 5Hz。この画面ではベクタ地図を描かないため 1 回の再描画は地図画面(平均64ms)より
// ずっと軽く、Core0 に十分な余裕がある。GPS シリアルも IRQ で 1024B までバッファされるため
// （38400bps で約 266ms 分）取りこぼさない。
// これ以上速くしたい場合は間隔を詰めるのではなく、VSI バーと同じように
// 値の部分だけ部分転送する方式にすること（全画面 115KB 転送が支配的になるため）。
#define SCREEN_FRESH_INTERVAL_DETAIL 200


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
#define BAT_HALF_VOLTAGE 3.8 // 50%未満 (4.2-3.4=0.8V の半分は0.4Vなので4.2-0.4=3.8Vが50%の目安)
#define BAT_LOW_VOLTAGE 3.5
#define BAT_ZERO_VOLTAGE 3.4
#define PIN_PWMTONE 38
#define PIN_AMP_SD 39 //アンプシャットダウン(HIGHでON)
#define USERLED_PIN 34 //ユーザーLED（エラー表示用。エラー時 HIGH）
#define SIN_VOLUME 0.15f  // Sin波の振幅倍率（0〜1.0f）。WAVと音量を合わせるため小さめにしてあるが、バリオが小さいと感じる場合は上げる。0.5fで±254、1.0fで±508（±512ヘッドルーム）。
#define VARIO_VOL_SCALE 3
// 上昇ビープ（高音）の音量を、下降音（低音）に対して何%にするかの補正。
// 小型スピーカーは低音の音響能率が低く、同じ振幅でも高音ばかり大きく聞こえるため高音側を絞る。
// 下降音側は VARIO_VOL_SCALE=3 で既にほぼ振幅上限（vario_volume 33%程度でクリップ開始）なので、
// バランス調整は原則こちらの値で行う。小さくするほど高音が控えめになる（目安 15〜40）。
#define VARIO_ASCEND_VOL_PCT 25

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
#define MODE_REPLAYSELECT 7  // リプレイ再生ファイルの選択画面
#define MODE_IMUDETAIL 8     // IMU / 姿勢 ESKF の詳細画面


//======= リプレイ再生設定 ======
// 固定エントリ（大会データ）の SD 上のパス。
// SD ルートに置くとファイル一覧にも重複表示されるため replay/ サブフォルダに置く。
#define REPLAY_2025_FILE "replay/2025taikai.csv"
#define REPLAY_2026_FILE "replay/2026taikai.csv"
#define REPLAY_2025_LABEL "2025 Taikai"
#define REPLAY_2026_LABEL "2026 Taikai"

#define REPLAY_BUF_SIZE   16   // 先読みするCSV行数（Core1が供給 → Core0が消費するリングバッファ）
                               // ※ インデックス計算にビットマスクを使うので必ず 2 のべき乗にすること
                               // 高速再生(x20)では 2Hz データを毎秒40行消費するため余裕を持たせている
#define REPLAY_LIST_ROWS  20   // リプレイ選択画面の1ページあたり表示行数（20行×12px = 240px）
#define REPLAY_FIXED_COUNT 5   // 一覧の先頭に並ぶ固定項目数の最大値
                               // （Replay OFF / PLAY FLIGHT ONLY / PLAY SPEED / 2025 / 2026）
                               // 2025・2026 は SD 上に実ファイルがあるときだけ表示するため、
                               // 実際の数は replay_menu_fixed_count()（3〜5）を使うこと。
// 再生速度の選択肢。x1 → x2 → x_FAST の順に切り替わる（既定は x1）。
// 高速側の倍率を変えたいときは REPLAY_SPEED_FAST だけ書き換えればよい。
#define REPLAY_SPEED_FAST 20
#define REPLAY_MIN_GS 0.2f     // PLAY FLIGHT ONLY 有効時、この対地速度 [m/s] 以下を「静止」とみなす
#define REPLAY_LEADIN_MS 5000  // 動き出しの手前この時間 [ms] 分は静止していても再生する（助走表示）。
                               // 静止区間がこの長さ未満なら一切スキップしない。
#define REPLAY_LEADIN_SLOTS 40 // 助走区間の先頭を探すために保持する行数（5秒 ÷ 最短サンプル間隔ぶん）
#define REPLAY_SCAN_MAX 2000   // 静止区間の先読み走査で1回に読む最大行数（Core1 を長時間占有しないための上限）
#define REPLAY_REQ_INTERVAL_MS 200  // Core1へのバッファ補充依頼の最短間隔 [ms]
#define REPLAY_FILENAME_LEN 40      // リプレイ対象ファイル名（パス込み）の最大長


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
// 生 IMU ロガー（姿勢 ESKF のオフライン開発用）
// ============================================================
// 目的: BNO085 内蔵フュージョンは比力を鉛直とみなすため、旋回中はロールを過小評価し、
//       加減速中はピッチがずれる。これを GNSS 速度で補正する ESKF を PC 上で開発するため、
//       生のジャイロ・加速度・地磁気と GNSS 速度を SD にバイナリ記録する。
//       機上では推定を行わない（詳細は imulog.h）。
//
// バス負荷: i2c0 は MS5611 と共用だが、下記レートでも占有率は約 20%（400kHz）。
//           MS5611 ~1.9% + 既存レポート ~2.2% を足しても十分余裕がある。
// SD 負荷 : 約 260 レコード/秒 × 28B ≒ 7.3kB/s（30 分飛行で約 13MB）。
#define IMULOG_DEFAULT_ENABLED  true  // 起動時の既定。SD 設定 imulog で切替可能

// ============================================================
// ★ 生レポートの有効化スイッチ ★
// ============================================================
// 0: 変更前とまったく同じ挙動（レポートは GRV/LACC/RV の 15/15/5Hz のみ、
//    ポーリング 30ms、KF predict ゲート無し）。ロガー自体は動くが
//    記録されるのは既存レポートと GNSS・気圧だけになる。
// 1: GYRO/ACCEL/MAG を追加し、ポーリングを速くする。
//
// 【2026-08-17 の回帰と、実機ログで特定した原因】
// GYRO/ACCEL を 100Hz、MAG 25Hz で要求（BNO085 への要求合計 260 レポート/秒）した
// ところ、上下に動かした際にバリオが過大な値を出し 3 秒ほど尾を引く回帰が出た。
// 60 秒ログの実測値:
//     rate GRV=4.0 LACC=5.0 RV=1.0 MS5611=40.0 Hz  drop=0
//   → MS5611 は 40Hz で正常。気圧観測レートの低下ではなかった。
//   → GRV/LACC/RV が設定値 15/15/5 に対し 4/5/1 まで飢餓状態になっていた。
//
// 機序: BNO085 の配信能力（ポーリング 1 回につきおおむね 1 レポート）に対して
//   要求レートが過大だったため、高レートのレポートが内部キューを占有し、
//   低レートの GRV/LACC/RV が押し出された。
//   結果 kf_predict() は 33Hz で回るのに _lax.. は 5Hz でしか更新されず、
//   同じ加速度サンプルを 6〜7 回繰り返し積分していた（＝スパイクが数倍に増幅され、
//   気圧観測が引き戻すまで数秒尾を引く）。
//
// 対策1: 要求レートを配信能力の内側に収める（下記 50/50/10 = 合計 145 レポート/秒）。
//        HPA の運動帯域は 5Hz 以下なので ESKF には 50Hz で十分。
// 対策2: ポーリングを 4ms（250Hz）に上げて配信能力の天井そのものを上げる。
// 対策3: imu.cpp に「古い加速度サンプルでは predict しない」安全網（下記 MAX_AGE）。
//
// ★ 変更後は必ず 60 秒ログの rate 行を確認すること。
//   GRV=15.0 LACC=15.0 RV=5.0 が出ていれば飢餓は解消している。
// （#if で判定するため true/false ではなく 1/0 で書く）
#define IMULOG_RAW_REPORTS_ENABLED  1

// BNO085 レポート周期 [Hz]（IMULOG_RAW_REPORTS_ENABLED が 1 のときのみ有効）
//   合計要求レートを配信能力の内側に保つこと。上げすぎると既存レポートが飢餓になり
//   バリオが壊れる（上記参照）。既存 35 + 生 110 = 145 レポート/秒。
#define IMU_RATE_GYRO_HZ    50  // SH2_GYROSCOPE_CALIBRATED
#define IMU_RATE_ACCEL_HZ   50  // SH2_ACCELEROMETER（重力込みの生比力。ESKF と、VARIO_USE_RAW_ACCEL=1 のときバリオ KF が使う）
#define IMU_RATE_MAG_HZ     10  // SH2_MAGNETIC_FIELD_CALIBRATED（ヨー絶対値は対象外なので低レートで十分）

// 既存レポート（バリオ KF・姿勢表示用。変更するとバリオのチューニングに影響する）
#define IMU_RATE_GRV_HZ     15  // SH2_GAME_ROTATION_VECTOR
#define IMU_RATE_LACC_HZ    15  // SH2_LINEAR_ACCELERATION
#define IMU_RATE_RV_HZ       5  // SH2_ROTATION_VECTOR

// ポーリング周期 [µs]。
//   生レポート有効時: 4ms（250Hz）。BNO085 はポーリング 1 回につきおおむね
//                     1 レポートしか返さないため、ポーリング周期が配信能力の
//                     上限を決める。要求 145 レポート/秒に対し約 1.7 倍の余裕。
//                     描画で 64ms 止まった後の取り戻しにもこの余裕が要る。
//   無効時          : 従来どおり 30ms（変更前と同一）。
#if IMULOG_RAW_REPORTS_ENABLED
  #define IMU_POLL_INTERVAL_US   4000
#else
  #define IMU_POLL_INTERVAL_US   30000
#endif

// 加速度サンプルの許容鮮度 [µs]（安全網）。
// LINEAR_ACCELERATION が 15Hz = 67ms 周期なので、150ms は正常時には決して発火しない。
// 発火するのは今回のような「配信飢餓」が起きたときだけで、そのとき predict を止める。
// predict を止めても imu_kalman_baro_update() が ~40Hz で出力を更新し続けるため、
// バリオは BNO085 なし時と同じ気圧ベース動作に劣化するだけで済む
// （古い加速度を繰り返し積分して暴れるより遥かに安全）。
#define IMU_LACC_MAX_AGE_US  150000

// ============================================================
// バリオ KF に入れる鉛直加速度の作り方
// ============================================================
// 1 : SH2_ACCELEROMETER（重力込みの生比力）を GAME_ROTATION_VECTOR で地球座標系に
//     回してから重力加速度 GRAVITY_MPS2 を引く。← 推奨
// 0 : SH2_LINEAR_ACCELERATION（BNO085 が重力を引いた値）をそのまま使う。従来の挙動。
//
// ■ なぜ変えたか（2026-08-19、tools/imulog/20260819.bin の屋内上げ下げで確認）
//   BNO085 の LINEAR_ACCELERATION は、GAME_ROTATION_VECTOR とは別の「適応的な重力推定」で
//   重力を引いている。数百 ms 続く鉛直加速度をこの重力推定が徐々に吸収してしまうため、
//   加速中は加速度を過小に、動作が終わった後は逆符号の残差を出す。
//   実測（静止 → 上げる → 止める、を繰り返した区間の ∫a_k dt。両端が静止なので期待値は 0）:
//       区間             LINEAR_ACCELERATION   生比力 - g
//        持ち上げ 4秒        -0.98 m/s            -0.10 m/s
//        下ろす   3秒        +0.76 m/s            +0.26 m/s
//        持ち上げ 5秒        -1.02 m/s            -0.34 m/s
//   ＝ 動かした向きと逆に約 1 m/s の速度誤差が入る。これがそのまま
//      「上げ下げを止めた瞬間に反対側の音が鳴る」症状になっていた。
//   77 秒通しての ∫a_k dt は -0.11 m/s なので、定常的なバイアスではなく
//   「動作に相関した誤差」である点が重要（KF のバイアス状態では取り除けない）。
//
//   KF に通した結果（屋内で上げ下げした 3 セッション）:
//       推定上昇率の符号が真値と逆になる時間の割合  18〜26% → 0.8〜6.5%
//       静止しているのに |v|>0.3 で鳴ってしまう割合  8〜16% → 0〜1.6%
//   屋外（車載・ゆっくりした高度変化）では両者ほぼ同等で、悪化はしない。
#define VARIO_USE_RAW_ACCEL  1

// 重力加速度 [m/s²]。生比力から鉛直成分を取り出すときに引く値。
// 標準重力。日本の実測値との差 (~0.01 m/s²) は KF のバイアス状態 x[2] が吸収する。
#define GRAVITY_MPS2  9.80665f


// ============================================================
// 姿勢 ESKF（GNSS 速度援用）チューニング
// ============================================================
// 実装は attitude.cpp。PC 側の tools/imulog/eskf.py と対になっているので、
// 値を変えたら両方に反映すること（片方だけ変えると比較が成立しなくなる）。

// プロセスノイズ（BNO085 の実力と HPA の運動を想定した初期値）
#define ESKF_SIGMA_G    2e-3f    // ジャイロ白色雑音 [rad/s/sqrt(Hz)]
#define ESKF_SIGMA_A    3e-2f    // 加速度計白色雑音 [m/s²/sqrt(Hz)]
#define ESKF_SIGMA_BG   1e-5f    // ジャイロバイアスのランダムウォーク
#define ESKF_SIGMA_BA   1e-4f    // 加速度バイアスのランダムウォーク

// バイアス推定の物理的な上限。
// BNO085 は校正済みの値を出すので残留バイアスは本来ごく小さい。
// 上限が無いと初期姿勢誤差の行き場としてバイアス状態が使われ、誤った値に固着する。
// 実測（2026-08-18 車載 session 2）では bg が 5.9deg/s に張り付き、
// ESKF が BNO085 と同じ偽ピッチを出すようになった。上限を入れて解消している。
#define ESKF_MAX_GYRO_BIAS   0.01745f  // 1.0 deg/s [rad/s]
#define ESKF_MAX_ACCEL_BIAS  1.0f      // [m/s²]

// GNSS 速度観測の信頼度（u-blox の sAcc をそのまま使う）
#define ESKF_SACC_MIN   0.05f    // これ未満は過信になるので下限を置く [m/s]
#define ESKF_SACC_MAX   1.0f     // これを超える観測は捨てる（実測で 248m/s の暴れ値あり）

// 静止判定（初期姿勢を静止中の GRV 平均で決めるために使う）
#define ESKF_STATIC_GYRO_RADS   0.026f    // 約 1.5 deg/s
#define ESKF_STATIC_ACCEL_TOL   0.30f     // |加速度| と重力の差 [m/s²]
#define ESKF_STATIC_INIT_US     2000000UL // この時間だけ静止が続いたら初期化 [µs]

// 初期姿勢の不確かさ [度]
#define ESKF_INIT_ATT_SIGMA_DEG         3.0f   // 静止平均から初期化した場合
// 静止区間が取れないまま走り出した場合のフォールバック。
// 初期姿勢が信用できないので大きめに取り、GNSS 観測で速く引き戻せるようにする。
#define ESKF_INIT_ATT_SIGMA_MOVING_DEG  30.0f

// ---- 初期ヨーに地磁気を使う（ROTATION_VECTOR）----
// ESKF のヨーは水平加速度がある間しか可観測にならないため、
// 起動直後〜収束前（旋回ありで約2分、フゴイドのみだと5〜10分）は絶対方位を持てない。
// BNO085 の ROTATION_VECTOR は地磁気で磁北基準の方位を即座に出せるので、
// これを初期値に使って収束を待たずに妥当な方位から始める。
// ロール・ピッチはどちらも重力基準で同じなので劣化しない。
#define ESKF_INIT_USE_RV        1

// BNO085 が報告するヘディング精度推定 [度] の上限。
// これより悪い（＝磁気キャリブレーション未完了・磁気環境が悪い）場合は RV を使わず
// GAME_ROTATION_VECTOR で初期化する。-1（未受信）も使わない。
#define ESKF_RV_ACC_MAX_DEG     25.0f

// 磁気偏角 [度]。東偏が正、西偏が負。
//   真方位 = 磁方位 + 偏角
// 琵琶湖（約 35.3N, 136.1E）は西偏およそ 8 度。
// ※ 国土地理院の最新値で確認すること。偏角は年 0.02〜0.03 度ずつ変化する。
// ※ 他の飛行場所で使うならここを変える。
#define ESKF_MAG_DECLINATION_DEG  (-8.0f)

// IMU データが途絶したと判断するまでの時間 [µs]。
// これを超えてジャイロが来なければ ESKF を無効扱いにし、GNSS 観測も取り込まない。
//
// これが無いと、BNO085 断線後にジャイロ・加速度が止まっても GNSS 速度だけ入り続け、
// フィルタが速度の食い違いを姿勢誤差のせいにして姿勢を回してしまう。
// 実測（模擬）では水平のまま加速しただけで 60 秒後にロールが +8 度になり、
// バンク警告の閾値 4 度を超えて偽警報が出た。
//
// 500ms は通常の欠測（地図描画による最大 206ms の遅延）より十分大きく、
// かつ異常を素早く捕まえられる値。
#define ESKF_IMU_TIMEOUT_US  500000UL

// 機体ゼロ点較正を許可する対地速度の上限 [m/s]。
// 飛行中に実行されると傾いた姿勢を水平として焼き付け、バンク角警告が
// 機能しなくなるため、地上でしか通さないためのガード。
#define LEVEL_CALIB_MAX_MPS  1.0f

// ---- 較正時に申告するピッチ角の範囲 [度] ----
// 「いま機体は何度か」を人が申告して、その値になるよう較正するための選択肢。
// 本番プラットホームは -3.5 度。試験飛行場の地面も水平とは限らない。
// マウントは充電などで頻繁に付け外しするので、その場で直せる必要がある。
#define PITCH_TARGET_MIN_DEG   (-3.5f)
#define PITCH_TARGET_MAX_DEG   ( 2.0f)
#define PITCH_TARGET_STEP_DEG  ( 0.5f)

// バンク角の警告閾値 [度]。これを超えたら地図上のロール表示を赤にする。
// HPA は旋回半径が大きく、実運用のバンクは数度程度。
// （対気 7m/s・バンク 5度で旋回半径 約75m）
#define BANK_WARN_DEG  4.0f

// ピッチによる警告は行わない。
// ピッチ単独では失速余裕を判定できず（本来は迎角が要る）、実測でも
// 30 秒平均ピッチと対気速度の相関は -0.14 と弱かった。
// 迎角推定も検討したが、対地経路角の 30 秒平均が中央 -0.01 度・|2度|超えが 0% で
// 補正項がほぼゼロ、ノイズだけ増える（std 0.73 → 0.88 度）ため見送った。
// ピッチは表示のみとし、閾値は実飛行で失速側のデータが蓄積してから決める。

// ---- バンク警告 ----
// 実測（2026-07-26 琵琶湖 45分）での発報回数:
//   閾値4度・継続条件なし → 31回（中央継続 0.2秒。チカチカするだけ）
//   閾値4度・1秒以上継続  → 3回   ← これを採用
#define BANK_WARN_HOLD_MS      1000UL   // この時間continuous に超えたら発報
// 再武装（再度鳴らせる状態に戻す）条件。想定している誤警報は「センサーのズレで
// バンクが 4 度以上のまま延々と続く」形なので、時間経過だけでは解除しない。
// バンクが十分戻り、かつ WAV の再生が終わるだけの時間が経ってから解除する。
#define BANK_WARN_CLEAR_DEG     3.0f    // これ未満まで戻ったら「解除できる」状態
#define BANK_WARN_REARM_MS    10000UL   // 発報からこの時間は解除しない（WAV 再生の待ち）

// ---- 直進中のロール自動トリム ----
// 飛行機は直進中のロールが理論上 0 になるはずなので、そこからのズレを
// ゆっくり戻す。飛行前の較正ミスや飛行中に PONS が傾いた場合の保険。
// 誤警報の連発を防ぐのが目的で、真のバンクを消さないよう条件を厳しくしてある。
#define ROLL_TRIM_YAWRATE_DPS   1.0f    // これ未満の旋回レートを「直進」とみなす
#define ROLL_TRIM_MIN_SPEED     3.0f    // これ以上の対地速度（＝飛行中）でのみ働く
#define ROLL_TRIM_WINDOW_S     60.0f    // この時間ぶん直進が続いたら 1 回評価する
#define ROLL_TRIM_DEADBAND_DEG  0.5f    // ズレがこの値未満なら補正しない
#define ROLL_TRIM_STEP_DEG      0.5f    // 1 回の補正量の上限（徐々に効かせる）
#define ROLL_TRIM_LIMIT_DEG     5.0f    // 自動補正の累積上限（本当の異常を隠さないため）

// ---- リプレイ用 ESKF 結果ログ ----
// 「そのとき画面に何が出ていたか」を後から再現するためのログ。
// 旧 euler/ は BNO085 由来だったが、ESKF 導入で役割が変わったので名前を変えた。
// 旧ファイルは再生時のフォールバックとして読む（IMU_REPLAYDATA_LEGACY_DIR）。
#define IMU_REPLAYDATA_DIR         "imu_replaydata"
#define IMU_REPLAYDATA_LEGACY_DIR  "euler"
#define IMU_REPLAYDATA_INTERVAL_MS  200UL   // 5Hz
// 再生中、姿勢ログのサンプルと CSV 行の時刻がこれ以上離れたら「その時刻のデータは
// 無い」とみなして非表示にする。姿勢ログが飛行 CSV より先に終わっている場合に、
// 最後のサンプルを貼り付けたまま固まって見えるのを防ぐ。
#define REPLAY_ATT_MAX_AGE_MS      2000UL

// ---- 平均ピッチ ----
// フゴイドで瞬時ピッチは±3度振れるため、巡航のトリム状態は平均で見る。
// 実測では 30 秒平均の std が 0.73 度で、1 度の変化が有意に読める。
#define PITCH_AVG_SEC          30.0f

// ヨー（機首方位）を信用してよい推定標準偏差 σ の上限 [度]。
// ESKF のヨーは水平加速度がある間しか可観測にならず、等速直進が続くと σ が育つ。
// この値以下のときだけ
//   ・地図上のヨー数値を黒（信頼できる）で表示する
//   ・自機アイコンを対地進路ではなく機首方位へ回す
// 超えたらグレー表示＋アイコンは対地進路のまま（誤った偏流角を見せないため）。
// 値は 95%（2σ）で表記する。航空分野の精度表記は 95% が慣例で、
// 1σ のまま判断基準に使うと 3 回に 1 回は外れることになるため。
// 20度(95%) は従来の 10度(1σ) と同じ動作。挙動は変えていない。
//
// 目安の根拠: 偏流角（HPA の巡航 7m/s に対し横風 1m/s で約 8度）と同程度までなら
// 機首方位に意味がある。厳しくすればほとんど回らなくなる。
// 実測（模擬）: 直進＋フゴイド±0.3m/s で 5 分後に 2σ=18.8度、10 分後に 14.4度。
//               完全等速だと 15 分でも 37度で閾値を割らない（安全側で正しい）。
#define ESKF_YAW_TRUST_95_DEG  20.0f

// バリオ KF の predict 周期 [µs]（生レポート有効時のみ使う）。
// kf_predict() は Q を dt でスケールせず「1 ステップあたり」で加算する
// （imu.cpp の kf_P[1][1] += q_vel_eff）。したがって predict 周期を変えると
// 単位時間あたりのプロセスノイズ注入量が変わり、チューニング済みのバリオが壊れる。
// ポーリングを 5ms に上げてもここで従来の 30ms を維持する。
//
// ※ ポーリングが 30ms のときはこのゲートを使ってはいけない。
//   ポーリング周期とゲート周期が同じだと、わずかなジッタでゲートを 1 回外し、
//   predict が 60ms 間隔になって dt が倍になる回が混ざるため。
#define IMU_KF_PREDICT_INTERVAL_US  30000

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
//   ② update (約 3.7Hz, 気圧高度):
//      ※ 50Hz ではない。MS5611 は ~40Hz でサンプルするが、airdata_update() が
//        true を返すのは VSPEED_WINDOW_MS(250ms) のトリム平均ウィンドウ完了時のみ。
//        チューニング時はこの比（predict 約 33Hz : update 約 3.7Hz）を前提にすること。
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
//   x[2] は本来「加速度計のゆっくりしたオフセット」を吸収するための状態。
//   0.005 は 1 predict(30ms) あたりの値なので毎秒 0.165 (m/s²)² 相当と大きく、
//   x[2] が数秒スケールで動けてしまい機動そのものを吸収していた
//   （実ログで機動に同期して ±0.2 m/s² 振れることを確認）。そのため 1/10 に下げてある。
//   ※ ただし「止めた瞬間に反対側へ振れる」現象の主因はこちらではなく、
//     VARIO_USE_RAW_ACCEL のコメントに書いた加速度ソースの方だった。
//     加速度ソースを直した後は Q_BIAS を 0.005 に戻しても指標はほぼ変わらない
//     （逆符号 6.4% vs 6.5%）。物理的に妥当な方を採る、という意味での 1/10。
// KF_R      : 気圧高度観測ノイズ [m²]
//   大きい → K[1] が小さくなり気圧ノイズの影響が減る（IMU 主体）
//   小さい → 気圧を強く信頼するため気圧ノイズが VSI に直接乗る

#define KF_Q_VEL    0.02f   // 速度プロセスノイズ  (旧 0.05 → 1/2.5。VSIふらつきをさらに抑制)
#define KF_Q_BIAS   0.0005f // バイアスプロセスノイズ (旧 0.005 → 1/10。上記コメント参照。効果は小さい)
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