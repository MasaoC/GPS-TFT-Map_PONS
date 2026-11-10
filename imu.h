// ============================================================
// File    : imu.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : IMU（SCH16T）+ Kalman フィルターフュージョンのヘッダー。
//           MS5611 気圧高度と IMU の加速度を融合した高精度バリオメーターを提供する。
//
// センサー: Murata SCH16T-K01（SPI1。ドライバは imu_sch16t.cpp）
//   生のジャイロと加速度を 250Hz で読む。姿勢は attitude.cpp の ESKF。
//
// Kalman フィルター 状態変数:
//   x[0] = z      : 推定高度 [m]
//   x[1] = z_dot  : 推定上昇率 [m/s]
//   x[2] = b      : 加速度バイアス [m/s²]
//
// 呼び出し方:
//   setup() 内で imu_setup() を呼ぶ。
//   loop() 内で毎回 imu_update() を呼ぶ（4ms ポーリング + 30ms ごとの Kalman predict）。
//   airdata_update() が true を返したタイミングで imu_kalman_baro_update() を呼ぶ。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/10
// ============================================================

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "attitude.h"   // imu_body_euler_rad()（マウント回転）
// ★ ここは**チップに依らない公開 API**。実装は 2 つに分かれている:
//   imu.cpp（バリオ KF・観測・ゲッター）と imu_sch16t.cpp（SCH16T ドライバ）。
//   呼び出し側はこのヘッダだけを見ればよい。境界は imu_sensor.h。

// ---- 初期化 ----
// IMU のバス・センサーレポートを初期化する（実体はドライバ側）。
// airdata_setup() の後に呼ぶこと（ログ用タスクキューが必要なため）。
void imu_setup();

// ---- メインループ処理 ----
// ドライバを 1 回進めてデータを引き取り、Kalman predict を実行する。
// loop() から毎回呼ぶ（ノンブロッキング）。
void imu_update();

// センサーのサンプルだけを引き取る（軽量・副作用なし）。
// ★ **長いブロッキング処理の中から呼ぶこと。** SCH16T はこちらから取りに行く方式なので、
//   描画・無線・SD などで Core0 が数十 ms 止まると、その間のサンプルが抜ける。
//   Kalman predict も復旧処理もしないので、どこから呼んでも安全。
//   ポーリング時刻は imu_update() と共有しているので、二重に叩く心配も無い。
void imu_service_if_due();

// ---- 気圧高度による Kalman 観測更新 ----
// MS5611 の高度が更新されたときに外部から呼ぶ。
// airdata_update() が true を返したタイミングで get_airdata_altitude() を渡す。
void imu_kalman_baro_update(float z_baro_m);

// ---- GNSS高度による気圧基準補正 ----
// gnssFixOK && fixtype==3D のとき、メインループから約1秒ごとに呼ぶ。
// GNSS高度（MSL）で気圧の基準高度をゆっくり修正し、絶対高度ドリフトを抑制する。
// Vertical speed（z_dot）は変更しない。
//   z_gnss_msl : GNSS高度 [m MSL]（get_gnss_altitude() の値）
//   vacc_m     : 垂直精度推定値 [m]（get_gnss_vacc_mm() / 1000.0f）
void imu_kalman_gnss_update(float z_gnss_msl, float vacc_m);

// ---- GNSS垂直速度による Kalman 速度観測更新 ----
// gnssFixOK && fixtype==3D のとき、GNSS高度補正と同じタイミングで呼ぶ。
// sAcc を観測ノイズ（R_vel = sAcc²）、vAcc をゲーティングに使い、KF の z_dot を補正する。
// IMU の有無によらず動作する。
//   veld_mps  : GNSS 垂直速度 [m/s]（上昇正）= get_gnss_veld_mps()
//   vacc_m    : 垂直位置精度 [m]（get_gnss_vacc_mm() / 1000.0f）
//   sacc_mps  : 速度精度 [m/s]（get_gnss_sacc_mmps() / 1000.0f）
void imu_kalman_gnss_vel_update(float veld_mps, float vacc_m, float sacc_mps);

// ---- センサー状態 ----
// IMU の接続・初期化が正常に完了しているか（imu_setup() 後に確定）。
bool get_imu_ok();
// IMU が正常稼働中か（接続済み かつ 直近1秒以内にデータ受信）。
// 通信途絶時は false を返す点が get_imu_ok() と異なる。
// update_vario() の MS5611 フォールバック判定に使用する。
bool get_imu_alive();

// ---- バリオの供給元（音と VSI の線で共有する）----
// ★ **音（src/sound.cpp の update_vario）と VSI のグレー線（display_tft.cpp の
//   draw_vsi）は必ずこの 3 つを通すこと。** 0.983 まで両者が別々に
//   `get_imu_alive()` / `get_imu_ok()` で分岐していて、いつ食い違っても
//   おかしくなかった（CLAUDE.md の「線は出ているのに鳴らない」）。
bool  vario_source_ok();         // V/S を出せる状態か（false なら VSI のバーを描かない）
float vario_vspeed_mps();        // 表示・音・無線に使う V/S [m/s]（供給元はここが決める。出せなければ 0）
float vario_deadband_mps();      // デッドバンド [m/s]（KF 融合中は狭い）

// ---- 推定値ゲッター ----
float get_imu_vspeed();          // Kalman 推定上昇率 [m/s]（正: 上昇、負: 下降）
// _raw 版は「必ず自機の値」を返す。無線のミラー中でも自機 CSV には
// 自分の値を書く必要があるため（link.cpp のコメント参照）。
float get_imu_vspeed_raw();
float get_imu_altitude_msl_raw();
float get_imu_altitude_msl();    // Kalman 推定高度 [m]（MSL 絶対値 = AGL + gnss_kf_offset）
                                 // gnss_kf_offset 未確定（GNSS fix 取得前）は AGL 値を返す
bool  get_imu_gnss_offset_ready(); // gnss_kf_offset が初期化済みか（MSL 値が有効か）
float get_imu_az();              // 地球座標系の鉛直加速度 [m/s²]（デバッグ・バリオ音用）
float get_imu_horiz_accel();     // 地球座標系の水平加速度の大きさ [m/s²]（KF Q_vel 動的増幅用）

// 加速度サンプルの鮮度不足で Kalman predict をスキップした累計回数。
// 正常時は 0 のまま。増えていれば読み出しが止まっている。
uint32_t get_imu_acc_stale_skips();

// imu_update() が呼ばれなかった最長時間 [µs]。**読むとリセットされる。**
uint32_t get_imu_poll_gap_max_us();

// 生ジャイロ・生加速度の受信レート [Hz]（1 秒ウィンドウで計測）
float get_imu_gyro_hz();
float get_imu_accel_hz();

// 生ジャイロ・生加速度の直近値（設定画面の IMU/ESKF ページで実測値を見るため）
void get_imu_raw_gyro(float g[3]);   // [rad/s]
void get_imu_raw_accel(float a[3]);  // [m/s²]

// ---- Kalman パラメーターの動的変更（実行時チューニング用）----
void imu_set_kf_params(float q_vel, float q_bias, float R);

// ---- Kalman パラメーターのゲッター（SD設定保存用）----
float get_imu_kf_q_vel();   // 速度プロセスノイズ
float get_imu_kf_q_bias();  // バイアスプロセスノイズ
float get_imu_kf_R();       // 気圧高度観測ノイズ [m²]

#endif // IMU_H
