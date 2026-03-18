// ============================================================
// File    : imu.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : BNO085 IMU モジュール + Kalman フィルターフュージョンのヘッダー。
//           MS5611 気圧高度と BNO085 加速度を融合した高精度バリオメーターを提供する。
//
// センサー: GY-BNO080 (BNO085), I2C アドレス 0x4B
// ライブラリ: Adafruit_BNO08x
//
// 使用レポート:
//   SH2_GAME_ROTATION_VECTOR  : クォータニオン（磁気干渉なし）30Hz
//   SH2_LINEAR_ACCELERATION   : 重力除去済み加速度（ボディフレーム）30Hz
//   SH2_ROTATION_VECTOR       : 地磁気補正クォータニオン（ヨー専用）5Hz
//
// Kalman フィルター 状態変数:
//   x[0] = z      : 推定高度 [m]
//   x[1] = z_dot  : 推定上昇率 [m/s]
//   x[2] = b      : 加速度バイアス [m/s²]
//
// 呼び出し方:
//   setup() 内で imu_setup() を呼ぶ。
//   loop() 内で毎回 imu_update() を呼ぶ（15ms ポーリング + Kalman predict）。
//   airdata_update() が true を返したタイミングで imu_kalman_baro_update() を呼ぶ。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/15
// ============================================================

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ---- 初期化 ----
// BNO085 の I2C バス・割り込み・センサーレポートを初期化する。
// airdata_setup() の後に呼ぶこと（ログ用タスクキューが必要なため）。
void imu_setup();

// ---- メインループ処理 ----
// 15ms ポーリングで BNO085 からデータを読んで Kalman predict を実行する。
// loop() から毎回呼ぶ（ノンブロッキング）。
void imu_update();

// ---- 気圧高度による Kalman 観測更新 ----
// MS5611 の高度が更新されたときに外部から呼ぶ。
// airdata_update() が true を返したタイミングで get_airdata_altitude() を渡す。
void imu_kalman_baro_update(float z_baro_m);

// ---- センサー状態 ----
// BNO085 の接続・初期化が正常に完了しているか（imu_setup() 後に確定）。
bool get_imu_ok();

// ---- 推定値ゲッター ----
float get_imu_vspeed();    // Kalman 推定上昇率 [m/s]（正: 上昇、負: 下降）
float get_imu_altitude();  // Kalman 推定高度 [m]（気圧高度基準）
float get_imu_az();        // 地球座標系の鉛直加速度 [m/s²]（デバッグ・バリオ音用）
void  get_imu_quaternion(float &qw, float &qx, float &qy, float &qz);  // クォータニオン (GAME_RV)
void  get_imu_linaccel(float &ax, float &ay, float &az);                // 線形加速度 ボディフレーム [m/s²]
void  get_imu_euler(float &roll, float &pitch, float &yaw);             // Euler 角 [度] ZYX 規約（ヨーは地磁気補正）
float get_imu_mag_accuracy_deg();                                       // ヘディング精度推定値 [度]（-1: 未受信）

// ---- 各センサーイベントの受信レート [Hz]（1 秒ウィンドウで計測）----
float get_imu_grv_hz();   // GAME_ROTATION_VECTOR
float get_imu_lacc_hz();  // LINEAR_ACCELERATION
float get_imu_rv_hz();    // ROTATION_VECTOR
bool  get_imu_rv_updated(); // ROTATION_VECTOR 新着フラグ（読み出しでクリア）

// ---- Kalman パラメーターの動的変更（実行時チューニング用）----
void imu_set_kf_params(float q_vel, float q_bias, float R);

#endif // IMU_H
