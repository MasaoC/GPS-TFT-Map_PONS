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
// Updated : 2026/03/23
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

// ---- BNO085 の校正（DCD）（v7 の 0.976）----
// 保存記録は**本体フラッシュ（EEPROM 領域）**に置く。SD ではない。理由は imu.cpp。
// 目的は「一度も校正していない機体」を画面で明示すること。
bool     imu_cal_ready();              // acc==3 かつ gyr==3。SAVE を許す条件
bool     imu_cal_saved_ever();         // 過去に一度でも手動保存したか
uint32_t imu_cal_saved_date();         // 最後に保存した日 YYYYMMDD。0 = 日付不明
uint16_t imu_cal_save_count();         // 手動保存の回数
bool     imu_cal_autosave_on();        // 今回の起動でチップの自動保存を許したか
// 手動保存。yyyymmdd は 0 可（GNSS 時刻が無いとき）。**長押しからのみ呼ぶこと**
// （中で EEPROM.commit() が Core1 を止め、割り込みを切って数十 ms ブロックする）。
bool     imu_save_calibration(uint32_t yyyymmdd);

// ---- 気圧高度による Kalman 観測更新 ----
// MS5611 の高度が更新されたときに外部から呼ぶ。
// airdata_update() が true を返したタイミングで get_airdata_altitude() を渡す。
void imu_kalman_baro_update(float z_baro_m);

// ---- GNSS高度による気圧基準補正 ----
// gnssFixOK && fixtype==3D のとき、メインループから約1秒ごとに呼ぶ。
// GNSS高度（MSL）で気圧の基準高度をゆっくり修正し、絶対高度ドリフトを抑制する。
// Vertical speed（z_dot）は変更しない。
//   z_gnss_msl : GNSS高度 [m MSL]（get_gps_altitude() の値）
//   vacc_m     : 垂直精度推定値 [m]（get_gps_vacc_mm() / 1000.0f）
void imu_kalman_gnss_update(float z_gnss_msl, float vacc_m);

// ---- GNSS垂直速度による Kalman 速度観測更新 ----
// gnssFixOK && fixtype==3D のとき、GNSS高度補正と同じタイミングで呼ぶ。
// sAcc を観測ノイズ（R_vel = sAcc²）、vAcc をゲーティングに使い、KF の z_dot を補正する。
// BNO085 の有無によらず動作する。
//   veld_mps  : GNSS 垂直速度 [m/s]（上昇正）= get_gps_veld_mps()
//   vacc_m    : 垂直位置精度 [m]（get_gps_vacc_mm() / 1000.0f）
//   sacc_mps  : 速度精度 [m/s]（get_gps_sacc_mmps() / 1000.0f）
void imu_kalman_gnss_vel_update(float veld_mps, float vacc_m, float sacc_mps);

// ---- センサー状態 ----
// BNO085 の接続・初期化が正常に完了しているか（imu_setup() 後に確定）。
bool get_imu_ok();
// 起動時に BNO085 が見つかったか（載っている機体か）。途中で止まっても false にならない。
// ★ get_imu_ok() は「いま動いているか」なので、載っているかの判定には使わないこと（本番前チェック）。
bool get_imu_present();
// BNO085 が正常稼働中か（接続済み かつ 直近1秒以内にデータ受信）。
// 通信途絶時は false を返す点が get_imu_ok() と異なる。
// update_vario() の MS5611 フォールバック判定に使用する。
bool get_imu_alive();

// ---- 推定値ゲッター ----
float get_imu_vspeed();          // Kalman 推定上昇率 [m/s]（正: 上昇、負: 下降）
float get_imu_altitude_msl();    // Kalman 推定高度 [m]（MSL 絶対値 = AGL + gnss_kf_offset）
                                 // gnss_kf_offset 未確定（GNSS fix 取得前）は AGL 値を返す
bool  get_imu_gnss_offset_ready(); // gnss_kf_offset が初期化済みか（MSL 値が有効か）
float get_imu_az();              // 地球座標系の鉛直加速度 [m/s²]（デバッグ・バリオ音用）
float get_imu_horiz_accel();     // 地球座標系の水平加速度の大きさ [m/s²]（KF Q_vel 動的増幅用）
void  get_imu_linaccel(float &ax, float &ay, float &az);                // 線形加速度 ボディフレーム [m/s²]
void  get_imu_euler(float &roll, float &pitch, float &yaw);             // Euler 角 [度] ZYX 規約（ヨーは地磁気補正）
float get_imu_mag_accuracy_deg();                                       // ヘディング精度推定値 [度]（-1: 未受信）

// ---- 各センサーイベントの受信レート [Hz]（1 秒ウィンドウで計測）----
float get_imu_grv_hz();   // GAME_ROTATION_VECTOR
float get_imu_lacc_hz();  // LINEAR_ACCELERATION
float get_imu_rv_hz();    // ROTATION_VECTOR
// 加速度サンプルの鮮度不足で Kalman predict をスキップした累計回数。
// 正常時は 0 のまま。増えていれば BNO085 のレポート配信が滞っている。
uint32_t get_imu_lacc_stale_skips();
// 生ログ用レポートの受信レート [Hz]（設定値どおり出ていれば配信飢餓は起きていない）
float get_imu_gyro_hz();   // GYROSCOPE_CALIBRATED
float get_imu_accel_hz();  // ACCELEROMETER
float get_imu_mag_hz();    // MAGNETIC_FIELD_CALIBRATED

// 生ジャイロ・生加速度の直近値（設定画面の IMU/ESKF ページで実測値を見るため）
void get_imu_raw_gyro(float g[3]);   // [rad/s]
void get_imu_raw_accel(float a[3]);  // [m/s²]
// BNO085 が申告するセンサー精度（0=信頼できない〜3=高）。sv.status の下位 2bit。
uint8_t  get_imu_acc_accuracy();
uint8_t  get_imu_gyr_accuracy();
uint8_t  get_imu_mag_accuracy();
uint8_t  get_imu_cal_cfg();      // 動的校正の設定（SH2_CAL_*）。0xFF = 未取得

// ---- Kalman パラメーターの動的変更（実行時チューニング用）----
void imu_set_kf_params(float q_vel, float q_bias, float R);

// ---- Kalman パラメーターのゲッター（SD設定保存用）----
float get_imu_kf_q_vel();   // 速度プロセスノイズ
float get_imu_kf_q_bias();  // バイアスプロセスノイズ
float get_imu_kf_R();       // 気圧高度観測ノイズ [m²]

#endif // IMU_H
