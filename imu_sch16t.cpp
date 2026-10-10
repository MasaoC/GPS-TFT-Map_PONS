// ============================================================
// File    : imu_sch16t.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : SCH16T-K01 ドライバ。**いまは骨組みだけ**（0.987）。
//           imu_sensor.h の契約を「載っていない」で埋めてあるだけで、
//           SPI も叩かない。姿勢もバリオの加速度項も出ない。
//
// ■ なぜ中身の無いファイルを先に置くのか
//   これがあると PONS_BOARD = PONS_BOARD_V71 で**リンクまで通る**ので、
//   基板が届く前に新基板向けのファーム全体（画面・ログ・無線・音）を
//   ビルドして確かめられる。SCH16T の中身を書くときは、下の関数を
//   埋めていくだけになる。
//
// ■ 実装するときの順番と仕様は docs/imu_sch16t_plan.md
//   §1 … 実装すべき surface（ここにある関数がその全量）
//   §3 … ピン（settings.h の SCH16T_PIN_* が正本）
//   §5 … SPI フレーム・CRC8・初期化手順・読み出しレートの選択
//   フレーム層は tools/sch16t_test/ から持ち込める。
//
// ★ **ファイル全体が #if IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T で囲ってある。**
//   Arduino はスケッチ直下の .cpp を全部コンパイルするので、囲わないと
//   imu_bno08x.cpp と imu_drv_* が多重定義になる。**外さないこと。**
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/10
// ============================================================

#include <Arduino.h>
#include "settings.h"

#if IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T

#include "imu.h"
#include "imu_sensor.h"
#include "mysd.h"     // enqueueTask / createLogSdTask

// ============================================================
//  能力ビット — SCH16T は融合出力を持たない
// ============================================================
// ★ ジャイロと加速度しか出ないので、QUAT / MAGYAW / LINACC / CAL はすべて 0。
//   これを 0 にしてあるおかげで、画面とログは自動的に "---" へ縮退する
//   （囲ってある箇所の一覧は docs/imu_sch16t_plan.md §2.5）。
// ★ IMU_CAP_STATUS は**本物のドライバと同時に立てる**。いま立てると
//   「故障ビットを読めている」という嘘になる（S1:S0 / CE はまだ読んでいない）。
uint32_t imu_caps() { return 0; }

const char* imu_sensor_name() { return "SCH16T"; }

// ============================================================
//  imu_drv_* — すべて「載っていない」を返す
// ============================================================
// ★ imu_drv_present() が false なので、呼び出し側（imu.cpp）は恒速モデルへ落ちる。
//   MS5611 があれば気圧ベースのバリオとして動き、姿勢は出ない。
//   起動画面のセンサー行も NG になるので、骨組みのまま焼いても見れば分かる。
void imu_drv_setup() {
    enqueueTask(createLogSdTask("SCH16T driver is a STUB (imu_sch16t.cpp). no attitude."));
    DEBUGW_PLN(20261010, "[IMU] SCH16T driver stub — not implemented");
}

ImuDrvState imu_drv_poll()      { return IMU_DRV_DEAD; }
void        imu_drv_service_if_due() { }
bool        imu_drv_present()   { return false; }
bool        imu_drv_alive()     { return false; }

// VARIO_USE_RAW_ACCEL=0 の旧経路用。SCH16T は LACC を持たないので常に false。
bool imu_drv_lacc_predict_sample(float a_body[3], float q[4], uint32_t* lacc_last_us) {
    (void)a_body; (void)q; (void)lacc_last_us;
    return false;
}

// ============================================================
//  融合出力のゲッター — 無いので「無効値」を返す
// ============================================================
// ★ **0 をそのまま返さないものがある。** get_imu_euler() の 0,0,0 は
//   「水平で北向き」というもっともらしい値なので、呼び出し側が imu_caps() を
//   見ずに表示すると静かな誤表示になる。いまは全箇所が囲ってあるため出ないが、
//   **囲い忘れに備えて参照は必ず埋める**（未初期化のまま返さない）。
void get_imu_euler(float &roll, float &pitch, float &yaw) {
    roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
}
void get_imu_linaccel(float &ax, float &ay, float &az) {
    ax = 0.0f; ay = 0.0f; az = 0.0f;
}
float get_imu_mag_accuracy_deg() { return -1.0f; }   // -1 = 未受信（BNO085 と同じ約束）

// レート [Hz]。融合レポートは存在しないので 0。
// ★ 生ジャイロ・生加速度のレート（get_imu_gyro_hz / get_imu_accel_hz）は
//   **本物のドライバで必ず埋めること。** あの 2 つは能力ビットで囲われていない
//   （ジャイロと加速度は全チップ必須なので）ため、0 のままだと 60 秒ログの
//   raw GYR=0.0 ACC=0.0 が「飢餓」に見える。
float get_imu_grv_hz()   { return 0.0f; }
float get_imu_lacc_hz()  { return 0.0f; }
float get_imu_rv_hz()    { return 0.0f; }
float get_imu_gyro_hz()  { return 0.0f; }
float get_imu_accel_hz() { return 0.0f; }

// ============================================================
//  校正と申告精度 — SCH16T には無い
// ============================================================
// ★ 自前のジャイロ ZRO 学習（attitude.cpp の learn_gyro_bias）が代わりを務める。
//   フラッシュには焼かない（理由は docs/imu_sch16t_plan.md §2.6）。
bool     imu_cal_ready()       { return false; }
bool     imu_cal_saved_ever()  { return false; }
uint32_t imu_cal_saved_date()  { return 0; }
uint16_t imu_cal_save_count()  { return 0; }
bool     imu_cal_autosave_on() { return false; }
bool     imu_save_calibration(uint32_t yyyymmdd) { (void)yyyymmdd; return false; }

uint8_t  get_imu_acc_accuracy() { return 0; }
uint8_t  get_imu_gyr_accuracy() { return 0; }
uint8_t  get_imu_cal_cfg()      { return 0xFF; }     // 0xFF = 未取得（BNO085 と同じ約束）

// ============================================================
//  健全性カウンタ — 中身は本物のドライバで差し替える
// ============================================================
// ★ SCH16T では意味が変わる。GRV/RV は無いので grv_bad / grv_jump / rv_bad は
//   そのまま 0 でよいが、**CRC エラー数・S1:S0 の故障ビット・飽和回数**を
//   出す枠として作り直すこと（docs/imu_sch16t_plan.md §2.7）。
uint32_t get_imu_grv_bad()  { return 0; }
uint32_t get_imu_grv_jump() { return 0; }
uint32_t get_imu_rv_bad()   { return 0; }

// Core0 が止まった最長時間 [µs]。**SCH16T でも有用なので本物のドライバで埋めること**
// （SPI を叩けていない時間が見える。計画書 §2.7）。読むたびに 0 へ戻す約束。
uint32_t get_imu_poll_gap_max_us() { return 0; }

#endif  // IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T
