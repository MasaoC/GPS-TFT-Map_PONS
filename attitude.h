// ============================================================
// File    : attitude.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS 速度援用の姿勢 ESKF（機上リアルタイム版）。
//           tools/imulog/eskf.py と同じ数式・同じマウント補正を実装する。
//           片方を直したらもう片方も必ず合わせること。
//
// ■ なぜ必要か
//   BNO085 内蔵フュージョンは加速度計が測る「比力」を鉛直とみなすため、
//     ・定常旋回中はロールを過小評価する
//       （協調旋回では比力の横成分が厳密に 0 になり、加速度計にバンクが見えない）
//     ・加減速中はピッチがずれる
//   GNSS 速度から対地加速度を与えると、比力から遠心力・加減速分を差し引けるので
//   真の重力方向が求まる。実測（2026-08-18 車載）での効果:
//       単位前後加速度あたりのピッチ誤差  BNO085 4.4〜5.6 → ESKF 0.2〜0.5 deg/(m/s²)
//       単位旋回レートあたりのロール誤差  BNO085 0.12〜0.22 → ESKF 0.01〜0.03 deg/(deg/s)
//
// ■ 座標系（重要）
//   ワールド : ENU（X=East, Y=North, Z=Up）。BNO085 (SH2) のワールド系に合わせてある。
//   ボディ   : BNO085 センサー軸そのもの。
//   q は body → ENU。フィルタ内部でマウントのズレを意識する必要はない。
//   機体軸の roll/pitch/yaw は出力時にマウント補正を通して得る。
//   GNSS は NED なので [E,N,U] = [velE, velN, -velD] に変換して渡す。
//
// ■ 状態
//   公称: q (body→ENU), v_ENU, bg (ジャイロバイアス), ba (加速度バイアス)
//   誤差 12: [dtheta(3), dv(3), dbg(3), dba(3)]
//   誤差回転はワールド系（global error）で定義: R_true = (I + [dtheta]x) R_nominal
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/18
// ============================================================

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <Arduino.h>

// ---- 初期化 ----
void attitude_setup();

// ---- センサー入力（imu.cpp の SH2 コールバックから呼ぶ）----
// ジャイロ到着で predict を回す（加速度は直近値を使う）。単位は rad/s と m/s²。
void attitude_on_gyro(const float g[3], uint32_t t_us);
void attitude_on_accel(const float a[3]);
// BNO085 の GAME_ROTATION_VECTOR。静止中の平均を初期姿勢に使う（磁気なし）。
void attitude_on_grv(float qw, float qx, float qy, float qz);
// BNO085 の ROTATION_VECTOR（地磁気補正あり）。初期ヨーの絶対基準に使う。
// accuracy_rad は BNO085 が報告するヘディング精度推定 [rad]（負なら未キャリブ）。
void attitude_on_rv(float qw, float qx, float qy, float qz, float accuracy_rad);

// ---- GNSS 速度観測（gps.cpp の NAV-PVT 解析から呼ぶ）----
// 引数は UBX 原義の NED（velD は下降正）。内部で ENU へ変換する。
void attitude_on_gnss_velocity(float velN, float velE, float velD, float sAcc);

// ---- 出力 ----
bool attitude_ready();                 // 初期化が完了して推定値が有効か
// 機体軸のオイラー角 [度]。マウント補正とゼロ点オフセットを適用済み。
void attitude_get_euler(float &roll, float &pitch, float &yaw);
// マウント補正のみ適用（ゼロ点オフセット適用前）。較正のときに使う。
void attitude_get_euler_raw(float &roll, float &pitch, float &yaw);

// ---- 診断（設定画面の IMU/ESKF ページ用）----
void  attitude_get_gyro_bias(float b[3]);    // 推定ジャイロバイアス [rad/s]
void  attitude_get_accel_bias(float b[3]);   // 推定加速度バイアス [m/s²]
void  attitude_get_velocity(float v[3]);     // 推定速度 ENU [m/s]
// ヨーの推定標準偏差 [度]。共分散 P の該当対角成分から求める。
// ヨーは水平加速度がある間しか可観測にならないため、等速直進が続くとこの値が育つ。
// 画面表示の信頼度判定に使う（大きいときは方位を信用しない）。
float attitude_get_yaw_sigma_deg();
// ヨー精度の 95% 値 [度]（= 2σ）。表示と判定にはこちらを使う。
// 航空分野の精度表記は 95% が慣例で、1σ のまま「これ以下なら信用してよい」と
// 判断すると 3 回に 1 回は外れることになるため。
float attitude_get_yaw_acc95_deg();

// ---- 30 秒平均ピッチ [度] ----
// フゴイドで瞬時値は±3度振れるので、巡航のトリム状態はこちらで見る。
float attitude_get_pitch_avg_deg();
bool  attitude_pitch_avg_valid();          // 平均が十分たまったか

// ---- 直進中のロール自動トリム ----
// 直進中はロールが 0 のはずなので、ズレをゆっくり戻す（誤警報の連発を防ぐ）。
// 真のバンクを消さないよう、旋回中は一切働かない。
float attitude_get_roll_trim_deg();        // 現在の累積補正量
// 自動トリムの有効/無効（設定画面で切替、SD に保存）。既定は ON。
// OFF にすると累積補正量は 0 に戻る。
bool  attitude_get_roll_trim_enabled();
void  attitude_set_roll_trim_enabled(bool on);
// 補正が入った瞬間に一度だけ true を返す（ログ記録用）。
// applied にその回の補正量、total に累積量が入る。
bool  attitude_take_roll_trim_event(float &applied, float &total);
// 初期ヨーに地磁気（ROTATION_VECTOR）を使えたか。false なら磁気なしで初期化した
// ＝ 起動直後の絶対方位は無意味で、機動して収束するまで待つ必要がある。
bool  attitude_yaw_from_mag();
bool  attitude_is_static();                  // 静止判定の現在値
float attitude_get_static_secs();            // 静止が継続している秒数
uint32_t attitude_get_gnss_updates();        // GNSS 観測を取り込んだ回数

// ---- 機体ゼロ点（マウント基準）の較正 ----
// 「いま機体のピッチは target_pitch_deg である」と申告して、その値になるよう
// オフセットを決める。ロールは常に 0 とみなす（運用上、傾いた状態では較正しない）。
//
//   オフセット = 現在の生の測定値 - target_pitch_deg
//
// 本番プラットホームのように傾いた場所（-3.5 度）でも、そこで -3.5 を申告すれば
// 正しく較正できる。充電などでマウントを外して付け直したあと、
// 期待と違う値が出たときにその場で直せるようにするための機能。
//
// ※ 地上でのみ実行すること。飛行中に呼ぶと傾いた姿勢を基準として焼き付けてしまい、
//   バンク角警告が機能しなくなる（呼び出し側で対地速度による地上判定を必ず入れる）。
void attitude_calibrate_to(float target_pitch_deg);
// 従来どおり「いま水平」として較正する（target 0 度と同じ）。
void attitude_calibrate_level();
void attitude_reset_level();                 // 較正を破棄して既定（オフセット 0）へ戻す
void attitude_get_level_offset(float &roll_deg, float &pitch_deg);
void attitude_set_level_offset(float roll_deg, float pitch_deg);  // SD 設定からの復元用

// ---- 較正時に申告するピッチ角（画面の SET PITCH 行の値）----
// 次に較正するときの目標値。SD に保存して次回起動でも同じ値から始められるようにする。
float attitude_get_pitch_target();
void  attitude_set_pitch_target(float deg);
// 目標値を 1 段進める（範囲の端まで行ったら先頭へ戻る）
void  attitude_cycle_pitch_target();

#endif // ATTITUDE_H
