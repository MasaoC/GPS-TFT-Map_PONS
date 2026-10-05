#pragma once
// ============================================================
// ★★ **このファイルは実際には使われていない。** 編集しても効果が無い。
//   attitude.cpp の `#include "settings.h"` は引用符形式なので、コンパイラは
//   まず **attitude.cpp 自身のディレクトリ（リポジトリ直下）** を見る。
//   Makefile の `-I.` はその後なので、常に **直下の settings.h** が使われる。
//   （2026-10-01 に `#error` を仕込んで確認。発火しなかった）
//   つまりテストは実機と同じ定数で走っている ＝ それ自体は望ましい状態。
//   このファイルは消してもビルドは通るが、履歴のため残してある。
//   **定数を変えたいときは直下の settings.h を直すこと。**
// ============================================================
#define IMULOG_RAW_REPORTS_ENABLED 1
#define IMU_RATE_GYRO_HZ 50
#define ESKF_SIGMA_G    2e-3f
#define ESKF_SIGMA_A    3e-2f
#define ESKF_SIGMA_BG   1e-5f
#define ESKF_SIGMA_BA   1e-4f
#define ESKF_MAX_GYRO_BIAS   0.01745f
#define ESKF_MAX_ACCEL_BIAS  1.0f
#define ESKF_SACC_MIN   0.05f
#define ESKF_SACC_MAX   1.0f
#define ESKF_STATIC_GYRO_RADS   0.026f
#define ESKF_STATIC_ACCEL_TOL   0.30f
#define ESKF_STATIC_INIT_US     2000000UL
#define ESKF_INIT_ATT_SIGMA_DEG         3.0f
#define ESKF_INIT_ATT_SIGMA_MOVING_DEG  30.0f


// BNO085 のマウント回転（実機 settings.h と同じ値にすること）。
// センサー Y 軸まわり -90 度。測定の根拠は実機側 settings.h のコメント参照。
#define IMU_MOUNT_QW   0.70710678f
#define IMU_MOUNT_QX   0.0f
#define IMU_MOUNT_QY  (-0.70710678f)
#define IMU_MOUNT_QZ   0.0f
#define ESKF_YAW_SIGMA_MAX_DEG 180.0f
// GNSS 速度観測の許容鮮度 [µs]。これを超えたら自動ロールトリムと風推定を止める。
// GNSS は 2Hz なので 3 秒 = 6 回分の欠測。短い遮蔽では止まらず、
// 「そもそも測位していない」状態だけを確実に弾く長さ。
#define ESKF_GNSS_VEL_TIMEOUT_US 3000000UL
