// ============================================================
// File    : imu.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : バリオ用 3 状態 Kalman フィルターと、推定側のゲッター。
//           **チップ非依存。** BNO085 固有のコードは imu_bno08x.cpp にある。
//
//   0.986 で 1918 行から BNO085 依存部を切り出した（docs/imu_sch16t_plan.md §1）。
//   チップ側とのやり取りは imu_sensor.h の imu_drv_* だけ。
//
// ■ アーキテクチャ
//   - imu_update() を毎ループ呼ぶ。中で imu_drv_poll() を 1 回進め、
//     戻り値（DEAD / WAIT / SERVICED）で Kalman predict へ進むかを決める
//   - サンプル到着時 → predict（状態を比力で時間更新）
//   - 気圧高度到着時 → update（get_airdata_altitude() で観測修正）
//
// ■ 座標系
//   imu_feed_accel() が受け取るのはセンサー軸の比力（重力込み）。
//   「上方向」は姿勢 ESKF（attitude_get_up_sensor）から取るので、
//   **チップの融合出力には依存しない**（0.983 で GRV 依存を外した）。
//
// ■ Kalman フィルター 状態 x = [z, z_dot, b]
//   z     : 推定高度 [m]
//   z_dot : 推定上昇率 [m/s]
//   b     : 加速度バイアス [m/s²]（ゆっくり変化するドリフト）
//
//   予測ステップ（IMU加速度 a_k で駆動）:
//     F = [[1, dt,  0],
//          [0,  1, -dt],
//          [0,  0,   1]]
//     B = [dt²/2, dt, 0]^T
//     x_pred = F*x + B*a_k
//     P_pred = F*P*F^T + Q   (Q = diag([0, q_vel, q_bias]))
//
//   観測ステップ（気圧高度 z_baro で更新）:
//     H = [1, 0, 0]
//     K = P*H^T / (H*P*H^T + R)
//     x = x_pred + K*(z_baro - H*x_pred)
//     P = (I - K*H)*P  （+ 対称化処理）
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/10
// ============================================================

#include <Arduino.h>

// ★ **Adafruit_BNO08x.h / EEPROM.h / imulog.h はここには無い。**
//   どれも BNO085 固有なので imu_bno08x.cpp 側にある。
//   ここへ戻すと「チップ非依存」という前提が静かに崩れる。
#include "imu.h"
#include "imu_sensor.h"   // チップ境界（能力ビット・feed の口・imu_drv_*）
#include "link.h"
#include "settings.h"
#include "airdata.h"  // airdata_* （気圧計は i2c0。IMU のバスとは別系統）
#include "mysd.h"     // enqueueTask / createLogSdfTask
#include "gnss.h"      // replay_has_value / replay_get_* （リプレイ時のセンサ値差し替え）
#include "attitude.h" // 姿勢 ESKF（機上リアルタイム版）


#if VARIO_USE_RAW_ACCEL
// ============================================================
// 生比力から作る鉛直加速度の蓄積バッファ（VARIO_USE_RAW_ACCEL=1 用）
// ============================================================
// SH2_ACCELEROMETER は 50Hz で届くが kf_predict() は 30ms 周期でしか回らない。
// 「最新の 1 サンプル」だけを使うと届いた 1.5 個に 1 個を捨てることになり、
// 振動成分がエイリアシングして推定上昇率のノイズになる。
// そこで受信のたびにここへ足し込み、predict のときに平均を取り出して使う。
// （実ログでの静止時σ: 最新値 0.153 → 平均 0.090。動きのある区間でも平均の方が良い）
//
// imu_sensor_handler() は sh2_service() の中から Core0 で呼ばれるので、
// imu_update() の predict と同じコアであり排他は不要。
static float    _acc_vsum = 0.0f;   // 鉛直加速度の合計 [m/s²]（重力除去済み）
static float    _acc_hsum = 0.0f;   // 水平加速度の大きさの合計 [m/s²]
static uint16_t _acc_cnt  = 0;      // 合計したサンプル数
static uint32_t _acc_last_us = 0;   // 最後に ACCELEROMETER を受信した時刻 [µs]（鮮度チェック用）
static bool     _accel_valid = false;

// ★ 0.983 で imu_sensor_handler() が compute_earth_z_accel() を呼ばなくなった
//   （上方向を ESKF から取るようになった）ので前方宣言は要らない。
//   関数自体は VARIO_USE_RAW_ACCEL=0 の経路（LINEAR_ACCELERATION）が使う。
#endif

// ============================================================
// Kalman フィルター 内部状態
// ============================================================

// 鮮度不足で predict をスキップした回数（診断用。正常時は 0 のまま）。
// ★ predict 側の計数なのでチップ非依存。0.986 の分割でここへ置いた。
static volatile uint32_t _lacc_stale_skips = 0;

// 状態ベクトル x = [z, z_dot, b]
static float kf_x[3] = { 0.0f, 0.0f, 0.0f };

// 誤差共分散行列 P（3x3, 対称行列として管理する）
// 対角成分の初期値: z=100m², z_dot=1(m/s)², b=0.1(m/s²)²
static float kf_P[3][3] = {
    { 100.0f,  0.0f,  0.0f },
    {   0.0f,  1.0f,  0.0f },
    {   0.0f,  0.0f,  0.1f }
};

// 最初の気圧高度を受け取ったときに x[0] を初期化する（それ以前は高度不明）
static bool kf_initialized = false;

// 前回の predict ステップを実行した時刻 [µs]（dt 計算に使用）
static uint32_t kf_last_predict_us = 0;

// ============================================================
// Kalman ノイズパラメーター（settings.h の定数で初期化）
// ============================================================
static float kf_q_vel  = KF_Q_VEL;   // 速度プロセスノイズ
static float kf_q_bias = KF_Q_BIAS;  // バイアスプロセスノイズ
static float kf_R      = KF_R;       // 気圧高度観測ノイズ [m²]

// ============================================================
// 出力値（volatile: 別コアから参照される可能性がある）
// ============================================================
// ARM Cortex-M33 (RP2350) は 32bit アライメント済みの float read/write がアトミックなため、
// volatile float で Core 間の読み出し競合を安全に扱える。
static volatile float _imu_vspeed   = 0.0f;  // Kalman 推定上昇率 [m/s]
static volatile float _imu_altitude = 0.0f;  // Kalman 推定高度 [m]
static volatile float _imu_az           = 0.0f;  // 地球座標系 鉛直加速度 [m/s²]（デバッグ用）
static volatile float _imu_horiz_accel  = 0.0f;  // 地球座標系 水平加速度の大きさ [m/s²]（KF Q_vel 動的増幅・表示用）





// 生ジャイロ・生加速度の直近値（設定画面の IMU/ESKF ページ表示用）。
// 静止しているデバイスが実際どんな値を出しているかを目視確認するために持つ。
static volatile float _raw_gyro[3]  = {0, 0, 0};   // [rad/s]
static volatile float _raw_accel[3] = {0, 0, 0};   // [m/s²]



// ============================================================
//  ドライバ → 推定 への口（imu_sensor.h の契約）
// ============================================================
// ★ **チップ固有のコードが推定へ触るのはこの 2 つだけ。** SCH16T の
//   ドライバを足すときも、ここを正しいレートで呼べば表示・音・記録が動く。
//   生ログの push はチップ側に残してある（理由は imu_sensor.h）。

void imu_feed_gyro(const float g[3], uint32_t t_us) {
    _raw_gyro[0] = g[0]; _raw_gyro[1] = g[1]; _raw_gyro[2] = g[2];
    // ESKF の伝播はジャイロ到着で回す（加速度は直近値を使う）。
    // ※ チップのタイムスタンプは使わない。BNO085 の sv.timestamp は sh2 の
    //   アンダーフローで壊れている。バーストで届く分は attitude.cpp が窓平均で吸収する。
    attitude_on_gyro(g, t_us);
}

// ドライバがリセット／復旧したときに呼ばれる（imu_sensor.h の契約）。
void imu_feed_reset() {
#if VARIO_USE_RAW_ACCEL
    _accel_valid = false;
    _acc_vsum = _acc_hsum = 0.0f;
    _acc_cnt  = 0;
#endif
}

void imu_feed_accel(const float a[3]) {
    _raw_accel[0] = a[0]; _raw_accel[1] = a[1]; _raw_accel[2] = a[2];
    attitude_on_accel(a);
#if VARIO_USE_RAW_ACCEL
    // ---- バリオ KF 用: 地球座標系の鉛直/水平加速度をここで作って足し込む ----
    // 重力込みの比力を「上方向」へ射影し、重力を引く。
    // BNO085 の LINEAR_ACCELERATION と違い、引く重力が固定値なので
    // 「動作に相関した誤差」が入らない（settings.h の VARIO_USE_RAW_ACCEL 参照）。
    //
    // ★ **0.983 で上方向の出どころを GRV から ESKF の姿勢へ移した。** 式は変えていない
    //   （attitude_get_up_sensor() が返すのは回転行列の第 3 行で、GRV から
    //   compute_earth_z_accel() が作っていた係数と同じもの）。実ログでの差は
    //   V/S の sd 0.012〜0.060 m/s・高度 |max| 0.42m で、最大差は GNSS 断の瞬間に出る。
    //   → docs/imu_sch16t_plan.md §2.3
    float u_up[3];
    if (attitude_get_up_sensor(u_up)) {
        float az_world = u_up[0]*a[0] + u_up[1]*a[1] + u_up[2]*a[2];
        // 水平成分: 二乗ノルムは回転で不変なので |a|² − a_world_z²。
        // ここでの a_world_z は重力を引く前の値である点に注意。
        float body_sq  = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
        float horiz_sq = body_sq - az_world * az_world;
        _acc_vsum += az_world - GRAVITY_MPS2;
        _acc_hsum += (horiz_sq > 0.0f) ? sqrtf(horiz_sq) : 0.0f;
    } else {
        // ★★ **上方向が無くても predict を止めてはいけない。**
        //   バリオ KF の初期 P は対角なので、**predict を一度も通らないと
        //   P[1][0] が 0 のままで、気圧観測のゲイン K[1] = P[1][0]/S が 0 に
        //   なり、速度状態（＝ V/S）が構造的に 1 度も動かない。**
        //   高度 kf_x[0] だけは K[0] で追従するので、症状は
        //   「センサーは生きているのに V/S だけ 0 で固まる」になる。
        //   2026-10-01 に実機で発覚（0.983 で attitude_get_up_sensor() が
        //   水平化前に false を返す作りにしたのが原因）。
        //   ここは加速度ゼロ＝恒速モデルとして回す。IMU なし時の
        //   kf_predict(0.0f, dt) と同じ縮退で、気圧ベースのバリオになる。
        _acc_vsum += 0.0f;
        _acc_hsum += 0.0f;
    }
    if (_acc_cnt < 60000) _acc_cnt++;   // 念のための飽和ガード
    _acc_last_us = time_us_32();
    _accel_valid = true;
#endif
}





// ============================================================
// 内部ヘルパー: ボディフレームの加速度を地球座標系の Z 成分に変換
// ============================================================
// BNO085 の GAME_ROTATION_VECTOR クォータニオン q = (qw, qx, qy, qz) を使って、
// ボディフレームのベクトル a = (ax, ay, az) を世界座標系に回転させたときの
// Z 成分（上向き正）を回転行列の第3行から直接計算する。
//
// 回転行列 R (ボディ→ワールド) の第3行:
//   R[2][0] = 2*(qx*qz - qw*qy)
//   R[2][1] = 2*(qy*qz + qw*qx)
//   R[2][2] = 1 - 2*(qx² + qy²)
//
// BNO085 LINEAR_ACCELERATION はすでに重力を除去しているため、
// このゼロ点は重力加速度 g を引く必要がない。
static float compute_earth_z_accel(float ax, float ay, float az,
                                   float qw, float qx, float qy, float qz) {
    return  2.0f * (qx * qz - qw * qy) * ax
          + 2.0f * (qy * qz + qw * qx) * ay
          + (1.0f - 2.0f * (qx * qx + qy * qy)) * az;
}


// ============================================================
// Kalman 予測ステップ（IMU 加速度で状態を時間更新する）
// ============================================================
// a_k              : 地球座標系の鉛直加速度 [m/s²]（重力除去済み）
// dt               : 前回 predict からの経過時間 [s]
// horiz_accel_m_s2 : ボディフレーム水平加速度の大きさ [m/s²]（IMU なし時は 0）
//                    水平加速度が大きいほど IMU の垂直加速度推定に誤差が混入するため、
//                    Q_vel を KF_HORIZ_ACCEL_GAIN × horiz_accel² 分だけ動的に増幅する。
//
// 状態遷移: x_pred = F*x + B*a_k
//   F = [[1, dt, 0], [0, 1, -dt], [0, 0, 1]]
//   B = [dt²/2, dt, 0]^T
//
// 共分散伝搬: P_pred = F*P*F^T + Q
//   Q = diag(0, q_vel_eff, kf_q_bias)
//   q_vel_eff = kf_q_vel + KF_HORIZ_ACCEL_GAIN * horiz_accel²
static void kf_predict(float a_k, float dt, float horiz_accel_m_s2 = 0.0f) {
    // ---- 状態予測 ----
    float z_pred    = kf_x[0] + kf_x[1] * dt;
    float vz_pred   = kf_x[1] + (a_k - kf_x[2]) * dt;
    float bias_pred = kf_x[2];  // バイアスはランダムウォーク: 定数モデル
    kf_x[0] = z_pred;
    kf_x[1] = vz_pred;
    kf_x[2] = bias_pred;

    // ---- 共分散予測: P_pred = F*P*F^T + Q ----
    // Step1: A = F * P  (F の各行で P の行を線形結合する)
    float A[3][3];
    for (int j = 0; j < 3; j++) {
        A[0][j] = kf_P[0][j] + dt * kf_P[1][j];       // F[0] = [1, dt, 0]
        A[1][j] = kf_P[1][j] - dt * kf_P[2][j];       // F[1] = [0, 1, -dt]
        A[2][j] = kf_P[2][j];                           // F[2] = [0, 0, 1]
    }
    // Step2: P_new = A * F^T  (F^T の各列 = F の各行)
    // F^T = [[1,0,0],[dt,1,0],[0,-dt,1]]  なので列ベクトルは [1,dt,0], [0,1,-dt], [0,0,1]
    for (int i = 0; i < 3; i++) {
        kf_P[i][0] = A[i][0] + dt * A[i][1];           // A * F^T の列0
        kf_P[i][1] = A[i][1] - dt * A[i][2];           // A * F^T の列1
        kf_P[i][2] = A[i][2];                           // A * F^T の列2
    }
    // Step3: Q を加算  (Q = diag([0, q_vel_eff, q_bias]))
    // 水平加速度が大きいとき IMU の鉛直加速度精度が低下するため、
    // KF_HORIZ_ACCEL_GAIN × horiz_accel² を上乗せして IMU への依存を自動的に弱める。
    float q_vel_eff = kf_q_vel + KF_HORIZ_ACCEL_GAIN * horiz_accel_m_s2 * horiz_accel_m_s2;
    kf_P[1][1] += q_vel_eff;
    kf_P[2][2] += kf_q_bias;
}


// ============================================================
// Kalman 観測更新ステップ（気圧高度で状態を修正する）
// ============================================================
// z_baro_m : MS5611 から得られた気圧高度 [m]（グランドレベル相対）
//
// 観測モデル: H = [1, 0, 0]（高度のみ観測する）
// イノベーション: v = z_baro - H*x = z_baro - x[0]
// カルマンゲイン: K = P*H^T / (H*P*H^T + R) = P[:,0] / (P[0][0] + R)
// 状態更新:       x = x + K * v
// 共分散更新:     P = (I - K*H) * P  （K*H は rank-1 行列）
void imu_kalman_baro_update(float z_baro_m) {
    // IMU の有無によらず気圧高度で KF を初期化・更新する。
    // IMU なし時は imu_update() の恒速 predict と組み合わせてバリオとして動作。

    // 初回: 気圧高度で状態を初期化する（初期値のない状態で始めると発散するため）
    if (!kf_initialized) {
        kf_x[0] = z_baro_m;
        kf_x[1] = 0.0f;
        kf_x[2] = 0.0f;
        kf_last_predict_us = time_us_32();
        kf_initialized = true;
        DEBUG_PLN(20260315, "[IMU] Kalman initialized with baro altitude");
        return;
    }

    // ---- カルマンゲイン計算 ----
    // H = [1,0,0] なので H*P*H^T = P[0][0]
    float S = kf_P[0][0] + kf_R;
    if (S < 1e-9f) return;  // ゼロ除算ガード（異常値防止）

    float K[3] = {
        kf_P[0][0] / S,  // K[0] = P[0][0] / S
        kf_P[1][0] / S,  // K[1] = P[1][0] / S  ← 気圧が来るたびに速度を補正する主経路
        kf_P[2][0] / S   // K[2] = P[2][0] / S
    };

    // ---- 状態更新 ----
    float innov = z_baro_m - kf_x[0];  // イノベーション（残差）
    kf_x[0] += K[0] * innov;
    kf_x[1] += K[1] * innov;
    kf_x[2] += K[2] * innov;

    // ---- 共分散更新: P = (I - K*H) * P ----
    // K*H は rank-1 行列で、第0列が K に等しく残りは 0。
    // P_new[i][j] = P[i][j] - K[i] * (H * P)[j] = P[i][j] - K[i] * P[0][j]
    // ⚠ i=0 のとき kf_P[0][j] を使いながら書き換えるため、第0行を先にコピーする。
    float P0[3] = { kf_P[0][0], kf_P[0][1], kf_P[0][2] };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf_P[i][j] -= K[i] * P0[j];
        }
    }

    // ---- 対称化（数値誤差の蓄積を防ぐ）----
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float avg = (kf_P[i][j] + kf_P[j][i]) * 0.5f;
            kf_P[i][j] = kf_P[j][i] = avg;
        }
    }

    // ---- 対角成分の下限クランプ（P が負定値になる数値的異常を防ぐ）----
    for (int i = 0; i < 3; i++) {
        if (kf_P[i][i] < 1e-9f) kf_P[i][i] = 1e-9f;
    }

    // ---- 出力を更新 ----
    _imu_altitude = kf_x[0];
    _imu_vspeed   = kf_x[1];
}


// ============================================================
// GNSS高度による気圧基準補正
// ============================================================
// GNSSの絶対高度（MSL）を使って、気圧の基準（ground_alt_abs）をゆっくり修正する。
//
// ■ なぜ kf_x[0] を直接書き換えないのか
//   気圧観測は ~40Hz・R=4m² で KF に入力されるため、kf_x[0] を書き換えても
//   数十ms後には気圧の観測値に上書きされてしまう。
//   代わりに airdata.cpp の ground_alt_abs を微調整することで、
//   気圧センサー自体の出力をシフトさせ、KF が自然に新しい基準に収束する。
//
// ■ 参照フレームの変換
//   GNSS高度は MSL 基準。KF高度は起動地点を 0m とした相対高度（AGL）。
//   起動時に 10 サンプルを平均して「GNSS MSL - KF AGL」= gnss_kf_offset を推定する。
//   これが起動地点の MSL 高度（標準大気換算）に相当する。
//
// ■ 補正ロジック（フェーズ2以降）
//   z_gnss_kf = z_gnss_msl - gnss_kf_offset      // GNSS→KFフレーム変換
//   innov     = z_gnss_kf - kf_x[0]              // 高度誤差
//   quality   = 1 - vacc_m / GNSS_VACC_MAX_M     // 精度係数 [0, 1]
//   delta     = clamp(innov * quality * GNSS_CORRECT_RATE, ±GNSS_MAX_DELTA_M)
//   ground_alt_abs -= delta  （innov>0 → 基準を下げてKF高度を上げる方向）
//
// ■ Varioへの影響
//   delta 最大 GNSS_MAX_DELTA_M [m] / 更新 ≒ 0.02m/s。表示分解能（0.1m/s）以下。

// GNSS補正の内部状態
static float  _gnss_kf_offset     = 0.0f;   // GNSS MSL高度 - KF AGL高度 の推定値 [m]
static float  _gnss_offset_sum    = 0.0f;   // 初期化用積算
static int    _gnss_offset_n      = 0;      // 初期化サンプル数
static bool   _gnss_offset_ready  = false;  // 初期化済みフラグ
static uint32_t _gnss_last_ms     = 0;      // 最後に補正を実行したシステム時刻 [ms]

void imu_kalman_gnss_update(float z_gnss_msl, float vacc_m) {
    if (!kf_initialized) return;  // IMU の有無によらず動作する（kf_initialized のみチェック）
    // 精度が閾値以下、または異常値の場合は無視する
    if (vacc_m <= 0.0f || vacc_m > GNSS_VACC_MAX_M) return;
    if (isnan(z_gnss_msl) || isinf(z_gnss_msl)) return;

    // 約1秒ごとに1回だけ実行する（ループ頻度によらず補正レートを安定させる）
    uint32_t now_ms = millis();
    if (now_ms - _gnss_last_ms < 1000) return;
    _gnss_last_ms = now_ms;

    // ---- フェーズ1: gnss_kf_offset の初期化 ----
    // GNSS MSL高度 - KF AGL高度 ≈ 起動地点の MSL 高度。初回 N サンプルの平均で推定する。
    if (!_gnss_offset_ready) {
        _gnss_offset_sum += z_gnss_msl - kf_x[0];
        _gnss_offset_n++;
        if (_gnss_offset_n >= GNSS_INIT_SAMPLES) {
            _gnss_kf_offset   = _gnss_offset_sum / _gnss_offset_n;
            _gnss_offset_ready = true;
            enqueueTask(createLogSdfTask(
                "[IMU] GNSS alt offset init: %.1f m (MSL-KF, %d samples)",
                _gnss_kf_offset, _gnss_offset_n));
        }
        return;  // 初期化中は補正しない
    }

    // ---- フェーズ2: 気圧基準補正（AGL精度の維持）----
    // GNSS高度を KF フレーム（起動地 0m 基準）に変換してイノベーションを計算する
    float z_gnss_kf = z_gnss_msl - _gnss_kf_offset;
    float innov     = z_gnss_kf - kf_x[0];

    // 50m を超える乖離はGNSSの一時的な外れ値（マルチパス等）として無視する
    if (fabsf(innov) > 50.0f) return;

    // 精度係数: vAcc が小さいほど 1 に近く、GNSS_VACC_MAX_M に近づくほど 0 になる
    float quality = 1.0f - (vacc_m / GNSS_VACC_MAX_M);

    // 補正量: イノベーション × 精度係数（vAcc依存）× ゲイン、最大補正量でクランプ
    float delta = innov * quality * GNSS_CORRECT_RATE;
    if      (delta >  GNSS_MAX_DELTA_M) delta =  GNSS_MAX_DELTA_M;
    else if (delta < -GNSS_MAX_DELTA_M) delta = -GNSS_MAX_DELTA_M;

    // ground_alt_abs を調整して気圧基準を補正する（KF はそれに追従する）
    // delta>0 (GNSSが高い): ground_alt_abs を下げてバロ相対高度を上げる
    airdata_adjust_ground_alt(-delta);

    // ---- フェーズ3: gnss_kf_offset の長期更新（KF MSL絶対高度の精度向上）----
    // GNSS_MSL - KF_AGL = 起動地MSL高度の推定値。vAcc依存レートでゆっくり更新することで
    // 初期fix品質が低かった場合でも長時間後には正確な絶対高度に収束する。
    // 飛行中でも正しく機能: GNSS_MSL - KF_AGL = 定数（起動地MSL高度）は高度によらず不変。
    float msl_est = z_gnss_msl - kf_x[0];
    float alpha   = quality * GNSS_OFFSET_UPDATE_RATE;
    _gnss_kf_offset += (msl_est - _gnss_kf_offset) * alpha;
}


// ============================================================
// imu_kalman_gnss_vel_update(): GNSS 垂直速度による Kalman 速度観測更新
// ============================================================
// GNSS velD（上昇正、m/s）を速度観測として KF の z_dot（x[1]）を補正する。
//
// 観測モデル: H = [0, 1, 0]（速度のみ観測する）
// イノベーション: innov = veld_mps - x[1]
// R_vel = sAcc² × GNSS_VSI_R_SCALE [m²/s²]
// ゲートは sAcc のみ（vAcc=垂直位置精度 は速度品質の指標として不適切なため使用しない）。
//
// IMU の有無によらず動作する（kf_initialized のみチェック）。
//
//   veld_mps  : GNSS 垂直速度 [m/s]（上昇正）= get_gnss_veld_mps()
//   vacc_m    : 垂直位置精度 [m]  — 本関数では使用しない（呼び出し元の互換性維持のため残す）
//   sacc_mps  : 速度精度 [m/s]   — ゲート判定と観測ノイズ R の算出に使用
// ============================================================
void imu_kalman_gnss_vel_update(float veld_mps, float vacc_m, float sacc_mps) {
    (void)vacc_m;  // 使用しない（引数互換性維持）
    if (!kf_initialized) return;

    // ---- ★★ 同じ測位を 2 回融合しないこと ----
    // **呼び出し元（GPS_TFT_map.ino）のブロックは「新しい測位が来たとき」ではなく
    //   loop() の毎回を通る。** imu_kalman_gnss_update() は内部に 1 秒のレート制限を
    //   持っているので守られているが、こちらには何も無かった。その結果 2Hz の
    //   1 サンプルが毎ループ **独立な観測として再融合**されていた。
    //   ループ回数は直接は測っていないが、生ログからの再現が合う倍率は
    //   1000〜2000 回/秒。つまり 1 つの観測を 500〜1000 回使っていた。
    //
    // 同一観測の多重融合は共分散を不当に潰す。実機 2026-09-30 のログを生データから
    // 再現して測った値（設計どおり 1 回だけ融合した場合の定常値と比較）:
    //   P[0][0]  3.08 → 0.15 (1/21)   気圧のゲイン K[0] 0.257 → 0.012
    //   P[1][1]  1.78 → 0.025 (1/71)  気圧から速度への補正 K[1] は実質ゼロ
    // つまり **気圧計と加速度計がほとんど効かず、V/S は実質 GNSS velD そのもの**に
    // なっていた。ふだんは GNSS velD が良いので問題に見えないが、
    // **sAcc ゲートが閉じた瞬間に速度も高度も何の観測でも止められなくなり**、
    // 加速度計バイアスの誤差をそのまま積分して走り出す。
    //   2026-09-30 18:09:18 実機: 停止中（GS=0.0・気圧 1014.65hPa で完全に平坦）に
    //   V/S が +1.6m/s まで伸び、KF 高度が +4m ずれたまま残った。
    //   sAcc が 0.30 を挟んで振動してゲートが 6 秒閉じたのが引き金。
    //   生ログからの再現で、この多重融合を入れたときだけ一致する（相関 0.97、
    //   1 回だけ融合した場合は 0.81）。
    //
    // 判定は millis() ではなく **iTOW（測位エポック）** で行う。ループが速いときに
    // 時間で区切ると同じ観測を取り込む余地が残るため。
    {
        static uint32_t s_last_itow_ms = 0xFFFFFFFFu;
        const uint32_t itow_ms = get_gnss_itow_ms();
        if (itow_ms == s_last_itow_ms) return;   // この測位はもう融合済み
        s_last_itow_ms = itow_ms;
    }

    // 無効値チェック
    if (isnan(veld_mps) || isinf(veld_mps)) return;

    // sAcc ゲート: 速度精度が閾値以上のデータは信頼性が低いためスキップ
    if (sacc_mps >= GNSS_VSI_SACC_MAX_MPS) return;

    // 観測ノイズ分散 R_vel = sAcc² × GNSS_VSI_R_SCALE [m²/s²]
    // sAcc は u-blox の速度精度推定（1-sigma）。Kalman の R は分散 = σ² なので sAcc² が理論値。
    // R_SCALE でさらに倍率をかけて影響度を調整する。
    float r_vel = sacc_mps * sacc_mps * GNSS_VSI_R_SCALE;
    if (r_vel < 0.001f) r_vel = 0.001f;  // 下限クランプ（ゼロ除算防止）

    // H = [0, 1, 0] → S = H*P*H^T + R = P[1][1] + R_vel
    float S = kf_P[1][1] + r_vel;
    if (S < 1e-9f) return;

    // Kalman ゲイン K = P * H^T / S = [P[0][1], P[1][1], P[2][1]] / S
    float K[3] = { kf_P[0][1] / S, kf_P[1][1] / S, kf_P[2][1] / S };

    // イノベーション（観測残差）
    float innov = veld_mps - kf_x[1];

    // 状態更新: x = x + K * innov
    kf_x[0] += K[0] * innov;
    kf_x[1] += K[1] * innov;
    kf_x[2] += K[2] * innov;

    // 共分散更新: P = (I - K*H)*P
    // H=[0,1,0] → K*H の各行 i は [0, K[i], 0]
    // (K*H)*P の (i,j) 要素 = K[i] * P[1][j]
    float P_row1[3] = { kf_P[1][0], kf_P[1][1], kf_P[1][2] };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            kf_P[i][j] -= K[i] * P_row1[j];

    // 数値安定化: 対称化と対角下限クランプ
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float avg = (kf_P[i][j] + kf_P[j][i]) * 0.5f;
            kf_P[i][j] = kf_P[j][i] = avg;
        }
        if (kf_P[i][i] < 1e-9f) kf_P[i][i] = 1e-9f;
    }

    // 出力更新
    _imu_altitude = kf_x[0];
    _imu_vspeed   = kf_x[1];
}














// Kalman 推定上昇率を返す。
// I2C エラー等で 1 秒以上データが途絶えた場合は 0 を返す（バリオ誤鳴動防止）。
// 1 秒 = 30Hz × 33 サンプル分のタイムアウト。正常時は ~33ms ごとに更新される。
// ミラー中（受信モード）は受信した機体の値を返す。
// CSV は自機の値を書く必要があるので、生の実装は get_imu_vspeed_raw() に残してある。
float get_imu_vspeed() {
  if (link_mirror_active()) return link_get_kf_vspeed();
  return get_imu_vspeed_raw();
}

// ============================================================
//  バリオの供給元を 1 か所で決める
// ============================================================
// ★ **ミラー中は「送信機の画面を再現する」のが受信モードの主旨**なので、
//   自機のセンサーの有無で分岐してはいけない。V/S は電波がそのまま運んでいる。
//   0.983 まで draw_vsi() が自機の `get_airdata_ok()` で早期 return しており、
//   **受信機の MS5611 が死んでいるとミラー中もバーが出なかった**。
//   さらに自機の IMU が死んでいると、**ボート自身の気圧 V/S（＝波の上下動）を
//   機体の V/S として表示・発音していた**。
// ★ デッドバンドの広い・狭いは送信機の選び方に合わせる。送信機の IMU の
//   生死は LINK_ST_IMU_ERROR で分かる（link.cpp が `!get_imu_ok()` で立てる）。
//   送信機の MS5611 が死んでいる場合は電波から判別できないが、そのとき送信機の
//   KF は初期化されず V/S は 0 のままなので、バーも音も出ない（結果は一致する）。

// V/S を出せる状態か。気圧が無いと KF が初期化されないので V/S は出ない。
bool vario_source_ok() {
  if (link_mirror_active()) return link_has_value(RHAVE_KFVS);
  return get_airdata_ok();
}

// 表示・音に使う V/S。IMU が途絶していれば MS5611 単独の上昇率へ落とす。
float vario_vspeed_mps() {
  if (link_mirror_active()) return get_imu_vspeed();   // = link_get_kf_vspeed()
  return get_imu_alive() ? get_imu_vspeed() : get_airdata_vspeed();
}

// デッドバンド。KF 融合中（IMU 生存 + MS5611 接続）は精度が高いので狭くする。
float vario_deadband_mps() {
  if (link_mirror_active())
    return (link_sender_status() & LINK_ST_IMU_ERROR) ? VARIO_DEADBAND_BARO_MPS
                                                      : VARIO_DEADBAND_KF_MPS;
  return (get_imu_alive() && get_airdata_ok()) ? VARIO_DEADBAND_KF_MPS
                                               : VARIO_DEADBAND_BARO_MPS;
}

float get_imu_vspeed_raw() {
    // リプレイ中で CSV に KF_Vspeed 列があれば、その値を返す（バリオ音も再生される）
    if (replay_has_value(RHAVE_KFVS)) return replay_get_kf_vspeed();

    // ★ 0.986 の分割でチップの状態はドライバが持つ。条件は分割前と同一:
    //   「載っている」なら生死で判断し、載っていないなら KF の初期化だけを見る。
    if (imu_drv_present()) {
        // センサーあり: I2C エラー等で 1 秒以上データが途絶えた場合は 0 を返す（誤鳴動防止）
        if (!imu_drv_alive()) return 0.0f;
    } else {
        // センサーなし: 気圧 + GNSS 速度による KF 出力を使用。未初期化なら 0 を返す。
        if (!kf_initialized) return 0.0f;
    }
    return _imu_vspeed;
}

// KF MSL高度 = KF AGL + gnss_kf_offset（起動地MSL高度の推定値）。
// gnss_kf_offset 未確定時（GNSS 3D fix 前）は AGL 値（＝起動時 0m）を返す。
// リプレイ中で CSV に KF_Altitude 列があれば、その値をそのまま返す。
// ミラー中（受信モード）は受信した機体の値を返す。
// CSV は自機の値を書く必要があるので、生の実装は get_imu_altitude_msl_raw() に残してある。
float get_imu_altitude_msl() {
  if (link_mirror_active()) return link_get_kf_altitude();
  return get_imu_altitude_msl_raw();
}

float get_imu_altitude_msl_raw() {
    if (replay_has_value(RHAVE_KFALT)) return replay_get_kf_altitude();
    return _imu_altitude + _gnss_kf_offset;
}
bool  get_imu_gnss_offset_ready() { return _gnss_offset_ready; }
float get_imu_az()           { return _imu_az; }
float get_imu_horiz_accel()  { return _imu_horiz_accel; }  // 地球座標系 水平加速度 [m/s²]

// 各センサーイベントの受信レート [Hz]（1 秒ウィンドウで計測）
// 加速度サンプルの鮮度不足で Kalman predict をスキップした累計回数。
// 正常時は 0。増えていれば IMU のサンプル配信が滞っている（レート要求過大など）。
uint32_t get_imu_lacc_stale_skips() { return _lacc_stale_skips; }
// 生ログ用レポートの受信レート（設定値どおり出ていれば飢餓は起きていない）
// 生ジャイロ [rad/s] / 生加速度 [m/s²] の直近値（IMU/ESKF 画面の表示用）
void get_imu_raw_gyro(float g[3])  { for (int i=0;i<3;i++) g[i] = _raw_gyro[i]; }
void get_imu_raw_accel(float a[3]) { for (int i=0;i<3;i++) a[i] = _raw_accel[i]; }

// 新着ROTATION_VECTORフラグ: trueを返し同時にクリア（Euler角ログのトリガー用）
// Kalman パラメーターの動的変更（設定画面からのチューニング用）
void imu_set_kf_params(float q_vel, float q_bias, float R) {
    kf_q_vel  = q_vel;
    kf_q_bias = q_bias;
    kf_R      = R;
}


// Kalman パラメーターのゲッター（SD設定保存用）
float get_imu_kf_q_vel()  { return kf_q_vel; }
float get_imu_kf_q_bias() { return kf_q_bias; }
float get_imu_kf_R()      { return kf_R; }


// ============================================================
//  チップとの境界 — 公開 API はドライバへ委譲するだけ
// ============================================================
// ★ **ここに「どのチップか」の分岐を書かないこと。** 実装はドライバ側の 1 ファイルで、
//   settings.h の IMU_SENSOR_DEFAULT（＝ PONS_BOARD 由来）がファイルごと切り替える。

void imu_setup() { imu_drv_setup(); }

// ★ 長いブロッキング処理の中から呼ぶためのもの（経緯はドライバ側のコメント）。
//   SCH16T では飢餓が起きないので意味が変わるが、**呼び出し側を変えずに済むよう
//   名前と役割（サンプルを引き取るだけ）は据え置く。**
void imu_service_if_due() { imu_drv_service_if_due(); }

bool get_imu_ok()    { return imu_drv_present(); }
bool get_imu_alive() { return imu_drv_alive(); }


// ============================================================
// imu_update(): ドライバを 1 回進め、Kalman predict を実行する
// ============================================================
// loop() から毎回呼ぶ（ノンブロッキング）。
// ★ 0.986 の分割で、前半（途絶検出・復旧・引き取り・レート計測）は
//   imu_drv_poll() へ移した。**3 つの戻り値が分割前の 3 分岐そのまま**:
//     IMU_DRV_DEAD     … センサーが居ない／途絶中 → 恒速モデルで伝播して返る
//     IMU_DRV_WAIT     … 生きているがポーリング周期前 → 何もせず返る
//     IMU_DRV_SERVICED … 引き取った → 下の predict へ進む
void imu_update() {
    const ImuDrvState st = imu_drv_poll();

    if (st == IMU_DRV_DEAD) {
        // IMU なし: KF 初期化済みなら恒速モデル（a_k=0）で共分散を伝播させる。
        // これにより気圧・GNSS 速度観測が有効に機能するようになる。
        if (kf_initialized) {
            static uint32_t _fallback_predict_us = 0;
            uint32_t _now_us = time_us_32();
            if (_now_us - _fallback_predict_us >= 66000UL) {  // ~15Hz
                float dt = (float)(_now_us - kf_last_predict_us) * 1e-6f;
                kf_last_predict_us = _now_us;
                _fallback_predict_us = _now_us;
                if (dt > 0.0f && dt < 0.5f) {
                    kf_predict(0.0f, dt);  // 加速度ゼロ = 恒速モデル
                    _imu_altitude = kf_x[0];
                    _imu_vspeed   = kf_x[1];
                }
            }
        }
        return;
    }
    if (st == IMU_DRV_WAIT) return;

    // ---- Kalman predict ステップ ----
    // クォータニオンと加速度の両方が揃っており、かつ Kalman が初期化済みのときのみ実行。
    // （初期化は最初の気圧高度到着時に imu_kalman_baro_update() が行う）
#if VARIO_USE_RAW_ACCEL
    // ★ _quat_valid（GRV）は条件から外した。上方向は ESKF から取るので、
    //   「足し込めたか」を表す _accel_valid だけで足りる（足し込みは
    //   attitude_get_up_sensor() が成功したときにしか起きない）。
    if (!_accel_valid || !kf_initialized) {
        // KF 初期化前（最初の気圧高度が来る前）は predict しない。
        // ここで捨てておかないと、初期化されるまで蓄積が伸び続けてしまう。
        _acc_vsum = _acc_hsum = 0.0f;
        _acc_cnt  = 0;
        return;
    }
#else
    // ★ 0.986 の分割で融合出力（LACC/GRV）はドライバ側にある。ここで取り出す。
    float _la[3], _q[4];
    uint32_t _la_last_us = 0;
    if (!imu_drv_lacc_predict_sample(_la, _q, &_la_last_us) || !kf_initialized) return;
#endif

    // ---- 安全網: 古い加速度サンプルでは predict しない ----
    // 加速度レポートの配信が滞ったとき、同じサンプルを何度も積分してしまうと
    // 加速度スパイクが数倍に増幅されて数秒尾を引く（2026-08-17 の回帰）。
    // 正常時は ACCEL 50Hz = 20ms 周期（LACC 使用時は 15Hz = 67ms 周期）なので
    // 150ms を超えることはなく、このゲートは発火しない。
    // 発火時は predict を止めるだけで、出力は imu_kalman_baro_update() が
    // 気圧更新のたびに更新し続けるため、気圧ベースのバリオに劣化するだけで済む。
    // ※ VARIO_USE_RAW_ACCEL=1 では受信時に足し込む方式なので「同じサンプルの重複積分」
    //   自体は起きないが、通信途絶からの復帰で巨大な dt を積分しない意味は変わらない。
#if VARIO_USE_RAW_ACCEL
    if ((uint32_t)(time_us_32() - _acc_last_us) > IMU_LACC_MAX_AGE_US) {
        _lacc_stale_skips++;
        kf_last_predict_us = time_us_32();  // 復帰時に巨大な dt で積分しないよう進めておく
        _acc_vsum = _acc_hsum = 0.0f;       // 溜まった古い分は捨てる
        _acc_cnt  = 0;
        return;
    }
#else
    if ((uint32_t)(time_us_32() - _la_last_us) > IMU_LACC_MAX_AGE_US) {
        _lacc_stale_skips++;
        kf_last_predict_us = time_us_32();  // 復帰時に巨大な dt で積分しないよう進めておく
        return;
    }
#endif

#if IMULOG_RAW_REPORTS_ENABLED
    // ---- predict はポーリング周期ではなく専用周期（30ms）で回す ----
    // kf_predict() は Q を dt でスケールせず「1 ステップあたり」で加算するため
    // （kf_P[1][1] += q_vel_eff）、predict 周期を変えると単位時間あたりの
    // プロセスノイズ注入量が変わり、チューニング済みのバリオが壊れる。
    // ポーリングを 30ms → 4ms に上げても、ここで従来の 30ms 周期を維持することで
    // バリオの挙動を変えずに生ログのレートだけを上げられる。
    //
    // ※ ポーリングが 30ms のとき（生レポート無効時）はこのゲートを通してはいけない。
    //   周期が同じだとジッタでゲートを 1 回外し、predict が 60ms 間隔になる回が混ざる。
    //   そのため #if で丸ごと除外し、変更前と同じ「ポーリング毎に predict」に戻す。
    if ((uint32_t)(time_us_32() - kf_last_predict_us) < IMU_KF_PREDICT_INTERVAL_US) return;
#endif

#if VARIO_USE_RAW_ACCEL
    // 前回 predict からこの瞬間までに届いた生比力サンプルの平均を使う。
    // 変換（回転 → 重力除去）は受信のたびに済ませてあるので、ここでは平均するだけ。
    if (_acc_cnt == 0) return;   // 新しいサンプルが1つも無ければ predict しない
    float a_k         = _acc_vsum / _acc_cnt;
    float horiz_accel = _acc_hsum / _acc_cnt;
    _acc_vsum = _acc_hsum = 0.0f;
    _acc_cnt  = 0;
#else
    // 地球座標系の鉛直加速度を計算（BNO085 が重力を除去済みなので g を引く必要なし）
    float a_k = compute_earth_z_accel(_la[0], _la[1], _la[2], _q[0], _q[1], _q[2], _q[3]);

    // 地球座標系の水平加速度の大きさ（ワールドフレーム X・Y 成分のノルム）
    // 全加速度の二乗ノルムは回転で不変なので:
    //   horiz² = |a_body|² − a_world_z²  = (lax²+lay²+laz²) − a_k²
    // これにより姿勢変化の影響を受けず、純粋な水平加速度のみを取り出せる。
    float _body_sq   = _la[0]*_la[0] + _la[1]*_la[1] + _la[2]*_la[2];
    float _horiz_sq  = _body_sq - a_k * a_k;
    float horiz_accel = (_horiz_sq > 0.0f) ? sqrtf(_horiz_sq) : 0.0f;
#endif

    // dt を計算（前回 predict からの経過時間 [s]）
    uint32_t now_us = time_us_32();
    float dt = (float)(now_us - kf_last_predict_us) * 1e-6f;
    kf_last_predict_us = now_us;

    // dt の安全チェック: 起動直後・uint32 オーバーフロー・長時間停止を除外
    if (dt <= 0.0f || dt > 0.5f) return;

    // Kalman 予測ステップを実行（水平加速度を渡して Q_vel を動的増幅）
    kf_predict(a_k, dt, horiz_accel);

    // 出力を更新（display 側から参照される volatile 変数）
    _imu_az           = a_k;
    _imu_horiz_accel  = horiz_accel;
    _imu_altitude     = kf_x[0];
    _imu_vspeed       = kf_x[1];
}
