// ============================================================
// File    : imu.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : バリオ用 3 状態 Kalman フィルターと、推定側のゲッター。
//           **チップ非依存。** チップ固有のコードは imu_sch16t.cpp にある。
//
//   チップ側とのやり取りは imu_sensor.h の imu_drv_* だけ。
//
// ■ アーキテクチャ
//   - imu_update() を毎ループ呼ぶ。中で imu_drv_poll() を 1 回進め、
//     戻り値（DEAD / WAIT / SERVICED）で Kalman predict へ進むかを決める
//   - サンプル到着時 → predict（状態を比力で時間更新）
//   - 気圧高度到着時 → update（get_airdata_altitude() で観測修正）
//   - 鉛直加速度がありえない値のあいだ（姿勢が壊れたときなど）は加速度を使わず、気圧だけで predict し、
//     加速度バイアスも学ばない（acc_guard_step / kf_predict_baro_only。settings.h の KF_ACC_GUARD_*。2026-11-11〜）
//
// ■ 座標系
//   imu_feed_accel() が受け取るのはセンサー軸の比力（重力込み）。
//   「上方向」は姿勢 ESKF（attitude_get_up_sensor）から取るので、
//   **チップの融合出力には依存しない。**
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
//     B = [0, dt, 0]^T   （前進オイラー。位置の ½·a·dt² の項は入れていない。下の kf_predict() のコメント）
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
// Updated : 2026/11/11
// ============================================================

#include <Arduino.h>

// ★ **imulog.h はここには無い。** 生ログのレコードはチップ固有なのでドライバ側にある。
//   ここへ戻すと「チップ非依存」という前提が静かに崩れる。
#include "imu.h"
#include "imu_sensor.h"   // チップ境界（feed の口・imu_drv_*）
#include "link.h"
#include "settings.h"
#include "airdata.h"  // airdata_* （気圧計は i2c0。IMU のバスとは別系統）
#include "mysd.h"     // enqueueTask / createLogSdfTask
#include "gnss.h"      // replay_has_value / replay_get_* （リプレイ時のセンサ値差し替え）
#include "attitude.h" // 姿勢 ESKF（機上リアルタイム版）


// ============================================================
// 生比力から作る鉛直加速度の蓄積バッファ
// ============================================================
// 加速度は 250Hz で届くが kf_predict() は 30ms 周期でしか回らない。
// 「最新の 1 サンプル」だけを使うと間のサンプルを捨てることになり、
// 振動成分がエイリアシングして推定上昇率のノイズになる。
// そこで受信のたびにここへ足し込み、predict のときに平均を取り出して使う。
// （v7 の実ログでの静止時σ: 最新値 0.153 → 平均 0.090。動きのある区間でも平均の方が良い）
//
// imu_feed_accel() は SCH16T の読み出し（imu_sch16t.cpp）から Core0 で呼ばれるので、
// imu_update() の predict と同じコアであり排他は不要。
static float    _acc_vsum = 0.0f;   // 鉛直加速度の合計 [m/s²]（重力除去済み）
static float    _acc_hsum = 0.0f;   // 水平加速度の大きさの合計 [m/s²]
static uint16_t _acc_cnt  = 0;      // 合計したサンプル数
static uint32_t _acc_last_us = 0;   // 最後に加速度を受信した時刻 [µs]（鮮度チェック用）
static bool     _accel_valid = false;

// ============================================================
// Kalman フィルター 内部状態
// ============================================================

// 鮮度不足で predict をスキップした回数（診断用。正常時は 0 のまま）。
// ★ predict 側の計数なのでチップ非依存。0.986 の分割でここへ置いた。
static volatile uint32_t _acc_stale_skips = 0;

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
    // ESKF の伝播はジャイロ到着で回す（加速度は直近値を使う。ドライバは同じサンプルの加速度を先に渡す）。
    // ※ 時刻は受信時刻（SCH16T はサンプルにタイムスタンプを付けない）。
    //   dt は attitude.cpp が直前のサンプルとの実際の時間差で取る。
    attitude_on_gyro(g, t_us);
}

// ドライバがリセット／復旧したときに呼ばれる（imu_sensor.h の契約）。
void imu_feed_reset() {
    _accel_valid = false;
    _acc_vsum = _acc_hsum = 0.0f;
    _acc_cnt  = 0;
}

void imu_feed_accel(const float a[3]) {
    _raw_accel[0] = a[0]; _raw_accel[1] = a[1]; _raw_accel[2] = a[2];
    attitude_on_accel(a);
    // ---- バリオ KF 用: 地球座標系の鉛直/水平加速度をここで作って足し込む ----
    // 重力込みの比力を「上方向」（ESKF の回転行列の第 3 行）へ射影し、重力を引く。
    // 引く重力が固定値なので「動作に相関した誤差」が入らない
    // （settings.h の「KF に入れる鉛直加速度の作り方」）。
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
//   B = [0, dt, 0]^T
// ★ 位置の更新は前進オイラー（z += v·dt）で、½·(a−b)·dt² の項は**入れていない**（2026-11-10 までコメントだけ
//   dt²/2 と書いてあった）。30ms では 1m/s² でも 0.45mm と表示に効かず、このバリオはこの式で調整してある
//   （settings.h の IMU_KF_PREDICT_INTERVAL_US）。足すなら F の [0][2] にも −dt²/2 を入れて、調整をやり直すこと。
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
// 加速度を使わない予測（鉛直加速度が信用できないあいだ。settings.h の KF_ACC_GUARD_*）
// ============================================================
// 恒速モデルで、**バイアスは止める**（速度にも効かせない）。
//   F = [[1, dt, 0], [0, 1, 0], [0, 0, 1]]、Q = diag(0, q_vel, 0)
// ★ 入るときに acc_guard_step() がバイアスと高度・速度の相関（P[0][2] / P[1][2]）を 0 にしてある。この F では
//   相関が育たないので、気圧と GNSS の観測のゲイン K[2] も 0 のまま＝**バイアスは観測でも動かない**。
// ★ IMU が居ないときの恒速モデル（imu_update の IMU_DRV_DEAD）は kf_predict(0, dt) で、こちらとは別。
static void kf_predict_baro_only(float dt) {
    kf_x[0] += kf_x[1] * dt;
    float A[3][3];
    for (int j = 0; j < 3; j++) {
        A[0][j] = kf_P[0][j] + dt * kf_P[1][j];       // F[0] = [1, dt, 0]
        A[1][j] = kf_P[1][j];                           // F[1] = [0, 1, 0]
        A[2][j] = kf_P[2][j];                           // F[2] = [0, 0, 1]
    }
    for (int i = 0; i < 3; i++) {
        kf_P[i][0] = A[i][0] + dt * A[i][1];
        kf_P[i][1] = A[i][1];
        kf_P[i][2] = A[i][2];
    }
    kf_P[1][1] += kf_q_vel;
}

// ============================================================
// 鉛直加速度が信用できるか（settings.h の KF_ACC_GUARD_*）
// ============================================================
// predict の 1 回ごと（30ms）に呼ぶ。false = 加速度を使わず気圧だけで回す。
// ★ 姿勢が壊れると鉛直加速度が偽の値になり、V/S とバイアスを引きずる（2026-11-11 のトンネル）。理由は settings.h。
static bool     s_guard_on      = false;
static float    s_guard_lp      = 0.0f;     // 鉛直加速度の 1 秒の低域 [m/s²]
static uint8_t  s_guard_fast_n  = 0;        // |a| が速い条件を超えた回の連続数
static uint32_t s_guard_calm_us = 0;        // 戻る条件がそろい始めた時刻。0 = そろっていない
static uint32_t s_guard_on_ms   = 0;        // 入った時刻（ログ用）
static uint32_t s_guard_log_ms  = 0;        // 入ったことを最後にログへ書いた時刻（10 秒に 1 回まで）
static uint16_t s_guard_quiet_n = 0;        // ログを省いた回数
static bool     s_guard_logged  = false;    // 今回入ったことをログへ書いたか（戻ったときも書く）

static bool acc_guard_step(float a_k, float dt) {
    s_guard_lp += (a_k - s_guard_lp) * (dt / (KF_ACC_GUARD_TAU_S + dt));
    const bool fast_hit = fabsf(a_k) > KF_ACC_GUARD_FAST_MPS2;
    if (fast_hit) { if (s_guard_fast_n < 255) s_guard_fast_n++; }
    else            s_guard_fast_n = 0;

    if (!s_guard_on) {
        if (s_guard_fast_n >= KF_ACC_GUARD_FAST_STEPS || fabsf(s_guard_lp) > KF_ACC_GUARD_SLOW_MPS2) {
            s_guard_on      = true;
            s_guard_calm_us = 0;
            s_guard_on_ms   = millis();
            // バイアスを観測から切り離す（kf_predict_baro_only のコメント）
            kf_P[0][2] = kf_P[2][0] = 0.0f;
            kf_P[1][2] = kf_P[2][1] = 0.0f;
            // 持ち直している間などは出入りを繰り返すので、ログは 10 秒に 1 回まで（省いた回数を次に書く）
            const uint32_t now_ms = millis();
            if (s_guard_log_ms == 0 || now_ms - s_guard_log_ms >= 10000UL) {
                s_guard_log_ms = now_ms ? now_ms : 1;
                s_guard_logged = true;
                enqueueTask(createLogSdfTask("VARIO: IMU accel ignored (a=%+.1f lp=%+.1f m/s2, bias %+.2f kept) -> baro only, +%u",
                                             a_k, s_guard_lp, kf_x[2], (unsigned)s_guard_quiet_n));
                s_guard_quiet_n = 0;
            } else {
                s_guard_logged = false;
                if (s_guard_quiet_n < 0xFFFF) s_guard_quiet_n++;
            }
        }
    } else {
        const bool calm = (fabsf(s_guard_lp) < KF_ACC_GUARD_CLEAR_MPS2) && !fast_hit;
        const uint32_t now_us = time_us_32();
        if (!calm) {
            s_guard_calm_us = 0;
        } else if (s_guard_calm_us == 0) {
            s_guard_calm_us = now_us ? now_us : 1;
        } else if ((uint32_t)(now_us - s_guard_calm_us) >= KF_ACC_GUARD_CLEAR_US) {
            s_guard_on = false;
            s_guard_calm_us = 0;
            if (s_guard_logged)
                enqueueTask(createLogSdfTask("VARIO: IMU accel back after %.1fs",
                                             (millis() - s_guard_on_ms) * 0.001f));
        }
    }
    return !s_guard_on;
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
//   補正は 1Hz で 1 回、1 回あたり最大 GNSS_MAX_DELTA_M [m] なので、
//   気圧基準は最大 **GNSS_MAX_DELTA_M [m/s]** の傾きで動く。これは
//   KF から見ると高度観測のランプ入力なので、V/S に同じ大きさの定常バイアスが乗る。
//   いまの値（0.05m/s）は表示分解能 0.1m/s の半分、デッドバンド
//   VARIO_DEADBAND_KF_MPS（0.25m/s）の 1/5 で、**音を鳴らすことはできない**。
//   ★ **数値をここに書かないこと。** 0.02 と書いてあった時代があり、
//     GNSS_MAX_DELTA_M を 0.02→0.05 に上げたときコメントだけ古いまま残った。

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

// ★ **リプレイ中は CSV の KF_Vspeed だけを見る**（ミラー中と同じく、自機のセンサーの有無で分岐しない）。
//   1.04 まで再生の分岐が無く、机の上で自機の IMU が無いと再生中のバリオが CSV ではなく
//   **自機の気圧 V/S（≒0）**になり、自機の MS5611 が無いと再生中の VSI バーが消えていた。
//   KF_Vspeed 列の無い古い CSV では V/S を出さない（自機の値を「再生した飛行の V/S」として出さない）。

// V/S を出せる状態か。気圧が無いと KF が初期化されないので V/S は出ない。
// ★ **飛行中に MS5611 が途絶えたときもここで落ちる。** KF は IMU の加速度だけで predict を続けて
//   漂う（気圧の観測が無いので加速度のゼロ点ずれを積分する）ので、出してはいけない。
//   音（update_vario）・VSI・無線はどれも vario_vspeed_mps() を通すので、そこで 0 にしてある。
bool vario_source_ok() {
  if (link_mirror_active()) return link_has_value(RHAVE_KFVS);
  if (getReplayMode())      return replay_has_value(RHAVE_KFVS);
  return get_airdata_ok();
}

// 表示・音・無線に使う V/S。IMU が途絶していれば MS5611 単独の上昇率へ落とす。
// 出せないとき（vario_source_ok() が false）は 0（音は止まり、VSI は vario_source_ok() で消える）。
// ★ 1.04 までは音がこの判定を見ておらず、飛行中に MS5611 が途絶えると、VSI は消えるのに
//   漂った KF の V/S でバリオが鳴り続けていた。
float vario_vspeed_mps() {
  if (!vario_source_ok())   return 0.0f;
  if (link_mirror_active()) return get_imu_vspeed();   // = link_get_kf_vspeed()
  if (getReplayMode())      return get_imu_vspeed();   // = replay_get_kf_vspeed()
  return get_imu_alive() ? get_imu_vspeed() : get_airdata_vspeed();
}

// デッドバンド。KF 融合中（IMU 生存 + MS5611 接続）は精度が高いので狭くする。
// リプレイ中は CSV の KF_Vspeed を鳴らすので KF 側（記録した機体の IMU が死んでいたかは CSV に無い）。
float vario_deadband_mps() {
  if (link_mirror_active())
    return (link_sender_status() & LINK_ST_IMU_ERROR) ? VARIO_DEADBAND_BARO_MPS
                                                      : VARIO_DEADBAND_KF_MPS;
  if (getReplayMode()) return VARIO_DEADBAND_KF_MPS;
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

// 加速度サンプルの鮮度不足で Kalman predict をスキップした累計回数。
// 正常時は 0。増えていれば読み出しが止まっている（Core0 の長いブロック・途絶）。
uint32_t get_imu_acc_stale_skips() { return _acc_stale_skips; }
// 生ジャイロ [rad/s] / 生加速度 [m/s²] の直近値（IMU/ESKF 画面の表示用）
void get_imu_raw_gyro(float g[3])  { for (int i=0;i<3;i++) g[i] = _raw_gyro[i]; }
void get_imu_raw_accel(float a[3]) { for (int i=0;i<3;i++) a[i] = _raw_accel[i]; }

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
// ★ **ここに「どのチップか」の分岐を書かないこと。** 実装はドライバ側の 1 ファイル（imu_sch16t.cpp）。

void imu_setup() { imu_drv_setup(); }

// ★ 長いブロッキング処理の中から呼ぶためのもの。SCH16T はこちらから取りに行く方式なので、
//   呼ばれない間のサンプルは抜ける（settings.h の IMU_POLL_INTERVAL_US）。
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
    // 加速度が足し込まれていて、かつ Kalman が初期化済みのときのみ実行。
    // （初期化は最初の気圧高度到着時に imu_kalman_baro_update() が行う）
    if (!_accel_valid || !kf_initialized) {
        // KF 初期化前（最初の気圧高度が来る前）は predict しない。
        // ここで捨てておかないと、初期化されるまで蓄積が伸び続けてしまう。
        _acc_vsum = _acc_hsum = 0.0f;
        _acc_cnt  = 0;
        return;
    }

    // ---- 安全網: 古い加速度サンプルでは predict しない ----
    // 通信途絶や Core0 の長いブロックからの復帰で、巨大な dt を積分しないためのゲート。
    // 正常時は 4ms 周期なので 150ms を超えることはなく、このゲートは発火しない。
    // 発火時は predict を止めるだけで、出力は imu_kalman_baro_update() が
    // 気圧更新のたびに更新し続けるため、気圧ベースのバリオに劣化するだけで済む。
    if ((uint32_t)(time_us_32() - _acc_last_us) > IMU_ACC_MAX_AGE_US) {
        _acc_stale_skips++;
        kf_last_predict_us = time_us_32();  // 復帰時に巨大な dt で積分しないよう進めておく
        _acc_vsum = _acc_hsum = 0.0f;       // 溜まった古い分は捨てる
        _acc_cnt  = 0;
        return;
    }

    // ---- predict はポーリング周期ではなく専用周期（30ms）で回す ----
    // kf_predict() は Q を dt でスケールせず「1 ステップあたり」で加算するため
    // （kf_P[1][1] += q_vel_eff）、predict 周期を変えると単位時間あたりの
    // プロセスノイズ注入量が変わり、チューニング済みのバリオが壊れる
    // （settings.h の IMU_KF_PREDICT_INTERVAL_US）。
    if ((uint32_t)(time_us_32() - kf_last_predict_us) < IMU_KF_PREDICT_INTERVAL_US) return;

    // 前回 predict からこの瞬間までに届いた生比力サンプルの平均を使う。
    // 変換（回転 → 重力除去）は受信のたびに済ませてあるので、ここでは平均するだけ。
    if (_acc_cnt == 0) return;   // 新しいサンプルが1つも無ければ predict しない
    float a_k         = _acc_vsum / _acc_cnt;
    float horiz_accel = _acc_hsum / _acc_cnt;
    _acc_vsum = _acc_hsum = 0.0f;
    _acc_cnt  = 0;

    // dt を計算（前回 predict からの経過時間 [s]）
    uint32_t now_us = time_us_32();
    float dt = (float)(now_us - kf_last_predict_us) * 1e-6f;
    kf_last_predict_us = now_us;

    // dt の安全チェック: 起動直後・uint32 オーバーフロー・長時間停止を除外
    if (dt <= 0.0f || dt > 0.5f) return;

    // Kalman 予測ステップを実行（水平加速度を渡して Q_vel を動的増幅）
    // ★ 鉛直加速度がありえない値のあいだは加速度を使わず、バイアスも学ばない（settings.h の KF_ACC_GUARD_*）。
    //   判定は使わない間も毎回 a_k で続ける（戻る条件を見るため）。
    if (acc_guard_step(a_k, dt)) kf_predict(a_k, dt, horiz_accel);
    else                         kf_predict_baro_only(dt);

    // 出力を更新（display 側から参照される volatile 変数）
    _imu_az           = a_k;
    _imu_horiz_accel  = horiz_accel;
    _imu_altitude     = kf_x[0];
    _imu_vspeed       = kf_x[1];
}
