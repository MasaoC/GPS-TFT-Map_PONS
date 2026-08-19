// ============================================================
// File    : attitude.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS 速度援用 姿勢 ESKF（機上リアルタイム版）の実装。
//           数式・座標系・マウント補正の説明は attitude.h を参照。
//           PC 側の tools/imulog/eskf.py と対になっている。片方を直したら両方直すこと。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/18
// ============================================================

#include <Arduino.h>
#include <math.h>
#include "attitude.h"
#include "settings.h"

// ESKF はジャイロと加速度の生レポートが無いと動かない。
// これらは settings.h の IMULOG_RAW_REPORTS_ENABLED でしか有効化されないため、
// 0 のままだと attitude_ready() が永久に false になる（画面には INIT... と出続ける）。
#if !IMULOG_RAW_REPORTS_ENABLED
  #warning "IMULOG_RAW_REPORTS_ENABLED=0 のため姿勢 ESKF は動作しません（ジャイロ/加速度レポートが無効）"
#endif

#define N_ERR 12   // 誤差状態の数: dtheta(3) + dv(3) + dbg(3) + dba(3)

static const float GRAVITY = 9.80665f;

// ============================================================
// 状態
// ============================================================
static float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // body → ENU
static float v_[3] = {0.0f, 0.0f, 0.0f};         // 速度 ENU [m/s]
static float bg_[3] = {0.0f, 0.0f, 0.0f};        // ジャイロバイアス [rad/s]
static float ba_[3] = {0.0f, 0.0f, 0.0f};        // 加速度バイアス [m/s²]

// 共分散と作業領域。12x12 float = 576B。スタックに置くと Core0 が苦しいので static にする。
static float P_[N_ERR][N_ERR];
static float Phi_[N_ERR][N_ERR];
static float T1_[N_ERR][N_ERR];
static float T2_[N_ERR][N_ERR];

static bool  initialized_ = false;
// 最後にジャイロを受け取った時刻。IMU 途絶の検出に使う。
static uint32_t last_gyro_us_ = 0;
static bool     gyro_seen_    = false;
static float accel_last_[3] = {0.0f, 0.0f, GRAVITY};
static bool  accel_valid_ = false;

// ---- 診断 ----
static volatile uint32_t gnss_updates_ = 0;

// ---- 機体ゼロ点（マウント基準）のオフセット [度] ----
static float level_roll_off_ = 0.0f;
static float level_pitch_off_ = 0.0f;
// 次に較正するときに申告するピッチ角。画面の SET PITCH 行で選ぶ値。
static float pitch_target_ = 0.0f;

// ---- 30 秒平均ピッチ ----
// フゴイドで瞬時値は±3度振れるので、巡航のトリム状態は平均で見る。
static float pitch_avg_ = 0.0f;
static float pitch_avg_fill_s_ = 0.0f;    // 平均がたまった時間 [s]

// ---- 直進中のロール自動トリム ----
// 直進中のロールは理論上 0。ズレをゆっくり戻して誤警報の連発を防ぐ。
// 旋回中は一切働かないので、真のバンクは消さない。
// Core0（設定画面・ESKF）が書き、Core1（設定保存）も読むので volatile
static volatile bool roll_trim_enabled_ = true;   // 設定画面で ON/OFF（既定 ON）
static float    roll_trim_deg_ = 0.0f;    // 累積補正量（出力から引く）
static float    trim_roll_sum_ = 0.0f;    // 直進中のロール積算
static float    trim_time_s_   = 0.0f;    // 直進が続いた時間 [s]
static float    yaw_rate_lp_   = 0.0f;    // ワールド系ヨーレートの平滑値 [deg/s]
// 前方宣言。attitude_on_gyro() が 30 秒平均ピッチとロール自動トリムの
// 判定に使うが、定義はファイル後方（出力セクション）にあるため。
static void euler_from_state(float &roll, float &pitch, float &yaw);

static volatile bool  trim_event_ = false;      // ログ用: 補正が入った
static volatile float trim_event_applied_ = 0.0f;

// ============================================================
// サンプル周期の推定
// ============================================================
// BNO085 のレポートは一定周期でスケジュールされるが、Core0 が地図描画で
// 64ms 止まるとその間のサンプルがバーストで届き、受信時刻の差分は
// 「0, 0, 64ms」のように歪む。そのまま dt に使うと、まとめて届いた回転が
// ごっそり抜け落ちる（PC 側でも同じ問題に当たり、等間隔復元で解決した）。
// ここでは直近 DT_WIN サンプルの経過時間を頭数で割って周期を推定する。
#define DT_WIN 64
static uint32_t dt_ring_[DT_WIN];
static uint8_t  dt_idx_ = 0;
static uint16_t dt_count_ = 0;
static float    dt_est_ = 1.0f / IMU_RATE_GYRO_HZ;

// ============================================================
// 静止判定
// ============================================================
static bool     static_now_ = false;
static uint32_t static_since_us_ = 0;
static float    grv_sum_[4] = {0, 0, 0, 0};   // 静止中の GRV 累積（初期姿勢の平均用）
static uint32_t grv_count_ = 0;
static bool     grv_have_ = false;
static float    grv_last_[4] = {1, 0, 0, 0};

// ROTATION_VECTOR（地磁気補正あり）。初期ヨーの絶対基準に使う。
static float    rv_sum_[4]  = {0, 0, 0, 0};
static uint32_t rv_count_   = 0;
static bool     rv_have_    = false;
static float    rv_last_[4] = {1, 0, 0, 0};
static float    rv_acc_rad_ = -1.0f;      // BNO085 のヘディング精度推定（負 = 未キャリブ）
static bool     yaw_from_mag_ = false;    // 初期ヨーに地磁気を使えたか


// ============================================================
// クォータニオン / ベクトル ユーティリティ
// ============================================================
static void quat_normalize(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    float inv = 1.0f / n;
    for (int i = 0; i < 4; i++) q[i] *= inv;
    if (q[0] < 0.0f) for (int i = 0; i < 4; i++) q[i] = -q[i];  // w>=0 に揃える
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
    float w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    float x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    float y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    float z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

// 回転ベクトル（軸×角[rad]）→ クォータニオン。微小角でも安定な形。
static void quat_from_rotvec(const float r[3], float out[4]) {
    float th2 = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
    if (th2 < 1e-16f) {
        out[0] = 1.0f; out[1] = 0.5f*r[0]; out[2] = 0.5f*r[1]; out[3] = 0.5f*r[2];
        return;
    }
    float th = sqrtf(th2);
    float s = sinf(th * 0.5f) / th;
    out[0] = cosf(th * 0.5f);
    out[1] = r[0]*s; out[2] = r[1]*s; out[3] = r[2]*s;
}

// body → world の回転行列
static void quat_to_R(const float q[4], float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1-2*(y*y+z*z); R[0][1] = 2*(x*y-w*z);   R[0][2] = 2*(x*z+w*y);
    R[1][0] = 2*(x*y+w*z);   R[1][1] = 1-2*(x*x+z*z); R[1][2] = 2*(y*z-w*x);
    R[2][0] = 2*(x*z-w*y);   R[2][1] = 2*(y*z+w*x);   R[2][2] = 1-2*(x*x+y*y);
}


// 磁北基準の姿勢を真北基準へ回す。
//
// 出力ヨーは euler_from_state() で yaw_out = -sensor_yaw と定義してある。
// 真方位 = 磁方位 + 偏角 にしたいので
//     -sensor_yaw_true = -sensor_yaw_mag + decl
//     sensor_yaw_true  = sensor_yaw_mag - decl
// ワールド Z 軸まわりに δ 回すと sensor_yaw は +δ 動くので δ = -decl。
// （符号を間違えやすいので tools/attitude_test で数値検証している）
static void apply_declination(float q[4]) {
    const float d = -ESKF_MAG_DECLINATION_DEG * (float)M_PI / 180.0f;
    const float qz[4] = { cosf(d * 0.5f), 0.0f, 0.0f, sinf(d * 0.5f) };
    float out[4];
    quat_mul(qz, q, out);      // ワールド系の回転なので左から掛ける
    for (int i = 0; i < 4; i++) q[i] = out[i];
    quat_normalize(q);
}


// ============================================================
// 初期化
// ============================================================
static void reset_covariance(float att_sigma_deg) {
    memset(P_, 0, sizeof(P_));
    float a = att_sigma_deg * (float)M_PI / 180.0f;
    for (int i = 0; i < 3; i++)  P_[i][i]       = a * a;
    for (int i = 3; i < 6; i++)  P_[i][i]       = 1.0f;                 // 速度 [m/s]²
    for (int i = 6; i < 9; i++)  P_[i][i]       = powf(1.0f * (float)M_PI/180.0f, 2);
    for (int i = 9; i < 12; i++) P_[i][i]       = 0.01f;                // 加速度バイアス
}

void attitude_setup() {
    q_[0] = 1; q_[1] = q_[2] = q_[3] = 0;
    v_[0] = v_[1] = v_[2] = 0;
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(10.0f);
    initialized_ = false;
    accel_valid_ = false;
    grv_have_ = false;
    grv_count_ = 0;
    for (int i = 0; i < 4; i++) grv_sum_[i] = 0;
    rv_have_ = false;
    rv_count_ = 0;
    rv_acc_rad_ = -1.0f;
    yaw_from_mag_ = false;
    for (int i = 0; i < 4; i++) rv_sum_[i] = 0;
    dt_count_ = 0;
    dt_idx_ = 0;
    dt_est_ = 1.0f / IMU_RATE_GYRO_HZ;
    gnss_updates_ = 0;
    gyro_seen_ = false;
    last_gyro_us_ = 0;
    pitch_avg_ = 0.0f;
    pitch_avg_fill_s_ = 0.0f;
    roll_trim_deg_ = 0.0f;
    trim_roll_sum_ = 0.0f;
    trim_time_s_ = 0.0f;
    yaw_rate_lp_ = 0.0f;
    trim_event_ = false;
}


// IMU が途絶していないか。ジャイロが来なくなったら伝播が止まるため、
// そのまま GNSS 観測だけ入れると姿勢が誤って回される（下記 attitude_on_gnss_velocity 参照）。
static bool imu_fresh() {
    if (!gyro_seen_) return false;
    return (uint32_t)(time_us_32() - last_gyro_us_) < ESKF_IMU_TIMEOUT_US;
}

// 静止中に貯めた GRV の平均で姿勢を初期化する。
// 静止していれば GRV は重力基準なのでロール・ピッチは正しい。
// （ヨーは磁気なしで漂うが、初期化時点のヨーは後の GNSS 観測で拘束されるので気にしない）
static bool init_from_static_grv() {
    float q[4];
    yaw_from_mag_ = false;

#if ESKF_INIT_USE_RV
    // 地磁気補正付きの ROTATION_VECTOR が十分な精度で得られていればそちらを使う。
    // ロール・ピッチは GRV と同じ（どちらも重力基準）で、ヨーだけ磁北基準になる。
    // これで起動直後から妥当な方位を持てる（ESKF 単独では収束まで絶対方位が無い）。
    if (rv_count_ >= 4 && rv_acc_rad_ >= 0.0f &&
        rv_acc_rad_ * 180.0f / (float)M_PI <= ESKF_RV_ACC_MAX_DEG) {
        for (int i = 0; i < 4; i++) q[i] = rv_sum_[i] / (float)rv_count_;
        quat_normalize(q);
        apply_declination(q);          // 磁北基準 → 真北基準（GNSS 速度と同じ系に揃える）
        yaw_from_mag_ = true;
    } else
#endif
    {
        if (grv_count_ < 4) return false;
        for (int i = 0; i < 4; i++) q[i] = grv_sum_[i] / (float)grv_count_;
        quat_normalize(q);
    }
    for (int i = 0; i < 4; i++) q_[i] = q[i];
    v_[0] = v_[1] = v_[2] = 0.0f;
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(ESKF_INIT_ATT_SIGMA_DEG);
    initialized_ = true;
    return true;
}

// 静止区間が取れないまま走り出した場合のフォールバック。
// 直近の GRV を初期姿勢にするが、静止平均ほど信用できないので
// 姿勢の初期不確かさを大きめに取り、GNSS 観測で速く引き戻せるようにする。
// （2026-08-18 の session 2 は屋内起動→走行中に記録開始で静止区間が無く、
//   初期姿勢誤差がジャイロバイアスに吸われて 5.9deg/s に固着した）
static bool init_from_last_grv() {
    yaw_from_mag_ = false;
#if ESKF_INIT_USE_RV
    if (rv_have_ && rv_acc_rad_ >= 0.0f &&
        rv_acc_rad_ * 180.0f / (float)M_PI <= ESKF_RV_ACC_MAX_DEG) {
        for (int i = 0; i < 4; i++) q_[i] = rv_last_[i];
        quat_normalize(q_);
        apply_declination(q_);
        yaw_from_mag_ = true;
    } else
#endif
    {
        if (!grv_have_) return false;
        for (int i = 0; i < 4; i++) q_[i] = grv_last_[i];
        quat_normalize(q_);
    }
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(ESKF_INIT_ATT_SIGMA_MOVING_DEG);
    initialized_ = true;
    return true;
}


// ============================================================
// センサー入力
// ============================================================
void attitude_on_accel(const float a[3]) {
    accel_last_[0] = a[0]; accel_last_[1] = a[1]; accel_last_[2] = a[2];
    accel_valid_ = true;
}

void attitude_on_grv(float qw, float qx, float qy, float qz) {
    grv_last_[0] = qw; grv_last_[1] = qx; grv_last_[2] = qy; grv_last_[3] = qz;
    quat_normalize(grv_last_);
    grv_have_ = true;

    if (!static_now_ || initialized_) return;
    // 静止中のみ平均に加える。符号の揺れで打ち消し合わないよう先頭に揃える。
    if (grv_count_ == 0) {
        for (int i = 0; i < 4; i++) grv_sum_[i] = grv_last_[i];
    } else {
        float dot = 0;
        for (int i = 0; i < 4; i++) dot += grv_sum_[i] * grv_last_[i];
        float s = (dot < 0.0f) ? -1.0f : 1.0f;
        for (int i = 0; i < 4; i++) grv_sum_[i] += s * grv_last_[i];
    }
    grv_count_++;
}

void attitude_on_rv(float qw, float qx, float qy, float qz, float accuracy_rad) {
    rv_last_[0] = qw; rv_last_[1] = qx; rv_last_[2] = qy; rv_last_[3] = qz;
    quat_normalize(rv_last_);
    rv_acc_rad_ = accuracy_rad;
    rv_have_ = true;

    if (!static_now_ || initialized_) return;
    // 静止中のみ平均に加える。符号の揺れで打ち消し合わないよう先頭に揃える。
    if (rv_count_ == 0) {
        for (int i = 0; i < 4; i++) rv_sum_[i] = rv_last_[i];
    } else {
        float dot = 0;
        for (int i = 0; i < 4; i++) dot += rv_sum_[i] * rv_last_[i];
        float s = (dot < 0.0f) ? -1.0f : 1.0f;
        for (int i = 0; i < 4; i++) rv_sum_[i] += s * rv_last_[i];
    }
    rv_count_++;
}

bool attitude_yaw_from_mag() { return yaw_from_mag_; }


// 静止判定を更新する。ジャイロが小さく、かつ加速度の大きさが重力に近いこと。
// 加速度も見るのは、等速直進でもジャイロは小さくなるため。
static void update_static(const float g[3], uint32_t t_us) {
    float wn = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    float an = accel_valid_
             ? sqrtf(accel_last_[0]*accel_last_[0] + accel_last_[1]*accel_last_[1]
                     + accel_last_[2]*accel_last_[2])
             : 0.0f;
    bool now = (wn < ESKF_STATIC_GYRO_RADS)
               && accel_valid_
               && (fabsf(an - GRAVITY) < ESKF_STATIC_ACCEL_TOL);
    if (now && !static_now_) {
        static_since_us_ = t_us;
        grv_count_ = 0;
        rv_count_ = 0;
        for (int i = 0; i < 4; i++) { grv_sum_[i] = 0; rv_sum_[i] = 0; }
    }
    static_now_ = now;
}

void attitude_on_gyro(const float g[3], uint32_t t_us) {
    last_gyro_us_ = t_us;
    gyro_seen_ = true;

    // ---- サンプル周期の推定（バースト配信に耐えるため窓平均で求める）----
    dt_ring_[dt_idx_] = t_us;
    uint8_t oldest = (uint8_t)((dt_idx_ + 1) % DT_WIN);
    if (dt_count_ >= DT_WIN) {
        uint32_t span = t_us - dt_ring_[oldest];
        float d = (float)span * 1e-6f / (float)(DT_WIN - 1);
        if (d > 0.2f / IMU_RATE_GYRO_HZ && d < 5.0f / IMU_RATE_GYRO_HZ) dt_est_ = d;
    } else {
        dt_count_++;
    }
    dt_idx_ = oldest;

    update_static(g, t_us);

    // ---- 初期化 ----
    if (!initialized_) {
        if (static_now_ && (t_us - static_since_us_) >= ESKF_STATIC_INIT_US) {
            init_from_static_grv();
        }
        if (!initialized_) return;   // まだ初期化できていないので伝播しない
    }

    if (!accel_valid_) return;

    const float dt = dt_est_;
    float w[3], f[3];
    for (int i = 0; i < 3; i++) {
        w[i] = g[i] - bg_[i];
        f[i] = accel_last_[i] - ba_[i];
    }

    float R[3][3];
    quat_to_R(q_, R);

    // ---- 公称状態の伝播 ----
    float dq[4], rot[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
    quat_from_rotvec(rot, dq);
    float qn[4];
    quat_mul(q_, dq, qn);
    for (int i = 0; i < 4; i++) q_[i] = qn[i];
    quat_normalize(q_);

    float Rf[3];
    for (int i = 0; i < 3; i++)
        Rf[i] = R[i][0]*f[0] + R[i][1]*f[1] + R[i][2]*f[2];
    v_[0] += Rf[0] * dt;
    v_[1] += Rf[1] * dt;
    v_[2] += (Rf[2] - GRAVITY) * dt;      // ENU なので重力は -Z

    // ---- 誤差状態の遷移 Phi = I + F*dt ----
    //   d(dtheta)/dt = -R dbg
    //   d(dv)/dt     = -[R f]x dtheta - R dba
    memset(Phi_, 0, sizeof(Phi_));
    for (int i = 0; i < N_ERR; i++) Phi_[i][i] = 1.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Phi_[i][6 + j] = -R[i][j] * dt;
    // -[Rf]x
    Phi_[3][1] =  Rf[2] * dt;  Phi_[3][2] = -Rf[1] * dt;
    Phi_[4][0] = -Rf[2] * dt;  Phi_[4][2] =  Rf[0] * dt;
    Phi_[5][0] =  Rf[1] * dt;  Phi_[5][1] = -Rf[0] * dt;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Phi_[3 + i][9 + j] = -R[i][j] * dt;

    // ---- P = Phi P Phi^T + Q ----
    for (int i = 0; i < N_ERR; i++)
        for (int j = 0; j < N_ERR; j++) {
            float s = 0;
            for (int k = 0; k < N_ERR; k++) s += Phi_[i][k] * P_[k][j];
            T1_[i][j] = s;
        }
    for (int i = 0; i < N_ERR; i++)
        for (int j = 0; j < N_ERR; j++) {
            float s = 0;
            for (int k = 0; k < N_ERR; k++) s += T1_[i][k] * Phi_[j][k];
            P_[i][j] = s;
        }
    // ---- 平均ピッチと直進中のロール自動トリムを更新 ----
    // ここは公称状態が更新された後に呼ぶ（共分散の計算とは独立）。
    {
        float r, p, y;
        euler_from_state(r, p, y);
        p -= level_pitch_off_;
        r -= level_roll_off_ + roll_trim_deg_;

        // 30 秒の一次遅れで平均ピッチを作る
        const float a = dt / (PITCH_AVG_SEC + dt);
        pitch_avg_ += (p - pitch_avg_) * a;
        if (pitch_avg_fill_s_ < PITCH_AVG_SEC * 2.0f) pitch_avg_fill_s_ += dt;

        // ワールド系のヨーレート（旋回しているか）。ボディ角速度を回して Z 成分を取る。
        const float wz = R[2][0]*w[0] + R[2][1]*w[1] + R[2][2]*w[2];
        const float wz_dps = wz * 180.0f / (float)M_PI;
        yaw_rate_lp_ += (wz_dps - yaw_rate_lp_) * (dt / (2.0f + dt));

        const float sp = sqrtf(v_[0]*v_[0] + v_[1]*v_[1]);
        const bool straight = roll_trim_enabled_ &&
                              (fabsf(yaw_rate_lp_) < ROLL_TRIM_YAWRATE_DPS) &&
                              (sp > ROLL_TRIM_MIN_SPEED);
        if (straight) {
            trim_roll_sum_ += r * dt;
            trim_time_s_   += dt;
            if (trim_time_s_ >= ROLL_TRIM_WINDOW_S) {
                const float avg = trim_roll_sum_ / trim_time_s_;
                // 不感帯を超えたときだけ、1 回あたりの上限を守って少しずつ寄せる
                if (fabsf(avg) >= ROLL_TRIM_DEADBAND_DEG) {
                    float step = avg;
                    if (step >  ROLL_TRIM_STEP_DEG) step =  ROLL_TRIM_STEP_DEG;
                    if (step < -ROLL_TRIM_STEP_DEG) step = -ROLL_TRIM_STEP_DEG;
                    float next = roll_trim_deg_ + step;
                    // 累積上限。本当の異常（機体の歪みなど）を隠さないため。
                    if (next >  ROLL_TRIM_LIMIT_DEG) { step = ROLL_TRIM_LIMIT_DEG - roll_trim_deg_; }
                    if (next < -ROLL_TRIM_LIMIT_DEG) { step = -ROLL_TRIM_LIMIT_DEG - roll_trim_deg_; }
                    if (fabsf(step) > 1e-3f) {
                        roll_trim_deg_ += step;
                        trim_event_applied_ = step;
                        trim_event_ = true;      // 呼び出し側がログに残す
                    }
                }
                trim_roll_sum_ = 0.0f;
                trim_time_s_   = 0.0f;
            }
        } else {
            // 旋回に入ったら積算を捨てる（旋回中のロールを平均に混ぜない）
            trim_roll_sum_ = 0.0f;
            trim_time_s_   = 0.0f;
        }
    }

    const float qg  = ESKF_SIGMA_G  * ESKF_SIGMA_G  * dt;
    const float qa  = ESKF_SIGMA_A  * ESKF_SIGMA_A  * dt;
    const float qbg = ESKF_SIGMA_BG * ESKF_SIGMA_BG * dt;
    const float qba = ESKF_SIGMA_BA * ESKF_SIGMA_BA * dt;
    for (int i = 0; i < 3; i++) {
        P_[i][i]       += qg;
        P_[3 + i][3 + i] += qa;
        P_[6 + i][6 + i] += qbg;
        P_[9 + i][9 + i] += qba;
    }
}


// ============================================================
// GNSS 速度による観測更新
// ============================================================
// バイアス推定が物理的にあり得ない大きさへ発散しないよう制限する。
// BNO085 は校正済みの値を出すので残留バイアスは本来ごく小さい。
// 上限が無いと初期姿勢誤差の行き場としてバイアス状態が使われ、
// 誤った値に固着して姿勢補正が効かなくなる
// （2026-08-18 の実測で bg が 5.9deg/s に張り付き、ESKF が BNO085 と同じ挙動になった）。
static void clamp_bias(float b[3], float lim) {
    float n = sqrtf(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
    if (n > lim && n > 1e-9f) {
        float s = lim / n;
        b[0] *= s; b[1] *= s; b[2] *= s;
    }
}

void attitude_on_gnss_velocity(float velN, float velE, float velD, float sAcc) {
    // IMU が途絶しているときは観測を取り込まない。
    // 伝播（ジャイロ・加速度）が止まった状態で速度観測だけ入れると、
    // フィルタは速度の食い違いを姿勢誤差のせいにして姿勢を回し続ける。
    // 実測（模擬）では水平のまま加速しただけで 60 秒後にロールが +8 度まで育った。
    if (!imu_fresh()) return;

    // NED → ENU
    const float z[3] = { velE, velN, -velD };

    if (!initialized_) {
        // 静止区間が無いまま動き出した場合の保険。姿勢を大きめの不確かさで置いてから走り出す。
        if (!init_from_last_grv()) return;
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2];
        return;
    }

    // 観測ノイズ。sAcc をそのまま使うが、極端に小さいと過信するので下限を置く。
    float r = sAcc;
    if (r < ESKF_SACC_MIN) r = ESKF_SACC_MIN;
    if (r > ESKF_SACC_MAX) return;          // フィックスが悪いときの暴れ値は捨てる
    const float rr = r * r;

    // H = [0 I 0 0] なので H P = P の 3..5 行
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P_[3 + i][3 + j] + ((i == j) ? rr : 0.0f);

    // S の逆行列（3x3）
    float a = S[0][0], b = S[0][1], c = S[0][2];
    float d = S[1][0], e = S[1][1], f = S[1][2];
    float g = S[2][0], h = S[2][1], i2 = S[2][2];
    float A =  (e*i2 - f*h), B = -(d*i2 - f*g), C =  (d*h - e*g);
    float det = a*A + b*B + c*C;
    if (fabsf(det) < 1e-12f) return;
    float invdet = 1.0f / det;
    float Si[3][3];
    Si[0][0] = A*invdet;            Si[0][1] = -(b*i2 - c*h)*invdet; Si[0][2] =  (b*f - c*e)*invdet;
    Si[1][0] = B*invdet;            Si[1][1] =  (a*i2 - c*g)*invdet; Si[1][2] = -(a*f - c*d)*invdet;
    Si[2][0] = C*invdet;            Si[2][1] = -(a*h - b*g)*invdet;  Si[2][2] =  (a*e - b*d)*invdet;

    // K = P[:,3:6] * S^-1  (12x3)
    static float K[N_ERR][3];
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < 3; n++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += P_[m][3 + k] * Si[k][n];
            K[m][n] = s;
        }

    // dx = K * (z - v)
    float y[3] = { z[0] - v_[0], z[1] - v_[1], z[2] - v_[2] };
    float dx[N_ERR];
    for (int m = 0; m < N_ERR; m++)
        dx[m] = K[m][0]*y[0] + K[m][1]*y[1] + K[m][2]*y[2];

    // ---- P = (I-KH) P (I-KH)^T + K R K^T （Joseph 形。対称性が保たれる）----
    // I-KH は「K の各列を P の 3..5 列位置から引いた」形になる
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++)
            T1_[m][n] = ((m == n) ? 1.0f : 0.0f) - ((n >= 3 && n < 6) ? K[m][n - 3] : 0.0f);
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++) {
            float s = 0;
            for (int k = 0; k < N_ERR; k++) s += T1_[m][k] * P_[k][n];
            T2_[m][n] = s;
        }
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++) {
            float s = 0;
            for (int k = 0; k < N_ERR; k++) s += T2_[m][k] * T1_[n][k];
            P_[m][n] = s + rr * (K[m][0]*K[n][0] + K[m][1]*K[n][1] + K[m][2]*K[n][2]);
        }

    // ---- 誤差の注入 ----
    float dq[4], qn[4];
    quat_from_rotvec(dx, dq);          // dtheta はワールド系なので左から掛ける
    quat_mul(dq, q_, qn);
    for (int k = 0; k < 4; k++) q_[k] = qn[k];
    quat_normalize(q_);
    for (int k = 0; k < 3; k++) {
        v_[k]  += dx[3 + k];
        bg_[k] += dx[6 + k];
        ba_[k] += dx[9 + k];
    }
    clamp_bias(bg_, ESKF_MAX_GYRO_BIAS);
    clamp_bias(ba_, ESKF_MAX_ACCEL_BIAS);
    gnss_updates_++;
}


// ============================================================
// 出力
// ============================================================
// 初期化済みで、かつ IMU が生きていること。
// IMU が途絶したら false に落として表示側に「無効」と伝える
//（古い姿勢を有効値として出し続けると、水平飛行中に偽のバンク警告が出る）。
bool attitude_ready() { return initialized_ && imu_fresh(); }

// センサー座標系のオイラー角 → 機体軸 [度]。
// imu.cpp の get_imu_euler() および tools/imulog/decode_imulog.py の
// mount_correct() と同じ変換であること。
static void euler_from_state(float &roll, float &pitch, float &yaw) {
    const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    float sensor_roll = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    float sinp = 2.0f*(w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float sensor_pitch = asinf(sinp);
    float sensor_yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

    const float rad2deg = 180.0f / (float)M_PI;
    roll  = sensor_pitch * rad2deg;
    pitch = (sensor_roll - (float)M_PI * 0.5f) * rad2deg;
    yaw   = -sensor_yaw * rad2deg;
    if (yaw < 0.0f)    yaw += 360.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;
}

void attitude_get_euler_raw(float &roll, float &pitch, float &yaw) {
    euler_from_state(roll, pitch, yaw);
}

void attitude_get_euler(float &roll, float &pitch, float &yaw) {
    euler_from_state(roll, pitch, yaw);
    // 手動較正のオフセットに加えて、直進中に貯めた自動トリムも引く
    roll  -= level_roll_off_ + roll_trim_deg_;
    pitch -= level_pitch_off_;
}

float attitude_get_pitch_avg_deg() { return pitch_avg_; }
bool  attitude_pitch_avg_valid()   { return pitch_avg_fill_s_ >= PITCH_AVG_SEC; }
float attitude_get_roll_trim_deg() { return roll_trim_deg_; }
bool  attitude_get_roll_trim_enabled() { return roll_trim_enabled_; }
void  attitude_set_roll_trim_enabled(bool on) {
  roll_trim_enabled_ = on;
  if (!on) {
    // OFF にしたら補正を捨てて素の測定値に戻す（半端に効いたままだと紛らわしい）
    roll_trim_deg_ = 0.0f;
    trim_roll_sum_ = 0.0f;
    trim_time_s_   = 0.0f;
  }
}

bool attitude_take_roll_trim_event(float &applied, float &total) {
    if (!trim_event_) return false;
    trim_event_ = false;
    applied = trim_event_applied_;
    total   = roll_trim_deg_;
    return true;
}

void attitude_get_gyro_bias(float b[3])  { for (int i=0;i<3;i++) b[i] = bg_[i]; }
void attitude_get_accel_bias(float b[3]) { for (int i=0;i<3;i++) b[i] = ba_[i]; }
void attitude_get_velocity(float v[3])   { for (int i=0;i<3;i++) v[i] = v_[i]; }

// ヨー誤差の標準偏差 [度]。誤差状態 dtheta の Z 成分（ワールド系＝鉛直軸まわり）。
// 等速直進では観測が入らず単調に育ち、旋回や加減速が入ると縮む。
float attitude_get_yaw_sigma_deg() {
    if (!initialized_) return 180.0f;
    float v = P_[2][2];
    if (v < 0.0f) v = 0.0f;
    return sqrtf(v) * 180.0f / (float)M_PI;
}

// 95% 値 = 2σ。内部の計算は 1σ のままで、表示と判定だけこちらに揃える。
float attitude_get_yaw_acc95_deg() {
    float s = attitude_get_yaw_sigma_deg();
    return (s >= 180.0f) ? 180.0f : s * 2.0f;
}
bool attitude_is_static()                { return static_now_; }
uint32_t attitude_get_gnss_updates()     { return gnss_updates_; }

float attitude_get_static_secs() {
    if (!static_now_) return 0.0f;
    return (float)(time_us_32() - static_since_us_) * 1e-6f;
}


// ============================================================
// 機体ゼロ点（マウント基準）の較正
// ============================================================
void attitude_calibrate_to(float target_pitch_deg) {
    if (!initialized_) return;
    float r, p, y;
    euler_from_state(r, p, y);      // オフセット適用前の生の値を基準にする
    // 表示は (生の値 - オフセット) なので、target になるように差を取る
    level_roll_off_  = r;                        // ロールは常に 0 にする
    level_pitch_off_ = p - target_pitch_deg;     // ピッチは申告値になるようにする

    // 自動トリムの累積量は捨てる。ここでロールのゼロ点を取り直したので、
    // 残したままだと表示が -roll_trim_deg_ になり「APPLY したのに 0 にならない」
    // 状態になる。溜めかけの平均もリセットして、新しい基準で測り直す。
    roll_trim_deg_ = 0.0f;
    trim_roll_sum_ = 0.0f;
    trim_time_s_   = 0.0f;
}

void attitude_calibrate_level() {
    attitude_calibrate_to(0.0f);
}

void attitude_reset_level() {
    level_roll_off_ = 0.0f;
    level_pitch_off_ = 0.0f;
    roll_trim_deg_ = 0.0f;   // 較正を捨てるので自動トリムの累積も捨てる
    trim_roll_sum_ = 0.0f;
    trim_time_s_   = 0.0f;
}

void attitude_get_level_offset(float &roll_deg, float &pitch_deg) {
    roll_deg = level_roll_off_;
    pitch_deg = level_pitch_off_;
}

void attitude_set_level_offset(float roll_deg, float pitch_deg) {
    level_roll_off_ = roll_deg;
    level_pitch_off_ = pitch_deg;
}

float attitude_get_pitch_target() { return pitch_target_; }
void  attitude_set_pitch_target(float deg) { pitch_target_ = deg; }

void attitude_cycle_pitch_target() {
    pitch_target_ += PITCH_TARGET_STEP_DEG;
    // 端まで行ったら先頭へ戻す。浮動小数の誤差で行き過ぎないよう余裕を持たせる。
    if (pitch_target_ > PITCH_TARGET_MAX_DEG + 0.01f)
        pitch_target_ = PITCH_TARGET_MIN_DEG;
}
