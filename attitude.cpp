// ============================================================
// File    : attitude.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS 速度援用 姿勢 ESKF（機上リアルタイム版）の実装。
//           数式・座標系・マウント補正の説明は attitude.h を参照。
//           PC 側の tools/imulog/eskf.py と対になっている。片方を直したら両方直すこと。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/01
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
// ★ **センサー座標系 → ENU。機体軸ではない。**
//   入力（attitude_on_gyro / on_accel / on_grv）はすべて BNO085 の生の
//   センサー軸で入れているので、この状態量もセンサー軸のまま。
//   機体軸へのマウント回転は **出力の euler_from_state() で 1 回だけ**掛ける。
//   ここを「機体軸」と誤解してマウント回転を二重に掛けないこと。
static float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // センサー軸 → ENU
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
// GNSS 速度観測が最後に入った時刻。**自動ロールトリムの安全装置**に使う。
static uint32_t last_gnss_vel_us_ = 0;
static bool     gnss_vel_seen_    = false;
static bool     gyro_seen_    = false;
static float accel_last_[3] = {0.0f, 0.0f, GRAVITY};
static bool  accel_valid_ = false;

// ---- 診断 ----
static volatile uint32_t gnss_updates_ = 0;
// 重力観測（Phase 2）の回数と、ゲート用の「前後加速度の持続成分」[m/s²]。
// ★ 前後加速度は GNSS からしか作れない。断中はゲートからこの条件を省く
//   （ピッチ側の誤差を許容して、錨が無いより良しとする）。
static volatile uint32_t level_updates_ = 0;
static float    longacc_lp_   = 0.0f;
static float    last_gsp_     = -1.0f;   // 負 = まだ測位が来ていない
static uint32_t last_gsp_us_  = 0;
static uint32_t level_last_us_ = 0;
// ワールド系ヨーレートの瞬時値 [deg/s]。持続成分は yaw_rate_lp_（下）。
static float    yaw_rate_inst_ = 0.0f;

// ---- 機体ゼロ点（マウント基準）のオフセット [度] ----
static float level_roll_off_ = 0.0f;
static float level_pitch_off_ = 0.0f;
// 次に較正するときに申告するピッチ角。画面の SET PITCH 行で選ぶ値。
static float pitch_target_ = 0.0f;
static float roll_target_  = 0.0f;   // APPLY 時に申告するロール角（既定 0）
// マウントから外されたことを検出したあと、APPLY されていない状態か。
// SD の設定に保存して起動をまたいで保持する（充電のために電源を切る運用のため）。
static volatile bool needs_apply_ = false;
// いまこの瞬間マウントから外れた姿勢か（ラッチしない）。警告抑制に使う。
static volatile bool off_mount_now_ = false;
// APPLY の予約。スイッチを押す力でマウント上のデバイスが傾くため、押した瞬間ではなく
// 指を離して落ち着いてから実行する。時刻はジャイロの t_us を使う（millis に依存しない）。
static volatile bool calib_pending_ = false;
static uint32_t      calib_req_us_  = 0;
static float         calib_pitch_   = 0.0f;
static float         calib_roll_    = 0.0f;
static volatile bool calib_done_    = false;
// 較正した日（JST の YYYYMMDD）。0 = 不明。
// ★ ここは attitude.cpp では埋められない。このモジュールは GNSS を知らない
//   （速度も外から押し込む作りになっている）ので、実際の日付は
//   attitude_take_calib_done() を受けた .ino 側が入れる。
//   SD の設定に保存して起動をまたいで保持する。
// ★ volatile。書くのは Core1（loadSettings）と Core0（APPLY 直後）、
//   読むのは Core0（.ino の発報と画面）。needs_apply_ と同じ理由で、
//   ループの中で値をレジスタに抱え込まれると測位後の変化を取りこぼす。
static volatile uint32_t calib_date_ = 0;

// ---- Roll/Pitch/Yaw 機能のマスタースイッチ ----
// ESKF から得た姿勢を「使う」機能を一括で ON/OFF する。既定 ON。
// OFF にすると地図上の姿勢表示・各種警報・自動トリム・風推定・姿勢ログが全て止まり、
// 見た目も動作も ESKF 導入前と同じになる。ESKF の計算自体は止めない
// （2 ページ目の診断表示と、再び ON にしたときの即応性のため）。
static volatile bool rpy_enabled_ = true;

// ---- 風の推定 ----
// Core0（ESKF）が書き、表示が読む。
static volatile bool  wind_enabled_ = true;    // 設定画面で ON/OFF（既定 ON）
static float    wind_e_ = 0.0f, wind_n_ = 0.0f; // 平滑化した風ベクトル ENU [m/s]
static float    wind_fill_s_ = 0.0f;            // 平滑化がたまった秒数

// ---- 平均ピッチ ----
// 瞬時値は周期 2〜6.7 秒の振動が主で std 1.23 度あるので、巡航のトリム状態は平均で見る。
static float pitch_avg_ = 0.0f;
static float pitch_avg_fill_s_ = 0.0f;    // 平均がたまった時間 [s]

// ---- 直進中のロール自動トリム ----
// 直進中のロールは理論上 0。ズレをゆっくり戻して誤警報の連発を防ぐ。
// 旋回中は一切働かないので、真のバンクは消さない。
// Core0（設定画面・ESKF）が書き、Core1（設定保存）も読むので volatile
static volatile bool roll_trim_enabled_ = true;   // 設定画面で ON/OFF（既定 ON）
static float    roll_trim_deg_ = 0.0f;    // 累積補正量（出力から引く）
// 連続して「同符号・不感帯超過」だった窓の情報。
// ROLL_TRIM_CONFIRM_WINDOWS 個ぶん溜まったら補正する。
static float    trim_run_sum_   = 0.0f;   // 連続中の窓の平均ロールの合計
static int      trim_run_count_ = 0;      // 連続した窓の数（0 = 連続なし）
static float    trim_roll_sum_ = 0.0f;    // 直進中のロール積算
static float    trim_time_s_   = 0.0f;    // 直進が続いた時間 [s]
static float    yaw_rate_lp_   = 0.0f;    // ワールド系ヨーレートの平滑値 [deg/s]
// ---- 対気速度モデルの係数 ----
// 既定は settings.h。実際の値は SD の settings.txt から上書きできる（機体依存のため）。
// ★ volatile。書くのは Core1（loadSettings）、読むのは Core0（下の airspeed_from_pitch）。
//   非 volatile だと、飛行中ずっと回るこのループで値をレジスタに抱え込まれうる。
static volatile float airspeed_v0_  = AIRSPEED_V0_MPS;
static volatile float airspeed_k_   = AIRSPEED_CURVE_K_DEG;
static volatile float airspeed_min_ = AIRSPEED_MIN_MPS;
static volatile float airspeed_max_ = AIRSPEED_MAX_MPS;

// ピッチ [度] から対気速度 [m/s] を引く。式の導出は settings.h のコメント。
//     V(θ) = V0 * sqrt( K / (K + θ) )
// ピッチが上がるほど遅く、下がるほど速い。上下限でクランプし、
// この機体で起こり得ない対気速度を出さないようにする。
static float airspeed_from_pitch(float pitch_deg) {
    // 入力が NaN／無限大なら設計点を返す。比較が全部 false になるので
    // 下のクランプでは捕まえられず、そのまま風の LPF を殺す（下記参照）。
    // ここを抜けた値は必ず有限なので、以降は d の符号だけ見ればよい。
    if (!(pitch_deg > -1000.0f && pitch_deg < 1000.0f)) return airspeed_v0_;
    const float k = airspeed_k_;
    const float d = k + pitch_deg;
    // ★ K + θ がゼロに近づくと発散し、負になると sqrtf() は NaN を返す。
    //   **NaN との比較はすべて false なので、下のクランプを素通りする。**
    //   そのまま風の推定（wind_e_/wind_n_）へ入ると LPF ごと NaN に汚染され、
    //   電源を切るまで風が出なくなる。ここで打ち切る。
    //   物理的には「揚力がゼロになるピッチまで機首を下げた」状況で、
    //   そこまで下げれば対気速度は上限側に張り付く。
    if (d <= 0.01f) return airspeed_max_;
    float v = airspeed_v0_ * sqrtf(k / d);
    // 上下限が逆に設定されていても NaN にはならない（max 側が残るだけ）。
    if (v < airspeed_min_) v = airspeed_min_;
    if (v > airspeed_max_) v = airspeed_max_;
    return v;
}

// 前方宣言。attitude_on_gyro() が 平均ピッチとロール自動トリムの
// 判定に使うが、定義はファイル後方（出力セクション）にあるため。
static void euler_from_state(float &roll, float &pitch, float &yaw);
#if ESKF_LEVEL_ENABLED
// 重力観測（Phase 2）。定義は下。attitude_on_gyro() から先に使う。
static void level_update(const float a[3], float sigma_mps2);
#endif

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
// 静止中の加速度累積（初期姿勢＝重力方向の平均用）。
// ★ GRV の代わり。実測で GRV とロール・ピッチが 0.05 度以内で一致する
//   （20260930 / 20260818 の計 5 セッション。→ docs/imu_sch16t_plan.md）。
static float    acc_sum_[3] = {0, 0, 0};
static uint32_t acc_count_  = 0;

// ---- ヨーの状態 ----
// ★ **ロール・ピッチ（leveled_ = initialized_）とヨー（yaw_set_）を分けている。**
//   重力からはロール・ピッチしか決まらないので、水平化した時点でヨーは未知。
//   バンク警報とバリオはロール・ピッチだけで動くので、ヨーを待たずに立てる。
static bool     yaw_set_    = false;
static uint8_t  yaw_src_    = ATT_YAW_SRC_NONE;
static uint8_t  yaw_arm_    = 0;          // しきい値超過が連続した測位数

// ---- 静止中のジャイロバイアス学習 ----
static float    bias_sum_[3] = {0, 0, 0};
static uint32_t bias_n_      = 0;

// ---- 水平化前に使う「重力方向」の暫定値（センサー軸）----
// ★★ **バリオは起動直後から鉛直加速度を要求する。ESKF の水平化（静止 2 秒）を
//   待たせてはいけない。** 0.983 で attitude_get_up_sensor() を「水平化済みのみ
//   true」にしたところ、**手持ちで静止区間が取れないとバリオが完全に止まった**
//   （2026-10-01 に実機で発覚。インドア 38 秒・|ω| 中央 5.1deg/s で静止 2 秒が無かった）。
//   症状は「高度は追従するのに V/S だけ 0 で固まる」。機序はバリオ側:
//   バリオ KF の初期 P は対角なので、**predict を一度も通らないと P[1][0] が 0 の
//   ままで、気圧観測のゲイン K[1] = P[1][0]/S が 0 になり速度状態が構造的に動かない。**
//   だから「気圧のみに劣化」ではなく「完全停止」だった。
//   対策は 2 段構え。こちらは (1) で、加速度の低域だけで重力方向を常に持っておく。
//   (2) は imu.cpp 側で「上方向が無くても predict は止めない」。
// ★ 旋回中は比力が鉛直からずれるので、これは ESKF の代わりにはならない。
//   あくまで水平化までの繋ぎ（＝ 0.982 までの GRV と同程度の品質）。
static float up_lp_[3] = {0.0f, 0.0f, 1.0f};
static bool  up_lp_valid_ = false;


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


// ワールド Z 軸まわりに δ [rad] だけ回す。**ロール・ピッチは動かない。**
// Rz(δ)·Rz(ψ)Ry(θ)Rx(φ) = Rz(δ+ψ)Ry(θ)Rx(φ) なので ZYX 抽出のヨーだけに δ が乗る。
// 誤差状態 dtheta と同じくワールド系の回転なので**左から**掛ける。
static void rotate_world_z(float q[4], float d_rad) {
    const float qz[4] = { cosf(d_rad * 0.5f), 0.0f, 0.0f, sinf(d_rad * 0.5f) };
    float out[4];
    quat_mul(qz, q, out);
    for (int i = 0; i < 4; i++) q[i] = out[i];
    quat_normalize(q);
}


// ============================================================
// 重力方向への水平化
// ============================================================
// 測った比力 a（静止中なら重力方向）へ姿勢を寄せる。
//   gain = 1.0 … 一発で合わせる（初期化用）
//   gain < 1.0 … 相補フィルタとしてゆっくり寄せる（静止中の維持用）
//
// ★ **回転軸は必ず水平になるのでヨーは動かない。**
//   姿勢が正しければ「a をワールドへ回したもの」は +Z になる。ずれていれば
//   その gw を +Z へ運ぶ回転が補正量で、軸 = gw × ẑ = (gw_y, -gw_x, 0)。
//   Z 成分が構造的に 0 なので、ヨーには一切触らない。
//
// 戻り値 false = 加速度が小さすぎて方向が決まらない（何もしていない）。
static bool level_toward_gravity(const float a[3], float gain) {
    const float n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (n < 1e-3f) return false;

    float R[3][3];
    quat_to_R(q_, R);
    float gw[3];
    for (int i = 0; i < 3; i++)
        gw[i] = (R[i][0]*a[0] + R[i][1]*a[1] + R[i][2]*a[2]) / n;

    // gw を +Z へ運ぶ回転（ワールド系）
    float axis[2] = { gw[1], -gw[0] };
    float s = sqrtf(axis[0]*axis[0] + axis[1]*axis[1]);   // = sin(角度)
    if (s < 1e-7f) {
        if (gw[2] > 0.0f) return true;     // すでに水平。何もしなくてよい
        // 真逆さ。軸が決まらないので任意の水平軸で 180 度回す
        axis[0] = 1.0f; axis[1] = 0.0f; s = 1.0f;
    }
    const float ang = atan2f(s, gw[2]) * gain;
    const float rot[3] = { axis[0] / s * ang, axis[1] / s * ang, 0.0f };
    float dq[4], qn[4];
    quat_from_rotvec(rot, dq);
    quat_mul(dq, q_, qn);                 // ワールド系なので左から
    for (int i = 0; i < 4; i++) q_[i] = qn[i];
    quat_normalize(q_);
    return true;
}


// ============================================================
// 初期化
// ============================================================
// att_sigma_deg はロール・ピッチ、yaw_sigma_deg はヨーの初期標準偏差。
// 誤差状態はワールド系（global error）なので、添字 2 が鉛直軸まわり＝ヨーになる。
// ヨーを別扱いにする理由は settings.h の ESKF_YAW_UNKNOWN_SIGMA_DEG のコメント参照。
static void reset_covariance(float att_sigma_deg, float yaw_sigma_deg) {
    memset(P_, 0, sizeof(P_));
    float a = att_sigma_deg * (float)M_PI / 180.0f;
    float yy = yaw_sigma_deg * (float)M_PI / 180.0f;
    for (int i = 0; i < 2; i++)  P_[i][i]       = a * a;
    P_[2][2]                                    = yy * yy;
    for (int i = 3; i < 6; i++)  P_[i][i]       = 1.0f;                 // 速度 [m/s]²
    for (int i = 6; i < 9; i++)  P_[i][i]       = powf(1.0f * (float)M_PI/180.0f, 2);
    for (int i = 9; i < 12; i++) P_[i][i]       = 0.01f;                // 加速度バイアス
}

void attitude_setup() {
    q_[0] = 1; q_[1] = q_[2] = q_[3] = 0;
    v_[0] = v_[1] = v_[2] = 0;
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(10.0f, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    initialized_ = false;
    accel_valid_ = false;
    acc_count_ = 0;
    for (int i = 0; i < 3; i++) acc_sum_[i] = 0.0f;
    yaw_set_ = false;
    yaw_src_ = ATT_YAW_SRC_NONE;
    yaw_arm_ = 0;
    bias_n_ = 0;
    for (int i = 0; i < 3; i++) bias_sum_[i] = 0.0f;
    // 重力方向の暫定値も捨てる。実機では setup() が 1 回しか呼ばれないので
    // 影響は無いが、残すと「加速度が 1 度も来ていないのに上方向を返す」状態になる
    // （tools/attitude_test が実際に捕まえた）。
    up_lp_valid_ = false;
    up_lp_[0] = up_lp_[1] = 0.0f; up_lp_[2] = 1.0f;
    level_updates_ = 0;
    longacc_lp_ = 0.0f;
    last_gsp_ = -1.0f;
    last_gsp_us_ = 0;
    level_last_us_ = 0;
    yaw_rate_inst_ = 0.0f;
    dt_count_ = 0;
    dt_idx_ = 0;
    dt_est_ = 1.0f / IMU_RATE_GYRO_HZ;
    gnss_updates_ = 0;
    gyro_seen_ = false;
    last_gyro_us_ = 0;
    last_gnss_vel_us_ = 0;
    gnss_vel_seen_ = false;
    pitch_avg_ = 0.0f;
    pitch_avg_fill_s_ = 0.0f;
    // roll_trim_deg_ はここでは触らない。level_roll_off_ と同じく SD から復元する値で、
    // 設定の読み込み（Core1 の setup_sd）と この関数（Core0 の setup）は並行に走るため、
    // ここで 0 にすると復元済みの値を消してしまうことがある。
    // 電源投入時の初期値は静的初期化（= 0.0f）が保証する。
    trim_roll_sum_ = 0.0f;
    trim_time_s_ = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
    yaw_rate_lp_ = 0.0f;
    trim_event_ = false;
    wind_e_ = wind_n_ = 0.0f;
    wind_fill_s_ = 0.0f;
    calib_pending_ = false;
    calib_done_    = false;
}


// IMU が途絶していないか。ジャイロが来なくなったら伝播が止まるため、
// そのまま GNSS 観測だけ入れると姿勢が誤って回される（下記 attitude_on_gnss_velocity 参照）。
static bool imu_fresh() {
    if (!gyro_seen_) return false;
    return (uint32_t)(time_us_32() - last_gyro_us_) < ESKF_IMU_TIMEOUT_US;
}

// GNSS の速度観測が生きているか。
// ★ **v_ は観測が無いと加速度バイアスを積分して際限なく育つ。**
//   「観測の無い区間も慣性で埋まっている」のは短い欠測を跨ぐための性質であって、
//   GNSS が最初から無い状態（屋内・アンテナ未接続）では**ただのドリフト**になる。
//   実機で踏んだ: 机の上で数分放置したら |v_| が 3m/s を超え、
//   ROLL_TRIM_MIN_SPEED を満たしてしまい、静止した機体に自動ロールトリムが
//   -0.5 度ずつ入り続けた（7 分で -1.5 度）。飛ぶ前から曲がった機体になる。
//   自動トリムと風推定は「飛行中」の機能なので、観測の鮮度を条件に加える。
static bool gnss_vel_fresh() {
    if (!gnss_vel_seen_) return false;
    return (uint32_t)(time_us_32() - last_gnss_vel_us_) < ESKF_GNSS_VEL_TIMEOUT_US;
}

// 重力方向から姿勢を初期化する。**ロール・ピッチだけが決まり、ヨーは未知。**
// ヨーは後で GNSS 航跡から入れる（init_yaw_from_track）。
//   a             : 重力方向とみなす比力（静止平均、または走行中なら直近値）
//   att_sigma_deg : ロール・ピッチの初期σ
static bool init_level(const float a[3], float att_sigma_deg) {
    q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
    if (!level_toward_gravity(a, 1.0f)) return false;
    v_[0] = v_[1] = v_[2] = 0.0f;
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(att_sigma_deg, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    initialized_ = true;      // = 水平化済み。ロール・ピッチとバリオはここから有効
    yaw_set_     = false;
    yaw_src_     = ATT_YAW_SRC_NONE;
    yaw_arm_     = 0;
    bias_sum_[0] = bias_sum_[1] = bias_sum_[2] = 0.0f;
    bias_n_ = 0;
    return true;
}

// GNSS 航跡でヨーだけを差し替える。ロール・ピッチは動かない。
//   出力方位は heading = 90 - yaw_math[deg] なので d(heading)/d(yaw_math) = -1。
//   ワールド Z を α 回すと yaw_math が +α 動くので α = -(want - now)。
//   （符号を間違えやすいので tools/attitude_test で数値検証している）
static void init_yaw_from_track(float track_deg, float yaw_sigma_deg) {
    float r, p, y;
    euler_from_state(r, p, y);                  // いまの出力方位
    float d = track_deg - y;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    rotate_world_z(q_, -d * (float)M_PI / 180.0f);

    // ヨーを外部情報で置き換えたので、ヨーの分散を入れ直し、
    // **他の状態との相関は意味を失うので落とす。**
    const float s = yaw_sigma_deg * (float)M_PI / 180.0f;
    for (int i = 0; i < N_ERR; i++) { P_[2][i] = 0.0f; P_[i][2] = 0.0f; }
    P_[2][2] = s * s;

    yaw_set_ = true;
    yaw_src_ = ATT_YAW_SRC_GNSS;
}

// 静止中にジャイロのゼロ点を測って bg_ に入れる。
// 引数は**生の**ジャイロ（bg_ を引く前）。目的と注意は settings.h の
// ESKF_BIAS_LEARN_SAMPLES のコメント参照。
static void learn_gyro_bias(const float g[3]) {
    for (int i = 0; i < 3; i++) bias_sum_[i] += g[i];
    if (++bias_n_ < ESKF_BIAS_LEARN_SAMPLES) return;
    for (int i = 0; i < 3; i++) {
        float b = bias_sum_[i] / (float)bias_n_;
        if (b >  ESKF_BIAS_LEARN_MAX_RADS) b =  ESKF_BIAS_LEARN_MAX_RADS;
        if (b < -ESKF_BIAS_LEARN_MAX_RADS) b = -ESKF_BIAS_LEARN_MAX_RADS;
        bg_[i] = b;
        bias_sum_[i] = 0.0f;
    }
    bias_n_ = 0;
}


// ============================================================
// センサー入力
// ============================================================
void attitude_on_accel(const float a[3]) {
    accel_last_[0] = a[0]; accel_last_[1] = a[1]; accel_last_[2] = a[2];
    accel_valid_ = true;

    // ---- 重力方向の暫定値を**常に**更新する（水平化を待たない）----
    // 理由は up_lp_ の宣言のコメント（バリオが止まる件）。
    // |a| が g から大きく外れている間（持ち上げ・衝撃）は更新しない。
    {
        const float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        if (an > 1.0f && fabsf(an - GRAVITY) < ESKF_UP_LP_ACC_TOL) {
            // 初回は一発で入れる（起動直後からバリオを動かすため）
            const float k = up_lp_valid_ ? ESKF_UP_LP_GAIN : 1.0f;
            for (int i = 0; i < 3; i++) up_lp_[i] += (a[i] / an - up_lp_[i]) * k;
            const float n = sqrtf(up_lp_[0]*up_lp_[0] + up_lp_[1]*up_lp_[1] + up_lp_[2]*up_lp_[2]);
            if (n > 1e-6f) { for (int i = 0; i < 3; i++) up_lp_[i] /= n; up_lp_valid_ = true; }
        }
    }

    // 静止中は平均を溜める。これが初期姿勢の重力方向になる（旧 GRV の平均の代わり）。
    if (!static_now_ || initialized_) return;
    for (int i = 0; i < 3; i++) acc_sum_[i] += a[i];
    acc_count_++;
}

uint8_t attitude_get_yaw_source() { return yaw_src_; }
bool    attitude_yaw_ready()      { return yaw_set_; }

// センサー座標系で見た「上」方向 = 回転行列の第 3 行。
// ★ imu.cpp の compute_earth_z_accel() が GRV から作っていたものと**同じ量**。
//   バリオはこれと比力の内積で鉛直加速度を取る。
// ★ **水平化前でも使える値を返すこと。** 呼び出し側（バリオ）は起動直後から
//   鉛直加速度を要求する。false を返して predict を止めさせると V/S が 0 に
//   固まる（up_lp_ の宣言のコメント参照。2026-10-01 に実機で発覚）。
//   戻り値 false は「加速度がまだ 1 度も来ていない」だけを意味する。
bool attitude_get_up_sensor(float u[3]) {
    if (initialized_) {
        const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
        u[0] = 2.0f * (x*z - w*y);
        u[1] = 2.0f * (y*z + w*x);
        u[2] = 1.0f - 2.0f * (x*x + y*y);
        return true;
    }
    if (up_lp_valid_) {          // 水平化までの繋ぎ（加速度の低域）
        for (int i = 0; i < 3; i++) u[i] = up_lp_[i];
        return true;
    }
    u[0] = u[1] = 0.0f; u[2] = 1.0f;
    return false;
}



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
        acc_count_ = 0;
        for (int i = 0; i < 3; i++) acc_sum_[i] = 0.0f;
    }
    if (!now) {
        // 動き出したらバイアスの溜めかけを捨てる（中途半端な平均を入れない）
        bias_n_ = 0;
        for (int i = 0; i < 3; i++) bias_sum_[i] = 0.0f;
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

    // 予約された APPLY を、要求から ESKF_APPLY_DELAY_US 経過してから実行する。
    if (calib_pending_ && initialized_ &&
        (uint32_t)(t_us - calib_req_us_) >= ESKF_APPLY_DELAY_US) {
        calib_pending_ = false;
        attitude_calibrate_to(calib_pitch_, calib_roll_);
        calib_done_ = true;      // 呼び出し側が保存と音声再生を行う
    }

    // ---- 初期化（水平化）----
    if (!initialized_) {
        if (static_now_ && (t_us - static_since_us_) >= ESKF_STATIC_INIT_US &&
            acc_count_ >= 4) {
            const float a_avg[3] = { acc_sum_[0] / (float)acc_count_,
                                     acc_sum_[1] / (float)acc_count_,
                                     acc_sum_[2] / (float)acc_count_ };
            init_level(a_avg, ESKF_INIT_ATT_SIGMA_DEG);
        }
        if (!initialized_) return;   // まだ初期化できていないので伝播しない
    }

    if (!accel_valid_) return;

    // ---- 待機中の維持（ヨー未設定のあいだ）----
    // ★ **ヨーが未設定のあいだ GNSS 速度観測を入れないので、観測が 1 つも無い。**
    //   放っておくとロール・ピッチはジャイロオフセットの積分で漂う
    //   （SCH16T なら 0.1deg/s typ で 10 分 60 度）。静止しているあいだは
    //   重力へ寄せ続け、同時にジャイロのゼロ点を測る。理由と数値は settings.h の
    //   ESKF_RELEVEL_GAIN / ESKF_BIAS_LEARN_SAMPLES のコメント。
    // ★ Phase 2 の重力観測（level_update）とここは**併存させている。**役割が違う:
    //   ・ここ（相補フィルタ・50Hz・時定数 1 秒）… 待機中の維持。下の
    //     reset_covariance が毎サンプル P を初期値へ戻すので、level_update が
    //     溜めた情報はどうせ捨てられる。実効の引き戻しはこちらが担う
    //   ・level_update（正式な観測・2Hz）… 飛行中と GNSS 断中の錨
    //   どちらも重力へ寄せる方向なので競合しない。1 つに統合するのは、
    //   待機中の挙動（tools/attitude_test の D1/D2）を再検証してからにする。
    if (!yaw_set_ && static_now_) {
        level_toward_gravity(accel_last_, ESKF_RELEVEL_GAIN);
        learn_gyro_bias(g);          // ★ bg_ を引く前の生の値を渡すこと
        // 観測が無いので共分散を伝播させても情報は増えない。
        // 水平化し続けている事実と釣り合うよう初期値へ置き直す。
        reset_covariance(ESKF_INIT_ATT_SIGMA_DEG, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    }

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
    // ---- ワールド系のヨーレート（旋回しているか）----
    // ボディ角速度を回して Z 成分を取る。自動ロールトリム・風推定・重力観測のゲートで共用。
    // ★★ **重力観測のゲートより前で更新すること。** 後ろに置くとゲートが
    //   1 サンプル前（20ms 前）の値で判定し、**旋回の入り口で観測が漏れる**
    //   （5 度の協調旋回 30 秒で 1 回漏れるのを tools/attitude_test の F2 が捕まえた）。
    {
        const float wz = R[2][0]*w[0] + R[2][1]*w[1] + R[2][2]*w[2];
        const float wz_dps = wz * 180.0f / (float)M_PI;
        yaw_rate_inst_ = wz_dps;
        yaw_rate_lp_ += (wz_dps - yaw_rate_lp_) * (dt / (2.0f + dt));
    }

    // ---- 重力観測（Phase 2）----
    // ★ 根拠と数値は settings.h の ESKF_LEVEL_* のコメント。要点だけ:
    //   主ゲートは**旋回レート**（ジャイロだけから作れるので GNSS 断中も働く）。
    //   `|f| ≒ g` は人力機のバンク角を判別できないので粗い健全性チェックのみ。
    //   前後加速度は GNSS からしか作れないので、断中はその条件を省く。
#if ESKF_LEVEL_ENABLED
    if ((uint32_t)(t_us - level_last_us_) >= ESKF_LEVEL_INTERVAL_US) {
        const float amag = sqrtf(accel_last_[0]*accel_last_[0]
                               + accel_last_[1]*accel_last_[1]
                               + accel_last_[2]*accel_last_[2]);
        bool pass = (fabsf(yaw_rate_lp_)   <= ESKF_LEVEL_MAX_YAWRATE_DPS)
                 && (fabsf(yaw_rate_inst_) <= ESKF_LEVEL_MAX_YAWRATE_INST)
                 && (fabsf(amag - GRAVITY) <= ESKF_LEVEL_ACCMAG_TOL_MPS2);
        // 前後加速度は測位があるときだけ見る（無いときは条件ごと省く）
        if (pass && last_gsp_ >= 0.0f && gnss_vel_fresh())
            pass = fabsf(longacc_lp_) <= ESKF_LEVEL_MAX_LONGACC_MPS2;
        if (pass) {
            level_update(accel_last_, ESKF_LEVEL_SIGMA_MPS2);
            level_last_us_ = t_us;
        }
    }
#endif

    // ---- 平均ピッチと直進中のロール自動トリムを更新 ----
    // ここは公称状態が更新された後に呼ぶ（共分散の計算とは独立）。
    {
        float r, p, y;
        euler_from_state(r, p, y);
        p -= level_pitch_off_;
        r -= level_roll_off_ + roll_trim_deg_;

        // マウントから外されたか。機体に付いている限りこの角度には達しない。
        // 検出したら、次に APPLY するまで「較正が無効」の警告を出し続ける。
        // ここは較正オフセット適用後の値で見る（生値だとマウント自体の傾きで誤検出する）。
        // 右バンク（r > 0）だけしきい値を緩める。プラットホームへ上げるときの
        // 大きな右バンクを「外した」と誤判定しないため。左バンクとピッチは従来どおり。
        const float roll_limit = (r > 0.0f) ? OFF_MOUNT_RIGHT_ROLL_DEG : OFF_MOUNT_DEG;
        // ★ **「いま外れている」と「外されたことがある」を分けて持つ。**
        //   needs_apply_ は APPLY するまで下がらないラッチで、較正のやり直しを促すもの。
        //   一方 off_mount_now_ は**今この瞬間の姿勢**で、充電中の警告抑制に使う
        //   （GPS_TFT_map.ino の warn_muted_off_mount）。しきい値をここ 1 か所に保つため
        //   判定はこの中で行い、外へは真偽値だけ出す。
        off_mount_now_ = rpy_enabled_ &&
                         (fabsf(r) > roll_limit || fabsf(p) > OFF_MOUNT_DEG);
        if (off_mount_now_) needs_apply_ = true;

        // 30 秒の一次遅れで平均ピッチを作る
        const float a = dt / (PITCH_AVG_SEC + dt);
        pitch_avg_ += (p - pitch_avg_) * a;
        if (pitch_avg_fill_s_ < PITCH_AVG_SEC * 2.0f) pitch_avg_fill_s_ += dt;

        const float sp = sqrtf(v_[0]*v_[0] + v_[1]*v_[1]);
        // 直進しているか。ロール自動トリムと風推定で共用する。
        // ★ sp は GNSS の対地速度ではなく **ESKF が積分した速度** なので、
        //   観測が無いと育つ（gnss_vel_fresh() のコメント参照）。鮮度を必ず併せて見る。
        const bool straight_flight = gnss_vel_fresh() &&
                                     (fabsf(yaw_rate_lp_) < ROLL_TRIM_YAWRATE_DPS) &&
                                     (sp > ROLL_TRIM_MIN_SPEED);

        // ---- 風の推定 ----
        // 対気速度の大きさはピッチから引き、向きは機首方位とする。
        //   風ベクトル = 対地速度ベクトル - 対気速度ベクトル
        // 対地速度は ESKF の推定値 v_（ENU）を使う。GNSS 生値より滑らかで、
        // 観測の無い区間も慣性で埋まっている。
        // 使うピッチは 平均。瞬時値は std 1.23 度の振動があり、そのまま入れると
        // 対気速度が 5.8〜9.0 の全域を往復してしまう。
        //
        // ★ 直進中だけ更新する。「対気速度の向き＝機首方位」は横滑り 0 の仮定であり、
        //   旋回中の人力飛行機はラダー権限が弱く大きく横滑りする。横滑り角 beta が
        //   あると風速の誤差は 対気速度 × sin(beta) になり、beta=30 度なら 3.5m/s に達する。
        //   実測（2026 大会）でも旋回中は直進中より 0.86m/s ずれていた。
        //   風はゆっくりしか変わらないので、旋回中は更新を止めて直前の値を保持すればよい。
        //   同飛行の 94% は直進だったので、捨てる情報はごくわずか。
        //   ※ ESKF のヨー推定自体は beta の影響を受けない（空力の仮定を含まないため）。
        if (rpy_enabled_ && wind_enabled_ && straight_flight &&
            pitch_avg_fill_s_ >= PITCH_AVG_SEC) {
            const float va  = airspeed_from_pitch(pitch_avg_);
            // y は真方位（北=0・東=90）。ENU では E=sin, N=cos。
            const float ya  = y * (float)M_PI / 180.0f;
            const float we  = v_[0] - va * sinf(ya);
            const float wn  = v_[1] - va * cosf(ya);
            const float aw  = dt / (WIND_LPF_SEC + dt);
            wind_e_ += (we - wind_e_) * aw;
            wind_n_ += (wn - wind_n_) * aw;
            if (wind_fill_s_ < WIND_LPF_SEC) wind_fill_s_ += dt;
        } else if (!rpy_enabled_ || !wind_enabled_ || sp <= WIND_MIN_SPEED_MPS) {
            // 地上・OFF のときだけ捨てる。旋回中は「保持」であって破棄ではない。
            wind_fill_s_ = 0.0f;
            wind_e_ = wind_n_ = 0.0f;
        }

        const bool straight = straight_flight && rpy_enabled_ && roll_trim_enabled_;
        if (straight) {
            trim_roll_sum_ += r * dt;
            trim_time_s_   += dt;
            if (trim_time_s_ >= ROLL_TRIM_WINDOW_S) {
                const float avg = trim_roll_sum_ / trim_time_s_;
                // ROLL_TRIM_CONFIRM_WINDOWS 窓連続で同符号・不感帯超過のときだけ補正する。
                // 60 秒窓の平均ロールは実測で std 0.27 度あり、不感帯 0.5 度でも
                // 約 10% の窓がノイズだけで超える。1 窓で補正すると 50 分の飛行で
                // ±0.7 度ほどランダムウォークするため、一致を要求して潰す。
                const bool over = (fabsf(avg) >= ROLL_TRIM_DEADBAND_DEG);
                if (!over) {
                    // 不感帯の内側 → 連続は途切れる
                    trim_run_count_ = 0;
                    trim_run_sum_   = 0.0f;
                } else if (trim_run_count_ > 0 && ((avg > 0.0f) != (trim_run_sum_ > 0.0f))) {
                    // 符号が反転した → これまでの連続は無効。今回の窓から数え直す
                    trim_run_count_ = 1;
                    trim_run_sum_   = avg;
                } else {
                    trim_run_count_++;
                    trim_run_sum_  += avg;
                }
                if (trim_run_count_ >= ROLL_TRIM_CONFIRM_WINDOWS) {
                    // 補正量は連続した窓の平均。N 窓ぶん平均するので
                    // 1 窓だけより測定ノイズが 1/√N になる。
                    float step = trim_run_sum_ / (float)trim_run_count_;
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
                    // 使った観測は捨てる。次の補正には新たに N 窓ぶん必要。
                    trim_run_count_ = 0;
                    trim_run_sum_   = 0.0f;
                }
                // 窓だけリセットする。trim_run_count_ はここで触らないこと
                // （まだ N 窓に届いていない連続を消してしまう）。
                trim_roll_sum_ = 0.0f;
                trim_time_s_   = 0.0f;
            }
        } else {
            // 旋回に入ったら積算を捨てる（旋回中のロールを平均に混ぜない）。
            // 直前の窓の平均は保持する。取り付けのズレは静的なので、旋回を挟んでも
            // その前後の窓は同じ量を測っているとみなしてよい。
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

    // ★ ヨーの分散だけ頭打ちにする（理由は settings.h の ESKF_YAW_SIGMA_MAX_DEG）。
    //   ロール・ピッチは重力で常に観測されるので育たない。ヨーだけが
    //   観測の入らない状態で伸び続けるので、ここで止める。
    //   **縮む側は触らない。**観測が入れば通常どおり小さくなる。
    {
        const float ymax = ESKF_YAW_SIGMA_MAX_DEG * (float)M_PI / 180.0f;
        const float yvar_max = ymax * ymax;
        if (P_[2][2] > yvar_max) P_[2][2] = yvar_max;
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
// 定義は下にあるが level_update() から先に使うため前方宣言する。
static void clamp_bias(float b[3], float lim);

#if ESKF_LEVEL_ENABLED
// ============================================================
// 重力観測（levelling update）— Phase 2
// ============================================================
// 準定常のとき、比力をワールドへ回した f_w は (0,0,g) になるはず。
// その水平 2 成分を観測量（期待値 0）にして dtheta の水平成分を直す。
//
//   誤差回転はワールド系（R_true = (I + [dtheta]x) R_nom）なので
//       f_w_true = f_w + dtheta x f_w = f_w - [f_w]x dtheta
//   f_w ~ (0,0,g) のとき
//       d f_w_x = +g * dtheta_y,   d f_w_y = -g * dtheta_x
//   よって H は 2x12 で H[0][1] = +g, H[1][0] = -g だけが非零。
//   **Z 成分（ヨー）には構造的に効かない。**
//
// ★ **dba は H に入れない。** 水平の加速度バイアスと傾き誤差は原理的に縮退して
//   いるので、入れると長い GNSS 断で ba が汚れる。入れない代償は「水平 ba を
//   全部傾きのせいにする」ぶんの偏りで、SCH16T のオフセット ±0.01 m/s² typ なら
//   0.06 度、max ±0.06 でも 0.35 度。無視できる。そのぶんは σ に含める。
//
// ★ ゲートは呼び出し側（attitude_on_gyro）で判定する。根拠と数値は settings.h の
//   ESKF_LEVEL_* のコメント。**`|f| ≒ g` では人力機のバンク角を判別できない**ので、
//   主ゲートは旋回レート。
//
// H が疎なので行列積は展開して書く（12x12 の掛け算を 2 回避ける）。
static void level_update(const float a[3], float sigma_mps2) {
    const float g = GRAVITY;

    // f_w = R (a - ba)。R は q_ から。
    float R[3][3];
    quat_to_R(q_, R);
    const float f[3] = { a[0] - ba_[0], a[1] - ba_[1], a[2] - ba_[2] };
    float fw[3];
    for (int i = 0; i < 3; i++)
        fw[i] = R[i][0]*f[0] + R[i][1]*f[1] + R[i][2]*f[2];

    // ★ H は f_w ≒ (0,0,+g) のまわりで線形化してある。**f_w が下向きなら符号が
    //   逆になり、補正が誤差を広げる方向に働く。** 姿勢推定が 90 度以上
    //   おかしくなっている状況なので、そのときは観測を入れない（安全側）。
    //   人力飛行機は背面にならないので、通常運用でここに来ることは無い。
    if (fw[2] < 0.5f * g) return;

    const float r = sigma_mps2 * sigma_mps2;

    // S = H P H^T + R  （H[0] は P の 1 行、H[1] は P の 0 行を拾う）
    const float g2 = g * g;
    const float S00 =  g2 * P_[1][1] + r;
    const float S01 = -g2 * P_[1][0];
    const float S10 = -g2 * P_[0][1];
    const float S11 =  g2 * P_[0][0] + r;
    const float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-12f) return;
    const float inv = 1.0f / det;
    const float Si00 =  S11 * inv, Si01 = -S01 * inv;
    const float Si10 = -S10 * inv, Si11 =  S00 * inv;

    // P H^T は 12x2 で、m 行目 = [ +g*P[m][1], -g*P[m][0] ]
    // K = P H^T S^-1
    static float K[N_ERR][2];
    for (int m = 0; m < N_ERR; m++) {
        const float p0 =  g * P_[m][1];
        const float p1 = -g * P_[m][0];
        K[m][0] = p0 * Si00 + p1 * Si10;
        K[m][1] = p0 * Si01 + p1 * Si11;
    }

    // y = 0 - f_w の水平成分
    const float y0 = -fw[0], y1 = -fw[1];

    // ---- P = (I-KH) P (I-KH)^T + K R K^T （Joseph 形。速度観測と同じ流儀）----
    // KH は列 0 と 1 だけ非零: KH[m][0] = -g*K[m][1], KH[m][1] = +g*K[m][0]
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++) {
            float kh = 0.0f;
            if      (n == 0) kh = -g * K[m][1];
            else if (n == 1) kh =  g * K[m][0];
            T1_[m][n] = ((m == n) ? 1.0f : 0.0f) - kh;
        }
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++) {
            float sum = 0;
            for (int k = 0; k < N_ERR; k++) sum += T1_[m][k] * P_[k][n];
            T2_[m][n] = sum;
        }
    for (int m = 0; m < N_ERR; m++)
        for (int n = 0; n < N_ERR; n++) {
            float sum = 0;
            for (int k = 0; k < N_ERR; k++) sum += T2_[m][k] * T1_[n][k];
            P_[m][n] = sum + r * (K[m][0]*K[n][0] + K[m][1]*K[n][1]);
        }

    // ---- 誤差の注入（速度観測と同じ）----
    float dx[N_ERR];
    for (int m = 0; m < N_ERR; m++) dx[m] = K[m][0]*y0 + K[m][1]*y1;
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
    level_updates_++;
}
#endif  // ESKF_LEVEL_ENABLED


static void clamp_bias(float b[3], float lim) {
    float n = sqrtf(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
    if (n > lim && n > 1e-9f) {
        float s = lim / n;
        b[0] *= s; b[1] *= s; b[2] *= s;
    }
}

void attitude_on_gnss_velocity(float velN, float velE, float velD, float sAcc) {
    // ---- 前後加速度の持続成分（重力観測のゲート用）----
    // ★ ここは観測を取り込むかどうかに関係なく常に更新する。ゲートの材料なので、
    //   ヨー未設定でも計算しておく。GNSS が無い間は last_gsp_ が古くなるので、
    //   ゲート側で gnss_vel_fresh() と併せて見る。
    {
        const float gsp = sqrtf(velN*velN + velE*velE);
        const uint32_t now_us = time_us_32();
        if (last_gsp_ >= 0.0f) {
            const float dtg = (float)(now_us - last_gsp_us_) * 1e-6f;
            if (dtg > 0.05f && dtg < 5.0f) {
                const float acc_long = (gsp - last_gsp_) / dtg;
                longacc_lp_ += (acc_long - longacc_lp_) * (dtg / (2.0f + dtg));
            }
        }
        last_gsp_    = gsp;
        last_gsp_us_ = now_us;
    }

    // IMU が途絶しているときは観測を取り込まない。
    // 伝播（ジャイロ・加速度）が止まった状態で速度観測だけ入れると、
    // フィルタは速度の食い違いを姿勢誤差のせいにして姿勢を回し続ける。
    // 実測（模擬）では水平のまま加速しただけで 60 秒後にロールが +8 度まで育った。
    if (!imu_fresh()) return;

    // NED → ENU
    const float z[3] = { velE, velN, -velD };

    if (!initialized_) {
        // 静止区間が無いまま動き出した場合の保険。直近の比力で水平化する。
        // 静止平均ほど信用できない（旋回・加減速の分が混じる）ので σ を大きく取り、
        // GNSS 観測で速く引き戻せるようにする。
        // （2026-08-18 の session 2 は屋内起動→走行中に記録開始で静止区間が無く、
        //   初期姿勢誤差がジャイロバイアスに吸われて 5.9deg/s に固着した）
        if (!accel_valid_) return;
        if (!init_level(accel_last_, ESKF_INIT_ATT_SIGMA_MOVING_DEG)) return;
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2];
        return;
    }

    // ---- ヨーが未設定なら、先に GNSS 航跡から入れる ----
    // ★ **ヨーが未知のまま速度観測を入れてはいけない。**
    //   観測は H = [0 I 0 0] の速度のみで、姿勢へは d(dv)/dt = -[Rf]x dtheta
    //   の結合を通してしか効かない。ヨーが例えば 90 度ずれていると、加速時の
    //   速度残差は予測と直交する方向に出るので、フィルタは**ロール・ピッチを
    //   回して合わせに行く**（試算で数度）。ヨーを入れるまでは観測を通さない。
    //
    //   しきい値は飛行速度より十分低く取ってあるので、注入の瞬間に機体は必ず
    //   接地している。接地中は横滑りできないので航跡＝機首方位が厳密に成立する。
    //   詳細は settings.h の ESKF_YAW_INIT_MIN_SPEED_MPS のコメント。
    if (!yaw_set_) {
        const float sp = sqrtf(z[0]*z[0] + z[1]*z[1]);
        if (sp < ESKF_YAW_INIT_MIN_SPEED_MPS) { yaw_arm_ = 0; return; }
        if (++yaw_arm_ < ESKF_YAW_INIT_ARM_FIXES) return;  // 単発の化けで焼き付けない
        // ENU なので航跡（真方位・北 0・時計回り）は atan2(East, North)
        float trk = atan2f(z[0], z[1]) * 180.0f / (float)M_PI;
        if (trk < 0.0f) trk += 360.0f;
        init_yaw_from_track(trk, ESKF_YAW_INIT_SIGMA_DEG);
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2];
        return;                     // この回は観測として使わない（速度は直接入れた）
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
    last_gnss_vel_us_ = time_us_32();
    gnss_vel_seen_    = true;
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
// mount_correct() と同じ変換であること（実体は attitude.h の imu_body_euler_rad）。
static void euler_from_state(float &roll, float &pitch, float &yaw) {
    // ★ マウント回転はクォータニオンの段階で掛ける（imu_body_euler_rad）。
    //   **Euler を組み替える旧方式には戻さないこと。**v7 の配置では機体が水平の
    //   ときセンサーがジンバルロックに入り、角の入れ替えでは原理的に直らない
    //   （settings.h の IMU_MOUNT_Q* のコメント参照）。
    float r, p, y;
    imu_body_euler_rad(q_[0], q_[1], q_[2], q_[3], r, p, y);
    const float rad2deg = 180.0f / (float)M_PI;
    roll  = r * rad2deg;
    pitch = p * rad2deg;
    yaw   = imu_yaw_to_heading_deg(y);   // 東基準の数学ヨー → 真方位（北=0・時計回り正）
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
// ============================================================
//  SD の settings.txt から復元する角度の検証
// ============================================================
// ★ **NaN と inf を必ず落とすこと。** setter に来る値は atof() の結果で、
//   atof("nan") は NaN を、atof("1e999") は inf をそのまま返す。
//   NaN が level_*_off_ や roll_trim_deg_ に入ると、表示・自動トリム・
//   バンク角警告のすべてが NaN になるうえ、**NaN はどの比較も false になるので
//   しきい値の警告に一つも引っかからない**。つまり黙って姿勢機能が死ぬ。
//   単純な上下クランプ（`if (v > hi)`）だけでは NaN を素通しするので足りない。
//
// ※ NaN のときは 0 を返す。下で使う 3 つの範囲はいずれも 0 を含むので、
//   0 は常に「範囲内かつ中立」な値になっている。
static float clamp_setting_deg(float v, float lo, float hi) {
    if (v != v) return 0.0f;         // NaN
    if (v < lo)  return lo;          // -inf もここで潰れる
    if (v > hi)  return hi;          // +inf もここで潰れる
    return v;
}

float attitude_get_roll_trim_deg() { return roll_trim_deg_; }
void  attitude_set_roll_trim_deg(float deg) {
    // 壊れた設定ファイルでとんでもない値が入っても、飛行中の姿勢表示を狂わせないよう
    // 通常動作と同じ上限でクランプする。
    roll_trim_deg_ = clamp_setting_deg(deg, -ROLL_TRIM_LIMIT_DEG, ROLL_TRIM_LIMIT_DEG);
}
// 設定ファイルは行の順序が保証されない（roll_trim が needs_apply より先に来ることがある）ため、
// 読み込みが全部終わってからまとめて判定する。
void  attitude_finish_settings_load() {
    if (roll_trim_deg_ == 0.0f) return;
    // 復元した累積補正量を捨てる条件:
    //  ・needs_apply … マウントから外したまま APPLY していない。取り付け角が
    //                  変わった可能性があるので、前回の補正量は当てにならない。
    //  ・機能が OFF  … ON/OFF の設定行が roll_trim より後に来た場合に、
    //                  OFF なのに補正量だけ残るのを防ぐ（setter 側と同じ扱いにする）。
    if (needs_apply_ || !roll_trim_enabled_ || !rpy_enabled_) {
        roll_trim_deg_ = 0.0f;
        trim_run_count_ = 0;
        trim_run_sum_   = 0.0f;
    }
}

bool attitude_get_wind(float &speed_mps, float &dir_to_deg) {
    if (!wind_enabled_ || wind_fill_s_ < WIND_LPF_SEC) return false;
    speed_mps = sqrtf(wind_e_*wind_e_ + wind_n_*wind_n_);
    // 「風が吹いていく方向」の真方位（北=0、東=90）
    dir_to_deg = atan2f(wind_e_, wind_n_) * 180.0f / (float)M_PI;
    if (dir_to_deg < 0.0f) dir_to_deg += 360.0f;
    return true;
}
bool  attitude_get_rpy_enabled() { return rpy_enabled_; }
void  attitude_set_rpy_enabled(bool on) {
    rpy_enabled_ = on;
    if (!on) {
        // OFF にしたら派生状態を捨てる。再び ON にしたとき古い値が一瞬出ないように。
        roll_trim_deg_ = 0.0f;
        trim_roll_sum_ = 0.0f;
        trim_time_s_   = 0.0f;
        trim_run_count_ = 0;
        trim_run_sum_   = 0.0f;
        wind_e_ = wind_n_ = 0.0f;
        wind_fill_s_ = 0.0f;
    }
}

bool  attitude_get_wind_enabled() { return wind_enabled_; }
void  attitude_set_wind_enabled(bool on) {
    wind_enabled_ = on;
    if (!on) { wind_fill_s_ = 0.0f; wind_e_ = wind_n_ = 0.0f; }
}
bool  attitude_get_roll_trim_enabled() { return roll_trim_enabled_; }
void  attitude_set_roll_trim_enabled(bool on) {
  roll_trim_enabled_ = on;
  if (!on) {
    // OFF にしたら補正を捨てて素の測定値に戻す（半端に効いたままだと紛らわしい）
    roll_trim_deg_ = 0.0f;
    trim_roll_sum_ = 0.0f;
    trim_time_s_   = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
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
uint32_t attitude_get_level_updates()    { return level_updates_; }

float attitude_get_static_secs() {
    if (!static_now_) return 0.0f;
    return (float)(time_us_32() - static_since_us_) * 1e-6f;
}


// ============================================================
// 機体ゼロ点（マウント基準）の較正
// ============================================================
void attitude_calibrate_to(float target_pitch_deg, float target_roll_deg) {
    if (!initialized_) return;
    float r, p, y;
    euler_from_state(r, p, y);      // オフセット適用前の生の値を基準にする
    // 表示は (生の値 - オフセット) なので、target になるように差を取る
    level_roll_off_  = r - target_roll_deg;      // ロールも申告値になるようにする
    level_pitch_off_ = p - target_pitch_deg;     // ピッチは申告値になるようにする
    needs_apply_ = false;                        // 較正したので警告を解除
    // 日付は一旦「不明」にする。正しい値は呼び出し側（.ino）が直後に入れる。
    // ここで古い日付を残すと、測位できていない場所で APPLY し直したときに
    // 「前の日の較正のままだ」と誤って警告することになる。不明 = 警告しない。
    calib_date_ = 0;

    // 自動トリムの累積量は捨てる。ここでロールのゼロ点を取り直したので、
    // 残したままだと表示が -roll_trim_deg_ になり「APPLY したのに 0 にならない」
    // 状態になる。溜めかけの平均もリセットして、新しい基準で測り直す。
    roll_trim_deg_ = 0.0f;
    trim_roll_sum_ = 0.0f;
    trim_time_s_   = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
}

void attitude_request_calibrate(float target_pitch_deg, float target_roll_deg) {
    calib_pitch_  = target_pitch_deg;
    calib_roll_   = target_roll_deg;
    calib_req_us_ = last_gyro_us_;   // 直近のジャイロ時刻を基準にする
    calib_pending_ = true;
}
bool attitude_calib_pending() { return calib_pending_; }
void attitude_hold_calibrate() {
    if (calib_pending_) calib_req_us_ = last_gyro_us_;
}
bool attitude_take_calib_done() {
    if (!calib_done_) return false;
    calib_done_ = false;
    return true;
}

void attitude_get_level_offset(float &roll_deg, float &pitch_deg) {
    roll_deg = level_roll_off_;
    pitch_deg = level_pitch_off_;
}

// ★ SD からの復元専用。UI からは attitude_calibrate_to() が直接書くので通らない。
//   壊れた値をそのまま入れると、ロール・ピッチ・平均ピッチ・自動トリム・
//   マウント外れ検出のすべてが同じだけずれる。roll_trim と同じ形でクランプする。
void attitude_set_level_offset(float roll_deg, float pitch_deg) {
    level_roll_off_  = clamp_setting_deg(roll_deg,  -LEVEL_OFFSET_LIMIT_DEG, LEVEL_OFFSET_LIMIT_DEG);
    level_pitch_off_ = clamp_setting_deg(pitch_deg, -LEVEL_OFFSET_LIMIT_DEG, LEVEL_OFFSET_LIMIT_DEG);
}

// ---- 対気速度モデルの係数（既定は settings.h、SD の settings.txt から上書き可）----
// 値の検査は read_setting_float()（mysd.cpp）側で行い、範囲外はクランプして
// log.txt に残す。ここでは「壊れた値で 0 除算・NaN を作らない」最低限だけ守る。
float attitude_get_airspeed_v0()  { return airspeed_v0_; }
void  attitude_set_airspeed_v0(float mps)  { if (mps > 0.0f) airspeed_v0_  = mps; }
float attitude_get_airspeed_k()   { return airspeed_k_; }
void  attitude_set_airspeed_k(float deg)   { if (deg > 0.0f) airspeed_k_   = deg; }
float attitude_get_airspeed_min() { return airspeed_min_; }
void  attitude_set_airspeed_min(float mps) { if (mps > 0.0f) airspeed_min_ = mps; }
float attitude_get_airspeed_max() { return airspeed_max_; }
void  attitude_set_airspeed_max(float mps) { if (mps > 0.0f) airspeed_max_ = mps; }

// ---- 較正した日 ----
uint32_t attitude_get_calib_date() { return calib_date_; }
// 呼び出し元は APPLY 直後の .ino と、SD 設定からの復元。
// 壊れた値を入れると「日付をまたいだ」判定が毎回成立して鳴り続けるので、
// もっともらしい範囲から外れたものは 0（不明＝警告しない）に倒す。
void attitude_set_calib_date(uint32_t yyyymmdd) {
    const uint32_t y = yyyymmdd / 10000;
    const uint32_t m = yyyymmdd / 100 % 100;
    const uint32_t d = yyyymmdd % 100;
    if (y < 2020 || y > 2099 || m < 1 || m > 12 || d < 1 || d > 31) { calib_date_ = 0; return; }
    calib_date_ = yyyymmdd;
}
// 較正した日と today_jst が違うか。
// today_jst は get_gnss_jst_yyyymmdd() の戻り値をそのまま渡す（0 = 測位前）。
// どちらかが不明なら false。屋内では経過を判定できないので、
// 「分からないときは鳴らさない」に倒す（机上テストのたびに鳴るのを防ぐ）。
bool attitude_calib_date_stale(uint32_t today_jst) {
    // volatile を 1 回だけ読む。APPLY は 0 を入れてから正しい日付を入れるので、
    // 2 回読むとその隙間に当たったときだけ「不明でないのに一致しない」と
    // 誤判定しうる（確率は低いが、読み方を固定しておけば考えなくて済む）。
    const uint32_t cd = calib_date_;
    if (cd == 0 || today_jst == 0) return false;
    return cd != today_jst;
}

float attitude_get_roll_target() { return roll_target_; }
// ★ SD からの復元専用。設定画面の巡回は attitude_cycle_roll_target() が
//   直接 roll_target_ を進めるので、ここを通らない（巡回の挙動は変わらない）。
//   範囲は画面で選べる値そのもの。外れた値が入ると APPLY のゼロ点と
//   地上ロールチェックの基準が同時にずれる。
void  attitude_set_roll_target(float deg) {
    roll_target_ = clamp_setting_deg(deg, ROLL_TARGET_MIN_DEG, ROLL_TARGET_MAX_DEG);
}
void  attitude_cycle_roll_target() {
    roll_target_ += ROLL_TARGET_STEP_DEG;
    if (roll_target_ > ROLL_TARGET_MAX_DEG + 0.01f)
        roll_target_ = ROLL_TARGET_MIN_DEG;
}
bool  attitude_needs_apply() { return needs_apply_; }
// いまこの瞬間マウントから外れた姿勢か。**ラッチしない**（needs_apply とは別物）。
bool attitude_is_off_mount() { return off_mount_now_; }
void  attitude_set_needs_apply(bool on) { needs_apply_ = on; }

float attitude_get_pitch_target() { return pitch_target_; }
// ★ SD からの復元専用（roll_target と同じ理由・同じ形）。
void  attitude_set_pitch_target(float deg) {
    pitch_target_ = clamp_setting_deg(deg, PITCH_TARGET_MIN_DEG, PITCH_TARGET_MAX_DEG);
}

void attitude_cycle_pitch_target() {
    pitch_target_ += PITCH_TARGET_STEP_DEG;
    // 端まで行ったら先頭へ戻す。浮動小数の誤差で行き過ぎないよう余裕を持たせる。
    if (pitch_target_ > PITCH_TARGET_MAX_DEG + 0.01f)
        pitch_target_ = PITCH_TARGET_MIN_DEG;
}
