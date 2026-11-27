// ============================================================
// File    : attitude.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS 速度援用 姿勢 ESKF（機上リアルタイム版）の実装。
//           数式・座標系・マウント補正の説明は attitude.h を参照。
//           PC 側の tools/imulog/eskf.py と対になっている。片方を直したら両方直すこと。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/11
// ============================================================

#include <Arduino.h>
#include <math.h>
#include "attitude.h"
#include "settings.h"

#define N_ERR 12   // 誤差状態の数: dtheta(3) + dv(3) + dbg(3) + dba(3)

static const float GRAVITY = 9.80665f;

// ============================================================
// 状態
// ============================================================
// ★ **センサー座標系 → ENU。機体軸ではない。**
//   入力（attitude_on_gyro / on_accel）はすべて IMU の生の
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
// |ω| が ESKF_LEVEL_MAX_RATE_DPS を最後に超えた時刻（重力観測のゲート用）。
static uint32_t rate_high_us_  = 0;
// 信用できる（sAcc <= ESKF_SACC_MAX）最後の測位の対地速度と時刻。地上の判定に使う
// （settings.h の ESKF_GROUND_*）。
// ★ last_gsp_ は前後加速度のゲート用で、sAcc を問わず全測位で更新している。別物。
static float    good_gsp_      = -1.0f;   // 負 = まだ来ていない
static uint32_t good_gsp_us_   = 0;
static float    good_sacc_     = ESKF_SACC_MAX;   // その測位の sAcc（「GNSS が動いている」の閾値。ESKF_ACCEL_GNSS_MOVING_SACC_K）
// ヨーが決まった後に、地上での再水平化に入った回数（60 秒ログの gnd=）。
static volatile uint32_t ground_relevels_ = 0;
static bool     ground_now_    = false;
// 加速中か（滑走の加速を傾きと取り違えないため。attitude_on_gyro の「加速中か」。settings.h の ESKF_ACCEL_*）
static bool     accel_horiz_on_ = false;     // 「加速中」の条件（加速度計の向きの変化か GNSS の速さ。settings.h の ESKF_ACCEL_*）
static uint32_t accel_horiz_since_us_ = 0;
static bool     accelerating_  = false;      // 上が ESKF_ACCEL_HOLD_MAX_US 未満続いている＝加速中とみなす
static float    acc_lp_[3]     = {0, 0, 0};  // 加速度計の値の直近の平均（時定数 ESKF_ACCEL_CHANGE_TAU_S。ジャイロで本体と一緒に回す）
static float    acc_f_[3]      = {0, 0, 0};  // 加速度計の値を ESKF_ACCEL_LP_S で均したもの
static bool     acc_lp_valid_  = false;
static bool     init_mov_on_   = false;      // 静止の初期化の門: 信用できる GNSS が動いていると言っている（gnss_moving_mps()）
static uint32_t init_mov_since_us_ = 0;
static float    acc_dp_raw2_   = 0.0f;       // 均していない加速度計の値で見た、平均の向きに直角な差の 2 乗（方位が決まる前の重力観測の門）
static bool     accel_gnss_stop_on_ = false; // 信用できる GNSS が止まっていると言っている（ESKF_ACCEL_GNSS_STOP_MPS）
static uint32_t accel_gnss_stop_since_us_ = 0;
// ---- 「加速中」になる前に寄せた分の記録（ESKF_ACCEL_UNDO_US。0.1 秒ごとの箱）----
#define LVL_UNDO_N        20
#define LVL_UNDO_BUCKET_US 100000UL
static uint32_t lvl_undo_t_[LVL_UNDO_N];
static float    lvl_undo_rot_[LVL_UNDO_N][2];   // ワールド系の水平の回転 [rad]
static float    lvl_undo_bg_[LVL_UNDO_N][3];
static float    lvl_undo_ba_[LVL_UNDO_N][3];
static bool     lvl_undo_used_[LVL_UNDO_N];
static int      lvl_undo_head_   = -1;
static uint32_t lvl_undo_now_us_ = 0;           // いまのサンプルの時刻（attitude_on_gyro の頭で入れる）
static volatile bool accel_undo_event_ = false; // ログ用（attitude_take_accel_undo_event）
static float    accel_undo_deg_  = 0.0f;
static uint8_t  accel_undo_why_  = 0;           // 0 = 加速度計 / 1 = GNSS の速さ
// 動いているときの、大きくずれた傾きの置き直し（settings.h の ESKF_BIGTILT_*）。
static float    bt_acc_lp_[3]  = {0.0f, 0.0f, 0.0f};   // 比力の低域（車の振動で |f| が揺れるのを均す）
static bool     bt_acc_lp_valid_ = false;
static float    bt_rate_lp_    = 0.0f;   // |ω| の低域 [deg/s]
static uint32_t bt_since_us_   = 0;      // 条件がそろい始めた時刻。0 = そろっていない
static volatile uint32_t bigtilt_relevels_ = 0;
static volatile bool  bigtilt_event_     = false;   // 置き直した（呼び出し側がログに残す）
static volatile float bigtilt_event_deg_ = 0.0f;    // そのときの食い違い [度]
static volatile bool  bigtilt_event_yaw_kept_ = false;   // 方位を保ったか（ヨーが決まっていなかったら false）
// 置き直したあと（IMU の途絶明けも。imu_gap_recover）、保った方位を GNSS 航跡でまだ確かめていない（ESKF_BIGTILT_YAW_CHECK_DEG）。
// true の間は GNSS 速度を観測に入れない（attitude_on_gnss_velocity）。2026-11-13〜 止まっているときだけは入れる（BUG-64）。
static bool     yaw_suspect_   = false;
static uint8_t  yaw_check_arm_ = 0;
static volatile uint8_t yaw_check_event_ = 0;      // 0 = なし / 1 = 保った / 2 = 航跡で入れ直した（呼び出し側がログに残す）
static volatile float   yaw_check_diff_  = 0.0f;   // そのときの 航跡 − 方位 [度]
// IMU が途絶えて戻ったとき（imu_gap_recover。BUG-62。2026-11-13〜）。
// true の間は、次の信用できる測位で速度を GNSS の値に入れ直す（attitude_on_gnss_velocity）。
static bool     v_stale_ = false;
static volatile bool  imu_gap_event_          = false;   // 途絶明けを扱った（呼び出し側がログに残す）
static volatile float imu_gap_event_s_        = 0.0f;    // 途絶えていた秒数
static volatile bool  imu_gap_event_yaw_kept_ = false;   // 方位を保ったか（ヨーが決まっていなかったら false）
// ジャイロの飽和（gyro_saturated()。settings.h の ESKF_GYRO_SAT_DPS。2026-11-27〜）。
static bool     gyro_sat_on_       = false;   // ひとまとまりの飽和の途中（ESKF_GYRO_SAT_END_US 下回るまで）
static uint32_t gyro_sat_since_us_ = 0;       // 最初に飽和したサンプルの時刻
static uint32_t gyro_sat_last_us_  = 0;       // 最後に飽和したサンプルの時刻
static uint32_t gyro_sat_n_        = 0;       // 飽和したサンプル数
static bool     gyro_sat_yaw_drop_ = false;   // 方位を捨てたか（始まったときにヨーが決まっていたか）
static volatile bool     gyro_sat_event_      = false;   // ひとまとまりが終わった（呼び出し側がログに残す）
static volatile float    gyro_sat_event_s_    = 0.0f;    // 最初から最後の飽和までの秒数
static volatile uint32_t gyro_sat_event_n_    = 0;
static volatile bool     gyro_sat_event_drop_ = false;

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
// 較正した時刻（JST の HHMM）。-1 = 不明（2026-11-27〜。設定画面の stored offset の下に出すだけで、判定には使わない）。
// 入れ方・保存・volatile の理由は calib_date_ と同じ。2026-11-27 より前の設定ファイルには無いので、日付だけ分かって時刻は不明になる。
static volatile int32_t calib_time_ = -1;

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
// ★ **過去 PITCH_AVG_SEC 秒の移動平均**（2026-11-07〜）。それまでは時定数 10 秒の一次遅れで、
//   古い値の重みが 0 にならず、発進から 10 秒経っても地上の姿勢が 37%、30 秒でも 5% 残っていた
//   （台 −3.5° からだと推定 TAS が 10 秒で 0.5m/s 速く出る）。移動平均なら 10 秒で完全に抜ける。
//   2025 年の対気速度センサーのログでも、一次遅れより当たりが良く遅れも短い（docs/pons_tas.md §2.4）。
// 0.1 秒ごとの小計（ピッチ×時間と時間）を PITCH_AVG_NBIN 個の輪に貯め、満ちたら平均する。
// IMU の 1 サンプルずつ貯めると 2500 個（10KB）要るので、小計にしてある。
#define PITCH_AVG_BIN_S 0.1f
static constexpr int PITCH_AVG_NBIN = (int)(PITCH_AVG_SEC / PITCH_AVG_BIN_S + 0.5f);
static float pavg_bin_sum_[PITCH_AVG_NBIN];   // 小計: ∫ピッチ dt [度·s]
static float pavg_bin_dt_[PITCH_AVG_NBIN];    // 小計: ∫dt [s]
static float vs_bin_sum_[PITCH_AVG_NBIN];     // 同じ区間の小計: ∫V/S dt [m]
static bool  vs_bin_valid_[PITCH_AVG_NBIN];   // その 0.1 秒区間の全サンプルで V/S が有効
static int   pavg_head_  = 0;                 // 次に書く小計の位置
static int   pavg_count_ = 0;                 // 埋まっている小計の数（PITCH_AVG_NBIN で頭打ち）
static float pavg_cur_sum_ = 0.0f, pavg_cur_dt_ = 0.0f;   // いま貯めている小計
static float pitch_avg_ = 0.0f;           // 満ちるまでは、それまでに貯まった分の平均
static float vs_avg_ = 0.0f;              // 平均ピッチと全く同じ時間窓の V/S 平均
static float pavg_cur_vs_sum_ = 0.0f;
static bool  pavg_cur_vs_valid_ = true;
static volatile float vs_input_mps_ = 0.0f;
static volatile bool  vs_input_valid_ = false;
static bool  vs_window_valid_ = false;
static bool  gamma_state_known_ = false;
static bool  gamma_state_active_ = false;
static volatile bool gamma_event_ = false;
static volatile bool gamma_event_active_ = false;

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
static volatile float airspeed_weight_kg_ = AIRSPEED_DESIGN_WEIGHT_KG;
static volatile float airspeed_min_ = AIRSPEED_MIN_MPS;
static volatile float airspeed_max_ = AIRSPEED_MAX_MPS;

// ピッチ [度] から対気速度 [m/s] を引く。式の導出は settings.h のコメント。
//     V(θ) = V0 * sqrt( (実重量 / 設計重量) * K / (K + θ) )
// ピッチが上がるほど遅く、下がるほど速い。上下限でクランプし、
// この機体で起こり得ない対気速度を出さないようにする。
// clamped を渡すと、上下限に張り付いたか（推定ではなく端の値か）を返す。
// 推定 TAS の表示で箱を灰色にするのに使う（docs/pons_tas.md §4.3）。
static float airspeed_from_pitch(float pitch_deg, bool *clamped = nullptr) {
    if (clamped) *clamped = false;
    // NaN・無限大は基準角 0° に戻し、通常と同じ上下限を適用する。
    if (!(pitch_deg > -1000.0f && pitch_deg < 1000.0f)) pitch_deg = 0.0f;
    const float weight_ratio = airspeed_weight_kg_ / AIRSPEED_DESIGN_WEIGHT_KG;
    const float k = airspeed_k_;
    const float d = k + pitch_deg;
    // ★ K + θ がゼロに近づくと発散し、負になると sqrtf() は NaN を返す。
    //   **NaN との比較はすべて false なので、下のクランプを素通りする。**
    //   そのまま風の推定（wind_e_/wind_n_）へ入ると LPF ごと NaN に汚染され、
    //   電源を切るまで風が出なくなる。ここで打ち切る。
    //   物理的には「揚力がゼロになるピッチまで機首を下げた」状況で、
    //   そこまで下げれば対気速度は上限側に張り付く。
    if (d <= 0.01f) { if (clamped) *clamped = true; return airspeed_max_; }
    float v = airspeed_v0_ * sqrtf(weight_ratio * k / d);
    // 上下限が逆に設定されていても NaN にはならない（max 側が残るだけ）。
    if (v < airspeed_min_) { v = airspeed_min_; if (clamped) *clamped = true; }
    if (v > airspeed_max_) { v = airspeed_max_; if (clamped) *clamped = true; }
    return v;
}

// ---- 平均ピッチ（過去 PITCH_AVG_SEC 秒の移動平均）----
static void pitch_avg_reset() {
    for (int i = 0; i < PITCH_AVG_NBIN; i++) {
        pavg_bin_sum_[i] = 0.0f; pavg_bin_dt_[i] = 0.0f;
        vs_bin_sum_[i] = 0.0f; vs_bin_valid_[i] = false;
    }
    pavg_head_ = 0;
    pavg_count_ = 0;
    pavg_cur_sum_ = pavg_cur_dt_ = 0.0f;
    pavg_cur_vs_sum_ = 0.0f;
    pavg_cur_vs_valid_ = true;
    // ★ V/S の入力（vs_input_*）はここでは消さない。窓の状態ではなく「いまの V/S」で、毎サンプル imu.cpp が入れ直す。
    //   消すと、窓を空にしたサンプルの小計が欠測扱いになり、窓が満ちるのが 1 小計（0.1 秒）遅れていた。消すのは attitude_setup()。
    pitch_avg_ = 0.0f;
    vs_avg_ = 0.0f;
    vs_window_valid_ = false;
    gamma_state_known_ = false;
    gamma_state_active_ = false;
    gamma_event_ = false;
}

// 平均が PITCH_AVG_SEC 秒ぶん満ちているか（満ちる前は発進直後と同じく地上の値しか無い）。
static bool pitch_avg_full() { return pavg_count_ >= PITCH_AVG_NBIN; }

static bool tas_gamma_ready() {
    return vs_window_valid_ && pavg_cur_vs_valid_ && vs_input_valid_;
}

// 1 サンプル足す。小計が 0.1 秒ぶん貯まったら輪へ移し、輪全体で平均し直す。
// ★ 合計を足し引きで持ち回さず、毎回 PITCH_AVG_NBIN 個を足し直す（0.1 秒に 1 回なので軽い）。
//   持ち回すと float の丸めが飛行中ずっと積もる。
// ★ 時間で重みを付ける（小計は dt の和）。IMU の読み出しが一瞬空いて dt が伸びた回も、
//   その長さぶんだけ効く。
static void pitch_avg_push(float p, float dt) {
    if (!(p > -1000.0f && p < 1000.0f) || !(dt > 0.0f)) return;   // NaN を輪に入れない
    pavg_cur_sum_ += p * dt;
    const float vs = vs_input_mps_;
    if (vs_input_valid_ && vs > -1000.0f && vs < 1000.0f) pavg_cur_vs_sum_ += vs * dt;
    else pavg_cur_vs_valid_ = false;
    pavg_cur_dt_  += dt;
    if (pavg_cur_dt_ < PITCH_AVG_BIN_S) return;
    pavg_bin_sum_[pavg_head_] = pavg_cur_sum_;
    pavg_bin_dt_[pavg_head_]  = pavg_cur_dt_;
    vs_bin_sum_[pavg_head_] = pavg_cur_vs_sum_;
    vs_bin_valid_[pavg_head_] = pavg_cur_vs_valid_;
    pavg_head_ = (pavg_head_ + 1) % PITCH_AVG_NBIN;
    if (pavg_count_ < PITCH_AVG_NBIN) pavg_count_++;
    pavg_cur_sum_ = pavg_cur_dt_ = pavg_cur_vs_sum_ = 0.0f;
    pavg_cur_vs_valid_ = true;
    // 満ちる前は 0 番から順に埋まっているので、先頭 pavg_count_ 個がそのまま有効
    float s = 0.0f, vs_s = 0.0f, t = 0.0f;
    bool vs_ok = pavg_count_ >= PITCH_AVG_NBIN;
    for (int i = 0; i < pavg_count_; i++) {
        s += pavg_bin_sum_[i];
        vs_s += vs_bin_sum_[i];
        t += pavg_bin_dt_[i];
        if (!vs_bin_valid_[i]) vs_ok = false;
    }
    if (t > 0.0f) pitch_avg_ = s / t;
    vs_window_valid_ = vs_ok && t > 0.0f && vs_input_valid_;
    if (t > 0.0f) vs_avg_ = vs_s / t;
    const bool gamma_active = tas_gamma_ready();
    if (!gamma_state_known_ && pitch_avg_full()) {
        gamma_state_known_ = true;
        gamma_state_active_ = gamma_active;
        gamma_event_active_ = gamma_active;
        gamma_event_ = true;
    } else if (gamma_state_known_ && gamma_active != gamma_state_active_) {
        gamma_state_active_ = gamma_active;
        gamma_event_active_ = gamma_active;
        gamma_event_ = true;
    }
}

// 平均ピッチから経路角を引いた推定 TAS。最初に補正前の速度を使い、補正後の速度で γ を 1 回出し直す。
static float alpha_from_pitch_avg() {
    const float to_deg = 180.0f / (float)M_PI;
    const float v0 = airspeed_from_pitch(pitch_avg_);
    const float v1 = airspeed_from_pitch(pitch_avg_ - atan2f(vs_avg_, v0) * to_deg);
    return pitch_avg_ - atan2f(vs_avg_, v1) * to_deg;
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
// サンプル間隔
// ============================================================
// ★ **dt は「直前のサンプルとの実際の時間差」**（attitude_on_gyro。tools/imulog/eskf.py と同じ）。
//   SCH16T は FIFO を持たず、こちらが 1 回読むと 1 サンプル（imu_sch16t.cpp）。Core0 が 40ms 止まれば
//   その次のサンプルは 40ms 後に 1 つだけ来るので、40ms ぶん積分するのが正しい。
//   1.04 までは BNO085 のまとめ届き（受信時刻の差が「0, 0, 64ms」になる）に合わせた 64 サンプルの
//   平均間隔を使っていて、止まった直後の回転を約 4.6ms ぶんしか積まず、残りを続く 0.25 秒へ薄めていた
//   （旋回に入る瞬間などで姿勢が遅れる）。まとめ届きのあるチップへ戻すときは考え直すこと。
// 0 以下（リセット後の最初の 1 サンプル）と ESKF_IMU_TIMEOUT_US 超（途絶明け）は伝播しない
// （eskf.py の predict の `dt <= 0 or dt > 0.5` と同じ）。

// ============================================================
// 静止判定
// ============================================================
static bool     static_now_ = false;
static uint32_t static_since_us_ = 0;
// 静止中の加速度累積（初期姿勢＝重力方向の平均用）。
// ★ v7 の実測で、チップ内融合のロール・ピッチと 0.05 度以内で一致した
//   （20260930 / 20260818 の計 5 セッション。0.983 の検証。git の履歴の docs/imu_sch16t_plan.md §2.2）。
static float    acc_sum_[3] = {0, 0, 0};
static uint32_t acc_count_  = 0;

// ---- ヨーの状態 ----
// ★ **ロール・ピッチ（leveled_ = initialized_）とヨー（yaw_set_）を分けている。**
//   重力からはロール・ピッチしか決まらないので、水平化した時点でヨーは未知。
//   バンク警報とバリオはロール・ピッチだけで動くので、ヨーを待たずに立てる。
static bool     yaw_set_    = false;
static uint8_t  yaw_src_    = ATT_YAW_SRC_NONE;
static uint8_t  yaw_arm_    = 0;          // ESKF_YAW_INIT_MIN_SPEED_MPS 以上が連続した測位数（航跡へ戻れる）
static uint8_t  yaw_arm_lo_ = 0;          // ESKF_YAW_INIT_CORR_MIN_SPEED_MPS 以上が連続した測位数（2D 相関だけで入れられる。2026-11-20〜）
// ---- 発進加速の常時前向き 2D 相関（ヨー未設定の間だけ使う。settings.h の ESKF_YAW_CORR_*）----
static float    yaw_imu_hp_[2]     = {0.0f, 0.0f};   // IMU 水平比力のリーキー積分 (East, North) [m/s]
static float    yaw_gnss_lp_[2]    = {0.0f, 0.0f};   // GNSS 水平速度の低域 (East, North) [m/s]
static bool     yaw_gnss_lp_valid_ = false;
static uint32_t yaw_gnss_last_us_  = 0;
static float    yaw_static_s_      = 0.0f;           // 静止かつ非加速が続いた時間 [s]
static float    yaw_corr_c_        = 0.0f;           // Σ w (v_imu · dv_gnss)
static float    yaw_corr_s_        = 0.0f;           // Σ w (v_imu × dv_gnss)_z
static float    yaw_corr_n_        = 0.0f;           // Σ w |v_imu| |dv_gnss|（まとまり hypot(C,S)/N の分母）
// 方位を入れたときの記録（log.txt の `ESKF YAW INIT` 行。attitude_take_yaw_init_event）
static volatile bool yaw_init_event_ = false;
static uint8_t  yaw_init_how_      = 0;     // 0 = 2D 相関 / 1 = 航跡（情報不足）/ 2 = 航跡（まとまり不足）/ 3 = 航跡（航跡と食い違い）
static float    yaw_init_mag_      = 0.0f;
static float    yaw_init_coh_      = 0.0f;
static float    yaw_init_trkdiff_  = 0.0f;  // 相関で出した方位 − 航跡 [度]（相関を計算できたときだけ意味がある）
static float    yaw_init_sigma_    = 0.0f;
static float    yaw_init_gs_       = 0.0f;  // 方位を入れた瞬間の GNSS の対地速度（ログを書く時点の速さではなく）
static bool     yaw_init_bias_ok_  = false; // ゼロ点を最近覚えていたので P_bg を小さくした

static void yaw_corr_reset() {
    yaw_imu_hp_[0] = yaw_imu_hp_[1] = 0.0f;
    yaw_gnss_lp_[0] = yaw_gnss_lp_[1] = 0.0f;
    yaw_gnss_lp_valid_ = false;
    yaw_gnss_last_us_  = 0;
    yaw_static_s_      = 0.0f;
    yaw_corr_c_        = 0.0f;
    yaw_corr_s_        = 0.0f;
    yaw_corr_n_        = 0.0f;
}

// ---- 静止中のジャイロバイアス学習 ----
// 和と二乗和は窓の最初のサンプル（bias_g0_）からの差で溜める（float の桁落ちを避けるため。
// ゼロ点は最大 0.3deg/s あり得るのに対し、見たいばらつきは 0.03deg/s 未満）。
static float    bias_g0_[3]  = {0, 0, 0};
static float    bias_sum_[3] = {0, 0, 0};   // Σ(g − g0)
static float    bias_sq_[3]  = {0, 0, 0};   // Σ(g − g0)²
static uint32_t bias_n_      = 0;
// 方位が決まったあとの地上での維持でゼロ点を覚えたか（動き出すときに P_bg を広げる。learn_gyro_bias のコメント）
static bool     bias_learned_on_ground_ = false;
// 静止中にゼロ点を覚えた最後の時刻（方位を入れるとき P_bg を小さくしてよいかの判断。ESKF_YAW_INIT_BIAS_FRESH_US）
static bool     bias_learn_ok_valid_ = false;
static uint32_t bias_learn_ok_us_    = 0;
// ---- 覚えた値を確かめる（2026-11-18〜。settings.h の ESKF_BIAS_CONFIRM_DIFF_RADS）----
// ばらつきの門は「一定の速さの回転」を見分けられないので、覚えた値を前回までの値（SD の gyro_bias.txt か、この起動で確かめた値）か、
// 続けて通った直前の窓と比べ、ESKF_BIAS_CONFIRM_DIFF_RADS 以内のときだけ「覚えた」（bias_learn_ok_*）にする。
static bool     bias_ref_valid_  = false;   // 前回までに確かめたゼロ点（SD の記録か、この起動で確かめた値）
static float    bias_ref_[3]     = {0, 0, 0};
static bool     bias_prev_valid_ = false;   // 切れ目なく続いた直前の窓が門を通ったか（その平均）
static float    bias_prev_[3]    = {0, 0, 0};
static bool     bias_confirmed_once_ = false;   // この起動で一度でも確かめたか（ログを最初の 1 回だけにする）
// SD へ書く頼み（Core0 の .ino が attitude_take_bias_to_save で受けて SD のタスクにする）
static bool     bias_save_req_   = false;
static float    bias_save_[3]    = {0, 0, 0};
static bool     bias_saved_valid_ = false;  // この起動で書いた（書くように頼んだ）値
static float    bias_saved_[3]   = {0, 0, 0};
static uint32_t bias_saved_us_   = 0;
// ログ用（attitude_take_bias_event）
static bool     bias_ev_         = false;
static uint8_t  bias_ev_how_     = 0;       // 0 = 前回の値と一致 / 1 = 続けて 2 つの窓が一致 / 2 = 前回の値と食い違い（覚えた扱いにしない）
static float    bias_ev_b_[3]    = {0, 0, 0};
static float    bias_ev_diff_    = 0.0f;    // 比べた相手との差の最大（各軸）[rad/s]

// 5 秒の窓の和だけを空にする（窓を締めたとき。続けて通った窓どうしの比べは残す）
static void bias_window_clear() {
    bias_n_ = 0;
    for (int i = 0; i < 3; i++) { bias_sum_[i] = 0.0f; bias_sq_[i] = 0.0f; }
}
// 窓の溜めかけを捨て、「直前の窓」も切る（動いた・加速した・初期化した）
static void bias_learn_reset() {
    bias_window_clear();
    bias_prev_valid_ = false;
}
static float bias_maxdiff(const float a[3], const float b[3]) {
    float m = 0.0f;
    for (int i = 0; i < 3; i++) { const float d = fabsf(a[i] - b[i]); if (d > m) m = d; }
    return m;
}

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
//   あくまで水平化までの繋ぎ（チップ内融合の姿勢と同程度の品質）。
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
// ★ **回転軸は必ず水平になる。**
//   姿勢が正しければ「a をワールドへ回したもの」は +Z になる。ずれていれば
//   その gw を +Z へ運ぶ回転が補正量で、軸 = gw × ẑ = (gw_y, -gw_x, 0)。
//   Z 成分が構造的に 0 なので、ずれが小さいときはヨーにほとんど触らない。
// ★★ **ずれが大きいと方位（出力のヨー＝機首の水平の向き）は振れる。** 水平な軸でも、機首が大きく傾いた姿勢を
//   回せば機首の水平の向きは変わる（2026-11-11 のトンネルの再現で、76 度の置き直しで 42.7 → 110.5 度）。
//   方位を保ちたいときは回したあとで rotate_world_z() で戻すこと（relevel_big_tilt がそうしている）。
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


// ---- 「加速中」になる前に寄せた分の記録と取り消し（settings.h の ESKF_ACCEL_UNDO_US）----
static void clamp_bias(float b[3], float lim);
static void lvl_undo_record(float rx, float ry, const float dbg[3], const float dba[3]) {
    const uint32_t now = lvl_undo_now_us_;
    if (lvl_undo_head_ < 0 || (uint32_t)(now - lvl_undo_t_[lvl_undo_head_]) >= LVL_UNDO_BUCKET_US) {
        lvl_undo_head_ = (lvl_undo_head_ + 1) % LVL_UNDO_N;
        lvl_undo_t_[lvl_undo_head_] = now;
        lvl_undo_used_[lvl_undo_head_] = true;
        lvl_undo_rot_[lvl_undo_head_][0] = lvl_undo_rot_[lvl_undo_head_][1] = 0.0f;
        for (int k = 0; k < 3; k++) { lvl_undo_bg_[lvl_undo_head_][k] = 0.0f; lvl_undo_ba_[lvl_undo_head_][k] = 0.0f; }
    }
    lvl_undo_rot_[lvl_undo_head_][0] += rx;
    lvl_undo_rot_[lvl_undo_head_][1] += ry;
    for (int k = 0; k < 3; k++) {
        if (dbg) lvl_undo_bg_[lvl_undo_head_][k] += dbg[k];
        if (dba) lvl_undo_ba_[lvl_undo_head_][k] += dba[k];
    }
}
// 待機中の維持・地上での維持の寄せ（level_toward_gravity）を記録つきで行う
static void level_toward_gravity_rec(const float a[3], float gain) {
    float q0[4] = { q_[0], q_[1], q_[2], q_[3] };
    if (!level_toward_gravity(a, gain)) return;
    // dq = q_ ⊗ conj(q0)（ワールド系の回転）。寄せは毎サンプル小さいので回転ベクトル ≒ 2·(x, y)
    const float qc[4] = { q0[0], -q0[1], -q0[2], -q0[3] };
    float dq[4];
    quat_mul(q_, qc, dq);
    const float sgn = (dq[0] < 0.0f) ? -2.0f : 2.0f;
    lvl_undo_record(sgn * dq[1], sgn * dq[2], nullptr, nullptr);
}
static void lvl_undo_clear() {
    lvl_undo_head_ = -1;
    for (int i = 0; i < LVL_UNDO_N; i++) lvl_undo_used_[i] = false;
}
// 直前 ESKF_ACCEL_UNDO_US の間に寄せた分を逆に回して戻す。戻した角度 [度] を返す
static float lvl_undo_apply(uint32_t t_us) {
    float r[3] = { 0.0f, 0.0f, 0.0f }, dbg[3] = { 0, 0, 0 }, dba[3] = { 0, 0, 0 };
    for (int i = 0; i < LVL_UNDO_N; i++) {
        if (!lvl_undo_used_[i]) continue;
        if ((int32_t)(t_us - lvl_undo_t_[i]) >= (int32_t)ESKF_ACCEL_UNDO_US) continue;   // 古い箱は戻さない
        r[0] += lvl_undo_rot_[i][0]; r[1] += lvl_undo_rot_[i][1];
        for (int k = 0; k < 3; k++) { dbg[k] += lvl_undo_bg_[i][k]; dba[k] += lvl_undo_ba_[i][k]; }
    }
    lvl_undo_clear();
    const float ang = sqrtf(r[0]*r[0] + r[1]*r[1]);
    if (ang <= 0.0f) return 0.0f;
    const float inv[3] = { -r[0], -r[1], 0.0f };
    float dq[4], qn[4];
    quat_from_rotvec(inv, dq);
    quat_mul(dq, q_, qn);                 // ワールド系なので左から
    for (int i = 0; i < 4; i++) q_[i] = qn[i];
    quat_normalize(q_);
    for (int k = 0; k < 3; k++) { bg_[k] -= dbg[k]; ba_[k] -= dba[k]; }
    clamp_bias(bg_, ESKF_MAX_GYRO_BIAS);
    clamp_bias(ba_, ESKF_MAX_ACCEL_BIAS);
    return ang * 57.29578f;
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

// ロール・ピッチを外部情報（重力）で置き換えたときの共分散の入れ直し。
// init_yaw_from_track() のヨーと同じ扱いで、**他の状態との相関は意味を失うので落とす。**
// ヨー・速度・バイアスの分散はそのまま残す（reset_covariance と違ってヨーを捨てない）。
static void reset_tilt_covariance(float att_sigma_deg) {
    const float s = att_sigma_deg * (float)M_PI / 180.0f;
    for (int k = 0; k < 2; k++)
        for (int i = 0; i < N_ERR; i++) { P_[k][i] = 0.0f; P_[i][k] = 0.0f; }
    P_[0][0] = P_[1][1] = s * s;
}

// ============================================================
// 速度の履歴（GNSS 速度の遅れの補償用。settings.h の ESKF_GNSS_VEL_DELAY_MS）
// ============================================================
// NAV-PVT は測位の瞬間より遅れて届く（GNSS の中の計算・UART・Core0 が描画などで受け取りに行けなかった時間）。
// 届いた時点の v_ と比べると、旋回中は「旋回率 × 遅れ」だけ方位が遅れる。そこで伝播のたびに v_ を覚えておき、
// 観測が来たら「測位の瞬間の v_」と比べる（直しはいまの状態に入れる。遅れが短いので P と H はいまのものを使う）。
// ★ 観測で v_ を直したら、同じ量を履歴にも足す（vhist_shift）。履歴を「いまの推定で見た過去の速度」に保つため。
//   足さないと、次の観測が前の直しのぶんをもう一度残差として見る。
// ★ v_ を入れ直したら（初期化・ヨーの注入・途絶明け）履歴を捨てる（vhist_clear）。前の速度とはつながっていない。
// tools/imulog/eskf.py の vh / hist_shift / v_at と同じ。
struct VHist { uint32_t t_us; float v[3]; };
static VHist    vhist_[ESKF_VHIST_N];
static uint16_t vhist_head_ = 0;     // 次に書く位置
static uint16_t vhist_n_    = 0;     // 入っている数

static void vhist_clear() { vhist_head_ = 0; vhist_n_ = 0; }

static void vhist_push(uint32_t t_us) {
    VHist &e = vhist_[vhist_head_];
    e.t_us = t_us;
    e.v[0] = v_[0]; e.v[1] = v_[1]; e.v[2] = v_[2];
    vhist_head_ = (uint16_t)((vhist_head_ + 1) % ESKF_VHIST_N);
    if (vhist_n_ < ESKF_VHIST_N) vhist_n_++;
}

static void vhist_shift(const float dv[3]) {
    for (uint16_t i = 0; i < vhist_n_; i++) {
        vhist_[i].v[0] += dv[0]; vhist_[i].v[1] += dv[1]; vhist_[i].v[2] += dv[2];
    }
}

// 時刻 t_us の推定速度。履歴より新しければいまの v_、古ければいちばん古いもの、間なら直線でつなぐ。
static void vhist_at(uint32_t t_us, float out[3]) {
    out[0] = v_[0]; out[1] = v_[1]; out[2] = v_[2];
    if (vhist_n_ == 0) return;
    // 新しいほうから古いほうへたどる
    int idx = (int)vhist_head_ - 1;
    if (idx < 0) idx += ESKF_VHIST_N;
    const VHist *newer = nullptr;
    for (uint16_t k = 0; k < vhist_n_; k++) {
        const VHist &e = vhist_[idx];
        if ((int32_t)(t_us - e.t_us) >= 0) {
            // e は t_us と同じか古い。1 つ新しいもの（newer）との間をつなぐ
            if (newer) {
                const float span = (float)(int32_t)(newer->t_us - e.t_us);
                const float w = (span > 0.0f) ? (float)(int32_t)(t_us - e.t_us) / span : 0.0f;
                for (int c = 0; c < 3; c++) out[c] = e.v[c] + (newer->v[c] - e.v[c]) * w;
            }   // newer が無い = いちばん新しい履歴より後。いまの v_ のまま
            return;
        }
        newer = &e;
        if (--idx < 0) idx += ESKF_VHIST_N;
    }
    // 履歴のどれよりも古い → いちばん古いもの
    out[0] = newer->v[0]; out[1] = newer->v[1]; out[2] = newer->v[2];
}

void attitude_setup() {
    q_[0] = 1; q_[1] = q_[2] = q_[3] = 0;
    v_[0] = v_[1] = v_[2] = 0;
    vhist_clear();
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(10.0f, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    initialized_ = false;
    accel_valid_ = false;
    acc_count_ = 0;
    for (int i = 0; i < 3; i++) acc_sum_[i] = 0.0f;
    yaw_set_ = false;
    yaw_src_ = ATT_YAW_SRC_NONE;
    yaw_arm_ = 0;
    yaw_arm_lo_ = 0;
    yaw_corr_reset();
    bias_learn_reset();
    bias_learned_on_ground_ = false;
    bias_learn_ok_valid_ = false;
    // 前回の値は setup のあとで SD から渡される（attitude_set_bias_ref）。実機では setup は 1 回だけ
    bias_ref_valid_ = false;
    bias_confirmed_once_ = false;
    bias_save_req_ = false;
    bias_saved_valid_ = false;
    bias_ev_ = false;
    yaw_init_event_ = false;
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
    rate_high_us_ = 0;
    good_gsp_ = -1.0f;
    good_gsp_us_ = 0;
    good_sacc_ = ESKF_SACC_MAX;
    ground_relevels_ = 0;
    ground_now_ = false;
    accel_horiz_on_ = false;
    accelerating_  = false;
    acc_lp_valid_  = false;
    accel_gnss_stop_on_ = false;
    init_mov_on_ = false;
    lvl_undo_head_ = -1;
    for (int i = 0; i < LVL_UNDO_N; i++) lvl_undo_used_[i] = false;
    accel_undo_event_ = false;
    bt_acc_lp_valid_ = false;
    bt_rate_lp_ = 0.0f;
    bt_since_us_ = 0;
    bigtilt_relevels_ = 0;
    bigtilt_event_ = false;
    yaw_suspect_ = false;
    yaw_check_arm_ = 0;
    yaw_check_event_ = 0;
    v_stale_ = false;
    imu_gap_event_ = false;
    gyro_sat_on_ = false;
    gyro_sat_event_ = false;
    gnss_updates_ = 0;
    gyro_seen_ = false;
    last_gyro_us_ = 0;
    last_gnss_vel_us_ = 0;
    gnss_vel_seen_ = false;
    vs_input_mps_ = 0.0f;
    vs_input_valid_ = false;
    pitch_avg_reset();
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

// 測位（sAcc を問わない）が新しいか。重力観測の前後加速度の門（longacc_lp_）の材料が生きているか。
// ★ gnss_vel_fresh() は「速度の観測を ESKF に入れた」時刻なので、方位が決まる前は常に false になる。
//   それを門に使っていたため、方位が決まる前の滑走加速中に前後加速度の門が外れ、加速度を傾きと取り違えていた
//   （2026-11-18 に見つけた。0→6 m/s を 4 秒の模擬でピッチ 8.7 度）。tools/imulog/eskf.py はもともとこちら（最後の測位から 3 秒）。
static bool gsp_fresh() {
    if (last_gsp_ < 0.0f) return false;
    return (uint32_t)(time_us_32() - last_gsp_us_) < ESKF_GNSS_VEL_TIMEOUT_US;
}

// ---- 地上の判定（settings.h の ESKF_GROUND_*）----
// 信用できる測位が新しいか。鮮度の基準は gnss_vel_fresh() と同じ。
static bool good_gnss_fresh() {
    if (good_gsp_ < 0.0f) return false;
    return (uint32_t)(time_us_32() - good_gsp_us_) < ESKF_GNSS_VEL_TIMEOUT_US;
}
// 「GNSS が動いている」とみなす対地速度 [m/s]（「加速中」の判定と静止の初期化の門。settings.h の ESKF_ACCEL_GNSS_MOVING_*。2026-11-20〜）。
// 止まっているときの速度の雑音で寄せとゼロ点の学習が止まらないよう、sAcc が悪いときは上げる。
static float gnss_moving_mps() {
    const float k = ESKF_ACCEL_GNSS_MOVING_SACC_K * good_sacc_;
    return (k > ESKF_ACCEL_GNSS_MOVING_MPS) ? k : ESKF_ACCEL_GNSS_MOVING_MPS;
}
// GNSS が対地速度で「地上で止まっている」と確かめているか（ゼロ点を学習してよいか）。
static bool gnss_says_ground() {
    return good_gnss_fresh() && good_gsp_ < ESKF_GROUND_MAX_GS_MPS;
}
// 静止が続いていて、GNSS が「動いている」と言っていないか（重力へ寄せてよいか）。
// ★ 信用できる測位が無いときも地上とみなす。屋内や GNSS 断ではこちらになる。
static bool on_ground_still(uint32_t t_us) {
    if (!static_now_) return false;
    if ((uint32_t)(t_us - static_since_us_) < ESKF_GROUND_STATIC_US) return false;
    return !(good_gnss_fresh() && good_gsp_ >= ESKF_GROUND_MAX_GS_MPS);
}

// 重力方向から姿勢を初期化する。**ロール・ピッチだけが決まり、ヨーは未知。**
// ヨーは後で GNSS 航跡から入れる（init_yaw_from_track）。
//   a             : 重力方向とみなす比力（静止平均、または走行中なら直近値）
//   att_sigma_deg : ロール・ピッチの初期σ
static bool init_level(const float a[3], float att_sigma_deg) {
    q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
    if (!level_toward_gravity(a, 1.0f)) return false;
    v_[0] = v_[1] = v_[2] = 0.0f;
    vhist_clear();
    for (int i = 0; i < 3; i++) { bg_[i] = 0; ba_[i] = 0; }
    reset_covariance(att_sigma_deg, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    initialized_ = true;      // = 水平化済み。ロール・ピッチとバリオはここから有効
    yaw_set_     = false;
    yaw_src_     = ATT_YAW_SRC_NONE;
    yaw_arm_     = 0;
    yaw_arm_lo_  = 0;
    yaw_corr_reset();
    bias_learn_reset();
    lvl_undo_clear();                 // 水平化し直したので、前の寄せの記録は意味が無い
    bias_learned_on_ground_ = false;
    bias_learn_ok_valid_ = false;     // bg_ を 0 に戻したので、覚えたゼロ点はもう無い
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

// 動いているときに、大きくずれた傾きを比力で置き直す（settings.h の ESKF_BIGTILT_*）。
//   a       : 比力の低域（センサー座標。重力込み）
//   err_deg : 置き直す前の食い違い [度]（ログ用）
// ★★ **ヨーは捨てない。方位（機首の水平の向き）を保ったまま、ロール・ピッチだけを合わせる。**
//   level_toward_gravity() は最短の回転なので、傾きが大きくずれていると方位まで振れる（11/11 のトンネルで 68 度）。
//   回したあとで方位だけ元へ戻す（rotate_world_z はロール・ピッチを動かさない）。
//   方位の不確かさは ESKF_BIGTILT_YAW_SIGMA_DEG まで広げ、GNSS が戻ったら航跡と比べる（yaw_suspect_）。理由は settings.h。
// ★ 平均ピッチの窓と、自動ロールトリムの溜めかけは捨てる（ずれていた間の値を混ぜない）。
static void relevel_big_tilt(const float a[3], float err_deg) {
    float r0, p0, h0;
    euler_from_state(r0, p0, h0);               // 置き直す前の方位
    level_toward_gravity(a, 1.0f);
    reset_tilt_covariance(ESKF_INIT_ATT_SIGMA_MOVING_DEG);
    bigtilt_event_yaw_kept_ = yaw_set_;
    if (yaw_set_) {
        float r1, p1, h1;
        euler_from_state(r1, p1, h1);
        float d = h0 - h1;                      // 戻す量（init_yaw_from_track と同じ向きの約束）
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        rotate_world_z(q_, -d * (float)M_PI / 180.0f);
        // 方位の不確かさを広げ、ほかの状態との相関は落とす（傾きを大きく直したので意味を失った）
        const float old = P_[2][2];
        const float ys = ESKF_BIGTILT_YAW_SIGMA_DEG * (float)M_PI / 180.0f;
        for (int i = 0; i < N_ERR; i++) { P_[2][i] = 0.0f; P_[i][2] = 0.0f; }
        P_[2][2] = (old > ys * ys) ? old : ys * ys;
        yaw_suspect_   = true;                  // GNSS が戻ったら航跡と比べる
        yaw_check_arm_ = 0;
    }
    pitch_avg_reset();
    trim_roll_sum_  = 0.0f;
    trim_time_s_    = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
    bigtilt_relevels_++;
    bigtilt_event_deg_ = err_deg;
    bigtilt_event_     = true;
}

// IMU が途絶えて戻ったとき（ジャイロの間隔が ESKF_IMU_TIMEOUT_US を超えた。2026-11-13〜。BUG-62）。
//   gap_s : 途絶えていた秒数（ログ用）
// ★ 途絶えていた間（SCH16T の復旧は 1 分ごとなので、ふつう 1 分以上）は伝播も観測も止まっていたので、
//   姿勢・速度は途絶える前のまま、不確かさも狭いまま。その間に旋回していると方位がずれたまま速度の観測を入れ、
//   フィルタはロール・ピッチを回して合わせに行く（attitude_on_gnss_velocity の yaw_set_ の説明と同じ理屈）。
//   2026-11-13 まで、途絶明けは平均ピッチの窓を空けるだけだった。
// ★ **大きな傾きの置き直し（relevel_big_tilt）と同じ扱いにする**（ユーザーの判断）:
//   ・傾きの不確かさを ESKF_INIT_ATT_SIGMA_MOVING_DEG へ広げる。重力の観測と速度の観測で引き戻す
//     （60 度以上狂っていたら BIGTILT が比力で置き直す）。**姿勢そのもの（q_）は動かさない**。
//     その場の比力で置き直すと、飛行中は振動・加速度のぶん傾く
//   ・方位は保ち、不確かさを ESKF_BIGTILT_YAW_SIGMA_DEG へ広げて、GNSS が 3 m/s 以上で航跡と比べる（yaw_suspect_）。
//     45 度を超えて食い違っていたら航跡で入れ直す。航跡でいきなり入れ直さないのは、飛行中は横滑りのぶんずれるため
//   ・速度は次の信用できる測位で GNSS の値に入れ直す（v_stale_）。途絶えた間に変わった分を姿勢のせいにしないため
//   ・自動ロールトリムの溜めかけと、BIGTILT の判定の途中の状態は捨てる（平均ピッチの窓は呼び出し側が空ける）
static void imu_gap_recover(float gap_s) {
    reset_tilt_covariance(ESKF_INIT_ATT_SIGMA_MOVING_DEG);
    imu_gap_event_yaw_kept_ = yaw_set_;
    if (yaw_set_) {
        const float old = P_[2][2];
        const float ys = ESKF_BIGTILT_YAW_SIGMA_DEG * (float)M_PI / 180.0f;
        for (int i = 0; i < N_ERR; i++) { P_[2][i] = 0.0f; P_[i][2] = 0.0f; }
        P_[2][2] = (old > ys * ys) ? old : ys * ys;
        yaw_suspect_   = true;                  // GNSS が戻ったら航跡と比べる
        yaw_check_arm_ = 0;
    }
    v_stale_ = true;
    bt_since_us_     = 0;
    bt_acc_lp_valid_ = false;
    trim_roll_sum_  = 0.0f;
    trim_time_s_    = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
    imu_gap_event_s_ = gap_s;
    imu_gap_event_   = true;
}

// ジャイロが飽和したサンプル（settings.h の ESKF_GYRO_SAT_DPS）。**方位を捨てる。**毎サンプル呼ぶ（何度呼んでも同じ）。
// ★ 頭打ちの値で回した姿勢は、取りこぼした角度だけ狂っている。方位は分からないので捨て、起動直後（水平化しただけ）と同じ
//   状態に戻す: σ 180°・表示は灰色と YAW ALIGN・動いている間は速度の観測を入れない・次の発進で 2D 相関か航跡で入れ直す。
//   σ だけ 180° にして方位を保たない理由は settings.h。保った方位を航跡で確かめる道（yaw_suspect_）にも乗せない
//   （航跡と比べるのは 3 m/s 以上なので、止まっている間「確かめ待ち」のまま狂った方位が出続ける。比べて 45° 以内なら
//   残るが、取りこぼしは 45° を簡単に超え、超えなくても σ は確かめた量にならない）。
// ★ ロール・ピッチは保つ（頭打ちは向きだけでなく傾きも狂わせ得るので、不確かさは IMU の途絶明けと同じ 30° へ広げる）。
//   止まっていれば待機中の維持（時定数 1 秒）、動いていれば重力の観測と大きな傾きの置き直し（ESKF_BIGTILT_*）が戻す。
// ★ 速度は次の信用できる測位で入れ直す（回している間の比力には回転に伴う加速度が混ざり、狂った姿勢で積分している）。
// ★ ゼロ点（bg_）は触らない。回している間は静止の判定に入らないので、学習の窓には混ざらない。
static void gyro_saturated() {
    yaw_set_       = false;
    yaw_src_       = ATT_YAW_SRC_NONE;
    yaw_arm_       = 0;
    yaw_arm_lo_    = 0;
    yaw_corr_reset();                       // 2D 相関は捨てた方位の座標で積んでいる
    yaw_suspect_   = false;
    yaw_check_arm_ = 0;
    const float ys = ESKF_YAW_UNKNOWN_SIGMA_DEG * (float)M_PI / 180.0f;
    for (int i = 0; i < N_ERR; i++) { P_[2][i] = 0.0f; P_[i][2] = 0.0f; }
    P_[2][2] = ys * ys;
    reset_tilt_covariance(ESKF_INIT_ATT_SIGMA_MOVING_DEG);
    v_stale_ = true;
    pitch_avg_reset();
    trim_roll_sum_  = 0.0f;
    trim_time_s_    = 0.0f;
    trim_run_count_ = 0;
    trim_run_sum_   = 0.0f;
}

// 静止中にジャイロのゼロ点を測って bg_ に入れる。
// 引数は**生の**ジャイロ（bg_ を引く前）。目的と注意は settings.h の
// ESKF_BIAS_LEARN_SAMPLES / ESKF_BIAS_LEARN_STD_RADS のコメント参照。
// ★ **5 秒の窓の中でジャイロが揺れていたら（標準偏差が ESKF_BIAS_LEARN_STD_RADS 以上）覚えずに捨てる**（2026-11-18）。
//   静止の判定（|ω| < 1.5deg/s）は SCH16T の雑音の 150 倍ゆるく、人が支えて揺れている自転車の平均を
//   ゼロ点として覚えていた（ヨー軸 −0.049 → +0.009deg/s。GNSS を切ると方位が 180 秒で −8 度ずれた）。
//   大きさではなくばらつきで見るのは、ゼロ点そのものが最大 0.3deg/s あり得るため（大きさで切ると覚えられない個体が出る）。
// ★ 門を通った値は、前回までの値（SD の gyro_bias.txt か、この起動で確かめた値）か続けて通った直前の窓と比べ、
//   ESKF_BIAS_CONFIRM_DIFF_RADS 以内のときだけ「覚えた」にする（2026-11-18。ばらつきの門は一定の速さの回転を見分けられないため）。
// ★ 方位が決まったあと（地上での維持）に覚えたら、動き出すときに P_bg を広げる（attitude_on_gyro の地上の維持のブロック）。
//   止まっている間に測った値は正確だが、走り出すと 0.016deg/s ほど違った（2026-11-18、振動か温度）。広げないと
//   推定がその差を信じ込み、学び直しが遅れる。止まっている間は広げない（広げると方位の σ が止まっているだけで育つ）。
// ★★ 時刻は**サンプルの時刻 t_us** で持つ（2026-11-18 の夜の車で発覚）。古くなったかの判定（attitude_on_gyro）も t_us で見ていて、
//   ここを time_us_32()（いま）にしていたら、サンプルはリングにためてから数 ms 遅れて渡るので t_us − いま が負になり、
//   符号なしの差が巨大になって**覚えた直後に毎回「古い」と捨てていた**（夜の 6 回とも `bias not learned`）。PC の試験は時計が 1 つで見えなかった。
static void learn_gyro_bias(const float g[3], uint32_t t_us) {
    if (bias_n_ == 0)
        for (int i = 0; i < 3; i++) bias_g0_[i] = g[i];
    for (int i = 0; i < 3; i++) {
        const float d = g[i] - bias_g0_[i];
        bias_sum_[i] += d;
        bias_sq_[i]  += d * d;
    }
    if (++bias_n_ < ESKF_BIAS_LEARN_SAMPLES) return;
    const float n = (float)bias_n_;
    const float lim2 = ESKF_BIAS_LEARN_STD_RADS * ESKF_BIAS_LEARN_STD_RADS;
    bool still = true;
    float mean[3];
    for (int i = 0; i < 3; i++) {
        const float md = bias_sum_[i] / n;
        const float var = bias_sq_[i] / n - md * md;
        if (var >= lim2) still = false;
        mean[i] = bias_g0_[i] + md;
    }
    if (!still) { bias_learn_reset(); return; }   // 揺れていた。続けて通った窓の比べも切る

    // ---- 確かめる: 前回までの値か、続けて通った直前の窓と ESKF_BIAS_CONFIRM_DIFF_RADS 以内か ----
    const float d_ref  = bias_ref_valid_  ? bias_maxdiff(mean, bias_ref_)  : 1e9f;
    const float d_prev = bias_prev_valid_ ? bias_maxdiff(mean, bias_prev_) : 1e9f;
    int how = -1;                                      // -1 = 比べる相手が無い（最初の窓）
    if (d_ref < ESKF_BIAS_CONFIRM_DIFF_RADS)       how = 0;
    else if (d_prev < ESKF_BIAS_CONFIRM_DIFF_RADS) how = 1;
    else if (bias_ref_valid_)                      how = 2;
    const bool confirmed = (how == 0 || how == 1);
    for (int i = 0; i < 3; i++) bias_prev_[i] = mean[i];
    bias_prev_valid_ = true;

    // 方位が決まる前は確かめる前でも入れる（P_bg は待機中 1 deg/s のままで、方位を入れるとき 0.3 deg/s。何も入れないよりよい）。
    // 方位が決まったあと（地上の維持）は確かめた値だけ入れる（GNSS で学んだ値を、確かめていない値で上書きしない）。
    if (!yaw_set_ || confirmed) {
        for (int i = 0; i < 3; i++) {
            float b = mean[i];
            if (b >  ESKF_BIAS_LEARN_MAX_RADS) b =  ESKF_BIAS_LEARN_MAX_RADS;
            if (b < -ESKF_BIAS_LEARN_MAX_RADS) b = -ESKF_BIAS_LEARN_MAX_RADS;
            bg_[i] = b;
        }
    }
    if (confirmed) {
        if (yaw_set_) bias_learned_on_ground_ = true;
        const uint32_t now = t_us;
        bias_learn_ok_valid_ = true;
        bias_learn_ok_us_    = now;
        for (int i = 0; i < 3; i++) bias_ref_[i] = mean[i];
        bias_ref_valid_ = true;
        // SD へ: この起動で初めて確かめたとき、そのあとは ESKF_BIAS_SAVE_DIFF_RADS 以上変わって ESKF_BIAS_SAVE_MIN_US たったとき
        if (!bias_saved_valid_ ||
            (bias_maxdiff(mean, bias_saved_) >= ESKF_BIAS_SAVE_DIFF_RADS &&
             (uint32_t)(now - bias_saved_us_) >= ESKF_BIAS_SAVE_MIN_US)) {
            for (int i = 0; i < 3; i++) { bias_saved_[i] = mean[i]; bias_save_[i] = mean[i]; }
            bias_saved_valid_ = true;
            bias_saved_us_    = now;
            bias_save_req_    = true;
        }
    }
    // ログ: 最初に確かめたとき、前回の値と食い違ったとき
    if ((confirmed && !bias_confirmed_once_) || how == 2) {
        bias_ev_how_  = (uint8_t)how;
        bias_ev_diff_ = (how == 1) ? d_prev : d_ref;
        for (int i = 0; i < 3; i++) bias_ev_b_[i] = mean[i];
        bias_ev_ = true;
    }
    if (confirmed) bias_confirmed_once_ = true;
    bias_window_clear();
}

// 地上での維持でゼロ点を覚えたあと、動き出すときに P_bg を ESKF_BIAS_LEARN_PBG_RADS まで広げる（小さければ）。
// 他の状態との相関は、ゼロ点を書き換えた時点で意味を失っているので落とす。
static void bias_widen_after_ground() {
    const float s2 = ESKF_BIAS_LEARN_PBG_RADS * ESKF_BIAS_LEARN_PBG_RADS;
    for (int k = 6; k < 9; k++) {
        const float old = P_[k][k];
        for (int i = 0; i < N_ERR; i++) { P_[k][i] = 0.0f; P_[i][k] = 0.0f; }
        P_[k][k] = (old > s2) ? old : s2;
    }
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

    // 静止中は平均を溜める。これが初期姿勢の重力方向になる。
    if (!static_now_ || initialized_) return;
    for (int i = 0; i < 3; i++) acc_sum_[i] += a[i];
    acc_count_++;
}

uint8_t attitude_get_yaw_source() { return yaw_src_; }
bool    attitude_yaw_ready()      { return yaw_set_; }

// センサー座標系で見た「上」方向 = 回転行列の第 3 行。
//   バリオはこれと比力の内積で鉛直加速度を取る（imu.cpp の imu_feed_accel）。
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
// ※ 静止判定はゼロ点バイアス学習の前提条件なので、バイアス除去後ではなく生ジャイロ値を見る（循環参照防止。eskf.py と同一）。
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
        bias_learn_reset();
    }
    static_now_ = now;
}

void attitude_on_gyro(const float g[3], uint32_t t_us) {
    // ---- サンプル間隔（上の「サンプル間隔」の説明）----
    // reset 後の最初の 1 サンプルは前が無いので 0（伝播しない）
    const float dt_meas = gyro_seen_ ? (float)(uint32_t)(t_us - last_gyro_us_) * 1e-6f : 0.0f;
    last_gyro_us_ = t_us;
    gyro_seen_ = true;
    lvl_undo_now_us_ = t_us;

    // 意図して直さない（2026-11-14 の監査）: IMU の途絶明けに静止の判定（static_since_us_・acc_sum_）を区切らない。水平化前は静止中の引き戻し（時定数 1 秒）で数秒で直り、水平化後は今の加速度で寄せるので害が無い（直すなら eskf.py も合わせる）
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
        // ★ GNSS が動いていると言っている間は静止の初期化をしない（2026-11-18 夜の車で発覚）。なめらかに減速・加速している車は
        //   ジャイロが静かで |a| も g に近いので静止の判定を通り、1.2 m/s で減速しながら初期化して傾きが 2〜4° ずれていた。
        //   ESKF_ACCEL_HOLD_MAX_US 続いたら許す（一定の速さで飛んでいる最中の再起動）。それより先に 3 m/s を超えれば
        //   attitude_on_gnss_velocity の「動き出したときの初期化」（σ 30°）が走る。
        {
            const bool mov = good_gnss_fresh() && good_gsp_ >= gnss_moving_mps();
            if (mov) { if (!init_mov_on_) { init_mov_on_ = true; init_mov_since_us_ = t_us; } }
            else init_mov_on_ = false;
        }
        const bool init_gnss_ok = !init_mov_on_ || (uint32_t)(t_us - init_mov_since_us_) >= ESKF_ACCEL_HOLD_MAX_US;
        if (static_now_ && (t_us - static_since_us_) >= ESKF_STATIC_INIT_US &&
            acc_count_ >= 4 && init_gnss_ok) {
            const float a_avg[3] = { acc_sum_[0] / (float)acc_count_,
                                     acc_sum_[1] / (float)acc_count_,
                                     acc_sum_[2] / (float)acc_count_ };
            init_level(a_avg, ESKF_INIT_ATT_SIGMA_DEG);
        }
        if (!initialized_) return;   // まだ初期化できていないので伝播しない
    }

    if (!accel_valid_) return;

    // ---- ジャイロの飽和（2026-11-27〜。settings.h の ESKF_GYRO_SAT_DPS）----
    // ★ 下の「加速中か」より前に置く（方位を捨てると hold_max が変わる。tools/imulog/eskf.py もループの頭）。
    //   伝播は頭打ちの値のまま続ける（ほかに使える値が無い）。
    {
        const float lim = ESKF_GYRO_SAT_DPS * (float)M_PI / 180.0f;
        if (fabsf(g[0]) >= lim || fabsf(g[1]) >= lim || fabsf(g[2]) >= lim) {
            if (!gyro_sat_on_) {
                gyro_sat_on_       = true;
                gyro_sat_since_us_ = t_us;
                gyro_sat_n_        = 0;
                gyro_sat_yaw_drop_ = yaw_set_;
            }
            gyro_sat_last_us_ = t_us;
            gyro_sat_n_++;
            gyro_saturated();
        } else if (gyro_sat_on_ && (uint32_t)(t_us - gyro_sat_last_us_) >= ESKF_GYRO_SAT_END_US) {
            gyro_sat_on_         = false;
            gyro_sat_event_s_    = (float)(uint32_t)(gyro_sat_last_us_ - gyro_sat_since_us_) * 1e-6f;
            gyro_sat_event_n_    = gyro_sat_n_;
            gyro_sat_event_drop_ = gyro_sat_yaw_drop_;
            gyro_sat_event_      = true;
        }
    }

    // ---- 加速中か（滑走の加速を傾きと取り違えないため。2026-11-18〜。判定の考え方は settings.h の ESKF_ACCEL_*）----
    // ★ 推定の座標では見ない（寄せたぶん判定が壊れる。2026-11-18 夜の車）。加速度計の値を、直近の平均を**ジャイロで本体と一緒に回したもの**と比べ、
    //   平均の向きに直角な差（回らずに比力の向きが変わった＝加速）を見る。GNSS が動いていると言っていれば、それも加速中。
    // ★ 「加速中」になった瞬間に、直前 ESKF_ACCEL_UNDO_US の寄せを取り消す（門が開くまでの立ち上がりに寄せた分）。
    {
        const float dta = (dt_meas > 0.0f && dt_meas < 0.1f) ? dt_meas : 0.0f;
        if (!acc_lp_valid_) {
            for (int i = 0; i < 3; i++) { acc_lp_[i] = accel_last_[i]; acc_f_[i] = accel_last_[i]; }
            acc_lp_valid_ = true;
        }
        // 平均を本体と一緒に回す（ワールドに固定した向きは、センサー軸では −ω × v で動く）
        const float w[3] = { g[0] - bg_[0], g[1] - bg_[1], g[2] - bg_[2] };
        const float c0 = w[1]*acc_lp_[2] - w[2]*acc_lp_[1];
        const float c1 = w[2]*acc_lp_[0] - w[0]*acc_lp_[2];
        const float c2 = w[0]*acc_lp_[1] - w[1]*acc_lp_[0];
        acc_lp_[0] -= c0 * dta; acc_lp_[1] -= c1 * dta; acc_lp_[2] -= c2 * dta;
        const float kf = (dta < ESKF_ACCEL_LP_S) ? dta / ESKF_ACCEL_LP_S : 1.0f;
        for (int i = 0; i < 3; i++) acc_f_[i] += (accel_last_[i] - acc_f_[i]) * kf;
        // 速く回っている間（静止の判定の角速度以上）は平均を加速度計の値へ寄せず、ジャイロで回すだけにする
        //   （回っている間の比力には回転に伴う加速度が混ざる。寄せると止まったあと平均がずれて「加速中」が続く。attitude_test の G2）
        if (w[0]*w[0] + w[1]*w[1] + w[2]*w[2] < ESKF_STATIC_GYRO_RADS * ESKF_STATIC_GYRO_RADS)
            for (int i = 0; i < 3; i++) acc_lp_[i] += (acc_f_[i] - acc_lp_[i]) * (dta / ESKF_ACCEL_CHANGE_TAU_S);
        const float ln = sqrtf(acc_lp_[0]*acc_lp_[0] + acc_lp_[1]*acc_lp_[1] + acc_lp_[2]*acc_lp_[2]);
        float dp2 = 0.0f;
        acc_dp_raw2_ = 0.0f;
        if (ln > 1.0f) {
            const float u[3] = { acc_lp_[0] / ln, acc_lp_[1] / ln, acc_lp_[2] / ln };
            const float d[3] = { acc_f_[0] - acc_lp_[0], acc_f_[1] - acc_lp_[1], acc_f_[2] - acc_lp_[2] };
            const float du = d[0]*u[0] + d[1]*u[1] + d[2]*u[2];
            for (int i = 0; i < 3; i++) { const float e = d[i] - du * u[i]; dp2 += e * e; }
            const float r[3] = { accel_last_[0] - acc_lp_[0], accel_last_[1] - acc_lp_[1], accel_last_[2] - acc_lp_[2] };
            const float ru = r[0]*u[0] + r[1]*u[1] + r[2]*u[2];
            for (int i = 0; i < 3; i++) { const float e = r[i] - ru * u[i]; acc_dp_raw2_ += e * e; }
        }
        // 信用できる GNSS の対地速度（good_gsp_。鮮度は測位の時刻）
        const bool gfresh = good_gnss_fresh();
        const bool gnss_moving = gfresh && good_gsp_ >= gnss_moving_mps();
        if (gfresh && good_gsp_ < ESKF_ACCEL_GNSS_STOP_MPS) {
            if (!accel_gnss_stop_on_) { accel_gnss_stop_on_ = true; accel_gnss_stop_since_us_ = t_us; }
        } else {
            accel_gnss_stop_on_ = false;
        }
        // ブレーキで止まったあとの平均の残り: GNSS が止まっていると 2 秒言い、「加速中」も 2 秒以上なら平均を取り直す
        if (accel_horiz_on_ && accel_gnss_stop_on_ &&
            (uint32_t)(t_us - accel_gnss_stop_since_us_) >= ESKF_ACCEL_GNSS_STOP_US &&
            (uint32_t)(t_us - accel_horiz_since_us_) >= ESKF_ACCEL_GNSS_STOP_US) {
            for (int i = 0; i < 3; i++) acc_lp_[i] = acc_f_[i];
            dp2 = 0.0f;
        }
        const bool by_acc = dp2 > ESKF_ACCEL_CHANGE_MPS2 * ESKF_ACCEL_CHANGE_MPS2;
        if (by_acc || gnss_moving) {
            if (!accel_horiz_on_) { accel_horiz_on_ = true; accel_horiz_since_us_ = t_us; accel_undo_why_ = by_acc ? 0 : 1; }
        } else {
            accel_horiz_on_ = false;
        }
        const bool was = accelerating_;
        // 方位が決まる前で GNSS が動いていると言い続けている間は、上限を ESKF_ACCEL_HOLD_MAX_PREYAW_US まで延ばす（2026-11-22。settings.h）
        const uint32_t hold_max = (!yaw_set_ && gnss_moving) ? ESKF_ACCEL_HOLD_MAX_PREYAW_US : ESKF_ACCEL_HOLD_MAX_US;
        accelerating_ = accel_horiz_on_ &&
                        (uint32_t)(t_us - accel_horiz_since_us_) < hold_max;
        // 上限が来たら、いまの値を新しい平均にする（加速ではなく、ジャイロと加速度計の食い違いが続いているとみなす。
        //   取り直さないと、方位が決まる前の重力観測の門（acc_dp_raw2_）が平均の時定数で何十秒も閉じたままになる）
        if (accel_horiz_on_ && !accelerating_ && was) {
            for (int i = 0; i < 3; i++) acc_lp_[i] = acc_f_[i];
            acc_dp_raw2_ = 0.0f;
        }
        if (accelerating_ && !was) {
            // 「加速中」になった瞬間: 門が開くまでに寄せた分を取り消す
            const float deg = lvl_undo_apply(t_us);
            if (deg > 0.0f) { accel_undo_deg_ = deg; accel_undo_event_ = true; }
        }
        if (accelerating_) bias_learn_reset();   // 加速中の分をゼロ点の窓に混ぜない
    }

    // ---- 待機中の維持（ヨー未設定のあいだ）----
    // ★ **ヨーが未設定のあいだ GNSS 速度観測を入れないので、観測が 1 つも無い。**
    //   放っておくとロール・ピッチはジャイロオフセットの積分で漂う
    //   （SCH16T なら 0.1deg/s typ で 10 分 60 度）。静止しているあいだは
    //   重力へ寄せ続け、同時にジャイロのゼロ点を測る。理由と数値は settings.h の
    //   ESKF_RELEVEL_GAIN / ESKF_BIAS_LEARN_SAMPLES のコメント。
    // ★ Phase 2 の重力観測（level_update）とここは**併存させている。**役割が違う:
    //   ・ここ（相補フィルタ・ジャイロレート 250Hz・時定数 1 秒）… 待機中の維持。下の
    //     reset_covariance が毎サンプル P を初期値へ戻すので、level_update が
    //     溜めた情報はどうせ捨てられる。実効の引き戻しはこちらが担う
    //   ・level_update（正式な観測・2Hz）… 飛行中と GNSS 断中の錨
    //   どちらも重力へ寄せる方向なので競合しない。1 つに統合するのは、
    //   待機中の挙動（tools/attitude_test の D1/D2）を再検証してからにする。
    if (!yaw_set_ && static_now_ && !accelerating_) {
        level_toward_gravity_rec(accel_last_, ESKF_RELEVEL_GAIN);   // 記録つき（「加速中」になったら直前の分を取り消す）
        learn_gyro_bias(g, t_us);    // ★ bg_ を引く前の生の値を渡すこと
        // 観測が無いので共分散を伝播させても情報は増えない。
        // 水平化し続けている事実と釣り合うよう初期値へ置き直す。
        reset_covariance(ESKF_INIT_ATT_SIGMA_DEG, ESKF_YAW_UNKNOWN_SIGMA_DEG);
    }

    // ---- 地上での維持（ヨーが決まった後）----
    // ★ 上の待機中の維持はヨーが決まるまでしか働かない。ヨーが決まった後に傾きの誤差が
    //   60 度を超えると戻す手段が無かったので、地上で止まっているときは寄せる
    //   （2026-10-27。理由と条件は settings.h の ESKF_GROUND_* のコメント）。
    //   上と違ってヨーの分散は残し、ゼロ点は GNSS が地上を確かめたときだけ学習する。
    {
        const bool ground = yaw_set_ && on_ground_still(t_us) && !accelerating_;   // 加速中は寄せない（上の「加速中か」）
        if (ground) {
            if (!ground_now_) ground_relevels_++;
            level_toward_gravity_rec(accel_last_, ESKF_RELEVEL_GAIN);   // 記録つき（「加速中」になったら直前の分を取り消す）
            if (gnss_says_ground()) learn_gyro_bias(g, t_us);   // ★ bg_ を引く前の生の値を渡すこと
            reset_tilt_covariance(ESKF_INIT_ATT_SIGMA_DEG);
        } else if (ground_now_ && bias_learned_on_ground_) {
            // 地上の維持を抜けた（動き出した）。止まっている間に覚えたゼロ点の不確かさを広げる（learn_gyro_bias のコメント）
            bias_widen_after_ground();
            bias_learned_on_ground_ = false;
        }
        ground_now_ = ground;
    }

    // 前が無い・途絶明けは伝播しない（上の「サンプル間隔」の説明）
    if (dt_meas <= 0.0f) return;
    if (dt_meas > (float)ESKF_IMU_TIMEOUT_US * 1e-6f) {
        // 途絶明けは平均ピッチの窓を空ける。途絶える前の値（復旧は 1 分ごとなので 1 分以上前）を
        // 新しい値と 10 秒混ぜないため。風はゆっくりしか変わらないので LPF は残す。
        pitch_avg_reset();
        // 姿勢・速度の不確かさを広げ、方位は航跡で確かめ直す（BUG-62。imu_gap_recover のコメント）
        imu_gap_recover(dt_meas);
        return;
    }
    const float dt = dt_meas;
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
    vhist_push(t_us);                     // GNSS 速度の遅れの補償用（ESKF_GNSS_VEL_DELAY_MS）

    // ---- 発進加速の常時前向き 2D 相関用の IMU 水平速度変化（ヨー未設定の間のみ。settings.h の ESKF_YAW_CORR_*）----
    // プラットホーム上での手押し・位置調整の誤蓄積を残さないよう、静止（かつ非加速）が
    // ESKF_YAW_CORR_STATIC_RESET_US（1 秒）続いたら IMU 側・GNSS 低域・相関器（C, S）をすべて 0 にクリアする。
    // 静止でゼロ点を覚えた記録が古くなったら捨てる（time_us_32 は約 71 分で一周するので、差を取る前に無効にしておく）
    if (bias_learn_ok_valid_ && (uint32_t)(t_us - bias_learn_ok_us_) >= ESKF_YAW_INIT_BIAS_FRESH_US) bias_learn_ok_valid_ = false;
    if (!yaw_set_) {
        if (static_now_ && !accelerating_) {
            yaw_static_s_ += dt;
            if (yaw_static_s_ * 1e6f >= (float)ESKF_YAW_CORR_STATIC_RESET_US) {
                yaw_imu_hp_[0] = yaw_imu_hp_[1] = 0.0f;
                // ★ GNSS の低域は 0 にせず、次の測位で取り直す（2026-11-18）。等速で動いている間に静止の判定が入ると
                //   （ジャイロが静かで |a|≒g）、0 にした低域との差が偽の速度変化になって相関を汚す。
                yaw_gnss_lp_valid_ = false;
                yaw_corr_c_ = yaw_corr_s_ = yaw_corr_n_ = 0.0f;
            }
        } else {
            yaw_static_s_ = 0.0f;
            const float dec_i = expf(-dt / ESKF_YAW_CORR_TAU_S);
            yaw_imu_hp_[0] = yaw_imu_hp_[0] * dec_i + Rf[0] * dt;
            yaw_imu_hp_[1] = yaw_imu_hp_[1] * dec_i + Rf[1] * dt;
        }
    }

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
    // ★ **+Q は重力観測（level_update）より前に足す。** 予測（Phi P Phi^T + Q）を済ませてから観測を入れる
    //   教科書の順で、tools/imulog/eskf.py（predict の中で + Q → update_level）と同じ。
    //   2026-11-10 まではこの関数の末尾で足していて、重力観測が Q を足す前の P で入っていた。
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

    // ---- ワールド系のヨーレート（旋回しているか）----
    // ボディ角速度を回して Z 成分を取る。自動ロールトリム・風推定・重力観測のゲートで共用。
    // ★★ **重力観測のゲートより前で更新すること。** 後ろに置くとゲートが
    //   1 サンプル前（250Hz では 4ms 前、旧 50Hz では 20ms 前）の値で判定し、**旋回の入り口で観測が漏れる**
    //   （5 度の協調旋回 30 秒で 1 回漏れるのを tools/attitude_test の F2 が捕まえた）。
    {
        const float wz = R[2][0]*w[0] + R[2][1]*w[1] + R[2][2]*w[2];
        const float wz_dps = wz * 180.0f / (float)M_PI;
        yaw_rate_inst_ = wz_dps;
        yaw_rate_lp_ += (wz_dps - yaw_rate_lp_) * (dt / (2.0f + dt));
        // 全角速度。重力観測のゲート用（settings.h の ESKF_LEVEL_MAX_RATE_DPS）。
        const float wn_dps = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]) * 180.0f / (float)M_PI;
        if (wn_dps > ESKF_LEVEL_MAX_RATE_DPS) rate_high_us_ = t_us;
    }

    // ---- 重力観測（Phase 2）----
    // ★ 根拠と数値は settings.h の ESKF_LEVEL_* のコメント。要点だけ:
    //   主ゲートは**旋回レート**（ジャイロだけから作れるので GNSS 断中も働く）。
    //   `|f| ≒ g` は人力機のバンク角を判別できないので粗い健全性チェックのみ。
    //   旋回レートは水平な軸まわりの回転を見ないので、全角速度 |ω| でも切る（2026-10-27）。
    //   前後加速度は GNSS からしか作れないので、断中はその条件を省く。
#if ESKF_LEVEL_ENABLED
    if ((uint32_t)(t_us - level_last_us_) >= ESKF_LEVEL_INTERVAL_US) {
        const float amag = sqrtf(accel_last_[0]*accel_last_[0]
                               + accel_last_[1]*accel_last_[1]
                               + accel_last_[2]*accel_last_[2]);
        bool pass = (fabsf(yaw_rate_lp_)   <= ESKF_LEVEL_MAX_YAWRATE_DPS)
                 && (fabsf(yaw_rate_inst_) <= ESKF_LEVEL_MAX_YAWRATE_INST)
                 && ((uint32_t)(t_us - rate_high_us_) >= ESKF_LEVEL_RATE_QUIET_US)
                 && (fabsf(amag - GRAVITY) <= ESKF_LEVEL_ACCMAG_TOL_MPS2);
        // 前後加速度は測位があるときだけ見る（無いときは条件ごと省く）
        // ★ 鮮度は gsp_fresh()（測位）で見る。gnss_vel_fresh()（速度の観測を入れた時刻）だと方位が決まる前に外れる（2026-11-18）。
        if (pass && gsp_fresh())
            pass = fabsf(longacc_lp_) <= ESKF_LEVEL_MAX_LONGACC_MPS2;
        // 方位が決まる前の加速中は入れない（上の「加速中か」）。前後加速度は GNSS から作るので、加速の出だし 0.3〜0.5 秒は
        // まだ上の門に見えず、そこで 1 回入ると傾きが 4 度ほど寄ってしまう。方位が決まったあとは GNSS 速度の観測が傾きを支えるので見ない。
        if (pass && !yaw_set_ && accelerating_) pass = false;
        // 方位が決まる前は、均していない今の値でも加速していないことを見る（2Hz の 1 回が門の開く直前の加速を拾うと、
        //   方位が決まる前は P が大きいので 1 回で数度寄る。いきなり 1.5 m/s² の模擬で 4.5°）。揺れで 1 回見送るだけなので害は無い
        if (pass && !yaw_set_ && acc_dp_raw2_ > ESKF_ACCEL_CHANGE_MPS2 * ESKF_ACCEL_CHANGE_MPS2) pass = false;
        if (pass) {
            level_update(accel_last_, ESKF_LEVEL_SIGMA_MPS2);
            level_last_us_ = t_us;
        }
    }
#endif

    // ---- 動いているときの、大きくずれた傾きの置き直し ----
    // ★ 根拠と数値は settings.h の ESKF_BIGTILT_* のコメント。要点だけ:
    //   ジャイロが静かで |f| ≒ g なのに、比力の向きと推定の上向きが 60 度以上食い違い続けたら、推定のほうが壊れている。
    //   重力観測は 60 度以上ずれていると入れないので（level_update）、動いている間はほかに戻す手段が無い。
    // ★ 低域は毎サンプル更新する（判定しない回も）。条件がそろってから均し始めると、直前の動きを引きずる。
    {
        if (!bt_acc_lp_valid_) {
            for (int i = 0; i < 3; i++) bt_acc_lp_[i] = accel_last_[i];
            bt_acc_lp_valid_ = true;
        } else {
            const float ka = dt / (ESKF_BIGTILT_ACC_TAU_S + dt);
            for (int i = 0; i < 3; i++) bt_acc_lp_[i] += (accel_last_[i] - bt_acc_lp_[i]) * ka;
        }
        const float wn_dps = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]) * 180.0f / (float)M_PI;
        bt_rate_lp_ += (wn_dps - bt_rate_lp_) * (dt / (ESKF_BIGTILT_RATE_TAU_S + dt));

        // 地上での維持・待機中の維持が働いている間は見ない（そちらが寄せる。G3 はヨーを捨てないことを見ている）
        const bool maintained = ground_now_ || (!yaw_set_ && static_now_);
        bool bad = false;
        float err_deg = 0.0f;
        if (!maintained && bt_rate_lp_ < ESKF_BIGTILT_MAX_RATE_DPS) {
            const float fb[3] = { bt_acc_lp_[0] - ba_[0], bt_acc_lp_[1] - ba_[1], bt_acc_lp_[2] - ba_[2] };
            const float fn = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);
            if (fn > 1.0f && fabsf(fn - GRAVITY) <= ESKF_LEVEL_ACCMAG_TOL_MPS2) {
                float Rn[3][3];
                quat_to_R(q_, Rn);
                float c = (Rn[2][0]*fb[0] + Rn[2][1]*fb[1] + Rn[2][2]*fb[2]) / fn;   // 推定の上向きとのなす角の cos
                if (c > 1.0f) c = 1.0f;
                if (c < -1.0f) c = -1.0f;
                err_deg = acosf(c) * 180.0f / (float)M_PI;
                bad = (err_deg >= ESKF_BIGTILT_MIN_DEG);
            }
        }
        if (!bad) {
            bt_since_us_ = 0;
        } else if (bt_since_us_ == 0) {
            bt_since_us_ = t_us ? t_us : 1;
        } else if ((uint32_t)(t_us - bt_since_us_) >= ESKF_BIGTILT_HOLD_US) {
            relevel_big_tilt(bt_acc_lp_, err_deg);
            bt_since_us_ = 0;
        }
    }

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
        // ★ **「いま外れている」と「外されたことがある」を分けて持つ。**
        //   needs_apply_ は APPLY するまで下がらないラッチで、較正のやり直しを促すもの。
        //   一方 off_mount_now_ は**今この瞬間の姿勢**で、充電中の警告抑制に使う
        //   （GPS_TFT_map.ino の warn_muted_off_mount）。しきい値をここ 1 か所に保つため
        //   判定はこの中で行い、外へは真偽値だけ出す。
        off_mount_now_ = rpy_enabled_ &&
                         (fabsf(r) > OFF_MOUNT_DEG || fabsf(p) > OFF_MOUNT_DEG);
        if (off_mount_now_) needs_apply_ = true;

        // 過去 PITCH_AVG_SEC 秒の移動平均で平均ピッチを作る（上の pavg_* の説明）
        pitch_avg_push(p, dt);

        const float sp = sqrtf(v_[0]*v_[0] + v_[1]*v_[1]);
        // 直進しているか。ロール自動トリムと風推定で共用する。速さの門だけ別（2026-11-22〜）:
        //   風は WIND_UPDATE_MIN_SPEED_MPS（2 m/s。強い向かい風の 2〜3 m/s の飛行でも出す）、
        //   トリムは ROLL_TRIM_MIN_SPEED（3 m/s のまま。覚えた値は SD に残って次の飛行へ持ち越すので、
        //   着水後の曳航やゆっくりの試走で覚えないように）。
        // ★ sp は GNSS の対地速度ではなく **ESKF が積分した速度** なので、
        //   観測が無いと育つ（gnss_vel_fresh() のコメント参照）。鮮度を必ず併せて見る。
        const bool straight_moving = gnss_vel_fresh() &&
                                     (fabsf(yaw_rate_lp_) < ROLL_TRIM_YAWRATE_DPS);
        const bool straight_flight = straight_moving && (sp > WIND_UPDATE_MIN_SPEED_MPS);

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
        if (rpy_enabled_ && wind_enabled_ && straight_flight && tas_gamma_ready()) {
            const float va  = airspeed_from_pitch(alpha_from_pitch_avg());
            // y は真方位（北=0・東=90）。ENU では E=sin, N=cos。
            const float ya  = y * (float)M_PI / 180.0f;
            const float we  = v_[0] - va * sinf(ya);
            const float wn  = v_[1] - va * cosf(ya);
            const float aw  = dt / (WIND_LPF_SEC + dt);
            wind_e_ += (we - wind_e_) * aw;
            wind_n_ += (wn - wind_n_) * aw;
            if (wind_fill_s_ < WIND_LPF_SEC) wind_fill_s_ += dt;
        } else if (!tas_gamma_ready() || !rpy_enabled_ || !wind_enabled_ ||
                   (gnss_vel_fresh() && sp <= WIND_MIN_SPEED_MPS)) {
            // TAS 欠測・地上・OFF では推定をやり直す。旋回中と GNSS 断中は直前の値を保持する。
            // ★ 遅いかどうかは **GNSS の速度の観測が生きているときだけ** sp で見る（2026-11-13〜。BUG-63）。
            //   断中の sp は加速度の積分で漂うので、飛んでいても 2 m/s を割ると保持していた風を捨てていた
            //   （画面では断中の風はオレンジの × 付きで残す作り。eskf_display_gnss_stale）。
            //   逆に断中に漂って 2 m/s を超えても、それで「飛んでいる」とはみなさない（更新は上の straight_flight が鮮度を見ている）。
            wind_fill_s_ = 0.0f;
            wind_e_ = wind_n_ = 0.0f;
        }

        const bool straight = straight_moving && (sp > ROLL_TRIM_MIN_SPEED) &&
                              rpy_enabled_ && roll_trim_enabled_;
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

}


// ============================================================
// GNSS 速度による観測更新
// ============================================================
// バイアス推定が物理的にあり得ない大きさへ発散しないよう制限する。
// 工場校正のチップなので残留バイアスは本来ごく小さい。
// 上限が無いと初期姿勢誤差の行き場としてバイアス状態が使われ、
// 誤った値に固着して姿勢補正が効かなくなる
// （2026-08-18 の v7 の実測で bg が 5.9deg/s に張り付き、偽ピッチを出した）。
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
    vhist_shift(&dx[3]);
    clamp_bias(bg_, ESKF_MAX_GYRO_BIAS);
    clamp_bias(ba_, ESKF_MAX_ACCEL_BIAS);
    // 方位が決まる前の重力観測は、発進の立ち上がりで加速度の向きへ寄せうるので記録する（ESKF_ACCEL_UNDO_US）
    if (!yaw_set_) lvl_undo_record(dx[0], dx[1], &dx[6], &dx[9]);
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

// age_us: この観測が測位の瞬間からどれだけ古いか [µs]（gnss.cpp の navpvt_age_us）。0 なら補償しない（届いた時点の v_ と比べる）。
void attitude_on_gnss_velocity(float velN, float velE, float velD, float sAcc, uint32_t age_us) {
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
        // 地上の判定用（settings.h の ESKF_GROUND_*）。こちらは信用できる測位だけで更新する。
        if (sAcc <= ESKF_SACC_MAX) {
            good_gsp_    = gsp;
            good_gsp_us_ = now_us;
            good_sacc_   = sAcc;
        }
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
        // ★ **本当に動き出したときだけ**（信用できる測位で ESKF_YAW_INIT_MIN_SPEED_MPS 以上）。
        //   1.04 までは門が無く、ファームを書いた直後のように GNSS が既に fix している起動では、
        //   止まっていても 2 秒の静止平均より先にここが走り、1 サンプルの比力で水平化していた。
        //   sAcc の悪い測位（屋内の偽の速度）の速度も v_ に入っていた。止まっているなら静止初期化を待つ
        //   （tools/imulog/eskf.py も静止区間があればそちらを使い、無いときだけ先頭で水平化する）。
        if (sAcc > ESKF_SACC_MAX) return;
        if (sqrtf(z[0]*z[0] + z[1]*z[1]) < ESKF_YAW_INIT_MIN_SPEED_MPS) return;
        if (!accel_valid_) return;
        // ★ 1 サンプルではなく加速度の低域（up_lp_。時定数 0.8 秒）で水平化する（2026-11-20）。揺れ続けて静止の判定が取れないまま
        //   滑走すると、1 サンプルに振動が乗って最初の傾きが 25〜47° ずれた（滑走の振動 σ 1〜2 m/s² の模擬）。低域なら振動は消え、
        //   残るのは滑走の加速の分（1.5 m/s² で約 9°）だけ。σ 30° のまま GNSS の観測で引き戻す。
        if (!init_level(up_lp_valid_ ? up_lp_ : accel_last_, ESKF_INIT_ATT_SIGMA_MOVING_DEG)) return;
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2]; vhist_clear();
        return;
    }

    // ---- IMU の途絶明け: 速度を GNSS の値で入れ直す（imu_gap_recover。BUG-62。2026-11-13〜）----
    // 途絶えた間に変わった速度を、残差として姿勢・バイアスのせいにしないため。速度の分散は測位の精度にし、
    // ほかとの相関は落とす（入れ直したので意味を失った）。この回は観測として使わない（ヨーを入れたときと同じ）。
    if (v_stale_) {
        if (sAcc > ESKF_SACC_MAX) return;
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2]; vhist_clear();
        const float rv = (sAcc < ESKF_SACC_MIN) ? ESKF_SACC_MIN : sAcc;
        for (int k = 3; k < 6; k++)
            for (int i = 0; i < N_ERR; i++) { P_[k][i] = 0.0f; P_[i][k] = 0.0f; }
        for (int k = 3; k < 6; k++) P_[k][k] = rv * rv;
        v_stale_ = false;
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
        // ★ **sAcc の悪い測位ではヨーを入れない**（観測と同じ ESKF_SACC_MAX）。
        //   以前はここに門が無く、屋内の偽の速度（3.8m/s・sAcc 2.06m/s）でヨーが決まった
        //   （2026-10-27）。ヨーが決まると待機中の維持が止まるので影響が大きい。
        //   悪い測位は「無かったもの」として扱い、連続の数え方は止めない
        //   （tools/imulog/eskf.py は sAcc で先に振り落としているので、それと同じになる）。
        if (sAcc > ESKF_SACC_MAX) return;
        const float sp = sqrtf(z[0]*z[0] + z[1]*z[1]);
        // ---- 常時前向き 2D 相関の更新（2026-11-18〜。docs/pons_yaw_init_plan.md）----
        // GNSS 水平速度の低域（yaw_gnss_lp_）は静止中から常に更新しておき、動き出しからの速度変化
        // dv_gnss = z - yaw_gnss_lp_ を作る。GS >= ESKF_YAW_CORR_MIN_GS_MPS（0.8m/s）のときだけ
        // IMU 側の速度変化（yaw_imu_hp_）との内積 C・外積 S を 1/sAcc² 重みで蓄積する。
        {
            const uint32_t now_us = time_us_32();
            if (!yaw_gnss_lp_valid_) {
                yaw_gnss_lp_[0] = z[0];
                yaw_gnss_lp_[1] = z[1];
                yaw_gnss_lp_valid_ = true;
                yaw_gnss_last_us_  = now_us;
            } else {
                const float dtg = (float)(uint32_t)(now_us - yaw_gnss_last_us_) * 1e-6f;
                yaw_gnss_last_us_ = now_us;
                if (dtg > 0.0f && dtg < 2.0f) {
                    const float alpha_g = 1.0f - expf(-dtg / ESKF_YAW_CORR_TAU_S);
                    yaw_gnss_lp_[0] += alpha_g * (z[0] - yaw_gnss_lp_[0]);
                    yaw_gnss_lp_[1] += alpha_g * (z[1] - yaw_gnss_lp_[1]);
                    const float dv_e = z[0] - yaw_gnss_lp_[0];
                    const float dv_n = z[1] - yaw_gnss_lp_[1];
                    if (sp >= ESKF_YAW_CORR_MIN_GS_MPS) {
                        const float rv = (sAcc < ESKF_SACC_MIN) ? ESKF_SACC_MIN : sAcc;
                        const float w = 1.0f / (rv * rv);
                        const float dec_cs = expf(-dtg / ESKF_YAW_CORR_CS_TAU_S);
                        yaw_corr_c_ = dec_cs * yaw_corr_c_ + w * (yaw_imu_hp_[0] * dv_e + yaw_imu_hp_[1] * dv_n);
                        yaw_corr_s_ = dec_cs * yaw_corr_s_ + w * (yaw_imu_hp_[0] * dv_n - yaw_imu_hp_[1] * dv_e);
                        yaw_corr_n_ = dec_cs * yaw_corr_n_ + w * sqrtf(yaw_imu_hp_[0] * yaw_imu_hp_[0] + yaw_imu_hp_[1] * yaw_imu_hp_[1])
                                                               * sqrtf(dv_e * dv_e + dv_n * dv_n);
                    }
                } else {
                    yaw_gnss_lp_[0] = z[0];
                    yaw_gnss_lp_[1] = z[1];
                }
            }
        }
        // ★ 2 段の門（2026-11-20〜。settings.h の ESKF_YAW_INIT_CORR_MIN_SPEED_MPS）:
        //   ・ESKF_YAW_INIT_CORR_MIN_SPEED_MPS（2 m/s）以上が 1 秒 … 2D 相関の門を全部通ったときだけ入れる（外れたら待つ）
        //   ・ESKF_YAW_INIT_MIN_SPEED_MPS（3 m/s）以上が 1 秒 …… 今までどおり。相関が外れたら航跡＋15°
        //   どちらも単発の化けで焼き付けないよう連続で数える。
        if (sp < ESKF_YAW_INIT_CORR_MIN_SPEED_MPS) { yaw_arm_ = 0; yaw_arm_lo_ = 0; return; }
        if (yaw_arm_lo_ < 255) yaw_arm_lo_++;
        if (sp < ESKF_YAW_INIT_MIN_SPEED_MPS) yaw_arm_ = 0;
        else if (yaw_arm_ < 255) yaw_arm_++;
        const bool arm_trk = yaw_arm_ >= ESKF_YAW_INIT_ARM_FIXES;      // 航跡へ戻ってよい
        if (yaw_arm_lo_ < ESKF_YAW_INIT_ARM_FIXES) return;
        // ENU なので航跡（真方位・北 0・時計回り）は atan2(East, North)
        float trk = atan2f(z[0], z[1]) * 180.0f / (float)M_PI;
        if (trk < 0.0f) trk += 360.0f;
        // 発進加速の 2D 相関が十分溜まっていれば atan2(S, C) で真の機首方位へ回し、mag から初期 σ を動的に決める。
        // 使わない（航跡＋ESKF_YAW_INIT_SIGMA_DEG へ戻る）条件（2026-11-18 の自転車の発進 8 回で決めた。settings.h の ESKF_YAW_CORR_*）:
        //   ・情報が足りない（mag < ESKF_YAW_CORR_MIN_MAG。等速のまま測位が始まったなど）
        //   ・まとまりが悪い（hypot(C,S)/N < ESKF_YAW_CORR_MIN_COH。1 つの回転で IMU と GNSS の向きがそろわない。止まらずに長く動いて
        //     傾きが狂ったときなど。11/18 午前の −44° はこれで 0.960）
        //   ・航跡と ESKF_YAW_CORR_MAX_TRK_DIFF_DEG を超えて食い違う（接地中の機体は横滑りしないので、まっすぐ付けた本体なら
        //     方位と航跡はほぼ同じ。傾きのかたよりは向きを一様に回すのでまとまりに出ない。11/18 午後の +22° はこれで 35°）
        const float mag = sqrtf(yaw_corr_c_ * yaw_corr_c_ + yaw_corr_s_ * yaw_corr_s_);
        const float coh = (yaw_corr_n_ > 1e-6f) ? mag / yaw_corr_n_ : 0.0f;
        uint8_t how = 1;
        float d_trk = 0.0f, sig_init = ESKF_YAW_INIT_SIGMA_DEG;
        if (mag >= ESKF_YAW_CORR_MIN_MAG) {
            how = 2;
            if (coh >= ESKF_YAW_CORR_MIN_COH) {
                const float dpsi = atan2f(yaw_corr_s_, yaw_corr_c_);
                float q_try[4] = { q_[0], q_[1], q_[2], q_[3] };
                rotate_world_z(q_try, dpsi);
                float q_keep[4] = { q_[0], q_[1], q_[2], q_[3] };
                for (int k = 0; k < 4; k++) q_[k] = q_try[k];
                float r_chk, p_chk, y_chk;
                euler_from_state(r_chk, p_chk, y_chk);
                d_trk = y_chk - trk;
                while (d_trk >  180.0f) d_trk -= 360.0f;
                while (d_trk < -180.0f) d_trk += 360.0f;
                if (fabsf(d_trk) <= ESKF_YAW_CORR_MAX_TRK_DIFF_DEG) {
                    how = 0;
                    const float sig_corr = ESKF_YAW_CORR_SIGMA_SCALE * (180.0f / (float)M_PI) / sqrtf(mag);
                    sig_init = sqrtf(ESKF_YAW_INIT_SIGMA_MIN_DEG * ESKF_YAW_INIT_SIGMA_MIN_DEG + sig_corr * sig_corr);
                    if (sig_init > ESKF_YAW_INIT_SIGMA_DEG) sig_init = ESKF_YAW_INIT_SIGMA_DEG;
                    const float s = sig_init * (float)M_PI / 180.0f;
                    for (int i = 0; i < N_ERR; i++) { P_[2][i] = 0.0f; P_[i][2] = 0.0f; }
                    P_[2][2] = s * s;
                    yaw_set_ = true;
                    yaw_src_ = ATT_YAW_SRC_GNSS;
                } else {
                    how = 3;
                    for (int k = 0; k < 4; k++) q_[k] = q_keep[k];   // 回す前に戻す（下の航跡で入れ直す）
                }
            }
        }
        // 3 m/s に届く前は、相関で入れられなかったら何もせずに待つ（q_ は how=3 のところで戻してある）
        if (how != 0 && !arm_trk) return;
        if (how != 0) init_yaw_from_track(trk, ESKF_YAW_INIT_SIGMA_DEG);
        // ★ 方位が決まる前は、待機中の維持が毎サンプル P_bg を 1°/s に戻している（ゼロ点を学び直せるように）。そのまま方位を入れると、
        //   方位の σ がゼロ点の不確かさで 1 秒に約 1° ずつ育ち、せっかく小さく入れた σ が 10 秒で 95% 15° に戻っていた（11/18 の発進 6 回）。
        //   直前の静止でゼロ点を覚えていた（ESKF_YAW_INIT_BIAS_FRESH_US 以内）なら、P_bg を ESKF_BIAS_LEARN_PBG_RADS（止まって測った値と
        //   走行中の差 0.016°/s を包む 0.02°/s）にする。
        yaw_init_bias_ok_ = bias_learn_ok_valid_ &&
                            (uint32_t)(time_us_32() - bias_learn_ok_us_) < ESKF_YAW_INIT_BIAS_FRESH_US;
        // 覚えていなければデータシートの最大 ESKF_BIAS_UNLEARNED_RADS（0.3 deg/s。待機中の 1 deg/s より小さければ）にする（2026-11-18）。
        {
            const float sb = yaw_init_bias_ok_ ? ESKF_BIAS_LEARN_PBG_RADS : ESKF_BIAS_UNLEARNED_RADS;
            const float s2 = sb * sb;
            for (int k = 6; k < 9; k++) {
                if (!yaw_init_bias_ok_ && P_[k][k] <= s2) continue;   // もう小さければそのまま
                for (int i = 0; i < N_ERR; i++) { P_[k][i] = 0.0f; P_[i][k] = 0.0f; }
                P_[k][k] = s2;
            }
        }
        yaw_init_how_     = how;
        yaw_init_mag_     = mag;
        yaw_init_coh_     = coh;
        yaw_init_trkdiff_ = d_trk;
        yaw_init_sigma_   = sig_init;
        yaw_init_gs_      = sqrtf(z[0]*z[0] + z[1]*z[1]);
        yaw_init_event_   = true;
        v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2]; vhist_clear();
        return;                     // この回は観測として使わない（速度は直接入れた）
    }

    // ---- 大きな傾きを置き直したあとは、保った方位を航跡で確かめてから観測に入れる ----
    // ★ 理由と数値は settings.h の ESKF_BIGTILT_YAW_CHECK_DEG。ずれた方位のまま速度観測を入れると、
    //   フィルタはロール・ピッチを回して合わせに行く（上の yaw_set_ の説明と同じ）。確かめ終わるまでは入れない。
    //   止まっている間は方位が航跡から分からないので待つ。
    if (yaw_suspect_) {
        if (sAcc > ESKF_SACC_MAX) return;
        const float sp = sqrtf(z[0]*z[0] + z[1]*z[1]);
        if (sp < ESKF_YAW_INIT_MIN_SPEED_MPS) {
            yaw_check_arm_ = 0;
            // ★ **止まっているとき（ESKF_GROUND_MAX_GS_MPS 未満）は、確かめは待つが速度の観測は入れる**
            //   （2026-11-13〜。BUG-64。ユーザーの判断）。止まっていれば比力は重力だけで真上を向いているので、
            //   方位のずれは速度に効かない（d(dv)/dt = −[Rf]× dθ で Rf が真上なら、真上まわりの dθ は 0 を出す）。
            //   入れないと v_ が加速度の積分で漂い、gnss_vel_fresh() が切れて風と姿勢に × が付いたままになっていた。
            //   1〜3 m/s（走り出し・加速中）は方位のずれがロール・ピッチへ回るので、今までどおり入れない。
            if (sp >= ESKF_GROUND_MAX_GS_MPS) return;
        } else {
            if (++yaw_check_arm_ < ESKF_YAW_INIT_ARM_FIXES) return;   // 単発の化けで判断しない
            float trk = atan2f(z[0], z[1]) * 180.0f / (float)M_PI;
            if (trk < 0.0f) trk += 360.0f;
            float r, p, y;
            euler_from_state(r, p, y);
            float d = trk - y;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            yaw_suspect_   = false;
            yaw_check_diff_ = d;
            if (fabsf(d) > ESKF_BIGTILT_YAW_CHECK_DEG) {
                // 方位を保ったのが外れていた（ひっくり返った推定など）。航跡で入れ直し、速度も GNSS の値にする（ヨーを入れたときと同じ）
                init_yaw_from_track(trk, ESKF_YAW_INIT_SIGMA_DEG);
                v_[0] = z[0]; v_[1] = z[1]; v_[2] = z[2]; vhist_clear();
                yaw_check_event_ = 2;
                return;                 // この回は観測として使わない（速度は直接入れた）
            }
            // 保った方位をそのまま使い（航跡では上書きしない）、この測位から観測に入れる。
            // ★ ここで v_ を GNSS の値に置き換えないこと。置き換えると速度の残差が無くなり、方位が直るのが遅れる
            //   （11/11 の再現で、トンネルを出た直後のずれが +3.6 度 → −10 度に悪化した）。
            yaw_check_event_ = 1;
        }
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

    // dx = K * (z - v)。v は測位の瞬間の推定速度（age_us が 0 ならいまの v_）
    float vp[3] = { v_[0], v_[1], v_[2] };
    if (age_us > 0) vhist_at(time_us_32() - age_us, vp);
    float y[3] = { z[0] - vp[0], z[1] - vp[1], z[2] - vp[2] };
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
    vhist_shift(&dx[3]);
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
// tools/imulog/decode_imulog.py の mount_correct() と同じ変換であること（実体は attitude.h の imu_body_euler_rad）。
static void euler_from_state(float &roll, float &pitch, float &yaw) {
    // ★ マウント回転はクォータニオンの段階で掛ける（imu_body_euler_rad）。
    //   **Euler を組み替える旧方式には戻さないこと。**この取付けでは機体が水平の
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
// ★ **IMU が途絶えたら（attitude_ready() が落ちたら）平均ピッチ・推定 TAS・風は「無し」。**
//   途絶えると attitude_on_gyro() が呼ばれず、平均の窓も風の LPF も満ちたまま凍る。
//   1.04 までここは窓だけを見ていて、自機の画面（get_imu_ok() で消える）とは別に、
//   無線（link_push_telemetry）と飛行 CSV の TAS に途絶える直前の値を送り・書き続けていた。
//   判定をここに置くのは、呼び出し側（無線・CSV・tas.cpp・画面）がそれぞれ条件を書き写さないため。
bool  attitude_pitch_avg_valid()   { return attitude_ready() && pitch_avg_full(); }

// 推定 TAS（平均ピッチを対気速度モデルで換算した値）。平均が満ちていない・IMU が途絶えたら false。
// ★ 姿勢機能が OFF でも平均ピッチ自体は回っているので、出してよいかは
//   呼び出し側（tas.cpp）が平均ピッチと同じ条件で決める。
bool attitude_get_tas(float &mps, bool &clamped, float *alpha_deg) {
    float alpha;
    if (!attitude_get_alpha_avg(alpha)) return false;
    mps = airspeed_from_pitch(alpha, &clamped);
    if (alpha_deg) *alpha_deg = alpha;
    return true;
}
bool attitude_get_alpha_avg(float &deg) {
    if (!attitude_ready() || !tas_gamma_ready()) return false;
    deg = alpha_from_pitch_avg();
    return true;
}
void attitude_set_vspeed(float mps, bool valid) {
    const bool ok = valid && mps > -1000.0f && mps < 1000.0f;
    vs_input_mps_ = ok ? mps : 0.0f;
    vs_input_valid_ = ok;
    // 小計中に復帰しても、この区間は欠測として窓から抜けるまで残す。
    if (!ok) pavg_cur_vs_valid_ = false;
    const bool active = tas_gamma_ready();
    if (gamma_state_known_ && active != gamma_state_active_) {
        gamma_state_active_ = active;
        gamma_event_active_ = active;
        gamma_event_ = true;
    }
}
bool attitude_take_gamma_event(bool &active) {
    if (!gamma_event_) return false;
    gamma_event_ = false;
    active = gamma_event_active_;
    return true;
}
float attitude_airspeed_from_pitch(float pitch_deg, bool *clamped) {
    return airspeed_from_pitch(pitch_deg, clamped);
}
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
    // IMU または TAS が欠測したら風も出さない。
    if (!attitude_ready() || !tas_gamma_ready() || !wind_enabled_ || wind_fill_s_ < WIND_LPF_SEC) return false;
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
// ★ 内部の gnss_vel_fresh() をそのまま公開する。**表示側で同じ判定を書き写さないこと**
//   （しきい値が 2 か所になり、片方だけ直した日に「錨は切れているのに取り消し線が
//     出ない」になる）。
bool     attitude_gnss_vel_fresh()       { return gnss_vel_fresh(); }
// 信用できる GNSS の速度（sAcc ≦ ESKF_SACC_MAX）が直近に届いているか（観測に使ったかどうかは問わない）。
// 表示が「GNSS が無い」と「GNSS はあるが向きが決まっていないので使えていない」を分けるのに使う（2026-11-13）
bool     attitude_gnss_good_fresh()      { return good_gnss_fresh(); }
// 向き（ヨー）が決まっていない・確かめ待ちか。この間は GNSS の速度を観測に入れない（止まっているときを除く）。
// 表示の「YAW ALIGN」（黄）に使う
bool     attitude_yaw_aligning()         { return !yaw_set_ || yaw_suspect_; }
uint32_t attitude_get_level_updates()    { return level_updates_; }
uint32_t attitude_get_ground_relevels()  { return ground_relevels_; }
uint32_t attitude_get_bigtilt_relevels() { return bigtilt_relevels_; }
bool attitude_take_bigtilt_event(float &err_deg, bool &yaw_kept) {
    if (!bigtilt_event_) return false;
    bigtilt_event_ = false;
    err_deg = bigtilt_event_deg_;
    yaw_kept = bigtilt_event_yaw_kept_;
    return true;
}
// 方位を入れた（GNSS で最初に決めた）ときの記録。how: 0 = 2D 相関 / 1 = 航跡（情報不足）/ 2 = 航跡（まとまり不足）/ 3 = 航跡（航跡と食い違い）
float attitude_yaw_init_gs() { return yaw_init_gs_; }
bool attitude_take_yaw_init_event(uint8_t &how, float &mag, float &coh, float &trk_diff_deg, float &sigma_deg, bool &bias_ok) {
    if (!yaw_init_event_) return false;
    yaw_init_event_ = false;
    how = yaw_init_how_; mag = yaw_init_mag_; coh = yaw_init_coh_;
    trk_diff_deg = yaw_init_trkdiff_; sigma_deg = yaw_init_sigma_; bias_ok = yaw_init_bias_ok_;
    return true;
}
bool attitude_take_accel_undo_event(float &deg, uint8_t &why) {
    if (!accel_undo_event_) return false;
    accel_undo_event_ = false;
    deg = accel_undo_deg_;
    why = accel_undo_why_;
    return true;
}
void attitude_set_bias_ref(const float b[3]) {
    for (int i = 0; i < 3; i++) bias_ref_[i] = b[i];
    bias_ref_valid_ = true;
}
bool attitude_take_bias_to_save(float b[3]) {
    if (!bias_save_req_) return false;
    bias_save_req_ = false;
    for (int i = 0; i < 3; i++) b[i] = bias_save_[i];
    return true;
}
bool attitude_take_bias_event(uint8_t &how, float b_dps[3], float &diff_dps) {
    if (!bias_ev_) return false;
    bias_ev_ = false;
    how = bias_ev_how_;
    for (int i = 0; i < 3; i++) b_dps[i] = bias_ev_b_[i] * 57.29578f;
    diff_dps = bias_ev_diff_ * 57.29578f;
    return true;
}
bool attitude_take_imu_gap_event(float &gap_s, bool &yaw_kept) {
    if (!imu_gap_event_) return false;
    imu_gap_event_ = false;
    gap_s    = imu_gap_event_s_;
    yaw_kept = imu_gap_event_yaw_kept_;
    return true;
}
bool attitude_take_gyro_sat_event(float &secs, uint32_t &samples, bool &yaw_dropped) {
    if (!gyro_sat_event_) return false;
    gyro_sat_event_ = false;
    secs        = gyro_sat_event_s_;
    samples     = gyro_sat_event_n_;
    yaw_dropped = gyro_sat_event_drop_;
    return true;
}
bool attitude_take_bigtilt_yaw_check(float &diff_deg, bool &reset) {
    if (yaw_check_event_ == 0) return false;
    reset = (yaw_check_event_ == 2);
    diff_deg = yaw_check_diff_;
    yaw_check_event_ = 0;
    return true;
}

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
    const float dpitch = (p - target_pitch_deg) - level_pitch_off_;   // ピッチのオフセットの変わった分
    level_roll_off_  = r - target_roll_deg;      // ロールも申告値になるようにする
    level_pitch_off_ = p - target_pitch_deg;     // ピッチは申告値になるようにする
    // ★ 平均ピッチの窓（過去 10 秒）は**前のオフセットで引いた値**。変わった分だけずらして新しい基準にそろえる
    //   （平均は線形なので、窓を捨てて 10 秒待つより正確で、表示も途切れない）。2026-11-10 までずらしておらず、
    //   APPLY のあと 10 秒間、平均ピッチと推定 TAS に較正前の値が混ざっていた。
    if (pavg_count_ > 0 || pavg_cur_dt_ > 0.0f) {
        for (int i = 0; i < PITCH_AVG_NBIN; i++) pavg_bin_sum_[i] -= dpitch * pavg_bin_dt_[i];
        pavg_cur_sum_ -= dpitch * pavg_cur_dt_;
        if (pavg_count_ > 0) pitch_avg_ -= dpitch;
    }
    needs_apply_ = false;                        // 較正したので警告を解除
    // 日付は一旦「不明」にする。正しい値は呼び出し側（.ino）が直後に入れる。
    // ここで古い日付を残すと、測位できていない場所で APPLY し直したときに
    // 「前の日の較正のままだ」と誤って警告することになる。不明 = 警告しない。
    calib_date_ = 0;
    calib_time_ = -1;

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
// ★ `< 1e30f` で inf も弾く（2026-11-13〜。BUG-66 の関連。NaN は比較が全部 false なので > 0 で落ちる）。
static bool airspeed_param_ok(float v) { return v > 0.0f && v < 1e30f; }
float attitude_get_airspeed_v0()  { return airspeed_v0_; }
void  attitude_set_airspeed_v0(float mps)  { if (airspeed_param_ok(mps)) airspeed_v0_  = mps; }
float attitude_get_airspeed_k()   { return airspeed_k_; }
void  attitude_set_airspeed_k(float deg)   { if (airspeed_param_ok(deg)) airspeed_k_   = deg; }
float attitude_get_airspeed_weight_kg() { return airspeed_weight_kg_; }
void  attitude_set_airspeed_weight_kg(float kg) {
    if (kg >= AIRSPEED_WEIGHT_MIN_KG && kg <= AIRSPEED_WEIGHT_MAX_KG) airspeed_weight_kg_ = kg;
}
float attitude_get_airspeed_min() { return airspeed_min_; }
void  attitude_set_airspeed_min(float mps) { if (airspeed_param_ok(mps)) airspeed_min_ = mps; }
float attitude_get_airspeed_max() { return airspeed_max_; }
void  attitude_set_airspeed_max(float mps) { if (airspeed_param_ok(mps)) airspeed_max_ = mps; }

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
// 較正した時刻（JST の HHMM、-1 = 不明）。日付と同じく、ありえない値は不明に倒す（壊れた設定ファイル対策）。
int32_t attitude_get_calib_time() { return calib_time_; }
void attitude_set_calib_time(int32_t hhmm) {
    if (hhmm < 0 || hhmm / 100 > 23 || hhmm % 100 > 59) { calib_time_ = -1; return; }
    calib_time_ = hhmm;
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
