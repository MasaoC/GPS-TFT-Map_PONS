// ============================================================
// File    : imu.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : BNO085 (GY-BNO080) IMU ドライバー + 3 状態 Kalman フィルター実装。
//
// ■ アーキテクチャ
//   - BNO085 はポーリングモードで動作（H_INT は未使用）
//   - imu_update() を毎ループ呼び出し、15ms 間隔でデータをポーリング読み出しする
//   - 30Hz で GAME_ROTATION_VECTOR（クォータニオン）と LINEAR_ACCELERATION を受信
//   - IMU データ到着時 → Kalman predict ステップ（状態を IMU 加速度で時間更新）
//   - 気圧高度到着時 → Kalman update ステップ（get_airdata_altitude() で観測修正）
//
// ■ 座標系
//   BNO085 (SH2) の世界座標: X=East, Y=North, Z=Up
//   LINEAR_ACCELERATION はボディフレームで出力（重力除去済み）
//   GAME_ROTATION_VECTOR のクォータニオンでボディ→ワールドに変換し、Z 成分を取る
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
// Updated : 2026/03/15
// ============================================================

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include "imu.h"
#include "settings.h"
#include "airdata.h"  // myWire (i2c0, GPIO32/33) を共用する
#include "mysd.h"     // enqueueTask / createLogSdfTask

// ============================================================
// I2C バス（MS5611 と共用: i2c0, GPIO32=SDA, GPIO33=SCL）
// ============================================================
// airdata.cpp で定義された myWire を extern で参照する。
// バスの begin() / setClock() は airdata_setup() で完了済みのため
// imu_setup() では呼ばない。

// ============================================================
// BNO085 ドライバーオブジェクト
// ============================================================
// ライブラリ側のリセット機能は無効(-1)にする。
// begin_I2C() 内部のリセット待ちは ~10ms しかなく BNO085 のブートに不足するため。
// 手動リセットは imu_setup() の先頭で行う（NRST を LOW→HIGH して 400ms 待つ）。
static Adafruit_BNO08x  bno08x(-1);
static sh2_SensorValue_t sv;  // getSensorEvent() の受け取りバッファ

// センサー初期化成功フラグ（imu_setup() 後に確定）
static bool bno085_ok = false;

// ============================================================
// センサー最新値（imu_update() で更新）
// ============================================================

// GAME_ROTATION_VECTOR クォータニオン（加速度計＋ジャイロのみ。磁気干渉なし）
// ロール・ピッチの計算と Kalman predict に使う。ヨーは長期ドリフトあり。
static float _qw = 1.0f, _qx = 0.0f, _qy = 0.0f, _qz = 0.0f;
static bool  _quat_valid = false;  // 少なくとも1回更新されたか

// ROTATION_VECTOR クォータニオン（加速度計＋ジャイロ＋地磁気）
// ヨー（磁北基準）の取得専用。表示のみに使用し Kalman には使わない。
// _rv_accuracy: ヘディング精度推定値 [rad]（BNO085 内部の磁気キャリブレーション品質）
static float _rv_qw = 1.0f, _rv_qx = 0.0f, _rv_qy = 0.0f, _rv_qz = 0.0f;
static float _rv_accuracy = -1.0f;  // -1 = 未受信
static bool  _rv_valid = false;
static bool  _rv_updated = false;  // Euler角ログ用: 新着ROTATION_VECTORフラグ（読み出しでクリア）

// LINEAR_ACCELERATION（重力除去済み、ボディフレーム [m/s²]）
static float _lax = 0.0f, _lay = 0.0f, _laz = 0.0f;
static bool  _linaccel_valid = false;

// ============================================================
// Kalman フィルター 内部状態
// ============================================================

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
static volatile float _imu_az       = 0.0f;  // 地球座標系 鉛直加速度 [m/s²]（デバッグ用）

// 最後に BNO085 からデータを正常受信した時刻 [ms]
// I2C エラー等で途絶えた場合のタイムアウト判定に使う
static unsigned long _last_sensor_event_ms = 0;

// ============================================================
// イベント受信レート計測（1 秒ウィンドウ）
// ============================================================
static uint32_t _grv_cnt  = 0;  // GAME_ROTATION_VECTOR カウンター
static uint32_t _lacc_cnt = 0;  // LINEAR_ACCELERATION カウンター
static uint32_t _rv_cnt   = 0;  // ROTATION_VECTOR カウンター
static float    _grv_hz   = 0.0f;
static float    _lacc_hz  = 0.0f;
static float    _rv_hz    = 0.0f;
static uint32_t _hz_last_ms = 0;


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
// a_k : 地球座標系の鉛直加速度 [m/s²]（重力除去済み）
// dt  : 前回 predict からの経過時間 [s]
//
// 状態遷移: x_pred = F*x + B*a_k
//   F = [[1, dt, 0], [0, 1, -dt], [0, 0, 1]]
//   B = [dt²/2, dt, 0]^T
//
// 共分散伝搬: P_pred = F*P*F^T + Q
//   Q = diag(0, kf_q_vel, kf_q_bias)
static void kf_predict(float a_k, float dt) {
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
    // Step3: Q を加算  (Q = diag([0, q_vel, q_bias]))
    kf_P[1][1] += kf_q_vel;
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
    if (!bno085_ok) return;

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
// imu_setup(): BNO085 の初期化
// ============================================================
void imu_setup() {
    DEBUG_PLN(20260315, "[IMU] imu_setup() start");

    // ---- NRST によるハードウェアリセット ----
    // USB 書き込みや RUN リセットでは RP2350 だけがリセットされ、BNO085 は
    // 前回実行時の SHTP/SH2 状態を保持したままになる。
    // この状態では I2C が不整合のまま begin_I2C() が失敗するため、
    // setup() 冒頭で NRST を手動でアサートして BNO085 自体を確実にリセットする。
    //   LOW 期間: 10ms（最小パルス幅 100µs を大幅に超える）
    //   ブート待機: 400ms（BNO085 データシート推奨値。電源投入後のブート時間と同等）
    //
    // ※ NRST は負論理: LOW でリセットアサート、HIGH で通常動作。
    // ※ setup() で INPUT_PULLUP 設定済みだが、ここで OUTPUT に切り替えてパルスを出す。
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW);   // リセットアサート
    delay(10);                         // 10ms 保持（min 100µs）
    digitalWrite(IMU_RST_PIN, HIGH);  // リセット解除
    delay(400);                        // BNO085 ブート完了を待つ（~400ms）
    DEBUG_PLN(20260315, "[IMU] BNO085 NRST pulse done, waiting boot.");
#endif

    // I2C バスは airdata_wire_begin() で初期化済みのため begin()/setClock() は不要。
    // myWire (i2c0, GPIO32/33, 100kHz) をそのまま使う。

    // BNO085 への接続確認（アドレス応答チェック）
    myWire.beginTransmission(IMU_I2C_ADDR);
    if (myWire.endTransmission() != 0) {
        DEBUGW_PLN(20260315, "[IMU] BNO085 not found on I2C bus. Check wiring/address.");
        bno085_ok = false;
        enqueueTask(createLogSdTask("BNO085 not found"));
        return;
    }

    // Adafruit_BNO08x でセンサーを初期化する。
    // BNO085 は I2C アドレスに ACK を返せても SH2/SHTP 層の Ready まで時間がかかるため、
    // begin_I2C() を最大 5 回リトライする（各リトライ前に 200ms 待機）。
    bool imu_begun = false;
    for (int retry = 0; retry < 5; retry++) {
        if (bno08x.begin_I2C(IMU_I2C_ADDR, &myWire)) {
            imu_begun = true;
            break;
        }
        DEBUGW_P(20260315, "[IMU] begin_I2C retry ");
        DEBUGW_PLN(20260315, retry + 1);
        delay(200);
    }
    if (!imu_begun) {
        DEBUGW_PLN(20260315, "[IMU] BNO085 begin_I2C() FAILED.");
        enqueueTask(createLogSdTask("BNO085 begin_I2C FAILED"));
        bno085_ok = false;
        return;
    }

    // ---- センサーレポートを有効化 ----
    // 30Hz = 33,333 µs 間隔。
    // Core0 の描画ブロック（avg ~64ms）により実測は ~34Hz 程度にとどまるため、
    // 50Hz をリクエストしても余剰なI2Cトラフィックになるだけ。
    // 実測値に合わせて 30Hz に設定し、バス負荷を下げる。
    const uint32_t interval_us = 1000000UL / 30;  // 30Hz → 33333 µs

    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, interval_us)) {
        DEBUGW_PLN(20260315, "[IMU] Failed to enable GAME_ROTATION_VECTOR.");
        enqueueTask(createLogSdTask("BNO085 enableReport GRV failed"));
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, interval_us)) {
        DEBUGW_PLN(20260315, "[IMU] Failed to enable LINEAR_ACCELERATION.");
        enqueueTask(createLogSdTask("BNO085 enableReport LACC failed"));
    }

    // ROTATION_VECTOR (加速度計＋ジャイロ＋地磁気): 磁北基準のヨー取得用。
    // Kalman には使わないため 5Hz で有効化。I2C トラフィックを抑える。
    // ヘディング精度推定値 (_rv_accuracy) が -1 のままならキャリブレーション未完了。
    const uint32_t rv_interval_us = 1000000UL / 5;   // 5Hz → 200000 µs
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, rv_interval_us)) {
        DEBUGW_PLN(20260315, "[IMU] Failed to enable ROTATION_VECTOR.");
        enqueueTask(createLogSdTask("BNO085 enableReport RV failed"));
    }

    bno085_ok = true;
    DEBUGW_PLN(20260315, "[IMU] BNO085 init OK!");
    enqueueTask(createLogSdTask("BNO085 init OK"));
    // センサー組み合わせのログは airdata_setup() の後に GPS_TFT_map.ino で出力する。
}


// ============================================================
// imu_update(): ポーリングでデータ読み出しと Kalman predict を実行
// ============================================================
// loop() から毎回呼ぶ（ノンブロッキング）。
// 15ms 未満の呼び出しは即リターン（67Hz ポーリング → 最大 15ms 遅延）。
// MS5611 と I2C バスを共用するため高頻度呼び出しを避ける（~90µs/回 at 100kHz）。
void imu_update() {
    if (!bno085_ok) return;

    // 15ms ≒ 67Hz でポーリング。BNO085 の出力周期（33ms/30Hz）より短い間隔でチェック。
    static uint32_t _poll_last_us = 0;
    uint32_t _now_us = time_us_32();
    if (_now_us - _poll_last_us < 15000UL) return;  // 15ms 未満なら即リターン
    _poll_last_us = _now_us;

    // データ取り出し:
    // getSensorEvent() は利用可能なレポートを 1 件取り出して sv に格納する。
    // false が返るまで繰り返してキューを空にする。
    // 最大 10 回でガードして無限ループを防ぐ（通常は 2 件 = GRV + LACC で終わる）。
    int read_count = 0;
    while (read_count < 10 && bno08x.getSensorEvent(&sv)) {
        read_count++;
        switch (sv.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                // クォータニオン (qw=real, qx=i, qy=j, qz=k) を保存
                _qw = sv.un.gameRotationVector.real;
                _qx = sv.un.gameRotationVector.i;
                _qy = sv.un.gameRotationVector.j;
                _qz = sv.un.gameRotationVector.k;
                _quat_valid = true;
                _grv_cnt++;
                break;

            case SH2_LINEAR_ACCELERATION:
                // ボディフレームの重力除去済み加速度を保存 [m/s²]
                _lax = sv.un.linearAcceleration.x;
                _lay = sv.un.linearAcceleration.y;
                _laz = sv.un.linearAcceleration.z;
                _linaccel_valid = true;
                _lacc_cnt++;
                break;

            case SH2_ROTATION_VECTOR:
                // 地磁気補正付きクォータニオン（ヨー：磁北基準）
                // accuracy: ヘディング精度推定値 [rad]（0 に近いほど磁気キャリブ良好）
                _rv_qw = sv.un.rotationVector.real;
                _rv_qx = sv.un.rotationVector.i;
                _rv_qy = sv.un.rotationVector.j;
                _rv_qz = sv.un.rotationVector.k;
                _rv_accuracy = sv.un.rotationVector.accuracy;
                _rv_valid = true;
                _rv_cnt++;
                _rv_updated = true;  // Euler角ログ用 新着フラグ
                break;

            default:
                break;
        }
    }
    // データを 1 件以上受信できた場合のみ最終受信時刻を更新する
    if (read_count > 0) _last_sensor_event_ms = millis();

    // ---- 1 秒ごとに各イベントの受信レートを計算 ----
    {
        uint32_t now_ms = (uint32_t)millis();
        uint32_t elapsed = now_ms - _hz_last_ms;
        if (elapsed >= 1000UL) {
            float dt_s = elapsed * 0.001f;
            _grv_hz  = _grv_cnt  / dt_s;
            _lacc_hz = _lacc_cnt / dt_s;
            _rv_hz   = _rv_cnt   / dt_s;
            _grv_cnt = _lacc_cnt = _rv_cnt = 0;
            _hz_last_ms = now_ms;

#ifndef RELEASE
            // 10 秒に 1 回、各センサーの受信レートをシリアル出力
            static uint32_t _hz_print_ms = 0;
            if (now_ms - _hz_print_ms >= 10000UL) {
                _hz_print_ms = now_ms;
                DEBUG_P(20260316,   "[Hz] GRV:");
                DEBUG_PN(20260316,  _grv_hz, 1);
                DEBUG_P(20260316,   " LACC:");
                DEBUG_PN(20260316,  _lacc_hz, 1);
                DEBUG_P(20260316,   " RV:");
                DEBUG_PN(20260316,  _rv_hz, 1);
                DEBUG_P(20260316,   " MS5611:");
                DEBUG_PN(20260316,  get_airdata_win_hz(), 1);
                DEBUG_PLN(20260316, " Hz");
            }
#endif
        }
    }

    // ---- Kalman predict ステップ ----
    // クォータニオンと線形加速度の両方が揃っており、かつ Kalman が初期化済みのときのみ実行。
    // （初期化は最初の気圧高度到着時に imu_kalman_baro_update() が行う）
    if (!_quat_valid || !_linaccel_valid || !kf_initialized) return;

    // 地球座標系の鉛直加速度を計算（BNO085 が重力を除去済みなので g を引く必要なし）
    float a_k = compute_earth_z_accel(_lax, _lay, _laz, _qw, _qx, _qy, _qz);

    // dt を計算（前回 predict からの経過時間 [s]）
    uint32_t now_us = time_us_32();
    float dt = (float)(now_us - kf_last_predict_us) * 1e-6f;
    kf_last_predict_us = now_us;

    // dt の安全チェック: 起動直後・uint32 オーバーフロー・長時間停止を除外
    if (dt <= 0.0f || dt > 0.5f) return;

    // Kalman 予測ステップを実行
    kf_predict(a_k, dt);

    // 出力を更新（display 側から参照される volatile 変数）
    _imu_az      = a_k;
    _imu_altitude = kf_x[0];
    _imu_vspeed   = kf_x[1];
}


// ============================================================
// ゲッター
// ============================================================
bool  get_imu_ok()       { return bno085_ok; }

// クォータニオン (GAME_ROTATION_VECTOR) を返す。未更新時は単位クォータニオン (1,0,0,0)。
void get_imu_quaternion(float &qw, float &qx, float &qy, float &qz) {
    qw = _qw; qx = _qx; qy = _qy; qz = _qz;
}

// 線形加速度（重力除去済み、ボディフレーム [m/s²]）を返す。未更新時は 0。
void get_imu_linaccel(float &ax, float &ay, float &az) {
    ax = _lax; ay = _lay; az = _laz;
}

// Euler 角 [度] を ZYX 規約で返す。
//   roll  : GAME_ROTATION_VECTOR 由来（磁気干渉なし・長期安定）
//   pitch : GAME_ROTATION_VECTOR 由来（同上）
//   yaw   : ROTATION_VECTOR 由来（地磁気補正・磁北基準）。
//           ROTATION_VECTOR 未受信時は GAME_RV で代替（ドリフトあり）。
// ジンバルロック（pitch ±90°付近）は copysignf でクランプして安全に処理。
void get_imu_euler(float &roll, float &pitch, float &yaw) {
    // ---- マウント補正: 軸の入れ替えとオフセット補正 ----
    // BNO085 は IC 直立・コンポーネント面後ろ向きにマウントされているため、
    // センサーの各軸と機体の軸が入れ替わっている。
    //
    // センサー座標系（ZYX オイラーの基底）と機体軸の対応:
    //   センサー X 軸（IC 長辺 = 左右）= 機体ピッチ軸
    //     → sensor_roll（センサー X まわり）= 実ピッチ + 90°
    //   センサー Y 軸（IC 短辺 = 上下）  = 機体ロール軸
    //     → sensor_pitch（センサー Y まわり）= 実ロール
    //   センサー Z 軸（IC 面法線 = 後ろ） = 機体ヨー軸（符号は同一）
    //     → sensor_yaw（センサー Z まわり）= 実ヨー
    //
    // ゆえに表示値への変換:
    //   display_roll  = sensor_pitch
    //   display_pitch = sensor_roll - 90°
    //   display_yaw   = sensor_yaw

    // センサー生値（GAME_RV ベース）
    float sensor_roll = atan2f(2.0f * (_qw * _qx + _qy * _qz),
                               1.0f - 2.0f * (_qx * _qx + _qy * _qy));

    float sinp = 2.0f * (_qw * _qy - _qz * _qx);
    float sensor_pitch = (fabsf(sinp) >= 1.0f)
                         ? copysignf(M_PI * 0.5f, sinp)   // ジンバルロック
                         : asinf(sinp);

    // Yaw: ROTATION_VECTOR（地磁気補正あり）。未受信時は GAME_RV で代替
    const float rw = _rv_valid ? _rv_qw : _qw;
    const float rx = _rv_valid ? _rv_qx : _qx;
    const float ry = _rv_valid ? _rv_qy : _qy;
    const float rz = _rv_valid ? _rv_qz : _qz;
    float sensor_yaw = atan2f(2.0f * (rw * rz + rx * ry),
                              1.0f - 2.0f * (ry * ry + rz * rz));

    // 軸の入れ替えとオフセット補正をしてラジアン → 度
    const float rad2deg = 180.0f / M_PI;
    roll  = sensor_pitch * rad2deg;
    pitch = (sensor_roll - M_PI * 0.5f) * rad2deg;

    // ヨー: センサー Z 軸が後ろ向きのため符号が逆 → 反転して 0〜360° に正規化
    yaw = -sensor_yaw * rad2deg;
    if (yaw < 0.0f)   yaw += 360.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;
}

// ヘディング精度推定値 [度] を返す（ROTATION_VECTOR の accuracy フィールド）。
// 値が小さいほど磁気キャリブレーションが良好。未受信時は -1 を返す。
float get_imu_mag_accuracy_deg() {
    if (!_rv_valid || _rv_accuracy < 0.0f) return -1.0f;
    return _rv_accuracy * (180.0f / M_PI);
}

// Kalman 推定上昇率を返す。
// I2C エラー等で 1 秒以上データが途絶えた場合は 0 を返す（バリオ誤鳴動防止）。
// 1 秒 = 30Hz × 33 サンプル分のタイムアウト。正常時は ~33ms ごとに更新される。
float get_imu_vspeed() {
    if (_last_sensor_event_ms == 0 ||
        millis() - _last_sensor_event_ms > 1000UL) return 0.0f;
    return _imu_vspeed;
}

float get_imu_altitude() { return _imu_altitude; }
float get_imu_az()       { return _imu_az; }

// 各センサーイベントの受信レート [Hz]（1 秒ウィンドウで計測）
float get_imu_grv_hz()  { return _grv_hz; }
float get_imu_lacc_hz() { return _lacc_hz; }
float get_imu_rv_hz()   { return _rv_hz; }

// 新着ROTATION_VECTORフラグ: trueを返し同時にクリア（Euler角ログのトリガー用）
bool get_imu_rv_updated() {
    if (!_rv_updated) return false;
    _rv_updated = false;
    return true;
}

// Kalman パラメーターの動的変更（設定画面からのチューニング用）
void imu_set_kf_params(float q_vel, float q_bias, float R) {
    kf_q_vel  = q_vel;
    kf_q_bias = q_bias;
    kf_R      = R;
}
