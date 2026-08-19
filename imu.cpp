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
// Updated : 2026/07/31
// ============================================================

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include "imu.h"
#include "settings.h"
#include "airdata.h"  // myWire (i2c0, GPIO32/33) を共用する
#include "mysd.h"     // enqueueTask / createLogSdfTask
#include "gps.h"      // replay_has_value / replay_get_* （リプレイ時のセンサ値差し替え）
#include "imulog.h"   // 姿勢 ESKF のオフライン開発用 生データロガー
#include "attitude.h" // 姿勢 ESKF（機上リアルタイム版）

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
static sh2_SensorValue_t sv;  // imu_sensor_handler() が sh2_decodeSensorEvent() で埋める作業バッファ

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
// 最後に LINEAR_ACCELERATION を受信した時刻 [µs]。
// kf_predict() に古いサンプルを繰り返し渡さないための鮮度チェックに使う。
static uint32_t _lacc_last_us = 0;
// 鮮度不足で predict をスキップした回数（診断用。正常時は 0 のまま）
static volatile uint32_t _lacc_stale_skips = 0;

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

// 定義は下（Kalman セクション）にあるが、imu_sensor_handler() から先に使うため前方宣言する。
static float compute_earth_z_accel(float ax, float ay, float az,
                                   float qw, float qx, float qy, float qz);
#endif

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
static volatile float _imu_az           = 0.0f;  // 地球座標系 鉛直加速度 [m/s²]（デバッグ用）
static volatile float _imu_horiz_accel  = 0.0f;  // 地球座標系 水平加速度の大きさ [m/s²]（KF Q_vel 動的増幅・表示用）

// 最後に BNO085 からデータを正常受信した時刻 [ms]
// I2C エラー等で途絶えた場合のタイムアウト判定に使う
static unsigned long _last_sensor_event_ms = 0;

// ============================================================
// イベント受信レート計測（1 秒ウィンドウ）
// ============================================================
static uint32_t _grv_cnt  = 0;  // GAME_ROTATION_VECTOR カウンター
static uint32_t _lacc_cnt = 0;  // LINEAR_ACCELERATION カウンター
static uint32_t _rv_cnt   = 0;  // ROTATION_VECTOR カウンター
static uint32_t _gyro_cnt = 0;  // GYROSCOPE_CALIBRATED カウンター（生ログ用）
static uint32_t _accel_cnt = 0; // ACCELEROMETER カウンター（生ログ用）
static uint32_t _mag_cnt  = 0;  // MAGNETIC_FIELD_CALIBRATED カウンター（生ログ用）
static float    _grv_hz   = 0.0f;
static float    _lacc_hz  = 0.0f;
static float    _rv_hz    = 0.0f;
static volatile float _gyro_hz  = 0.0f;
static volatile float _accel_hz = 0.0f;
static volatile float _mag_hz   = 0.0f;
static uint32_t _hz_last_ms = 0;

// sh2_service() 1 回で処理したレポート数（imu_sensor_handler が加算する）
static int _report_count = 0;

// 生ジャイロ・生加速度の直近値（設定画面の IMU/ESKF ページ表示用）。
// 静止しているデバイスが実際どんな値を出しているかを目視確認するために持つ。
static volatile float _raw_gyro[3]  = {0, 0, 0};   // [rad/s]
static volatile float _raw_accel[3] = {0, 0, 0};   // [m/s²]


// ============================================================
// imu_sensor_handler(): SH2 レポート受信コールバック
// ============================================================
// sh2_service() から、届いたレポート 1 件ごとに呼ばれる。
// imu_setup() で sh2_setSensorCallback() により登録し、
// Adafruit ライブラリ既定のハンドラを上書きする。
//
// なぜ自前のコールバックが必要か:
//   Adafruit の sensorHandler は decode 結果を単一スロット _sensor_value に格納し、
//   getSensorEvent() はそのうち最後の 1 件だけを返す。
//   BNO085 は同時刻にスケジュールされた複数レポートを 1 SHTP パケットにまとめるため、
//   その場合レポートが黙って捨てられる。ここで 1 件ずつ確実に処理する。
static void imu_sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    (void)cookie;
    if (sh2_decodeSensorEvent(&sv, event) != SH2_OK) return;
    _report_count++;

    switch (sv.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            // クォータニオン (qw=real, qx=i, qy=j, qz=k) を保存
            _qw = sv.un.gameRotationVector.real;
            _qx = sv.un.gameRotationVector.i;
            _qy = sv.un.gameRotationVector.j;
            _qz = sv.un.gameRotationVector.k;
            _quat_valid = true;
            _grv_cnt++;
            imulog_push(IMULOG_ID_GAMERV, (uint32_t)sv.timestamp, sv.status, _qw, _qx, _qy, _qz);
            attitude_on_grv(_qw, _qx, _qy, _qz);   // 静止中の平均で ESKF を初期化する
            break;

        case SH2_LINEAR_ACCELERATION:
            // ボディフレームの重力除去済み加速度を保存 [m/s²]
            // ※ BNO085 自身の姿勢推定で重力を引いた値。バリオ KF 用であり、
            //   姿勢 ESKF の入力には使えない（SH2_ACCELEROMETER を使うこと）。
            _lax = sv.un.linearAcceleration.x;
            _lay = sv.un.linearAcceleration.y;
            _laz = sv.un.linearAcceleration.z;
            _linaccel_valid = true;
            _lacc_last_us   = time_us_32();  // 鮮度チェック用（predict の安全網）
            _lacc_cnt++;
            imulog_push(IMULOG_ID_LINACC, (uint32_t)sv.timestamp, sv.status, _lax, _lay, _laz);
            break;

        // ---- 以下は姿勢 ESKF のオフライン開発用。機上の推定には使わない ----
        case SH2_GYROSCOPE_CALIBRATED: {
            _gyro_cnt++;
            imulog_push(IMULOG_ID_GYRO, (uint32_t)sv.timestamp, sv.status,
                        sv.un.gyroscope.x, sv.un.gyroscope.y, sv.un.gyroscope.z);
            // ESKF の伝播はジャイロ到着で回す（加速度は直近値を使う）。
            // ※ sv.timestamp は sh2 のアンダーフローで壊れているので使わず、
            //   受信時刻を渡す。バーストで届く分は attitude.cpp 側が窓平均で吸収する。
            const float g[3] = { sv.un.gyroscope.x, sv.un.gyroscope.y, sv.un.gyroscope.z };
            _raw_gyro[0] = g[0]; _raw_gyro[1] = g[1]; _raw_gyro[2] = g[2];
            attitude_on_gyro(g, time_us_32());
            break;
        }

        case SH2_ACCELEROMETER: {
            // 重力込みの生比力 [m/s²]。ESKF はこれを伝播に使う。
            _accel_cnt++;
            imulog_push(IMULOG_ID_ACCEL, (uint32_t)sv.timestamp, sv.status,
                        sv.un.accelerometer.x, sv.un.accelerometer.y, sv.un.accelerometer.z);
            const float a[3] = { sv.un.accelerometer.x, sv.un.accelerometer.y,
                                 sv.un.accelerometer.z };
            _raw_accel[0] = a[0]; _raw_accel[1] = a[1]; _raw_accel[2] = a[2];
            attitude_on_accel(a);
#if VARIO_USE_RAW_ACCEL
            // ---- バリオ KF 用: 地球座標系の鉛直/水平加速度をここで作って足し込む ----
            // 重力込みの比力を GRV で回し、鉛直成分から重力を引く。
            // BNO085 の LINEAR_ACCELERATION と違い、引く重力が固定値なので
            // 「動作に相関した誤差」が入らない（settings.h の VARIO_USE_RAW_ACCEL 参照）。
            if (_quat_valid) {
                float az_world = compute_earth_z_accel(a[0], a[1], a[2], _qw, _qx, _qy, _qz);
                // 水平成分: 二乗ノルムは回転で不変なので |a|² − a_world_z²。
                // ここでの a_world_z は重力を引く前の値である点に注意。
                float body_sq  = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
                float horiz_sq = body_sq - az_world * az_world;
                _acc_vsum += az_world - GRAVITY_MPS2;
                _acc_hsum += (horiz_sq > 0.0f) ? sqrtf(horiz_sq) : 0.0f;
                if (_acc_cnt < 60000) _acc_cnt++;   // 念のための飽和ガード
                _acc_last_us = time_us_32();
                _accel_valid = true;
            }
#endif
            break;
        }

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            _mag_cnt++;
            imulog_push(IMULOG_ID_MAG, (uint32_t)sv.timestamp, sv.status,
                        sv.un.magneticField.x, sv.un.magneticField.y, sv.un.magneticField.z);
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
            imulog_push(IMULOG_ID_RV, (uint32_t)sv.timestamp, sv.status, _rv_qw, _rv_qx, _rv_qy, _rv_qz);
            // 地磁気補正付きの方位を ESKF の初期ヨーに使う（収束を待たずに絶対方位を持つため）
            attitude_on_rv(_rv_qw, _rv_qx, _rv_qy, _rv_qz, _rv_accuracy);
            break;

        default:
            break;
    }
}


// ============================================================
// imu_enable_reports(): レポート有効化と SH2 コールバック登録
// ============================================================
// imu_setup() と imu_try_recovery() の両方から呼ぶ。
// 復旧時にここを通さないと、begin_I2C() が Adafruit 既定のハンドラを
// 再登録してしまい、レポート取りこぼしのバグが静かに復活する。
//
// レートは settings.h の IMU_RATE_*_HZ で一元管理する。
//
// 【既存レポート（バリオ KF・姿勢表示用）】
//   15Hz。バリオ用途は 10Hz 以上あれば十分。
//   ※ 気圧による観測更新は約 3.7Hz（MS5611 のサンプルは ~40Hz だが、
//     airdata_update() が true を返すのは 250ms のトリム平均ウィンドウ完了時のみ）。
//   ※ このレートを変えるとバリオのチューニングに影響するので触らないこと。
//
// 【生データレポート（姿勢 ESKF のオフライン開発用）】
//   ACCELEROMETER は「重力込みの生比力」であることが重要。
//   LINEAR_ACCELERATION は BNO085 自身の（旋回中に誤る）姿勢推定で重力を除去した値なので、
//   まさに信用できない情報が混入しており ESKF の入力には使えない。
//
// 戻り値: 全レポートの有効化に成功したら true。
static bool imu_enable_reports() {
    struct { sh2_SensorId_t id; uint16_t hz; const char* name; } reports[] = {
        // 既存（バリオ KF・表示用）
        { SH2_GAME_ROTATION_VECTOR,      IMU_RATE_GRV_HZ,   "GRV"   },
        { SH2_LINEAR_ACCELERATION,       IMU_RATE_LACC_HZ,  "LACC"  },
        { SH2_ROTATION_VECTOR,           IMU_RATE_RV_HZ,    "RV"    },
#if VARIO_USE_RAW_ACCEL && !IMULOG_RAW_REPORTS_ENABLED
        // 生比力はバリオ KF が使うので、生ログを止めていても ACCEL だけは必要。
        { SH2_ACCELEROMETER,             IMU_RATE_ACCEL_HZ, "ACCEL" },
#endif
#if IMULOG_RAW_REPORTS_ENABLED
        // 生データ（ESKF 開発用）。settings.h のスイッチで無効化できる。
        // ACCEL は VARIO_USE_RAW_ACCEL=1 のときバリオ KF も使う。
        { SH2_GYROSCOPE_CALIBRATED,      IMU_RATE_GYRO_HZ,  "GYRO"  },
        { SH2_ACCELEROMETER,             IMU_RATE_ACCEL_HZ, "ACCEL" },
        { SH2_MAGNETIC_FIELD_CALIBRATED, IMU_RATE_MAG_HZ,   "MAG"   },
#endif
    };
    bool all_ok = true;
    for (unsigned i = 0; i < sizeof(reports) / sizeof(reports[0]); i++) {
        if (!bno08x.enableReport(reports[i].id, 1000000UL / reports[i].hz)) {
            all_ok = false;
            DEBUGW_P(20260315,   "[IMU] Failed to enable ");
            DEBUGW_PLN(20260315, reports[i].name);
            enqueueTask(createLogSdfTask("BNO085 enableReport %s failed", reports[i].name));
        }
    }

    // ---- 自前の SH2 コールバックを登録（Adafruit 既定のものを上書きする）----
    // begin_I2C() 内で Adafruit の sensorHandler が登録済みなので、必ずその後に呼ぶこと。
    // これにより 1 パケットに複数レポートが載っていても全件処理できる（取りこぼし解消）。
    sh2_setSensorCallback(imu_sensor_handler, NULL);
    return all_ok;
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
// horiz_accel_m_s2 : ボディフレーム水平加速度の大きさ [m/s²]（BNO085なし時は 0）
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
    // BNO085 の有無によらず気圧高度で KF を初期化・更新する。
    // BNO085 なし時は imu_update() の恒速 predict と組み合わせてバリオとして動作。

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
    if (!kf_initialized) return;  // BNO085 の有無によらず動作する（kf_initialized のみチェック）
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
// BNO085 の有無によらず動作する（kf_initialized のみチェック）。
//
//   veld_mps  : GNSS 垂直速度 [m/s]（上昇正）= get_gps_veld_mps()
//   vacc_m    : 垂直位置精度 [m]  — 本関数では使用しない（呼び出し元の互換性維持のため残す）
//   sacc_mps  : 速度精度 [m/s]   — ゲート判定と観測ノイズ R の算出に使用
// ============================================================
void imu_kalman_gnss_vel_update(float veld_mps, float vacc_m, float sacc_mps) {
    (void)vacc_m;  // 使用しない（引数互換性維持）
    if (!kf_initialized) return;

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

    if (!imu_enable_reports()) {
        // 個々の失敗はログに残すが、致命ではないので続行する
    }

    // ---- センサーが自己申告する レンジ / 分解能 を SD に記録する ----
    // 実ログから求めた実効分解能（加速度 0.039 m/s²、ジャイロ 約1.0 deg/s）が
    // センサー本体の上限なのか、設定で上げられるのかを確定させるため。
    //
    // 調査済みの事実:
    //   ・sh2_SensorConfig_t にレンジ／分解能の項目は無い
    //     （設定できるのは reportInterval / batchInterval / wakeup / changeSensitivity のみ）
    //   ・changeSensitivity は Adafruit の enableReport が 0 にしているので
    //     「変化閾値による間引き」でもない
    //   ・FRS レコードは META_* すなわち読み取り専用の仕様記述しかない
    //   → SH2 API 経由では分解能を変更できない。ここではその裏付けを取る。
    {
        struct { sh2_SensorId_t id; const char* name; } q[] = {
            { SH2_ACCELEROMETER,        "ACCEL" },
            { SH2_GYROSCOPE_CALIBRATED, "GYRO"  },
        };
        for (unsigned i = 0; i < sizeof(q) / sizeof(q[0]); i++) {
            sh2_SensorMetadata_t md;
            if (sh2_getMetadata(q[i].id, &md) == SH2_OK) {
                // range / resolution はレポートと同じ固定小数点表現なので qPoint1 で実単位に直す
                const float sc = 1.0f / (float)(1UL << md.qPoint1);
                enqueueTask(createLogSdfTask(
                    "%s meta range=%.4f res=%.5f q=%u minPeriod=%luus",
                    q[i].name, md.range * sc, md.resolution * sc,
                    (unsigned)md.qPoint1, (unsigned long)md.minPeriod_uS));
            } else {
                enqueueTask(createLogSdfTask("%s meta read FAILED", q[i].name));
            }
        }
    }

    bno085_ok = true;
    DEBUGW_PLN(20260315, "[IMU] BNO085 init OK!");
    enqueueTask(createLogSdTask("BNO085 init OK"));
    // センサー組み合わせのログは airdata_setup() の後に GPS_TFT_map.ino で出力する。
}


// ============================================================
// imu_try_recovery(): BNO085 通信途絶時の再起動試行（内部関数）
// ============================================================
// imu_update() から呼ばれる。内部で1分レート制限する。
// NRST を LOW→HIGH してハードウェアリセット後、begin_I2C() で再初期化を試みる。
// 成功すれば bno085_ok=true に戻し、失敗なら次の1分後に再試行する。
// MS5611 と I2C バスを共用しているため Wire のリセットは行わない
//（NRST による BNO085 リセットで SDA が解放されることを期待する）。
// ※ NRST パルス＋ブート待機で Core0 が最大 410ms ブロックする。
//   1分に1度のみ実行のため、画面の一時的な停止は許容する。
// CORE0
static void imu_try_recovery() {
    static unsigned long last_recovery_ms = 0;
    unsigned long now_ms = millis();
    // 1分以内の再試行はスキップ（last_recovery_ms==0 は初回なので即実行）
    if (last_recovery_ms > 0 && now_ms - last_recovery_ms < 60000UL) return;
    last_recovery_ms = now_ms;

    enqueueTask(createLogSdTask("[IMU] BNO085 comm lost, attempting recovery"));
    DEBUGW_PLN(20260325, "[IMU] BNO085 recovery attempt");

    // ---- BNO085 ハードウェアリセット ----
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW);   // リセットアサート（負論理）
    delay(10);
    digitalWrite(IMU_RST_PIN, HIGH);  // リセット解除
    delay(400);                        // BNO085 ブート完了待機（データシート推奨値）
#else
    delay(200);  // RST ピンなし: SH2 リセット完了を待つだけ
#endif

    // ---- 再初期化 ----
    // レポート有効化と SH2 コールバック登録は imu_enable_reports() に集約している。
    // ここで直接 enableReport() を並べてはいけない:
    //   ・settings.h のレート設定と二重管理になる
    //   ・生レポート（ESKF 用）が復旧後に復活しない
    //   ・begin_I2C() が再登録した Adafruit 既定ハンドラを上書きし損ね、
    //     レポート取りこぼしのバグが静かに戻る
    if (bno08x.begin_I2C(IMU_I2C_ADDR, &myWire) && imu_enable_reports()) {
        // 受信時刻をリセット（クォータニオン・加速度も無効化）。
        // millis() をセットすることで、1秒以内にデータが届かなければ再度 timeout → 再試行の
        // サイクルに入れる。0 にすると bno085_ok=true のまま再試行が永遠に発火しなくなる。
        _last_sensor_event_ms = millis();
        _quat_valid     = false;
        _linaccel_valid = false;
#if VARIO_USE_RAW_ACCEL
        _accel_valid = false;
        _acc_vsum = _acc_hsum = 0.0f;
        _acc_cnt  = 0;
#endif
        bno085_ok = true;
        enqueueTask(createLogSdTask("[IMU] BNO085 recovery OK"));
        DEBUGW_PLN(20260325, "[IMU] BNO085 recovery OK");
    } else {
        // 失敗: bno085_ok は false のまま。次の1分後に再試行する。
        enqueueTask(createLogSdTask("[IMU] BNO085 recovery FAILED"));
        DEBUGW_PLN(20260325, "[IMU] BNO085 recovery FAILED");
    }
}


// ============================================================
// imu_update(): ポーリングでデータ読み出しと Kalman predict を実行
// ============================================================
// loop() から毎回呼ぶ（ノンブロッキング）。
// 30ms 未満の呼び出しは即リターン（33Hz ポーリング → 最大 30ms 遅延）。
// MS5611 と I2C バスを共用するため高頻度呼び出しを避ける（i2c0）。
// ※ バスクロックは airdata.cpp:414 の setClock() が実際の設定値。
//   現在 400000 が書かれているが、同ファイルの直上コメントは
//   「100kHz にする／400kHz では error:5 でバスがロックする」と逆のことを述べており、
//   両者は同一コミットで同時に入っている（要確認事項）。
void imu_update() {
    // ---- 通信途絶の自動検出 ----
    // bno085_ok=true でも1秒以上データが届かない場合は途絶と判定し false に落とす。
    // これにより get_imu_alive() が false を返し、update_vario() が MS5611 にフォールバックする。
    if (bno085_ok && _last_sensor_event_ms > 0 &&
        millis() - _last_sensor_event_ms > 1000UL) {
        DEBUGW_PLN(20260325, "[IMU] BNO085 comm timeout, marking unavailable");
        enqueueTask(createLogSdTask("[IMU] BNO085 comm timeout"));
        bno085_ok = false;
    }

    if (!bno085_ok) {
        // 起動後に一度でもデータを受信した（= 途中で途絶えた）場合のみ復旧を試みる。
        // _last_sensor_event_ms==0 は最初から未接続 → 復旧試行しない。
        if (_last_sensor_event_ms > 0) {
            imu_try_recovery();  // 内部で1分レート制限
        }
        // BNO085 なし: KF 初期化済みなら恒速モデル（a_k=0）で共分散を伝播させる。
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

    // ポーリング周期は IMU_POLL_INTERVAL_US（settings.h）。
    //   生レポート有効時: 4ms ≒ 250Hz。高レートのレポートを溜めずに引き取るため。
    //                     このときバリオ KF の predict はこの周期では動かさず、
    //                     関数末尾の IMU_KF_PREDICT_INTERVAL_US ゲートで 30ms に保つ
    //                     （Q の注入量が変わってしまうため）。
    //   無効時          : 30ms ≒ 33Hz。変更前と同一で、ポーリング毎に predict が走る。
    static uint32_t _poll_last_us = 0;
    uint32_t _now_us = time_us_32();
    if (_now_us - _poll_last_us < IMU_POLL_INTERVAL_US) return;
    _poll_last_us = _now_us;

    // ---- データ取り出し ----
    // sh2_service() は届いている SHTP パケットを全部処理し、
    // レポート 1 件ごとに imu_sensor_handler() を呼ぶ。取りこぼしは発生しない。
    //
    // ※ Adafruit の bno08x.getSensorEvent() は使わない。
    //   あちらは sh2_service() の結果を単一スロット _sensor_value に上書きするため、
    //   1 パケットに複数レポートが載っていると最後の 1 件しか返さず、残りを黙って捨てる
    //   （Adafruit_BNO08x.cpp の sensorHandler / getSensorEvent 参照）。
    //   BNO085 は同時刻にスケジュールされたレポートをまとめて 1 パケットで送るため、
    //   要求レートを上げるほどこの取りこぼしが増える。
    //   2026-08-17 に GRV/LACC/RV が 15/15/5Hz 設定に対し 4/5/1Hz まで飢餓になり、
    //   古い加速度を繰り返し積分してバリオが暴れた原因がこれだった。
    _report_count = 0;
    sh2_service();
    int read_count = _report_count;

    // データを 1 件以上受信できた場合のみ最終受信時刻を更新する
    if (read_count > 0) _last_sensor_event_ms = millis();

    // ---- 1 秒ごとに各イベントの受信レートを計算 ----
    {
        uint32_t now_ms = (uint32_t)millis();
        uint32_t elapsed = now_ms - _hz_last_ms;
        if (elapsed >= 1000UL) {
            float dt_s = elapsed * 0.001f;
            _grv_hz   = _grv_cnt   / dt_s;
            _lacc_hz  = _lacc_cnt  / dt_s;
            _rv_hz    = _rv_cnt    / dt_s;
            _gyro_hz  = _gyro_cnt  / dt_s;
            _accel_hz = _accel_cnt / dt_s;
            _mag_hz   = _mag_cnt   / dt_s;
            _grv_cnt = _lacc_cnt = _rv_cnt = 0;
            _gyro_cnt = _accel_cnt = _mag_cnt = 0;
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
    // クォータニオンと加速度の両方が揃っており、かつ Kalman が初期化済みのときのみ実行。
    // （初期化は最初の気圧高度到着時に imu_kalman_baro_update() が行う）
#if VARIO_USE_RAW_ACCEL
    if (!_quat_valid || !_accel_valid || !kf_initialized) {
        // KF 初期化前（最初の気圧高度が来る前）は predict しない。
        // ここで捨てておかないと、初期化されるまで蓄積が伸び続けてしまう。
        _acc_vsum = _acc_hsum = 0.0f;
        _acc_cnt  = 0;
        return;
    }
#else
    if (!_quat_valid || !_linaccel_valid || !kf_initialized) return;
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
    if ((uint32_t)(time_us_32() - _lacc_last_us) > IMU_LACC_MAX_AGE_US) {
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
    float a_k = compute_earth_z_accel(_lax, _lay, _laz, _qw, _qx, _qy, _qz);

    // 地球座標系の水平加速度の大きさ（ワールドフレーム X・Y 成分のノルム）
    // 全加速度の二乗ノルムは回転で不変なので:
    //   horiz² = |a_body|² − a_world_z²  = (lax²+lay²+laz²) − a_k²
    // これにより姿勢変化の影響を受けず、純粋な水平加速度のみを取り出せる。
    float _body_sq   = _lax * _lax + _lay * _lay + _laz * _laz;
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


// ============================================================
// ゲッター
// ============================================================
bool  get_imu_ok()       { return bno085_ok; }

// BNO085 が正常稼働中かどうかを返す（接続済み かつ 直近1秒以内にデータ受信）。
// bno085_ok=true でも通信途絶の場合 false を返す点が get_imu_ok() と異なる。
// ※ imu_update() が1秒タイムアウトで bno085_ok=false に落とすため、
//    このチェックは通常 bno085_ok とほぼ同等になる。
bool get_imu_alive() {
    if (!bno085_ok) return false;
    if (_last_sensor_event_ms == 0) return false;
    return (millis() - _last_sensor_event_ms <= 1000UL);
}

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
    // リプレイ中で CSV に KF_Vspeed 列があれば、その値を返す（バリオ音も再生される）
    if (replay_has_value(RHAVE_KFVS)) return replay_get_kf_vspeed();

    if (bno085_ok) {
        // BNO085 あり: I2C エラー等で 1 秒以上データが途絶えた場合は 0 を返す（誤鳴動防止）
        if (_last_sensor_event_ms == 0 ||
            millis() - _last_sensor_event_ms > 1000UL) return 0.0f;
    } else {
        // BNO085 なし: 気圧 + GNSS 速度による KF 出力を使用。未初期化なら 0 を返す。
        if (!kf_initialized) return 0.0f;
    }
    return _imu_vspeed;
}

float get_imu_altitude()     { return _imu_altitude; }
// KF MSL高度 = KF AGL + gnss_kf_offset（起動地MSL高度の推定値）。
// gnss_kf_offset 未確定時（GNSS 3D fix 前）は AGL 値（＝起動時 0m）を返す。
// リプレイ中で CSV に KF_Altitude 列があれば、その値をそのまま返す。
float get_imu_altitude_msl() {
    if (replay_has_value(RHAVE_KFALT)) return replay_get_kf_altitude();
    return _imu_altitude + _gnss_kf_offset;
}
bool  get_imu_gnss_offset_ready() { return _gnss_offset_ready; }
float get_imu_az()           { return _imu_az; }
float get_imu_horiz_accel()  { return _imu_horiz_accel; }  // 地球座標系 水平加速度 [m/s²]

// 各センサーイベントの受信レート [Hz]（1 秒ウィンドウで計測）
float get_imu_grv_hz()  { return _grv_hz; }
float get_imu_lacc_hz() { return _lacc_hz; }
float get_imu_rv_hz()   { return _rv_hz; }
// 加速度サンプルの鮮度不足で Kalman predict をスキップした累計回数。
// 正常時は 0。増えていれば BNO085 のレポート配信が滞っている（レート要求過大など）。
uint32_t get_imu_lacc_stale_skips() { return _lacc_stale_skips; }
// 生ログ用レポートの受信レート（設定値どおり出ていれば飢餓は起きていない）
float get_imu_gyro_hz()  { return _gyro_hz; }
// 生ジャイロ [rad/s] / 生加速度 [m/s²] の直近値（IMU/ESKF 画面の表示用）
void get_imu_raw_gyro(float g[3])  { for (int i=0;i<3;i++) g[i] = _raw_gyro[i]; }
void get_imu_raw_accel(float a[3]) { for (int i=0;i<3;i++) a[i] = _raw_accel[i]; }
float get_imu_accel_hz() { return _accel_hz; }
float get_imu_mag_hz()   { return _mag_hz; }

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

// Kalman パラメーターのゲッター（SD設定保存用）
float get_imu_kf_q_vel()  { return kf_q_vel; }
float get_imu_kf_q_bias() { return kf_q_bias; }
float get_imu_kf_R()      { return kf_R; }
