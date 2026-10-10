// ============================================================
// File    : imu_bno08x.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : BNO085 (SH-2) ドライバ。**チップ依存のコードはここだけ。**
//           バス初期化・リセット・レポート受信・DCD 校正・復旧と、
//           融合出力（GRV / RV / LACC）由来のゲッターを持つ。
//           推定（姿勢 ESKF・バリオ KF）へは imu_feed_*() だけで渡す。
//
// ■ 0.986 で imu.cpp（1918 行）から切り出した（docs/imu_sch16t_plan.md §1）。
//   **本文は 1 行も書き換えず行範囲ごと移した**（挙動を変えないため）。
//   契約は imu_sensor.h の imu_drv_*。
//
// ★ **ファイル全体が #if IMU_SENSOR_DEFAULT == IMU_SENSOR_BNO085 で囲ってある。**
//   Arduino はスケッチ直下の .cpp を全部コンパイルするので、囲わないと
//   imu_sch16t.cpp を足した日に imu_drv_* が多重定義になる。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/10
// ============================================================

#include <Arduino.h>
#include "settings.h"

#if IMU_SENSOR_DEFAULT == IMU_SENSOR_BNO085

// ★ **山括弧でなく相対パス。**ライブラリは src/bno08x/ に取り込んである
//   （理由と改変点は src/bno08x/PONS_VENDORING.md）。
//   <Adafruit_BNO08x.h> と書くと Arduino が **インストール済みの方**を拾い、
//   ローカル改変が効かないまま静かにビルドが通ってしまう。
#include "src/bno08x/Adafruit_BNO08x.h"

#include "imu.h"
#include "imu_sensor.h"   // チップ境界（能力ビット・feed の口・imu_drv_*）
#include "attitude.h"     // imu_body_euler_rad() / imu_yaw_to_heading_deg()
#include "mysd.h"         // enqueueTask / createLogSdfTask
#include "src/imulog.h"   // 生データロガー（★ チップ側に残す。理由は imu_sensor.h）
#include <EEPROM.h>       // DCD 校正の保存記録（本体フラッシュ。下の ImuCalRec）


// ============================================================
// 注意: BNO085 は MS5611 とはバスを共有していない
// ============================================================
// MS5611 は i2c0(GPIO32/33、airdata.cpp の myWire)。
// BNO085 は i2c1(GPIO34/35) で、下の imuWire を使う。
// v6 までは両方 i2c0 だったが、v7 基板で分離した。

// ============================================================
// BNO085 ホストバス — **i2c1 固定**
// ============================================================
// ★ 0.976 で SPI 対応を削除した。触ると必ずジャイロが化ける問題（触診で約
//   0.8 件/分、gx=+v gy=-v gz=-v の署名）が SPI でしか起きず、I2C では
//   一度も再現しなかったため、**I2C 運用に確定した**。
//   SPI に戻す必要が生じたら 0.975 の imu.cpp / settings.h を見ること。
//
// ★★ **GPIO42 と GPIO44 は I2C バスそのものである。**
//   v7 基板は BNO085 の H_SCL/H_SDA に i2c1(34/35) と旧 SPI1(42/44) の両方を
//   繋いでいる（R46/R47 の 0Ω で連結。実測でショートを確認）。
//   つまり 42/44 を出力にすると I2C バスを殺す。**入力のまま放置してもいけない**:
//   RP2350 のパッドは入力・プル無しで放置すると Low 側へ張り付くことがあり、
//   外部 4.7kΩ のプルアップと分圧して約 2.1V（VIH 2.31V 未満）＝ Low に見える。
//   実機 2026-09-27 に `stage=2(no-SHTP) SDA=0 SCL=0` で 5 回中 4 回起動失敗した。
//   → **必ず INPUT_PULLUP で明示的に「バスのアイドル状態」に合わせて停める。**
//   実体は imu_bus_park_unused_pins()。
static TwoWire imuWire(i2c1, IMU_I2C_SDA, IMU_I2C_SCL);

// ============================================================
// BNO085 ドライバーオブジェクト
// ============================================================
// ライブラリ側のリセット機能は無効(-1)にする。
// begin_*() 内部のリセット待ちは ~10ms しかなく BNO085 のブートに不足するため。
// 手動リセットは imu_reset_start()/imu_reset_poll() で行う（NRST を LOW→HIGH して 400ms 待つ）。
static Adafruit_BNO08x  bno08x(-1);
static sh2_SensorValue_t sv;  // imu_sensor_handler() が sh2_decodeSensorEvent() で埋める作業バッファ

// センサー初期化成功フラグ（imu_drv_setup() 後に確定）
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

// LINEAR_ACCELERATION（重力除去済み、ボディフレーム [m/s²]）
static float _lax = 0.0f, _lay = 0.0f, _laz = 0.0f;
static bool  _linaccel_valid = false;
// 最後に LINEAR_ACCELERATION を受信した時刻 [µs]。
// kf_predict() に古いサンプルを繰り返し渡さないための鮮度チェックに使う。
static uint32_t _lacc_last_us = 0;

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
static float    _grv_hz   = 0.0f;
static float    _lacc_hz  = 0.0f;
static float    _rv_hz    = 0.0f;
static volatile float _gyro_hz  = 0.0f;
static volatile float _accel_hz = 0.0f;
// ★ クォータニオンの健全性カウンタ（V/S が暴れる件の切り分け・2026-09-22）
//   GRV の R/P が -23 → +40 のように飛ぶ症状。バリオは Euler ではなく
//   **クォータニオンを直接**使って重力を抜くので、これが飛ぶと V/S が暴れる。
//   0.944（I2C）では起きず、SPI に移してから出ている。切り分けの要点:
//     bad  … ノルムが 1 から外れた = **パケットが化けている**（転送層の問題。
//            同じ化け方がジャイロ・加速度にも起きているはずで、GRV を
//            廃止しても症状が移るだけ）
//     jump … ノルムは正常なのに姿勢が飛んだ = BNO 内部の融合の問題
//            （加速度外乱で GRV の重力基準が一時的に崩れた等）
//   bad は**採用しない**（_qw.. を更新しない）。jump は数えるだけ。
// ★ BNO085 が申告するセンサー精度（0=信頼できない 1=低 2=中 3=高）。
//   SH-2 の sv.status の下位 2bit。**融合（GRV/RV）の品質はここに強く依存する。**
//   実機（v7 個体）では加速度が 2 で止まり、ジャイロは全件 0 だった。
//   加速度が 2 のとき静止時 |a| が 9.355（-4.6%）、6 面を動かして 3 に上がると
//   約 9.80 へ改善した。**つまりこの値が低い間の加速度は信用できない。**
static uint8_t _acc_accuracy = 0;
static uint8_t _gyr_accuracy = 0;
static uint8_t _cal_cfg      = 0xFF;   // sh2_getCalConfig の結果。0xFF = 未取得

// ============================================================
//  BNO085 の校正（DCD）の保存記録 — **本体フラッシュ（EEPROM 領域）**
// ============================================================
// ★ **SD には置かない。** DCD は基板上の BNO085 の中にあるので、記録の寿命を
//   基板に合わせる。SD に置くと 3 台でカードを入れ替えたときに別基板の日付が出て、
//   「校正済みの機体が NEVER」「未校正の機体が日付付き」という一番まずい嘘になる。
//   （level offset の較正日は SD の設定に入っているが、あれは設定値なので別物）
// ★ 目的は日付そのものではなく **「一度も校正していない」を画面で明示すること**。
//   RTC が無く時刻は GNSS 由来なので、屋内で校正すると日付は手に入らない。
//   その場合も「保存した」ことだけは残し、日付は不明（0）として出す。
#define IMU_CAL_EE_MAGIC   0x504E5343UL   // 'P','N','S','C'
#define IMU_CAL_EE_VERSION 1
#define IMU_CAL_EE_ADDR    0
#define IMU_CAL_EE_SIZE    64             // EEPROM エミュレーションに要求する大きさ

struct ImuCalRec {
    uint32_t magic;
    uint8_t  version;
    uint8_t  saved;        // 1 = 過去に一度以上「手動で」保存した
    uint16_t count;        // 手動保存の回数
    uint32_t date;         // 最後に保存した日 YYYYMMDD。0 = 日付不明
};
static ImuCalRec _cal_rec        = { 0, 0, 0, 0, 0 };
static bool      _cal_autosave_on = false;   // 今回の起動で自動保存を許したか

static void imu_cal_rec_load() {
    EEPROM.begin(IMU_CAL_EE_SIZE);
    ImuCalRec r;
    EEPROM.get(IMU_CAL_EE_ADDR, r);
    if (r.magic == IMU_CAL_EE_MAGIC && r.version == IMU_CAL_EE_VERSION) {
        _cal_rec = r;
        return;
    }
    // 未初期化または版違い。**ここで書き戻さない**（起動時に commit しないため）。
    _cal_rec.magic   = IMU_CAL_EE_MAGIC;
    _cal_rec.version = IMU_CAL_EE_VERSION;
    _cal_rec.saved   = 0;
    _cal_rec.count   = 0;
    _cal_rec.date    = 0;
}

static bool imu_cal_rec_store() {
    _cal_rec.magic   = IMU_CAL_EE_MAGIC;
    _cal_rec.version = IMU_CAL_EE_VERSION;
    EEPROM.put(IMU_CAL_EE_ADDR, _cal_rec);
    // ★★ **commit() は Core1 を止め、割り込みを切ってから 4KB セクタを消す。**
    //   数十 ms 止まるので音が一瞬切れ、その間 BNO085 の読み出しも進まない。
    //   **人が長押ししたときだけ呼ぶこと。起動時は読むだけで commit しない。**
    return EEPROM.commit();
}

// ★ imu_update() が呼ばれない最長時間 [µs]。Core0 が他所で止まると
//   BNO085 のレポートが溜まり、1 パケットが大きくなる（384 バイト上限）。
//   描画・無線・SD のどれが効いているかを、スパイクの頻度と並べて見る。
static uint32_t _poll_gap_max_us = 0;

static uint32_t _grv_bad  = 0;   // ノルム異常で捨てた GRV
static uint32_t _grv_jump = 0;   // ノルムは正常だが 1 サンプルで大きく飛んだ
static uint32_t _rv_bad   = 0;
#define QUAT_NORM_TOL   0.02f    // |q|^2 がこれ以上 1 から外れたら化けとみなす
// 連続サンプル間の |dot| がこれを下回ったら「大きく飛んだ」とみなす。
// **|dot| = cos(θ/2)**（θ は 2 姿勢のなす回転角）なので、0.906 は θ≈50 度。
// 捨てずに数えるだけなので、多少粗くてよい。
#define QUAT_JUMP_COS   0.906f

// クォータニオンが単位ノルムか。化けたパケットはまずここで落ちる。
static inline bool quat_sane(float w, float x, float y, float z) {
    const float n2 = w*w + x*x + y*y + z*z;
    return (n2 > 1.0f - QUAT_NORM_TOL) && (n2 < 1.0f + QUAT_NORM_TOL);
}

static uint32_t _hz_last_ms = 0;

// sh2_service() 1 回で処理したレポート数（imu_sensor_handler が加算する）
static int _report_count = 0;

// ============================================================
//  このチップに何が有るか（imu_sensor.h の能力ビット）
// ============================================================
// ★ **融合出力に触る画面・ログは必ずここを見ること。**
//   SCH16T に差し替えると QUAT / MAGYAW / LINACC / CAL が全部落ちる。
//   「有って当然」と書いた箇所は、そのとき**古い値を出し続ける**か
//   0 を本物として表示する（どちらも静かな誤表示）。
//   MAG は 0.984 で購読をやめたので BNO085 でも立てない。
uint32_t imu_caps() {
    // IMU_CAPS_DEBUG_MASK は既定 0xFFFFFFFF（素通し）。0 にすると融合出力の
    // 無いチップを BNO085 のまま再現できる（settings.h のコメント参照）。
    return (IMU_CAP_QUAT      // GRV（比較表示と quat 健全性カウンタ）
          | IMU_CAP_MAGYAW    // RV（比較表示のヨーとヘディング精度）
          | IMU_CAP_LINACC    // LACC（VARIO 2 の LinAccel 表示）
          | IMU_CAP_CAL)      // DCD と申告精度
         & IMU_CAPS_DEBUG_MASK;
}

const char* imu_sensor_name() { return "BNO085"; }

// ============================================================
// imu_sensor_handler(): SH2 レポート受信コールバック
// ============================================================
// sh2_service() から、届いたレポート 1 件ごとに呼ばれる。
// imu_drv_setup() で sh2_setSensorCallback() により登録し、
// Adafruit ライブラリ既定のハンドラを上書きする。
//
// なぜ自前のコールバックが必要か:
//   Adafruit の sensorHandler は decode 結果を単一スロット _sensor_value に格納し、
//   getSensorEvent() はそのうち最後の 1 件だけを返す。
//   BNO085 は同時刻にスケジュールされた複数レポートを 1 SHTP パケットにまとめるため、
//   その場合レポートが黙って捨てられる。ここで 1 件ずつ確実に処理する。
//
// ★ **ここは BNO085 固有（チップ依存）。** 推定へ渡すのは imu_feed_*() 経由だけで、
//   このファイルを imu_bno08x.cpp へ割るときはこの関数ごと動かす（imu_sensor.h）。
// ============================================================
static void imu_sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    (void)cookie;
    if (sh2_decodeSensorEvent(&sv, event) != SH2_OK) return;
    _report_count++;

    switch (sv.sensorId) {
        case SH2_GAME_ROTATION_VECTOR: {
            // クォータニオン (qw=real, qx=i, qy=j, qz=k)
            const float nw = sv.un.gameRotationVector.real;
            const float nx = sv.un.gameRotationVector.i;
            const float ny = sv.un.gameRotationVector.j;
            const float nz = sv.un.gameRotationVector.k;

            // ★ 化けたパケットを採用しない。バリオはこのクォータニオンで
            //   重力を抜くので、1 サンプルの異常がそのまま V/S の尖りになる。
            if (!quat_sane(nw, nx, ny, nz)) { _grv_bad++; break; }

            // ノルムは正常でも姿勢が大きく飛ぶことがある。**捨てずに数えるだけ**に
            //   する（速い手振りでも起こりうるので、捨てると本物を落とす）。
            //   |dot| は 2 つの姿勢のなす角の cos(θ/2)。1 から離れるほど大きく回った。
            if (_quat_valid) {
                float d = _qw*nw + _qx*nx + _qy*ny + _qz*nz;
                if (d < 0.0f) d = -d;            // q と -q は同じ姿勢
                if (d < QUAT_JUMP_COS) _grv_jump++;
            }

            _qw = nw; _qx = nx; _qy = ny; _qz = nz;
            _quat_valid = true;
            _grv_cnt++;
            imulog_push(IMULOG_ID_GAMERV, (uint32_t)sv.timestamp, sv.status, _qw, _qx, _qy, _qz);
            // ★ 0.983 で attitude_on_grv() の呼び出しをやめた。ESKF は静止中の
            //   平均加速度から水平化する（GRV と実測 0.05 度以内で一致）。
            //   GRV 自体は「BNO085 との比較表示」と quat 健全性カウンタに残している。
            break;
        }

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
            _gyr_accuracy = (uint8_t)(sv.status & 0x03);
            imulog_push(IMULOG_ID_GYRO, (uint32_t)sv.timestamp, sv.status,
                        sv.un.gyroscope.x, sv.un.gyroscope.y, sv.un.gyroscope.z);
            // ESKF の伝播はジャイロ到着で回す（加速度は直近値を使う）。
            // ※ sv.timestamp は sh2 のアンダーフローで壊れているので使わず、
            //   受信時刻を渡す。バーストで届く分は attitude.cpp 側が窓平均で吸収する。
            const float g[3] = { sv.un.gyroscope.x, sv.un.gyroscope.y, sv.un.gyroscope.z };
            imu_feed_gyro(g, time_us_32());
            break;
        }

        case SH2_ACCELEROMETER: {
            // 重力込みの生比力 [m/s²]。ESKF はこれを伝播に使う。
            _accel_cnt++;
            _acc_accuracy = (uint8_t)(sv.status & 0x03);
            imulog_push(IMULOG_ID_ACCEL, (uint32_t)sv.timestamp, sv.status,
                        sv.un.accelerometer.x, sv.un.accelerometer.y, sv.un.accelerometer.z);
            const float a[3] = { sv.un.accelerometer.x, sv.un.accelerometer.y,
                                 sv.un.accelerometer.z };
            imu_feed_accel(a);
            break;
        }

        case SH2_ROTATION_VECTOR:
            // 地磁気補正付きクォータニオン（ヨー：磁北基準）
            // accuracy: ヘディング精度推定値 [rad]（0 に近いほど磁気キャリブ良好）
            if (!quat_sane(sv.un.rotationVector.real, sv.un.rotationVector.i,
                           sv.un.rotationVector.j, sv.un.rotationVector.k)) {
                _rv_bad++;
                break;                          // 化けた RV も採用しない
            }
            _rv_qw = sv.un.rotationVector.real;
            _rv_qx = sv.un.rotationVector.i;
            _rv_qy = sv.un.rotationVector.j;
            _rv_qz = sv.un.rotationVector.k;
            _rv_accuracy = sv.un.rotationVector.accuracy;
            _rv_valid = true;
            _rv_cnt++;
            imulog_push(IMULOG_ID_RV, (uint32_t)sv.timestamp, sv.status, _rv_qw, _rv_qx, _rv_qy, _rv_qz);
            // 地磁気補正付きの方位を ESKF の初期ヨーに使う（収束を待たずに絶対方位を持つため）
            // ★ 0.983 で attitude_on_rv() の呼び出しをやめた。初期ヨーは GNSS 航跡から
            //   入れる。地磁気ヨーは実測で GNSS 航跡から 57.8 度ずれていた
            //   （settings.h の ESKF_YAW_INIT_MIN_SPEED_MPS のコメント）。
            //   RV は方位の比較表示（get_imu_euler のヨー）にだけ残している。
            break;

        default:
            break;
    }
}

// ============================================================
// imu_enable_reports(): レポート有効化と SH2 コールバック登録
// ============================================================
// imu_drv_setup() と imu_try_recovery() の両方から呼ぶ。
// 復旧時にここを通さないと、begin_*() が Adafruit 既定のハンドラを
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
#endif
    };
    bool all_ok = true;
    // ★ **総時間に上限を設ける。** enableReport() 1 回ごとに
    //   sh2_setSensorConfig() が走り、デバイスが応答しないと SH-2 側の待ちで
    //   **1 回あたり数百 ms** かかる。レポートは 5 件あるので最悪で数秒
    //   （0.984 で MAG を外して 6 → 5 件）。
    //   Core0 が止まる。
    //   リセットを 410ms の delay からステートマシンに変えたのは、その間
    //   GNSS FIFO が溢れるからで、ここが素通しでは意味が無かった。
    //   正常時は 1 件あたり数 ms なので、この上限に当たることは無い。
    //   途中で打ち切っても false を返すので、復旧は 1 分後に再試行される。
    const unsigned long enable_t0 = millis();
    for (unsigned i = 0; i < sizeof(reports) / sizeof(reports[0]); i++) {
        if (millis() - enable_t0 > IMU_ENABLE_BUDGET_MS) {
            all_ok = false;
            DEBUGW_PLN(20260917, "[IMU] enableReport budget exceeded, aborting");
            enqueueTask(createLogSdfTask("BNO085 enableReport aborted at %s (slow bus)",
                                         reports[i].name));
            break;
        }
        if (!bno08x.enableReport(reports[i].id, 1000000UL / reports[i].hz)) {
            all_ok = false;
            DEBUGW_P(20260315,   "[IMU] Failed to enable ");
            DEBUGW_PLN(20260315, reports[i].name);
            enqueueTask(createLogSdfTask("BNO085 enableReport %s failed", reports[i].name));
        }
    }

    // ---- 自前の SH2 コールバックを登録（Adafruit 既定のものを上書きする）----
    // begin_*() 内で Adafruit の sensorHandler が登録済みなので、必ずその後に呼ぶこと。
    // これにより 1 パケットに複数レポートが載っていても全件処理できる（取りこぼし解消）。
    sh2_setSensorCallback(imu_sensor_handler, NULL);

    // ---- 動的校正の設定を読む → 足りない分だけ書く ----
    // ★ 0.973 までは「読むだけ」だった。いまは下で sh2_setCalConfig() を呼び、
    //   さらに下で sh2_setDcdAutoSave() の可否を決める。
    //   SH2_CAL_ACCEL=0x01 / GYRO=0x02 / MAG=0x04 / PLANAR=0x08。
    {
        uint8_t cfg = 0;
        const int rc = sh2_getCalConfig(&cfg);
        if (rc == SH2_OK) {
            _cal_cfg = cfg;
            enqueueTask(createLogSdfTask(
                "BNO085 calcfg=0x%02X (accel=%d gyro=%d mag=%d planar=%d)",
                cfg, (cfg & 0x01) ? 1 : 0, (cfg & 0x02) ? 1 : 0,
                (cfg & 0x04) ? 1 : 0, (cfg & 0x08) ? 1 : 0));
        } else {
            enqueueTask(createLogSdfTask("BNO085 calcfg read FAILED rc=%d", rc));
        }

        // ---- ★ ジャイロの動的校正を有効にする ----
        //   実機（v7）の既定は 0x05 ＝ accel と mag だけで、**gyro(0x02) が無効**。
        //   そのため申告精度が全レポートで 0 のまま（実測: 3 本のログすべてで 0）。
        //   GRV は**加速度とジャイロの融合**なので、ジャイロのゼロ点(ZRO)が
        //   工場出荷値のまま温度ドリフトすると、融合が周期的に重力で引き戻される。
        //   実機で「静止しているのに GRV が 96 度飛ぶ」現象の機構として筋が通る。
        //   ※ これで飛びが直るかは未確認。次のログで grvjump を見て判断する。
        // ★ SH2_CAL_MAG は 0.984 以降も残す。**地磁気レポートの購読はやめたが、
        //   チップ内の RV（比較表示用）は内部で地磁気を使う**ので、校正を切ると
        //   その比較値が劣化する。購読（レポート）と校正（チップ内部）は別物。
        const uint8_t want = (uint8_t)(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
        if (_cal_cfg != 0xFF && _cal_cfg != want) {
            const int rc2 = sh2_setCalConfig(want);
            uint8_t back = 0;
            if (rc2 == SH2_OK && sh2_getCalConfig(&back) == SH2_OK) _cal_cfg = back;
            enqueueTask(createLogSdfTask("BNO085 calcfg set 0x%02X -> 0x%02X (rc=%d)",
                                         want, _cal_cfg, rc2));
        }
    }

    // ---- DCD 自動保存の可否を決める ----
    // ★ **ここ（imu_enable_reports）に置くのは意図的。** この関数は imu_drv_setup() と
    //   imu_try_recovery() の両方から呼ばれる。チップをリセットすると自動保存の
    //   設定は既定へ戻るので、**復旧のたびに再適用しないと効かなくなる。**
    //   判定そのものはフラッシュの記録を読むだけなので、何度通っても同じ結果。
    // ★★ **一度も手動保存していない機体だけ、チップ自身の自動保存を許す。**
    //   未校正のまま飛ぶのを防ぐ保険。一度でも手動で保存したなら、その校正を
    //   黙って上書きされたくないので自動保存は切る。
    //   こうしておくと画面の「CAL SAVED / NEVER」が**チップの中身と一致する**。
    //   （自動保存を常時オンにすると、チップは良い校正を持っているのに
    //     画面は NEVER と出る、という嘘になる）
    {
        imu_cal_rec_load();
        _cal_autosave_on = !imu_cal_saved_ever();
        const int rc = sh2_setDcdAutoSave(_cal_autosave_on);
        enqueueTask(createLogSdfTask(
            "BNO085 dcdAutoSave=%d rc=%d (savedEver=%d date=%lu cnt=%u)",
            (int)_cal_autosave_on, rc, (int)imu_cal_saved_ever(),
            (unsigned long)_cal_rec.date, _cal_rec.count));
    }
    return all_ok;
}

// ============================================================
// BNO085 ホストバスの共通処理（SPI / I2C 両対応）
// ============================================================
// imu_drv_setup() と imu_try_recovery() の両方から使う。
// リセット手順とバス初期化を 1 箇所に集約して、片方だけ直し忘れるのを防ぐ。

// プロトコル選択ピン（PS0）を、これから使うバスに合わせて駆動する。
// バスに繋がっているのに使わないピンを、**明示的に停める**。
//
// ★★ **入力のまま放置してはいけない。** GPIO42/44 は R46/R47 の 0Ω で
//   H_SCL/H_SDA に連結されており、**i2c1 と同じ線**である（実測でショート確認）。
//   RP2350 のパッドは入力・プル無しで放置すると Low 側へ張り付くことがあり、
//   外部 4.7kΩ のプルアップと分圧して約 2.1V（VIH 2.31V 未満）＝ Low に見える。
//   実機 2026-09-27: `BNO085 begin FAILED (i2c1) stage=2(no-SHTP) SDA=0 SCL=0` で
//   5 回中 4 回起動に失敗した（アドレス ACK は返るのに SHTP が確立しない）。
//   INPUT_PULLUP にしておけばバスのアイドル状態（High）と一致し、悪さをしない。
// ★ **出力にはしないこと。** 出力にすると I2C バスを殺す。
static void imu_bus_park_unused_pins() {
    pinMode(IMU_UNUSED_SCK_PIN,  INPUT_PULLUP);   // GPIO42 = H_SCL 側
    pinMode(IMU_UNUSED_MISO_PIN, INPUT_PULLUP);   // GPIO44 = H_SDA 側
}

// データシート Figure 1-5:  PS1=0,PS0=0 → I2C（PS1 は半田ジャンパ JP1 = GND）。
// ソフトが触るのは PS0 だけ。
static void imu_bus_select_protocol() {
    pinMode(IMU_PS0_WAKE_PIN, OUTPUT);
    digitalWrite(IMU_PS0_WAKE_PIN, LOW);    // PS0=0 → I2C
    // I2C ではこのピンが SA0（アドレス下位ビット）になる。HIGH で 0x4B。
    pinMode(IMU_I2C_SA0_PIN, OUTPUT);
    digitalWrite(IMU_I2C_SA0_PIN, (IMU_I2C_ADDR & 1) ? HIGH : LOW);
    // ★★ **GPIO41 には絶対に触らないこと。** あれは E220 の M0/M1 専用。
    //   R53 を外してあるので **BNO085 の H_CSN とは繋がっていない**（H_CSN は
    //   基板上で未接続。I2C では don't care なのでそれでよい）。
    //   0.975 まで「H_CSN を浮かせない」ために HIGH 駆動していたが、
    //   **H_CSN には届いておらず、E220 を mode 3（Config/DeepSleep）へ
    //   叩き落とすだけだった**。この関数は IMU の復旧からも呼ばれるので、
    //   **飛行中に無線が黙る**。戻さないこと。
    // ★ H_INTN は I2C では読まない（完全ポーリング）が、入力プルアップは入れておく。
    //   ログの `attempting recovery (INT=..)` が意味を持つ値になり、
    //   BNO085 がブートする前の浮きも無くなる。
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    imu_bus_park_unused_pins();
}

// NRST パルスで BNO085 を確実にリセットする（**ステートマシン**）。
// USB 書き込みや RUN リセットでは RP2350 だけがリセットされ、BNO085 は前回実行時の
// SHTP/SH2 状態を保持したままになる。この状態では begin_*() が失敗するため必ず通す。
//   LOW 期間 : 10ms（最小パルス幅 100µs を大幅に超える）
//   ブート待機: 400ms（データシート推奨値）
//
// ★ **ここを delay() で書いてはいけない。**
//   以前は delay(10)+delay(400) を直に並べていたため、飛行中の復旧試行のたびに
//   Core0 が 410ms 止まっていた。地図・ボタン・バリオが同時に止まり、
//   GNSS の受信も巻き添えになる（受信バッファの余裕は GNSS_SERIAL_FIFO_SIZE 参照）。
//   しかも BNO085 が死んでいる限り復旧は 1 分ごとに走り続けるので、症状が繰り返す。
//   経過時間で進める形にして、待っている間 Core0 を返せるようにした。
//
// ※ プロトコル選択ピンはリセット時にラッチされる。
//   imu_reset_start() が先頭で imu_bus_select_protocol() を呼ぶので、
//   呼び出し側で気にする必要は無い（忘れると I2C でラッチされて復帰不能になる）。
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
  #define IMU_RST_LOW_MS   10    // NRST を LOW に保持する時間
  #define IMU_RST_BOOT_MS  400   // 解除してからブート完了までの待ち
#else
  #define IMU_RST_LOW_MS   0     // NRST が繋がっていないので、ピンは触らず待つだけ
  #define IMU_RST_BOOT_MS  200
#endif

typedef enum {
    IMU_RST_IDLE = 0,   // 進行中のリセットは無い
    IMU_RST_LOW,        // NRST を LOW にした。パルス幅を待っている
    IMU_RST_BOOT,       // NRST を HIGH に戻した。ブート完了を待っている
} ImuResetPhase;

static ImuResetPhase imu_rst_phase = IMU_RST_IDLE;
static unsigned long imu_rst_ms    = 0;

// リセットを開始する（ブロックしない）。
static void imu_reset_start() {
    // ★ **プロトコル選択はここで必ずやり直す。**
    //   PS1/PS0 はリセット時にラッチされる。SPI 構成では初期化後に PS0 を WAKE として
    //   LOW に落としている（imu_bus_begin() 参照）ので、そのままリセットを掛けると
    //   **PS0=0 = I2C** がラッチされ、BNO085 が二度と SPI で喋らなくなる。
    //   呼び出し側の順序に頼ると必ず忘れるので、リセット手順の中に取り込む。
    imu_bus_select_protocol();
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW);   // リセットアサート（負論理）
#endif
    imu_rst_phase = IMU_RST_LOW;
    imu_rst_ms    = millis();
}

// リセットを進める。完了したら true を返し、IDLE へ戻る。ブロックしない。
static bool imu_reset_poll() {
    const unsigned long now = millis();
    switch (imu_rst_phase) {
        case IMU_RST_LOW:
            if (now - imu_rst_ms < IMU_RST_LOW_MS) return false;
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
            digitalWrite(IMU_RST_PIN, HIGH);      // リセット解除
#endif
            imu_rst_phase = IMU_RST_BOOT;
            imu_rst_ms    = now;
            return false;
        case IMU_RST_BOOT:
            if (now - imu_rst_ms < IMU_RST_BOOT_MS) return false;
            imu_rst_phase = IMU_RST_IDLE;
            return true;
        default:
            return true;                          // 開始していない＝待つものが無い
    }
}

// 起動時（imu_setup）専用。ここで止まるのは構わないので待ち切る。
// タイミングの実体は上のステートマシン 1 つだけにして、
// 「起動時と復旧時で片方だけ直し忘れる」を構造的に防ぐ。
static void imu_reset_blocking() {
    imu_reset_start();
    while (!imu_reset_poll()) delay(1);
}

// 失敗段階の名前。
static const char* imu_fail_stage_name(uint8_t stage) {
    return (stage == 1) ? "no-ACK" : "no-SHTP";
}

// imu_bus_begin() がどこで失敗したかを残す（実機立ち上げの切り分け用）。
//   1 = アドレス 0x4B が ACK を返さなかった（Adafruit_I2CDevice::detected()）
//       → BNO085 がブートしていない / I2C モードで起動していない。
//         JP1(PS1)=GND・R46/R47・3V3・NRST・半田を疑う。**波形以前の問題**。
//   2 = ACK は返るのに SHTP で喋れなかった（sh2_open / getProdIds が失敗）
//       → チップは生きて I2C モードにいる。プルアップ・クロック・ノイズを疑う。
//         `init_step=2` なら sh2_open、`3` なら getProdIds（詳細は失敗ログ）。
// この 2 つは対処がまったく違うので、log.txt だけで分かるようにしておく。
static uint8_t imu_begin_fail_stage = 0;

// バスを初期化して Adafruit_BNO08x を開始する。成功で true。
// リトライは呼び出し側が行う。
// sh2_open() が成功したままか。**sh2_close() を呼ぶ判断にだけ使う。**
static bool sh2_opened = false;

static bool imu_bus_begin() {
    // ★★ **再オープンの前に必ず sh2_close() を呼ぶ。呼ばないと 2 回目は必ず失敗する。**
    //   src/bno08x/shtp.c の SHTP インスタンスプールは MAX_INSTANCES=1 で、
    //   空きの判定は instances[n].pHal == 0。これを 0 に戻すのは shtp_close() だけ。
    //   ところが Adafruit_BNO08x::_init() は sh2_open() を呼ぶだけで close しない。
    //   その結果、2 回目の begin_SPI() は getInstance() が 0 を返し、
    //   shtp_open() → sh2_open() が SH2_ERR になって _init() が false を返す。
    //   **これが「BNO085 recovery FAILED」の正体**（実機 2026-09-23）。
    //   INT は来ていた＝チップは生きていたのに、再初期化だけが構造的に失敗していた。
    //   つまり **復旧は一度も成功したことがなかった。**
    //   i2chal_close() は実質何もしないので、相手が無応答でも安全。
    if (sh2_opened) {
        sh2_close();
        sh2_opened = false;
    }
    // ★★ **ここで自前のアドレス ACK 確認をしてはいけない。**
    //   0.976 の途中まで `imuWire.beginTransmission()/endTransmission()` で
    //   先に叩いていたが、**それが起動失敗の原因だった**（実機 2026-09-27、
    //   5 回中 4〜5 回 `init_step=1`）。理由:
    //     ・arduino-pico の 0 バイト書き込みは I2C ペリフェラルを使わず、
    //       ピンを SIO に切り替えて **ビットバンギングで叩く**（Wire.cpp の _probe）。
    //       ビット幅は (1000000 / _clkHz) / 2 なので 400kHz では **1µs** しかない。
    //     ・begin_I2C() → Adafruit_I2CDevice::begin() → detected() が
    //       **まったく同じ _probe() をもう一度**叩く（間の _wire->begin() は
    //       `if (_running) return;` で何もしない）。
    //     ・結果、**同じ探査が背中合わせで 2 回**走り、**1 回目は通って 2 回目が落ちる**
    //       という形で毎回失敗していた。こちらの確認は完全に冗長だった。
    //   居ないチップに対してライブラリを呼ばない保護は、**detected() 自身が果たす**
    //   （_init() へ進む前に false を返すので sh2_getProdIds() には到達しない）。
    const bool i2c_ok = bno08x.begin_I2C(IMU_I2C_ADDR, &imuWire);
    sh2_opened = i2c_ok || (pons_init_fail_step >= 3);
    if (!i2c_ok) {
        // 段階は pons_init_fail_step から決める（1 = アドレスが応答しない）。
        imu_begin_fail_stage = (pons_init_fail_step == 1) ? 1 : 2;
        DEBUGW_PLN(20260901, "[IMU] BNO085 begin_I2C failed");
    }
    return i2c_ok;
}

// バスの名前（ログ用）
static const char* imu_bus_name() { return "i2c1"; }

// I2C バスのロックアップを解く。**リトライの前に必ず通すこと。**
//
// ★★ **NRST を打つだけでは足りない。** NRST は BNO085 を初期化するが、
//   **RP2350 側の I2C ブロックは初期化されない。** 転送の途中で中断すると
//   RP2350 は SCL を Low に握ったまま止まり、スレーブも次のクロックを待って
//   SDA を握ったままになる。この状態は電源を切るまで解けず、
//   **3 回のリトライが全部同じ理由で失敗する**（実機 2026-09-27、
//   ログの `stage=2(no-SHTP) SDA=0 SCL=0` がこれ。両線が能動的に Low）。
//
// 手順は I2C の標準的な復旧手順:
//   1. ペリフェラルを外してピンを GPIO に戻す（SCL/SDA を解放する）
//   2. SCL を 9 回叩いてスレーブに残りのビットを吐き出させる
//   3. STOP を作って解放する
//   4. ペリフェラルを張り直す
static void imu_i2c_bus_recover() {
    imuWire.end();                       // ピンを SIO へ戻す（ペリフェラルを解放）

    pinMode(IMU_I2C_SCL, OUTPUT);
    pinMode(IMU_I2C_SDA, INPUT_PULLUP);  // SDA はスレーブに任せて観測する
    for (int i = 0; i < 9; i++) {        // 1 バイト分 + ACK のクロックを供給
        digitalWrite(IMU_I2C_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(IMU_I2C_SCL, HIGH); delayMicroseconds(5);
    }
    // STOP 条件（SCL が High のあいだに SDA を Low → High）
    pinMode(IMU_I2C_SDA, OUTPUT);
    digitalWrite(IMU_I2C_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(IMU_I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(IMU_I2C_SDA, HIGH); delayMicroseconds(5);
    pinMode(IMU_I2C_SDA, INPUT_PULLUP);
    pinMode(IMU_I2C_SCL, INPUT_PULLUP);

    imu_bus_park_unused_pins();           // 42/44 を停め直す（end() で SIO に戻るため）
    imuWire.begin();
    imuWire.setClock(IMU_I2C_HZ);
    enqueueTask(createLogSdfTask("BNO085 i2c bus recover -> SDA=%d SCL=%d",
                                 (int)digitalRead(IMU_I2C_SDA),
                                 (int)digitalRead(IMU_I2C_SCL)));
}

// ============================================================
// imu_drv_setup(): BNO085 の初期化
// ============================================================
// ★ 0.986 の分割前は公開 API の imu_setup() そのものだった。**中身は当時のまま。**
//   公開 API 側（imu.cpp の imu_setup()）はこれを呼ぶだけ。
void imu_drv_setup() {
    DEBUG_PLN(20260315, "[IMU] imu_setup() start");
    DEBUGW_P(20260901, "[IMU] host bus = "); DEBUGW_PLN(20260901, imu_bus_name());

    // ---- プロトコル選択 → ハードウェアリセット ----
    // PS1/PS0 はリセット時にラッチされるので、必ずこの順で行う。
    // I2C バス（i2c1）は BNO085 専用。MS5611 の i2c0 とは別系統なので独立に初期化する。
    imu_bus_select_protocol();
    imuWire.begin();
    imuWire.setClock(IMU_I2C_HZ);
    imu_reset_blocking();
    DEBUG_PLN(20260315, "[IMU] BNO085 NRST pulse done, waiting boot.");

    // Adafruit_BNO08x でセンサーを初期化する。
    // ★ **リトライのたびに NRST を打ち直し、その前にバスを解くこと。**
    //   sh2_open() はリセットを送らず、ブート直後に積まれた advertisement を
    //   待つだけ（bno08x(-1) なのでライブラリも NRST を触らない）。1 回目が
    //   advertisement を消費してしまうと、**単に待ち直すだけのリトライは必ず失敗する**
    //   （以前は delay(200) だけだったので実質 1 回きりだった）。
    //   リセットを挟めば advertisement が積み直されるので、2 回目以降も意味を持つ。
    //   1 回のリセットが 410ms かかるぶん、回数は 5 → 3 に減らす
    //   （不在時の起動待ちは 5×(300+200)=2.5 秒 → 3×300+2×410=1.7 秒で短くなる）。
    bool imu_begun = false;
    for (int retry = 0; retry < 3; retry++) {
        if (retry > 0) {
            DEBUGW_P(20260315, "[IMU] begin retry ");
            DEBUGW_PLN(20260315, retry);
            // NRST パルス。中で imu_bus_select_protocol() が走るので、
            // 直前に WAKE として LOW にした PS0 が I2C としてラッチされることは無い。
            // ★ **NRST より先にバスを解く。** 握られたままだとリセット後の
            //   最初の転送も同じ理由で失敗し、リトライが無意味になる。
            imu_i2c_bus_recover();
            imu_reset_blocking();   // advertisement を積み直させる
        }
        if (imu_bus_begin()) {
            imu_begun = true;
            break;
        }
    }
    if (!imu_begun) {
        DEBUGW_PLN(20260315, "[IMU] BNO085 begin() FAILED.");
        // ★ **どこで失敗したかとピンの実測値まで残す。**
        //   ここが実機立ち上げで最初に見る 1 行になる。"FAILED" だけだと
        //   「チップが居ない」のか「バスが死んでいる」のかが分からず、
        //   当てずっぽうで基板を触ることになる。
        // ★ init_step まで出す。2 = sh2_open が失敗（SHTP の確立そのもの）、
        //   3 = getProdIds が失敗（SHTP は開いたが応答が取れない）。
        //   ここが分からないと「プロトコルの問題」と「バスの問題」を分けられない。
        enqueueTask(createLogSdfTask(
            "BNO085 begin FAILED (%s) stage=%u(%s) init_step=%u sh2=%ld SDA=%d SCL=%d addr=0x%02X",
            imu_bus_name(), imu_begin_fail_stage,
            imu_fail_stage_name(imu_begin_fail_stage),
            pons_init_fail_step, (long)pons_init_fail_status,
            (int)digitalRead(IMU_I2C_SDA), (int)digitalRead(IMU_I2C_SCL),
            IMU_I2C_ADDR));
        bno085_ok = false;
        return;
    }
    if (!imu_enable_reports()) {
        // 個々の失敗はログに残すが、致命ではないので続行する
    }

    // ---- センサーが自己申告する レンジ / 分解能 を SD に記録する ----
    // ★ **RELEASE では実行しない。ハングし得るため。**
    //   sh2_getMetadata() は getFrsOp を使う。getFrsStart() は opCompleted() を
    //   呼ばず応答を待つ op で、sh2.c の sh2_Op_t は 17 個すべて timeout_us を
    //   設定していない（= 0 = 無期限）。FRS 応答が 1 回失われると opProcess() が
    //   永久ループに入り（1 周ごとに 500ms の INT 待ち）、**setup() が終わらない
    //   ＝ 画面が真っ暗のまま起動しない**。
    //   INT の生存確認は begin_SPI() の手前までしか守らないので、ここは無防備だった。
    //   配線が新しく信号品質が不安定なときに最も踏みやすい。
    //   これは「分解能を設定で上げられるか」を確かめるための診断ログであって
    //   飛行に必要な情報ではないので、実機ビルドからは外す。
#ifndef RELEASE
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
#endif  // !RELEASE

    bno085_ok = true;
    DEBUGW_PLN(20260315, "[IMU] BNO085 init OK!");
    enqueueTask(createLogSdTask("BNO085 init OK"));
    // センサー組み合わせのログは airdata_setup() の後に GPS_TFT_map.ino で出力する。
}

// ============================================================
// imu_try_recovery(): BNO085 通信途絶時の再起動試行（内部関数・ステートマシン）
// ============================================================
// imu_update() から毎ループ呼ばれる。1 分に 1 回だけ復旧手順を開始し、
// 以降は 1 ループにつき 1 段ずつ進める。**待ちのために Core0 を止めない。**
//
//   IDLE ──(1分経過)──▶ RESET ──(410ms 経過)──▶ BEGIN ──▶ IDLE
//                       NRST パルス            生存確認         バス初期化＋レポート再有効化
//                                                  │
//                                                  └─(300ms 応答なし)──▶ IDLE（失敗・次は 1 分後）
//
// ★ 以前はこの関数の中で delay(10)+delay(400) を通していたため、
//   1 分ごとに Core0 が 410ms 止まり、GNSS の受信や描画が巻き添えになって
//   溢れて NAV-PVT を落としていた。地図もボタンもバリオも同時に止まっていた。
//
// ★★ 「BNO085 が本当に居るか」は **ライブラリを呼ぶ前に** 確かめること。
//   I2C では begin_I2C() の中の detected() がそれを果たす（_init() へ進む前に
//   false を返すので sh2_getProdIds() には到達しない）。
//   ★ **自前で二重に探査しないこと。**背中合わせの 2 回目が落ちる（imu_bus_begin 参照）。
//   復旧は BNO085 が死んでいるときにこそ走るので、ここを素通しにはできない。
//
// ※ 残るブロックは BEGIN の 1 ティックだけ。ACK が返る＝応答している状態なので
//   begin_I2C() は通常数 ms で返る。**ただし ACK には応答したが直後に黙る、
//   という壊れ方をすると今も長く止まる**（ライブラリ側にタイムアウトが無いため、
//   こちらからは塞げない）。
//
// 成功すれば bno085_ok=true に戻し、失敗なら次の 1 分後に再試行する。
// v7 では BNO085 専用バス（i2c1）なので、MS5611 側への影響は無い。
// CORE0
typedef enum {
    IMU_RECOV_IDLE = 0,   // 待機中（次の試行までのインターバル）
    IMU_RECOV_RESET,      // NRST パルス〜ブート待ち（imu_reset_poll() が進める）
    IMU_RECOV_BEGIN,      // バス初期化とレポート再有効化
} ImuRecovState;
static ImuRecovState imu_recov_state = IMU_RECOV_IDLE;
static unsigned long imu_recov_wait_ms = 0;   // BEGIN へ進んだ時刻（所要時間のログ用）

static void imu_try_recovery() {
    static unsigned long last_recovery_ms = 0;
    const unsigned long now_ms = millis();

    switch (imu_recov_state) {
    case IMU_RECOV_IDLE:
        // 1分以内の再試行はスキップ（last_recovery_ms==0 は初回なので即実行）
        if (last_recovery_ms > 0 && now_ms - last_recovery_ms < IMU_RECOVERY_INTERVAL_MS) return;
        last_recovery_ms = now_ms;   // 間隔は「試行の開始から開始まで」で数える

        enqueueTask(createLogSdfTask("[IMU] BNO085 comm lost, attempting recovery (INT=%d)",
                                     (int)digitalRead(IMU_INT_PIN)));
        DEBUGW_PLN(20260325, "[IMU] BNO085 recovery attempt");

        // ---- プロトコル再選択 → ハードウェアリセット開始 ----
        // PS0 は SPI 運用中 WAKE として LOW に落としてあるので、リセット前に
        // プロトコル選択のレベル（SPI なら HIGH）へ戻してからパルスを掛ける。
        imu_bus_select_protocol();
        imu_reset_start();
        imu_recov_state = IMU_RECOV_RESET;
        return;                      // ★ 待たずに帰る。410ms は次のループ以降で数える

    case IMU_RECOV_RESET:
        if (!imu_reset_poll()) return;   // まだリセット中。1 ループも止めない
        // ★ I2C では生存確認を H_INTN では行わない（読んでいないため）。
        //   代わりに imu_bus_begin() の先頭でアドレス ACK を見る。
        //   そこで弾けるので、ここは待たずに再初期化へ進んでよい。
        imu_recov_wait_ms = now_ms;
        imu_recov_state   = IMU_RECOV_BEGIN;
        return;

    case IMU_RECOV_BEGIN:
    default:
        imu_recov_state = IMU_RECOV_IDLE;
        break;                           // 下の再初期化へ進む
    }

    // ---- 再初期化 ----
    // レポート有効化と SH2 コールバック登録は imu_enable_reports() に集約している。
    // ここで直接 enableReport() を並べてはいけない:
    //   ・settings.h のレート設定と二重管理になる
    //   ・生レポート（ESKF 用）が復旧後に復活しない
    //   ・begin_*() が再登録した Adafruit 既定ハンドラを上書きし損ね、
    //     レポート取りこぼしのバグが静かに戻る
    bool recovered = imu_bus_begin();
    if (recovered) {
        // WAKE は imu_bus_begin() の中（最初の H_INTN アサート直後）で LOW にしてある。
        recovered = imu_enable_reports();
    }
    if (recovered) {
        // 受信時刻をリセット（クォータニオン・加速度も無効化）。
        // millis() をセットすることで、1秒以内にデータが届かなければ再度 timeout → 再試行の
        // サイクルに入れる。0 にすると bno085_ok=true のまま再試行が永遠に発火しなくなる。
        _last_sensor_event_ms = millis();
        _quat_valid     = false;
        _linaccel_valid = false;
        imu_feed_reset();   // 溜めた比力の平均を捨てる（実体は imu.cpp）
        bno085_ok = true;
        // ★ 表記は「リセット完了から再初期化が終わるまで」。0.976 で INT 待ちを
        //   やめたので "INT wait" ではない。
        enqueueTask(createLogSdfTask("[IMU] BNO085 recovery OK (begin %lums)",
                                     (unsigned long)(millis() - imu_recov_wait_ms)));
        DEBUGW_PLN(20260325, "[IMU] BNO085 recovery OK");
    } else {
        // 失敗: bno085_ok は false のまま。次の1分後に再試行する。
        // ★ どの段で落ちたかを残す。stage1=INT が来ない（配線・電源・リセット）、
        //   stage2=INT は来ているのに SHTP が成立しない（sh2_open / getProdIds 失敗）。
        //   これが無いと「FAILED」だけで原因が絞れない。
        enqueueTask(createLogSdfTask(
            "[IMU] BNO085 recovery FAILED (stage=%u %s init_step=%u sh2=%ld)",
            imu_begin_fail_stage,
            imu_fail_stage_name(imu_begin_fail_stage),
            pons_init_fail_step, (long)pons_init_fail_status));
        DEBUGW_PLN(20260325, "[IMU] BNO085 recovery FAILED");
    }
}

// ============================================================
// imu_drv_service_if_due(): BNO085 のレポートだけを引き取る（軽量・再入可）
// ============================================================
// ★ **長いブロッキング処理の中から呼ぶためのもの。**
//   実機で確定した不具合（2026-09-23）: Core0 が数十 ms 止まると BNO085 の
//   レポートが溜まり、そのあと読んだジャイロが化ける。
//   署名は gx=+v, gy=-v, gz=-v（3 軸が 1 つの値の符号違いのコピー）で、
//   加速度計は同時刻に何も感じていない。BNO085 内部の融合がそれを積分するので
//   GRV と LACC も同時に飛び、バリオの V/S が暴れる。
//   検証: loop() で 500ms ごとに 60ms 止めるだけで 14.8 件/分。
//         同じ 60ms の間 imu_update() を回すと **ゼロ**になった。
//   つまり原因は「Core0 が他所に取られること」ではなく「**読まないこと**」。
// ★ Kalman predict も復旧処理もしない。**レポートを引き取るだけ**なので、
//   描画や通信の途中から呼んでも副作用が無い。
// ★ ポーリング時刻は imu_update() と共有する（二重に叩かない）。
static uint32_t _poll_last_us = 0;

void imu_drv_service_if_due() {
    if (!bno085_ok) return;
    const uint32_t now_us = time_us_32();
    if ((uint32_t)(now_us - _poll_last_us) < IMU_POLL_INTERVAL_US) return;
    // ★ **実際に引き取れた間隔**の最長を記録する。imu_update() の呼び出し間隔では
    //   意味が無い（長いブロッキング処理の中からもここを呼ぶようにしたため）。
    //   60 秒ログの gapmax=。ここが数十 ms に伸びたら、どこかで飢えている。
    if (_poll_last_us) {
        const uint32_t gap = now_us - _poll_last_us;
        if (gap > _poll_gap_max_us) _poll_gap_max_us = gap;
    }
    _poll_last_us = now_us;

    // ★ 溜まっている分をまとめて引き取る。sh2_service() は 1 回で 1 パケットしか
    //   処理しないので（shtp_service() はループしない）、1 回だけだと
    //   「4ms に 1 パケット」しか掃けず、ブロックのあとで追いつけない。
    //   上限を付けるのは、ここで長居して別の飢餓を作らないため。
    for (int n = 0; n < IMU_DRAIN_MAX_PACKETS; n++) {
        _report_count = 0;
        sh2_service();
        if (_report_count > 0) _last_sensor_event_ms = millis();
        // ★ I2C は「まだ溜まっているか」を知る手段が無い（H_INTN を読んでいない）。
        //   レポートが 1 件も取れなかった回で打ち切る。advertisement や
        //   コマンド応答だけを処理した回もここで抜けるが、次の 4ms で拾える。
        if (_report_count == 0) break;
    }
}

// ============================================================
// imu_update(): ポーリングでデータ読み出しと Kalman predict を実行
// ============================================================
// loop() から毎回呼ぶ（ノンブロッキング）。
// ポーリング周期は IMU_POLL_INTERVAL_US（settings.h）で決まる。
//   生レポート有効時: 4ms（250Hz）／無効時: 30ms（33Hz）。詳細は関数の中のコメント。
// ★ v6 までは「MS5611 と i2c0 を共用するので高頻度に叩けない」という制約があったが、
//   v7 で BNO085 を i2c1 へ移してバスを分離したため、その制約は無い。
//   周期を決めているのは BNO085 のレポート配信能力のほうで、バスではない。
// 校正（DCD）をフラッシュへ保存する。**1 起動につき最大 1 回。**
// ★ なぜ要るか: このスケッチは全履歴で一度も保存しておらず（I2C 時代も同じ）、
//   BNO085 が収束させた校正が電源を切るたびに消えていた。実測で、精度 2 の
//   起動では静止時 |a| が -4.6% ずれ、6 面を動かして 3 に上がると約 -1% に改善した。
//   保存しておけば**次回は最初から良い状態で立ち上がる**。
// ★ フラッシュ書込みなので回数を抑える。保存済みの DCD が良ければ精度は起動直後に
//   3 へ達するので、**20 秒より後に初めて 3 になったときだけ**保存する。
//   これで「既に良い個体では書かない」が自動的に成立する。
// 校正が終わったかどうか。**SAVE を押せる条件。**
// ★ 地磁気は条件に入れない。屋内の金属付近では 3 に届かないことが多く、
//   バリオも GRV も地磁気を使わない（ESKF の初期ヨーだけ）。
//   地磁気まで揃えたいなら屋外で 8 の字を回すこと。
bool imu_cal_ready() {
    return bno085_ok && _acc_accuracy >= 3 && _gyr_accuracy >= 3;
}

bool     imu_cal_saved_ever()  { return _cal_rec.saved != 0; }
uint32_t imu_cal_saved_date()  { return _cal_rec.date; }
uint16_t imu_cal_save_count()  { return _cal_rec.count; }
bool     imu_cal_autosave_on() { return _cal_autosave_on; }

// 校正（DCD）を BNO085 のフラッシュへ書き、記録をこちらのフラッシュへ残す。
// yyyymmdd は 0 可（GNSS 時刻が無い＝屋内で保存したとき）。
// ★ **ここは人が長押しで呼ぶときだけ通る。**自動では呼ばない。
bool imu_save_calibration(uint32_t yyyymmdd) {
    if (!bno085_ok) {
        enqueueTask(createLogSdTask("BNO085 saveDcd SKIP (not connected)"));
        return false;
    }
    const int rc = sh2_saveDcdNow();
    if (rc != SH2_OK) {
        enqueueTask(createLogSdfTask("BNO085 saveDcd FAILED rc=%d (acc=%u gyr=%u)",
                                     rc, _acc_accuracy, _gyr_accuracy));
        return false;
    }
    _cal_rec.saved = 1;
    if (_cal_rec.count < 65535) _cal_rec.count++;
    // ★ 日付は**毎回上書きする**。取れなかったら 0（不明）にする。
    //   前回の日付を残すと「その日に校正した」と読めてしまい嘘になる。
    _cal_rec.date = yyyymmdd;
    const bool ee = imu_cal_rec_store();
    // ★ 手動で保存したら、この起動からもう自動保存は切る。
    //   次回起動を待たずに切るのは、画面の autosave 表示と実際を一致させるため
    //   （一度意図して焼いた校正を、同じ起動中にチップが上書きしてしまうのも防ぐ）。
    if (_cal_autosave_on) {
        sh2_setDcdAutoSave(false);
        _cal_autosave_on = false;
    }
    enqueueTask(createLogSdfTask(
        "BNO085 saveDcd OK date=%lu cnt=%u ee=%d (acc=%u gyr=%u)",
        (unsigned long)yyyymmdd, _cal_rec.count, (int)ee,
        _acc_accuracy, _gyr_accuracy));
    return true;
}

// ============================================================
// ゲッター（融合出力由来。能力ビットが立っているときだけ意味を持つ）
// ============================================================
// ★ 生死の 2 つ（旧 get_imu_ok / get_imu_alive）は **imu_drv_present() /
//   imu_drv_alive()** としてこのファイルの末尾にある。公開 API の名前は
//   imu.cpp が 1 行で委譲している（判定の実体を 2 か所に置かないため）。

// クォータニオン (GAME_ROTATION_VECTOR) を返す。未更新時は単位クォータニオン (1,0,0,0)。
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
    // ---- マウント補正 ----
    // ★ クォータニオンの段階で回してから 1 回だけ Euler を出す。
    //   v7 の配置（IC が基板裏面・1番ピンが天）では、機体が水平のときセンサーが
    //   ジンバルロックに入るため、**角を入れ替える旧方式では直らない**
    //   （測定の根拠と経緯は settings.h の IMU_MOUNT_Q* のコメント）。
    //
    // ロール／ピッチは GAME_RV（地磁気に依らない）、ヨーは ROTATION_VECTOR
    // （地磁気補正あり）から取る。出所が違うので**それぞれに回転を掛ける**。
    const float rad2deg = 180.0f / (float)M_PI;

    float r, p, ydummy;
    imu_body_euler_rad(_qw, _qx, _qy, _qz, r, p, ydummy);
    roll  = r * rad2deg;
    pitch = p * rad2deg;

    const float rw = _rv_valid ? _rv_qw : _qw;
    const float rx = _rv_valid ? _rv_qx : _qx;
    const float ry = _rv_valid ? _rv_qy : _qy;
    const float rz = _rv_valid ? _rv_qz : _qz;
    float rdummy, pdummy, y;
    imu_body_euler_rad(rw, rx, ry, rz, rdummy, pdummy, y);
    yaw = imu_yaw_to_heading_deg(y);   // 東基準の数学ヨー → 真方位（北=0・時計回り正）
}

// ヘディング精度推定値 [度] を返す（ROTATION_VECTOR の accuracy フィールド）。
// 値が小さいほど磁気キャリブレーションが良好。未受信時は -1 を返す。
float get_imu_mag_accuracy_deg() {
    if (!_rv_valid || _rv_accuracy < 0.0f) return -1.0f;
    return _rv_accuracy * (180.0f / M_PI);
}

float get_imu_grv_hz()  { return _grv_hz; }
float get_imu_lacc_hz() { return _lacc_hz; }
float get_imu_rv_hz()   { return _rv_hz; }

float get_imu_gyro_hz()  { return _gyro_hz; }

float get_imu_accel_hz() { return _accel_hz; }

uint8_t  get_imu_acc_accuracy() { return _acc_accuracy; }
uint8_t  get_imu_gyr_accuracy() { return _gyr_accuracy; }
uint8_t  get_imu_cal_cfg()      { return _cal_cfg; }
uint32_t get_imu_poll_gap_max_us() { const uint32_t v = _poll_gap_max_us; _poll_gap_max_us = 0; return v; }
uint32_t get_imu_grv_bad()  { return _grv_bad; }
uint32_t get_imu_grv_jump() { return _grv_jump; }
uint32_t get_imu_rv_bad()   { return _rv_bad; }

// ============================================================
//  imu_drv_poll(): 途絶検出・復旧・引き取り・レート計測を 1 回分進める
// ============================================================
// ★ 0.986 の分割前は imu_update() の前半だった。**中身は当時の行をそのまま**で、
//   「predict へ進んでよいか」を戻り値で呼び出し側（imu.cpp）へ返す形にしただけ。
//   IMU_DRV_DEAD / WAIT / SERVICED の 3 値が、分割前の
//   「return する / return する / 下へ進む」の 3 分岐に 1 対 1 で対応する。
ImuDrvState imu_drv_poll() {
    // ★★ **「データが来ない」と「こちらが見ていなかった」を区別する。**
    //   下の途絶判定は 1 秒しか猶予が無い。ところが起動時は setup() が長く、
    //   loop() に入るまで imu_update() が一度も呼ばれない。その状態で最初の 1 回を
    //   迎えると、**生きている BNO085 を「途絶」と誤判定してリセットしに行き、
    //   復旧に失敗してそのまま死ぬ**（実機 2026-09-23。起動画面の OSM 帰属表示
    //   3.5 秒ホールドのあと、loop() の 1 回目で必ず発火していた）。
    //   以前これが起きなかったのは link_setup() が起動画面の後ろにあり、
    //   その中の imu_service_if_due() が偶然この穴を塞いでいたから。
    //   **偶然に頼らない。** 呼び出し間隔が空いていたら、その回の判定は見送る。
    //   見送っても検出が 1 周期遅れるだけで、本当の断線は次の回で捕まる。
    {
        static unsigned long _last_update_call_ms = 0;
        const unsigned long now_ms = millis();
        const bool resumed = (_last_update_call_ms == 0) ||
                             (now_ms - _last_update_call_ms > 500UL);
        _last_update_call_ms = now_ms;
        if (resumed && _last_sensor_event_ms > 0) _last_sensor_event_ms = now_ms;
    }

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
        return IMU_DRV_DEAD;     // 恒速モデルへの縮退は呼び出し側（imu.cpp）が行う
    }
    // ポーリング周期は IMU_POLL_INTERVAL_US（settings.h）。
    //   生レポート有効時: 4ms ≒ 250Hz。高レートのレポートを溜めずに引き取るため。
    //                     このときバリオ KF の predict はこの周期では動かさず、
    //                     関数末尾の IMU_KF_PREDICT_INTERVAL_US ゲートで 30ms に保つ
    //                     （Q の注入量が変わってしまうため）。
    //   無効時          : 30ms ≒ 33Hz。変更前と同一で、ポーリング毎に predict が走る。
    uint32_t _now_us = time_us_32();
    if (_now_us - _poll_last_us < IMU_POLL_INTERVAL_US) return IMU_DRV_WAIT;

    // ---- データ取り出し ----
    // sh2_service() は HAL の read() を「1 回だけ」呼び、SHTP パケットを 1 個処理する
    // （shtp.c の shtp_service() はループしない）。1 パケットに複数レポートが載っている
    // 場合は、その全部が imu_sensor_handler() に配られる。
    // つまり「1 ポーリング = 1 パケット」で、パケット内の取りこぼしは無いが、
    // パケットが溜まっている場合は 1 回の呼び出しでは 1 個しか引き取れない。
    // 溜まった分は BNO085 内部のキューに残り、次のポーリングで順に引き取る。
    // ドレイン能力 = 1/IMU_POLL_INTERVAL_US（4ms なら 250 パケット/秒）。
    // 要求レート（合計 145 レポート/秒。同時刻のものは 1 パケットにまとまる）に対する
    // 余裕がこれで決まる。足りないと BNO085 側でレポートが捨てられる
    // （settings.h の IMULOG_RAW_REPORTS_ENABLED に書いた 2026-08-17 の飢餓事件）。
    //
    // ※ Adafruit の bno08x.getSensorEvent() は使わない。
    //   あちらは sh2_service() の結果を単一スロット _sensor_value に上書きするため、
    //   1 パケットに複数レポートが載っていると最後の 1 件しか返さず、残りを黙って捨てる
    //   （Adafruit_BNO08x.cpp の sensorHandler / getSensorEvent 参照）。
    //   BNO085 は同時刻にスケジュールされたレポートをまとめて 1 パケットで送るため、
    //   要求レートを上げるほどこの取りこぼしが増える。
    //   2026-08-17 に GRV/LACC/RV が 15/15/5Hz 設定に対し 4/5/1Hz まで飢餓になり、
    //   古い加速度を繰り返し積分してバリオが暴れた原因がこれだった。
    // ★ 引き取りは imu_drv_service_if_due() に集約してある（ポーリング時刻も共有）。
    //   **公開 API の imu_service_if_due() を呼ばないこと。**あちらは imu.cpp の
    //   委譲で、ドライバから呼ぶと層が逆流する（向こうに判定が足された日に化ける）。
    imu_drv_service_if_due();

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
            _grv_cnt = _lacc_cnt = _rv_cnt = 0;
            _gyro_cnt = _accel_cnt = 0;
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

    return IMU_DRV_SERVICED;
}

// ============================================================
//  imu_sensor.h の契約のうち、状態を答えるだけのもの
// ============================================================
// ★ 公開 API の get_imu_ok() / get_imu_alive() は imu.cpp がこれを呼ぶだけ。
//   判定の実体を 2 か所に置かないため（CLAUDE.md「設定値は一箇所だけ」と同じ理由）。
bool imu_drv_present() { return bno085_ok; }

bool imu_drv_alive() {
    if (!bno085_ok) return false;
    if (_last_sensor_event_ms == 0) return false;
    return (millis() - _last_sensor_event_ms <= 1000UL);
}

// VARIO_USE_RAW_ACCEL=0（LACC ベースの predict）の旧経路用。
// ★ この経路は既定では使わない（settings.h は 1）。**消さないのは選択肢を残すため**で、
//   分割後は imu.cpp から _lax.. / _qw.. が見えないので、ここを通して渡す。
bool imu_drv_lacc_predict_sample(float a_body[3], float q[4], uint32_t* lacc_last_us) {
    if (!_quat_valid || !_linaccel_valid) return false;
    a_body[0] = _lax; a_body[1] = _lay; a_body[2] = _laz;
    q[0] = _qw; q[1] = _qx; q[2] = _qy; q[3] = _qz;
    if (lacc_last_us) *lacc_last_us = _lacc_last_us;
    return true;
}

#endif  // IMU_SENSOR_DEFAULT == IMU_SENSOR_BNO085
