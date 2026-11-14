// ============================================================
// File    : tools/vario_test/main.cpp
// Role    : imu.cpp（3状態バリオ KF・加速度ガード・MS5611 復帰 reinit・供給元選択）のホスト単体テスト。
//           1) 静止・定常上昇（+1.5 m/s）での KF 収束とパラメータ異常値ガード
//           2) MS5611 途絶→復帰時の imu_kalman_reinit()（BUG-118 / BUG-121）による V/S スパイク防止
//           3) 鉛直加速度ガード（KF_ACC_GUARD_*）の発動・バイアス保護・2 秒静定後の復帰
//           4) IMU 途絶時（IMU_DRV_DEAD）の kf_predict_baro_only()（BUG-65）
//           5) vario_source_ok / vario_vspeed_mps / vario_deadband_mps の 3 系統切り替え
// ============================================================
#include "Arduino.h"
#include "imu.h"
#include "imu_sensor.h"
#include "attitude.h"
#include "settings.h"
#include <limits>

uint32_t    g_test_us = 1000000UL;
DummySerial Serial;

bool     g_stub_mirror_active = false;
float    g_stub_link_vs       = 0.0f;
float    g_stub_link_alt      = 0.0f;
uint16_t g_stub_link_have     = 0;
uint16_t g_stub_link_status   = 0;

bool     g_stub_airdata_ok     = true;
float    g_stub_airdata_vs     = 0.0f;
float    g_stub_ground_alt_adj = 0.0f;

bool     g_stub_replay_mode = false;
int      g_stub_log_count   = 0;
char     g_stub_last_log[256] = "";

uint32_t g_stub_itow_ms     = 0;
uint16_t g_stub_replay_have = 0;
float    g_stub_replay_vs   = 0.0f;
float    g_stub_replay_alt  = 0.0f;

// ドライバスタブの状態
static bool        g_drv_present = true;
static bool        g_drv_alive   = true;
static ImuDrvState g_drv_state   = IMU_DRV_SERVICED;

const char* imu_sensor_name()         { return "SCH16T-STUB"; }
void        imu_drv_setup()           {}
ImuDrvState imu_drv_poll()            { return g_drv_state; }
void        imu_drv_service_if_due()  {}
bool        imu_drv_present()         { return g_drv_present; }
bool        imu_drv_alive()           { return g_drv_alive; }

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] line %d: " msg "\n", __LINE__, ##__VA_ARGS__); } \
} while (0)

// センサー座標系で鉛直加速度 az_lin [m/s²]（上向き正、重力除去後）に相当する比力を 1 サンプル流し込む。
// 水平静止時の上方向はセンサー軸で {-1, 0, 0}。
static void feed_imu_sample(float az_lin_mps2) {
    const float g0[3] = { 0.0f, 0.0f, 0.0f };
    const float a0[3] = { -(GRAVITY_MPS2 + az_lin_mps2), 0.0f, 0.0f };
    imu_feed_accel(a0);
    imu_feed_gyro(g0, g_test_us);
}

// 4ms 刻みで secs 秒間シミュレーションを回す（30ms ごとに imu_update が predict、24ms ごとに MS5611 気圧観測更新）
static void run_sim(float secs, float& true_vz_mps, float az_true_mps2, float az_meas_mps2, float& z_baro, bool feed_baro = true) {
    const int steps = (int)(secs * 250.0f); // 4ms step
    for (int i = 0; i < steps; i++) {
        test_advance_us(4000);
        true_vz_mps += az_true_mps2 * 0.004f;
        z_baro      += true_vz_mps * 0.004f;
        if (g_drv_state != IMU_DRV_DEAD) {
            feed_imu_sample(az_meas_mps2);
        }
        imu_update();
        // MS5611 の計測完了（約 40Hz = 24ms ごと）に相当
        if (feed_baro && (i % 6 == 0)) {
            imu_kalman_baro_update(z_baro);
        }
    }
}

int main() {
    // ---- 0) ESKF 水平化 + KF 初期化 ----
    attitude_setup();
    imu_kalman_reinit();
    float z_baro = 100.0f;
    float vz_true = 0.0f;
    imu_kalman_baro_update(z_baro); // 初回観測で kf_initialized = true, z = 100m
    run_sim(6.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    CHECK(attitude_ready(), "attitude ready after 6s");
    CHECK(fabsf(get_imu_vspeed()) < 0.05f, "stationary vspeed=%.3f", get_imu_vspeed());
    CHECK(fabsf(get_imu_altitude_msl() - 100.0f) < 0.2f, "stationary alt=%.2f", get_imu_altitude_msl());

    // ---- 1) 加速 (1.5 m/s² × 1s) → 定常上昇 (+1.5 m/s) への追従とパラメータ異常値ガード ----
    run_sim(1.0f, vz_true, 1.5f, 1.5f, z_baro, true);
    run_sim(5.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    CHECK(fabsf(get_imu_vspeed() - 1.5f) < 0.08f, "climb vspeed=%.3f (expect ~1.5)", get_imu_vspeed());
    run_sim(1.0f, vz_true, -1.5f, -1.5f, z_baro, true);
    run_sim(5.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    CHECK(fabsf(get_imu_vspeed()) < 0.08f, "back to level vspeed=%.3f", get_imu_vspeed());

    const float qv0 = get_imu_kf_q_vel();
    imu_set_kf_params(-1.0f, 0.0f, std::numeric_limits<float>::quiet_NaN());
    CHECK(get_imu_kf_q_vel() == qv0, "reject invalid KF params");

    // ---- 2) MS5611 途絶→復帰時の imu_kalman_reinit()（BUG-118 / BUG-121）----
    // 気圧観測が 10 秒途絶え、その間に微小な加速度バイアス (+0.3 m/s²) で KF の高度・速度が漂った状況を模擬
    g_stub_airdata_ok = false;
    run_sim(10.0f, vz_true, 0.0f, 0.30f, z_baro, false); // 気圧なし・加速度のみで 10 秒漂う
    CHECK(get_imu_vspeed() > 1.5f, "drifted internal vspeed=%.2f during baro loss", get_imu_vspeed());
    // ただし vario_vspeed_mps() は get_airdata_ok() == false なので 0 を返し音を止める
    CHECK(vario_vspeed_mps() == 0.0f && !vario_source_ok(), "vario muted while airdata dead");

    // 復帰時に imu_kalman_reinit() を呼んでから気圧を入れると V/S が跳ねないことを確認
    imu_kalman_reinit();
    g_stub_airdata_ok = true;
    CHECK(get_imu_vspeed() == 0.0f, "vspeed zeroed right after reinit");
    imu_kalman_baro_update(z_baro); // 再初期化後の最初の気圧観測
    run_sim(1.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    CHECK(fabsf(get_imu_vspeed()) < 0.08f, "no V/S spike after baro recovery with reinit (got %.3f)", get_imu_vspeed());

    // ---- 3) 鉛直加速度ガード（KF_ACC_GUARD_*）の発動と復帰 ----
    // 壊れた姿勢で -7.5 m/s² の偽の鉛直加速度が入り続けても、120ms（4 predict）でガードが入り、
    // 気圧が平坦なら V/S が大きく引きずられないこと。
    const int log_before = g_stub_log_count;
    run_sim(4.0f, vz_true, 0.0f, -7.5f, z_baro, true);
    CHECK(g_stub_log_count > log_before, "acc guard logged entry (%s)", g_stub_last_log);
    CHECK(fabsf(get_imu_vspeed()) < 0.25f, "acc guard prevented V/S runaway (got %.3f)", get_imu_vspeed());

    // 静定後（az_meas = 0）に数秒回すとガードが解除されること
    run_sim(6.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    CHECK(strstr(g_stub_last_log, "IMU accel back") != nullptr, "acc guard logged recovery (%s)", g_stub_last_log);

    // ---- 4) IMU 途絶時（IMU_DRV_DEAD）の kf_predict_baro_only（BUG-65）----
    g_drv_alive = false;
    g_drv_state = IMU_DRV_DEAD;
    vz_true = -1.0f;
    run_sim(6.0f, vz_true, 0.0f, 0.0f, z_baro, true);
    // IMU が死んでいても気圧だけで KF は -1.0 m/s へ追従する
    g_drv_present = false; // get_imu_vspeed_raw() で KF の値を直接見るため
    CHECK(fabsf(get_imu_vspeed_raw() - (-1.0f)) < 0.15f, "baro-only KF tracks -1.0 m/s (got %.3f)", get_imu_vspeed_raw());
    g_drv_present = true;
    g_drv_alive   = true;
    g_drv_state   = IMU_DRV_SERVICED;
    vz_true = 0.0f;
    run_sim(3.0f, vz_true, 0.0f, 0.0f, z_baro, true);

    // ---- 5) 供給元（実センサー / リプレイ / 無線ミラー）とデッドバンドの切り替え ----
    g_stub_mirror_active = true;
    g_stub_link_have     = RHAVE_KFVS;
    g_stub_link_vs       = 2.35f;
    g_stub_link_status   = 0;
    CHECK(vario_source_ok() && fabsf(vario_vspeed_mps() - 2.35f) < 1e-4f, "mirror vspeed");
    CHECK(fabsf(vario_deadband_mps() - VARIO_DEADBAND_KF_MPS) < 1e-4f, "mirror deadband KF");
    g_stub_link_status   = LINK_ST_IMU_ERROR;
    CHECK(fabsf(vario_deadband_mps() - VARIO_DEADBAND_BARO_MPS) < 1e-4f, "mirror deadband BARO when sender IMU error");
    g_stub_mirror_active = false;

    g_stub_replay_mode = true;
    g_stub_replay_have = RHAVE_KFVS;
    g_stub_replay_vs   = -0.85f;
    CHECK(vario_source_ok() && fabsf(vario_vspeed_mps() - (-0.85f)) < 1e-4f, "replay vspeed");
    g_stub_replay_mode = false;
    g_stub_replay_have = 0;

    printf("vario_test: %d passed, %d failed\n", g_pass, g_fail);
    printf("結果: %s\n", g_fail == 0 ? "PASS — バリオ KF・加速度ガード・MS5611 復帰 全テスト合格" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
