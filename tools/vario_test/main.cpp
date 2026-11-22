// ============================================================
// File    : tools/vario_test/main.cpp
// Role    : imu.cpp（3状態バリオ KF・加速度ガード・MS5611 復帰 reinit・供給元選択）のホスト単体テスト。
//           1) 静止・定常上昇（+1.5 m/s）での KF 収束とパラメータ異常値ガード
//           2) MS5611 途絶→復帰時の imu_kalman_reinit()（BUG-118 / BUG-121）による V/S スパイク防止
//           3) 鉛直加速度ガード（KF_ACC_GUARD_*）の発動・バイアス保護・2 秒静定後の復帰
//           4) IMU 途絶時（IMU_DRV_DEAD）の kf_predict_baro_only()（BUG-65）
//           5) vario_source_ok / vario_vspeed_mps / vario_deadband_mps の 3 系統切り替え
//           8) GNSS の上下速度の食い違いの門と、GNSS の高度による逃がし道（GNSS_VSI_CONSIST_*。2026-11-19〜）
// ============================================================
#include "Arduino.h"
#include "imu.h"
#include "src/imu_sensor.h"
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
    // 壊れた姿勢で -7.5 m/s² の偽の鉛直加速度が入り続けても、ガードが入り（2026-11-18〜 速い方は 6 m/s²・300ms なので、
    // 1 秒の低域 2 m/s² の遅い方で約 0.3 秒）、気圧が平坦なら V/S が大きく引きずられないこと。
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

    // ---- 6) GNSS の上下速度を入れる条件（2026-11-18〜。imu_gnss_vsi_gate と残差の門）----
    CHECK(imu_gnss_vsi_gate(true, 20, 0.10f, 1.0f) == GVSI_OK, "gate OK");
    CHECK(imu_gnss_vsi_gate(false, 20, 0.10f, 1.0f) == GVSI_NO_3D, "gate no 3D");
    CHECK(imu_gnss_vsi_gate(true, GNSS_VSI_MIN_SATS - 1, 0.10f, 1.0f) == GVSI_SATS, "gate sats");
    CHECK(imu_gnss_vsi_gate(true, 20, GNSS_VSI_SACC_MAX_MPS, 1.0f) == GVSI_SACC, "gate sAcc (>= の側で切る)");
    CHECK(imu_gnss_vsi_gate(true, 20, NAN, 1.0f) == GVSI_SACC, "gate sAcc NaN");
    CHECK(imu_gnss_vsi_gate(true, 20, 0.10f, GNSS_VSI_VACC_MAX_M) == GVSI_VACC, "gate vAcc");
    {
        uint32_t c[GVSI_N_REASONS];
        imu_gnss_vsi_take_counts(c);                       // それまでの数を捨てる
        g_stub_itow_ms += 500;
        imu_kalman_gnss_vel_update(0.30f, 1.0f, 0.10f, 20);   // 入る
        imu_kalman_gnss_vel_update(0.30f, 1.0f, 0.10f, 20);   // 同じ測位はもう入れない（数えもしない）
        CHECK(imu_gnss_vsi_active(), "vsi active after a used fix");
        g_stub_itow_ms += 500;
        imu_kalman_gnss_vel_update(0.30f, 1.0f, 0.10f, 10);   // 衛星不足
        g_stub_itow_ms += 500;
        imu_kalman_gnss_vel_update(30.0f, 1.0f, 0.10f, 20);   // 30 m/s は残差の門で捨てる（反射の保険）
        imu_gnss_vsi_take_counts(c);
        CHECK(c[GVSI_OK] == 1 && c[GVSI_SATS] == 1 && c[GVSI_INNOV] == 1,
              "vsi counts used=%u sats=%u innov=%u (1/1/1)", (unsigned)c[GVSI_OK], (unsigned)c[GVSI_SATS], (unsigned)c[GVSI_INNOV]);
        CHECK(fabsf(get_imu_vspeed_raw()) < 1.0f, "outlier did not move V/S (%.3f)", get_imu_vspeed_raw());
    }

    // ---- 7) 手で上下に動かしてもガードが入らず、V/S が追う（2026-11-18〜 KF_ACC_GUARD_FAST 6 m/s²・300ms）----
    //   ±10cm・1.5Hz（加速度のピーク約 8.9 m/s²）。4 m/s²・120ms のころは半周期ごとに約 210ms 超えて入っていた。
    {
        run_sim(4.0f, vz_true, 0.0f, 0.0f, z_baro, true);       // 落ち着かせる（前の試験のガードを戻す）
        const int   log0 = g_stub_log_count;
        const float A = 0.10f, w = 2.0f * (float)M_PI * 1.5f;
        const float zb0 = z_baro;
        float vmax = 0.0f, vmin = 0.0f;
        for (int i = 0; i < (int)(6.0f * 250.0f); i++) {
            test_advance_us(4000);
            const float tt = (i + 1) * 0.004f;
            const float a  = A * w * w * cosf(w * tt);           // z = A(1 − cos ωt): 止まった所から動き出す
            const float zz = A * (1.0f - cosf(w * tt));
            feed_imu_sample(a);
            imu_update();
            if (i % 6 == 0) imu_kalman_baro_update(zb0 + zz);
            if (tt > 1.0f) {
                const float v = get_imu_vspeed();
                if (v > vmax) vmax = v;
                if (v < vmin) vmin = v;
            }
        }
        z_baro = zb0;
        const bool entered = g_stub_log_count > log0 && strstr(g_stub_last_log, "ignored") != nullptr;
        CHECK(!entered, "hand shake did not trip the guard (%s)", g_stub_last_log);
        CHECK(vmax > 0.6f && vmin < -0.6f, "V/S follows hand shake (%.2f .. %.2f, true ±%.2f)", vmin, vmax, A * w);

        // 7 m/s² が 0.5 秒続く（遅い方は 0.5 秒で 2.75 にしか届かない…ではなく 2.75 を超える前に速い方が 300ms で入る）
        run_sim(4.0f, vz_true, 0.0f, 0.0f, z_baro, true);
        const int log1 = g_stub_log_count;
        run_sim(0.5f, vz_true, 0.0f, 7.0f, z_baro, true);
        CHECK(g_stub_log_count > log1 && strstr(g_stub_last_log, "ignored") != nullptr,
              "7 m/s² for 0.5 s trips the guard (%s)", g_stub_last_log);
        run_sim(6.0f, vz_true, 0.0f, 0.0f, z_baro, true);
        CHECK(strstr(g_stub_last_log, "IMU accel back") != nullptr, "guard recovered after 7 m/s² (%s)", g_stub_last_log);
    }

    // ---- 8) GNSS の上下速度の食い違いの門（GNSS_VSI_CONSIST_*。2026-11-19〜）----
    //   GNSS を 2Hz で入れながら回す。gnss_fn(t) が velD（上昇正）を返す。
    {
        auto run_with_gnss = [&](float secs, float az_true, float (*gnss_fn)(float), float& vmin, float& vmax, uint32_t c[GVSI_N_REASONS]) {
            imu_gnss_vsi_take_counts(c);
            vmin = 1e9f; vmax = -1e9f;
            const int steps = (int)(secs * 250.0f);
            for (int i = 0; i < steps; i++) {
                test_advance_us(4000);
                const float tt = (i + 1) * 0.004f;
                vz_true += az_true * 0.004f;
                z_baro  += vz_true * 0.004f;
                feed_imu_sample(az_true);
                imu_update();
                if (i % 6 == 0) imu_kalman_baro_update(z_baro);
                if (i % 125 == 124) {                        // 2Hz の測位
                    g_stub_itow_ms += 500;
                    imu_kalman_gnss_vel_update(gnss_fn(tt) + vz_true, 1.0f, 0.09f, 29);
                }
                const float v = get_imu_vspeed_raw();
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
            }
            imu_gnss_vsi_take_counts(c);
        };
        uint32_t c[GVSI_N_REASONS];
        float vmin, vmax;
        g_drv_present = false;                              // get_imu_vspeed_raw() で KF の値を直接見る
        vz_true = 0.0f;
        // まず正しい GNSS で落ち着かせる
        run_with_gnss(6.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c);
        CHECK(c[GVSI_OK] >= 10 && c[GVSI_CONSIST] == 0, "clean GNSS used (used=%u consist=%u)", (unsigned)c[GVSI_OK], (unsigned)c[GVSI_CONSIST]);

        // 11/19 13:20:16 の形: 平らな所で、GNSS の velD だけが 3 秒かけて下向き 0.87 m/s まで狂い、1 秒続いて 1 秒で戻る。
        //   加速度も気圧も平ら。直す前は V/S が −0.8 m/s まで付いていった。
        run_with_gnss(8.0f, 0.0f, [](float t) {
            if (t < 1.0f) return 0.0f;
            if (t < 4.0f) return -0.87f * (t - 1.0f) / 3.0f;
            if (t < 5.0f) return -0.87f;
            if (t < 6.0f) return -0.87f * (6.0f - t);
            return 0.0f;
        }, vmin, vmax, c);
        CHECK(vmin > -0.35f, "multipath velD drift rejected: V/S min %.2f (was ~-0.8 before the consist gate)", vmin);
        CHECK(c[GVSI_CONSIST] >= 3, "consist counted (%u)", (unsigned)c[GVSI_CONSIST]);

        // 本物の上昇: 加速度・気圧・GNSS がそろって 0 → +1.0 m/s（1 秒の加速）。GNSS は捨てず、V/S は付いていく。
        run_with_gnss(4.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c);
        run_with_gnss(1.0f, 1.0f, [](float) { return 0.0f; }, vmin, vmax, c);
        uint32_t c2[GVSI_N_REASONS];
        run_with_gnss(5.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c2);
        CHECK(c[GVSI_CONSIST] == 0 && c2[GVSI_CONSIST] == 0, "real climb not rejected (consist=%u/%u)", (unsigned)c[GVSI_CONSIST], (unsigned)c2[GVSI_CONSIST]);
        CHECK(fabsf(get_imu_vspeed_raw() - 1.0f) < 0.1f, "V/S follows real climb (%.3f, expect 1.0)", get_imu_vspeed_raw());

        // 加速度計の積分だけが狂っている（GNSS を入れない KF とは合う）なら捨てない: 加速度に +0.2 m/s² の偏りを 5 秒
        run_with_gnss(1.0f, -1.0f, [](float) { return 0.0f; }, vmin, vmax, c);   // 水平に戻す
        run_with_gnss(4.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c);
        // 逃がし道（GNSS_VSI_CONSIST_HEIGHT_*）: 13:20 と同じ velD の狂い方でも、GNSS 自身の高度がそれに合って下がっていれば入れる。
        //   高度が平らなら今までどおり捨てる。
        auto run_drift_with_height = [&](bool height_follows, float& vmin_out, uint32_t c[GVSI_N_REASONS]) {
            run_with_gnss(4.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c);   // 落ち着かせる
            imu_gnss_vsi_take_counts(c);
            vmin_out = 1e9f;
            float h = 50.0f;
            for (int i = 0; i < (int)(8.0f * 250.0f); i++) {
                test_advance_us(4000);
                const float tt = (i + 1) * 0.004f;
                const float vg = (tt < 1.0f) ? 0.0f : (tt < 4.0f) ? -0.87f * (tt - 1.0f) / 3.0f
                               : (tt < 5.0f) ? -0.87f : (tt < 6.0f) ? -0.87f * (6.0f - tt) : 0.0f;
                if (height_follows) h += vg * 0.004f;
                feed_imu_sample(0.0f);
                imu_update();
                if (i % 6 == 0) imu_kalman_baro_update(z_baro);
                if (i % 125 == 124) { g_stub_itow_ms += 500; imu_kalman_gnss_vel_update(vg, 1.0f, 0.09f, 29, h); }
                const float v = get_imu_vspeed_raw();
                if (v < vmin_out) vmin_out = v;
            }
            imu_gnss_vsi_take_counts(c);
        };
        float vmin_h;
        run_drift_with_height(false, vmin_h, c);
        CHECK(vmin_h > -0.35f && c[GVSI_CONSIST] >= 3 && c[GVSI_RESCUED] == 0,
              "flat GNSS height: drift still rejected (V/S min %.2f, consist=%u rescued=%u)", vmin_h, (unsigned)c[GVSI_CONSIST], (unsigned)c[GVSI_RESCUED]);
        run_drift_with_height(true, vmin_h, c);
        CHECK(c[GVSI_RESCUED] >= 3, "GNSS height follows velD: rescued (rescued=%u consist=%u)", (unsigned)c[GVSI_RESCUED], (unsigned)c[GVSI_CONSIST]);
        CHECK(vmin_h < -0.5f, "GNSS height follows velD: V/S follows GNSS (min %.2f)", vmin_h);
        // 13:30:37 の形: velD と高度が一緒に 1 秒だけ跳ねる。高度は合っていても跳ねなので逃がさない（STEP_MPS）。
        {
            run_with_gnss(6.0f, 0.0f, [](float) { return 0.0f; }, vmin, vmax, c);
            imu_gnss_vsi_take_counts(c);
            float h = 50.0f, vmin_s = 1e9f;
            for (int i = 0; i < (int)(5.0f * 250.0f); i++) {
                test_advance_us(4000);
                const float tt = (i + 1) * 0.004f;
                // 2Hz の測位で −1.0 → −1.5 → 0（実際は 4Hz で −1.0 / −1.53 / −2.56 / −0.84。−2.6 だけなら残差の門で捨てられる）
                const float vg = (tt >= 2.0f && tt < 2.5f) ? -1.0f : (tt >= 2.5f && tt < 3.0f) ? -1.5f : 0.0f;
                feed_imu_sample(0.0f);
                imu_update();
                if (i % 6 == 0) imu_kalman_baro_update(z_baro);
                // GNSS の高度も測位ごとに velD どおり動く（GNSS の中では辻褄が合っている）
                if (i % 125 == 124) { g_stub_itow_ms += 500; h += vg * 0.5f; imu_kalman_gnss_vel_update(vg, 1.0f, 0.09f, 29, h); }
                const float v = get_imu_vspeed_raw();
                if (v < vmin_s) vmin_s = v;
            }
            imu_gnss_vsi_take_counts(c);
            CHECK(c[GVSI_RESCUED] == 0 && vmin_s > -0.6f,
                  "velD+height spike not rescued (V/S min %.2f, rescued=%u consist=%u innov=%u)", vmin_s,
                  (unsigned)c[GVSI_RESCUED], (unsigned)c[GVSI_CONSIST], (unsigned)c[GVSI_INNOV]);
        }

        {
            imu_gnss_vsi_take_counts(c);
            for (int i = 0; i < (int)(5.0f * 250.0f); i++) {
                test_advance_us(4000);
                feed_imu_sample(0.2f);                       // 偽の +0.2 m/s²（本当は平ら）
                imu_update();
                if (i % 6 == 0) imu_kalman_baro_update(z_baro);
                if (i % 125 == 124) { g_stub_itow_ms += 500; imu_kalman_gnss_vel_update(0.0f, 1.0f, 0.09f, 29); }
            }
            imu_gnss_vsi_take_counts(c);
            CHECK(c[GVSI_CONSIST] <= 2, "accel-only error does not lock out GNSS (consist=%u used=%u)", (unsigned)c[GVSI_CONSIST], (unsigned)c[GVSI_OK]);
            CHECK(fabsf(get_imu_vspeed_raw()) < 0.25f, "V/S held near 0 by GNSS despite accel offset (%.3f)", get_imu_vspeed_raw());
        }
        g_drv_present = true;
    }

    // ---- 8) 起動地の MSL 高度（gnss_kf_offset）の決め方と決め直し（2026-11-22。settings.h の GNSS_OFFSET_*）----
    //   11/22 の車: 最初の測位の GNSS 高度がでたらめ（vAcc 9.9 m と出て実際は 50 m 以上ずれ）でオフセットが +82.7 m で決まり、
    //   50 m の外れ値の門で長期更新も届かず 30 分ずっと 80 m 高かった。
    {
        auto gnss_1s = [&](float msl_minus_kf, float vacc, int n) {
            for (int k = 0; k < n; k++) {
                run_sim(1.0f, vz_true, 0.0f, 0.0f, z_baro, true);     // 1 秒（imu_kalman_gnss_update は 1 秒に 1 回）
                imu_kalman_gnss_update(z_baro + msl_minus_kf, vacc);
            }
        };
        const float TRUE_MSL = 3.0f;          // 起動地の本当の MSL（KF は起動地 = z_baro の高さ）
        gnss_1s(TRUE_MSL + 80.0f, 9.9f, 12);  // vAcc 9.9 m（GNSS_VACC_MAX_M は通るが決めるのには使わない）
        CHECK(!get_imu_gnss_offset_ready(), "offset not set from vAcc 9.9 m fixes");
        gnss_1s(TRUE_MSL + 80.0f, 4.0f, GNSS_INIT_SAMPLES);   // vAcc は良いと言いながら 80 m ずれた測位で決まってしまった
        CHECK(get_imu_gnss_offset_ready() && fabsf(get_imu_altitude_msl() - (z_baro + TRUE_MSL + 80.0f)) < 1.0f,
              "bad first fix sets offset +80 (alt %.1f, expect %.1f)", get_imu_altitude_msl(), z_baro + TRUE_MSL + 80.0f);
        const int log0 = g_stub_log_count;
        gnss_1s(TRUE_MSL, 2.0f, GNSS_OFFSET_REINIT_SAMPLES - 1);  // 正しい測位が 19 回: まだ決め直さない
        CHECK(g_stub_log_count == log0 && fabsf(get_imu_altitude_msl() - (z_baro + TRUE_MSL + 80.0f)) < 1.0f,
              "no re-init before %d fixes", GNSS_OFFSET_REINIT_SAMPLES);
        gnss_1s(TRUE_MSL, 6.0f, 3);           // vAcc 6 m（決め直しに数えない）を挟むと数え直し
        gnss_1s(TRUE_MSL, 2.0f, GNSS_OFFSET_REINIT_SAMPLES - 1);
        CHECK(g_stub_log_count == log0, "re-init count restarts after a vAcc 6 m fix");
        gnss_1s(TRUE_MSL, 2.0f, 1);           // 20 回目
        CHECK(strstr(g_stub_last_log, "re-init") != nullptr && fabsf(get_imu_altitude_msl() - (z_baro + TRUE_MSL)) < 1.0f,
              "re-init to true MSL (alt %.2f, expect %.2f; %s)", get_imu_altitude_msl(), z_baro + TRUE_MSL, g_stub_last_log);
        // マルチパスの外れ（80 m・5 秒）では決め直さない
        const int log1 = g_stub_log_count;
        gnss_1s(TRUE_MSL + 80.0f, 3.0f, 5);
        gnss_1s(TRUE_MSL, 2.0f, 30);
        CHECK(g_stub_log_count == log1 && fabsf(get_imu_altitude_msl() - (z_baro + TRUE_MSL)) < 1.0f,
              "5 s multipath outlier does not re-init (alt %.2f)", get_imu_altitude_msl());
    }

    printf("vario_test: %d passed, %d failed\n", g_pass, g_fail);
    printf("結果: %s\n", g_fail == 0 ? "PASS — バリオ KF・加速度ガード・MS5611 復帰 全テスト合格" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
