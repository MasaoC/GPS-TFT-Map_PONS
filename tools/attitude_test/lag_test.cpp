// ============================================================
// File    : tools/attitude_test/lag_test.cpp
// Project : PONS v7.1
// Role    : GNSS 速度の遅れの補償（attitude_on_gnss_velocity の age_us と速度の履歴）を PC 上で確かめる。
//           協調旋回の合成データで、GNSS 速度を遅らせて渡したときの方位の遅れが、補償で消えるかを見る。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/15
// ============================================================
#include "attitude.h"
#include "settings.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

static const double G = 9.80665;

static void body_to_sensor(const double b[3], float s_out[3]) {   // main.cpp と同じ
    s_out[0] = (float)(-b[2]);
    s_out[1] = (float)( b[1]);
    s_out[2] = (float)( b[0]);
}

struct Result { double yaw_err_mean, yaw_err_rms, roll_err_mean; };

// bank_deg の協調旋回を 60 秒。GNSS 速度（4Hz）は lag_s だけ前の値を渡し、age_us を添える。
// jitter_s > 0 なら、渡すのを 1 回おきにさらに jitter_s 遅らせる（Core0 が描画で受け取りに行けなかった回のまね。
// そのぶんは age_us に足す＝gnss.cpp の navpvt_age_us が iTOW から測るぶん）。
static Result run(double bank_deg, double lag_s, uint32_t age_us, double jitter_s, bool age_has_jitter) {
    attitude_setup();
    const double phi = bank_deg * M_PI / 180.0, V = 8.0;
    const double fs = (double)IMU_RATE_GYRO_HZ, dt = 1.0 / fs;
    double tbase = 0.0;

    // 1) 静止 4 秒（水平・機首東）
    {
        const double ab[3] = {0, 0, G};
        float g0[3] = {0, 0, 0}, a0[3];
        body_to_sensor(ab, a0);
        for (int i = 0; i < (int)(4.0 * fs); i++) {
            attitude_on_accel(a0);
            test_set_us((uint32_t)((tbase + i * dt) * 1e6)); attitude_on_gyro(g0, g_test_us);
        }
        tbase += 4.0;
    }
    // 2) 東へ 8 m/s で直進 3 秒（方位を航跡から入れる）
    {
        const double ab[3] = {0, 0, G};
        float g0[3] = {0, 0, 0}, a0[3];
        body_to_sensor(ab, a0);
        for (int i = 0; i < (int)(3.0 * fs); i++) {
            attitude_on_accel(a0);
            test_set_us((uint32_t)((tbase + i * dt) * 1e6)); attitude_on_gyro(g0, g_test_us);
            if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, (float)V, 0.0f, 0.08f, age_us);
        }
        tbase += 3.0;
    }
    // 3) 協調旋回 60 秒。バンクは 3 秒かけて滑らかに入れる（いきなり傾けるとジャイロに出ない回転になり、
    //    フィルターがそのぶんをバイアスと方位に押し込んで、遅れと関係ない誤差が残る）
    const int n = (int)(60 * fs);
    static double psi_hist[60 * 1000];
    double psi = 0.0;
    double se = 0, se2 = 0, sr = 0; int ne = 0;
    int pend = -1; double pend_t = 0; int k = 0;     // 遅らせて渡す 1 回
    auto psi_at = [&](double tq, int i_now) {        // 過去の ψ（直線でつなぐ）
        if (tq <= 0) return 0.0;
        const double x = tq / dt; int j = (int)x; if (j >= i_now) return psi_hist[i_now];
        return psi_hist[j] + (psi_hist[j + 1] - psi_hist[j]) * (x - j);
    };
    for (int i = 0; i < n; i++) {
        const double t = i * dt;
        const double ramp = 3.0;
        const double u = t < ramp ? t / ramp : 1.0;
        const double sm = u * u * (3 - 2 * u), smd = t < ramp ? 6 * u * (1 - u) / ramp : 0.0;
        const double ph = phi * sm, phd = phi * smd;
        const double psid = -G * tan(ph) / V;
        psi_hist[i] = psi;
        const double c = cos(psi), s = sin(psi);
        const double R[3][3] = {{c, -s * cos(ph),  s * sin(ph)},
                                {s,  c * cos(ph), -c * sin(ph)},
                                {0,  sin(ph),      cos(ph)}};
        const double aw[3] = {-V * psid * s, V * psid * c, 0};
        const double d[3]  = {aw[0], aw[1], aw[2] + G};
        double fb[3], wb[3];
        for (int r = 0; r < 3; r++) fb[r] = R[0][r] * d[0] + R[1][r] * d[1] + R[2][r] * d[2];
        for (int r = 0; r < 3; r++) wb[r] = R[2][r] * psid;
        wb[0] += phd;                                               // バンクを入れる回転（機体の前後軸まわり）
        float f[3], w[3];
        body_to_sensor(fb, f); body_to_sensor(wb, w);
        attitude_on_accel(f);
        test_set_us((uint32_t)((tbase + t) * 1e6)); attitude_on_gyro(w, g_test_us);

        auto give = [&](double t_meas, uint32_t age) {
            const double pm = psi_at(t_meas - lag_s, i);            // GNSS が出す速度は lag_s だけ前のもの
            attitude_on_gnss_velocity((float)(V * sin(pm)), (float)(V * cos(pm)), 0.0f, 0.08f, age);
        };
        if (pend >= 0 && t >= pend_t + jitter_s) {                  // 待たされた回をいま渡す
            give(pend_t, age_us ? age_us + (age_has_jitter ? (uint32_t)(jitter_s * 1e6) : 0) : 0);
            pend = -1;
        }
        if (i % (int)(fs / 4) == 0) {                               // 4Hz
            if (jitter_s > 0 && (k++ % 2) == 1) { pend = i; pend_t = t; }
            else give(t, age_us);
        }
        if (t > 30.0 && i % (int)(fs / 4) == 0) {
            float roll, pitch, yaw; attitude_get_euler_raw(roll, pitch, yaw);
            double want = 90.0 - psi * 180.0 / M_PI;                // 真方位（北 0・時計回り）
            double e = fmod(yaw - want + 180.0 + 360.0 * 100, 360.0) - 180.0;
            se += e; se2 += e * e; sr += roll - bank_deg; ne++;
            if (getenv("LAGDBG") && ne % 20 == 1) { float bg[3]; attitude_get_gyro_bias(bg); printf("    t=%.1f yawerr=%+.2f roll=%.2f pitch=%.2f bg=%.3f %.3f %.3f deg/s ysig=%.2f\n", t, e, roll, pitch, bg[0]*57.3, bg[1]*57.3, bg[2]*57.3, attitude_get_yaw_sigma_deg()); }
        }
        psi += psid * dt;
    }
    return { se / ne, sqrt(se2 / ne), sr / ne };
}

int main() {
    const double bank = 10.0;                       // 旋回率は約 12.4 deg/s
    const double rate = G * tan(bank * M_PI / 180.0) / 8.0 * 180.0 / M_PI;
    printf("協調旋回 バンク %.0f 度・8 m/s（旋回率 %.1f deg/s）。GNSS 速度は 4Hz\n", bank, rate);
    const Result a = run(bank, 0.0, 0, 0.0, false);
    const Result b = run(bank, 0.15, 0, 0.0, false);
    const Result c = run(bank, 0.15, 150000, 0.0, false);
    const Result d = run(bank, 0.15, 150000, 0.10, false);
    const Result e = run(bank, 0.15, 150000, 0.10, true);
    printf("  遅れなし                         : 方位の誤差 平均 %+.2f 度（RMS %.2f）  ロール %+.2f 度\n", a.yaw_err_mean, a.yaw_err_rms, a.roll_err_mean);
    printf("  150ms 遅れ・補償なし             : 方位の誤差 平均 %+.2f 度（RMS %.2f）  ロール %+.2f 度（旋回率 × 遅れ = %.2f 度）\n", b.yaw_err_mean, b.yaw_err_rms, b.roll_err_mean, rate * 0.15);
    printf("  150ms 遅れ・補償あり             : 方位の誤差 平均 %+.2f 度（RMS %.2f）  ロール %+.2f 度\n", c.yaw_err_mean, c.yaw_err_rms, c.roll_err_mean);
    printf("  ＋1 回おきに 100ms 待たされる・待ちは補償しない : 平均 %+.2f 度（RMS %.2f）\n", d.yaw_err_mean, d.yaw_err_rms);
    printf("  ＋1 回おきに 100ms 待たされる・待ちも補償する   : 平均 %+.2f 度（RMS %.2f）\n", e.yaw_err_mean, e.yaw_err_rms);
    bool ok = true;
    ok &= fabs(a.yaw_err_mean) < 0.3;                               // もとから合っている
    ok &= fabs(b.yaw_err_mean) > 0.6 * rate * 0.15;                 // 補償なしでは遅れが方位に出る（試験が効いている証拠）
    ok &= fabs(c.yaw_err_mean) < 0.3 && fabs(c.roll_err_mean) < 0.5;   // 補償で消える
    ok &= fabs(e.yaw_err_mean) < 0.3;                               // 待たされた回も消える
    ok &= fabs(d.yaw_err_mean) > fabs(e.yaw_err_mean) + 0.2;        // 待ちを補償しないと残る
    printf("\n結果: %s\n", ok ? "PASS — GNSS 速度の遅れ（一定のぶんと、待たされたぶん）が補償で消える" : "FAIL");
    return ok ? 0 : 1;
}
