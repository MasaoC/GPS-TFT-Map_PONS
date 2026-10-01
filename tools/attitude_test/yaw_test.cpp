// 0.983 で GRV / RV への依存を外したときに入れたテスト。
//
// 検証するもの
//   A) 加速度だけで水平化できるか（GRV なし）。上方向が測った比力と平行か
//   B) GNSS 航跡でヨーを入れると、要求方位に一致し **ロール・ピッチが動かない**か
//   C) ヨー未設定のあいだ GNSS 速度観測が入らないか（入るとロール・ピッチが汚れる）
//   D) 待機中の維持 — ジャイロにオフセットがあっても 10 分間ロールが保たれるか、
//      静止中に機体を傾け直したら追従するか（再水平化が効いているか）
//
// 旧 decl_test.cpp（地磁気偏角の符号）はここに置き換わった。
// 初期ヨーに地磁気を使わなくなったので、あのテストは対象そのものが無くなった。
#include "attitude.h"
#include <cstdio>
#include <cmath>
#include <cstring>

static const double G = 9.80665;

// ---- 機体座標(前/左/上) → センサー座標 ----
// main.cpp と同一。R_bs = [[0,0,1],[0,1,0],[-1,0,0]]
static void body_to_sensor(const double b[3], float s_out[3]) {
    s_out[0] = (float)(-b[2]);
    s_out[1] = (float)( b[1]);
    s_out[2] = (float)( b[0]);
}

// バンク角 phi で静止しているときのセンサー座標の比力。
//   R = Rx(phi) （ヨー 0・ピッチ 0）、f_body = R^T * (0,0,G) = (0, G sin, G cos)
static void static_accel_for_bank(double phi, float out[3]) {
    const double fb[3] = { 0.0, G * sin(phi), G * cos(phi) };
    body_to_sensor(fb, out);
}

static void feed(int n, double dt_s, uint32_t &t_us,
                 const float a[3], const float g[3]) {
    for (int i = 0; i < n; i++) {
        t_us += (uint32_t)(dt_s * 1e6);
        test_set_us(t_us);
        attitude_on_accel(a);
        attitude_on_gyro(g, g_test_us);
    }
}

int main() {
    bool ok = true;
    const double fs = 50.0, dt = 1.0 / fs;
    const double phi = 20.0 * M_PI / 180.0;

    // ================= A) 加速度だけで水平化 =================
    attitude_setup();
    uint32_t t = 0;
    float a20[3]; static_accel_for_bank(phi, a20);
    const float gz[3] = {0, 0, 0};
    feed(200, dt, t, a20, gz);                 // 4 秒静止（初期化は 2 秒）

    float r, p, y;
    attitude_get_euler_raw(r, p, y);
    printf("A) GRV なしで水平化\n");
    printf("   ready=%d (1 が正しい)  yaw_ready=%d (0 が正しい)\n",
           (int)attitude_ready(), (int)attitude_yaw_ready());
    printf("   出力ロール %.2f deg（真値 %.2f）  ピッチ %.2f deg（真値 0）\n",
           r, phi * 180 / M_PI, p);
    ok &= attitude_ready() && !attitude_yaw_ready();
    ok &= fabsf(r - (float)(phi * 180 / M_PI)) < 0.5f;
    ok &= fabsf(p) < 0.5f;

    // 上方向が測った比力と平行か（座標系の約束に依らない検証）
    float u[3];
    const bool have_up = attitude_get_up_sensor(u);
    const float an = sqrtf(a20[0]*a20[0] + a20[1]*a20[1] + a20[2]*a20[2]);
    float dot = 0;
    for (int i = 0; i < 3; i++) dot += u[i] * a20[i] / an;
    printf("   上方向と比力のなす角 %.3f deg（0 が正しい）\n",
           acosf(dot > 1.0f ? 1.0f : dot) * 180 / (float)M_PI);
    ok &= have_up && dot > 0.99999f;

    // ヨーが未知のあいだ 95% 値が 180 度に張り付いているか（表示の灰色化の条件）
    printf("   yaw_acc95 = %.0f deg（180 が正しい＝表示側が信頼しない）\n",
           attitude_get_yaw_acc95_deg());
    ok &= attitude_get_yaw_acc95_deg() > 179.0f;
    ok &= attitude_get_yaw_source() == ATT_YAW_SRC_NONE;

    // ================= C) ヨー未設定なら速度観測を入れない =================
    // （B より先にやる。B でヨーを入れてしまうと確認できない）
    // 姿勢と食い違う速度を 10 秒ぶん流しても、ロール・ピッチが動かないこと。
    // しきい値未満（2m/s）にしてヨー注入も起こらないようにする。
    {
        float r0, p0, y0; attitude_get_euler_raw(r0, p0, y0);
        for (int i = 0; i < 20; i++) {
            feed(25, dt, t, a20, gz);                       // 0.5 秒
            attitude_on_gnss_velocity(2.0f, 0.0f, 0.0f, 0.08f);   // 北へ 2m/s
        }
        attitude_get_euler_raw(r, p, y);
        printf("\nC) ヨー未設定のあいだ速度観測を通さない\n");
        printf("   yaw_ready=%d (0 が正しい)  GNSS 観測回数 %u (0 が正しい)\n",
               (int)attitude_yaw_ready(), (unsigned)attitude_get_gnss_updates());
        printf("   ロール %.2f→%.2f  ピッチ %.2f→%.2f（変化なしが正しい）\n",
               r0, r, p0, p);
        ok &= !attitude_yaw_ready() && attitude_get_gnss_updates() == 0;
        ok &= fabsf(r - r0) < 0.2f && fabsf(p - p0) < 0.2f;
    }

    // ================= B) GNSS 航跡でヨーを入れる =================
    {
        float r0, p0, y0; attitude_get_euler_raw(r0, p0, y0);
        // 真方位 300 度へ 8m/s。ENU で East = 8 sin300, North = 8 cos300
        const double trk = 300.0 * M_PI / 180.0;
        const float vN = (float)(8.0 * cos(trk)), vE = (float)(8.0 * sin(trk));
        for (int i = 0; i < ESKF_YAW_INIT_ARM_FIXES; i++) {
            feed(25, dt, t, a20, gz);
            attitude_on_gnss_velocity(vN, vE, 0.0f, 0.08f);   // NED で渡す
        }
        attitude_get_euler_raw(r, p, y);
        printf("\nB) GNSS 航跡でヨーを入れる\n");
        printf("   yaw_ready=%d (1 が正しい)  yaw_source=%d (1=GNSS が正しい)\n",
               (int)attitude_yaw_ready(), (int)attitude_get_yaw_source());
        printf("   出力方位 %.2f deg（要求 300.00）  差 %+.3f\n", y, y - 300.0f);
        printf("   ロール %.2f→%.2f  ピッチ %.2f→%.2f（変化なしが正しい）\n", r0, r, p0, p);
        printf("   yaw_acc95 = %.1f deg（2σ = %.1f）\n",
               attitude_get_yaw_acc95_deg(), 2.0f * ESKF_YAW_INIT_SIGMA_DEG);
        ok &= attitude_yaw_ready() && attitude_get_yaw_source() == ATT_YAW_SRC_GNSS;
        ok &= fabsf(y - 300.0f) < 0.05f;
        ok &= fabsf(r - r0) < 0.01f && fabsf(p - p0) < 0.01f;
    }

    // ================= D) 待機中の維持 =================
    // D1) ジャイロに 0.2 deg/s のオフセットを乗せて 10 分静止させる。
    //     再水平化とバイアス学習が無ければロールは 0.2*600 = 120 度漂う。
    {
        attitude_setup();
        t = 0;
        const float bias = (float)(0.2 * M_PI / 180.0);   // 0.2 deg/s（クランプ 0.5 未満）
        const float gb[3] = { bias, 0.0f, 0.0f };          // センサー X 軸まわり
        feed(200, dt, t, a20, gb);                         // 初期化
        attitude_get_euler_raw(r, p, y);
        const float r_init = r;
        feed((int)(600 * fs), dt, t, a20, gb);             // 10 分待機
        attitude_get_euler_raw(r, p, y);
        float bg[3]; attitude_get_gyro_bias(bg);
        printf("\nD1) 0.2deg/s のオフセットで 10 分静止\n");
        printf("   ロール %.2f → %.2f deg（変化 %+.3f。対策なしなら約 120 度）\n",
               r_init, r, r - r_init);
        printf("   学習したバイアス %.4f %.4f %.4f deg/s（X が 0.200 に近いのが正しい）\n",
               bg[0] * 180 / M_PI, bg[1] * 180 / M_PI, bg[2] * 180 / M_PI);
        ok &= fabsf(r - r_init) < 1.0f;
        ok &= fabsf(bg[0] * 180 / (float)M_PI - 0.2f) < 0.05f;

        // D2) 静止中に機体を傾け直したら追従するか（再水平化の直接確認）
        float a10[3]; static_accel_for_bank(10.0 * M_PI / 180.0, a10);
        feed((int)(10 * fs), dt, t, a10, gb);              // 10 秒（時定数 1 秒）
        attitude_get_euler_raw(r, p, y);
        printf("\nD2) 静止中にバンクを 20→10 度へ変えた\n");
        printf("   出力ロール %.2f deg（10.00 が正しい。再水平化が無ければ 20 のまま）\n", r);
        ok &= fabsf(r - 10.0f) < 0.5f;

        // D3) バイアスがクランプを超えたら頭打ちになるか
        attitude_setup();
        t = 0;
        const float big = (float)(1.2 * M_PI / 180.0);     // 1.2 deg/s（静止判定 1.5 未満）
        const float gbig[3] = { 0.0f, big, 0.0f };
        feed(1000, dt, t, a20, gbig);                      // 20 秒
        attitude_get_gyro_bias(bg);
        const float lim = ESKF_BIAS_LEARN_MAX_RADS * 180.0f / (float)M_PI;
        printf("\nD3) 1.2deg/s を学習させる（クランプ %.2f deg/s）\n", lim);
        printf("   学習値 Y = %.3f deg/s（クランプで頭打ちが正しい）\n", bg[1] * 180 / M_PI);
        ok &= fabsf(bg[1] * 180 / (float)M_PI) <= lim + 1e-3f;
    }

    printf("\n結果: %s\n", ok ? "PASS — 加速度水平化・航跡ヨー・待機中の維持がすべて正しい"
                             : "FAIL");
    return ok ? 0 : 1;
}
