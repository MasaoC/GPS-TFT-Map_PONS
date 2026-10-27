// 0.983 で GRV / RV への依存を外したときに入れたテスト。
//
// 検証するもの
//   A) 加速度だけで水平化できるか（GRV なし）。上方向が測った比力と平行か
//   B) GNSS 航跡でヨーを入れると、要求方位に一致し **ロール・ピッチが動かない**か
//   C) ヨー未設定のあいだ GNSS 速度観測が入らないか（入るとロール・ピッチが汚れる）
//   D) 待機中の維持 — ジャイロにオフセットがあっても 10 分間ロールが保たれるか、
//      静止中に機体を傾け直したら追従するか（再水平化が効いているか）
//   E) **水平化できない状況でもバリオ用の上方向が出るか**（実機回帰の再発防止）
//   F) 重力観測（Phase 2）— 直進では入り、**5 度の協調旋回では入らない**こと、
//      GNSS 無しでもロール・ピッチの錨になること
//   G) 2026-10-27 の発散（VertAccel -18 m/s²）の再発防止 — 悪い測位でヨーを入れない、
//      速く回している間は重力観測を入れない、ヨーが決まった後も地上で止まれば戻る
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
        printf("   学習値 Y = %.3f deg/s（クランプ %.2f で頭打ち。1.2 にならないこと）\n",
               bg[1] * 180 / M_PI, lim);
        // ★ 厳密に lim 以下にはならない。重力観測（Phase 2）が相関経由で bg_ を
        //   微調整するため（実測で +0.003 deg/s 程度）。見たいのは
        //   「学習器がクランプしている」ことなので、生値 1.2 から大きく離れていればよい。
        ok &= fabsf(bg[1] * 180 / (float)M_PI) <= lim * 1.2f;
    }


    // ================= E) 水平化できなくてもバリオ用の上方向が出るか =================
    // ★★ **これは実機で起きた回帰の再発防止テスト。**
    //   0.983 は attitude_get_up_sensor() を「水平化済みのみ true」にしていた。
    //   手持ちで静止 2 秒が取れないと水平化できず、バリオの predict が 1 度も回らない。
    //   バリオ KF の初期 P は対角なので predict を通らないと P[1][0] が 0 のままで、
    //   気圧観測のゲイン K[1] = P[1][0]/S が 0 になり **V/S が構造的に 0 で固まる**。
    //   2026-10-01 にインドアテスト（38 秒・|ω| 中央 5.1deg/s）で発覚。
    //   → 水平化前でも加速度の低域から上方向を返すこと。
    {
        attitude_setup();
        t = 0;
        // 静止させない（静止判定を通さない）。1.5deg/s を超える角速度を与え続ける。
        const float spin[3] = { (float)(5.0 * M_PI / 180.0), 0.0f, 0.0f };
        float u0[3];
        const bool before_any = attitude_get_up_sensor(u0);
        feed(400, dt, t, a20, spin);          // 8 秒。静止しないので水平化はされない
        float u2[3];
        const bool have = attitude_get_up_sensor(u2);
        const float an2 = sqrtf(a20[0]*a20[0] + a20[1]*a20[1] + a20[2]*a20[2]);
        float dot2 = 0;
        for (int i = 0; i < 3; i++) dot2 += u2[i] * a20[i] / an2;
        const float ang2 = acosf(dot2 > 1.0f ? 1.0f : (dot2 < -1.0f ? -1.0f : dot2))
                           * 180.0f / (float)M_PI;
        printf("\nE) 水平化できない状況でも上方向が出るか（実機回帰の再発防止）\n");
        printf("   加速度が来る前 = %d（0 が正しい）\n", (int)before_any);
        printf("   ready=%d（0 = 静止しないので水平化はされない。それが正しい）\n",
               (int)attitude_ready());
        printf("   上方向あり=%d（1 が正しい）  比力とのなす角 %.2f deg\n",
               (int)have, ang2);
        ok &= !before_any;            // まだ加速度が来ていない時点では false
        ok &= !attitude_ready();      // 水平化はされていない
        ok &= have;                   // ★ それでも上方向は返ること
        ok &= ang2 < 2.0f;            // 低域なので比力とほぼ平行
    }

    // ================= F) 重力観測（Phase 2）=================
    // ★ **ゲートのゲート。** 無条件に入れると BNO085 の GRV と同じ間違いをする
    //   （協調旋回では比力の横成分が厳密に 0 なのでバンクを過小評価する）。
    //   F1: 直進・静止では観測が入ること
    //   F2: **5 度の協調旋回では入らないこと**（人力機の実運用バンク角）
    //   F3: GNSS が無くてもロール・ピッチの錨になること
    {
        // ---- F1) 静止・直進では観測が入る ----
        attitude_setup();
        t = 0;
        feed(200, dt, t, a20, gz);                  // 水平化
        const uint32_t n0 = attitude_get_level_updates();
        feed((int)(10 * fs), dt, t, a20, gz);       // 10 秒
        const uint32_t n1 = attitude_get_level_updates();
        printf("\nF1) 静止・直進では重力観測が入る\n");
        printf("   10 秒で %u 回（2Hz なので 20 回前後が正しい）\n", (unsigned)(n1 - n0));
        ok &= (n1 - n0) >= 15 && (n1 - n0) <= 25;

        // ---- F2) 5 度バンク相当の旋回レートでは入らない ----
        // ★ ここで見たいのは**ゲートが旋回レートで閉じること**だけなので、
        //   比力は水平のまま、ワールド系ヨーレートだけ協調旋回の値を与える。
        //   バンク復元まで含めた厳密な協調旋回は main.cpp が担当（20 度で検証）。
        // 旋回レート = g*tan(phi)/V。V=8, phi=5 → 6.14 deg/s（しきい値 1.0/3.0）
        const double V = 8.0, phi5 = 5.0 * M_PI / 180.0;
        const double psid = -G * tan(phi5) / V;     // [rad/s]
        const uint32_t n2 = attitude_get_level_updates();
        // 機体が水平のまま鉛直軸まわりに psid で回る（＝ワールド系ヨーレート psid）
        const double wb[3] = { 0.0, 0.0, psid };
        float wv[3]; body_to_sensor(wb, wv);
        float a_lv[3]; static_accel_for_bank(0.0, a_lv);
        feed((int)(30 * fs), dt, t, a_lv, wv);
        const uint32_t n3 = attitude_get_level_updates();
        printf("\nF2) 5 度バンク相当の旋回レート（%.2f deg/s）では入らない\n",
               fabs(psid) * 180 / M_PI);
        printf("   30 秒で %u 回（**0 が正しい**。入るとバンクを過小評価する）\n",
               (unsigned)(n3 - n2));
        ok &= (n3 - n2) == 0;

        // ---- F3) GNSS 無しでもロール・ピッチの錨になる ----
        // 静止判定を通さないため |a| を g から 0.4 ずらす（重力観測のゲートは 0.5 まで許す）。
        // これで「再水平化（静止時のみ）」は働かず、重力観測だけが錨になる。
        attitude_setup();
        t = 0;
        float a_off[3];
        static_accel_for_bank(phi, a_off);                 // バンク 20 度の向き
        const float sc = (float)((G + 0.4) / G);
        for (int i = 0; i < 3; i++) a_off[i] *= sc;        // |a| = g + 0.4 → 静止判定を外す
        const float gb3[3] = { (float)(0.5 * M_PI / 180.0), 0.0f, 0.0f };  // 0.5deg/s
        feed(200, dt, t, a20, gz);                         // まず静止で水平化
        attitude_get_euler_raw(r, p, y);
        const float r0 = r;
        feed((int)(120 * fs), dt, t, a_off, gb3);          // 2 分（GNSS は一切入れない）
        attitude_get_euler_raw(r, p, y);
        printf("\nF3) GNSS 無し・静止判定も外した状態で 2 分\n");
        printf("   ロール %.2f → %.2f deg（錨が無ければ 0.5*120 = 60 度漂う）\n", r0, r);
        printf("   重力観測 %u 回\n", (unsigned)attitude_get_level_updates());
        ok &= fabsf(r - r0) < 2.0f;
        ok &= attitude_get_level_updates() > 100;
    }

    // ================= G) 2026-10-27 の発散の再発防止 =================
    // ★ 実機で VertAccel が -18 m/s² になり、バリオが数分沈降で鳴り続けた。
    //   バリオの上方向は ESKF から取るので、姿勢が上下逆さまになると鉛直加速度も狂う。
    //   手で 90 度回している間に重力観測が通って bg_ が壊れ、屋内の偽の速度でヨーが
    //   決まり、そのあと傾きの誤差が 60 度を超えて戻せなくなった。
    //   G1: sAcc の悪い測位ではヨーを入れない
    //   G2: 水平な軸まわりに速く回している間は重力観測を入れない
    //   G3: ヨーが決まった後でも、地上で止まれば 60 度を超える傾きの誤差から戻る
    //   G4: GNSS が「動いている」と言っている間は G3 が働かない（飛行中の誤作動の防止）
    {
        float a_lv[3]; static_accel_for_bank(0.0, a_lv);   // 機体水平
        // 上方向と比力のなす角 [deg]（0 が正しい）
        auto up_err_deg = [](const float a[3]) {
            float uu[3];
            if (!attitude_get_up_sensor(uu)) return 999.0f;
            const float n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            float d = 0;
            for (int i = 0; i < 3; i++) d += uu[i] * a[i] / n;
            if (d > 1.0f) d = 1.0f;
            if (d < -1.0f) d = -1.0f;
            return acosf(d) * 180.0f / (float)M_PI;
        };
        // 良い測位でヨーを入れる（真方位 0 度へ 8m/s）
        auto set_yaw_good = [&]() {
            for (int i = 0; i < ESKF_YAW_INIT_ARM_FIXES; i++) {
                feed(25, dt, t, a_lv, gz);
                attitude_on_gnss_velocity(8.0f, 0.0f, 0.0f, 0.08f);
            }
        };

        // ---- G1) sAcc の悪い測位ではヨーを入れない ----
        attitude_setup();
        t = 0;
        feed(200, dt, t, a_lv, gz);                       // 水平化
        for (int i = 0; i < 6; i++) {
            feed(25, dt, t, a_lv, gz);
            attitude_on_gnss_velocity(3.8f, 0.0f, 0.0f, 2.06f);   // 実機の屋内の値
        }
        const bool yaw_bad = attitude_yaw_ready();
        for (int i = 0; i < ESKF_YAW_INIT_ARM_FIXES; i++) {
            feed(25, dt, t, a_lv, gz);
            attitude_on_gnss_velocity(3.8f, 0.0f, 0.0f, 0.30f);
        }
        printf("\nG1) sAcc の悪い測位ではヨーを入れない\n");
        printf("   sAcc 2.06 を 6 回: yaw_ready=%d（0 が正しい）\n", (int)yaw_bad);
        printf("   続けて sAcc 0.30 を %d 回: yaw_ready=%d（1 が正しい）\n",
               ESKF_YAW_INIT_ARM_FIXES, (int)attitude_yaw_ready());
        ok &= !yaw_bad && attitude_yaw_ready();

        // ---- G2) 水平な軸まわりに速く回している間は重力観測を入れない ----
        // 機体のピッチ軸（水平のまま）まわりに 20deg/s。ワールド系のヨーレートは 0 なので
        // 旋回レートのゲートは開いたまま。比力は動かさない（回転に伴う加速度の代わり）。
        attitude_setup();
        t = 0;
        feed(200, dt, t, a_lv, gz);
        float bg0[3]; attitude_get_gyro_bias(bg0);
        const double wb2[3] = { 0.0, 20.0 * M_PI / 180.0, 0.0 };
        float wv2[3]; body_to_sensor(wb2, wv2);
        const uint32_t m0 = attitude_get_level_updates();
        feed((int)(1.5 * fs), dt, t, a_lv, wv2);         // 1.5 秒（30 度）
        const uint32_t m1 = attitude_get_level_updates();
        float bg1[3]; attitude_get_gyro_bias(bg1);
        float dbg = 0;
        for (int i = 0; i < 3; i++) dbg += (bg1[i] - bg0[i]) * (bg1[i] - bg0[i]);
        dbg = sqrtf(dbg) * 180.0f / (float)M_PI;
        feed((int)(3 * fs), dt, t, a_lv, gz);            // 止まって 3 秒
        const uint32_t m2 = attitude_get_level_updates();
        printf("\nG2) 水平な軸まわりに 20deg/s で回している間は重力観測を入れない\n");
        printf("   回している 1.5 秒で %u 回（0 が正しい）  bg の変化 %.3f deg/s（0 が正しい）\n",
               (unsigned)(m1 - m0), dbg);
        printf("   止まってから 3 秒で %u 回（0.5 秒待ってから 2Hz で入るのが正しい）\n",
               (unsigned)(m2 - m1));
        ok &= (m1 - m0) == 0 && dbg < 0.01f;
        ok &= (m2 - m1) >= 4;

        // ---- G3) ヨーが決まった後でも、地上で止まれば大きな誤差から戻る ----
        attitude_setup();
        t = 0;
        feed(200, dt, t, a_lv, gz);
        set_yaw_good();
        const bool yaw3 = attitude_yaw_ready();
        // 傾きの推定だけを 120 度狂わせる。ジャイロだけ 60deg/s で 2 秒回し、比力は水平のまま。
        // 実機の「狂った bg_ のまま積分した」状態の代わり。|ω| が大きいので重力観測は入らない。
        const double wb3[3] = { 0.0, 60.0 * M_PI / 180.0, 0.0 };
        float wv3[3]; body_to_sensor(wb3, wv3);
        feed((int)(2 * fs), dt, t, a_lv, wv3);
        const float e_before = up_err_deg(a_lv);
        const uint32_t g0 = attitude_get_ground_relevels();
        feed((int)(10 * fs), dt, t, a_lv, gz);           // 止まって 10 秒。GNSS は来ない
        const float e_after = up_err_deg(a_lv);
        const uint32_t g1 = attitude_get_ground_relevels();
        printf("\nG3) ヨーが決まった後に傾きの推定が 120 度狂った状態で、止まって 10 秒\n");
        printf("   yaw_ready=%d（1 が正しい）  上方向の誤差 %.1f → %.2f deg（2 未満が正しい）\n",
               (int)yaw3, e_before, e_after);
        printf("   地上での再水平化 %u 回（1 が正しい）  yaw_ready=%d  yaw_acc95=%.1f deg（ヨーを捨てないこと）\n",
               (unsigned)(g1 - g0), (int)attitude_yaw_ready(), attitude_get_yaw_acc95_deg());
        ok &= yaw3 && e_before > 100.0f && e_after < 2.0f;
        ok &= (g1 - g0) == 1;
        ok &= attitude_yaw_ready() && attitude_get_yaw_acc95_deg() < 179.0f;

        // ---- G4) GNSS が「動いている」と言っている間は働かない ----
        // 止まっているのに GNSS は 8m/s（飛行中に静止判定が成立した場合の代わり）。
        attitude_setup();
        t = 0;
        feed(200, dt, t, a_lv, gz);
        set_yaw_good();
        const uint32_t h0 = attitude_get_ground_relevels();
        for (int i = 0; i < 20; i++) {                   // 10 秒
            feed(25, dt, t, a_lv, gz);
            attitude_on_gnss_velocity(8.0f, 0.0f, 0.0f, 0.08f);
        }
        const uint32_t h1 = attitude_get_ground_relevels();
        for (int i = 0; i < 6; i++) {                    // GNSS が 0.3m/s に変わった
            feed(25, dt, t, a_lv, gz);
            attitude_on_gnss_velocity(0.3f, 0.0f, 0.0f, 0.08f);
        }
        const uint32_t h2 = attitude_get_ground_relevels();
        printf("\nG4) GNSS が 8m/s と言っている間は地上の再水平化をしない\n");
        printf("   静止 10 秒・GNSS 8m/s: %u 回（0 が正しい）  GNSS 0.3m/s へ変わったら %u 回（1 が正しい）\n",
               (unsigned)(h1 - h0), (unsigned)(h2 - h1));
        ok &= (h1 - h0) == 0 && (h2 - h1) == 1;
    }
    printf("\n結果: %s\n", ok ? "PASS — 水平化・航跡ヨー・待機中の維持・上方向の常時提供・重力観測・発散の再発防止がすべて正しい"
                             : "FAIL");
    return ok ? 0 : 1;
}
