// 水平化・航跡ヨー・待機中の維持のテスト。
//
// 検証するもの
//   A) 加速度だけで水平化できるか。上方向が測った比力と平行か
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
#include <vector>
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
    const double fs = (double)IMU_RATE_GYRO_HZ, dt = 1.0 / fs;
    const double phi = 20.0 * M_PI / 180.0;

    // ================= A) 加速度だけで水平化 =================
    attitude_setup();
    uint32_t t = 0;
    float a20[3]; static_accel_for_bank(phi, a20);
    const float gz[3] = {0, 0, 0};
    feed((int)(4 * fs), dt, t, a20, gz);                 // 4 秒静止（初期化は 2 秒）

    float r, p, y;
    attitude_get_euler_raw(r, p, y);
    printf("A) 加速度だけで水平化\n");
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
    // しきい値未満（1.5m/s。2 m/s 以上は 2D 相関だけで入りうる＝ESKF_YAW_INIT_CORR_MIN_SPEED_MPS）にしてヨー注入も起こらないようにする。
    {
        float r0, p0, y0; attitude_get_euler_raw(r0, p0, y0);
        for (int i = 0; i < 20; i++) {
            feed((int)(0.5 * fs), dt, t, a20, gz);                       // 0.5 秒
            attitude_on_gnss_velocity(1.5f, 0.0f, 0.0f, 0.08f);   // 北へ 1.5m/s
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
            feed((int)(0.5 * fs), dt, t, a20, gz);
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
        feed((int)(4 * fs), dt, t, a20, gb);               // 初期化
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
        feed((int)(20 * fs), dt, t, a20, gbig);            // 20 秒
        attitude_get_gyro_bias(bg);
        const float lim = ESKF_BIAS_LEARN_MAX_RADS * 180.0f / (float)M_PI;
        printf("\nD3) 1.2deg/s を学習させる（クランプ %.2f deg/s）\n", lim);
        printf("   学習値 Y = %.3f deg/s（クランプ %.2f で頭打ち。1.2 にならないこと）\n",
               bg[1] * 180 / M_PI, lim);
        // ★ 厳密に lim 以下にはならない。重力観測（Phase 2）が相関経由で bg_ を
        //   微調整するため（実測で +0.003 deg/s 程度）。見たいのは
        //   「学習器がクランプしている」ことなので、生値 1.2 から大きく離れていればよい。
        ok &= fabsf(bg[1] * 180 / (float)M_PI) <= lim * 1.2f;

        // D4) 揺れている間はゼロ点を覚えない（2026-11-18。ESKF_BIAS_LEARN_STD_RADS）
        //   自転車を人が支えて揺れていた 6 分（1.5〜2 deg/s）の平均を覚え、ヨー軸のゼロ点が 0.06 deg/s ずれた件の再発防止。
        //   0.1 deg/s のゼロ点に ±1.0 deg/s・0.5Hz の揺れを乗せて 30 秒（|ω| < 1.5 なので静止の判定は通る）→ 覚えない。
        //   揺れが止まったら 10 秒で 0.1 を覚える。
        attitude_setup();
        t = 0;
        const float off = (float)(0.1 * M_PI / 180.0);
        const float g0[3] = { off, 0.0f, 0.0f };
        feed((int)(4 * fs), dt, t, a20, g0);               // 初期化（ここで 0.1 を覚えることがある）
        // 覚えた値を消して、揺れの間に覚えるかだけを見る
        attitude_setup();
        t = 0;
        {
            const int n = (int)(30 * fs);
            for (int i = 0; i < n; i++) {
                const float sway = (float)(1.0 * M_PI / 180.0 * sin(2.0 * M_PI * 0.5 * i * dt));
                const float gs[3] = { off + sway, 0.0f, 0.0f };
                feed(1, dt, t, a20, gs);
            }
        }
        attitude_get_gyro_bias(bg);
        const float bg_sway = bg[0] * 180.0f / (float)M_PI;
        feed((int)(10 * fs), dt, t, a20, g0);              // 揺れが止まった
        attitude_get_gyro_bias(bg);
        const float bg_still = bg[0] * 180.0f / (float)M_PI;
        printf("\nD4) 0.1deg/s のゼロ点に ±1deg/s の揺れを乗せて 30 秒 → 揺れが止まって 10 秒\n");
        printf("   揺れの間に覚えた X = %.4f deg/s（0 のままが正しい）  止まってから X = %.4f deg/s（0.100 が正しい）\n",
               bg_sway, bg_still);
        ok &= fabsf(bg_sway) < 0.01f;
        ok &= fabsf(bg_still - 0.1f) < 0.01f;

        // D5) 滑走の加速を傾きと取り違えない（2026-11-18。settings.h の ESKF_ACCEL_* と attitude.cpp の gsp_fresh()）
        //   ＋ 1 回の離陸加速で Y95% が 10 度未満（飛行機アイコン・ドリフト点線の上限）まで縮むこと（ESKF_YAW_CORR_*）
        //   静止 4 秒 → 0→6 m/s を 4 秒（1.5 m/s²）→ 6 m/s で 6 秒。本当のピッチ・ロールは 0。GNSS は 4Hz で東向き。
        //   等加速度ではジャイロが静かで |a| も g に近いので静止の判定を通り、待機中の維持が加速度の向きへ傾きを寄せていた。
        //   重力観測の前後加速度の門も方位が決まる前は外れていた。直す前はピッチが 8.7 度（atan(1.5/g)）ずれた。
        attitude_setup();
        t = 0;
        {
            const float gq[3] = {0, 0, 0};
            float pmax = 0.0f, rmax = 0.0f;
            float y95_at_init = 180.0f, y95_at_8s = 180.0f, y_at_init = 0.0f;
            bool yaw_seen = false;
            const int n = (int)(14.0 * fs);
            for (int i = 0; i < n; i++) {
                const double tt = i * dt;
                double a = 0, v = 0;
                if (tt >= 4.0 && tt < 8.0) { a = 1.5; v = 1.5 * (tt - 4.0); }
                else if (tt >= 8.0)        { v = 6.0; }
                const double ab[3] = { a, 0.0, G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, gq);
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, (float)v, 0.0f, 0.10f);
                if (tt >= 4.0) {
                    float r, p, y; attitude_get_euler(r, p, y);
                    if (fabsf(p) > pmax) pmax = fabsf(p);
                    if (fabsf(r) > rmax) rmax = fabsf(r);
                    if (!yaw_seen && attitude_yaw_ready()) {
                        yaw_seen = true;
                        y95_at_init = attitude_get_yaw_acc95_deg();
                        y_at_init = y;
                    }
                    if (i == (int)(8.0 * fs)) y95_at_8s = attitude_get_yaw_acc95_deg();
                }
            }
            printf("\nD5) 静止 → 0→6 m/s を 4 秒で加速 → 6 m/s（本当のピッチ 0・真方位 90 度）\n");
            printf("   滑走から 10 秒の間のピッチの最大 %.2f deg・ロールの最大 %.2f deg（0.5 未満が正しい。直す前は 8.7）  方位 %d（1 が正しい）\n",
                   pmax, rmax, (int)yaw_seen);
            printf("   2D 相関注入時: 方位 %.2f deg（90.0 が正しい）  Y95%% = %.1f deg → 加速終了時(8s) Y95%% = %.1f deg（10.0 未満が正しい。旧方式は 16.9）\n",
                   y_at_init, y95_at_init, y95_at_8s);
            ok &= pmax < 0.5f && rmax < 0.5f && yaw_seen;
            ok &= fabsf(y_at_init - 90.0f) < 0.5f && y95_at_init < 10.0f && y95_at_8s < 10.0f;
        }

        // D6) 常時前向き 2D 相関: プラットホーム手押し誤蓄積のリセットと、取付オフセット（12 度）下での真の方位抽出
        //   静止 4 秒 → 南向き(180°)に 1.8 m/s まで手押し 2.0 秒 → 静止 1.5 秒（>= 1.0s で相関と低域が 0 にリセット）
        //   → 東向き(航跡 90.0°)へ 0→6 m/s（1.5 m/s²）で発進するが、本体は右へ 12.0 度（真の方位 102.0°）ずれて付いている。
        //   従来方式（瞬間航跡）では航跡 90.0° がそのまま入って 12° ずれるが、2D 相関は加速度ベクトルを整合させるため
        //   注入の瞬間から真の方位 102.0° が出る。
        attitude_setup();
        t = 0;
        {
            const float gq[3] = {0, 0, 0};
            float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
            feed((int)(4.0 * fs), dt, t, a_lv0, gq);           // 4 秒静止（水平化）
            // 南向き(180°)へ 0.9 m/s² で 2 秒手押し（最大 1.8 m/s < 3.0 m/s）
            for (int i = 0; i < (int)(2.0 * fs); i++) {
                const double v_south = 0.9 * (i * dt);
                const double ab[3] = { 0.9, 0.0, G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, gq);
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity((float)(-v_south), 0.0f, 0.0f, 0.08f);
            }
            const bool yaw_after_push = attitude_yaw_ready();
            // 発進前に 1.5 秒静止（ここで手押し分の相関 C, S と GNSS 低域が 0 にリセットされる）
            for (int i = 0; i < (int)(1.5 * fs); i++) {
                feed(1, dt, t, a_lv0, gq);
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, 0.0f, 0.0f, 0.08f);
            }
            // 東向き(航跡 90.0°)へ 1.5 m/s² で 4 秒加速。本体の方位は 102.0°（右へ +12.0° オフセット）なので、
            // 機体軸(前, 左, 上)での加速度は (a cos12°, +a sin12°, G)
            const double off_rad = 12.0 * M_PI / 180.0;
            float y_init6 = 0.0f, y95_init6 = 180.0f;
            bool seen6 = false;
            for (int i = 0; i < (int)(4.0 * fs); i++) {
                const double a_east = 1.5;
                const double v_east = a_east * (i * dt);
                const double ab[3] = { a_east * cos(off_rad), a_east * sin(off_rad), G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, gq);
                if (i % (int)(fs / 4) == 0) {
                    attitude_on_gnss_velocity(0.0f, (float)v_east, 0.0f, 0.08f);
                    if (!seen6 && attitude_yaw_ready()) {
                        seen6 = true;
                        float r6, p6; attitude_get_euler(r6, p6, y_init6);
                        y95_init6 = attitude_get_yaw_acc95_deg();
                    }
                }
            }
            printf("\nD6) 南へ手押し(1.8m/s) → 1.5秒静止でリセット → 東(航跡 90.0°)へ発進・取付オフセット +12.0°（真の方位 102.0°）\n");
            printf("   手押し後の yaw_ready=%d（0 が正しい）  発進注入時の方位 %.2f deg（102.0 が正しい。旧方式は 90.0）  Y95%% = %.1f deg\n",
                   (int)yaw_after_push, y_init6, y95_init6);
            ok &= !yaw_after_push && seen6;
            ok &= fabsf(y_init6 - 102.0f) < 0.5f && y95_init6 < 10.0f;
        }

        // D7) 相関の方位が航跡と ESKF_YAW_CORR_MAX_TRK_DIFF_DEG（20°）を超えて食い違うときは、航跡＋15° に戻る（2026-11-18）
        //   取付オフセット +30.0°（真の方位 120.0°）で東（航跡 90.0°）へ発進。相関は 120° を出すが、航跡と 30° 違うので使わない。
        //   （11/18 の自転車で、傾きのかたよりで向きが一様に回った +22° の外れがあり、まとまりでは見分けられなかったための門）
        attitude_setup();
        t = 0;
        {
            const float gq[3] = {0, 0, 0};
            float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
            feed((int)(4.0 * fs), dt, t, a_lv0, gq);
            const double off_rad = 30.0 * M_PI / 180.0;
            float y_init7 = 0.0f, y95_init7 = 0.0f;
            bool seen7 = false;
            for (int i = 0; i < (int)(4.0 * fs); i++) {
                const double v_east = 1.5 * (i * dt);
                const double ab[3] = { 1.5 * cos(off_rad), 1.5 * sin(off_rad), G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, gq);
                if (i % (int)(fs / 4) == 0) {
                    attitude_on_gnss_velocity(0.0f, (float)v_east, 0.0f, 0.08f);
                    if (!seen7 && attitude_yaw_ready()) {
                        seen7 = true;
                        float r7, p7; attitude_get_euler(r7, p7, y_init7);
                        y95_init7 = attitude_get_yaw_acc95_deg();
                    }
                }
            }
            uint8_t how7 = 255; float mag7, coh7, td7, sg7; bool bok7;
            const bool ev7 = attitude_take_yaw_init_event(how7, mag7, coh7, td7, sg7, bok7);
            printf("\nD7) 取付オフセット +30°（真の方位 120°）で東（航跡 90°）へ発進 → 航跡と 20° を超えて食い違うので航跡に戻る\n");
            printf("   注入時の方位 %.2f deg（90.0 が正しい）  Y95%% = %.1f deg（30.0 が正しい）  記録 how=%d（3 が正しい）・corr-trk %+.1f deg（+30 前後）\n",
                   y_init7, y95_init7, ev7 ? (int)how7 : -1, td7);
            ok &= seen7 && fabsf(y_init7 - 90.0f) < 0.5f && fabsf(y95_init7 - 30.0f) < 1.0f;
            ok &= ev7 && how7 == 3 && fabsf(td7 - 30.0f) < 2.0f;
        }

        // D8) 静止で覚えたゼロ点を確かめる（2026-11-18。settings.h の ESKF_BIAS_CONFIRM_DIFF_RADS）
        //   ばらつきの門は一定の速さの回転を見分けられないので、前回の値（SD）か続けて通った直前の窓と 0.02 deg/s 以内のときだけ「覚えた」。
        {
            const float b0[3] = { 0.0005f, -0.0003f, 0.00075f };          // 本当のゼロ点 [rad/s]（0.029 / −0.017 / 0.043 deg/s）
            float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
            auto run = [&](const float g[3], double secs, bool set_ref, const float ref[3],
                           int &n_ev, uint8_t hows[4], int &n_save, float saved[3]) {
                attitude_setup();
                t = 0;
                if (set_ref) attitude_set_bias_ref(ref);
                n_ev = 0; n_save = 0;
                for (int i = 0; i < (int)(secs * fs); i++) {
                    feed(1, dt, t, a_lv0, g);
                    uint8_t h; float bd[3], df;
                    if (attitude_take_bias_event(h, bd, df) && n_ev < 4) hows[n_ev++] = h;
                    float sb[3];
                    if (attitude_take_bias_to_save(sb)) { n_save++; for (int k = 0; k < 3; k++) saved[k] = sb[k]; }
                }
            };
            int ne, ns; uint8_t hw[4] = {9, 9, 9, 9}; float sv[3] = {0, 0, 0};
            // (a) 前回の値なし・本当に静止 13 秒 → 1 つめの窓は確かめられず、2 つめで「続けて 2 つの窓が一致」（how=1）、SD へ 1 回
            run(b0, 13.0, false, b0, ne, hw, ns, sv);
            const bool a_ok = ne == 1 && hw[0] == 1 && ns == 1 && fabsf(sv[2] - b0[2]) < 1e-5f;
            printf("\nD8) ゼロ点の確かめ（本当のゼロ点 z=%.3f deg/s）\n", b0[2] * 57.29578f);
            printf("   (a) 前回の値なし・静止 13 秒: 記録 %d 件 how=%d（1 件・1 が正しい）  SD へ %d 回（1 が正しい）  z=%.4f deg/s\n",
                   ne, (int)hw[0], ns, sv[2] * 57.29578f);
            // (b) 前回の値あり（同じ値）・静止 8 秒 → 1 つめの窓で「前回の値と一致」（how=0）
            for (auto &x : hw) x = 9;
            run(b0, 8.0, true, b0, ne, hw, ns, sv);
            const bool b_ok = ne == 1 && hw[0] == 0 && ns == 1;
            printf("   (b) 前回の値あり・静止 8 秒: 記録 %d 件 how=%d（1 件・0 が正しい）  SD へ %d 回（1 が正しい）\n", ne, (int)hw[0], ns);
            // (c) 前回の値あり・ヨー軸に一定の 0.2 deg/s の回転が 8 秒 → 門は通るが前回の値と食い違うので覚えた扱いにしない（how=2、SD へ書かない）
            float grot[3] = { b0[0], b0[1], b0[2] + 0.0035f };
            for (auto &x : hw) x = 9;
            run(grot, 8.0, true, b0, ne, hw, ns, sv);
            const bool c_ok = ne == 1 && hw[0] == 2 && ns == 0;
            printf("   (c) 前回の値あり・一定の 0.2 deg/s の回転 8 秒: 記録 %d 件 how=%d（1 件・2 が正しい）  SD へ %d 回（0 が正しい）\n", ne, (int)hw[0], ns);
            // (d) 同じ回転が 13 秒（10 秒以上一定に回し続けた）→ 続けて 2 つの窓が一致して覚えてしまう。実機の機体はこう回せない（ユーザー、11/18）ので
            //     受け入れている限界。挙動が変わったら気づけるように固定しておく
            for (auto &x : hw) x = 9;
            run(grot, 13.0, true, b0, ne, hw, ns, sv);
            const bool d_ok = ne == 2 && hw[0] == 2 && hw[1] == 1 && ns == 1;
            printf("   (d) 同じ回転が 13 秒（限界の確認）: 記録 %d 件 how=%d,%d（2 件・2,1 が正しい）  SD へ %d 回（1）\n",
                   ne, (int)hw[0], (int)hw[1], ns);
            ok &= a_ok && b_ok && c_ok && d_ok;
        }

        // D9) 実機のように「いま」がサンプルの時刻より 4ms 先でも、静止で覚えたゼロ点が方位を入れるときに「覚えた」になるか（2026-11-18 夜の車で発覚）
        //   覚えた時刻を time_us_32() で、古さを t_us で見ていたため、差が負になって覚えた直後に捨てていた（夜の 6 回とも bias not learned）。
        g_test_now_skew_us = 4000;
        attitude_setup();
        t = 0;
        {
            const float b0[3] = { 0.0005f, -0.0003f, 0.00075f };
            float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
            feed((int)(13.0 * fs), dt, t, a_lv0, b0);               // 13 秒止めて置く（水平化 2 秒＋続けて 2 つの窓）
            for (int i = 0; i < (int)(4.0 * fs); i++) {             // 東へ 1.5 m/s² で発進
                const double v_east = 1.5 * (i * dt);
                const double ab[3] = { 1.5, 0.0, G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, b0);
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, (float)v_east, 0.0f, 0.08f);
            }
            uint8_t how9 = 255; float mag9, coh9, td9, sg9; bool bok9 = false;
            const bool ev9 = attitude_take_yaw_init_event(how9, mag9, coh9, td9, sg9, bok9);
            printf("\nD9) いまがサンプルより 4ms 先・13 秒止めてから発進: 方位の記録 %d・bias %s（learned が正しい。直す前は not learned）\n",
                   (int)ev9, bok9 ? "learned" : "not learned");
            ok &= ev9 && bok9;
        }
        g_test_now_skew_us = 0;

        // D10) 実際の発進の形（2026-11-18 夜の車）: 加速が 0 から徐々に上がる。「いきなり 1.5 m/s²」の D5 では見えなかった。
        //   車では 2.2 m/s² の発進で方位が決まるまでピッチ +3.2°（門が開く前の立ち上がり 0.5〜1 秒に寄せた）。
        //   GNSS の速度は 170ms 遅れて届く。本当のピッチは 0。
        {
            auto run_ramp = [&](double a_max, double ramp_s, double hold_s, float &pmax_pre, double acc_end_s = 1e9, float *p_end = nullptr) {
                attitude_setup();
                t = 0;
                const float gq[3] = {0, 0, 0};
                float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
                feed((int)(6.0 * fs), dt, t, a_lv0, gq);
                for (int i = 0; i < (int)(4.0 * fs); i++) {          // 動き出す前の GNSS（止まっている）
                    if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, 0.0f, 0.0f, 0.08f);
                    feed(1, dt, t, a_lv0, gq);
                }
                pmax_pre = 0.0f;
                double v = 0.0;
                std::vector<double> vh;                                  // GNSS の遅れ用
                const int n = (int)((ramp_s + hold_s) * fs);
                for (int i = 0; i < n; i++) {
                    const double tt = i * dt;
                    const double a = (tt >= acc_end_s) ? 0.0 : ((tt < ramp_s) ? a_max * tt / ramp_s : a_max);
                    v += a * dt;
                    vh.push_back(v);
                    const double ab[3] = { a, 0.0, G };
                    float as[3]; body_to_sensor(ab, as);
                    feed(1, dt, t, as, gq);
                    if (i % (int)(fs / 4) == 0) {
                        const int lag = (int)(0.17 * fs);
                        const double vd = (i >= lag) ? vh[i - lag] : 0.0;
                        attitude_on_gnss_velocity(0.0f, (float)vd, 0.0f, 0.08f);
                    }
                    if (!attitude_yaw_ready()) {
                        float r, p, y; attitude_get_euler(r, p, y);
                        if (fabsf(p) > pmax_pre) pmax_pre = fabsf(p);
                    }
                }
                if (p_end) { float r, p, y; attitude_get_euler(r, p, y); *p_end = fabsf(p); }
            };
            float p1, p2, p2end = 99.0f;
            run_ramp(2.2, 1.5, 3.0, p1);
            // ゆっくり押す: 0.25 m/s² を 4 秒（1 m/s）→ そのまま 1 m/s で 10 秒。門（0.3 m/s²）より弱い加速は見分けられず、
            //   方位が分からない間は GNSS の加速で傾きを補えないので、押している間は atan(0.25/g) = 1.46° まで寄りうる（限界）。
            //   GNSS が 0.5 m/s を言ったら直前 1.5 秒の寄せを取り消し、一定の速さになったら（上限 8 秒のあと）水平へ戻る。
            run_ramp(0.25, 0.5, 14.0, p2, 4.0, &p2end);
            float du; uint8_t dw; (void)attitude_take_accel_undo_event(du, dw);
            printf("\nD10) 実際の発進の形（加速が徐々に上がる・GNSS は 170ms 遅れ）。本当のピッチ 0\n");
            printf("   0→2.2 m/s² を 1.5 秒で: 方位が決まるまでのピッチの最大 %.2f deg（0.5 未満が正しい。直す前は 2〜3）\n", p1);
            printf("   0.25 m/s² で 4 秒押して 1 m/s で 10 秒: 押している間の最大 %.2f deg（1.5 以下＝atan(0.25/g) を超えないのが正しい）"
                   "  最後 %.2f deg（0.3 未満が正しい）\n", p2, p2end);
            ok &= p1 < 0.5f && p2 <= 1.5f && p2end < 0.3f;
        }

        // D11) 2〜3 m/s の 2 段の門（2026-11-20。settings.h の ESKF_YAW_INIT_CORR_MIN_SPEED_MPS）
        //   (a) 強い向かい風の離陸: 0→2.5 m/s を 1.5 m/s² で加速し、2.5 m/s のまま（3 m/s に届かない）。前は東（90°）
        //       → 2D 相関の門を通るので 3 m/s を待たずに決まる（直す前は決まらないまま）
        //   (b) 機体を横向きに持って東へ 2.5 m/s で運ぶ（前は北 0°）。相関は 0° を出すが航跡 90° と食い違う
        //       → 決めない（航跡に戻すと 90° ずれた向きが σ 15° で決まってしまう）
        {
            auto run_d11 = [&](double front_deg, float &y_out, bool &ready, uint8_t &how) {
                attitude_setup();
                t = 0;
                const float gq[3] = {0, 0, 0};
                float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
                feed((int)(6.0 * fs), dt, t, a_lv0, gq);
                for (int i = 0; i < (int)(2.0 * fs); i++) {
                    if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, 0.0f, 0.0f, 0.08f);
                    feed(1, dt, t, a_lv0, gq);
                }
                // 東（90°）へ動く。本体の前が front_deg を向いているとき、機体軸（前, 左, 上）での加速度は
                // 前 = a cos(90 − front)、左 = a sin(front − 90)（D6 と同じ約束）
                const double rel = (90.0 - front_deg) * M_PI / 180.0;
                double v = 0.0;
                ready = false; how = 255;
                for (int i = 0; i < (int)(8.0 * fs); i++) {
                    const double a = (v < 2.5) ? 1.5 : 0.0;
                    v += a * dt;
                    if (v > 2.5) v = 2.5;
                    const double ab[3] = { a * cos(rel), -a * sin(rel), G };
                    float as[3]; body_to_sensor(ab, as);
                    feed(1, dt, t, as, gq);
                    if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, (float)v, 0.0f, 0.08f);
                }
                float r, p; attitude_get_euler(r, p, y_out);
                ready = attitude_yaw_ready();
                float mg, ch, td, sg; bool bk;
                if (!attitude_take_yaw_init_event(how, mg, ch, td, sg, bk)) how = 255;
            };
            float ya, yb; bool ra, rb; uint8_t ha, hb;
            run_d11(90.0, ya, ra, ha);
            run_d11(0.0, yb, rb, hb);
            printf("\nD11) 2〜3 m/s の門（2D 相関の門を通ったときだけ決める）\n");
            printf("   (a) 向かい風の離陸 0→2.5 m/s・前は東: 決まった %d（1 が正しい）  方位 %.2f deg（90.0 が正しい）  how=%d（0 が正しい）\n",
                   (int)ra, ya, (int)ha);
            printf("   (b) 横向きに持って東へ 2.5 m/s・前は北: 決まった %d（0 が正しい。航跡に戻すと 90° ずれて決まる）\n", (int)rb);
            ok &= ra && fabsf(ya - 90.0f) < 1.0f && ha == 0;
            ok &= !rb;
        }

        // D12) 「GNSS が動いている」の閾値 max(0.2, 2·sAcc)（2026-11-20。settings.h の ESKF_ACCEL_GNSS_MOVING_*）
        //   止まっていて GNSS が 0.3 m/s と言う。sAcc 0.30 なら閾値 0.6 で「動いていない」→ ゼロ点を覚える。
        //   sAcc 0.08 なら閾値 0.2 で「動いている」（＝加速中）→ 覚えない（8 秒の上限より短い 7 秒で見る）。
        {
            auto run_d12 = [&](float sacc) {
                attitude_setup();
                t = 0;
                float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
                // ゼロ点 0.1 deg/s は最初から乗せる（ゼロ点の窓 5 秒は初期化の 2 秒後から始まり、途中で値が変わると揺れとして捨てられる）
                const float off = (float)(0.1 * M_PI / 180.0);
                const float g1[3] = { off, 0.0f, 0.0f };
                feed((int)(4.0 * fs), dt, t, a_lv0, g1);                // 初期化。窓は 2〜7 秒で、GNSS は 4 秒目から
                for (int i = 0; i < (int)(7.0 * fs); i++) {
                    if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.3f, 0.0f, 0.0f, sacc);
                    feed(1, dt, t, a_lv0, g1);
                }
                float bg[3]; attitude_get_gyro_bias(bg);
                return bg[0] * 180.0f / (float)M_PI;
            };
            const float b_noisy = run_d12(0.30f);
            const float b_good  = run_d12(0.08f);
            printf("\nD12) 止まっていて GNSS が 0.3 m/s と言う（閾値 max(0.2, 2·sAcc)）\n");
            printf("   sAcc 0.30（閾値 0.6）: 覚えたゼロ点 %.3f deg/s（0.100 が正しい＝雑音で学習を止めない）\n", b_noisy);
            printf("   sAcc 0.08（閾値 0.2）: 覚えたゼロ点 %.3f deg/s（0 のままが正しい＝動いているとみなす）\n", b_good);
            ok &= fabsf(b_noisy - 0.1f) < 0.02f && fabsf(b_good) < 0.02f;
        }

        // D13) 風で揺れ続けて静止の判定が一度も取れないまま、振動のある滑走で離陸（2026-11-20）
        //   ロール ±2°・0.4 Hz（5°/s）で 30 秒揺れる → 1.5 m/s² で滑走（加速度に σ 2 m/s² の振動）。
        //   3 m/s で「動き出したときの初期化」が走る。1 サンプルで水平化していた頃は最初の傾きが最悪 47° ずれた。
        //   加速度の低域（up_lp_）なら滑走の加速の分（約 9°）だけ。
        {
            attitude_setup();
            t = 0;
            unsigned rs = 12345u;
            auto nrm = [&]() {                                   // 簡単な正規乱数（12 個の一様の和）
                double s = 0; for (int k = 0; k < 12; k++) { rs = rs * 1103515245u + 12345u; s += (rs >> 8) / 16777216.0; } return s - 6.0;
            };
            double v = 0.0, tilt_init = -1.0;
            const int n = (int)(36.0 * fs);
            for (int i = 0; i < n; i++) {
                const double tt = i * dt;
                const bool fly = tt >= 30.0;
                const double ph = fly ? 0.0 : 2.0 * M_PI / 180 * sin(2 * M_PI * 0.4 * tt);
                const double dph = fly ? 0.0 : 2.0 * M_PI / 180 * 2 * M_PI * 0.4 * cos(2 * M_PI * 0.4 * tt);
                const double a = (fly && v < 7.0) ? 1.5 : 0.0;
                v += a * dt;
                double fb[3] = { a, G * sin(ph), G * cos(ph) };
                if (fly) for (int k = 0; k < 3; k++) fb[k] += 2.0 * nrm();
                const double wb[3] = { dph, 0.0, 0.0 };
                float as[3], gs[3]; body_to_sensor(fb, as); body_to_sensor(wb, gs);
                feed(1, dt, t, as, gs);
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, (float)v, 0.0f, 0.10f);
                if (tilt_init < 0.0 && attitude_ready()) {
                    float u[3]; attitude_get_up_sensor(u);
                    const double ub[3] = { 0.0, sin(ph), cos(ph) };
                    float us[3]; body_to_sensor(ub, us);
                    float d = u[0] * us[0] + u[1] * us[1] + u[2] * us[2];
                    tilt_init = acos(d > 1.0f ? 1.0f : d) * 180.0 / M_PI;
                }
            }
            printf("\nD13) 揺れ続けて静止が取れないまま、振動のある滑走（1.5 m/s²・振動 σ 2 m/s²）で離陸\n");
            printf("   動き出したときの初期化の傾きの誤差 %.2f deg（12 未満が正しい。1 サンプルだと最悪 47）\n", tilt_init);
            ok &= tilt_init >= 0.0 && tilt_init < 12.0;
        }

        // D14) はうように 8 秒以上動いてから本当に加速する（2026-11-22 の車。settings.h の ESKF_ACCEL_HOLD_MAX_PREYAW_US）
        //   0.2 m/s² で 7 秒（1.4 m/s）→ 3 秒ゆるく減速（1.1 m/s）→ 1.0 m/s² で 2.5 秒（3.6 m/s）→ そのまま。前は東・本当のピッチ 0。
        //   「加速中」が 8 秒で切れていた頃は、そのあとの加速を方位が決まる前の重力観測が傾きとして取り込み（車で +3.8°）、
        //   「静止」の判定が 2D 相関を 0 に戻して方位は航跡に戻った（mag=0）。
        {
            attitude_setup();
            t = 0;
            const float gq[3] = {0, 0, 0};
            float a_lv0[3]; static_accel_for_bank(0.0, a_lv0);
            feed((int)(6.0 * fs), dt, t, a_lv0, gq);
            for (int i = 0; i < (int)(4.0 * fs); i++) {
                if (i % (int)(fs / 4) == 0) attitude_on_gnss_velocity(0.0f, 0.0f, 0.0f, 0.08f);
                feed(1, dt, t, a_lv0, gq);
            }
            double v = 0.0;
            float pmax = 0.0f;
            std::vector<double> vh;
            const int n = (int)(16.0 * fs);
            for (int i = 0; i < n; i++) {
                const double tt = i * dt;
                const double a = tt < 7.0 ? 0.2 : (tt < 10.0 ? -0.1 : (tt < 12.5 ? 1.0 : 0.0));
                v += a * dt;
                vh.push_back(v);
                const double ab[3] = { a, 0.0, G };
                float as[3]; body_to_sensor(ab, as);
                feed(1, dt, t, as, gq);
                if (i % (int)(fs / 4) == 0) {
                    const int lag = (int)(0.17 * fs);
                    const double vd = (i >= lag) ? vh[i - lag] : 0.0;
                    attitude_on_gnss_velocity(0.0f, (float)vd, 0.0f, 0.08f);
                }
                if (!attitude_yaw_ready()) {
                    float r, p, y; attitude_get_euler(r, p, y);
                    if (fabsf(p) > pmax) pmax = fabsf(p);
                }
            }
            uint8_t how14 = 255; float mag14 = 0, coh14 = 0, td14 = 0, sg14 = 0; bool bok14 = false;
            const bool ev14 = attitude_take_yaw_init_event(how14, mag14, coh14, td14, sg14, bok14);
            float r14, p14, y14; attitude_get_euler(r14, p14, y14);
            printf("\nD14) はうように 10 秒動いてから 1.0 m/s² で加速（11/22 の車の形）。本当のピッチ 0・前は東\n");
            printf("   方位が決まるまでのピッチの最大 %.2f deg（1.2 以下＝はう加速 atan(0.2/g) を超えないのが正しい。直す前は 5.3）\n", pmax);
            printf("   方位の決まり方 how=%d（0 = 2D 相関が正しい。直す前の 11/22 の車は 1 = 航跡）  mag %.0f  方位 %.1f deg（90 が正しい）\n",
                   ev14 ? (int)how14 : -1, mag14, y14);
            ok &= pmax <= 1.2f && ev14 && how14 == 0 && fabsf(y14 - 90.0f) < 5.0f;
        }
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
        feed((int)(8 * fs), dt, t, a20, spin);        // 8 秒。静止しないので水平化はされない
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
    // ★ **ゲートのゲート。** 無条件に入れると「比力＝鉛直」の AHRS と同じ間違いをする
    //   （協調旋回では比力の横成分が厳密に 0 なのでバンクを過小評価する）。
    //   F1: 直進・静止では観測が入ること
    //   F2: **5 度の協調旋回では入らないこと**（人力機の実運用バンク角）
    //   F3: GNSS が無くてもロール・ピッチの錨になること
    {
        // ---- F1) 静止・直進では観測が入る ----
        attitude_setup();
        t = 0;
        feed((int)(4 * fs), dt, t, a20, gz);                // 水平化
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
        feed((int)(4 * fs), dt, t, a20, gz);               // まず静止で水平化
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
    //   G5: 動いている間（GNSS 断・振動あり）でも、60 度を超える傾きの誤差から比力で戻る（ESKF_BIGTILT_*。2026-11-11）。
    //       方位は保ち、ひっくり返った推定で 180 度逆を保っていたら、GNSS が戻ったときに航跡で入れ直す
    //   G6: 誤差が 60 度未満なら G5 は働かない。回している間も働かない
    //   G7: 保った方位が合っていれば、横滑りで 10 度ずれた航跡では上書きしない（ヨーを捨てない。2026-11-12）
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
                feed((int)(0.5 * fs), dt, t, a_lv, gz);
                attitude_on_gnss_velocity(8.0f, 0.0f, 0.0f, 0.08f);
            }
        };

        // ---- G1) sAcc の悪い測位ではヨーを入れない ----
        attitude_setup();
        t = 0;
        feed((int)(4 * fs), dt, t, a_lv, gz);                       // 水平化
        for (int i = 0; i < 6; i++) {
            feed((int)(0.5 * fs), dt, t, a_lv, gz);
            attitude_on_gnss_velocity(3.8f, 0.0f, 0.0f, 2.06f);   // 実機の屋内の値
        }
        const bool yaw_bad = attitude_yaw_ready();
        for (int i = 0; i < ESKF_YAW_INIT_ARM_FIXES; i++) {
            feed((int)(0.5 * fs), dt, t, a_lv, gz);
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
        feed((int)(4 * fs), dt, t, a_lv, gz);
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
        // 止まって 3 秒。止まったあとの比力は、回った先の重力（ジャイロの積分と合う向き）にする。
        //   （2026-11-18〜。「加速中」の判定はジャイロで回した平均と比べるので、回ったのに加速度計が動かないままだと加速と読む。
        //    実機では回ったぶん加速度計も回る。回している間だけ比力を動かさないのは、回転に伴う加速度の代わり）
        float up2[3]; attitude_get_up_sensor(up2);
        const float a_after2[3] = { (float)(up2[0] * G), (float)(up2[1] * G), (float)(up2[2] * G) };
        feed((int)(3 * fs), dt, t, a_after2, gz);
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
        feed((int)(4 * fs), dt, t, a_lv, gz);
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
        feed((int)(4 * fs), dt, t, a_lv, gz);
        set_yaw_good();
        const uint32_t h0 = attitude_get_ground_relevels();
        for (int i = 0; i < 20; i++) {                   // 10 秒
            feed((int)(0.5 * fs), dt, t, a_lv, gz);
            attitude_on_gnss_velocity(8.0f, 0.0f, 0.0f, 0.08f);
        }
        const uint32_t h1 = attitude_get_ground_relevels();
        for (int i = 0; i < 6; i++) {                    // GNSS が 0.3m/s に変わった
            feed((int)(0.5 * fs), dt, t, a_lv, gz);
            attitude_on_gnss_velocity(0.3f, 0.0f, 0.0f, 0.08f);
        }
        const uint32_t h2 = attitude_get_ground_relevels();
        printf("\nG4) GNSS が 8m/s と言っている間は地上の再水平化をしない\n");
        printf("   静止 10 秒・GNSS 8m/s: %u 回（0 が正しい）  GNSS 0.3m/s へ変わったら %u 回（1 が正しい）\n",
               (unsigned)(h1 - h0), (unsigned)(h2 - h1));
        ok &= (h1 - h0) == 0 && (h2 - h1) == 1;

        // ---- G5) 動いている間でも、大きくずれた傾きから比力で戻る ----
        // 2026-11-11 16:09 の車載試験の再現: トンネル（GNSS なし）の中で本体を大きく動かされ、
        // ロールが約 95 度ずれたまま 35 秒戻らなかった。止まっていないので地上での維持は働かない。
        // ここでは走行中の振動の代わりに、ジャイロへ 4deg/s の揺れ（静止判定は外れるが 10deg/s 未満）と、
        // 比力へ ±0.8 m/s² の揺れ（瞬時の |f| は g ± 0.5 を外れることがある）を足す。
        attitude_setup();
        t = 0;
        feed((int)(4 * fs), dt, t, a_lv, gz);
        set_yaw_good();
        const uint32_t b0 = attitude_get_bigtilt_relevels();
        feed((int)(2 * fs), dt, t, a_lv, wv3);           // 60deg/s で 2 秒 → 傾きの推定が 120 度狂う
        const float e5_before = up_err_deg(a_lv);
        const uint32_t b1 = attitude_get_bigtilt_relevels();   // 回している間は働かない（G6 の一部）
        const uint32_t gr0 = attitude_get_ground_relevels();
        float e5_at_1s = 0.0f;
        for (int i = 0; i < (int)(6 * fs); i++) {        // 揺れながら 6 秒。GNSS は来ない（トンネル）
            const double ph = 2.0 * M_PI * 7.0 * i / fs; // 7Hz の揺れ
            const double wb[3] = { 4.0 * M_PI / 180.0 * sin(ph), 4.0 * M_PI / 180.0 * cos(ph), 0.0 };
            float wv[3]; body_to_sensor(wb, wv);
            float av[3];
            for (int k = 0; k < 3; k++) av[k] = a_lv[k];
            av[0] += (float)(0.8 * sin(ph * 1.3));
            feed(1, dt, t, av, wv);
            if (i == (int)(1 * fs)) e5_at_1s = up_err_deg(a_lv);
        }
        const float e5_after = up_err_deg(a_lv);
        const uint32_t b2 = attitude_get_bigtilt_relevels();
        float evd = 0.0f;
        bool kept5 = false;
        const bool ev = attitude_take_bigtilt_event(evd, kept5);
        float r5, p5, h5; attitude_get_euler_raw(r5, p5, h5);
        printf("\nG5) 走行中（GNSS なし・振動あり）に傾きの推定が 120 度狂った状態から（ピッチの軸で回ってひっくり返った推定）\n");
        printf("   上方向の誤差 %.1f → 1 秒後 %.1f（まだ戻さない）→ 6 秒後 %.2f deg（5 未満が正しい）\n",
               e5_before, e5_at_1s, e5_after);
        printf("   置き直し %u 回（1 が正しい）  回している間 %u 回（0 が正しい）  地上の再水平化 %u 回（0 が正しい）\n",
               (unsigned)(b2 - b1), (unsigned)(b1 - b0), (unsigned)(attitude_get_ground_relevels() - gr0));
        printf("   ログ用の印 %d・食い違い %.0f deg  方位を保った=%d  yaw_ready=%d（1 と 1 が正しい）  保った方位 %.0f deg（ひっくり返ったので 180）\n",
               (int)ev, evd, (int)kept5, (int)attitude_yaw_ready(), h5);
        ok &= e5_before > 100.0f && e5_at_1s > 60.0f && e5_after < 5.0f;
        ok &= (b2 - b1) == 1 && (b1 - b0) == 0 && attitude_get_ground_relevels() == gr0;
        ok &= ev && evd > 60.0f && kept5 && attitude_yaw_ready();
        ok &= fabsf(fabsf(h5 - 180.0f)) < 5.0f;
        // GNSS が戻ったら（真方位 0 度へ 8m/s）航跡と比べ、180 度食い違っているので入れ直す
        const uint32_t gu5 = attitude_get_gnss_updates();
        set_yaw_good();
        float dd5 = 0.0f; bool reset5 = false;
        const bool chk5 = attitude_take_bigtilt_yaw_check(dd5, reset5);
        attitude_get_euler_raw(r5, p5, h5);
        const float h5e = fabsf(h5 > 180.0f ? h5 - 360.0f : h5);
        printf("   GNSS が戻って航跡と比べた: 印 %d・差 %+.0f deg・入れ直し %d（1・±180・1 が正しい）  方位 %.1f deg（0 が正しい）  上方向の誤差 %.2f deg\n",
               (int)chk5, dd5, (int)reset5, h5, up_err_deg(a_lv));
        printf("   比べる前の速度観測 %u 回（0 が正しい＝比べ終わるまで入れない）\n",
               (unsigned)(attitude_get_gnss_updates() - gu5));
        ok &= chk5 && reset5 && fabsf(dd5) > 170.0f && h5e < 3.0f && up_err_deg(a_lv) < 5.0f;
        ok &= attitude_get_gnss_updates() == gu5;

        // ---- G6) 誤差が 60 度未満なら働かない ----
        // 30deg/s で 1.5 秒（45 度）だけ狂わせ、同じ揺れで 6 秒。重力観測で戻るのは許す（そちらの仕事）。
        attitude_setup();
        t = 0;
        feed((int)(4 * fs), dt, t, a_lv, gz);
        set_yaw_good();
        const double wb6[3] = { 0.0, 30.0 * M_PI / 180.0, 0.0 };
        float wv6[3]; body_to_sensor(wb6, wv6);
        feed((int)(1.5 * fs), dt, t, a_lv, wv6);
        const float e6 = up_err_deg(a_lv);
        const uint32_t c0 = attitude_get_bigtilt_relevels();
        for (int i = 0; i < (int)(6 * fs); i++) {
            const double ph = 2.0 * M_PI * 7.0 * i / fs;
            const double wb[3] = { 4.0 * M_PI / 180.0 * sin(ph), 4.0 * M_PI / 180.0 * cos(ph), 0.0 };
            float wv[3]; body_to_sensor(wb, wv);
            feed(1, dt, t, a_lv, wv);
        }
        printf("\nG6) 誤差 %.1f deg（60 未満）では置き直さない: %u 回（0 が正しい）  yaw_ready=%d（1 が正しい）\n",
               e6, (unsigned)(attitude_get_bigtilt_relevels() - c0), (int)attitude_yaw_ready());
        ok &= e6 > 40.0f && e6 < 55.0f;
        ok &= attitude_get_bigtilt_relevels() == c0 && attitude_yaw_ready();

        // ---- G7) 保った方位が合っていれば、横滑りでずれた航跡では上書きしない ----
        // 真の機首は 0 度。ロールの軸で 50deg/s × 2 秒（100 度）狂わせ、GNSS なし・揺れで置き直させる。
        // そのあと GNSS が戻るが、航跡は 10 度（飛行中の横滑りの代わり）。方位は 0 度のまま残ること。
        attitude_setup();
        t = 0;
        feed((int)(4 * fs), dt, t, a_lv, gz);
        set_yaw_good();                                  // 真方位 0 度
        const double wb7[3] = { 50.0 * M_PI / 180.0, 0.0, 0.0 };
        float wv7[3]; body_to_sensor(wb7, wv7);
        feed((int)(2 * fs), dt, t, a_lv, wv7);
        for (int i = 0; i < (int)(6 * fs); i++) {
            const double ph = 2.0 * M_PI * 7.0 * i / fs;
            const double wb[3] = { 4.0 * M_PI / 180.0 * sin(ph), 4.0 * M_PI / 180.0 * cos(ph), 0.0 };
            float wv[3]; body_to_sensor(wb, wv);
            feed(1, dt, t, a_lv, wv);
        }
        float evd7 = 0.0f; bool kept7 = false;
        const bool ev7 = attitude_take_bigtilt_event(evd7, kept7);
        const double trk7 = 10.0 * M_PI / 180.0;
        for (int i = 0; i < ESKF_YAW_INIT_ARM_FIXES + 2; i++) {
            feed((int)(0.5 * fs), dt, t, a_lv, gz);
            attitude_on_gnss_velocity((float)(8.0 * cos(trk7)), (float)(8.0 * sin(trk7)), 0.0f, 0.08f);
        }
        float dd7 = 0.0f; bool reset7 = true;
        const bool chk7 = attitude_take_bigtilt_yaw_check(dd7, reset7);
        float r7, p7, h7; attitude_get_euler_raw(r7, p7, h7);
        const float h7e = fabsf(h7 > 180.0f ? h7 - 360.0f : h7);
        printf("\nG7) 方位 0 度でロールが 100 度狂ったのを置き直し、GNSS は横滑りで 10 度の航跡\n");
        printf("   置き直し %d・方位を保った %d（1・1 が正しい）  航跡と比べた %d・差 %+.1f deg・入れ直し %d（1・+10・0 が正しい）\n",
               (int)ev7, (int)kept7, (int)chk7, dd7, (int)reset7);
        printf("   方位 %.1f deg（0 のまま＝航跡の 10 で上書きしないのが正しい）  上方向の誤差 %.2f deg\n", h7, up_err_deg(a_lv));
        ok &= ev7 && kept7 && chk7 && !reset7 && fabsf(dd7 - 10.0f) < 3.0f;
        ok &= h7e < 3.0f && up_err_deg(a_lv) < 5.0f;
    }
    printf("\n結果: %s\n", ok ? "PASS — 水平化・航跡ヨー・待機中の維持・上方向の常時提供・重力観測・発散の再発防止がすべて正しい"
                             : "FAIL");
    return ok ? 0 : 1;
}
