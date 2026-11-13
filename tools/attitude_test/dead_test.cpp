// IMU（SCH16T）の断線・未接続時に ESKF が安全側に倒れるかを検証する。
//
// 1) 未接続（センサー入力が一度も来ない）: 初期化されず ready=false のまま
// 2) 初期化後に断線: ジャイロが止まったら ready=false に落ち、
//    GNSS 観測だけで姿勢が回されない
//
// 3) 復帰したら再び使えるか
// 4) 途絶えている間に 90 度旋回していたら、戻ったあと方位を航跡で確かめて入れ直すか（BUG-62。2026-11-13〜）
// 5) 途絶明けに止まっていたら、方位の確かめは待ったまま速度の観測は入るか（BUG-64。2026-11-13〜）
//
// 対策前は 2) で「水平のまま加速しただけでロールが 60 秒後に +8 度」まで育ち、
// バンク警告の閾値 4 度を超えて偽警報が出ていた。
#include "attitude.h"
#include <cstdio>
#include <cmath>

static void feed_static(int n, uint32_t t0, uint32_t step) {
    // ★ 水平に置いた比力（機体の上向き g）をセンサー座標で。main.cpp / yaw_test.cpp の
    //   body_to_sensor({0,0,g}) と同じ {-g,0,0}。2026-11-10 までは {0,0,g}（機首が真上／真下）で、
    //   ロールが定まらない特異点に置いていたため、「断線後にロールが回されない」の確認がほとんど効いていなかった。
    float g0[3]={0,0,0}, a0[3]={-9.80665f,0,0};
    for (int i=0;i<n;i++){
        test_set_us(t0 + i*step);
        attitude_on_accel(a0);
        attitude_on_gyro(g0, g_test_us);
    }
}

int main(){
    bool ok = true;

    // ---- 1) 未接続 ----
    attitude_setup();
    for (int i=0;i<40;i++){ test_advance_us(500000); attitude_on_gnss_velocity(0,3,0,0.08f); }
    printf("[未接続] ready=%d (0 が正しい)  ", (int)attitude_ready());
    float r,p,y; attitude_get_euler_raw(r,p,y);
    printf("roll=%+.2f\n", r);
    ok &= !attitude_ready();

    // ---- 2) 初期化後に断線 ----
    attitude_setup();
    const uint32_t step_us = 1000000UL / IMU_RATE_GYRO_HZ; // 250Hz -> 4000us
    feed_static((int)(6 * IMU_RATE_GYRO_HZ), 0, step_us);  // 6 秒静止して初期化
    printf("[初期化後] ready=%d (1 が正しい)  ", (int)attitude_ready());
    attitude_get_euler_raw(r,p,y);
    printf("roll=%+.2f\n", r);
    ok &= attitude_ready();
    float roll0 = r;

    // ここで断線。ジャイロ・加速度は来ない。時刻だけ進み GNSS は 2Hz で入り続ける。
    // 車は水平のまま東へ加速していく想定。
    uint32_t t = g_test_us;
    for (int k=0;k<120;k++){
        t += 500000; test_set_us(t);
        attitude_on_gnss_velocity(0.0f, k*0.0833f, 0.0f, 0.08f);
        if (k==1 || k==20 || k==119){
            attitude_get_euler_raw(r,p,y);
            printf("[断線後 %5.1f秒] ready=%d roll=%+.2f (基準 %+.2f からの変化 %+.2f)\n",
                   k*0.5f, (int)attitude_ready(), r, roll0, r-roll0);
        }
    }
    attitude_get_euler_raw(r,p,y);
    ok &= !attitude_ready();                       // 途読を検出できたか
    ok &= (fabsf(r-roll0) < 0.1f);                 // 姿勢が回されていないか

    // ---- 3) 復帰したら再び使えるか ----
    feed_static((int)(6 * IMU_RATE_GYRO_HZ), t+1000000, step_us);
    printf("[復帰後] ready=%d (1 が正しい)\n", (int)attitude_ready());
    ok &= attitude_ready();

    // ---- 4) 途絶えている間に 90 度旋回していた（BUG-62）----
    // 北へ 8m/s で走ってヨーを決め、61 秒途絶える（SCH16T の復旧は 1 分ごと）。その間に東へ向きを変えた。
    // 水平・等速なので比力はセンサー座標で同じ。ESKF から見ると「方位が 90 度ずれたまま」戻ってくる。
    // 対策前は途絶える前の方位と狭い不確かさのまま速度の観測を入れていたので、方位は北のまま残った。
    auto run_gnss = [&](float secs, float vn, float ve) {
        const int fixes = (int)(secs * 2.0f);
        for (int k = 0; k < fixes; k++) {
            feed_static((int)(IMU_RATE_GYRO_HZ / 2), g_test_us + step_us, step_us);
            attitude_on_gnss_velocity(vn, ve, 0.0f, 0.08f);
        }
    };
    attitude_setup();
    feed_static((int)(4 * IMU_RATE_GYRO_HZ), 0, step_us);  // 水平化
    run_gnss(10.0f, 8.0f, 0.0f);                           // 北へ 8m/s。ヨーが決まる
    attitude_get_euler_raw(r,p,y);
    const float y4_before = y;
    float gs_tmp; bool kept_tmp;
    (void)attitude_take_imu_gap_event(gs_tmp, kept_tmp);   // 前の分を捨てる
    float dd_tmp; bool rs_tmp;
    (void)attitude_take_bigtilt_yaw_check(dd_tmp, rs_tmp);
    t = g_test_us;
    for (int k = 0; k < 122; k++) {                        // 61 秒途絶。GNSS は東へ 8m/s（取り込まれない）
        t += 500000; test_set_us(t);
        attitude_on_gnss_velocity(0.0f, 8.0f, 0.0f, 0.08f);
    }
    test_set_us(t + 4000);
    run_gnss(5.0f, 0.0f, 8.0f);                            // 戻った。東へ 8m/s が 5 秒
    float gap4 = 0.0f; bool kept4 = false;
    const bool gev4 = attitude_take_imu_gap_event(gap4, kept4);
    float dd4 = 0.0f; bool reset4 = false;
    const bool chk4 = attitude_take_bigtilt_yaw_check(dd4, reset4);
    attitude_get_euler_raw(r,p,y);
    printf("[途絶中に 90 度旋回] 途絶前の方位 %.1f deg → 戻って 5 秒後 %.1f deg（90 が正しい）  ロール %+.2f ピッチ %+.2f（0 が正しい）\n",
           y4_before, y, r, p);
    printf("   途絶明けの印 %d・%.1f 秒・方位を保った %d（1・61・1 が正しい）  航跡と比べた %d・差 %+.0f deg・入れ直し %d（1・+90・1 が正しい）\n",
           (int)gev4, gap4, (int)kept4, (int)chk4, dd4, (int)reset4);
    ok &= fabsf(y4_before) < 3.0f || fabsf(y4_before - 360.0f) < 3.0f;
    ok &= fabsf(y - 90.0f) < 5.0f && fabsf(r) < 1.0f && fabsf(p) < 1.0f;
    ok &= gev4 && gap4 > 60.0f && kept4 && chk4 && reset4 && fabsf(dd4 - 90.0f) < 5.0f;

    // ---- 5) 途絶明けに止まっていた（BUG-64）----
    // 北へ走ってヨーを決めてから止まり、61 秒途絶。戻ったときは止まったまま（GNSS 0m/s）。
    // 方位の確かめは 3m/s 以上まで待つが、止まっている間の速度の観測は入る（入らないと v_ が漂い、風と姿勢に × が残る）。
    // 1.5m/s（走り出し）では入らない。
    attitude_setup();
    feed_static((int)(4 * IMU_RATE_GYRO_HZ), 0, step_us);
    run_gnss(10.0f, 8.0f, 0.0f);
    run_gnss(3.0f, 0.0f, 0.0f);                            // 止まった
    (void)attitude_take_imu_gap_event(gs_tmp, kept_tmp);
    (void)attitude_take_bigtilt_yaw_check(dd_tmp, rs_tmp);
    t = g_test_us + 61000000UL;                            // 61 秒途絶
    test_set_us(t);
    feed_static(1, t, step_us);                            // 戻った最初のサンプル（途絶明けの扱い）
    run_gnss(1.0f, 0.0f, 0.0f);                            // 最初の測位は速度の入れ直しに使う
    const uint32_t u5a = attitude_get_gnss_updates();
    run_gnss(5.0f, 0.0f, 0.0f);                            // 止まったまま 5 秒
    const uint32_t u5b = attitude_get_gnss_updates();
    run_gnss(3.0f, 1.5f, 0.0f);                            // 走り出し 1.5m/s が 3 秒
    const uint32_t u5c = attitude_get_gnss_updates();
    const bool chk5 = attitude_take_bigtilt_yaw_check(dd_tmp, rs_tmp);
    const bool fresh5 = attitude_gnss_vel_fresh();
    run_gnss(3.0f, 8.0f, 0.0f);                            // 8m/s で確かめが済む
    float dd5 = 0.0f; bool reset5 = true;
    const bool chk5b = attitude_take_bigtilt_yaw_check(dd5, reset5);
    printf("[途絶明けに止まっていた] 止まって 5 秒の速度観測 %u 回（10 が正しい）  1.5m/s で %u 回（0 が正しい）  確かめ %d（0 が正しい）\n",
           (unsigned)(u5b - u5a), (unsigned)(u5c - u5b), (int)chk5);
    printf("   8m/s で確かめた %d・差 %+.0f deg・入れ直し %d（1・0・0 が正しい）  走り出しの時点で GNSS の鮮度 %d\n",
           (int)chk5b, dd5, (int)reset5, (int)fresh5);
    ok &= (u5b - u5a) == 10 && (u5c - u5b) == 0 && !chk5;
    ok &= chk5b && !reset5 && fabsf(dd5) < 3.0f;

    printf("\n結果: %s\n", ok ? "PASS — 断線時に安全側へ倒れ、途絶明けは方位を確かめ直す" : "FAIL");
    return ok?0:1;
}
