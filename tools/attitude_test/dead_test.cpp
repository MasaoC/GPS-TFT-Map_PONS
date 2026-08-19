// BNO085 の断線・未接続時に ESKF が安全側に倒れるかを検証する。
//
// 1) 未接続（センサー入力が一度も来ない）: 初期化されず ready=false のまま
// 2) 初期化後に断線: ジャイロが止まったら ready=false に落ち、
//    GNSS 観測だけで姿勢が回されない
//
// 対策前は 2) で「水平のまま加速しただけでロールが 60 秒後に +8 度」まで育ち、
// バンク警告の閾値 4 度を超えて偽警報が出ていた。
#include "attitude.h"
#include <cstdio>
#include <cmath>

static void feed_static(int n, uint32_t t0, uint32_t step) {
    float g0[3]={0,0,0}, a0[3]={0,0,9.80665f};
    for (int i=0;i<n;i++){
        test_set_us(t0 + i*step);
        attitude_on_accel(a0);
        attitude_on_grv(1,0,0,0);
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
    feed_static(300, 0, 20000);            // 6 秒静止して初期化
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
    ok &= !attitude_ready();                       // 途絶を検出できたか
    ok &= (fabsf(r-roll0) < 0.1f);                 // 姿勢が回されていないか

    // ---- 3) 復帰したら再び使えるか ----
    feed_static(300, t+1000000, 20000);
    printf("[復帰後] ready=%d (1 が正しい)\n", (int)attitude_ready());
    ok &= attitude_ready();

    printf("\n結果: %s\n", ok ? "PASS — 断線時に安全側へ倒れる" : "FAIL");
    return ok?0:1;
}
