// 偏角補正の符号を検証する。
//   真方位 = 磁方位 + 偏角   （琵琶湖は西偏 -8 度）
// ROTATION_VECTOR（磁北基準）で初期化したとき、出力ヨーが
// 「そのクォータニオンの磁方位 + 偏角」になっていることを確かめる。
#include "attitude.h"
#include <cstdio>
#include <cmath>

static const float DECL = -8.0f;

// euler_from_state() のヨー部分と同じ式（マウント補正込み）
static float mount_yaw_deg(float w, float x, float y, float z) {
    float sy = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    float d = -sy * 180.0f / (float)M_PI;
    if (d < 0) d += 360.0f;
    if (d >= 360.0f) d -= 360.0f;
    return d;
}

int main() {
    // 適当な姿勢（水平ではない）を磁北基準の RV として与える
    float q[4] = {0.80f, 0.52f, 0.10f, 0.28f};
    float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for (int i=0;i<4;i++) q[i]/=n;

    attitude_setup();
    float g0[3]={0,0,0}, a0[3]={0,0,9.80665f};
    for (int i=0;i<300;i++) {
        attitude_on_accel(a0);
        attitude_on_grv(1,0,0,0);                       // GRV は別物にしておく（RV が使われるか確認）
        attitude_on_rv(q[0],q[1],q[2],q[3], 0.05f);     // 精度 0.05rad = 2.9度（良好）
        test_set_us((uint32_t)(i*20000)); attitude_on_gyro(g0, g_test_us);
    }
    float r,p,yaw; attitude_get_euler_raw(r,p,yaw);
    float mag = mount_yaw_deg(q[0],q[1],q[2],q[3]);
    float expect = mag + DECL;
    if (expect < 0) expect += 360.0f;
    float diff = yaw - expect;
    while (diff > 180) diff -= 360; while (diff < -180) diff += 360;

    printf("ready=%d  地磁気で初期化=%d\n", (int)attitude_ready(), (int)attitude_yaw_from_mag());
    printf("RV の磁方位        : %7.2f deg\n", mag);
    printf("期待する真方位     : %7.2f deg  (磁方位 %+.1f)\n", expect, DECL);
    printf("ESKF の出力ヨー    : %7.2f deg  (差 %+.3f)\n", yaw, diff);

    // 精度が悪い RV は使わない、というガードも確認
    attitude_setup();
    for (int i=0;i<300;i++) {
        attitude_on_accel(a0);
        attitude_on_grv(1,0,0,0);
        attitude_on_rv(q[0],q[1],q[2],q[3], 0.80f);     // 0.80rad = 45.8度（悪い）
        attitude_on_gyro(g0, (uint32_t)(i*20000));
    }
    bool used = attitude_yaw_from_mag();
    printf("\n精度45.8度の RV を与えた場合: 地磁気で初期化=%d (0 が正しい)\n", (int)used);

    bool ok = fabsf(diff) < 0.05f && !used;
    printf("\n結果: %s\n", ok ? "PASS — 偏角の符号と精度ガードが正しい" : "FAIL");
    return ok?0:1;
}
