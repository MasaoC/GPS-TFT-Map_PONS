// attitude.cpp の数学を PC 上で検証する。
// 実機と同じ協調旋回の合成データを流し、真のバンク角を復元できるか見る。
#include "attitude.h"
#include <cstdio>
#include <cmath>

// attitude.cpp の内部を使わず、公開 API だけで駆動する。
// 初期化は静止判定を通す必要があるので、まず静止データを流す。
static const double G = 9.80665;

static void Rz(double a, double R[3][3]) {
    double c=cos(a), s=sin(a);
    double M[3][3]={{c,-s,0},{s,c,0},{0,0,1}};
    memcpy(R,M,sizeof(M));
}
static void Rx(double a, double R[3][3]) {
    double c=cos(a), s=sin(a);
    double M[3][3]={{1,0,0},{0,c,-s},{0,s,c}};
    memcpy(R,M,sizeof(M));
}
static void mul(const double A[3][3], const double B[3][3], double C[3][3]) {
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){double s=0;for(int k=0;k<3;k++)s+=A[i][k]*B[k][j];C[i][j]=s;}
}

int main() {
    attitude_setup();

    const double phi = 20.0*M_PI/180.0;      // 真のバンク角
    const double V = 8.0;
    const double psid = -G*tan(phi)/V;        // 協調旋回（符号は test_eskf.py と同じ導出）
    const double fs = 50.0, dt = 1.0/fs;

    // ---- 1) 静止させて初期化する（水平・機首東）----
    // 静止中は比力が +Z（機体上向き）に g だけ出る
    {
        double R[3][3]; Rz(0,R);   // 単位行列相当
        float g0[3]={0,0,0}, a0[3]={0,0,(float)G};
        // GRV は水平姿勢のクォータニオン（単位）
        for (int i=0;i<200;i++) {
            attitude_on_accel(a0);
            attitude_on_grv(1,0,0,0);
            test_set_us((uint32_t)(i*dt*1e6)); attitude_on_gyro(g0, g_test_us);
        }
    }
    printf("初期化後 ready=%d\n", (int)attitude_ready());

    // ---- 2) 協調旋回を流す ----
    int n = (int)(60*fs);
    double t=0;
    for (int i=0;i<n;i++, t+=dt) {
        double A[3][3],B[3][3],R[3][3];
        Rz(psid*t,A); Rx(phi,B); mul(A,B,R);
        // v = V * R[:,0]
        double v[3]={V*R[0][0], V*R[1][0], V*R[2][0]};
        // a_world = d/dt v = V*psid * (zhat x forward)
        double fw[3]={R[0][0],R[1][0],R[2][0]};
        double aw[3]={-V*psid*fw[1], V*psid*fw[0], 0};
        // f_body = R^T (a_world - g_world),  g_world = (0,0,-G)
        double d[3]={aw[0], aw[1], aw[2]+G};
        float f[3];
        for(int r=0;r<3;r++) f[r]=(float)(R[0][r]*d[0]+R[1][r]*d[1]+R[2][r]*d[2]);
        // omega_body = R^T * (0,0,psid)
        float w[3];
        for(int r=0;r<3;r++) w[r]=(float)(R[2][r]*psid);

        attitude_on_accel(f);
        test_set_us((uint32_t)((10.0+t)*1e6)); attitude_on_gyro(w, g_test_us);
        if (i%25==0) {   // GNSS 2Hz。NED で渡す: velN=v[1], velE=v[0], velD=-v[2]
            attitude_on_gnss_velocity((float)v[1],(float)v[0],(float)(-v[2]),0.08f);
        }
    }

    // ---- 3) 結果 ----
    // ロール（機体軸）が真のバンク角に一致するか
    float roll,pitch,yaw; attitude_get_euler_raw(roll,pitch,yaw);
    // マウント補正の定義は roll=sensor_pitch, pitch=sensor_roll-90 なので、
    // シミュレーションの機体バンク（= sensor_roll）は pitch+90 で取り出す。
    float sensor_roll = pitch + 90.0f;
    float bg[3]; attitude_get_gyro_bias(bg);
    printf("真のバンク角 : %.2f deg\n", phi*180/M_PI);
    printf("ESKF (sensor_roll) : %.2f deg  (誤差 %+.2f)\n",
           sensor_roll, sensor_roll-(float)(phi*180/M_PI));
    printf("  参考 出力roll=%.2f 出力pitch=%.2f yaw=%.2f\n", roll, pitch, yaw);
    printf("推定ジャイロバイアス: %.3f %.3f %.3f deg/s\n",
           bg[0]*180/M_PI, bg[1]*180/M_PI, bg[2]*180/M_PI);
    printf("GNSS 観測回数: %u\n", (unsigned)attitude_get_gnss_updates());
    bool ok = fabsf(sensor_roll - (float)(phi*180/M_PI)) < 1.5f;
    printf("\n結果: %s\n", ok ? "PASS — C++ 版も真のバンクを復元した" : "FAIL");
    return ok?0:1;
}
