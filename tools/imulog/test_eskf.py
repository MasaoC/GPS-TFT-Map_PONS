#!/usr/bin/env python3
"""
ESKF の合成データ検証: 定常協調旋回で真のバンク角を復元できるか。

■ なぜこの試験なのか
協調旋回中、機体が感じる比力は機体 -Z 方向一直線になり、横方向成分がゼロになる。
つまり「加速度計から見ると水平飛行と区別がつかない」。
BNO085 内蔵フュージョンは長期的に加速度計を鉛直の基準にするため、
旋回が続くとロールが 0 へ引き戻される ＝ 実機で報告された「旋回中のロール過小評価」。

GNSS 速度から対地加速度を与えれば遠心力分を差し引けるので、真のバンクが復元できる。
その差をここで数値で示す。

実行: python3 test_eskf.py
"""

import numpy as np
from eskf import AttitudeESKF, GRAVITY, G_ENU, quat_to_R, skew

# ---- ボディ系: x=前方, y=左, z=上（ワールドは ENU）----

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

def R_to_roll(R):   # R = Rz(psi)Ry(theta)Rx(phi) より R[2,1]=c(th)s(ph), R[2,2]=c(th)c(ph)
    return np.arctan2(R[2, 1], R[2, 2])

def R_to_pitch(R):  # 同上より R[2,0] = -sin(theta)
    return np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))


def simulate(V=8.0, bank_deg=20.0, dur=40.0, fs=50.0):
    """定常協調旋回の真値と、そこから逆算したジャイロ・加速度計出力を作る。"""
    phi = np.deg2rad(bank_deg)
    # 協調旋回の関係式。符号に注意:
    #   R = Rz(psi)Rx(phi) かつ ボディ y=左 の規約では、phi>0 は「右バンク」になる。
    #   一方 Rz の psi>0 は ENU 上で反時計回り＝左旋回。両者は逆向きなので符号が要る。
    #   比力の横成分を 0 とおくと cos(phi)*V*psi_dot + sin(phi)*g = 0
    #   → V*psi_dot = -g*tan(phi)
    psi_dot = -GRAVITY * np.tan(phi) / V
    n = int(dur * fs)
    t = np.arange(n) / fs

    R_true = np.zeros((n, 3, 3))
    v_true = np.zeros((n, 3))
    for i in range(n):
        R_true[i] = Rz(psi_dot * t[i]) @ Rx(phi)
        v_true[i] = V * (R_true[i] @ np.array([1.0, 0.0, 0.0]))   # 機首方向に V

    # 対地加速度は速度の時間微分（中心差分）
    a_true = np.gradient(v_true, 1.0 / fs, axis=0, edge_order=2)

    gyro = np.zeros((n, 3))
    accel = np.zeros((n, 3))
    for i in range(n):
        R = R_true[i]
        # 比力 = R^T (a_world - g_world)
        accel[i] = R.T @ (a_true[i] - G_ENU)
        # ω は R^T Ṙ の反対称部分から。ここでは解析的に: ワールド z 軸まわり psi_dot
        gyro[i] = R.T @ np.array([0.0, 0.0, psi_dot])
    return t, R_true, v_true, gyro, accel, phi


def run_turn(fs=50.0, gnss_hz=2.0, seed=0, bank_deg=20.0):
    """協調旋回を回して (真値deg, 加速度計のみdeg, ESKFの時系列deg, t) を返す。"""
    rng = np.random.default_rng(seed)
    t, R_true, v_true, gyro, accel, phi = simulate(fs=fs, bank_deg=bank_deg)
    n = len(t)
    gyro_m = gyro + np.deg2rad([0.5, -0.3, 0.2]) + rng.normal(0, 2e-3, (n, 3))
    accel_m = accel + rng.normal(0, 3e-2, (n, 3))
    roll_accel = np.arctan2(-accel_m[:, 1], accel_m[:, 2])

    f = AttitudeESKF()
    f.init(q0=[1, 0, 0, 0], v0=v_true[0])
    step = max(1, int(round(fs / gnss_hz)))
    roll = np.zeros(n)
    for i in range(n):
        f.predict(gyro_m[i], accel_m[i], 1.0 / fs)
        if i % step == 0:
            f.update_velocity(v_true[i] + rng.normal(0, 0.05, 3), sacc=0.08)
        roll[i] = R_to_roll(quat_to_R(f.q))
    return np.rad2deg(phi), np.rad2deg(roll_accel), np.rad2deg(roll), t


def test_pitch_accel(fs=50.0, ax=1.0, dur=20.0):
    """水平直進しながら加速。真のピッチは 0 だが加速度計は機首上げと誤認する。"""
    rng = np.random.default_rng(1)
    n = int(dur * fs)
    t = np.arange(n) / fs
    R = np.eye(3)                                  # 水平・東向き・ピッチ0
    v_true = np.zeros((n, 3))
    v_true[:, 0] = 8.0 + ax * t                    # 東向きに加速
    a_world = np.zeros((n, 3)); a_world[:, 0] = ax

    accel = np.zeros((n, 3)); gyro = np.zeros((n, 3))
    for i in range(n):
        accel[i] = R.T @ (a_world[i] - G_ENU)
    accel_m = accel + rng.normal(0, 3e-2, (n, 3))
    gyro_m = gyro + rng.normal(0, 2e-3, (n, 3))

    pitch_accel = np.arctan2(accel_m[:, 0], accel_m[:, 2])

    f = AttitudeESKF()
    f.init(q0=[1, 0, 0, 0], v0=v_true[0])
    pitch = np.zeros(n)
    for i in range(n):
        f.predict(gyro_m[i], accel_m[i], 1.0 / fs)
        if i % int(fs / 2) == 0:
            f.update_velocity(v_true[i] + rng.normal(0, 0.05, 3), sacc=0.08)
        pitch[i] = R_to_pitch(quat_to_R(f.q))

    settled = t > 8.0
    pa = np.rad2deg(pitch_accel[settled]).mean()
    pe = np.rad2deg(pitch[settled]).mean()
    print()
    print(f"== 水平直進 + 加速 {ax} m/s^2 (真のピッチ = 0) ==")
    print(f"  加速度計のみ(BNO085相当): {pa:6.2f} deg  (誤差 {pa:+.2f})")
    print(f"  ESKF                  : {pe:6.2f} deg  (誤差 {pe:+.2f})")
    return abs(pe) < 1.0 and abs(pa) > 2.0


def test_gnss_rate():
    """GNSS 2Hz と 10Hz の差を測る（10Hz 化が必要かの判断材料）。"""
    print()
    print("== GNSS レートの影響（協調旋回 bank=20deg）==")
    print(f"  {'GNSS':>6}{'定常誤差[deg]':>16}{'ばらつき[deg]':>16}{'収束時間[s]':>14}")
    res = {}
    for hz in (2.0, 5.0, 10.0):
        truth, _, roll, t = run_turn(gnss_hz=hz)
        settled = t > 15.0
        err = roll[settled].mean() - truth
        std = roll[settled].std()
        # 収束時間: 誤差が 1deg 以内に入って以降ずっと保たれる最初の時刻
        within = np.abs(roll - truth) < 1.0
        conv = t[-1]
        for i in range(len(t)):
            if within[i:].all():
                conv = t[i]; break
        res[hz] = (err, std, conv)
        print(f"  {hz:5.0f}Hz{err:>16.3f}{std:>16.3f}{conv:>14.1f}")
    return res


def main():
    rng = np.random.default_rng(0)
    fs = 50.0
    t, R_true, v_true, gyro, accel, phi = simulate(fs=fs)
    n = len(t)

    # ---- 前提の確認: 協調旋回では比力の横成分がゼロ ----
    lat = np.abs(accel[n//2, 1])
    mag = np.linalg.norm(accel[n//2])
    print("== 前提確認（旋回中の比力）==")
    print(f"  横成分 |f_y|        : {lat:.4f} m/s^2  (0 なら加速度計にバンクは見えない)")
    print(f"  大きさ |f|          : {mag:.4f} m/s^2  (理論値 g/cos(phi) = {GRAVITY/np.cos(phi):.4f})")

    # ---- センサー誤差を載せる ----
    bg_true = np.deg2rad([0.5, -0.3, 0.2])      # ジャイロバイアス
    gyro_m = gyro + bg_true + rng.normal(0, 2e-3, (n, 3))
    accel_m = accel + rng.normal(0, 3e-2, (n, 3))

    # ---- 加速度計だけで水平出し（BNO085 が長期的に収束する先）----
    roll_accel = np.arctan2(-accel_m[:, 1], accel_m[:, 2])

    # ---- ESKF ----
    f = AttitudeESKF()
    f.init(q0=[1, 0, 0, 0], v0=v_true[0])
    # 姿勢の初期値はわざと水平（真値と 20 度ずれた状態）から始めて収束を見る
    from eskf import quat_norm
    w0 = np.sqrt(0.5)
    f.q = quat_norm(np.array([1.0, 0.0, 0.0, 0.0]))

    roll_eskf = np.zeros(n)
    gnss_every = int(fs / 2)                     # GNSS 2Hz（現状の実機設定）
    for i in range(n):
        f.predict(gyro_m[i], accel_m[i], 1.0 / fs)
        if i % gnss_every == 0:
            v_meas = v_true[i] + rng.normal(0, 0.05, 3)
            f.update_velocity(v_meas, sacc=0.08)
        roll_eskf[i] = R_to_roll(quat_to_R(f.q))

    settled = t > 15.0                            # 収束後の区間で評価
    truth_deg = np.rad2deg(phi)
    eskf_deg = np.rad2deg(roll_eskf[settled])
    accel_deg = np.rad2deg(roll_accel[settled])

    print()
    print("== 定常協調旋回 (V=8m/s, bank=20deg, GNSS 2Hz) ==")
    print(f"  真のバンク角          : {truth_deg:6.2f} deg")
    print(f"  加速度計のみ(BNO085相当): {accel_deg.mean():6.2f} deg  "
          f"(誤差 {accel_deg.mean()-truth_deg:+.2f})")
    print(f"  ESKF                  : {eskf_deg.mean():6.2f} deg  "
          f"(誤差 {eskf_deg.mean()-truth_deg:+.2f}, std {eskf_deg.std():.2f})")
    print()
    print(f"  推定ジャイロバイアス   : {np.rad2deg(f.bg)} deg/s")
    print(f"  真のジャイロバイアス   : {np.rad2deg(bg_true)} deg/s")

    ok = abs(eskf_deg.mean() - truth_deg) < 1.0 and abs(accel_deg.mean()) < 2.0

    ok_pitch = test_pitch_accel()
    test_gnss_rate()

    print()
    print("結果: " + ("PASS — 旋回中のロールも加減速中のピッチも ESKF が復元できる"
                      if (ok and ok_pitch) else "FAIL"))
    return 0 if (ok and ok_pitch) else 1


if __name__ == "__main__":
    raise SystemExit(main())
