#!/usr/bin/env python3
"""
GNSS 速度援用 姿勢 ESKF（誤差状態カルマンフィルタ）。

■ 目的
BNO085 内蔵フュージョンは加速度計が測る「比力」を鉛直とみなすため、
  ・定常旋回中はロールを過小評価する（協調旋回では比力が機体 -Z 一直線になり、
    加速度計からはバンクがまったく見えない）
  ・加減速中はピッチがずれる
これを直すには GNSS 速度から対地加速度を与え、比力から遠心力・加減速分を
差し引いて真の重力方向を求める必要がある。それがこのフィルタの役割。

■ 座標系（重要）
  ワールド : ENU（X=East, Y=North, Z=Up）。BNO085 (SH2) のワールド系に合わせてある。
  ボディ   : BNO085 センサー軸そのもの。
  クォータニオン q は body → ENU。
  → フィルタ内部でマウント（センサー軸と機体軸のズレ）を意識する必要がない。
    機体軸の roll/pitch/yaw は出力時に decode_imulog の補正を通して得る。
  GNSS は NED で記録されているので [E,N,U] = [velE, velN, -velD] に変換して使う。

■ 状態
  公称: q (body→ENU), v_ENU, bg (ジャイロバイアス), ba (加速度バイアス)
  誤差 12: [dtheta(3), dv(3), dbg(3), dba(3)]
  誤差回転はワールド系（global error）で定義する: R_true = (I + [dtheta]x) R_nominal

■ 観測
  GNSS 速度のみ（対気速度センサーは無い方針）。R は sAcc から作る。

■ ヨーの可観測性について
  「対気速度が無いとヨーが出ない」というのは GNSS の対地コースをそのまま
  機首方位とみなす場合の話。このフィルタでは加速度計と速度の整合から姿勢全体が
  拘束されるため、水平加速度があるとき（＝旋回中）はヨーも可観測になる。
  等速直進中は不可観測でジャイロバイアス分だけ漂うが、旋回のたびに引き戻される。
  カニ角（機首方位と対地コースの差）もこの原理でヨー側に現れる。
"""

import numpy as np

GRAVITY = 9.80665
G_ENU = np.array([0.0, 0.0, -GRAVITY])   # ENU なので重力は -Z


# ---------------- クォータニオン ユーティリティ ----------------
# 規約: q = [w, x, y, z]、body → world の回転を表す。

def skew(v):
    x, y, z = v
    return np.array([[0.0, -z,  y],
                     [z,  0.0, -x],
                     [-y,  x, 0.0]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_norm(q):
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    return -q if q[0] < 0.0 else q      # w>=0 に正規化（符号の暴れを防ぐ）


def quat_from_rotvec(r):
    """回転ベクトル（軸×角[rad]）→ クォータニオン。微小角でも安定な形。"""
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.array([1.0, 0.5*r[0], 0.5*r[1], 0.5*r[2]])
    axis = r / theta
    s = np.sin(theta * 0.5)
    return np.array([np.cos(theta * 0.5), axis[0]*s, axis[1]*s, axis[2]*s])


def quat_to_R(q):
    """body → world の回転行列。"""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z),     2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


# ============================================================
# 水平化と航跡ヨー（機上 attitude.cpp の level_toward_gravity /
# init_yaw_from_track と同じ数式。★ 片方を直したら両方直すこと）
# ============================================================
def level_quat_from_accel(a):
    """静止中の比力 a（センサー軸）から「センサー軸 → ENU」の姿勢を作る。

    R(q)·â = ẑ となる最小回転。**ロール・ピッチだけが決まり、ヨーは任意。**
    u→v の最小回転は q = normalize([1 + u·v, u×v])。v = ẑ なので
      q = normalize([1 + az_n, ay_n, -ax_n, 0])
    0.983 まで BNO085 の GRV を初期姿勢に使っていたが、実ログで
    ロール・ピッチが 0.05 度以内で一致したので置き換えた。
    """
    a = np.asarray(a, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    ax, ay, az = a / n
    if az < -0.999999:              # 真逆さ。軸が決まらないので任意の水平軸で 180 度
        return np.array([0.0, 1.0, 0.0, 0.0])
    return quat_norm(np.array([1.0 + az, ay, -ax, 0.0]))


def level_toward_gravity(q, a, gain):
    """測った比力 a（センサー軸）へ姿勢を寄せる。機上 level_toward_gravity と同じ。

    gain=1 で一発、gain<1 で相補フィルタ。**回転軸は必ず水平になるのでヨーは動かない。**
    """
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-3:
        return q
    gw = quat_to_R(q) @ (np.asarray(a, dtype=float) / n)
    ax, ay = gw[1], -gw[0]
    sn = np.hypot(ax, ay)
    if sn < 1e-7:
        if gw[2] > 0.0:
            return q
        ax, ay, sn = 1.0, 0.0, 1.0        # 真逆さ。任意の水平軸で 180 度
    ang = np.arctan2(sn, gw[2]) * gain
    rot = np.array([ax / sn * ang, ay / sn * ang, 0.0])
    return quat_norm(quat_mul(quat_from_rotvec(rot), q))


def set_heading(q, want_deg):
    """出力の真方位が want_deg になるようワールド Z 軸まわりに回す。

    ロール・ピッチは動かない（Rz(d)·Rz(psi)Ry(th)Rx(ph) = Rz(d+psi)Ry(th)Rx(ph)）。
    出力方位は heading = 90 - yaw_math[deg] なので alpha = -(want - now)。
    """
    from decode_imulog import mount_correct_quat
    q = np.asarray(q, dtype=float)
    _, _, y = mount_correct_quat(q[0], q[1], q[2], q[3])
    now = float(np.atleast_1d(y)[0])
    d = (want_deg - now + 180.0) % 360.0 - 180.0
    a = -np.deg2rad(d)
    qz = np.array([np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)])
    return quat_norm(quat_mul(qz, q))


class AttitudeESKF:
    """GNSS 速度で援用する 12 誤差状態の姿勢 ESKF。

    チューニングパラメータ（既定値は BNO085 の実力と HPA の運動を想定した初期値。
    実飛行ログで詰めること）:
      sigma_g  : ジャイロ白色雑音 [rad/s/sqrt(Hz)]
      sigma_a  : 加速度計白色雑音 [m/s^2/sqrt(Hz)]
      sigma_bg : ジャイロバイアスのランダムウォーク [rad/s^2/sqrt(Hz)]
      sigma_ba : 加速度バイアスのランダムウォーク [m/s^3/sqrt(Hz)]
    """

    def __init__(self, sigma_g=2e-3, sigma_a=3e-2, sigma_bg=1e-5, sigma_ba=1e-4,
                 estimate_accel_bias=True,
                 max_gyro_bias=np.deg2rad(1.0), max_accel_bias=1.0):
        self.sg, self.sa = sigma_g, sigma_a
        self.sbg, self.sba = sigma_bg, sigma_ba
        self.estimate_accel_bias = estimate_accel_bias
        # バイアス推定の物理的な上限。
        # BNO085 は校正済みのジャイロ・加速度を出すので、残留バイアスは本来ごく小さい。
        # 上限を置かないと、初期姿勢誤差の行き場としてバイアス状態が使われ、
        # 物理的にあり得ない値に固着して二度と戻らないことがある
        # （2026-08-18 の session 2 で bg が 5.9deg/s に張り付き、姿勢補正が効かなくなった。
        #   同じ過渡は session 1 でも +3.1deg/s まで出たが、そちらは 120 秒で収束していた）。
        self.max_bg = max_gyro_bias
        self.max_ba = max_accel_bias

        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.v = np.zeros(3)
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)

        self.P = np.eye(12)
        self.P[0:3, 0:3] *= np.deg2rad(10.0) ** 2   # 姿勢の初期不確かさ
        self.P[3:6, 3:6] *= 1.0 ** 2                # 速度 [m/s]
        self.P[6:9, 6:9] *= np.deg2rad(1.0) ** 2    # ジャイロバイアス
        self.P[9:12, 9:12] *= 0.1 ** 2              # 加速度バイアス

    def reset_covariance(self, att_sigma_deg, yaw_sigma_deg):
        """機上 attitude.cpp の reset_covariance() と同じ値を置く。
        ★ 片方を直したらもう片方も直すこと。"""
        self.P = np.zeros((12, 12))
        a = np.deg2rad(att_sigma_deg)
        yy = np.deg2rad(yaw_sigma_deg)
        self.P[0, 0] = self.P[1, 1] = a * a
        self.P[2, 2] = yy * yy
        for i in range(3, 6):
            self.P[i, i] = 1.0                      # 速度 [m/s]^2
        for i in range(6, 9):
            self.P[i, i] = np.deg2rad(1.0) ** 2     # ジャイロバイアス
        for i in range(9, 12):
            self.P[i, i] = 0.01                     # 加速度バイアス

    def init(self, q0, v0=None, att_sigma_deg=None, yaw_sigma_deg=None):
        self.q = quat_norm(np.asarray(q0, dtype=float))
        self.v = np.zeros(3)
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)
        if att_sigma_deg is not None:
            self.reset_covariance(att_sigma_deg, yaw_sigma_deg)
        if v0 is not None:
            self.v = np.asarray(v0, dtype=float)

    # ---------------- 伝播 ----------------
    def predict(self, gyro, accel, dt):
        """gyro [rad/s], accel [m/s^2]（重力込みの生比力）をボディ系で受け取る。"""
        if dt <= 0.0 or dt > 0.5:
            return
        w = np.asarray(gyro, float) - self.bg
        f = np.asarray(accel, float) - self.ba
        R = quat_to_R(self.q)

        # 公称状態
        self.q = quat_norm(quat_mul(self.q, quat_from_rotvec(w * dt)))
        a_world = R @ f + G_ENU
        self.v = self.v + a_world * dt

        # 誤差状態の遷移（global error 定義）
        #   d(dtheta)/dt = -R dbg
        #   d(dv)/dt     = -[R f]x dtheta - R dba
        F = np.zeros((12, 12))
        F[0:3, 6:9] = -R
        F[3:6, 0:3] = -skew(R @ f)
        F[3:6, 9:12] = -R
        Phi = np.eye(12) + F * dt

        Q = np.zeros((12, 12))
        Q[0:3, 0:3] = np.eye(3) * (self.sg ** 2) * dt
        Q[3:6, 3:6] = np.eye(3) * (self.sa ** 2) * dt
        Q[6:9, 6:9] = np.eye(3) * (self.sbg ** 2) * dt
        Q[9:12, 9:12] = np.eye(3) * (self.sba ** 2) * dt

        self.P = Phi @ self.P @ Phi.T + Q

    # ---------------- 観測（GNSS 速度）----------------
    def update_velocity(self, v_enu, sacc):
        """v_enu: ENU の速度観測 [m/s]、sacc: u-blox の速度精度推定 [m/s]。"""
        H = np.zeros((3, 12))
        H[:, 3:6] = np.eye(3)
        r = max(float(sacc), 0.05)          # sAcc が極端に小さいと過信するので下限を置く
        Rm = np.eye(3) * (r ** 2)

        y = np.asarray(v_enu, float) - self.v
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y

        I_KH = np.eye(12) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T   # Joseph 形（対称性を保つ）
        self._inject(dx)

    def _inject(self, dx):
        dtheta = dx[0:3]
        # global error なので左から掛ける
        self.q = quat_norm(quat_mul(quat_from_rotvec(dtheta), self.q))
        self.v += dx[3:6]
        self.bg += dx[6:9]
        if self.estimate_accel_bias:
            self.ba += dx[9:12]
        # 物理的にあり得ない大きさへ発散させない（方向は保ったまま大きさだけ制限する）
        for attr, lim in (("bg", self.max_bg), ("ba", self.max_ba)):
            v = getattr(self, attr)
            n = np.linalg.norm(v)
            if n > lim:
                setattr(self, attr, v * (lim / n))


# ============================================================
# 実ログへの適用
# ============================================================

def _map_host_to_sensor_time(imu_df):
    """互換のため残しているが、現在 ts は decode_imulog.reconstruct_time() が
    ホスト時刻と同じ時計の上で復元しているため、変換は恒等でよい。
    （以前は BNO085 の sv.timestamp を別時計として扱っていたが、
      あの値は sh2 のアンダーフローで壊れていたので使うのをやめた）
    """
    return lambda x: np.asarray(x, dtype=float)


def run_on_log(parts, sigma_kw=None, sacc_max=1.0, use_gnss=True,
               gnss_cutoff=None, init_from_grv=False):
    """decode_imulog.split() の結果に ESKF を適用して姿勢時系列を返す。

    セッション（起動）ごとに独立して処理し、フィルタも都度初期化する。
    ログは O_APPEND なので 1 ファイルに複数回の起動分が入り、境界で時刻が 0 に戻る。
    跨いで回すと GNSS 観測が適用されなくなり姿勢が発散する（2026-08-18 に実際に発生）。

    use_gnss: False にすると速度観測を一切入れない（純粋なジャイロ/加速度の積分）。
      GNSS 援用の効果を見るための比較用。観測が無いと姿勢を引き戻すものが何も無いので、
      初期姿勢＋ジャイロ積分のまま漂う。加速度計は伝播にしか入らないため、
      BNO085 のように比力へ引き寄せられることもない（＝別種の壊れ方をする）。

    gnss_cutoff: ホスト時刻 t がこの値を超えたら GNSS 観測を止める（None で常時使う）。
      「飛行中に GNSS が途絶したら姿勢がどれだけ崩れるか」を測るために使う。
      セッションごとに時刻が 0 付近へ戻るため、{セッション番号: 打ち切り時刻} の
      辞書でも渡せる（スカラーを渡すと全セッションに同じ値を使う）。

    sacc_max: GNSS 速度を採用する sAcc の上限 [m/s]。
      フィックスが悪いと u-blox は数百 m/s の速度解を返すことがあり、
      そのまま観測に入れるとフィルタが壊れる（実ログで最大 248m/s を確認）。
      なお機上のバリオ KF は settings.h の GNSS_VSI_SACC_MAX_MPS=0.3 を使っている。

    戻り値の列:
      session            : 起動ごとの通し番号
      t, ts              : 時刻 [s]
      s_roll/s_pitch/s_yaw : センサー座標系のオイラー角 [deg]（フィルタの素の出力）
      roll/pitch/yaw     : マウント補正後の機体軸オイラー角 [deg]
      yaw_sigma          : ヨーの推定標準偏差 [deg]（共分散 P の該当対角成分）
        ヨーは水平加速度がある間しか可観測にならないため、等速直進が続くと育つ。
        機上では表示にしか使っていないので、飛行後の検証はここから読む。
    """
    import pandas as pd
    from decode_imulog import euler_from_quat, mount_correct_quat

    gyro, accel = parts.get("gyro"), parts.get("accel")
    if gyro is None or accel is None or gyro.empty or accel.empty:
        raise SystemExit("gyro / accel レコードが無い。"
                         "settings.h の IMULOG_RAW_REPORTS_ENABLED を 1 にして記録すること。")

    gnss_all = parts.get("gnssvel")
    grv_all = parts.get("gamerv")
    out = []

    for sess in sorted(gyro["session"].unique()):
        g = gyro[gyro["session"] == sess]
        a = accel[accel["session"] == sess]
        if len(g) < 2 or a.empty:
            continue

        # ジャイロと加速度は別レコードで届くのでセンサー時刻で最近傍ペアリングする
        imu = pd.merge_asof(g.sort_values("ts"),
                            a[["ts", "ax", "ay", "az"]].sort_values("ts"),
                            on="ts", direction="nearest")
        imu = imu.dropna(subset=["ax", "ay", "az"]).reset_index(drop=True)
        if len(imu) < 2:
            continue

        to_ts = _map_host_to_sensor_time(imu)

        g_ts = np.array([])
        if use_gnss and gnss_all is not None and not gnss_all.empty:
            gn = gnss_all[(gnss_all["session"] == sess) &
                          (gnss_all["sAcc"] <= sacc_max)].sort_values("t")
            if not gn.empty:
                g_ts = to_ts(gn["t"].to_numpy())
                # GNSS は NED で記録してある。フィルタは ENU なので変換する。
                g_enu = np.column_stack([gn["velE"].to_numpy(),
                                         gn["velN"].to_numpy(),
                                         -gn["velD"].to_numpy()])
                g_sacc = gn["sAcc"].to_numpy()
        if not len(g_ts) and use_gnss:
            print(f"warn: session {sess} に使える GNSS 速度が無い"
                  f"（sAcc<={sacc_max} で 0 件）。姿勢は漂う", flush=True)

        cutoff = (gnss_cutoff.get(sess) if isinstance(gnss_cutoff, dict)
                  else gnss_cutoff)

        f = AttitudeESKF(**(sigma_kw or {}))
        # ---- 初期姿勢 ----
        # ★ 0.983 で GRV をやめ、機上と同じ「静止中の平均加速度で水平化 →
        #   GS がしきい値を超えた最初の測位の航跡でヨーを入れる」に合わせた。
        #   GRV を使いたい（旧挙動と比べたい）ときは init_from_grv=True を渡す。
        i_start = 0
        yaw_set = True          # GRV 経路は「ヨーあり」として扱う（旧挙動の再現用）
        static_flags = None
        if init_from_grv and grv_all is not None and not grv_all.empty:
            gv = grv_all[grv_all["session"] == sess]
            if not gv.empty:
                f.init(q0=[gv.qw.iloc[0], gv.qx.iloc[0], gv.qy.iloc[0], gv.qz.iloc[0]])
        else:
            # ---- 機上と同じ手順 ----
            #   1) 静止 2 秒の平均加速度で水平化（ヨーは未知）
            #   2) 静止しているあいだ再水平化＋ジャイロバイアス学習（ループ内）
            #   3) GS >= 3m/s を 2 測位連続で満たしたら航跡でヨーを注入（ループ内）
            # ★ attitude.cpp と同じ数式・同じ順序であること。片方を直したら両方直す。
            gyv = imu[["gx", "gy", "gz"]].to_numpy(float)
            acv = imu[["ax", "ay", "az"]].to_numpy(float)
            static_flags = ((np.linalg.norm(gyv, axis=1) < 0.026) &
                            (np.abs(np.linalg.norm(acv, axis=1) - 9.80665) < 0.30))
            tsv = imu["ts"].to_numpy(float)
            i_lv = run0 = None
            for i in range(len(tsv)):
                if static_flags[i]:
                    if run0 is None:
                        run0 = i
                    elif tsv[i] - tsv[run0] >= 2.0:
                        i_lv = i
                        break
                else:
                    run0 = None
            if i_lv is None:
                print(f"warn: session {sess} に 2 秒の静止区間が無い。"
                      f"先頭の比力で水平化する（機上の走行中フォールバック相当）")
                a0, sig = acv[0], 30.0            # ESKF_INIT_ATT_SIGMA_MOVING_DEG
            else:
                a0, sig = acv[run0:i_lv + 1].mean(axis=0), 3.0   # ESKF_INIT_ATT_SIGMA_DEG
                i_start = i_lv
            f.init(q0=level_quat_from_accel(a0), att_sigma_deg=sig, yaw_sigma_deg=180.0)
            yaw_set = False

        ts = imu["ts"].to_numpy(float)
        w = imu[["gx", "gy", "gz"]].to_numpy(float)
        acc = imu[["ax", "ay", "az"]].to_numpy(float)

        # ★ **水平化した時点より前は伝播しない。**
        #   機上は初期化が完了するまで伝播しないので（attitude_on_gyro の
        #   `if (!initialized_) return;`）、ここも合わせる。
        #   合わせないと「後半の静止区間で求めた姿勢」を先頭に置いて回すことになり、
        #   GNSS が無いセッションでは丸ごとずれる（実際に踏んだ）。
        n_out = len(ts) - i_start
        qs = np.zeros((n_out, 4))
        ysig = np.zeros(n_out)
        gi = 0
        while gi < len(g_ts) and g_ts[gi] < ts[i_start]:
            gi += 1                      # 初期化前の測位は捨てる
        bias_sum = np.zeros(3)
        bias_n = 0
        yaw_arm = 0
        for k, i in enumerate(range(i_start, len(ts))):
            dt = ts[i] - ts[i - 1] if i else 0.0
            # ---- 待機中の維持（機上 attitude_on_gyro の該当ブロックと同じ）----
            if (not yaw_set) and static_flags is not None and static_flags[i]:
                f.q = level_toward_gravity(f.q, acc[i], 0.02)     # ESKF_RELEVEL_GAIN
                bias_sum += w[i]
                bias_n += 1
                if bias_n >= 250:                                # ESKF_BIAS_LEARN_SAMPLES
                    f.bg = np.clip(bias_sum / bias_n, -0.0087, 0.0087)
                    bias_sum[:] = 0.0
                    bias_n = 0
                f.reset_covariance(3.0, 180.0)
            elif static_flags is not None and not static_flags[i]:
                bias_sum[:] = 0.0
                bias_n = 0
            f.predict(w[i], acc[i], dt)
            while gi < len(g_ts) and g_ts[gi] <= ts[i]:
                if not yaw_set:
                    sp = float(np.hypot(g_enu[gi][0], g_enu[gi][1]))
                    if sp < 3.0:                                 # ESKF_YAW_INIT_MIN_SPEED_MPS
                        yaw_arm = 0
                    else:
                        yaw_arm += 1
                        if yaw_arm >= 2:                         # ESKF_YAW_INIT_ARM_FIXES
                            trk = np.degrees(np.arctan2(g_enu[gi][0], g_enu[gi][1])) % 360.0
                            f.q = set_heading(f.q, trk)
                            f.P[2, :] = 0.0
                            f.P[:, 2] = 0.0
                            f.P[2, 2] = np.deg2rad(15.0) ** 2    # ESKF_YAW_INIT_SIGMA_DEG
                            f.v = g_enu[gi].copy()
                            yaw_set = True
                elif cutoff is None or g_ts[gi] <= cutoff:
                    f.update_velocity(g_enu[gi], g_sacc[gi])
                gi += 1
            qs[k] = f.q
            ysig[k] = np.degrees(np.sqrt(max(f.P[2, 2], 0.0)))
        imu = imu.iloc[i_start:].reset_index(drop=True)
        ts = ts[i_start:]

        s_roll, s_pitch, s_yaw = euler_from_quat(qs[:, 0], qs[:, 1], qs[:, 2], qs[:, 3])
        roll, pitch, yaw = mount_correct_quat(qs[:, 0], qs[:, 1], qs[:, 2], qs[:, 3])
        out.append(pd.DataFrame({
            "session": sess,
            "t": imu["t"].to_numpy(), "ts": ts,
            "s_roll": np.degrees(s_roll), "s_pitch": np.degrees(s_pitch),
            "s_yaw": np.degrees(s_yaw),
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "yaw_sigma": ysig,
        }))

    if not out:
        raise SystemExit("処理できるセッションが無かった")
    return pd.concat(out, ignore_index=True)


def main():
    import argparse, os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_imulog import load, split, build_euler
    import pandas as pd

    ap = argparse.ArgumentParser(description="生 IMU ログに姿勢 ESKF を適用する")
    ap.add_argument("path")
    ap.add_argument("--csv", metavar="FILE", help="結果を CSV で書き出す")
    args = ap.parse_args()

    parts = split(load(args.path))
    est = run_on_log(parts)

    # BNO085 内蔵フュージョンの結果と並べる（旋回中の差がこの計画の狙い）
    # ※ セッションごとに t が 0 に戻るので、突き合わせも必ずセッション単位で行う。
    #   全体を通して merge_asof すると別セッションの行に接続されてしまう。
    bno = build_euler(parts)
    if bno is not None:
        bno = bno.rename(columns={"roll": "bno_roll", "pitch": "bno_pitch",
                                  "yaw": "bno_yaw"})
        merged = []
        for sess, e in est.groupby("session"):
            b = bno[bno["session"] == sess]
            if b.empty:
                merged.append(e)
                continue
            merged.append(pd.merge_asof(
                e.sort_values("t"),
                b[["t", "bno_roll", "bno_pitch", "bno_yaw"]].sort_values("t"),
                on="t", direction="nearest"))
        est = pd.concat(merged, ignore_index=True)

        print(f"サンプル数 : {len(est)}   セッション数 : {est['session'].nunique()}")
        print(f"{'sess':>5}{'件数':>9}{'長さ[s]':>10}"
              f"{'ロール差 平均':>15}{'最大':>9}")
        for sess, e in est.groupby("session"):
            if "bno_roll" not in e or e["bno_roll"].isna().all():
                continue
            d = (e["roll"] - e["bno_roll"]).abs()
            dur = e["ts"].max() - e["ts"].min()
            print(f"{sess:>5}{len(e):>9}{dur:>10.1f}{d.mean():>15.2f}{d.max():>9.2f}")
        print("  ※ 旋回中に大きく開いていれば、それが BNO085 の過小評価分。")

    if args.csv:
        est.to_csv(args.csv, index=False)
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
