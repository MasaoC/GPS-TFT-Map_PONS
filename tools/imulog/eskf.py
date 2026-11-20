#!/usr/bin/env python3
"""
GNSS 速度援用 姿勢 ESKF（誤差状態カルマンフィルタ）。

■ 目的
加速度計が測る「比力」を鉛直とみなすだけの AHRS では、
  ・定常旋回中はロールを過小評価する（協調旋回では比力が機体 -Z 一直線になり、
    加速度計からはバンクがまったく見えない）
  ・加減速中はピッチがずれる
これを直すには GNSS 速度から対地加速度を与え、比力から遠心力・加減速分を
差し引いて真の重力方向を求める必要がある。それがこのフィルタの役割。

■ 座標系（重要）
  ワールド : ENU（X=East, Y=North, Z=Up）。
  ボディ   : IMU のセンサー軸そのもの。
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
    v7 の実ログで、チップ内融合のロール・ピッチと 0.05 度以内で一致した。
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

    gain=1 で一発、gain<1 で相補フィルタ。**回転軸は必ず水平になる**ので、ずれが小さいときはヨーにほとんど触らない。
    ★ ずれが大きいと方位（出力のヨー）は振れる（11/11 のトンネルで 42.7 → 110.5 度）。方位を保つなら後で set_heading()。
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


def world_z_rot(q_old, q_new):
    """q_new = Rz(a)·q_old のときの a [rad]（評価用。純粋に z まわりの回転のときだけ正しい）。"""
    dq = quat_mul(np.asarray(q_new, float), np.array([q_old[0], -q_old[1], -q_old[2], -q_old[3]]))
    return float(2.0 * np.arctan2(dq[3], dq[0]))


class AttitudeESKF:
    """GNSS 速度で援用する 12 誤差状態の姿勢 ESKF。

    チューニングパラメータ（既定値は機上 settings.h の ESKF_SIGMA_* と同じ。2026-11-18 に sigma_g 2e-3 → 5e-4、
    sigma_bg 1e-5 → 3e-6 へ SCH16T に合わせて下げた。理由と数字は settings.h のコメント）:
      sigma_g  : ジャイロ白色雑音 [rad/s/sqrt(Hz)]
      sigma_a  : 加速度計白色雑音 [m/s^2/sqrt(Hz)]
      sigma_bg : ジャイロバイアスのランダムウォーク [rad/s^2/sqrt(Hz)]
      sigma_ba : 加速度バイアスのランダムウォーク [m/s^3/sqrt(Hz)]
    ★ 2026-11-17 より前のログを当時の機上と比べるときは sigma_g=2e-3, sigma_bg=1e-5 を渡す（ゼロ点の学習の直しは戻せない）。
    """

    def __init__(self, sigma_g=5e-4, sigma_a=3e-2, sigma_bg=3e-6, sigma_ba=1e-4,
                 estimate_accel_bias=True,
                 max_gyro_bias=np.deg2rad(1.0), max_accel_bias=1.0):
        self.sg, self.sa = sigma_g, sigma_a
        self.sbg, self.sba = sigma_bg, sigma_ba
        self.estimate_accel_bias = estimate_accel_bias
        # バイアス推定の物理的な上限。
        # 工場校正のチップなので、残留バイアスは本来ごく小さい。
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
        # ヨーの分散だけ頭打ちにする（機上 attitude_on_gyro() と同じ。settings.h の ESKF_YAW_SIGMA_MAX_DEG = 180）。
        # 2026-11-10 まで無く、ヨーの観測が無い区間で機上より大きく育っていた。縮む側は触らない。
        yvar_max = np.deg2rad(180.0) ** 2
        if self.P[2, 2] > yvar_max:
            self.P[2, 2] = yvar_max

    # ---------------- 観測（GNSS 速度）----------------
    def update_velocity(self, v_enu, sacc, v_pred=None):
        """v_enu: ENU の速度観測 [m/s]、sacc: u-blox の速度精度推定 [m/s]。

        v_pred: 観測と比べる推定速度。None ならいまの速度。GNSS の遅れを補償するときは
          「測位の瞬間の推定速度」（速度の履歴から引いたもの）を渡す（機上 attitude_on_gnss_velocity の
          age_us と vhist_。settings.h の ESKF_GNSS_VEL_DELAY_MS）。直しはいまの状態に入れる。
        """
        H = np.zeros((3, 12))
        H[:, 3:6] = np.eye(3)
        r = max(float(sacc), 0.05)          # sAcc が極端に小さいと過信するので下限を置く
        Rm = np.eye(3) * (r ** 2)

        y = np.asarray(v_enu, float) - (self.v if v_pred is None else np.asarray(v_pred, float))
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y

        I_KH = np.eye(12) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T   # Joseph 形（対称性を保つ）
        self._inject(dx)

    def update_level(self, accel, sigma_mps2):
        """重力観測（levelling update）。**機上 attitude.cpp の level_update() と同じ。**

        準定常のとき、比力をワールドへ回した f_w は (0,0,g) になるはず。
        水平 2 成分を観測量にして、その残差から dtheta の水平成分を直す。

        誤差回転はワールド系（R_true = (I + [dtheta]x) R_nom）なので
            f_w_true = (I + [dtheta]x) f_w = f_w + dtheta x f_w = f_w - [f_w]x dtheta
        f_w ~ (0,0,g) のとき
            d f_w_x = +g * dtheta_y,   d f_w_y = -g * dtheta_x
        よって H は 2x12 で H[0][1] = +g, H[1][0] = -g だけが非零。

        ★ **dba は H に入れない。** 水平の加速度バイアスと傾き誤差は原理的に縮退して
          いるので、入れると長い GNSS 断で ba が汚れる。入れない代償は
          「水平 ba を全部傾きのせいにする」ぶんの偏りで、SCH16T のオフセット
          ±0.01 m/s2 typ なら 0.06 度、max 0.06 m/s2 でも 0.35 度。無視できる。
          そのぶんは sigma に含める。
        ★ ゲート（旋回レート等）は呼び出し側で判定する。ここは観測の計算だけ。
        """
        g = 9.80665
        fw = quat_to_R(self.q) @ (np.asarray(accel, float) - self.ba)
        # ★ H は f_w ≒ (0,0,+g) のまわりで線形化してある。下向きなら符号が逆に
        #   なり補正が誤差を広げるので入れない（機上 level_update と同じガード）。
        if fw[2] < 0.5 * g:
            return
        H = np.zeros((2, 12))
        H[0, 1] = +g
        H[1, 0] = -g
        Rm = np.eye(2) * (float(sigma_mps2) ** 2)
        y = -fw[:2]                      # 観測は (0,0)。残差 = 0 - f_w の水平成分
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        I_KH = np.eye(12) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T   # Joseph 形
        self._inject(dx)

    def _inject(self, dx):
        dtheta = dx[0:3]
        self.yaw_fix = getattr(self, "yaw_fix", 0.0) + float(dtheta[2])   # ワールド z まわりに直した量の累計（評価用。仮のヨーの基準）
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
    """ts は decode_imulog.reconstruct_time() がホスト時刻と同じ時計の上で
    復元しているため、変換は恒等でよい。
    """
    return lambda x: np.asarray(x, dtype=float)


# 地上の小さな動きの「仮のヨー」の既定（docs/pons_yaw_init_plan.md §5。保留・機上には未実装。名前は settings.h の ESKF_YAW_PROV_* の案）
PROV_DEFAULT = dict(tau_s=60.0,      # ESKF_YAW_PROV_TAU_S
                    static_s=2.0,    # ESKF_YAW_PROV_STATIC_US
                    seg_s=8.0,       # ESKF_YAW_PROV_SEG_US
                    min_mag=130.0,   # ESKF_YAW_PROV_MIN_MAG（σ の相関の項が約 10°）
                    min_coh=0.90,    # ESKF_YAW_PROV_MIN_COH
                    k_lo=0.6, k_hi=1.5,
                    skip_frozen=True)

GNSS_DELAY_DEFAULT = 0.170    # 機上 settings.h の ESKF_GNSS_VEL_DELAY_MS と同じにしておく（None = 補償しない）[s]。2026-11-18 に 0.150 → 0.170


def itow_host_time_causal(t, itow, bucket_s=15.0, jump_s=10.0):
    """iTOW [s] を本体の時計の上へ写す（待たされなければ届いたはずの時刻）。**機上 gnss.cpp の gnss_pvt_age_ms() と同じ。**

    届いた時刻 t − iTOW は描画などで数十〜数百 ms 揺れる。その最小値（＝待たされなかったときの値）を、
    15 秒の箱 2 つ（いまの箱と 1 つ前の箱）で持つ。本体の時計のずれ（数十 ppm）は 30 秒で数 ms 以内。
    最小値から jump_s 以上離れたら（iTOW の週またぎ・時刻の飛び）数え直す。
    戻り値: (iTOW を写した時刻 m, 届きの遅れ t − m)
    """
    t = np.asarray(t, float)
    off = t - np.asarray(itow, float)
    m = np.empty(len(t))
    cur = prev = None
    b0 = None
    for i in range(len(t)):
        o = off[i]
        ref = cur if prev is None else (prev if cur is None else min(cur, prev))
        if ref is not None and abs(o - ref) > jump_s:
            cur = prev = None
            b0 = None
        if b0 is None:
            b0 = t[i]
        elif t[i] - b0 >= bucket_s:
            prev, cur, b0 = cur, None, t[i]
        cur = o if cur is None else min(cur, o)
        mn = cur if prev is None else min(cur, prev)
        m[i] = t[i] - (o - mn)
    return m, t - m


def run_on_log(parts, sigma_kw=None, sacc_max=1.0, use_gnss=True,
               gnss_cutoff=None, use_level=True, gnss_delay=GNSS_DELAY_DEFAULT,
               level_sigma=0.5, level_yawrate_dps=1.0,
               level_yawrate_inst_dps=3.0,
               level_accmag_tol=0.5, level_longacc=0.2,
               level_rate_dps=10.0, level_rate_quiet=0.5, use_ground=True,
               use_bigtilt=True, events=None, bias_ref=None, prov_kw=None):
    """decode_imulog.split() の結果に ESKF を適用して姿勢時系列を返す。

    セッション（起動）ごとに独立して処理し、フィルタも都度初期化する。
    ログは O_APPEND なので 1 ファイルに複数回の起動分が入り、境界で時刻が 0 に戻る。
    跨いで回すと GNSS 観測が適用されなくなり姿勢が発散する（2026-08-18 に実際に発生）。

    use_level: 重力観測（Phase 2 の levelling update）を入れるか。
      ★ **既定 True は機上の ESKF_LEVEL_ENABLED=1 に合わせてある。**
        機上の挙動を再現したいときは触らない。効果を見たいときだけ False にする。

    level_rate_dps / level_rate_quiet: 重力観測のゲートの全角速度 |ω| の上限 [deg/s] と、
      それを下回り続けていなければならない時間 [s]（機上 ESKF_LEVEL_MAX_RATE_DPS /
      ESKF_LEVEL_RATE_QUIET_US）。旋回レートは水平な軸まわりの回転を見ないので足した
      （2026-10-27。手で回している間に観測が通って bg が壊れた）。

    use_ground: ヨーが決まった後も、地上で止まっているときは重力へ寄せるか
      （機上 attitude_on_gyro の「地上での維持」と ESKF_GROUND_*）。
      ★ 既定 True は機上に合わせてある。効果を見たいときだけ False にする。

    use_bigtilt: 動いているときに 60 度以上ずれた傾きを比力で置き直すか（機上 ESKF_BIGTILT_*。2026-11-11〜）。
      方位は保ち、GNSS が戻ったら航跡と比べる（2026-11-12〜）。
      ★ 既定 True は機上に合わせてある。効果を見たいときだけ False にする。
    prov_kw: 辞書を渡すと（{} で既定の値）、地上の小さな動きの「仮のヨー」（計画書 §5。保留・機上には未実装）を計算して
      列 prov_yaw / prov_sigma / prov_mag / prov_coh / prov_k / prov_valid / prov_dpsi（いまの姿勢をワールド z まわりに回す量 [deg]）と
      yaw_fix（ESKF がワールド z まわりに直した量の累計 [deg]。当たり外れの基準）を足す。
      ESKF の計算は変えない（読むだけの出力）。値は PROV_DEFAULT を上書きする。
    events: リストを渡すと、出来事を (session, t, 種類, 値) で足す（検証用）。
      'relevel' … 置き直した（値 = 置き直す前の食い違い [deg]）
      'yaw_kept' / 'yaw_reset' … 保った方位を航跡と比べた（値 = 航跡 − 方位 [deg]。reset は航跡で入れ直した）

    gnss_delay: GNSS 速度の遅れ [s]（機上 ESKF_GNSS_VEL_DELAY_MS）。None なら補償しない（届いた時点の速度と比べる）。
      数字を渡すと、測位の瞬間（iTOW を本体の時計へ写した時刻 − gnss_delay）の推定速度を速度の履歴から引いて
      観測と比べる。求め方は gnss_lag.py。

    use_gnss: False にすると速度観測を一切入れない（純粋なジャイロ/加速度の積分）。
      GNSS 援用の効果を見るための比較用。観測が無いと姿勢を引き戻すものが何も無いので、
      初期姿勢＋ジャイロ積分のまま漂う。加速度計は伝播にしか入らないため、
      「比力＝鉛直」の AHRS のように比力へ引き寄せられることもない（＝別種の壊れ方をする）。

    gnss_cutoff: ホスト時刻 t がこの値を超えたら GNSS 観測を止める（None で常時使う）。
      「飛行中に GNSS が途絶したら姿勢がどれだけ崩れるか」を測るために使う。
      セッションごとに時刻が 0 付近へ戻るため、{セッション番号: 打ち切り時刻} の
      辞書でも渡せる（スカラーを渡すと全セッションに同じ値を使う）。

    sacc_max: GNSS 速度を採用する sAcc の上限 [m/s]。
      フィックスが悪いと u-blox は数百 m/s の速度解を返すことがあり、
      そのまま観測に入れるとフィルタが壊れる（実ログで最大 248m/s を確認）。
      なお機上のバリオ KF は 2026-11-12 から GNSS の速度を使っていない（settings.h の KF_GNSS_VEL_ENABLED = 0。
      それまでは GNSS_VSI_SACC_MAX_MPS = 1.0 で切っていた）。

    戻り値の列:
      session            : 起動ごとの通し番号
      t, ts              : 時刻 [s]
      s_roll/s_pitch/s_yaw : センサー座標系のオイラー角 [deg]（フィルタの素の出力）
      roll/pitch/yaw     : マウント補正後の機体軸オイラー角 [deg]
      qw/qx/qy/qz        : フィルタの姿勢クォータニオン（センサー→ENU）
        回転行列の第 3 行がセンサー軸で見た「上」で、バリオの重力除去はこれを使う。
      yaw_sigma          : ヨーの推定標準偏差 [deg]（共分散 P の該当対角成分）
        ヨーは水平加速度がある間しか可観測にならないため、等速直進が続くと育つ。
        機上では表示にしか使っていないので、飛行後の検証はここから読む。
    """
    import pandas as pd
    from decode_imulog import euler_from_quat, mount_correct_quat

    gyro, accel = parts.get("gyro"), parts.get("accel")
    if gyro is None or accel is None or gyro.empty or accel.empty:
        raise SystemExit("gyro / accel レコードが無い。"
                         "settings.h の IMULOG_DEFAULT_ENABLED を確かめること。")

    gnss_all = parts.get("gnssvel")
    out = []

    # 静止で確かめたゼロ点の「前回の値」（機上は SD の gyro_bias.txt。2026-11-18〜。ESKF_BIAS_CONFIRM_DIFF_RADS）。
    # bias_ref で最初の値を渡せる（None = 記録なし）。セッション（起動）をまたいで、機上が SD へ書く値を引き継ぐ。
    sd_bias = None if bias_ref is None else np.array(bias_ref, dtype=float)
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
        arm_fixes = 2
        if use_gnss and gnss_all is not None and not gnss_all.empty:
            ga = gnss_all[gnss_all["session"] == sess].sort_values("t")
            # 測位の時刻（機上は全部の NAV-PVT で最小値を追うので、sAcc で振り落とす前に作る）
            ga_m, _ = itow_host_time_causal(ga["t"].to_numpy(), ga["ts"].to_numpy())
            keep = (ga["sAcc"] <= sacc_max).to_numpy()
            gn = ga[keep]
            if not gn.empty:
                g_ts = to_ts(gn["t"].to_numpy())
                g_m = to_ts(ga_m[keep])
                # 方位を入れる・確かめるのに要る連続の測位の数 = 1 秒ぶん（機上 ESKF_YAW_INIT_ARM_FIXES = GNSS_NAV_RATE_HZ。
                # 2Hz のログなら 2、4Hz のログなら 4）。生ログの iTOW の間隔から作る
                d_itow = np.diff(ga["ts"].to_numpy())
                d_itow = d_itow[(d_itow > 0.05) & (d_itow < 2.0)]
                if len(d_itow):
                    arm_fixes = max(2, int(round(1.0 / float(np.median(d_itow)))))
                # GNSS は NED で記録してある。フィルタは ENU なので変換する。
                g_enu = np.column_stack([gn["velE"].to_numpy(),
                                         gn["velN"].to_numpy(),
                                         -gn["velD"].to_numpy()])
                g_sacc = gn["sAcc"].to_numpy()
        if not len(g_ts) and use_gnss:
            print(f"warn: session {sess} に使える GNSS 速度が無い"
                  f"（sAcc<={sacc_max} で 0 件）。姿勢は漂う", flush=True)

        if not len(g_ts):
            arm_fixes = 2
        cutoff = (gnss_cutoff.get(sess) if isinstance(gnss_cutoff, dict)
                  else gnss_cutoff)

        f = AttitudeESKF(**(sigma_kw or {}))
        # ---- 初期姿勢（機上と同じ手順）----
        i_start = 0
        #   1) 静止 2 秒の平均加速度で水平化（ヨーは未知）
        #   2) 静止しているあいだ再水平化＋ジャイロバイアス学習（ループ内）
        #   3) GS >= 3m/s を 2 測位連続で満たしたら航跡でヨーを注入（ループ内）
        # ★ attitude.cpp と同じ数式・同じ順序であること。片方を直したら両方直す。
        gyv = imu[["gx", "gy", "gz"]].to_numpy(float)
        acv = imu[["ax", "ay", "az"]].to_numpy(float)
        static_flags = ((np.linalg.norm(gyv, axis=1) < 0.026) &
                        (np.abs(np.linalg.norm(acv, axis=1) - 9.80665) < 0.30))
        tsv = imu["ts"].to_numpy(float)
        # ★ 機上と同じ: 信用できる GNSS が 0.5 m/s 以上と言っている間は静止の初期化をしない（8 秒続けば許す）。
        #   その前に 3 m/s 以上の測位が来たら、その時点の比力で水平化する（機上 attitude_on_gnss_velocity の「動き出したときの初期化」）。
        #   （2026-11-18 夜の車: なめらかに減速しながら 1.2 m/s で静止の判定を通り、傾きが 2〜4° ずれて初期化していた）
        g_sp = np.hypot(g_enu[:, 0], g_enu[:, 1]) if len(g_ts) else np.array([])
        i_lv = run0 = None
        i_mv = None
        gk = -1
        mov_on, mov_since = False, 0.0
        for i in range(len(tsv)):
            while gk + 1 < len(g_ts) and g_ts[gk + 1] <= tsv[i]:
                gk += 1
                if g_sp[gk] >= 3.0 and i_mv is None:                 # ESKF_YAW_INIT_MIN_SPEED_MPS
                    i_mv = i
            gfresh = gk >= 0 and tsv[i] - g_ts[gk] < 3.0
            mov = gfresh and g_sp[gk] >= max(0.2, 2.0 * g_sacc[gk])     # 機上 gnss_moving_mps()（ESKF_ACCEL_GNSS_MOVING_MPS / _SACC_K。2026-11-20〜）
            if mov:
                if not mov_on:
                    mov_on, mov_since = True, tsv[i]
            else:
                mov_on = False
            gnss_ok = (not mov_on) or (tsv[i] - mov_since >= 8.0)    # ESKF_ACCEL_HOLD_MAX_US
            if static_flags[i]:
                if run0 is None:
                    run0 = i
                elif tsv[i] - tsv[run0] >= 2.0 and gnss_ok:
                    i_lv = i
                    break
            else:
                run0 = None
            if i_mv is not None:
                break
        if i_lv is None and i_mv is not None:
            a0, sig = acv[i_mv], 30.0          # ESKF_INIT_ATT_SIGMA_MOVING_DEG
            i_start = i_mv
        elif i_lv is None:
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
        bias_sum = np.zeros(3)           # Σ(w − w0)·dt（窓の最初のサンプル w0 からの差。機上 bias_sum_）
        bias_sq = np.zeros(3)            # Σ(w − w0)²·dt（機上 bias_sq_）
        bias_w0 = np.zeros(3)
        bias_n = 0
        bias_sum_t = 0.0
        learned_on_ground = False        # 方位が決まったあとの地上の維持で覚えたか（機上 bias_learned_on_ground_）
        acc_lp = None                    # 加速度計の値の直近の平均（機上 acc_lp_。ESKF_ACCEL_CHANGE_TAU_S。ジャイロで本体と一緒に回す）
        acc_f = None                     # 加速度計の値を ESKF_ACCEL_LP_S で均したもの（機上 acc_f_）
        acc_dp_raw2 = 0.0                # 均していない値で見た直角の差の 2 乗（機上 acc_dp_raw2_）
        gstop_on, gstop_since = False, 0.0   # 信用できる GNSS が止まっていると言っている（機上 accel_gnss_stop_*）
        ah_on, ah_since = False, 0.0     # 「加速中」の条件（機上 accel_horiz_on_ / accel_horiz_since_us_）
        accelerating = False
        undo_why = 0
        # 「加速中」になる前に寄せた分の記録（機上 lvl_undo_*。0.1 秒ごとの箱: [開始時刻, rx, ry, dbg(3), dba(3)]）
        undo_boxes = []
        ground_prev = False              # 前のサンプルで地上の維持だったか（機上 ground_now_）
        yaw_arm = 0
        yaw_arm_lo = 0                   # 2 m/s 以上が連続した測位数（機上 yaw_arm_lo_。2026-11-20〜）
        # ---- 発進加速の常時前向き 2D 相関（機上 yaw_imu_hp_ / yaw_gnss_lp_ / yaw_corr_*。settings.h の ESKF_YAW_CORR_*）----
        yaw_imu_hp = np.zeros(2)
        yaw_gnss_lp = None
        yaw_gnss_last_t = None
        yaw_static_s = 0.0
        yaw_corr_c = 0.0
        yaw_corr_s = 0.0
        yaw_corr_n = 0.0                 # Σ w |v_imu| |dv_gnss|（機上 yaw_corr_n_。まとまりの分母）
        # ---- 地上の小さな動きの「仮のヨー」（2026-11-19〜。docs/pons_yaw_init_plan.md §5。保留・**機上には未実装**）----
        # 読むだけの出力。ESKF の状態（q・P・yaw_set）は一切変えない。v_imu_hp と dv_gnss は上の相関器と共用。
        prov = prov_kw is not None
        pk = dict(PROV_DEFAULT, **(prov_kw or {}))
        prov_c = prov_s = prov_n = prov_k = 0.0   # C2, S2, N2, K2
        prov_seg_until = -1e9            # 「止まっていた状態から始まった動き」の窓の終わり（静止 2 秒以上の間は今 + 8 秒）
        prov_last_add = None             # 最後に積んだ時刻（ゼロ点のずれの項）
        prov_out = np.full((n_out, 7), np.nan) if prov else None   # [yaw, sigma, mag, coh, k, valid, dpsi]
        prov_cur = np.full(7, np.nan)
        yaw_fix = np.zeros(n_out)        # ESKF がワールド z まわりに直した量の累計 [deg]（評価用。仮のヨーの当たり外れの基準）
        bias_ok_t = None                 # 静止でゼロ点を最後に覚えた時刻（機上 bias_learn_ok_us_）
        bias_ref_v = None if sd_bias is None else sd_bias.copy()   # 前回までに確かめた値（機上 bias_ref_）
        bias_prev = None                 # 切れ目なく続いて門を通った直前の窓の平均（機上 bias_prev_）
        bias_saved = None                # この起動で SD へ書いた値（機上 bias_saved_）
        bias_saved_t = 0.0
        bias_confirmed_once = False
        cur_t = 0.0
        # ---- 重力観測（Phase 2）のためのゲート用の量 ----
        # ★ ゲートはジャイロだけから作れるものを主にする。GNSS 断中に働かないと
        #   意味が無いため（重力観測はまさにその場面のためにある）。
        yaw_rate_lp = 0.0        # ワールド系ヨーレートの持続成分 [deg/s]（2 秒 LPF）
        # 意図して直さない（2026-11-14 の監査）: --gnss-cutoff は速度の観測にだけ効く（longacc_lp・ヨーの初期化・再確認には効かない）
        longacc_lp = 0.0         # 前後加速度の持続成分 [m/s^2]（GNSS 由来・2 秒 LPF）
        last_gsp = None
        last_gsp_t = None
        level_last_t = -1e9
        rate_high_t = -1e9       # |ω| が level_rate_dps を最後に超えた時刻（機上 rate_high_us_）
        static_t0 = None         # 静止が始まった時刻（機上 static_since_us_）
        good_gsp = None          # 信用できる最後の測位の対地速度（機上 good_gsp_）
        good_gsp_t = None
        good_sacc = 1.0                  # 機上 good_sacc_（ESKF_SACC_MAX）
        ground_flags = np.zeros(n_out, dtype=bool)
        # ---- 動いているときの大きな傾きの置き直し（機上 bt_*。settings.h の ESKF_BIGTILT_*）----
        bt_acc_lp = None         # 比力の低域（時定数 0.5 秒）
        bt_rate_lp = 0.0         # |ω| の低域 [deg/s]（時定数 0.25 秒）
        bt_since = None          # 条件がそろい始めた時刻
        yaw_suspect = False      # 置き直したあと、保った方位を航跡でまだ確かめていない（機上 yaw_suspect_）
        yaw_check_arm = 0
        v_stale = False          # IMU の途絶明け。次の測位で速度を入れ直す（機上 v_stale_。BUG-62）
        # ---- GNSS の遅れの補償のための速度の履歴（機上 vhist_。約 1 秒ぶん）----
        vh = np.zeros((len(ts), 3))      # vh[j] = サンプル j を処理し終えたときの速度（あとの直しも足し込む）
        hist_from = i_start              # これより前の履歴は使えない（速度を入れ直した）

        def hist_shift(dv, i_now):
            # 観測で速度を直したぶんを履歴にも足す（履歴を「いまの推定で見た過去の速度」に保つ）
            lo = max(hist_from, int(np.searchsorted(ts, ts[i_now] - 1.0)))
            if lo < i_now:
                vh[lo:i_now] += dv

        def v_at(tq, i_now):
            # 時刻 tq の推定速度。履歴より古ければいちばん古いもの、新しければいまの速度
            lo = max(hist_from, int(np.searchsorted(ts, ts[i_now] - 1.0)))
            if gnss_delay is None or lo >= i_now or tq >= ts[i_now]:
                return f.v
            tt = np.append(ts[lo:i_now], ts[i_now])
            vv = np.vstack([vh[lo:i_now], f.v])
            return np.array([np.interp(tq, tt, vv[:, c]) for c in range(3)])

        def vel_update(gi_, i_now):
            vb = f.v.copy()
            vp = None if gnss_delay is None else v_at(g_m[gi_] - gnss_delay, i_now)
            f.update_velocity(g_enu[gi_], g_sacc[gi_], vp)
            hist_shift(f.v - vb, i_now)

        def bias_reset(chain=True):
            # chain=False は窓を締めたとき（機上 bias_window_clear）。True は動いた・加速した（機上 bias_learn_reset）
            nonlocal bias_n, bias_sum_t, bias_prev
            bias_sum[:] = 0.0
            bias_sq[:] = 0.0
            bias_sum_t = 0.0
            bias_n = 0
            if chain:
                bias_prev = None

        def undo_record(rx, ry, dbg=None, dba=None):
            # 機上 lvl_undo_record（0.1 秒ごとの箱。20 個まで）
            if not undo_boxes or cur_t - undo_boxes[-1][0] >= 0.1:
                undo_boxes.append([cur_t, 0.0, 0.0, np.zeros(3), np.zeros(3)])
                if len(undo_boxes) > 20:
                    del undo_boxes[0]
            bx = undo_boxes[-1]
            bx[1] += rx
            bx[2] += ry
            if dbg is not None:
                bx[3] += dbg
            if dba is not None:
                bx[4] += dba

        def level_rec(a_i, gain):
            # 機上 level_toward_gravity_rec: 寄せて、ワールド系の水平の回転を記録する
            q0 = f.q.copy()
            f.q = level_toward_gravity(f.q, a_i, gain)
            dq = quat_mul(f.q, np.array([q0[0], -q0[1], -q0[2], -q0[3]]))
            sg = -2.0 if dq[0] < 0.0 else 2.0
            undo_record(sg * dq[1], sg * dq[2])

        def undo_apply(t_now):
            # 機上 lvl_undo_apply: 直前 1.5 秒（ESKF_ACCEL_UNDO_US）の寄せを逆に回して戻す。戻した角度 [度]
            r = np.zeros(2); dbg = np.zeros(3); dba = np.zeros(3)
            for bx in undo_boxes:
                if t_now - bx[0] >= 1.5:
                    continue
                r += bx[1:3]; dbg += bx[3]; dba += bx[4]
            undo_boxes.clear()
            ang = float(np.hypot(r[0], r[1]))
            if ang <= 0.0:
                return 0.0
            f.q = quat_norm(quat_mul(quat_from_rotvec(np.array([-r[0], -r[1], 0.0])), f.q))
            f.bg -= dbg
            f.ba -= dba
            f.bg = np.clip(f.bg, -f.max_bg, f.max_bg)
            if hasattr(f, "max_ba"):
                f.ba = np.clip(f.ba, -f.max_ba, f.max_ba)
            return float(np.degrees(ang))

        def learn_bias(wi, dt_i):
            # 機上 learn_gyro_bias() と同じ（時定数 5.0 秒 / ESKF_BIAS_LEARN_MAX_RADS）
            # サンプリング間隔 dt から時間を積算し、250Hz 機上と 62.5Hz ログの両方で一致させる
            # ★ 5 秒の窓の各軸の標準偏差が ESKF_BIAS_LEARN_STD_RADS（0.03 deg/s）以上なら覚えない（2026-11-18。揺れを覚えないため）
            nonlocal bias_n, bias_sum_t, learned_on_ground, bias_ok_t, bias_prev, bias_ref_v, bias_saved, bias_saved_t
            nonlocal bias_confirmed_once, sd_bias
            if dt_i <= 0.0:
                return
            if bias_n == 0:
                bias_w0[:] = wi
            d = wi - bias_w0
            bias_sum[:] += d * dt_i
            bias_sq[:] += d * d * dt_i
            bias_sum_t += dt_i
            bias_n += 1
            if bias_sum_t >= 5.0:
                md = bias_sum / bias_sum_t
                var = bias_sq / bias_sum_t - md * md
                if not np.all(var < 0.000524 ** 2):             # ESKF_BIAS_LEARN_STD_RADS
                    bias_reset()                                # 揺れていた。続けて通った窓の比べも切る
                    return
                mean = bias_w0 + md
                # 確かめる: 前回までの値か、続けて通った直前の窓と 0.02 deg/s 以内か（機上 ESKF_BIAS_CONFIRM_DIFF_RADS）
                d_ref = np.max(np.abs(mean - bias_ref_v)) if bias_ref_v is not None else 1e9
                d_prev = np.max(np.abs(mean - bias_prev)) if bias_prev is not None else 1e9
                how = 0 if d_ref < 0.000349 else (1 if d_prev < 0.000349 else (2 if bias_ref_v is not None else -1))
                confirmed = how in (0, 1)
                bias_prev = mean.copy()
                # 方位が決まる前は確かめる前でも入れる。決まったあとは確かめた値だけ（機上と同じ）
                if (not yaw_set) or confirmed:
                    f.bg = np.clip(mean, -0.0087, 0.0087)
                if confirmed:
                    if yaw_set:
                        learned_on_ground = True
                    bias_ok_t = cur_t
                    bias_ref_v = mean.copy()
                    # SD へ（機上 ESKF_BIAS_SAVE_DIFF_RADS 0.005 deg/s / ESKF_BIAS_SAVE_MIN_US 5 分）
                    if bias_saved is None or (np.max(np.abs(mean - bias_saved)) >= 0.0000873 and cur_t - bias_saved_t >= 300.0):
                        bias_saved = mean.copy()
                        bias_saved_t = cur_t
                        sd_bias = mean.copy()
                if events is not None and ((confirmed and not bias_confirmed_once) or how == 2):
                    events.append((sess, float(cur_t), "gyro_bias", dict(how=how, b_dps=np.degrees(mean),
                                                                          diff_dps=float(np.degrees(d_prev if how == 1 else d_ref)))))
                if confirmed:
                    bias_confirmed_once = True
                bias_reset(chain=False)

        for k, i in enumerate(range(i_start, len(ts))):
            dt = ts[i] - ts[i - 1] if i else 0.0
            cur_t = ts[i]
            relevel_gain = min(1.0, dt / 1.0) if dt > 0.0 else 0.004   # 時定数 1.0 秒（機上 ESKF_RELEVEL_GAIN）
            if static_flags is not None:
                if static_flags[i]:
                    if static_t0 is None:
                        static_t0 = ts[i]
                else:
                    static_t0 = None
            # ---- 加速中か（機上 attitude_on_gyro の「加速中か」と同じ。2026-11-18〜、同日夜に作り直し。settings.h の ESKF_ACCEL_*）----
            # 推定の座標では見ない。加速度計の値（0.1 秒で均す）と、直近の平均（時定数 5 秒）をジャイロで本体と一緒に回したものの、
            # 平均の向きに直角な差 > 0.3 m/s²、または信用できる GNSS の対地速度 ≥ 0.5 m/s なら「加速中」。8 秒を超えたら再開。
            # 「加速中」になった瞬間に直前 1.5 秒の寄せを取り消す。
            dta = dt if 0.0 < dt < 0.1 else 0.0
            a_i = np.asarray(acc[i], dtype=float)
            if acc_lp is None:
                acc_lp = a_i.copy()
                acc_f = a_i.copy()
            w_c = w[i] - f.bg
            acc_lp = acc_lp - np.cross(w_c, acc_lp) * dta              # ワールドに固定した向きはセンサー軸で −ω × v
            acc_f = acc_f + (a_i - acc_f) * (dta / 0.1 if dta < 0.1 else 1.0)    # ESKF_ACCEL_LP_S
            if float(w_c @ w_c) < 0.026 ** 2:                            # 速く回っている間は寄せない（ESKF_STATIC_GYRO_RADS）
                acc_lp = acc_lp + (acc_f - acc_lp) * (dta / 5.0)       # ESKF_ACCEL_CHANGE_TAU_S
            ln = float(np.linalg.norm(acc_lp))
            dp2 = 0.0
            acc_dp_raw2 = 0.0
            if ln > 1.0:
                u = acc_lp / ln
                dd = acc_f - acc_lp
                e = dd - (dd @ u) * u
                dp2 = float(e @ e)
                rr = a_i - acc_lp
                er = rr - (rr @ u) * u
                acc_dp_raw2 = float(er @ er)
            gfresh = good_gsp_t is not None and (ts[i] - good_gsp_t) < 3.0
            gnss_moving = gfresh and good_gsp >= max(0.2, 2.0 * good_sacc)   # 機上 gnss_moving_mps()（ESKF_ACCEL_GNSS_MOVING_MPS / _SACC_K）
            if gfresh and good_gsp < 0.2:                                # ESKF_ACCEL_GNSS_STOP_MPS
                if not gstop_on:
                    gstop_on, gstop_since = True, ts[i]
            else:
                gstop_on = False
            if ah_on and gstop_on and ts[i] - gstop_since >= 2.0 and ts[i] - ah_since >= 2.0:   # ESKF_ACCEL_GNSS_STOP_US
                acc_lp = acc_f.copy()
                dp2 = 0.0
            by_acc = dp2 > 0.3 ** 2                                      # ESKF_ACCEL_CHANGE_MPS2
            if by_acc or gnss_moving:
                if not ah_on:
                    ah_on, ah_since = True, ts[i]
                    undo_why = 0 if by_acc else 1
            else:
                ah_on = False
            was_acc = accelerating
            accelerating = ah_on and (ts[i] - ah_since) < 8.0          # ESKF_ACCEL_HOLD_MAX_US
            if ah_on and (not accelerating) and was_acc:
                acc_lp = acc_f.copy()                                    # 上限が来たら平均を取り直す
                acc_dp_raw2 = 0.0
            if accelerating and not was_acc:
                deg = undo_apply(ts[i])
                if deg > 0.0 and events is not None:
                    events.append((sess, float(ts[i]), "accel_undo", dict(deg=deg, why=undo_why)))
            if accelerating:
                bias_reset()                                           # 加速中の分をゼロ点の窓に混ぜない
            # ---- 地上の判定（機上 on_ground_still / gnss_says_ground と同じ）----
            good_fresh = good_gsp_t is not None and (ts[i] - good_gsp_t) < 3.0  # ESKF_GNSS_VEL_TIMEOUT_US
            ground = (use_ground and yaw_set and static_t0 is not None
                      and ts[i] - static_t0 >= 2.0                     # ESKF_GROUND_STATIC_US
                      and not (good_fresh and good_gsp >= 1.0)         # ESKF_GROUND_MAX_GS_MPS
                      and not accelerating)
            # ---- 待機中の維持（機上 attitude_on_gyro の該当ブロックと同じ）----
            if (not yaw_set) and static_flags is not None and static_flags[i] and not accelerating:
                level_rec(acc[i], relevel_gain)                           # 時定数 1.0 秒（記録つき。機上 level_toward_gravity_rec）
                learn_bias(w[i], dt)
                f.reset_covariance(3.0, 180.0)
            elif ground:
                # ---- 地上での維持（ヨーが決まった後。機上の同名ブロックと同じ）----
                # ヨーの分散は残し、ロール・ピッチの分だけ入れ直す（機上 reset_tilt_covariance）。
                level_rec(acc[i], relevel_gain)
                if good_fresh and good_gsp < 1.0:
                    learn_bias(w[i], dt)
                f.P[0:2, :] = 0.0
                f.P[:, 0:2] = 0.0
                f.P[0, 0] = f.P[1, 1] = np.deg2rad(3.0) ** 2      # ESKF_INIT_ATT_SIGMA_DEG
            elif static_flags is not None and not static_flags[i]:
                bias_reset()
            if ground_prev and not ground and learned_on_ground:
                # 地上の維持を抜けた（動き出した）。止まっている間に覚えたゼロ点の不確かさを広げる
                # （機上 bias_widen_after_ground。ESKF_BIAS_LEARN_PBG_RADS = 0.02 deg/s。2026-11-18〜）
                s2 = 0.000349 ** 2
                for kk in range(6, 9):
                    old = f.P[kk, kk]
                    f.P[kk, :] = 0.0
                    f.P[:, kk] = 0.0
                    f.P[kk, kk] = max(old, s2)
                learned_on_ground = False
            ground_prev = ground
            ground_flags[k] = ground
            # ★ ヨーレートを作る回転は**伝播する前**の姿勢（機上 attitude_on_gyro() は伝播の前に作った R を使う）。
            #   2026-11-10 までは predict の後の f.q で作っていて、1 サンプルぶん機上と違っていた。
            # ---- IMU の途絶明け（機上 attitude_on_gyro → imu_gap_recover。2026-11-13〜。BUG-62）----
            # 大きな傾きの置き直しと同じ扱い: 傾きと方位の不確かさを広げ、方位は保って航跡で確かめ直し、速度は次の測位で入れ直す。
            # 姿勢そのものは動かさない。機上はこのサンプルで return するが、ここは下へ続ける（predict は dt > 0.5 で何もしない。前からの差）
            if dt > 0.5:                                                   # ESKF_IMU_TIMEOUT_US
                f.P[0:2, :] = 0.0
                f.P[:, 0:2] = 0.0
                f.P[0, 0] = f.P[1, 1] = np.deg2rad(30.0) ** 2              # ESKF_INIT_ATT_SIGMA_MOVING_DEG
                if yaw_set:
                    old = f.P[2, 2]
                    f.P[2, :] = 0.0
                    f.P[:, 2] = 0.0
                    f.P[2, 2] = max(old, np.deg2rad(15.0) ** 2)            # ESKF_BIGTILT_YAW_SIGMA_DEG
                    yaw_suspect = True
                    yaw_check_arm = 0
                v_stale = True
                bt_since = None
                bt_acc_lp = None
                if prov and not yaw_set:
                    prov_c = prov_s = prov_n = prov_k = 0.0; prov_cur[:] = np.nan
                if events is not None:
                    events.append((sess, float(ts[i]), "imu_gap", float(dt)))
            R_ = quat_to_R(f.q)
            f.predict(w[i], acc[i], dt)
            # ---- 発進加速の常時前向き 2D 相関用の IMU 水平速度変化（機上 attitude_on_gyro の同名ブロックと同じ）----
            if (not yaw_set) and 0.0 < dt <= 0.5:
                if static_flags is not None and static_flags[i] and not accelerating:
                    yaw_static_s += dt
                    if yaw_static_s >= 1.0:                                        # ESKF_YAW_CORR_STATIC_RESET_US
                        yaw_imu_hp[:] = 0.0
                        yaw_gnss_lp = None                                         # 次の測位で取り直す（機上 yaw_gnss_lp_valid_ = false）
                        yaw_corr_c = 0.0
                        yaw_corr_s = 0.0
                        yaw_corr_n = 0.0
                else:
                    yaw_static_s = 0.0
                    rf_en = (R_ @ (np.asarray(acc[i], float) - f.ba))[:2]
                    yaw_imu_hp = yaw_imu_hp * np.exp(-dt / 3.0) + rf_en * dt       # ESKF_YAW_CORR_TAU_S
                if prov and yaw_static_s >= pk["static_s"]:
                    prov_seg_until = ts[i] + pk["seg_s"]                           # 止まっている間は窓を開けておく
            # ---- ワールド系ヨーレートの持続成分（機上 yaw_rate_lp_ と同じ式）----
            wz_dps = 0.0
            if dt > 0.0:
                wz = float(R_[2] @ (np.asarray(w[i], float) - f.bg))
                wz_dps = np.degrees(wz)
                yaw_rate_lp += (wz_dps - yaw_rate_lp) * (dt / (2.0 + dt))
                # 全角速度（機上 rate_high_us_ と同じ。ESKF_LEVEL_MAX_RATE_DPS）
                if np.degrees(np.linalg.norm(np.asarray(w[i], float) - f.bg)) > level_rate_dps:
                    rate_high_t = ts[i]
            # ---- 重力観測（ゲートを通ったときだけ）----
            # ★ ヨーレートの更新は**この直前**で済ませてある。後ろに置くと
            #   1 サンプル前の値で判定して旋回の入り口で漏れる（機上で踏んだ）。
            if use_level and (ts[i] - level_last_t) >= 0.5:      # 2Hz
                amag = float(np.linalg.norm(acc[i]))
                # ★ 瞬時値も見る。LPF だけだと 2 秒の遅れのぶん、旋回の入り口で
                #   観測が漏れてバンクを過小評価する（合成旋回で確認）。
                ok = (abs(yaw_rate_lp) <= level_yawrate_dps and
                      abs(wz_dps) <= level_yawrate_inst_dps and
                      ts[i] - rate_high_t >= level_rate_quiet and
                      abs(amag - 9.80665) <= level_accmag_tol)
                # ★ 機上と同じ: 測位が古い（断中）ならこの条件を省く。
                #   前後加速度は GNSS からしか作れないので、断中に古い値で
                #   判定するとゲートが誤って開く／閉じる。
                if ok and last_gsp is not None and (ts[i] - last_gsp_t) < 3.0:
                    ok = abs(longacc_lp) <= level_longacc
                # 方位が決まる前の加速中は入れない（機上と同じ。2026-11-18〜）
                if ok and (not yaw_set) and accelerating:
                    ok = False
                # 方位が決まる前は均していない今の値でも加速していないこと（機上 acc_dp_raw2_。2026-11-18 夜〜）
                if ok and (not yaw_set) and acc_dp_raw2 > 0.3 ** 2:
                    ok = False
                if ok:
                    vb = f.v.copy()
                    q0 = f.q.copy(); bg0 = f.bg.copy(); ba0 = f.ba.copy()
                    f.update_level(acc[i], level_sigma)
                    hist_shift(f.v - vb, i)
                    if not yaw_set:
                        # 方位が決まる前の重力観測は記録する（機上 level_update の lvl_undo_record）
                        dq = quat_mul(f.q, np.array([q0[0], -q0[1], -q0[2], -q0[3]]))
                        sg = -2.0 if dq[0] < 0.0 else 2.0
                        undo_record(sg * dq[1], sg * dq[2], f.bg - bg0, f.ba - ba0)
                    level_last_t = ts[i]
            # ---- 動いているときの、大きくずれた傾きの置き直し（機上 attitude_on_gyro の同名ブロックと同じ）----
            # ★ 低域は毎サンプル更新する。地上での維持・待機中の維持が働いている間は見ない。
            if use_bigtilt and dt > 0.0:
                if bt_acc_lp is None:
                    bt_acc_lp = np.asarray(acc[i], float).copy()
                else:
                    bt_acc_lp += (np.asarray(acc[i], float) - bt_acc_lp) * (dt / (0.5 + dt))   # ESKF_BIGTILT_ACC_TAU_S
                wn_dps = np.degrees(np.linalg.norm(np.asarray(w[i], float) - f.bg))
                bt_rate_lp += (wn_dps - bt_rate_lp) * (dt / (0.25 + dt))                      # ESKF_BIGTILT_RATE_TAU_S
                maintained = ground or ((not yaw_set) and static_flags is not None and static_flags[i])
                bad = False
                err_deg = 0.0
                if (not maintained) and bt_rate_lp < 10.0:                                    # ESKF_BIGTILT_MAX_RATE_DPS
                    fb = bt_acc_lp - f.ba
                    fn = float(np.linalg.norm(fb))
                    if fn > 1.0 and abs(fn - 9.80665) <= level_accmag_tol:                     # ESKF_LEVEL_ACCMAG_TOL_MPS2
                        c = float(quat_to_R(f.q)[2] @ fb) / fn
                        err_deg = float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))
                        bad = err_deg >= 60.0                                                  # ESKF_BIGTILT_MIN_DEG
                if not bad:
                    bt_since = None
                elif bt_since is None:
                    bt_since = ts[i]
                elif ts[i] - bt_since >= 2.0:                                                  # ESKF_BIGTILT_HOLD_US
                    # 機上 relevel_big_tilt(): 比力で水平化し、**方位は保つ**（最短の回転で振れた分を戻す）。
                    # 方位の不確かさを広げ、GNSS が戻ったら航跡と比べる（yaw_suspect）。理由は settings.h の ESKF_BIGTILT_*。
                    from decode_imulog import mount_correct_quat
                    _, _, h0 = mount_correct_quat(*f.q)
                    f.q = level_toward_gravity(f.q, bt_acc_lp, 1.0)
                    f.P[0:2, :] = 0.0
                    f.P[:, 0:2] = 0.0
                    f.P[0, 0] = f.P[1, 1] = np.deg2rad(30.0) ** 2                              # ESKF_INIT_ATT_SIGMA_MOVING_DEG
                    if yaw_set:
                        q_old = f.q.copy()
                        f.q = set_heading(f.q, float(np.atleast_1d(h0)[0]))
                        f.yaw_fix = getattr(f, "yaw_fix", 0.0) + world_z_rot(q_old, f.q)
                        old = f.P[2, 2]
                        f.P[2, :] = 0.0
                        f.P[:, 2] = 0.0
                        f.P[2, 2] = max(old, np.deg2rad(15.0) ** 2)                            # ESKF_BIGTILT_YAW_SIGMA_DEG
                        yaw_suspect = True
                        yaw_check_arm = 0
                    bt_since = None
                    if prov and not yaw_set:
                        prov_c = prov_s = prov_n = prov_k = 0.0; prov_cur[:] = np.nan
                    if events is not None:
                        events.append((sess, float(ts[i]), "relevel", err_deg))
            while gi < len(g_ts) and g_ts[gi] <= ts[i]:
                # 前後加速度の持続成分（対地速度の変化率）
                gsp = float(np.hypot(g_enu[gi][0], g_enu[gi][1]))
                if last_gsp is not None and g_ts[gi] > last_gsp_t:
                    dtg = g_ts[gi] - last_gsp_t
                    longacc_lp += ((gsp - last_gsp) / dtg - longacc_lp) * (dtg / (2.0 + dtg))
                last_gsp, last_gsp_t = gsp, g_ts[gi]
                # 地上の判定用（機上 good_gsp_）。g_ts は sAcc <= sacc_max で振り落とし済みなので、
                # ここに来る測位はすべて「信用できる」測位になる。
                # ★ 機上の last_gsp_（前後加速度）は sAcc を問わず全測位で更新するが、ここは
                #   振り落とした後の測位で作っている。前後加速度のゲートだけは機上と入力が違う
                #   （生ログに 3D フィックスかどうかが残っていないため。0.983 からの既知の差）。
                good_gsp, good_gsp_t = gsp, g_ts[gi]
                good_sacc = float(g_sacc[gi])            # 機上 good_sacc_
                # ★ 機上も sAcc の悪い測位ではヨーを入れない（2026-10-27）。ここは振り落とし済み。
                if v_stale:
                    # 機上 attitude_on_gnss_velocity の v_stale_: 途絶明けは速度を GNSS の値に入れ直す（観測には使わない）
                    if cutoff is None or g_ts[gi] <= cutoff:
                        f.v = g_enu[gi].copy(); hist_from = i
                        f.P[3:6, :] = 0.0
                        f.P[:, 3:6] = 0.0
                        rv = max(float(g_sacc[gi]), 0.05)                  # ESKF_SACC_MIN
                        f.P[3, 3] = f.P[4, 4] = f.P[5, 5] = rv * rv
                        v_stale = False
                elif not yaw_set:
                    sp = float(np.hypot(g_enu[gi][0], g_enu[gi][1]))
                    # ---- 常時前向き 2D 相関の更新（機上 attitude_on_gnss_velocity の同名ブロックと同じ）----
                    if yaw_gnss_lp is None:
                        yaw_gnss_lp = g_enu[gi][:2].copy()
                        yaw_gnss_last_t = g_ts[gi]
                    else:
                        dtg_y = g_ts[gi] - yaw_gnss_last_t
                        yaw_gnss_last_t = g_ts[gi]
                        if 0.0 < dtg_y < 2.0:
                            yaw_gnss_lp += (1.0 - np.exp(-dtg_y / 3.0)) * (g_enu[gi][:2] - yaw_gnss_lp)   # ESKF_YAW_CORR_TAU_S
                            dv_g = g_enu[gi][:2] - yaw_gnss_lp
                            if prov:
                                # 仮のヨーの相関器（§4.3）: 減衰はいつも、積むのは窓の中で IMU 側が凍っていないときだけ（§4.8 の 1 つめ）
                                dec2 = np.exp(-dtg_y / pk["tau_s"])
                                prov_c *= dec2; prov_s *= dec2; prov_n *= dec2; prov_k *= dec2
                                frozen = static_flags[i] and not accelerating
                                if ts[i] < prov_seg_until and not (pk["skip_frozen"] and frozen):
                                    rv2 = max(float(g_sacc[gi]), 0.05)
                                    w2 = 1.0 / (rv2 * rv2)
                                    hx, hy = yaw_imu_hp
                                    prov_c += w2 * (hx * dv_g[0] + hy * dv_g[1])
                                    prov_s += w2 * (hx * dv_g[1] - hy * dv_g[0])
                                    prov_n += w2 * float(np.hypot(hx, hy) * np.hypot(dv_g[0], dv_g[1]))
                                    prov_k += w2 * float(hx * hx + hy * hy)
                                    prov_last_add = ts[i]
                                if pk.get("debug") is not None:
                                    pk["debug"].append((sess, float(ts[i]), yaw_imu_hp.copy(), dv_g.copy(),
                                                        bool(ts[i] < prov_seg_until), bool(frozen), float(g_sacc[gi]),
                                                        float(np.degrees(getattr(f, "yaw_fix", 0.0)))))
                                mag2 = float(np.hypot(prov_c, prov_s))
                                if mag2 > 1e-9:
                                    from decode_imulog import mount_correct_quat
                                    dps2 = float(np.arctan2(prov_s, prov_c))
                                    qz2 = np.array([np.cos(dps2 * 0.5), 0.0, 0.0, np.sin(dps2 * 0.5)])
                                    _, _, yv = mount_correct_quat(*quat_norm(quat_mul(qz2, f.q)))
                                    coh2 = mag2 / prov_n if prov_n > 1e-9 else 0.0
                                    k2 = mag2 / prov_k if prov_k > 1e-9 else 0.0
                                    sc = 2.0 * (180.0 / np.pi) / np.sqrt(mag2)                 # ESKF_YAW_CORR_SIGMA_SCALE
                                    bok = bias_ok_t is not None and (ts[i] - bias_ok_t) < 120.0
                                    sbg = 0.02 if bok else 0.3                                  # [deg/s]
                                    age = ts[i] - prov_last_add if prov_last_add is not None else 0.0
                                    sg2 = float(np.sqrt(3.5 ** 2 + sc ** 2 + (sbg * age) ** 2))
                                    ok = (mag2 >= pk["min_mag"] and coh2 >= pk["min_coh"]
                                          and pk["k_lo"] <= k2 <= pk["k_hi"])
                                    prov_cur[:] = [float(np.atleast_1d(yv)[0]) % 360.0, sg2, mag2, coh2, k2, 1.0 if ok else 0.0,
                                                   float(np.degrees(dps2))]
                            if sp >= 0.8:                                                                 # ESKF_YAW_CORR_MIN_GS_MPS
                                rv_y = max(float(g_sacc[gi]), 0.05)                                       # ESKF_SACC_MIN
                                w_y = 1.0 / (rv_y * rv_y)
                                dec_cs = np.exp(-dtg_y / 4.0)                                             # ESKF_YAW_CORR_CS_TAU_S
                                yaw_corr_c = dec_cs * yaw_corr_c + w_y * (yaw_imu_hp[0] * dv_g[0] + yaw_imu_hp[1] * dv_g[1])
                                yaw_corr_s = dec_cs * yaw_corr_s + w_y * (yaw_imu_hp[0] * dv_g[1] - yaw_imu_hp[1] * dv_g[0])
                                yaw_corr_n = dec_cs * yaw_corr_n + w_y * float(np.hypot(yaw_imu_hp[0], yaw_imu_hp[1]) * np.hypot(dv_g[0], dv_g[1]))
                        else:
                            yaw_gnss_lp = g_enu[gi][:2].copy()
                    # 2 段の門（機上と同じ。2026-11-20〜）: 2 m/s 以上が 1 秒なら 2D 相関の門を全部通ったときだけ、
                    # 3 m/s 以上が 1 秒なら今までどおり（相関が外れたら航跡＋15°）
                    if sp < 2.0:                                 # ESKF_YAW_INIT_CORR_MIN_SPEED_MPS
                        yaw_arm = 0
                        yaw_arm_lo = 0
                    else:
                        yaw_arm_lo += 1
                        yaw_arm = yaw_arm + 1 if sp >= 3.0 else 0   # ESKF_YAW_INIT_MIN_SPEED_MPS
                        arm_trk = yaw_arm >= arm_fixes           # ESKF_YAW_INIT_ARM_FIXES。航跡へ戻ってよい
                        if yaw_arm_lo >= arm_fixes:
                            trk = float(np.degrees(np.arctan2(g_enu[gi][0], g_enu[gi][1])) % 360.0)
                            # 機上 attitude_on_gnss_velocity の同名ブロックと同じ（settings.h の ESKF_YAW_CORR_*。2026-11-18）
                            mag = float(np.hypot(yaw_corr_c, yaw_corr_s))
                            coh = mag / yaw_corr_n if yaw_corr_n > 1e-6 else 0.0
                            how, d_trk, sig_init = 1, 0.0, 15.0
                            if mag >= 80.0:                      # ESKF_YAW_CORR_MIN_MAG
                                how = 2
                                if coh >= 0.98:                  # ESKF_YAW_CORR_MIN_COH
                                    from decode_imulog import mount_correct_quat
                                    dpsi = float(np.arctan2(yaw_corr_s, yaw_corr_c))
                                    qz = np.array([np.cos(dpsi * 0.5), 0.0, 0.0, np.sin(dpsi * 0.5)])
                                    q_try = quat_norm(quat_mul(qz, f.q))
                                    _, _, y_chk = mount_correct_quat(*q_try)
                                    d_trk = (float(np.atleast_1d(y_chk)[0]) - trk + 180.0) % 360.0 - 180.0
                                    if abs(d_trk) <= 20.0:       # ESKF_YAW_CORR_MAX_TRK_DIFF_DEG
                                        how = 0
                                        f.yaw_fix = getattr(f, "yaw_fix", 0.0) + world_z_rot(f.q, q_try)
                                        f.q = q_try
                                        sig_corr = 2.0 * (180.0 / np.pi) / np.sqrt(mag)   # ESKF_YAW_CORR_SIGMA_SCALE
                                        sig_init = float(min(np.hypot(3.5, sig_corr), 15.0))  # ESKF_YAW_INIT_SIGMA_MIN_DEG..DEG
                                        f.P[2, :] = 0.0
                                        f.P[:, 2] = 0.0
                                        f.P[2, 2] = np.deg2rad(sig_init) ** 2
                                    else:
                                        how = 3
                            # 3 m/s に届く前は、相関で入れられなければ何もせずに待つ（機上と同じ。f.q は how=3 でも回していない）
                            if how == 0 or arm_trk:
                                if how != 0:
                                    q_old = f.q.copy()
                                    f.q = set_heading(f.q, trk)
                                    f.yaw_fix = getattr(f, "yaw_fix", 0.0) + world_z_rot(q_old, f.q)
                                    f.P[2, :] = 0.0
                                    f.P[:, 2] = 0.0
                                    f.P[2, 2] = np.deg2rad(15.0) ** 2    # ESKF_YAW_INIT_SIGMA_DEG
                                yaw_set = True
                                # 直前の静止でゼロ点を覚えていたら P_bg を 0.02 deg/s に（機上 ESKF_YAW_INIT_BIAS_FRESH_US / ESKF_BIAS_LEARN_PBG_RADS）
                                bias_ok = bias_ok_t is not None and (ts[i] - bias_ok_t) < 120.0
                                # 覚えていなければデータシートの最大 0.3 deg/s（ESKF_BIAS_UNLEARNED_RADS。いまより小さいときだけ）
                                sb2 = (0.000349 if bias_ok else 0.005236) ** 2
                                for kk in range(6, 9):
                                    if (not bias_ok) and f.P[kk, kk] <= sb2:
                                        continue
                                    f.P[kk, :] = 0.0
                                    f.P[:, kk] = 0.0
                                    f.P[kk, kk] = sb2
                                if events is not None:
                                    events.append((sess, float(ts[i]), "yaw_init", dict(how=how, mag=mag, coh=coh, corr_trk=d_trk,
                                                                                         sigma=sig_init, bias_ok=bias_ok)))
                                    if prov:
                                        # 確定の瞬間の仮のヨー（入れる前の座標で出した値なので、入れた方位と比べるには dpsi と yaw_fix を使う）
                                        events.append((sess, float(ts[i]), "prov_at_init", dict(
                                            yaw=float(prov_cur[0]), sigma=float(prov_cur[1]), mag=float(prov_cur[2]),
                                            coh=float(prov_cur[3]), k=float(prov_cur[4]), valid=bool(prov_cur[5] == 1.0),
                                            dpsi=float(prov_cur[6]))))
                                if prov:
                                    prov_cur[:] = np.nan          # 確定したら使わない
                                f.v = g_enu[gi].copy(); hist_from = i
                elif yaw_suspect:
                    # 機上 attitude_on_gnss_velocity の同名ブロック: 保った方位を航跡で確かめてから観測に入れる
                    sp = float(np.hypot(g_enu[gi][0], g_enu[gi][1]))
                    if sp < 3.0:                                 # ESKF_YAW_INIT_MIN_SPEED_MPS
                        yaw_check_arm = 0
                        # 機上と同じ: 止まっているとき（ESKF_GROUND_MAX_GS_MPS 未満）は確かめを待ったまま速度の観測は入れる
                        # （止まっていれば方位のずれは速度に効かない。2026-11-13〜。BUG-64）
                        if sp < 1.0 and (cutoff is None or g_ts[gi] <= cutoff):
                            vel_update(gi, i)
                    else:
                        yaw_check_arm += 1
                        if yaw_check_arm >= arm_fixes:           # ESKF_YAW_INIT_ARM_FIXES
                            from decode_imulog import mount_correct_quat
                            trk = np.degrees(np.arctan2(g_enu[gi][0], g_enu[gi][1])) % 360.0
                            _, _, hh = mount_correct_quat(*f.q)
                            d = (trk - float(np.atleast_1d(hh)[0]) + 180.0) % 360.0 - 180.0
                            if abs(d) > 45.0:                    # ESKF_BIGTILT_YAW_CHECK_DEG
                                q_old = f.q.copy()
                                f.q = set_heading(f.q, trk)
                                f.yaw_fix = getattr(f, "yaw_fix", 0.0) + world_z_rot(q_old, f.q)
                                f.P[2, :] = 0.0
                                f.P[:, 2] = 0.0
                                f.P[2, 2] = np.deg2rad(15.0) ** 2    # ESKF_YAW_INIT_SIGMA_DEG
                                f.v = g_enu[gi].copy(); hist_from = i
                            else:
                                # 保った方位をそのまま使い、この測位から観測に入れる（v を置き換えない。機上と同じ）
                                vel_update(gi, i)
                            yaw_suspect = False
                            if events is not None:
                                events.append((sess, float(ts[i]), "yaw_reset" if abs(d) > 45.0 else "yaw_kept", float(d)))
                elif cutoff is None or g_ts[gi] <= cutoff:
                    vel_update(gi, i)
                gi += 1
            vh[i] = f.v
            qs[k] = f.q
            if prov:
                yaw_fix[k] = np.degrees(getattr(f, "yaw_fix", 0.0))
                prov_out[k] = prov_cur
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
            # 生のクォータニオン（センサー→ENU）。回転行列の第 3 行が
            # 「上方向」で、バリオの重力除去の検証に使う。
            "qw": qs[:, 0], "qx": qs[:, 1], "qy": qs[:, 2], "qz": qs[:, 3],
            # ヨーが決まった後の「地上での維持」が働いていたサンプル（機上 gnd= の元）
            "ground": ground_flags,
            **({} if not prov else {
                "prov_yaw": prov_out[:, 0], "prov_sigma": prov_out[:, 1], "prov_mag": prov_out[:, 2],
                "prov_coh": prov_out[:, 3], "prov_k": prov_out[:, 4], "prov_valid": prov_out[:, 5],
                "prov_dpsi": prov_out[:, 6], "yaw_fix": yaw_fix}),
        }))

    if not out:
        raise SystemExit("処理できるセッションが無かった")
    return pd.concat(out, ignore_index=True)


def main():
    import argparse, os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_imulog import load, split

    ap = argparse.ArgumentParser(description="生 IMU ログに姿勢 ESKF を適用する")
    ap.add_argument("path")
    ap.add_argument("--csv", metavar="FILE", help="結果を CSV で書き出す")
    args = ap.parse_args()

    parts = split(load(args.path))
    est = run_on_log(parts)

    print(f"サンプル数 : {len(est)}   セッション数 : {est['session'].nunique()}")
    print(f"{'sess':>5}{'件数':>9}{'長さ[s]':>10}{'|roll|最大':>12}{'yawσ 最終':>11}")
    for sess, e in est.groupby("session"):
        dur = e["ts"].max() - e["ts"].min()
        print(f"{sess:>5}{len(e):>9}{dur:>10.1f}{e['roll'].abs().max():>12.1f}"
              f"{e['yaw_sigma'].iloc[-1]:>11.1f}")

    if args.csv:
        est.to_csv(args.csv, index=False)
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
