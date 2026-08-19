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

    def init(self, q0, v0=None):
        self.q = quat_norm(np.asarray(q0, dtype=float))
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
               gnss_cutoff=None):
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
    from decode_imulog import euler_from_quat, mount_correct

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
        # 初期姿勢は BNO085 の GAME_RV から貰う（静止時は BNO085 が正しい）
        if grv_all is not None and not grv_all.empty:
            gv = grv_all[grv_all["session"] == sess]
            if not gv.empty:
                f.init(q0=[gv.qw.iloc[0], gv.qx.iloc[0], gv.qy.iloc[0], gv.qz.iloc[0]])
        if len(g_ts):
            f.v = g_enu[0].copy()
        elif not use_gnss and gnss_all is not None and not gnss_all.empty:
            # 比較を公平にするため、初期速度だけは GNSS から貰う。
            # ここまで揃えないと「初期速度が違うせい」で差が出てしまう。
            gn0 = gnss_all[(gnss_all["session"] == sess) &
                           (gnss_all["sAcc"] <= sacc_max)].sort_values("t")
            if not gn0.empty:
                f.v = np.array([gn0["velE"].iloc[0], gn0["velN"].iloc[0],
                                -gn0["velD"].iloc[0]], dtype=float)

        ts = imu["ts"].to_numpy(float)
        w = imu[["gx", "gy", "gz"]].to_numpy(float)
        acc = imu[["ax", "ay", "az"]].to_numpy(float)

        qs = np.zeros((len(ts), 4))
        ysig = np.zeros(len(ts))
        gi = 0
        for i in range(len(ts)):
            dt = ts[i] - ts[i - 1] if i else 0.0
            f.predict(w[i], acc[i], dt)
            while gi < len(g_ts) and g_ts[gi] <= ts[i]:
                if cutoff is None or g_ts[gi] <= cutoff:
                    f.update_velocity(g_enu[gi], g_sacc[gi])
                gi += 1
            qs[i] = f.q
            ysig[i] = np.degrees(np.sqrt(max(f.P[2, 2], 0.0)))

        s_roll, s_pitch, s_yaw = euler_from_quat(qs[:, 0], qs[:, 1], qs[:, 2], qs[:, 3])
        roll, pitch, yaw = mount_correct(s_roll, s_pitch, s_yaw)
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
