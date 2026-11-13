#!/usr/bin/env python3
"""
バリオ用 Kalman フィルター（機上の imu.cpp）の PC 版。生 IMU ログから V/S を再現する。

★ **imu.cpp と同じ数式の二重実装。片方を直したらもう片方も必ず合わせる**（CLAUDE.md の約束。
  attitude.cpp と eskf.py の関係と同じ）。2026-11-11 に作った。

■ 何に使うか
  ・V/S がおかしかった場面を、機上と同じ計算で再現して原因を切り分ける
    （2026-11-11 16:09 のトンネル: 姿勢が壊れて鉛直加速度が −7.5 m/s² になり、V/S が −5 → +3 m/s と振れた）
  ・KF_Q_VEL / KF_Q_BIAS / KF_R やガード（KF_ACC_GUARD_*）を変えたら V/S がどう変わるかを、実機に焼く前に見る
  ・機上の CSV の KF_Vspeed と比べて、この再現が機上と合っているかを確かめる（--flight-csv）。
    2026-11-12 より前のファームで取った CSV（機上で GNSS の velD も入れていた）と比べるときは --gnss-vel を付ける

■ 機上と同じにしてあるもの（imu.cpp / settings.h の名前）
  ・状態 x = [高度, V/S, 加速度バイアス]、初期 P = diag(100, 1, 0.1)。最初の気圧で x = [z, 0, 0]
  ・予測は加速度が届いた回に、前の予測から IMU_KF_PREDICT_INTERVAL_US（30ms）以上たっていたら、
    その間の比力の平均で 1 回（imu_update）。dt は前の予測からの実時間で、0 以下か 0.5 秒超なら捨てる
  ・Q = diag(0, KF_Q_VEL + KF_HORIZ_ACCEL_GAIN·水平加速度², KF_Q_BIAS)（1 ステップあたり。dt で割らない）
  ・鉛直加速度は「上向き（ESKF の回転行列の第 3 行）· 比力 − g」。水平化の前は比力の低域の向き（attitude.cpp の up_lp_）
  ・気圧高度の観測（R = KF_R）。生ログの気圧は機上の KF に入れたのと同じ値・同じ回（約 3.7Hz）
  ・GNSS の上向き速度の観測は**入れない**（2026-11-12〜 機上の KF_GNSS_VEL_ENABLED = 0）。
    --gnss-vel で以前の動き（sAcc < GNSS_VSI_SACC_MAX_MPS、R = max(sAcc² × GNSS_VSI_R_SCALE, 0.001)）を再現できる
  ・鉛直加速度のガード（KF_ACC_GUARD_*。acc_guard_step / kf_predict_baro_only）

■ 合わせていないもの（既知の差）
  ・2026-11-13 より前の生ログは加速度が 62.5Hz（SCH16T_IMULOG_DECIM = 4 で間引き。機上は 250Hz）で、30ms の平均に入る数が 1/4。
    それ以降のログ（間引き 1）は機上と同じ 250Hz
  ・上向きは eskf.py の再現（50Hz）から取る。機上の ESKF と少し違う（eskf.py の頭のコメント）
  ・GNSS の観測は機上では 3D の fixOK のときだけだが、生ログにフィックスの種類が無いので sAcc だけで決める
    （トンネルでは受信機が速度 0・sAcc 数 m/s〜を出すので、sAcc で落ちる）
  ・GNSS 高度で気圧の基準を直す処理（imu_kalman_gnss_update）は再現しない。生ログの気圧高度は機上で直したあとの値
    （get_airdata_altitude()）なので、効果はすでに入っている
  ・SD の設定で変えた KF の値は読まない（--q-vel / --q-bias / --r で渡す）
  ・IMU が途絶えたときの恒速モデル（IMU_DRV_DEAD）とリプレイ・ミラー中の差し替え

使い方:
    python3 vario.py imuraw/20261111.bin                      # セッションごとの概要とガードの出入り
    python3 vario.py imuraw/20261111.bin --flight-csv data/   # 機上の CSV の KF_Vspeed と比べる（相関と差）
    python3 vario.py imuraw/20261111.bin --csv vario.csv      # 30ms ごとの結果を書き出す
    python3 vario.py imuraw/20261111.bin --no-guard           # ガードを外して比べる
    python3 vario.py imuraw/20261111.bin --baro-only          # 加速度を使わない（気圧だけ）
    python3 vario.py imuraw/20261111.bin --gnss-vel           # GNSS の上向き速度も入れる（2026-11-12 より前の機上）
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

GRAVITY = 9.80665

# ---- settings.h の値（機上で変えたらここも直す）----
IMU_KF_PREDICT_INTERVAL_S = 0.030     # IMU_KF_PREDICT_INTERVAL_US
KF_Q_VEL = 0.02
KF_Q_BIAS = 0.0005
KF_R = 12.0
KF_HORIZ_ACCEL_GAIN = 0.01
KF_GNSS_VEL_ENABLED = False           # settings.h の KF_GNSS_VEL_ENABLED（0 = GNSS の velD を V/S に入れない）
GNSS_VSI_SACC_MAX_MPS = 1.0
GNSS_VSI_R_SCALE = 64.0
KF_ACC_GUARD_FAST_MPS2 = 4.0
KF_ACC_GUARD_FAST_STEPS = 4
KF_ACC_GUARD_SLOW_MPS2 = 2.0
KF_ACC_GUARD_TAU_S = 1.0
KF_ACC_GUARD_CLEAR_MPS2 = 0.5
KF_ACC_GUARD_CLEAR_S = 2.0            # KF_ACC_GUARD_CLEAR_US
# attitude.cpp の up_lp_（水平化の前の上向き）。機上は 250Hz で 1 サンプルあたり ESKF_UP_LP_GAIN = 0.005
#   → 時定数 1 / (0.005 × 250) = 0.8 秒。生ログは間引いてあるので時間で同じにする。
UP_LP_TAU_S = 1.0 / (0.005 * 250.0)
UP_LP_ACC_TOL = 2.0                   # ESKF_UP_LP_ACC_TOL


def _quat_up(qw, qx, qy, qz):
    """クォータニオン（センサー→ENU）の回転行列の第 3 行 = センサー軸で見た「上」（attitude_get_up_sensor）。"""
    return np.column_stack([2.0 * (qx * qz - qw * qy),
                            2.0 * (qy * qz + qw * qx),
                            1.0 - 2.0 * (qx * qx + qy * qy)])


def vertical_accel(accel, est):
    """1 セッション分の加速度レコードに、機上と同じ鉛直加速度・水平加速度を付ける。

    accel : decode_imulog.split() の accel（そのセッションだけ）
    est   : eskf.run_on_log() の結果（そのセッションだけ）。空なら全部 up_lp_ で作る
    戻り値: (t, az_world [m/s²]（重力を引いた鉛直）, horiz [m/s²]) の配列
    """
    a = accel.sort_values("t")
    t = a["t"].to_numpy(float)
    A = a[["ax", "ay", "az"]].to_numpy(float)
    up = np.zeros_like(A)

    # 水平化の前（ESKF の出力が始まる前）は比力の低域の向き（attitude.cpp の up_lp_ と同じ規則）
    t_init = est["t"].min() if len(est) else np.inf
    lp = np.array([0.0, 0.0, 1.0])
    lp_valid = False
    for i in range(len(t)):
        if t[i] >= t_init:
            break
        an = float(np.linalg.norm(A[i]))
        if an > 1.0 and abs(an - GRAVITY) < UP_LP_ACC_TOL:
            if not lp_valid:
                lp = A[i] / an
                lp_valid = True
            else:
                dt = t[i] - t[i - 1] if i else 0.0
                k = 1.0 - np.exp(-max(dt, 0.0) / UP_LP_TAU_S)
                lp = lp + (A[i] / an - lp) * k
                lp = lp / np.linalg.norm(lp)
        up[i] = lp if lp_valid else (0.0, 0.0, 1.0)

    if len(est):
        e = est.sort_values("t")
        ue = _quat_up(*(e[c].to_numpy(float) for c in ("qw", "qx", "qy", "qz")))
        te = e["t"].to_numpy(float)
        m = t >= t_init
        idx = np.clip(np.searchsorted(te, t[m], side="right") - 1, 0, len(te) - 1)
        up[m] = ue[idx]

    proj = (up * A).sum(axis=1)
    az_world = proj - GRAVITY
    horiz = np.sqrt(np.maximum((A * A).sum(axis=1) - proj * proj, 0.0))
    return t, az_world, horiz


def run_vario(parts, est, q_vel=KF_Q_VEL, q_bias=KF_Q_BIAS, r_baro=KF_R,
              guard=True, use_accel=True, use_gnss_vel=KF_GNSS_VEL_ENABLED):
    """セッションごとにバリオ KF を回す。

    use_gnss_vel: GNSS の上向き速度を速度の観測として入れるか（機上の KF_GNSS_VEL_ENABLED。既定は機上に合わせて False）

    戻り値: (結果の DataFrame, ガードの出入りのリスト)
      列: session, t（ホスト時刻 [s]）, vs [m/s], alt [m], bias [m/s²], az [m/s²]（その回の鉛直加速度の平均）, guard（0/1）
      出入り: (session, t, 'on'|'off', a_k, 低域, バイアス)
    """
    accel_all, baro_all, gnss_all = parts.get("accel"), parts.get("baro"), parts.get("gnssvel")
    if accel_all is None or baro_all is None:
        raise SystemExit("accel / baro レコードが無い")
    out, events = [], []

    for sess in sorted(baro_all["session"].unique()):
        acc = accel_all[accel_all["session"] == sess]
        bar = baro_all[baro_all["session"] == sess].sort_values("t")
        if acc.empty or bar.empty:
            continue
        e = est[est["session"] == sess] if est is not None else pd.DataFrame(columns=["t"])
        ta, az, hz = vertical_accel(acc, e)
        gn = (gnss_all[gnss_all["session"] == sess].sort_values("t")
              if (gnss_all is not None and use_gnss_vel) else pd.DataFrame(columns=["t", "ts", "velD", "sAcc"]))
        # 同じ測位（iTOW）は 1 回だけ（imu_kalman_gnss_vel_update の s_last_itow_ms）
        if len(gn):
            gn = gn.drop_duplicates(subset="ts", keep="first")

        # 時刻順に 3 種類の出来事を並べる（同じ時刻なら 気圧 → GNSS → 加速度。機上の loop() の順に近い）
        ev_t = np.concatenate([bar["t"].to_numpy(float), gn["t"].to_numpy(float), ta])
        ev_k = np.concatenate([np.zeros(len(bar), int), np.ones(len(gn), int), np.full(len(ta), 2)])
        ev_i = np.concatenate([np.arange(len(bar)), np.arange(len(gn)), np.arange(len(ta))])
        order = np.lexsort((ev_k, ev_t))
        zb = bar["alt"].to_numpy(float)
        gvd = -gn["velD"].to_numpy(float) if len(gn) else np.zeros(0)   # NED → 上向き正
        gsa = gn["sAcc"].to_numpy(float) if len(gn) else np.zeros(0)

        x = np.zeros(3)
        P = np.array([[100.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.1]])
        init = False
        last_pred = None
        acc_sum = hz_sum = 0.0
        acc_n = 0
        g_on, g_lp, g_fast, g_calm = False, 0.0, 0, None
        rows = []

        for j in order:
            t_now, kind, i = ev_t[j], ev_k[j], ev_i[j]
            if kind == 0:                                   # ---- 気圧（imu_kalman_baro_update）----
                if not init:
                    x = np.array([zb[i], 0.0, 0.0])
                    last_pred = t_now
                    init = True
                    continue
                S = P[0, 0] + r_baro
                K = P[:, 0] / S
                x = x + K * (zb[i] - x[0])
                P = P - np.outer(K, P[0, :])
                P = (P + P.T) * 0.5
                np.fill_diagonal(P, np.maximum(np.diag(P), 1e-9))
            elif kind == 1:                                 # ---- GNSS の上向き速度（imu_kalman_gnss_vel_update）----
                if not init or not (gsa[i] < GNSS_VSI_SACC_MAX_MPS) or not np.isfinite(gvd[i]):
                    continue
                rv = max(gsa[i] * gsa[i] * GNSS_VSI_R_SCALE, 0.001)
                S = P[1, 1] + rv
                K = P[:, 1] / S
                x = x + K * (gvd[i] - x[1])
                P = P - np.outer(K, P[1, :])
                P = (P + P.T) * 0.5
                np.fill_diagonal(P, np.maximum(np.diag(P), 1e-9))
            else:                                           # ---- 加速度（imu_feed_accel → imu_update の predict）----
                if not init:
                    acc_sum = hz_sum = 0.0                  # KF の初期化前は捨てる（imu_update と同じ）
                    acc_n = 0
                    continue
                acc_sum += az[i]
                hz_sum += hz[i]
                acc_n += 1
                if t_now - last_pred < IMU_KF_PREDICT_INTERVAL_S or acc_n == 0:
                    continue
                a_k = acc_sum / acc_n
                h_k = hz_sum / acc_n
                acc_sum = hz_sum = 0.0
                acc_n = 0
                dt = t_now - last_pred
                last_pred = t_now
                if dt <= 0.0 or dt > 0.5:
                    continue
                ok = use_accel
                if use_accel and guard:                     # ---- acc_guard_step ----
                    g_lp += (a_k - g_lp) * (dt / (KF_ACC_GUARD_TAU_S + dt))
                    fast = abs(a_k) > KF_ACC_GUARD_FAST_MPS2
                    g_fast = g_fast + 1 if fast else 0
                    if not g_on:
                        if g_fast >= KF_ACC_GUARD_FAST_STEPS or abs(g_lp) > KF_ACC_GUARD_SLOW_MPS2:
                            g_on, g_calm = True, None
                            P[0, 2] = P[2, 0] = P[1, 2] = P[2, 1] = 0.0
                            events.append((sess, float(t_now), "on", float(a_k), float(g_lp), float(x[2])))
                    else:
                        calm = abs(g_lp) < KF_ACC_GUARD_CLEAR_MPS2 and not fast
                        if not calm:
                            g_calm = None
                        elif g_calm is None:
                            g_calm = t_now
                        elif t_now - g_calm >= KF_ACC_GUARD_CLEAR_S:
                            g_on, g_calm = False, None
                            events.append((sess, float(t_now), "off", float(a_k), float(g_lp), float(x[2])))
                    ok = not g_on
                if ok:                                      # ---- kf_predict ----
                    F = np.array([[1.0, dt, 0.0], [0.0, 1.0, -dt], [0.0, 0.0, 1.0]])
                    x = np.array([x[0] + x[1] * dt, x[1] + (a_k - x[2]) * dt, x[2]])
                    P = F @ P @ F.T + np.diag([0.0, q_vel + KF_HORIZ_ACCEL_GAIN * h_k * h_k, q_bias])
                else:                                       # ---- kf_predict_baro_only ----
                    F = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                    x = np.array([x[0] + x[1] * dt, x[1], x[2]])
                    P = F @ P @ F.T + np.diag([0.0, q_vel, 0.0])
                rows.append((t_now, x[1], x[0], x[2], a_k, 1 if g_on else 0))

        if rows:
            r = np.array(rows)
            out.append(pd.DataFrame({"session": sess, "t": r[:, 0], "vs": r[:, 1], "alt": r[:, 2],
                                     "bias": r[:, 3], "az": r[:, 4], "guard": r[:, 5].astype(int)}))
    if not out:
        raise SystemExit("処理できるセッションが無かった")
    return pd.concat(out, ignore_index=True), events


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_imulog import load, split
    import eskf
    import merge_flight as mf

    ap = argparse.ArgumentParser(description="生 IMU ログからバリオ（imu.cpp）の V/S を再現する")
    ap.add_argument("path")
    ap.add_argument("--csv", metavar="FILE", help="30ms ごとの結果を CSV で書き出す")
    ap.add_argument("--flight-csv", metavar="DIR", help="機上の飛行 CSV のディレクトリ。KF_Vspeed と比べる")
    ap.add_argument("--no-guard", action="store_true", help="鉛直加速度のガード（KF_ACC_GUARD_*）を外す")
    ap.add_argument("--baro-only", action="store_true", help="加速度を使わない（気圧だけ）")
    ap.add_argument("--gnss-vel", action="store_true",
                    help="GNSS の上向き速度も観測に入れる（2026-11-12 より前の機上。settings.h の KF_GNSS_VEL_ENABLED = 1）")
    ap.add_argument("--q-vel", type=float, default=KF_Q_VEL)
    ap.add_argument("--q-bias", type=float, default=KF_Q_BIAS)
    ap.add_argument("--r", type=float, default=KF_R, help="気圧高度の観測ノイズ [m²]（KF_R）")
    args = ap.parse_args()

    parts = split(load(args.path))
    est = eskf.run_on_log(parts)                 # 上向きは機上と同じ ESKF（PC 版）から
    res, events = run_vario(parts, est, q_vel=args.q_vel, q_bias=args.q_bias, r_baro=args.r,
                            guard=not args.no_guard, use_accel=not args.baro_only,
                            use_gnss_vel=args.gnss_vel or KF_GNSS_VEL_ENABLED)

    # ホスト時刻 → JST（merge_flight と同じ橋渡し）
    gn = parts.get("gnssvel")
    to_jst = {}
    if gn is not None:
        for sess, g in gn.groupby("session"):
            f, _, n = mf.host_to_jst_fit(g)
            if f is not None:
                to_jst[sess] = f

    def clock(sess, t):
        if sess not in to_jst:
            return f"t={t:8.1f}s"
        s = float(to_jst[sess](t)) % 86400
        return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:04.1f}"

    print(f"{'sess':>5}{'長さ[s]':>9}{'V/S 最小':>10}{'V/S 最大':>10}{'|bias|最大':>11}{'ガード[s]':>10}")
    for sess, r in res.groupby("session"):
        dur = r["t"].max() - r["t"].min()
        gsec = r["guard"].sum() * IMU_KF_PREDICT_INTERVAL_S
        print(f"{sess:>5}{dur:>9.0f}{r['vs'].min():>10.2f}{r['vs'].max():>10.2f}"
              f"{r['bias'].abs().max():>11.2f}{gsec:>10.0f}")
    if events:
        print("\nガードの出入り（鉛直加速度がありえない値のあいだ気圧だけで回した）:")
        for sess, t, what, a_k, lp, b in events:
            print(f"  sess {sess:>3}  {clock(sess, t)}  {what:>3}  a={a_k:+.1f} lp={lp:+.1f} bias={b:+.2f}")

    if args.flight_csv:
        fl = mf.load_flight_csvs(args.flight_csv)
        print("\n機上の CSV の KF_Vspeed との比べ（同じセッションで JST が重なる所）:")
        for sess, r in res.groupby("session"):
            if sess not in to_jst:
                continue
            jst = to_jst[sess](r["t"].to_numpy(float))
            m = (fl["jst"] >= jst.min()) & (fl["jst"] <= jst.max()) & fl["KF_Vspeed"].notna()
            if m.sum() < 30:
                continue
            sim = np.interp(fl.loc[m, "jst"], jst, r["vs"].to_numpy(float))
            obs = fl.loc[m, "KF_Vspeed"].to_numpy(float)
            print(f"  sess {sess:>3}: {m.sum():6d} 行  相関 {np.corrcoef(sim, obs)[0, 1]:.3f}  "
                  f"差の RMS {np.sqrt(np.mean((sim - obs) ** 2)):.3f} m/s")

    if args.csv:
        res.to_csv(args.csv, index=False)
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
