#!/usr/bin/env python3
# ============================================================
# File    : tools/imulog/gnss_lag.py
# Project : PONS v7 / v7.1
# Role    : GNSS 速度（NAV-PVT）が IMU に対してどれだけ遅れているかを、生 IMU ログから求める。
#           結果は settings.h の ESKF_GNSS_VEL_DELAY_MS を決める材料。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/11/15
# ============================================================
"""
GNSS 速度の遅れの推定。

■ 何を比べるか
  IMU の比力を姿勢（eskf.py の結果）でワールドへ回し、重力を引いた水平加速度を積分すると、
  「測位と測位の間に速度がどれだけ変わったか」が IMU だけで出る。同じものは GNSS 速度の差分でも出る。
  GNSS 側の時刻を τ だけ前へずらしながら 2 つを比べ、食い違いが最小になる τ が遅れ。

  ヨーと航跡の差（旋回率 × 遅れ）から求める方法は、車体の幾何（後輪軸より前では進行方向が
  機首より内側を向く。距離 / 速度 ぶんの「進み」に見える）と区別できない。速度の変化で比べれば、
  IMU と GNSS アンテナは同じ箱の中なので、車体の幾何は両方に同じく入って消える。

■ GNSS の時刻
  生ログの GNSS レコードは t = Core0 が NAV-PVT を処理した時刻、ts = iTOW。
  t は描画などで数十〜数百 ms 揺れるので、そのままでは測位の時刻に使えない。
  「t − iTOW」のうち、近く（±30 秒）でいちばん小さい値を「待たされなかったときの届き方」とみなし、
  iTOW ＋ その値 を「待たされなければ届いたはずの時刻」（m）として使う。機上の補償も同じ作りにする
  （attitude.cpp / gnss.cpp）。求まる τ は「測位の瞬間 → 待たされなければ届く時刻」の遅れ。

使い方:
    python3 gnss_lag.py imuraw/20261113.bin
    python3 gnss_lag.py imuraw/20261113.bin --session 3 --plot lag.png
"""

import argparse
import os
import sys

import numpy as np

GRAVITY = 9.80665
WEEK_S = 604800.0


def itow_host_time(t, itow, win_s=30.0):
    """iTOW [s] を本体の時計の上へ写す（待たされなければ届いたはずの時刻）。

    t − iTOW の、前後 win_s 秒の最小値を使う。本体の時計のずれ（数十 ppm）は窓の中で数 ms 以内。
    """
    t = np.asarray(t, float)
    itow = np.asarray(itow, float).copy()
    # 週またぎ
    wrap = np.concatenate([[0.0], np.cumsum(np.diff(itow) < -WEEK_S / 2)]) * WEEK_S
    itow += wrap
    off = t - itow
    lo = np.searchsorted(t, t - win_s, side="left")
    hi = np.searchsorted(t, t + win_s, side="right")
    offmin = np.array([off[a:b].min() for a, b in zip(lo, hi)])
    return itow + offmin, off - offmin


def quat_rotate(q, v):
    """q（n×4、body→world）で v（n×3）を回す。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return np.einsum("nij,nj->ni", R, v)


def session_prep(est, accel):
    """姿勢で比力をワールドへ回し、水平の速度の変化を積分しておく。戻り値 (ts, V)。"""
    import pandas as pd
    e = est.sort_values("ts")
    a = pd.merge_asof(e[["ts"]], accel[["ts", "ax", "ay", "az"]].sort_values("ts"),
                      on="ts", direction="nearest")
    f = a[["ax", "ay", "az"]].to_numpy(float)
    q = e[["qw", "qx", "qy", "qz"]].to_numpy(float)
    ts = e["ts"].to_numpy(float)
    aw = quat_rotate(q, f)                       # ENU。上向きは +g が乗る
    dt = np.diff(ts, prepend=ts[0])
    dt[dt > 0.1] = 0.0                           # 途切れは積分しない
    return ts, np.cumsum(aw[:, :2] * dt[:, None], axis=0)


def session_lag(ts, V, gnss, taus, min_speed=3.0, sacc_max=0.5, win_s=5.0, use_arrival=False):
    """1 セッション（かその一部）ぶん。戻り値 (taus ごとの食い違いの 2 乗平均 [m²/s²], 使った測位の数, 届きの揺れ [s])。

    GNSS 速度 − IMU を積分した速度 を win_s 秒ごとに区切り、区切りの中で直線を引いて残りを食い違いとする。
    傾きの誤差・加速度のバイアスは積分すると直線に近いずれになるので、直線を引けば消える。残るのは
    「速度が変わる瞬間のずれ」＝遅れのぶん。
    （測位ごとの速度の差分で比べる形も試したが、GNSS 速度の雑音が差分で大きくなり、こちらのほうが谷がはっきりする）
    """
    g = gnss.sort_values("t")
    m, jit = itow_host_time(g["t"].to_numpy(), g["ts"].to_numpy())
    if use_arrival:
        m = g["t"].to_numpy(float)
    vg = np.column_stack([g["velE"].to_numpy(float), g["velN"].to_numpy(float)])
    sp = np.hypot(vg[:, 0], vg[:, 1])
    ok = (g["sAcc"].to_numpy(float) <= sacc_max) & (sp >= min_speed)
    ok &= (m > ts[0] + 1.0) & (m < ts[-1] - 1.0)
    med_dt = float(np.median(np.diff(m))) if len(m) > 1 else 0.5
    need = max(4, int(win_s / max(med_dt, 0.05) * 0.8))
    wid = np.floor((m - m[0]) / win_s).astype(int)
    groups = [np.flatnonzero((wid == w) & ok) for w in np.unique(wid)]
    groups = [ix for ix in groups if len(ix) >= need]
    J = np.full(len(taus), np.nan)
    n_used = int(sum(len(ix) for ix in groups))
    if n_used < 100:
        return J, 0, jit
    for i, tau in enumerate(taus):
        tq = m - tau
        D = vg - np.column_stack([np.interp(tq, ts, V[:, 0]), np.interp(tq, ts, V[:, 1])])
        ss = 0.0
        for ix in groups:
            x = m[ix] - m[ix].mean()
            for c in range(2):
                p = np.polyfit(x, D[ix, c], 1)
                r = D[ix, c] - np.polyval(p, x)
                ss += float(r @ r)
        J[i] = ss / n_used
    return J, n_used, jit


def parabola_min(taus, J):
    i = int(np.nanargmin(J))
    if i == 0 or i == len(taus) - 1 or np.isnan(J[i - 1]) or np.isnan(J[i + 1]):
        return float(taus[i])
    y0, y1, y2 = J[i - 1], J[i], J[i + 1]
    d = (y0 - 2 * y1 + y2)
    return float(taus[i] + 0.5 * (y0 - y2) / d * (taus[1] - taus[0])) if d > 0 else float(taus[i])


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_imulog import load, split
    from eskf import run_on_log

    ap = argparse.ArgumentParser(description="GNSS 速度の遅れを生 IMU ログから求める")
    ap.add_argument("path")
    ap.add_argument("--session", type=int, action="append", help="このセッションだけ（複数可）")
    ap.add_argument("--min-speed", type=float, default=3.0, help="この対地速度 [m/s] 以上の測位だけ使う")
    ap.add_argument("--eskf-delay", type=float, default=None,
                    help="姿勢を作る ESKF に入れる遅れの補償 [s]（run_on_log の gnss_delay）。"
                         "付けなければ eskf.py の GNSS_DELAY_DEFAULT（機上と同じ。いまは補償しない）")
    ap.add_argument("--plot", metavar="PNG", help="食い違いの曲線を書き出す")
    args = ap.parse_args()

    parts = split(load(args.path))
    if args.session:
        for k in list(parts):
            parts[k] = parts[k][parts[k]["session"].isin(args.session)]
    if args.eskf_delay is None:
        est = run_on_log(parts)                       # 機上と同じ既定
    else:
        est = run_on_log(parts, gnss_delay=args.eskf_delay)
    taus = np.arange(-0.20, 0.6001, 0.01)

    print(f"{'sess':>5}{'測位':>8}{'遅れ[ms]':>10}{'届いた時刻で[ms]':>16}"
          f"{'食い違い最小':>12}{'τ=0 で':>9}  {'届きの揺れ p50/p90/max [ms]':<28}  10 分ごとの遅れ [ms]")
    curves = []
    tot = np.zeros(len(taus)); tot_n = 0
    allchunks = []
    for sess in sorted(est["session"].unique()):
        g = parts["gnssvel"][parts["gnssvel"]["session"] == sess]
        if len(g) < 200:
            continue
        e = est[est["session"] == sess]
        a = parts["accel"][parts["accel"]["session"] == sess]
        ts, V = session_prep(e, a)
        J, n, jit = session_lag(ts, V, g, taus, min_speed=args.min_speed)
        if n == 0:
            continue
        Ja, _, _ = session_lag(ts, V, g, taus, min_speed=args.min_speed, use_arrival=True)
        lag, lag_a = parabola_min(taus, J), parabola_min(taus, Ja)
        j0 = J[np.argmin(np.abs(taus))]
        # 10 分ごとに分けて求める（ばらつきが、この値をどこまで信じてよいかの目安）
        chunks = []
        for c0 in np.arange(g["t"].min(), g["t"].max(), 600.0):
            gg = g[(g["t"] >= c0) & (g["t"] < c0 + 600.0)]
            Jc, nc, _ = session_lag(ts, V, gg, taus, min_speed=args.min_speed)
            if nc >= 300:
                chunks.append(parabola_min(taus, Jc) * 1000)
        allchunks += chunks
        print(f"{sess:>5}{n:>8}{lag*1000:>10.0f}{lag_a*1000:>16.0f}"
              f"{np.sqrt(np.nanmin(J)):>12.3f}{np.sqrt(j0):>9.3f}  "
              f"{np.percentile(jit,50)*1000:>6.0f} /{np.percentile(jit,90)*1000:>4.0f} /{jit.max()*1000:>5.0f}".ljust(80)
              + "  " + " ".join(f"{c:.0f}" for c in chunks))
        curves.append((sess, J, Ja))
        tot += np.nan_to_num(J) * n; tot_n += n
    if tot_n:
        Jt = tot / tot_n
        print(f"全体: 遅れ {parabola_min(taus, Jt)*1000:.0f} ms（食い違い RMS {np.sqrt(Jt.min()):.3f} m/s、"
              f"τ=0 で {np.sqrt(Jt[np.argmin(np.abs(taus))]):.3f} m/s）")
    if len(allchunks) >= 3:
        q = np.percentile(allchunks, [25, 50, 75])
        print(f"10 分ごとの値: 中央 {q[1]:.0f} ms、半分が {q[0]:.0f}〜{q[2]:.0f} ms に入る（{len(allchunks)} 個）。"
              f"この幅が ±30 ms より広ければ、値はまだ決められない")
    if args.plot and curves:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        for sess, J, Ja in curves:
            ax.plot(taus * 1000, np.sqrt(J), label=f"session {sess} (iTOW)")
            ax.plot(taus * 1000, np.sqrt(Ja), "--", label=f"session {sess} (arrival)")
        ax.set_xlabel("GNSS delay tau [ms]"); ax.set_ylabel("RMS of dv mismatch [m/s]")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(args.plot, dpi=120)
        print(f"wrote {args.plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
