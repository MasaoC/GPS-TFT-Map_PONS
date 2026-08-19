#!/usr/bin/env python3
"""
merged.csv を可視化する。

■ 何を見るための図か
  1) 時系列 : 実際の走行中、加減速・旋回に対して ESKF と BNO085 の姿勢が
              どう動いたかを時間軸で並べる。
  2) 応答   : 「単位入力あたり」の時間変化。加速イベント（または旋回イベント）の
              立ち上がりで揃えて平均し、入力の大きさで割る。
              縦軸は deg/(m/s²) または deg/(deg/s) になり、
              BNO085 が数秒かけて偽の傾きへ寄っていく様子が直接読める。
              ESKF が平坦なら、その偽の傾きを消せているということ。

■ 作図上の約束
  ・2 軸グラフにはしない。入力と姿勢は縦に並べて x 軸を共有する。
  ・系列色は参照パレットのスロット 1/2 を固定順で使う（ESKF=青, BNO085=橙）。
  ・凡例は常に出す。グリッドと軸は控えめにする。

使い方: python3 plot_response.py merged.csv -o figs/
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams["font.family"] = "Hiragino Sans"
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.25
plt.rcParams["grid.linewidth"] = 0.6
plt.rcParams["axes.spines.top"] = False
plt.rcParams["axes.spines.right"] = False
plt.rcParams["axes.edgecolor"] = "#8a8a85"
plt.rcParams["axes.labelcolor"] = "#52514e"
plt.rcParams["xtick.color"] = "#52514e"
plt.rcParams["ytick.color"] = "#52514e"
plt.rcParams["text.color"] = "#0b0b0b"

C_ESKF = "#2a78d6"     # 参照パレット スロット1: ESKF（GNSS あり）
C_BNO  = "#eb6834"     # 参照パレット スロット2: BNO085
C_NOG  = "#1baf7a"     # 参照パレット スロット3: ESKF（GNSS なし）
# 色は系列（エンティティ）に固定で紐付ける。系列が増えても既存の割り当ては変えない。
C_IN   = "#52514e"     # 入力（加速度・旋回率）は系列色を使わず地の色で描く
LW = 1.8


def baseline(df, col, quiet_col, quiet_th):
    """静穏区間（入力がほぼゼロ）の中央値を、そのセッションのゼロ点とする。

    機器の設置角がセッションごとに違うため、絶対値のままでは比較できない。
    """
    q = df[df[quiet_col].abs() < quiet_th]
    return df[col].median() if q.empty else q[col].median()


def lagged_slope(t, x, y, lags_s, clip=None):
    """入力 x に対する出力 y の「単位入力あたりの感度」をラグごとに求める。

    slope(τ) = x(t) に対する y(t+τ) の回帰係数（切片あり）。
    単位は [yの単位 / xの単位]。τ を横軸に取れば、
    「単位入力に対して応答が何秒かけてどこまで育つか」がそのまま読める。

    なぜイベント平均ではなくこれを使うか:
      当初はイベント立ち上がりで揃えて平均する方式にしたが、
      車の走行では機動が連続していて「前後に静穏のある孤立ステップ」が
      セッションあたり 2〜4 件しか取れず、±1σ が ±10 deg/(m/s²) を超えて
      解釈不能だった。ラグ回帰なら全サンプル（数千点）を使えるので安定する。

    戻り値: (slope, stderr) 各 lags_s と同じ長さ。
    """
    t = np.asarray(t, float); x = np.asarray(x, float); y = np.asarray(y, float)
    dt = np.median(np.diff(t))
    if clip is not None:
        x = np.where(np.abs(x) > clip, np.nan, x)
    slopes, errs = [], []
    for L in lags_s:
        k = int(round(L / dt))
        if k >= 0:
            xa, ya = x[:len(x) - k or None], y[k:]
        else:
            xa, ya = x[-k:], y[:len(y) + k or None]
        n = min(len(xa), len(ya))
        xa, ya = xa[:n], ya[:n]
        ok = np.isfinite(xa) & np.isfinite(ya)
        if ok.sum() < 30:
            slopes.append(np.nan); errs.append(np.nan); continue
        xa, ya = xa[ok], ya[ok]
        xm, ym = xa.mean(), ya.mean()
        sxx = ((xa - xm) ** 2).sum()
        if sxx <= 0:
            slopes.append(np.nan); errs.append(np.nan); continue
        b = ((xa - xm) * (ya - ym)).sum() / sxx
        resid = ya - (ym + b * (xa - xm))
        se = np.sqrt((resid ** 2).sum() / max(1, len(xa) - 2) / sxx)
        slopes.append(b); errs.append(se)
    return np.array(slopes), np.array(errs)


def hhmm(x):
    return f"{int(x//3600):02d}:{int(x%3600//60):02d}"


def plot_timeseries(m, outdir):
    for kind, in_col, in_lab, cols, labs, unit in [
        ("pitch", "accel_long", "前後加速度 [m/s²]",
         ("pitch", "bno_pitch"), ("ESKF", "BNO085"), "ピッチ [deg]"),
        ("roll", "turn_rate", "旋回レート [deg/s]",
         ("roll", "bno_roll"), ("ESKF", "BNO085"), "ロール [deg]"),
    ]:
        sess = sorted(m["session"].unique())
        fig, axes = plt.subplots(len(sess) * 2, 1, figsize=(13, 3.1 * len(sess)),
                                 sharex=False,
                                 gridspec_kw={"height_ratios": [1, 1.7] * len(sess),
                                              "hspace": 0.45})
        for si, s in enumerate(sess):
            d = m[(m.session == s) & m[in_col].notna()].sort_values("jst")
            if d.empty:
                continue
            ax_in, ax_y = axes[si * 2], axes[si * 2 + 1]
            b0 = baseline(d, cols[0], in_col, 0.3)
            b1 = baseline(d, cols[1], in_col, 0.3)

            ax_in.plot(d.jst, d[in_col], color=C_IN, lw=1.0)
            ax_in.axhline(0, color="#c9c9c4", lw=0.8)
            # 端点効果などの外れ値で軸が潰れないよう、分位でレンジを決める
            lim = np.nanpercentile(np.abs(d[in_col]), 99.5) * 1.3
            if np.isfinite(lim) and lim > 0:
                ax_in.set_ylim(-lim, lim)
            ax_in.set_ylabel(in_lab, fontsize=8)
            ax_in.set_title(f"session {s}   {hhmm(d.jst.min())}–{hhmm(d.jst.max())} JST",
                            fontsize=10, loc="left", pad=4)
            ax_in.tick_params(labelbottom=False, labelsize=8)

            ax_y.plot(d.jst, d[cols[0]] - b0, color=C_ESKF, lw=LW, label=labs[0])
            ax_y.plot(d.jst, d[cols[1]] - b1, color=C_BNO, lw=LW, label=labs[1], alpha=0.9)
            ax_y.axhline(0, color="#c9c9c4", lw=0.8)
            yl = np.nanpercentile(np.abs(pd.concat([d[cols[0]] - b0,
                                                    d[cols[1]] - b1])), 99.5) * 1.3
            if np.isfinite(yl) and yl > 0:
                ax_y.set_ylim(-yl, yl)
            ax_y.set_ylabel(unit, fontsize=8)
            ax_y.tick_params(labelsize=8)
            ax_y.legend(loc="upper right", fontsize=8, frameon=False, ncol=2)
            ticks = np.linspace(d.jst.min(), d.jst.max(), 7)
            ax_y.set_xticks(ticks)
            ax_y.set_xticklabels([hhmm(x) for x in ticks])
        fig.suptitle(
            f"実走行の時系列 — {'加減速とピッチ' if kind=='pitch' else '旋回とロール'}"
            f"（各セッションの静穏時をゼロ点に補正）", fontsize=12, y=0.995)
        fig.savefig(os.path.join(outdir, f"timeseries_{kind}.png"),
                    dpi=150, bbox_inches="tight", facecolor="#fcfcfb")
        plt.close(fig)


def plot_response(m, outdir):
    specs = [
        ("pitch", "accel_long", {"ESKF": "pitch", "BNO085": "bno_pitch"},
         8.0, "ピッチ感度 [deg / (m/s²)]",
         "単位前後加速度あたりのピッチ応答", 9.80665 / np.pi * 180 / 9.80665),
        ("roll", "turn_rate", {"ESKF": "roll", "BNO085": "bno_roll"},
         60.0, "ロール感度 [deg / (deg/s)]",
         "単位旋回レートあたりのロール応答", None),
    ]
    lags = np.arange(-3.0, 8.01, 0.2)
    for kind, in_col, series, clip, ylab, title, _ in specs:
        sess = sorted(m["session"].unique())
        fig, axes = plt.subplots(1, len(sess), figsize=(3.7 * len(sess), 3.7),
                                 sharey=True)
        axes = np.atleast_1d(axes)
        for ax, s_ in zip(axes, sess):
            d = m[(m.session == s_) & m[in_col].notna()].sort_values("jst")
            ax.axhline(0, color="#c9c9c4", lw=0.8)
            ax.axvline(0, color="#c9c9c4", lw=0.8, ls=":")
            npts = 0
            plots = [("ESKF", C_ESKF, series["ESKF"]),
                     ("BNO085", C_BNO, series["BNO085"])]
            nog_col = series["ESKF"] + "_nognss"
            if nog_col in d.columns and d[nog_col].notna().any():
                plots.insert(1, ("ESKF (GNSS なし)", C_NOG, nog_col))
            for name, c, ycol in plots:
                b, se = lagged_slope(d["jst"], d[in_col], d[ycol],
                                     lags, clip=clip)
                ax.fill_between(lags, b - 1.96 * se, b + 1.96 * se,
                                color=c, alpha=0.15, lw=0)
                ax.plot(lags, b, color=c, lw=LW, label=name)
                npts = len(d)
            ax.set_title(f"session {s_}   n={npts}", fontsize=9, loc="left")
            ax.set_xlabel("ラグ τ [s]（正 = 姿勢が入力より遅れる）", fontsize=8)
            ax.tick_params(labelsize=8)
        axes[0].set_ylabel(ylab, fontsize=8)
        axes[0].legend(loc="best", fontsize=8, frameon=False)
        fig.suptitle(f"{title}（全サンプルのラグ回帰、帯は95%信頼区間）",
                     fontsize=12, y=1.03)
        fig.savefig(os.path.join(outdir, f"response_{kind}.png"),
                    dpi=150, bbox_inches="tight", facecolor="#fcfcfb")
        plt.close(fig)


def attach_gnss_variants(m, binpath, cut_frac=0.6):
    """GNSS 援用の効き目を見るための追加ランを merged へ結合する。

      roll_nognss / pitch_nognss : 速度観測を一切入れずに回した結果
                                   （初期姿勢＋ジャイロ積分のまま漂う）
      roll_cut / pitch_cut       : セッションの cut_frac の時点で GNSS を打ち切った結果
                                   （飛行中に GNSS が途絶した場合の再現）
      cut_t                      : そのセッションの打ち切り時刻

    戻り値: (結合後の DataFrame, {セッション: 打ち切り時刻})
    """
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_imulog import load, split
    from eskf import run_on_log

    parts = split(load(binpath))
    base = run_on_log(parts)
    cut = {s: g["t"].min() + cut_frac * (g["t"].max() - g["t"].min())
           for s, g in base.groupby("session")}
    nog = run_on_log(parts, use_gnss=False)
    cutrun = run_on_log(parts, gnss_cutoff=cut)

    out = []
    for s, e in m.groupby("session"):
        e = e.sort_values("t")
        for src, suf in ((nog, "_nognss"), (cutrun, "_cut")):
            d = src[src["session"] == s][["t", "roll", "pitch"]].sort_values("t")
            d = d.rename(columns={"roll": "roll" + suf, "pitch": "pitch" + suf})
            e = pd.merge_asof(e, d, on="t", direction="nearest", tolerance=0.2)
        e["cut_t"] = cut.get(s, np.nan)
        out.append(e)
    return pd.concat(out, ignore_index=True), cut


def plot_gnss_timeseries(m, outdir):
    """GNSS あり／なし／BNO085 の姿勢を時系列で並べる。"""
    for kind, col, labl in (("roll", "roll", "ロール [deg]"),
                            ("pitch", "pitch", "ピッチ [deg]")):
        sess = sorted(m["session"].unique())
        fig, axes = plt.subplots(len(sess), 1, figsize=(13, 2.7 * len(sess)))
        axes = np.atleast_1d(axes)
        for ax, s_ in zip(axes, sess):
            d = m[m.session == s_].sort_values("jst")
            if d.empty or (col + "_nognss") not in d:
                continue
            b0 = d[col].median()
            ax.plot(d.jst, d[col] - b0, color=C_ESKF, lw=LW, label="ESKF (GNSS あり)")
            ax.plot(d.jst, d[col + "_nognss"] - d[col + "_nognss"].median(),
                    color=C_NOG, lw=LW, label="ESKF (GNSS なし)")
            ax.plot(d.jst, d["bno_" + col] - d["bno_" + col].median(),
                    color=C_BNO, lw=1.2, alpha=0.85, label="BNO085")
            ax.axhline(0, color="#c9c9c4", lw=0.8)
            ax.set_ylabel(labl, fontsize=8)
            ax.set_title(f"session {s_}   {hhmm(d.jst.min())}–{hhmm(d.jst.max())} JST",
                         fontsize=10, loc="left", pad=4)
            ax.tick_params(labelsize=8)
            ax.legend(loc="upper right", fontsize=8, frameon=False, ncol=3)
            ticks = np.linspace(d.jst.min(), d.jst.max(), 7)
            ax.set_xticks(ticks); ax.set_xticklabels([hhmm(x) for x in ticks])
            yl = np.nanpercentile(np.abs(d[col] - b0), 99.5) * 2.2
            if np.isfinite(yl) and yl > 0:
                ax.set_ylim(-yl, yl)
        fig.suptitle(f"GNSS 援用の有無による姿勢の違い — {labl}"
                     f"（各系列の中央値をゼロ点に補正）", fontsize=12, y=0.995)
        fig.tight_layout(rect=[0, 0, 1, 0.98])
        fig.savefig(os.path.join(outdir, f"gnss_timeseries_{kind}.png"),
                    dpi=150, bbox_inches="tight", facecolor="#fcfcfb")
        plt.close(fig)


def plot_gnss_dropout(m, outdir):
    """GNSS が途絶してからの経過時間に対する姿勢誤差の伸び。

    基準は GNSS を最後まで入れたラン。飛行中に測位が切れた場合、
    どのくらいの時間なら姿勢を信用してよいかの目安になる。

    表示は「そこまでの最大誤差」の累積にしてある。瞬時値は漂いの向きが
    行ったり来たりしてギザギザになり、運用判断に使えないため
    （いちど 3 度ずれたら、その後たまたま戻っても信用は戻らない）。

    セッションはパネルで分ける。色は他の図で系列（ESKF/BNO085）に
    割り当ててあるので、ここで別の意味に再利用しない。
    """
    sess = [s_ for s_ in sorted(m["session"].unique())
            if "roll_cut" in m and m[(m.session == s_)]["roll_cut"].notna().any()]
    if not sess:
        return
    fig, axes = plt.subplots(1, len(sess), figsize=(3.6 * len(sess), 3.6), sharey=True)
    axes = np.atleast_1d(axes)
    for ax, s_ in zip(axes, sess):
        d = m[(m.session == s_) & m["roll_cut"].notna()].sort_values("t")
        d = d[d.t > d.cut_t]
        if len(d) < 10:
            continue
        el = (d.t - d.cut_t).to_numpy()
        for col, style, lab in (("roll", "-", "ロール"), ("pitch", "--", "ピッチ")):
            err = (d[col] - d[col + "_cut"]).abs().to_numpy()
            ax.plot(el, np.fmax.accumulate(np.nan_to_num(err)),
                    color="#2a2a28", ls=style, lw=1.8, label=lab)
        ax.axhline(1.0, color="#c9c9c4", lw=0.8, ls=":")
        ax.set_title(f"session {s_}", fontsize=10, loc="left")
        ax.set_xlabel("GNSS 途絶からの経過 [s]", fontsize=8)
        ax.tick_params(labelsize=8)
    axes[0].set_ylabel("最大誤差（GNSS 継続時との差）[deg]", fontsize=8)
    axes[0].legend(fontsize=8, frameon=False, loc="upper left")
    fig.suptitle("GNSS 途絶後に姿勢がどれだけ漂うか（各セッションの 60% 地点で打ち切り、そこまでの最大値）",
                 fontsize=11, y=1.03)
    fig.savefig(os.path.join(outdir, "gnss_dropout.png"),
                dpi=150, bbox_inches="tight", facecolor="#fcfcfb")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="merged.csv")
    ap.add_argument("-o", "--outdir", default="figs")
    ap.add_argument("--bin", metavar="FILE",
                    help="生 IMU ログ。指定すると GNSS なし／途絶のランも計算して比較図を出す")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    m = pd.read_csv(args.path)
    m = m[m.latitude.notna()].copy()
    if args.bin:
        print("GNSS なし／途絶のランを計算中...", flush=True)
        m, cut = attach_gnss_variants(m, args.bin)
    plot_timeseries(m, args.outdir)
    plot_response(m, args.outdir)
    if args.bin:
        plot_gnss_timeseries(m, args.outdir)
        plot_gnss_dropout(m, args.outdir)
    print("wrote:", ", ".join(sorted(os.listdir(args.outdir))))


if __name__ == "__main__":
    main()
