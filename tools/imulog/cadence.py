#!/usr/bin/env python3
# ============================================================
# File    : tools/imulog/cadence.py
# Project : PONS v7 / v7.1
# Role    : 生 IMU ログの前後の加速度から、機体の前後の振動の周波数 [Hz] とパイロットのケイデンス [RPM] を推定する。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/11/18
# ============================================================
"""
前後の振動からケイデンスを推定する（2026-11-18 に作った。**人力機の飛行ログではまだ確かめていない**）。

■ 考え方
  パイロットが漕ぐ力はプロペラを通して推力になり、機体を前後に押す。左右の脚で 1 回転に 2 回踏むので、推力は
  踏み込みごとに強弱がつき、機体はその周期で前後に揺れる（とされている）。この揺れを前後の加速度から拾い、
  周波数をケイデンスに直す。

■ 機体の目安（2026-11-18 に聞いた値。**機体が変わったら下の PROP_RPM_REF などを直す**）
  プロペラ 135 RPM。歯車でクランク（ペダル）はプロペラの 1/1.5 倍速 → 135 / 1.5 = 90 RPM。
  左右の脚で 1 回転に 2 回踏むので、脚力の周期は 135 / 1.5 × 2 = 180 回/分 = 3.0Hz。
  機体の前後の振動も 180 回/分（3.0Hz）程度に出るはず。
  同じ回転から出うる別の線（どれもケイデンスに比例して動く。グラフに点線で描く）:
      クランク 1 回転（左右の脚の差）            1.5Hz  ＝ 踏み込みの ×0.5
      プロペラ 1 回転                            2.25Hz ＝ 踏み込みの ×0.75
      羽根の通過（2 枚羽根なら。枚数は未確認）   4.5Hz  ＝ 踏み込みの ×1.5
      踏み込みの 2 倍（力の波形が正弦でない分）  6.0Hz  ＝ 踏み込みの ×2
  踏み込みの線が弱くて別の線を拾うと、ケイデンスがこの比でずれて出る。出力の「比のスペクトル」とグラフで確かめること。
  **このツールのケイデンスはクランクの回転数 [RPM]**（自転車と同じ数え方。目安 90）。
  踏み込みの回数 [回/分]（目安 180）はその 2 倍で、振動の周波数 × 60 と同じ。

■ 前後の加速度
  加速度計が測るのは比力 f（加速度 − 重力）。機首方向の成分 f·e には、機首が θ 上を向いていると重力の分 g·sinθ が乗る。
  ESKF の姿勢（eskf.run_on_log() の qw..qz。センサー → ENU）でセンサー軸で見た「上」u を出し、
      a_fwd = f·e − g·(u·e)        （u·e = sinθ。ロールは前後の成分に効かない）
  を前後の加速度とする。e は機首方向のセンサー軸（decode_imulog.MOUNT_Q から作る。いまはセンサー +Z）。
  ESKF はジャイロで 250Hz の姿勢を追うので、踏み込みに合わせて機首が上下に揺れても、その重力の分は a_fwd に混ざらない。
  --att lp（比力の低域で上を決める。ESKF を回さないので速い）では混ざる。周波数は同じに出るが、振れの大きさが違う。
  ※ IMU は重心に無いので、機体が回る揺れ（ピッチ・ヨー）の角加速度 × 腕の長さも少し乗る。

■ 周波数の求め方
  1. 一定の間隔に並べ直し、--win 秒（既定 8 秒）の窓を --step 秒（既定 2 秒）ずつずらして、直線を引き Hann 窓を掛けて FFT する。
  2. 白色化: dB にして、周波数方向の中央値（±0.6Hz）を引く。「その周波数が周りより何 dB 高いか」になり、
     細い線だけが浮く。そのままの高さで山を選ぶと、路面や乱気流の幅の広い揺れ（高い周波数ほど強い）を拾ってしまう
     （2026-11-18 の自転車のログで、窓の半分が 3〜4Hz の雑音を「拾えた」ことにしていた）。
  3. 前後 --smooth 秒（既定 4 秒）の窓と平均する。続いている線は残り、たまたまの山は均される。
  4. --band（既定 2.2〜4.0Hz ＝ 踏み込み 132〜240 回/分 ＝ ケイデンス 66〜120 RPM）の中でいちばん高い所をその窓の振動とし
     （頂点は放物線で補う）、周りより --min-snr dB（既定 8）以上高く、帯の端でない窓を「拾えた」とする。
     8 dB は 2026-11-18 のログで、止まっている窓（雑音だけ）の最大が 7.8 dB だったことから（走っている窓は 64% が越えた）。
     **帯は 2 倍より狭くする。** 2 倍以上あると、同じ漕ぎのクランク 1 回転の線と踏み込みの線が両方入り、窓ごとに強いほうへ
     入れ替わる（2026-11-18 の自転車のログを 1.0〜2.6Hz で見たとき、踏み込み 2.1Hz の所で 1.05Hz を拾った窓があった）。
  まとめ（中央値など）は拾えた窓のうち、GNSS の対地速度が --min-gs（既定 3 m/s）以上のものだけ（10 個未満ならまとめない）。
  記録が 0.05 秒より長く途切れた所にかかる窓は使わない。

■ 比のスペクトル（どの線を拾ったかの確かめ）
  拾えた窓ごとに、白色化したスペクトルを「拾った周波数を 1 とした比」の軸で読み、平均する。ケイデンスが変わっても
  ×0.5（クランク）・×0.75（プロペラ）・×1.5（羽根）・×2 の線は同じ位置に重なるので、どれが本当にあるかが分かる。
  ×1 はいちばん高い所を選んだので高めに出る（選んだ分の偏り）。**×0.5 か ×2 が ×1 と同じくらい強ければ、
  拾った線が踏み込みではない（クランク 1 回転か、踏み込みの 2 倍）かもしれない**ので、--band を動かして比べる。
  2026-11-18 の自転車のログ（--band 1.0 2.6）では ×1 が 13 dB、×0.5 と ×2 が 5 dB ほど、×0.75 と ×1.5 は無し
  （踏み込みが主、左右の差が従。人力機で想定しているのと同じ並び）。既定の帯で見ると、踏み込みが 1.5Hz の所では
  その ×2 の 3Hz を拾い、×0.5 が ×1 と同じくらい強いという注意が出た（注意が働いた例）。

■ 時間
  ESKF の PC 版は全サンプルを 1 つずつ回すので、250Hz のログ 30 分で約 4 分かかる。同じログを何度も見るときは
  --est FILE を付ける（無ければ回して保存し、あれば読む）。**FILE を tools/ の中に置かないこと**
  （約 100MB になり、Arduino がビルドのたびにスケッチのフォルダごと写す。CLAUDE.md）。

■ 確かめ方（飛ぶ前に）
  台に載せた機体を漕ぐ（--min-gs 0）、または自転車に PONS を載せて漕ぎ、メトロノームやサイクルコンピュータのケイデンスと比べる。
  自転車は人力機と揺れ方が違う（路面の揺れも入り、変速で同じ速さでもケイデンスが変わる）ので、合えば「拾える」ことの
  確かめにはなるが、人力機で同じ強さで出るかは飛行ログを見るまで分からない。

使い方:
    python3 cadence.py imuraw/20261118.bin                                   # セッションごとのまとめ
    python3 cadence.py imuraw/20261118.bin --plot cadence.png --csv cadence.csv
    python3 cadence.py imuraw/20261118.bin --from 10:05 --to 10:30           # 時刻（JST）で切る。数字だけならセッションの頭からの秒
    python3 cadence.py imuraw/20261118.bin --est /tmp/est_1118.csv           # ESKF の結果を使い回す（tools/ の外に置く）
    python3 cadence.py imuraw/20261118.bin --band 1.0 2.6                    # 自転車（ケイデンス 30〜80 RPM）
    python3 cadence.py imuraw/20261118.bin --att lp                          # ESKF を回さず速く（振れの大きさは少し違う）
    python3 cadence.py imuraw/20261118.bin --axis up                         # 上下の加速度で見る（比べる用）
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_imulog import MOUNT_Q, load, split  # noqa: E402

GRAVITY = 9.80665

# ---- 機体の目安（2026-11-18 に聞いた値。機体が変わったらここを直す）----
PROP_RPM_REF = 135.0      # プロペラの回転数 [RPM]
PROP_PER_CRANK = 1.5      # 歯車: プロペラはクランクの 1.5 倍で回る（クランクはプロペラの 1/1.5 倍速）
STROKES_PER_REV = 2       # 左右の脚で 1 回転に 2 回踏む
PROP_BLADES = 2           # 羽根の数（未確認。羽根の通過の線の目安にだけ使う）
CRANK_RPM_REF = PROP_RPM_REF / PROP_PER_CRANK                  # 90 RPM
STROKE_HZ_REF = CRANK_RPM_REF * STROKES_PER_REV / 60.0         # 3.0Hz（180 回/分）

# 踏み込みの周波数に対する、同じ回転から出うる線の比（比のスペクトルとグラフの点線）
LINES = [
    (1.0 / STROKES_PER_REV, "crank 1/rev", "クランク 1 回転（左右の脚の差）"),
    (PROP_PER_CRANK / STROKES_PER_REV, "prop 1/rev", "プロペラ 1 回転"),
    (1.0, "stroke", "踏み込み（拾った線）"),
    (PROP_PER_CRANK * PROP_BLADES / STROKES_PER_REV, f"blade pass x{PROP_BLADES}",
     f"羽根の通過（{PROP_BLADES} 枚羽根として）"),
    (2.0, "2x stroke", "踏み込みの 2 倍"),
]
RATIO_GRID = np.arange(0.30, 2.60, 0.005)   # 比のスペクトルの横軸
RATIO_TOL = 0.03                            # 線の位置の許し（比で ±3%。窓ごとの周波数の誤差を吸う）

# ---- 解析の既定 ----
WIN_S = 8.0               # 1 回の FFT の長さ [s]（周波数の分解能 1/8 = 0.125Hz。頂点は補う）
STEP_S = 2.0              # 窓をずらす間隔 [s]
SMOOTH_S = 4.0            # 白色化したスペクトルを前後この秒数の窓と平均する
BAND_HZ = (2.2, 4.0)      # 踏み込みを探す帯 [Hz]。2 倍より狭くする（クランク 1 回転の線と踏み込みの線を両方入れない）
MIN_SNR_DB = 8.0          # 周りより何 dB 高ければ「拾えた」とするか（白色化して平均したあとの値）
MIN_GS_MPS = 3.0          # まとめに使う対地速度の下限 [m/s]
MIN_WINDOWS = 10          # 拾えた窓がこれより少なければまとめない（既定の 2 秒おきで 20 秒ぶん）。
                          # 2026-11-17 の走行（漕いでいない）は動いている窓 900 個近くのうち 1 個と 3 個だった
WHITEN_HZ = 1.2           # 白色化の中央値の幅 [Hz]（±0.6Hz）
PAD = 8                   # 0 を詰める倍率
GAP_S = 0.05              # これより長い記録の途切れにかかる窓は捨てる [s]
LP_HZ = 0.2               # --att lp の上向きの低域 [Hz]
SHOW_HZ = 10.0            # グラフの周波数の上限 [Hz]（SCH16T は LPF2 13Hz）

# ---- 時刻 ----
WEEK_S = 604800.0
GPS_UTC_LEAP_S = 18.0     # GPS 時刻 − UTC（2017-01-01〜。うるう秒が入ったら直す。表示にしか使わない）
JST_S = 9 * 3600.0

AXES = {"fwd": 0, "left": 1, "up": 2}   # 機体軸（前・左・上）の列。MOUNT_Q は機体軸 → センサー軸
AXIS_JA = {"fwd": "前後", "left": "左右", "up": "上下"}


# ============================================================
# 前後の加速度
# ============================================================
def axis_in_sensor(name):
    """機体の軸（前・左・上）をセンサー軸で表した単位ベクトル。

    mount_correct_quat() は q_body = q_sensor ⊗ q_mount なので、機体軸のベクトル v はセンサー軸で R(q_mount)·v。
    いまの配置では 前 = センサー +Z、左 = +Y、上 = −X（settings.h の IMU_MOUNT_Q* の測定と同じ）。
    """
    from eskf import quat_to_R
    return quat_to_R(np.asarray(MOUNT_Q, float))[:, AXES[name]]


def up_from_quat(qw, qx, qy, qz):
    """クォータニオン（センサー → ENU）の回転行列の第 3 行 = センサー軸で見た「上」（vario.py の _quat_up と同じ）。"""
    return np.column_stack([2.0 * (qx * qz - qw * qy),
                            2.0 * (qy * qz + qw * qx),
                            1.0 - 2.0 * (qx * qx + qy * qy)])


def get_attitude(parts, est_path):
    """ESKF の姿勢（session, ts, qw..qz）。est_path があれば読み、無ければ回して保存する。"""
    cols = ["session", "ts", "qw", "qx", "qy", "qz"]
    if est_path and os.path.exists(est_path):
        print(f"ESKF の結果を読む: {est_path}", flush=True)
        return pd.read_csv(est_path, usecols=cols)
    import eskf
    print(f"ESKF を回す（PC 版。{len(parts['gyro'])} サンプル。250Hz のログ 30 分で約 4 分）…", flush=True)
    est = eskf.run_on_log(parts)[cols]
    if est_path:
        est.to_csv(est_path, index=False)
        print(f"ESKF の結果を保存した: {est_path}")
    return est


def axis_accel(acc, est, e, att):
    """1 セッション分の比力から、軸 e の向きの加速度（重力を引いたもの）を作る。戻り値 (ts, a)。"""
    a = acc.sort_values("ts")
    ts = a["ts"].to_numpy(float)
    F = a[["ax", "ay", "az"]].to_numpy(float)
    if att == "eskf":
        q = est.sort_values("ts")
        m = pd.merge_asof(a[["ts"]], q[["ts", "qw", "qx", "qy", "qz"]], on="ts",
                          direction="nearest", tolerance=0.01)
        ok = m["qw"].notna().to_numpy()
        if ok.mean() < 0.5:
            print(f"  warn: 加速度の {100 * (1 - ok.mean()):.0f}% に姿勢が無い"
                  f"（ESKF は水平化した後から。--est が別のログの結果ではないか）")
        ts, F = ts[ok], F[ok]
        u = up_from_quat(*(m.loc[ok, c].to_numpy(float) for c in ("qw", "qx", "qy", "qz")))
    else:
        from scipy.signal import butter, sosfiltfilt
        fs = 1.0 / np.median(np.diff(ts))
        Flp = sosfiltfilt(butter(2, LP_HZ, fs=fs, output="sos"), F, axis=0)
        u = Flp / np.linalg.norm(Flp, axis=1, keepdims=True)
    return ts, F @ e - GRAVITY * (u @ e)


def resample(ts, x, fs):
    """一定の間隔 1/fs に並べ直す。GAP_S より長い途切れの中は NaN。"""
    tg = np.arange(ts[0], ts[-1], 1.0 / fs)
    xg = np.interp(tg, ts, x)
    j = np.clip(np.searchsorted(ts, tg), 1, len(ts) - 1)
    xg[(ts[j] - ts[j - 1]) > GAP_S] = np.nan
    return tg, xg


# ============================================================
# 窓ごとのスペクトルと周波数
# ============================================================
def spectra(tg, xg, fs, win_s, step_s, fmax):
    """窓ごとの振幅スペクトル。戻り値 (窓の中心の時刻, 途切れにかかったか, 周波数, 振幅 [窓 × 周波数])。

    振幅は正弦なら片振幅 [m/s²] になるように割ってある。途切れにかかった窓の行は NaN。
    """
    n = int(round(win_s * fs))
    hop = max(1, int(round(step_s * fs)))
    nfft = 1 << int(np.ceil(np.log2(n * PAD)))
    w = np.hanning(n)
    freqs = np.fft.rfftfreq(nfft, 1.0 / fs)
    fm = freqs <= fmax
    tt = np.arange(n)
    tc, gap, amp = [], [], []
    for i0 in range(0, len(xg) - n + 1, hop):
        x = xg[i0:i0 + n]
        tc.append(tg[i0 + n // 2])
        if np.isnan(x).any():
            gap.append(True)
            amp.append(np.full(int(fm.sum()), np.nan))
            continue
        x = x - np.polyval(np.polyfit(tt, x, 1), tt)
        gap.append(False)
        amp.append((2.0 * np.abs(np.fft.rfft(x * w, nfft)) / w.sum())[fm])
    return np.array(tc), np.array(gap, bool), freqs[fm], np.array(amp).reshape(len(tc), int(fm.sum()))


def whiten(amp, freqs):
    """dB にして周波数方向の中央値（±WHITEN_HZ/2）を引く = その周波数が周りより何 dB 高いか。途切れの行は NaN。"""
    from scipy.ndimage import median_filter
    gap = ~np.isfinite(amp[:, 0])
    db = 20.0 * np.log10(np.maximum(np.nan_to_num(amp, nan=1.0), 1e-9))
    k = max(3, int(round(WHITEN_HZ / (freqs[1] - freqs[0]))) | 1)
    W = db - median_filter(db, size=(1, k), mode="nearest")
    W[gap] = np.nan
    return W


def smooth_rows(W, k):
    """前後 k 行（窓）の平均。途切れの行（NaN）は数に入れず、その行自身は NaN のまま。"""
    ok = np.isfinite(W[:, 0])
    if k <= 0:
        return W.copy()
    Z = np.where(ok[:, None], W, 0.0)
    c = np.vstack([np.zeros((1, W.shape[1])), np.cumsum(Z, axis=0)])
    nc = np.concatenate([[0.0], np.cumsum(ok)])
    i = np.arange(len(W))
    lo, hi = np.clip(i - k, 0, len(W)), np.clip(i + k + 1, 0, len(W))
    S = (c[hi] - c[lo]) / np.maximum(nc[hi] - nc[lo], 1.0)[:, None]
    S[~ok] = np.nan
    return S


def detect(freqs, Ws, band):
    """窓ごとに、帯の中で白色化・平均したスペクトルがいちばん高い所。戻り値 (周波数, 高さ dB, 帯の端か)。"""
    ib = np.flatnonzero((freqs >= band[0]) & (freqs <= band[1]))
    if len(ib) < 3:
        raise SystemExit(f"--band {band[0]:g} {band[1]:g} が狭すぎる（または --win が短すぎる）")
    Y = np.nan_to_num(Ws[:, ib], nan=-np.inf)
    j = np.argmax(Y, axis=1)
    k = ib[j]
    r = np.arange(len(Ws))
    y0, y1, y2 = (Ws[r, np.clip(k + d, 0, Ws.shape[1] - 1)] for d in (-1, 0, 1))
    den = y0 - 2.0 * y1 + y2
    with np.errstate(invalid="ignore", divide="ignore"):
        delta = np.where(den < 0, 0.5 * (y0 - y2) / den, 0.0)
    delta = np.clip(np.nan_to_num(delta), -0.5, 0.5)
    f = freqs[k] + delta * (freqs[1] - freqs[0])
    snr = y1 - 0.25 * (y0 - y2) * delta
    edge = (j == 0) | (j == len(ib) - 1)
    return f, snr, edge


def ratio_spectrum(freqs, W, f_det, rows):
    """拾った周波数を 1 とした比の軸で、白色化したスペクトル（平均する前）を読んで平均する。"""
    acc = [np.interp(RATIO_GRID * f_det[i], freqs, W[i], right=np.nan) for i in np.flatnonzero(rows)]
    if not acc:
        return np.full(len(RATIO_GRID), np.nan)
    with np.errstate(invalid="ignore"):
        return np.nanmean(np.array(acc), axis=0)


def ratio_levels(rs):
    """比のスペクトルの、決まった比（LINES）の高さ [dB]。窓ごとの誤差を吸うため ±RATIO_TOL の中の最大。"""
    out = []
    for r, en, ja in LINES:
        m = np.abs(RATIO_GRID / r - 1.0) <= RATIO_TOL
        v = rs[m]
        out.append((r, en, ja, float(np.nanmax(v)) if np.isfinite(v).any() else np.nan))
    return out


# ============================================================
# 時刻
# ============================================================
def jst_mapper(gn):
    """本体の時刻 t [s] → JST の 0 時からの秒。GNSS のレコードが無ければ None。

    t − iTOW の中央値で写す（届くまでの待ちの数十 ms と時計のずれは表示には効かない）。
    """
    if gn is None or len(gn) < 10:
        return None
    t = gn["t"].to_numpy(float)
    itow = gn["ts"].to_numpy(float)
    itow = itow + np.concatenate([[0.0], np.cumsum(np.diff(itow) < -WEEK_S / 2)]) * WEEK_S
    off = float(np.median(t - itow))
    return lambda th: (np.asarray(th, float) - off - GPS_UTC_LEAP_S + JST_S) % 86400.0


def hms(s):
    s = float(s) % 86400.0
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:04.1f}"


def parse_when(s):
    """--from / --to。"HH:MM[:SS]" は JST、数字だけならセッションの頭からの秒。"""
    if s is None:
        return None
    if ":" in s:
        p = [float(v) for v in s.split(":")] + [0.0, 0.0]
        return ("jst", p[0] * 3600 + p[1] * 60 + p[2])
    return ("rel", float(s))


# ============================================================
# まとめ
# ============================================================
def print_session(sess, res, jst, args, levels):
    mov = res["moving"]
    use = res["valid"] & mov
    t0, t1 = res["t"].iloc[0], res["t"].iloc[-1]
    span = f"{hms(jst(t0))}〜{hms(jst(t1))} JST" if jst else f"{t0:.0f}〜{t1:.0f} s"
    print()
    print(f"■ セッション {sess}   {span}（{t1 - t0:.0f} 秒）")
    gs_txt = (f"対地速度 {args.min_gs:g} m/s 以上 {int(mov.sum())} 個" if res["gs_mps"].notna().any()
              else "GNSS が無いので速度で選ばない")
    print(f"  窓 {args.win:g} 秒 × {args.step:g} 秒おき: {len(res)} 個（記録の途切れ {int(res['gap'].sum())}）、{gs_txt}、"
          f"そのうち振動を拾えた {int(use.sum())} 個（{100.0 * use.sum() / max(1, mov.sum()):.0f}%）")
    if use.sum() < MIN_WINDOWS:
        print(f"  拾えた窓が {int(use.sum())} 個しかないので、まとめない（帯 {args.band[0]:g}〜{args.band[1]:g}Hz に、"
              f"周りより {args.min_snr:g} dB 以上高い線が続いていない。漕いでいないか、線が帯の外）")
        return
    f = res.loc[use, "freq_hz"].to_numpy()
    q25, q50, q75 = np.percentile(f, [25, 50, 75])

    def rng(k, fmt, unit):
        return f"{fmt.format(q50 * k)} {unit}（半分が {fmt.format(q25 * k)}〜{fmt.format(q75 * k)}）"
    k_rpm = 60.0 / STROKES_PER_REV
    print(f"  振動        : {rng(1.0, '{:.2f}', 'Hz')}")
    print(f"  踏み込み    : {rng(60.0, '{:.0f}', '回/分')}")
    print(f"  ケイデンス  : {rng(k_rpm, '{:.1f}', 'RPM')}   ← クランクの回転数（目安 {CRANK_RPM_REF:.0f}）")
    print(f"  プロペラ    : {rng(k_rpm * PROP_PER_CRANK, '{:.0f}', 'RPM')}   "
          f"← ケイデンス × {PROP_PER_CRANK:g}（目安 {PROP_RPM_REF:.0f}）")
    print(f"  振れの大きさ: 片振幅 {np.median(res.loc[use, 'amp_mps2']):.3f} m/s²（中央値）、"
          f"線の高さ {np.median(res.loc[use, 'snr_db']):.1f} dB（中央値。周りより）")
    print("  比のスペクトル（拾った線を ×1 として、ほかの線が周りより何 dB 高いか）:")
    v1 = next(v for r, _, _, v in levels if r == 1.0)
    for r, _, ja, v in levels:
        print(f"    ×{r:<5g} {v:5.1f} dB  {ja}")
    strong = [ja for r, _, ja, v in levels if r != 1.0 and np.isfinite(v) and v >= v1 - 3.0]
    if strong:
        print(f"  ★ {'・'.join(strong)} の線が、拾った線と同じくらい強い。拾ったのが踏み込みとは限らないので、"
              f"--band を動かしてグラフで確かめること")


def plot_session(path, sess, res, freqs, W, rs, levels, jst, args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FuncFormatter

    x = jst(res["t"].to_numpy()) if jst else res["t"].to_numpy() - res["t"].iloc[0]
    fm = freqs <= SHOW_HZ
    use = (res["valid"] & res["moving"]).to_numpy()

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), gridspec_kw=dict(height_ratios=[3, 2, 2.2]))
    ax = axs[0]
    pm = ax.pcolormesh(x, freqs[fm], W[:, fm].T, shading="nearest", cmap="magma", vmin=0, vmax=15)
    fig.colorbar(pm, ax=ax, pad=0.01, label="above local median [dB]")
    ax.plot(x[use], res.loc[use, "freq_hz"], ".", ms=2.5, color="#33d1ff", label="detected")
    ax.axhline(STROKE_HZ_REF, color="w", ls="--", lw=0.8, label=f"ref {STROKE_HZ_REF * 60:.0f}/min")
    for b in args.band:
        ax.axhline(b, color="#aaaaaa", ls=":", lw=0.7)
    ax.set_ylim(0, SHOW_HZ)
    ax.set_ylabel("frequency [Hz]")
    ax.set_title(f"session {sess}: {args.axis} accel ({args.att}), whitened, window {args.win:g}s", loc="left")
    ax.legend(loc="upper right", fontsize=8)

    ax = axs[1]
    rpm = res["freq_hz"] * 60.0 / STROKES_PER_REV
    ax.plot(x[use], rpm[use], ".", ms=4, color="#2a78d6", label="cadence (crank)")
    ax.axhline(CRANK_RPM_REF, color="#8a8880", ls="--", lw=0.8, label=f"ref {CRANK_RPM_REF:.0f} rpm")
    ax.set_ylim(args.band[0] * 60 / STROKES_PER_REV, args.band[1] * 60 / STROKES_PER_REV)
    ax.set_ylabel("cadence [rpm]")
    ax.legend(loc="upper left", fontsize=8)
    if res["gs_mps"].notna().any():
        ax2 = ax.twinx()
        ax2.plot(x, res["gs_mps"], color="#eb6834", lw=0.8, alpha=0.7)
        ax2.set_ylabel("ground speed [m/s]", color="#eb6834")
    for a in axs[:2]:
        a.set_xlim(x.min(), x.max())
        if jst:
            a.xaxis.set_major_formatter(FuncFormatter(lambda v, p: hms(v)[:8]))
        else:
            a.set_xlabel("time from session start [s]")

    ax = axs[2]
    ax.plot(RATIO_GRID, rs, color="#0b0b0b", lw=1)
    for r, en, _, v in levels:
        ax.axvline(r, color="#2a78d6" if r == 1.0 else "#8a8880", ls=":", lw=1)
        ax.text(r, 1.0, f" {en}", transform=ax.get_xaxis_transform(), rotation=90,
                va="top", ha="left", fontsize=7, color="#52514e")
    ax.set_xlim(RATIO_GRID[0], RATIO_GRID[-1])
    ax.set_xlabel("frequency / detected stroke frequency")
    ax.set_ylabel("above local median [dB]")
    ax.set_title(f"spectrum on the ratio axis, mean of {int(use.sum())} detected windows", loc="left", fontsize=10)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"  wrote {path}")


# ============================================================
def main():
    ap = argparse.ArgumentParser(description="生 IMU ログの前後の加速度から振動 [Hz] とケイデンス [RPM] を推定する")
    ap.add_argument("path", help="imuraw/YYYYMMDD.bin")
    ap.add_argument("--session", type=int, action="append", help="このセッションだけ（複数可）")
    ap.add_argument("--from", dest="t_from", help="ここから（HH:MM[:SS] は JST、数字だけならセッションの頭からの秒）")
    ap.add_argument("--to", dest="t_to", help="ここまで（同上）")
    ap.add_argument("--axis", choices=list(AXES), default="fwd", help="見る向き（既定 fwd = 前後）")
    ap.add_argument("--att", choices=["eskf", "lp"], default="eskf",
                    help="重力を引く姿勢。eskf（既定）か lp（比力の低域。速いが機首の揺れの分が混ざる）")
    ap.add_argument("--est", metavar="CSV", help="ESKF の結果の置き場。あれば読み、無ければ回して保存する（tools/ の外に置く）")
    ap.add_argument("--win", type=float, default=WIN_S, help=f"窓の長さ [s]（既定 {WIN_S:g}）")
    ap.add_argument("--step", type=float, default=STEP_S, help=f"窓をずらす間隔 [s]（既定 {STEP_S:g}）")
    ap.add_argument("--smooth", type=float, default=SMOOTH_S, help=f"前後この秒数の窓と平均する（既定 {SMOOTH_S:g}。0 で平均しない）")
    ap.add_argument("--band", type=float, nargs=2, default=list(BAND_HZ), metavar=("LO", "HI"),
                    help=f"踏み込みを探す帯 [Hz]（既定 {BAND_HZ[0]:g} {BAND_HZ[1]:g}）")
    ap.add_argument("--min-snr", type=float, default=MIN_SNR_DB, help=f"拾えたとする線の高さ [dB]（既定 {MIN_SNR_DB:g}）")
    ap.add_argument("--min-gs", type=float, default=MIN_GS_MPS,
                    help=f"まとめに使う対地速度の下限 [m/s]（既定 {MIN_GS_MPS:g}。台の上で漕ぐなら 0）")
    ap.add_argument("--csv", metavar="FILE", help="窓ごとの結果を書き出す")
    ap.add_argument("--plot", metavar="PNG", help="スペクトログラム・ケイデンス・比のスペクトルを描く（セッションが複数なら _s番号 を付ける）")
    args = ap.parse_args()
    if args.band[0] >= args.band[1]:
        ap.error("--band は LO < HI")
    if args.band[1] >= 2.0 * args.band[0]:
        print(f"注意: 帯 {args.band[0]:g}〜{args.band[1]:g}Hz は 2 倍以上の幅。同じ漕ぎのクランク 1 回転の線と踏み込みの線が"
              f"両方入り、窓ごとに強いほうへ入れ替わることがある")

    parts = split(load(args.path))
    if "accel" not in parts:
        raise SystemExit("加速度のレコードが無い")
    if args.session:
        for k in list(parts):
            parts[k] = parts[k][parts[k]["session"].isin(args.session)]
    est = get_attitude(parts, args.est) if args.att == "eskf" else None
    e = axis_in_sensor(args.axis)
    t_from, t_to = parse_when(args.t_from), parse_when(args.t_to)
    print(f"{AXIS_JA[args.axis]}の加速度（機体の {args.axis} = センサー軸 {np.round(e, 3).tolist()}）、"
          f"重力は{' ESKF の姿勢' if args.att == 'eskf' else '比力の低域'}で引く。"
          f"目安: 踏み込み {STROKE_HZ_REF:.2f}Hz = {STROKE_HZ_REF * 60:.0f} 回/分 = ケイデンス {CRANK_RPM_REF:.0f} RPM")

    acc_all = parts["accel"]
    gn_all = parts.get("gnssvel")
    sessions = [s for s in sorted(acc_all["session"].unique()) if (acc_all["session"] == s).sum() > 1]
    csv_rows = []
    for sess in sessions:
        acc = acc_all[acc_all["session"] == sess]
        gn = gn_all[gn_all["session"] == sess].sort_values("t") if gn_all is not None else None
        if gn is not None and gn.empty:
            gn = None
        es = None
        if est is not None:
            es = est[est["session"] == sess]
            if es.empty:
                print(f"\n■ セッション {sess}: 姿勢が無い（ESKF が水平化できなかった・--est に無い）。飛ばす")
                continue
        ts, a = axis_accel(acc, es, e, args.att)
        if len(ts) < 2 or ts[-1] - ts[0] < args.win:
            continue
        fs = float(np.round(1.0 / np.median(np.diff(ts))))
        fmax = min(0.45 * fs, max(SHOW_HZ, RATIO_GRID[-1] * args.band[1]) + WHITEN_HZ)
        tg, xg = resample(ts, a, fs)
        tc, gap, freqs, amp = spectra(tg, xg, fs, args.win, args.step, fmax)
        if len(tc) == 0:
            continue
        W = whiten(amp, freqs)
        Ws = smooth_rows(W, int(round(args.smooth / args.step)))
        f_det, snr, edge = detect(freqs, Ws, args.band)
        res = pd.DataFrame({"t": tc, "gap": gap, "edge": edge, "freq_hz": f_det, "snr_db": snr})
        res["amp_mps2"] = [np.interp(f, freqs, amp[i]) for i, f in enumerate(f_det)]
        res.loc[res["gap"], ["freq_hz", "snr_db", "amp_mps2"]] = np.nan
        jst = jst_mapper(gn)

        # 時刻で切る（平均は切る前の窓も使ってある）
        keep = np.ones(len(res), bool)
        for when, cmp in ((t_from, np.greater_equal), (t_to, np.less_equal)):
            if when is None:
                continue
            if when[0] == "jst":
                if jst is None:
                    raise SystemExit("GNSS が無いセッションは時刻（JST）で切れない。セッションの頭からの秒で指定する")
                keep &= cmp(jst(tc), when[1])
            else:
                keep &= cmp(tc - ts[0], when[1])
        if not keep.any():
            continue
        res, W, amp = res[keep].reset_index(drop=True), W[keep], amp[keep]

        if gn is not None:
            gs = np.hypot(gn["velN"].to_numpy(float), gn["velE"].to_numpy(float))
            res["gs_mps"] = np.interp(res["t"], gn["t"].to_numpy(float), gs)
            res["moving"] = res["gs_mps"] >= args.min_gs
        else:
            res["gs_mps"] = np.nan
            res["moving"] = True
        res["valid"] = ~res["gap"] & ~res["edge"] & (res["snr_db"] >= args.min_snr)
        use = (res["valid"] & res["moving"]).to_numpy()
        rs = ratio_spectrum(freqs, W, res["freq_hz"].to_numpy(), use)
        levels = ratio_levels(rs)
        print_session(sess, res, jst, args, levels)

        if args.plot:
            path = args.plot
            if len(args.session or sessions) > 1:
                root, ext = os.path.splitext(args.plot)
                path = f"{root}_s{sess}{ext or '.png'}"
            plot_session(path, sess, res, freqs, W, rs, levels, jst, args)

        if args.csv:
            fz = res["freq_hz"]
            csv_rows.append(pd.DataFrame({
                "session": sess,
                "t": res["t"].round(3),
                "jst": [hms(v) for v in jst(res["t"])] if jst else "",
                "gs_mps": res["gs_mps"].round(2),
                "freq_hz": fz.round(3),
                "strokes_per_min": (fz * 60.0).round(1),
                "cadence_rpm": (fz * 60.0 / STROKES_PER_REV).round(1),
                "prop_rpm": (fz * 60.0 / STROKES_PER_REV * PROP_PER_CRANK).round(1),
                "amp_mps2": res["amp_mps2"].round(4),
                "snr_db": res["snr_db"].round(1),
                "gap": res["gap"], "edge": res["edge"], "moving": res["moving"], "valid": res["valid"],
            }))

    if args.csv and csv_rows:
        pd.concat(csv_rows, ignore_index=True).to_csv(args.csv, index=False)
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
