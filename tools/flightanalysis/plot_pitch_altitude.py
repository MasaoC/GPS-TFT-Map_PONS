# -*- coding: utf-8 -*-
# 高度とピッチの関係を調べる散布図
#
# 縦軸にピッチ(一定時間の平均)、横軸は時刻または高度を取り、
# 推定TAS(対気速度)で色分けした散布図を描く。
# ピッチが時間とともにどう変わるか、高度や対気速度とどう絡んでいるかを
# 見るためのもの。動画とは独立した分析用。
# 横軸は X_AXIS で切り替える。
#
# 高度について:
#   CSVの pressure から計算した気圧高度(長時間で安定)と、KF_Altitude
#   (短時間で滑らか)を相補フィルタで合成し、さらに
#   createmovie_csv2mp4.py と同じ「高度を0〜13mに収める」補正をかける。
#   動画に表示している高度と同じ値になる。
#
# 使い方: python3 plot_pitch_altitude.py
# 出力: pitch_vs_altitude.png

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
from scipy.optimize import linprog
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

# 設定 -------------------------------------------------------
EULER_FILENAME = '20260726_compensated.txt'   # flight_preprocess.py の出力(補正済み姿勢角)
BETA_DA_FILENAME = 'beta_da_20260726.txt'     # flight_preprocess.py の出力(TASなど)
CSV_FILENAME = '2026-07-26_0804.csv'

X_AXIS = 'time'        # 横軸に何を取るか:   'time'(時刻) / 'altitude'(補正後高度)
COLOR_BY = 'altitude'  # 色分けに何を使うか: 'altitude' / 'tas'(推定対気速度) / 'time'
PLOTNAME = f'pitch_vs_{X_AXIS}_by_{COLOR_BY}.png'

FLIGHT_START = '08:21:26.30'   # 離陸
FLIGHT_END = '09:07:42.00'     # 着水
AVG_SEC = 7.0                 # 平均する窓の長さ [秒]
FUSE_TAU = 30.0                # 気圧高度とKF高度を合成する時定数 [秒]
ALT_LAND_TARGET = 0.0          # 着水時の高度 [m] (気圧高度の基準)
ALT_MIN, ALT_MAX = 0.0, 13.0   # 飛行中の高度の下限・上限 [m]
ALT_KNOT_CANDIDATES = [10.0, 7.0, 5.0, 4.0, 3.0, 2.0, 1.5, 1.0]  # ドリフト曲線の折れ点間隔[分]

# 配色 -------------------------------------------------------
# TASは大きさを表す量なので、単一色相(青)の明→暗で表す。
# 連続グラデーションだと中間の段が見分けられないため、区分に分けて
# 段の離れた色を割り当て、凡例で対応を示す。
# 一番明るい段は背景に埋もれるので250番から始める
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
# 色分けの区分の境目。両端は開いた区間(これ未満 / これ以上)になる。
# time は None にしてあり、飛行時間を等分する
COLOR_EDGES = {
    'altitude': [2.5, 5.0, 7.5, 10.0],   # [m]
    'tas': [6.4, 6.8, 7.2, 7.6],         # [m/s] 実データが6.5〜7.5に集中しているため細かく刻む
    'time': None,
}
COLOR_TITLE = {
    'altitude': 'Altitude [m]',
    'tas': 'Estimated TAS [m/s]',
    'time': 'Time of day',
}
# 250 / 350 / 450 / 550 / 700番。明度が十分離れるよう間を空けて選んである
BIN_COLORS = ['#86b6ef', '#5598e7', '#2a78d6', '#1c5cab', '#0d366b']


def time_to_daysec(t):
    h, m, s = t.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def daysec_to_hhmm(sec):
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}"


# データ読み込みと同期 ---------------------------------------
euler = pd.read_csv(EULER_FILENAME)
csv = pd.read_csv(CSV_FILENAME)
betada = pd.read_csv(BETA_DA_FILENAME)
euler['t'] = euler['time'].apply(time_to_daysec)
csv['t'] = csv['time'].apply(time_to_daysec)
betada['t'] = betada['time'].apply(time_to_daysec)
t0, t1 = time_to_daysec(FLIGHT_START), time_to_daysec(FLIGHT_END)

# 高度の合成 -------------------------------------------------
ts = csv['t'].to_numpy()
dt = float(np.median(np.diff(ts)))


def lowpass(v, tau):
    """位相がずれないローパス(前後両方向に掛ける)"""
    wn = (1.0 / (2.0 * np.pi * tau)) / (0.5 / dt)
    b, a = butter(2, min(wn, 0.99), btype='low')
    return filtfilt(b, a, v)


# 着水時の気圧を基準にして気圧から高度を出す(国際標準大気の式)
pres = csv['pressure'].to_numpy()
p_ref = float(np.interp(t1, ts, pres))
baro = 44330.0 * (1.0 - (pres / p_ref) ** (1.0 / 5.255)) + ALT_LAND_TARGET
kf = csv['KF_Altitude'].to_numpy()
# ゆっくりした成分は気圧高度、速い成分はKF高度(KF側の定数のずれはここで消える)
alt = lowpass(baro, FUSE_TAU) + kf - lowpass(kf, FUSE_TAU)


def solve_alt_drift(t_all, a_all, t_takeoff, t_land, knot_min):
    """高度が ALT_MIN〜ALT_MAX に収まり、離陸時は補正0、着水時にちょうど
    ALT_LAND_TARGET になるドリフト曲線のうち、最も滑らかなものを線形計画法で求める。
    createmovie_csv2mp4.py と同じ処理"""
    n_k = int(np.ceil((t_land - t_takeoff) / (knot_min * 60.0))) + 1
    if n_k < 3:
        return None, None
    knots = np.linspace(t_takeoff, t_land, n_k)
    sel = (t_all >= t_takeoff) & (t_all <= t_land)
    t_in, a_in = t_all[sel], a_all[sel]
    step = knots[1] - knots[0]
    idx = np.clip(((t_in - t_takeoff) / step).astype(int), 0, n_k - 2)
    w = (t_in - knots[idx]) / step
    W = np.zeros((len(t_in), n_k))
    r_ = np.arange(len(t_in))
    W[r_, idx] = 1.0 - w
    W[r_, idx + 1] = w
    n_u = n_k - 2
    D2 = np.zeros((n_u, n_k))
    for k in range(n_u):
        D2[k, k], D2[k, k + 1], D2[k, k + 2] = 1.0, -2.0, 1.0
    pad = lambda m: np.hstack([m, np.zeros((m.shape[0], n_u))])
    A_ub = np.vstack([pad(W), pad(-W),
                      np.hstack([D2, -np.eye(n_u)]), np.hstack([-D2, -np.eye(n_u)])])
    b_ub = np.concatenate([a_in - ALT_MIN, -(a_in - ALT_MAX), np.zeros(2 * n_u)])
    A_eq = np.zeros((2, n_k + n_u))
    A_eq[0, 0], A_eq[1, n_k - 1] = 1.0, 1.0
    b_eq = [0.0, float(np.interp(t_land, t_all, a_all)) - ALT_LAND_TARGET]
    res = linprog(np.concatenate([np.zeros(n_k), np.ones(n_u)]), A_ub, b_ub, A_eq, b_eq,
                  bounds=[(None, None)] * n_k + [(0, None)] * n_u, method='highs')
    return (knots, res.x[:n_k]) if res.success else (None, None)


# 折れ点の間隔を粗い方から試し、条件を満たせた最初の解を使う
for knot_min in ALT_KNOT_CANDIDATES:
    a_knots, a_drift = solve_alt_drift(ts, alt, t0, t1, knot_min)
    if a_knots is not None:
        break
if a_knots is None:
    raise ValueError("高度の上下限を満たすドリフト曲線が見つかりません")
alt = alt - np.interp(ts, a_knots, a_drift)
print(f"高度のドリフト補正: 折れ点{knot_min:.0f}分間隔、"
      f"補正量{a_drift.min():+.2f}〜{a_drift.max():+.2f}m")

# 10秒ごとに区切って平均 -------------------------------------
# ピッチと高度を同じ時間の区切りで平均し、1区切り1点として散布図にする
edges = np.arange(t0, t1 + AVG_SEC, AVG_SEC)
pitch_e = euler['pitch'].to_numpy()
te = euler['t'].to_numpy()
tas_b = betada['tas'].to_numpy()
tb = betada['t'].to_numpy()
rows = []
for lo, hi in zip(edges[:-1], edges[1:]):
    m_e = (te >= lo) & (te < hi)
    m_c = (ts >= lo) & (ts < hi)
    m_b = (tb >= lo) & (tb < hi)
    if m_e.sum() >= 5 and m_c.sum() >= 5 and m_b.sum() >= 5:
        rows.append(((lo + hi) / 2.0, float(alt[m_c].mean()),
                     float(pitch_e[m_e].mean()), float(tas_b[m_b].mean())))
t_mid, alt_avg, pitch_avg, tas_avg = (np.array(c) for c in zip(*rows))
print(f"{AVG_SEC:.0f}秒平均のデータ点: {len(t_mid)}点 "
      f"({daysec_to_hhmm(t0)}〜{daysec_to_hhmm(t1)})")
print(f"  高度  {alt_avg.min():+.1f}〜{alt_avg.max():+.1f} m (平均{alt_avg.mean():+.1f})")
print(f"  ピッチ {pitch_avg.min():+.2f}〜{pitch_avg.max():+.2f} deg (平均{pitch_avg.mean():+.2f})")
print(f"  TAS   {tas_avg.min():.2f}〜{tas_avg.max():.2f} m/s (平均{tas_avg.mean():.2f})")
for lab, x, y in (('高度 - ピッチ', alt_avg, pitch_avg),
                  ('TAS  - ピッチ', tas_avg, pitch_avg),
                  ('高度 - TAS  ', alt_avg, tas_avg)):
    rr = float(np.corrcoef(x, y)[0, 1])
    sl = float(np.polyfit(x, y, 1)[0])
    print(f"  {lab}: 相関係数 r = {rr:+.3f}  (傾き {sl:+.4f})")

# 作図 -------------------------------------------------------
# 色分けに使う量を区分に分け、区分ごとに色を割り当てる
color_val = {'altitude': alt_avg, 'tas': tas_avg, 'time': t_mid}.get(COLOR_BY)
if color_val is None:
    raise ValueError("COLOR_BY は 'altitude' / 'tas' / 'time' を指定してください")
color_edges = COLOR_EDGES[COLOR_BY]
if color_edges is None:
    # 区切りを決めていない量(時刻)は、範囲を等分する
    color_edges = list(np.linspace(color_val.min(), color_val.max(), len(BIN_COLORS) + 1)[1:-1])
n_bins = len(color_edges) + 1
bin_of = np.digitize(color_val, color_edges)

if X_AXIS == 'time':
    x_val, x_label, title = t_mid, 'Time of day', 'Pitch over time'
elif X_AXIS == 'altitude':
    x_val, x_label = alt_avg, 'Altitude [m]  (0 m = water surface)'
    title = 'Pitch vs Altitude'
else:
    raise ValueError("X_AXIS は 'time' か 'altitude' を指定してください")

fig, ax = plt.subplots(figsize=(9.5, 6.5), facecolor=SURFACE)
ax.set_facecolor(SURFACE)
# 点が重なっても輪郭が分かるよう、背景色のリングを付ける。
# 小さい区分から順に描き、大きい(濃い)点が上に来るようにする
for k in range(n_bins):
    m = bin_of == k
    ax.scatter(x_val[m], pitch_avg[m], color=BIN_COLORS[k],
               s=64, linewidths=1.5, edgecolors=SURFACE, zorder=3 + k)
ax.axhline(0, color=GRID, linewidth=1.0, zorder=1)
ax.grid(True, color=GRID, linewidth=1.0, linestyle='-', zorder=0)
ax.set_axisbelow(True)
for side in ('top', 'right'):
    ax.spines[side].set_visible(False)
for side in ('left', 'bottom'):
    ax.spines[side].set_color(GRID)
ax.tick_params(colors=INK_SUB, labelsize=10)
ax.set_xlabel(x_label, color=INK_SUB, fontsize=11)
ax.set_ylabel(f'Pitch [deg]  ({AVG_SEC:.0f} s average)', color=INK_SUB, fontsize=11)
ax.set_title(title, color=INK, fontsize=15, fontweight='bold', loc='left', pad=14)
if X_AXIS == 'time':
    # 目盛りは通算秒のままだと読めないので時刻に直す
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hhmm(v)))

# 色と値の対応は凡例で示す(段が離れているので点と照合しやすい)
def fmt(v):
    return daysec_to_hhmm(v) if COLOR_BY == 'time' else f"{v:.1f}"


def bin_label(k):
    if k == 0:
        return f"< {fmt(color_edges[0])}"
    if k == n_bins - 1:
        return f"≥ {fmt(color_edges[-1])}"
    return f"{fmt(color_edges[k - 1])} - {fmt(color_edges[k])}"


handles = [Line2D([], [], marker='o', linestyle='none', markersize=9,
                  markerfacecolor=BIN_COLORS[k], markeredgecolor=SURFACE, markeredgewidth=1.5,
                  label=f"{bin_label(k)}  ({int((bin_of == k).sum())})")
           for k in range(n_bins)]
leg = ax.legend(handles=handles, title=COLOR_TITLE[COLOR_BY], loc='center left',
                bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=10,
                labelcolor=INK_SUB, handletextpad=0.6)
leg.get_title().set_color(INK_SUB)
leg.get_title().set_fontsize(11)

fig.tight_layout()
fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"グラフを保存: {PLOTNAME}")
