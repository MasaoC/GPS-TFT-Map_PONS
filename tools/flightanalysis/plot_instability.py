# -*- coding: utf-8 -*-
# ピッチ・ロール・昇降速度の「不安定さ」が何と相関するかを調べる散布図
#
# パイロットの証言では旋回後のフライトが不安定だった。その原因が
# 風によるものか、ピッチが下がったことによる桁のT曲げ変形によるものかを
# 切り分けたい。その前段階として、まず不安定さが
#   ・時間の経過
#   ・その時間帯の平均ピッチ
# と相関しているかを見る。
#
# 「不安定さ」の測り方:
#   一定時間の窓ごとに、その窓の中の値から直線トレンドを引いた残りの
#   標準偏差を取る。トレンドを引くのは、ゆっくりした変化(例えば
#   降下し続けている)を振動と取り違えないため。
#   値が大きいほど、その時間帯は暴れていたことを意味する。
#
# 使い方: python3 plot_instability.py
# 出力: instability_correlation.png

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, savgol_filter
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

# 設定 -------------------------------------------------------
EULER_FILENAME = '20260726_compensated.txt'   # flight_preprocess.py の出力(補正済み姿勢角)
BETA_DA_FILENAME = 'beta_da_20260726.txt'     # flight_preprocess.py の出力(風・TASなど)
CSV_FILENAME = '2026-07-26_0804.csv'
PLOTNAME = 'instability_correlation.png'

FLIGHT_START = '08:21:26.30'   # 離陸
FLIGHT_END = '09:07:42.00'     # 着水
WINDOW_SEC = 45.0              # 不安定さを測る窓の長さ [秒]
MIN_SAMPLES = 20               # 窓内にこれだけデータがないと採用しない

# 配色 -------------------------------------------------------
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
TREND = '#0b0b0b'     # 回帰直線。データ点と役割が違うので無彩色にする
MARK = '#52514e'      # 旋回時刻の基準線
# 点の色は時刻を表す。旋回を境に色相が変わる発散型の配色にしてあるので、
# 「時間の流れ」と「旋回の前か後か」が同時に読める。
# 旋回から離れるほど濃く、旋回に近づくほど薄い(中間は無彩色に近づく)。
# 青側は参照パレットの段をそのまま使用。赤側は中間色と赤の基準色から作った。
TIME_COLORS = [
    '#104281', '#2a78d6', '#86b6ef',   # 旋回前(古い順→旋回直前)
    '#e99c9a', '#e34948', '#882c2b',   # 旋回後(旋回直後→新しい順)
]
N_ARM = 3             # 旋回の前後それぞれを何段階に分けるか


def time_to_daysec(t):
    h, m, s = t.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def daysec_to_hhmm(sec):
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}"


# データ読み込み ---------------------------------------------
euler = pd.read_csv(EULER_FILENAME)
csv = pd.read_csv(CSV_FILENAME)
betada = pd.read_csv(BETA_DA_FILENAME)
for df in (euler, csv, betada):
    df['t'] = df['time'].apply(time_to_daysec)
t0, t1 = time_to_daysec(FLIGHT_START), time_to_daysec(FLIGHT_END)

# 旋回の時刻を求める -----------------------------------------
# ヨー角速度が最も大きかった瞬間を、機体が向きを変えた時刻とみなす
te = euler['t'].to_numpy()
m_fl = (te >= t0) & (te <= t1)
yaw_un = np.degrees(np.unwrap(np.radians(euler['yaw'].to_numpy())))
win_y = max(5, int(4.0 / np.median(np.diff(te))) // 2 * 2 + 1)
yaw_rate = np.gradient(savgol_filter(yaw_un, win_y, 2), te)
t_turn = float(te[m_fl][np.argmax(np.abs(yaw_rate[m_fl]))])
print(f"旋回の時刻(ヨー角速度が最大): {daysec_to_hhmm(t_turn)} "
      f"(最大 {yaw_rate[m_fl][np.argmax(np.abs(yaw_rate[m_fl]))]:+.1f} deg/s)")


def detrended_std(t_seg, v_seg):
    """窓の中の直線トレンドを引いた残りの標準偏差。
    ゆっくりした変化を振動と取り違えないようにするための処理"""
    if len(v_seg) < 3:
        return np.nan
    slope, intercept = np.polyfit(t_seg, v_seg, 1)
    return float((v_seg - (slope * t_seg + intercept)).std())


# 窓ごとに集計 -----------------------------------------------
pitch = euler['pitch'].to_numpy()
roll = euler['roll'].to_numpy()
tc = csv['t'].to_numpy()
vspeed = csv['KF_Vspeed'].to_numpy()
tb = betada['t'].to_numpy()
wind = betada['wind_speed'].to_numpy()
tas = betada['tas'].to_numpy()

edges = np.arange(t0, t1 + WINDOW_SEC, WINDOW_SEC)
rows = []
for lo, hi in zip(edges[:-1], edges[1:]):
    me = (te >= lo) & (te < hi)
    mc = (tc >= lo) & (tc < hi)
    mb = (tb >= lo) & (tb < hi)
    if me.sum() < MIN_SAMPLES or mc.sum() < 5 or mb.sum() < 5:
        continue
    rows.append(((lo + hi) / 2.0,
                 float(pitch[me].mean()),
                 detrended_std(te[me], pitch[me]),
                 detrended_std(te[me], roll[me]),
                 detrended_std(tc[mc], vspeed[mc]),
                 float(wind[mb].mean()),
                 float(tas[mb].mean())))
(t_mid, pitch_mean, pitch_std, roll_std, vs_std,
 wind_mean, tas_mean) = (np.array(c) for c in zip(*rows))
print(f"{WINDOW_SEC:.0f}秒窓のデータ点: {len(t_mid)}点")
print(f"  ピッチの不安定さ  {pitch_std.min():.3f}〜{pitch_std.max():.3f} deg (中央値{np.median(pitch_std):.3f})")
print(f"  ロールの不安定さ  {roll_std.min():.3f}〜{roll_std.max():.3f} deg (中央値{np.median(roll_std):.3f})")
print(f"  V/Sの不安定さ     {vs_std.min():.3f}〜{vs_std.max():.3f} m/s (中央値{np.median(vs_std):.3f})")
print(f"  平均ピッチ        {pitch_mean.min():+.2f}〜{pitch_mean.max():+.2f} deg")

# 旋回の前後で比べる -----------------------------------------
before, after = t_mid < t_turn, t_mid >= t_turn
print(f"\n旋回前({before.sum()}窓) と 旋回後({after.sum()}窓) の比較:")
for lab, v, unit in (('ピッチの不安定さ', pitch_std, 'deg'),
                     ('ロールの不安定さ', roll_std, 'deg'),
                     ('V/Sの不安定さ  ', vs_std, 'm/s'),
                     ('平均ピッチ      ', pitch_mean, 'deg'),
                     ('風速            ', wind_mean, 'm/s')):
    print(f"  {lab}: 旋回前 {v[before].mean():+.3f} → 旋回後 {v[after].mean():+.3f} {unit} "
          f"({(v[after].mean() / v[before].mean() - 1) * 100:+.0f}%)")

# 相関 -------------------------------------------------------
print(f"\n相関係数:")
targets = (('ピッチの不安定さ', pitch_std), ('ロールの不安定さ', roll_std),
           ('V/Sの不安定さ', vs_std))
drivers = (('時間', t_mid), ('平均ピッチ', pitch_mean), ('風速', wind_mean), ('TAS', tas_mean))
for tl, tv in targets:
    for dl, dv in drivers:
        r = float(np.corrcoef(dv, tv)[0, 1])
        print(f"  {tl} - {dl}: r = {r:+.3f}")

# 作図 -------------------------------------------------------
# 「不安定さ3種」×「時間・平均ピッチ」の6枚を並べて比べる
# 各点がどの時間帯に属するか。旋回の前後をそれぞれ等分する
time_bin_edges = np.concatenate([
    np.linspace(t_mid.min(), t_turn, N_ARM + 1)[:-1],
    np.linspace(t_turn, t_mid.max(), N_ARM + 1),
])
time_bin_of = np.clip(np.digitize(t_mid, time_bin_edges[1:-1]), 0, 2 * N_ARM - 1)

fig, axes = plt.subplots(3, 2, figsize=(11.5, 11.5), facecolor=SURFACE)
panels = []
for row, (ylab, yv) in enumerate((('Pitch instability [deg]', pitch_std),
                                  ('Roll instability [deg]', roll_std),
                                  ('V/S instability [m/s]', vs_std))):
    panels.append((axes[row][0], t_mid, yv, 'Time of day', ylab, True))
    panels.append((axes[row][1], pitch_mean, yv, 'Mean pitch [deg]', ylab, False))
for ax, x, y, xlab, ylab, is_time in panels:
    ax.set_facecolor(SURFACE)
    if is_time:
        # 旋回の時刻。パイロットの証言はこれ以降のフライトについてのもの
        ax.axvline(t_turn, color=MARK, linewidth=1.2, linestyle='--', zorder=2)
        ax.text(t_turn, ax.get_ylim()[1], ' turn', color=MARK, fontsize=9,
                va='top', ha='left', transform=ax.get_xaxis_transform())
    # 時間帯ごとに色を変えて描く。古い順に描いて新しい点を上に重ねる
    for k in range(2 * N_ARM):
        sel = time_bin_of == k
        ax.scatter(x[sel], y[sel], color=TIME_COLORS[k], s=48,
                   linewidths=1.5, edgecolors=SURFACE, zorder=3 + k)
    # 傾向を見やすくする回帰直線と相関係数
    slope, intercept = np.polyfit(x, y, 1)
    xs = np.array([x.min(), x.max()])
    ax.plot(xs, slope * xs + intercept, color=TREND, linewidth=2, zorder=20)
    r = float(np.corrcoef(x, y)[0, 1])
    ax.text(0.98, 0.95, f"r = {r:+.3f}", transform=ax.transAxes, color=INK_SUB,
            fontsize=11, ha='right', va='top', fontweight='bold')
    ax.grid(True, color=GRID, linewidth=1.0, linestyle='-', zorder=0)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=10)
    ax.set_xlabel(xlab, color=INK_SUB, fontsize=10)
    ax.set_ylabel(ylab, color=INK_SUB, fontsize=10)
    if is_time:
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hhmm(v)))

# 色と時間帯の対応を図の下にまとめて示す(6枚とも同じ配色なので1つで足りる)
handles = [Line2D([], [], marker='o', linestyle='none', markersize=8,
                  markerfacecolor=TIME_COLORS[k], markeredgecolor=SURFACE, markeredgewidth=1.5,
                  label=f"{daysec_to_hhmm(time_bin_edges[k])}-{daysec_to_hhmm(time_bin_edges[k + 1])}"
                        + ("  | turn" if k == N_ARM - 1 else ""))
           for k in range(2 * N_ARM)]
leg = fig.legend(handles=handles, loc='lower center', ncol=2 * N_ARM, frameon=False,
                 fontsize=9, labelcolor=INK_SUB, title='Time of day  (blue = before turn, red = after)')
leg.get_title().set_color(INK_SUB)
leg.get_title().set_fontsize(10)

fig.suptitle(f'What drives the instability?  ({WINDOW_SEC:.0f} s windows, trend removed)',
             color=INK, fontsize=15, fontweight='bold', x=0.01, ha='left')
fig.tight_layout(rect=(0, 0.06, 1, 0.96))
fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"\nグラフを保存: {PLOTNAME}")
