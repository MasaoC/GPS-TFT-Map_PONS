# -*- coding: utf-8 -*-
# ピッチと昇降速度(KF V/S)の周期性を調べる
#
# 「どのくらいの周期で揺れていたか」を、旋回の前後で比べる。
# パイロットの証言では旋回後が不安定だったので、
# 揺れの大きさだけでなく「揺れの速さ(周期)」が変わったかを見る。
#
# 手法について:
#   生のFFTを1回かけるだけだと、結果がぎざぎざに暴れて読めない。
#   そこで区間を少しずつずらしながら何回もFFTをかけて平均する
#   Welch法を使う。これで滑らかで信頼できるスペクトルが得られる。
#   各区間は直線トレンドを引いてから変換する(ゆっくりした変化を
#   振動と取り違えないため)。
#
# 見られる周期の限界:
#   姿勢角は約0.21秒間隔だが最大0.5秒の粗さがあるので、周期1秒より
#   速い成分は信頼できない。KF V/Sは0.5秒間隔なので周期2秒が限界。
#   グラフはその範囲に限って描いている。
#
# 使い方: python3 plot_spectrum.py
# 出力: spectrum_pitch_vspeed.png

import numpy as np
import pandas as pd
from scipy.signal import welch, savgol_filter
import matplotlib.pyplot as plt

# 設定 -------------------------------------------------------
EULER_FILENAME = '20260726_compensated.txt'   # flight_preprocess.py の出力
CSV_FILENAME = '2026-07-26_0804.csv'
PLOTNAME = 'spectrum_pitch_vspeed.png'

FLIGHT_START = '08:21:26.30'   # 離陸
FLIGHT_END = '09:07:42.00'     # 着水
PITCH_FS = 4.0                 # ピッチを並べ直すサンプリング周波数 [Hz]
SEGMENT_SEC = 120.0            # Welch法で1回に変換する区間の長さ [秒]
PITCH_MIN_PERIOD = 1.0         # ピッチで信頼できる最短周期 [秒]
VS_MIN_PERIOD = 2.0            # V/Sで信頼できる最短周期 [秒]
MAX_PERIOD = 120.0             # 表示する最長周期 [秒]
# 揺れの大きさを周期帯ごとに集計するときの区切り [秒]
BANDS = [(2, 4), (4, 10), (10, 30), (30, 120)]

# 配色 -------------------------------------------------------
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C_BEFORE = '#2a78d6'   # 旋回前(スロット1)
C_AFTER = '#eb6834'    # 旋回後(スロット2)
MARK = '#8a8880'       # 理論値の基準線


def time_to_daysec(t):
    h, m, s = t.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def daysec_to_hhmm(sec):
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}"


# データ読み込み ---------------------------------------------
euler = pd.read_csv(EULER_FILENAME)
csv = pd.read_csv(CSV_FILENAME)
for df in (euler, csv):
    df['t'] = df['time'].apply(time_to_daysec)
t0, t1 = time_to_daysec(FLIGHT_START), time_to_daysec(FLIGHT_END)

# 旋回の時刻(ヨー角速度が最大の瞬間) ------------------------
te_all = euler['t'].to_numpy()
m_fl = (te_all >= t0) & (te_all <= t1)
yaw_un = np.degrees(np.unwrap(np.radians(euler['yaw'].to_numpy())))
win_y = max(5, int(4.0 / np.median(np.diff(te_all))) // 2 * 2 + 1)
yaw_rate = np.gradient(savgol_filter(yaw_un, win_y, 2), te_all)
t_turn = float(te_all[m_fl][np.argmax(np.abs(yaw_rate[m_fl]))])
print(f"旋回の時刻: {daysec_to_hhmm(t_turn)}  "
      f"(旋回前 {(t_turn - t0) / 60:.0f}分 / 旋回後 {(t1 - t_turn) / 60:.0f}分)")

# 等間隔に並べ直す -------------------------------------------
# ピッチは間隔が不揃いなので、等間隔のグリッドへ補間してから変換する
tg_pitch = np.arange(t0, t1, 1.0 / PITCH_FS)
pitch_g = np.interp(tg_pitch, te_all, euler['pitch'].to_numpy())
# KF V/S はもともと0.5秒間隔で揃っているのでそのまま使う
m_c = (csv['t'] >= t0) & (csv['t'] <= t1)
tg_vs = csv['t'][m_c].to_numpy()
vs_g = csv['KF_Vspeed'][m_c].to_numpy()
VS_FS = 1.0 / float(np.median(np.diff(tg_vs)))


def psd(t_grid, v_grid, fs):
    """旋回前と旋回後に分けて、それぞれのパワースペクトル密度を返す"""
    out = {}
    for lab, sel in (('before', t_grid < t_turn), ('after', t_grid >= t_turn)):
        n_seg = min(int(SEGMENT_SEC * fs), int(sel.sum()) // 3)
        f, p = welch(v_grid[sel], fs=fs, nperseg=n_seg, noverlap=n_seg // 2,
                     detrend='linear', window='hann')
        keep = f > 0
        out[lab] = (f[keep], p[keep], (sel.sum() - n_seg) // (n_seg // 2) + 1)
    return out


def band_rms(f, p, lo_period, hi_period):
    """ある周期帯に含まれる揺れの大きさ(実効値)。
    スペクトルをその範囲で積分して平方根を取る"""
    m = (f >= 1.0 / hi_period) & (f <= 1.0 / lo_period)
    if m.sum() < 2:
        return np.nan
    return float(np.sqrt(np.trapz(p[m], f[m])))


results = {}
for name, (tg, vg, fs, unit, min_period) in {
        'Pitch': (tg_pitch, pitch_g, PITCH_FS, 'deg', PITCH_MIN_PERIOD),
        'V/S': (tg_vs, vs_g, VS_FS, 'm/s', VS_MIN_PERIOD)}.items():
    res = psd(tg, vg, fs)
    results[name] = (res, unit, min_period)
    print(f"\n{name} のスペクトル (サンプリング {fs:.2f} Hz, "
          f"平均した区間数 旋回前{res['before'][2]}回 / 旋回後{res['after'][2]}回)")
    for lab, jp in (('before', '旋回前'), ('after', '旋回後')):
        f, p, _ = res[lab]
        m = (f >= 1.0 / MAX_PERIOD) & (f <= 1.0 / min_period)
        peak_period = 1.0 / f[m][np.argmax(p[m])]
        print(f"  {jp}: 最も強い周期 {peak_period:.1f} 秒")
    print(f"  周期帯ごとの揺れの大きさ [{unit}]:")
    for lo, hi in BANDS:
        if lo < min_period:
            continue
        b = band_rms(*res['before'][:2], lo, hi)
        a = band_rms(*res['after'][:2], lo, hi)
        print(f"    {lo:3d}〜{hi:3d}秒: 旋回前 {b:.4f} → 旋回後 {a:.4f} "
              f"({(a / b - 1) * 100:+.0f}%)")

# 作図 -------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), facecolor=SURFACE)
V_TAS = 6.97
phugoid = 2 * np.pi * V_TAS / (np.sqrt(2) * 9.797)   # 長周期振動の理論周期

for ax, (name, (res, unit, min_period)) in zip(axes, results.items()):
    ax.set_facecolor(SURFACE)
    # 理論上の長周期振動(フゴイド)の位置。ここに山があれば機体固有の振動
    ax.axvline(phugoid, color=MARK, linewidth=1.2, linestyle='--', zorder=2)
    ax.text(phugoid, 0.97, f' phugoid {phugoid:.1f}s', color=MARK, fontsize=9,
            va='top', ha='left', transform=ax.get_xaxis_transform())
    for lab, jp, color in (('before', 'Before turn', C_BEFORE),
                           ('after', 'After turn', C_AFTER)):
        f, p, _ = res[lab]
        m = (f >= 1.0 / MAX_PERIOD) & (f <= 1.0 / min_period)
        ax.plot(1.0 / f[m], p[m], color=color, linewidth=2, label=jp, zorder=3)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.invert_xaxis()   # 左が長い周期(ゆっくり)、右が短い周期(速い)
    ax.grid(True, color=GRID, linewidth=1.0, linestyle='-', zorder=0)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=10)
    ax.set_xlabel('Period [s]   (slow ← → fast)', color=INK_SUB, fontsize=10)
    ax.set_ylabel(f'Power spectral density [{unit}²/Hz]', color=INK_SUB, fontsize=10)
    ax.set_title(name, color=INK, fontsize=13, fontweight='bold', loc='left', pad=8)
    leg = ax.legend(frameon=False, fontsize=10, labelcolor=INK_SUB, loc='lower left')

fig.suptitle('Where is the motion? — periodicity of pitch and vertical speed',
             color=INK, fontsize=15, fontweight='bold', x=0.01, ha='left')
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"\nグラフを保存: {PLOTNAME}")
