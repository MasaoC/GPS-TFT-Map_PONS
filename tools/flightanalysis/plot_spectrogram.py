# -*- coding: utf-8 -*-
# ピッチ・ロール・昇降速度(KF V/S)の揺れが、いつ・どの周期で現れたかを見る
#
# 横方向の揺れ(ロール)は乗っている人の体感に直結するので、
# パイロットの「旋回後は不安定だった」という証言との対応を見るために
# ピッチ・V/Sと並べて比べられるようにしてある。
#
# plot_spectrum.py は旋回という一点でデータを2つに割って比べたが、
# 実際の変化は連続的に起きているはず。そこで時間とともにスペクトルが
# どう移り変わったかを見る。
#
# 上段: スペクトログラム。横軸が時刻、縦軸が周期、色が揺れの強さ。
#       「いつ、どのくらいの速さの揺れがあったか」が一枚で分かる。
# 下段: 周期帯ごとの揺れの大きさの推移。上段を4本の線に集約したもので、
#       数値として読み取りやすい。
#
# 時間と周期はどちらかを細かくすると他方が粗くなる関係にある。
# ここでは周期120秒までを見たいので4分の窓を使い、30秒ずつずらしている。
# そのため時間方向は4分程度でならされている点に注意。
#
# 使い方: python3 plot_spectrogram.py
# 出力: spectrogram_pitch_vspeed.png

import numpy as np
import pandas as pd
from scipy.signal import spectrogram, savgol_filter
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.ticker import FuncFormatter

# 設定 -------------------------------------------------------
EULER_FILENAME = '20260726_compensated.txt'   # flight_preprocess.py の出力
CSV_FILENAME = '2026-07-26_0804.csv'
PLOTNAME = 'spectrogram_pitch_vspeed.png'

FLIGHT_START = '08:21:26.30'   # 離陸
FLIGHT_END = '09:07:42.00'     # 着水
ATT_FS = 4.0                   # 姿勢角(ピッチ・ロール)を並べ直すサンプリング周波数 [Hz]
SEGMENT_SEC = 240.0            # 1回の変換に使う窓の長さ [秒]
HOP_SEC = 30.0                 # 窓をずらす量 [秒]
ATT_MIN_PERIOD = 1.0           # 姿勢角で信頼できる最短周期 [秒]
VS_MIN_PERIOD = 2.0            # V/Sで信頼できる最短周期 [秒]
MAX_PERIOD = 120.0             # 表示する最長周期 [秒]
BANDS = [(2, 4), (4, 10), (10, 30), (30, 120)]   # 集計する周期帯 [秒]
DB_RANGE = 30.0                # 色の濃淡がつく幅 [dB]。最大値からこの分だけ下まで

# 配色 -------------------------------------------------------
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
MARK = '#eb6834'       # 旋回時刻の基準線(データと役割が違うので別色相)
# 揺れの強さは大きさを表す量なので、単一色相(青)の明→暗ランプで表す
BLUE_RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
# 周期帯は順序のある区分なので、同じ青の段を離して割り当てる
BAND_COLORS = ['#86b6ef', '#3987e5', '#1c5cab', '#0d366b']


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
print(f"旋回の時刻: {daysec_to_hhmm(t_turn)}")

# 等間隔に並べ直す -------------------------------------------
# 姿勢角は間隔が不揃いなので、等間隔のグリッドへ補間してから変換する
tg_att = np.arange(t0, t1, 1.0 / ATT_FS)
pitch_g = np.interp(tg_att, te_all, euler['pitch'].to_numpy())
roll_g = np.interp(tg_att, te_all, euler['roll'].to_numpy())
# KF V/S はもともと0.5秒間隔で揃っているのでそのまま使う
m_c = (csv['t'] >= t0) & (csv['t'] <= t1)
tg_vs = csv['t'][m_c].to_numpy()
vs_g = csv['KF_Vspeed'][m_c].to_numpy()
VS_FS = 1.0 / float(np.median(np.diff(tg_vs)))

signals = {
    'Pitch': (tg_att, pitch_g, ATT_FS, 'deg', ATT_MIN_PERIOD),
    'Roll': (tg_att, roll_g, ATT_FS, 'deg', ATT_MIN_PERIOD),
    'V/S': (tg_vs, vs_g, VS_FS, 'm/s', VS_MIN_PERIOD),
}

# スペクトログラムの計算 -------------------------------------
spec = {}
for name, (tg, vg, fs, unit, min_period) in signals.items():
    n_seg = int(SEGMENT_SEC * fs)
    f, t_rel, sxx = spectrogram(vg, fs=fs, nperseg=n_seg,
                                noverlap=n_seg - int(HOP_SEC * fs),
                                detrend='linear', window='hann', mode='psd')
    keep = (f >= 1.0 / MAX_PERIOD) & (f <= 1.0 / min_period)
    f, sxx = f[keep], sxx[keep]
    t_abs = tg[0] + t_rel
    # 周期帯ごとの揺れの大きさ(その帯のスペクトルを積分して平方根)
    bands = {}
    for lo, hi in BANDS:
        if lo < min_period:
            continue
        m = (f >= 1.0 / hi) & (f <= 1.0 / lo)
        bands[(lo, hi)] = np.sqrt(np.trapz(sxx[m], f[m], axis=0)) if m.sum() >= 2 else None
    spec[name] = (t_abs, f, sxx, bands, unit)
    print(f"\n{name}: 窓{SEGMENT_SEC:.0f}秒 / {HOP_SEC:.0f}秒ずつ移動 → {len(t_abs)}列, "
          f"周期{1 / f.max():.1f}〜{1 / f.min():.0f}秒")
    for (lo, hi), v in bands.items():
        b = v[t_abs < t_turn].mean()
        a = v[t_abs >= t_turn].mean()
        print(f"  {lo:3d}〜{hi:3d}秒帯: 旋回前 {b:.4f} → 旋回後 {a:.4f} {unit} "
              f"({(a / b - 1) * 100:+.0f}%)  最大 {v.max():.4f} @ {daysec_to_hhmm(t_abs[np.argmax(v)])}")

# 作図 -------------------------------------------------------
cmap = LinearSegmentedColormap.from_list('blue_seq', BLUE_RAMP)
fig, axes = plt.subplots(2, len(spec), figsize=(6.4 * len(spec), 8.5), facecolor=SURFACE,
                         gridspec_kw={'height_ratios': [1.25, 1]})

for col, (name, (t_abs, f, sxx, bands, unit)) in enumerate(spec.items()):
    # --- 上段: スペクトログラム
    ax = axes[0][col]
    ax.set_facecolor(SURFACE)
    db = 10 * np.log10(sxx)
    vmax = float(np.percentile(db, 99))
    mesh = ax.pcolormesh(t_abs, 1.0 / f, db, cmap=cmap,
                         vmin=vmax - DB_RANGE, vmax=vmax, shading='gouraud')
    ax.set_yscale('log')
    ax.invert_yaxis()   # 上が短い周期(速い揺れ)、下が長い周期
    ax.axvline(t_turn, color=MARK, linewidth=1.5, linestyle='--')
    ax.text(t_turn, 0.98, ' turn', color=MARK, fontsize=9, va='top', ha='left',
            transform=ax.get_xaxis_transform(), fontweight='bold')
    ax.set_ylabel('Period [s]', color=INK_SUB, fontsize=10)
    ax.set_title(name, color=INK, fontsize=13, fontweight='bold', loc='left', pad=8)
    cb = fig.colorbar(mesh, ax=ax, pad=0.015)
    cb.set_label(f'Power [dB re {unit}²/Hz]', color=INK_SUB, fontsize=9)
    cb.ax.tick_params(colors=INK_SUB, labelsize=9)
    cb.outline.set_visible(False)

    # --- 下段: 周期帯ごとの推移
    ax2 = axes[1][col]
    ax2.set_facecolor(SURFACE)
    for k, ((lo, hi), v) in enumerate(bands.items()):
        ax2.plot(t_abs, v, color=BAND_COLORS[k], linewidth=2, label=f'{lo}-{hi} s')
    ax2.axvline(t_turn, color=MARK, linewidth=1.5, linestyle='--')
    ax2.set_ylabel(f'Amplitude [{unit}]', color=INK_SUB, fontsize=10)
    ax2.set_xlabel('Time of day', color=INK_SUB, fontsize=10)
    leg = ax2.legend(frameon=False, fontsize=9, labelcolor=INK_SUB, ncol=2,
                     title='Period band')
    leg.get_title().set_color(INK_SUB)
    leg.get_title().set_fontsize(9)

    for a in (ax, ax2):
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            a.spines[side].set_color(GRID)
        a.tick_params(colors=INK_SUB, labelsize=9)
        a.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hhmm(v)))
    ax2.grid(True, color=GRID, linewidth=1.0, linestyle='-')
    ax2.set_axisbelow(True)

fig.suptitle('When did the motion appear? — time-frequency view of pitch and vertical speed',
             color=INK, fontsize=15, fontweight='bold', x=0.01, ha='left')
fig.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"\nグラフを保存: {PLOTNAME}")
