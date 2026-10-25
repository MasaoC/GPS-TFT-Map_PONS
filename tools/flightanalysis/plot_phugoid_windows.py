# -*- coding: utf-8 -*-
# フゴイドらしい縦揺れを、フライト全体にわたって区間ごとに見るマルチページPDF
#
# plot_phugoid.py は「フライト全体をまとめて1つの周期・減衰比」に要約したが、
# 実際には揺れの強さも位相のそろい方も時間とともに変わる。60秒ずつに区切って
# 全区間を並べ、どこで強く出ていたか・どこは出ていなかったかを追えるようにする。
#
# 出力するPDFの構成:
#   1ページ目 : 全体の推移。区間ごとの「フゴイドらしさ」「揺れの大きさ」
#               「2つの位相」を時系列で並べたもの。どの時間帯を見るべきかが分かる。
#   2ページ目 : 一覧(コンタクトシート)。全区間の波形を小さく並べたもの。
#   3ページ目〜: 各区間の詳細。plot_phugoid.py の1段目左と同じ4本の波形を
#               1ページ1区間で描く。
#
# ===== 区間ごとの指標の作り方 ===============================
# 60秒しかない短い区間では、Welch法のコヒーレンスは窓が足りず当てにならない。
# そこでヒルベルト変換で各時刻の位相を直接求め、その差の平均をとる。
#
#   s1 = <cos(ピッチとV/Sの位相差)>        → +1に近いほど同位相  (条件1)
#   s2 = <cos(高度とG/Sの位相差 - 180度)>  → +1に近いほど逆位相  (条件2)
#
# 平均は振幅で重み付けする。振幅が小さい瞬間の位相はノイズでしかないため。
#
#   フゴイドらしさ = 揺れの大きさ × max(s1,0) × max(s2,0)
#
# 「大きく揺れていて、かつ2つの位相条件を両方満たす」区間ほど高くなる。
# どれかが欠ければ0に近づく。単位は揺れの大きさ(ピッチのRMS[度])に合わせてある。
#
# 使い方: python3 plot_phugoid_windows.py
# 出力: phugoid_windows.pdf

import os
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, hilbert, savgol_filter
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

# PDFの既定のフォント埋め込み(Type 3)は日本語のグリフ名を扱えず書き出しに失敗する。
# TrueTypeとして埋め込むよう切り替える
plt.rcParams['pdf.fonttype'] = 42

# 設定 -------------------------------------------------------
CSV_2026 = '2026-07-26_0804.csv'      # 高度・KF_Vspeed・対地速度
EULER_2026 = '20260726.txt'           # BNO085の生ログ(補正前のピッチ)
BETADA_2026 = 'beta_da_20260726.txt'  # 推定TAS
PDFNAME = 'phugoid_windows.pdf'

FLIGHT_2026 = ('08:21:26.30', '09:07:42.00')  # 離陸/着水。他のツールと同じ値
GS_MIN = 4.0          # これ未満の対地速度は地上/離着水とみなして捨てる [m/s]

FS = 2.0              # 解析のサンプリング周波数 [Hz]
PHUGOID_BAND = (4.0, 7.0)   # フゴイドが入る周期帯 [秒]
WINDOW_SEC = 60.0     # 1区間の長さ [秒]
HOP_SEC = 60.0        # 区間をずらす量 [秒]。WINDOW_SECより小さくすると区間が重なる
ORDER = 'time'        # 詳細ページの並び順: 'time'(時刻順) / 'score'(フゴイドらしさ順)
MAX_DETAIL_PAGES = 0  # 詳細ページの上限。0なら全区間

GRID_ROWS, GRID_COLS = 8, 6   # コンタクトシートの並べ方(1ページに 8×6 = 48区間)

# 縦軸の範囲は全区間で共通にする。そうしないと区間ごとに勝手に伸縮して、
# 「いつ大きく揺れていたか」が図から読み取れなくなるため。
# 範囲は全区間の振幅の分位点で決める。最大値に合わせると、ごく一部の大きな
# 山のせいで他の区間が潰れて見えなくなる。
AXIS_PERCENTILE = 99.9   # この分位点を範囲の基準にする。これを超える山ははみ出す
AXIS_MARGIN = 1.05       # 基準値に対する余白

# 図の下の注記に使う日本語フォント。見つからなければ英語のみで描く
JP_FONT_CANDIDATES = ['Hiragino Sans', 'Hiragino Maru Gothic Pro',
                      'Noto Sans CJK JP', 'IPAexGothic', 'Arial Unicode MS']

# 配色 -------------------------------------------------------
# plot_phugoid.py と同じ色を使う
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C_PITCH = '#2a78d6'   # ピッチ(IMU)
C_VS = '#eb6834'      # V/S
C_ALT = '#7a3d9e'     # 気圧高度
MARK = '#8a8880'      # 基準線
BAND_FILL = '#f0e6da'  # 強調の塗り


# ログの置き場所 --------------------------------------------
# スクリプトはこのフォルダ、ログは1つ上の tools/ にある構成なので、
# カレント → スクリプトと同じ場所 → その親、の順に探す
_HERE = os.path.dirname(os.path.abspath(__file__))
_SEARCH_DIRS = [os.getcwd(), _HERE, os.path.dirname(_HERE)]


def find_log(name):
    for d in _SEARCH_DIRS:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"{name} が見つかりません。探した場所: {_SEARCH_DIRS}")


def time_to_daysec(t):
    h, m, s = str(t).split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def daysec_to_hms(sec):
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}:{int(sec % 60):02d}"


def daysec_to_hm(sec):
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}"


def bandpass(x, lo, hi, fs):
    """周期 lo〜hi 秒の成分だけを取り出す"""
    b, a = butter(2, [1 / hi / (fs / 2), 1 / lo / (fs / 2)], btype='band')
    return filtfilt(b, a, x - np.mean(x))


def phase_score(x, y, offset_deg=0.0):
    """帯域制限した2つの信号の位相差を、振幅で重み付けして平均する。

    戻り値は (score, 平均位相[度])。
    score は cos(位相差 - offset) の重み付き平均で、
      +1 = いつも狙いどおりの位相関係、0 = 無関係、-1 = 逆
    となる。振幅の小さい瞬間の位相はノイズなので重みを下げている。
    """
    hx, hy = hilbert(x), hilbert(y)
    dphi = np.angle(hx * np.conj(hy)) - np.radians(offset_deg)
    w = np.abs(hx) * np.abs(hy)
    if w.sum() <= 0:
        return 0.0, 0.0
    score = float(np.sum(w * np.cos(dphi)) / np.sum(w))
    mean_phase = float(np.degrees(np.angle(np.sum(w * np.exp(1j * dphi)))))
    return score, mean_phase


# データ読み込み ---------------------------------------------
t0, t1 = (time_to_daysec(x) for x in FLIGHT_2026)
d = pd.read_csv(find_log(CSV_2026))
# v0.924 以降の CSV は GNSS 高度の列名が GNSS_Altitude。旧ログの Altitude と両対応にする
d = d.rename(columns={'GNSS_Altitude': 'Altitude'})
d['t'] = d['time'].apply(time_to_daysec)
d = d[(d['t'] >= t0) & (d['t'] <= t1) & (d['gs'] > GS_MIN)]
eu = pd.read_csv(find_log(EULER_2026))
eu['t'] = eu['time'].apply(time_to_daysec)
eu = eu[(eu['t'] >= t0) & (eu['t'] <= t1)]
TAS = float(pd.read_csv(find_log(BETADA_2026))['tas'].mean())

# 共通グリッドに揃える(位相の比較は同じ時刻で行う必要がある)
tg = np.arange(d['t'].min(), d['t'].max(), 1 / FS)
alt = np.interp(tg, d['t'].to_numpy(), d['Altitude'].to_numpy())
vs = np.gradient(alt, tg)     # KFを通さないV/S。KF出力はIMUと人工的に相関するため
gs = np.interp(tg, d['t'].to_numpy(), d['gs'].to_numpy())
pitch = np.interp(tg, eu['t'].to_numpy(), eu['pitch'].to_numpy())

# 帯域制限した4本。以降はすべてこれを使う
pb = bandpass(pitch, *PHUGOID_BAND, FS)
vb = bandpass(vs, *PHUGOID_BAND, FS)
ab = bandpass(alt, *PHUGOID_BAND, FS)
gb = bandpass(gs, *PHUGOID_BAND, FS)
# V/S は経路角[度]に直すとピッチと同じ単位で重ねられる
vb_ang = np.degrees(np.arcsin(np.clip(vb / TAS, -1, 1)))

# 旋回の時刻。他のツールと同じくヨー角速度が最大の瞬間とする
yaw_un = np.degrees(np.unwrap(np.radians(eu['yaw'].to_numpy())))
te = eu['t'].to_numpy()
win_y = max(5, int(4.0 / np.median(np.diff(te))) // 2 * 2 + 1)
yaw_rate = np.gradient(savgol_filter(yaw_un, win_y, 2), te)
t_turn = float(te[np.argmax(np.abs(yaw_rate))])

# 区間ごとの指標 ---------------------------------------------
n_win = int(WINDOW_SEC * FS)
hop = int(HOP_SEC * FS)
wins = []
for a0 in range(0, len(tg) - n_win + 1, hop):
    sl = slice(a0, a0 + n_win)
    s1, ph1 = phase_score(pb[sl], vb_ang[sl])            # 条件1: 同位相なら+1
    s2, ph2 = phase_score(ab[sl], gb[sl], offset_deg=180)  # 条件2: 逆位相なら+1
    amp = float(pb[sl].std())                            # 揺れの大きさ(ピッチRMS[度])
    wins.append(dict(
        i=len(wins), sl=sl, t_start=float(tg[a0]), t_end=float(tg[a0 + n_win - 1]),
        amp=amp, amp_vs=float(vb[sl].std()), amp_alt=float(ab[sl].std()),
        amp_gs=float(gb[sl].std()),
        s1=s1, ph1=ph1, s2=s2, ph2=ph2,
        score=amp * max(s1, 0.0) * max(s2, 0.0),
    ))

scores = np.array([w['score'] for w in wins])
amps = np.array([w['amp'] for w in wins])
print('=' * 64)
print(f'フゴイドらしさを区間ごとに見る  ({WINDOW_SEC:.0f}秒 × {len(wins)}区間, '
      f'{HOP_SEC:.0f}秒ずつずらす)')
print('=' * 64)
print(f"揺れの大きさ(ピッチRMS): {amps.min():.2f}〜{amps.max():.2f} 度 "
      f"(中央値 {np.median(amps):.2f})")
print(f"フゴイドらしさ:          {scores.min():.3f}〜{scores.max():.3f} "
      f"(中央値 {np.median(scores):.3f})")
print(f"旋回の時刻: {daysec_to_hms(t_turn)}")
top = sorted(wins, key=lambda w: -w['score'])[:8]
print('\n--- フゴイドらしさが高い区間 上位8 ---')
print(f"{'時刻':>10s} {'らしさ':>7s} {'ピッチRMS':>9s} {'条件1':>7s} {'位相':>7s} "
      f"{'条件2':>7s} {'位相':>7s}")
for w in top:
    print(f"{daysec_to_hms(w['t_start']):>10s} {w['score']:7.3f} {w['amp']:8.2f}度 "
          f"{w['s1']:+7.2f} {w['ph1']:+6.0f}度 {w['s2']:+7.2f} {w['ph2']:+6.0f}度")
print('\n  条件1の位相は0度が、条件2の位相は0度(=逆位相からのずれ)が理想')
before = np.array([w['score'] for w in wins if w['t_end'] < t_turn])
after = np.array([w['score'] for w in wins if w['t_start'] > t_turn])
if len(before) and len(after):
    print(f"\n旋回前({len(before)}区間) 平均 {before.mean():.3f}  →  "
          f"旋回後({len(after)}区間) 平均 {after.mean():.3f}")

# 全区間で共通に使う縦軸の範囲 -------------------------------
PLIM = float(np.percentile(np.abs(np.concatenate([pb, vb_ang])),
                           AXIS_PERCENTILE)) * AXIS_MARGIN
RLIM = float(np.percentile(np.abs(np.concatenate([gb, ab])),
                           AXIS_PERCENTILE)) * AXIS_MARGIN
clip_l = float(np.mean(np.abs(np.concatenate([pb, vb_ang])) > PLIM) * 100)
clip_r = float(np.mean(np.abs(np.concatenate([gb, ab])) > RLIM) * 100)
print(f"\n縦軸(全区間で共通): 左 ±{PLIM:.2f} 度 / 右 ±{RLIM:.3f} (m, m/s)")
print(f"  この範囲をはみ出すのは全サンプルの {clip_l:.2f}% / {clip_r:.2f}%")

# 作図の共通部品 ---------------------------------------------
avail = {f.name for f in fm.fontManager.ttflist}
JP = next((f for f in JP_FONT_CANDIDATES if f in avail), None)


def style(ax, xlab, ylab, title=None, fs_tick=9):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=1.0, zorder=0)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=fs_tick)
    if xlab:
        ax.set_xlabel(xlab, color=INK_SUB, fontsize=10)
    if ylab:
        ax.set_ylabel(ylab, color=INK_SUB, fontsize=10)
    if title:
        ax.set_title(title, color=INK, fontsize=11.5, fontweight='bold', loc='left')


def draw_window(ax, w, small=False):
    """1区間の波形。左軸にピッチとV/S(経路角)、右軸に高度とG/S。
    左軸の2本が同位相なら条件1、右軸の2本が逆位相なら条件2を満たす"""
    sl = w['sl']
    tt = tg[sl]
    ax.axhline(0, color=MARK, linewidth=0.8, zorder=1)
    lw = 1.1 if small else 1.8
    ax.plot(tt, pb[sl], color=C_PITCH, linewidth=lw, zorder=3)
    ax.plot(tt, vb_ang[sl], color=C_VS, linewidth=lw, zorder=4)
    # 縦軸は全区間で同じ(区間ごとに伸縮させない)。揺れの大小をページ間で比べるため
    ax.set_ylim(-PLIM, PLIM)
    ax.set_xlim(tt[0], tt[-1])
    axr = ax.twinx()
    axr.plot(tt, gb[sl], color=INK, linewidth=lw * 0.85, linestyle='--', zorder=5)
    axr.plot(tt, ab[sl], color=C_ALT, linewidth=lw * 0.9, linestyle=':', zorder=6)
    axr.set_ylim(-RLIM, RLIM)
    axr.set_xlim(tt[0], tt[-1])
    for s in ('top', 'left'):
        axr.spines[s].set_visible(False)
    axr.spines['right'].set_color(GRID)
    if small:
        # 一覧では波形の形だけ見えれば良い。目盛りを消して描画領域を広く取る
        axr.set_xticks([])
        axr.set_yticks([])
        axr.spines['right'].set_visible(False)
    return axr


def legend_handles():
    return [
        Line2D([], [], color=C_PITCH, lw=2, label='pitch [deg] (left)'),
        Line2D([], [], color=C_VS, lw=2, label='V/S as path angle [deg] (left)'),
        Line2D([], [], color=INK, lw=1.5, ls='--', label='G/S [m/s] (right)'),
        Line2D([], [], color=C_ALT, lw=1.6, ls=':', label='altitude [m] (right)'),
    ]


order_key = (lambda w: w['t_start']) if ORDER == 'time' else (lambda w: -w['score'])
ordered = sorted(wins, key=order_key)
if MAX_DETAIL_PAGES:
    ordered = ordered[:MAX_DETAIL_PAGES]

with PdfPages(PDFNAME) as pdf:
    # --- 1ページ目: 全体の推移 -----------------------------
    fig, axes = plt.subplots(3, 1, figsize=(11.7, 8.3), facecolor=SURFACE,
                             sharex=True)
    fig.subplots_adjust(top=0.86, bottom=0.20, left=0.09, right=0.97, hspace=0.28)
    tm = np.array([(w['t_start'] + w['t_end']) / 2 for w in wins])

    ax = axes[0]
    ax.bar(tm, scores, width=WINDOW_SEC * 0.85,
           color=[C_VS if s >= np.median(scores) else '#c9c7c1' for s in scores],
           zorder=3)
    style(ax, None, 'Phugoid-likeness', 'How phugoid-like was each window?')
    ax = axes[1]
    ax.plot(tm, amps, color=C_PITCH, linewidth=1.8, marker='o', markersize=3, zorder=3)
    style(ax, None, 'Pitch RMS [deg]', 'How big was the oscillation?')
    ax = axes[2]
    ax.axhline(0, color=MARK, linewidth=1.2, linestyle='--', zorder=2)
    ax.plot(tm, [w['ph1'] for w in wins], color=C_PITCH, linewidth=1.5,
            marker='o', markersize=3, zorder=3)
    ax.plot(tm, [w['ph2'] for w in wins], color=C_ALT, linewidth=1.5,
            marker='s', markersize=3, zorder=4)
    ax.set_ylim(-180, 180)
    ax.set_yticks([-180, -90, 0, 90, 180])
    style(ax, 'Time of day', 'Phase [deg]',
          'Are the two phase conditions met?  (0 = yes for both)')
    ax.legend(handles=[
        Line2D([], [], color=C_PITCH, lw=1.5, marker='o', markersize=4,
               label='test 1: pitch vs V/S (0 = in phase)'),
        Line2D([], [], color=C_ALT, lw=1.5, marker='s', markersize=4,
               label='test 2: altitude vs G/S (0 = anti-phase)'),
    ], frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='upper right', ncol=2)
    for a in axes:
        a.axvline(t_turn, color=MARK, linewidth=1.2, linestyle='--', zorder=2)
        a.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hm(v)))
    axes[0].text(t_turn, axes[0].get_ylim()[1], ' turn', color=INK_SUB, fontsize=9,
                 va='top', ha='left')

    fig.suptitle('Phugoid across the whole flight  (2026)', color=INK,
                 fontsize=15, fontweight='bold', x=0.012, ha='left', y=0.975)
    if JP:
        fig.text(0.012, 0.93,
                 f"{WINDOW_SEC:.0f}秒ごとに区切って、フゴイドらしさを追ったもの。"
                 f"全{len(wins)}区間。",
                 fontname=JP, fontsize=10, color=INK_SUB, va='top')
        fig.text(0.09, 0.155,
                 "「フゴイドらしさ」= 揺れの大きさ × 条件1の満たし具合 × 条件2の満たし具合。\n"
                 "　条件1: ピッチとV/Sが同位相（迎角がほぼ一定）\n"
                 "　条件2: 高度とG/Sが逆位相（高いとき遅い＝位置エネルギーと運動エネルギーの交換）\n"
                 "どちらか一方でも崩れると0に近づくので、「大きく揺れていて、かつフゴイドらしい」区間だけが高くなる。\n"
                 "位相は振幅で重み付けして平均している（振幅の小さい瞬間の位相はノイズのため）。",
                 fontname=JP, fontsize=9.5, color=INK_SUB, va='top', linespacing=1.7)
    pdf.savefig(fig, facecolor=SURFACE)
    plt.close(fig)

    # --- 2ページ目: 一覧(コンタクトシート) -----------------
    per_page = GRID_ROWS * GRID_COLS
    for page0 in range(0, len(wins), per_page):
        chunk = wins[page0:page0 + per_page]
        fig, axes = plt.subplots(GRID_ROWS, GRID_COLS, figsize=(11.7, 8.3),
                                 facecolor=SURFACE)
        fig.subplots_adjust(top=0.865, bottom=0.04, left=0.03, right=0.97,
                            hspace=0.55, wspace=0.12)
        for k, ax in enumerate(axes.flat):
            if k >= len(chunk):
                ax.axis('off')
                continue
            w = chunk[k]
            draw_window(ax, w, small=True)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ('top', 'right', 'left', 'bottom'):
                ax.spines[s].set_color(GRID)
            ax.set_facecolor(BAND_FILL if w['score'] >= np.median(scores) else SURFACE)
            ax.set_title(f"{daysec_to_hms(w['t_start'])[:8]}  {w['score']:.2f}",
                         color=INK, fontsize=7.5, pad=2)
        fig.suptitle('Contact sheet — every window  '
                     '(title = start time and phugoid-likeness)',
                     color=INK, fontsize=14, fontweight='bold', x=0.012, ha='left',
                     y=0.983)
        # 凡例は日本語の説明と段を分けて置く(重ならないように)
        fig.legend(handles=legend_handles(), frameon=False, fontsize=9,
                   labelcolor=INK_SUB, loc='upper left', ncol=4,
                   bbox_to_anchor=(0.012, 0.958))
        if JP:
            fig.text(0.012, 0.928,
                     "色が濃い区間はフゴイドらしさが中央値以上。詳しく見たい区間は次ページ以降で。\n"
                     f"縦軸は全区間で共通（左 ±{PLIM:.1f}度 / 右 ±{RLIM:.2f}）なので、コマの大小が"
                     "そのまま揺れの大小を表す。",
                     fontname=JP, fontsize=9.5, color=INK_SUB, va='top')
        pdf.savefig(fig, facecolor=SURFACE)
        plt.close(fig)

    # --- 3ページ目以降: 各区間の詳細 -----------------------
    for rank, w in enumerate(ordered, start=1):
        fig = plt.figure(figsize=(11.7, 8.3), facecolor=SURFACE)
        ax = fig.add_axes([0.085, 0.30, 0.84, 0.52])
        axr = draw_window(ax, w)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hms(v)))
        style(ax, f"Time of day  ({daysec_to_hms(w['t_start'])} "
                  f"+{WINDOW_SEC:.0f} s)",
              'Pitch [deg]  /  V/S as path angle [deg]')
        axr.set_ylabel('G/S [m/s]  /  altitude [m]', color=INK_SUB, fontsize=10)
        axr.tick_params(colors=INK_SUB, labelsize=9)
        # 縦軸を全区間共通にした結果、軸の中に凡例を置く余白が無い。図の側に出す
        fig.legend(handles=legend_handles(), frameon=False, fontsize=9,
                   labelcolor=INK_SUB, loc='upper left', ncol=4,
                   bbox_to_anchor=(0.085, 0.845))

        fig.suptitle(f"{daysec_to_hms(w['t_start'])} - {daysec_to_hms(w['t_end'])}"
                     f"   (window {w['i'] + 1} of {len(wins)}, "
                     f"{'time order' if ORDER == 'time' else f'rank {rank}'})",
                     color=INK, fontsize=14, fontweight='bold', x=0.012, ha='left',
                     y=0.965)
        head = (f"phugoid-likeness {w['score']:.3f}    "
                f"pitch RMS {w['amp']:.2f} deg    V/S RMS {w['amp_vs']:.3f} m/s    "
                f"altitude RMS {w['amp_alt']:.3f} m    G/S RMS {w['amp_gs']:.3f} m/s")
        fig.text(0.012, 0.90, head, color=INK_SUB, fontsize=10, va='top')
        test = (f"test 1 (pitch vs V/S, want 0 deg): {w['ph1']:+.0f} deg, score {w['s1']:+.2f}"
                f"        "
                f"test 2 (altitude vs G/S, want anti-phase): {w['ph2']:+.0f} deg "
                f"off, score {w['s2']:+.2f}")
        fig.text(0.012, 0.865, test, color=INK_SUB, fontsize=10, va='top')
        if JP:
            fig.text(0.085, 0.21,
                     "左軸の2本（ピッチとV/S）が重なって動いていれば条件1を満たす。\n"
                     "右軸の2本（高度とG/S）が上下逆に動いていれば条件2を満たす。\n"
                     "両方そろっていれば、この区間の揺れはフゴイドらしいと言える。\n"
                     "縦軸は全ページ共通なので、ページをめくれば揺れの大小がそのまま比べられる。",
                     fontname=JP, fontsize=9.5, color=INK_SUB, va='top', linespacing=1.7)
        pdf.savefig(fig, facecolor=SURFACE)
        plt.close(fig)

n_pages = 1 + (len(wins) + GRID_ROWS * GRID_COLS - 1) // (GRID_ROWS * GRID_COLS) \
    + len(ordered)
print(f"\nPDFを保存: {PDFNAME}  ({n_pages}ページ, 並び順は '{ORDER}')")
if JP is None:
    print('警告: 日本語フォントが見つからないため、日本語の説明は省略しました')
