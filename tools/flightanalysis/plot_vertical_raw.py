# -*- coding: utf-8 -*-
# 縦の運動の「生データ」をフライト全体にわたって見るマルチページPDF
#
# plot_phugoid.py / plot_phugoid_windows.py は周期4〜7秒の帯域だけを取り出して
# いる。そのため周期7秒より遅い成分(高度のドリフト、トリムの変化、
# ゆっくりした上昇・降下)は定義上すべて消えていて、右肩上がり・右肩下がりは
# 原理的に出ない。
# こちらはフィルタを一切かけず、ログの値をそのまま並べる。
#
# 出力するPDFの構成:
#   1ページ目 : フライト全体を1枚に。全体の流れとドリフトが分かる。
#   2ページ目〜: 一定時間ずつ拡大したもの。1ページ1区間。
#
# ===== 縦軸の決め方 =========================================
# 生データは量ごとに単位も値域も違うので、1つの軸に重ねられない。
# そこで量ごとに段を分けて、時間軸だけをそろえた形にする。
#
# 段ごとの縦軸の「幅」は全ページで共通にしてある(Y_MODE='span')。
# 高度は46分で20m近く動くので、全ページを同じ絶対値の範囲にすると
# 各ページの細かい動きが潰れてしまう。かといってページごとに
# 自動で伸縮させると、揺れの大小がページ間で比べられなくなる。
# そこで「幅は共通・中心はその区間の中央値」とすることで、
# ドリフトを追いながら振れ幅は比較できるようにしている。
# Y_MODE='absolute' にすると全ページ完全に同じ範囲になる(ドリフトが見える代わりに
# 細かい動きは潰れる)。
#
# 使い方: python3 plot_vertical_raw.py
# 出力: vertical_raw.pdf

import os
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

# PDFの既定のフォント埋め込み(Type 3)は日本語のグリフ名を扱えず書き出しに失敗する
plt.rcParams['pdf.fonttype'] = 42

# 設定 -------------------------------------------------------
CSV_2026 = '2026-07-26_0804.csv'              # 高度・気圧・KF_Vspeed・対地速度
EULER_2026 = '20260726.txt'                   # BNO085の生ログ(補正前)
EULER_2026_COMP = '20260726_compensated.txt'  # flight_preprocess.py の補正済み(比較用)
PDFNAME = 'vertical_raw.pdf'

FLIGHT_2026 = ('08:21:26.30', '09:07:42.00')  # 離陸/着水。他のツールと同じ値
GS_MIN = 4.0          # これ未満の対地速度は地上/離着水とみなして捨てる [m/s]

FS = 2.0              # 並べ直すサンプリング周波数 [Hz]。CSVの0.5秒間隔に合わせる
WINDOW_SEC = 120.0    # 1ページに描く長さ [秒]
HOP_SEC = 120.0       # ページをずらす量 [秒]。WINDOW_SECより小さくすると重なる
Y_MODE = 'span'       # 'span'(幅だけ共通・中心は区間ごと) / 'absolute'(全ページ同一)
SPAN_PERCENTILE = 95  # 縦軸の幅を決める分位点。区間ごとの振れ幅のこの分位点を使う
SPAN_MARGIN = 1.15    # 幅に対する余白

# 図の注記に使う日本語フォント。見つからなければ英語のみで描く
JP_FONT_CANDIDATES = ['Hiragino Sans', 'Hiragino Maru Gothic Pro',
                      'Noto Sans CJK JP', 'IPAexGothic', 'Arial Unicode MS']

# 配色 -------------------------------------------------------
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C_ALT = '#7a3d9e'     # 高度
C_VS = '#eb6834'      # 昇降速度
C_PITCH = '#2a78d6'   # ピッチ
C_GS = '#0b0b0b'      # 対地速度
SUB = '#c9c7c1'       # 副系列(比較用に薄く重ねるもの)
MARK = '#8a8880'      # 基準線


# ログの置き場所 --------------------------------------------
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
euc = pd.read_csv(find_log(EULER_2026_COMP))
euc['t'] = euc['time'].apply(time_to_daysec)
euc = euc[(euc['t'] >= t0) & (euc['t'] <= t1)]

# 共通の時間グリッドへ並べ直すだけ。平滑化も帯域制限も一切しない
tg = np.arange(d['t'].min(), d['t'].max(), 1 / FS)
alt = np.interp(tg, d['t'].to_numpy(), d['Altitude'].to_numpy())
alt_kf = np.interp(tg, d['t'].to_numpy(), d['KF_Altitude'].to_numpy())
vs_kf = np.interp(tg, d['t'].to_numpy(), d['KF_Vspeed'].to_numpy())
vs_baro = np.gradient(alt, tg)   # 気圧高度の差分。フィルタではないが導出値である点に注意
gs = np.interp(tg, d['t'].to_numpy(), d['gs'].to_numpy())
pitch = np.interp(tg, eu['t'].to_numpy(), eu['pitch'].to_numpy())
pitch_c = np.interp(tg, euc['t'].to_numpy(), euc['pitch'].to_numpy())

# 旋回の時刻。他のツールと同じくヨー角速度が最大の瞬間とする
te = eu['t'].to_numpy()
yaw_un = np.degrees(np.unwrap(np.radians(eu['yaw'].to_numpy())))
win_y = max(5, int(4.0 / np.median(np.diff(te))) // 2 * 2 + 1)
t_turn = float(te[np.argmax(np.abs(np.gradient(savgol_filter(yaw_un, win_y, 2), te)))])

# 段の定義。(ラベル, 主系列, 副系列, 副系列のラベル, 色)
STRIPS = [
    ('Altitude [m]', alt, alt_kf, 'KF_Altitude', C_ALT),
    ('V/S [m/s]', vs_kf, vs_baro, 'from baro (differenced)', C_VS),
    ('Pitch [deg]', pitch, pitch_c, 'acceleration-compensated', C_PITCH),
    ('G/S [m/s]', gs, None, None, C_GS),
]

# 区間の切り出し ---------------------------------------------
n_win = int(WINDOW_SEC * FS)
hop = int(HOP_SEC * FS)
slices = [slice(a0, a0 + n_win) for a0 in range(0, len(tg) - n_win + 1, hop)]

# 段ごとの縦軸の幅を決める -----------------------------------
# 区間ごとの振れ幅(最大-最小)の分位点を使う。最大値に合わせると、
# 一部の大きな区間のせいで他が潰れる
spans, absr = [], []
for lab, main, sub, _, _ in STRIPS:
    series = [main] + ([sub] if sub is not None else [])
    pp = [max(np.ptp(s[sl]) for s in series) for sl in slices]
    spans.append(float(np.percentile(pp, SPAN_PERCENTILE)) * SPAN_MARGIN)
    lo = min(float(np.min(s)) for s in series)
    hi = max(float(np.max(s)) for s in series)
    pad = (hi - lo) * 0.05
    absr.append((lo - pad, hi + pad))

print('=' * 66)
print(f'縦の運動の生データ  ({WINDOW_SEC:.0f}秒 × {len(slices)}区間, フィルタなし)')
print('=' * 66)
print(f"旋回の時刻: {daysec_to_hms(t_turn)}")
print(f"\n{'量':>14s} {'全体の範囲':>22s} {'区間内の振れ幅(中央値)':>22s} {'縦軸の幅':>10s}")
for (lab, main, sub, _, _), span, (lo, hi) in zip(STRIPS, spans, absr):
    pp = np.median([np.ptp(main[sl]) for sl in slices])
    print(f"{lab:>14s} {f'{np.min(main):.2f} 〜 {np.max(main):.2f}':>22s} "
          f"{pp:>21.3f} {span:>10.3f}")
if Y_MODE == 'span':
    print("\n縦軸: 幅は全ページ共通、中心は区間ごとの中央値")
    print("  → ドリフトを追いながら、振れ幅はページ間で比較できる")
    for (lab, main, sub, _, _), span in zip(STRIPS, spans):
        over = np.mean([np.ptp(main[sl]) > span for sl in slices]) * 100
        print(f"  {lab:>14s}: 幅 {span:.3f} をはみ出す区間 {over:.0f}%")
else:
    print("\n縦軸: 全ページ完全に同一の絶対範囲")

# 高度のドリフト(帯域制限では消えてしまう情報)
tt_h = (tg - tg[0]) / 3600.0
slope_alt = np.polyfit(tt_h, alt, 1)[0]
slope_pitch = np.polyfit(tt_h, pitch, 1)[0]
print(f"\nフライト全体の傾き(生データなので出る):")
print(f"  高度  {slope_alt:+.2f} m/時   ({alt[0]:.1f} m → {alt[-1]:.1f} m)")
print(f"  ピッチ {slope_pitch:+.2f} 度/時 ({pitch[:120].mean():.2f} → {pitch[-120:].mean():.2f} 度)")

# 作図 -------------------------------------------------------
avail = {f.name for f in fm.fontManager.ttflist}
JP = next((f for f in JP_FONT_CANDIDATES if f in avail), None)


def style(ax, ylab, last=False):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=1.0, zorder=0)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=9)
    ax.set_ylabel(ylab, color=INK_SUB, fontsize=10)
    if not last:
        ax.tick_params(labelbottom=False)


def draw_page(sl, title, sub_title, fmt, y_mode):
    """4段のストリップチャートを1ページ描く"""
    fig, axes = plt.subplots(len(STRIPS), 1, figsize=(11.7, 8.3),
                             facecolor=SURFACE, sharex=True)
    fig.subplots_adjust(top=0.86, bottom=0.19, left=0.085, right=0.975, hspace=0.14)
    tt = tg[sl]
    for k, ((lab, main, sub, sublab, color), ax) in enumerate(zip(STRIPS, axes)):
        if sub is not None:
            ax.plot(tt, sub[sl], color=SUB, linewidth=1.2, zorder=3)
        ax.plot(tt, main[sl], color=color, linewidth=1.4, zorder=4)
        if y_mode == 'span':
            # 中心は副系列も含めて決める。KF_Altitude のように系列間で
            # オフセットがある場合、主系列だけで中心を取ると副系列が軸外に出る
            vals = np.concatenate([main[sl]] + ([sub[sl]] if sub is not None else []))
            c = (float(np.max(vals)) + float(np.min(vals))) / 2
            ax.set_ylim(c - spans[k] / 2, c + spans[k] / 2)
        else:
            ax.set_ylim(*absr[k])
        ax.set_xlim(tt[0], tt[-1])
        if tt[0] <= t_turn <= tt[-1]:
            ax.axvline(t_turn, color=MARK, linewidth=1.2, linestyle='--', zorder=2)
        style(ax, lab, last=(k == len(STRIPS) - 1))
    axes[-1].xaxis.set_major_formatter(FuncFormatter(lambda v, _: fmt(v)))
    axes[-1].set_xlabel('Time of day', color=INK_SUB, fontsize=10)
    fig.suptitle(title, color=INK, fontsize=14, fontweight='bold', x=0.012,
                 ha='left', y=0.975)
    fig.text(0.012, 0.935, sub_title, color=INK_SUB, fontsize=10, va='top')
    handles = [Line2D([], [], color=c, lw=1.6, label=lab)
               for lab, _, _, _, c in STRIPS]
    handles += [Line2D([], [], color=SUB, lw=1.2,
                       label='grey = ' + ' / '.join(
                           s for _, _, sub, s, _ in STRIPS if sub is not None))]
    fig.legend(handles=handles, frameon=False, fontsize=8.5, labelcolor=INK_SUB,
               loc='upper left', ncol=5, bbox_to_anchor=(0.012, 0.915))
    return fig


with PdfPages(PDFNAME) as pdf:
    # --- 1ページ目: フライト全体 ---------------------------
    fig = draw_page(slice(0, len(tg)),
                    'Vertical motion — raw log, whole flight  (2026)',
                    f"{daysec_to_hms(tg[0])} - {daysec_to_hms(tg[-1])}   "
                    f"({(tg[-1] - tg[0]) / 60:.0f} min, no filtering)",
                    daysec_to_hm, 'absolute')
    if JP:
        fig.text(0.085, 0.115,
                 "フィルタを一切かけていないので、ゆっくりした変化もそのまま出る。\n"
                 f"このフライトでは高度が {slope_alt:+.1f} m/時、"
                 f"ピッチが {slope_pitch:+.2f} 度/時 の傾きを持っている。\n"
                 "帯域制限をかけた phugoid 系のツールでは、こうした遅い成分は定義上すべて消える。",
                 fontname=JP, fontsize=9.5, color=INK_SUB, va='top', linespacing=1.7)
    pdf.savefig(fig, facecolor=SURFACE)
    plt.close(fig)

    # --- 2ページ目以降: 区間ごとに拡大 ---------------------
    for i, sl in enumerate(slices, start=1):
        fig = draw_page(
            sl,
            f"{daysec_to_hms(tg[sl][0])} - {daysec_to_hms(tg[sl][-1])}"
            f"   (window {i} of {len(slices)})",
            '   '.join(f"{lab.split(' [')[0]} {np.mean(main[sl]):.2f} "
                       f"(range {np.ptp(main[sl]):.2f})"
                       for lab, main, _, _, _ in STRIPS),
            daysec_to_hms, Y_MODE)
        if JP:
            fig.text(0.085, 0.10,
                     "縦軸の幅は全ページ共通なので、振れ幅の大小はページをまたいで比較できる。\n"
                     "中心は区間ごとの中央値に合わせてあるため、絶対値は目盛りを読むこと。",
                     fontname=JP, fontsize=9.5, color=INK_SUB, va='top', linespacing=1.7)
        pdf.savefig(fig, facecolor=SURFACE)
        plt.close(fig)

print(f"\nPDFを保存: {PDFNAME}  ({1 + len(slices)}ページ)")
if JP is None:
    print('警告: 日本語フォントが見つからないため、日本語の説明は省略しました')
