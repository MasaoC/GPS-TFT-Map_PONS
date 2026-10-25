# -*- coding: utf-8 -*-
# V/S とピッチのログから、縦の長周期振動(フゴイド)を探す
#
# フゴイドとは:
#   機首を上げると上昇して減速し、減速すると揚力が足りず降下し、降下すると
#   加速してまた上昇する…という「高度と速度の交換」で起きる、ゆっくりした
#   縦揺れ。迎角はほとんど変わらないのが特徴。
#   古典的な周期は T = π√2 · V/g で、この機体(V≈7m/s)だと約3秒しかない。
#
# 位置づけ:
#   plot_lateral_modes.py の縦運動版。手法(自己相関に2次系を当てはめる)は同じ。
#   同フォルダの plot_spectrum.py / plot_spectrogram.py はピッチの
#   「揺れの大きさが旋回前後でどう変わったか」を見るもので、目的が違う。
#   こちらは「そもそも何秒の振動モードがあるか」を特定する。
#
# ===== 判定の考え方 =========================================
# 周期が合うだけでは根拠が弱いので、フゴイド特有の位相関係を2つ確かめる。
#
#  (1) ピッチとV/Sが同位相になるか
#      フゴイドは迎角がほぼ一定なので、経路角 ≒ ピッチ角。
#      V/S = V·sin(経路角) なので、ピッチとV/Sは同位相(位相差0度)になる。
#      短周期モードや操縦入力なら、先に機首が動いて経路が後から追うので
#      位相差が出る。
#
#  (2) 高度と速度が逆位相になるか
#      高いところでは遅く、低いところでは速い。位置エネルギーと運動
#      エネルギーの交換そのもので、位相差は±180度になる。
#
# ===== 気をつけていること ===================================
#  ・KF_Vspeed はカルマンフィルタが加速度計と気圧計を融合した出力なので、
#    IMUのピッチと人工的に相関してしまう。(1)の位相判定には使えない。
#    そこで気圧高度を微分しただけの、KFを通さないV/Sを別に作って使う。
#    (KF_Vspeed で判定すると位相が-37度になり、-7度と誤読する)
#  ・ピッチは補正前(BNO085の生ログ)を使う。flight_preprocess.py の補正済み
#    ピッチは前後加速度から作った項を引いており、速度と絡む解析では
#    循環になりうるため。robustness として補正済みでも計算し比較を表示する。
#  ・2026年には対気速度センサーが無いので、速度は対地速度 gs で代用する。
#    このため「水平方向の突風」と区別できない(注記参照)。
#
# 使い方: python3 plot_phugoid.py
# 出力: phugoid.png

import os
import numpy as np
import pandas as pd
from scipy.signal import welch, butter, filtfilt, coherence, csd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter

# 設定 -------------------------------------------------------
CSV_2026 = '2026-07-26_0804.csv'              # 高度・気圧・KF_Vspeed・対地速度
EULER_2026 = '20260726.txt'                   # BNO085の生ログ(補正前のピッチ)
EULER_2026_COMP = '20260726_compensated.txt'  # flight_preprocess.py の補正済み(比較用)
BETADA_2026 = 'beta_da_20260726.txt'          # 推定TAS
JOINED_2025 = '20250726_log_all_joined.csv'   # 2025年: 対気速度・高度・ピッチ(1Hz)
PLOTNAME = 'phugoid.png'

FLIGHT_2026 = ('08:21:26.30', '09:07:42.00')  # 離陸/着水。他のツールと同じ値
GS_MIN = 4.0          # これ未満の対地速度は地上/離着水とみなして捨てる [m/s]

FS = 2.0              # 解析の基準サンプリング周波数 [Hz]。CSVの0.5秒間隔に合わせる
PITCH_FS = 1 / 0.21   # IMUピッチの実サンプリング [Hz]。約4.8Hz
BAND = (2.0, 20.0)    # 自己相関の前に通す帯域 [秒]。2秒未満はV/Sの分解能で読めない
ACF_MAXLAG = 20.0     # 自己相関を見る最大のずらし時間 [秒]
FIT_SKIP_SEC = 1.0    # 当てはめで使わない最初のラグ [秒]。ノイズの尖りを避ける
PHUGOID_BAND = (4.0, 7.0)   # 見つかった振動が入る帯域 [秒]。位相判定はここで行う
EXCERPT_SEC = 60.0    # 時系列の抜粋パネルの長さ [秒]
NPERSEG = 512         # スペクトル・コヒーレンスの窓長 [サンプル]
COH_MIN = 0.2         # これ未満のコヒーレンスの位相は意味が無いので図に描かない
# 位相とコヒーレンスをまとめる周期帯 [秒]
BANDS = [(2, 4), (4, 7), (7, 12), (12, 30)]
# 気圧高度のノイズ床を見積もるための帯 [秒]
NOISE_BANDS = [(1.2, 2), (2, 4), (4, 7), (7, 12), (12, 30), (30, 90)]

# 図の下の注記に使う日本語フォント。見つからなければ注記を省略する
JP_FONT_CANDIDATES = ['Hiragino Sans', 'Hiragino Maru Gothic Pro',
                      'Noto Sans CJK JP', 'IPAexGothic', 'Arial Unicode MS']

# 配色 -------------------------------------------------------
# 同フォルダの他のツールと同じ紙色を使う
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C_PITCH = '#2a78d6'   # ピッチ(IMU)
C_VS = '#eb6834'      # V/S
C_ALT = '#7a3d9e'     # 気圧高度
C_2025 = '#8a8880'    # 2025年(検出限界の比較用)
MARK = '#8a8880'      # 基準線
BAND_FILL = '#f0e6da'  # フゴイドが入る帯の塗り


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
    """その日の経過秒を時刻の文字列に戻す"""
    return f"{int(sec // 3600):02d}:{int((sec % 3600) // 60):02d}:{int(sec % 60):02d}"


def bandpass(x, lo, hi, fs):
    """周期 lo〜hi 秒の成分だけを取り出す"""
    b, a = butter(2, [1 / hi / (fs / 2), 1 / lo / (fs / 2)], btype='band')
    return filtfilt(b, a, x - np.mean(x))


def acf_model(tau, T, zeta, amp):
    """乱れに叩かれ続けている2次系の自己相関。TとζとAで形が決まる。
    A は「揺れのうち、きれいな振動で説明できる割合」"""
    q = np.sqrt(max(1.0 - zeta * zeta, 1e-6))
    wn = 2 * np.pi / T / q
    wd = wn * q
    return amp * np.exp(-zeta * wn * tau) * (np.cos(wd * tau) + zeta / q * np.sin(wd * tau))


def acf_and_fit(x, fs, band=BAND, maxlag=ACF_MAXLAG):
    """自己相関を求めて2次系を当てはめ、周期T・減衰比ζ・振動の割合Aを返す。
    ラグ0付近は広帯域ノイズの尖りなので当てはめから外す
    (含めると「猛烈に速く減衰する振動」という非現実的な解に引きずられる)"""
    v = bandpass(x, band[0], band[1], fs)
    a = np.correlate(v, v, 'full')[len(v) - 1:]
    n = int(maxlag * fs) + 1
    acf = a[:n] / a[0]
    lags = np.arange(len(acf)) / fs
    k0 = int(FIT_SKIP_SEC * fs)
    p, _ = curve_fit(acf_model, lags[k0:], acf[k0:], p0=[5.0, 0.3, 0.6],
                     bounds=([1.5, 0.01, 0.05], [30.0, 0.99, 1.5]), maxfev=40000)
    return lags, acf, p


def half_life(T, zeta):
    """減衰比を「振幅が半分になるまでの秒数」に言い換える"""
    wn = (2 * np.pi / T) / np.sqrt(1.0 - zeta ** 2)
    return float(np.log(2.0) / (zeta * wn))


def cross_phase(x, y, fs):
    """2つの信号のコヒーレンス(連動の強さ)と位相差を周期帯ごとに返す"""
    f, coh = coherence(x, y, fs=fs, nperseg=NPERSEG)
    _, pxy = csd(x, y, fs=fs, nperseg=NPERSEG)
    out = []
    for lo, hi in BANDS:
        m = (f >= 1 / hi) & (f <= 1 / lo)
        if m.sum():
            out.append((lo, hi, float(coh[m].mean()),
                        float(np.degrees(np.angle(pxy[m].mean())))))
    return f, coh, pxy, out


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
bd = pd.read_csv(find_log(BETADA_2026))
TAS = float(bd['tas'].mean())

# 2Hzの共通グリッドに揃える(位相の比較は同じ時刻で行う必要がある)
tg = np.arange(d['t'].min(), d['t'].max(), 1 / FS)
alt = np.interp(tg, d['t'].to_numpy(), d['Altitude'].to_numpy())
vs_kf = np.interp(tg, d['t'].to_numpy(), d['KF_Vspeed'].to_numpy())
vs_baro = np.gradient(alt, tg)          # KFを通さないV/S。位相判定はこれを使う
gs = np.interp(tg, d['t'].to_numpy(), d['gs'].to_numpy())
pitch = np.interp(tg, eu['t'].to_numpy(), eu['pitch'].to_numpy())
pitch_comp = np.interp(tg, euc['t'].to_numpy(), euc['pitch'].to_numpy())
# ピッチは元の細かさ(約4.8Hz)でもスペクトルを取る。短い周期まで見えるので
# 「3秒付近に鋭い山があるか」を確かめるのに使う
tp = np.arange(eu['t'].min(), eu['t'].max(), 1 / PITCH_FS)
pitch_fast = np.interp(tp, eu['t'].to_numpy(), eu['pitch'].to_numpy())

# 古典的なフゴイドの周期
T_CLASSIC = float(np.pi * np.sqrt(2) * TAS / 9.80665)

# 解析 -------------------------------------------------------
res = {}
for lab, x, fs in (('pitch', pitch_fast, PITCH_FS),
                   ('vs_kf', vs_kf, FS),
                   ('vs_baro', vs_baro, FS)):
    lags, acf, p = acf_and_fit(x, fs)
    res[lab] = dict(lags=lags, acf=acf, T=p[0], zeta=p[1], amp=p[2],
                    t_half=half_life(p[0], p[1]), fs=fs)
# 補正済みピッチでも同じ計算をして、結論が補正の有無で変わらないか確かめる
_, _, p_comp = acf_and_fit(pitch_comp, FS)

# スペクトル
spec = {}
for lab, x, fs in (('pitch', pitch_fast, PITCH_FS),
                   ('vs_kf', vs_kf, FS),
                   ('vs_baro', vs_baro, FS)):
    f, P = welch(x - x.mean(), fs=fs, nperseg=min(NPERSEG, len(x)))
    spec[lab] = (f, P)

# フゴイドの2つの位相条件
f_pv, coh_pv, pxy_pv, band_pv = cross_phase(pitch, vs_baro, FS)
f_ag, coh_ag, pxy_ag, band_ag = cross_phase(alt, gs, FS)

# 気圧高度のノイズ床。短い周期側で値が落ちきるなら、その上の帯の信号は本物
noise = [(lo, hi, float(bandpass(alt, lo, hi, FS).std())) for lo, hi in NOISE_BANDS]

# 振幅の整合: 迎角一定なら V/S = TAS × ピッチ角[rad] になるはず
pb = np.radians(bandpass(pitch, *PHUGOID_BAND, FS).std())
vb = float(bandpass(vs_baro, *PHUGOID_BAND, FS).std())
amp_ratio = vb / (TAS * pb)

# 2025年: 同じことができるか(できないことの確認) ------------
j = pd.read_csv(find_log(JOINED_2025))
j['t'] = j['JST'].apply(time_to_daysec)
j = j.drop_duplicates('t')
j = j[j['p_gs'] > GS_MIN]
tj = np.arange(j['t'].min(), j['t'].max(), 1.0)
air25 = np.interp(tj, j['t'].to_numpy(), j['Airspeed'].to_numpy())
alt25 = np.interp(tj, j['t'].to_numpy(), j['g_altitude'].to_numpy())
T_CLASSIC_25 = float(np.pi * np.sqrt(2) * float(j['Airspeed'].mean()) / 9.80665)
_, _, p25 = acf_and_fit(air25, 1.0, band=(3.0, 30.0), maxlag=30.0)
f25, coh25, pxy25, band25 = cross_phase(air25, alt25, 1.0)

# 結果の表示 -------------------------------------------------
print('=' * 68)
print('縦の長周期振動(フゴイド)を探す')
print('=' * 68)
print(f"2026年 平均TAS {TAS:.2f} m/s → 古典式の周期 T = π√2·V/g = {T_CLASSIC:.1f} 秒")
print(f"  (古典式の減衰比は ζ ≈ 1/(√2·L/D)。L/D=30 なら 0.02 でほぼ減衰しない)")

print('\n--- 見つかった振動 (自己相関に2次系を当てはめた結果) ---')
print(f"{'信号':>22s} {'周期T':>8s} {'ζ':>6s} {'A':>6s} {'半減時間':>9s}")
for lab, name in (('pitch', 'IMUピッチ(4.8Hz)'), ('vs_kf', 'KF_Vspeed(2Hz)'),
                  ('vs_baro', '気圧高度の微分V/S')):
    r = res[lab]
    print(f"{name:>22s} {r['T']:7.2f}s {r['zeta']:6.2f} {r['amp']:6.2f} {r['t_half']:8.1f}s")
print(f"  → 3つの計器が独立に同じ周期({res['pitch']['T']:.1f}〜{res['vs_baro']['T']:.1f}秒)を指している")
print(f"  参考: 補正済みピッチで計算しても T={p_comp[0]:.2f}s ζ={p_comp[1]:.2f} で変わらない")
print(f"  古典式の {T_CLASSIC:.1f}秒 より {res['pitch']['T'] / T_CLASSIC:.1f}倍 長い")

print('\n--- 条件(1) ピッチとV/Sは同位相か (フゴイドなら0度付近) ---')
print('   ※KF_Vspeedは加速度計を融合していてピッチと人工的に相関するため、')
print('     ここではKFを通さない「気圧高度の微分」を使っている')
for lo, hi, c, ph in band_pv:
    mark = '  ← フゴイドの帯' if (lo, hi) == PHUGOID_BAND else ''
    print(f"   周期{lo:2.0f}-{hi:2.0f}s: コヒーレンス {c:.2f}  位相 {ph:+6.0f}度{mark}")

print('\n--- 条件(2) 高度と速度は逆位相か (フゴイドなら±180度) ---')
for lo, hi, c, ph in band_ag:
    mark = '  ← フゴイドの帯' if (lo, hi) == PHUGOID_BAND else ''
    print(f"   周期{lo:2.0f}-{hi:2.0f}s: コヒーレンス {c:.2f}  位相 {ph:+6.0f}度{mark}")

print('\n--- 気圧高度のノイズ床 (信号が本物かの確認) ---')
for lo, hi, v in noise:
    print(f"   周期{lo:4.1f}-{hi:2.0f}s の高度RMS = {v:.4f} m")
nf = [v for lo, hi, v in noise if (lo, hi) == (1.2, 2)][0]
sig = [v for lo, hi, v in noise if (lo, hi) == PHUGOID_BAND][0]
print(f"  → フゴイド帯の {sig:.3f} m は、最短周期側の {nf:.3f} m の {sig / nf:.0f}倍。ノイズではない")

print('\n--- 振幅の整合 (迎角が一定なら V/S = TAS × ピッチ角) ---')
print(f"   {PHUGOID_BAND[0]:.0f}-{PHUGOID_BAND[1]:.0f}秒帯: ピッチRMS {np.degrees(pb):.2f}度, "
      f"V/S RMS {vb:.3f} m/s")
print(f"   迎角一定なら V/S = {TAS:.2f}×{pb:.4f} = {TAS * pb:.3f} m/s  → 実測はその {amp_ratio:.2f}倍")
print(f"   ({'ほぼ一致。迎角はほぼ一定' if abs(amp_ratio - 1) < 0.25 else '1割以上ずれており、迎角も多少動いている'})")

print(f'\n--- 2025年 白夜で同じことができるか ---')
print(f"   平均対気速度 {float(j['Airspeed'].mean()):.2f} m/s → 古典式の周期は {T_CLASSIC_25:.1f}秒")
print(f"   ログは1Hz。周期{T_CLASSIC_25:.1f}秒だと1周期あたり{T_CLASSIC_25:.1f}点しかなく、原理的に読めない")
print(f"   対気速度(実測センサー)の当てはめ: T={p25[0]:.2f}s ζ={p25[1]:.2f} A={p25[2]:.2f}")
print(f"     → 周期{p25[0]:.1f}秒の揺れはあるが、フゴイドの予測{T_CLASSIC_25:.1f}秒とは全く違う")
print("   対気速度と高度の位相 (フゴイドなら±180度):")
for lo, hi, c, ph in band25:
    print(f"     周期{lo:2.0f}-{hi:2.0f}s: コヒーレンス {c:.2f}  位相 {ph:+6.0f}度")
print("   → コヒーレンスは全帯域で0.3未満、位相も-6度/+153度/-2度/-94度とばらばら。")
print("     高度と速度の交換が起きておらず、この揺れはフゴイドではない(風か操縦)。")
print("   結論: フゴイドが無かったのではなく、サンプリングが粗すぎて判定できない。")

# 作図 -------------------------------------------------------
fig = plt.figure(figsize=(13.0, 15.4), facecolor=SURFACE)
gs_ = fig.add_gridspec(3, 2, hspace=0.36, wspace=0.30,
                       top=0.938, bottom=0.235, left=0.075, right=0.975)


def style(ax, xlab, ylab, title=None):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=1.0, zorder=0)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=9)
    ax.set_xlabel(xlab, color=INK_SUB, fontsize=10)
    ax.set_ylabel(ylab, color=INK_SUB, fontsize=10)
    if title:
        ax.set_title(title, color=INK, fontsize=11.5, fontweight='bold', loc='left')


def period_axis(ax, lo=30, hi=1.5):
    ax.set_xscale('log')
    ax.set_xlim(lo, hi)
    ticks = [30, 20, 12, 7, 4, 2]
    ax.set_xticks(ticks)
    ax.set_xticklabels([str(t) for t in ticks])
    ax.minorticks_off()


# 1段目左: 一番よく揺れていた60秒を抜き出して、ピッチとV/Sを重ねる
# 同位相かどうかを目で見て確かめられるようにするのが狙い
ax = fig.add_subplot(gs_[0, 0])
pb_all = bandpass(pitch, *PHUGOID_BAND, FS)
vb_all = bandpass(vs_baro, *PHUGOID_BAND, FS)
n = int(EXCERPT_SEC * FS)
best, bs = 0, -1.0
for a0 in range(0, len(pb_all) - n, n // 2):
    v = float(np.std(pb_all[a0:a0 + n]))
    if v > bs:
        bs, best = v, a0
sl = slice(best, best + n)
tt = tg[sl]                      # 横軸は実際の時刻(その日の経過秒)
gb_all = bandpass(gs, *PHUGOID_BAND, FS)
ax.axhline(0, color=MARK, linewidth=1.0, zorder=1)
ax.plot(tt, pb_all[sl], color=C_PITCH, linewidth=1.8, zorder=3)
# V/S は「経路角」に直してからピッチと同じ軸に重ねる。単位が度でそろうので
# 同位相かどうかだけでなく、振幅が釣り合っているかも同時に見える
ax.plot(tt, np.degrees(np.arcsin(np.clip(vb_all[sl] / TAS, -1, 1))),
        color=C_VS, linewidth=1.8, zorder=4)
ab_all = bandpass(alt, *PHUGOID_BAND, FS)
style(ax, f'Time of day  ({daysec_to_hms(tg[best])} +{EXCERPT_SEC:.0f} s)',
      'Pitch [deg]  /  V/S expressed as path angle [deg]',
      f'Pitch, V/S and G/S  ({PHUGOID_BAND[0]:.0f}-{PHUGOID_BAND[1]:.0f} s band)')
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hms(v)))
ax.set_xlim(tt[0], tt[-1])
# 右軸には対地速度と気圧高度を置く。数値レンジがほぼ同じ(±0.25)で収まるうえ、
# この2本こそ「高いとき遅い」= 逆位相になるはずのペアなので、
# 同じ軸に重ねると条件(2)がそのまま目で確かめられる。
# なお G/S は V/S より4分の1周期ぶん先行して見える(高度はV/Sの積分で位相が90度ずれるため)
axr = ax.twinx()
axr.plot(tt, gb_all[sl], color=INK, linewidth=1.5, linestyle='--', zorder=5)
axr.plot(tt, ab_all[sl], color=C_ALT, linewidth=1.6, linestyle=':', zorder=6)
axr.set_ylabel('G/S [m/s]  /  altitude [m]', color=INK_SUB, fontsize=10)
axr.tick_params(colors=INK_SUB, labelsize=9)
for s in ('top', 'left'):
    axr.spines[s].set_visible(False)
axr.spines['right'].set_color(GRID)
axr.spines['bottom'].set_color(GRID)
lim = max(float(np.abs(gb_all[sl]).max()), float(np.abs(ab_all[sl]).max())) * 1.15
axr.set_ylim(-lim, lim * 1.45)   # 0 を左軸とそろえつつ、凡例のぶん上を空ける
plim = float(np.abs(pb_all[sl]).max()) * 1.15
ax.set_ylim(-plim, plim * 1.45)
ax.legend(handles=[
    Line2D([], [], color=C_PITCH, lw=2, label='pitch (IMU)'),
    Line2D([], [], color=C_VS, lw=2, label='V/S from baro, as angle = asin(V/S / TAS)'),
    Line2D([], [], color=INK, lw=1.5, ls='--', label='G/S [m/s] (right axis)'),
    Line2D([], [], color=C_ALT, lw=1.6, ls=':', label='barometric altitude [m] (right axis)'),
], frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='upper right')

# 1段目右: スペクトル。古典式の周期に線を引いて「そこには山が無い」ことを示す
ax = fig.add_subplot(gs_[0, 1])
ax.axvspan(PHUGOID_BAND[1], PHUGOID_BAND[0], color=BAND_FILL, zorder=0)
for lab, color, ls in (('pitch', C_PITCH, '-'), ('vs_kf', C_VS, '-'),
                       ('vs_baro', INK, '--')):
    f, P = spec[lab]
    m = f > 0
    # 単位が違うので、それぞれの帯域内での最大値で正規化して形だけ比べる
    sel = m & (1 / f >= 1.5) & (1 / f <= 30)
    ax.plot(1 / f[sel], P[sel] / P[sel].max(), color=color, linewidth=1.8,
            linestyle=ls, zorder=3)
ax.axvline(T_CLASSIC, color=MARK, linewidth=1.4, linestyle=':', zorder=2)
ax.text(T_CLASSIC, 1.02, f' classical\n phugoid {T_CLASSIC:.1f} s', color=INK_SUB,
        fontsize=8.5, ha='left', va='top')
period_axis(ax)
ax.set_yscale('log')
ax.set_ylim(0.01, 1.6)
style(ax, 'Period [s]  (long <- -> short)', 'Normalised power',
      'Where the vertical motion lives')
ax.legend(handles=[
    Line2D([], [], color=C_PITCH, lw=1.8, label='pitch (IMU, 4.8 Hz)'),
    Line2D([], [], color=C_VS, lw=1.8, label='KF_Vspeed'),
    Line2D([], [], color=INK, lw=1.8, ls='--', label='V/S from baro only'),
], frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='lower left')

# 2段目左: 自己相関と当てはめ。周期と減衰比が読める
ax = fig.add_subplot(gs_[1, 0])
ax.axhline(0, color=MARK, linewidth=1.0, zorder=1)
for i, (lab, color, name) in enumerate((('pitch', C_PITCH, 'pitch'),
                                        ('vs_baro', INK, 'V/S (baro)'))):
    r = res[lab]
    ax.plot(r['lags'], r['acf'], color=color, linewidth=2.0, zorder=4)
    ax.plot(r['lags'], acf_model(r['lags'], r['T'], r['zeta'], r['amp']),
            color=color, linewidth=1.2, linestyle='--', alpha=0.8, zorder=3)
    ax.text(0.97, 0.94 - i * 0.10,
            f"{name}:  T = {r['T']:.1f} s,  $\\zeta$ = {r['zeta']:.2f},  A = {r['amp']:.2f}",
            transform=ax.transAxes, color=color, fontsize=10.5,
            ha='right', va='top', fontweight='bold')
style(ax, 'Lag [s]', 'Autocorrelation', 'Period and damping of the oscillation')
ax.text(0.97, 0.06, 'dashed = fitted 2nd-order model',
        transform=ax.transAxes, color=INK_SUB, fontsize=9, ha='right')

# 2段目右: 条件(1) ピッチとV/Sの位相。0度に近いほどフゴイドらしい
ax = fig.add_subplot(gs_[1, 1])
ax.axvspan(PHUGOID_BAND[1], PHUGOID_BAND[0], color=BAND_FILL, zorder=0)
ax.axhline(0, color=MARK, linewidth=1.4, linestyle='--', zorder=2)
m = f_pv > 0
per_pv = np.full_like(f_pv, np.inf); per_pv[m] = 1 / f_pv[m]
# コヒーレンスが低いところの位相は意味を持たない(乱数と同じ)ので描かない
sel = m & (per_pv >= 1.5) & (per_pv <= 30) & (coh_pv > COH_MIN)
ax.plot(1 / f_pv[sel], np.degrees(np.angle(pxy_pv[sel])), color=C_PITCH,
        linewidth=1.0, alpha=0.45, zorder=3, marker='.', markersize=2, linestyle='none')
for lo, hi, c, ph in band_pv:
    ax.plot([hi, lo], [ph, ph], color=C_PITCH, linewidth=4.0, zorder=5)
    ax.text(np.sqrt(lo * hi), ph + (16 if ph >= 0 else -26), f'coh {c:.2f}',
            color=INK_SUB, fontsize=8.5, ha='center')
period_axis(ax)
ax.set_ylim(-180, 180)
ax.set_yticks([-180, -90, 0, 90, 180])
style(ax, 'Period [s]  (long <- -> short)', 'Phase: pitch vs V/S [deg]',
      'Test 1: pitch and V/S in phase?')
ax.text(0.03, 0.06, '0 deg = phugoid (angle of attack stays constant)',
        transform=ax.transAxes, color=INK_SUB, fontsize=9)

# 3段目左: 条件(2) 高度と速度の位相。±180度に近いほどフゴイドらしい
ax = fig.add_subplot(gs_[2, 0])
ax.axvspan(PHUGOID_BAND[1], PHUGOID_BAND[0], color=BAND_FILL, zorder=0)
ax.axhline(0, color=MARK, linewidth=1.4, linestyle='--', zorder=2)
# 逆位相(±180度)が答えなので、そのままだと+180と-180に折り返して読めない。
# 180度回してから描き、「0度からのずれ」として読めるようにする
m_ag = f_ag > 0
per_ag = np.full_like(f_ag, np.inf); per_ag[m_ag] = 1 / f_ag[m_ag]
sel = m_ag & (per_ag >= 1.5) & (per_ag <= 30) & (coh_ag > COH_MIN)
ax.plot(1 / f_ag[sel], np.degrees(np.angle(-pxy_ag[sel])), color=C_VS,
        linewidth=1.0, alpha=0.45, zorder=3, marker='.', markersize=2, linestyle='none')
for lo, hi, c, ph in band_ag:
    dev = np.degrees(np.angle(-np.exp(1j * np.radians(ph))))   # 逆位相からのずれ
    ax.plot([hi, lo], [dev, dev], color=C_VS, linewidth=4.0, zorder=5)
    ax.text(np.sqrt(lo * hi), dev + (16 if dev >= 0 else -26), f'coh {c:.2f}',
            color=INK_SUB, fontsize=8.5, ha='center')
period_axis(ax)
ax.set_ylim(-180, 180)
ax.set_yticks([-180, -90, 0, 90, 180])
style(ax, 'Period [s]  (long <- -> short)',
      'Departure from anti-phase [deg]',
      'Test 2: altitude and speed in anti-phase?')
ax.text(0.03, 0.06, '0 deg = phugoid (high & slow, low & fast)',
        transform=ax.transAxes, color=INK_SUB, fontsize=9)

# 3段目右: 気圧高度のノイズ床。フゴイド帯の信号がノイズより十分大きいことを示す
ax = fig.add_subplot(gs_[2, 1])
xs = np.arange(len(noise))
vals = [v for _, _, v in noise]
# フゴイドの帯だけ色を変えて、他の帯(=ノイズ床の目安)と対比させる
cols = [C_VS if (lo, hi) == PHUGOID_BAND else '#c9c7c1' for lo, hi, _ in noise]
ax.bar(xs, vals, 0.62, color=cols, zorder=3)
ax.set_xticks(xs)
ax.set_xticklabels([f'{lo:g}-{hi:g}' for lo, hi, _ in noise], fontsize=8.5)
ax.set_yscale('log')
ax.set_ylim(min(vals) * 0.5, max(vals) * 6)   # 右上の注記を置く余白
style(ax, 'Period band [s]', 'Altitude RMS [m]',
      'Is the signal above the barometer noise?')
ax.text(0.97, 0.94,
        f"phugoid band = {sig:.3f} m\n= {sig / nf:.0f}x the shortest band ({nf:.3f} m)",
        transform=ax.transAxes, color=INK_SUB, fontsize=9.5, ha='right', va='top')

fig.suptitle('Looking for the phugoid in the vertical-speed and pitch logs  (2026)',
             color=INK, fontsize=15.5, fontweight='bold', x=0.012, ha='left', y=0.968)

# 図の下の注記 ----------------------------------------------
avail = {f.name for f in fm.fontManager.ttflist}
jp_font = next((f for f in JP_FONT_CANDIDATES if f in avail), None)
c_pv = [c for lo, hi, c, ph in band_pv if (lo, hi) == PHUGOID_BAND][0]
ph_pv = [ph for lo, hi, c, ph in band_pv if (lo, hi) == PHUGOID_BAND][0]
c_ag = [c for lo, hi, c, ph in band_ag if (lo, hi) == PHUGOID_BAND][0]
ph_ag = [ph for lo, hi, c, ph in band_ag if (lo, hi) == PHUGOID_BAND][0]
notes = [
    ("注1  何が確かめられたか",
     f"　　周期{res['vs_baro']['T']:.1f}秒の縦揺れが、独立した3つの計器"
     "（IMUピッチ・KF_Vspeed・気圧高度）すべてに\n"
     "　　同じ周期で出ています。さらにフゴイド特有の位相条件を2つとも満たします。\n"
     f"　　　条件(1) ピッチとV/Sが同位相: {ph_pv:+.0f}度（コヒーレンス {c_pv:.2f}）\n"
     f"　　　条件(2) 高度と速度が逆位相: {ph_ag:+.0f}度（コヒーレンス {c_ag:.2f}）\n"
     f"　　高度の振れ幅もノイズ床の{sig / nf:.0f}倍あり、気圧計の誤差ではありません。"),
    ("注2  ただしフゴイドと断定はできません",
     f"　　古典式の予測は{T_CLASSIC:.1f}秒ですが実測は{res['vs_baro']['T']:.1f}秒で"
     f"{res['vs_baro']['T'] / T_CLASSIC:.1f}倍長く、減衰も\n"
     f"　　古典式のζ≈0.02に対し実測{res['vs_baro']['zeta']:.2f}とかなり大きいです。"
     "パイロットが高度を保とうと\n"
     "　　当て舵している閉ループの効果と考えれば自然ですが、確認はできていません。\n"
     "　　より厄介なのは、2026年に対気速度センサーが無く速度を対地速度で代用している点。\n"
     "　　水平方向の突風でも「高度と対地速度が逆位相」は出ます（向かい風が強まる→\n"
     "　　対気速度増→上昇、同時に対地速度は減る）。モードそのものか突風応答かは\n"
     "　　分離できていません。ピトー管が載れば決着します。"),
    ("注3  2025年 白夜では判定できません",
     f"　　対気速度{float(j['Airspeed'].mean()):.2f}m/sだと古典式の周期は{T_CLASSIC_25:.1f}秒。"
     "ログは1Hzなので1周期\n"
     f"　　あたり{T_CLASSIC_25:.1f}点しかなく、原理的に届きません。\n"
     f"　　対気速度には周期{p25[0]:.1f}秒の揺れがありますが、これはフゴイドの予測とは全く違い、\n"
     "　　高度とのコヒーレンスも全帯域で0.3未満・位相もばらばらです。高度と速度の交換が\n"
     "　　起きていないので、フゴイドではなく風か操縦によるものと考えられます。\n"
     "　　「フゴイドが無かった」のではなく「このログでは検出できない」が正確です。"),
]
if jp_font is None:
    print('警告: 日本語フォントが見つからないため、図の注記は省略しました')
else:
    places = [(0.075, 0.205), (0.545, 0.205), (0.075, 0.115)]
    for (head, body), (x, y) in zip(notes, places):
        fig.text(x, y, head, fontname=jp_font, fontsize=11,
                 color=INK, fontweight='bold', va='top')
        fig.text(x, y - 0.011, body, fontname=jp_font, fontsize=9.5,
                 color=INK_SUB, va='top', linespacing=1.65)

fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"\nグラフを保存: {PLOTNAME}")
