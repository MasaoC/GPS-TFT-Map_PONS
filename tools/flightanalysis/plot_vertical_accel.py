# -*- coding: utf-8 -*-
# ============================================================
# File    : plot_vertical_accel.py
# Project : PONS v7 (Pilot Oriented Navigation System for HPA)
# Role    : フライトログから鉛直加速度（G）と荷重倍数の最大値を推定する。
#           G は直接記録していないので、記録されている量から導出する。
#           導出には必ずノイズと帯域の代償がつくので、その代償の大きさを
#           数値で出すところまでを仕事とする。
#
# 使い方  : python3 plot_vertical_accel.py
# 出力    : vertical_accel.pdf  ＋ 標準出力のレポート
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/09/18
# ============================================================
#
# ===== どの量から G を出すか ==================================
# ログにある鉛直方向の量は 3 つ。どれも「Gそのもの」ではない。
#
#   (a) pressure      : MS5611 の生の気圧。2Hz。0.01hPa 刻み（＝8.5cm 刻み）
#   (b) KF_Altitude   : バリオ KF の高度推定。2Hz
#   (c) KF_Vspeed     : バリオ KF の昇降率推定。2Hz
#
# 素直に思いつくのは (a) か (b) の 2 階微分だが、これは筋が悪い。
# 位置を 2 回微分すると、ノイズは 1/T² で増える（T=0.5秒なら 4 倍）。
# 実際、(a) の 2 階微分は地上静止中の rms が 0.55〜0.70 m/s² あり、
# 飛行中の rms 0.47 m/s² より大きい。つまり S/N が 1 を下回っていて、
# 何を測っているのか分からない。0.01hPa の記録刻みだけでも
# √6 × (0.085/√12) / 0.25 = 0.24 m/s² のノイズが立つ。
#
# 正解は (c) の 1 階微分。理由は KF の中身にある。
#
#   大会時ファーム 0.917 のバリオ KF:
#     predict : BNO085 の LINEAR_ACCELERATION（重力除去済み）を地球座標系の
#               鉛直成分に直し、15Hz で x[1] += (a - bias)·dt と積分
#     update  : MS5611 の気圧高度（250ms トリム平均）で 4Hz 観測更新。R=12m²
#
# つまり KF_Vspeed は「鉛直加速度を積分したもの」であって、位置の微分ではない。
# 積分値を差分すると、その区間の加速度の**厳密な平均値**が戻ってくる。
#
#     (v[k+1] - v[k]) / T  =  区間 T の間の a_z の平均
#
# これは微分ではなく積分の逆操作なので、
#   ・ノイズが 1/T で増える経路が無い（位置の 2 階微分と決定的に違う）
#   ・区間平均＝箱型フィルタが掛かった形なので、折り返しが構造的に起きない
# という 2 つの利点がある。代償は箱型フィルタによる高域の減衰だけで、
# これは sinc(fT) として計算できる既知量である。
#
# ===== KF は真の加速度をどれだけ通すか ========================
# 上の理屈が成り立つのは「KF が加速度をそのまま積分している」間だけ。
# KF にはバイアス状態 x[2] と気圧観測があるので、遅い加速度は吸われて
# 消える可能性がある。そこで 0.917 の KF をそのまま PC 上で回し、
# 既知の正弦加速度を入れて出力を差分した（伝達率＝出力振幅/入力振幅）:
#
#     周期 60秒 → 1.00    周期 4秒 → 1.00    周期 2.0秒 → 0.93
#     周期 20秒 → 1.00    周期 3秒 → 0.97    周期 1.5秒 → 0.83
#     周期  8秒 → 1.05    周期2.5秒 → 0.95   周期 1.2秒 → 0.74
#
# 60 秒周期でも 1.00。バイアス状態は加速度を吸っていない（Q_BIAS が
# 十分小さい）。減衰は高域だけで、しかも 2 秒周期で 0.93 と軽い。
# 前進差分（0.5秒）ではなく中心差分（1.0秒）にすると 2 秒周期で 0.66 まで
# 落ちるので、**中心差分（np.gradient）を使ってはいけない**。
#
# ===== ノイズをどう扱うか =====================================
# 「ノイズを考慮したい」という問題意識は正しいが、この経路では
# 結論として**ノイズは律速ではない**。離陸前の静止区間で測った
# ノイズ床は 0.5 秒窓で 0.087 m/s²（= 0.009G）しかなく、
# 飛行中の rms 0.39 m/s² に対して S/N は振幅で 4.5、パワーで 20 倍ある。
#
# 律速なのは帯域のほう。2Hz 記録なので 1Hz より上は原理的に見えない。
# したがってこのツールは「ノイズを消す」のではなく、
# **平均化窓を変えたときに最大値がどう動くかを全部出す**方針をとる。
# 最大 G は窓の長さを言わずに引用してはいけない量である。
#
# ===== 統計の区間と、図の区間は別 =============================
# 統計（rms・最大値・ノイズ床）は巡航区間だけで取る。離水と着水の過渡は
# 突風荷重とは成因が違うので、混ぜると「最大 G」の意味が濁る。
# 一方で **図は必ずログ全体を描く**。離水の瞬間（滑走開始からの数秒）は
# このフライトで最大級のイベントで、巡航区間だけ描くと図から消えてしまう。
# 離水・着水フェーズの最大値は、統計とは別枠でレポートに出す。
#
# ===== 荷重倍数への換算 =======================================
# 構造の話をするときに要るのは鉛直加速度そのものではなく荷重倍数 n=L/W。
# 定常釣合旋回でバンク φ、鉛直加速度 a_z（上向き正）のとき
#     L·cosφ - W = m·a_z   →   n = (1 + a_z/g) / cosφ
# 人力飛行機のピッチ角は小さいので、この近似で十分。
#
# ===== 交差検証 ===============================================
# KF 経路が本物を見ているかは、独立な系統で裏を取る。
#   ・生の pressure 列（KF を通っていない）
#   ・姿勢ログのピッチ角速度から a ≒ V·dθ/dt（迎角一定仮定・5Hz）
# 前者は上位イベントの符号と大きさの一致で、後者は帯域別コヒーレンスで見る。

import os
import numpy as np
import pandas as pd
from scipy.signal import welch, savgol_filter
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import FuncFormatter

# PDFの既定のフォント埋め込み(Type 3)は日本語のグリフ名を扱えず書き出しに失敗する
plt.rcParams['pdf.fonttype'] = 42

# 設定 -------------------------------------------------------
CSV = '2026-07-26_0804.csv'              # 飛行 CSV（2Hz）
EULER_COMP = '20260726_compensated.txt'  # flight_preprocess.py の補正済み姿勢（5Hz）
PDFNAME = 'vertical_accel.pdf'

FLIGHT = ('08:21:26.30', '09:07:42.00')  # 離陸/着水。他のツールと同じ値
EDGE_SEC = 30.0       # 離陸直後・着水直前をこれだけ除外する [秒]
                      # 離水と着水の過渡は「飛行中の突風荷重」とは別物なので分けて出す
G = 9.80665

# 最大値を評価する平均化窓 [秒]。0.5 が 2Hz ログで到達できる最小
WINDOWS = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 10.0, 20.0]
WIN_MAIN = 0.5        # 本文で「最大 G」として採用する窓

# 時系列ページの分割。全体を 1 枚に描くと 2Hz × 47 分 = 5600 点が潰れて読めないので、
# PAGE_SEC 秒ずつのページに分ける（plot_vertical_raw.py と同じ考え方）。
# HOP_SEC を PAGE_SEC より小さくすると、ページの継ぎ目が重なる
PAGE_SEC = 180.0
HOP_SEC = 180.0

N_EVENTS = 6          # 拡大ページを作る上位イベント数
EVENT_SEP = 8.0       # これより近いピークは同一イベントとみなす [秒]
EVENT_HALF = 1.5      # イベント突き合わせの片側窓 [秒]

# 図の注記に使う日本語フォント。見つからなければ英語のみで描く
JP_FONT_CANDIDATES = ['Hiragino Sans', 'Hiragino Maru Gothic Pro',
                      'Noto Sans CJK JP', 'IPAexGothic', 'Arial Unicode MS']

# 配色（plot_vertical_raw.py と共通）------------------------
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C_ACC = '#eb6834'     # 鉛直加速度（KF 由来・主系列）
C_BARO = '#7a3d9e'    # 気圧由来（交差検証用）
C_PITCH = '#2a78d6'   # ピッチ角速度由来
C_VS = '#0b0b0b'      # 昇降率
SUB = '#c9c7c1'
MARK = '#8a8880'
C_NOISE = '#d9d7d1'   # ノイズ床の帯


# ログの置き場所 --------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_SEARCH_DIRS = [os.getcwd(), _HERE, os.path.dirname(_HERE),
                os.path.join(os.path.dirname(_HERE), 'olddata')]


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


# ============================================================
# データ読み込み
# ============================================================
t_to, t_ld = (time_to_daysec(x) for x in FLIGHT)
d = pd.read_csv(find_log(CSV))
# v0.924 以降の CSV は GNSS 高度の列名が GNSS_Altitude。旧ログの Altitude と両対応にする
d = d.rename(columns={'GNSS_Altitude': 'Altitude'})
d['t'] = d['time'].apply(time_to_daysec)

# 巡航区間（離着水の過渡を除いた本体）
cr = d[(d['t'] >= t_to + EDGE_SEC) & (d['t'] <= t_ld - EDGE_SEC)].reset_index(drop=True)
t = cr['t'].to_numpy()
vs = cr['KF_Vspeed'].to_numpy()
gs = cr['gs'].to_numpy()
alt = cr['KF_Altitude'].to_numpy()
# 気圧を高度に直す。地上付近では 1hPa ≒ 8.43m の線形近似で十分
# （絶対高度は要らず、変化分の 2 階微分しか使わないため）
h_baro = -(cr['pressure'].to_numpy() - cr['pressure'].mean()) * 8.43

# ログ全体（作図用）。統計は巡航区間だけで取るが、**図は必ず全部描く**。
# 離水の瞬間は最大級のイベントなので、巡航区間だけ描くと見落とす。
t_all = d['t'].to_numpy()
vs_all = d['KF_Vspeed'].to_numpy()
alt_all = d['KF_Altitude'].to_numpy()
gs_all = d['gs'].to_numpy()

# プラットフォーム滑走の開始 ＝ 対地速度が立ち上がった最初の時刻。
# FLIGHT[0] は他ツールと揃えた「離水」の定義で、滑走はその数秒前から始まっている。
_run = np.flatnonzero(gs_all > 1.0)
t_run = float(t_all[_run[0]]) if len(_run) else t_to

# 離陸前の静止区間 ＝ ノイズ床の実測。真の鉛直加速度が 0 と分かっている唯一の区間
rest = d[(d['t'] < t_run) & (d['gs'] < 0.2)]
v_rest = rest['KF_Vspeed'].to_numpy()
t_rest = rest['t'].to_numpy()
SIG_V = float(v_rest.std())            # KF_Vspeed のノイズ [m/s]


def accel(window_sec, tt=None, vv=None):
    """KF_Vspeed の前進差分から、窓 window_sec の平均鉛直加速度を出す。

    中心差分（np.gradient）ではなく前進差分を使う。中心差分は実質 2 倍の
    窓になり、2 秒周期の成分を 0.66 まで落としてしまう（ファイル冒頭参照）。
    戻り値の時刻は区間の中央。
    """
    tt = t if tt is None else tt
    vv = vs if vv is None else vv
    k = max(1, int(round(window_sec * 2.0)))   # ログは 2Hz
    a = (vv[k:] - vv[:-k]) / (tt[k:] - tt[:-k])
    return (tt[k:] + tt[:-k]) / 2, a


def noise_floor(window_sec):
    """窓 window_sec での ノイズ床 [m/s²]。

    静止区間そのものを同じ手順で差分して測る（サンプルが足りる窓だけ）。
    足りない長い窓は σ_a = √2·σ_v/W で外挿する。実測（0.5秒 0.087 /
    1.0秒 0.045 / 2.0秒 0.030）と式（0.107 / 0.054 / 0.027）はよく合うので、
    外挿しても大きくは外さない。
    """
    _, a_r = accel(window_sec, t_rest, v_rest)
    if len(a_r) >= 8:
        return float(a_r.std())
    return np.sqrt(2.0) * SIG_V / window_sec


# ============================================================
# 本体の計算
# ============================================================
t_a, a_main = accel(WIN_MAIN)
nf_main = noise_floor(WIN_MAIN)

# 窓ごとの統計
sweep = []
for w in WINDOWS:
    _, aw = accel(w)
    sweep.append(dict(w=w, rms=float(aw.std()), nf=noise_floor(w),
                      mx=float(np.abs(aw).max()),
                      p999=float(np.percentile(np.abs(aw), 99.9))))

# ログ全体の加速度（作図と、離着水フェーズの評価に使う）
t_all_a, a_all = accel(WIN_MAIN, t_all, vs_all)
t_all_a2, a_all2 = accel(2.0, t_all, vs_all)

# 離水・着水フェーズ（巡航とは別物なので分けて評価する）。
# 離水は「滑走開始からその 30 秒後」まで。着水は「着水の 30 秒前からログ末尾」まで
# ＝ ログの最後まで含める。着水後の衝撃もログに残っているため。
def phase_peak(t0, t1):
    m = (t_all_a >= t0) & (t_all_a <= t1)
    if not m.any():
        return np.nan, np.nan
    i = int(np.argmax(np.abs(a_all[m])))
    return float(a_all[m][i]), float(t_all_a[m][i])


to_peak, to_peak_t = phase_peak(t_run, t_to + EDGE_SEC)
ld_peak, ld_peak_t = phase_peak(t_ld - EDGE_SEC, t_all_a[-1])

# ---- 交差検証 1: 独立な気圧列 ------------------------------
# KF を通っていない生の気圧から、イベント前後の昇降率の変化を直線当てはめで出す
def baro_accel_at(tc, half=EVENT_HALF):
    m1 = (t > tc - half) & (t <= tc)
    m2 = (t > tc) & (t < tc + half)
    if m1.sum() < 3 or m2.sum() < 3:
        return np.nan
    v1 = np.polyfit(t[m1], h_baro[m1], 1)[0]
    v2 = np.polyfit(t[m2], h_baro[m2], 1)[0]
    return (v2 - v1) / half


def kf_accel_at(tc, half=EVENT_HALF):
    m1 = (t > tc - half) & (t <= tc)
    m2 = (t > tc) & (t < tc + half)
    if m1.sum() < 1 or m2.sum() < 1:
        return np.nan
    return (vs[m2].mean() - vs[m1].mean()) / half


# 上位イベントの抽出（近接ピークは 1 個にまとめる）
events = []
for i in np.argsort(-np.abs(a_main)):
    if all(abs(t_a[i] - e['t']) > EVENT_SEP for e in events):
        events.append(dict(t=float(t_a[i]), a=float(a_main[i])))
    if len(events) >= N_EVENTS * 2:
        break
for e in events:
    e['kf3'] = kf_accel_at(e['t'])
    e['baro3'] = baro_accel_at(e['t'])
    # 一致判定: 符号が同じで、気圧側が KF 側の 30% 以上の大きさを持つ
    same = (np.sign(e['kf3']) == np.sign(e['baro3']))
    e['ok'] = '○' if (same and abs(e['baro3']) > 0.3 * abs(e['kf3'])) else ('△' if same else '×')

# ---- 交差検証 2: 姿勢ログのピッチ角速度（5Hz・別物理量）----
eu = pd.read_csv(find_log(EULER_COMP))
eu['t'] = eu['time'].apply(time_to_daysec)
eu = eu[(eu['t'] >= t_to + EDGE_SEC) & (eu['t'] <= t_ld - EDGE_SEC)].reset_index(drop=True)
FE = 1.0 / float(np.median(np.diff(eu['t'].to_numpy())))
te = np.arange(eu['t'].iloc[0], eu['t'].iloc[-1], 1.0 / FE)
pitch = np.radians(np.interp(te, eu['t'].to_numpy(), eu['pitch'].to_numpy()))
roll = np.radians(np.interp(te, eu['t'].to_numpy(), eu['roll'].to_numpy()))
V_air = np.interp(te, t, gs)
# 迎角一定なら経路角の変化率＝ピッチ角速度。a_n ≒ V·dγ/dt
n_sg = int(1.0 * FE) // 2 * 2 + 1
a_pitch = V_air * savgol_filter(pitch, n_sg, 2, deriv=1, delta=1.0 / FE)

# ---- 荷重倍数 n = (1 + a_z/g)/cosφ --------------------------
a_on_te = np.interp(te, t_a, a_main)
n_load = (1.0 + a_on_te / G) / np.cos(roll)
i_nmax, i_nmin = int(np.argmax(n_load)), int(np.argmin(n_load))
bank_only = float((1.0 / np.cos(roll)).max())


# ============================================================
# レポート（標準出力）
# ============================================================
print('=' * 74)
print('鉛直加速度（G）の推定   %s  %s - %s' % (CSV, FLIGHT[0], FLIGHT[1]))
print('=' * 74)
print('巡航区間 %s - %s  (%.1f 分, N=%d)  ※離着水の前後 %.0f 秒は除外'
      % (daysec_to_hms(t[0]), daysec_to_hms(t[-1]), (t[-1] - t[0]) / 60, len(t), EDGE_SEC))

print('\n--- ノイズ床の実測（離陸前の静止 %.0f 秒, N=%d）---' % (t_rest[-1] - t_rest[0], len(v_rest)))
print('  KF_Vspeed のばらつき         %.4f m/s' % SIG_V)
print('  → %.1f秒窓での加速度ノイズ床 %.3f m/s² = %.4f G' % (WIN_MAIN, nf_main, nf_main / G))
print('  参考: 生の気圧の2階微分は地上で rms 0.55〜0.70 m/s²（飛行中の信号より大きい）')

print('\n--- 平均化窓ごとの最大値（ここが結論）---')
print('%8s %9s %9s %7s %10s %10s' % ('窓[秒]', 'rms', 'ノイズ床', 'S/N', 'max|a_z|', 'max [G]'))
for s in sweep:
    print('%8.1f %9.3f %9.3f %7.1f %10.3f %10.4f'
          % (s['w'], s['rms'], s['nf'], s['rms'] / s['nf'], s['mx'], s['mx'] / G))
print('  ※ 最大 G は窓の長さとセットでしか意味を持たない。')
print('     2Hz 記録なので %.1f 秒より短い窓は作れず、これが分解能の限界。' % WIN_MAIN)

print('\n--- 離水・着水（巡航とは別物なので分けて表示。ログ全体を見る）---')
print('  ログの範囲   %s - %s' % (daysec_to_hms(t_all[0]), daysec_to_hms(t_all[-1])))
print('  滑走開始     %s（対地速度が立ち上がった時刻）' % daysec_to_hms(t_run))
print('  離水         %s（FLIGHT[0]。他ツールと共通の定義）' % daysec_to_hms(t_to))
print('  離水フェーズ（%s - %s）の最大 %+.3f m/s² = %+.4f G @ %s'
      % (daysec_to_hms(t_run), daysec_to_hms(t_to + EDGE_SEC),
         to_peak, to_peak / G, daysec_to_hms(to_peak_t)))
print('  着水フェーズ（%s - %s）の最大 %+.3f m/s² = %+.4f G @ %s'
      % (daysec_to_hms(t_ld - EDGE_SEC), daysec_to_hms(t_all_a[-1]),
         ld_peak, ld_peak / G, daysec_to_hms(ld_peak_t)))

print('\n--- 上位イベントと、独立な気圧列での裏取り（±%.1f秒）---' % EVENT_HALF)
print('%-11s %10s %10s %10s %5s' % ('時刻', 'a_z(%.1fs)' % WIN_MAIN, 'KF(%.0fs)' % (2 * EVENT_HALF),
                                     '気圧(%.0fs)' % (2 * EVENT_HALF), '一致'))
for e in sorted(events, key=lambda x: -abs(x['a']))[:N_EVENTS * 2]:
    print('%-11s %10.3f %10.3f %10.3f %5s'
          % (daysec_to_hms(e['t']), e['a'], e['kf3'], e['baro3'], e['ok']))
n_ok = sum(1 for e in events if e['ok'] == '○')
print('  ○=符号も大きさも一致 / △=符号のみ一致 / ×=不一致 …… %d/%d が ○' % (n_ok, len(events)))

print('\n--- 荷重倍数 n = (1 + a_z/g)/cosφ  （%.1f秒窓）---' % WIN_MAIN)
print('  最大 n = %.3f  @ %s  (a_z %+.2f m/s², roll %+.1f°)'
      % (n_load[i_nmax], daysec_to_hms(te[i_nmax]), a_on_te[i_nmax], np.degrees(roll[i_nmax])))
print('  最小 n = %.3f  @ %s  (a_z %+.2f m/s², roll %+.1f°)'
      % (n_load[i_nmin], daysec_to_hms(te[i_nmin]), a_on_te[i_nmin], np.degrees(roll[i_nmin])))
print('  バンクだけの寄与は最大 %.4f（最大ロール %.1f°）。旋回そのものは効いていない'
      % (bank_only, np.degrees(np.abs(roll)).max()))

print('\n--- 帯域の限界（ここが精度の律速）---')
f_ps, P_ps = welch(a_main - a_main.mean(), 2.0, nperseg=512)
f_pt, P_pt = welch(a_pitch - a_pitch.mean(), FE, nperseg=512)


def band(P, f, lo, hi):
    m = (f >= lo) & (f < hi)
    return float(np.trapz(P[m], f[m])) if m.sum() > 1 else 0.0


print('  KF由来 a_z の帯域別分散 [(m/s²)²]: ' + '  '.join(
    '%.1f-%.1fHz %.4f' % (lo, hi, band(P_ps, f_ps, lo, hi))
    for lo, hi in [(0, 0.2), (0.2, 0.5), (0.5, 1.0)]))
print('  ピッチ角速度由来（%.1fHz・ナイキスト%.1fHz）の帯域別分散: ' % (FE, FE / 2) + '  '.join(
    '%.1f-%.1fHz %.4f' % (lo, hi, band(P_pt, f_pt, lo, hi))
    for lo, hi in [(0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.4)]))
print('  → 姿勢は 2.4Hz まで見えるが 0.5Hz より上はほぼ空。機体（剛体）の')
print('     経路運動は 0.5Hz 以下に収まっている。ただし迎角変化による突風荷重は')
print('     ピッチにすぐ出ないので、これは「上に何も無い」証明にはならない。')
print('  相関係数（KF由来 vs ピッチ由来）%.3f' % np.corrcoef(a_on_te, a_pitch)[0, 1])

# 0.5 秒窓の分散のうち、どれだけが高域（＝ナイキストの際）に居るか。
# 高域成分は「1サンプルごとに符号が反転する」形で出るので、ラグ1の
# 自己相関が負に振れる。滑らかな信号を差分すればラグ1は正になるはず。
rho1 = float(np.corrcoef(a_main[:-1], a_main[1:])[0, 1])
hi_frac = band(P_ps, f_ps, 0.5, 1.0) / band(P_ps, f_ps, 0.0, 1.0)
noise_frac = (nf_main ** 2) / (a_main.std() ** 2)
print('\n--- %.1f秒窓の中身の内訳 ---' % WIN_MAIN)
print('  ラグ1 自己相関 %+.3f（負＝1サンプルごとに反転する成分が優勢）' % rho1)
print('  分散のうち 0.5Hz より上にあるのが %.0f%%' % (hi_frac * 100))
print('  そのうち計器ノイズで説明できるのは最大でも %.0f%%（地上実測のノイズ床から）'
      % (noise_frac * 100))
print('  → 高域成分の大半は本物の加速度。ただし「機体全体の荷重」ではなく')
print('     センサー位置の振動（ペダリング由来など）が混ざっている可能性がある。')
print('     姿勢ログのロールに 1.4-1.7Hz の山があり、これは 2Hz ログでは折り返す。')

# ---- 分解能の代償: 短い突風をどれだけ過小評価するか ----------
# 半正弦パルス（持続 τ）を 0.5 秒の箱型平均に通し、2Hz で拾ったときの
# ピーク保存率。これが「0.5秒窓の値がどれだけ控えめか」の目安になる。
print('\n--- 短い突風の過小評価（半正弦パルスでの検証）---')
print('%12s %14s' % ('パルス持続[秒]', 'ピーク保存率'))
fs_fine = 200.0
for tau in [0.25, 0.5, 1.0, 2.0, 4.0]:
    tf = np.arange(-6, 6, 1 / fs_fine)
    pulse = np.where(np.abs(tf) < tau / 2, np.cos(np.pi * tf / tau), 0.0)
    nb = int(round(WIN_MAIN * fs_fine))
    boxed = np.convolve(pulse, np.ones(nb) / nb, mode='same')
    # 2Hz の格子は突風に対して任意の位相で来るので、最悪位相ではなく
    # 位相を振った中央値をとる（どの位相で当たるかは運任せのため）
    ratios = [boxed[i::int(fs_fine / 2.0)].max() for i in range(int(fs_fine / 2.0))]
    print('%12.2f %14.2f' % (tau, float(np.median(ratios))))
print('  → %.1f 秒窓の最大値は「%.1f 秒以上続く荷重」ならほぼ正しい。'
      % (WIN_MAIN, 1.0))
print('     それより短いものは過小評価になる。この表のぶんだけ割り増して読む。')


# ============================================================
# 作図
# ============================================================
avail = {f.name for f in fm.fontManager.ttflist}
JP = next((f for f in JP_FONT_CANDIDATES if f in avail), None)


def style(ax, ylab=None, xlab=None):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=1.0, zorder=0)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_SUB, labelsize=9)
    if ylab:
        ax.set_ylabel(ylab, color=INK_SUB, fontsize=10)
    if xlab:
        ax.set_xlabel(xlab, color=INK_SUB, fontsize=10)


def page(title, sub):
    fig = plt.figure(figsize=(11.7, 8.3), facecolor=SURFACE)
    fig.suptitle(title, color=INK, fontsize=14, fontweight='bold', x=0.012, ha='left', y=0.975)
    fig.text(0.012, 0.935, sub, color=INK_SUB, fontsize=10, va='top')
    return fig


def note(fig, txt, y=0.075):
    if JP:
        fig.text(0.075, y, txt, fontname=JP, fontsize=9.3, color=INK_SUB,
                 va='top', linespacing=1.7)


with PdfPages(PDFNAME) as pdf:
    # --- 1ページ目: 平均化窓 vs 最大値（この解析の結論そのもの）----
    fig = page('Peak vertical acceleration vs averaging window',
               'The answer depends on the window. 2Hz logging cannot go below %.1f s.' % WIN_MAIN)
    ax = fig.add_axes([0.085, 0.30, 0.40, 0.55])
    ww = [s['w'] for s in sweep]
    ax.plot(ww, [s['mx'] / G for s in sweep], 'o-', color=C_ACC, lw=1.8, ms=5, zorder=4,
            label='max |a_z|')
    ax.plot(ww, [s['p999'] / G for s in sweep], 's--', color=C_BARO, lw=1.3, ms=4, zorder=4,
            label='99.9 percentile')
    ax.plot(ww, [s['rms'] / G for s in sweep], '^-', color=C_VS, lw=1.3, ms=4, zorder=4,
            label='rms')
    ax.fill_between(ww, 0, [s['nf'] / G for s in sweep], color=C_NOISE, zorder=1,
                    label='noise floor (measured on ground)')
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xticks(ww)
    ax.set_xticklabels([('%g' % w) for w in ww])
    style(ax, 'vertical acceleration [G]', 'averaging window [s]')
    ax.legend(frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='lower left')

    ax2 = fig.add_axes([0.58, 0.30, 0.39, 0.55])
    # 超過頻度（ロード・スペクトラム）: |a_z| が横軸の値を超えた回数 / 時間
    dur_h = (t[-1] - t[0]) / 3600.0
    lv = np.linspace(0, np.abs(a_main).max() / G, 120)
    cnt = [np.sum(np.abs(a_main) / G >= x) / dur_h for x in lv]
    ax2.semilogy(lv, np.maximum(cnt, 1e-3), color=C_ACC, lw=1.8, zorder=4)
    ax2.axvline(nf_main / G, color=MARK, ls='--', lw=1.2, zorder=3)
    ax2.text(nf_main / G, ax2.get_ylim()[1], ' noise floor', color=INK_SUB, fontsize=8.5,
             va='top', ha='left')
    style(ax2, 'exceedances per hour', '|a_z| level [G]  (%.1f s window)' % WIN_MAIN)

    note(fig,
         "左: 平均化窓を長くするほど最大値は下がる。ノイズ床(灰)より rms が桁で上にあるので、\n"
         "    この推定はノイズではなく帯域で決まっている。%.1f 秒窓の %.3fG が 2Hz ログの到達点。\n"
         "右: 超過頻度。%.2fG を超える瞬間は 1 時間あたり %.0f 回あった。"
         % (WIN_MAIN, sweep[0]['mx'] / G, 0.1,
            np.sum(np.abs(a_main) / G >= 0.1) / dur_h))
    pdf.savefig(fig, facecolor=SURFACE)
    plt.close(fig)

    # --- 2ページ目以降: 時系列 ---------------------------------
    # 描画はログ全体（離水前の地上 → 滑走 → 巡航 → 着水 → ログ末尾）を通してやる。
    # 巡航区間だけ描くと、最大級のイベントである離水の瞬間が図から消える。
    #
    # 1 枚目は全体を 1 ページに、2 枚目からは PAGE_SEC 秒ずつ拡大する。
    # 縦軸は全ページ共通の固定値にしてあるので、振れ幅をページ間で直接比べられる。
    # （a_z は平均 0 まわりの量なので、中心を区間ごとにずらす必要が無い）
    ylim_a = float(np.abs(a_all).max()) / G * 1.06
    ylim_a2 = float(np.abs(a_all2).max()) / G * 1.10
    ylim_v = float(np.abs(vs_all).max()) * 1.10
    alt_span = float(np.percentile(
        [np.ptp(alt_all[(t_all >= s0) & (t_all < s0 + PAGE_SEC)])
         for s0 in np.arange(t_all[0], t_all[-1] - PAGE_SEC, PAGE_SEC)], 95)) * 1.2
    ev_top = sorted(events, key=lambda x: -abs(x['a']))[:N_EVENTS]

    def draw_series(t0, t1, title, sub, dense, xfmt):
        """[t0, t1) の 4 段ストリップチャートを 1 ページ描く"""
        fig = page(title, sub)
        axs = [fig.add_axes([0.085, 0.655, 0.89, 0.225]),
               fig.add_axes([0.085, 0.475, 0.89, 0.165]),
               fig.add_axes([0.085, 0.295, 0.89, 0.165]),
               fig.add_axes([0.085, 0.145, 0.89, 0.135])]
        lw, ms = (0.7, 0) if dense else (1.2, 2.5)
        m0 = (t_all_a >= t0) & (t_all_a <= t1)
        m2 = (t_all_a2 >= t0) & (t_all_a2 <= t1)
        mv = (t_all >= t0) & (t_all <= t1)

        axs[0].axhspan(-nf_main / G, nf_main / G, color=C_NOISE, zorder=1)
        axs[0].plot(t_all_a[m0], a_all[m0] / G, '-o', color=C_ACC, lw=lw, ms=ms, zorder=4)
        axs[0].set_ylim(-ylim_a, ylim_a)
        axs[0].set_ylabel('a_z [G]  %.1fs' % WIN_MAIN, color=INK_SUB, fontsize=9.5)

        axs[1].plot(t_all_a2[m2], a_all2[m2] / G, '-o', color=C_BARO, lw=lw, ms=ms, zorder=4)
        axs[1].set_ylim(-ylim_a2, ylim_a2)
        axs[1].set_ylabel('a_z [G]  2s', color=INK_SUB, fontsize=9.5)

        axs[2].plot(t_all[mv], vs_all[mv], '-o', color=C_VS, lw=lw, ms=ms, zorder=4)
        axs[2].set_ylim(-ylim_v, ylim_v)
        axs[2].set_ylabel('V/S [m/s]', color=INK_SUB, fontsize=9.5)

        axs[3].plot(t_all[mv], alt_all[mv], '-', color=C_PITCH, lw=1.2, zorder=4)
        if not dense and mv.sum() > 1:
            c = (float(alt_all[mv].max()) + float(alt_all[mv].min())) / 2
            axs[3].set_ylim(c - alt_span / 2, c + alt_span / 2)
        axs[3].set_ylabel('Alt [m]', color=INK_SUB, fontsize=9.5)

        for ax in axs:
            ax.set_xlim(t0, t1)
            # フェーズの境目。離水前と着水後は地上なので薄く塗る
            ax.axvspan(t0, min(t1, t_run), color=GRID, alpha=0.6, zorder=0)
            ax.axvspan(max(t0, t_ld), t1, color=GRID, alpha=0.6, zorder=0)
            for tm, col in ((t_run, C_ACC), (t_to, MARK), (t_ld, MARK)):
                if t0 <= tm <= t1:
                    ax.axvline(tm, color=col, ls='-', lw=1.2, zorder=3)
            for e in ev_top:
                if t0 <= e['t'] <= t1:
                    ax.axvline(e['t'], color=MARK, ls='--', lw=1.0, zorder=2)
            style(ax)
        for ax in axs[:-1]:
            ax.tick_params(labelbottom=False)
        axs[-1].xaxis.set_major_formatter(FuncFormatter(lambda v, _: xfmt(v)))
        axs[-1].set_xlabel('Time of day', color=INK_SUB, fontsize=10)
        return fig

    # 全体を 1 ページ
    fig = draw_series(t_all[0], t_all[-1],
                      'Vertical acceleration — whole log',
                      '%s - %s   (%.0f min, includes launch and ditching)'
                      % (daysec_to_hms(t_all[0]), daysec_to_hms(t_all[-1]),
                         (t_all[-1] - t_all[0]) / 60),
                      dense=True, xfmt=daysec_to_hm)
    note(fig,
         "灰色の縦帯は地上（滑走前・着水後）。橙の縦線が滑走開始 %s、\n"
         "灰の縦線が離水 %s と着水 %s。破線は上位 %d イベント。\n"
         "この 1 枚は潰れて見えないので、次ページ以降に %.0f 秒ずつ拡大したものを付ける。"
         % (daysec_to_hms(t_run), daysec_to_hms(t_to), daysec_to_hms(t_ld),
            N_EVENTS, PAGE_SEC), y=0.095)
    pdf.savefig(fig, facecolor=SURFACE)
    plt.close(fig)

    # PAGE_SEC 秒ずつの拡大ページ
    starts = np.arange(t_all[0], t_all[-1], HOP_SEC)
    for i, s0 in enumerate(starts, start=1):
        s1 = min(s0 + PAGE_SEC, t_all[-1])
        m0 = (t_all_a >= s0) & (t_all_a <= s1)
        if m0.sum() < 4:
            continue
        pk = float(a_all[m0][np.argmax(np.abs(a_all[m0]))])
        fig = draw_series(s0, s1,
                          '%s - %s   (%d / %d)'
                          % (daysec_to_hms(s0), daysec_to_hms(s1), i, len(starts)),
                          'peak in this window  %+.3f m/s² = %+.4f G      rms %.3f m/s²'
                          % (pk, pk / G, float(a_all[m0].std())),
                          dense=False, xfmt=daysec_to_hms)
        note(fig,
             "縦軸の範囲は全ページ共通なので、振れ幅はページをまたいで比べられる。\n"
             "灰の帯は地上で実測したノイズ床（±%.3f G）。" % (nf_main / G), y=0.085)
        pdf.savefig(fig, facecolor=SURFACE)
        plt.close(fig)

    # --- 3ページ目: スペクトルと帯域の限界 ----------------------
    fig = page('Where the bandwidth limit bites',
               'CSV is 2 Hz (Nyquist 1 Hz). The attitude log is %.1f Hz (Nyquist %.1f Hz).'
               % (FE, FE / 2))
    ax = fig.add_axes([0.085, 0.32, 0.40, 0.53])
    ax.loglog(f_ps[1:], P_ps[1:], color=C_ACC, lw=1.4, zorder=4, label='a_z from KF_Vspeed')
    ax.axvline(1.0, color=MARK, ls='--', lw=1.2, zorder=3)
    ax.text(1.0, ax.get_ylim()[1], ' Nyquist', color=INK_SUB, fontsize=8.5, va='top')
    style(ax, 'PSD [(m/s²)²/Hz]', 'frequency [Hz]')
    ax.legend(frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='lower left')

    ax2 = fig.add_axes([0.58, 0.32, 0.39, 0.53])
    ax2.loglog(f_pt[1:], P_pt[1:], color=C_PITCH, lw=1.4, zorder=4,
               label='a from pitch rate (%.1f Hz log)' % FE)
    ax2.axvline(1.0, color=MARK, ls='--', lw=1.2, zorder=3)
    style(ax2, 'PSD [(m/s²)²/Hz]', 'frequency [Hz]')
    ax2.legend(frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='lower left')
    note(fig,
         "右は 2.4Hz まで見える別系統（姿勢のピッチ角速度）。0.5Hz より上はほぼ空で、\n"
         "機体の経路運動そのものは 0.5Hz 以下に収まっている。\n"
         "ただし突風による荷重は迎角変化で先に出てピッチには遅れて出るため、\n"
         "「1Hz 以上に荷重が無い」ことの証明にはならない。ここが残る不確かさ。")
    pdf.savefig(fig, facecolor=SURFACE)
    plt.close(fig)

    # --- 4ページ目以降: 上位イベントの拡大 ----------------------
    for e in sorted(events, key=lambda x: -abs(x['a']))[:N_EVENTS]:
        tc = e['t']
        m = (t > tc - 12) & (t < tc + 12)
        ma = (t_a > tc - 12) & (t_a < tc + 12)
        fig = page('Event  %s    a_z = %+.3f m/s² = %+.4f G'
                   % (daysec_to_hms(tc), e['a'], e['a'] / G),
                   'cross-check: KF %+.3f  /  barometer (independent) %+.3f m/s²  →  %s'
                   % (e['kf3'], e['baro3'], e['ok']))
        axs = [fig.add_axes([0.085, 0.62, 0.89, 0.25]),
               fig.add_axes([0.085, 0.38, 0.89, 0.22]),
               fig.add_axes([0.085, 0.16, 0.89, 0.20])]
        axs[0].axhspan(-nf_main / G, nf_main / G, color=C_NOISE, zorder=1)
        axs[0].plot(t_a[ma], a_main[ma] / G, 'o-', color=C_ACC, lw=1.4, ms=3, zorder=4)
        axs[0].set_ylabel('a_z [G]', color=INK_SUB, fontsize=10)
        axs[1].plot(t[m], vs[m], 'o-', color=C_VS, lw=1.4, ms=3, zorder=4)
        axs[1].set_ylabel('KF_Vspeed [m/s]', color=INK_SUB, fontsize=10)
        # 気圧（独立系統）と KF 高度を重ねる。気圧は平均を合わせて比較する
        axs[2].plot(t[m], h_baro[m] - h_baro[m].mean(), 'o-', color=C_BARO, lw=1.4, ms=3,
                    zorder=4, label='barometer (raw pressure)')
        axs[2].plot(t[m], alt[m] - alt[m].mean(), color=SUB, lw=1.4, zorder=3,
                    label='KF_Altitude')
        axs[2].set_ylabel('Alt [m, mean removed]', color=INK_SUB, fontsize=10)
        axs[2].legend(frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='upper right')
        for k, ax in enumerate(axs):
            ax.set_xlim(tc - 12, tc + 12)
            ax.axvline(tc, color=MARK, ls='--', lw=1.2, zorder=2)
            style(ax)
            if k < 2:
                ax.tick_params(labelbottom=False)
        axs[-1].xaxis.set_major_formatter(FuncFormatter(lambda v, _: daysec_to_hms(v)))
        axs[-1].set_xlabel('Time of day', color=INK_SUB, fontsize=10)
        note(fig,
             "一番下が KF を通っていない生の気圧。ここが KF と同じ向きに動いていれば、\n"
             "そのイベントは IMU 側の単発の誤りではなく実際の運動である。", y=0.095)
        pdf.savefig(fig, facecolor=SURFACE)
        plt.close(fig)

    n_pages = pdf.get_pagecount()

print('\nPDFを保存: %s  (%d ページ)' % (PDFNAME, n_pages))
if JP is None:
    print('警告: 日本語フォントが見つからないため、日本語の説明は省略しました')
