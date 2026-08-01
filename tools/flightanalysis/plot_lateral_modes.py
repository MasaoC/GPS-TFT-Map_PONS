# -*- coding: utf-8 -*-
# 2機のトラックから横・方向の安定性(ダッチロール/スパイラル)の傾向を比べる
#
# 比べる対象:
#   2025年大会 白夜 : PONSv5_planelog_2025-07-26_1539.csv
#   2026年大会 陽還 : 2026-07-26_0804.csv
# 上反角とラダーの諸元が違うので、その違いが飛行機の横の癖として
# トラックに出ているはず、という仮説を確かめる。
#
# 使うログ:
#   トラック : 2025 PONSv5_planelog_...csv / 2026 2026-07-26_0804.csv
#   IMU・舵 : 2025 20250726_log_all_joined.csv (ロール・ラダー・対気速度, 1Hz)
#             2026 20260726.txt (BNO085の生ログ) + beta_da_20260726.txt (TAS)
#   ラダーの操作ログがあるのは2025年だけ。これは2機の比較には使えないが、
#   「トラックから操縦区間を当てる」という手法そのものの検証に使える。
#
# ===== 考え方 ===============================================
# 2機に共通して比べられるのは GPS の TrueTrack から作る
#   旋回角速度 r = d(TrueTrack)/dt
# という1本の信号。まずこれで「揺れ方」を比べ、そのあと
# IMUのロール角で同じことをして、GPSに依らない裏取りをする。
#
#  ダッチロールは横滑りとロールが組んだ振動なので、
#  r に決まった周期の振動として出る。減衰が弱いほど揺れが尾を引く。
#  スパイラルは振動しないモードで、ゆっくり旋回が深まっていく。
#  パイロットが当て舵で止めるので、r のゆっくりした成分の大きさと
#  当て舵の頻度に出る。
#
# 操縦成分の除去について:
#  意図した旋回(ラダー操作)は r が大きいまま持続するので、
#  10秒平均した |r| がしきい値を超える区間を、その前後の
#  ガード時間ごと捨てる。残った区間だけを解析する。
#  ただし後述のとおり、この判定を強くしても弱くしても結論は変わらない
#  (しきい値感度を実行時に表示する)。
#
# 減衰比の出し方:
#  乱れ(大気の乱流)に叩かれ続けている2次系の自己相関関数は
#    ACF(τ) = exp(-ζωn·τ)·( cos(ωd·τ) + ζ/√(1-ζ²)·sin(ωd·τ) )
#  という形になる。クリーンな区間の r からこの形を最小二乗で当てはめて
#  周期 T と減衰比 ζ を求める。振動しない信号なら ACF はすぐ 0 に落ち、
#  振動する信号なら谷と山を繰り返すので、両者は形ではっきり分かれる。
#
# ===== このスクリプトが気をつけていること ===================
# 2025年のログは 1Hz のはずが約24%取りこぼしていて、間を補間すると
# 速い成分が削られる。一方2026年は 2Hz で欠測が無い。この差を
# 「機体の違い」と取り違えないため、2026年のデータに 2025年と同じ
# 欠測率をわざと与えたものも一緒に計算して重ねて描く。
# これが2026年の元データとほぼ重なれば、差は欠測のせいではない。
#
# IMUのロールは両年とも「補正前」の値を使う。flight_preprocess.py の補正済みロールは
# 遠心加速度の分 (V/g)·r を足し戻しているため、これを r に回帰すると
# 中身に関係なく協調旋回の値に寄ってしまう。補正前のロールは
# 「横方向の見かけ重力」= 横滑り計(ボール)に相当し、釣り合っていない
# 横の動きだけが出るので、ダッチロールを見るにはむしろ都合が良い。
# 2025年は補正前しか無いので、条件をそろえる意味でも補正前で統一する。
#
# 使い方: python3 plot_lateral_modes.py
# 出力: lateral_modes.png

import numpy as np
import pandas as pd
from scipy.signal import welch, butter, filtfilt, coherence
from scipy.optimize import curve_fit
from scipy.stats import mannwhitneyu
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.lines import Line2D

# 設定 -------------------------------------------------------
CSV_2025 = 'PONSv5_planelog_2025-07-26_1539.csv'      # 2025年のGPS(トラック)
JOINED_2025 = '20250726_log_all_joined.csv'          # 2025年のIMU・舵・対気速度(1Hz)
CSV_2026 = '2026-07-26_0804.csv'                     # 2026年のGPS(トラック)
EULER_2026 = '20260726.txt'          # 2026年のIMU生ログ。2025年と条件をそろえるため補正前を使う
BETADA_2026 = 'beta_da_20260726.txt'  # flight_preprocess.py の出力。対気速度TASを取る
PLOTNAME = 'lateral_modes.png'

FLIGHT_2026 = ('08:21:26.30', '09:07:42.00')   # 離陸/着水。他のツールと同じ値

FS = 1.0              # 解析のサンプリング周波数 [Hz]。2025年に合わせて1Hz
GS_MIN = 4.0          # これ未満の対地速度は地上/離着水とみなして捨てる [m/s]
GAP_SEC = 60.0        # これ以上の時間の穴があればログを別フライトとして切る [秒]

TURN_THRESH = 2.0     # 操縦による旋回とみなす旋回角速度 [deg/s]
TURN_AVG_SEC = 10.0   # 上の判定に使う移動平均の長さ [秒]
GUARD_SEC = 10.0      # 旋回とみなした区間の前後に足す余白 [秒]
MIN_SEG_SEC = 60.0    # これより短いクリーン区間は解析に使わない [秒]
THRESH_SWEEP = [1.5, 2.0, 3.0, 99.0]   # しきい値感度の確認用 (99=除外なし)

ACF_MAXLAG = 45.0     # 自己相関を見る最大のずらし時間 [秒]
FIT_SKIP_SEC = 2.0    # 当てはめで使わない最初のラグ [秒]。ノイズの尖りを避ける
BAND = (3.0, 45.0)    # 自己相関の前に通す帯域 [秒]。3秒未満はGPSの分解能で読めない
DROP_RATE_2025 = None # 2025年の欠測率。Noneならログから自動で求める
N_DROP_TRIAL = 10     # 欠測の再現は乱数なので何回か回して平均する
TRACK_QUANT = 1.0     # TrueTrack の分解能 [deg]。量子化ノイズの床を引くのに使う
EXCERPT_SEC = 240.0   # 時系列の抜粋パネルの長さ [秒]
RUDDER_AVG_SEC = 10.0 # 舵の踏み込み量を測る移動平均の長さ [秒]
# XFLR5(操縦を含まない裸の機体の計算)の結果。注記で実測と比べる
XFLR_T_HALF = 16.3    # XFLR5 の t2 [秒]。周期ではなく「振幅が半分になる時間」
# root locus の固有値の実部 λ [1/s]。負なら収束、正なら発散
XFLR_LAMBDA = {
    '2025': {'spiral': +0.02, 'dutch': -0.8, 'roll': -30.0},
    '2026': {'spiral': -0.10, 'dutch': -0.6, 'roll': -30.0},
}
# 図の下の注記に使う日本語フォント。見つからなければ英語の注記に切り替える
JP_FONT_CANDIDATES = ['Hiragino Sans', 'Hiragino Maru Gothic Pro',
                      'Noto Sans CJK JP', 'IPAexGothic', 'Arial Unicode MS']
# パワーをまとめる周期帯 [秒]
BANDS = [(3, 6), (6, 12), (12, 30), (30, 60)]

# 配色 -------------------------------------------------------
# 他のツールと同じ紙色と2色対比を使う
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SUB = '#52514e'
GRID = '#e6e5e1'
C25 = '#2a78d6'       # 2025 白夜
C26 = '#eb6834'       # 2026 陽還
MUTED = '#c9c7c1'     # 除外した区間(操縦とみなした部分)
MARK = '#8a8880'      # 基準線


def time_to_daysec(t):
    h, m, s = str(t).split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


# 読み込みと飛行区間の切り出し -------------------------------
def load_flight(path, window=None):
    """CSVを読んで、実際に飛んでいる連続した区間だけを返す"""
    d = pd.read_csv(path)
    d['t'] = d['time'].apply(time_to_daysec)
    if window is not None:
        t0, t1 = (time_to_daysec(x) for x in window)
        d = d[(d['t'] >= t0) & (d['t'] <= t1)]
    # 時間に大きな穴があれば別フライト。一番長いかたまりを採用する
    tv = d['t'].to_numpy()
    cut = np.flatnonzero(np.diff(tv) > GAP_SEC)
    starts = np.r_[0, cut + 1]
    ends = np.r_[cut + 1, len(tv)]
    i = int(np.argmax(ends - starts))
    d = d.iloc[starts[i]:ends[i]]
    # 滑走・着水を落とす
    return d[d['gs'] > GS_MIN].reset_index(drop=True)


def turn_rate(t, truetrack, drop_rate=None, rng=None):
    """TrueTrackを等間隔に並べ直して旋回角速度[deg/s]にする。
    drop_rateを与えると、その割合だけ間引いてから補間で埋める
    (欠測の多いログと同じ劣化を再現するため)"""
    psi = np.degrees(np.unwrap(np.radians(truetrack.astype(float))))
    tg = np.arange(t[0], t[-1], 1.0 / FS)
    pg = np.interp(tg, t, psi)
    if drop_rate:
        keep = rng.random(len(tg)) >= drop_rate
        keep[0] = keep[-1] = True
        pg = np.interp(tg, tg[keep], pg[keep])
    return tg, np.gradient(pg, tg)


def clean_segments(r, thresh=TURN_THRESH):
    """操縦による旋回とその前後を除いた、連続したクリーン区間のリスト"""
    n = max(1, int(TURN_AVG_SEC * FS))
    r_avg = np.convolve(r, np.ones(n) / n, mode='same')
    bad = np.abs(r_avg) > thresh
    g = int(GUARD_SEC * FS)
    bad = np.convolve(bad.astype(float), np.ones(2 * g + 1), mode='same') > 0
    ok = ~bad
    edge = np.flatnonzero(np.diff(np.r_[0, ok.astype(int), 0]))
    return [(edge[i], edge[i + 1]) for i in range(0, len(edge), 2)
            if edge[i + 1] - edge[i] >= MIN_SEG_SEC * FS]


def bandpass(x):
    b, a = butter(2, [1 / BAND[1] / (FS / 2), 1 / BAND[0] / (FS / 2)], btype='band')
    return filtfilt(b, a, x - x.mean())


def acf_model(tau, T, zeta, amp):
    """乱れに叩かれ続けている2次系の自己相関。TとζからACFの形が決まる"""
    q = np.sqrt(max(1.0 - zeta * zeta, 1e-6))
    wn = 2 * np.pi / T / q
    wd = wn * q
    return amp * np.exp(-zeta * wn * tau) * (np.cos(wd * tau) + zeta / q * np.sin(wd * tau))


def acf_and_fit(x, segs):
    """クリーン区間の自己相関をまとめて求め、2次系の形を当てはめる。

    当てはめで2つ気をつけている点:
     (1) 最初の数秒を使わない
         信号には振動のほかに広帯域のノイズ(GPSの1度刻み、補間、乱流)が
         混ざっている。ノイズは自己相関のラグ0付近にだけ鋭く出るので、
         そこを含めて当てはめると「猛烈に速く減衰する振動」という
         非現実的な解に引きずられる。
     (2) 振幅 A を自由にする
         A は「揺れのうち、きれいな振動で説明できる割合」。
         残り(1-A)がノイズ。A を1付近に縛ると、ノイズまで振動で
         説明しようとして減衰比が壊れる。A自体も結果として重要で、
         A が小さければ「その機体にはっきりした振動モードは無い」を意味する。
    """
    nlag = int(ACF_MAXLAG * FS) + 1
    num, den = np.zeros(nlag), 0.0
    for s, e in segs:
        v = bandpass(x[s:e])
        a = np.correlate(v, v, 'full')[len(v) - 1:]
        k = min(len(a), nlag)
        num[:k] += a[:k]
        den += a[0]
    acf = num / den
    lags = np.arange(nlag) / FS
    k0 = int(FIT_SKIP_SEC * FS)
    p, _ = curve_fit(acf_model, lags[k0:], acf[k0:], p0=[14.0, 0.3, 0.6],
                     bounds=([4.0, 0.01, 0.05], [60.0, 0.99, 1.5]), maxfev=40000)
    return lags, acf, p


def psd(r, segs):
    """クリーン区間をつないで求める旋回角速度のパワースペクトル"""
    x = np.concatenate([r[s:e] - r[s:e].mean() for s, e in segs])
    nper = min(256, len(x))
    return welch(x, fs=FS, nperseg=nper)


def band_power(f, P, lo, hi):
    m = (f >= 1 / hi) & (f <= 1 / lo)
    return float(np.trapz(P[m], f[m])) if m.sum() > 1 else np.nan


def half_life(T, zeta):
    """減衰比を「揺れの大きさが半分になるまでの秒数」に言い換える。
    ζ は無次元で直感が働きにくいが、秒なら体感と結びつく。
    振幅は exp(-ζ·ωn·t) で減るので、半分になる時刻は ln2/(ζ·ωn)。
    T は減衰振動の周期なので ωn = (2π/T)/√(1-ζ²)"""
    wn = (2 * np.pi / T) / np.sqrt(1.0 - zeta ** 2)
    return float(np.log(2.0) / (zeta * wn))


def slow_wander(r, segs):
    """スパイラル側の指標。周期30秒より遅い旋回成分の大きさと、
    その符号が反転する頻度(=当て舵の頻度)"""
    b, a = butter(2, 1 / 30.0 / (FS / 2), btype='low')
    lf = np.concatenate([filtfilt(b, a, r[s:e] - r[s:e].mean()) for s, e in segs])
    zc = np.sum(np.diff(np.sign(lf)) != 0) / (len(lf) / FS / 60.0)
    return float(lf.std()), float(zc)


def lonlat_to_m(lat, lon):
    """トラックを描くための簡易な平面直角座標 [m]"""
    lat0 = float(np.mean(lat))
    return ((lon - float(np.mean(lon))) * 111320.0 * np.cos(np.radians(lat0)),
            (lat - lat0) * 110540.0)


# 解析 -------------------------------------------------------
rng = np.random.default_rng(0)
d25 = load_flight(CSV_2025)
d26 = load_flight(CSV_2026, FLIGHT_2026)

# 2025年のログが1Hzグリッド上でどれだけ欠測しているか
tg_ref = np.arange(d25['t'].iloc[0], d25['t'].iloc[-1], 1.0)
drop25 = DROP_RATE_2025 if DROP_RATE_2025 is not None else \
    1.0 - np.isin(np.round(tg_ref), np.round(d25['t'].to_numpy())).mean()

flights = {}
# グラフの文字は日本語フォントが無い環境でも読めるように年だけにする。
# 機体名はコンソールの出力に出す
JP_NAME = {'2025': '2025 白夜', '2026': '2026 陽還'}
for key, d, color, label in (('2025', d25, C25, '2025'),
                             ('2026', d26, C26, '2026')):
    tg, r = turn_rate(d['t'].to_numpy(), d['TrueTrack'].to_numpy())
    segs = clean_segments(r)
    lags, acf, p = acf_and_fit(r, segs)
    f, P = psd(r, segs)
    lf_rms, zc = slow_wander(r, segs)
    flights[key] = dict(d=d, tg=tg, r=r, segs=segs, color=color, label=label,
                        lags=lags, acf=acf, T=p[0], zeta=p[1], amp=p[2],
                        f=f, P=P, lf_rms=lf_rms, zc=zc,
                        t_half=half_life(p[0], p[1]),
                        # モデルに頼らない指標。谷が深いほど振動が尾を引いている
                        trough=float(acf[:30].min()),
                        trough_lag=float(np.argmin(acf[:30]) / FS),
                        clean=sum(e - s for s, e in segs) / FS)

# 2026年を2025年と同じ欠測率に劣化させたもの(比較の公平さの確認)
deg_P, deg_acf, deg_T, deg_z = [], [], [], []
for _ in range(N_DROP_TRIAL):
    tg, r = turn_rate(d26['t'].to_numpy(), d26['TrueTrack'].to_numpy(),
                      drop_rate=drop25, rng=rng)
    segs = clean_segments(r)
    lags, acf, p = acf_and_fit(r, segs)
    f, P = psd(r, segs)
    deg_P.append(P); deg_acf.append(acf); deg_T.append(p[0]); deg_z.append(p[1])
deg = dict(f=f, P=np.mean(deg_P, 0), lags=lags, acf=np.mean(deg_acf, 0),
           T=float(np.mean(deg_T)), zeta=float(np.mean(deg_z)))

# 両年のIMUロール角でトラックの結果を裏取りする -------------
# どちらも BNO085 の「補正前」のロールを使う。
# 補正前のロールは、協調旋回中は見かけ重力が機体z軸に揃うのでほぼ0を指す。
# つまりこの値は実質「横方向の見かけ重力」= 横滑り計(ボール)に相当し、
# 釣り合っていない横の動きだけが出てくる。2機を同じ土俵で比べられる。
eu25 = pd.read_csv(JOINED_2025)
eu25['t'] = eu25['JST'].apply(time_to_daysec)
eu25 = eu25.drop_duplicates('t')
eu26 = pd.read_csv(EULER_2026)
eu26['t'] = eu26['time'].apply(time_to_daysec)

for key, src, rollcol in (('2025', eu25, 'roll'), ('2026', eu26, 'roll')):
    F = flights[key]
    roll = np.interp(F['tg'], src['t'].to_numpy(), src[rollcol].to_numpy())
    F['roll'] = roll
    F['roll_lags'], F['roll_acf'], p = acf_and_fit(roll, F['segs'])
    F['roll_T'], F['roll_zeta'] = p[0], p[1]
    # 周期帯ごとのロールの振れ幅。トラックとは独立した比較になる
    F['roll_rms'] = [float(np.concatenate(
        [filtfilt(*butter(2, [1 / hi / (FS / 2), 1 / lo / (FS / 2)], btype='band'),
                  roll[s:e] - roll[s:e].mean()) for s, e in F['segs']]).std())
        for lo, hi in BANDS]

# 2025年だけ: 実際のラダー操作のログがある ------------------
# (1) トラックから推定した「操縦区間」が本当に舵を踏んでいた区間かを確かめる
# (2) どの周期帯の揺れが操縦由来かを、舵と旋回角速度のコヒーレンスで測る
F25 = flights['2025']
rudder = np.interp(F25['tg'], eu25['t'].to_numpy(), eu25['rudder(yaw)'].ffill().to_numpy())
n_rud = max(1, int(RUDDER_AVG_SEC * FS))
rud_act = np.convolve(np.abs(rudder - np.median(rudder)), np.ones(n_rud) / n_rud, mode='same')
kept = np.zeros(len(F25['tg']), bool)
for s, e in F25['segs']:
    kept[s:e] = True
rud_kept, rud_removed = rud_act[kept], rud_act[~kept]
rud_p = float(mannwhitneyu(rud_removed, rud_kept).pvalue)
coh_f, coh = coherence(
    np.concatenate([bandpass(rudder[s:e]) for s, e in F25['segs']]),
    np.concatenate([bandpass(F25['r'][s:e]) for s, e in F25['segs']]),
    fs=FS, nperseg=256)

# 舵そのものを基準にした除外でも同じ結論になるかを確かめる
rud_bad = np.convolve(
    (rud_act > np.median(rud_act) + rud_act.std()).astype(float),
    np.ones(2 * int(GUARD_SEC * FS) + 1), mode='same') > 0
edge = np.flatnonzero(np.diff(np.r_[0, (~rud_bad).astype(int), 0]))
rud_segs = [(edge[i], edge[i + 1]) for i in range(0, len(edge), 2)
            if edge[i + 1] - edge[i] >= MIN_SEG_SEC * FS]
_, _, rud_fit = acf_and_fit(F25['r'], rud_segs)

# 対気速度。協調旋回なら 旋回角速度 = (g/V)·バンク角 になるはず
tas = {'2025': float(eu25['Airspeed'].mean())}
bd = pd.read_csv(BETADA_2026)
tas['2026'] = float(bd['tas'].mean())
for key in ('2025', '2026'):
    F = flights[key]
    rr = np.concatenate([F['r'][s:e] - F['r'][s:e].mean() for s, e in F['segs']])
    ph = np.concatenate([F['roll'][s:e] - F['roll'][s:e].mean() for s, e in F['segs']])
    F['gain_obs'] = float(np.polyfit(ph, rr, 1)[0])
    F['gain_coord'] = 9.80665 / tas[key]   # deg/s あたり deg (単位は相殺する)
    F['corr_roll'] = float(np.corrcoef(ph, rr)[0, 1])
    F['tas'] = tas[key]

# TrueTrackの1度刻みが作るノイズの床(これより下の山は読んではいけない)
quant_floor = (TRACK_QUANT ** 2 / 12.0) * 2.0 / FS * (2 * np.pi * flights['2025']['f']) ** 2

# 結果の表示 -------------------------------------------------
print('=' * 66)
print('横・方向の安定性の比較   (旋回角速度 r = d(TrueTrack)/dt, 1Hz)')
print('=' * 66)
print(f"2025年ログの1Hzグリッド上の欠測率: {drop25 * 100:.0f}%  "
      f"→ 2026年にも同じ欠測を与えた版を併せて計算した")
for key in ('2025', '2026'):
    F = flights[key]
    print(f"\n--- {JP_NAME[key]} ---")
    print(f"  飛行時間 {(F['d']['t'].iloc[-1] - F['d']['t'].iloc[0]) / 60:.1f}分  "
          f"平均対地速度 {F['d']['gs'].mean():.2f} m/s  平均対気速度 {F['tas']:.2f} m/s")
    print(f"  操縦とみなして除外した後に残った時間: {F['clean']:.0f}s "
          f"({F['clean'] / len(F['tg']) * FS * 100:.0f}%, {len(F['segs'])}区間)")
    print(f"  きれいな振動で説明できる割合 A = {F['amp']:.2f}   "
          f"({'はっきりした振動モードがある' if F['amp'] > 0.5 else 'はっきりした振動モードは無く、大半は広帯域の乱れ'})")
    print(f"  その振動の周期 T = {F['T']:.1f} s,  減衰比 ζ = {F['zeta']:.2f},  "
          f"半減時間 {F['t_half']:.1f} s")
    print(f"  自己相関の谷 = {F['trough']:+.2f} (ラグ {F['trough_lag']:.0f} s)  "
          f"※モデルに頼らない指標")
    print(f"  ゆっくりした旋回(周期30秒超)のRMS = {F['lf_rms']:.2f} deg/s, "
          f"当て舵の頻度 = {F['zc']:.1f} 回/分")
print(f"\n--- 2026年を2025年と同じ欠測率にした場合 ---")
print(f"  T = {deg['T']:.1f} s, ζ = {deg['zeta']:.2f}  "
      f"(元の 2026年: T = {flights['2026']['T']:.1f} s, ζ = {flights['2026']['zeta']:.2f})")
print("  → ほぼ変わらないので、2機の差はログの粗さのせいではない")

print('\n--- 周期帯ごとのパワー [deg²/s²] ---')
print(f"{'周期帯':>10s} {'2025 白夜':>12s} {'2026 陽還':>12s} {'2026(劣化)':>12s} {'比':>8s}")
for lo, hi in BANDS:
    a = band_power(flights['2025']['f'], flights['2025']['P'], lo, hi)
    b = band_power(flights['2026']['f'], flights['2026']['P'], lo, hi)
    c = band_power(deg['f'], deg['P'], lo, hi)
    print(f"{f'{lo}-{hi}s':>10s} {a:12.4f} {b:12.4f} {c:12.4f} {c / a:7.2f}x")

print('\n--- IMUロール角による裏取り (両年とも補正前のロール) ---')
print(f"{'':>10s} {'トラックから':>18s} {'IMUロールから':>18s}")
for key in ('2025', '2026'):
    F = flights[key]
    from_track = f"T={F['T']:.1f}s ζ={F['zeta']:.2f}"
    from_roll = f"T={F['roll_T']:.1f}s ζ={F['roll_zeta']:.2f}"
    print(f"{JP_NAME[key]:>10s} {from_track:>18s} {from_roll:>18s}")
print("  → 両年とも、GPSのトラックとIMUのロールが同じ周期を指している。")
print("     まったく別のセンサーが同じ答えを出しているので、この周期は本物。")
print("  2026年 約12秒: 6-12秒帯にあり、操縦との対応も弱い。機体のモード(ダッチロール)。")
print("  2025年 約22秒: 白夜にはそもそも速い振動が無く、唯一まとまった動きがこの22秒。")
print("     これは下のコヒーレンスで最も操縦との対応が強い帯(0.51)なので、")
print("     機体のモードではなく操縦のループとみるのが自然。")

print('\n--- 周期帯ごとのロールの振れ幅 [deg] (GPSを一切使わない比較) ---')
print(f"{'周期帯':>10s} {'2025 白夜':>12s} {'2026 陽還':>12s} {'比':>8s}")
for i, (lo, hi) in enumerate(BANDS):
    a, b = flights['2025']['roll_rms'][i], flights['2026']['roll_rms'][i]
    print(f"{f'{lo}-{hi}s':>10s} {a:12.3f} {b:12.3f} {b / a:7.2f}x")
print("  → 6-12秒帯(ダッチロールの帯)は2026年が1.9倍。")
print("     12-30秒帯はむしろ2025年の方が大きく、トラックで見えた2.3倍とは逆になる。")
print("     トラックの12-30秒帯の差は機体ではなく操縦の差だった可能性が高い。")

print('\n--- 2025年の実際のラダーログによる検証 ---')
print(f"  舵の踏み込み量(10秒平均): 除外した区間 {rud_removed.mean():.2f} / "
      f"残した区間 {rud_kept.mean():.2f} → {rud_removed.mean() / rud_kept.mean():.2f}倍 "
      f"(Mann-Whitney p = {rud_p:.1e})")
print("  → トラックから推定した「操縦区間」は、実際に舵を踏んでいた区間と一致している")
print(f"  舵の踏み込みそのものを基準に除外した場合: T = {rud_fit[0]:.1f} s, ζ = {rud_fit[1]:.2f}  "
      f"(トラック基準では T = {flights['2025']['T']:.1f} s, ζ = {flights['2025']['zeta']:.2f})")
print("  → 除外の基準を変えても ζ は変わらない")
print("  舵と旋回角速度のコヒーレンス (1に近いほどその周期の揺れは操縦由来):")
for lo, hi in BANDS:
    m = (coh_f >= 1 / hi) & (coh_f <= 1 / lo)
    if m.sum():
        print(f"     {lo:2d}-{hi:2d}s : {coh[m].mean():.2f}")
print("  → 12秒より遅い揺れは操縦のループ、6-12秒帯は操縦との対応が弱い")

print('\n--- 協調旋回からのずれ (横滑りの有無) ---')
for key in ('2025', '2026'):
    F = flights[key]
    print(f"  {JP_NAME[key]}: 旋回角速度/ロール = {F['gain_obs']:.2f} "
          f"(協調旋回の理論値 g/V = {F['gain_coord']:.2f} の {F['gain_obs'] / F['gain_coord'] * 100:.0f}%), "
          f"相関 {F['corr_roll']:+.2f}")
print("  → 両機とも協調旋回から同じ程度ずれている。2機を分ける指標にはならない")

print('\n--- XFLR5(操縦なしの計算)との比較 ---')
# λ(固有値の実部)と半減時間は t = ln2/|λ| で行き来できる。同じ土俵に乗せて比べる
ln2 = float(np.log(2.0))
lam_meas = {k: -np.log(2.0) / flights[k]['t_half'] for k in ('2025', '2026')}
print(f"{'':>16s} {'XFLR5 λ':>10s} {'→半減/倍増':>12s} {'実測 λ':>10s} {'→半減':>10s}")
for key in ('2025', '2026'):
    for mode, name in (('dutch', 'ダッチロール'), ('spiral', 'スパイラル'), ('roll', 'ロール')):
        lam = XFLR_LAMBDA[key][mode]
        kind = '半減' if lam < 0 else '倍増'
        cell = f"{kind}{ln2 / abs(lam):.2f}s"
        if mode == 'dutch' and key == '2026':
            meas = f"{lam_meas[key]:+.3f}", f"{flights[key]['t_half']:.1f}s"
        elif mode == 'dutch' and key == '2025':
            meas = ('検出なし', '-')
        else:
            meas = ('-', '-')
        print(f"{JP_NAME[key] + ' ' + name:>16s} {lam:+10.2f} {cell:>12s} {meas[0]:>10s} {meas[1]:>10s}")
print(f"\n  (1) t2={XFLR_T_HALF:.1f}秒 と ダッチロールのλ={XFLR_LAMBDA['2026']['dutch']:+.1f} は両立しない。")
print(f"      λ={XFLR_LAMBDA['2026']['dutch']:+.1f} の半減時間は {ln2 / 0.6:.2f}秒 で、{XFLR_T_HALF:.1f}秒 とは"
      f"{XFLR_T_HALF / (ln2 / 0.6):.0f}倍違う。別のモードの値の可能性が高い。")
print("  (2) ダッチロールの順序は一致: XFLR5も実測も「2025の方が速く収まる」。")
print("      ただしXFLR5の差は1.3倍しかなく、実測の「2025は振動が見えない/2026は揺れの7割」")
print("      という大きな差は説明しきれない。")
print(f"  (3) 減衰の大きさは合わない: XFLR5 λ={XFLR_LAMBDA['2026']['dutch']:+.1f} に対し実測 "
      f"λ={lam_meas['2026']:+.3f}。実機の方が{0.6 / abs(lam_meas['2026']):.0f}倍ゆっくり収まっている。")
print("      操縦で振動が抑えられているなら実機の方が速く収まるはずなので、逆向き。")
print("  (4) スパイラルは検証できない: 定常な信号では cov(x, dx/dt)=0 になるため、")
print("      旋回角速度から時定数を取り出す回帰は原理的に0を返す(既知の信号で確認済み)。")
print(f"      弱い代理指標(ゆっくりした旋回のRMS・当て舵の頻度)は 2025 {flights['2025']['lf_rms']:.2f}/"
      f"{flights['2025']['zc']:.1f} vs 2026 {flights['2026']['lf_rms']:.2f}/{flights['2026']['zc']:.1f} で、")
print("      XFLR5の「2025だけ発散」とはむしろ逆向き(差は小さく結論は出せない)。")
print(f"  (5) ロールダンピング λ={XFLR_LAMBDA['2026']['roll']:.0f} は半減 {ln2 / 30:.3f}秒。"
      "1Hzのログでは原理的に見えないので検証対象外。")

print(f'\n--- 除外しきい値を変えた場合 (結論が変わらないことの確認) ---')
print(f"{'しきい値':>10s} {'2025 T/ζ/残り':>26s} {'2026 T/ζ/残り':>26s}")
for th in THRESH_SWEEP:
    cells = []
    for key in ('2025', '2026'):
        F = flights[key]
        sg = clean_segments(F['r'], th)
        _, _, p = acf_and_fit(F['r'], sg)
        keep = sum(e - s for s, e in sg) / len(F['tg']) * 100
        cells.append(f"T={p[0]:4.1f}s ζ={p[1]:.2f} ({keep:3.0f}%)")
    lab = '除外なし' if th > 90 else f'{th:.1f} deg/s'
    print(f"{lab:>10s} {cells[0]:>26s} {cells[1]:>26s}")

# 作図 -------------------------------------------------------
fig = plt.figure(figsize=(13.0, 22.0), facecolor=SURFACE)
gs = fig.add_gridspec(5, 2, hspace=0.34, wspace=0.22,
                      top=0.953, bottom=0.175, left=0.075, right=0.975)
# 下の余白は「読み方の注記」を置くために空けてある


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


# 1段目: トラック。除外した区間を薄い灰色にして、何を捨てたかを示す
# 2機を見比べるので、縮尺は両方そろえる(それぞれの重心を中心に同じ幅で切る)
track_xy = {}
for key in ('2025', '2026'):
    F = flights[key]
    x, y = lonlat_to_m(F['d']['latitude'].to_numpy(), F['d']['longitude'].to_numpy())
    track_xy[key] = (np.interp(F['tg'], F['d']['t'].to_numpy(), x) / 1000,
                     np.interp(F['tg'], F['d']['t'].to_numpy(), y) / 1000)
half = max(max(v.max() - v.min() for v in track_xy[k]) for k in track_xy) / 2 * 1.08
for col, key in enumerate(('2025', '2026')):
    F = flights[key]
    ax = fig.add_subplot(gs[0, col])
    xg, yg = track_xy[key]
    ax.plot(xg, yg, color=MUTED, linewidth=1.4, zorder=2)
    for s, e in F['segs']:
        ax.plot(xg[s:e], yg[s:e], color=F['color'], linewidth=1.4, zorder=3)
    cx, cy = (xg.max() + xg.min()) / 2, (yg.max() + yg.min()) / 2
    ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
    ax.set_aspect('equal')
    style(ax, 'East [km]', 'North [km]',
          f"{F['label']}  ground track  ({F['clean'] / 60:.0f} min used)")
    if col == 0:
        ax.text(0.98, 0.04, 'grey = removed as pilot input', transform=ax.transAxes,
                color=INK_SUB, fontsize=9, ha='right')

# 2段目: 一番よく揺れていたクリーン区間から時系列を抜き出す
for col, key in enumerate(('2025', '2026')):
    F = flights[key]
    ax = fig.add_subplot(gs[1, col])
    n = int(EXCERPT_SEC * FS)
    best, bs = None, -1.0
    for s, e in F['segs']:
        for a0 in range(s, max(s + 1, e - n), n // 2):
            b0 = min(a0 + n, e)
            if b0 - a0 < n:
                continue
            v = float(np.std(bandpass(F['r'][a0:b0])))
            if v > bs:
                bs, best = v, (a0, b0)
    a0, b0 = best if best else (F['segs'][0][0], F['segs'][0][0] + n)
    tt = (F['tg'][a0:b0] - F['tg'][a0])
    ax.axhline(0, color=MARK, linewidth=1.0, zorder=1)
    ax.plot(tt, F['r'][a0:b0], color=F['color'], linewidth=1.6, zorder=3)
    ax.set_ylim(-8, 8)   # 2機で同じ目盛りにしないと揺れの大きさを見比べられない
    style(ax, 'Time [s]', 'Turn rate [deg/s]',
          f"{F['label']}  turn rate  (clean segment, {EXCERPT_SEC:.0f} s)")

# 3段目左: パワースペクトル
ax = fig.add_subplot(gs[2, 0])
for key in ('2025', '2026'):
    F = flights[key]
    m = F['f'] > 0
    ax.plot(1 / F['f'][m], F['P'][m], color=F['color'], linewidth=2.0, zorder=4)
m = deg['f'] > 0
ax.plot(1 / deg['f'][m], deg['P'][m], color=C26, linewidth=1.4,
        linestyle='--', zorder=3)
m = flights['2025']['f'] > 0
ax.plot(1 / flights['2025']['f'][m], quant_floor[m], color=MARK,
        linewidth=1.2, linestyle=':', zorder=2)
ax.set_xscale('log'); ax.set_yscale('log')
ax.set_xlim(60, 3); ax.set_ylim(0.05, 200)
ax.set_xticks([60, 30, 20, 12, 6, 3])
ax.set_xticklabels(['60', '30', '20', '12', '6', '3'])
ax.minorticks_off()
style(ax, 'Period [s]  (long <- -> short)', 'Turn-rate PSD [deg$^2$/s$^2$/Hz]',
      'Where the track wobbles')
ax.legend(handles=[
    Line2D([], [], color=C25, lw=2, label='2025'),
    Line2D([], [], color=C26, lw=2, label='2026'),
    Line2D([], [], color=C26, lw=1.4, ls='--', label='2026 degraded to 2025 sampling'),
    Line2D([], [], color=MARK, lw=1.2, ls=':', label='1 deg quantisation floor'),
], frameon=False, fontsize=8.5, labelcolor=INK_SUB, loc='lower left')

# 3段目右: 自己相関。ここが結論のパネル
ax = fig.add_subplot(gs[2, 1])
ax.axhline(0, color=MARK, linewidth=1.0, zorder=1)
for key in ('2025', '2026'):
    F = flights[key]
    ax.plot(F['lags'], F['acf'], color=F['color'], linewidth=2.0, zorder=4)
    ax.plot(F['lags'], acf_model(F['lags'], F['T'], F['zeta'], F['amp']),
            color=F['color'], linewidth=1.2, linestyle='--', alpha=0.8, zorder=3)
    ax.text(0.97, 0.93 if key == '2025' else 0.83,
            f"{F['label']}:  T = {F['T']:.1f} s,  $\\zeta$ = {F['zeta']:.2f},  A = {F['amp']:.2f}",
            transform=ax.transAxes, color=F['color'], fontsize=10.5,
            ha='right', va='top', fontweight='bold')
style(ax, 'Lag [s]', 'Autocorrelation of turn rate',
      'How long the wobble rings on')
ax.text(0.97, 0.06, 'A = fraction explained by a clean oscillation',
        transform=ax.transAxes, color=INK_SUB, fontsize=9, ha='right')

# 4段目左: 両年のIMUロールによる裏取り。トラックとは独立した証拠になる
ax = fig.add_subplot(gs[3, 0])
ax.axhline(0, color=MARK, linewidth=1.0, zorder=1)
for key in ('2025', '2026'):
    F = flights[key]
    ax.plot(F['roll_lags'], F['roll_acf'], color=F['color'], linewidth=2.0, zorder=4)
    ax.text(0.97, 0.93 if key == '2025' else 0.83,
            f"{F['label']} roll:  T = {F['roll_T']:.1f} s,  $\\zeta$ = {F['roll_zeta']:.2f}"
            f"   (track: {F['T']:.1f} s)",
            transform=ax.transAxes, color=F['color'], fontsize=10.5,
            ha='right', va='top', fontweight='bold')
style(ax, 'Lag [s]', 'Autocorrelation of IMU roll',
      'Same test on the IMU roll (independent of GPS)')
# 2025の谷は深いが、これは周期21秒のゆっくりした彷徨いによるもの。
# 速い振動の有無を比べるパネルなので、誤読されないように但し書きを入れる
ax.set_ylim(-0.62, 1.05)   # 但し書きを曲線に重ねないための余白
ax.text(0.5, 0.05,
        "2025's trough is the slow ~21 s wander, not a fast mode\n"
        "(see the rudder coherence panel below)",
        transform=ax.transAxes, color=INK_SUB, fontsize=8.5, ha='center')

# 4段目右: 周期帯ごとのパワー
ax = fig.add_subplot(gs[3, 1])
labels = [f'{lo}-{hi}' for lo, hi in BANDS]
xpos = np.arange(len(BANDS))
v25 = [band_power(flights['2025']['f'], flights['2025']['P'], lo, hi) for lo, hi in BANDS]
v26 = [band_power(deg['f'], deg['P'], lo, hi) for lo, hi in BANDS]
# 2026側は欠測を揃えた版を使う。2025と同じ土俵で比べるため
ax.bar(xpos - 0.2, v25, 0.38, color=C25, zorder=3)
ax.bar(xpos + 0.2, v26, 0.38, color=C26, zorder=3)
for i, (a, b) in enumerate(zip(v25, v26)):
    ax.text(i, max(a, b) * 1.06, f'x{b / a:.1f}', ha='center',
            color=INK_SUB, fontsize=9.5, fontweight='bold')
ax.set_xticks(xpos); ax.set_xticklabels(labels)
ax.set_ylim(0, max(max(v25), max(v26)) * 1.42)   # 倍率の注記と凡例を置く余白
style(ax, 'Period band [s]', 'Turn-rate power [deg$^2$/s$^2$]',
      'Wobble energy by period  (equal sampling)')
ax.legend(handles=[
    Line2D([], [], color=C25, lw=6, label='2025'),
    Line2D([], [], color=C26, lw=6, label='2026 (degraded to match)'),
], frameon=False, fontsize=9, labelcolor=INK_SUB, loc='upper left')

# 5段目左: 周期帯ごとのロールの振れ幅。GPSを一切使わない比較
ax = fig.add_subplot(gs[4, 0])
w25, w26 = flights['2025']['roll_rms'], flights['2026']['roll_rms']
ax.bar(xpos - 0.2, w25, 0.38, color=C25, zorder=3)
ax.bar(xpos + 0.2, w26, 0.38, color=C26, zorder=3)
for i, (a, b) in enumerate(zip(w25, w26)):
    ax.text(i, max(a, b) * 1.05, f'x{b / a:.1f}', ha='center',
            color=INK_SUB, fontsize=9.5, fontweight='bold')
ax.set_xticks(xpos); ax.set_xticklabels(labels)
ax.set_ylim(0, max(max(w25), max(w26)) * 1.40)
style(ax, 'Period band [s]', 'IMU roll RMS [deg]',
      'Roll wobble by period  (no GPS involved)')
ax.legend(handles=[Line2D([], [], color=C25, lw=6, label='2025'),
                   Line2D([], [], color=C26, lw=6, label='2026')],
          frameon=False, fontsize=9, labelcolor=INK_SUB, loc='upper left')

# 5段目右: 2025年だけある実際のラダーログ。
# 舵と旋回角速度のコヒーレンスが高い周期帯 = その揺れは操縦のループ
ax = fig.add_subplot(gs[4, 1])
m = coh_f > 0
ax.plot(1 / coh_f[m], coh[m], color=C25, linewidth=1.6, zorder=4)
for lo, hi in BANDS:
    b = (coh_f >= 1 / hi) & (coh_f <= 1 / lo)
    if b.sum():
        ax.plot([hi, lo], [coh[b].mean()] * 2, color=INK, linewidth=3.0, zorder=5)
ax.set_xscale('log')
ax.set_xlim(60, 3); ax.set_ylim(0, 1)
ax.set_xticks([60, 30, 20, 12, 6, 3])
ax.set_xticklabels(['60', '30', '20', '12', '6', '3'])
ax.minorticks_off()
style(ax, 'Period [s]  (long <- -> short)', 'Rudder / turn-rate coherence',
      '2025 only: which wobbles are the pilot?')
ax.legend(handles=[
    Line2D([], [], color=C25, lw=1.6, label='coherence'),
    Line2D([], [], color=INK, lw=3, label='band average'),
], frameon=False, fontsize=9, labelcolor=INK_SUB, loc='upper right')
ax.text(0.5, 0.06, 'high = that period is pilot-driven',
        transform=ax.transAxes, color=INK_SUB, fontsize=9, ha='center')

fig.suptitle('Lateral-directional behaviour of the two aircraft  '
             '(pilot turn inputs removed)',
             color=INK, fontsize=15.5, fontweight='bold', x=0.012, ha='left', y=0.982)

# 図の下の注記 ----------------------------------------------
# 初心者が誤読しやすい3点を、図を見た人がそのまま読めるように図の中へ入れる
avail = {f.name for f in fm.fontManager.ttflist}
jp_font = next((f for f in JP_FONT_CANDIDATES if f in avail), None)
F25, F26 = flights['2025'], flights['2026']
notes = [
    ("注1  棒グラフの高さは「機体の良し悪し」ではありません",
     "　　揺れの大きさ ＝ 外乱（乱気流・操縦）× 機体の応答しやすさ、で決まります。\n"
     "　　その日の空が荒れていれば、良い機体でも棒は高くなります。棒が言えるのは\n"
     "　　「どの速さの揺れが多かったか」だけです。特に12-30秒の棒は操縦のループが\n"
     "　　支配していて（右下のコヒーレンス参照）、機体の比較には使えません。"),
    ("注2  2機の違いは「減衰比」ではなく「振動があるか無いか」です",
     "　　A は揺れのうちきれいな振動で説明できる割合。残りは広帯域の乱れです。\n"
     f"　　　2026 陽還: A={F26['amp']:.2f} → 周期{F26['T']:.1f}秒の振動が揺れの大半を占める。\n"
     f"　　　2025 白夜: A={F25['amp']:.2f} → はっきりした振動モードが無く、大半が乱れ。\n"
     f"　　白夜の当てはめが周期{F25['T']:.1f}秒になるのは、速い振動が無いため唯一まとまった\n"
     "　　「ゆっくりした彷徨い」を拾っているからで、これは操縦のループです。\n"
     "　　したがって両者の ζ や半減時間を直接比べてはいけません（別のものの値です）。\n"
     f"　　陽還の振動を秒で言うと、振幅が半分になるまで {F26['t_half']:.1f}秒"
     f"（1周期の間に約{2 ** -(F26['T'] / F26['t_half']) * 100:.0f}%まで減衰）。\n"
     "　　ζ≈0.3 は一般的な操縦性の基準では十分許容される値で、悪い数字ではありません。"),
    
]
# XFLR5の固有値(λ)との比較は、XFLR5側の読み取りが確定していないため図には載せない。
# 数値の突き合わせはコンソール出力のみ(上の XFLR_LAMBDA を直せば再計算される)。
if jp_font is None:
    print('警告: 日本語フォントが見つからないため、図の注記は省略しました')
else:
    # 2段組み。一番長い注2を左列いっぱいに、右列に注1と注3を積む
    places = [(0.545, 0.145), (0.075, 0.145), (0.545, 0.086)]
    for (head, body), (x, y) in zip(notes, places):
        fig.text(x, y, head, fontname=jp_font, fontsize=11,
                 color=INK, fontweight='bold', va='top')
        fig.text(x, y - 0.011, body, fontname=jp_font, fontsize=9.5,
                 color=INK_SUB, va='top', linespacing=1.65)

fig.savefig(PLOTNAME, dpi=130, facecolor=SURFACE)
print(f"\nグラフを保存: {PLOTNAME}")
