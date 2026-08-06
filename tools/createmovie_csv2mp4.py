import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
import matplotlib.patheffects as path_effects
from scipy.interpolate import interp1d
from scipy.optimize import linprog
from scipy.signal import butter, filtfilt

# Last update 2026/7/27

FILENAME = '2026-07-26_0804.csv'
EULER_FILENAME = '20260726_compensated.txt'  # flight_preprocess.py の出力(補正済み姿勢角)
LOG_FILENAME = 'log_edit.txt'  # システムログ。電圧・GPS精度・衛星数をCSVと同期表示する
BETA_DA_FILENAME = 'beta_da_20260726.txt'  # flight_preprocess.py の出力(β・偏流角・風・TAS)
SAVENAME = '0726.mp4'
LOGDATE = "2026-07-11"
starttime = "08:21:15"
stoptime = "08:24:15"
#stoptime = "09:07:48"

kf_alt_offset = 83.5

# 高度の合成と補正 -------------------------------------------
# KF_Altitude は気圧高度に加えてGNSS高度も融合しているため、
# 短時間の動きは滑らかだが、GNSSの鉛直誤差が数分スケールでゆっくり効いて
# 長時間では大きくずれる。一方CSVの pressure から計算した気圧高度は、
# 短時間はノイズが多いものの長時間では安定している。
# (実データでの変化率の標準偏差は KF=0.21、気圧高度=0.61 m/s。
#  逆に長時間側は、気圧高度なら着水時の気圧を0m基準にするだけで
#  離陸時がほぼプラットホーム高さに一致する)
#
# そこで両者の良いところを取る:
#   合成高度 = 気圧高度のゆっくりした成分 + KF_Altitudeの速い成分
# 位相のずれが出ないよう、ローパスは前後両方向に掛けている(filtfilt)。
# なおKF側は速い成分しか使わないため、kf_alt_offset の値は結果に影響しない。
KF_ALT_FUSE_BARO = True
KF_ALT_FUSE_TAU = 30.0     # 合成の時定数 [秒]。これより遅い変化は気圧高度に従う

# 合成しても、飛行中の高度が現地で分かっている範囲に収まるとは限らない。
# そこで「高度の上下限」と「着水時は0m」を条件として与え、
# それを満たす中で最も滑らかな(不自然に曲がらない)ドリフト曲線を求めて差し引く。
# 曲がり具合(2階差分)の合計が最小になる曲線を線形計画法で解いている。
# 補正後の高度は動画では "Alt (KF_C)" として表示する。
KF_ALT_CORRECT = True
KF_ALT_TAKEOFF_TIME = '08:21:26.30'  # 離陸時刻。ここでは補正量0(表示はプラットホーム高さ)
KF_ALT_LAND_TIME = '09:07:42.00'     # 着水時刻
KF_ALT_LAND_TARGET = 0.0             # 着水時に表示されるべき高度 [m]
KF_ALT_MIN = 0.0                     # 飛行中の高度の下限 [m] (水面。これより下はあり得ない)
KF_ALT_MAX = 13.0                    # 飛行中の高度の上限 [m]
# ドリフト曲線の折れ点の間隔 [分]。粗いほど滑らかだが条件を満たせないことがある。
# 先頭から順に試し、条件を満たせた最初の(最も滑らかな)ものを使う
KF_ALT_KNOT_CANDIDATES = [10.0, 7.0, 5.0, 4.0, 3.0, 2.0, 1.5, 1.0]


# 風向計の設定 -----------------------------------------------
# コンパス右上に置く小さな風向計。矢印は風が吹いていく向きを指し、
# 長さは風速に比例する(WIND_ARROW_FULL で最大長になる)
WIND_CX, WIND_CY = 0.75, 0.30   # 風向計の中心(tt_axの座標系)
# 無風に近くても向きが見えるよう最低長を持たせ、風速に応じて伸ばす
WIND_ARROW_MIN = 0.22           # 最小の矢印の長さ
WIND_ARROW_LEN = 0.30           # 風速に比例して伸びる分の長さ
WIND_ARROW_FULL = 4.0           # この風速[m/s]で最大長になる

# 地図表示の設定 ---------------------------------------------
MAP_ZOOM = 0.9        # 地図の拡大率。1.0が基準で、2.0にすると2倍ズーム
MAP_OFFSET_LON = 0.0  # 地図中心の経度オフセット [度] (+で東へずれる)
MAP_OFFSET_LAT = 0.0  # 地図中心の緯度オフセット [度] (+で北へずれる)

# Function to convert time to seconds since the start time
# 新CSVフォーマットは秒に小数が含まれる(例 07:57:22.50)ため、秒はfloatで解析する
def time_to_seconds(time_str, start_time_str):
    start_h, start_m, start_s = start_time_str.split(':')
    h, m, s = time_str.split(':')
    start_total = int(start_h) * 3600 + int(start_m) * 60 + float(start_s)
    total = int(h) * 3600 + int(m) * 60 + float(s)
    return total - start_total

# Data loading and preprocessing
data = pd.read_csv(FILENAME)
data['time_seconds'] = data['time'].apply(lambda x: time_to_seconds(x, starttime))

# 地図を正しい縦横比で描くため、経度に cos(36°) を掛けて距離スケールを緯度に合わせる。
# ただしこれは描画専用の値なので、表示用に元の経度を別列に残しておく
# (この2つを混同すると、動画に実際とは違う経度が表示されてしまう)
data['longitude_true'] = data['longitude']
data['longitude'] = data['longitude'] * 0.8090169  # *cos(36deg)

# KF_Altitudeはオフセットがあるため、最初の行の値を0点とする相対高度に変換
data['KF_Alt_rel'] = data['KF_Altitude'] - kf_alt_offset

# 気圧高度とKF高度の合成 -------------------------------------
if KF_ALT_FUSE_BARO:
    _ts_all = data['time_seconds'].to_numpy()
    _t_land_b = time_to_seconds(KF_ALT_LAND_TIME, starttime)
    # 着水時の気圧を基準にして、気圧から高度を計算する(国際標準大気の式)
    _p = data['pressure'].to_numpy()
    _p_ref = float(np.interp(_t_land_b, _ts_all, _p))
    _baro = 44330.0 * (1.0 - (_p / _p_ref) ** (1.0 / 5.255)) + KF_ALT_LAND_TARGET

    def _lowpass(v, tau):
        """位相がずれないローパス(前後両方向に掛ける)"""
        dt = float(np.median(np.diff(_ts_all)))
        wn = (1.0 / (2.0 * np.pi * tau)) / (0.5 / dt)  # ナイキスト周波数で正規化
        b, a = butter(2, min(wn, 0.99), btype='low')
        return filtfilt(b, a, v)

    # ゆっくりした成分は気圧高度から、速い成分はKF高度から取る
    _kf = data['KF_Alt_rel'].to_numpy()
    data['KF_Alt_rel'] = (_lowpass(_baro, KF_ALT_FUSE_TAU)
                          + _kf - _lowpass(_kf, KF_ALT_FUSE_TAU))
    print(f"高度の合成(時定数{KF_ALT_FUSE_TAU:.0f}秒): "
          f"気圧の基準{_p_ref:.2f}hPa、合成後の全体範囲"
          f"{data['KF_Alt_rel'].min():+.1f}〜{data['KF_Alt_rel'].max():+.1f}m")

def solve_alt_drift(ts, alt, t_takeoff, t_land, knot_min):
    """高度が KF_ALT_MIN〜KF_ALT_MAX に収まり、離陸時は補正0、
    着水時にちょうど KF_ALT_LAND_TARGET になるようなドリフト曲線のうち、
    最も滑らかなものを線形計画法で求める。求まらなければ None を返す。

    未知数は折れ点ごとのドリフト量。曲がり具合(2階差分)の絶対値の合計を
    最小化するので、条件を満たす範囲で余計な補正をしない解が選ばれる。"""
    n_k = int(np.ceil((t_land - t_takeoff) / (knot_min * 60.0))) + 1
    if n_k < 3:
        return None, None
    knots = np.linspace(t_takeoff, t_land, n_k)
    sel = (ts >= t_takeoff) & (ts <= t_land)
    t_in, a_in = ts[sel], alt[sel]
    # 各サンプルのドリフトを折れ点の線形補間で表す重み行列
    step = knots[1] - knots[0]
    idx = np.clip(((t_in - t_takeoff) / step).astype(int), 0, n_k - 2)
    w = (t_in - knots[idx]) / step
    W = np.zeros((len(t_in), n_k))
    rows = np.arange(len(t_in))
    W[rows, idx] = 1.0 - w
    W[rows, idx + 1] = w
    # 2階差分(曲がり具合)。絶対値を補助変数uで表して線形計画に載せる
    n_u = n_k - 2
    D2 = np.zeros((n_u, n_k))
    for k in range(n_u):
        D2[k, k], D2[k, k + 1], D2[k, k + 2] = 1.0, -2.0, 1.0
    pad = lambda m: np.hstack([m, np.zeros((m.shape[0], n_u))])
    A_ub = np.vstack([pad(W), pad(-W),
                      np.hstack([D2, -np.eye(n_u)]), np.hstack([-D2, -np.eye(n_u)])])
    b_ub = np.concatenate([a_in - KF_ALT_MIN, -(a_in - KF_ALT_MAX), np.zeros(2 * n_u)])
    # 離陸時のドリフトは0(既存のkf_alt_offsetの校正をそのまま活かす)、
    # 着水時のドリフトは表示が KF_ALT_LAND_TARGET になる値に固定する
    A_eq = np.zeros((2, n_k + n_u))
    A_eq[0, 0], A_eq[1, n_k - 1] = 1.0, 1.0
    b_eq = [0.0, float(np.interp(t_land, ts, alt)) - KF_ALT_LAND_TARGET]
    res = linprog(np.concatenate([np.zeros(n_k), np.ones(n_u)]), A_ub, b_ub, A_eq, b_eq,
                  bounds=[(None, None)] * n_k + [(0, None)] * n_u, method='highs')
    if not res.success:
        return None, None
    return knots, res.x[:n_k]


# 気圧ドリフトを補正する。折れ点の間隔を粗い方から試し、
# 高度の上下限を満たせた最初の(最も滑らかな)解を採用する
if KF_ALT_CORRECT:
    _t_takeoff = time_to_seconds(KF_ALT_TAKEOFF_TIME, starttime)
    _t_land = time_to_seconds(KF_ALT_LAND_TIME, starttime)
    _ts = data['time_seconds'].to_numpy()
    _alt = data['KF_Alt_rel'].to_numpy()
    for _knot_min in KF_ALT_KNOT_CANDIDATES:
        _knots, _drift = solve_alt_drift(_ts, _alt, _t_takeoff, _t_land, _knot_min)
        if _knots is not None:
            break
    if _knots is None:
        raise ValueError("高度の上下限を満たすドリフト曲線が見つかりません。"
                         "KF_ALT_MIN/KF_ALT_MAX や離着水時刻を見直してください")
    data['KF_Alt_c'] = data['KF_Alt_rel'] - np.interp(_ts, _knots, _drift)
    _fl = (_ts >= _t_takeoff) & (_ts <= _t_land)
    print(f"KF高度の気圧ドリフト補正: 折れ点{_knot_min:.0f}分間隔({len(_knots)}点)で解けました。"
          f"ドリフト量{_drift.min():+.2f}〜{_drift.max():+.2f}m")
    print(f"  補正後の飛行中の高度: {data['KF_Alt_c'][_fl].min():+.2f}〜"
          f"{data['KF_Alt_c'][_fl].max():+.2f}m "
          f"(離陸時{np.interp(_t_takeoff, _ts, data['KF_Alt_c']):+.1f}m, "
          f"着水時{np.interp(_t_land, _ts, data['KF_Alt_c']):+.2f}m)")
else:
    data['KF_Alt_c'] = data['KF_Alt_rel']

# 姿勢角データの読み込み。CSVと同じ開始時刻を基準に秒へ変換して同期する
euler = pd.read_csv(EULER_FILENAME)
euler['time_seconds'] = euler['time'].apply(lambda x: time_to_seconds(x, starttime))

# β・偏流角・風・TASの推定結果を読み込む(flight_preprocess.py の出力)。
# 飛行区間のみのデータなので、離着陸前後は端の値がそのまま伸びる点に注意
betada = pd.read_csv(BETA_DA_FILENAME)
betada['time_seconds'] = betada['time'].apply(lambda x: time_to_seconds(x, starttime))

# システムログ(log.txt)の解析 --------------------------------
# 各行は「起動後秒数:メッセージ」形式。GPS TIME行(UTC)を+9時間でJSTに変換し、
# 起動後秒数→動画基準の秒数へ対応付ける。
# ファイル先頭に再起動前のログが残っていることがあるため、"0:SD INIT"を見たら読み直す
log_volt_rows = []   # (起動後秒数, 電圧V)
log_acc_rows = []    # (起動後秒数, hAcc, vAcc, sAcc, sats)
log_anchor_sec = None    # GPS TIME行の起動後秒数
log_anchor_jst = None    # その時点のJST時刻(0時からの秒数)
with open(LOG_FILENAME) as f:
    for line in f:
        if line.startswith('0:SD INIT'):
            log_volt_rows.clear()
            log_acc_rows.clear()
            log_anchor_sec = None
            log_anchor_jst = None
            continue
        m = re.match(r'(\d+):GPS TIME: \d+-\d+-\d+ (\d+):(\d+):(\d+) UTC', line)
        if m:
            log_anchor_sec = int(m.group(1))
            utc_sec = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4))
            log_anchor_jst = (utc_sec + 9 * 3600) % 86400  # UTC+9h、日付跨ぎは剰余で処理
            continue
        m = re.match(r'(\d+):volt=([\d.]+)V', line)
        if m:
            log_volt_rows.append((int(m.group(1)), float(m.group(2))))
            continue
        m = re.match(r'(\d+):ACC hAcc=([\d.]+)m vAcc=([\d.]+)m sAcc=([\d.]+)m/s fix=\d+ sats=(\d+)', line)
        if m:
            log_acc_rows.append((int(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), int(m.group(5))))

if log_anchor_sec is None:
    raise ValueError(f"{LOG_FILENAME} にGPS TIME行が見つかりません")

# 起動後秒数を動画基準の秒数(starttimeからの経過秒)へ変換
starttime_abs = time_to_seconds(starttime, "00:00:00")
def log_sec_to_video_seconds(sec):
    return (log_anchor_jst - starttime_abs) + (sec - log_anchor_sec)

log_volt_t = [log_sec_to_video_seconds(r[0]) for r in log_volt_rows]
log_acc_t = [log_sec_to_video_seconds(r[0]) for r in log_acc_rows]

# 地図の表示中心と表示幅を計算(MAP_ZOOM/MAP_OFFSETをここで反映)
# 経度はcos(36deg)でスケール済みのため、オフセットも同じ係数を掛けて合わせる
map_center_lon = (data['longitude'].min() + data['longitude'].max()) / 2 + MAP_OFFSET_LON * 0.8090169
map_center_lat = (data['latitude'].min() + data['latitude'].max()) / 2 + MAP_OFFSET_LAT
map_half_lon = ((data['longitude'].max() - data['longitude'].min()) / 2) / MAP_ZOOM
map_half_lat = ((data['latitude'].max() - data['latitude'].min()) / 2) / MAP_ZOOM

def set_map_limits(ax):
    ax.set_xlim(map_center_lon - map_half_lon, map_center_lon + map_half_lon)
    ax.set_ylim(map_center_lat - map_half_lat, map_center_lat + map_half_lat)

# Interpolation setup
start = 0
stop = time_to_seconds(stoptime, starttime)
frame_interval = 0.25
new_time = np.arange(start, stop, frame_interval)

# Convert TrueTrack to radians for sine and cosine components
data['TrueTrack_rad'] = np.deg2rad(data['TrueTrack'])
data['sin_truetrack'] = np.sin(data['TrueTrack_rad'])
data['cos_truetrack'] = np.cos(data['TrueTrack_rad'])

# Create interpolation functions for sin(TrueTrack), cos(TrueTrack), gs, latitude, and longitude
interp_sin_truetrack = interp1d(data['time_seconds'], data['sin_truetrack'], kind='linear', fill_value="extrapolate")
interp_cos_truetrack = interp1d(data['time_seconds'], data['cos_truetrack'], kind='linear', fill_value="extrapolate")
interp_gs = interp1d(data['time_seconds'], data['gs'], kind='linear', fill_value="extrapolate")
interp_latitude = interp1d(data['time_seconds'], data['latitude'], kind='linear', fill_value="extrapolate")
interp_longitude = interp1d(data['time_seconds'], data['longitude'], kind='linear', fill_value="extrapolate")
# 表示用は実際の経度。地図描画用(スケール済み)と取り違えないこと
interp_longitude_true = interp1d(data['time_seconds'], data['longitude_true'], kind='linear', fill_value="extrapolate")
# 新規追加データ(KF高度・垂直速度・気圧)の補間
interp_kf_alt = interp1d(data['time_seconds'], data['KF_Alt_c'], kind='linear', fill_value="extrapolate")
interp_kf_vspeed = interp1d(data['time_seconds'], data['KF_Vspeed'], kind='linear', fill_value="extrapolate")
interp_pressure = interp1d(data['time_seconds'], data['pressure'], kind='linear', fill_value="extrapolate")

# 姿勢角(roll/pitch/yaw)の補間。yawは角度なので360°跨ぎ対策にsin/cos成分で補間する
interp_roll = interp1d(euler['time_seconds'], euler['roll'], kind='linear', fill_value="extrapolate")
interp_pitch = interp1d(euler['time_seconds'], euler['pitch'], kind='linear', fill_value="extrapolate")
euler_yaw_rad = np.deg2rad(euler['yaw'])
interp_sin_yaw = interp1d(euler['time_seconds'], np.sin(euler_yaw_rad), kind='linear', fill_value="extrapolate")
interp_cos_yaw = interp1d(euler['time_seconds'], np.cos(euler_yaw_rad), kind='linear', fill_value="extrapolate")

# β・偏流角・風・TASの補間。方位(風向・真方位)は360°跨ぎ対策にsin/cos成分で補間する
def interp_bd(col):
    return interp1d(betada['time_seconds'], betada[col], kind='linear', fill_value="extrapolate")

interp_beta = interp_bd('beta')
interp_da = interp_bd('da')
interp_windspd = interp_bd('wind_speed')
interp_tas = interp_bd('tas')
_wd = np.deg2rad(betada['wind_dir'])
interp_sin_wd = interp1d(betada['time_seconds'], np.sin(_wd), kind='linear', fill_value="extrapolate")
interp_cos_wd = interp1d(betada['time_seconds'], np.cos(_wd), kind='linear', fill_value="extrapolate")
_yt = np.deg2rad(betada['yaw_true'])
interp_sin_yawtrue = interp1d(betada['time_seconds'], np.sin(_yt), kind='linear', fill_value="extrapolate")
interp_cos_yawtrue = interp1d(betada['time_seconds'], np.cos(_yt), kind='linear', fill_value="extrapolate")
# 推定が有効なのは飛行区間のみ。その外は外挿になるので表示を伏せる
BETADA_T0 = betada['time_seconds'].iloc[0]
BETADA_T1 = betada['time_seconds'].iloc[-1]

# システムログの値はステータス情報なので、直前のログ値を保持する階段状補間(kind='previous')にする
interp_volt = interp1d(log_volt_t, [r[1] for r in log_volt_rows], kind='previous', bounds_error=False, fill_value=(log_volt_rows[0][1], log_volt_rows[-1][1]))
interp_hacc = interp1d(log_acc_t, [r[1] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][1], log_acc_rows[-1][1]))
interp_vacc = interp1d(log_acc_t, [r[2] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][2], log_acc_rows[-1][2]))
interp_sacc = interp1d(log_acc_t, [r[3] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][3], log_acc_rows[-1][3]))
interp_sats = interp1d(log_acc_t, [r[4] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][4], log_acc_rows[-1][4]))

# Initialize figure and axes
fig, (tt_ax, gs_ax, map_ax) = plt.subplots(3, 1, figsize=(8, 9), facecolor='green', gridspec_kw={'height_ratios': [0.3, 0.08, 0.5]})
tt_ax.set_xlim(-1, 1)
tt_ax.set_ylim(-1, 1)
tt_ax.set_aspect('equal')
tt_ax.set_facecolor('green')
tt_ax.grid(False)
tt_ax.xaxis.set_visible(False)
tt_ax.yaxis.set_visible(False)
for spine in tt_ax.spines.values():
    spine.set_visible(False)

gs_ax.set_xlim(2, 10)
gs_ax.set_xticks(range(11))
gs_ax.tick_params(axis='x', labelsize=15)  # 目盛り数字(2～10)を見やすく大きめに
gs_ax.set_facecolor('lightgreen')
gs_ax.grid(axis='x')
gs_ax.yaxis.set_visible(False)

# Adjust the map subplot
set_map_limits(map_ax)
map_ax.set_facecolor('green')
for spine in map_ax.spines.values():
    spine.set_visible(False)

# Plot settings
tt_text = tt_ax.text(0.5, 1.09, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
latlon_text = tt_ax.text(-0.9, -0.13, "", horizontalalignment='left', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=16, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
gs_text = gs_ax.text(0, 0.5, "", verticalalignment='center', horizontalalignment='left', transform=gs_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])
time_text = tt_ax.text(0.5, 1.3, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=22, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
# 矢印とテキストの色を対応させる。どの矢印がどの値なのか一目で分かるようにする
YAW_COLOR = 'deepskyblue'   # 機首方位(Yaw)
BETA_COLOR = 'yellow'       # 機首方位+β = 対気速度ベクトルの向き(偏流を受ける前の進行方向)
TRACK_COLOR = 'black'       # 実際の対地進行方向(TrueTrack)

# 行ごとに色を変えられるよう、1行を1つのテキストとして作る。
# まとめて1つの複数行テキストにすると行単位で色を変えられないため
COL_LINE_DY = 0.17   # 行間(tt_axの座標系)。詰めすぎると下の緯度経度と重なる
def make_column(x, y_top, colors, fontsize=15):
    arts = []
    for k, color in enumerate(colors):
        # 黄色は白フチだと見えにくいので、明るい色には黒フチを合わせる
        stroke = 'black' if color in (BETA_COLOR,) else 'white'
        arts.append(tt_ax.text(x, y_top - k * COL_LINE_DY, "",
                               horizontalalignment='left', verticalalignment='top',
                               transform=tt_ax.transAxes, color=color, fontsize=fontsize,
                               fontweight='bold',
                               path_effects=[path_effects.withStroke(linewidth=4, foreground=stroke)]))
    return arts

# 右列: 高度・気圧・姿勢角(機体そのものの状態)。Yawの行だけ矢印と同じ水色にする
euler_texts = make_column(1.25, 1.0, ['black', 'black', 'black', 'black', 'black', YAW_COLOR])
# 左列: 空力データ(対気速度・横滑り角・偏流角・風)。
# flight_preprocess.py の推定結果なので、実測値と区別できるよう見出しを付ける。
# Betaの行だけ矢印と同じ黄色にする
air_texts = make_column(-1.35, 1.0, ['black', 'black', BETA_COLOR, 'black', 'black', 'black'])
# 風向計の見出し(位置固定なのでupdate内で作り直す必要がない)
wind_label = tt_ax.text(WIND_CX, WIND_CY + (WIND_ARROW_MIN + WIND_ARROW_LEN) / 2 + 0.10, "WIND", horizontalalignment='center', verticalalignment='bottom', color='mediumblue', fontsize=13, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])
# システムログ(電圧・GPS精度・衛星数)の表示。重要度低めなので画面左下に小さめに1行表示
# fig.textは図全体の座標基準なのでax.clear()の影響を受けない
sys_text = fig.text(0.02, 0.015, "", horizontalalignment='left', verticalalignment='bottom', color='black', fontsize=13, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=4, foreground='white')])

# Function to convert seconds back to time string
# 秒は小数点2桁まで表示する(フレーム間隔0.25秒に対応)
def seconds_to_time(seconds, start_time_str):
    start_h, start_m, start_s = map(int, start_time_str.split(':'))
    total_seconds = seconds + (start_h * 3600 + start_m * 60 + start_s)
    h = int(total_seconds // 3600)
    m = int((total_seconds % 3600) // 60)
    s = total_seconds % 60
    return f"{h:02d}:{m:02d}:{s:05.2f}"

# Update function for animation
def update(i):
    tt_ax.clear()
    tt_ax.set_xlim(-1, 1)
    tt_ax.set_ylim(-1, 1)
    tt_ax.set_aspect('equal')
    tt_ax.set_facecolor('green')
    tt_ax.grid(False)
    tt_ax.xaxis.set_visible(False)
    tt_ax.yaxis.set_visible(False)
    for spine in tt_ax.spines.values():
        spine.set_visible(False)
    
    gs_ax.clear()
    gs_ax.set_xticks(range(11))
    gs_ax.set_xlim(2, 10)
    gs_ax.tick_params(axis='x', labelsize=15)  # 目盛り数字(2～10)を見やすく大きめに
    gs_ax.set_facecolor('lightgreen')
    gs_ax.grid(axis='x')
    gs_ax.yaxis.set_visible(False)

    # Interpolate values for the current time
    sin_truetrack = interp_sin_truetrack(i)
    cos_truetrack = interp_cos_truetrack(i)
    truetrack = np.mod(np.degrees(np.arctan2(sin_truetrack, cos_truetrack)), 360)  # Reconstruct angle
    gs = interp_gs(i)
    latitude = interp_latitude(i)
    longitude = interp_longitude(i)            # 地図描画用(cos36°でスケール済み)
    longitude_disp = interp_longitude_true(i)  # 表示用(実際の経度)
    kf_alt = interp_kf_alt(i)
    kf_vspeed = interp_kf_vspeed(i)
    pressure = interp_pressure(i)
    roll = interp_roll(i)
    pitch = interp_pitch(i)
    yaw = np.mod(np.degrees(np.arctan2(interp_sin_yaw(i), interp_cos_yaw(i))), 360)
    # 推定値(β・偏流角・風・TAS)。飛行区間外は推定できないのでフラグで伏せる
    betada_valid = BETADA_T0 <= i <= BETADA_T1
    beta = interp_beta(i)
    da = interp_da(i)
    wind_spd = interp_windspd(i)
    wind_dir = np.mod(np.degrees(np.arctan2(interp_sin_wd(i), interp_cos_wd(i))), 360)
    tas = interp_tas(i)
    # ヨー誤差を補正した真方位。矢印もこちらを使う方が物理的に正しい
    yaw_true = np.mod(np.degrees(np.arctan2(interp_sin_yawtrue(i), interp_cos_yawtrue(i))), 360)
    if not betada_valid:
        yaw_true = yaw
    volt = interp_volt(i)
    hacc = interp_hacc(i)
    vacc = interp_vacc(i)
    sacc = interp_sacc(i)
    sats = interp_sats(i)
    current_time = seconds_to_time(i, starttime)

    # TrueTrack arrow
    # KF表示を右列へ移して左が空いたため、矢印は中心をやや左に寄せて大きく表示する。
    # コンパス針のように矢印の中央が回転中心になるよう、後端を中心の反対側に置く
    ARROW_CX, ARROW_CY = -0.25, 0.0  # 矢印の回転中心
    ARROW_LEN = 1.2                  # 矢印の全長
    angle_rad = np.deg2rad(90 - truetrack)
    x = ARROW_LEN * np.cos(angle_rad)
    y = ARROW_LEN * np.sin(angle_rad)
    arrow = FancyArrow(ARROW_CX - x / 2, ARROW_CY - y / 2, x, y, facecolor=TRACK_COLOR, width=0.07, edgecolor='white', linewidth=3, clip_on=False)
    tt_ax.add_patch(arrow)

    # Yaw arrow (機首方位)。破線・水色の中抜き矢印
    yaw_rad = np.deg2rad(90 - yaw_true)
    yx = ARROW_LEN * np.cos(yaw_rad)
    yy = ARROW_LEN * np.sin(yaw_rad)
    yaw_arrow = FancyArrow(ARROW_CX - yx / 2, ARROW_CY - yy / 2, yx, yy, facecolor='none', width=0.07, edgecolor=YAW_COLOR, linewidth=3, linestyle='--', clip_on=False)
    tt_ax.add_patch(yaw_arrow)

    # Air path arrow (機首方位 + β)。対気速度ベクトルの向き = 風に流される前の進行方向。
    # 点線・細い黄色の矢印。これを挟むことで
    #   水色→黄色の開き = Beta、黄色→黒の開き = 偏流角(DA)
    # と、2つの角度を目で区別できるようになる
    air_patches = []
    if betada_valid:
        air_rad = np.deg2rad(90 - (yaw_true + beta))
        ax_ = ARROW_LEN * np.cos(air_rad)
        ay_ = ARROW_LEN * np.sin(air_rad)
        air_arrow = FancyArrow(ARROW_CX - ax_ / 2, ARROW_CY - ay_ / 2, ax_, ay_,
                               facecolor='none', width=0.035, edgecolor=BETA_COLOR,
                               linewidth=2, linestyle=':', clip_on=False)
        tt_ax.add_patch(air_arrow)
        air_patches.append(air_arrow)

    # Wind arrow (風)。コンパスとは別に、右上に小さな風向計として描く。
    # 機体の矢印と同じ「上が北」の向きで、風が吹いていく方向を指す
    wind_patches = []
    if betada_valid and wind_spd > 0.05:
        wind_rad = np.deg2rad(90 - (wind_dir + 180.0))  # 吹いてくる方位 → 吹いていく向き
        wlen = WIND_ARROW_MIN + WIND_ARROW_LEN * min(wind_spd / WIND_ARROW_FULL, 1.0)
        wx, wy = wlen * np.cos(wind_rad), wlen * np.sin(wind_rad)
        wind_arrow = FancyArrow(WIND_CX - wx / 2, WIND_CY - wy / 2, wx, wy,
                                facecolor='mediumblue', width=0.04,
                                edgecolor='white', linewidth=2, clip_on=False)
        tt_ax.add_patch(wind_arrow)
        wind_patches.append(wind_arrow)
    
    # Ground speed bar。推定TASをオレンジの縦線で重ねると、
    # 対地速度との差がそのまま風の影響として読み取れる
    gs_ax.barh(0.5, gs, height=0.05, color='blue')
    gs_text.set_text(f"PONS G/S {gs:.1f}m/s")
    if betada_valid:
        gs_ax.axvline(tas, color='darkorange', linewidth=5)
        # 縦線が何を指すのかバーの上に小さく添える。
        # バー内は大きなG/S文字が占めるため、外側に出さないと隠れてしまう
        gs_ax.text(tas, 1.08, f"TAS {tas:.1f}", color='darkorange', fontsize=14, fontweight='bold',
                   horizontalalignment='center', verticalalignment='bottom',
                   path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])
    
    # Text updates
    tt_text.set_text(f"TrueTrack: {int(truetrack)}°")
    latlon_text.set_text(f"{latitude:.6f}N {longitude_disp:.6f}E")
    time_text.set_text(f"Time: {current_time}")
    # 右列: 高度・気圧・姿勢角。roll/pitchは±付き、yawは0-360°表示。
    # 高度は気圧ドリフトを補正済みなので (KF_C) と明示する
    for art, txt in zip(euler_texts, [
            f"Alt (KF_C) {kf_alt:+.1f}m", f"KF V/S {kf_vspeed:+.2f}m/s", f"{pressure:.2f}hPa",
            f"Roll {roll:+.1f}°", f"Pitch {pitch:+.1f}°", f"Yaw {yaw_true:.0f}°"]):
        art.set_text(txt)
    # 左列: 空力データ。すべて flight_preprocess.py による推定値なので
    # ESTIMATED の見出しを付けて実測値と区別する。飛行区間外は "--" にする
    if betada_valid:
        air_lines = ["- ESTIMATED -", f"TAS {tas:.1f}m/s", f"Beta {beta:+.1f}°",
                     f"Drift {da:+.1f}°", f"Wind {wind_spd:.1f}m/s", f"  from {wind_dir:.0f}°"]
    else:
        air_lines = ["- ESTIMATED -", "TAS --", "Beta --", "Drift --", "Wind --", "  from --"]
    for art, txt in zip(air_texts, air_lines):
        art.set_text(txt)
    # システムログの電圧・GPS精度・衛星数(直前のログ値を保持)
    sys_text.set_text(f"Batt {volt:.2f}V  hAcc {hacc:.1f}m vAcc {vacc:.1f}m sAcc {sacc:.2f}m/s  sats {sats:.0f}")

    tt_ax.add_artist(tt_text)
    tt_ax.add_artist(latlon_text)
    gs_ax.add_artist(gs_text)
    tt_ax.add_artist(time_text)
    for art in euler_texts + air_texts:
        tt_ax.add_artist(art)
    tt_ax.add_artist(wind_label)

    # Map plot
    map_ax.clear()
    map_ax.set_aspect('equal')
    set_map_limits(map_ax)
    map_ax.set_facecolor('green')
    map_ax.grid(False)
    map_ax.xaxis.set_visible(False)
    map_ax.yaxis.set_visible(False)
    for spine in map_ax.spines.values():
        spine.set_visible(False)
    
    # Interpolate path up to current time
    times_up_to_i = new_time[new_time <= i]
    map_ax.plot(interp_longitude(times_up_to_i), interp_latitude(times_up_to_i), color='white', linewidth=2)
    map_ax.plot(longitude, latitude, marker='o', color='black', markersize=10, markeredgecolor='white', markeredgewidth=4)

    return ([arrow, yaw_arrow, tt_text, latlon_text, gs_text, time_text, wind_label, sys_text]
            + euler_texts + air_texts + air_patches + wind_patches
            + gs_ax.patches + map_ax.lines + map_ax.patches)

ani = FuncAnimation(fig, update, frames=new_time, interval=frame_interval*1000)
ani.save(SAVENAME, writer='ffmpeg', fps=1/frame_interval)