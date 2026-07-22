import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
import matplotlib.patheffects as path_effects
from scipy.interpolate import interp1d

# Last update 2026/7/16

FILENAME = '2026-07-11_0423_lastTF.csv'
EULER_FILENAME = 'euler_20260711edit_compensated.txt'  # 姿勢角(オイラー角)ログ。CSVと同時刻で同期表示する
LOG_FILENAME = 'log.txt'  # システムログ。電圧・GPS精度・衛星数をCSVと同期表示する
SAVENAME = '0711.mp4'
LOGDATE = "2026-07-11"
starttime = "07:57:23"
stoptime = "07:58:18"

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

# Apply scaling to longitude
data['longitude'] = data['longitude'] * 0.8090169  # *cos(36deg)

# KF_Altitudeはオフセットがあるため、最初の行の値を0点とする相対高度に変換
kf_alt_offset = data['KF_Altitude'].iloc[0]
data['KF_Alt_rel'] = data['KF_Altitude'] - kf_alt_offset

# 姿勢角データの読み込み。CSVと同じ開始時刻を基準に秒へ変換して同期する
euler = pd.read_csv(EULER_FILENAME)
euler['time_seconds'] = euler['time'].apply(lambda x: time_to_seconds(x, starttime))

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
# 新規追加データ(KF高度・垂直速度・気圧)の補間
interp_kf_alt = interp1d(data['time_seconds'], data['KF_Alt_rel'], kind='linear', fill_value="extrapolate")
interp_kf_vspeed = interp1d(data['time_seconds'], data['KF_Vspeed'], kind='linear', fill_value="extrapolate")
interp_pressure = interp1d(data['time_seconds'], data['pressure'], kind='linear', fill_value="extrapolate")

# 姿勢角(roll/pitch/yaw)の補間。yawは角度なので360°跨ぎ対策にsin/cos成分で補間する
interp_roll = interp1d(euler['time_seconds'], euler['roll'], kind='linear', fill_value="extrapolate")
interp_pitch = interp1d(euler['time_seconds'], euler['pitch'], kind='linear', fill_value="extrapolate")
euler_yaw_rad = np.deg2rad(euler['yaw'])
interp_sin_yaw = interp1d(euler['time_seconds'], np.sin(euler_yaw_rad), kind='linear', fill_value="extrapolate")
interp_cos_yaw = interp1d(euler['time_seconds'], np.cos(euler_yaw_rad), kind='linear', fill_value="extrapolate")

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
tt_text = tt_ax.text(0.5, 1.05, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
latlon_text = tt_ax.text(-0.9, -0.13, "", horizontalalignment='left', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=16, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
gs_text = gs_ax.text(0, 0.5, "", verticalalignment='center', horizontalalignment='left', transform=gs_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])
time_text = tt_ax.text(0.5, 1.3, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=22, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
# KF高度・垂直速度・気圧と姿勢角(roll/pitch/yaw/β)の表示。
# コンパス右の空きスペースに一列に整列して表示する(左側は矢印スペースとして空ける)
euler_text = tt_ax.text(1.25, 1.0, "", horizontalalignment='left', verticalalignment='top', transform=tt_ax.transAxes, color='black', fontsize=15, fontweight='bold', linespacing=1.6, path_effects=[path_effects.withStroke(linewidth=4, foreground='white')])
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
    longitude = interp_longitude(i)
    kf_alt = interp_kf_alt(i)
    kf_vspeed = interp_kf_vspeed(i)
    pressure = interp_pressure(i)
    roll = interp_roll(i)
    pitch = interp_pitch(i)
    yaw = np.mod(np.degrees(np.arctan2(interp_sin_yaw(i), interp_cos_yaw(i))), 360)
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
    arrow = FancyArrow(ARROW_CX - x / 2, ARROW_CY - y / 2, x, y, facecolor='black', width=0.07, edgecolor='white', linewidth=3, clip_on=False)
    tt_ax.add_patch(arrow)

    # Yaw arrow (機首方位)。点線・水色の中抜き矢印で、TrueTrack矢印との開き角がβとして見える
    yaw_rad = np.deg2rad(90 - yaw)
    yx = ARROW_LEN * np.cos(yaw_rad)
    yy = ARROW_LEN * np.sin(yaw_rad)
    yaw_arrow = FancyArrow(ARROW_CX - yx / 2, ARROW_CY - yy / 2, yx, yy, facecolor='none', width=0.07, edgecolor='deepskyblue', linewidth=3, linestyle='--', clip_on=False)
    tt_ax.add_patch(yaw_arrow)
    
    # Ground speed bar
    gs_ax.barh(0.5, gs, height=0.05, color='blue')
    gs_text.set_text(f"PONS G/S {gs:.1f}m/s")
    
    # Text updates
    tt_text.set_text(f"TrueTrack: {int(truetrack)}°")
    latlon_text.set_text(f"{latitude:.6f}N {longitude:.6f}E")
    time_text.set_text(f"Time: {current_time}")
    # 推定横滑り角β(無風仮定): β = TrueTrack - Yaw を±180°に折り返す。
    # 符号は航空標準に合わせ、正 = 速度ベクトルが機首より右
    # (機体が右に流されている = 相対風が機首の右から当たる状態)。
    # 低速時はTrueTrackが方位不定でβが無意味になるため "--" 表示にする
    beta = (truetrack - yaw + 180.0) % 360.0 - 180.0
    beta_str = f"{beta:+.1f}°" if gs >= 1.0 else "--"
    # 右列にKF高度(初期値0mの相対表示)・垂直速度・気圧と姿勢角を一列に整列表示。
    # roll/pitchは±付き、yawは0-360°表示
    euler_text.set_text(f"KF Alt {kf_alt:+.1f}m\nKF V/S {kf_vspeed:+.2f}m/s\n{pressure:.2f}hPa\nRoll {roll:+.1f}°\nPitch {pitch:+.1f}°\nYaw {yaw:.0f}°\nEst. Beta {beta_str}")
    # システムログの電圧・GPS精度・衛星数(直前のログ値を保持)
    sys_text.set_text(f"Batt {volt:.2f}V  hAcc {hacc:.1f}m vAcc {vacc:.1f}m sAcc {sacc:.2f}m/s  sats {sats:.0f}")

    tt_ax.add_artist(tt_text)
    tt_ax.add_artist(latlon_text)
    gs_ax.add_artist(gs_text)
    tt_ax.add_artist(time_text)
    tt_ax.add_artist(euler_text)
    
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

    return [arrow, yaw_arrow, tt_text, latlon_text, gs_text, time_text, euler_text, sys_text] + gs_ax.patches + map_ax.lines + map_ax.patches

ani = FuncAnimation(fig, update, frames=new_time, interval=frame_interval*1000)
ani.save(SAVENAME, writer='ffmpeg', fps=1/frame_interval)