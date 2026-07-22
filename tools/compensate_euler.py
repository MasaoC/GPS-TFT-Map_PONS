# -*- coding: utf-8 -*-
# BNO085 オイラー角ログの補正スクリプト
# Last update 2026/7/18
#
# 補正内容:
#  (1) 設置誤差の除去
#      静止区間(gs < REST_GS_THRESH)の平均姿勢と、既知の真姿勢
#      (roll=0, pitch=0, yaw=TRUE_YAW_AT_REST) から、センサーの
#      取り付け回転行列を求めて全サンプルから除去する。
#      3軸を個別に引き算するのではなく回転行列で処理するため、
#      軸間の干渉(ロールオフセットがある状態でのヨー回転など)も正しく扱える。
#
#  (2) 加速度によるエラーの除去
#      BNO085は加速度環境で重力方向の推定がずれる。
#      GNSSの対地速度gs(m/s)とTrueTrack(°)から機体加速度を推定し、
#      見かけの重力傾斜分をピッチ・ロールから取り除く。
#        前後加速度 a_long = d(gs)/dt      → ピッチ誤差 +atan(a_long/g) を減算
#        旋回の遠心加速度 a_lat = gs * ω   → ロール誤差 -atan(a_lat/g) を加算
#      (ω = TrueTrackの変化率[rad/s]。右旋回で正)
#      物理的な意味: 前方加速すると見かけ上「機首上げ」に、
#      協調旋回中は見かけ重力が機体z軸に揃うため「バンク角が小さく」誤推定される。
#
# 使い方: python3 compensate_euler.py
# 出力: euler_20260711edit_compensated.txt (元ファイルと同じ time,roll,pitch,yaw 形式)
#       compensation_check.png (補正前後の比較グラフ)

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# 設定 -------------------------------------------------------
EULER_FILENAME = 'euler_20260711edit.txt'
CSV_FILENAME = '2026-07-11_0423_lastTF.csv'
SAVENAME = 'euler_20260711edit_compensated.txt'
PLOTNAME = 'compensation_check.png'

TRUE_YAW_AT_REST = 156.0   # 静止時の既知の真方位 [deg]
REST_GS_THRESH = 0.3       # これ未満のgs[m/s]を「静止」とみなす
GS_VALID_THRESH = 1.0      # これ未満のgsではTrueTrackを信用しない(方位不定のため)
ACCEL_GAIN = 0.7           # 加速度補正の強さ (1.0=フル補正。効きすぎる場合は下げる)
ERR_LPF_SEC = 2.0          # 誤差モデルの時定数 [秒] (0=無効)。BNO085は内部でジャイロと
                           # 融合しているため、短時間の加速では誤差が atan(a/g) まで
                           # 育たない。補正が効きすぎる場合は 2～5秒程度を試す

# 補正係数の自動フィッティング --------------------------------
# GROUND_END_TIME までは機体が接地して滑走している(=真のピッチ≈0)という
# 既知情報を教師データとして、地上滑走区間の補正後ピッチのRMSが最小に
# なるよう ACCEL_GAIN と ERR_LPF_SEC を自動推定する。
# True: 上の手動設定値を無視して自動推定した値を使う / False: 手動設定値を使う
AUTO_FIT = True
GROUND_END_TIME = '07:57:36.50'  # 離陸(接地終了)時刻。この時刻まで真ピッチ≈0とみなす
SMOOTH_SEC = 2.1           # gs/TrueTrackの平滑化窓 [秒] (微分ノイズ対策)
GRAVITY = 9.80665          # 重力加速度 [m/s^2]

# 時刻文字列(HH:MM:SS.ss)をその日の通算秒に変換
def time_to_daysec(t):
    h, m, s = t.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)

# 通算秒をHH:MM:SS.ss形式に戻す(出力ファイル用)
def daysec_to_time(sec):
    h = int(sec // 3600)
    m = int((sec % 3600) // 60)
    s = sec % 60
    return f"{h:02d}:{m:02d}:{s:05.2f}"

# データ読み込み ---------------------------------------------
euler = pd.read_csv(EULER_FILENAME)
csv = pd.read_csv(CSV_FILENAME)
euler['t'] = euler['time'].apply(time_to_daysec)
csv['t'] = csv['time'].apply(time_to_daysec)

# GNSSデータの前処理 -----------------------------------------
# 静止中(gs小)のTrueTrackは方位不定でゴミ値(今回は198)が入るため、
# 最初に動き出した時点の値で置き換えてから平滑化する
track = csv['TrueTrack'].astype(float).to_numpy()
gs = csv['gs'].to_numpy()
moving = gs >= GS_VALID_THRESH
if moving.any():
    first_valid = track[moving][0]
    track[~moving] = first_valid
# 360°の折り返しを連続値に展開(例 359→1 を 359→361 に)
track_unwrapped = np.degrees(np.unwrap(np.radians(track)))

# 10Hzの等間隔グリッドに補間してから平滑化・微分する
t0, t1 = csv['t'].iloc[0], csv['t'].iloc[-1]
tg = np.arange(t0, t1, 0.1)
gs_g = np.interp(tg, csv['t'], gs)
track_g = np.interp(tg, csv['t'], track_unwrapped)

win = max(5, int(SMOOTH_SEC / 0.1) // 2 * 2 + 1)  # 奇数の窓幅
gs_s = savgol_filter(gs_g, win, 2)
track_s = savgol_filter(track_g, win, 2)

a_long = np.gradient(gs_s, tg)                    # 前後加速度 [m/s^2]
omega = np.radians(np.gradient(track_s, tg))      # 旋回角速度 [rad/s] 右旋回で正
omega[gs_s < GS_VALID_THRESH] = 0.0               # 低速時は旋回情報なし
a_lat = gs_s * omega                              # 遠心(向心)加速度 [m/s^2] 右向き正

# オイラー角の各時刻に補間
e_along = np.interp(euler['t'], tg, a_long)
e_alat = np.interp(euler['t'], tg, a_lat)
e_gs = np.interp(euler['t'], tg, gs_s)

# 動き出し前は実際には加速していないのに、平滑化の影響で
# 加速度が漏れ込むため、静止区間の加速度補正はゼロにする
if moving.any():
    t_move = csv['t'][gs >= REST_GS_THRESH].iloc[0]
    still = euler['t'] < t_move
    e_along[still] = 0.0
    e_alat[still] = 0.0

# (1) 設置誤差の推定と除去 -----------------------------------
# 静止区間 = 記録開始からgsがREST_GS_THRESHを初めて超えるまで
gs_at_euler = np.interp(euler['t'], csv['t'], gs)
move_idx = np.argmax(gs_at_euler >= REST_GS_THRESH)
rest = euler.iloc[:move_idx] if move_idx > 0 else euler.iloc[:10]
print(f"静止区間: {rest['time'].iloc[0]} ～ {rest['time'].iloc[-1]} ({len(rest)}サンプル)")
print(f"静止時平均: roll={rest['roll'].mean():.2f}  pitch={rest['pitch'].mean():.2f}  yaw={rest['yaw'].mean():.2f}")

# 回転の定義: 機体座標(x前方,y右,z下) → 'ZYX'順(ヨー→ピッチ→ロール)の航空慣例
# R_meas = R_true * C  (C: センサー取り付けの固定回転)
R_meas_rest = Rotation.from_euler(
    'ZYX', [rest['yaw'].mean(), rest['pitch'].mean(), rest['roll'].mean()], degrees=True)
R_true_rest = Rotation.from_euler('ZYX', [TRUE_YAW_AT_REST, 0.0, 0.0], degrees=True)
C = R_true_rest.inv() * R_meas_rest
c_yaw, c_pitch, c_roll = C.as_euler('ZYX', degrees=True)
print(f"設置オフセット: roll={c_roll:.2f}  pitch={c_pitch:.2f}  yaw={c_yaw:.2f}")

# 全サンプルに適用: R_true = R_meas * C^-1
R_meas = Rotation.from_euler(
    'ZYX', euler[['yaw', 'pitch', 'roll']].to_numpy(), degrees=True)
ypr = (R_meas * C.inv()).as_euler('ZYX', degrees=True)
yaw_c, pitch_c, roll_c = ypr[:, 0] % 360.0, ypr[:, 1], ypr[:, 2]

# (2) 加速度誤差の除去 ---------------------------------------
pitch_err = np.degrees(np.arctan2(e_along, GRAVITY))   # 前方加速 → 機首上げに誤推定
roll_err = -np.degrees(np.arctan2(e_alat, GRAVITY))    # 右旋回 → バンク角が浅く誤推定

# BNO085内部フィルタの応答を模擬する一次遅れローパス。
# 加速度が続くと誤差が徐々に育ち、なくなると徐々に戻る挙動を再現する
dt_arr = np.diff(euler['t'].to_numpy(), prepend=euler['t'].iloc[0])
def apply_err_lpf(err, tau):
    if tau <= 0.01:
        return err.copy()
    out_arr = np.zeros_like(err)
    y = 0.0
    for i in range(len(err)):
        alpha = 1.0 - np.exp(-dt_arr[i] / tau)
        y += alpha * (err[i] - y)
        out_arr[i] = y
    return out_arr

if AUTO_FIT:
    # 地上滑走区間(開始～GROUND_END_TIME)の補正後ピッチが0に近づくよう
    # (ACCEL_GAIN, ERR_LPF_SEC) をNelder-Mead法で最適化する
    ground = (euler['t'] <= time_to_daysec(GROUND_END_TIME)).to_numpy()

    def fit_cost(params):
        gain, tau = params
        if gain < 0 or tau < 0:
            return 1e9
        residual = pitch_c - gain * apply_err_lpf(pitch_err, tau)
        return np.sqrt(np.mean(residual[ground] ** 2))

    rms_before = np.sqrt(np.mean(pitch_c[ground] ** 2))
    result = minimize(fit_cost, [1.0, 3.0], method='Nelder-Mead')
    ACCEL_GAIN, ERR_LPF_SEC = result.x
    print(f"自動フィット: ACCEL_GAIN={ACCEL_GAIN:.3f}  ERR_LPF_SEC={ERR_LPF_SEC:.3f}")
    print(f"地上滑走区間のピッチRMS: 補正なし{rms_before:.2f}° → 補正後{result.fun:.2f}°")

pitch_c = pitch_c - ACCEL_GAIN * apply_err_lpf(pitch_err, ERR_LPF_SEC)
roll_c = roll_c - ACCEL_GAIN * apply_err_lpf(roll_err, ERR_LPF_SEC)

# 保存 -------------------------------------------------------
out = pd.DataFrame({
    'time': euler['time'],
    'roll': np.round(roll_c, 2),
    'pitch': np.round(pitch_c, 2),
    'yaw': np.round(yaw_c, 2),
})
out.to_csv(SAVENAME, index=False)
print(f"補正結果を保存: {SAVENAME}")

# 比較グラフ -------------------------------------------------
ts = euler['t'] - euler['t'].iloc[0]
fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
axes[0].plot(ts, euler['roll'], label='roll (raw)', color='lightcoral')
axes[0].plot(ts, roll_c, label='roll (compensated)', color='red')
axes[0].set_ylabel('roll [deg]')
axes[1].plot(ts, euler['pitch'], label='pitch (raw)', color='lightgreen')
axes[1].plot(ts, pitch_c, label='pitch (compensated)', color='green')
axes[1].set_ylabel('pitch [deg]')
axes[2].plot(ts, euler['yaw'], label='yaw (raw)', color='lightskyblue')
axes[2].plot(ts, yaw_c, label='yaw (compensated)', color='blue')
axes[2].set_ylabel('yaw [deg]')
axes[3].plot(ts, e_gs, label='gs [m/s]', color='gray')
axes[3].plot(ts, e_along, label='a_long [m/s2]', color='orange')
axes[3].plot(ts, e_alat, label='a_lat [m/s2]', color='purple')
axes[3].set_ylabel('GNSS')
axes[3].set_xlabel('time [s]')
for ax in axes:
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(alpha=0.3)
fig.suptitle('BNO085 Euler compensation check')
fig.tight_layout()
fig.savefig(PLOTNAME, dpi=110)
print(f"比較グラフを保存: {PLOTNAME}")
