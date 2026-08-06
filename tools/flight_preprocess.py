# -*- coding: utf-8 -*-
# フライトログ前処理スクリプト (姿勢補正 + β/偏流角/風の推定)
# Last update 2026/7/28
#
# 位置づけ: [このファイル] → createmovie_csv2mp4.py
#
# 旧 compensate_euler.py と beta_da_estimation.py を1つにまとめたもの。
# 姿勢の補正結果はβ推定の入力になるため、姿勢側の設定を変えたら必ず
# β推定もやり直す必要がある。別ファイルだと再実行を忘れて古い推定結果が
# 残ってしまうため、1回の実行で最後まで通すようにした。
#
# ステージ1: BNO085のオイラー角の補正   → 20260726_compensated.txt
# ステージ2: β・偏流角・風の推定        → beta_da_20260726.txt
# どちらの出力も createmovie_csv2mp4.py が読み込む。
#
# 使い方: python3 flight_preprocess.py
#
# ===== ステージ1: 姿勢(オイラー角)の補正 =====================
# BNO085のオイラー角ログを補正する。
#
# 補正内容:
#  (1) 設置誤差の除去
#      静止区間(gs < REST_GS_THRESH)の平均姿勢と、既知の真姿勢
#      (roll=0, pitch=GROUND_PITCH, ヨーは実測のまま) から、センサーの
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

# ===== ステージ2: β・偏流角・風の推定 ========================
# 横滑り角(Beta)・偏流角(Drift Angle)・風を推定する。
#
# 成り立つ関係式:
#   TrueTrack = Yaw(True) + Beta + DriftAngle
#   ベクトルで書くと  対地速度ベクトル = 対気速度ベクトル + 風ベクトル
#   対気速度ベクトルの方位 = Yaw + Beta
#   偏流角 DA = TrueTrack - (Yaw + Beta)   ← 風によって流される角度
#
# 推定の考え方:
#  (1) 風が分かれば全部決まる
#      GPSの対地速度ベクトルから風ベクトルを引けば対気速度ベクトルが求まり、
#      Beta も DA も一意に決まる。つまり推定すべき未知数は「風」だけ。
#
#  (2) 水平飛行中は Beta ≈ 0 という既知情報で風を解く(風の三角形)
#      対地速度 = TAS * u(Yaw) + 風   という関係を、機首方位が異なる
#      複数の区間について連立させると TAS と風の2成分が最小二乗で解ける。
#      このフライトは機首方位が大きく異なる2方向に分かれているため
#      条件が良く、安定して解ける。
#
#      同時に「ヨー角の誤差」も未知数に含めて推定する。
#      BNO085のヨー角は地磁気で補正されているが、機体の磁気的な影響を受けるため
#      時間とともにドリフトする。一方でジャイロ由来の短時間の相対変化は信頼できる。
#      そこで誤差を「ゆっくり変化する量」としてモデル化し、
#      滑らかさの罰則で長時間のドリフトだけを吸収させる。
#
#      風とヨーバイアスの分離について:
#        風は地面に固定されたベクトルなので、機首方位が変わっても同じ向きに効く。
#        ヨーバイアスは機体に対して垂直方向に効くので、機首方位と一緒に回る。
#        したがって機首方位が変化すれば両者は分離できる。逆に直進が続く区間では
#        原理的に区別できないため、変化の滑らかさ(罰則の強さ)で切り分けている。
#        なお対気速度の大きさ TAS = |対地速度ベクトル - 風| はヨー角を一切含まないため、
#        「TASは一定のはず」という条件はヨー誤差の影響を受けずに風の大きさを決められる。
#        本スクリプトは TAS を一定の未知数として解くことでこれを利用している。
#
#  (3) 旋回中の Beta は横滑りの運動方程式で推定する
#      機体座標の横力のつり合いから
#        d(Beta)/dt = -Beta/TAU + GAIN * ( -r + (g/TAS)*cos(pitch)*sin(roll) )
#      ・第1項: 方向安定(風見安定)によって Beta が減衰する一次遅れ
#      ・第2項の -r     : 機首が回ることで相対的に横滑りが打ち消される効果
#      ・第2項の g*sinφ : バンクにより機体が横に滑り落ちる効果
#      協調旋回(r = (g/TAS)*sinφ)なら括弧内が0になり Beta も0になる。
#      バンクしているのに機首の回りが足りないと Beta が溜まる、という
#      上反角効果まわりの挙動がこの式で表現できる。
#      GAIN はラダー等による協調操作の度合いを吸収する係数(1.0 = 無操舵)。
#
#  (4) (2)と(3)を反復
#      (3)で得た Beta を既知として (2) の風推定に旋回区間も含める。
#      これを数回繰り返して両者を整合させる。
#
#  (5) 風はゆっくりしか変化しないという事前情報を使う
#      風を一定時間ごとのノットで表し、隣接ノットの差を罰則にかけて
#      滑らかに変化させる。推定された風速は、当日の実測値 WIND_MAX と
#      照らし合わせて妥当性を確認する(超えていれば警告を出す)。
#

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# 共通の設定 -------------------------------------------------
EULER_FILENAME = '20260726.txt'        # BNO085の生ログ
CSV_FILENAME = '2026-07-26_0804.csv'   # PONS本体のログ
GRAVITY = 9.797                        # 滋賀県重力加速度 [m/s^2]

# ステージ1の設定 ---------------------------------------------
EULER_SAVENAME = '20260726_compensated.txt'
EULER_PLOTNAME = 'compensation_check.png'

REST_GS_THRESH = 0.3       # これ未満のgs[m/s]を「静止」とみなす
GS_VALID_THRESH = 1.0      # これ未満のgsではTrueTrackを信用しない(方位不定のため)
ACCEL_GAIN = 0.7           # 加速度補正の強さ (1.0=フル補正。効きすぎる場合は下げる)
ERR_LPF_SEC = 2.0          # 誤差モデルの時定数 [秒] (0=無効)。BNO085は内部でジャイロと
                           # 融合しているため、短時間の加速では誤差が atan(a/g) まで
                           # 育たない。補正が効きすぎる場合は 2～5秒程度を試す

# 補正係数の自動フィッティング --------------------------------
# GROUND_END_TIME までは機体が接地して滑走している(=真のピッチ≈GROUND_PITCH)という
# 既知情報を教師データとして、地上滑走区間の補正後ピッチのGROUND_PITCHからの
# RMS偏差が最小になるよう ACCEL_GAIN と ERR_LPF_SEC を自動推定する。
# True: 上の手動設定値を無視して自動推定した値を使う / False: 手動設定値を使う
#
# 地上滑走区間の残差はもともと小さく最適化の谷が浅いため、制限をかけないと
# gain/tau の比だけが効く縮退した領域(gain=3.7e7, tau=5.3e8秒 など)へ発散し、
# 本来のatan(a/g)モデルではなく単なる加速度の積分補正になってしまう。
# そのため物理的に妥当な範囲へ制限をかけて探索する。
AUTO_FIT = True
FIT_GAIN_RANGE = (0.0, 2.0)   # ACCEL_GAINの探索範囲 (1.0=フル補正)
FIT_LPF_RANGE = (0.0, 5.0)   # ERR_LPF_SECの探索範囲 [秒]
GROUND_END_TIME = '08:21:26.30'  # 離陸(接地終了)時刻。この時刻まで真ピッチ≈GROUND_PITCHとみなす
GROUND_PITCH = 0        # 滑走路の傾斜による地上滑走中の真ピッチ [deg] (下り勾配で負)
SMOOTH_SEC = 2.1           # gs/TrueTrackの平滑化窓 [秒] (微分ノイズ対策)

# (3) ロール中立点のドリフト除去 ------------------------------
# 直進している間は、機体は釣り合っているので本来ロールはほぼ0のはず
# (バンクしたまま直進はできない)。それが数十秒〜分の単位で0からずれて
# 見えるのは、加速度センサのバイアスなど姿勢推定側のゆっくりした誤差。
# そこで「旋回していない区間のロールの中央値」を中立点のずれとみなし、
# 時間とともにゆっくり変化する量として取り除く。
# 中央値を使うのは、短時間の本物のバンクに引きずられないようにするため。
ROLL_BIAS_CORRECT = True
ROLL_BIAS_WINDOW_SEC = 60.0   # 中立点を求める窓の長さ [秒]。これより速い変化は残す
ROLL_STRAIGHT_TURNRATE = 0.3  # このヨー角速度[deg/s]未満を「直進」とみなす
ROLL_SMOOTH_SEC = 4.0         # ヨー角速度を求める前の平滑化窓 [秒]
ROLL_BIAS_MIN_SAMPLES = 20    # 窓内にこれだけ直進サンプルがないと中立点を求めない
# 地上や着水後は機体が傾いて静止しているが、それは実際の姿勢であって
# センサの誤差ではない。飛行中(この対地速度[m/s]以上)だけを基準に使う
ROLL_BIAS_MIN_GS = 3.0


# ステージ2の設定 ---------------------------------------------
BETADA_SAVENAME = 'beta_da_20260726.txt'
BETADA_PLOTNAME = 'beta_da_check.png'

# 飛行区間の判定 ---------------------------------------------
# None なら対地速度から自動判定する。手動指定する場合は 'HH:MM:SS.ss' 形式
FLIGHT_START = None
FLIGHT_END = None
AIRBORNE_GS = 3.0          # これを超える対地速度[m/s]が続く区間を飛行中とみなす

# 風推定の設定 -----------------------------------------------
WIND_KNOT_SEC = 120.0      # 風を表すノットの間隔 [秒]。短いほど細かい変化を追える
WIND_LAMBDA = 300.0        # 風の滑らかさ(隣接ノットの差)の罰則
WIND_MAX = 3.0             # 当日の実測最大風速 [m/s]

# 現場で分かっている対気速度の範囲 [m/s] ---------------------
# 風とヨー誤差は互いに置き換えがきくため、データだけでは切り分けられない。
# 幸いこの2つを動かすとTASも一緒に動くので、機体の飛行速度という
# 独立した情報を使って解を絞り込める。
TAS_MEAN_RANGE = (6.1, 7.1)  # フライト全体平均のTASが収まるべき範囲
TAS_RANGE = (5.4, 8.0)       # 瞬時のTASが収まるべき範囲(検証用)

# ヨー角の誤差モデル -----------------------------------------
# BNO085のヨー角は地磁気で補正されるが、機体自身の磁気の影響を受けるため
# 誤差を持ち、時間とともにドリフトする。一方でジャイロ由来の短時間の
# 相対変化は信頼できるので、「ゆっくりとしか変化しない」という前提を
# 滑らかさの罰則として与え、長時間のドリフトだけを吸収させる。
#
# ★重要: 「風が強い」のか「ヨー角が誤っている」のかは、データだけでは
#   区別できない。実際 YAW_BIAS_LAMBDA を 1〜100000 まで変えても
#   水平飛行時の残差は 3.10〜3.14deg とほぼ変わらず、風速だけが
#   3.1〜4.2m/s の間で入れ替わる。さらにこのフライトは前半30分が方位265度、
#   後半が方位40度と、時間と方位が交絡しているため両者の切り分けは
#   本質的に曖昧である。以下の既定値は、当日の実測風速(WIND_MAX)と
#   ドリフト速度の妥当さの両方が釣り合う点として選んである。
#
# 誤差の形として、時間ドリフトのほかに機首方位依存(船のコンパス自差と同じ、
# 機体磁気によるハードアイアン/ソフトアイアン誤差)も選べる。
#   ヨー誤差(ψ) = A + B*sin(ψ) + C*cos(ψ) + D*sin(2ψ) + E*cos(2ψ)
# ただし今回のデータで比較したところ、方位依存モデルは風をほぼゼロまで
# 吸収してしまい(風速0.2〜2.2m/s、風向が222度も振れる)、当てはまりも
# 悪化した。時間ドリフトモデルの方が実測風速とも整合したため既定にしている。
#
#   モデル                    残差      TAS標準偏差  風速         風向変化
#   定数バイアスのみ          3.05deg   0.38         1.83〜4.12   81度
#   方位依存(1周期)           3.43deg   0.39         0.21〜2.20   222度
#   方位依存(2周期)           3.42deg   0.40         0.09〜2.11   222度
#   時間ドリフト(既定)        3.07deg   0.35         1.03〜3.49   110度
YAW_ALLOW_DRIFT = True     # ゆっくりした時間ドリフトを許すか
YAW_BIAS_KNOT_SEC = 300.0  # 時間ドリフトのノット間隔 [秒]
YAW_BIAS_LAMBDA = 20.0     # 時間ドリフトの滑らかさの罰則。大きいほど定数バイアスに近づく
# 罰則を弱めるほど、ヨー誤差が増える代わりに推定される風とTASが小さくなる。
# 現場の実測(WIND_MAX と TAS_MEAN_RANGE)を両方満たす範囲で、
# ヨー誤差が最も小さくなる罰則を自動的に選ぶ。
AUTO_YAW_LAMBDA = True
YAW_LAMBDA_RANGE = (1.0, 1.0e5)  # 自動調整の探索範囲
# このドリフト速度[deg/分]を超えたら注記を出す。5deg/分でも0.08deg/秒なので、
# 10秒程度の短時間で見れば1度未満のズレにしかならず、
# 「短時間の相対変化は信頼できる」という性質とは矛盾しない
YAW_DRIFT_WARN = 5.0
# 機首方位依存(コンパス自差)の成分。時間ドリフトと併用すると未知数が過剰になる
YAW_HARMONICS = 0          # 使う周期成分の数 (0: 使わない / 1: ハードアイアン / 2: ソフトアイアンも)
YAW_CONST_TERM = False     # 定数項。時間ドリフトを使う場合は重複するのでFalse
YAW_RIDGE = 1.0e-3         # 自差係数を暴れさせないための微小な罰則

# 水平飛行(Beta≈0とみなせる)の判定 --------------------------
ROLL_LEVEL_THRESH = 2.0    # このバンク角[deg]未満を水平とみなす
TURNRATE_LEVEL_THRESH = 1.0  # このヨー角速度[deg/s]未満を直進とみなす

# 横滑り運動方程式のパラメータ -------------------------------
AUTO_FIT_BETA = True       # True: 風から求めた Beta に合うよう TAU/GAIN を自動推定
BETA_TAU = 5.0             # 横滑りの減衰時定数 [秒] (方向安定が強いほど短い)
BETA_GAIN = 0.25           # 協調操作の度合い (1.0=無操舵、小さいほど協調が良い)
FIT_TAU_RANGE = (0.5, 20.0)
FIT_BETA_GAIN_RANGE = (0.0, 1.5)

N_ITER = 3                 # 風推定 ⇔ Beta推定 の反復回数
BETA_SMOOTH_SEC = 2.0      # 角速度を求める前の平滑化窓 [秒]
GRID_DT = 0.1              # 内部計算の時間刻み [秒]



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
# ヨーは静止時の実測値をそのまま真値として使う(=ヨー方向は補正しない)。
# 方位の絶対的な誤差は決め打ちできないので、ステージ2でGPSから推定する。
# 静止時は傾斜地に接地しているため、真のピッチは0ではなくGROUND_PITCH
R_true_rest = Rotation.from_euler(
    'ZYX', [rest['yaw'].mean(), GROUND_PITCH, 0.0], degrees=True)
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
    # 地上滑走区間(開始～GROUND_END_TIME)の補正後ピッチがGROUND_PITCHに
    # 近づくよう (ACCEL_GAIN, ERR_LPF_SEC) をNelder-Mead法で最適化する
    ground = (euler['t'] <= time_to_daysec(GROUND_END_TIME)).to_numpy()

    def fit_cost(params):
        gain, tau = params
        residual = pitch_c - gain * apply_err_lpf(pitch_err, tau) - GROUND_PITCH
        return np.sqrt(np.mean(residual[ground] ** 2))

    rms_before = np.sqrt(np.mean((pitch_c[ground] - GROUND_PITCH) ** 2))
    result = minimize(fit_cost, [1.0, 3.0], method='Nelder-Mead',
                      bounds=[FIT_GAIN_RANGE, FIT_LPF_RANGE])
    ACCEL_GAIN, ERR_LPF_SEC = result.x
    print(f"自動フィット: ACCEL_GAIN={ACCEL_GAIN:.3f}  ERR_LPF_SEC={ERR_LPF_SEC:.3f}")
    # 上限に張り付いた場合は範囲が狭すぎる可能性があるので警告する
    for name, val, rng in (('ACCEL_GAIN', ACCEL_GAIN, FIT_GAIN_RANGE),
                           ('ERR_LPF_SEC', ERR_LPF_SEC, FIT_LPF_RANGE)):
        if abs(val - rng[1]) < 1e-3 or abs(val - rng[0]) < 1e-3:
            print(f"  警告: {name}が探索範囲{rng}の端に張り付いています")
    print(f"地上滑走区間のピッチRMS(目標{GROUND_PITCH:+.1f}°からの偏差): 補正なし{rms_before:.2f}° → 補正後{result.fun:.2f}°")

pitch_c = pitch_c - ACCEL_GAIN * apply_err_lpf(pitch_err, ERR_LPF_SEC)
roll_c = roll_c - ACCEL_GAIN * apply_err_lpf(roll_err, ERR_LPF_SEC)

# (3) ロール中立点のドリフト除去 -----------------------------
# 旋回による見かけのバンク誤差は(2)で取り除き済みなので、ここで残るのは
# 姿勢推定そのもののゆっくりした偏り。直進区間のロールを基準に補正する
roll_bias = np.zeros_like(roll_c)
if ROLL_BIAS_CORRECT:
    et = euler['t'].to_numpy()
    # ヨー角速度[deg/s]。360°の折り返しを展開してから平滑化して微分する
    yaw_unwrap = np.degrees(np.unwrap(np.radians(yaw_c)))
    dt_med = np.median(np.diff(et))
    yaw_win = max(5, int(ROLL_SMOOTH_SEC / dt_med) // 2 * 2 + 1)
    yaw_rate = np.gradient(savgol_filter(yaw_unwrap, yaw_win, 2), et)
    # 飛行中かつ旋回していない区間だけを中立点の基準にする
    straight = (np.abs(yaw_rate) < ROLL_STRAIGHT_TURNRATE) & (gs_at_euler > ROLL_BIAS_MIN_GS)

    # 一定時間ごとに区切り、その窓の直進サンプルの中央値を中立点のずれとする
    edges = np.arange(et[0], et[-1] + ROLL_BIAS_WINDOW_SEC, ROLL_BIAS_WINDOW_SEC)
    centers, biases = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = straight & (et >= lo) & (et < hi)
        if sel.sum() >= ROLL_BIAS_MIN_SAMPLES:
            centers.append((lo + hi) / 2.0)
            biases.append(float(np.median(roll_c[sel])))
    if len(centers) >= 2:
        # 窓の中心どうしを直線でつなぎ、両端はそのまま伸ばす
        roll_bias = np.interp(et, centers, biases)
        roll_c = roll_c - roll_bias
        print(f"ロール中立点の補正: {min(biases):+.2f}〜{max(biases):+.2f} deg "
              f"({len(centers)}窓, 直進サンプル{100 * straight.mean():.0f}%)")
    else:
        print("ロール中立点の補正: 直進区間が足りないため実施せず")

# 保存 -------------------------------------------------------
out = pd.DataFrame({
    'time': euler['time'],
    'roll': np.round(roll_c, 2),
    'pitch': np.round(pitch_c, 2),
    'yaw': np.round(yaw_c, 2),
})
out.to_csv(EULER_SAVENAME, index=False)
print(f"補正結果を保存: {EULER_SAVENAME}")

# 比較グラフ -------------------------------------------------
ts = euler['t'] - euler['t'].iloc[0]
fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
axes[0].plot(ts, euler['roll'], label='roll (raw)', color='lightcoral')
axes[0].plot(ts, roll_c, label='roll (compensated)', color='red')
# 取り除いた中立点のずれ。ゆっくりした変化になっているか確認する
axes[0].plot(ts, roll_bias, label='roll neutral drift (removed)', color='black', linewidth=1.5)
axes[0].axhline(0, color='gray', linewidth=0.5)
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
fig.savefig(EULER_PLOTNAME, dpi=110)
print(f"比較グラフを保存: {EULER_PLOTNAME}")


# ===== ステージ2 ここから ====================================
# 角度差を -180〜+180 に畳む
def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


# ステージ1の結果をそのまま受け取る ---------------------------
# ファイル経由ではなくメモリ上で渡すので、丸め誤差が入らず、
# 姿勢補正の設定を変えたのに再推定し忘れる、という取り違えも起きない
euler = pd.DataFrame({
    'time': euler['time'], 'roll': roll_c, 'pitch': pitch_c, 'yaw': yaw_c, 't': euler['t'],
})

# 飛行区間の決定 ---------------------------------------------
# 姿勢とGPSの両方が揃っている時間帯に限る
t_lo = max(euler['t'].iloc[0], csv['t'].iloc[0])
t_hi = min(euler['t'].iloc[-1], csv['t'].iloc[-1])
if FLIGHT_START is None or FLIGHT_END is None:
    fly = csv[(csv['t'] >= t_lo) & (csv['t'] <= t_hi) & (csv['gs'] > AIRBORNE_GS)]
    if len(fly) == 0:
        raise ValueError(f"対地速度が{AIRBORNE_GS}m/sを超える区間がありません")
    t0 = fly['t'].iloc[0] if FLIGHT_START is None else time_to_daysec(FLIGHT_START)
    t1 = fly['t'].iloc[-1] if FLIGHT_END is None else time_to_daysec(FLIGHT_END)
else:
    t0, t1 = time_to_daysec(FLIGHT_START), time_to_daysec(FLIGHT_END)
t0, t1 = max(t0, t_lo), min(t1, t_hi)
print(f"飛行区間: {daysec_to_time(t0)} ～ {daysec_to_time(t1)}  ({t1 - t0:.0f}秒)")

# 等間隔グリッドへの補間と平滑化 -----------------------------
# 姿勢は微分してヨー角速度を求めるため、平滑化してから数値微分する
tg = np.arange(t0, t1, GRID_DT)
win = max(5, int(BETA_SMOOTH_SEC / GRID_DT) // 2 * 2 + 1)  # 奇数の窓幅


def to_grid(t_src, v_src, smooth=True):
    v = np.interp(tg, t_src, v_src)
    return savgol_filter(v, win, 2) if smooth else v


roll = to_grid(euler['t'], euler['roll'].to_numpy())
pitch = to_grid(euler['t'], euler['pitch'].to_numpy())
# 角度は360°の折り返しを展開してから補間・平滑化する
yaw_unwrap = np.degrees(np.unwrap(np.radians(euler['yaw'].to_numpy())))
yaw = to_grid(euler['t'], yaw_unwrap)
track_unwrap = np.degrees(np.unwrap(np.radians(csv['TrueTrack'].astype(float).to_numpy())))
track = to_grid(csv['t'], track_unwrap)
gs = to_grid(csv['t'], csv['gs'].to_numpy())

# オイラー角速度 → 機体角速度 -------------------------------
# 横力のつり合いは機体座標で立てるため、ヨー角速度も機体のz軸まわり(r)に直す
#   p = φ' - ψ'sinθ
#   q = θ'cosφ + ψ'cosθ sinφ
#   r = -θ'sinφ + ψ'cosθ cosφ
phi, theta = np.radians(roll), np.radians(pitch)
phi_dot = np.gradient(phi, tg)
theta_dot = np.gradient(theta, tg)
psi_dot = np.gradient(np.radians(yaw), tg)
r_body = -theta_dot * np.sin(phi) + psi_dot * np.cos(theta) * np.cos(phi)

# 対地速度ベクトル(北・東成分)
track_rad = np.radians(track)
vg_n = gs * np.cos(track_rad)
vg_e = gs * np.sin(track_rad)

# 水平・直進とみなせる区間 = Beta≈0 の教師データ
level = (np.abs(roll) < ROLL_LEVEL_THRESH) & \
        (np.abs(np.degrees(r_body)) < TURNRATE_LEVEL_THRESH)
print(f"水平直進とみなせるサンプル: {level.sum()} / {len(tg)} ({100 * level.mean():.0f}%)")
if level.sum() < 100:
    raise ValueError("水平直進区間が少なすぎます。判定しきい値を緩めてください")

# 機首方位の多様性を確認(風の三角形が解けるかの前提条件)
yaw_dirs = np.radians(yaw[level])
spread = np.hypot(np.mean(np.cos(yaw_dirs)), np.mean(np.sin(yaw_dirs)))
print(f"機首方位のばらつき: R={spread:.2f} (1.0=全部同じ方位で推定不能, 小さいほど良い)")


# 風・TAS・ヨーバイアスの推定 --------------------------------
# モデル:  対地速度 = TAS * u(Yaw + Beta + dpsi(t)) + 風(t)
# Beta は既知として与える。ヨーバイアス dpsi(t) は角度なので本来は非線形だが、
#   TAS*u(Yaw+dpsi) = a*u(Yaw) + b*u_perp(Yaw)   (a = TAScos(dpsi), b = TASsin(dpsi))
# と機体前方・右方の2成分に分解すると、a を定数(TASは一定)、b を時変とすることで
# すべての未知数について線形になる。復元は dpsi = atan2(b, a)。
# 未知数の並び: [a, b_0..b_(ny-1), Wn_0..Wn_(nw-1), We_0..We_(nw-1)]


def make_knots(interval):
    """区間全体を覆うノット時刻と、各サンプルの線形補間の重みを作る"""
    n = int(np.ceil((tg[-1] - tg[0]) / interval)) + 1
    kt = tg[0] + np.arange(n) * interval
    idx = np.clip(((tg - tg[0]) / interval).astype(int), 0, n - 2)
    w = (tg - kt[idx]) / interval
    return n, kt, idx, w


n_w, w_knots, w_idx, w_wt = make_knots(WIND_KNOT_SEC)          # 風
n_y, y_knots, y_idx, y_wt = make_knots(YAW_BIAS_KNOT_SEC)      # ヨーの時間ドリフト


def yaw_error_basis(psi_rad):
    """ヨー誤差を表す基底関数を並べた行列を返す。
    方位依存(コンパス自差)の項と、任意で時間ドリフトの項からなる。
    列の意味は yaw_basis_labels と対応する"""
    cols = []
    if YAW_CONST_TERM:
        cols.append(np.ones_like(psi_rad))
    for k in range(1, YAW_HARMONICS + 1):
        cols.append(np.sin(k * psi_rad))
        cols.append(np.cos(k * psi_rad))
    return cols


# 定数項と時間ドリフトはどちらも「一定のズレ」を表せるため、
# 両方を有効にすると未知数が重複して解が一意に定まらない
if YAW_CONST_TERM and YAW_ALLOW_DRIFT:
    raise ValueError("YAW_CONST_TERM と YAW_ALLOW_DRIFT は同時に有効にできません "
                     "(定数のズレを2通りで表すことになり解が定まりません)")

yaw_basis_labels = (['const'] if YAW_CONST_TERM else []) + \
    [f'{f}{k}psi' for k in range(1, YAW_HARMONICS + 1) for f in ('sin', 'cos')]
n_dev = len(yaw_basis_labels)                       # 方位依存の係数の数
n_yaw = n_dev + (n_y if YAW_ALLOW_DRIFT else 0)     # ヨー誤差の未知数の合計
OFS_Y, OFS_WN, OFS_WE = 1, 1 + n_yaw, 1 + n_yaw + n_w
n_unk = 1 + n_yaw + 2 * n_w
yaw_desc = ' + '.join(
    ([f"自差{'+'.join(yaw_basis_labels)}"] if n_dev else []) +
    ([f'時間ドリフト{n_y}個({YAW_BIAS_KNOT_SEC:.0f}秒間隔)'] if YAW_ALLOW_DRIFT else []))
print(f"未知数: TAS 1個 + ヨー誤差 {n_yaw}個[{yaw_desc}] "
      f"+ 風 {2 * n_w}個({WIND_KNOT_SEC:.0f}秒間隔)")


def diff_matrix(n, offset, n_blocks):
    """隣接ノットの差分をとる罰則行列。滑らかに変化させるために使う"""
    rows = []
    for blk in range(n_blocks):
        base = offset + blk * n
        for k in range(n - 1):
            row = np.zeros(n_unk)
            row[base + k] = -1.0
            row[base + k + 1] = 1.0
            rows.append(row)
    return np.array(rows)


reg_wind = diff_matrix(n_w, OFS_WN, 2)   # 風の北・東成分を滑らかに
# 自差の係数には微小なリッジをかける(暴れ防止)
reg_ridge = np.array([np.eye(n_unk)[OFS_Y + i] * np.sqrt(YAW_RIDGE) for i in range(n_dev)]
                     ).reshape(n_dev, n_unk)
# 時間ドリフトの滑らかさの罰則。強さは呼び出し時に指定する
reg_drift = diff_matrix(n_y, OFS_Y + n_dev, 1) if YAW_ALLOW_DRIFT else np.zeros((0, n_unk))


def solve_wind(beta_deg, mask, lam_yaw):
    """Beta を既知として TAS・ヨー誤差・時変風を最小二乗で解く。

    対気速度ベクトルを機体の前方成分と右方成分に分けると
      TAS * u(ψ + δψ) = a * u(ψ) + b * u_perp(ψ)
    となり、a = TAScos(δψ) ≈ TAS、b = TASsin(δψ) ≈ TAS*δψ。
    b をヨー誤差の基底関数の線形結合で表せば、全体が線形最小二乗になる。
    復元は δψ = atan2(b, a)。"""
    psi = np.radians(yaw[mask] + beta_deg[mask])
    n = int(mask.sum())
    A = np.zeros((2 * n, n_unk))
    b = np.empty(2 * n)
    rows = np.arange(n)
    sin_psi, cos_psi = np.sin(psi), np.cos(psi)
    # TAS(機体前方成分)
    A[2 * rows, 0] = cos_psi
    A[2 * rows + 1, 0] = sin_psi
    # ヨー誤差(機体右方成分 u_perp = (-sinψ, cosψ))。
    # 機首方位と一緒に向きが回るのが、地面に固定された風との決定的な違い
    for i, basis in enumerate(yaw_error_basis(psi)):
        A[2 * rows, OFS_Y + i] = -sin_psi * basis
        A[2 * rows + 1, OFS_Y + i] = cos_psi * basis
    if YAW_ALLOW_DRIFT:
        yi, yw = y_idx[mask], y_wt[mask]
        base = OFS_Y + n_dev
        A[2 * rows, base + yi] = -sin_psi * (1.0 - yw)
        A[2 * rows, base + yi + 1] = -sin_psi * yw
        A[2 * rows + 1, base + yi] = cos_psi * (1.0 - yw)
        A[2 * rows + 1, base + yi + 1] = cos_psi * yw
    # 風(地面に固定された向き。機首方位によらず同じ向きに効く)
    wi, ww = w_idx[mask], w_wt[mask]
    A[2 * rows, OFS_WN + wi] = 1.0 - ww
    A[2 * rows, OFS_WN + wi + 1] = ww
    A[2 * rows + 1, OFS_WE + wi] = 1.0 - ww
    A[2 * rows + 1, OFS_WE + wi + 1] = ww
    b[0::2] = vg_n[mask]
    b[1::2] = vg_e[mask]
    # 正則化を縦に積んで通常の最小二乗として解く
    A_aug = np.vstack([A, np.sqrt(WIND_LAMBDA) * reg_wind, reg_ridge,
                       np.sqrt(lam_yaw) * reg_drift])
    b_aug = np.concatenate([b, np.zeros(A_aug.shape[0] - A.shape[0])])
    sol, *_ = np.linalg.lstsq(A_aug, b_aug, rcond=None)
    a_c = sol[0]
    tas = float(abs(a_c))
    # 全サンプルについてヨー誤差を再構成する
    psi_all = np.radians(yaw + beta_deg)
    b_all = np.zeros_like(tg)
    for i, basis in enumerate(yaw_error_basis(psi_all)):
        b_all += sol[OFS_Y + i] * basis
    if YAW_ALLOW_DRIFT:
        b_all += np.interp(tg, y_knots, sol[OFS_Y + n_dev:OFS_Y + n_yaw])
    dpsi = np.degrees(np.arctan2(b_all, a_c))
    wn = np.interp(tg, w_knots, sol[OFS_WN:OFS_WN + n_w])
    we = np.interp(tg, w_knots, sol[OFS_WE:OFS_WE + n_w])
    return tas, dpsi, wn, we, sol[OFS_Y:OFS_Y + n_dev] / a_c




def fits_field_data(beta_deg, mask, lam_yaw):
    """その罰則での解が、現場の実測(風速の上限とTAS平均の範囲)に収まるか"""
    tas, _, wn, we, _ = solve_wind(beta_deg, mask, lam_yaw)
    tas_mean = float(np.hypot(vg_n - wn, vg_e - we).mean())
    return float(np.hypot(wn, we).max()) <= WIND_MAX and tas_mean <= TAS_MEAN_RANGE[1]


def tune_yaw_lambda(beta_deg, mask):
    """実測と整合する範囲で、ヨー誤差が最小になる(罰則が最大の)設定を選ぶ。

    罰則を強めるほどヨー誤差は小さくなるが、その分を風とTASが引き受けて
    両方が大きくなる。どちらの解でも当てはまり(残差)はほとんど変わらず、
    データだけでは選べないため、現場の実測値で決める。
    必要以上にヨー誤差のせいにしないよう、条件を満たす最大の罰則を採る。"""
    lo, hi = YAW_LAMBDA_RANGE
    if fits_field_data(beta_deg, mask, hi):
        return hi  # ヨー誤差をほぼ許さなくても実測と整合する
    if not fits_field_data(beta_deg, mask, lo):
        print("  注記: ヨー誤差を最大限許しても実測(風速・TAS)と整合しません。"
              "WIND_MAX や TAS_MEAN_RANGE を見直してください")
        return lo
    for _ in range(30):  # 条件を満たす最大の罰則に絞る
        mid = np.sqrt(lo * hi)
        if fits_field_data(beta_deg, mask, mid):
            lo = mid
        else:
            hi = mid
    return lo


# 横滑り運動方程式の積分 -------------------------------------
def integrate_beta(tau, gain, tas_series):
    """d(Beta)/dt = -Beta/tau + gain * drive を解く。
    drive = -r + (g/TAS)cos(pitch)sin(roll)  [rad/s]
    区間内で drive 一定とみなした厳密解(指数積分)で進める"""
    drive = -r_body + (GRAVITY / np.maximum(tas_series, 1.0)) * np.cos(theta) * np.sin(phi)
    decay = np.exp(-GRID_DT / tau)
    step = gain * tau * (1.0 - decay)
    out = np.empty_like(drive)
    y = 0.0
    for i in range(len(drive)):
        y = y * decay + step * drive[i]
        out[i] = y
    return np.degrees(out)


# 風から逆算した Beta(観測値に相当)
def beta_from_wind(dpsi, wn, we):
    va_n, va_e = vg_n - wn, vg_e - we
    psi_air = np.degrees(np.arctan2(va_e, va_n))
    return wrap180(psi_air - yaw - dpsi), np.hypot(va_n, va_e)


# 反復推定 ---------------------------------------------------
beta = np.zeros_like(tg)       # 初回は Beta=0 から出発
tas_series = np.full_like(tg, np.nan)
tau, gain = BETA_TAU, BETA_GAIN
lam_yaw = YAW_BIAS_LAMBDA

for it in range(N_ITER):
    # --- 風の推定。初回は水平直進区間のみ、以降は Beta が分かるので全区間を使う
    mask = level if it == 0 else np.ones_like(level, dtype=bool)
    if AUTO_YAW_LAMBDA and YAW_ALLOW_DRIFT:
        lam_yaw = tune_yaw_lambda(beta, mask)
    tas, dpsi, wn, we, dev_coef = solve_wind(beta, mask, lam_yaw)
    beta_wind, tas_series = beta_from_wind(dpsi, wn, we)

    # --- 横滑り運動方程式のパラメータを、風から求めた Beta に合うよう推定
    if AUTO_FIT_BETA:
        def cost(params):
            b_model = integrate_beta(params[0], params[1], tas_series)
            return float(np.sqrt(np.mean((b_model - beta_wind) ** 2)))

        res = minimize(cost, [tau, gain], method='Nelder-Mead',
                       bounds=[FIT_TAU_RANGE, FIT_BETA_GAIN_RANGE])
        tau, gain = float(res.x[0]), float(res.x[1])
    beta = integrate_beta(tau, gain, tas_series)

    wind_speed = np.hypot(wn, we)
    resid = beta_wind[level] - beta[level]
    print(f"反復{it + 1}: TAS={tas:.2f}m/s  ヨー誤差{dpsi.min():+.1f}〜{dpsi.max():+.1f}deg  "
          f"風速{wind_speed.min():.2f}〜{wind_speed.max():.2f}m/s  "
          f"TAU={tau:.2f}s GAIN={gain:.2f}  水平時の残差={resid.std():.2f}deg")

# 最終結果 ---------------------------------------------------
# 方位バイアスを取り除いた「真の機首方位」
yaw_true = yaw + dpsi
# 対気速度ベクトル = 対地速度 - 風  → これで DA と Beta が確定する
va_n, va_e = vg_n - wn, vg_e - we
psi_air = np.degrees(np.arctan2(va_e, va_n))
tas_series = np.hypot(va_n, va_e)

# Beta は運動方程式による滑らかな推定値を採用する。
# 風から逆算した Beta は GPS の分解能や風推定の誤差でばらつくため、
# そのノイズを Beta 側ではなく DA 側に残す方が物理的に自然
# (Beta は数度に収まる小さな量、DA は風で大きく振れる量)。
# こうすると TrueTrack = Yaw(True) + Beta + DA が厳密に成立する。
da = wrap180(track - yaw_true - beta)

wind_speed = np.hypot(wn, we)
# 風が吹いてくる方位。360°をまたぐと最大最小が意味をなさないので、
# 表示・保存用には連続値に展開したものを使う
wind_dir_unwrap = np.degrees(np.unwrap(np.arctan2(we, wn))) + 180.0
wind_dir = wind_dir_unwrap % 360.0

print(f"\n推定結果:")
print(f"  TAS(対気速度)   : 平均{tas_series.mean():.2f} m/s (標準偏差{tas_series.std():.2f}, "
      f"範囲{tas_series.min():.2f}〜{tas_series.max():.2f})")
# 現場の想定範囲と照合する。ここが合っていれば風とヨー誤差の切り分けも妥当といえる
ok_mean = TAS_MEAN_RANGE[0] <= tas_series.mean() <= TAS_MEAN_RANGE[1]
inside = np.mean((tas_series >= TAS_RANGE[0]) & (tas_series <= TAS_RANGE[1])) * 100
print(f"    想定平均{TAS_MEAN_RANGE[0]}〜{TAS_MEAN_RANGE[1]} m/s に対し "
      f"{'収まっている' if ok_mean else '外れている'}、"
      f"想定範囲{TAS_RANGE[0]}〜{TAS_RANGE[1]} m/s に入るサンプルは{inside:.1f}%")
print(f"  風速            : {wind_speed.min():.2f}〜{wind_speed.max():.2f} m/s (平均{wind_speed.mean():.2f})")
print(f"  風向(吹いてくる): {wind_dir_unwrap.min() % 360:.0f}°から{wind_dir_unwrap.max() % 360:.0f}°へ "
      f"(変化幅{wind_dir_unwrap.max() - wind_dir_unwrap.min():.0f}°)")
print(f"  Beta            : 平均{beta.mean():+.2f} deg (範囲 {beta.min():+.2f}〜{beta.max():+.2f})")
print(f"  DA(偏流角)      : 平均{da.mean():+.2f} deg (範囲 {da.min():+.2f}〜{da.max():+.2f})")
# ヨー角の誤差。地磁気補正が機体磁気の影響を受けて生じるズレ
print(f"  ヨー誤差        : {dpsi[0]:+.2f} → {dpsi[-1]:+.2f} deg "
      f"(範囲 {dpsi.min():+.2f}〜{dpsi.max():+.2f}, 変化幅{dpsi.max() - dpsi.min():.2f})")
print(f"    開始時の{dpsi[0]:+.2f}degが 静止時の方位の誤差、"
      f"その後の変化がドリフト")
if n_dev:
    print(f"    自差係数: " + "  ".join(
        f"{lab}={np.degrees(c):+.2f}deg" for lab, c in zip(yaw_basis_labels, dev_coef)))
    head_grid = np.arange(0, 360, 45)
    dev_at = np.zeros(len(head_grid))
    for i, basis in enumerate(yaw_error_basis(np.radians(head_grid.astype(float)))):
        dev_at += dev_coef[i] * basis
    print("    機首方位ごとの自差: " + " ".join(
        f"{h:3.0f}:{np.degrees(d):+5.1f}" for h, d in zip(head_grid, dev_at)))
if YAW_ALLOW_DRIFT:
    drift_rate = np.abs(np.gradient(dpsi, tg)) * 60.0  # [deg/分]
    print(f"    ドリフト速度: 平均{drift_rate.mean():.2f} deg/分 (最大{drift_rate.max():.2f})")
    if drift_rate.max() > YAW_DRIFT_WARN:
        print(f"    注記: 一部で{YAW_DRIFT_WARN}deg/分を超えています。"
              f"風を吸収しすぎている可能性があるので YAW_BIAS_LAMBDA を上げて感度を確認してください")

# 妥当性の確認 -----------------------------------------------
# 関係式が厳密に成立しているか(構成上ゼロになるはず)
identity = np.abs(wrap180(track - (yaw_true + beta + da))).max()
print(f"\n検証:")
print(f"  関係式 TrueTrack = Yaw + Beta + DA の最大誤差: {identity:.4f} deg")
# DA の幾何学的な上限。sin(DA) = 風速/対地速度 * sin(...) なので |DA| <= asin(風速/対地速度)
da_limit = np.degrees(np.arcsin(np.clip(wind_speed / np.maximum(gs, 0.1), -1, 1)))
over = np.abs(da) > da_limit + 1.0
print(f"  DAが幾何学的上限asin(風速/対地速度)を超えるサンプル: {100 * over.mean():.1f}%")
# 風から逆算したBetaと運動方程式のBetaの食い違い = ヨー角と風の推定誤差の目安
mismatch = wrap180(beta_wind - beta)
print(f"  Beta(風から) と Beta(運動方程式) の差: 標準偏差{mismatch.std():.2f} deg")
print(f"    -> この分がヨー角誤差・突風など、モデルで説明しきれない不確かさ")
if wind_speed.max() > WIND_MAX:
    print(f"  注記: 推定最大風速{wind_speed.max():.2f}m/sが当日の実測{WIND_MAX}m/sを"
          f"{wind_speed.max() - WIND_MAX:+.2f}m/s上回っています。")
    print(f"    ヨー誤差と風は互いに置き換えがきくため、YAW_BIAS_LAMBDA を下げると"
          f"風が減ってヨー誤差が増えます(当てはまりはほぼ変わりません)")

# 保存 -------------------------------------------------------
# 元の姿勢ログと同じ時刻に戻して保存する(後段が補間しやすいように)
out_t = euler['t'][(euler['t'] >= tg[0]) & (euler['t'] <= tg[-1])].to_numpy()
out = pd.DataFrame({
    'time': [daysec_to_time(s) for s in out_t],
    'beta': np.round(np.interp(out_t, tg, beta), 2),
    'da': np.round(np.interp(out_t, tg, da), 2),
    'wind_speed': np.round(np.interp(out_t, tg, wind_speed), 2),
    'wind_dir': np.round(np.interp(out_t, tg, wind_dir_unwrap) % 360, 1),
    'tas': np.round(np.interp(out_t, tg, tas_series), 2),
    'yaw_true': np.round(np.interp(out_t, tg, yaw_true) % 360, 2),
    'truetrack': np.round(np.interp(out_t, tg, track) % 360, 2),
})
out.to_csv(BETADA_SAVENAME, index=False)
print(f"推定結果を保存: {BETADA_SAVENAME}")

# 検証グラフ -------------------------------------------------
ts = (tg - tg[0]) / 60.0  # 経過時間 [分]
fig, axes = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
axes[0].plot(ts, roll, color='red', linewidth=0.8, label='roll [deg]')
axes[0].axhline(0, color='gray', linewidth=0.5)
axes[0].set_ylabel('roll [deg]')
axes[1].plot(ts, beta_wind, color='lightgray', linewidth=0.8, label='beta (from wind)')
axes[1].plot(ts, beta, color='darkgreen', linewidth=1.2, label='beta (dynamics model)')
axes[1].axhline(0, color='gray', linewidth=0.5)
axes[1].set_ylabel('beta [deg]')
axes[2].plot(ts, da, color='blue', linewidth=0.8, label='drift angle')
axes[2].axhline(0, color='gray', linewidth=0.5)
axes[2].set_ylabel('DA [deg]')
axes[3].plot(ts, wind_speed, color='purple', label='wind speed [m/s]')
axes[3].plot(ts, tas_series, color='orange', linewidth=0.8, label='TAS [m/s]')
axes[3].plot(ts, gs, color='gray', linewidth=0.8, label='GS [m/s]')
axes[3].set_ylabel('speed [m/s]')
# 方位は360°の折り返しでグラフが飛ぶため、連続値のまま描く
axes[4].plot(ts, wind_dir_unwrap, color='brown', label='wind from [deg]')
axes[4].plot(ts, np.degrees(np.unwrap(np.radians(yaw_true))), color='lightblue',
             linewidth=0.6, label='yaw(true) [deg]')
axes[4].set_ylabel('direction [deg]')
# 地磁気補正の誤差によるヨー角のドリフト
axes[5].plot(ts, dpsi, color="magenta", label="yaw error (compass deviation) [deg]")
axes[5].axhline(0, color='gray', linewidth=0.5)
axes[5].set_ylabel('yaw error [deg]')
axes[5].set_xlabel('elapsed time [min]')
for ax in axes:
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(alpha=0.3)
fig.suptitle('Beta / Drift Angle / Wind estimation check')
fig.tight_layout()
fig.savefig(BETADA_PLOTNAME, dpi=110)
print(f"検証グラフを保存: {BETADA_PLOTNAME}")
