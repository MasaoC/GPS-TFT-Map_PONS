import re
import subprocess
import sys
import tempfile
import time as timertime
from pathlib import Path

import pandas as pd
import matplotlib
matplotlib.use('Agg')   # 画面を持たない書き出し専用。並列実行しても窓が出ない
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
import matplotlib.patheffects as path_effects
from scipy.interpolate import interp1d
from scipy.optimize import linprog
from scipy.signal import butter, filtfilt

# Last update 2026/7/27

_t_render_start = timertime.time()

FILENAME = '2026-07-26_0804.csv'
EULER_FILENAME = '20260726_compensated.txt'  # flight_preprocess.py の出力(補正済み姿勢角)
LOG_FILENAME = 'log_edit.txt'  # システムログ。電圧・GPS精度・衛星数をCSVと同期表示する
BETA_DA_FILENAME = 'beta_da_20260726.txt'  # flight_preprocess.py の出力(β・偏流角・風・TAS)
SAVENAME = '0726.mp4'        # all のときに出来上がる完成品の名前
LOGDATE = "2026-07-11"
starttime = "08:21:15"
#stoptime = "08:22:15"
stoptime = "09:07:48"

# 分割書き出しの設定 -----------------------------------------
# 全長46分をそのまま1プロセスで描くと非常に時間がかかる。
# matplotlibの描画は実質シングルスレッドで1コアを使い切るため、
# 区間を分けて別プロセスで同時に走らせると本数ぶん速くなる。
#   python3 createmovie_csv2mp4.py       … part0(確認用の短尺)
#   python3 createmovie_csv2mp4.py 3     … part3 だけ
#   python3 createmovie_csv2mp4.py all   … 全パートを並列に書き出して1本に結合する
N_PARTS = 6                  # 分割数。CPUのコア数までなら増やすほど速い
PART0_SEC = 12.0             # part0(動作確認用)の長さ [秒]
PART_PREFIX = '0726_part'    # パート動画のファイル名の頭
FRAME_INTERVAL = 0.25        # 1フレームあたりの秒数(fps = 1/これ)

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
# 飛行中に高度が正確に分かっている時刻があれば ('HH:MM:SS', 高度[m]) で並べる。
# ドリフト曲線がこれらの点をちょうど通るように拘束するので、
# 上下限だけで決めるより実際の高度に近づく。空リストなら離着水の2点だけで決める。
KF_ALT_FIXES = [
    ('08:32:56', 6.0),
    ('08:39:32', 8.0),
    ('08:52:35', 4.0),
    ('09:00:18', 1.0),
]
# 拘束点の補正量を求めるとき、その時刻の1サンプルだけを見るとノイズをそのまま
# 焼き付けてしまうため、前後この幅[秒]で平均した高度を使う
KF_ALT_FIX_WINDOW_SEC = 4.0
# ドリフト曲線の折れ点の間隔 [分]。粗いほど滑らかだが条件を満たせないことがある。
# 先頭から順に試し、条件を満たせた最初の(最も滑らかな)ものを使う
KF_ALT_KNOT_CANDIDATES = [10.0, 7.0, 5.0, 4.0, 3.0, 2.0, 1.5, 1.0]


# ベクトル図(風の三角形)の設定 -------------------------------
# 矢印を中心で回すのではなく、物理的に正しいベクトルの足し算として描く:
#   原点(機首) …Yaw…   機首の向き(点線。向きの参照だけで、長さに意味はない)
#   原点(機首) --TAS--> 対気移動ベクトル(向き = Yaw + β、長さ = TAS)
#   TASの先端  --WIND-> 風ベクトル(風が吹いていく向き、長さ = 風速)
#   原点       --TT---> 対地移動ベクトル(= 風ベクトルの先端。三角形が必ず閉じる)
# 原点には機首を原点に置いた機体アイコンを描き、機首方位に合わせて回す。
VEC_ORIGIN = (0.0, -0.10)   # 原点(機体の機首の位置)。tt_axの座標系
VEC_SCALE = 0.14            # 1 m/s あたりの矢印の長さ。全ベクトル共通なので長さを比較できる
VEC_REF_MPS = 5.0           # スケールバーの基準速度 [m/s]
VEC_NOEST_LEN_MPS = 6.0     # 推定値が無い区間で機首方位の点線に使う仮の長さ [m/s]
VEC_YAW_LEN_FACTOR = 1.18   # 機首方位の点線の長さ。TAS矢印の何倍にするか
# 矢印の軸の太さ。TT(実測)を一番太くして主従を付ける
VEC_W_AIR, VEC_W_WIND, VEC_W_TT = 0.055, 0.055, 0.075

# 機体アイコン(漢字の「士」のような平面形)の寸法。
# すべて tt_ax の座標系の固定値で、速度とは無関係
PLANE_LEN = 0.30       # 機首(原点)から尾部までの長さ
PLANE_WING_POS = 0.09  # 機首から主翼までの距離
PLANE_WING = 0.24      # 主翼の半スパン
PLANE_TAIL = 0.09      # 尾翼の半スパン

# G/S グラフの設定 -------------------------------------------
# 横バー表示だと「今いくつか」しか分からず、数秒間でどう振れているかが読めない。
# そこで左へ流れていくスクロールグラフ(ストリップチャート)にする。
# 右端(x=0)が現在時刻で、左へ行くほど過去。GS_WINDOW_SEC 秒で画面外に消える
GS_WINDOW_SEC = 10.0    # グラフの横幅 [秒]
GS_YMIN, GS_YMAX = 3.0, 9.0   # 縦軸の範囲 [m/s]。範囲外の値は枠外に切れる
GS_PLOT_DT = 0.1        # 波形を描くときのサンプル間隔 [秒]。細かいほど滑らか
GS_XTICK_SEC = 2.0      # 縦のグリッド線(時間目盛)の間隔 [秒]

# Roll/Pitch グラフの設定 ------------------------------------
# G/Sと同じ形式のスクロールグラフ。横幅・サンプル間隔はG/Sと共通なので、
# 上下のグラフで同じ時刻が同じ横位置に並ぶ
RP_YMIN, RP_YMAX = -8.0, 8.0     # 縦軸の範囲 [deg]。実績の振れ幅(±7.3度)がぎりぎり収まる
RP_YTICK = 4.0                   # 横のグリッド線の間隔 [deg]
ROLL_COLOR = 'crimson'
PITCH_COLOR = 'darkviolet'

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


# 分割書き出しとパートの結合 ---------------------------------
TOTAL_STOP = time_to_seconds(stoptime, starttime)   # 全長 [秒]
# 区間の境目はフレームの切れ目にそろえる。こうしておくと
# パートを繋いだときにフレームの重複も抜けも起きない
_edges = np.round(np.linspace(0.0, TOTAL_STOP, N_PARTS + 1) / FRAME_INTERVAL) * FRAME_INTERVAL
# part0 は動作確認用の短尺。1以降が本番のパート
PARTS = {0: (0.0, min(PART0_SEC, TOTAL_STOP))}
PARTS.update({k + 1: (_edges[k], _edges[k + 1]) for k in range(N_PARTS)})
ALL_PARTS = [p for p in sorted(PARTS) if p != 0]


def part_filename(partnum):
    return f"{PART_PREFIX}{partnum}.mp4"


def concat_parts(output, inputs):
    """パート動画を1本に結合する。
    同じスクリプトから出したのでコーデック・解像度・fpsが揃っており、
    ffmpeg の concat demuxer で再エンコードせずに繋げられる
    (一瞬で終わり、画質も落ちない)"""
    missing = [p for p in inputs if not Path(p).exists()]
    if missing:
        raise SystemExit(f"パート動画が見つかりません: {missing}")
    # concat demuxer に渡すリストファイル。パスは絶対パスにしておく
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as listfile:
        for path in inputs:
            resolved = str(Path(path).resolve()).replace("'", r"'\''")
            listfile.write(f"file '{resolved}'\n")
        listname = listfile.name
    try:
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error',
                        '-f', 'concat', '-safe', '0', '-i', listname,
                        '-c', 'copy', output], check=True)
    finally:
        Path(listname).unlink()
    print(f"{output} を書き出しました ({len(inputs)}本を結合)")


def render_all_parts():
    """全パートを別プロセスで同時に走らせ、終わったら1本に結合する。
    パート同士は独立していて、各プロセスはmatplotlibの描画で1コアを
    使い切る実質シングルスレッドなので、並列にすると本数ぶん速くなる"""
    procs = {p: subprocess.Popen([sys.executable, __file__, str(p)]) for p in ALL_PARTS}
    print(f"part{ALL_PARTS[0]}〜{ALL_PARTS[-1]} を {len(procs)}並列で書き出します "
          f"(PID {', '.join(str(q.pid) for q in procs.values())})")
    failed = [p for p, q in procs.items() if q.wait() != 0]
    if failed:
        raise SystemExit(f"part{failed} の書き出しに失敗しました")
    print(f"全{len(procs)}本の書き出しが完了しました "
          f"(合計 {timertime.time() - _t_render_start:.1f}秒)")
    concat_parts(SAVENAME, [part_filename(p) for p in ALL_PARTS])
    print(f"完了 (合計 {timertime.time() - _t_render_start:.1f}秒)")


_arg = sys.argv[1] if len(sys.argv) > 1 else '0'
if _arg == 'all':
    render_all_parts()
    raise SystemExit(0)
if not (_arg.isdigit() and int(_arg) in PARTS):
    raise SystemExit(f"引数は {sorted(PARTS)} か 'all' です (指定: {_arg})")
PARTNUM = int(_arg)
PART_START, PART_STOP = PARTS[PARTNUM]
PART_SAVENAME = part_filename(PARTNUM)

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

# 補正なしの生の高度 -----------------------------------------
# 動画には Alt(KF_C) と並べて、加工していない気圧高度・GNSS高度も出す。
# どちらも補正の類を一切かけていないので、Alt(KF_C)との差が
# そのまま「補正で動かした量」および各センサのドリフトになる。
_ts_all = data['time_seconds'].to_numpy()
_t_land_b = time_to_seconds(KF_ALT_LAND_TIME, starttime)
# 着水時の気圧を基準にして、気圧から高度を計算する(国際標準大気の式)
_p = data['pressure'].to_numpy()
_p_ref = float(np.interp(_t_land_b, _ts_all, _p))
_baro = 44330.0 * (1.0 - (_p / _p_ref) ** (1.0 / 5.255)) + KF_ALT_LAND_TARGET
data['Baro_Alt'] = _baro
# GNSS高度は CSV の GNSS_Altitude(= NAV-PVT hMSL、ジオイド基準のMSL高度)。
# 琵琶湖面(B.S.L.)基準に直すため、T.P.での湖面標高を引く。
# 値は settings.h の ELEVATION_GEOID_OFFSET_M と同じものを使う
BSL_OFFSET_M = 84.371
_gnss_raw = data['Altitude'].to_numpy()
data['GNSS_Alt'] = _gnss_raw - BSL_OFFSET_M
print(f"生の高度の基準: 気圧{_p_ref:.2f}hPa(着水時)、GNSSはB.S.L.({BSL_OFFSET_M:.3f}m)基準")

# 気圧高度とKF高度の合成 -------------------------------------
if KF_ALT_FUSE_BARO:

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

def alt_around(ts, alt, t, half_width):
    """時刻tの前後half_width秒の高度の平均。1点だけ見るとノイズを拾うため"""
    sel = np.abs(ts - t) <= half_width
    return float(alt[sel].mean()) if sel.any() else float(np.interp(t, ts, alt))


def solve_alt_drift(ts, alt, t_takeoff, t_land, knot_min, fixes=()):
    """高度が KF_ALT_MIN〜KF_ALT_MAX に収まり、離陸時は補正0、
    着水時にちょうど KF_ALT_LAND_TARGET になるようなドリフト曲線のうち、
    最も滑らかなものを線形計画法で求める。求まらなければ None を返す。

    fixes は (時刻[秒], その時刻の正しい高度[m]) の並び。
    与えると、補正後の高度がその値にちょうど一致するよう追加で拘束する。

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
    def knot_row(t):
        """時刻tのドリフト量を、折れ点の線形補間で表す行ベクトル"""
        j = int(np.clip((t - t_takeoff) / step, 0, n_k - 2))
        wt = (t - knots[j]) / step
        row = np.zeros(n_k + n_u)
        row[j], row[j + 1] = 1.0 - wt, wt
        return row

    # 離陸時のドリフトは0(既存のkf_alt_offsetの校正をそのまま活かす)、
    # 着水時のドリフトは表示が KF_ALT_LAND_TARGET になる値に固定する。
    # 高度が分かっている時刻(fixes)も同じように「補正後にその値になる」で拘束する
    eq_rows = [knot_row(t_takeoff), knot_row(t_land)]
    b_eq = [0.0, float(np.interp(t_land, ts, alt)) - KF_ALT_LAND_TARGET]
    for t_fix, alt_fix in fixes:
        eq_rows.append(knot_row(t_fix))
        b_eq.append(alt_around(ts, alt, t_fix, KF_ALT_FIX_WINDOW_SEC / 2.0) - alt_fix)
    A_eq = np.vstack(eq_rows)
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
    # 高度が分かっている時刻を秒に直す。飛行区間の外は拘束できないので弾く
    _fixes = []
    for _tstr, _av in KF_ALT_FIXES:
        _tf = time_to_seconds(_tstr, starttime)
        if not (_t_takeoff <= _tf <= _t_land):
            raise ValueError(f"KF_ALT_FIXES の {_tstr} が離陸〜着水の外です")
        _fixes.append((_tf, float(_av)))
    for _knot_min in KF_ALT_KNOT_CANDIDATES:
        _knots, _drift = solve_alt_drift(_ts, _alt, _t_takeoff, _t_land, _knot_min, _fixes)
        if _knots is not None:
            break
    if _knots is None:
        raise ValueError("高度の上下限を満たすドリフト曲線が見つかりません。"
                         "KF_ALT_MIN/KF_ALT_MAX や離着水時刻、KF_ALT_FIXES の値を"
                         "見直してください(折れ点を細かくしても通せない拘束があります)")
    data['KF_Alt_c'] = data['KF_Alt_rel'] - np.interp(_ts, _knots, _drift)
    _fl = (_ts >= _t_takeoff) & (_ts <= _t_land)
    print(f"KF高度の気圧ドリフト補正: 折れ点{_knot_min:.0f}分間隔({len(_knots)}点)で解けました。"
          f"ドリフト量{_drift.min():+.2f}〜{_drift.max():+.2f}m")
    print(f"  補正後の飛行中の高度: {data['KF_Alt_c'][_fl].min():+.2f}〜"
          f"{data['KF_Alt_c'][_fl].max():+.2f}m "
          f"(離陸時{np.interp(_t_takeoff, _ts, data['KF_Alt_c']):+.1f}m, "
          f"着水時{np.interp(_t_land, _ts, data['KF_Alt_c']):+.2f}m)")
    # 指定した時刻でちょうどその高度になっているかの確認
    for (_tf, _av), (_tstr, _) in zip(_fixes, KF_ALT_FIXES):
        _got = alt_around(_ts, data['KF_Alt_c'].to_numpy(), _tf, KF_ALT_FIX_WINDOW_SEC / 2.0)
        print(f"  拘束点 {_tstr}: 指定{_av:.1f}m → 補正後{_got:+.2f}m "
              f"(差{_got - _av:+.2f}m)")
else:
    data['KF_Alt_c'] = data['KF_Alt_rel']

# 姿勢角データの読み込み。CSVと同じ開始時刻を基準に秒へ変換して同期する
euler = pd.read_csv(EULER_FILENAME)
euler['time_seconds'] = euler['time'].apply(lambda x: time_to_seconds(x, starttime))
EULER_T0 = float(euler['time_seconds'].iloc[0])  # 姿勢角の最初の時刻。これより前はグラフに出さない

# β・偏流角・風・TASの推定結果を読み込む(flight_preprocess.py の出力)。
# 飛行区間のみのデータなので、離着陸前後は端の値がそのまま伸びる点に注意
betada = pd.read_csv(BETA_DA_FILENAME)
betada['time_seconds'] = betada['time'].apply(lambda x: time_to_seconds(x, starttime))

# システムログ(log.txt)の解析 --------------------------------
# 各行は「起動後秒数:メッセージ」形式。GPS TIME行(UTC)を+9時間でJSTに変換し、
# 起動後秒数→動画基準の秒数へ対応付ける。
# ファイル先頭に再起動前のログが残っていることがあるため、"0:SD INIT"を見たら読み直す
log_volt_rows = []   # (起動後秒数, 電圧V, CPU温度C)
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
        # 例: "60:volt=4.08V cpu=30.9C 08:05 JST"。CPU温度は無い書式のログもあるので任意扱い
        m = re.match(r'(\d+):volt=([\d.]+)V(?: cpu=([\d.]+)C)?', line)
        if m:
            log_volt_rows.append((int(m.group(1)), float(m.group(2)),
                                  float(m.group(3)) if m.group(3) else float('nan')))
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
frame_interval = FRAME_INTERVAL
# このプロセスが描くのは担当パートの区間だけ
new_time = np.arange(PART_START, PART_STOP, frame_interval)
# 地図の航跡は「飛行の最初から現在まで」を描くので、パートの区間ではなく
# 全長の時間軸を使う(これがないとパートの頭で航跡が消えてしまう)
trail_time = np.arange(0.0, TOTAL_STOP, frame_interval)
print(f"part{PARTNUM}: {PART_START:.2f}〜{PART_STOP:.2f}秒 ({len(new_time)}フレーム) → {PART_SAVENAME}")

# Convert TrueTrack to radians for sine and cosine components
data['TrueTrack_rad'] = np.deg2rad(data['TrueTrack'])
data['sin_truetrack'] = np.sin(data['TrueTrack_rad'])
data['cos_truetrack'] = np.cos(data['TrueTrack_rad'])

# Create interpolation functions for sin(TrueTrack), cos(TrueTrack), gs, latitude, and longitude
interp_sin_truetrack = interp1d(data['time_seconds'], data['sin_truetrack'], kind='linear', fill_value="extrapolate")
interp_cos_truetrack = interp1d(data['time_seconds'], data['cos_truetrack'], kind='linear', fill_value="extrapolate")
interp_gs = interp1d(data['time_seconds'], data['gs'], kind='linear', fill_value="extrapolate")
GS_T0 = float(data['time_seconds'].iloc[0])  # CSVの最初の時刻。これより前は外挿なのでグラフに出さない
interp_latitude = interp1d(data['time_seconds'], data['latitude'], kind='linear', fill_value="extrapolate")
interp_longitude = interp1d(data['time_seconds'], data['longitude'], kind='linear', fill_value="extrapolate")
# 表示用は実際の経度。地図描画用(スケール済み)と取り違えないこと
interp_longitude_true = interp1d(data['time_seconds'], data['longitude_true'], kind='linear', fill_value="extrapolate")
# 新規追加データ(KF高度・垂直速度・気圧)の補間
interp_kf_alt = interp1d(data['time_seconds'], data['KF_Alt_c'], kind='linear', fill_value="extrapolate")
interp_kf_vspeed = interp1d(data['time_seconds'], data['KF_Vspeed'], kind='linear', fill_value="extrapolate")
interp_pressure = interp1d(data['time_seconds'], data['pressure'], kind='linear', fill_value="extrapolate")
# 無加工の高度2種(どちらも着水時0m基準)
interp_baro_alt = interp1d(data['time_seconds'], data['Baro_Alt'], kind='linear', fill_value="extrapolate")
interp_gnss_alt = interp1d(data['time_seconds'], data['GNSS_Alt'], kind='linear', fill_value="extrapolate")

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
# ベクトル図に使うのは beta_air / da_air の方。
# beta(運動方程式による滑らかな推定値)で対気ベクトルを描くと、
# 実際の対気ベクトルの向き(psi_air)とは平均3度ほどずれ、三角形が閉じない。
# beta_air = psi_air - yaw_true なので、こちらで描けば
#   対地ベクトル = 対気ベクトル + 風ベクトル
# が厳密に成立する。詳しくは flight_preprocess.py の beta_air 定義部を参照
BETADA_HAS_AIR = {'beta_air', 'da_air'} <= set(betada.columns)
if BETADA_HAS_AIR:
    interp_beta_air = interp_bd('beta_air')
    interp_da_air = interp_bd('da_air')
else:
    print(f"注記: {BETA_DA_FILENAME} に beta_air/da_air がありません(古い形式)。"
          "beta/da で代用するため、ベクトルの三角形が数度ぶん閉じません。"
          "flight_preprocess.py を実行し直すと解消します")
    interp_beta_air, interp_da_air = interp_beta, interp_da
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
interp_cputemp = interp1d(log_volt_t, [r[2] for r in log_volt_rows], kind='previous', bounds_error=False, fill_value=(log_volt_rows[0][2], log_volt_rows[-1][2]))
interp_hacc = interp1d(log_acc_t, [r[1] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][1], log_acc_rows[-1][1]))
interp_vacc = interp1d(log_acc_t, [r[2] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][2], log_acc_rows[-1][2]))
interp_sacc = interp1d(log_acc_t, [r[3] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][3], log_acc_rows[-1][3]))
interp_sats = interp1d(log_acc_t, [r[4] for r in log_acc_rows], kind='previous', bounds_error=False, fill_value=(log_acc_rows[0][4], log_acc_rows[-1][4]))

# Initialize figure and axes
# 高さの比率。tt_ax は縦横比が正方形に固定されるため、ここを増やすとベクトル図が大きくなる。
# 上から ベクトル図 / G/Sグラフ / Roll・Pitchグラフ / 地図
fig, (tt_ax, gs_ax, rp_ax, map_ax) = plt.subplots(4, 1, figsize=(8, 9), facecolor='green', gridspec_kw={'height_ratios': [0.54, 0.16, 0.14, 0.24]})
# 余白を詰めて、ベクトル図(正方形に固定される tt_ax)にできるだけ大きさを確保する
fig.subplots_adjust(left=0.055, right=0.98, top=0.935, bottom=0.05, hspace=0.25)
tt_ax.set_xlim(-1, 1)
tt_ax.set_ylim(-1, 1)
tt_ax.set_aspect('equal')
tt_ax.set_facecolor('green')
tt_ax.grid(False)
tt_ax.xaxis.set_visible(False)
tt_ax.yaxis.set_visible(False)
for spine in tt_ax.spines.values():
    spine.set_visible(False)

# G/S グラフの軸設定。update() 内で gs_ax.clear() するたびに同じ設定が要るので関数にまとめる
def setup_gs_axis():
    gs_ax.set_xlim(-GS_WINDOW_SEC, 0)          # 右端が現在、左が過去
    gs_ax.set_ylim(GS_YMIN, GS_YMAX)
    # 時間目盛は「何秒前か」。0(現在)は目盛り数字を出さず、右端が今であることは
    # 大きなデジタル表示で分かるようにする
    gs_ax.set_xticks(np.arange(-GS_WINDOW_SEC, 0.001, GS_XTICK_SEC))
    gs_ax.set_xticklabels([f"{int(t)}" if t < 0 else "" for t in
                           np.arange(-GS_WINDOW_SEC, 0.001, GS_XTICK_SEC)])
    gs_ax.set_yticks(np.arange(GS_YMIN, GS_YMAX + 0.001, 1.0))
    gs_ax.tick_params(axis='x', labelsize=11)
    gs_ax.tick_params(axis='y', labelsize=13)
    gs_ax.set_facecolor('lightgreen')
    gs_ax.grid(True, color='white', linewidth=0.8, alpha=0.8)

# Roll/Pitch グラフの軸設定。時間軸はG/Sと共通で、真下に並べて同じ時刻を縦にそろえる。
# 秒の目盛り数字は下のこちらにだけ付ける(2つ付けると煩いため)
def setup_rp_axis():
    rp_ax.set_xlim(-GS_WINDOW_SEC, 0)
    rp_ax.set_ylim(RP_YMIN, RP_YMAX)
    rp_ax.set_xticks(np.arange(-GS_WINDOW_SEC, 0.001, GS_XTICK_SEC))
    rp_ax.set_xticklabels([f"{int(t)}" if t < 0 else "" for t in
                           np.arange(-GS_WINDOW_SEC, 0.001, GS_XTICK_SEC)])
    rp_ax.set_yticks(np.arange(RP_YMIN, RP_YMAX + 0.001, RP_YTICK))
    rp_ax.tick_params(axis='x', labelsize=11)
    rp_ax.tick_params(axis='y', labelsize=13)
    rp_ax.set_xlabel("sec", fontsize=11, labelpad=0)
    rp_ax.set_facecolor('lightgreen')
    rp_ax.grid(True, color='white', linewidth=0.8, alpha=0.8)
    rp_ax.axhline(0, color='white', linewidth=1.8)  # 水平(0°)の基準線

setup_gs_axis()
setup_rp_axis()

# Adjust the map subplot
set_map_limits(map_ax)
map_ax.set_facecolor('green')
for spine in map_ax.spines.values():
    spine.set_visible(False)

# Plot settings
# 見出し・数値の文字は fig.text(図全体の座標)で置く。
# tt_ax は aspect='equal' のため軸の枠の大きさが図の比率で変わってしまい、
# 軸座標(transAxes)で置くとベクトル図の大きさを変えるたびに文字位置がずれるため
# 機体名。TrueTrackの真上に、同じ右寄せで置く
fig.text(0.985, 0.998, "PONS v6", horizontalalignment='right', verticalalignment='top',
         color='black', fontsize=26, fontweight='bold',
         path_effects=[path_effects.withStroke(linewidth=5, foreground='white')])
# TrueTrackは右寄せ。左上を編集用の余白として空けておくため
tt_text = fig.text(0.985, 0.935, "", horizontalalignment='right', verticalalignment='top', color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
latlon_text = fig.text(0.44, 0.542, "", horizontalalignment='left', verticalalignment='center', color='black', fontsize=16, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
# 現在値のデジタル表示は、それぞれのグラフの枠の中に置く。
# 半透明の白地を敷いてあるので、波形が下を通っても文字が読める
_readout_bg = dict(facecolor='white', alpha=0.72, edgecolor='none', pad=2.0)
def make_readout(ax, x, y, color, fontsize):
    return ax.text(x, y, "", verticalalignment='top', horizontalalignment='left',
                   transform=ax.transAxes, color=color, fontsize=fontsize,
                   fontweight='bold', bbox=_readout_bg)

# 2つの値は横並びではなく上下2段にする(横に並べると波形の見える幅が狭くなるため)
gs_text = make_readout(gs_ax, 0.015, 0.97, 'black', 20)          # 対地速度(実測)
tas_text = make_readout(gs_ax, 0.015, 0.55, 'darkorange', 16)    # 対気速度(推定。オレンジの破線)
roll_text = make_readout(rp_ax, 0.015, 0.97, ROLL_COLOR, 17)
pitch_text = make_readout(rp_ax, 0.015, 0.55, PITCH_COLOR, 17)
# 時刻は編集の都合で入れている情報なので目立たせない。
# 緯度経度の左隣に、同じ大きさで並べる
time_text = fig.text(0.015, 0.542, "", horizontalalignment='left', verticalalignment='center', color='black', fontsize=16, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
# 矢印とテキストの色を対応させる。どの矢印がどの値なのか一目で分かるようにする
YAW_COLOR = 'deepskyblue'   # 機首方位(Yaw)
BETA_COLOR = 'yellow'       # 機首方位+β = 対気速度ベクトルの向き(偏流を受ける前の進行方向)
TRACK_COLOR = 'black'       # 実際の対地進行方向(TrueTrack)

# 行ごとに色を変えられるよう、1行を1つのテキストとして作る。
# まとめて1つの複数行テキストにすると行単位で色を変えられないため
COL_LINE_DY = 0.032  # 行間(図全体の座標系)。詰めすぎると下の緯度経度と重なる
def make_column(x, y_top, colors, fontsize=15):
    arts = []
    for k, color in enumerate(colors):
        # 黄色は白フチだと見えにくいので、明るい色には黒フチを合わせる
        stroke = 'black' if color in (BETA_COLOR,) else 'white'
        arts.append(fig.text(x, y_top - k * COL_LINE_DY, "",
                             horizontalalignment='left', verticalalignment='top',
                             color=color, fontsize=fontsize, fontweight='bold',
                             path_effects=[path_effects.withStroke(linewidth=4, foreground=stroke)]))
    return arts

# ベクトル図の描画ヘルパー -----------------------------------
def _tip(x0, y0, bearing_deg, mps):
    """(x0,y0)から方位bearing_deg・大きさmpsだけ進んだ点。
    方位は北=0°・東=90°なので、画面のx(東)/y(北)成分は sin/cos で求まる"""
    r = mps * VEC_SCALE
    rad = np.deg2rad(bearing_deg)
    return (x0 + r * np.sin(rad), y0 + r * np.cos(rad))

def draw_plane(x0, y0, bearing_deg):
    """原点を機首とする人力飛行機の平面形(漢字の「士」のような形)を描く。
    胴体・主翼・尾翼の3本の線を、機首方位に合わせて回して置く"""
    rad = np.deg2rad(bearing_deg)
    fx, fy = np.sin(rad), np.cos(rad)    # 機首の向き(前)
    rx, ry = np.cos(rad), -np.sin(rad)   # 右翼の向き(前から90度右)
    def pt(back, side):
        """機首から後ろにback、右にsideだけ離れた点"""
        return (x0 - fx * back + rx * side, y0 - fy * back + ry * side)
    segs = [(pt(0, 0), pt(PLANE_LEN, 0)),                             # 胴体
            (pt(PLANE_WING_POS, -PLANE_WING), pt(PLANE_WING_POS, PLANE_WING)),  # 主翼
            (pt(PLANE_LEN, -PLANE_TAIL), pt(PLANE_LEN, PLANE_TAIL))]  # 尾翼
    # NaNで区切ると3本を1つのLine2Dとしてまとめて描ける
    xs, ys = [], []
    for p0, p1 in segs:
        xs += [p0[0], p1[0], np.nan]
        ys += [p0[1], p1[1], np.nan]
    return tt_ax.plot(xs, ys, color=YAW_COLOR, linewidth=4, solid_capstyle='round',
                      clip_on=False,
                      path_effects=[path_effects.withStroke(linewidth=7, foreground='white')])

def _polar(x0, y0, x1, y1):
    """2点を結ぶベクトルを (方位[deg], 大きさ[m/s]) に戻す。_tip の逆変換"""
    dx, dy = x1 - x0, y1 - y0
    return np.degrees(np.arctan2(dx, dy)) % 360.0, np.hypot(dx, dy) / VEC_SCALE

def draw_vector(x0, y0, bearing_deg, mps, width, **kw):
    """(x0,y0)を始点に、方位bearing_deg[deg]・大きさmps[m/s]の矢印を1本描く。
    戻り値は (矢印, 終点)。終点を次の矢印の始点にすればベクトルを繋げられる。
    方位は北=0°・東=90°なので、画面のx(東)/y(北)成分は sin/cos で求まる"""
    r = mps * VEC_SCALE
    rad = np.deg2rad(bearing_deg)
    dx, dy = r * np.sin(rad), r * np.cos(rad)
    # 短いベクトル(弱い風など)で矢じりだけが目立たないよう、矢じりは長さに応じて小さくする
    head_l = min(width * 2.6, r * 0.5)
    head_w = min(width * 2.8, r * 0.45)
    arrow = FancyArrow(x0, y0, dx, dy, length_includes_head=True, width=width,
                       head_width=head_w, head_length=head_l, clip_on=False, **kw)
    tt_ax.add_patch(arrow)
    return arrow, (x0 + dx, y0 + dy)

def _label_pos(p0, p1, offset):
    """ベクトルの中点から、ベクトルと直交する向きにoffsetだけずらした位置。
    矢印の上に文字が重ならないようにするため"""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = np.hypot(dx, dy)
    if n < 1e-9:
        return p0
    return ((p0[0] + p1[0]) / 2 - dy / n * offset, (p0[1] + p1[1]) / 2 + dx / n * offset)

# 右列: 高度3行 + 気圧・上昇率・姿勢角を、間を空けずに1つの列として並べる。
# 高度は、補正の入った Alt(KF_C) だけでは元がどうだったのか分からないので、
# 加工していない気圧高度・GNSS高度を真下に並べて見比べられるようにしてある
# (ラベルの幅をそろえてあるので数値も縦にそろう)。
# Yawの行だけ矢印と同じ水色にする
RIGHT_COL_X = 0.723     # 右列の左端(図全体の座標系)。行が画面右端に収まる位置
RIGHT_COL_TOP = 0.815   # 右列の上端
# Roll/Pitch は下のスクロールグラフに移したので、ここには出さない
euler_texts = make_column(RIGHT_COL_X, RIGHT_COL_TOP,
                          ['black'] * 5 + [YAW_COLOR])
# 左列: 空力データ(対気速度・横滑り角・偏流角・風)。
# flight_preprocess.py の推定結果なので、実測値と区別できるよう見出しを付ける。
# Betaの行だけ矢印と同じ黄色にする
air_texts = make_column(0.015, 0.788, ['black', 'black', BETA_COLOR, 'black', 'black', 'black'])
# 風の見出しは風ベクトルの横に付けるため、位置が毎フレーム変わる → update内で作る
# システムログ(電圧・GPS精度・衛星数)の表示。重要度低めなので画面左下に小さめに1行表示
# fig.textは図全体の座標基準なのでax.clear()の影響を受けない
sys_text = fig.text(0.02, 0.015, "", horizontalalignment='left', verticalalignment='bottom', color='black', fontsize=13, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=4, foreground='white')])
# 高度の基準の断り書き。読み解くときだけ必要な情報なので、画面下にごく小さく置く。
# 内容は時間で変わらないので、ここで一度だけ書き込む
fig.text(0.02, 0.040, f"Alt datum:  GNSS = B.S.L. {BSL_OFFSET_M:.2f}m (T.P.)    "
                      f"Baro = {_p_ref:.2f}hPa",
         horizontalalignment='left', verticalalignment='bottom', color='black',
         fontsize=11, path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])

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
    setup_gs_axis()
    rp_ax.clear()
    setup_rp_axis()

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
    baro_alt = interp_baro_alt(i)
    gnss_alt = interp_gnss_alt(i)
    roll = interp_roll(i)
    pitch = interp_pitch(i)
    yaw = np.mod(np.degrees(np.arctan2(interp_sin_yaw(i), interp_cos_yaw(i))), 360)
    # 推定値(β・偏流角・風・TAS)。飛行区間外は推定できないのでフラグで伏せる
    betada_valid = BETADA_T0 <= i <= BETADA_T1
    # β・偏流角は、ベクトル図と数字が一致する幾何学的な値(beta_air/da_air)を使う。
    # 運動方程式による滑らかな beta/da は表示には使わない(interp_beta/interp_da は
    # 古い形式のファイル用のフォールバックとしてだけ残してある)
    beta_air = interp_beta_air(i)
    da_air = interp_da_air(i)
    wind_spd = interp_windspd(i)
    wind_dir = np.mod(np.degrees(np.arctan2(interp_sin_wd(i), interp_cos_wd(i))), 360)
    tas = interp_tas(i)
    # ヨー誤差を補正した真方位。矢印もこちらを使う方が物理的に正しい
    yaw_true = np.mod(np.degrees(np.arctan2(interp_sin_yawtrue(i), interp_cos_yawtrue(i))), 360)
    if not betada_valid:
        yaw_true = yaw
    volt = interp_volt(i)
    cputemp = interp_cputemp(i)
    hacc = interp_hacc(i)
    vacc = interp_vacc(i)
    sacc = interp_sacc(i)
    sats = interp_sats(i)
    current_time = seconds_to_time(i, starttime)

    # ベクトル図(風の三角形)。矢印はすべて始点から終点へ伸ばし、
    # 長さは VEC_SCALE で速度[m/s]に比例させる(矢印同士の長さを比較できる)
    vec_patches = []
    vec_texts = []
    OX, OY = VEC_ORIGIN

    # 3本はほぼ同じ向きに並ぶので、描く順番＝重なりの順番になる。
    # 太い順に下から重ね、細い機首方位(中抜き)を一番上にすると全部見える

    if betada_valid:
        # 対気移動ベクトルの先端と、そこから風ベクトルを伸ばした先端。
        # 対地移動ベクトルは「原点 → 風ベクトルの先端」そのものとして描くので、
        # 三角形は必ず閉じる(閉じるかどうかは beta_air の正しさで決まる)
        air_tip = _tip(OX, OY, yaw_true + beta_air, tas)
        wind_tip = _tip(air_tip[0], air_tip[1], wind_dir + 180.0, wind_spd)
        tt_bearing, tt_mps = _polar(OX, OY, *wind_tip)
    else:
        # 推定がない区間はGPSの実測値だけで対地移動ベクトルを描く
        tt_bearing, tt_mps = truetrack, gs

    # 1) 対地移動ベクトル(TrueTrack・G/S)。原点から伸ばす
    a, _ = draw_vector(OX, OY, tt_bearing, tt_mps, facecolor=TRACK_COLOR,
                       edgecolor='white', linewidth=2.5, width=VEC_W_TT)
    vec_patches.append(a)

    if betada_valid:
        # 2) 対気移動ベクトル。Yawからβだけ回した向きに、長さTASで原点から伸ばす
        a, _ = draw_vector(OX, OY, yaw_true + beta_air, tas, facecolor=BETA_COLOR,
                           edgecolor='black', linewidth=1.5, width=VEC_W_AIR)
        vec_patches.append(a)
        # 3) 風ベクトル。対気移動ベクトルの先端から、風が吹いていく向きに伸ばす
        if wind_spd > 0.05:
            a, _ = draw_vector(air_tip[0], air_tip[1], wind_dir + 180.0, wind_spd,
                               facecolor='mediumblue', edgecolor='white',
                               linewidth=1.5, width=VEC_W_WIND)
            vec_patches.append(a)
            vec_texts.append(tt_ax.text(*_label_pos(air_tip, wind_tip, 0.10), "WIND",
                                        color='mediumblue', fontsize=13, fontweight='bold',
                                        ha='center', va='center', clip_on=False,
                                        path_effects=[path_effects.withStroke(linewidth=3, foreground='white')]))

    # 4) 機首方位(Yaw)。矢印ではなく1本の点線で向きだけを示す。
    #    長さに意味はないので、βの開きが見えるようTAS矢印より少し長くしてある
    yaw_len = (tas if betada_valid else VEC_NOEST_LEN_MPS) * VEC_YAW_LEN_FACTOR
    yaw_end = _tip(OX, OY, yaw_true, yaw_len)
    tt_ax.plot([OX, yaw_end[0]], [OY, yaw_end[1]], color=YAW_COLOR,
               linewidth=3, linestyle=(0, (6, 4)), clip_on=False)

    # 機体(機首が原点)。点線と同じ機首方位で回るので、機首の向きが一目で分かる
    draw_plane(OX, OY, yaw_true)

    # 長さの目安が分かるようにスケールバーを左下に置く
    tt_ax.plot([-0.98, -0.98 + VEC_REF_MPS * VEC_SCALE], [-0.95, -0.95],
               color='black', linewidth=3, solid_capstyle='butt', clip_on=False)
    vec_texts.append(tt_ax.text(-0.98 + VEC_REF_MPS * VEC_SCALE / 2, -0.91, f"{VEC_REF_MPS:.0f}m/s",
                                color='black', fontsize=12, fontweight='bold',
                                ha='center', va='bottom', clip_on=False,
                                path_effects=[path_effects.withStroke(linewidth=3, foreground='white')]))

    # Ground speed のスクロールグラフ。
    # 直近 GS_WINDOW_SEC 秒ぶんを毎フレーム描き直すことで、
    # 波形が左へ流れて画面外へ消えていくように見える。
    # 推定TASを同じ時間軸にオレンジで重ねると、
    # 対地速度との差がそのまま風の影響として読み取れる
    gs_win_t = np.arange(i - GS_WINDOW_SEC, i + GS_PLOT_DT * 0.5, GS_PLOT_DT)
    gs_win_t = gs_win_t[gs_win_t >= GS_T0]  # データ開始前は外挿になるので描かない
    if len(gs_win_t) > 0:
        gs_rel_t = gs_win_t - i  # 右端(現在)を0とした相対時刻 [秒]
        if betada_valid:
            # TASの推定が有効な区間だけ重ねる
            tas_mask = (gs_win_t >= BETADA_T0) & (gs_win_t <= BETADA_T1)
            if tas_mask.any():
                gs_ax.plot(gs_rel_t[tas_mask], interp_tas(gs_win_t[tas_mask]),
                           color='darkorange', linewidth=2.5, linestyle='--')
        gs_ax.plot(gs_rel_t, interp_gs(gs_win_t), color='blue', linewidth=3)
        # 現在値の位置に丸印を打ち、波形の右端＝今であることを分かりやすくする。
        # 表示範囲外(離陸前など)のときは枠の外に点が浮くので描かない
        if GS_YMIN <= gs <= GS_YMAX:
            gs_ax.plot([0], [gs], marker='o', color='blue', markersize=9,
                       markeredgecolor='white', markeredgewidth=2, clip_on=False)
    gs_text.set_text(f"G/S {gs:.1f}m/s")
    # オレンジの破線が何なのかは、同じ枠の中のデジタル表示で示す
    tas_text.set_text(f"TAS(Estimate) {tas:.1f}m/s" if betada_valid else "TAS(Estimate) --")

    # Roll/Pitch のスクロールグラフ。G/Sと同じ時間軸で真下に並べているので、
    # 「速度が落ちたときに機体がどう傾いていたか」を縦に見比べられる
    rp_win_t = gs_win_t[gs_win_t >= EULER_T0] if len(gs_win_t) > 0 else gs_win_t
    if len(rp_win_t) > 0:
        rp_rel_t = rp_win_t - i
        rp_ax.plot(rp_rel_t, interp_roll(rp_win_t), color=ROLL_COLOR, linewidth=2.5)
        rp_ax.plot(rp_rel_t, interp_pitch(rp_win_t), color=PITCH_COLOR, linewidth=2.5)
        for _v, _c in ((roll, ROLL_COLOR), (pitch, PITCH_COLOR)):
            if RP_YMIN <= _v <= RP_YMAX:
                rp_ax.plot([0], [_v], marker='o', color=_c, markersize=8,
                           markeredgecolor='white', markeredgewidth=2, clip_on=False)
    roll_text.set_text(f"Roll {roll:+.1f}°")
    pitch_text.set_text(f"Pitch {pitch:+.1f}°")


    # Text updates
    tt_text.set_text(f"TrueTrack: {int(truetrack)}°")
    latlon_text.set_text(f"{latitude:.6f}N {longitude_disp:.6f}E")
    time_text.set_text(f"Time: {current_time}")
    # 右列。高度は 補正済み(KF_C)・気圧のみ(Baro)・GNSSのみ(BSL基準) の3つ。
    # (KF_C)は気圧ドリフトを補正済みであることを明示する。
    # 他の2つは無加工で、それぞれの基準は画面下に小さく断ってある。
    # 続けて気圧・上昇率・姿勢角。roll/pitchは±付き、yawは0-360°表示
    for art, txt in zip(euler_texts, [
            f"Alt (KF_C) {kf_alt:+5.1f}m",
            f"Alt (Baro) {baro_alt:+5.1f}m",
            f"Alt (GNSS) {gnss_alt:+5.1f}m",
            f"{pressure:.2f}hPa", f"KF V/S {kf_vspeed:+.2f}m/s",
            f"Yaw {yaw_true:.0f}°"]):
        art.set_text(txt)
    # 左列: 空力データ。すべて flight_preprocess.py による推定値なので
    # ESTIMATED の見出しを付けて実測値と区別する。飛行区間外は "--" にする
    # Beta・Drift は矢印として描いている角度と同じ値(beta_air/da_air)を出す。
    # 滑らかな beta の方を出すと、図の開き具合と数字が食い違ってしまう
    if betada_valid:
        air_lines = ["- ESTIMATED -", f"TAS {tas:.1f}m/s", f"Beta {beta_air:+.1f}°",
                     f"Drift {da_air:+.1f}°", f"Wind {wind_spd:.1f}m/s", f"  from {wind_dir:.0f}°"]
    else:
        air_lines = ["- ESTIMATED -", "TAS --", "Beta --", "Drift --", "Wind --", "  from --"]
    for art, txt in zip(air_texts, air_lines):
        art.set_text(txt)
    # システムログの電圧・GPS精度・衛星数(直前のログ値を保持)
    # CPU温度は電圧と同じログ行(volt=...)から。古い書式のログには無いので、その場合は "--"
    _cpu = f"{cputemp:.1f}C" if np.isfinite(cputemp) else "--"
    sys_text.set_text(f"Batt {volt:.2f}V  CPU {_cpu}  "
                      f"hAcc {hacc:.1f}m vAcc {vacc:.1f}m sAcc {sacc:.2f}m/s  sats {sats:.0f}")

    # グラフ枠内のデジタル表示は各Axesに属するので、clear() のあとに戻す必要がある。
    # 他の文字は fig.text なので ax.clear() の影響を受けない
    gs_ax.add_artist(gs_text)
    gs_ax.add_artist(tas_text)
    rp_ax.add_artist(roll_text)
    rp_ax.add_artist(pitch_text)

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
    times_up_to_i = trail_time[trail_time <= i]
    map_ax.plot(interp_longitude(times_up_to_i), interp_latitude(times_up_to_i), color='white', linewidth=2)
    map_ax.plot(longitude, latitude, marker='o', color='black', markersize=10, markeredgecolor='white', markeredgewidth=4)

    return ([tt_text, latlon_text, gs_text, tas_text, roll_text, pitch_text, time_text, sys_text]
            + euler_texts + air_texts + vec_patches + vec_texts + tt_ax.lines
            + gs_ax.lines + gs_ax.patches + rp_ax.lines + map_ax.lines + map_ax.patches)

ani = FuncAnimation(fig, update, frames=new_time, interval=frame_interval*1000)
ani.save(PART_SAVENAME, writer='ffmpeg', fps=1/frame_interval)
print(f"part{PARTNUM} 完了: {PART_SAVENAME} ({timertime.time() - _t_render_start:.1f}秒)")