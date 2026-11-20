#!/usr/bin/env python3
# ============================================================
# File    : tools/convert_old_contest_csv.py
# Project : PONS v7 / v7.1
# Role    : 2024・2025 年大会のログを、いまの飛行 CSV と同じ列の並びに直して Flight Viewer で開けるようにする
#           （2026 年は tools/fuse_attitude_csv.py）
# Author  : Claude (2026-11-20)
# Updated : 2026-11-20
# ============================================================
#
# ■ 2024（2024-07-27_0936_edited.csv。PONS の位置・G/S・航跡だけ、1 秒ごと）
#   ★ gs は**ノット**で記録されている（位置から出した速さ 6.9 m/s に対して gs の中央値 12.75）。m/s に直す。
#   姿勢・高度は無い。ble_plotter の Garmin ログ（pilotdata2024.csv。高度は 81.0 で一定）と対気速度
#   （alba2024pilotlog_airspeed_added.csv）は時刻が「記録開始からの秒数」しか無く、速度の波形で合わせても
#   相関 0.86・位置で合わせた結果と約 10 秒食い違って時刻を決めきれないので入れない。
#
# ■ 2025（PONSv5_planelog_2025-07-26_1539.csv ＋ 20250726_log_all_joined.csv）
#   行は joined（1 秒ごと・15:42:16〜16:28:31）を土台にし、その前後は planelog の行で補う。
#   latitude/longitude/gs/TrueTrack ← planelog（PONS の元の記録。1〜2 秒ごと）を時刻で補間（3 秒より空けば空欄）
#   GNSS_Altitude ← joined の g_altitude（Garmin の高度。KF_Altitude は空欄なので Viewer はこれを高度に使う）
#   roll   ← joined の roll − 取り付けのずれ（直進中 = G/S>4 かつ航跡の変化 <0.5°/s の中央値。約 8.3°）
#            向きは 2026 と同じ（旋回率との相関 2025 +0.84 / 2026 +0.74）
#   pitch  ← −joined の pitch。★ 機首下げが正で記録されている（対気速度との相関 +0.97。docs/pons_tas.md §2.3 と同じ判断）。
#            0 点（機体の取り付け・SET PITCH）は分からないので、ずれは引かない
#   yaw    ← 空欄。joined の yaw は回る向きが航跡と逆で（変化率の相関 −0.69）、ずれも時間で動くので真方位として使えない
#   tas_mps ← joined の Airspeed（対気速度センサーの実測。PONS の推定 TAS ではない）
#   風・電圧・衛星数は無いので空欄
#
# 使い方:
#   python3 tools/convert_old_contest_csv.py            # 2024 と 2025 の両方を olddata に書き出す
#   python3 tools/convert_old_contest_csv.py 2025

import csv
import math
import os
import sys

import numpy as np
import pandas as pd

OLDDATA = os.path.expanduser('~/Documents/Arduino/GPS_TFT_map/tools/olddata')
KNOT = 0.514444
POS_MAX_GAP_SEC = 3.0

OUT_HEADER = ['latitude', 'longitude', 'gs', 'TrueTrack', 'GNSS_Altitude', 'KF_Altitude', 'KF_Vspeed', 'pressure',
              'voltage', 'numsat', 'date', 'time', 'lux', 'backlight',
              'roll', 'pitch', 'yaw', 'pitch_avg', 'roll_trim', 'yaw_acc95', 'wind_mps', 'wind_dir', 'tas_mps', 'hdr']


def tsec(s):
    h, m, sec = str(s).strip().split(':')
    return int(h) * 3600 + int(m) * 60 + float(sec)


def hms(t):
    t = t % 86400
    h = int(t // 3600); m = int((t % 3600) // 60); s = t - h * 3600 - m * 60
    return f'{h:02d}:{m:02d}:{s:05.2f}'


def f(v, nd):
    return '' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{nd}f}'


def interp_gap(ts, vs, t, max_gap, angle=False):
    """時刻 t の値。範囲外か、前後の点が max_gap より離れていれば None。angle=True は角度（度）として補間"""
    i = int(np.searchsorted(ts, t))
    if i < len(ts) and abs(ts[i] - t) < 1e-6:
        return float(vs[i])
    if i == 0 or i >= len(ts) or ts[i] - ts[i - 1] > max_gap:
        return None
    a, b = float(vs[i - 1]), float(vs[i])
    if angle:
        d = (b - a + 540.0) % 360.0 - 180.0
        b = a + d
    v = a + (t - ts[i - 1]) / (ts[i] - ts[i - 1]) * (b - a)
    return v % 360.0 if angle else v


def write_rows(path, rows):
    with open(path, 'w', newline='', encoding='utf-8') as fo:
        w = csv.writer(fo, lineterminator='\n')
        w.writerow(OUT_HEADER)
        for r in rows:
            w.writerow([r.get(k, '') for k in OUT_HEADER])
    print(f'書き出し: {path}（{len(rows)} 行）')


def convert_2024():
    src = os.path.join(OLDDATA, '2024-07-27_0936_edited.csv')
    p = pd.read_csv(src)
    rows = []
    for _, r in p.iterrows():
        rows.append({
            'latitude': f'{r.latitude:.6f}', 'longitude': f'{r.longitude:.6f}',
            'gs': f(float(r.gs) * KNOT, 2),            # ノット → m/s
            'TrueTrack': f'{r.TrueTrack:.0f}', 'date': r.date, 'time': r.time,
        })
    write_rows(os.path.join(OLDDATA, '2024-07-27_0936_fused.csv'), rows)
    print('  gs をノットから m/s に直した（中央値 %.2f m/s）。姿勢・高度・TAS は無し' % (p.gs[p.gs > 1].median() * KNOT))


def convert_2025():
    q = pd.read_csv(os.path.join(OLDDATA, 'PONSv5_planelog_2025-07-26_1539.csv'))
    j = pd.read_csv(os.path.join(OLDDATA, '20250726_log_all_joined.csv'))
    q['t'] = q.time.map(tsec)
    j['t'] = j.JST.map(tsec)
    q = q.sort_values('t').reset_index(drop=True)
    # joined には同じ時刻の行が 1 か所ある（15:43:34 が 2 行）。先の行を残す（同じ時刻だと微分が 0 割りになる）
    j = j.sort_values('t').drop_duplicates('t', keep='first').reset_index(drop=True)
    date = str(q.date.iloc[0])
    qt = q.t.to_numpy()

    # 取り付けのロールのずれ: 直進中（G/S>4・航跡の変化 <0.5°/s、15 秒平均）の中央値
    jt = j.t.to_numpy(dtype=float)
    track = np.array([interp_gap(qt, q.TrueTrack.to_numpy(), t, POS_MAX_GAP_SEC, angle=True) or np.nan for t in jt])
    gsj = np.array([interp_gap(qt, q.gs.to_numpy(), t, POS_MAX_GAP_SEC) or np.nan for t in jt])
    rate = np.degrees(np.gradient(np.unwrap(np.radians(pd.Series(track).ffill().bfill().to_numpy())), jt))
    rate_s = pd.Series(rate).rolling(15, center=True, min_periods=5).mean().to_numpy()
    straight = (gsj > 4) & (np.abs(rate_s) < 0.5)
    roll_ofs = float(np.nanmedian(j.roll.to_numpy()[straight]))

    rows = []
    t_first, t_last = jt[0], jt[-1]
    # joined の前: planelog の位置だけ
    for _, r in q[q.t < t_first].iterrows():
        rows.append({'latitude': f'{r.latitude:.6f}', 'longitude': f'{r.longitude:.6f}', 'gs': f(float(r.gs), 2),
                     'TrueTrack': f'{r.TrueTrack:.0f}', 'date': date, 'time': hms(r.t)})
    n_att = 0
    for k, r in j.iterrows():
        t = jt[k]
        lat = interp_gap(qt, q.latitude.to_numpy(), t, POS_MAX_GAP_SEC)
        lon = interp_gap(qt, q.longitude.to_numpy(), t, POS_MAX_GAP_SEC)
        if lat is None or lon is None:
            continue
        roll = None if pd.isna(r.roll) else float(r.roll) - roll_ofs
        pitch = None if pd.isna(r.pitch) else -float(r.pitch)
        n_att += roll is not None
        rows.append({
            'latitude': f'{lat:.6f}', 'longitude': f'{lon:.6f}',
            'gs': f(gsj[k], 2), 'TrueTrack': f(track[k], 0),
            'GNSS_Altitude': '' if pd.isna(r.g_altitude) else f(float(r.g_altitude), 1),
            'date': date, 'time': hms(t),
            'roll': f(roll, 2), 'pitch': f(pitch, 2),
            'tas_mps': '' if pd.isna(r.Airspeed) else f(float(r.Airspeed), 2),
        })
    # joined の後: planelog の位置だけ
    for _, r in q[q.t > t_last].iterrows():
        rows.append({'latitude': f'{r.latitude:.6f}', 'longitude': f'{r.longitude:.6f}', 'gs': f(float(r.gs), 2),
                     'TrueTrack': f'{r.TrueTrack:.0f}', 'date': date, 'time': hms(r.t)})
    write_rows(os.path.join(OLDDATA, '2025-07-26_1539_fused.csv'), rows)
    print(f'  姿勢あり {n_att} 行（{hms(t_first)}〜{hms(t_last)}）。ロールの取り付けのずれ {roll_ofs:+.2f}° を引いた'
          f'（直進 {int(straight.sum())} 点の中央値）。ピッチは符号を反転（0 点は未補正。飛行中の中央値 '
          f'{-j.pitch[gsj > 4].median():+.2f}°）')


if __name__ == '__main__':
    which = sys.argv[1:] or ['2024', '2025']
    for y in which:
        {'2024': convert_2024, '2025': convert_2025}[y]()
