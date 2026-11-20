#!/usr/bin/env python3
# ============================================================
# File    : tools/fuse_attitude_csv.py
# Project : PONS v7 / v7.1
# Role    : 姿勢を別ファイルに持っていた古い飛行ログ（2026-07-26 大会など）を、
#           いまの飛行 CSV と同じ列の並びにして姿勢・TAS・風を融合する
# Author  : Claude (2026-11-20)
# Updated : 2026-11-20
# ============================================================
#
# 入力（どれも time 列 = JST の HH:MM:SS.ss で突き合わせる）
#   --csv    PONS 本体の飛行 CSV（古い列の並び: latitude,longitude,gs,TrueTrack,Altitude,KF_Altitude,,pressure,date,time,KF_Vspeed）
#   --euler  flight_preprocess.py のステージ 1 の出力（time,roll,pitch,yaw。BNO085 の姿勢を補正したもの）
#   --betada flight_preprocess.py のステージ 2 の出力（time,...,wind_speed,wind_dir,tas,yaw_true,...。飛行区間だけ）
#   --log    SD の log.txt（起動からの秒数:メッセージ）。"GPS TIME: ... UTC" の行で JST に合わせ、
#            "volt=4.08V"（60 秒ごと）と "ACC ... sats=11"（約 10 秒ごと）を拾う（createmovie_csv2mp4.py と同じ読み方）
#
# 出力: いまの飛行 CSV（README の「飛行 CSV フォーマット」）と同じ 24 列。Flight Viewer でそのまま開ける。
#   roll, pitch      ← euler（線形補間）
#   yaw              ← betada の yaw_true（機体磁気の誤差を推定して直した真方位。角度で補間）。
#                      euler の yaw は地磁気のままで機体磁気のずれ（十数度）を含むので使わない。飛行区間の外は空欄
#   tas_mps          ← betada の tas
#   wind_mps         ← betada の wind_speed
#   wind_dir         ← betada の wind_dir + 180°。★ betada は「吹いてくる」真方位、いまの CSV（attitude_get_wind の
#                      dir_to_deg）と Flight Viewer の矢印は「吹いていく」真方位。そのまま入れると風ベクトルが逆を向き、
#                      風の三角形（対地 = 対気 + 風）が閉じない（2026-11-20 に実際に間違えた）
#   voltage          ← log の volt=（60 秒ごとの点を線形補間。前後の点が 90 秒より離れていれば空欄）
#   numsat           ← log の ACC 行の sats=（直前の値。15 秒より古ければ空欄）
#   GNSS_Altitude    ← 元の Altitude
#   元に無い列（lux, backlight, pitch_avg, roll_trim, yaw_acc95, hdr）は空欄（値を作らない）
#   補間の両側のサンプルが MAX_GAP_SEC より離れているか、範囲の外なら空欄
#
# 使い方:
#   python3 tools/fuse_attitude_csv.py                       # 既定: 2026-07-26 大会のファイル
#   python3 tools/fuse_attitude_csv.py --csv A.csv --euler B.txt --betada C.txt --log log.txt --out A_fused.csv

import argparse
import bisect
import csv
import math
import os
import re
import sys

OLDDATA = os.path.expanduser('~/Documents/Arduino/GPS_TFT_map/tools/olddata')
MAX_GAP_SEC = 1.0
VOLT_MAX_GAP_SEC = 90.0    # volt= は 60 秒ごと
SATS_MAX_AGE_SEC = 15.0    # ACC 行は約 10 秒ごと

OUT_HEADER = ['latitude', 'longitude', 'gs', 'TrueTrack', 'GNSS_Altitude', 'KF_Altitude', 'KF_Vspeed', 'pressure',
              'voltage', 'numsat', 'date', 'time', 'lux', 'backlight',
              'roll', 'pitch', 'yaw', 'pitch_avg', 'roll_trim', 'yaw_acc95', 'wind_mps', 'wind_dir', 'tas_mps', 'hdr']


def tsec(s):
    """'08:21:14.50' → 日内秒。読めなければ None"""
    try:
        h, m, sec = s.strip().split(':')
        return int(h) * 3600 + int(m) * 60 + float(sec)
    except (ValueError, AttributeError):
        return None


def load_series(path, cols):
    """time 列と指定列を読む。戻り値 {'t': [...], col: [...]}（時刻順・数値にできない行は捨てる）"""
    out = {'t': []}
    for c in cols:
        out[c] = []
    with open(path, newline='', encoding='utf-8') as f:
        rd = csv.DictReader(f)
        missing = [c for c in ['time'] + cols if c not in rd.fieldnames]
        if missing:
            sys.exit(f'{path} に列 {missing} がありません')
        rows = []
        for r in rd:
            t = tsec(r['time'])
            try:
                vals = [float(r[c]) for c in cols]
            except (ValueError, TypeError):
                continue
            if t is None or not all(math.isfinite(v) for v in vals):
                continue
            rows.append((t, vals))
    rows.sort(key=lambda x: x[0])
    for t, vals in rows:
        out['t'].append(t)
        for c, v in zip(cols, vals):
            out[c].append(v)
    return out


def unwrap_deg(a):
    out, off = [], 0.0
    for i, v in enumerate(a):
        if i > 0:
            d = v - a[i - 1]
            if d > 180:
                off -= 360
            elif d < -180:
                off += 360
        out.append(v + off)
    return out


def interp(series_t, series_v, t):
    """t の値を線形補間。範囲外・間が MAX_GAP_SEC より空いていれば None"""
    i = bisect.bisect_left(series_t, t)
    if i < len(series_t) and abs(series_t[i] - t) < 1e-6:
        return series_v[i]
    if i == 0 or i >= len(series_t):
        return None
    t0, t1 = series_t[i - 1], series_t[i]
    if t1 - t0 > MAX_GAP_SEC:
        return None
    f = (t - t0) / (t1 - t0)
    return series_v[i - 1] + f * (series_v[i] - series_v[i - 1])


def load_log(path):
    """log.txt から (JST 日内秒, 電圧) と (JST 日内秒, 衛星数) の列を作る。
    ★ 起動直後（10 秒未満）の SD INIT で読み直す（前の起動のログが頭に残っていることがある）。飛行中の SD の掴み直しでは読み直さない。"""
    volt, sats = [], []
    anchor = None   # (起動からの秒数, JST 日内秒)
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.match(r'(\d+):SD INIT', line)
            if m and int(m.group(1)) < 10:
                volt.clear(); sats.clear(); anchor = None
                continue
            m = re.match(r'(\d+):GPS TIME: \d+-\d+-\d+ (\d+):(\d+):(\d+) UTC', line)
            if m:
                utc = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4))
                anchor = (int(m.group(1)), (utc + 9 * 3600) % 86400)
                continue
            m = re.match(r'(\d+):volt=([\d.]+)V', line)
            if m:
                volt.append((int(m.group(1)), float(m.group(2))))
                continue
            m = re.match(r'(\d+):ACC .*sats=(\d+)', line)
            if m:
                sats.append((int(m.group(1)), int(m.group(2))))
    if anchor is None:
        sys.exit(f'{path} に GPS TIME の行がありません（JST に合わせられない）')
    to_jst = lambda sec: anchor[1] + (sec - anchor[0])
    vt = [to_jst(a) for a, _ in volt]
    st = [to_jst(a) for a, _ in sats]
    return vt, [v for _, v in volt], st, [n for _, n in sats]


def interp_gap(ts, vs, t, max_gap):
    i = bisect.bisect_left(ts, t)
    if i < len(ts) and abs(ts[i] - t) < 1e-6:
        return vs[i]
    if i == 0 or i >= len(ts) or ts[i] - ts[i - 1] > max_gap:
        return None
    f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return vs[i - 1] + f * (vs[i] - vs[i - 1])


def last_within(ts, vs, t, max_age):
    i = bisect.bisect_right(ts, t + 1e-6)
    if i == 0 or t - ts[i - 1] > max_age:
        return None
    return vs[i - 1]


def fmt(v, nd):
    return '' if v is None else f'{v:.{nd}f}'


def main():
    ap = argparse.ArgumentParser(description='古い飛行 CSV に姿勢・TAS・風を融合して、いまの CSV の列の並びにする')
    ap.add_argument('--csv', default=os.path.join(OLDDATA, '2026-07-26_0804.csv'))
    ap.add_argument('--euler', default=os.path.join(OLDDATA, '20260726_compensated.txt'))
    ap.add_argument('--betada', default=os.path.join(OLDDATA, 'beta_da_20260726.txt'))
    ap.add_argument('--log', default=os.path.join(OLDDATA, 'log_edit.txt'), help='SD の log.txt（"" で使わない）')
    ap.add_argument('--out', default=None, help='既定: <csv>_fused.csv（元のファイルは書き換えない）')
    args = ap.parse_args()
    out_path = args.out or os.path.splitext(args.csv)[0] + '_fused.csv'
    if os.path.abspath(out_path) == os.path.abspath(args.csv):
        sys.exit('出力が入力と同じファイルです')

    eu = load_series(args.euler, ['roll', 'pitch'])
    bd = load_series(args.betada, ['yaw_true', 'tas', 'wind_speed', 'wind_dir'])
    bd_yaw = unwrap_deg(bd['yaw_true'])
    # 吹いてくる → 吹いていく（いまの CSV の向き）
    bd_wdir = unwrap_deg([(d + 180.0) % 360.0 for d in bd['wind_dir']])
    vt, vv, st, sv = load_log(args.log) if args.log else ([], [], [], [])

    with open(args.csv, newline='', encoding='utf-8') as f:
        rd = csv.reader(f)
        header = [h.strip() for h in next(rd)]
        idx = {h.lower(): i for i, h in enumerate(header) if h}
        need = ['latitude', 'longitude', 'time']
        if any(n not in idx for n in need):
            sys.exit(f'{args.csv} に {need} の列がありません（ヘッダー: {header}）')

        def col(row, *names):
            for n in names:
                i = idx.get(n.lower())
                if i is not None and i < len(row):
                    return row[i].strip()
            return ''

        n_rows = n_att = n_air = n_volt = n_sats = 0
        with open(out_path, 'w', newline='', encoding='utf-8') as fo:
            w = csv.writer(fo, lineterminator='\n')
            w.writerow(OUT_HEADER)
            for row in rd:
                if not row or all(not c.strip() for c in row):
                    continue
                t = tsec(col(row, 'time'))
                roll = pitch = yaw = tas = wspd = wdir = volt = sats = None
                if t is not None:
                    roll = interp(eu['t'], eu['roll'], t)
                    pitch = interp(eu['t'], eu['pitch'], t)
                    yaw = interp(bd['t'], bd_yaw, t)
                    tas = interp(bd['t'], bd['tas'], t)
                    wspd = interp(bd['t'], bd['wind_speed'], t)
                    wdir = interp(bd['t'], bd_wdir, t)
                    volt = interp_gap(vt, vv, t, VOLT_MAX_GAP_SEC)
                    sats = last_within(st, sv, t, SATS_MAX_AGE_SEC)
                if yaw is not None:
                    yaw %= 360.0
                if wdir is not None:
                    wdir %= 360.0
                n_rows += 1
                n_att += roll is not None
                n_air += tas is not None
                n_volt += volt is not None
                n_sats += sats is not None
                w.writerow([
                    col(row, 'latitude'), col(row, 'longitude'), col(row, 'gs'), col(row, 'TrueTrack'),
                    col(row, 'GNSS_Altitude', 'Altitude'), col(row, 'KF_Altitude'), col(row, 'KF_Vspeed'),
                    col(row, 'pressure'),
                    col(row, 'voltage') or fmt(volt, 2),
                    col(row, 'numsat') or ('' if sats is None else str(sats)),
                    col(row, 'date'), col(row, 'time'), col(row, 'lux'), col(row, 'backlight'),
                    fmt(roll, 2), fmt(pitch, 2), fmt(yaw, 2), '', '', '',
                    fmt(wspd, 2), fmt(wdir, 1), fmt(tas, 2), '',
                ])

    print(f'書き出し: {out_path}')
    print(f'  行 {n_rows} / 姿勢（roll・pitch）あり {n_att} / 方位・TAS・風あり {n_air}（飛行区間だけ）')
    print(f'  電圧あり {n_volt}（log の volt= {len(vt)} 点）/ 衛星数あり {n_sats}（ACC 行 {len(st)} 点）')
    print(f'  姿勢 {os.path.basename(args.euler)}: {len(eu["t"])} 点  '
          f'β・風 {os.path.basename(args.betada)}: {len(bd["t"])} 点')


if __name__ == '__main__':
    main()
