#!/usr/bin/env python3
"""
生 IMU ログ（ESKF 姿勢）と 飛行 CSV（緯度経度・対地速度）を結合する後解析ツール。

■ 目的
「実際に加速・減速した地点でピッチがどう変わったか」
「カーブでバンクがどう変わったか」を 1 つの表で見られるようにする。
BNO085 内蔵フュージョンの値も併記するので、両者の差 = 過小評価分が直接読める。

■ 時刻の橋渡し
  生 IMU ログ : ホスト時刻 t（time_us_32。起動ごとに 0 に戻る）
  飛行 CSV    : JST の壁時計 (HH:MM:SS.cc)
  この 2 つは GNSS レコードが持つ iTOW（GPS 週内時刻）で繋ぐ。
      UTC 秒 = iTOW/1000 - 閏秒(18)
      JST    = UTC + 9h
  セッションごとに「ホスト時刻 → JST」の一次式を最小二乗で当てはめる
  （どちらも安定した時計なので、受信ジッタを均せば直線で十分）。

使い方:
    python3 merge_flight.py 20260818.bin --csvdir . -o merged.csv
    python3 merge_flight.py 20260818.bin --csvdir . -o merged.csv --hz 10
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

from decode_imulog import load, split, build_euler
from eskf import run_on_log

GPS_LEAP_SECONDS = 18       # GPS 時系は UTC より 18 秒進んでいる（2017 年以降）
JST_OFFSET = 9 * 3600


def itow_to_jst_sec(itow_sec):
    """iTOW [s] → その日の JST 秒（0..86400）。"""
    return ((itow_sec - GPS_LEAP_SECONDS) % 86400 + JST_OFFSET) % 86400


def host_to_jst_fit(gnss_sess):
    """1 セッション分の GNSS レコードから「ホスト時刻 → JST 秒」の一次式を作る。

    戻り値: (変換関数, 残差RMS[s], 使用点数)。点が足りなければ (None, nan, 0)。
    """
    if len(gnss_sess) < 2:
        return None, float("nan"), 0
    t = gnss_sess["t"].to_numpy(float)
    jst = itow_to_jst_sec(gnss_sess["ts"].to_numpy(float))
    # 日付をまたぐと不連続になるので、その場合は連続化してから当てはめる
    jst = np.unwrap(jst / 86400.0 * 2 * np.pi) / (2 * np.pi) * 86400.0
    a, b = np.polyfit(t, jst, 1)
    resid = jst - (a * t + b)
    return (lambda x: a * np.asarray(x, float) + b), float(np.sqrt((resid ** 2).mean())), len(t)


def load_flight_csvs(csvdir):
    """飛行 CSV をすべて読み、JST 秒の列を付けて 1 つにまとめる。"""
    rows = []
    for fn in sorted(glob.glob(os.path.join(csvdir, "*.csv"))):
        base = os.path.basename(fn)
        try:
            d = pd.read_csv(fn)
        except Exception:
            continue
        if "latitude" not in d.columns or "time" not in d.columns:
            continue        # 飛行 CSV ではない（merged.csv 等を読み飛ばす）
        tt = d["time"].astype(str).str.strip()
        parts = tt.str.split(":", expand=True)
        if parts.shape[1] < 3:
            continue
        d["jst"] = (parts[0].astype(float) * 3600 +
                    parts[1].astype(float) * 60 +
                    parts[2].astype(float))
        d["csv_file"] = base
        rows.append(d)
    if not rows:
        raise SystemExit("飛行 CSV が見つからない（--csvdir を確認）")
    return pd.concat(rows, ignore_index=True).sort_values("jst").reset_index(drop=True)


def motion_from_gnss(gnss_sess, sacc_max=1.0, smooth_s=1.5, min_speed=2.0):
    """GNSS 速度から 前後加速度・旋回率・横加速度 を作る（加速/カーブ地点の目印）。

    2Hz の速度を素で微分するとノイズが乗るので、移動平均で均してから差分する。

    min_speed 未満では旋回率・横加速度を NaN にする。
    低速だと速度ベクトルの向き（コース角）がノイズで暴れ、
    実測で横加速度 176 m/s² のような無意味な値が出るため
    （速度が小さいほど同じ速度誤差が大きな角度誤差になる）。
    """
    g = gnss_sess[gnss_sess["sAcc"] <= sacc_max].sort_values("t")
    if len(g) < 3:
        return None
    t = g["t"].to_numpy(float)
    vn, ve = g["velN"].to_numpy(float), g["velE"].to_numpy(float)
    speed = np.hypot(vn, ve)
    course = np.degrees(np.arctan2(ve, vn)) % 360.0

    dt_med = np.median(np.diff(t))
    win = max(3, int(round(smooth_s / dt_med)) | 1)      # 奇数窓
    k = np.ones(win) / win
    sm = lambda x: np.convolve(x, k, mode="same")

    sp_s = sm(speed)
    # 前後加速度 = 速さの時間微分
    a_long = np.gradient(sp_s, t)
    # 旋回率 = コースの時間微分（±180 をまたぐので unwrap してから）
    course_u = np.degrees(np.unwrap(np.radians(course)))
    turn = np.gradient(sm(course_u), t)
    a_lat = sp_s * np.radians(turn)                       # 横加速度 = v * ψ̇

    # 低速区間ではコース角が信用できないので旋回系の指標を無効化する
    slow = sp_s < min_speed
    turn = np.where(slow, np.nan, turn)
    a_lat = np.where(slow, np.nan, a_lat)

    return pd.DataFrame({"t": t, "gnss_speed": speed,
                         "gnss_course": course,
                         "accel_long": a_long,
                         "turn_rate": turn,
                         "accel_lat": a_lat})


def main():
    ap = argparse.ArgumentParser(description="ESKF 姿勢と飛行 CSV を結合する")
    ap.add_argument("path", help="生 IMU ログ (.bin)")
    ap.add_argument("--csvdir", default=".", help="飛行 CSV のあるディレクトリ")
    ap.add_argument("-o", "--out", default="merged.csv")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="出力レート [Hz]（既定 10。生は約 50Hz）")
    ap.add_argument("--sacc-max", type=float, default=1.0,
                    help="運動量の算出に使う GNSS の sAcc 上限 [m/s]")
    args = ap.parse_args()

    parts = split(load(args.path))
    est = run_on_log(parts)
    bno = build_euler(parts)
    gnss = parts.get("gnssvel")
    if gnss is None or gnss.empty:
        raise SystemExit("GNSS レコードが無いので時刻を対応付けられない")

    csv = load_flight_csvs(args.csvdir)
    out = []

    print(f"{'sess':>5}{'点数':>8}{'残差[ms]':>10}{'JST 範囲':>26}{'CSV 一致':>10}")
    for sess, e in est.groupby("session"):
        gs = gnss[gnss["session"] == sess]
        to_jst, rms, npts = host_to_jst_fit(gs)
        if to_jst is None:
            print(f"{sess:>5}{0:>8}{'-':>10}{'GNSS 不足':>26}{'-':>10}")
            continue

        e = e.copy()
        e["jst"] = to_jst(e["t"].to_numpy()) % 86400.0

        # 出力レートまで間引く
        step = max(1, int(round((len(e) / (e["ts"].max() - e["ts"].min())) / args.hz)))
        e = e.iloc[::step].reset_index(drop=True)

        # BNO085 の姿勢（同じセッション内で結合）
        if bno is not None:
            b = bno[bno["session"] == sess]
            if not b.empty:
                e = pd.merge_asof(
                    e.sort_values("t"),
                    b[["t", "roll", "pitch", "yaw"]]
                     .rename(columns={"roll": "bno_roll", "pitch": "bno_pitch",
                                      "yaw": "bno_yaw"}).sort_values("t"),
                    on="t", direction="nearest")

        # GNSS 由来の運動量
        mot = motion_from_gnss(gs, args.sacc_max)
        if mot is not None:
            e = pd.merge_asof(e.sort_values("t"), mot.sort_values("t"),
                              on="t", direction="nearest", tolerance=1.0)

        # 飛行 CSV（緯度経度など）を JST で結合
        e = pd.merge_asof(e.sort_values("jst"),
                          csv[["jst", "latitude", "longitude", "gs", "TrueTrack",
                               "GNSS_Altitude", "KF_Altitude", "KF_Vspeed",
                               "pressure", "csv_file"]].sort_values("jst"),
                          on="jst", direction="nearest", tolerance=1.0)

        if "bno_roll" in e:
            e["d_roll"] = e["roll"] - e["bno_roll"]
            e["d_pitch"] = e["pitch"] - e["bno_pitch"]

        matched = int(e["latitude"].notna().sum())
        def hhmm(x):
            return f"{int(x//3600):02d}:{int(x%3600//60):02d}:{x%60:05.2f}"
        print(f"{sess:>5}{npts:>8}{rms*1000:>10.1f}"
              f"{hhmm(e['jst'].min())+' - '+hhmm(e['jst'].max()):>26}"
              f"{matched}/{len(e):>10}")
        out.append(e)

    if not out:
        raise SystemExit("結合できるセッションが無かった")
    m = pd.concat(out, ignore_index=True)
    m["jst_time"] = m["jst"].apply(
        lambda x: f"{int(x//3600):02d}:{int(x%3600//60):02d}:{x%60:05.2f}")

    cols = ["session", "jst_time", "jst", "t",
            "latitude", "longitude", "gs", "TrueTrack",
            "gnss_speed", "accel_long", "accel_lat", "turn_rate",
            "roll", "pitch", "yaw", "yaw_sigma", "bno_roll", "bno_pitch", "bno_yaw",
            "d_roll", "d_pitch",
            "GNSS_Altitude", "KF_Altitude", "KF_Vspeed", "pressure", "csv_file"]
    cols = [c for c in cols if c in m.columns]
    m[cols].to_csv(args.out, index=False)
    print(f"\nwrote {args.out}  ({len(m)} 行, {args.hz:g}Hz)")
    return m[cols]


if __name__ == "__main__":
    main()
