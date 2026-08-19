#!/usr/bin/env python3
"""
PONS v6 生 IMU ログ (imuraw/YYYYMMDD.bin) のデコーダ。

姿勢 ESKF を PC 上で開発するための入力を作る。
バイナリ形式の定義は imulog.h のコメントが正（このファイルはそれに従う）。

使い方:
    python3 decode_imulog.py imuraw/20260817.bin              # 概要を表示
    python3 decode_imulog.py imuraw/20260817.bin --csv out/    # 種別ごとに CSV 出力

出力される DataFrame の時刻:
    t   : ホスト時刻 [s]。32bit ラップをアンラップ済み。全種別で共通の時間軸。
    ts  : センサー時刻 [s]（BNO085 の sv.timestamp 由来）。同じくアンラップ済み。
          GNSS レコードでは iTOW [s]（ラップ処理はしない）。
"""

import argparse
import os
import struct
import sys

import numpy as np
import pandas as pd

REC_FMT = "<IIBBH4f"          # imulog.h の ImuLogRec と一致させること
REC_SIZE = struct.calcsize(REC_FMT)
assert REC_SIZE == 28, REC_SIZE

WRAP = 1 << 32                 # t_us / s_us は 32bit で約 71.6 分ごとにラップする

# id -> (名前, 意味のある v の列名)
KINDS = {
    0x01: ("gyro",    ["gx", "gy", "gz"]),           # [rad/s]
    0x02: ("accel",   ["ax", "ay", "az"]),           # [m/s^2] 重力込みの生比力
    0x03: ("mag",     ["mx", "my", "mz"]),           # [uT]
    0x04: ("gamerv",  ["qw", "qx", "qy", "qz"]),     # BNO085 の推定（比較用）
    0x05: ("rv",      ["qw", "qx", "qy", "qz"]),     # 地磁気補正（比較用）
    0x06: ("linacc",  ["lax", "lay", "laz"]),        # 既存バリオ KF 用（比較用）
    0x10: ("gnssvel", ["velN", "velE", "velD", "sAcc"]),  # [m/s] NED
    0x20: ("baro",    ["press", "alt", "temp"]),     # [hPa][m][degC]
}


def unwrap_u32(x):
    """32bit でラップするカウンタを単調増加へ戻す（1 セッション内で使うこと）。"""
    x = np.asarray(x, dtype=np.int64)
    if x.size == 0:
        return x
    # 前の値より大きく減っていたらラップとみなす
    jumps = np.diff(x) < -(WRAP // 2)
    return x + np.concatenate([[0], np.cumsum(jumps)]) * WRAP


def detect_sessions(t_us):
    """再起動の境目を検出してセッション番号を返す。

    ログファイルは O_APPEND で開くため、同じ日に複数回起動すると 1 つの .bin に
    複数セッションが連結される。再起動すると time_us_32() が 0 付近へ戻るので
    t_us が後戻りする。これを跨いで処理すると時刻が単調でなくなり、
    ESKF の GNSS 観測が適用されなくなる（2026-08-18 に実際に踏んだ）。

    32bit ラップとの区別: ラップなら直前の値が u32 の上限付近にあるはず。
    そうでない後戻りは再起動とみなす。
    """
    t = np.asarray(t_us, dtype=np.int64)
    if t.size < 2:
        return np.zeros(t.size, dtype=int)
    d = np.diff(t)
    backward = d < 0
    is_wrap = backward & (t[:-1] > int(0.9 * WRAP))
    reboot = backward & ~is_wrap
    return np.concatenate([[0], np.cumsum(reboot)]).astype(int)


def reconstruct_time(t, max_gap=0.5):
    """ホスト時刻の列から、センサーの等間隔サンプリング時刻を復元する。

    なぜ必要か:
      レコードの s_us（BNO085 の sv.timestamp）は使えない。sh2 が
      「ホスト時刻 - 遅延」を計算する際にアンダーフローしており、
      実測では値が 2^32 近傍（符号付きなら小さな負値）に張り付いていた。
      一方ホスト時刻 t は Core0 が処理した時刻なので、地図描画で 64ms 止まると
      その間のサンプルがバーストで届き dt≈0 が並ぶ（実測で 5〜7%）。
      そのまま積分すると、まとめて届いた回転がごっそり抜け落ちる。

    復元方法:
      BNO085 はレポートを一定周期でスケジュールするので、i 番目のサンプルは
      t0 + i*T にある。取りこぼしが無ければ T = (t[-1]-t[0])/(n-1) で正確に求まる
      （実測: gyro 19.97ms ≒ 設定 50Hz、accel 15.95ms ≒ 実効 62.7Hz）。
      max_gap を超える中断があれば、そこで区切って区間ごとに復元する。
    """
    t = np.asarray(t, dtype=float)
    if t.size < 2:
        return t.copy()
    out = np.empty_like(t)
    breaks = np.where(np.diff(t) > max_gap)[0] + 1
    for seg in np.split(np.arange(t.size), breaks):
        if seg.size < 2:
            out[seg] = t[seg]
            continue
        t0, t1 = t[seg[0]], t[seg[-1]]
        out[seg] = t0 + np.arange(seg.size) * ((t1 - t0) / (seg.size - 1))
    return out


def load(path):
    raw = open(path, "rb").read()
    n_full, tail = divmod(len(raw), REC_SIZE)
    if tail:
        # 電源断などで最後のレコードが途中で切れている場合。切り捨てて続行する。
        print(f"warn: 末尾 {tail} バイトが不完全なため切り捨てた", file=sys.stderr)
    recs = struct.iter_unpack(REC_FMT, raw[: n_full * REC_SIZE])
    df = pd.DataFrame(recs, columns=["t_us", "s_us", "id", "acc", "seq",
                                     "v0", "v1", "v2", "v3"])
    if df.empty:
        return df

    # セッション（起動）ごとに分けてからアンラップする。
    # 全体を通してアンラップすると再起動の後戻りが残り、時刻が単調でなくなる。
    df["session"] = detect_sessions(df["t_us"].to_numpy())
    t = np.zeros(len(df), dtype=np.int64)
    tv = df["t_us"].to_numpy()
    for _, idx in df.groupby("session").indices.items():
        t[idx] = unwrap_u32(tv[idx])
    df["t"] = t * 1e-6
    return df


def split(df):
    """種別ごとの DataFrame に分ける。センサー時刻は種別ごとにアンラップする。"""
    out = {}
    for rid, (name, cols) in KINDS.items():
        sub = df[df["id"] == rid].copy()
        if sub.empty:
            continue
        sub = sub.reset_index(drop=True)

        if rid == 0x10:
            sub["ts"] = sub["s_us"] * 1e-3        # GNSS は iTOW [ms]
        elif rid == 0x20:
            sub["ts"] = np.nan                    # baro はセンサー時刻を持たない
        else:
            # ※ s_us（sv.timestamp）は sh2 のアンダーフローで壊れているため使わない。
            #   等間隔サンプリングを前提にホスト時刻から復元する。
            tsv = np.zeros(len(sub), dtype=float)
            tv = sub["t"].to_numpy()
            for _, idx in sub.groupby("session").indices.items():
                tsv[idx] = reconstruct_time(tv[idx])
            sub["ts"] = tsv

        for i, c in enumerate(cols):
            sub[c] = sub[f"v{i}"]
        keep = ["t", "ts", "session", "seq", "acc"] + cols
        out[name] = sub[keep]
    return out


def euler_from_quat(qw, qx, qy, qz):
    """クォータニオン → センサー座標系の ZYX オイラー角 [rad]。

    imu.cpp の get_imu_euler() と同じ式を使うこと（値が食い違うと比較にならない）。
    """
    qw, qx, qy, qz = (np.asarray(v, dtype=float) for v in (qw, qx, qy, qz))
    sensor_roll = np.arctan2(2.0 * (qw * qx + qy * qz),
                             1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)   # ジンバルロック対策
    sensor_pitch = np.arcsin(sinp)
    sensor_yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz))
    return sensor_roll, sensor_pitch, sensor_yaw


def mount_correct(sensor_roll, sensor_pitch, sensor_yaw):
    """センサー座標系のオイラー角 → 機体軸の roll/pitch/yaw [deg]。

    BNO085 は IC 直立・コンポーネント面後ろ向きに取り付けられており、
    センサー軸と機体軸が入れ替わっている（imu.cpp の get_imu_euler() と同一）:
        roll  = sensor_pitch
        pitch = sensor_roll - 90deg
        yaw   = -sensor_yaw を 0..360 に正規化
    ※ ESKF の出力にもこの変換を通すこと。定義がここ 1 箇所になるよう関数化してある。
    """
    roll = np.degrees(sensor_pitch)
    pitch = np.degrees(sensor_roll - np.pi / 2.0)
    yaw = np.degrees(-sensor_yaw) % 360.0
    return roll, pitch, yaw


def build_euler(parts):
    """機体軸の roll/pitch/yaw [deg] を作る（機上の表示値と一致させる）。

    ---- マウント補正 ----
    BNO085 は IC 直立・コンポーネント面後ろ向きに取り付けられており、
    センサー軸と機体軸が入れ替わっている（imu.cpp の get_imu_euler() 参照）:
        roll  = sensor_pitch
        pitch = sensor_roll - 90deg
        yaw   = -sensor_yaw を 0..360 に正規化
    roll/pitch は GAME_ROTATION_VECTOR（磁気なし）、
    yaw は ROTATION_VECTOR（地磁気補正）から取る。RV が無ければ GRV で代替する。

    ※ ESKF を書くときもこの変換を通すこと。
      生クォータニオンのままでは機体軸の姿勢にならない。
    """
    grv = parts.get("gamerv")
    if grv is None or grv.empty:
        return None

    s_roll, s_pitch, _ = euler_from_quat(grv.qw, grv.qx, grv.qy, grv.qz)
    roll, pitch, _ = mount_correct(s_roll, s_pitch, 0.0)
    out = pd.DataFrame({"session": grv["session"].to_numpy(),
                        "t": grv["t"].to_numpy(), "roll": roll, "pitch": pitch})

    rv = parts.get("rv")
    if rv is not None and not rv.empty:
        _, _, r_yaw = euler_from_quat(rv.qw, rv.qx, rv.qy, rv.qz)
        _, _, yaw_deg = mount_correct(0.0, 0.0, r_yaw)
        yaw_src = pd.DataFrame({"session": rv["session"].to_numpy(),
                                "t": rv["t"].to_numpy(), "yaw": yaw_deg})
        # RV は 5Hz と低レートなので GRV の各時刻へ直近値を割り当てる。
        # セッションごとに t が 0 に戻るため、必ずセッション単位で結合すること。
        merged = []
        for sess, o in out.groupby("session"):
            y = yaw_src[yaw_src["session"] == sess]
            if y.empty:
                o = o.copy(); o["yaw"] = np.nan
            else:
                o = pd.merge_asof(o.sort_values("t"),
                                  y[["t", "yaw"]].sort_values("t"),
                                  on="t", direction="nearest")
            merged.append(o)
        out = pd.concat(merged, ignore_index=True)
    else:
        _, _, g_yaw = euler_from_quat(grv.qw, grv.qx, grv.qy, grv.qz)
        out["yaw"] = mount_correct(0.0, 0.0, g_yaw)[2]
    return out


def report(df, parts):
    """概要を表示する。

    ※ 記録時間も seq の欠落も、必ずセッション単位で集計すること。
      ログは O_APPEND なので再起動をまたぐと t も seq も 0 付近へ戻り、
      通しで計算すると記録時間が負になったり、境界が丸ごと
      「取りこぼし」に見えたりする（実際に両方やらかした）。
    """
    n_sess = df["session"].nunique()
    dur_total = 0.0
    print(f"レコード数 : {len(df)}   セッション数 : {n_sess}")
    print()
    print(f"{'sess':>5}{'件数':>10}{'長さ[s]':>11}{'取りこぼし':>12}")
    lost_total = 0
    for sess, g in df.groupby("session"):
        dur = g["t"].iloc[-1] - g["t"].iloc[0]
        dur_total += dur
        seq = g["seq"].to_numpy(dtype=np.int64)
        step = np.diff(seq)
        step = np.where(step < 0, step + 65536, step)   # uint16 のラップを戻す
        lost = int(np.sum(step[step > 1] - 1))
        lost_total += lost
        print(f"{sess:>5}{len(g):>10}{dur:>11.1f}{lost:>12}")

    print()
    print(f"{'種別':<10}{'件数':>9}{'実測Hz':>10}")
    for name, sub in parts.items():
        hz = len(sub) / dur_total if dur_total > 0 else float("nan")
        print(f"{name:<10}{len(sub):>9}{hz:>10.1f}")

    print()
    if lost_total:
        pct = 100.0 * lost_total / (len(df) + lost_total)
        print(f"取りこぼし合計 : {lost_total} 件 ({pct:.2f}%) "
              f"— SD 書き出しが間に合わなかった箇所がある")
    else:
        print("取りこぼし合計 : なし")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--csv", metavar="DIR", help="種別ごとに CSV を書き出すディレクトリ")
    args = ap.parse_args()

    df = load(args.path)
    if df.empty:
        print("レコードがありません", file=sys.stderr)
        return 1

    parts = split(df)
    eul = build_euler(parts)
    if eul is not None:
        parts["euler"] = eul
    report(df, parts)

    if args.csv:
        os.makedirs(args.csv, exist_ok=True)
        base = os.path.splitext(os.path.basename(args.path))[0]
        for name, sub in parts.items():
            dst = os.path.join(args.csv, f"{base}_{name}.csv")
            sub.to_csv(dst, index=False)
            print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
