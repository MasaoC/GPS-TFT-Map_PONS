#!/usr/bin/env python3
"""
PONS 生 IMU ログ (imuraw/YYYYMMDD.bin) のデコーダ。

姿勢 ESKF を PC 上で開発するための入力を作る。
バイナリ形式の定義は imulog.h のコメントが正（このファイルはそれに従う）。

使い方:
    python3 decode_imulog.py imuraw/20260817.bin              # 概要を表示
    python3 decode_imulog.py imuraw/20260817.bin --csv out/    # 種別ごとに CSV 出力

出力される DataFrame の時刻:
    t   : ホスト時刻 [s]。32bit ラップをアンラップ済み。全種別で共通の時間軸。
    ts  : サンプル時刻 [s]。t と同じ時計の上にある。
          v7.1（SCH16T）はレコードの s_us（ドライバが読んだ時刻）をそのまま使う。
          v7 のログは s_us が壊れているので、t から等間隔サンプリングを仮定して復元する。
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

ID_BOOT = 0x00                 # 起動マーカー。ファイルを開くたびに先頭へ 1 件入る

# id -> (名前, 意味のある v の列名)
KINDS = {
    ID_BOOT: ("boot", ["boot_id", "nopen"]),         # 起動識別（s_us は BUILDDATE）
    0x01: ("gyro",    ["gx", "gy", "gz"]),           # [rad/s]
    0x02: ("accel",   ["ax", "ay", "az"]),           # [m/s^2] 重力込みの生比力
    # ★ 0x03 は 0.984 で**書き出しをやめた**（地磁気は推定にも表示にも使わない）。
    #   退避済みの過去ログ（〜2026-10-03）には入っているので定義は残す。
    #   **この番号を別の用途に再利用してはいけない**（古いログが誤デコードされる）。
    0x03: ("mag",     ["mx", "my", "mz"]),           # [uT] 0.984〜欠番
    # ★ 0x04〜0x06 は v7 のログにだけ入っている（チップ内融合の出力）。v7.1 は書き出さない。
    #   読む側のツールはもう無いが、0x03 と同じ理由で定義は残す（番号を再利用しない）。
    0x04: ("gamerv",  ["qw", "qx", "qy", "qz"]),     # v7 のみ
    0x05: ("rv",      ["qw", "qx", "qy", "qz"]),     # v7 のみ
    0x06: ("linacc",  ["lax", "lay", "laz"]),        # v7 のみ
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


def detect_sessions(t_us, ids=None, v0=None):
    """再起動の境目を検出してセッション番号を返す。

    ログファイルは O_APPEND で開くため、同じ日に複数回起動すると 1 つの .bin に
    複数セッションが連結される。これを跨いで処理すると時刻が単調でなくなり、
    ESKF の GNSS 観測が適用されなくなる（2026-08-18 に実際に踏んだ）。

    境界は次の 2 つの **和** を取る。どちらか一方では足りない。

      (a) BOOT レコード（id=0x00）の boot_id が変わった点
      (b) t_us が後戻りした点（32bit ラップを除く）

    (b) だけでは不足する理由: 測位前のレコードは nofixNNN.bin へ行くので、
      測位できた瞬間に日付ファイルへ戻ると t_us も seq も「前へ飛ぶ」。
      後戻りしないので検出できず、起動 2 回が 1 セッションに結合される。
      2026-08-18 のログではこれで 50681 件の継ぎ目を「取りこぼし」と誤読した。

    (a) だけでは不足する理由: ファームを更新した日は 1 つの .bin に旧形式
      （マーカー無し）と新形式が連結される。旧側の再起動はマーカーでは見えない。

    新形式の再起動では (a) と (b) が同じ位置を指すので二重には区切られない
    （マーカーは新しい起動の 1 件目なので、その手前で t_us が後戻りする）。
    """
    t = np.asarray(t_us, dtype=np.int64)
    if t.size < 2:
        return np.zeros(t.size, dtype=int)

    # ---- (b) t_us の後戻り ----
    d = np.diff(t)
    backward = d < 0
    is_wrap = backward & (t[:-1] > int(0.9 * WRAP))
    start = np.concatenate([[False], backward & ~is_wrap])

    # ---- (a) BOOT マーカーの boot_id 変化 ----
    if ids is not None and v0 is not None:
        boot = np.asarray(ids) == ID_BOOT
        cur = None
        for i in np.flatnonzero(boot):
            bid = int(round(float(v0[i])))
            # boot_id=0 は本物のマーカーではない。ファームは 0 を必ず避けて
            # 振るので（imulog.cpp の imulog_write_boot_marker）、0 が出たら
            # ゼロ埋めされた領域を読んでいる。電源断で書きかけになった箇所や、
            # 未書き込みセクタがそう見える。区切ると偽のセッションができるため無視する。
            if bid == 0:
                continue
            # 同じ起動で nofix → 日付 と開き直すと、1 ファイル内に同じ
            # boot_id のマーカーが 2 つ並ぶことがある。境界はあくまで
            # 「boot_id が変わった点」なので、同じ値では区切らない。
            if cur is None or bid != cur:
                # 最初のマーカーでも、その手前にレコードがあれば区切る。
                # 旧形式のデータに新形式を追記した場合、手前は別の起動なので
                # 必ず分ける必要がある（t_us が前へ飛んでいると (b) では見えない）。
                if cur is not None or i > 0:
                    start[i] = True
                cur = bid

    sess = np.cumsum(start)
    return (sess - sess.min()).astype(int)


def reconstruct_time(t, max_gap=0.5):
    """ホスト時刻の列から、センサーの等間隔サンプリング時刻を復元する（**v7 のログ専用**）。

    なぜ必要か:
      v7 のログの s_us は使えない（チップ側の計算がアンダーフローして 2^32 近傍に
      張り付いていた）。一方ホスト時刻 t は Core0 が処理した時刻なので、
      地図描画で 64ms 止まるとその間のサンプルがバーストで届き dt≈0 が並ぶ。
      そのまま積分すると、まとめて届いた回転がごっそり抜け落ちる。
      ★ v7.1 では使わない（split() の _uses_host_read_time）。SCH16T はこちらから
        読みに行く方式なのでバーストが起きず、s_us が本当の読み出し時刻になっている。
        等間隔に均すと、描画で読み出しが空いた区間の dt がかえって狂う。

    復元方法:
      v7 のチップはレポートを一定周期でスケジュールするので、i 番目のサンプルは
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

    # ゼロ埋めレコードを落とす。
    # 電源断で書きかけになった箇所や未書き込みセクタは 28 バイトの 0 として読める。
    # 本物のレコードは id が 0x01〜0x20 か、BOOT なら boot_id（v0）が必ず非 0
    # （ファームが 0 を避けて振る）。したがって id=0 かつ v0=0 は必ず壊れたデータ。
    # 落とさないと t_us=0 が後戻りとみなされ、偽のセッション境界ができる。
    junk = (df["id"] == ID_BOOT) & (df["v0"] == 0.0)
    if junk.any():
        print(f"warn: ゼロ埋めレコード {int(junk.sum())} 件を捨てた"
              f"（書きかけ・未書き込み領域）", file=sys.stderr)
        df = df[~junk].reset_index(drop=True)
        if df.empty:
            return df

    # セッション（起動）ごとに分けてからアンラップする。
    # 全体を通してアンラップすると再起動の後戻りが残り、時刻が単調でなくなる。
    df["session"] = detect_sessions(df["t_us"].to_numpy(),
                                    df["id"].to_numpy(),
                                    df["v0"].to_numpy())
    t = np.zeros(len(df), dtype=np.int64)
    tv = df["t_us"].to_numpy()
    for _, idx in df.groupby("session").indices.items():
        t[idx] = unwrap_u32(tv[idx])
    df["t"] = t * 1e-6
    return df


def _uses_host_read_time(sub):
    """s_us が「ドライバが読んだ時刻」（v7.1 / SCH16T）か。

    v7.1 のドライバは読み出した瞬間の time_us_32() を s_us に入れ、imulog_push() が
    その直後に t_us を入れるので、両者の差は数十 µs しかない。v7 のログの s_us は
    2^32 近傍に張り付いた壊れた値なので、差は桁違いに大きい。
    """
    d = (sub["t_us"].to_numpy(np.int64) - sub["s_us"].to_numpy(np.int64)) % WRAP
    return bool(np.median(d) < 5000)      # 5ms 未満なら同じ時計


def split(df):
    """種別ごとの DataFrame に分ける。センサー時刻は種別ごとにアンラップする。"""
    out = {}
    for rid, (name, cols) in KINDS.items():
        sub = df[df["id"] == rid].copy()
        if sub.empty:
            continue
        sub = sub.reset_index(drop=True)

        # cols は v[0..3] への割り当てなので触らない。追加列は extra へ
        extra = []
        if rid == ID_BOOT:
            # BOOT はセンサー時刻を持たない。s_us の枠は BUILDDATE に使っている
            # （20260919 は float32 では表せないので uint32 側へ逃がしてある）。
            sub["ts"] = np.nan
            extra = ["builddate"]
        elif rid == 0x10:
            sub["ts"] = sub["s_us"] * 1e-3        # GNSS は iTOW [ms]
        elif rid == 0x20:
            sub["ts"] = np.nan                    # baro はセンサー時刻を持たない
        else:
            # IMU レコード。セッションごとに時刻の出どころを決める。
            #   v7.1: s_us = 読み出し時刻。t からの差（数十 µs）を引くだけ
            #   v7  : s_us が壊れているので、等間隔サンプリングを前提に t から復元する
            tsv = np.zeros(len(sub), dtype=float)
            tv = sub["t"].to_numpy()
            for _, idx in sub.groupby("session").indices.items():
                ss = sub.iloc[idx]
                if _uses_host_read_time(ss):
                    lag = ((ss["t_us"].to_numpy(np.int64) - ss["s_us"].to_numpy(np.int64))
                           % WRAP) * 1e-6
                    tsv[idx] = tv[idx] - lag
                else:
                    tsv[idx] = reconstruct_time(tv[idx])
            sub["ts"] = tsv

        for i, c in enumerate(cols):
            sub[c] = sub[f"v{i}"]
        # v の割り当てが終わってから追加列を作る（先に作ると v2 などで踏み潰される）
        if rid == ID_BOOT:
            sub["boot_id"] = sub["boot_id"].round().astype(np.int64)
            sub["nopen"] = sub["nopen"].round().astype(np.int64)
            sub["builddate"] = sub["s_us"].astype(np.int64)
        keep = ["t", "ts", "session", "seq", "acc"] + cols + extra
        out[name] = sub[keep]
    return out


def euler_from_quat(qw, qx, qy, qz):
    """クォータニオン → センサー座標系の ZYX オイラー角 [rad]。

    eskf.py の s_roll / s_pitch / s_yaw（フィルタの素の出力）に使う。機体軸の姿勢は
    mount_correct_quat() を通すこと。
    """
    qw, qx, qy, qz = (np.asarray(v, dtype=float) for v in (qw, qx, qy, qz))
    sensor_roll = np.arctan2(2.0 * (qw * qx + qy * qz),
                             1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)   # ジンバルロック対策
    sensor_pitch = np.arcsin(sinp)
    sensor_yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz))
    return sensor_roll, sensor_pitch, sensor_yaw


MOUNT_Q = (0.70710678, 0.0, -0.70710678, 0.0)   # w,x,y,z（センサー Y 軸まわり -90 度）


def mount_correct_quat(qw, qx, qy, qz):
    """センサー座標のクォータニオン → 機体軸の roll/pitch/yaw [deg]。

    ★ 機上の attitude.h `imu_body_euler_rad()` と**同じ変換であること。**
      片方だけ直すと、実機の表示と PC の解析が静かに食い違う。

    2026-09-21 に v7 実機で測定した配置（IC が基板裏面・1番ピンが天）:
        機体の「下」= センサー +X / 「前」= センサー +Z / 「右」= センサー -Y
    この配置では**機体が水平のときセンサーがジンバルロック**に入るため、
    以前のように Euler を組み替える方式では原理的に正しくならない。
    クォータニオンの段階で回してから 1 回だけ Euler を出すこと。
    """
    qw, qx, qy, qz = (np.asarray(v, dtype=float) for v in (qw, qx, qy, qz))
    mw, mx, my, mz = MOUNT_Q
    bw = qw*mw - qx*mx - qy*my - qz*mz          # q_body = q_sensor ⊗ q_mount
    bx = qw*mx + qx*mw + qy*mz - qz*my
    by = qw*my - qx*mz + qy*mw + qz*mx
    bz = qw*mz + qx*my - qy*mx + qz*mw
    roll = np.arctan2(2.0*(bw*bx + by*bz), 1.0 - 2.0*(bx*bx + by*by))
    # 符号反転は機上 attitude.h の imu_body_euler_rad() と同じ理由（機首上げが正）
    pitch = -np.arcsin(np.clip(2.0*(bw*by - bz*bx), -1.0, 1.0))
    yaw = np.arctan2(2.0*(bw*bz + bx*by), 1.0 - 2.0*(by*by + bz*bz))
    # ワールドが ENU（X=East）なので ZYX 抽出のヨーは東基準。方位（北=0）へは
    # 符号反転だけでなく 90 度足す（機上 attitude.h の imu_yaw_to_heading_deg と同じ）。
    return np.degrees(roll), np.degrees(pitch), (90.0 - np.degrees(yaw)) % 360.0



def report(df, parts):
    """概要を表示する。

    ※ 記録時間も seq の欠落も、必ずセッション単位で集計すること。
      ログは O_APPEND なので再起動をまたぐと t も seq も 0 付近へ戻り、
      通しで計算すると記録時間が負になったり、境界が丸ごと
      「取りこぼし」に見えたりする（実際に両方やらかした）。
    """
    n_sess = df["session"].nunique()
    has_marker = bool((df["id"] == ID_BOOT).any())
    dur_total = 0.0
    print(f"レコード数 : {len(df)}   セッション数 : {n_sess}"
          f"   （分割: {'BOOT マーカー + t_us 後戻り' if has_marker else 't_us 後戻りのみ（旧形式）'}）")
    print()
    print(f"{'sess':>5}{'boot_id':>9}{'件数':>10}{'長さ[s]':>11}{'seq欠落':>10}{'時刻検算':>10}")
    lost_total = 0
    warn = []
    for sess, g in df.groupby("session"):
        dur = g["t"].iloc[-1] - g["t"].iloc[0]
        dur_total += dur

        # seq の欠落。BOOT は連番に参加しないので必ず除外する
        # （BOOT の seq は 0xFFFF 固定なので、混ぜると欠落が捏造される）。
        real = g[g["id"] != ID_BOOT]
        seq = real["seq"].to_numpy(dtype=np.int64)
        step = np.diff(seq)
        step = np.where(step < 0, step + 65536, step)   # uint16 のラップを戻す
        lost = int(np.sum(step[step > 1] - 1))
        lost_total += lost

        b = g[g["id"] == ID_BOOT]
        bid = f"{int(round(float(b['v0'].iloc[0]))):06X}" if len(b) else "-"

        # 時刻の検算: t_us の経過と GNSS(iTOW) の経過が一致するか。
        # ずれていたら、そのセッションは起動 2 回が結合されている疑いが濃い。
        gn = g[g["id"] == 0x10]
        chk = "-"
        if len(gn) >= 2:
            d_us = gn["t"].iloc[-1] - gn["t"].iloc[0]
            d_gps = (gn["s_us"].iloc[-1] - gn["s_us"].iloc[0]) / 1000.0
            if abs(d_gps - d_us) > 5.0:
                chk = "NG"
                warn.append((sess, d_us, d_gps))
            else:
                chk = "OK"
        print(f"{sess:>5}{bid:>9}{len(g):>10}{dur:>11.1f}{lost:>10}{chk:>10}")

    print()
    print(f"{'種別':<10}{'件数':>9}{'実測Hz':>10}")
    for name, sub in parts.items():
        hz = len(sub) / dur_total if dur_total > 0 else float("nan")
        print(f"{name:<10}{len(sub):>9}{hz:>10.1f}")

    print()
    if lost_total:
        pct = 100.0 * lost_total / (len(df) + lost_total)
        print(f"seq の欠落合計 : {lost_total} 件 ({pct:.2f}%)")
        print("  ※ これは推測値。欠落 = 取りこぼしとは限らない。")
        print("     ファイルが nofixNNN.bin から日付ファイルへ切り替わった箇所でも seq は飛ぶ。")
        print("     同じ boot_id を持つ別ファイルが無いか確かめること。")
        print("     実際に捨てられた件数は機上の log.txt（60 秒ごとの dropped=）が正。")
    else:
        print("seq の欠落 : なし")

    for sess, d_us, d_gps in warn:
        print()
        print(f"警告: セッション {sess} は t_us の経過 {d_us:.1f} 秒に対し "
              f"GNSS の経過が {d_gps:.1f} 秒（差 {d_gps - d_us:+.1f} 秒）。")
        print("  マイコンの µs カウンタが遅れることはないので、これは連続した 1 回の")
        print("  記録ではない。起動 2 回が 1 セッションに結合されている可能性が高い。")
        if not has_marker:
            print("  このファイルは BOOT マーカーを持たない旧形式なので、分割は推測に頼っている。")


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
