#!/usr/bin/env python3
# ============================================================
# File    : tools/imulog/compare_grv.py
# Project : PONS v7 (Pilot Oriented Navigation System for HPA)
# Role    : 0.983 の変更（GRV/RV 依存の除去 ＋ 重力観測）を実ログで検証する。
#           走行・飛行のあと imuraw/*.bin をこれに食わせると、
#           「新しい経路が旧経路（BNO085 の融合出力）と比べてどうか」が出る。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/10/01
# ============================================================
#
# 使い方:
#     python3 compare_grv.py /Volumes/PONSV7TEST/imuraw/20261005.bin
#     python3 compare_grv.py <bin> --session 7        # セッションを絞る
#     python3 compare_grv.py <bin> --cutoff 60        # GNSS 断の模擬開始を早める
#
# 検証できるのは次の 6 つ。
#   1) 水平化 vs GRV          … 初期ロール・ピッチが BNO085 の融合出力と合うか
#   2) 地磁気ヨー vs GNSS 航跡 … 地磁気をやめた判断が正しかったか
#   3) バリオ V/S             … 重力除去を GRV から ESKF へ移した影響
#   4) ESKF 姿勢 新 vs 旧      … 初期化方法を変えた影響
#   5) 重力観測（Phase 2）     … GNSS 断を模擬したときのドリフト改善
#   6) ゲートの開口率          … 重力観測がどれだけ入れるか
#
# ★ **GRV/RV が生ログに残っているから比較できる。** 0.983 で推定には使わなく
#   なったが、imu.cpp は imulog_push(IMULOG_ID_GAMERV / _RV) を続けている。
#   **BNO085 を降ろしたら 1) 2) 3) 4) は比較対象を失う。** それまでに測っておくこと。
#
# ★ 姿勢の計算そのものは eskf.py（attitude.cpp の PC 側の双子）に任せている。
#   ここでフィルタを書き直さないこと。三重実装になる。
import argparse
import sys

import numpy as np
import pandas as pd

import decode_imulog as D
import eskf as E

G = 9.80665

# imu.cpp / settings.h と同じ値（★ 片方を直したらもう片方も直すこと）
KF_R, KF_Q_VEL, KF_Q_BIAS, KF_HORIZ_GAIN = 12.0, 0.02, 0.0005, 0.01
PREDICT_DT = 0.030
VSI_SACC_MAX, VSI_R_SCALE = 1.0, 16.0
STATIC_GYRO, STATIC_ACC_TOL, STATIC_INIT_S = 0.026, 0.30, 2.0
YAW_INIT_MIN_SPEED, YAW_INIT_ARM = 3.0, 2
LEVEL_YAWRATE_LP, LEVEL_YAWRATE_INST, LEVEL_ACCMAG_TOL = 1.0, 3.0, 0.5
MAG_DECLINATION_DEG = -8.0      # 0.983 で settings.h から消した。比較のためここだけ持つ


def subset(parts, sess):
    """parts をそのセッションだけに絞る。

    run_on_log() は渡された全セッションを回すので、絞らないと 1 回の比較で
    何十秒も余計にかかる（セッションが 6 本ある実ログで実測 4 倍）。
    """
    return {k: v[v.session == sess] for k, v in parts.items()
            if isinstance(v, pd.DataFrame) and 'session' in v.columns}


def up_row(q):
    """回転行列の第 3 行 = センサー軸で見た「上」。imu.cpp と同じ量。"""
    q = np.atleast_2d(np.asarray(q, float))
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.column_stack([2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)])


def vario_kf(t_acc, ak, horiz, t_baro, z_baro, t_gnss, veld_up, sacc):
    """imu.cpp のバリオ 3 状態 KF。a_k（重力除去後の鉛直加速度）を外から与える。

    ★ 0.981 の修正（同じ測位を 1 回だけ融合）を含む。
    ★ 0.983 の修正（上方向が取れなくても predict を止めない）は、ここでは
      「ak を必ず与える」ことで満たしている。止めると K[1]=0 のまま
      V/S が構造的に 0 で固まる（実機で踏んだ）。
    """
    x = np.zeros(3)
    P = np.diag([100.0, 1.0, 0.1])
    init = False
    last = None
    vs = hs = 0.0
    cnt = 0
    bi = gi = 0
    out = []
    for i in range(len(t_acc)):
        t = t_acc[i]
        while bi < len(t_baro) and t_baro[bi] <= t:
            z = z_baro[bi]
            bi += 1
            if not init:
                x = np.array([z, 0.0, 0.0]); init = True; last = t
                continue
            S = P[0, 0] + KF_R
            K = P[:, 0] / S
            x = x + K * (z - x[0])
            P = P - np.outer(K, P[0, :].copy()); P = (P + P.T) / 2
            np.fill_diagonal(P, np.maximum(np.diag(P), 1e-9))
        while gi < len(t_gnss) and t_gnss[gi] <= t:
            if init and sacc[gi] < VSI_SACC_MAX:
                r = max(sacc[gi] ** 2 * VSI_R_SCALE, 0.001)
                S = P[1, 1] + r
                K = P[:, 1] / S
                x = x + K * (veld_up[gi] - x[1])
                P = P - np.outer(K, P[1, :].copy()); P = (P + P.T) / 2
                np.fill_diagonal(P, np.maximum(np.diag(P), 1e-9))
            gi += 1
        vs += ak[i]; hs += horiz[i]; cnt += 1
        if init and last is not None and (t - last) >= PREDICT_DT:
            dt = t - last; last = t
            a_k, ha = vs / cnt, hs / cnt
            vs = hs = 0.0; cnt = 0
            if 0.0 < dt < 0.5:
                x = np.array([x[0] + x[1]*dt, x[1] + (a_k - x[2])*dt, x[2]])
                F = np.array([[1, dt, 0], [0, 1, -dt], [0, 0, 1]], float)
                P = F @ P @ F.T
                P[1, 1] += KF_Q_VEL + KF_HORIZ_GAIN * ha * ha
                P[2, 2] += KF_Q_BIAS
                out.append((t, x[1], x[0]))
        elif init and last is None:
            last = t
    if not out:
        return np.array([]), np.array([]), np.array([])
    o = np.array(out)
    return o[:, 0], o[:, 1], o[:, 2]


def pair_imu(g, a):
    """ジャイロと加速度を最近傍でペアリングする（eskf.run_on_log と同じ手順）。"""
    imu = pd.merge_asof(g.sort_values('ts'),
                        a[['ts', 'ax', 'ay', 'az']].sort_values('ts'),
                        on='ts', direction='nearest')
    return imu.dropna(subset=['ax', 'ay', 'az']).reset_index(drop=True)


def analyse(parts, sess, cutoff_s):
    P = subset(parts, sess)
    g, a = P['gyro'], P['accel']
    gv = P.get('gamerv')
    rv = P.get('rv')
    ba = P['baro'].sort_values('t') if 'baro' in P and not P['baro'].empty else None
    gn = None
    if 'gnssvel' in P and not P['gnssvel'].empty:
        gn = P['gnssvel'][P['gnssvel'].sAcc <= VSI_SACC_MAX].sort_values('t')
        if gn.empty:
            gn = None

    imu = pair_imu(g, a)
    if len(imu) < 2000:
        return
    # ★ ヨーが入らないセッションでは GNSS 速度観測が一度も適用されない
    #   （機上も yaw_set_ でゲートしている）。5) の「GNSS 断」が
    #   「断でない状態」と同じになり、差が 0 になって誤読する。
    yaw_ok = False
    if gn is not None:
        _sp = np.hypot(gn['velN'].to_numpy(), gn['velE'].to_numpy())
        yaw_ok = int((_sp >= YAW_INIT_MIN_SPEED).sum()) >= YAW_INIT_ARM
    ts = imu['ts'].to_numpy(float)
    th = imu['t'].to_numpy(float)
    w = imu[['gx', 'gy', 'gz']].to_numpy(float)
    acc = imu[['ax', 'ay', 'az']].to_numpy(float)
    an = np.linalg.norm(acc, axis=1)
    wn = np.linalg.norm(w, axis=1)
    st = (wn < STATIC_GYRO) & (np.abs(an - G) < STATIC_ACC_TOL)

    print(f'\n{"="*68}')
    print(f'session {sess}:  {ts[-1]-ts[0]:.0f} 秒 / {len(ts)} サンプル')
    print(f'{"="*68}')

    # ---- 静止区間（水平化の契機）----
    i_lv = run0 = None
    for i in range(len(ts)):
        if st[i]:
            if run0 is None:
                run0 = i
            elif ts[i] - ts[run0] >= STATIC_INIT_S:
                i_lv = i
                break
        else:
            run0 = None
    print(f'静止条件を満たす割合 {100*st.mean():.1f}%  '
          f'|ω| 中央 {np.degrees(np.median(wn)):.2f} deg/s')
    if i_lv is None:
        print('★ 2 秒連続の静止区間が無い → 機上なら「走行中フォールバック」'
              '（先頭の比力で水平化・σ=30deg）になる。1) は評価できない')
    else:
        print(f'水平化できる時刻 t={th[i_lv]:.1f}s')

    # ---- 1) 水平化 vs GRV ----
    if i_lv is not None and gv is not None and not gv.empty:
        q_lv = E.level_quat_from_accel(acc[run0:i_lv+1].mean(axis=0))
        j = (gv['ts'] - ts[i_lv]).abs().idxmin()
        q_grv = [gv.qw[j], gv.qx[j], gv.qy[j], gv.qz[j]]
        f = lambda v: float(np.atleast_1d(v)[0])
        r1, p1, _ = D.mount_correct_quat(*q_lv)
        r2, p2, _ = D.mount_correct_quat(*q_grv)
        print(f'\n1) 水平化 vs GRV（初期ロール・ピッチ）')
        print(f'   加速度水平化  roll {f(r1):+7.2f}  pitch {f(p1):+7.2f}')
        print(f'   GRV           roll {f(r2):+7.2f}  pitch {f(p2):+7.2f}')
        print(f'   ★ 差 roll {f(r1)-f(r2):+.3f} / pitch {f(p1)-f(p2):+.3f} deg'
              f'（0.5 度以内なら GRV は何も足していない）')
        if abs(f(p1)) > 80.0:
            # ピッチが ±90 度に近いとロールとヨーが縮退する（ジンバルロック）。
            # 手持ちで立てたままのセッションで roll 差が十数度に見えるのはこれ。
            print(f'   ※ ピッチが {f(p1):+.0f} 度でロールが縮退している。'
                  f'roll の差は見ても意味が無い')

    # ---- 2) 地磁気ヨー vs GNSS 航跡 ----
    if gn is not None and rv is not None and not rv.empty:
        vn, ve = gn['velN'].to_numpy(), gn['velE'].to_numpy()
        gt = gn['t'].to_numpy()
        sp = np.hypot(vn, ve)
        hit = np.flatnonzero(sp >= YAW_INIT_MIN_SPEED)   # yaw_ok と同じ条件
        print(f'\n2) 地磁気ヨー vs GNSS 航跡（ヨー注入の時点）')
        if len(hit) >= YAW_INIT_ARM:
            k = int(hit[YAW_INIT_ARM-1])
            trk = np.degrees(np.arctan2(ve[k], vn[k])) % 360.0
            jr = (rv['t'] - gt[k]).abs().idxmin()
            _, _, y_rv = D.mount_correct_quat(rv.qw[jr], rv.qx[jr], rv.qy[jr], rv.qz[jr])
            y_true = (float(np.atleast_1d(y_rv)[0]) + MAG_DECLINATION_DEG) % 360.0
            d = (y_true - trk + 180.0) % 360.0 - 180.0
            print(f'   t={gt[k]:.1f}s  GS={sp[k]:.1f} m/s')
            print(f'   GNSS 航跡 {trk:7.2f} deg   '
                  f'RV（偏角 {MAG_DECLINATION_DEG:+.0f} 補正後）{y_true:7.2f} deg')
            print(f'   ★ 差 {d:+.2f} deg（大きいほど地磁気をやめた判断が正しい）')
        else:
            print(f'   GS が {YAW_INIT_MIN_SPEED} m/s を超えないので評価できない')

    # ---- 3) バリオ V/S: GRV 由来 vs ESKF 由来の重力除去 ----
    if gv is not None and not gv.empty and ba is not None and gn is not None:
        att = E.run_on_log(P)
        # run_on_log の行は IMU サンプルと 1:1（水平化より前は無い）。ts で突き合わせる。
        m = imu[['ts']].merge(att[['ts', 'qw', 'qx', 'qy', 'qz']], on='ts', how='inner')
        q_eskf = m[['qw', 'qx', 'qy', 'qz']].to_numpy()
        idx = imu.ts.isin(m.ts).to_numpy()
        gvv = gv.sort_values('ts')
        q_grv = np.column_stack([np.interp(ts[idx], gvv.ts, gvv[c])
                                 for c in ('qw', 'qx', 'qy', 'qz')])
        q_grv /= np.linalg.norm(q_grv, axis=1, keepdims=True)
        gsacc = gn['sAcc'].to_numpy()
        res = {}
        for name, q in (('GRV', q_grv), ('ESKF', q_eskf)):
            u = up_row(q)
            azw = (u * acc[idx]).sum(axis=1)
            res[name] = vario_kf(th[idx], azw - G,
                                 np.sqrt(np.maximum(an[idx]**2 - azw**2, 0.0)),
                                 ba['t'].to_numpy(), ba['alt'].to_numpy(),
                                 gn['t'].to_numpy(), -gn['velD'].to_numpy(), gsacc)
        t1, v1, z1 = res['GRV']; t2, v2, z2 = res['ESKF']
        print(f'\n3) バリオ V/S（重力除去の出どころ GRV vs ESKF）')
        if len(v1) > 100 and len(v2) > 100:
            v2i = np.interp(t1, t2, v2); z2i = np.interp(t1, t2, z2)
            # 収束待ちの 30 秒は除く（バリオ KF の初期 P が大きい）
            k = t1 > t1[0] + 30
            if k.sum() > 100:
                d = v2i[k] - v1[k]
                print(f'   V/S の sd   GRV {v1[k].std():.3f} / ESKF {v2i[k].std():.3f} m/s')
                print(f'   ★ 差  平均 {d.mean():+.4f}  sd {d.std():.4f}  '
                      f'|max| {np.abs(d).max():.3f} m/s')
                print(f'   |差|>0.1 {100*(np.abs(d)>0.1).mean():.2f}%  '
                      f'高度差 |max| {np.abs(z2i[k]-z1[k]).max():.2f} m')
                print(f'   （差が小さければ「GRV をやめてもバリオは変わらない」）')
        else:
            print('   気圧または測位が足りず評価できない')

    # ---- 4) ESKF 姿勢 新 vs 旧 ----
    try:
        A = E.run_on_log(P)
        B = E.run_on_log(P, init_from_grv=True)
        k = A.t.to_numpy() > A.t.min() + 60       # 初期化の収束待ち
        if k.sum() > 100:
            tk = A.t.to_numpy()[k]
            dr = (A.roll.to_numpy()[k] - np.interp(tk, B.t, B.roll) + 180) % 360 - 180
            dp = (A.pitch.to_numpy()[k] - np.interp(tk, B.t, B.pitch) + 180) % 360 - 180
            print(f'\n4) ESKF 姿勢 新（水平化+航跡+重力観測） vs 旧（GRV 初期化）')
            print(f'   roll 差  平均 {dr.mean():+.3f}  sd {dr.std():.3f}  '
                  f'|95%| {np.percentile(np.abs(dr),95):.2f} deg')
            print(f'   pitch 差 平均 {dp.mean():+.3f}  sd {dp.std():.3f}  '
                  f'|95%| {np.percentile(np.abs(dp),95):.2f} deg')
            print(f'   ★ 平均の差は取り付け基準のずれ。sd が大きいときは追従が違う')
        else:
            print(f'\n4) 初期化後 60 秒の比較区間が取れないので評価できない')
    except SystemExit as e:
        print(f'\n4) スキップ: {e}')

    # ---- 5) 重力観測の効果（GNSS 断を模擬）----
    # ★ 打ち切りは「初期化が済んで落ち着いたあと」に置く。先頭から切ると
    #   ヨーが入る前に断になり、飛行中の途絶という状況にならない。
    try:
        t0 = float(E.run_on_log(P).t.min())
        cut = t0 + cutoff_s
        print(f'\n5) 重力観測（Phase 2）— t={cut:.0f}s 以降 GNSS 断の |誤差|')
        if not yaw_ok:
            print(f'   GS が {YAW_INIT_MIN_SPEED} m/s を超えずヨーが入らない'
                  f'→ GNSS 速度観測が一度も効かないので評価できない')
        else:
            # ★ 基準は**その条件自身**の GNSS 常時。重力観測あり／なしを
            #   同じ基準と比べると、断の影響ではなく重力観測そのものの差を
            #   測ってしまう（ヨー未確定のセッションで 0.00 になって気付いた）。
            wrap = lambda d: (d + 180.0) % 360.0 - 180.0
            for lbl, kw in (('なし', dict(use_level=False)), ('あり', dict(use_level=True))):
                R = E.run_on_log(P, **kw)
                C = E.run_on_log(P, gnss_cutoff={sess: cut}, **kw)
                k = R.t.to_numpy() > cut + 10
                if k.sum() < 100:
                    print(f'   断後の区間が短くて評価できない（--cutoff を小さく）')
                    break
                tk = R.t.to_numpy()[k]
                e = np.hypot(wrap(R.roll.to_numpy()[k] - np.interp(tk, C.t, C.roll)),
                             wrap(R.pitch.to_numpy()[k] - np.interp(tk, C.t, C.pitch)))
                print(f'   重力観測{lbl}  中央 {np.median(e):5.2f}  '
                      f'95% {np.percentile(e,95):5.2f}  最大 {e.max():5.2f} deg')
            print(f'   （それぞれ自分の GNSS 常時が基準。中央値が小さいほど'
                  f'錨が効いている）')
    except SystemExit as e:
        print(f'\n5) スキップ: {e}')

    # ---- 6) ゲートの開口率 ----
    # 姿勢を持たない簡易版（比力方向まわりの角速度で近似）。実機の gobs= と併せて見る。
    u = acc / an[:, None]
    wz = np.degrees(np.einsum('ij,ij->i', u, w))
    dt = np.clip(np.diff(ts, prepend=ts[0]), 0.0, 0.1)
    lp = np.zeros(len(wz))
    for i in range(1, len(lp)):
        kk = dt[i] / (2.0 + dt[i])
        lp[i] = lp[i-1] + (wz[i] - lp[i-1]) * kk
    gate = ((np.abs(lp) <= LEVEL_YAWRATE_LP) & (np.abs(wz) <= LEVEL_YAWRATE_INST)
            & (np.abs(an - G) <= LEVEL_ACCMAG_TOL))
    print(f'\n6) 重力観測のゲート開口率  {100*gate.mean():.1f}%'
          f'  → 2Hz なら {120*gate.mean():.0f} 回/分')
    print(f'   内訳: ヨーレート持続 {100*(np.abs(lp)<=LEVEL_YAWRATE_LP).mean():.1f}% / '
          f'瞬時 {100*(np.abs(wz)<=LEVEL_YAWRATE_INST).mean():.1f}% / '
          f'||f|-g| {100*(np.abs(an-G)<=LEVEL_ACCMAG_TOL).mean():.1f}%')
    print(f'   ★ 実機の 60 秒ログの gobs= が増えていればゲートは開いている')


def main():
    ap = argparse.ArgumentParser(
        description='0.983 の変更（GRV 依存の除去・重力観測）を実ログで検証する')
    ap.add_argument('path', help='imuraw/*.bin（または decode_imulog.load() の pickle）')
    ap.add_argument('--session', type=int, default=None, help='このセッションだけ見る')
    ap.add_argument('--cutoff', type=float, default=120.0,
                    help='5) で GNSS を打ち切るまでの秒数（既定 120）')
    ap.add_argument('--min-samples', type=int, default=2000,
                    help='これ未満のセッションは飛ばす（既定 2000）')
    args = ap.parse_args()

    df = pd.read_pickle(args.path) if args.path.endswith('.pkl') else D.load(args.path)
    parts = D.split(df)
    if 'gyro' not in parts or 'accel' not in parts:
        raise SystemExit('gyro / accel が無い。settings.h の '
                         'IMULOG_RAW_REPORTS_ENABLED を 1 にして記録すること')
    if 'gamerv' not in parts:
        print('warn: GAMERV（GRV）が無いので 1) 3) 4) は比較できない。'
              'BNO085 を降ろした後のログか？', file=sys.stderr)

    for sess in sorted(parts['gyro'].session.unique()):
        if args.session is not None and sess != args.session:
            continue
        if (parts['gyro'].session == sess).sum() < args.min_samples:
            continue
        analyse(parts, sess, args.cutoff)


if __name__ == '__main__':
    main()
