# -*- coding: utf-8 -*-
# ============================================================
# File    : tools/flightanalysis/tas_gamma_check.py
# Project : PONS v7.1
# Role    : 推定 TAS の上昇・降下（経路角 γ）の扱いを決めるための分析（docs/pons_tas_gamma_plan.md の数字の出どころ）。
#           2025・2026 年大会のログから、上下動の大きさ・古いピッチが γ を拾っていたか・γ を引く（C）／横線（B）の効き目・
#           30 秒の試験飛行のシナリオを出す。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/11/27
# ============================================================
#
# 使い方: python3 tools/flightanalysis/tas_gamma_check.py [OLDDATA] [BLE2026]
#   OLDDATA … 大会ログの平文（既定 ~/Documents/Arduino/GPS_TFT_map/tools/olddata）
#              2026-07-26_0804_fused.csv（KF 高度・GNSS 高度・補正済みピッチ・0.5 秒）と
#              20250726_log_all_joined0811edit.csv（2025。対気速度センサー・ピッチ・Garmin 高度・1 秒）
#   BLE2026 … 2026 年の電装のログ（既定 ~/Documents/GitHub/ble_plotter/tools/csv/2026）
#              2026flight.csv（電装の対気速度・1 秒）
# ★ ログは公開の許可が無い。平文をこのリポジトリに置かないこと（tools/olddata/README.md）。
#
# 計算の約束（ファームに合わせる）:
#   平均ピッチ・V/S の平均は「過去 10 秒」の移動平均。V/S の 10 秒平均は KF 高度の 10 秒差 ÷ 10 と同じ。
#   ファームの TAS は V(α) = V0·√((実重量/設計重量)·K/(K+α))。以下の比較は重量比 1 で行う。
#   γ だけで生じる TAS のずれは「ピッチ 1° あたり SENS m/s」×γ[°] で見積もる（新しい本体のピッチは γ を理屈どおり拾うとして）。

import os
import sys
import numpy as np
import pandas as pd
from scipy.optimize import least_squares

OLD = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/Documents/Arduino/GPS_TFT_map/tools/olddata")
BLE = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else "~/Documents/GitHub/ble_plotter/tools/csv/2026")
SENS = 0.36      # ピッチ 1° あたりの TAS [m/s]（2025 の当てはめ 0.36・既定 V0 7.07/K 10 で 0.35）
V_REF = 6.5      # γ を出すときの対気速度 [m/s]（2026 の巡航）
EXTREME = 0.30   # 極端なときだけ横線（B）を出す |V/S の 10 秒平均| [m/s]


def model(th, v0, k):
    return v0 * np.sqrt(k / np.maximum(k + th, 0.5))


def head(s):
    print("\n" + "=" * 70 + "\n" + s + "\n" + "=" * 70)


# ---------------------------------------------------------------- 2026: 上下動は本物か・どれくらいか
head("1. 2026 年の上下動（KF 高度・GNSS 高度・0.5 秒）")
f26 = pd.read_csv(f"{OLD}/2026-07-26_0804_fused.csv")
f26["t"] = pd.to_datetime(f26["date"] + " " + f26["time"])
f26 = f26.set_index("t")
fly = f26.index[f26.gs > 3.0]
t0, t1 = fly.min(), fly.max()
cr = f26.loc[t0 + pd.Timedelta(seconds=60):t1 - pd.Timedelta(seconds=10)]
cr = cr[cr.gs > 3.0]
N = 20   # 0.5 秒 × 20 = 10 秒
dh = (cr.KF_Altitude - cr.KF_Altitude.shift(N))
dg = (cr.GNSS_Altitude - cr.GNSS_Altitude.shift(N))
ok = dh.notna() & dg.notna()
cc = np.corrcoef(dh[ok], dg[ok])[0, 1]
noise_m = np.sqrt(max(dh[ok].var() - cc * dh[ok].std() * dg[ok].std(), 0.0))   # KF の 10 秒差の雑音（GNSS と独立とみて）
print(f"巡航 {(t1 - t0).total_seconds() / 60:.0f} 分。高度の範囲 KF {cr.KF_Altitude.min():.1f}〜{cr.KF_Altitude.max():.1f} m"
      f"（GNSS {cr.GNSS_Altitude.min():.1f}〜{cr.GNSS_Altitude.max():.1f} m。湖面は約 84 m）")
print(f"10 秒の高度差: KF と GNSS の相関 {cc:+.2f}、標準偏差 KF {dh[ok].std():.2f} m・GNSS {dg[ok].std():.2f} m"
      f" → KF の雑音 約 {noise_m:.2f} m（10 秒差）")
vs10 = (dh / 10.0).dropna()
a = np.abs(vs10)
print("|V/S の 10 秒平均| [m/s]: " + "  ".join(f"{q}% {np.percentile(a, q):.2f}" for q in (50, 75, 90, 95, 99)) + f"  最大 {a.max():.2f}")
dke = (cr.gs ** 2 - cr.gs.shift(N) ** 2) / 2 / 9.80665
e = pd.DataFrame({"dke": dke, "dpe": dh}).dropna()
print(f"速度とのやり取り（GS の変化を高さに換算 vs 高度差）: 傾き {np.polyfit(e.dpe, e.dke, 1)[0]:+.2f}"
      f"（風と外部からの仕事を無視したエネルギー交換なら −1）")
print("発進直後の KF 高度: " + "  ".join(
    f"+{s}s {f26.KF_Altitude.asof(t0 + pd.Timedelta(seconds=s)):.1f}" for s in (0, 10, 25, 60, 110)))

# ---------------------------------------------------------------- 古いピッチは γ を拾っていたか
head("2. 古いピッチ（BNO085）は γ を拾っていたか（迎角一定ならピッチの γ への傾きは +1）")
ble = pd.read_csv(f"{BLE}/2026flight.csv")
ble["t"] = pd.to_datetime(ble["timestamp"], format="%Y-%m-%d %H:%M:%S", errors="coerce")
air = ble.dropna(subset=["t"]).groupby("t")["airspeed"].mean()
d = pd.concat([air.rename("air"), f26["pitch"].resample("1s").mean().rename("pitch"),
               f26["KF_Altitude"].resample("1s").mean().rename("alt"), f26["gs"].resample("1s").mean().rename("gs")], axis=1)
d = d.loc[t0 + pd.Timedelta(seconds=60):t1]
d["a10"] = d.air.rolling(10).mean()
d["p10"] = d.pitch.rolling(10).mean()
d["vs10"] = (d.alt - d.alt.shift(10)) / 10
d = d[(d.gs > 3) & (d.air > 4)].dropna(subset=["a10", "p10", "vs10"])
d["gam"] = np.degrees(np.arctan2(d.vs10, d.a10))

j25 = pd.read_csv(f"{OLD}/20250726_log_all_joined0811edit.csv").dropna(
    subset=["Airspeed_compensated2", "pitch_compensated", "g_altitude_compensated", "p_gs"]).reset_index(drop=True)
e25 = pd.DataFrame({"a10": j25.Airspeed_compensated2.rolling(10).mean(), "p10": j25.pitch_compensated.rolling(10).mean(),
                    "vs10": (j25.g_altitude_compensated - j25.g_altitude_compensated.shift(10)) / 10, "gs": j25.p_gs})
i0 = int(np.argmax((j25.p_gs > 3).values))
e25 = e25.loc[i0 + 60:]
e25 = e25[e25.gs > 3].dropna()
e25["gam"] = np.degrees(np.arctan2(e25.vs10, e25.a10))
for name, q in (("2025", e25), ("2026", d)):
    s = np.polyfit(q.gam, q.p10, 1)[0]
    lv = q[np.abs(q.vs10) < 0.05]
    r = least_squares(lambda x: model(lv.p10, x[0], x[1]) - lv.a10, x0=[7, 10], bounds=([3, 1], [15, 60]))
    res = model(q.p10, *r.x) - q.a10
    res_g = model(q.p10 - q.gam, *r.x) - q.a10
    print(f"{name}: ピッチの γ への傾き {s:+.2f}（相関 {np.corrcoef(q.gam, q.p10)[0, 1]:+.2f}）  "
          f"ピッチ 1°あたり {r.x[0] / 2 / r.x[1]:.2f} m/s  推定のずれの γ への傾き {np.polyfit(q.gam, res, 1)[0]:+.3f} m/s/°"
          f"（理屈 {-r.x[0] / 2 / r.x[1]:+.3f}）  RMS: 引かない {np.sqrt((res ** 2).mean()):.2f} / 引く {np.sqrt((res_g ** 2).mean()):.2f}")
print("→ 古いピッチと推定 γ の相関は弱く、この比較では補正後の RMS が大きい。")

# ---------------------------------------------------------------- C の効き目（新しい本体のピッチは γ を拾う前提）
head(f"3. γ だけで生じる TAS のずれ（2026 の上下動・ピッチ 1°={SENS} m/s・V {V_REF} m/s）")
vs = vs10.values
deg = lambda v: np.degrees(np.arctan2(v, V_REF))
rng = np.random.default_rng(0)


def st(x):
    x = np.abs(x[np.isfinite(x)])
    return f"RMS {np.sqrt((x ** 2).mean()):.2f}  中央 {np.median(x):.2f}  90% {np.percentile(x, 90):.2f}  95% {np.percentile(x, 95):.2f}  最大 {x.max():.2f}"


print(f"{'C なし':22s}", st(SENS * deg(vs)))
noise = rng.normal(0, noise_m / 10, len(vs))
for lag in (0.0, 1.0, 2.0):
    sh = int(lag / 0.5)
    meas = 0.88 * pd.Series(vs).shift(sh).values + noise          # 振幅 0.88（KF←GNSS の回帰の傾き）
    print(f"{f'C あり（V/S 遅れ {lag:.0f} 秒）':22s}", st(SENS * (deg(vs) - deg(meas))))
dn = vs <= -0.15
print(f"降下 0.15 m/s 以上の時間 {dn.mean() * 100:.1f}%: C なしだと表示が平均 {SENS * np.abs(deg(vs[dn])).mean():.2f} m/s 速め")

# ---------------------------------------------------------------- B（横線）の出入り
head("4. |V/S の 10 秒平均| がしきい値を超えたら横線（B）にしたとき（2026 の巡航）")
print(f"{'入る':>6}{'抜ける':>7}{'隠れる時間':>10}{'回数/分':>8}{'1 回の中央':>10}{'入口に相当する補正前のずれ':>14}")
dur = len(vs) * 0.5 / 60
for on, off in ((0.10, 0.10), (0.15, 0.15), (0.20, 0.20), (0.20, 0.15), (EXTREME, 0.20)):
    st_on, n, cur, lens, tot = False, 0, 0.0, [], 0.0
    for x in np.abs(vs):
        if not st_on and x >= on:
            st_on, n, cur = True, n + 1, 0.0
        elif st_on and x < off:
            st_on = False; lens.append(cur)
        if st_on:
            cur += 0.5; tot += 0.5
    if st_on: lens.append(cur)
    print(f"{on:6.2f}{off:7.2f}{tot / (len(vs) * 0.5) * 100:9.1f}%{n / dur:8.2f}{(np.median(lens) if lens else 0):9.1f}s"
          f"{SENS * deg(on):13.2f} m/s")

# ---------------------------------------------------------------- 30 秒の試験飛行
head("5. 試験飛行のシナリオ: 浮いてから 15 秒で 3 m 上昇 → 15 秒で 3 m 降下（本当の対気速度は 7.0 m/s で一定）")
dt, V0, K, VT = 0.1, 7.07, 10.0, 7.0
alpha = K * ((V0 / VT) ** 2 - 1)
t = np.arange(-10.0, 30.0 + 1e-9, dt)
v = np.where(t < 0, 0.0, np.where(t < 15, 0.2, -0.2))
k = int(1.0 / dt)
v = np.convolve(np.r_[np.zeros(k - 1), v], np.ones(k) / k, mode="valid")     # 機首の上げ下げは 1 秒で
th = np.where(t < 0, 0.0, alpha + np.degrees(np.arctan2(v, VT)))
n10 = int(10 / dt)
ma = lambda x: np.array([x[max(0, i - n10 + 1):i + 1].mean() for i in range(len(x))])
p10 = ma(th)
noC = model(p10, V0, K)


def withC(lag, iterate):
    s = int(lag / dt)
    vv = ma(np.r_[np.zeros(s), v[:len(v) - s]] if s else v)
    vg = noC.copy()
    for _ in range(2 if iterate else 1):     # iterate: γ を補正後の速度で出し直す
        out = model(p10 - np.degrees(np.arctan2(vv, vg)), V0, K)
        vg = out
    return out


c0, c0i, c1 = withC(0, False), withC(0, True), withC(1, True)
alt = np.cumsum(v) * dt
print(f"{'t[s]':>5}{'高度':>6}{'C なし':>8}{'C あり':>8}{'C（出し直し）':>12}{'C（遅れ 1s）':>12}")
for tt in (5, 10, 15, 20, 25, 30):
    i = int(round((tt + 10) / dt))
    print(f"{tt:5d}{alt[i]:6.1f}{noC[i]:8.2f}{c0[i]:8.2f}{c0i[i]:12.2f}{c1[i]:12.2f}")
fl = (t >= 4) & (t <= 30)
for name, x in (("C なし", noC), ("C あり", c0), ("C（出し直し）", c0i), ("C（遅れ 1s）", c1)):
    print(f"{name:14s} 4〜30 秒: {x[fl].min():.2f}〜{x[fl].max():.2f}（幅 {x[fl].max() - x[fl].min():.2f}・本当からの最大ずれ {np.abs(x[fl] - VT).max():.2f} m/s）")
