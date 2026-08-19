#!/usr/bin/env python3
"""
走行データから路面の縦断勾配・横断勾配（カント）を推定して KML/CSV に出す。

■ 考え方
ESKF は加速度由来の「見かけの傾き」を除去済みなので、残るのは車体の本当の姿勢:

    測定ピッチ = 路面勾配(場所) + サスの沈み込み(前後加速度) + 治具角(セッション)
    測定ロール = 路面カント(場所) + サスのロール(横加速度)     + オフセット(セッション)

同じコースを3周しているため、「路面形状は場所で決まり、加速度は毎周違う」
という条件で両者を最小二乗で同時に分離できる。
場所は約15m格子に丸め、2周以上のデータがある格子だけを使う。

■ 検証
推定した路面形状は3周とも一致するはず。周ごとの残差を出して確認する。
"""
import argparse, os, sys
import numpy as np, pandas as pd

GRID_M = 15.0
DEG_LAT = 111000.0

def fit(df, att_col, acc_col):
    """att = road(bin) + k*acc + c(session) を最小二乗で解く。

    戻り値: (k, 格子ごとの路面値 dict, セッション定数 dict, 残差RMS)
    """
    d = df.dropna(subset=[att_col, acc_col, "latitude", "longitude"]).copy()
    lat0 = d.latitude.mean()
    dlat = GRID_M / DEG_LAT
    dlon = GRID_M / (DEG_LAT * np.cos(np.radians(lat0)))
    d["bin"] = (np.round(d.latitude/dlat).astype(int).astype(str) + "_" +
                np.round(d.longitude/dlon).astype(int).astype(str))
    # 2 周以上のデータがある格子のみ（k と路面を分離するために必要）
    nses = d.groupby("bin").session.nunique()
    d = d[d.bin.isin(nses[nses >= 2].index)]
    if len(d) < 500:
        return None
    bins = sorted(d.bin.unique()); sess = sorted(d.session.unique())
    bi = {b:i for i,b in enumerate(bins)}; si = {s:i for i,s in enumerate(sess)}
    n, nb, ns = len(d), len(bins), len(sess)
    A = np.zeros((n, nb + ns + 1))
    A[np.arange(n), d.bin.map(bi).to_numpy()] = 1.0          # 路面（場所ごと）
    A[np.arange(n), nb + d.session.map(si).to_numpy()] = 1.0  # セッション定数
    A[:, -1] = d[acc_col].to_numpy()                          # 加速度係数 k
    y = d[att_col].to_numpy()
    # 路面の平均を 0 に固定して不定性を除く
    A = np.vstack([A, np.r_[np.ones(nb), np.zeros(ns+1)]])
    y = np.r_[y, 0.0]
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    road = {b: sol[bi[b]] for b in bins}
    csess = {s: sol[nb+si[s]] for s in sess}
    k = sol[-1]
    pred = A[:-1] @ sol
    resid = y[:-1] - pred
    d["resid"] = resid
    d["road"] = d.bin.map(road)
    return dict(k=k, road=road, c=csess, rms=float(np.sqrt((resid**2).mean())), d=d)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="pitchtest_merged.csv")
    ap.add_argument("--laps", default="1,3,5")
    ap.add_argument("-o", "--out", default="road")
    args = ap.parse_args()

    m = pd.read_csv(args.csv)
    laps = [int(x) for x in args.laps.split(",")]
    m = m[m.session.isin(laps) & m.latitude.notna() & (m.gnss_speed > 2)]

    print("=== サスペンション係数と路面形状の同時推定 ===")
    res = {}
    for name, att, acc in (("縦断勾配", "pitch", "accel_long"),
                           ("横断勾配", "roll",  "accel_lat")):
        r = fit(m, att, acc)
        if r is None:
            print(f"  {name}: データ不足"); continue
        res[att] = r
        print(f"  {name}: サス係数 k = {r['k']:+.3f} deg/(m/s²)   "
              f"格子 {len(r['road'])} 個   残差RMS {r['rms']:.2f} deg")
        for s, c in r["c"].items():
            print(f"      session {s} の定数 {c:+.2f} deg")

    # ---- 周ごとの一致検証 ----
    print()
    print("=== 推定路面形状の周ごと一致（同じ格子を各周で比較）===")
    for att, lab in (("pitch","縦断勾配"),("roll","横断勾配")):
        if att not in res: continue
        d = res[att]["d"]
        piv = d.groupby(["bin","session"])[att].mean().unstack()
        # 加速度成分を除いた「路面だけ」の推定値を周ごとに作る
        k = res[att]["k"]; c = res[att]["c"]
        d2 = d.copy()
        d2["road_est"] = d2[att] - k*d2[{"pitch":"accel_long","roll":"accel_lat"}[att]] \
                         - d2.session.map(c)
        p = d2.groupby(["bin","session"]).road_est.mean().unstack()
        p = p.dropna()
        if len(p) < 20: print(f"  {lab}: 比較できる格子が少ない"); continue
        cols = list(p.columns)
        print(f"  {lab}: 共通格子 {len(p)} 個")
        for i in range(len(cols)):
            for j in range(i+1, len(cols)):
                diff = (p[cols[i]] - p[cols[j]])
                print(f"      session {cols[i]} vs {cols[j]}: "
                      f"差の平均 {diff.mean():+.2f} / RMS {np.sqrt((diff**2).mean()):.2f} deg "
                      f"/ 相関 {p[cols[i]].corr(p[cols[j]]):.3f}")

    # ---- 出力 ----
    out = []
    dp = res.get("pitch"); dr = res.get("roll")
    if dp is None: raise SystemExit("縦断勾配を推定できませんでした")
    base = dp["d"].groupby("bin").agg(lat=("latitude","mean"), lon=("longitude","mean"),
                                      n=("latitude","size"))
    base["slope_deg"] = pd.Series(dp["road"])
    if dr is not None:
        base["camber_deg"] = pd.Series(dr["road"])
    base = base.dropna(subset=["slope_deg"]).reset_index()
    base.to_csv(args.out + ".csv", index=False)
    print(f"\nwrote {args.out}.csv  ({len(base)} 点)")

    # KML: 勾配とカントを色分けした点群
    def color(v, lo, hi):
        t = np.clip((v-lo)/(hi-lo), 0, 1)
        r = int(255*t); b = int(255*(1-t))
        return f"ff{b:02x}40{r:02x}"          # aabbggrr
    kml = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
           '<name>road geometry</name>']
    for col, lab, lo, hi in (("slope_deg","縦断勾配", -4, 4),
                             ("camber_deg","横断勾配(カント)", -4, 4)):
        if col not in base: continue
        kml.append(f'<Folder><name>{lab}</name>')
        for _, r in base.iterrows():
            v = r[col]
            if not np.isfinite(v): continue
            kml.append(
              f'<Placemark><name>{v:+.1f}deg</name>'
              f'<Style><IconStyle><color>{color(v,lo,hi)}</color><scale>0.6</scale>'
              f'<Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>'
              f'</Icon></IconStyle></Style>'
              f'<Point><coordinates>{r.lon:.7f},{r.lat:.7f},0</coordinates></Point></Placemark>')
        kml.append('</Folder>')
    kml.append('</Document></kml>')
    open(args.out + ".kml","w",encoding="utf-8").write("\n".join(kml))
    print(f"wrote {args.out}.kml  (青=負 赤=正、±4度でスケール)")

if __name__ == "__main__":
    main()
