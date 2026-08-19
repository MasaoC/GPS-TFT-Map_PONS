#!/usr/bin/env python3
"""
ヨー角を地図に重ねる KML を作る。

■ なぜ車のデータでヨーを検証できるか
車は横滑りも風ドリフトもほぼ無いので「機首方位 ≒ GNSS 対地コース」。
つまり真値が手に入る。飛行機では風のカニ角があるのでこの比較はできない。

■ 出力
  軌跡        : 走行ライン
  ESKF ヨー   : 推定方位へ伸ばした線分。色は真値（対地コース）との誤差
  BNO085 ヨー : 同上（比較用）
  色: 青=負 緑=0付近 赤=正、±20度でスケール

■ 注意
デバイスの取り付け方位のズレ（定数）はセッションごとに円平均で除去してある。
誤差として見えるのはそれ以外の成分。
"""
import argparse, os, sys
import numpy as np, pandas as pd

def wrap(d): return (d+180.0)%360.0-180.0
def cmean(d):
    r=np.radians(d); return np.degrees(np.arctan2(np.sin(r).mean(), np.cos(r).mean()))

def color_for(v, span=20.0):
    """誤差 -span..+span を 青→緑→赤 に割り当てる。KML は aabbggrr。"""
    t = np.clip((v+span)/(2*span), 0, 1)
    if t < 0.5:  r,g,b = 0, int(255*(t*2)), int(255*(1-t*2))
    else:        r,g,b = int(255*((t-0.5)*2)), int(255*(1-(t-0.5)*2)), 0
    return f"ff{b:02x}{g:02x}{r:02x}"

# 方位はアイコンではなく短い線分で描く。
# アイコン（arrow.png）は既定の向きが仕様として保証されておらず、
# 実機で確認したところ矢印が進行方向と 180 度逆に出た。
# 線分なら「点から推定方位へ伸ばす」だけなので向きの曖昧さが無い。
SEG_M = 12.0          # 方位線の長さ [m]
DEG_LAT = 111000.0

def heading_segment(lat, lon, hd_deg, length_m=SEG_M):
    """(lat,lon) から方位 hd_deg（真北基準・時計回り）へ length_m 伸ばした端点を返す。"""
    r = np.radians(hd_deg)
    dlat = (length_m * np.cos(r)) / DEG_LAT
    dlon = (length_m * np.sin(r)) / (DEG_LAT * np.cos(np.radians(lat)))
    return lat + dlat, lon + dlon


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="pitchtest2_merged.csv")
    ap.add_argument("--sessions", default="1,3,5,9")
    ap.add_argument("--skip", type=float, default=60.0,
                    help="各セッション先頭の除外秒数（ヨーの収束待ち）")
    ap.add_argument("--every", type=float, default=2.0, help="矢印の間隔[秒]")
    ap.add_argument("-o","--out", default="yaw.kml")
    args=ap.parse_args()

    m=pd.read_csv(args.csv)
    sess=[int(x) for x in args.sessions.split(",")]
    kml=['<?xml version="1.0" encoding="UTF-8"?>',
         '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
         '<name>ESKF yaw</name>']
    print(f"{'sess':>5}{'点数':>8}{'ESKF残差RMS':>13}{'BNO残差RMS':>12}")
    for s in sess:
        d=m[(m.session==s)&m.latitude.notna()&(m.gnss_speed>4)].dropna(
            subset=["yaw","bno_yaw","TrueTrack"]).sort_values("t")
        if len(d)<100: continue
        d=d[d.t >= d.t.min()+args.skip]        # 収束前を除外
        if len(d)<100: continue
        # 取り付け方位のズレ（定数）を除去
        de=wrap(d.yaw-d.TrueTrack);  oe=cmean(de); ee=wrap(de-oe)
        db=wrap(d.bno_yaw-d.TrueTrack); ob=cmean(db); eb=wrap(db-ob)
        print(f"{s:>5}{len(d):>8}{np.sqrt((ee**2).mean()):>13.2f}{np.sqrt((eb**2).mean()):>12.2f}")

        kml.append(f'<Folder><name>session {s}</name>')
        # 軌跡
        coords=" ".join(f"{r.longitude:.7f},{r.latitude:.7f},0" for _,r in d.iterrows())
        kml.append(f'<Placemark><name>track</name>'
                   f'<Style><LineStyle><color>ff808080</color><width>2</width></LineStyle></Style>'
                   f'<LineString><coordinates>{coords}</coordinates></LineString></Placemark>')
        # 矢印（間引き）
        step=max(1,int(args.every/np.median(np.diff(d.t))))
        for lab, yawcol, err, off in (("ESKF", d.yaw, ee, oe), ("BNO085", d.bno_yaw, eb, ob)):
            kml.append(f'<Folder><name>{lab} yaw</name>')
            for i in range(0, len(d), step):
                r=d.iloc[i]; e=err.iloc[i]
                # 取り付けズレを引いた「真方位として推定された向き」へ線分を伸ばす
                hd=(yawcol.iloc[i]-off)%360.0
                la,lo = heading_segment(r.latitude, r.longitude, hd)
                kml.append(
                  f'<Placemark><name>{e:+.0f}deg</name>'
                  f'<Style><LineStyle><color>{color_for(e)}</color><width>3</width></LineStyle></Style>'
                  f'<LineString><coordinates>'
                  f'{r.longitude:.7f},{r.latitude:.7f},0 {lo:.7f},{la:.7f},0'
                  f'</coordinates></LineString></Placemark>')
            kml.append('</Folder>')
        kml.append('</Folder>')
    kml.append('</Document></kml>')
    open(args.out,"w",encoding="utf-8").write("\n".join(kml))
    print(f"\nwrote {args.out}  (青=負 緑=0 赤=正、±20度)")

if __name__=="__main__":
    main()
