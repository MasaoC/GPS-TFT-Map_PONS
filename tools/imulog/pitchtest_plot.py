#!/usr/bin/env python3
"""
傾斜台ピッチ試験（20260819）の可視化。

治具角度という「真値」があるので、走行中に ESKF と BNO085 の
どちらが正しいかを直接判定できる。これまでの車載データには真値が無く、
両者の差しか見られなかった。

構成（電源再投入でセッションが分かれている）:
  s0 駐車場 0度静止 / s1 0度で1周 / s2 路肩10度静止 / s3 10度で1周
  s4 路肩20度静止 / s5 20度で1周 / s6 駐車場20度静止
"""
import os, sys, contextlib
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_imulog import load, split, build_euler
from eskf import run_on_log
from merge_flight import motion_from_gnss

plt.rcParams["font.family"]="Hiragino Sans"
plt.rcParams["axes.grid"]=True; plt.rcParams["grid.alpha"]=0.25
plt.rcParams["grid.linewidth"]=0.6
plt.rcParams["axes.spines.top"]=False; plt.rcParams["axes.spines.right"]=False
plt.rcParams["axes.edgecolor"]="#8a8a85"
for k in ("axes.labelcolor","xtick.color","ytick.color"): plt.rcParams[k]="#52514e"
C_ESKF="#2a78d6"; C_BNO="#eb6834"; C_IN="#52514e"; LW=1.8

BIN = sys.argv[1] if len(sys.argv)>1 else "20260819_pitchtest.bin"
LAPS = {1:("0度", 0.0), 3:("10度", 10.0), 5:("20度", 20.0)}
STATIC_REF = {1:0, 3:2, 5:4}      # 各周回の直前の静止セッション

parts = split(load(BIN))
with contextlib.redirect_stdout(open(os.devnull,"w")):
    est = run_on_log(parts)
bno = build_euler(parts)

def sess_df(sess):
    e = est[est.session==sess][["t","roll","pitch"]].sort_values("t")
    b = bno[bno.session==sess][["t","roll","pitch"]].rename(
        columns={"roll":"b_roll","pitch":"b_pitch"}).sort_values("t")
    d = pd.merge_asof(e, b, on="t", direction="nearest", tolerance=0.2)
    gn = parts["gnssvel"]; gs = gn[gn.session==sess]
    mot = motion_from_gnss(gs, sacc_max=3.0)
    if mot is not None:
        d = pd.merge_asof(d.sort_values("t"), mot.sort_values("t"),
                          on="t", direction="nearest", tolerance=1.0)
    return d

# 各静止セッションの基準ピッチ
ref = {}
for s in (0,2,4):
    d = sess_df(s)
    ref[s] = (d.pitch.median(), d.b_pitch.median())

fig, axes = plt.subplots(4, 3, figsize=(15,10),
                         gridspec_kw={"height_ratios":[1,2,2,2],"hspace":0.45,"wspace":0.25})
rows=[]
for col,(sess,(lab,jig)) in enumerate(LAPS.items()):
    d = sess_df(sess)
    d = d[d.gnss_speed.notna() & (d.gnss_speed>2)]
    t0 = d.t.min(); tt = d.t - t0
    pe_ref, pb_ref = ref[STATIC_REF[sess]]

    ax=axes[0,col]
    ax.plot(tt, d.accel_long, color=C_IN, lw=0.9)
    ax.axhline(0,color="#c9c9c4",lw=.8); ax.set_ylim(-2.5,2.5)
    ax.set_title(f"session {sess}  治具 {lab}  1周",fontsize=11,loc="left")
    ax.set_ylabel("前後加速度\n[m/s²]",fontsize=8); ax.tick_params(labelsize=8,labelbottom=False)

    ax=axes[1,col]
    ax.plot(tt, d.pitch-pe_ref, color=C_ESKF, lw=LW, label="ESKF")
    ax.plot(tt, d.b_pitch-pb_ref, color=C_BNO, lw=1.2, alpha=.9, label="BNO085")
    ax.axhline(0,color="#c9c9c4",lw=1.0,ls="--")
    ax.set_ylabel("ピッチ（直前静止を\nゼロ点に補正）[deg]",fontsize=8)
    ax.set_ylim(-8,8); ax.tick_params(labelsize=8,labelbottom=False)
    if col==0: ax.legend(fontsize=8,frameon=False,ncol=2,loc="upper right")

    ax=axes[2,col]
    ax.plot(tt, d.pitch-d.b_pitch, color="#2a2a28", lw=1.4)
    ax.axhline(0,color="#c9c9c4",lw=.8); ax.set_ylim(-5,5)
    ax.set_ylabel("ESKF − BNO085\n[deg]",fontsize=8)
    ax.set_xlabel("経過 [s]",fontsize=8); ax.tick_params(labelsize=8)

    # 前後加速度に対する感度（真値が一定なので、傾きがそのまま誤差感度）
    ax=axes[3,col]
    q=d.dropna(subset=["accel_long"])
    for nm,c,y in (("ESKF",C_ESKF,q.pitch-pe_ref),("BNO085",C_BNO,q.b_pitch-pb_ref)):
        ax.scatter(q.accel_long,y,s=2,color=c,alpha=.18,linewidths=0)
        k=np.polyfit(q.accel_long,y,1)
        xs=np.linspace(-2,2,10); ax.plot(xs,np.polyval(k,xs),color=c,lw=2)
        if nm=="ESKF": se=k[0]
        else: sb=k[0]
    ax.axhline(0,color="#c9c9c4",lw=.8); ax.set_ylim(-8,8); ax.set_xlim(-2,2)
    ax.set_xlabel("前後加速度 [m/s²]",fontsize=8)
    ax.set_ylabel("ピッチ偏差 [deg]",fontsize=8); ax.tick_params(labelsize=8)
    ax.set_title(f"感度 ESKF {se:+.2f} / BNO {sb:+.2f} deg/(m/s²)",fontsize=9,loc="left")

    rows.append(dict(sess=sess, jig=lab,
        e_mean=(d.pitch-pe_ref).mean(), e_sd=(d.pitch-pe_ref).std(),
        b_mean=(d.b_pitch-pb_ref).mean(), b_sd=(d.b_pitch-pb_ref).std(),
        e_sens=se, b_sens=sb))

fig.suptitle("傾斜台ピッチ試験 — 走行中に治具角度を保てるか（真値が既知）",fontsize=13,y=0.995)
fig.savefig("figs/pitchtest.png",dpi=150,bbox_inches="tight",facecolor="#fcfcfb")
print(pd.DataFrame(rows).round(3).to_string(index=False))
print("\nwrote figs/pitchtest.png")
