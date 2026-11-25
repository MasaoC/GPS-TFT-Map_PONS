#!/usr/bin/env python3
# ============================================================
# File    : tools/repons/test/make_live_sample.py
# Project : PONS v7.1 / v7.2 Re:PONS
# Role    : ライブ配信の窓（live.html）の再生で使う、模擬の記録（受信機が流す行）を作る。
#           受信機も送信機も無しで画面・補間・NO SIGNAL・帯を確かめるため（設計書 §5.7）。
#           琵琶湖のプラットホームで止まってから北パイロンへ飛び、途中で 12 秒受信が切れる。
#           25〜34 秒は「送信機の目的地が受信機に無い」（受信機は目的地を未選択にし、LS の 0x08 を立てる）。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026-11-24
# ============================================================
# 使い方: python3 make_live_sample.py [出力]（既定は同じフォルダの live_sample.txt）
#   live.html の「▶ 記録を再生」で開くか、http で開いて live.html?replay=test/live_sample.txt
# ★ LinkTelem の並びは lora_link/link_proto.h（LINK_PROTO_VER 8・69 バイト）。変わったらここも直す。
import math
import struct
import sys
from pathlib import Path

PLA = (35.294222, 136.254333)
NP = (35.377878, 136.189907)
SP = (35.274220, 136.136183)
K1 = (35.297961, 136.243624)
TK = (35.296584352454964, 136.1780537684742)
RHAVE = dict(GS=0x1, TTRACK=0x2, GNSSALT=0x4, KFALT=0x8, KFVS=0x10, PRESS=0x20, DATE=0x40, NUMSAT=0x80,
             ATT=0x200, ATT_YAW=0x400, ATT_AVG=0x800, ATT_TRIM=0x1000, ATT_WIND=0x2000, TAS=0x4000)
FMT = '<BBBBHBBHHBBBBBBiiHHhhhHhhhhhBBHBBBBHHBbBHHH'
assert struct.calcsize(FMT) == 69


def bearing(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    d = lo2 - lo1
    return (math.degrees(math.atan2(math.sin(d) * math.cos(la2),
                                    math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(d))) + 360) % 360


def dist_km(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 6371.0 * 2 * math.atan2(math.sqrt(h), math.sqrt(1 - h))


def move(p, brg, m):
    R = 6371000.0
    la, lo, b = math.radians(p[0]), math.radians(p[1]), math.radians(brg)
    d = m / R
    la2 = math.asin(math.sin(la) * math.cos(d) + math.cos(la) * math.sin(d) * math.cos(b))
    lo2 = lo + math.atan2(math.sin(b) * math.sin(d) * math.cos(la), math.cos(d) - math.sin(la) * math.sin(la2))
    return (math.degrees(la2), math.degrees(lo2))


def telem(seq, pos, gs, trk, t_utc, roll, pitch, yaw, yacc, pavg, tas, vs, numsat=18, hacc_m=1.4, sacc=0.12,
          batt=1 + 76, status=0, wind=(1.8, 250.0), scale_index=3, dest_id=0x5a3c, dest_index=1, nav=2, fix=True):
    h, m, s, cs = t_utc
    have = (RHAVE['GS'] | RHAVE['TTRACK'] | RHAVE['GNSSALT'] | RHAVE['KFALT'] | RHAVE['KFVS'] | RHAVE['PRESS'] |
            RHAVE['DATE'] | RHAVE['NUMSAT'] | RHAVE['ATT'] | RHAVE['ATT_YAW'] | RHAVE['ATT_AVG'] | RHAVE['ATT_TRIM'] |
            RHAVE['TAS'])
    if wind:
        have |= RHAVE['ATT_WIND']
    return struct.pack(FMT, ord('P'), ord('L'), 8, 0, seq & 0xFFFF, 0, scale_index, dest_id, 2026, 11, 23, h, m, s, cs,
                       round(pos[0] * 1e7), round(pos[1] * 1e7), round(gs * 100), round(trk * 100) % 36000,
                       round(85.0 * 10), round(84.6 * 10), round(vs * 100), round(1008.2 * 10),
                       round(roll * 100), round(pitch * 100), round(yaw * 100) % 36000, round(pavg * 100), round(0.12 * 100),
                       yacc, round((wind or (0, 0))[0] * 10), round((wind or (0, 0))[1] * 100), round(tas * 10), 0,
                       batt, numsat, round(hacc_m * 10), round(sacc * 100), 1 if fix else 0, dest_index, nav,
                       have, status, 0)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent / 'live_sample.txt'
    lines = []
    t = 0.0

    def emit(line, dt=0.0):
        nonlocal t
        t += dt
        lines.append(f'{round(t)}\t{line}')

    emit('PONS:LIVE,OK,ON,1,8,69,0,6,8,1000,1.12')
    dests = [('PLATHOME', PLA), ('N_PILON', NP), ('S_PILON', SP), ('TAKESHIMA', TK), ('SHINURA', (35.650433188178, 139.913699521520))]
    for i, (n, p) in enumerate(dests):
        emit(f'PONS:LIVE,DEST,{i},{p[0]:.7f},{p[1]:.7f},{n}', 2)
    emit(f'PONS:LIVE,PILON,{PLA[0]:.7f},{PLA[1]:.7f},{NP[0]:.7f},{NP[1]:.7f},{SP[0]:.7f},{SP[1]:.7f},{K1[0]:.7f},{K1[1]:.7f},{TK[0]:.7f},{TK[1]:.7f}', 2)
    # 多景島の輪郭のつもりの小さな多角形（内蔵の地図の形を確かめるだけ）
    ring = [move(TK, a, 120) for a in range(0, 360, 45)] + [move(TK, 0, 120)]
    emit(f'PONS:LIVE,MAP,F,1,{len(ring)},g_Takeshima', 2)
    for k in range(0, len(ring), 7):
        pts = ','.join(f'{la:.7f},{lo:.7f}' for la, lo in ring[k:k + 7])
        emit(f'PONS:LIVE,MP,F,1,{k},{pts}', 2)
    emit(f'PONS:LIVE,DUMP_END,{len(dests)},1', 2)

    pos = PLA
    trk = 305.0
    gs = 0.0
    seq = 100
    ms = 500000
    last_trk = trk
    utc = 23 * 3600 + 30 * 60   # 08:30 JST
    rx, miss = 0, 0
    mg, mgh, mgm = 0.0, 8, 30
    lost = range(70, 82)        # この秒は受信が切れる（NO SIGNAL）
    dest_missing = range(25, 35)   # この秒は送信機の目的地が受信機に無い（目的地の線・TC・距離が消え、帯に出る）
    for sec in range(0, 120):
        # 0〜10 秒は止まっている（GROUND）、そこから加速、40〜60 秒は左右にゆっくり S 字
        if sec < 10:
            gs = 0.0
        else:
            gs = min(7.2, gs + 0.9)
        if sec >= 40:
            trk = 305.0 + 18.0 * math.sin((sec - 40) / 9.0)
        dps = ((trk - last_trk + 540) % 360) - 180
        last_trk = trk
        pos = move(pos, trk, gs)
        seq += 1
        ms += 1000
        utc += 1
        mg = max(mg, gs)
        if sec in lost:
            miss += 1
            emit(f'PONS:LS,00,{(sec - lost.start + 1) * 1000},-101,0,{rx},{miss},-1,1000,{len(lines)},0', 1000)
            continue
        rx += 1
        flying = gs >= 2.0
        roll = 6.0 * math.sin((sec - 40) / 9.0 + 1.2) if sec >= 40 else 0.4
        pitch = 3.1 + (0.3 * math.sin(sec / 3.0) if flying else 0.0)
        yaw = trk - 2.5 if flying else trk
        pavg = 3.4 if flying else 1.2
        tas = 7.8 if flying else 6.9
        vs = 0.25 * math.sin(sec / 4.0) if flying else 0.0
        hhmmss = (utc // 3600 % 24, utc // 60 % 60, utc % 60, 0)
        raw = telem(seq, pos, gs, trk, hhmmss, roll, pitch, yaw, 4 if flying else 30, pavg, tas, vs)
        known = 0xFF if sec > 8 else (1 << ((seq % 8) + 1)) - 1
        emit(f'PONS:LT,{ms},{seq % 8},{known:02x},{raw.hex()}', 1000)
        truec = bearing(pos, NP)
        truec = (truec + 180) % 360   # Auto10km の AWAY（目的地から離れる。往路）
        steer = ((truec - trk + 540) % 360) - 180
        hdr = 2 if flying else 4
        strike = 0 if flying else 1
        wait = 0 if (not flying or sec > 22) else max(0, 22 - sec)
        flags = 0x0001 | 0x0400 | 0x2000 | 0x0200
        flags |= 0x0002 | 0x0004 if flying else 0
        if flying:
            flags |= 0x4000
        if 10 <= sec < 22:
            flags |= 0x8000
        if 52 <= sec < 62:
            flags |= 0x0100     # コース警告の箱
        if 44 <= sec < 52:
            flags |= 0x0080     # 方位変化の警告
        cwi = min(900.0, max(0.0, (abs(steer) - 15) * 12)) if flying else 0.0
        dest = -1 if sec in dest_missing else 1
        emit(f'PONS:LN,{seq & 0xFFFF},{flags:04x},{hdr},{strike},{wait},0,2,{trk:.2f},{steer:.2f},{cwi:.1f},'
             f'{dps:.3f},0.20,{dest},2,0,{truec:.2f},{dist_km(pos, NP):.4f},{mg:.2f},{mgh},{mgm + sec // 60},{mg:.2f},{mgh},{mgm + sec // 60}', 40)
        dist_m = round(dist_km(PLA, pos) * 1000)
        lsf = 0x09 if sec in dest_missing else 0x01
        emit(f'PONS:LS,{lsf:02x},300,-77,2,{rx},{miss},{dist_m},1000,{len(lines)},0', 260)
        t -= 300   # LT の間隔を 1000ms にそろえる
    out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'{out}（{len(lines)} 行）')


if __name__ == '__main__':
    main()
