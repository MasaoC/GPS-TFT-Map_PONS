// ============================================================
// File    : link_beacon.h
// Project : PONS v7 — PONS Link
// Role    : GNSS の時刻の助け合いに使うビーコン（32 バイト）の並び。テレメトリ（link_wire2.h）とは
//           印だけで見分ける。要求（時刻がほしい）と提供（いまの時刻）の 2 種類。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/13
// ============================================================
//
// ★ 使い方と決めごとは docs/pons_ttff_assist_plan.md §4（2026-11-13）。動きは link_assist.cpp。
// ★ **並び（バイトの位置）は PONSv7 と PONSv7.1 で同じに保つ**（テレメトリと同じ約束）。
//   形を変えたら版（種類のバイトの 2〜3 ビット）を上げる。知らない版は読まない。
// ★ 長さはテレメトリと同じ 32 バイト。e220.cpp の受信は 32 バイト＋RSSI で切り出していて、
//   印と CRC で 1 フレームを確かめる。長さをそろえておけば、受信の切り出しはそのまま使える。
// ★ 提供の時刻には送り主の GNSS の精度（tAcc）を添える。fix の前の時刻（GNSS の時計＝RTC から）も共有するため（2026-11-13）。
// ★ 時刻だけを GNSS へ入れる（2026-11-13 決定）。位置も載せておくが、いまは受けた側で使わない
//   （並びは後から変えにくいので、空いているバイトに先に入れておいた。使うかは机の上の試験で決める。計画書 §8）。
//
#ifndef LINK_BEACON_H
#define LINK_BEACON_H

#include <stdint.h>
#include <string.h>
#include "link_proto.h"
#include "link_wire2.h"

#define LINK_BC_LEN         32
#define LINK_BC_MAGIC       0x58   // 印（上位 6 ビット）。下位 2 ビットはグループ（テレメトリと同じ）
#define LINK_BC_MAGIC_MASK  0xFC
static_assert(LINK_BC_LEN == LINK_W2_LEN, "ビーコンはテレメトリと同じ長さ（e220.cpp の切り出しを共有する）");
static_assert((LINK_BC_MAGIC & ~LINK_BC_MAGIC_MASK) == 0, "印の下位 2 ビットはグループ用に空けておく");
// テレメトリの印（いまの 0xA4・古い 0xA2）と、65 バイト時代の先頭 'P'（0x50）と見分けがつくこと。
// テレメトリの印は版を変えるたびに 4 つ飛ばしで上がる（0xA8 …）ので、離れた値にしてある（6 ビットが全部違う）。
static_assert((LINK_BC_MAGIC & LINK_BC_MAGIC_MASK) != (LINK_W2_MAGIC & LINK_W2_MAGIC_MASK), "テレメトリの印とぶつかっている");
static_assert((LINK_BC_MAGIC & LINK_BC_MAGIC_MASK) != (0xA2 & LINK_BC_MAGIC_MASK), "古い並び 0xA2 とぶつかっている");
static_assert((LINK_BC_MAGIC & LINK_BC_MAGIC_MASK) != (0x50 & LINK_BC_MAGIC_MASK), "65 バイト時代の先頭とぶつかっている");

// ---- 位置（バイト） ----
#define BC_MAGIC   0    // 1B: 印 6 ビット | グループ 2 ビット（下位）
#define BC_SEQ     1    // 1B: 送るたびに 1 つ進める（ログ用）
#define BC_KIND    2    // 1B: 種類 2 ビット（LINK_BC_REQ / LINK_BC_OFFER）| 版 2 ビット | 位置あり 1 ビット
#define BC_UTC     3    // 4B: UTC の秒（2020-01-01 00:00:00 から）。要求では 0xFFFFFFFF
#define BC_MS      7    // 2B: その秒の中の ms（0〜999）。要求では 0xFFFF
#define BC_ID      9    // 2B: 送り主の短い ID（本体の固有 ID の CRC16。0 は使わない）
#define BC_NSAT    11   // 1B: 送り主の衛星数（ログ用）
#define BC_LAT     12   // 4B: 緯度 1e-7 度（位置ありのときだけ）
#define BC_LON     16   // 4B: 経度 1e-7 度
#define BC_ALT     20   // 2B: GNSS 高度 [m]（符号付き）
#define BC_TACC    22   // 2B: 送り主の時刻の精度 [ms]（GNSS の NAV-PVT tAcc。fix の前の RTC の時刻でも入る）。0xFFFF = 分からない
                        // 24〜29: 0（予備）
#define BC_CRC     30   // 2B: 0..29 の CRC16-CCITT（テレメトリと同じ）
static_assert(BC_CRC + 2 == LINK_BC_LEN, "並びが 32 バイトに合っていない");

#define LINK_BC_REQ        1   // 要求: 時刻がほしい（時刻と位置は空）
#define LINK_BC_OFFER      2   // 提供: 送り主のいまの時刻（UART へ書き始めた瞬間の UTC）
#define LINK_BC_KIND_MASK  0x03
#define LINK_BC_VER        0   // 並びの版（BC_KIND の 2〜3 ビット）。並びを変えたら上げる
#define LINK_BC_VER_SHIFT  2
#define LINK_BC_F_POS      0x10

#define LINK_BC_EPOCH_UNIX 1577836800ULL   // 2020-01-01 00:00:00 UTC の UNIX 時刻 [s]

// メモリの上の形
struct LinkBeacon {
    uint8_t  group;      // 0..3（A〜D）
    uint8_t  seq;
    uint8_t  kind;       // LINK_BC_REQ / LINK_BC_OFFER
    bool     has_time;   // 提供のときだけ true
    uint64_t utc_ms;     // UNIX 時刻 [ms]（has_time のとき）
    uint16_t sender;     // 送り主の短い ID
    uint8_t  numsat;
    bool     has_pos;
    int32_t  lat_1e7, lon_1e7;
    int16_t  alt_m;
    uint16_t tacc_ms;    // 送り主の時刻の精度 [ms]（提供のとき）。0xFFFF = 分からない（使わない）
};

static inline bool link_bc_is_magic(uint8_t b) { return (b & LINK_BC_MAGIC_MASK) == LINK_BC_MAGIC; }
static inline bool link_bc_valid(const uint8_t* p) {
    return link_bc_is_magic(p[BC_MAGIC]) && w2_get16(p + BC_CRC) == link_crc16(p, BC_CRC);
}
// 32 バイトのフレーム（テレメトリかビーコン）の先頭になりうるか・形として正しいか。e220.cpp の受信の切り出しが使う。
static inline bool link_frame_is_magic(uint8_t b) { return link_w2_is_magic(b) || link_bc_is_magic(b); }
static inline bool link_frame_valid(const uint8_t* p) { return link_w2_valid(p) || link_bc_valid(p); }

static inline void link_bc_encode(const LinkBeacon* b, uint8_t* out) {
    memset(out, 0, LINK_BC_LEN);
    out[BC_MAGIC] = (uint8_t)(LINK_BC_MAGIC | (b->group & 0x03));
    out[BC_SEQ]   = b->seq;
    uint8_t k = (uint8_t)((b->kind & LINK_BC_KIND_MASK) | (LINK_BC_VER << LINK_BC_VER_SHIFT));
    const bool tok = b->has_time && b->utc_ms >= LINK_BC_EPOCH_UNIX * 1000ULL;
    if (tok) {
        w2_put32(out + BC_UTC, (uint32_t)(b->utc_ms / 1000ULL - LINK_BC_EPOCH_UNIX));
        w2_put16(out + BC_MS,  (uint16_t)(b->utc_ms % 1000ULL));
    } else {
        w2_put32(out + BC_UTC, 0xFFFFFFFFUL);
        w2_put16(out + BC_MS,  0xFFFF);
    }
    w2_put16(out + BC_TACC, tok ? b->tacc_ms : (uint16_t)0xFFFF);
    w2_put16(out + BC_ID, b->sender);
    out[BC_NSAT] = b->numsat;
    if (b->has_pos) {
        k |= LINK_BC_F_POS;
        w2_put32(out + BC_LAT, (uint32_t)b->lat_1e7);
        w2_put32(out + BC_LON, (uint32_t)b->lon_1e7);
        w2_put16(out + BC_ALT, (uint16_t)b->alt_m);
    }
    out[BC_KIND] = k;
    w2_put16(out + BC_CRC, link_crc16(out, BC_CRC));
}

// 形として正しいもの（link_bc_valid）を中身に戻す。知らない版・種類なら false（読まない）。
static inline bool link_bc_decode(const uint8_t* p, LinkBeacon* b) {
    memset(b, 0, sizeof(*b));
    const uint8_t k = p[BC_KIND];
    if (((k >> LINK_BC_VER_SHIFT) & 0x03) != LINK_BC_VER) return false;
    b->kind  = (uint8_t)(k & LINK_BC_KIND_MASK);
    if (b->kind != LINK_BC_REQ && b->kind != LINK_BC_OFFER) return false;
    b->group  = (uint8_t)(p[BC_MAGIC] & 0x03);
    b->seq    = p[BC_SEQ];
    b->sender = w2_get16(p + BC_ID);
    b->numsat = p[BC_NSAT];
    const uint32_t s  = w2_get32(p + BC_UTC);
    const uint16_t ms = w2_get16(p + BC_MS);
    b->has_time = (s != 0xFFFFFFFFUL && ms < 1000);
    b->tacc_ms  = w2_get16(p + BC_TACC);
    if (b->has_time) b->utc_ms = ((uint64_t)s + LINK_BC_EPOCH_UNIX) * 1000ULL + ms;
    b->has_pos = (k & LINK_BC_F_POS) != 0;
    if (b->has_pos) {
        b->lat_1e7 = (int32_t)w2_get32(p + BC_LAT);
        b->lon_1e7 = (int32_t)w2_get32(p + BC_LON);
        b->alt_m   = (int16_t)w2_get16(p + BC_ALT);
    }
    return true;
}

#endif // LINK_BEACON_H
