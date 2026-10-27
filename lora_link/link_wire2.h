// ============================================================
// File    : link_wire2.h
// Project : PONS v7 — PONS Link
// Role    : 電波に載せる 32 バイトのフレーム（形式 2）。LinkTelem（65 バイト）を詰めて送り、
//           受けたら LinkTelem に戻す。画面・ログ・リプレイは LinkTelem のまま何も変えない。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/27
// ============================================================
//
// ★ **2026-10-27 から、電波に載せるのはどの CH でもこの 32 バイトだけ**（ローワーもアッパーも）。
//   形式を見分ける仕掛けは無い。**65 バイトを送っていた古い版の機体とは通じない**ので、3 台ともそろえる。
//   形は debug/v7-noise-test で先に試したものと同じ（印 0xA2）なので、試験ブランチの機体とも通じる。
//
// ★ なぜ 32 バイトにするのか（docs/pons_link.md §2「電波に載せる 32 バイトのフレーム」）
//   ・送信時間が 65 バイトの約 6 割になる。送信電流もそのぶん減り、同じ送信時間なら SF を 1 段上げられる
//     （約 2.5dB 遠くまで届く）。
//   ・アッパーバンド（CH13〜）は 1 パケットが 32 バイトに固定される。65 バイトだと 3 つに分割され、
//     休止（送信時間の 10 倍）も 3 回入って 1Hz で送れない。32 バイトなら 1 パケットに収まる。
//
// ★ 作り
//   ・毎回送る「核」（位置・速度・方位・高度・昇降・姿勢・状態）と、3 バイトの「回転枠」。
//     回転枠の中身は seq を LINK_W2_SLOTS で割った余りで決まり、ゆっくりした項目
//     （日付・平均ピッチ・風・電圧・気圧・目的地など）を順番に運ぶ（8 発ごと）。
//   ・受信側は回転枠の値を覚えておき、LinkTelem を組み立て直す（LinkW2Rx）。
//     回転枠の項目は、その枠を受けるまで have ビットを立てない（分からない値を出さない）。
//     have ビットの無い項目（目的地・目的地の ID・ナビモード・縮尺・水平精度）は link_w2_slot_seen() で確かめる。
//
// ★ 細かさは表示より細かく取ってある（位置は 1e-7 度のまま。角度 0.1°、速度 0.025m/s、
//   昇降 0.05m/s、時刻 0.1 秒）。値の範囲を超えたら端に丸める（昇降 ±6.35m/s、速度 102m/s まで）。
//
// ★ 形を変えたら LINK_W2_MAGIC を変えること（古い機体が新しい形を誤読しないように）。
//   PONSv7 / PONSv7.1 / debug/v7-noise-test の並び（バイトの位置）は同じに保つ。
//   目的地の ID（回転枠 7 の上 16 ビット）は 2026-10-27 に PONSv7 / v7.1 だけ足した。noise-test の機体は
//   そこに 0（＝送っていない）を送るので、混ぜても受信側が番号で合わせるだけで通じる。
//   受信ログ（received/）は LinkTelem から書く。末尾の slot 列で、どの回転枠が入っていたか
//   （＝その行で更新されたゆっくりした項目）が分かる。
//
#ifndef LINK_WIRE2_H
#define LINK_WIRE2_H

#include <stdint.h>
#include <string.h>
#include "link_proto.h"

#define LINK_W2_LEN        32
#define LINK_W2_MAGIC      0xA2   // 形式 2 の印（以前の 65 バイトは 'P'＝0x50 で始まる）
#define LINK_W2_SLOTS      8      // 回転枠の数。**256 の約数にすること**（seq の 8 ビットが一周しても続くように）

// ---- 位置（バイト） ----
#define W2_MAGIC   0
#define W2_SEQ     1
#define W2_GROUP   2
#define W2_STATUS  3    // 2B
#define W2_FLAGS   5
#define W2_TOD     6    // 2B: その時の中の 0.1 秒（0..35999）。0xFFFF = 時刻なし
#define W2_LAT     8    // 4B: 1e-7 度
#define W2_LON     12   // 4B
#define W2_GSTRK   16   // 3B: 対地速度 12bit（0.025m/s）| 真方位 12bit（0.1°）
#define W2_KFALT   19   // 2B: 0.1m
#define W2_KFVS    21   // 1B: 0.05m/s（符号付き）
#define W2_ATT     22   // 5B: ロール 12bit | ピッチ 12bit（0.1°、符号付き）| ヨー 12bit（0.1°、0..3599）
#define W2_SLOT    27   // 3B: 回転枠
#define W2_CRC     30   // 2B: 0..29 の CRC16-CCITT
static_assert(W2_CRC + 2 == LINK_W2_LEN, "形式 2 の並びが 32 バイトに合っていない");

// 核のフラグ（W2_FLAGS）
#define W2F_FIXOK  0x01   // gnssFixOK
#define W2F_GS     0x02   // 対地速度・真方位（RHAVE_GS | RHAVE_TTRACK）
#define W2F_KFALT  0x04
#define W2F_KFVS   0x08
#define W2F_ATT    0x10   // ロール・ピッチ（RHAVE_ATT）
#define W2F_YAW    0x20   // ヨー（RHAVE_ATT_YAW。精度は回転枠 1）
#define W2F_TOD    0x40   // 時刻が有効

// 回転枠の番号
#define W2S_DATE     0   // 年(7) 月(4) 日(5) 時(5) 有効(1)
#define W2S_PAVG     1   // 平均ピッチ 12bit(0.1°) | ヨー精度 8bit | 平均ピッチ有効(1) | ヨー精度有効(1)
#define W2S_TRIMW    2   // ロールトリム 12bit(0.01°) | 風速 8bit(0.1m/s) | トリム有効(1) | 風有効(1)
#define W2S_WDIR     3   // 風向 12bit(0.1°) | 衛星数 6bit | 衛星数有効(1) | 風有効(1)
#define W2S_ALTV     4   // GNSS 高度 16bit(0.1m。INT16_MIN = 無し) | 電圧 8bit(link_pack_volt。0 = 無し)
#define W2S_PRESS    5   // 気圧 16bit(0.1hPa。0 = 無し) | 地図の縮尺 8bit
#define W2S_HACC     6   // 水平精度 16bit(0.1m) | 目的地 8bit(符号付き)
#define W2S_NAV      7   // ナビモード 8bit | 目的地の ID 16bit（0 = 送っていない。2026-10-27〜）

// 回転枠で運ぶ have ビット（受けるまで立てない）
#define W2_SLOT_HAVE (RHAVE_DATE | RHAVE_ATT_AVG | RHAVE_ATT_TRIM | RHAVE_ATT_WIND | \
                      RHAVE_NUMSAT | RHAVE_GNSSALT | RHAVE_VOLT | RHAVE_PRESS)

// ---- 小さな道具 ----
static inline void w2_put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint16_t w2_get16(const uint8_t* p)   { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void w2_put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static inline uint32_t w2_get32(const uint8_t* p)   { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void w2_put24(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); }
static inline uint32_t w2_get24(const uint8_t* p)   { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }
static inline int32_t  w2_clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
// n ビットの符号付き値を取り出す
static inline int32_t  w2_sext(uint32_t v, int bits) { const uint32_t m = 1u << (bits - 1); v &= (1u << bits) - 1; return (int32_t)((v ^ m) - m); }
// 割り算の四捨五入（負の数も）
static inline int32_t  w2_divr(int32_t a, int32_t b) { return (a >= 0) ? (a + b / 2) / b : -((-a + b / 2) / b); }

static inline uint8_t link_w2_slot_of(uint8_t seq8) { return (uint8_t)(seq8 % LINK_W2_SLOTS); }

// ============================================================
//  送信側: LinkTelem → 32 バイト
// ============================================================
static inline void link_w2_encode(const LinkTelem* t, uint8_t* out) {
    memset(out, 0, LINK_W2_LEN);
    const uint8_t seq8 = (uint8_t)t->seq;
    out[W2_MAGIC] = LINK_W2_MAGIC;
    out[W2_SEQ]   = seq8;
    out[W2_GROUP] = t->group;
    w2_put16(out + W2_STATUS, t->status);

    uint8_t f = 0;
    if (t->fixflags & 0x01)                         f |= W2F_FIXOK;
    if ((t->have & (RHAVE_GS | RHAVE_TTRACK)) == (RHAVE_GS | RHAVE_TTRACK)) f |= W2F_GS;
    if (t->have & RHAVE_KFALT)                      f |= W2F_KFALT;
    if (t->have & RHAVE_KFVS)                       f |= W2F_KFVS;
    if (t->have & RHAVE_ATT)                        f |= W2F_ATT;
    if (t->have & RHAVE_ATT_YAW)                    f |= W2F_YAW;
    // 時刻は年か時分秒のどれかが入っていれば有効とみなす（LinkTelem は時刻だけの have ビットを持たない）
    const bool tod_ok = (t->hour < 24 && t->minute < 60 && t->second < 60);
    if (tod_ok && (t->year != 0 || t->hour || t->minute || t->second)) f |= W2F_TOD;
    out[W2_FLAGS] = f;

    const uint16_t tod = (f & W2F_TOD)
        ? (uint16_t)((t->minute * 60 + t->second) * 10 + t->centi / 10) : 0xFFFF;
    w2_put16(out + W2_TOD, tod);

    w2_put32(out + W2_LAT, (uint32_t)t->lat_1e7);
    w2_put32(out + W2_LON, (uint32_t)t->lon_1e7);

    const uint32_t gs12  = (uint32_t)w2_clampi(w2_divr(t->gs_cms * 2, 5), 0, 4095);    // 0.01 → 0.025 m/s
    const uint32_t trk12 = (uint32_t)(w2_divr(t->track_cdeg, 10) % 3600);              // 0.01 → 0.1°
    w2_put24(out + W2_GSTRK, gs12 | (trk12 << 12));

    w2_put16(out + W2_KFALT, (uint16_t)t->kf_alt_dm);
    out[W2_KFVS] = (uint8_t)(int8_t)w2_clampi(w2_divr(t->kf_vs_cms, 5), -127, 127);   // 0.01 → 0.05 m/s

    const uint32_t r12 = (uint32_t)w2_clampi(w2_divr(t->roll_cdeg,  10), -2047, 2047) & 0xFFF;
    const uint32_t p12 = (uint32_t)w2_clampi(w2_divr(t->pitch_cdeg, 10), -2047, 2047) & 0xFFF;
    const uint32_t y12 = (uint32_t)(((w2_divr(t->yaw_cdeg, 10) % 3600) + 3600) % 3600);
    const uint64_t att = (uint64_t)r12 | ((uint64_t)p12 << 12) | ((uint64_t)y12 << 24);
    for (int i = 0; i < 5; i++) out[W2_ATT + i] = (uint8_t)(att >> (8 * i));

    uint32_t s = 0;
    switch (link_w2_slot_of(seq8)) {
    case W2S_DATE: {
        const bool ok = (t->have & RHAVE_DATE) && t->year >= 2000 && t->year < 2128;
        if (ok) s = (uint32_t)(t->year - 2000) | ((uint32_t)(t->month & 0xF) << 7) |
                    ((uint32_t)(t->day & 0x1F) << 11) | ((uint32_t)(t->hour & 0x1F) << 16) | (1u << 21);
        break; }
    case W2S_PAVG: {
        const uint32_t pa = (uint32_t)w2_clampi(w2_divr(t->pitch_avg_cdeg, 10), -2047, 2047) & 0xFFF;
        s = pa | ((uint32_t)t->yaw_acc95_deg << 12);
        if (t->have & RHAVE_ATT_AVG) s |= 1u << 20;
        if (t->have & RHAVE_ATT_YAW) s |= 1u << 21;
        break; }
    case W2S_TRIMW: {
        const uint32_t tr = (uint32_t)w2_clampi(t->roll_trim_cdeg, -2047, 2047) & 0xFFF;
        s = tr | ((uint32_t)t->wind_dmps << 12);
        if (t->have & RHAVE_ATT_TRIM) s |= 1u << 20;
        if (t->have & RHAVE_ATT_WIND) s |= 1u << 21;
        break; }
    case W2S_WDIR: {
        const uint32_t wd = (uint32_t)(w2_divr(t->wind_dir_cdeg, 10) % 3600);
        s = wd | ((uint32_t)(t->numsat > 63 ? 63 : t->numsat) << 12);
        if (t->have & RHAVE_NUMSAT)   s |= 1u << 18;
        if (t->have & RHAVE_ATT_WIND) s |= 1u << 19;
        break; }
    case W2S_ALTV: {
        const uint16_t alt = (t->have & RHAVE_GNSSALT) ? (uint16_t)w2_clampi(t->gnss_alt_dm, -32767, 32767)
                                                       : (uint16_t)0x8000;
        const uint8_t  v   = (t->have & RHAVE_VOLT) ? (t->volt_cv ? t->volt_cv : 1) : 0;
        s = alt | ((uint32_t)v << 16);
        break; }
    case W2S_PRESS: {
        const uint16_t pr = (t->have & RHAVE_PRESS) ? (t->press_dpa ? t->press_dpa : 1) : 0;
        s = pr | ((uint32_t)t->scale_index << 16);
        break; }
    case W2S_HACC:
        s = t->hacc_dm | ((uint32_t)(uint8_t)t->dest_index << 16);
        break;
    case W2S_NAV:
        // ★ 目的地の ID は以前「予備（0）」だった 16 ビット。0 は「送っていない」に取ってあるので、
        //   ID を送らない古い機体と混ざっても受信側は番号で合わせるだけ（LINK_W2_MAGIC は変えない）。
        s = t->nav_mode | ((uint32_t)t->dest_id << 8);
        break;
    }
    w2_put24(out + W2_SLOT, s);
    w2_put16(out + W2_CRC, link_crc16(out, W2_CRC));
}

// ============================================================
//  受信側
// ============================================================
// 形式 2 として正しいか（印と CRC）。
// ★ **グループ ID はここで見ない。** ここで弾くと「電波は来ているがグループが違う」状態を無信号と
//   区別できなくなり、SD が飛んで group が 0 に戻った事故に気づけなくなる。突き合わせは link.cpp
//   が別に行う（docs/pons_link.md §1）。
static inline bool link_w2_valid(const uint8_t* p) {
    return p[W2_MAGIC] == LINK_W2_MAGIC && w2_get16(p + W2_CRC) == link_crc16(p, W2_CRC);
}
static inline uint8_t link_w2_group(const uint8_t* p) { return p[W2_GROUP]; }
static inline uint8_t link_w2_slot(const uint8_t* p)  { return link_w2_slot_of(p[W2_SEQ]); }

// 受信側の組み立ての状態。回転枠で受けた値と、16 ビットの seq・時の繰り上がりを覚える。
struct LinkW2Rx {
    LinkTelem acc;          // 組み立て中の LinkTelem（最後に分かった値）
    uint16_t  slot_have;    // 回転枠で受けて分かっている have ビット（W2_SLOT_HAVE の部分集合）
    // 受けたことのある回転枠（1 << 枠の番号）。have ビットを持たない項目（縮尺・目的地・ナビモード・
    // 水平精度）は、受けるまで 0 のままなので、使う側がこれで確かめる（link_w2_slot_seen）。
    uint8_t   slot_seen;
    bool      yawacc_known; // ヨー精度（回転枠 1）を受けたか。受けるまでヨーは出さない
    bool      hour_known;
    uint8_t   hour;
    uint16_t  prev_tod;
    bool      seq_known;
    uint16_t  seq16;
    uint32_t  last_ms;      // 最後に受けた時刻（回転枠の値が古くなったかの判定）
};

// 回転枠の値を「送る間隔 × これ」より長く受けていなければ、分からなくなったとみなす。
// 受信機の「無信号」（ローワーで 10 秒）と同じ長さ。電波が戻った直後に古い風や電圧を出さないため。
#define LINK_W2_STALE_FRAMES  10

static inline void link_w2_rx_reset(LinkW2Rx* st) { memset(st, 0, sizeof(*st)); }

// その回転枠を（途切れてから）受けたか。have ビットの無い項目を使う前に見ること。
// ★ 受けるまで 0 のままの目的地・ナビモード・縮尺をそのまま使うと、受信機が
//   「送信機は目的地 0・縮尺 0」と思い込んで、自分の設定を書き換えてしまう（link_sync_*）。
static inline bool link_w2_slot_seen(const LinkW2Rx* st, uint8_t slot) {
    return (st->slot_seen & (1u << slot)) != 0;
}

// 1 フレームを組み立て状態へ反映し、st->acc を「いま分かっている LinkTelem」にする。
// per_ms = 送信機が送る間隔 [ms]（受信機での推定値。ローワーで約 1000）。
// ★ 呼ぶのは**自分と同じグループのフレームだけ**（別グループの値を混ぜない）。
static inline void link_w2_decode(const uint8_t* p, LinkW2Rx* st, uint32_t now_ms, uint32_t per_ms) {
    LinkTelem& t = st->acc;
    if (per_ms == 0) per_ms = 1000;
    const uint32_t gap_ms = now_ms - st->last_ms;
    if (st->seq_known && gap_ms > LINK_W2_STALE_FRAMES * per_ms) {
        // 長く途切れた後。回転枠で覚えていた値はもう信用しない（受け直すまで出さない）
        st->slot_have = 0; st->slot_seen = 0; st->yawacc_known = false;
        // ★ 時は 30 分より短い途切れなら、下の「その時の中の 0.1 秒が戻ったら繰り上げる」で追える
        //   （時をまたげば必ず 30 分以上戻る）ので覚えておく。忘れると回転枠 0 が来るまでの最大 8 発、
        //   受信ログの時刻が 0 時になる。それより長い途切れは分からないので捨てる。
        if (gap_ms >= 30UL * 60UL * 1000UL) st->hour_known = false;
    }
    st->last_ms = now_ms;

    // seq を 16 ビットに戻す。前回からの差が 128 以上なら「戻った」とみなす
    // （送信機の再起動や 2 台目。link.cpp の重複送信機の判定がそのまま働くように）。
    // ★ ただし途切れが 128 発ぶん以上続いたあとは、8 ビットでは何周したか分からない。
    //   経った時間から進んだはずの数を見積もり、それに一番近い「差 + 256 の倍数」を取る
    //   （落ちた数をローワーで 2 分以上の途切れでも数え続けるため。途切れ中の送信機の
    //   再起動は見分けられず、進んだように見える）。
    const uint8_t seq8 = p[W2_SEQ];
    if (!st->seq_known) { st->seq16 = seq8; st->seq_known = true; }
    else {
        const uint8_t  d      = (uint8_t)(seq8 - (uint8_t)st->seq16);
        const uint32_t expect = (gap_ms + per_ms / 2) / per_ms;
        if (expect >= 128) {
            const uint32_t k = (expect > d) ? (expect - d + 128) / 256 : 0;
            st->seq16 = (uint16_t)(st->seq16 + d + 256 * k);
        } else {
            st->seq16 = (d < 128) ? (uint16_t)(st->seq16 + d) : (uint16_t)(st->seq16 - (uint16_t)(256 - d));
        }
    }

    t.magic0 = 'P'; t.magic1 = 'L'; t.ver = LINK_PROTO_VER; t.type = LINK_TYPE_TELEM;
    t.seq    = st->seq16;
    t.group  = p[W2_GROUP];
    t.status = w2_get16(p + W2_STATUS);

    const uint8_t f = p[W2_FLAGS];
    t.fixflags = (f & W2F_FIXOK) ? 0x01 : 0x00;

    const uint16_t tod = w2_get16(p + W2_TOD);
    if ((f & W2F_TOD) && tod < 36000) {
        // 時は回転枠 0 から。その間に時刻が大きく戻ったら次の時へ繰り上げる
        if (st->hour_known && st->prev_tod != 0xFFFF && tod + 18000 < st->prev_tod)
            st->hour = (uint8_t)((st->hour + 1) % 24);
        st->prev_tod = tod;
        t.minute = (uint8_t)(tod / 600);
        t.second = (uint8_t)((tod / 10) % 60);
        t.centi  = (uint8_t)((tod % 10) * 10);
        if (st->hour_known) t.hour = st->hour;   // 分からない間は前の値のまま（受け始めは 0。日付も無い）
    } else {
        st->prev_tod = 0xFFFF;
    }

    t.lat_1e7 = (int32_t)w2_get32(p + W2_LAT);
    t.lon_1e7 = (int32_t)w2_get32(p + W2_LON);
    const uint32_t gt = w2_get24(p + W2_GSTRK);
    t.gs_cms     = (uint16_t)(((gt & 0xFFF) * 5) / 2);
    t.track_cdeg = (uint16_t)(((gt >> 12) & 0xFFF) * 10);
    t.kf_alt_dm  = (int16_t)w2_get16(p + W2_KFALT);
    t.kf_vs_cms  = (int16_t)((int8_t)p[W2_KFVS] * 5);
    uint64_t att = 0;
    for (int i = 0; i < 5; i++) att |= (uint64_t)p[W2_ATT + i] << (8 * i);
    t.roll_cdeg  = (int16_t)(w2_sext((uint32_t)(att & 0xFFF), 12) * 10);
    t.pitch_cdeg = (int16_t)(w2_sext((uint32_t)((att >> 12) & 0xFFF), 12) * 10);
    // ★ ヨーは ±180° に戻す。LinkTelem の yaw_cdeg は int16（0.01°）なので 327.67° までしか入らず、
    //   0〜359.9° のまま戻すと 350° が 35000 で溢れて化ける。
    int32_t y = (int32_t)((att >> 24) & 0xFFF);
    if (y > 1800) y -= 3600;
    t.yaw_cdeg   = (int16_t)(y * 10);

    // ---- 回転枠 ----
    const uint32_t s = w2_get24(p + W2_SLOT);
    st->slot_seen |= (uint8_t)(1u << link_w2_slot_of(seq8));
    switch (link_w2_slot_of(seq8)) {
    case W2S_DATE:
        if (s & (1u << 21)) {
            t.year  = (uint16_t)(2000 + (s & 0x7F));
            t.month = (uint8_t)((s >> 7) & 0xF);
            t.day   = (uint8_t)((s >> 11) & 0x1F);
            st->hour = (uint8_t)((s >> 16) & 0x1F); st->hour_known = true;
            t.hour  = st->hour;
            st->slot_have |= RHAVE_DATE;
        } else st->slot_have &= (uint16_t)~RHAVE_DATE;
        break;
    case W2S_PAVG:
        t.pitch_avg_cdeg = (int16_t)(w2_sext(s & 0xFFF, 12) * 10);
        t.yaw_acc95_deg  = (uint8_t)((s >> 12) & 0xFF);
        if (s & (1u << 20)) st->slot_have |= RHAVE_ATT_AVG; else st->slot_have &= (uint16_t)~RHAVE_ATT_AVG;
        st->yawacc_known = (s & (1u << 21)) != 0;
        break;
    case W2S_TRIMW:
        t.roll_trim_cdeg = (int16_t)w2_sext(s & 0xFFF, 12);
        t.wind_dmps      = (uint8_t)((s >> 12) & 0xFF);
        if (s & (1u << 20)) st->slot_have |= RHAVE_ATT_TRIM; else st->slot_have &= (uint16_t)~RHAVE_ATT_TRIM;
        if (s & (1u << 21)) st->slot_have |= RHAVE_ATT_WIND; else st->slot_have &= (uint16_t)~RHAVE_ATT_WIND;
        break;
    case W2S_WDIR:
        t.wind_dir_cdeg = (uint16_t)((s & 0xFFF) * 10);
        t.numsat        = (uint8_t)((s >> 12) & 0x3F);
        if (s & (1u << 18)) st->slot_have |= RHAVE_NUMSAT; else st->slot_have &= (uint16_t)~RHAVE_NUMSAT;
        if (!(s & (1u << 19))) st->slot_have &= (uint16_t)~RHAVE_ATT_WIND;   // 風が切れたら速度の枠を待たない
        break;
    case W2S_ALTV: {
        const uint16_t alt = (uint16_t)(s & 0xFFFF);
        const uint8_t  v   = (uint8_t)((s >> 16) & 0xFF);
        if (alt != 0x8000) { t.gnss_alt_dm = (int16_t)alt; st->slot_have |= RHAVE_GNSSALT; }
        else               st->slot_have &= (uint16_t)~RHAVE_GNSSALT;
        if (v) { t.volt_cv = v; st->slot_have |= RHAVE_VOLT; } else st->slot_have &= (uint16_t)~RHAVE_VOLT;
        break; }
    case W2S_PRESS: {
        const uint16_t pr = (uint16_t)(s & 0xFFFF);
        if (pr) { t.press_dpa = pr; st->slot_have |= RHAVE_PRESS; } else st->slot_have &= (uint16_t)~RHAVE_PRESS;
        t.scale_index = (uint8_t)((s >> 16) & 0xFF);
        break; }
    case W2S_HACC:
        t.hacc_dm    = (uint16_t)(s & 0xFFFF);
        t.dest_index = (int8_t)((s >> 16) & 0xFF);
        break;
    case W2S_NAV:
        t.nav_mode = (uint8_t)(s & 0xFF);
        t.dest_id  = (uint16_t)((s >> 8) & 0xFFFF);
        break;
    }

    // ---- have ビット: 核のフラグ ＋ 回転枠で分かっている分 ----
    uint16_t have = st->slot_have;
    if (f & W2F_GS)    have |= RHAVE_GS | RHAVE_TTRACK;
    if (f & W2F_KFALT) have |= RHAVE_KFALT;
    if (f & W2F_KFVS)  have |= RHAVE_KFVS;
    if (f & W2F_ATT)   have |= RHAVE_ATT;
    // ★ ヨーは精度（回転枠 1）が分かるまで出さない。精度 0 のまま出すと「とても確か」に見え、
    //   ヨーの信頼度で絞っている風や機首の表示がそのまま出てしまう。
    if ((f & W2F_YAW) && st->yawacc_known) have |= RHAVE_ATT_YAW;
    t.have = have;
}

#endif // LINK_WIRE2_H
