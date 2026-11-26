// ============================================================
// File    : tools/wire2_test/main.cpp
// Role    : lora_link/link_wire2.h・link_beacon.h・link_proto.h のホスト単体テスト。
//           1) 8 回転枠（slot 0〜7）のエンコード→デコード往復恒等性
//           2) 負角・飽和クランプ値が「無し」センチネル（0x800 / -512 / 0x8000 / -128）に化けないこと
//           3) NaN パック・23→0 時跨ぎ（BUG-72 途切れ明けを含む）・うるう年・seq16 復元
//           4) link_w2_utc_ms（BUG-68 GNSS_LOST 拒否を含む）と LinkBeacon の往復・印の識別
//           5) 電池の表示（LINK_BATT_*。回転枠 4 の 8 ビット。電池・充電中・SW OFF・分からない）
//           6) ヨーの全周（0〜360°）の往復。327.7° 以上が頭打ちにならず、いつも 0〜35999 に戻ること（link_pack_heading_cdeg）
// ============================================================
#include <cstdio>
#include <cmath>
#include <cstring>
#include <limits>
#include "link_wire2.h"
#include "link_beacon.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] line %d: " msg "\n", __LINE__, ##__VA_ARGS__); } \
} while (0)

static LinkTelem make_full_telem(uint16_t seq) {
    LinkTelem t;
    memset(&t, 0, sizeof(t));
    t.magic0 = 'P'; t.magic1 = 'L'; t.ver = LINK_PROTO_VER; t.type = LINK_TYPE_TELEM;
    t.seq = seq;
    t.group = 2; // Group C
    t.scale_index = 3;
    t.dest_id = 0xBEEF;
    t.year = 2026; t.month = 11; t.day = 14;
    t.hour = 12; t.minute = 34; t.second = 56; t.centi = 70;
    t.lat_1e7 = 352945678;
    t.lon_1e7 = 1362543210;
    t.gs_cms = 925;          // 9.25 m/s (0.025m/s 単位で割り切れる)
    t.track_cdeg = 27430;    // 274.3°
    t.gnss_alt_dm = 1234;    // 123.4 m
    t.kf_alt_dm = 1205;      // 120.5 m
    t.kf_vs_cms = -135;      // -1.35 m/s (0.05m/s 単位)
    t.press_dpa = 10132;     // 1013.2 hPa
    t.roll_cdeg = -1240;     // -12.4°
    t.pitch_cdeg = 350;      // +3.5°
    t.yaw_cdeg = 31440;      // 314.4°
    t.pitch_avg_cdeg = 210;  // +2.1°
    t.roll_trim_cdeg = -85;  // -0.85°
    t.yaw_acc95_deg = 4;
    t.wind_dmps = 38;        // 3.8 m/s
    t.wind_dir_cdeg = 19540; // 195.4°
    t.tas_dmps = 96;         // 9.6 m/s
    t.tas_flags = LINK_TAS_CLAMPED | LINK_TAS_HDR;
    t.batt = link_batt_on_battery(77);   // 電池で 77%
    t.numsat = 18;
    t.hacc_dm = 14;          // 1.4 m
    t.sacc_cms = 23;         // 0.23 m/s（0.5 未満は 0.01 刻みでそのまま戻る）
    t.fixflags = 0x01;       // gnssFixOK
    t.dest_index = 5;
    t.nav_mode = 2;
    t.status = LINK_ST_BAT_LOW | LINK_ST_TIME_WANTED;
    t.have = RHAVE_GS | RHAVE_TTRACK | RHAVE_GNSSALT | RHAVE_KFALT | RHAVE_KFVS |
             RHAVE_PRESS | RHAVE_DATE | RHAVE_NUMSAT |
             RHAVE_ATT | RHAVE_ATT_YAW | RHAVE_ATT_AVG | RHAVE_ATT_TRIM |
             RHAVE_ATT_WIND | RHAVE_TAS;
    return t;
}

static void test_roundtrip_8slots() {
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);

    for (uint16_t seq = 0; seq < 8; seq++) {
        LinkTelem tx = make_full_telem(seq);
        uint8_t wire[LINK_W2_LEN];
        link_w2_encode(&tx, wire);

        CHECK(link_w2_valid(wire), "slot %u valid", seq);
        CHECK(!link_bc_valid(wire), "slot %u is not beacon", seq);
        CHECK(link_w2_group(wire) == 2, "slot %u group", seq);
        CHECK(link_w2_slot(wire) == seq, "slot %u index", seq);

        link_w2_decode(wire, &rx, 1000 + seq * 1000, 1000);

        // 回転枠 2（W2S_TRIM）を受けるまでは RHAVE_ATT_YAW は立たない仕様
        if (seq < W2S_TRIM) {
            CHECK((rx.acc.have & RHAVE_ATT_YAW) == 0, "yaw gated before slot 2 (seq=%u)", seq);
        } else {
            CHECK((rx.acc.have & RHAVE_ATT_YAW) != 0, "yaw enabled from slot 2 (seq=%u)", seq);
        }
        CHECK(link_w2_slot_seen(&rx, (uint8_t)seq), "slot %u seen", seq);
    }

    // 8 枠すべて受けた後の完全一致確認
    const LinkTelem ref = make_full_telem(7);
    const LinkTelem& got = rx.acc;
    CHECK(got.have == ref.have, "have mask 0x%04X == 0x%04X", got.have, ref.have);
    CHECK(got.seq == 7, "seq=%u", got.seq);
    CHECK(got.group == ref.group, "group=%u", got.group);
    CHECK(got.status == ref.status, "status=0x%04X", got.status);
    CHECK(got.fixflags == ref.fixflags, "fixflags=0x%02X", got.fixflags);
    CHECK(got.year == ref.year && got.month == ref.month && got.day == ref.day, "date");
    CHECK(got.hour == ref.hour && got.minute == ref.minute && got.second == ref.second && got.centi == ref.centi, "time");
    CHECK(got.lat_1e7 == ref.lat_1e7 && got.lon_1e7 == ref.lon_1e7, "lat/lon");
    CHECK(got.gs_cms == ref.gs_cms && got.track_cdeg == ref.track_cdeg, "gs/trk");
    CHECK(got.kf_alt_dm == ref.kf_alt_dm && got.kf_vs_cms == ref.kf_vs_cms, "kf alt/vs");
    CHECK(got.roll_cdeg == ref.roll_cdeg && got.pitch_cdeg == ref.pitch_cdeg, "roll/pitch");
    CHECK(got.yaw_cdeg == ref.yaw_cdeg, "yaw got=%u ref=%u", got.yaw_cdeg, ref.yaw_cdeg);
    CHECK(got.pitch_avg_cdeg == ref.pitch_avg_cdeg, "pavg got=%d ref=%d", got.pitch_avg_cdeg, ref.pitch_avg_cdeg);
    CHECK(got.tas_dmps == ref.tas_dmps && got.tas_flags == ref.tas_flags, "tas");
    CHECK(got.wind_dmps == ref.wind_dmps && got.wind_dir_cdeg == ref.wind_dir_cdeg, "wind");
    CHECK(got.roll_trim_cdeg == ref.roll_trim_cdeg && got.yaw_acc95_deg == ref.yaw_acc95_deg, "trim/yawacc");
    CHECK(got.hacc_dm == ref.hacc_dm && got.numsat == ref.numsat && got.dest_index == ref.dest_index, "hacc/nsat/dest");
    CHECK(got.gnss_alt_dm == ref.gnss_alt_dm && got.batt == ref.batt, "gnss_alt/batt");
    CHECK(got.press_dpa == ref.press_dpa && got.scale_index == ref.scale_index, "press/scale");
    CHECK(got.nav_mode == ref.nav_mode && got.dest_id == ref.dest_id, "nav/dest_id");
    CHECK(got.sacc_cms == ref.sacc_cms, "sacc got=%u ref=%u", got.sacc_cms, ref.sacc_cms);
}

static void test_extremes_and_sentinels() {
    // 1) 全 have = 0 のとき、核も回転枠も have が立たないこと
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);
    for (uint8_t s = 0; s < 8; s++) {
        LinkTelem empty;
        memset(&empty, 0, sizeof(empty));
        empty.seq = s;
        uint8_t wire[LINK_W2_LEN];
        link_w2_encode(&empty, wire);
        link_w2_decode(wire, &rx, 1000 + s * 1000, 1000);
        CHECK(rx.acc.have == 0, "empty frame seq=%u have=0x%04X", s, rx.acc.have);
    }

    // 2) 極端な負値・正値（クランプ上限超え）を送っても「無し」センチネルに化けないこと
    LinkTelem ext = make_full_telem(W2S_TRIM); // slot 2 で yawacc_known を立てる
    ext.kf_alt_dm = -32768;       // INT16_MIN -> -32767 にクランプされ W2_ALT_NONE(0x8000) を避ける
    ext.kf_vs_cms = -1000;        // -10 m/s -> -127 (-6.35m/s) にクランプされ W2_VS_NONE(-128) を避ける
    ext.roll_cdeg = -30000;       // -300° -> -2047 (-204.7°) にクランプされ W2_ROLL_NONE(0x800) を避ける
    ext.pitch_cdeg = -30000;
    ext.pitch_avg_cdeg = -10000;  // -100° -> -511 (-51.1°) にクランプされ W2_PAVG_NONE(-512) を避ける
    ext.yaw_cdeg = 35990;         // 359.9°（int16 には入らない値）
    ext.tas_dmps = 0;             // have & RHAVE_TAS なら 0 でも 1 (0.1m/s) に持ち上げられる

    link_w2_rx_reset(&rx);
    uint8_t wire[LINK_W2_LEN];
    link_w2_encode(&ext, wire);
    link_w2_decode(wire, &rx, 1000, 1000);

    CHECK((rx.acc.have & RHAVE_KFALT) != 0 && rx.acc.kf_alt_dm == -32767, "kf_alt min clamp got=%d", rx.acc.kf_alt_dm);
    CHECK((rx.acc.have & RHAVE_KFVS) != 0 && rx.acc.kf_vs_cms == -635, "kf_vs min clamp got=%d", rx.acc.kf_vs_cms);
    CHECK((rx.acc.have & RHAVE_ATT) != 0 && rx.acc.roll_cdeg == -20470 && rx.acc.pitch_cdeg == -20470,
          "roll/pitch min clamp got=%d/%d", rx.acc.roll_cdeg, rx.acc.pitch_cdeg);
    CHECK((rx.acc.have & RHAVE_ATT_AVG) != 0 && rx.acc.pitch_avg_cdeg == -5110,
          "pavg min clamp got=%d", rx.acc.pitch_avg_cdeg);
    CHECK((rx.acc.have & RHAVE_ATT_YAW) != 0 && rx.acc.yaw_cdeg == 35990,
          "yaw 359.9 got=%u", rx.acc.yaw_cdeg);
    CHECK((rx.acc.have & RHAVE_TAS) != 0 && rx.acc.tas_dmps == 1, "tas zero promoted to 1");

    // 3) NaN パック安全性
    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    CHECK(link_pack_i16(nan_v, 100.0f) == 0, "pack_i16 NaN");
    CHECK(link_pack_u16(nan_v, 100.0f, 35999) == 0, "pack_u16 NaN");
    CHECK(link_pack_u8(nan_v, 10.0f) == 0, "pack_u8 NaN");

    // 4) 古い印 0xA2 / 0xA4 / 0xA8 や CRC 改ざんを弾くこと
    uint8_t bad[LINK_W2_LEN];
    memcpy(bad, wire, LINK_W2_LEN);
    bad[W2_MAGIC] = 0xA2;
    w2_put16(bad + W2_CRC, link_crc16(bad, W2_CRC));
    CHECK(!link_w2_valid(bad), "reject old 0xA2 magic");
    bad[W2_MAGIC] = 0xA4 | 0x02;   // 2026-11-20 までの並び（sAcc なし）
    w2_put16(bad + W2_CRC, link_crc16(bad, W2_CRC));
    CHECK(!link_w2_valid(bad), "reject old 0xA4 magic");
    bad[W2_MAGIC] = 0xA8 | 0x02;   // 2026-11-23 までの並び（回転枠 4 が電圧）
    w2_put16(bad + W2_CRC, link_crc16(bad, W2_CRC));
    CHECK(!link_w2_valid(bad), "reject old 0xA8 magic");
    memcpy(bad, wire, LINK_W2_LEN);
    bad[10] ^= 0x01;
    CHECK(!link_w2_valid(bad), "reject bad CRC");
}

static void test_midnight_and_stale() {
    // BUG-72 回帰テスト:
    // 枠 0 で 2028-02-28 23:59:55 UTC を受けたあと、15 秒途切れて（LINK_W2_STALE_FRAMES 超え・30 分未満）
    // 枠 1 で 00:00:10 UTC を受けたとき、日付の have ビットは落ちていても覚えている日付が 2028-02-29（うるう年）に進み、
    // さらに次の日跨ぎで 2028-03-01 に進むこと。
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);

    LinkTelem t0 = make_full_telem(W2S_DATE);
    t0.year = 2028; t0.month = 2; t0.day = 28;
    t0.hour = 23; t0.minute = 59; t0.second = 55; t0.centi = 0;
    uint8_t w[LINK_W2_LEN];
    link_w2_encode(&t0, w);
    link_w2_decode(w, &rx, 1000, 1000);
    CHECK(rx.acc.year == 2028 && rx.acc.month == 2 && rx.acc.day == 28 && rx.acc.hour == 23, "init date 2028-02-28 23h");

    // 15 秒後（stale だが 30 分未満）に seq=1 (W2S_WIND) で 00:00:10 が届く
    LinkTelem t1 = make_full_telem(W2S_WIND);
    t1.year = 2028; t1.month = 2; t1.day = 29;
    t1.hour = 0; t1.minute = 0; t1.second = 10; t1.centi = 0;
    link_w2_encode(&t1, w);
    link_w2_decode(w, &rx, 16000, 1000);
    CHECK(rx.acc.hour == 0 && rx.acc.year == 2028 && rx.acc.month == 2 && rx.acc.day == 29,
          "leap day rollover after stale gap: %04u-%02u-%02u %02u:%02u",
          rx.acc.year, rx.acc.month, rx.acc.day, rx.acc.hour, rx.acc.minute);

    // 16bit seq の周回と長い途切れ後の復元（seq=1 から 300 秒後に seq=45 が届く -> 1 + 300 = 301, 301 & 0xFF = 45）
    LinkTelem t2 = make_full_telem(301);
    link_w2_encode(&t2, w);
    link_w2_decode(w, &rx, 316000, 1000);
    CHECK(rx.seq16 == 301, "long gap seq16 expected 301, got %u", rx.seq16);
}

// sAcc（回転枠 7 の 6 ビット。2026-11-20〜）
static void test_sacc() {
    // 段の表: 0.5 未満は 0.01 刻み、0.5〜3.5 は 0.25 刻み、3.5 以上は頭打ち、無しは 0
    struct { uint16_t in, out; } tbl[] = {
        {0, 0}, {1, 1}, {29, 29}, {30, 30}, {49, 49}, {50, 50}, {62, 50}, {63, 75},
        {100, 100}, {299, 300}, {300, 300}, {349, 350}, {350, 350}, {1000, 350}, {65534, 350},
        {LINK_SACC_NONE, LINK_SACC_NONE},
    };
    for (auto& e : tbl) {
        const uint8_t c = link_w2_sacc_code(e.in);
        CHECK(c <= 63, "sacc code %u fits 6 bits (%u)", e.in, c);
        const uint16_t o = link_w2_sacc_cms(c);
        CHECK(o == e.out, "sacc %u -> code %u -> %u (expect %u)", e.in, c, o, e.out);
    }
    CHECK(link_w2_sacc_code(LINK_SACC_NONE) == W2_SACC_NONE, "sacc none -> code 0");

    // 受けるまでは無し。枠 7 で届き、ナビモードと目的地の ID を壊さない
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);
    CHECK(rx.acc.sacc_cms == LINK_SACC_NONE, "sacc none before slot 7");
    LinkTelem t = make_full_telem(W2S_NAV);
    t.sacc_cms = 180;   // 1.80 -> 1.75
    uint8_t w[LINK_W2_LEN];
    link_w2_encode(&t, w);
    link_w2_decode(w, &rx, 1000, 1000);
    CHECK(rx.acc.sacc_cms == 175, "sacc 1.80 -> 1.75 got=%u", rx.acc.sacc_cms);
    CHECK(rx.acc.nav_mode == 2 && rx.acc.dest_id == 0xBEEF, "nav/dest_id intact with sacc");
    for (uint8_t m = 0; m <= 2; m++) {
        t.nav_mode = m; t.sacc_cms = 3500;   // 上の段いっぱいでもナビモードに漏れない
        link_w2_encode(&t, w);
        link_w2_decode(w, &rx, 2000, 1000);
        CHECK(rx.acc.nav_mode == m && rx.acc.sacc_cms == 350, "nav_mode %u with sacc max", m);
    }

    // 送信機の GNSS が黙っている（無し）→ 無しで届く
    t.sacc_cms = LINK_SACC_NONE;
    link_w2_encode(&t, w);
    link_w2_decode(w, &rx, 3000, 1000);
    CHECK(rx.acc.sacc_cms == LINK_SACC_NONE, "sacc none on wire");

    // 長い途切れのあとは、枠 7 を受け直すまで無し（古い値を出さない）
    t.sacc_cms = 40;
    link_w2_encode(&t, w);
    link_w2_decode(w, &rx, 4000, 1000);
    CHECK(rx.acc.sacc_cms == 40, "sacc 0.40 back");
    LinkTelem t2 = make_full_telem(8 + W2S_DATE);
    link_w2_encode(&t2, w);
    link_w2_decode(w, &rx, 4000 + (LINK_W2_STALE_FRAMES + 1) * 1000, 1000);
    CHECK(rx.acc.sacc_cms == LINK_SACC_NONE, "sacc none after stale gap");
}

// 電池の表示（回転枠 4 の 8 ビット。2026-11-23〜、印 0xAC）。送信機のヘッダーの電池の欄そのもの
static void test_battery() {
    // 値の表: 電池 1〜101、充電中 102〜202、SW OFF 255、0 と 203〜254 は分からない。範囲の外の % は端に丸める
    CHECK(link_batt_on_battery(-5) == 1 && link_batt_on_battery(0) == 1 && link_batt_on_battery(87) == 88 &&
          link_batt_on_battery(100) == 101 && link_batt_on_battery(150) == 101, "on_battery codes");
    CHECK(link_batt_charging(-1) == 102 && link_batt_charging(0) == 102 && link_batt_charging(87) == 189 &&
          link_batt_charging(100) == 202 && link_batt_charging(999) == 202, "charging codes");
    for (int c = 0; c <= 255; c++) {
        int pct = -7;
        const LinkBattKind k = link_batt_decode((uint8_t)c, pct);
        if (c >= 1 && c <= 101)        CHECK(k == LINK_BATT_K_BATTERY  && pct == c - 1,   "code %d battery %d", c, pct);
        else if (c >= 102 && c <= 202) CHECK(k == LINK_BATT_K_CHARGING && pct == c - 102, "code %d charging %d", c, pct);
        else if (c == 255)             CHECK(k == LINK_BATT_K_SW_OFF   && pct == -7,      "code 255 sw off");
        else                           CHECK(k == LINK_BATT_K_UNKNOWN  && pct == -7,      "code %d unknown", c);
    }

    // 受けるまでは分からない
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);
    int pct = -1;
    CHECK(link_batt_decode(rx.acc.batt, pct) == LINK_BATT_K_UNKNOWN, "unknown before slot 4");

    // 電池・充電中の 0〜100% と SW OFF が回転枠 4 でそのまま戻る。GNSS 高度は壊れない
    uint8_t w[LINK_W2_LEN];
    for (int kind = 0; kind < 2; kind++) {
        for (int p = 0; p <= 100; p++) {
            LinkTelem t = make_full_telem(W2S_ALTV);
            t.batt = kind ? link_batt_charging(p) : link_batt_on_battery(p);
            link_w2_encode(&t, w);
            link_w2_decode(w, &rx, 1000 + kind * 200 + p, 1000);
            pct = -1;
            const LinkBattKind k = link_batt_decode(rx.acc.batt, pct);
            CHECK(k == (kind ? LINK_BATT_K_CHARGING : LINK_BATT_K_BATTERY) && pct == p, "kind %d %d%% got %d %d", kind, p, (int)k, pct);
            CHECK(rx.acc.gnss_alt_dm == t.gnss_alt_dm && (rx.acc.have & RHAVE_GNSSALT), "gnss_alt intact (%d %d%%)", kind, p);
            CHECK((rx.acc.have & RHAVE_VOLT) == 0, "link never sets RHAVE_VOLT");
        }
    }
    {
        LinkTelem t = make_full_telem(W2S_ALTV);
        t.batt = LINK_BATT_SW_OFF;
        link_w2_encode(&t, w);
        link_w2_decode(w, &rx, 2000, 1000);
        CHECK(link_batt_decode(rx.acc.batt, pct) == LINK_BATT_K_SW_OFF, "sw off roundtrip");
    }

    // 別の回転枠では書き換えない（届くのは回転枠 4 だけ）
    {
        LinkTelem t = make_full_telem(8 + W2S_ALTV);
        t.batt = link_batt_charging(64);
        link_w2_encode(&t, w);
        link_w2_decode(w, &rx, 3000, 1000);
        LinkTelem t2 = make_full_telem(8 + W2S_PRESS);
        t2.batt = link_batt_on_battery(3);
        link_w2_encode(&t2, w);
        link_w2_decode(w, &rx, 4000, 1000);
        pct = -1;
        CHECK(link_batt_decode(rx.acc.batt, pct) == LINK_BATT_K_CHARGING && pct == 64, "kept on other slot got %d", pct);
    }

    // 長い途切れのあとは、回転枠 4 を受け直すまで分からない（古い値を出さない・受信ログに書かない）
    {
        LinkTelem t2 = make_full_telem(16 + W2S_DATE);
        link_w2_encode(&t2, w);
        link_w2_decode(w, &rx, 4000 + (LINK_W2_STALE_FRAMES + 1) * 1000, 1000);
        CHECK(rx.acc.batt == LINK_BATT_NONE, "unknown after stale gap");
    }
}

static void test_utc_ms_and_beacon() {
    LinkW2Rx rx;
    link_w2_rx_reset(&rx);

    LinkTelem tx = make_full_telem(W2S_DATE);
    tx.year = 2026; tx.month = 11; tx.day = 14;
    tx.hour = 3; tx.minute = 12; tx.second = 45; tx.centi = 60;
    tx.status = 0;
    uint8_t w[LINK_W2_LEN];
    link_w2_encode(&tx, w);
    link_w2_decode(w, &rx, 1000, 1000);

    uint64_t utc = 0;
    CHECK(link_w2_utc_ms(w, &rx, &utc) == LINK_W2_TIME_FINE, "utc_ms FINE");
    // 2026-11-14 03:12:45.600 UTC
    const int64_t days = w2_days_from_civil(2026, 11, 14);
    const uint64_t expect_ms = (uint64_t)((days * 86400LL + 3 * 3600LL + 12 * 60LL + 45LL) * 1000LL + 600LL);
    CHECK(utc == expect_ms, "utc_ms value match");

    // BUG-68: LINK_ST_GNSS_LOST または LINK_ST_TIME_BAD が立っていたら使わない
    tx.status = LINK_ST_GNSS_LOST;
    link_w2_encode(&tx, w);
    link_w2_decode(w, &rx, 2000, 1000);
    CHECK(link_w2_utc_ms(w, &rx, &utc) == LINK_W2_TIME_NONE, "utc_ms rejected on GNSS_LOST");

    tx.status = LINK_ST_TIME_COARSE;
    link_w2_encode(&tx, w);
    link_w2_decode(w, &rx, 3000, 1000);
    CHECK(link_w2_utc_ms(w, &rx, &utc) == LINK_W2_TIME_COARSE, "utc_ms COARSE");

    // LinkBeacon 往復テスト
    LinkBeacon bc_in;
    memset(&bc_in, 0, sizeof(bc_in));
    bc_in.group = 1;
    bc_in.seq = 42;
    bc_in.kind = LINK_BC_OFFER;
    bc_in.has_time = true;
    bc_in.utc_ms = expect_ms + 78;
    bc_in.sender = 0x1234;
    bc_in.numsat = 15;
    bc_in.has_pos = true;
    bc_in.lat_1e7 = 352900000;
    bc_in.lon_1e7 = 1362500000;
    bc_in.alt_m = 85;
    bc_in.tacc_ms = 25;

    uint8_t bw[LINK_BC_LEN];
    link_bc_encode(&bc_in, bw);
    CHECK(link_bc_valid(bw), "beacon valid");
    CHECK(!link_w2_valid(bw), "beacon is not wire2 telem");
    CHECK(link_frame_valid(bw) && link_frame_valid(w), "link_frame_valid accepts both");

    LinkBeacon bc_out;
    CHECK(link_bc_decode(bw, &bc_out), "beacon decode");
    CHECK(bc_out.group == 1 && bc_out.seq == 42 && bc_out.kind == LINK_BC_OFFER, "beacon header");
    CHECK(bc_out.has_time && bc_out.utc_ms == bc_in.utc_ms && bc_out.tacc_ms == 25, "beacon time");
    CHECK(bc_out.has_pos && bc_out.lat_1e7 == bc_in.lat_1e7 && bc_out.lon_1e7 == bc_in.lon_1e7 && bc_out.alt_m == 85, "beacon pos");
}

// 6) ヨーを 0〜360° の全周で「送信機の値 → 詰める → 電波 → 戻す」と通し、どこでも元の向きに戻ること、
//    戻した値がいつも 0〜35999（負の数・360.00 にならない）であること。
//    2026-11-26 まで yaw_cdeg は int16 で、送信機が 0〜360° のまま詰めていて 327.7〜359.9° が 32767 に頭打ちになり、
//    受信機のヨーが「328」で止まっていた（link_pack_heading_cdeg。LINK_PROTO_VER 9 で uint16 の 0〜35999 にした）。
static void test_yaw_full_circle() {
    int bad = 0;
    for (int i = 0; i < 36000; i += 7) {          // 0.07° 刻み（0.1° の格子からずらして丸めの境目も通す）
        const float deg = i * 0.01f;
        LinkTelem t = make_full_telem(W2S_TRIM);
        t.yaw_cdeg = link_pack_heading_cdeg(deg);   // link.cpp の link_push_telemetry() と同じ
        LinkW2Rx rx;
        link_w2_rx_reset(&rx);
        uint8_t wire[LINK_W2_LEN];
        link_w2_encode(&t, wire);
        link_w2_decode(wire, &rx, 1000, 1000);
        const float got = rx.acc.yaw_cdeg * 0.01f;   // link_get_yaw()
        float d = fabsf(got - deg);
        if (d > 180.0f) d = 360.0f - d;
        if ((rx.acc.have & RHAVE_ATT_YAW) == 0 || rx.acc.yaw_cdeg >= 36000 || d > 0.051f) {
            if (bad++ < 5) printf("  yaw %.2f -> packed %u -> got %.2f\n", deg, t.yaw_cdeg, got);
        }
    }
    CHECK(bad == 0, "yaw full circle: %d angles did not come back", bad);
    CHECK(link_pack_heading_cdeg(340.0f) == 34000 && link_pack_heading_cdeg(180.0f) == 18000 &&
          link_pack_heading_cdeg(-20.0f) == 34000 && link_pack_heading_cdeg(720.5f) == 50 &&
          link_pack_heading_cdeg(359.996f) == 0 && link_pack_heading_cdeg(360.0f) == 0,
          "pack heading 340/180/-20/720.5/359.996/360");
    // 電波の 12 ビットの 3600〜4094（送る側は出さない）は「無し」に戻すこと
    {
        LinkTelem t = make_full_telem(W2S_TRIM);
        uint8_t wire[LINK_W2_LEN];
        link_w2_encode(&t, wire);
        uint64_t att = 0;
        for (int i = 0; i < 6; i++) att |= (uint64_t)wire[W2_ATT + i] << (8 * i);
        att = (att & ~((uint64_t)0xFFF << 24)) | ((uint64_t)3600 << 24);
        for (int i = 0; i < 6; i++) wire[W2_ATT + i] = (uint8_t)(att >> (8 * i));
        LinkW2Rx rx;
        link_w2_rx_reset(&rx);
        link_w2_decode(wire, &rx, 1000, 1000);
        CHECK((rx.acc.have & RHAVE_ATT_YAW) == 0, "yaw 3600 on air -> none");
    }
    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    const float inf_v = std::numeric_limits<float>::infinity();
    CHECK(link_pack_heading_cdeg(nan_v) == 0 && link_pack_heading_cdeg(inf_v) == 0 &&
          link_pack_heading_cdeg(-inf_v) == 0, "pack heading NaN/inf");
}

int main() {
    test_roundtrip_8slots();
    test_extremes_and_sentinels();
    test_midnight_and_stale();
    test_sacc();
    test_battery();
    test_utc_ms_and_beacon();
    test_yaw_full_circle();

    printf("wire2_test: %d passed, %d failed\n", g_pass, g_fail);
    printf("結果: %s\n", g_fail == 0 ? "PASS — Wire2 / Beacon 全テスト合格" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
