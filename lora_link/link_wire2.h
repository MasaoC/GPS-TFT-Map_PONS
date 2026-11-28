// ============================================================
// File    : link_wire2.h
// Project : PONS v7 — PONS Link
// Role    : 電波に載せる 32 バイトのフレーム。LinkTelem（69 バイト）を詰めて送り、
//           受けたら LinkTelem に戻す。画面・ログ・リプレイは LinkTelem のまま何も変えない。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/26
// ============================================================
//
// ★ **2026-10-27 から、電波に載せるのはどの CH でもこの 32 バイトだけ**（ローワーもアッパーも）。
//   形式を見分ける仕掛けは印だけ。**並びの違う機体とは通じない**ので、3 台ともそろえる。
//
// ★ なぜ 32 バイトにするのか（docs/pons_link.md §2「電波に載せる 32 バイトのフレーム」）
//   ・送信時間が 65 バイトの約 6 割になる。送信電流もそのぶん減り、同じ送信時間なら SF を 1 段上げられる
//     （約 2.5dB 遠くまで届く）。
//   ・アッパーバンド（CH13〜）は 1 パケットが 32 バイトに固定される。65 バイトだと 3 つに分割され、
//     休止（送信時間の 10 倍）も 3 回入って 1Hz で送れない。32 バイトなら 1 パケットに収まる。
//
// ★ 作り
//   ・毎回送る「核」（位置・速度・方位・高度・昇降・姿勢・平均ピッチ・平均迎角・推定 TAS・状態）と、4 バイトの「回転枠」。
//     回転枠の中身は seq を LINK_W2_SLOTS で割った余りで決まり、ゆっくりした項目
//     （日付・風・電池の表示・気圧・目的地など）を順番に運ぶ（8 発で一周。風は 2 回載る）。
//   ・受信側は回転枠の値を覚えておき、LinkTelem を組み立て直す（LinkW2Rx）。
//     回転枠の項目は、その枠を受けるまで have ビットを立てない（分からない値を出さない）。
//     have ビットの無い項目（目的地・目的地の ID・ナビモード・縮尺・水平精度）は link_w2_slot_seen() で確かめる。
//     sAcc も have ビットを持たないが、受けるまでと途切れたあとは値が LINK_SACC_NONE になっている。
//     電池の表示（回転枠 4）も同じで、受けるまでと途切れたあとは LINK_BATT_NONE（分からない）になっている。
//   ・核の「無い」は**その項目の使わない値**で表す（フラグのバイトは持たない）。
//     時刻 0xFFFF / 真方位 0xFFF（対地速度ごと無し）/ KF 高度 0x3FFF / 昇降 -128 / ロール 0x800（ロール・ピッチとも無し）/
//     ヨー 0xFFF / 平均迎角 -512 / 推定 TAS 0。送る側は値をその手前で頭打ちにする。
//
// ★ 細かさは表示より細かく取ってある（位置は 1e-6 度。姿勢・平均迎角 0.1°、速度 0.025m/s、
//   昇降 0.05m/s、時刻 0.1 秒、推定 TAS 0.1m/s）。値の範囲を超えたら端に丸める
//   （昇降 ±6.35m/s、速度 102m/s、平均迎角 ±51.1°、推定 TAS 25.5m/s まで）。
//
// ★ 形を変えたら LINK_W2_MAGIC を変えること（古い機体が新しい形を誤読しないように）。
//   **印は上位 6 ビットで見る**（下位 2 ビットはグループ）ので、変えるときは 4 つ飛ばし（0xA4 → 0xA8 → 0xAC …）。
//
// ★ 並びの履歴
//   0xA2（2026-10-27〜11-07）: 印 1 バイト・グループ 1 バイト・フラグ 1 バイト・姿勢 5 バイト。
//     平均ピッチと風は回転枠で 8 発に 1 回。
//   0xA4（2026-11-07〜）: グループを印の下位 2 ビットへ（A〜D）、フラグを「無し」の値に置き換えて
//     空いた 2 バイトで、推定 TAS（核）と平均ピッチ（姿勢を 6 バイトに）を毎回送る。風は 4 発に 1 回。
//     docs/pons_tas.md §5.2。
//   0xA8（2026-11-20〜）: 回転枠 7 のナビモードを 8 → 2 ビットに縮め、空いた 6 ビットで速度精度 sAcc を送る
//     （受信機の地図と GNSS 詳細画面の sAcc を機体の値で出すため。それまでは「--」）。8 発に 1 回。
//   0xAC（2026-11-23〜）: 回転枠 4 の 8 ビットを電池の電圧から「電池の表示」（link_proto.h の LINK_BATT_*。送信機のヘッダーの
//     87% / CHG 87% / FULL / SW OFF そのもの）に替えた。状態の LINK_ST_USB_SW_OFF（0x0200）はそこへ移して空けた。
//     送信機が USB で充電中に、受信機が持ち上がった電圧を % にして s98（送信機は CHG 87%）と出ていたのと、
//     電池で動いているときも 10mV 刻みの電圧では 4 つに 1 つの % が出せず 1% ずれていたため。
//   0xB0: 核に平均迎角を載せ、平均ピッチを枠 6 へ移した。
//   0xB4: 緯度・経度を 1e-6 度に丸め、座標と平均ピッチを核の同じ 8 バイトに詰める。
//   0xB8: KF 高度と状態を核の 3 バイトに詰め、回転枠を 4 バイトに拡張。遅い状態とヘッダー設定は枠 6 へ。
//   0xBC: 状態 3 項目とヘッダー設定を枠 2 にも載せ、4 発に 1 回にする。
//   0xC0: ヨー精度も枠 2 と 6 に載せ、4 発に 1 回にする。
//   0xC4: 風の枠 1・5 の上位 8 ビットで機体側の設定確認結果を送る。
//   受信ログ（received/）は LinkTelem から書く。末尾近くの slot 列で、どの回転枠が入っていたか
//   （＝その行で更新されたゆっくりした項目）が分かる。
//
#ifndef LINK_WIRE2_H
#define LINK_WIRE2_H

#include <stdint.h>
#include <string.h>
#include "link_proto.h"

#define LINK_W2_LEN        32
#define LINK_W2_MAGIC      0xC4   // 印（上位 6 ビット）。下位 2 ビットはグループ
#define LINK_W2_MAGIC_MASK 0xFC
#define LINK_W2_SLOTS      8      // 回転枠の数。**256 の約数にすること**（seq の 8 ビットが一周しても続くように）
static_assert((LINK_W2_MAGIC & ~LINK_W2_MAGIC_MASK) == 0, "印の下位 2 ビットはグループ用に空けておく");
// 古い並びの印（0xA2）と、65 バイト時代の先頭（'P'）を誤読しないこと
static_assert((0xA2 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "古い並び 0xA2 と印が区別できない");
static_assert((0xA4 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "古い並び 0xA4 と印が区別できない");
static_assert((0xA8 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "古い並び 0xA8 と印が区別できない");
static_assert((0xAC & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "平均ピッチを核に持つ旧版と区別できない");
static_assert((0xB0 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "平均ピッチを枠 6 だけに持つ旧版と区別できない");
static_assert((0xB4 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "回転枠が 3 バイトの旧版と区別できない");
static_assert((0xB8 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "状態とヘッダー設定を枠 6 だけに持つ旧版と区別できない");
static_assert((0xBC & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "ヨー精度を枠 2 だけに持つ旧版と区別できない");
static_assert((0x50 & LINK_W2_MAGIC_MASK) != LINK_W2_MAGIC, "65 バイト時代の先頭と印が区別できない");

// ---- 位置（バイト） ----
#define W2_MAGIC   0    // 1B: 印 6 ビット | グループ 2 ビット（下位）
#define W2_SEQ     1
#define W2_STATUS_ALT 2  // 3B: 状態 10bit | KF 高度 14bit（0.1m、-300m から。0x3FFF = 無し）
#define W2_TAS     5    // 1B: 推定 TAS 0.1m/s（1..255）。0 = 無し
#define W2_TOD     6    // 2B: その時の中の 0.1 秒（0..35999）。0xFFFF = 時刻なし
#define W2_POS     8    // 8B: 緯度 28bit | 経度 29bit（1e-6 度）| 平均ピッチ 7bit（0.25°）
#define W2_GSTRK   16   // 3B: 対地速度 12bit（0.025m/s）| 真方位 12bit（0.1°。0xFFF = 両方無し）
#define W2_KFVS    19   // 1B: 0.05m/s（符号付き）。-128 = 無し
#define W2_ATT     20   // 6B: ロール 12bit | ピッチ 12bit（0.1°、符号付き。ロール 0x800 = 両方無し）|
                        //     ヨー 12bit（0.1°、0..3599。0xFFF = 無し）| 平均迎角 10bit（0.1°、符号付き。-512 = 無し）|
                        //     推定 TAS が上下限に張り付いている 1bit | 未使用 1bit
#define W2_SLOT    26   // 4B: 回転枠
#define W2_CRC     30   // 2B: 0..29 の CRC16-CCITT
static_assert(W2_SLOT + 4 == W2_CRC && W2_CRC + 2 == LINK_W2_LEN, "並びが 32 バイトに合っていない");

// 無線と LinkTelem でビット位置が異なる。LinkTelem の既存マスクは変更しない。
#define W2_ST_FIXOK   0x0200  // 無線の bit 9。LinkTelem では fixflags の bit 0
static constexpr uint16_t W2_ST_GROUND_FAULTS = LINK_ST_GROUND_ROLL | LINK_ST_GROUND_PITCH |
    LINK_ST_CALIB_GND | LINK_ST_GNSS_LOST | LINK_ST_PWR_FAULT;
static constexpr uint16_t W2_ST_TIME = LINK_ST_TIME_WANTED | LINK_ST_TIME_COARSE | LINK_ST_TIME_BAD;
static constexpr uint16_t W2_ST_SLOW = LINK_ST_SD_ERROR | LINK_ST_BAT_LOW | LINK_ST_NEEDS_APPLY;
static_assert(LINK_ST_IMU_ERROR == 0x0002 && W2_ST_GROUND_FAULTS == 0x01F0 && W2_ST_TIME == 0x1C00 &&
              (LINK_ST_IMU_ERROR | W2_ST_GROUND_FAULTS | W2_ST_TIME | W2_ST_SLOW) == LINK_ST_ALL,
              "LinkTelem の状態ビットと無線の詰め方を合わせる");

// 「無し」の値
#define W2_TRK_NONE   0xFFF
#define W2_ROLL_NONE  0x800
#define W2_YAW_NONE   0xFFF
#define W2_ALPHA_NONE (-512)
#define W2_PAVG_NONE  (-64)
#define W2_KFALT_NONE 0x3FFF
#define W2_KFALT_MIN_DM (-3000)
#define W2_KFALT_MAX_DM (W2_KFALT_MIN_DM + W2_KFALT_NONE - 1)
#define W2_VS_NONE    (-128)
#define W2_NUMSAT_NONE 63

// 回転枠の番号
#define W2S_DATE     0   // 年(7) 月(4) 日(5) 時(5) 有効(1)
#define W2S_WIND     1   // 風速 8bit(0.1m/s) | 風向 12bit(0.1°) | 風有効(1)
#define W2S_TRIM     2   // ロールトリム 12bit(0.01°) | ヨー精度 8bit | ヨー精度有効(1) | トリム有効(1) | 状態・ヘッダー 4bit
#define W2S_HACC     3   // 水平精度 10bit(0.1m。1023 で頭打ち) | 衛星数 6bit(63 = 無し) | 目的地 8bit(符号付き)
#define W2S_ALTV     4   // GNSS 高度 16bit(0.1m。INT16_MIN = 無し) | 電池の表示 8bit(LINK_BATT_*。0 = 分からない)
#define W2S_WIND2    5   // 風（W2S_WIND と同じ。1 周に 2 回載せて 4 発に 1 回にする）
#define W2S_PRESS    6   // 気圧 14bit | 地図縮尺 3bit | 状態・ヘッダー 4bit | ヨー精度 8bit | ヨー精度有効(1) | 未使用 2bit
#define W2S_NAV      7   // ナビモード 2bit | sAcc 6bit（link_w2_sacc_code。0 = 無し）| 目的地の ID 16bit（0 = 送っていない）

// 回転枠で運ぶ have ビット（受けるまで立てない）
#define W2_SLOT_HAVE (RHAVE_DATE | RHAVE_ATT_TRIM | RHAVE_ATT_WIND | \
                      RHAVE_NUMSAT | RHAVE_GNSSALT | RHAVE_PRESS)

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

// 速度精度 sAcc の 6 ビットの段（回転枠 7）。0 = 無し（古い並びの予備ビットと同じく 0 を「無し」にしておく）。
//   1〜50  : 0.00〜0.49 m/s（0.01 刻み）
//   51〜63 : 0.50〜3.50 m/s（0.25 刻み。63 は「3.50 以上」）
// ★ 画面の色の境目（0.3 / 3.0 m/s。display_tft.cpp）はどちらも段の上にある。細かさが要るのは 0.5 未満だけ。
#define W2_SACC_NONE  0
static inline uint8_t link_w2_sacc_code(uint16_t cms) {
    if (cms == LINK_SACC_NONE) return W2_SACC_NONE;
    if (cms < 50) return (uint8_t)(1 + cms);
    const int32_t c = 51 + w2_divr((int32_t)cms - 50, 25);
    return (uint8_t)(c > 63 ? 63 : c);
}
static inline uint16_t link_w2_sacc_cms(uint8_t code) {
    code &= 0x3F;
    if (code == W2_SACC_NONE) return LINK_SACC_NONE;
    if (code <= 50) return (uint16_t)(code - 1);
    return (uint16_t)(50 + (code - 51) * 25);
}

static inline uint8_t link_w2_slot_of(uint8_t seq8) { return (uint8_t)(seq8 % LINK_W2_SLOTS); }
static inline uint8_t w2_slow_shift(uint8_t slot) {
    // 同じ 4 ビットを 4 発離れた 2 枠で送る。0 はこの枠に無いという意味。
    return slot == W2S_TRIM ? 22 : slot == W2S_PRESS ? 17 : 0;
}
// 枠 2 と 6 で共通の 9 ビット。下位 8 ビットは精度、上位 1 ビットは有効。
static inline uint8_t w2_yaw_acc_shift(uint8_t slot) { return slot == W2S_TRIM ? 12 : 21; }
// その 1 バイトが印か（グループの 2 ビットを落として比べる）。
// ★ **バイトごと == で比べないこと。** 別グループの電波が「無信号」に見えてしまう（下の link_w2_valid）。
static inline bool link_w2_is_magic(uint8_t b) { return (b & LINK_W2_MAGIC_MASK) == LINK_W2_MAGIC; }

// ============================================================
//  送信側: LinkTelem → 32 バイト
// ============================================================
// 風の枠の上位バイトには送信機の設定チェックを載せる。0 は未判定、bit 7 が有効。
static inline void link_w2_encode(const LinkTelem* t, uint8_t* out, uint8_t settings_check = 0) {
    memset(out, 0, LINK_W2_LEN);
    const uint8_t seq8 = (uint8_t)t->seq;
    out[W2_MAGIC] = (uint8_t)(LINK_W2_MAGIC | (t->group & 0x03));
    out[W2_SEQ]   = seq8;
    uint16_t st = (uint16_t)(((t->status & LINK_ST_IMU_ERROR) >> 1) |
                             ((t->status & W2_ST_GROUND_FAULTS) >> 3) |
                             ((t->status & W2_ST_TIME) >> 4));
    if (t->fixflags & 0x01) st |= W2_ST_FIXOK;
    const uint16_t alt = (t->have & RHAVE_KFALT) && t->kf_alt_dm >= W2_KFALT_MIN_DM && t->kf_alt_dm <= W2_KFALT_MAX_DM
        ? (uint16_t)(t->kf_alt_dm - W2_KFALT_MIN_DM) : W2_KFALT_NONE;
    w2_put24(out + W2_STATUS_ALT, st | ((uint32_t)alt << 10));

    // 推定 TAS。0 は「無し」なので、有るときは 0.1m/s 以上に上げる
    out[W2_TAS] = (t->have & RHAVE_TAS) ? (uint8_t)(t->tas_dmps ? t->tas_dmps : 1) : 0;

    // 時刻は年か時分秒のどれかが入っていれば有効とみなす（LinkTelem は時刻だけの have ビットを持たない）
    const bool tod_ok = (t->hour < 24 && t->minute < 60 && t->second < 60) &&
                        (t->year != 0 || t->hour || t->minute || t->second);
    const uint16_t tod = tod_ok
        ? (uint16_t)((t->minute * 60 + t->second) * 10 + t->centi / 10) : 0xFFFF;
    w2_put16(out + W2_TOD, tod);

    const int32_t lat = w2_divr(w2_clampi(t->lat_1e7, -900000000, 900000000), 10);
    const int32_t lon = w2_divr(w2_clampi(t->lon_1e7, -1800000000, 1800000000), 10);
    const int32_t avg = w2_divr(t->pitch_avg_cdeg, 25);
    const int32_t pa = (t->have & RHAVE_ATT_AVG) && avg >= -63 && avg <= 63 ? avg : W2_PAVG_NONE;
    const uint64_t pos = ((uint32_t)lat & 0x0FFFFFFF)
                       | ((uint64_t)((uint32_t)lon & 0x1FFFFFFF) << 28)
                       | ((uint64_t)((uint32_t)pa & 0x7F) << 57);
    for (int i = 0; i < 8; i++) out[W2_POS + i] = (uint8_t)(pos >> (8 * i));

    const bool gs_ok = (t->have & (RHAVE_GS | RHAVE_TTRACK)) == (RHAVE_GS | RHAVE_TTRACK);
    const uint32_t gs12  = gs_ok ? (uint32_t)w2_clampi(w2_divr(t->gs_cms * 2, 5), 0, 4095) : 0;   // 0.01 → 0.025 m/s
    const uint32_t trk12 = gs_ok ? (uint32_t)(w2_divr(t->track_cdeg, 10) % 3600) : W2_TRK_NONE;  // 0.01 → 0.1°
    w2_put24(out + W2_GSTRK, gs12 | (trk12 << 12));

    out[W2_KFVS] = (uint8_t)(int8_t)((t->have & RHAVE_KFVS)
             ? w2_clampi(w2_divr(t->kf_vs_cms, 5), -127, 127) : W2_VS_NONE);   // 0.01 → 0.05 m/s

    const bool att_ok = (t->have & RHAVE_ATT) != 0;
    const uint32_t r12 = att_ok ? ((uint32_t)w2_clampi(w2_divr(t->roll_cdeg,  10), -2047, 2047) & 0xFFF) : W2_ROLL_NONE;
    const uint32_t p12 = att_ok ? ((uint32_t)w2_clampi(w2_divr(t->pitch_cdeg, 10), -2047, 2047) & 0xFFF) : 0;
    const uint32_t y12 = (t->have & RHAVE_ATT_YAW)
                       ? (uint32_t)(w2_divr((int32_t)t->yaw_cdeg, 10) % 3600) : W2_YAW_NONE;   // 0.01 → 0.1°（359.95〜 は 0）
    const uint32_t alpha10 = (uint32_t)((t->have & RHAVE_ALPHA)
                         ? w2_clampi(w2_divr(t->alpha_avg_cdeg, 10), -511, 511) : W2_ALPHA_NONE) & 0x3FF;
    const uint64_t att = (uint64_t)r12 | ((uint64_t)p12 << 12) | ((uint64_t)y12 << 24) | ((uint64_t)alpha10 << 36)
                       | ((uint64_t)((t->tas_flags & LINK_TAS_CLAMPED) ? 1 : 0) << 46);
    for (int i = 0; i < 6; i++) out[W2_ATT + i] = (uint8_t)(att >> (8 * i));

    uint32_t s = 0;
    const uint8_t slot = link_w2_slot_of(seq8);
    switch (slot) {
    case W2S_DATE: {
        const bool ok = (t->have & RHAVE_DATE) && t->year >= 2000 && t->year < 2128;
        if (ok) s = (uint32_t)(t->year - 2000) | ((uint32_t)(t->month & 0xF) << 7) |
                    ((uint32_t)(t->day & 0x1F) << 11) | ((uint32_t)(t->hour & 0x1F) << 16) | (1u << 21);
        break; }
    case W2S_WIND:
    case W2S_WIND2: {
        if (t->have & RHAVE_ATT_WIND) {
            const uint32_t wd = (uint32_t)(w2_divr(t->wind_dir_cdeg, 10) % 3600);
            s = (uint32_t)t->wind_dmps | (wd << 8) | (1u << 20);
        }
        s |= (uint32_t)settings_check << 24;
        break; }
    case W2S_TRIM: {
        const uint32_t tr = (uint32_t)w2_clampi(t->roll_trim_cdeg, -2047, 2047) & 0xFFF;
        s = tr;
        if (t->have & RHAVE_ATT_TRIM) s |= 1u << 21;
        break; }
    case W2S_HACC: {
        const uint32_t ha = (t->hacc_dm > 1023) ? 1023u : t->hacc_dm;
        const uint32_t ns = (t->have & RHAVE_NUMSAT)
                          ? (uint32_t)(t->numsat >= W2_NUMSAT_NONE ? W2_NUMSAT_NONE - 1 : t->numsat) : W2_NUMSAT_NONE;
        s = ha | (ns << 10) | ((uint32_t)(uint8_t)t->dest_index << 16);
        break; }
    case W2S_ALTV: {
        const uint16_t alt = (t->have & RHAVE_GNSSALT) ? (uint16_t)w2_clampi(t->gnss_alt_dm, -32767, 32767)
                                                       : (uint16_t)0x8000;
        s = alt | ((uint32_t)t->batt << 16);   // 電池の表示はそのまま（0 = 分からない）
        break; }
    case W2S_PRESS: {
        const int32_t pr = (t->have & RHAVE_PRESS) ? w2_clampi(t->press_dpa, 1, 0x3FFF) : 0;
        s = (uint32_t)pr | ((uint32_t)(t->scale_index & 7) << 14);
        break; }
    case W2S_NAV:
        // 目的地の ID の 0 は「送っていない」。受信側は番号で合わせる。
        // ナビモードは 2 ビット（DMODE_* は 0〜2。link.cpp の static_assert）。上の 6 ビットは sAcc。
        s = (t->nav_mode & 0x03u) | ((uint32_t)link_w2_sacc_code(t->sacc_cms) << 2) | ((uint32_t)t->dest_id << 8);
        break;
    }
    if (const uint8_t shift = w2_slow_shift(slot)) {
        if (t->status & LINK_ST_SD_ERROR)    s |= 1u << shift;
        if (t->status & LINK_ST_BAT_LOW)     s |= 2u << shift;
        if (t->status & LINK_ST_NEEDS_APPLY) s |= 4u << shift;
        if (t->tas_flags & LINK_TAS_HDR)     s |= 8u << shift;
        if (t->have & RHAVE_ATT_YAW) s |= (0x100u | t->yaw_acc95_deg) << w2_yaw_acc_shift(slot);
    }
    w2_put32(out + W2_SLOT, s);
    w2_put16(out + W2_CRC, link_crc16(out, W2_CRC));
}

// ============================================================
//  受信側
// ============================================================
// 形式として正しいか（印と CRC）。
// ★ **グループ ID はここで見ない。** ここで弾くと「電波は来ているがグループが違う」状態を無信号と
//   区別できなくなり、SD が飛んで group が既定に戻った事故に気づけなくなる。突き合わせは link.cpp
//   が別に行う（docs/pons_link.md §1）。印もグループの 2 ビットを落として比べる（link_w2_is_magic）。
static inline bool link_w2_crc_ok(const uint8_t* p) {
    return w2_get16(p + W2_CRC) == link_crc16(p, W2_CRC);
}

static inline bool link_w2_valid(const uint8_t* p) {
    return link_w2_is_magic(p[W2_MAGIC]) && link_w2_crc_ok(p);
}

// 同じ長さと CRC 配置を持つ旧形式の印。表示の診断専用。
static inline uint8_t link_w2_legacy_magic(uint8_t b) {
    if (b == 0xA2) return b;
    const uint8_t magic = b & LINK_W2_MAGIC_MASK;
    return (magic >= 0xA4 && magic <= 0xC0 && magic != LINK_W2_MAGIC) ? magic : 0;
}

static inline uint8_t link_w2_legacy_group(const uint8_t* p) {
    return p[W2_MAGIC] == 0xA2 ? p[2] : (p[W2_MAGIC] & 0x03);  // A2 のグループは独立したバイト
}
static inline uint8_t link_w2_group(const uint8_t* p) { return (uint8_t)(p[W2_MAGIC] & 0x03); }
static inline uint8_t link_w2_slot(const uint8_t* p)  { return link_w2_slot_of(p[W2_SEQ]); }

// 受信側の組み立ての状態。回転枠で受けた値と、16 ビットの seq・時の繰り上がりを覚える。
struct LinkW2Rx {
    LinkTelem acc;          // 組み立て中の LinkTelem（最後に分かった値）
    uint16_t  slot_have;    // 回転枠で受けて分かっている have ビット（W2_SLOT_HAVE の部分集合）
    // 受けたことのある回転枠（1 << 枠の番号）。have ビットを持たない項目（縮尺・目的地・ナビモード・
    // 水平精度）は、受けるまで 0 のままなので、使う側がこれで確かめる（link_w2_slot_seen）。
    uint8_t   slot_seen;
    bool      yawacc_known; // 枠 2 または 6 から有効なヨー精度を受けたか
    bool      hour_known;
    uint8_t   hour;
    uint16_t  prev_tod;
    bool      seq_known;
    uint16_t  seq16;
    uint32_t  last_ms;      // 最後に受けた時刻（回転枠の値が古くなったかの判定）
    uint32_t  last_press_ms; // 気圧・縮尺を最後に受けた時刻（枠 6）
    uint32_t  last_slow_ms;  // 状態・ヘッダー設定を最後に受けた時刻（枠 2 または 6）
    bool      slow_known;
    uint8_t   settings_check;
    uint32_t  settings_ms;
};

// 回転枠の値を「送る間隔 × これ」より長く受けていなければ、分からなくなったとみなす。
// 受信機の「無信号」（ローワーで 10 秒）と同じ長さ。電波が戻った直後に古い風や電池を出さないため。
#define LINK_W2_STALE_FRAMES  10

static inline void link_w2_rx_reset(LinkW2Rx* st) {
    memset(st, 0, sizeof(*st));
    st->acc.sacc_cms = LINK_SACC_NONE;   // 0 は「0.00 m/s」なので、受けるまでは「無し」にしておく
    // 電池の表示（batt）は 0 が「分からない」なので memset のままでよい
}

// その回転枠を（途切れてから）受けたか。have ビットの無い項目を使う前に見ること。
// ★ 受けるまで 0 のままの目的地・ナビモード・縮尺をそのまま使うと、受信機が
//   「送信機は目的地 0・縮尺 0」と思い込んで、自分の設定を書き換えてしまう（link_sync_destination）、
//   設定画面に送信機の縮尺を嘘で出す（link_get_sender_scale_index）。
static inline bool link_w2_slot_seen(const LinkW2Rx* st, uint8_t slot) {
    return (st->slot_seen & (1u << slot)) != 0;
}

// 日付を 1 日進める（時の繰り上がりで 23 → 0 になったとき。日付は回転枠 0 でしか来ないため）。
static inline void link_w2_next_day(LinkTelem& t) {
    static const uint8_t kDim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (t.year == 0 || t.month < 1 || t.month > 12 || t.day < 1) return;   // まだ日付を受けていない
    uint8_t md = kDim[t.month - 1];
    if (t.month == 2 && (t.year % 4 == 0 && (t.year % 100 != 0 || t.year % 400 == 0))) md = 29;
    if (++t.day > md) { t.day = 1; if (++t.month > 12) { t.month = 1; t.year++; } }
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
        st->settings_check = 0;
        st->slot_have = 0; st->slot_seen = 0; st->yawacc_known = false;
        t.sacc_cms = LINK_SACC_NONE;   // have ビットが無いので値で「無し」に戻す（古い sAcc を出さない）
        t.batt     = LINK_BATT_NONE;   // 電池の表示も同じ（受信ログに古い値を書かない）
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
    t.group  = link_w2_group(p);
    const uint32_t status_alt = w2_get24(p + W2_STATUS_ALT);
    const uint8_t press_bit = 1u << W2S_PRESS;
    if (!(st->slot_seen & press_bit) || now_ms - st->last_press_ms > LINK_W2_STALE_FRAMES * per_ms) {
        st->slot_seen &= (uint8_t)~press_bit;
        st->slot_have &= (uint16_t)~RHAVE_PRESS;
    }
    if (st->slow_known && now_ms - st->last_slow_ms > LINK_W2_STALE_FRAMES * per_ms) {
        st->slow_known = false;
        st->yawacc_known = false;
        t.yaw_acc95_deg = 0;
        t.status &= (uint16_t)~W2_ST_SLOW;
        t.tas_flags &= (uint8_t)~LINK_TAS_HDR;
    }
    t.status = (uint16_t)((t.status & W2_ST_SLOW) |
                          ((status_alt << 1) & LINK_ST_IMU_ERROR) |
                          ((status_alt << 3) & W2_ST_GROUND_FAULTS) |
                          ((status_alt << 4) & W2_ST_TIME));
    t.fixflags = (status_alt & W2_ST_FIXOK) ? 0x01 : 0x00;

    const uint8_t tas = p[W2_TAS];
    t.tas_dmps = tas;

    const uint16_t tod = w2_get16(p + W2_TOD);
    if (tod < 36000) {
        // 時は回転枠 0 から。その間に時刻が大きく戻ったら次の時へ繰り上げる
        if (st->hour_known && st->prev_tod != 0xFFFF && tod + 18000 < st->prev_tod) {
            // ★ 0 時をまたいだら日付も進める（UTC。送信機は GNSS の UTC の日時をそのまま載せている）。進めないと、
            //   次の回転枠 0（最大 8 発後）まで受信ログが「前日の 00:xx」＝丸 1 日戻った時刻になっていた（2026-11-10 まで）。
            // ★ 日付の have ビット（回転枠 0）は見ない。途切れ（LINK_W2_STALE_FRAMES）で have ビットは消えるが、時は 30 分まで
            //   覚えているので、途切れ明けに 23 → 0 時をまたぐと日付だけ進まず、次の枠 0 まで前日になった（BUG-72）。
            //   覚えている日付を進めておく（日付を一度も受けていなければ link_w2_next_day() が何もしない）。
            if (st->hour == 23) link_w2_next_day(t);
            st->hour = (uint8_t)((st->hour + 1) % 24);
        }
        st->prev_tod = tod;
        t.minute = (uint8_t)(tod / 600);
        t.second = (uint8_t)((tod / 10) % 60);
        t.centi  = (uint8_t)((tod % 10) * 10);
        if (st->hour_known) t.hour = st->hour;   // 分からない間は前の値のまま（受け始めは 0。日付も無い）
    } else {
        // ※ tod == 0xFFFF（時刻無効）時は 00:00:00 に飛ばさず直前の時刻・日付を保持する設計
        st->prev_tod = 0xFFFF;
    }

    uint64_t pos = 0;
    for (int i = 0; i < 8; i++) pos |= (uint64_t)p[W2_POS + i] << (8 * i);
    t.lat_1e7 = w2_clampi(w2_sext((uint32_t)pos, 28), -90000000, 90000000) * 10;
    t.lon_1e7 = w2_clampi(w2_sext((uint32_t)(pos >> 28), 29), -180000000, 180000000) * 10;
    const int32_t pa = w2_sext((uint32_t)(pos >> 57), 7);
    t.pitch_avg_cdeg = pa != W2_PAVG_NONE ? (int16_t)(pa * 25) : 0;
    const uint32_t gt = w2_get24(p + W2_GSTRK);
    const uint32_t trk = (gt >> 12) & 0xFFF;
    const bool gs_ok = (trk != W2_TRK_NONE);
    if (gs_ok) {
        t.gs_cms     = (uint16_t)(((gt & 0xFFF) * 5) / 2);
        t.track_cdeg = (uint16_t)(trk * 10);
    }
    const uint16_t alt = (uint16_t)(status_alt >> 10);
    t.kf_alt_dm = alt != W2_KFALT_NONE ? (int16_t)(alt + W2_KFALT_MIN_DM) : 0;
    const int8_t vs = (int8_t)p[W2_KFVS];
    t.kf_vs_cms = vs != W2_VS_NONE ? (int16_t)(vs * 5) : 0;

    uint64_t att = 0;
    for (int i = 0; i < 6; i++) att |= (uint64_t)p[W2_ATT + i] << (8 * i);
    const uint32_t r12 = (uint32_t)(att & 0xFFF);
    const bool att_ok = (r12 != W2_ROLL_NONE);
    if (att_ok) {
        t.roll_cdeg  = (int16_t)(w2_sext(r12, 12) * 10);
        t.pitch_cdeg = (int16_t)(w2_sext((uint32_t)((att >> 12) & 0xFFF), 12) * 10);
    }
    // ヨーは 0〜359.9° のまま戻す（LinkTelem の yaw_cdeg は uint16 の 0〜35999。LINK_PROTO_VER 9 〜。
    //   8 までは int16 で ±180° に戻していた）。送る側は 3600 以上を出さないので、W2_YAW_NONE 以外の 3600〜4094 も「無し」。
    const uint32_t y12 = (uint32_t)((att >> 24) & 0xFFF);
    const bool yaw_ok = (y12 < 3600);
    if (yaw_ok) t.yaw_cdeg = (uint16_t)(y12 * 10);
    const int32_t alpha = w2_sext((uint32_t)((att >> 36) & 0x3FF), 10);
    if (alpha != W2_ALPHA_NONE) t.alpha_avg_cdeg = (int16_t)(alpha * 10);
    t.tas_flags &= LINK_TAS_HDR;
    if ((att >> 46) & 1) t.tas_flags |= LINK_TAS_CLAMPED;

    // ---- 回転枠 ----
    const uint32_t s = w2_get32(p + W2_SLOT);
    const uint8_t slot = link_w2_slot_of(seq8);
    st->slot_seen |= (uint8_t)(1u << slot);
    switch (slot) {
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
    case W2S_WIND:
    case W2S_WIND2:
        st->settings_check = (uint8_t)(s >> 24);
        st->settings_ms = now_ms;
        if (s & (1u << 20)) {
            t.wind_dmps     = (uint8_t)(s & 0xFF);
            t.wind_dir_cdeg = (uint16_t)(((s >> 8) & 0xFFF) * 10);
            st->slot_have |= RHAVE_ATT_WIND;
        } else st->slot_have &= (uint16_t)~RHAVE_ATT_WIND;
        break;
    case W2S_TRIM:
        t.roll_trim_cdeg = (int16_t)w2_sext(s & 0xFFF, 12);
        if (s & (1u << 21)) st->slot_have |= RHAVE_ATT_TRIM; else st->slot_have &= (uint16_t)~RHAVE_ATT_TRIM;
        break;
    case W2S_HACC: {
        t.hacc_dm    = (uint16_t)(s & 0x3FF);
        const uint8_t ns = (uint8_t)((s >> 10) & 0x3F);
        if (ns != W2_NUMSAT_NONE) { t.numsat = ns; st->slot_have |= RHAVE_NUMSAT; }
        else                      st->slot_have &= (uint16_t)~RHAVE_NUMSAT;
        t.dest_index = (int8_t)((s >> 16) & 0xFF);
        break; }
    case W2S_ALTV: {
        const uint16_t galt = (uint16_t)(s & 0xFFFF);
        if (galt != 0x8000) { t.gnss_alt_dm = (int16_t)galt; st->slot_have |= RHAVE_GNSSALT; }
        else                st->slot_have &= (uint16_t)~RHAVE_GNSSALT;
        t.batt = (uint8_t)((s >> 16) & 0xFF);   // 電池の表示（読むのは link_batt_decode()）
        break; }
    case W2S_PRESS: {
        const uint16_t pr = (uint16_t)(s & 0x3FFF);
        if (pr) { t.press_dpa = pr; st->slot_have |= RHAVE_PRESS; } else st->slot_have &= (uint16_t)~RHAVE_PRESS;
        t.scale_index = (uint8_t)((s >> 14) & 7);
        st->last_press_ms = now_ms;
        break; }
    case W2S_NAV:
        t.nav_mode = (uint8_t)(s & 0x03);
        t.sacc_cms = link_w2_sacc_cms((uint8_t)((s >> 2) & 0x3F));
        t.dest_id  = (uint16_t)((s >> 8) & 0xFFFF);
        break;
    }

    if (const uint8_t shift = w2_slow_shift(slot)) {
        const uint32_t flags = s >> shift;
        t.status &= (uint16_t)~W2_ST_SLOW;
        if (flags & 1) t.status |= LINK_ST_SD_ERROR;
        if (flags & 2) t.status |= LINK_ST_BAT_LOW;
        if (flags & 4) t.status |= LINK_ST_NEEDS_APPLY;
        t.tas_flags &= (uint8_t)~LINK_TAS_HDR;
        if (flags & 8) t.tas_flags |= LINK_TAS_HDR;
        const uint32_t yaw_acc = s >> w2_yaw_acc_shift(slot);
        st->yawacc_known = (yaw_acc & 0x100u) != 0;
        t.yaw_acc95_deg = st->yawacc_known ? (uint8_t)(yaw_acc & 0xFFu) : 0;
        st->last_slow_ms = now_ms;
        st->slow_known = true;
    }

    // TAS が欠測したときは次の風の回転枠を待たずに、キャッシュした風を捨てる。
    if (tas == 0) st->slot_have &= (uint16_t)~RHAVE_ATT_WIND;

    // ---- have ビット: 核の「無し」でない項目 ＋ 回転枠で分かっている分 ----
    uint16_t have = st->slot_have;
    if (gs_ok)                  have |= RHAVE_GS | RHAVE_TTRACK;
    if (alt != W2_KFALT_NONE)   have |= RHAVE_KFALT;
    if (vs != W2_VS_NONE)       have |= RHAVE_KFVS;
    if (att_ok)                 have |= RHAVE_ATT;
    if (pa != W2_PAVG_NONE)      have |= RHAVE_ATT_AVG;
    if (alpha != W2_ALPHA_NONE) have |= RHAVE_ALPHA;
    if (tas != 0)               have |= RHAVE_TAS;
    // ★ ヨーは精度（回転枠 2 / 6）が分かるまで出さない。精度 0 のまま出すと「とても確か」に見え、
    //   ヨーの信頼度で絞っている風や機首の表示がそのまま出てしまう。
    if (yaw_ok && st->yawacc_known) have |= RHAVE_ATT_YAW;
    t.have = have;
}

// 1970-01-01 からの日数（gnss_dbd.cpp・rtc_backup.cpp と同じ。Howard Hinnant の days_from_civil）
static inline int32_t w2_days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

// いま受けた 1 フレーム（p。link_w2_decode() を通したあと）の時刻を UTC の UNIX 時刻 [ms] にする。
// GNSS の時刻の助け合い（link_assist.cpp）が、ボートの GNSS へ入れるために使う（計画書 §4.1）。
// ★ 送信機は回転枠 0 に **GNSS の UTC の日付と時** をそのまま載せている（link.cpp の link_push_telemetry。
//   飛行 CSV は JST に直して書くが、電波は UTC）。ここでも UTC のまま扱う。
// ★ **送信機の fix は条件にしない**（2026-11-13〜）。fix の前でも、GNSS の時計（RTC）から時刻が出ていれば使う。
//   確からしさは送信機が status で知らせる（LINK_ST_TIME_COARSE / LINK_ST_TIME_BAD。NAV-PVT の tAcc から）。
//   止まった時刻（送信機の GNSS が黙った）は、受ける側の 2 発の確かめ（link_assist.cpp の consider_time）で弾く。
// 戻り値: LINK_W2_TIME_NONE  使えない
//          LINK_W2_TIME_FINE  精度 1 秒以内（古い送信機はビットを立てないので、こちらになる）
//          LINK_W2_TIME_COARSE 精度 1〜4 秒
// 使えないとき:
//   ・このフレームの時刻が「無し」（0xFFFF）。decode は直前の時刻を残すので、残った古い値を使わない
//   ・時と日付がまだ分からない（回転枠 0 を受ける前、長い途切れのあと）。受け始めは時が 0 のままで、
//     そのまま使うと何時間もずれた時刻を GNSS に入れてしまう（何もしないより悪い）
//   ・送信機が「時刻の精度が悪い・分からない」と言っている（LINK_ST_TIME_BAD）、GNSS が黙っている（LINK_ST_GNSS_LOST）
#define LINK_W2_TIME_NONE    0
#define LINK_W2_TIME_FINE    1
#define LINK_W2_TIME_COARSE  2
static inline uint8_t link_w2_utc_ms(const uint8_t* p, const LinkW2Rx* st, uint64_t* utc_ms) {
    const uint16_t tod = w2_get16(p + W2_TOD);
    if (tod >= 36000) return LINK_W2_TIME_NONE;
    const uint16_t sb = st->acc.status;
    // 送信機が「時刻が悪い」と言っている・GNSS が黙っている（古い送信機は BAD を立てないので、こちらでも見る。BUG-68）
    if (sb & (LINK_ST_TIME_BAD | LINK_ST_GNSS_LOST)) return LINK_W2_TIME_NONE;
    if (!st->hour_known || !(st->slot_have & RHAVE_DATE)) return LINK_W2_TIME_NONE;
    const LinkTelem& t = st->acc;
    if (t.year < 2020 || t.month < 1 || t.month > 12 || t.day < 1 || t.day > 31 || st->hour > 23) return LINK_W2_TIME_NONE;
    const int64_t days = w2_days_from_civil(t.year, t.month, t.day);
    *utc_ms = (uint64_t)((days * 86400LL + (int64_t)st->hour * 3600LL) * 1000LL) + (uint64_t)tod * 100ULL;
    return (sb & LINK_ST_TIME_COARSE) ? LINK_W2_TIME_COARSE : LINK_W2_TIME_FINE;
}

#endif // LINK_WIRE2_H
