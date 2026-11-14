// ============================================================
// File    : imu_sch16t.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : SCH16T-K01 ドライバ（SPI1 / SafeSPI2 48bit アウトオブフレーム）。
//           imu_sensor.h の契約を実装する。チップ依存はこのファイルだけ。
//
// ■ このチップは「ジャイロと加速度しか出さない」
//   融合が無いので、姿勢は attitude.cpp の ESKF、重力除去はバリオ KF が自分でやる。
//
// ■ フレーム層は tools/sch16t_test/ からの移植
//   あちらは Pico 2 単体で動く単独テストで、**実機で全項目が通っている**
//   （2026-10-14。SYS_TEST 5/5・不良 0/約 213 万フレーム・感度 1599.3/1600・
//     |a|/g = 0.99916）。CRC8 とフレーム組み立ては Murata 自身の BSD コードの
//   事前計算値 47 件とも一致させてある。**ここを「改善」しないこと。**
//   実測値と立ち上げ手順 → docs/imu_sch16t_bringup.md
//
// ■ 設定値は settings.h の SCH16T_* が正本
//   LPF2(13Hz) + DEC5(737.5Hz、実測 721.5Hz)、読み出し 250Hz（最小推奨 150Hz）。
//   選定理由は settings.h のコメントと docs/imu_sch16t_plan.md §5 の (2)。
//
// ■ 読み出しはタイマー割り込み（SCH16T_READ_IN_IRQ。2026-11-11〜）
//   Core0 のハードウェアアラームが 4ms ごとに SPI1 を読み、生のフレームをリングバッファへ積む（imu_alarm_cb）。
//   検査・換算・推定への受け渡しはスレッド側（drain_ring。imu_drv_poll と imu_drv_service_if_due から）。
//   それまでは loop() で取りに行っていたので、地図の描き直しで Core0 が止まる間（2Hz ごとに 70〜90ms）のサンプルが抜けていた。
//   理由・優先度（最低にする）・SPI1 の取り合いの約束は settings.h の SCH16T_READ_IN_IRQ のコメント。
//
// ★ **本体基板では起動と机上確認（Step 2〜3）まで済み**（2026-11-03）。
//   docs/imu_sch16t_bringup.md の Step 4〜5 を通すまで飛ばさないこと。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/11
// ============================================================

#include <Arduino.h>
#include "settings.h"


#include <SPI.h>
#if SCH16T_READ_IN_IRQ
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#endif
#include "imu.h"
#include "imu_sensor.h"
#include "mysd.h"         // enqueueTask / createLogSdTask
#include "src/imulog.h"   // 生データロガー（★ チップ側に残す。理由は imu_sensor.h）

// ============================================================
//  レジスタ（データシート Table 29。PX4 の Murata_SCH16T_registers.hpp と一致）
// ============================================================
enum : uint8_t {
    REG_RATE_X2 = 0x0A, REG_RATE_Y2 = 0x0B, REG_RATE_Z2 = 0x0C,
    REG_ACC_X2  = 0x0D, REG_ACC_Y2  = 0x0E, REG_ACC_Z2  = 0x0F,
    REG_TEMP    = 0x10,

    REG_STAT_SUM      = 0x14, REG_STAT_SUM_SAT = 0x15, REG_STAT_COM    = 0x16,
    REG_STAT_RATE_COM = 0x17, REG_STAT_RATE_X  = 0x18, REG_STAT_RATE_Y = 0x19,
    REG_STAT_RATE_Z   = 0x1A, REG_STAT_ACC_X   = 0x1B, REG_STAT_ACC_Y  = 0x1C,
    REG_STAT_ACC_Z    = 0x1D,

    REG_CTRL_FILT_RATE  = 0x25, REG_CTRL_FILT_ACC12 = 0x26,
    REG_CTRL_FILT_ACC3  = 0x27, REG_CTRL_RATE       = 0x28,
    REG_CTRL_ACC12      = 0x29, REG_CTRL_ACC3       = 0x2A,
    REG_CTRL_USER_IF    = 0x33, REG_CTRL_MODE       = 0x35,
    REG_CTRL_RESET      = 0x36,
    REG_ASIC_ID = 0x3B, REG_COMP_ID = 0x3C,
};

static const uint16_t BIT_EOI            = (1 << 1);   // End of Initialization
static const uint16_t BIT_EN_SENSOR      = (1 << 0);   // RATE/ACC 測定の開始
static const uint16_t VAL_SPI_SOFT_RESET = 0b1010;     // CTRL_RESET へ書く値
static const uint16_t BIT_DRY_DRV_EN     = (1 << 5);   // CTRL_USER_IF: DRY 出力
static const uint16_t BIT_DRY_POL_LOW    = (1 << 6);   // 0 = high active（既定）

// 応答フレームのステータス（[36]IDS [35]CE [34:33]S1:S0）
static const uint64_t MASK48_CE = (uint64_t)1 << 35;
static const uint64_t MASK48_S1 = (uint64_t)1 << 34;
static const uint64_t MASK48_S0 = (uint64_t)1 << 33;

// ============================================================
//  状態
// ============================================================
static bool     s_present = false;          // 初期化に成功して「載っている」
// ★★ **「一度でも動いたか」を別に持つ。** これが false の間は復旧を試みない。
//   sch_init() は合計 300ms 前後ブロックするので、センサーが載っていない機体で
//   60 秒ごとに呼ぶと **Core0 が定期的に 300ms 止まる**（地図描画と E220 の
//   AUX 窓に効く）。
static bool     s_ever_alive = false;
static uint32_t s_last_sample_ms = 0;       // 最後にサンプルを引き取った時刻
static uint32_t s_poll_last_us = 0;
static uint32_t s_poll_gap_max_us = 0;
static uint32_t s_recover_last_ms = 0;

// レート計測（60 秒ログの raw GYR / ACC）
static uint32_t s_hz_last_ms = 0;
static uint16_t s_gyro_cnt = 0, s_accel_cnt = 0;
static uint32_t s_taken = 0;        // 推定へ渡したサンプルの通し数（imu_drv_poll が「この回に受け取ったか」を見る）
static float    s_gyro_hz = 0.0f, s_accel_hz = 0.0f;

// 健全性カウンタ。★ **いまは数えるだけで、どこへも出していない**（シリアルにも SD ログにも。2026-11-13 に確かめた。
//   それまでのコメントは「シリアルと SD ログへ出している」と書いていた）。
//   画面と 60 秒ログへ繋ぐのは docs/imu_sch16t_plan.md §2.7 の作業（docs/pons_todo.md で保留中）。
static uint32_t s_crc_err = 0;      // CRC 不一致
static uint32_t s_ce_err  = 0;      // CE（コマンドエラー）
static uint32_t s_sat_err = 0;      // S=10（飽和）。データ自体は使える
static uint32_t s_stat_bad = 0;     // STAT_SUM に 0 のビットがあった回数

// ============================================================
//  SPI 48bit フレーム層（tools/sch16t_test から移植）
// ============================================================
// CRC8: 多項式 0x2F・初期値 0xFF・上位 40bit（[47:8]）が対象。
// ★ Murata / PX4 と同一実装。**下位 8bit を 0 にしてから 48 回まわす**のが要点で、
//   「[47:8] を 40 回」に書き換えると値が変わる。
static uint8_t crc8_frame(uint64_t frame) {
    uint64_t data = frame & 0xFFFFFFFFFF00ULL;
    uint8_t crc = SCH16T_CRC8_INIT;
    for (int i = 47; i >= 0; i--) {
        const uint8_t bit = (uint8_t)((data >> i) & 0x01);
        crc = (crc & 0x80) ? (uint8_t)(((uint8_t)(crc << 1)) ^ SCH16T_CRC8_POLY) ^ bit
                           : (uint8_t)((uint8_t)(crc << 1) | bit);
    }
    return crc;
}

// 要求フレームの組み立て。TA[47:38]（上位 2bit=チップ選択 / 下位 8bit=レジスタ）、
// RW[37]、FT[35]=1（次の MISO を 48bit に）、DATAI[27:8]、CRC8[7:0]。
// ★ TA9:TA8 はブレイクアウト上で GND なので 0 固定。
static uint64_t build_read_frame(uint8_t addr) {
    uint64_t f = ((uint64_t)addr << 38) | ((uint64_t)1 << 35);
    return f | (uint64_t)crc8_frame(f);
}
static uint64_t build_write_frame(uint8_t addr, uint16_t value) {
    uint64_t f = ((uint64_t)addr << 38) | ((uint64_t)1 << 37)
               | ((uint64_t)1 << 35)    | ((uint64_t)value << 8);
    return f | (uint64_t)crc8_frame(f);
}

// 1 フレーム送受信。
// ★ **6 バイトを 1 回の transfer() で送る。`transfer16()` を 3 回呼んではいけない。**
//   arduino-pico の transfer16() は 1 ワードごとに戻るのでワード間で SCK が止まる。
//   transfer(tx, rx, 6) なら TX FIFO（8 段）へ先に積むので 48bit 連続する。
static uint64_t xfer48(uint64_t frame) {
    uint8_t tx[6], rx[6];
    for (int i = 0; i < 6; i++) tx[i] = (uint8_t)(frame >> (8 * (5 - i)));   // big endian

    SPI1.beginTransaction(SPISettings(SCH16T_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(SCH16T_PIN_CS, LOW);
    delayMicroseconds(1);                 // CS 立ち下がり → データ有効まで 40ns。余裕を取る
    SPI1.transfer(tx, rx, sizeof(tx));
    digitalWrite(SCH16T_PIN_CS, HIGH);
    SPI1.endTransaction();
    delayMicroseconds(1);                 // CS の High 期間を必ず作る

    uint64_t r = 0;
    for (int i = 0; i < 6; i++) r = (r << 8) | rx[i];
    return r;
}

// ★ **応答は 1 フレーム遅れて返る**（SafeSPI アウトオブフレーム）。
//   先頭の捨てフレームを落とすと、全軸が 1 つ前のレジスタの値になる
//   （gx に前回の何かが入るという形で静かに壊れる）。
static uint64_t req_read(uint8_t addr) { return xfer48(build_read_frame(addr)); }
static void reg_write(uint8_t addr, uint16_t v) { (void)xfer48(build_write_frame(addr, v)); }

// 単発読み出し。同じアドレスを 2 回投げて 2 回目の応答を取る。
static uint64_t reg_read(uint8_t addr) {
    req_read(addr);
    return req_read(addr);
}

// 20bit 2 の補数 → int32（符号拡張）。フレームの [27:8]。
static inline int32_t frame_data_i20(uint64_t f) {
    return (int32_t)(((uint32_t)(f << 4)) & 0xFFFFF000UL) >> 12;
}
static inline uint16_t frame_data_u16(uint64_t f) {
    return (uint16_t)((f >> 8) & 0xFFFFUL);
}

// ★ **ID やレジスタ読みの検証に S1:S0 を使ってはいけない。**
//   S は「センサーの測定状態」で、**EOI の前は 11（初期化中）**。そこで弾くと
//   初期化中のレジスタ読みが全部「壊れている」になる。CRC（届いたか）と
//   CE（そのアドレスへの読みが妥当か）だけ見れば足りる。
static bool frame_integrity_ok(uint64_t f) {
    if ((uint8_t)(f & 0xFF) != crc8_frame(f)) { s_crc_err++; return false; }
    if (f & MASK48_CE)                        { s_ce_err++;  return false; }
    return true;
}

// サンプル用の健全性判定。こちらは S も見る（測定中なので 00 が正常）。
static bool frame_sample_ok(uint64_t f) {
    if (!frame_integrity_ok(f)) return false;
    const bool s1 = (f & MASK48_S1) != 0, s0 = (f & MASK48_S0) != 0;
    if (s1 && s0) return false;             // 11 = 初期化中
    if (s1)       { s_sat_err++; return true; }  // 10 = 飽和。データ自体は使える
    if (s0)       return false;             // 01 = エラー
    return true;
}

// ============================================================
//  初期化
// ============================================================
// CTRL_RATE / CTRL_ACC12: [2:0][5:3][8:6] = 各軸の DEC、[11:9] DYN_XYZ2、[14:12] DYN_XYZ1
static uint16_t build_ctrl_rate() {
    const uint16_t d = SCH16T_DEC_SEL & 7, y = SCH16T_RATE_DYN_SEL & 7;
    return (uint16_t)(d | (d << 3) | (d << 6) | (y << 9) | (y << 12));
}
static uint16_t build_ctrl_acc12() {
    const uint16_t d = SCH16T_DEC_SEL & 7, y = SCH16T_ACC12_DYN_SEL & 7;
    return (uint16_t)(d | (d << 3) | (d << 6) | (y << 9) | (y << 12));
}
// CTRL_FILT_*: 3 軸それぞれ 3bit
static uint16_t build_filt(uint8_t sel) {
    const uint16_t v = sel & 7;
    return (uint16_t)(v | (v << 3) | (v << 6));
}

// リセット後、チップが「まともな値を返す」状態になるまで待つ。
// ★★ **固定 delay だけに頼らないこと。** リセット直後の SCH16T は
//   「フレーム構造は正しく CRC も合っているが、データ field が全部 0」の応答を返す。
//   それを待たずに設定を書くと**黙って捨てられる**（実機 2026-10-14 で踏んだ）。
//   ASIC_ID は NVM 由来なので、0x0000/0xFFFF 以外が読めたら NVM 読み出し完了の証拠。
// 戻り値: 待った時間 [ms]。-1 = タイムアウト。
static int32_t wait_chip_ready(uint32_t timeout_ms) {
    const uint32_t t0 = millis();
    bool any_frame = false;     // CRC の合うフレームが 1 つでも来たか（= チップは SPI の上に居る）
    for (;;) {
        const uint64_t f = reg_read(REG_ASIC_ID);
        const uint16_t v = frame_data_u16(f);
        // ★ 待っている間の CRC/CE 不一致は健全性カウンタへ入れない（居ないチップを数えても意味が無い）。
        const uint32_t crc0 = s_crc_err, ce0 = s_ce_err;
        const bool ok = frame_integrity_ok(f);
        s_crc_err = crc0; s_ce_err = ce0;
        if (ok) any_frame = true;
        if (ok && v != 0x0000 && v != 0xFFFF) return (int32_t)(millis() - t0);
        // ★ 居ないチップは早く見切る（settings.h の SCH16T_READY_NOFRAME_MS）。居るチップの NVM 待ちは
        //   CRC の合うフレームが来ているので、ここでは切らずに timeout_ms まで待つ。
        if (!any_frame && millis() - t0 >= SCH16T_READY_NOFRAME_MS) return -1;
        if (millis() - t0 >= timeout_ms) return -1;
        delay(2);
    }
}

// CTRL_USER_IF は **読んでから必要なビットだけ立てて書き戻す**。
// ★★ 素で書くとリセット値で 1 のビット（FTREE_TDEL / MISO_SR_CTRL / DRY_SR_CTRL。
//   リセット値 0x200C。実機で確認済み）を潰す。データシートが
//   "This option is not supported by Murata" と書いている側の設定になる。
//   **PX4 のドライバはこれを踏んでいる**ので、あちらを参考にするときは注意。
// ★ 読んだフレームの CRC を確かめてから書き戻す。化けた値を元にすると、化けたビットを書き込み、
//   読み返しも同じ値なので「通った」ことになる（2026-11-10 まで確かめていなかった）。
static bool set_dry(bool enable) {
    const uint64_t fb = reg_read(REG_CTRL_USER_IF);
    if (!frame_integrity_ok(fb)) return false;
    const uint16_t before = frame_data_u16(fb);
    uint16_t v = before;
    if (enable) v |= BIT_DRY_DRV_EN; else v &= (uint16_t)~BIT_DRY_DRV_EN;
    v &= (uint16_t)~BIT_DRY_POL_LOW;      // high active
    reg_write(REG_CTRL_USER_IF, v);
    const uint64_t fa = reg_read(REG_CTRL_USER_IF);
    return frame_integrity_ok(fa) && frame_data_u16(fa) == v;
}

// 初期化シーケンス（データシート §5.2 と Murata の C コード例に合わせた形）。
//   EXTRESN → ソフトリセット → NVM 待ち → 応答待ち → 設定 → EN_SENSOR
//   → 安定待ち → ステータス 1 周 → EOI → ステータス 2 周 → 検証
// ★ **EOI を書くとソフトリセットと SYS_TEST 以外の R/W レジスタが全部ロックされる。**
//   以降フィルターもレンジも変えられない（リセットするまで）。
static bool sch_init() {
    // ---- リセット（EXTRESN と SPI ソフトリセットの両方。Murata の例と同じ）----
    digitalWrite(SCH16T_PIN_EXTRESN, LOW);
    delay(2);
    digitalWrite(SCH16T_PIN_EXTRESN, HIGH);
    delay(10);
    reg_write(REG_CTRL_RESET, VAL_SPI_SOFT_RESET);
    delay(SCH16T_WAIT_NVM_MS + 2);        // +2 = ソフトリセット後の SPI 禁止期間

    const int32_t ready_ms = wait_chip_ready(SCH16T_READY_TIMEOUT_MS);
    if (ready_ms < 0) {
        enqueueTask(createLogSdTask("[IMU] SCH16T not responding (ASIC_ID unreadable)"));
        DEBUGW_PLN(20261014, "[IMU] SCH16T: ASIC_ID unreadable (check J6 wiring / SPI1 pins)");
        return false;
    }

    // ---- 品種の確認 ----
    // ★ **COMP_ID で見る。ASIC_ID はシリコンのリビジョンなので品種判定に使えない。**
    //   他品種だと**ジャイロの感度だけが 16 倍違い、加速度は共通**なので、
    //   1g 検証では原理的に気づけない（詳細は tools/sch16t_test/README.md）。
    const uint64_t fcomp = reg_read(REG_COMP_ID);
    const uint16_t comp = frame_data_u16(fcomp);
    if (!frame_integrity_ok(fcomp) || comp != SCH16T_COMP_ID) {
        // ★ 値は **16 進で**残す。0x0023 と比べる話なので 10 進では読めない。
        //   DEBUGW_P は基数を指定できないので、値は createLogSdfTask 側に出す。
        enqueueTask(createLogSdfTask("[IMU] SCH16T COMP_ID 0x%04X != K01 0x%04X",
                                     comp, SCH16T_COMP_ID));
        DEBUGW_PLN(20261014, "[IMU] SCH16T COMP_ID mismatch (value in log.txt)");
        return false;                      // 感度が違う可能性があるので使わない
    }

    // ---- 設定（EN_SENSOR と EOI の前に済ませる）----
    reg_write(REG_CTRL_FILT_RATE,  build_filt(SCH16T_FILT_SEL));
    reg_write(REG_CTRL_FILT_ACC12, build_filt(SCH16T_FILT_SEL));
    reg_write(REG_CTRL_FILT_ACC3,  build_filt(SCH16T_FILT_SEL));
    reg_write(REG_CTRL_RATE,       build_ctrl_rate());
    reg_write(REG_CTRL_ACC12,      build_ctrl_acc12());
    reg_write(REG_CTRL_ACC3,       SCH16T_ACC3_DYN_SEL);
    if (!set_dry(true)) {
        // DRY 自体は使っていない（ポーリングで読む）が、ここが失敗するのは
        // 書き込みが通っていない証拠なので初期化失敗にする。
        enqueueTask(createLogSdTask("[IMU] SCH16T CTRL_USER_IF write failed"));
        return false;
    }

    // ---- EN_SENSOR → 安定待ち → ステータス 1 周 ----
    // ★ **この待ちの間ステータスフラグの値は未定義**（データシート §5.2）。
    reg_write(REG_CTRL_MODE, BIT_EN_SENSOR);
    delay(SCH16T_WAIT_START_MS + 35);     // 規定 215ms に余裕

    static const uint8_t stat_regs[] = {
        REG_STAT_SUM, REG_STAT_SUM_SAT, REG_STAT_COM, REG_STAT_RATE_COM,
        REG_STAT_RATE_X, REG_STAT_RATE_Y, REG_STAT_RATE_Z,
        REG_STAT_ACC_X, REG_STAT_ACC_Y, REG_STAT_ACC_Z,
    };
    const unsigned n_stat = sizeof(stat_regs) / sizeof(stat_regs[0]);
    for (unsigned i = 0; i < n_stat; i++) req_read(stat_regs[i]);

    // ---- EOI（ここでレジスタがロックされる）----
    reg_write(REG_CTRL_MODE, (uint16_t)(BIT_EOI | BIT_EN_SENSOR));
    delay(SCH16T_WAIT_EOI_MS + 2);

    // ★ ステータスは 2 周読む。1 周目は初期化中のラッチが残っている。
    for (int pass = 0; pass < 2; pass++)
        for (unsigned i = 0; i < n_stat; i++) req_read(stat_regs[i]);

    // ---- 検証 ----
    // ★ **DYN は「書いた値」ではなく「読み返したビット」で確かめる。**
    //   スケーリングに使う SCH16T_LSB_PER_* が、いま本当にチップに入っている
    //   レンジ設定と対応しているかを見る唯一の経路。
    // ★ どの読み出しも CRC を確かめる（MISO が High に張り付くと STAT_SUM が 0xFFFF＝正常に見える）。
    const uint64_t fcr = reg_read(REG_CTRL_RATE);
    const uint64_t fca = reg_read(REG_CTRL_ACC12);
    const uint64_t ffi = reg_read(REG_CTRL_FILT_RATE);
    const uint64_t fsu = reg_read(REG_STAT_SUM);
    const bool frames_ok = frame_integrity_ok(fcr) & frame_integrity_ok(fca) &
                           frame_integrity_ok(ffi) & frame_integrity_ok(fsu);   // 短絡させない（数えるため）
    const uint16_t cr = frame_data_u16(fcr);
    const uint16_t ca = frame_data_u16(fca);
    const bool dyn_ok = (((cr >> 12) & 7) == (SCH16T_RATE_DYN_SEL & 7)) &&
                        (((cr >>  9) & 7) == (SCH16T_RATE_DYN_SEL & 7)) &&
                        (((ca >> 12) & 7) == (SCH16T_ACC12_DYN_SEL & 7)) &&
                        (((ca >>  9) & 7) == (SCH16T_ACC12_DYN_SEL & 7));
    const uint16_t filt = frame_data_u16(ffi);
    const bool filt_ok = (filt == build_filt(SCH16T_FILT_SEL));
    const uint16_t sum = frame_data_u16(fsu);
    const bool stat_ok = (sum == 0xFFFF);
    if (!stat_ok) s_stat_bad++;

    if (!(frames_ok && dyn_ok && filt_ok && stat_ok)) {
        enqueueTask(createLogSdfTask("[IMU] SCH16T verify NG crc=%d dyn=%d filt=%d STAT_SUM=0x%04X",
                                     (int)frames_ok, (int)dyn_ok, (int)filt_ok, (unsigned)sum));
        DEBUGW_PLN(20261014, "[IMU] SCH16T init verify failed (detail in log.txt)");
        return false;
    }

    enqueueTask(createLogSdTask("[IMU] SCH16T-K01 init OK"));
    DEBUGW_P(20261014, "[IMU] SCH16T-K01 ready. chip responded in ");
    DEBUGW_P(20261014, ready_ms);
    DEBUGW_PLN(20261014, " ms");
    return true;
}

// ============================================================
//  サンプルの引き取り
// ============================================================
// デシメーション出力（*_2）を 1 サンプルずつ読む。
// ★ **温度は毎回読まない**（間隔は settings.h の SCH16T_TEMP_EVERY）。
//   1 サンプルあたりのフレーム数が 8 → 7 に減る。
// ★ s_temp_c はいま**書くだけで誰も読んでいない**。画面と 60 秒ログへ出すのは
//   docs/imu_sch16t_plan.md §2.7 の作業で、そのときの受け皿として置いてある。
static float s_temp_c = 0.0f;      // 最後に読めたダイ温度 [℃]

// 読んだフレーム（RATE_X2 Y2 Z2 ACC_X2 Y2 Z2 の 6 本と、温度を読んだ回はその 1 本）を検査して推定へ渡す。
// ポーリングでも割り込みでも同じここを通る。t_us はそのサンプルを読み終えた時刻。
// ★ 健全性カウンタ（frame_sample_ok が数える）を触るのはスレッド側のここだけにしてある。
static bool take_sample(const uint64_t fr[6], const uint64_t *ft, uint32_t t_us) {
    if (ft) {
        // ★ 温度は 16bit 幅で 20bit フィールドの**上位に詰まっている**ので >>4。
        if (frame_sample_ok(*ft)) s_temp_c = (frame_data_i20(*ft) >> 4) / SCH16T_LSB_PER_DEGC;
    }

    // ★ **`&&` で繋いではいけない。** frame_sample_ok() は健全性カウンタを増やす
    //   副作用があるので、短絡すると 1 本目で落ちた時点で残りが数えられない。
    bool ok = true;
    for (int i = 0; i < 6; i++) ok &= frame_sample_ok(fr[i]);
    if (!ok) return false;

    // ジャイロ [LSB] → [rad/s]。
    // ★ **バイアスを引かずに渡す。** ESKF が自分で推定する（imu_sensor.h の約束）。
    const float gx = frame_data_i20(fr[0]) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    const float gy = frame_data_i20(fr[1]) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    const float gz = frame_data_i20(fr[2]) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    // 加速度 [LSB] → [m/s²]。★ **重力を含んだままの比力を渡す。**
    const float ax = frame_data_i20(fr[3]) / SCH16T_LSB_PER_MPS2;
    const float ay = frame_data_i20(fr[4]) / SCH16T_LSB_PER_MPS2;
    const float az = frame_data_i20(fr[5]) / SCH16T_LSB_PER_MPS2;

    // ★ 生ログの s_us には**受信時刻**を入れる。SCH16T はサンプルに
    //   タイムスタンプを付けない。acc（申告精度）も無いので 0 固定。decode_imulog.py はどちらも
    //   「チップ固有」として扱うので、ID は GYRO/ACCEL をそのまま使ってよい。
    // ★ 割り込みで読んでいるときは、t_us は**読んだ時刻**（推定へ渡した時刻ではない）。
    //   decode_imulog.py は s_us（= t_us）で読み出しの空きを測れる。
    const float g[3] = { gx, gy, gz };
    const float a[3] = { ax, ay, az };
    // ★ **feed は毎サンプル、ログは間引く**（SCH16T_IMULOG_DECIM。理由は settings.h）。
    //   推定は 250Hz で回したいが、その全数を SD へ落とすと予算を超える。
    static uint8_t log_skip = 0;
    const bool do_log = (++log_skip >= SCH16T_IMULOG_DECIM);
    if (do_log) log_skip = 0;
    // ★ **加速度を先に渡す。** ESKF の伝播はジャイロの到着で回り、そのとき手元の加速度（accel_last_）を使う。
    //   ジャイロを先に渡すと同じ時刻に読んだ加速度ではなく **1 サンプル前（4ms 前）** の加速度で伝播し、
    //   起動して最初のサンプルは静止の積算から漏れ、復旧直後の 1 サンプルは途絶える前の加速度を使っていた
    //   （2026-11-10 まで）。tools/imulog/eskf.py と tools/attitude_test も同じサンプルの組で回している。
    if (do_log) imulog_push(IMULOG_ID_GYRO,  t_us, 0, gx, gy, gz);
    if (do_log) imulog_push(IMULOG_ID_ACCEL, t_us, 0, ax, ay, az);
    imu_feed_accel(a);
    imu_feed_gyro(g, t_us);

    s_gyro_cnt++;
    s_accel_cnt++;
    s_taken++;
    s_last_sample_ms = millis();
    s_ever_alive = true;
    return true;
}

// ポーリングで 1 サンプル読む（SCH16T_READ_IN_IRQ = 0 のとき）。
// ★ **先頭に捨てフレームが 1 つ必要**（アウトオブフレーム）。全部で 7 フレーム（温度の回は 8）。
//   各応答は「1 つ前に要求したレジスタ」の値なので、要求を 1 つずらして並べる。
// ★ **先頭の捨てフレームを「前回の続き」で省いてはいけない。** アウトオブフレームの
//   パイプラインを呼び出しをまたいで繋げば 6 フレームで済むが、描画で 40ms 空いた
//   あとに**40ms 前の値を新しい値として受け取る**ことになる（静かに壊れる形）。
//   毎回捨てフレームから始める。割り込み版（imu_alarm_cb）も同じ並び。
static bool read_sample() {
    static uint8_t temp_skip = 0;
    const bool do_temp = (++temp_skip >= SCH16T_TEMP_EVERY);
    if (do_temp) temp_skip = 0;

    uint64_t fr[6], ft = 0;
    req_read(REG_RATE_X2);                    // 捨てフレーム
    fr[0] = req_read(REG_RATE_Y2);            // → RATE_X2
    fr[1] = req_read(REG_RATE_Z2);            // → RATE_Y2
    fr[2] = req_read(REG_ACC_X2);             // → RATE_Z2
    fr[3] = req_read(REG_ACC_Y2);             // → ACC_X2
    fr[4] = req_read(REG_ACC_Z2);             // → ACC_Y2
    // 最後の 1 本は ACC_Z2 の値を回収するための要求。アドレスは「次に読みたいもの」で、
    // 温度を読む回だけ TEMP にして、もう 1 フレーム足してその値を取る。
    fr[5] = req_read(do_temp ? REG_TEMP : REG_ACC_Z2);   // → ACC_Z2
    if (do_temp) ft = req_read(REG_TEMP);     // → TEMP
    return take_sample(fr, do_temp ? &ft : nullptr, time_us_32());
}

#if SCH16T_READ_IN_IRQ
// ============================================================
//  タイマー割り込みでの読み出し（settings.h の SCH16T_READ_IN_IRQ）
// ============================================================
// 割り込みが読んだ 1 サンプルぶんの生のフレーム。検査と換算はスレッド側（take_sample）。
typedef struct {
    uint64_t f[6];       // RATE_X2 Y2 Z2 ACC_X2 Y2 Z2
    uint64_t ft;         // TEMP（has_temp のときだけ意味がある）
    uint32_t t_us;       // 読み終えた時刻（time_us_32）
    uint8_t  has_temp;
} RawSample;
static RawSample s_ring[SCH16T_RING_LEN];
static volatile uint16_t s_ring_head = 0;     // 割り込みが次に書く位置
static volatile uint16_t s_ring_tail = 0;     // スレッドが次に読む位置
static volatile uint32_t s_ring_ovf  = 0;     // 一杯で読まなかった回数（起動処理の間の分は数えない）
static int      s_alarm   = -1;               // 使っているハードウェアアラーム。-1 = まだ取っていない
static volatile bool s_irq_on = false;
static absolute_time_t s_irq_next;            // 次に読む時刻（割り込みの中だけで進める）
static volatile uint32_t s_irq_last_us = 0;   // 最後に読んだ時刻。0 = 次の 1 回は間隔を数えない
static volatile uint32_t s_irq_gap_max_us = 0;   // 読み出しの間隔の最長（60 秒ログの gapmax=）
static uint32_t s_drain_lat_max_us = 0;       // 読んでから推定へ渡すまでの最長（60 秒ログの drain=）。スレッド側だけ
static uint64_t s_req[REG_TEMP + 1];          // 要求フレーム（CRC 込み）を先に作っておく。割り込みで計算しない

// 割り込みの中の 1 フレーム送受信。xfer48() と同じ形を pico-sdk で直に書く
// （Arduino の SPI1 は beginTransaction などで状態を持つので、割り込みからは呼ばない）。
// ★ SPI の設定（5MHz・モード 0・8bit）は sch_init() の SPI1.beginTransaction() が入れたものをそのまま使う。
static inline uint64_t xfer48_irq(uint64_t frame) {
    uint8_t tx[6], rx[6];
    for (int i = 0; i < 6; i++) tx[i] = (uint8_t)(frame >> (8 * (5 - i)));
    gpio_put(SCH16T_PIN_CS, 0);
    busy_wait_us_32(1);                   // CS 立ち下がり → データ有効まで 40ns。余裕を取る
    spi_write_read_blocking(spi1, tx, rx, 6);
    gpio_put(SCH16T_PIN_CS, 1);
    busy_wait_us_32(1);                   // CS の High 期間を必ず作る
    uint64_t r = 0;
    for (int i = 0; i < 6; i++) r = (r << 8) | rx[i];
    return r;
}

// 4ms ごとのアラーム（Core0・優先度は最低）。7〜8 フレームで約 0.3ms。
static void imu_alarm_cb(uint alarm_num) {
    if (!s_irq_on) return;
    // ★ 次の目標を先に積む（読み出しの時間に引きずられて周期がずれないように）。
    //   ほかの割り込みに長く割り込まれて目標を過ぎていたら、いまから数え直す（追いつこうとして連打しない）。
    s_irq_next = delayed_by_us(s_irq_next, IMU_POLL_INTERVAL_US);
    if (absolute_time_diff_us(get_absolute_time(), s_irq_next) < 200)
        s_irq_next = delayed_by_us(get_absolute_time(), IMU_POLL_INTERVAL_US);
    while (hardware_alarm_set_target(alarm_num, s_irq_next))   // true = もう過ぎていた
        s_irq_next = delayed_by_us(get_absolute_time(), IMU_POLL_INTERVAL_US);

    const uint16_t head = s_ring_head;
    const uint16_t next = (uint16_t)((head + 1) % SCH16T_RING_LEN);
    if (next == s_ring_tail) {            // 一杯。スレッドが受け取りに来ていない
        s_ring_ovf++;
        return;
    }

    static uint8_t temp_skip = 0;
    const bool do_temp = (++temp_skip >= SCH16T_TEMP_EVERY);
    if (do_temp) temp_skip = 0;

    RawSample &r = s_ring[head];
    (void)xfer48_irq(s_req[REG_RATE_X2]);                    // 捨てフレーム（read_sample と同じ並び）
    r.f[0] = xfer48_irq(s_req[REG_RATE_Y2]);                 // → RATE_X2
    r.f[1] = xfer48_irq(s_req[REG_RATE_Z2]);                 // → RATE_Y2
    r.f[2] = xfer48_irq(s_req[REG_ACC_X2]);                  // → RATE_Z2
    r.f[3] = xfer48_irq(s_req[REG_ACC_Y2]);                  // → ACC_X2
    r.f[4] = xfer48_irq(s_req[REG_ACC_Z2]);                  // → ACC_Y2
    r.f[5] = xfer48_irq(s_req[do_temp ? REG_TEMP : REG_ACC_Z2]);   // → ACC_Z2
    r.ft = do_temp ? xfer48_irq(s_req[REG_TEMP]) : 0;        // → TEMP
    r.has_temp = do_temp ? 1 : 0;
    const uint32_t t_us = time_us_32();
    r.t_us = t_us;

    // ★ 実際に読めた間隔の最長（60 秒ログの gapmax=）。割り込みなので、ふだんは 4ms ちょうど。
    //   伸びていたら、割り込みが長く止められていた（フラッシュの書き込み・リングが一杯）。
    if (s_irq_last_us) {
        const uint32_t gap = t_us - s_irq_last_us;
        if (gap > s_irq_gap_max_us) s_irq_gap_max_us = gap;
    }
    s_irq_last_us = t_us;

    __compiler_memory_barrier();          // 中身を書き終えてから head を進める
    s_ring_head = next;
}

// 割り込みを止める。★ スレッド側（Core0）から呼ぶ。同じコアなので、戻った時点で割り込みの途中ということは無い。
static void irq_stop() {
    s_irq_on = false;
    if (s_alarm >= 0) hardware_alarm_cancel((uint)s_alarm);
}

// 割り込みを始める（sch_init() が通ったあと）。取れるアラームが無い・Core0 でないときは false（ポーリングに戻る）。
static bool irq_start() {
    if (get_core_num() != 0) return false;            // 推定も SPI1 の初期化も Core0。割り込みも Core0 で回す
    if (s_alarm < 0) {
        s_alarm = hardware_alarm_claim_unused(false);
        if (s_alarm < 0) return false;
        hardware_alarm_set_callback((uint)s_alarm, imu_alarm_cb);   // この呼び出しで Core0 の割り込みになる
        irq_set_priority(hardware_alarm_get_irq_num((uint)s_alarm), PICO_LOWEST_IRQ_PRIORITY);
        for (unsigned a = 0; a <= REG_TEMP; a++) s_req[a] = build_read_frame((uint8_t)a);
    }
    s_ring_head = s_ring_tail = 0;        // 割り込みは止まっているので素直に戻してよい
    s_irq_last_us = 0;
    s_irq_on = true;
    s_irq_next = delayed_by_us(get_absolute_time(), IMU_POLL_INTERVAL_US);
    while (hardware_alarm_set_target((uint)s_alarm, s_irq_next))
        s_irq_next = delayed_by_us(get_absolute_time(), IMU_POLL_INTERVAL_US);
    return true;
}

// 溜まったサンプルを古い順に推定へ渡す。★ スレッド側（Core0）から呼ぶ。
// 推定の中から imu_service_if_due() が呼ばれても入れ子にしない。
static void drain_ring() {
    static bool s_draining = false;
    if (s_draining) return;
    s_draining = true;
    for (;;) {
        const uint16_t tail = s_ring_tail;
        if (tail == s_ring_head) break;
        __compiler_memory_barrier();      // head を見てから中身を読む
        const RawSample &r = s_ring[tail];
        const uint32_t lat = time_us_32() - r.t_us;
        if (lat > s_drain_lat_max_us) s_drain_lat_max_us = lat;
        (void)take_sample(r.f, r.has_temp ? &r.ft : nullptr, r.t_us);
        __compiler_memory_barrier();      // 読み終えてから tail を進める（割り込みが上書きしてよいのはその後）
        s_ring_tail = (uint16_t)((tail + 1) % SCH16T_RING_LEN);
    }
    s_draining = false;
}
#endif  // SCH16T_READ_IN_IRQ

// 復旧。★ 1 分のレート制限（IMU_RECOVERY_INTERVAL_MS）。
static void try_recovery() {
    // ★ **最初から未接続なら試みない**（上の s_ever_alive のコメント）。
    if (!s_ever_alive) return;
    const uint32_t now_ms = millis();
    if (s_recover_last_ms > 0 && now_ms - s_recover_last_ms < IMU_RECOVERY_INTERVAL_MS) return;
    s_recover_last_ms = now_ms;
    DEBUGW_PLN(20261014, "[IMU] SCH16T recovery attempt");
    // ★ SPI は I2C と違ってバスが固まらない（クロックはこちらが出す）ので、
    //   バスの復旧処理は要らない。リセットから入れ直すだけ。
#if SCH16T_READ_IN_IRQ
    irq_stop();                  // ★ sch_init() はスレッド側で SPI1 を使う。割り込みと取り合わない
#endif
    if (sch_init()) {
        s_present = true;
        s_last_sample_ms = millis();
        // ★ 読み出しの間隔とレートの計測も起点から取り直す。途絶えている間は止まっていたので、残すと
        //   最初の 1 回が途絶えていた時間（1 分以上）を gapmax= に入れ、レートも ~0Hz と出た（2026-11-10 まで）。
        s_poll_last_us = 0;                    // 0 = 次の 1 回は間隔を数えない（imu_drv_service_if_due）
        s_hz_last_ms   = (uint32_t)millis();
        s_gyro_cnt = s_accel_cnt = 0;
        imu_feed_reset();        // 溜めた比力を捨てる（途絶前の古いサンプルを混ぜない）
#if SCH16T_READ_IN_IRQ
        if (!irq_start()) enqueueTask(createLogSdTask("[IMU] SCH16T timer IRQ unavailable, polling"));
#endif
        enqueueTask(createLogSdTask("[IMU] SCH16T recovery OK"));
        DEBUGW_PLN(20261014, "[IMU] SCH16T recovery OK");
    } else {
        enqueueTask(createLogSdTask("[IMU] SCH16T recovery FAILED"));
    }
}

// ============================================================
//  名前
// ============================================================
const char* imu_sensor_name() { return "SCH16T"; }

// ============================================================
//  imu_drv_*
// ============================================================
void imu_drv_setup() {
    // ★ ピンが UNSET なら何もしない。存在しない GPIO を叩かないため。
    if (SCH16T_PIN_CS == SCH16T_PIN_UNSET) {
        enqueueTask(createLogSdTask("[IMU] SCH16T pins unset (check settings.h)"));
        DEBUGW_PLN(20261014, "[IMU] SCH16T pins are UNSET — check settings.h");
        return;
    }

    pinMode(SCH16T_PIN_CS, OUTPUT);
    digitalWrite(SCH16T_PIN_CS, HIGH);          // ★ 負論理。未選択で始める
    pinMode(SCH16T_PIN_EXTRESN, OUTPUT);
    digitalWrite(SCH16T_PIN_EXTRESN, HIGH);     // ★ 負論理。Low でリセット
    // ★ **DRY_SYNC を出力にしないこと。** SYNC 入力との兼用ピンで、こちらが駆動すると
    //   DRY が出せなくなる。プルダウンは、DRY_DRV_EN が立つまで誰も駆動しておらず、
    //   プル無しだとパッドが中間電位に張り付くため（DRY は high-active の push-pull）。
    pinMode(SCH16T_PIN_DRY, INPUT_PULLDOWN);

    // ★ **SPI1**（`SPI` は spi0）。GPIO46 は SPI1_SCK の代替機能でもあるので
    //   setSCK(42) を明示する。41/42/43/44 は SPI1 の正規割り当て。
    SPI1.setRX(SCH16T_PIN_MISO);
    SPI1.setSCK(SCH16T_PIN_SCK);
    SPI1.setTX(SCH16T_PIN_MOSI);
    SPI1.begin(false);                          // CS はこちらで叩く

    // ★ **起動時は 2 回まで試す。** 単独テストの sch_init も同じ（Murata の begin() も
    //   失敗したらリセットして 1 回やり直す）。立ち上がりのタイミングが絡むので、
    //   1 回目の失敗が最終判定になると取りこぼす。
    //   ここで失敗すると s_ever_alive が立たず、以降の復旧も試みない
    //   （載っていない機体で 300ms 止まるのを避けるため）。
    for (int attempt = 0; attempt < 2 && !s_present; attempt++) {
        // ★ 中括弧を省かないこと。RELEASE では DEBUGW_* が空に展開されるので
        //   `if (x) ;` になり -Wempty-body が出る（意図も読めなくなる）。
        if (attempt > 0) { DEBUGW_PLN(20261014, "[IMU] SCH16T init retry"); }
        s_present = sch_init();
    }
    if (s_present) {
        s_ever_alive = true;
        s_last_sample_ms = millis();
        s_hz_last_ms = (uint32_t)millis();
#if SCH16T_READ_IN_IRQ
        // ★ ここから割り込みで読み始める。setup() の残り（起動画面など）の間も読み、
        //   imu_service_if_due() が呼ばれたときに推定へ渡す（受け取りが遅れた分は ovf に数えない。imu_drv_poll）。
        if (irq_start()) {
            enqueueTask(createLogSdfTask("[IMU] SCH16T read by timer IRQ (alarm %d, %uus, ring %u)",
                                         s_alarm, (unsigned)IMU_POLL_INTERVAL_US, (unsigned)SCH16T_RING_LEN));
        } else {
            enqueueTask(createLogSdTask("[IMU] SCH16T timer IRQ unavailable, polling"));
        }
#endif
    }
}

ImuDrvState imu_drv_poll() {
    // ★★ **「データが来ない」と「こちらが見ていなかった」を区別する。**
    //   v7 で実際に踏んだ穴（生きているチップをリセットしに行き、復旧に失敗した）。
    //   起動時は setup() が長く、loop() に入るまで imu_update() が呼ばれない。
    //   その状態で最初の 1 回を迎えると、生きているチップを「途絶」と誤判定して
    //   リセットしに行く。**呼び出し間隔が空いていたら、その回の判定は見送る。**
    {
        static unsigned long last_call_ms = 0;
        const unsigned long now_ms = millis();
        const bool resumed = (last_call_ms == 0) || (now_ms - last_call_ms > 500UL);
        last_call_ms = now_ms;
        if (resumed && s_last_sample_ms > 0) s_last_sample_ms = now_ms;
    }

    // ---- 通信途絶の検出 ----
    if (s_present && s_last_sample_ms > 0 && millis() - s_last_sample_ms > 1000UL) {
        DEBUGW_PLN(20261014, "[IMU] SCH16T comm timeout, marking unavailable");
        enqueueTask(createLogSdTask("[IMU] SCH16T comm timeout"));
        // 途絶時は s_present を落として復帰（try_recovery）を試行する（readiness 側でも NOT READY を正しく判定）
        s_present = false;
        // ★ 受信レートも 0 にする（2026-11-14、BUG-116）。下の 1 秒ごとの計算は途絶の間は通らないので、
        //   落とさないと 60 秒ログの raw GYR / ACC に途絶える前の 250Hz が出続けた
        s_gyro_hz = s_accel_hz = 0.0f;
        s_gyro_cnt = s_accel_cnt = 0;
#if SCH16T_READ_IN_IRQ
        irq_stop();              // 黙ったチップを読み続けない（復旧が通ったら try_recovery が始め直す）
#endif
    }

    if (!s_present) {
        try_recovery();           // 中で s_ever_alive と 1 分のレート制限を見る
        return IMU_DRV_DEAD;      // 恒速モデルへの縮退は呼び出し側（imu.cpp）が行う
    }

#if SCH16T_READ_IN_IRQ
    const bool irq_mode = s_irq_on;
#else
    const bool irq_mode = false;
#endif
    if (irq_mode) {
#if SCH16T_READ_IN_IRQ
        // ★ 起動処理（setup()）の間に受け取りが遅れて一杯になった分は数えない。loop() に入ってからの分だけ。
        static bool s_first_poll = true;
        if (s_first_poll) {
            s_first_poll = false;
            drain_ring();
            s_ring_ovf = 0;
            s_drain_lat_max_us = 0;
        }
        drain_ring();
#endif
    } else {
        const uint32_t now_us = time_us_32();
        if (now_us - s_poll_last_us < IMU_POLL_INTERVAL_US) return IMU_DRV_WAIT;
        imu_drv_service_if_due();
    }

    // ---- 1 秒ごとにレートを計算（60 秒ログの raw GYR / ACC）----
    {
        const uint32_t now_ms = (uint32_t)millis();
        const uint32_t elapsed = now_ms - s_hz_last_ms;
        if (elapsed >= 1000UL) {
            const float dt_s = elapsed * 0.001f;
            s_gyro_hz  = s_gyro_cnt  / dt_s;
            s_accel_hz = s_accel_cnt / dt_s;
            s_gyro_cnt = s_accel_cnt = 0;
            s_hz_last_ms = now_ms;
            // ★ 最小推奨読み出しレートを下回ったら警告する。下回ると
            //   フィルター阻止域のノイズが折り返して帯域内に入る（settings.h）。
            if (s_gyro_hz > 0.0f && s_gyro_hz < (float)SCH16T_MIN_READ_HZ) {
                DEBUGW_P(20261014, "[IMU] SCH16T read rate too low: ");
                DEBUGW_P(20261014, s_gyro_hz);
                DEBUGW_PLN(20261014, " Hz (noise folds into band)");
            }
        }
    }

    // 割り込みのときは、前の回から 1 つも受け取っていなかったら WAIT（predict へ進んでも新しい比力が無い）。
    // ★ 「この回に受け取ったか」で見ないこと。描画の途中の imu_service_if_due() が先に受け取っていると、
    //   直後のここは 0 件になり、バリオの predict が 1 周遅れる。
    static uint32_t s_taken_seen = 0;
    if (irq_mode) {
        if (s_taken == s_taken_seen) return IMU_DRV_WAIT;
        s_taken_seen = s_taken;
    }
    return IMU_DRV_SERVICED;
}

void imu_drv_service_if_due() {
    if (!s_present) return;
#if SCH16T_READ_IN_IRQ
    // 割り込みで読んでいるときは、溜まった分を推定へ渡すだけ（読みには行かない）
    if (s_irq_on) { drain_ring(); return; }
#endif
    const uint32_t now_us = time_us_32();
    if ((uint32_t)(now_us - s_poll_last_us) < IMU_POLL_INTERVAL_US) return;
    // ★ **実際に引き取れた間隔**の最長を記録する（60 秒ログの gapmax=）。
    //   ここが数十 ms に伸びたら、どこかで Core0 が飢えている。
    if (s_poll_last_us) {
        const uint32_t gap = now_us - s_poll_last_us;
        if (gap > s_poll_gap_max_us) s_poll_gap_max_us = gap;
    }
    s_poll_last_us = now_us;

    // ★ SCH16T は**こちらが取りに行く**方式なので、溜まったデータを掃くループは
    //   要らない（読まなければ出力レジスタが上書きされるだけ）。1 回で 1 サンプル。
    (void)read_sample();
}

bool imu_drv_present() { return s_present; }

bool imu_drv_alive() {
    if (!s_present) return false;
    if (s_last_sample_ms == 0) return false;
    return (millis() - s_last_sample_ms <= 1000UL);
}

// 生ジャイロ・加速度の受信レート [Hz]（60 秒ログの raw GYR / ACC）
float get_imu_gyro_hz()  { return s_gyro_hz; }
float get_imu_accel_hz() { return s_accel_hz; }

// ★ 健全性（CRC エラー・CE・飽和・STAT_SUM の故障ビット）は**いまは数えるだけでどこへも出していない**
//   （上の s_crc_err の宣言のコメント）。**画面と 60 秒ログへ繋ぐのは docs/imu_sch16t_plan.md §2.7 の作業。**

// 読み出しの間隔の最長 [µs]（60 秒ログの gapmax=）。★ **読むたびに 0 へ戻す約束**。
//   割り込みで読んでいるときは割り込みの間隔（ふだん 4ms ちょうど）、ポーリングのときは Core0 が止まった最長時間。
uint32_t get_imu_poll_gap_max_us() {
#if SCH16T_READ_IN_IRQ
    if (s_irq_on) {
        const uint32_t save = save_and_disable_interrupts();   // 割り込みが書く値なので、読みと 0 戻しの間を守る
        const uint32_t v = s_irq_gap_max_us;
        s_irq_gap_max_us = 0;
        restore_interrupts(save);
        return v;
    }
#endif
    const uint32_t v = s_poll_gap_max_us;
    s_poll_gap_max_us = 0;
    return v;
}

// 読んでから推定へ渡すまでの最長 [µs]（60 秒ログの drain=）。★ 読むたびに 0 へ戻す。
// 割り込みで読んでいるときだけ意味がある（= Core0 が受け取りに来られなかった最長時間。地図の描き直しなど）。
uint32_t get_imu_drain_max_us() {
#if SCH16T_READ_IN_IRQ
    const uint32_t v = s_drain_lat_max_us;
    s_drain_lat_max_us = 0;
    return v;
#else
    return 0;
#endif
}

// リングが一杯で読まなかった回数（起動してから。60 秒ログの ovf=）。ポーリングのときは 0。
uint32_t get_imu_ring_ovf() {
#if SCH16T_READ_IN_IRQ
    return s_ring_ovf;
#else
    return 0;
#endif
}

// 割り込みで読んでいるか（60 秒ログの行の形を決める）。
bool imu_read_in_irq() {
#if SCH16T_READ_IN_IRQ
    return s_irq_on;
#else
    return false;
#endif
}

