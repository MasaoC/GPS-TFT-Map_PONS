// ============================================================
// File    : imu_sch16t.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : SCH16T-K01 ドライバ（SPI1 / SafeSPI2 48bit アウトオブフレーム）。
//           imu_sensor.h の契約を実装する。チップ依存はこのファイルだけ。
//
// ■ このチップは「ジャイロと加速度しか出さない」
//   BNO085 と違って融合が無いので、姿勢は attitude.cpp の ESKF、重力除去は
//   バリオ KF が自分でやる（0.983〜0.984 で依存を外した）。
//   したがって **imu_caps() は融合系を全部 0 にする**。画面とログはそれで縮退する。
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
// ★ **ファイル全体が #if IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T で囲ってある。**
//   Arduino はスケッチ直下の .cpp を全部コンパイルするので、囲わないと
//   imu_bno08x.cpp と imu_drv_* が多重定義になる。**外さないこと。**
//
// ★ **本体基板では一度も動かしていない**（0.991 時点）。
//   docs/imu_sch16t_bringup.md の Step 2〜5 を通すまで飛ばさないこと。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/14
// ============================================================

#include <Arduino.h>
#include "settings.h"

#if IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T

#include <SPI.h>
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
//   AUX 窓に効く。BNO085 側も同じ理由で「最初から未接続なら復旧しない」にしてある）。
static bool     s_ever_alive = false;
static uint32_t s_last_sample_ms = 0;       // 最後にサンプルを引き取った時刻
static uint32_t s_poll_last_us = 0;
static uint32_t s_poll_gap_max_us = 0;
static uint32_t s_recover_last_ms = 0;

// レート計測（60 秒ログの raw GYR / ACC）
static uint32_t s_hz_last_ms = 0;
static uint16_t s_gyro_cnt = 0, s_accel_cnt = 0;
static float    s_gyro_hz = 0.0f, s_accel_hz = 0.0f;

// 健全性カウンタ。★ SCH16T では GRV/RV が無いので、見るのはこちら。
//   いまは 60 秒ログではなくシリアルと SD ログへ出している
//   （画面と 60 秒ログへ繋ぐのは docs/imu_sch16t_plan.md §2.7 の作業）。
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
    for (;;) {
        const uint64_t f = reg_read(REG_ASIC_ID);
        const uint16_t v = frame_data_u16(f);
        // ★ 待っている間の CRC/CE 不一致は**当然**なので健全性カウンタへ入れない。
        const uint32_t crc0 = s_crc_err, ce0 = s_ce_err;
        const bool ok = frame_integrity_ok(f);
        s_crc_err = crc0; s_ce_err = ce0;
        if (ok && v != 0x0000 && v != 0xFFFF) return (int32_t)(millis() - t0);
        if (millis() - t0 >= timeout_ms) return -1;
        delay(2);
    }
}

// CTRL_USER_IF は **読んでから必要なビットだけ立てて書き戻す**。
// ★★ 素で書くとリセット値で 1 のビット（FTREE_TDEL / MISO_SR_CTRL / DRY_SR_CTRL。
//   リセット値 0x200C。実機で確認済み）を潰す。データシートが
//   "This option is not supported by Murata" と書いている側の設定になる。
//   **PX4 のドライバはこれを踏んでいる**ので、あちらを参考にするときは注意。
static bool set_dry(bool enable) {
    const uint16_t before = frame_data_u16(reg_read(REG_CTRL_USER_IF));
    uint16_t v = before;
    if (enable) v |= BIT_DRY_DRV_EN; else v &= (uint16_t)~BIT_DRY_DRV_EN;
    v &= (uint16_t)~BIT_DRY_POL_LOW;      // high active
    reg_write(REG_CTRL_USER_IF, v);
    return frame_data_u16(reg_read(REG_CTRL_USER_IF)) == v;
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
        DEBUGW_PLN(20261014, "[IMU] SCH16T: ASIC_ID unreadable (check J7 wiring / SPI1 pins)");
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
    const uint16_t cr = frame_data_u16(reg_read(REG_CTRL_RATE));
    const uint16_t ca = frame_data_u16(reg_read(REG_CTRL_ACC12));
    const bool dyn_ok = (((cr >> 12) & 7) == (SCH16T_RATE_DYN_SEL & 7)) &&
                        (((cr >>  9) & 7) == (SCH16T_RATE_DYN_SEL & 7)) &&
                        (((ca >> 12) & 7) == (SCH16T_ACC12_DYN_SEL & 7)) &&
                        (((ca >>  9) & 7) == (SCH16T_ACC12_DYN_SEL & 7));
    const uint16_t filt = frame_data_u16(reg_read(REG_CTRL_FILT_RATE));
    const bool filt_ok = (filt == build_filt(SCH16T_FILT_SEL));
    const uint16_t sum = frame_data_u16(reg_read(REG_STAT_SUM));
    const bool stat_ok = (sum == 0xFFFF);
    if (!stat_ok) s_stat_bad++;

    if (!(dyn_ok && filt_ok && stat_ok)) {
        enqueueTask(createLogSdfTask("[IMU] SCH16T verify NG dyn=%d filt=%d STAT_SUM=0x%04X",
                                     (int)dyn_ok, (int)filt_ok, (unsigned)sum));
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
// デシメーション出力（*_2）を 1 サンプル読む。
// ★ **先頭に捨てフレームが 1 つ必要**（アウトオブフレーム）。全部で 8 フレーム。
//   各応答は「1 つ前に要求したレジスタ」の値なので、要求を 1 つずらして並べる。
// ★ **温度は毎回読まない**（間隔は settings.h の SCH16T_TEMP_EVERY）。
//   1 サンプルあたりのフレーム数が 8 → 7 に減る。
// ★ s_temp_c はいま**書くだけで誰も読んでいない**。画面と 60 秒ログへ出すのは
//   docs/imu_sch16t_plan.md §2.7 の作業で、そのときの受け皿として置いてある。
static float s_temp_c = 0.0f;      // 最後に読めたダイ温度 [℃]

static bool read_sample() {
    static uint8_t temp_skip = 0;
    const bool do_temp = (++temp_skip >= SCH16T_TEMP_EVERY);
    if (do_temp) temp_skip = 0;

    // ★ **先頭の捨てフレームを「前回の続き」で省いてはいけない。** アウトオブフレームの
    //   パイプラインを呼び出しをまたいで繋げば 6 フレームで済むが、描画で 40ms 空いた
    //   あとに**40ms 前の値を新しい値として受け取る**ことになる（静かに壊れる形）。
    //   毎回捨てフレームから始める。
    req_read(REG_RATE_X2);                       // 捨てフレーム
    const uint64_t fgx = req_read(REG_RATE_Y2);  // → RATE_X2
    const uint64_t fgy = req_read(REG_RATE_Z2);  // → RATE_Y2
    const uint64_t fgz = req_read(REG_ACC_X2);   // → RATE_Z2
    const uint64_t fax = req_read(REG_ACC_Y2);   // → ACC_X2
    const uint64_t fay = req_read(REG_ACC_Z2);   // → ACC_Y2
    // 最後の 1 本は ACC_Z2 の値を回収するための要求。アドレスは「次に読みたいもの」で、
    // 温度を読む回だけ TEMP にして、もう 1 フレーム足してその値を取る。
    const uint64_t faz = req_read(do_temp ? REG_TEMP : REG_ACC_Z2);   // → ACC_Z2
    if (do_temp) {
        const uint64_t ft = req_read(REG_TEMP);  // → TEMP
        // ★ 温度は 16bit 幅で 20bit フィールドの**上位に詰まっている**ので >>4。
        if (frame_sample_ok(ft)) s_temp_c = (frame_data_i20(ft) >> 4) / SCH16T_LSB_PER_DEGC;
    }

    // ★ **`&&` で繋いではいけない。** frame_sample_ok() は健全性カウンタを増やす
    //   副作用があるので、短絡すると 1 本目で落ちた時点で残りが数えられない。
    bool ok = true;
    ok &= frame_sample_ok(fgx); ok &= frame_sample_ok(fgy); ok &= frame_sample_ok(fgz);
    ok &= frame_sample_ok(fax); ok &= frame_sample_ok(fay); ok &= frame_sample_ok(faz);
    if (!ok) return false;

    const uint32_t t_us = time_us_32();

    // ジャイロ [LSB] → [rad/s]。
    // ★ **バイアスを引かずに渡す。** ESKF が自分で推定する（imu_sensor.h の約束）。
    const float gx = frame_data_i20(fgx) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    const float gy = frame_data_i20(fgy) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    const float gz = frame_data_i20(fgz) / SCH16T_LSB_PER_DPS * (float)DEG_TO_RAD;
    // 加速度 [LSB] → [m/s²]。★ **重力を含んだままの比力を渡す。**
    const float ax = frame_data_i20(fax) / SCH16T_LSB_PER_MPS2;
    const float ay = frame_data_i20(fay) / SCH16T_LSB_PER_MPS2;
    const float az = frame_data_i20(faz) / SCH16T_LSB_PER_MPS2;

    // ★ 生ログの s_us には**受信時刻**を入れる。SCH16T はサンプルに
    //   タイムスタンプを付けないので、BNO085 の `sv.timestamp` 相当が存在しない。
    //   acc（申告精度）も無いので 0 固定。decode_imulog.py はどちらも
    //   「チップ固有」として扱うので、ID は GYRO/ACCEL をそのまま使ってよい。
    const float g[3] = { gx, gy, gz };
    const float a[3] = { ax, ay, az };
    // ★ **feed は毎サンプル、ログは間引く**（SCH16T_IMULOG_DECIM。理由は settings.h）。
    //   推定は 250Hz で回したいが、その全数を SD へ落とすと予算を超える。
    static uint8_t log_skip = 0;
    const bool do_log = (++log_skip >= SCH16T_IMULOG_DECIM);
    if (do_log) log_skip = 0;
    if (do_log) imulog_push(IMULOG_ID_GYRO,  t_us, 0, gx, gy, gz);
    imu_feed_gyro(g, t_us);
    if (do_log) imulog_push(IMULOG_ID_ACCEL, t_us, 0, ax, ay, az);
    imu_feed_accel(a);

    s_gyro_cnt++;
    s_accel_cnt++;
    s_last_sample_ms = millis();
    s_ever_alive = true;
    return true;
}

// 復旧。★ 1 分のレート制限（BNO085 の imu_try_recovery と同じ流儀）。
static void try_recovery() {
    // ★ **最初から未接続なら試みない**（上の s_ever_alive のコメント）。
    if (!s_ever_alive) return;
    const uint32_t now_ms = millis();
    if (s_recover_last_ms > 0 && now_ms - s_recover_last_ms < IMU_RECOVERY_INTERVAL_MS) return;
    s_recover_last_ms = now_ms;
    DEBUGW_PLN(20261014, "[IMU] SCH16T recovery attempt");
    // ★ SPI は I2C と違ってバスが固まらない（クロックはこちらが出す）ので、
    //   BNO085 の imu_i2c_bus_recover() 相当は要らない。リセットから入れ直すだけ。
    if (sch_init()) {
        s_present = true;
        s_last_sample_ms = millis();
        imu_feed_reset();        // 溜めた比力を捨てる（途絶前の古いサンプルを混ぜない）
        enqueueTask(createLogSdTask("[IMU] SCH16T recovery OK"));
        DEBUGW_PLN(20261014, "[IMU] SCH16T recovery OK");
    } else {
        enqueueTask(createLogSdTask("[IMU] SCH16T recovery FAILED"));
    }
}

// ============================================================
//  能力ビット
// ============================================================
// ★ 融合出力（QUAT / MAGYAW / LINACC / CAL）はこのチップに無いので全部 0。
//   これを 0 にしてあるおかげで、画面とログは自動的に "---" へ縮退する。
// ★ IMU_CAP_STATUS を立てる。S1:S0 / CE / STAT_SUM を実際に読んで判定に使っている
//   （frame_sample_ok / sch_init）。
// ★ **ただし外へ出す accessor はまだ無い。** いまこのビットを読んでいる箇所も無い。
//   故障ビット・CRC エラー数・飽和回数を画面と 60 秒ログへ出すのは
//   docs/imu_sch16t_plan.md §2.7 の作業。**「立っているから accessor がある」と
//   思って書かないこと**（s_crc_err などは static のまま）。
uint32_t imu_caps() {
    return IMU_CAP_STATUS & IMU_CAPS_DEBUG_MASK;
}

const char* imu_sensor_name() { return "SCH16T"; }

// ============================================================
//  imu_drv_*
// ============================================================
void imu_drv_setup() {
    // ★ ピンが UNSET（= v7 基板）なら何もしない。存在しない GPIO を叩かないため。
    if (SCH16T_PIN_CS == SCH16T_PIN_UNSET) {
        enqueueTask(createLogSdTask("[IMU] SCH16T pins unset (wrong PONS_BOARD?)"));
        DEBUGW_PLN(20261014, "[IMU] SCH16T pins are UNSET — check PONS_BOARD");
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
    //   （BNO085 と同じ約束。載っていない機体で 300ms 止まるのを避けるため）。
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
    }
}

ImuDrvState imu_drv_poll() {
    // ★★ **「データが来ない」と「こちらが見ていなかった」を区別する。**
    //   BNO085 で踏んだのと同じ穴（imu_bno08x.cpp の同じ位置のコメント参照）。
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
        s_present = false;
    }

    if (!s_present) {
        try_recovery();           // 中で s_ever_alive と 1 分のレート制限を見る
        return IMU_DRV_DEAD;      // 恒速モデルへの縮退は呼び出し側（imu.cpp）が行う
    }

    const uint32_t now_us = time_us_32();
    if (now_us - s_poll_last_us < IMU_POLL_INTERVAL_US) return IMU_DRV_WAIT;

    imu_drv_service_if_due();

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

    return IMU_DRV_SERVICED;
}

void imu_drv_service_if_due() {
    if (!s_present) return;
    const uint32_t now_us = time_us_32();
    if ((uint32_t)(now_us - s_poll_last_us) < IMU_POLL_INTERVAL_US) return;
    // ★ **実際に引き取れた間隔**の最長を記録する（60 秒ログの gapmax=）。
    //   ここが数十 ms に伸びたら、どこかで Core0 が飢えている。
    if (s_poll_last_us) {
        const uint32_t gap = now_us - s_poll_last_us;
        if (gap > s_poll_gap_max_us) s_poll_gap_max_us = gap;
    }
    s_poll_last_us = now_us;

    // ★ SCH16T は**こちらが取りに行く**方式なので、BNO085 のような
    //   「溜まったレポートを掃く」ループは要らない（読まなければ DRY が立ったまま
    //   になるだけで、レポートが溢れて化けることがない）。1 回で 1 サンプル。
    //   **ただし BNO085 側の飢餓対策コメントは消さないこと**（戻す可能性があるため。
    //   docs/imu_sch16t_plan.md §1）。
    (void)read_sample();
}

bool imu_drv_present() { return s_present; }

bool imu_drv_alive() {
    if (!s_present) return false;
    if (s_last_sample_ms == 0) return false;
    return (millis() - s_last_sample_ms <= 1000UL);
}

// VARIO_USE_RAW_ACCEL=0 の旧経路用。SCH16T は LACC を持たないので常に false。
bool imu_drv_lacc_predict_sample(float a_body[3], float q[4], uint32_t* lacc_last_us) {
    (void)a_body; (void)q; (void)lacc_last_us;
    return false;
}

// ============================================================
//  融合出力のゲッター — 無いので「無効値」を返す
// ============================================================
// ★ **0 をそのまま返さないものがある。** get_imu_euler() の 0,0,0 は
//   「水平で北向き」というもっともらしい値なので、呼び出し側が imu_caps() を
//   見ずに表示すると静かな誤表示になる。いまは全箇所が囲ってあるが、
//   **囲い忘れに備えて参照は必ず埋める**（未初期化のまま返さない）。
void get_imu_euler(float &roll, float &pitch, float &yaw) {
    roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
}
void get_imu_linaccel(float &ax, float &ay, float &az) {
    ax = 0.0f; ay = 0.0f; az = 0.0f;
}
float get_imu_mag_accuracy_deg() { return -1.0f; }   // -1 = 未受信（BNO085 と同じ約束）

// 融合レポートは存在しないので 0。生ジャイロ・加速度は実測値を返す。
float get_imu_grv_hz()   { return 0.0f; }
float get_imu_lacc_hz()  { return 0.0f; }
float get_imu_rv_hz()    { return 0.0f; }
float get_imu_gyro_hz()  { return s_gyro_hz; }
float get_imu_accel_hz() { return s_accel_hz; }

// ============================================================
//  校正と申告精度 — SCH16T には無い
// ============================================================
// ★ 自前のジャイロ ZRO 学習（attitude.cpp の learn_gyro_bias）が代わりを務める。
//   バイアスの温度係数は実測 約 −0.005 °/s/℃（docs/imu_sch16t_bringup.md §1）で、
//   ESKF がオンライン推定するので焼く必要はない。
bool     imu_cal_ready()       { return false; }
bool     imu_cal_saved_ever()  { return false; }
uint32_t imu_cal_saved_date()  { return 0; }
uint16_t imu_cal_save_count()  { return 0; }
bool     imu_cal_autosave_on() { return false; }
bool     imu_save_calibration(uint32_t yyyymmdd) { (void)yyyymmdd; return false; }

uint8_t  get_imu_acc_accuracy() { return 0; }
uint8_t  get_imu_gyr_accuracy() { return 0; }
uint8_t  get_imu_cal_cfg()      { return 0xFF; }     // 0xFF = 未取得（BNO085 と同じ約束）

// ============================================================
//  健全性カウンタ
// ============================================================
// ★ GRV / RV はこのチップに無いので 0。SCH16T 側で意味があるのは
//   CRC エラー・CE・飽和・STAT_SUM の故障ビットで、いまはシリアルと SD ログへ
//   出している。**画面と 60 秒ログへ繋ぐのは docs/imu_sch16t_plan.md §2.7 の作業。**
uint32_t get_imu_grv_bad()  { return 0; }
uint32_t get_imu_grv_jump() { return 0; }
uint32_t get_imu_rv_bad()   { return 0; }

// Core0 が止まった最長時間 [µs]。★ **読むたびに 0 へ戻す約束**（BNO085 と同じ）。
uint32_t get_imu_poll_gap_max_us() {
    const uint32_t v = s_poll_gap_max_us;
    s_poll_gap_max_us = 0;
    return v;
}

#endif  // IMU_SENSOR_DEFAULT == IMU_SENSOR_SCH16T
