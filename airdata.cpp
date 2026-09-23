// ============================================================
// File    : airdata.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 大気データ取得の実装。
//           気圧センサー MS5611（I2C接続）から気圧・気温を読み取り、
//           気圧高度を算出する。airdata_update() をループから毎回呼ぶ
//           ステートマシン方式で非ブロッキング動作する。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/17
// ============================================================
#include <Wire.h>
#include "airdata.h"
#include "link.h"
#include "mysd.h"
#include "gnss.h"    // replay_has_value / replay_get_pressure （リプレイ時の気圧差し替え）
// MS5611 の I2C アドレス（SDO=VCC の場合は 0x76）
#define MS5611_ADDR 0x77

// MS5611 コマンドバイト定義（データシート Table 1 より）
#define CMD_RESET      0x1E  // リセットコマンド（PROM 読み出し失敗時のリカバリーにだけ使う）
#define CMD_READ_PROM  0xA0  // PROM 読み出しコマンドの先頭アドレス。係数番号を左シフトして加算する
#define CMD_CONVERT_D1 0x48  // 気圧 ADC 変換コマンド（OSR=4096: 最高精度、変換時間 ≈ 9ms）
#define CMD_CONVERT_D2 0x58  // 温度 ADC 変換コマンド（OSR=4096）
#define CMD_ADC_READ   0x00  // ADC 変換結果読み出しコマンド

// OSR=4096 の最大変換時間は 9.04ms。余裕を持たせて 12ms 待つ。
// D1変換(12ms) + D2変換(12ms) = 24ms サイクル → 約 42Hz
#define OSR_DELAY_MS   12

// i2c0 バスを使用（SDA=GPIO32, SCL=GPIO33）。
// RP2350 の Wire ライブラリはデフォルトで i2c0 を使うが、ピン番号が異なるため明示的に指定。
TwoWire myWire(i2c0, 32, 33);

// MS5611 の工場出荷時キャリブレーション係数（PROM から読み出し）
// C[0]: 製造データ（未使用）, C[1]-C[6]: 温度・気圧補正用係数
uint16_t C[7];

// ----------------------------
// ステートマシン用 内部状態
// ----------------------------

// MS5611_IDLE   : 次の変換を開始できる状態
// MS5611_WAIT_D1: D1（気圧）変換完了待ち
// MS5611_WAIT_D2: D2（温度）変換完了待ち
enum MS5611State { MS5611_IDLE, MS5611_WAIT_D1, MS5611_WAIT_D2 };

static MS5611State ms5611_state  = MS5611_IDLE;
static uint32_t    convert_start = 0;    // 変換コマンド送信時刻 [ms]
static uint32_t    D1_raw        = 0;    // 気圧 ADC 生値（WAIT_D2 ステートで使用）
static uint32_t    last_D2_raw   = 0;    // D2（温度）ADC 生値

// airdata_update() が true を返すたびに更新される最新計測値
static float last_temperature = 0.0f;  // [℃]
static float last_pressure    = 0.0f;  // [hPa]
static float last_altitude    = 0.0f;  // [m]（VSPEED_WINDOW_MS ウィンドウ平均値）

// オーバーサンプリング ウィンドウ幅 [ms]。
// 短くすると vspeed の応答が速くなるがノイズが増える。
// 長くすると平滑化されるがバリオの反応が遅くなる。
#define VSPEED_WINDOW_MS 250

// 垂直速度 (vspeed) 計算用: ウィンドウ トリム平均によるオーバーサンプリング
// 24ms周期（D1+D2）のサンプルを VSPEED_WINDOW_MS 分バッファに蓄積し、挿入ソート後に
// 上下 2 サンプルずつ棄却したトリム平均で代表値を求める。
// プロペラ（135rpm 2枚ペラ）が ~4.5Hz で干渉して生じる周期的外れ値を除去するのが目的。
//
// ※ 棄却率は固定 2 サンプルなので、窓のサンプル数で変わる。
//   VSPEED_WINDOW_MS=250 では 250/24 ≒ 10 サンプルなので上下 20%。
//   窓幅を変えたらこの数字も変わることに注意（以前は 500ms 前提のコメントが残っていた）。
#define ALT_WIN_BUF_SIZE 32             // 24ms周期×250ms ≒ 10 サンプル、余裕込みで 32
static float alt_win_buf[ALT_WIN_BUF_SIZE]; // ウィンドウ内の高度サンプルバッファ
static int   alt_win_count = 0;             // バッファ内の有効サンプル数
static float alt_win_prev  = 0.0f;          // 直前ウィンドウのトリム平均高度 [m]（GND相対）
// 直前ウィンドウを閉じた時刻 [ms]。vspeed は「窓が隣り合っている」前提で
// (差分 ÷ 窓幅) としていたが、I2C が途絶えて窓が飛ぶとその前提が崩れ、
// 何秒も前の高度との差を 1 窓分の時間で割って**巨大な偽の昇降率**が出る。
// 実経過時間で割り、飛びすぎていたら更新しない。
static unsigned long alt_win_prev_ms = 0;
// ★ 「まだ 1 窓目」の判定に alt_win_prev != 0.0f を使ってはいけない。
//   0.0m は起動地点そのものを指す**正当な高度値**なので、番兵として不適切。
//   専用のフラグで持つ。
static bool  alt_win_prev_valid = false;
static float last_vspeed   = 0.0f;          // 最新の垂直速度 [m/s]
static unsigned long last_vspeed_update_ms = 0; // 最後に vspeed が正常更新された時刻 [ms]
                                                // I2C エラー等で途絶えた場合のタイムアウト判定に使う
static float last_win_hz      = 0.0f;       // 直前ウィンドウの総サンプル数から算出した更新レート [Hz]
static unsigned long win_start = 0;         // 現在ウィンドウ開始時刻 [ms]

// 起動時を 0m 基準とするグランドレベル
static float   ground_alt_abs  = 0.0f;  // 起動時の絶対高度（標準大気基準）[m]
static float   ground_pressure = 0.0f;  // 起動時の気圧 [hPa]（グランドレベル確定時点の計測値）
static bool    ground_set      = false; // グランドレベル確定済みフラグ

// MS5611 の接続・初期化が正常に完了しているかどうか
// airdata_setup() で設定され、airdata_update() / get_airdata_ok() で参照される
static bool ms5611_ok = false;
// 連続失敗回数。MS5611_FAIL_LIMIT を超えたら ms5611_ok を落として通信をやめる。
// 成功するたびに 0 に戻すので、たまの 1 回のエラーでは落ちない。
static int  ms5611_fail_count = 0;
// 最後に復旧を試みた時刻 [ms]（0 = まだ一度も試していない）
static unsigned long ms5611_recovery_ms = 0;

// ----------------------------
// I2C スキャン (デバッグ用)
// 0x01 ～ 0x7E のアドレスを総当たりで確認し、応答があったデバイスを Serial に出力する。
// MS5611 が認識されているか確認するときに airdata_setup() 内から手動で呼ぶ。
// ----------------------------
// ★ **RELEASE では中身ごと消す。** 出力は全部 DEBUG_* なので RELEASE では
//   何も表示されないのに、126 アドレスへ I2C トランザクションを投げて結果を捨てていた。
//   起動時間の無駄であり、-Wempty-body 警告（if (!found) DEBUG_PLN(...)）の出所でもあった。
void i2c_scan() {
#ifndef RELEASE
    DEBUG_PLN(20260310, "=== I2C Scan ===");
    bool found = false;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        myWire.beginTransmission(addr);
        uint8_t err = myWire.endTransmission();
        if (err == 0) {
            DEBUG_P(20260310, "Found: 0x");
            DEBUG_PNLN(20260310, addr, HEX);
            found = true;
        }
    }
    if (!found) DEBUG_PLN(20260310, "No devices found!");
    DEBUG_PLN(20260310, "================");
#endif  // !RELEASE
}

// ----------------------------
// 低レベル I2C 操作関数
// ----------------------------

// MS5611 に 1 バイトのコマンドを送信する。
// 戻り値: 成功=true、I2C エラー=false（エラーコードを Serial に出力）。
bool ms5611_write_cmd(uint8_t cmd) {
    myWire.beginTransmission(MS5611_ADDR);
    myWire.write(cmd);
    uint8_t err = myWire.endTransmission();
    if (err != 0) {
        DEBUGW_P(20260310, "I2C write error: ");
        DEBUGW_PLN(20260310, err);
        // ★ **毎ループ積まないこと。** この関数は airdata_update() の IDLE から
        //   毎ループ呼ばれる。センサーが無応答になるとログタスクが毎ループ積まれ、
        //   0.5 秒ほどでタスクキュー(40)が溢れて**以降の全タスクが捨てられる**
        //   ＝ 警報音が鳴らなくなる。気圧センサー 1 個の不調で音響警報が
        //   全停止するのは割に合わないので、ここで間引く。
        static unsigned long last_errlog_ms = 0;
        const unsigned long now_ms = millis();
        if (last_errlog_ms == 0 || now_ms - last_errlog_ms >= MS5611_ERRLOG_INTERVAL_MS) {
            last_errlog_ms = now_ms;
            enqueueTask(createLogSdfTask("MS5611 I2C write error: %d", err));
        }
        return false;
    }
    return true;
}

// PROM から指定番号（0〜6）のキャリブレーション係数を 2 バイト読み取る。
// コマンドは CMD_READ_PROM(0xA0) に (coef_num << 1) を加算して送る（データシート仕様）。
// val に読み取った 16-bit 値を格納する。戻り値: 成功=true。
bool ms5611_read_prom(uint8_t coef_num, uint16_t &val) {
    myWire.beginTransmission(MS5611_ADDR);
    myWire.write(CMD_READ_PROM + (coef_num << 1));
    uint8_t err = myWire.endTransmission();
    if (err != 0) {
        DEBUGW_P(20260310, "PROM write error: ");
        DEBUGW_PLN(20260310, err);
        enqueueTask(createLogSdfTask("MS5611 PROM write error: %d", err));
        return false;
    }

    uint8_t n = myWire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)2);
    if (n != 2) {
        DEBUGW_P(20260310, "PROM read error, got bytes: ");
        DEBUGW_PLN(20260310, n);
        enqueueTask(createLogSdfTask("MS5611 PROM read error: got %d bytes", n));
        return false;
    }
    val = (uint16_t)myWire.read() << 8;
    val |= myWire.read();
    return true;
}

// ADC 変換コマンドを送信する（変換完了の待機は含まない）。
// 非ブロッキング動作の起点。戻り値: 送信成功=true。
static bool ms5611_start_convert(uint8_t cmd) {
    return ms5611_write_cmd(cmd);
}

// 変換完了後に ADC 結果 3 バイトを読み取る。
// 必ず ms5611_start_convert() → OSR_DELAY_MS 以上待機 → この関数の順で呼ぶこと。
// val に 24-bit の生値を格納する。戻り値: 成功=true。
static bool ms5611_read_adc_result(uint32_t &val) {
    myWire.beginTransmission(MS5611_ADDR);
    myWire.write(CMD_ADC_READ);
    if (myWire.endTransmission() != 0) return false;
    uint8_t n = myWire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)3);
    if (n != 3) return false;
    val  = (uint32_t)myWire.read() << 16;
    val |= (uint32_t)myWire.read() << 8;
    val |=  myWire.read();
    return true;
}

// D1・D2 の生値からキャリブレーション補正済みの気温・気圧を計算する（データシート 4.9 章）。
//
// 変数名の意味（データシートに準拠）:
//   dT   : 実測温度と基準温度の差 = D2 - C[5]*256
//   tempd: 補正前温度 (100倍値) = 2000 + dT*C[6]/2^23
//   OFF  : ゼロ点オフセット     = C[2]*2^16 + C[4]*dT/2^7
//   SENS : 感度                 = C[1]*2^15 + C[3]*dT/2^8
//
// 低温補正（Second Order Temperature Compensation）:
//   tempd < 2000（＝20.00℃未満）のとき T2/OFF2/SENS2 を追加補正する。
//   さらに tempd < -1500（＝-15.00℃未満）では追加の補正項を加える。
//
// 最終気圧: P = (D1*SENS/2^21 - OFF) / 2^15  [単位: 0.01 hPa]
static void ms5611_calculate(uint32_t D1, uint32_t D2, float &temperature, float &pressure) {
    int32_t dT    = (int32_t)D2 - (int32_t)C[5] * 256L;
    int32_t tempd = 2000 + (int64_t)dT * C[6] / 8388608LL;

    int64_t OFF  = (int64_t)C[2] * 65536LL + (int64_t)C[4] * dT / 128LL;
    int64_t SENS = (int64_t)C[1] * 32768LL + (int64_t)C[3] * dT / 256LL;

    // 低温補正（20℃未満で追加補正を実施）
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (tempd < 2000) {
        T2    = (int64_t)dT * dT / 2147483648LL;
        OFF2  = 5LL * (tempd - 2000) * (tempd - 2000) / 2;
        SENS2 = 5LL * (tempd - 2000) * (tempd - 2000) / 4;
        if (tempd < -1500) {    // -15℃未満ではさらに追加補正
            OFF2  += 7LL * (tempd + 1500) * (tempd + 1500);
            SENS2 += 11LL * (tempd + 1500) * (tempd + 1500) / 2;
        }
    }

    tempd -= T2;
    OFF   -= OFF2;
    SENS  -= SENS2;

    int32_t P = ((int64_t)D1 * SENS / 2097152LL - OFF) / 32768LL;

    temperature = tempd / 100.0f;   // 0.01℃単位 → ℃
    pressure    = P    / 100.0f;    // 0.01hPa単位 → hPa
}

// 気圧から気圧高度を計算する（国際標準大気モデル）。
// 式: altitude = 44330 * (1 - (P / P0)^(1/5.255))
//   P0=1013.25hPa（標準海面気圧）、指数 0.1902949 = 1/5.255。
// sea_level_hpa をその日の QNH に設定するとより正確な高度が得られる。
float pressure_to_altitude(float pressure_hpa, float sea_level_hpa) {
    return 44330.0f * (1.0f - pow(pressure_hpa / sea_level_hpa, 0.1902949f));
}

// GNSS補正によるグランドレベル微調整（imu_kalman_gnss_update から呼ぶ）。
// delta_m [m] だけ ground_alt_abs を動かす。正値で気圧相対高度が下がり、負値で上がる。
// グランドレベルが確定していない起動直後は無視する。
void airdata_adjust_ground_alt(float delta_m) {
    if (!ground_set) return;
    ground_alt_abs += delta_m;
}

// ----------------------------
// 初期化
// ----------------------------

// MS5611 の初期化処理。PROM から 7 つのキャリブレーション係数をすべて読み取り C[] に格納する。
// 通常はリセット後に呼ぶべきだが、現在はリセットをスキップしている（コメントより）。
// 係数の読み取りに 1 つでも失敗したら false を返す。
// MS5611 にリセットコマンドを送り、内部 PROM を再ロードさせる。
// データシート上の復帰時間は約 2.8ms。余裕を見て 10ms 待つ。
// 送り先は 0x77 なので、同じ i2c0 にいる BNO085(0x4B) には影響しない。
static bool ms5611_reset() {
    if (!ms5611_write_cmd(CMD_RESET)) return false;
    delay(10);
    return true;
}

// PROM 8 ワードの CRC4 を計算する（データシート AN520 のアルゴリズムそのまま）。
// prom[7] の下位 4bit が工場で書かれた期待値。
// 計算はシフトと XOR だけで数マイクロ秒。**係数 1 ビットの化けを黙って通さない**ための検査で、
// 全ゼロ／全 FF のような派手な壊れ方しか見ていなかった従来の検査を補う。
// 係数が 1 ビットずれると、そのフライト中ずっと気圧高度が静かにずれ続ける。
static uint8_t ms5611_crc4(uint16_t prom[8]) {
    uint16_t n_rem = 0;
    const uint16_t crc_read = prom[7];
    prom[7] = (0xFF00 & prom[7]);          // CRC の 4bit を 0 にしてから計算する
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) n_rem ^= (uint16_t)(prom[cnt >> 1] & 0x00FF);
        else         n_rem ^= (uint16_t)(prom[cnt >> 1] >> 8);
        for (int n_bit = 8; n_bit > 0; n_bit--) {
            n_rem = (n_rem & 0x8000) ? (uint16_t)((n_rem << 1) ^ 0x3000)
                                     : (uint16_t)(n_rem << 1);
        }
    }
    prom[7] = crc_read;                    // 呼び出し側のために元へ戻す
    return (uint8_t)((n_rem >> 12) & 0x000F);
}

// 補正係数 C[0..6] を PROM から読み込む。
// 読めない、CRC が合わない、または明らかに異常な値だった場合は false。
// ★ 検査を全部通るまで C[] へは書かない。途中で失敗したときに
//   **半分だけ新しい係数**が残ると、気圧高度が静かに狂ったまま飛ぶことになる。
static bool ms5611_load_prom() {
    uint16_t prom[8];
    for (uint8_t i = 0; i <= 7; i++) {     // CRC のためワード 7 まで読む
        if (!ms5611_read_prom(i, prom[i])) {
            DEBUGW_P(20260310, "PROM read failed at C[");
            DEBUGW_P(20260310, i);
            DEBUGW_PLN(20260310, "]");
            return false;
        }
    }
    // C[1]〜C[6] が全部 0x0000 か全部 0xFFFF なら、I2C は応答しているのに
    // 中身が読めていない（バスが不安定・デバイスが未起動）。
    bool all_zero = true, all_ff = true;
    for (uint8_t i = 1; i <= 6; i++) {
        if (prom[i] != 0x0000) all_zero = false;
        if (prom[i] != 0xFFFF) all_ff   = false;
    }
    if (all_zero || all_ff) {
        DEBUGW_PLN(20260310, "MS5611 PROM looks invalid");
        enqueueTask(createLogSdTask("MS5611 PROM invalid (all 0x0000 or 0xFFFF)"));
        return false;
    }
    // CRC4 照合
    const uint8_t crc_calc = ms5611_crc4(prom);
    const uint8_t crc_want = (uint8_t)(prom[7] & 0x000F);
    if (crc_calc != crc_want) {
        DEBUGW_P(20260310, "MS5611 PROM CRC mismatch: ");
        DEBUGW_PLN(20260310, crc_calc);
        enqueueTask(createLogSdfTask("MS5611 PROM CRC NG (calc %u want %u)",
                                     (unsigned)crc_calc, (unsigned)crc_want));
        return false;
    }
    for (uint8_t i = 0; i <= 6; i++) C[i] = prom[i];
    return true;
}

bool ms5611_init() {
    // 通常はリセットを送らない。電源投入で PROM は既にロードされており、
    // 送ると 2.8ms 以上の待ちが要るだけで得が無いため。
    if (ms5611_load_prom()) return true;

    // 読めなかった場合だけ、リセットして 1 回だけやり直す。
    // ★ この関数は airdata_setup()（起動時）に加えて airdata_try_recovery()
    //   （飛行中・30 秒に 1 回）からも呼ばれる。したがって下の delay(10) は
    //   飛行中にも入り得るが、PROM 8 ワード ×2 + 10ms でも 20ms 未満に収まる。
    //   GNSS の受信バッファの余裕（GNSS_SERIAL_FIFO_SIZE、実流量で数秒）より十分内側。
    //   復旧は「アドレス応答があった」ときしか呼ばれないので、不在の機体で
    //   この待ちを毎回踏むこともない。
    // 失敗しても呼び出し側が ms5611_ok = false にして気圧なしで動作を続ける。
    DEBUGW_PLN(20260310, "MS5611 PROM read failed. retry with reset.");
    enqueueTask(createLogSdTask("MS5611 PROM read failed, retry with reset"));
    if (!ms5611_reset()) return false;
    return ms5611_load_prom();
}

// ----------------------------
// 非ブロッキング計測（ステートマシン）
// ----------------------------

// D1_raw と last_D2_raw から補正計算・高度計算・ウィンドウ処理を実行する。
// WAIT_D2 完了時に呼ばれる（D1→D2 の毎サイクル完了時）。
static bool ms5611_process_data() {
    ms5611_calculate(D1_raw, last_D2_raw, last_temperature, last_pressure);
    float raw_alt_abs = pressure_to_altitude(last_pressure);

    // 起動時の高度をグランドレベル（0m）として記録
    if (!ground_set) {
        ground_alt_abs  = raw_alt_abs;
        ground_pressure = last_pressure;   // 起動時気圧を保存
        ground_set = true;
        enqueueTask(createLogSdfTask("Initial airdata: Temp=%.2f C, Press=%.2f hPa, Alt=%.1f m",
            last_temperature, last_pressure, raw_alt_abs));
    }
    float raw_alt = raw_alt_abs - ground_alt_abs;  // 起動地点からの相対高度 [m]

    // トリム平均によるオーバーサンプリング:
    // VSPEED_WINDOW_MS 分のサンプルをバッファに蓄積し、挿入ソート後に上下15%を棄却して
    // トリム平均を求める。プロペラ干渉（~4.5Hz）による周期的外れ値を除去する。
    if (win_start == 0) win_start = millis();
    // 異常値（NaN・Inf・範囲外）はバッファに追加しない
    // D1 がゴミ値のとき pressure_to_altitude() が極端な値や NaN を返すことがある
    bool raw_alt_valid = !isnan(raw_alt) && !isinf(raw_alt) && raw_alt > -200.0f && raw_alt < 6000.0f;
    if (alt_win_count < ALT_WIN_BUF_SIZE && raw_alt_valid) {
        alt_win_buf[alt_win_count++] = raw_alt;
    }
    if (millis() - win_start >= VSPEED_WINDOW_MS && alt_win_count > 0) {
        // 挿入ソート（最大32サンプルなので十分高速）
        for (int i = 1; i < alt_win_count; i++) {
            float key = alt_win_buf[i];
            int j = i - 1;
            while (j >= 0 && alt_win_buf[j] > key) {
                alt_win_buf[j + 1] = alt_win_buf[j];
                j--;
            }
            alt_win_buf[j + 1] = key;
        }
        // 上下2つずつカット
        int trim = 2;
        int lo = trim;
        int hi = alt_win_count - trim;  // exclusive
        float sum = 0.0f;
        for (int i = lo; i < hi; i++) sum += alt_win_buf[i];
        float avg_cur = (hi > lo) ? sum / (hi - lo) : alt_win_buf[alt_win_count / 2];
        // vspeed 計算。
        // ★ **窓幅ではなく実経過時間で割ること。**
        //   以前は (差分 ÷ VSPEED_WINDOW_MS) 固定だったが、これは「窓が隣り合っている」
        //   前提でしか正しくない。I2C が数秒途絶えると窓が飛び、何秒も前の高度との差を
        //   1 窓分の時間で割って**巨大な偽の昇降率**が出る（復旧直後に必ず踏む）。
        //   飛びすぎていたときは更新せず、次の窓から取り直す。
        // 1 窓目も差分を取れない（前の窓が無い）ので更新しない。
        const unsigned long win_now_ms = millis();
        const unsigned long win_dt_ms  = win_now_ms - alt_win_prev_ms;
        if (alt_win_prev_valid && win_dt_ms > 0 &&
            win_dt_ms <= (unsigned long)VSPEED_WINDOW_MS * 3UL) {
            last_vspeed = (avg_cur - alt_win_prev) * (1000.0f / (float)win_dt_ms);
            // ★ 実際に計算できたときだけ時刻を記録する。窓が飛んだ回に記録すると
            //   get_airdata_vspeed() の 2 秒ステイル判定が古い値を「新鮮」と誤認する。
            last_vspeed_update_ms = win_now_ms;
        }
        last_altitude     = avg_cur;
        alt_win_prev      = avg_cur;
        alt_win_prev_ms   = win_now_ms;
        alt_win_prev_valid = true;
        last_win_hz       = alt_win_count * 1000.0f / VSPEED_WINDOW_MS; // 総サンプルから算出した Hz
        alt_win_count     = 0;
        win_start         = win_now_ms;
        return true;   // ウィンドウ完了（VSI 再描画のトリガー）
    } else {
        last_altitude = raw_alt;    // ウィンドウ未完了時は瞬時値
        return false;  // ウィンドウ未完了、last_vspeed 未更新
    }
}

// ステートマシン方式で MS5611 の計測サイクルを進める。
// loop() から毎回呼ぶことで D1→D2 の変換を非同期に実行する。
//
// 動作フロー:
//   IDLE → D1 変換開始 → WAIT_D1 → (12ms後) D1 読取＋D2 変換開始
//        → WAIT_D2 → (12ms後) D2 読取＋計算 → IDLE（1サイクル完了、約24ms）
//
// 戻り値: 1 サイクル（気圧＋温度）が完了したとき true（約 24ms ごと）。
//         完了時に last_temperature / last_pressure / last_altitude が更新される。
// 1 回の I2C 失敗を数える。連続で MS5611_FAIL_LIMIT を超えたら通信をやめる。
// やめた後は airdata_try_recovery() が定期的に拾いに行く。
static void ms5611_note_failure() {
    ms5611_state = MS5611_IDLE;             // 途中状態を残さない
    if (++ms5611_fail_count < MS5611_FAIL_LIMIT) return;

    ms5611_ok = false;
    ms5611_fail_count = 0;
    ms5611_recovery_ms = millis();          // 直後に再試行しない
    enqueueTask(createLogSdTask("MS5611 lost (I2C). airdata disabled, will retry"));
    DEBUGW_PLN(20260917, "[MS5611] lost, disabled");
}

// 見失った MS5611 を拾い直す。MS5611_RECOVERY_INTERVAL_MS ごとに 1 回だけ試す。
// ★ まず**アドレス応答の確認 1 トランザクションだけ**を投げる。
//   不在なら数十 µs で諦めるので、Core0 を止めない。PROM 8 ワードまで読みに行くのは
//   応答があったときだけ。
static void airdata_try_recovery() {
    const unsigned long now_ms = millis();
    if (ms5611_recovery_ms != 0 &&
        now_ms - ms5611_recovery_ms < MS5611_RECOVERY_INTERVAL_MS) return;
    ms5611_recovery_ms = now_ms;

    myWire.beginTransmission(MS5611_ADDR);
    if (myWire.endTransmission() != 0) return;   // まだ居ない。ログも出さない（毎回出ると溢れる）

    if (!ms5611_init()) {
        enqueueTask(createLogSdTask("MS5611 responded but PROM reload failed"));
        return;
    }

    // ★ 窓の履歴を捨てる。前の窓は何十秒も前のものなので、そのまま差分を取ると
    //   偽の昇降率になる（ms5611_process_data() の実経過時間チェックでも弾かれるが、
    //   ここで明示的に捨てておくほうが意図が読める）。
    //   ground_set は**触らない**。起動地点の基準高度は復旧後も同じでなければならない。
    alt_win_count      = 0;
    alt_win_prev_valid = false;
    win_start          = now_ms;
    ms5611_state       = MS5611_IDLE;
    ms5611_fail_count  = 0;
    ms5611_ok          = true;
    enqueueTask(createLogSdTask("MS5611 recovered"));
    DEBUGW_PLN(20260917, "[MS5611] recovered");
}

bool airdata_update() {
    if (!ms5611_ok) {           // 未接続・初期化失敗・通信途絶。定期的に拾いに行く
        airdata_try_recovery();
        return false;
    }
    switch (ms5611_state) {
        case MS5611_IDLE:
            // D1（気圧）変換を開始し、次のステートへ
            if (ms5611_start_convert(CMD_CONVERT_D1)) {
                convert_start = millis();
                ms5611_state  = MS5611_WAIT_D1;
            } else {
                ms5611_note_failure();
            }
            return false;

        case MS5611_WAIT_D1:
            // 変換時間が経過するまで待機
            if (millis() - convert_start < OSR_DELAY_MS) return false;
            // D1 を読み取り、続けて D2（温度）変換を開始
            if (!ms5611_read_adc_result(D1_raw)) {
                ms5611_note_failure();
                return false;
            }
            if (ms5611_start_convert(CMD_CONVERT_D2)) {
                convert_start = millis();
                ms5611_state  = MS5611_WAIT_D2;
            } else {
                ms5611_note_failure();
            }
            return false;

        case MS5611_WAIT_D2: {
            // 変換時間が経過するまで待機
            if (millis() - convert_start < OSR_DELAY_MS) return false;
            if (!ms5611_read_adc_result(last_D2_raw)) {
                ms5611_note_failure();
                return false;
            }
            ms5611_state = MS5611_IDLE;
            ms5611_fail_count = 0;          // 1 サイクル通ったので失敗の連続は切れた
            return ms5611_process_data();
        }
    }
    return false;
}

// MS5611 が正常に接続・初期化されているか返す
bool  get_airdata_ok()             { return ms5611_ok; }
// 最新の気圧高度 [m] を返す（起動地点からの相対高度）
float get_airdata_altitude()       { return last_altitude; }
// 最新の気圧 [hPa] を返す
// リプレイ中で CSV に pressure 列があれば、その値をそのまま返す
// ミラー中（受信モード）は受信した機体の値を返す。
// CSV は自機の値を書く必要があるので、生の実装は get_airdata_pressure_raw() に残してある。
float get_airdata_pressure() {
  if (link_mirror_active()) return link_get_pressure();
  return get_airdata_pressure_raw();
}

float get_airdata_pressure_raw() {
  if (replay_has_value(RHAVE_PRESS)) return replay_get_pressure();
  return last_pressure;
}
// 最新の気温 [℃] を返す
float get_airdata_temperature()    { return last_temperature; }
// 起動時の気圧 [hPa]（グランドレベル確定時点の計測値、"0m基準" に相当）
float get_airdata_ground_pressure(){ return ground_pressure; }
// 起動時の標準大気高度 [m]（ISA 1013.25hPa 基準。起動地点が何メートルに相当するか）
float get_airdata_ground_altitude(){ return ground_alt_abs; }
// 最新の鉛直速度 [m/s] を返す（トリム平均差分。初回ウィンドウ完了まで 0）
// I2C エラー等で 2 秒以上更新が途絶えた場合は 0 を返す（バリオ誤鳴動防止）。
// 2 秒 = VSPEED_WINDOW_MS(500ms) の 4 倍。正常時は 500ms ごとに更新されるため十分な余裕。
float get_airdata_vspeed() {
    if (last_vspeed_update_ms == 0 ||
        millis() - last_vspeed_update_ms > 2000UL) return 0.0f;
    return last_vspeed;
}
// 直前ウィンドウの総サンプルから算出した更新レート [Hz] を返す（診断用）
float get_airdata_win_hz()      { return last_win_hz; }

// ----------------------------
// setup / loop
// ----------------------------

// I2C バス（myWire）を初期化する。
// imu_setup()（BNO085）と airdata_setup()（MS5611）の両方が使うバスであるため、
// どちらよりも先に呼ぶ必要がある。GPS_TFT_map.ino の setup() 冒頭から呼ぶこと。
void airdata_wire_begin() {
    myWire.begin();
    // i2c0 は **MS5611 専用**。400kHz で動かす。
    // ★ v6 までは BNO085 と共用していて、BNO085 のクロックストレッチで
    //   RP2350 側がタイムアウト（error: 5）しバスがロックするため 100kHz に落としていた。
    //   v7 で BNO085 を SPI1（予備で i2c1）へ移してバスを分離したので、
    //   その制約は無くなった。MS5611 は 400kHz で問題なく動く。
    //   （コメントだけ 100kHz のまま残っていて、コードと正反対のことを述べていた）
    myWire.setClock(400000);
    delay(100);
}

// MS5611 の接続確認とキャリブレーション係数の読み込みを行う。
// airdata_wire_begin() と imu_setup() の後に呼ぶこと。
void airdata_setup() {
    DEBUG_PLN(20260310, "MS5611 + RP2354B Start");

    i2c_scan();

    // I2C アドレスに応答があるか確認（接続チェック）
    myWire.beginTransmission(MS5611_ADDR);
    bool connected = (myWire.endTransmission() == 0);

    if (!connected) {
        DEBUGW_PLN(20260310, "MS5611 not found. Skipping.");
        enqueueTask(createLogSdTask("MS5611 not found"));
        ms5611_ok = false;
        return;
    }

    ms5611_ok = ms5611_init();
    if (!ms5611_ok) {
        DEBUGW_PLN(20260310, "MS5611 init FAILED. Continuing without airdata.");
        enqueueTask(createLogSdTask("MS5611 init FAILED"));
    } else {
        DEBUGW_PLN(20260310, "MS5611 init OK!");
        enqueueTask(createLogSdTask("MS5611 init OK"));
    }
}

