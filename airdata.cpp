// ============================================================
// File    : airdata.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 大気データ取得の実装（開発中）。
//           気圧センサー MS5611（I2C接続）から気圧・気温を読み取り、
//           気圧高度を算出する。airdata_update() をループから毎回呼ぶ
//           ステートマシン方式で非ブロッキング動作する。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/04
// ============================================================
#include <Wire.h>
#include "airdata.h"
// MS5611 の I2C アドレス（SDO=VCC の場合は 0x76）
#define MS5611_ADDR 0x77

// MS5611 コマンドバイト定義（データシート Table 1 より）
#define CMD_RESET      0x1E  // リセットコマンド（起動時に必ず実行する）
#define CMD_READ_PROM  0xA0  // PROM 読み出しコマンドの先頭アドレス。係数番号を左シフトして加算する
#define CMD_CONVERT_D1 0x48  // 気圧 ADC 変換コマンド（OSR=4096: 最高精度、変換時間 ≈ 9ms）
#define CMD_CONVERT_D2 0x58  // 温度 ADC 変換コマンド（OSR=4096）
#define CMD_ADC_READ   0x00  // ADC 変換結果読み出しコマンド

// OSR=4096 の最大変換時間は 9.04ms。余裕を持たせて 10ms 待つ。
#define OSR_DELAY_MS   10

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

// airdata_update() が true を返すたびに更新される最新計測値
static float last_temperature = 0.0f;  // [℃]
static float last_pressure    = 0.0f;  // [hPa]
static float last_altitude    = 0.0f;  // [m]（1秒ウィンドウ平均値）

// 垂直速度 (vspeed) 計算用: 1秒ウィンドウ平均によるオーバーサンプリング
// 40Hz+ のサンプルを1秒分累積し、連続する2ウィンドウの平均差分から vspeed を求める。
// これにより個別サンプルのノイズを大幅に低減できる。
static double  alt_win_sum   = 0.0;    // 現在ウィンドウの高度累積値（GND相対）
static int     alt_win_count = 0;      // 現在ウィンドウのサンプル数
static float   alt_win_prev  = 0.0f;   // 直前ウィンドウの平均高度 [m]（GND相対）
static float   last_vspeed   = 0.0f;   // 最新の垂直速度 [m/s]
static unsigned long win_start = 0;    // 現在ウィンドウ開始時刻 [ms]

// 起動時を 0m 基準とするグランドレベル
static float   ground_alt_abs = 0.0f;  // 起動時の絶対高度（標準大気基準）[m]
static bool    ground_set     = false; // グランドレベル確定済みフラグ

// MS5611 の接続・初期化が正常に完了しているかどうか
// airdata_setup() で設定され、airdata_update() / get_airdata_ok() で参照される
static bool ms5611_ok = false;

// ----------------------------
// I2C スキャン (デバッグ用)
// 0x01 ～ 0x7E のアドレスを総当たりで確認し、応答があったデバイスを Serial に出力する。
// MS5611 が認識されているか確認するときに airdata_setup() 内から手動で呼ぶ。
// ----------------------------
void i2c_scan() {
    Serial.println("=== I2C Scan ===");
    bool found = false;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        myWire.beginTransmission(addr);
        uint8_t err = myWire.endTransmission();
        if (err == 0) {
            Serial.print("Found: 0x");
            Serial.println(addr, HEX);
            found = true;
        }
    }
    if (!found) Serial.println("No devices found!");
    Serial.println("================");
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
        Serial.print("I2C write error: ");
        Serial.println(err);
        return false;
    }
    return true;
}

// MS5611 にリセットコマンドを送り、内部 PROM データをリロードさせる。
// リセット後 50ms 待機することでデバイスが安定するのを待つ（データシートでは約 2.8ms 必要）。
bool ms5611_reset() {
    if (!ms5611_write_cmd(CMD_RESET)) return false;
    delay(50);  // 余裕を持たせる
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
        Serial.print("PROM write error: ");
        Serial.println(err);
        return false;
    }

    uint8_t n = myWire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)2);
    if (n != 2) {
        Serial.print("PROM read error, got bytes: ");
        Serial.println(n);
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

// ----------------------------
// 初期化
// ----------------------------

// MS5611 の初期化処理。PROM から 7 つのキャリブレーション係数をすべて読み取り C[] に格納する。
// 通常はリセット後に呼ぶべきだが、現在はリセットをスキップしている（コメントより）。
// 係数の読み取りに 1 つでも失敗したら false を返す。
bool ms5611_init() {
  // skip reset
    for (uint8_t i = 0; i <= 6; i++) {
        if (!ms5611_read_prom(i, C[i])) {
            Serial.print("PROM read failed at C[");
            Serial.print(i);
            Serial.println("]");
            return false;
        }
        //Serial.print("C["); Serial.print(i); Serial.print("] = ");
        //Serial.println(C[i]);
    }
    return true;
}

// ----------------------------
// 非ブロッキング計測（ステートマシン）
// ----------------------------

// ステートマシン方式で MS5611 の計測サイクルを進める。
// loop() から毎回呼ぶことで D1→D2 の変換を非同期に実行する。
//
// 動作フロー:
//   IDLE → D1 変換開始 → WAIT_D1 → (10ms後) D1 読取＋D2 変換開始
//        → WAIT_D2 → (10ms後) D2 読取＋計算 → IDLE（1サイクル完了）
//
// 戻り値: 1 サイクル（気圧＋温度）が完了したとき true（約 20ms ごと）。
//         完了時に last_temperature / last_pressure / last_altitude が更新される。
bool airdata_update() {
    if (!ms5611_ok) return false;  // 未接続・初期化失敗時は何もしない
    switch (ms5611_state) {
        case MS5611_IDLE:
            // D1（気圧）変換を開始し、次のステートへ
            if (ms5611_start_convert(CMD_CONVERT_D1)) {
                convert_start = millis();
                ms5611_state  = MS5611_WAIT_D1;
            }
            return false;

        case MS5611_WAIT_D1:
            // 変換時間が経過するまで待機
            if (millis() - convert_start < OSR_DELAY_MS) return false;
            // D1 を読み取り、続けて D2（温度）変換を開始
            if (!ms5611_read_adc_result(D1_raw)) {
                ms5611_state = MS5611_IDLE;  // エラー時はリセット
                return false;
            }
            if (ms5611_start_convert(CMD_CONVERT_D2)) {
                convert_start = millis();
                ms5611_state  = MS5611_WAIT_D2;
            }
            return false;

        case MS5611_WAIT_D2: {
            // 変換時間が経過するまで待機
            if (millis() - convert_start < OSR_DELAY_MS) return false;
            uint32_t D2_raw;
            if (!ms5611_read_adc_result(D2_raw)) {
                ms5611_state = MS5611_IDLE;  // エラー時はリセット
                return false;
            }
            // 補正計算して結果を保存
            ms5611_calculate(D1_raw, D2_raw, last_temperature, last_pressure);
            float raw_alt_abs = pressure_to_altitude(last_pressure);  // 標準大気基準の絶対高度
            ms5611_state  = MS5611_IDLE;

            // 起動時の高度をグランドレベル（0m）として記録
            if (!ground_set) { ground_alt_abs = raw_alt_abs; ground_set = true; }
            float raw_alt = raw_alt_abs - ground_alt_abs;  // 起動地点からの相対高度 [m]

            // 1秒ウィンドウによるオーバーサンプリング:
            // 40Hz+ のサンプルを1秒分累積し、ウィンドウ平均の差分から vspeed を計算する。
            // 個別サンプルのノイズを大幅に低減できる。
            if (win_start == 0) win_start = millis();
            alt_win_sum += raw_alt;
            alt_win_count++;
            if (millis() - win_start >= 1000 && alt_win_count > 0) {
                float avg_cur = (float)(alt_win_sum / alt_win_count);
                if (alt_win_prev != 0.0f) last_vspeed = avg_cur - alt_win_prev;
                last_altitude = avg_cur;    // 1秒平均高度に更新
                alt_win_prev  = avg_cur;
                alt_win_sum   = 0.0;
                alt_win_count = 0;
                win_start     = millis();
            } else {
                last_altitude = raw_alt;    // ウィンドウ未完了時は瞬時値
            }

            return true;  // 新しいデータ取得完了
        }
    }
    return false;
}

// MS5611 が正常に接続・初期化されているか返す
bool  get_airdata_ok()          { return ms5611_ok; }
// 最新の気圧高度 [m] を返す（airdata_update() が true を返した後に更新される）
float get_airdata_altitude()    { return last_altitude; }
// 最新の気圧 [hPa] を返す
float get_airdata_pressure()    { return last_pressure; }
// 最新の気温 [℃] を返す
float get_airdata_temperature() { return last_temperature; }
// 最新の鉛直速度 [m/s] を返す（1秒ウィンドウ平均差分。初回ウィンドウ完了まで 0）
float get_airdata_vspeed()      { return last_vspeed; }

// ----------------------------
// setup / loop
// ----------------------------

// MS5611 の I2C バスを初期化し、センサーのキャリブレーション係数を読み込む。
// 初期化失敗時は while(1) で停止する（開発中のため意図的なハルト）。
// GPS_TFT_map.ino の setup1() から呼ばれる予定（現在は開発中で未使用）。
void airdata_setup() {
    Serial.println("MS5611 + RP2354B Start");

    myWire.begin();
    myWire.setClock(100000);  // I2C クロック 100kHz（標準モード）
    delay(100);

    // I2C アドレスに応答があるか確認（接続チェック）
    myWire.beginTransmission(MS5611_ADDR);
    bool connected = (myWire.endTransmission() == 0);

    if (!connected) {
        Serial.println("MS5611 not found. Skipping.");
        ms5611_ok = false;
        return;
    }

    ms5611_ok = ms5611_init();
    if (!ms5611_ok) {
        Serial.println("MS5611 init FAILED. Continuing without airdata.");
    } else {
        Serial.println("MS5611 init OK!");
    }
}

// airdata_update() を呼び出し、計測完了時のみ Serial に結果を出力するテスト関数。
// loop() から毎回呼ぶことで非ブロッキングに動作する。
void airdata_test() {
    if (airdata_update()) {
        Serial.print("Temp: "); Serial.print(get_airdata_temperature(), 2);
        Serial.print(" C  |  Press: "); Serial.print(get_airdata_pressure(), 2);
        Serial.print(" hPa  |  Alt: "); Serial.print(get_airdata_altitude(), 1);
        Serial.println(" m");
    }
}
