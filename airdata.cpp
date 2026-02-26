// ============================================================
// File    : airdata.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 大気データ取得の実装（開発中）。
//           気圧センサー MS5611（I2C接続）から気圧・気温を読み取り、
//           気圧高度を算出する予定。現時点はセットアップとテストのみ。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
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

// i2c0 バスを使用（SDA=GPIO32, SCL=GPIO33）。
// RP2350 の Wire ライブラリはデフォルトで i2c0 を使うが、ピン番号が異なるため明示的に指定。
TwoWire myWire(i2c0, 32, 33);

// MS5611 の工場出荷時キャリブレーション係数（PROM から読み出し）
// C[0]: 製造データ（未使用）, C[1]-C[6]: 温度・気圧補正用係数
uint16_t C[7];

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

// ADC 変換コマンド（D1 または D2）を送り、変換完了後に 24-bit の生データを読み取る。
// OSR=4096 の変換時間は最大 9.04ms なので、15ms 待機してから結果を読む。
// val に読み取った 24-bit 値を格納する。戻り値: 成功=true。
bool ms5611_read_adc(uint8_t cmd, uint32_t &val) {
    if (!ms5611_write_cmd(cmd)) return false;
    delay(15);  // OSR=4096: 9.04ms + マージン

    myWire.beginTransmission(MS5611_ADDR);
    myWire.write(CMD_ADC_READ);
    uint8_t err = myWire.endTransmission();
    if (err != 0) {
        Serial.print("ADC cmd error: ");
        Serial.println(err);
        return false;
    }

    uint8_t n = myWire.requestFrom((uint8_t)MS5611_ADDR, (uint8_t)3);
    if (n != 3) {
        Serial.print("ADC read error, got bytes: ");
        Serial.println(n);
        return false;
    }
    val  = (uint32_t)myWire.read() << 16;
    val |= (uint32_t)myWire.read() << 8;
    val |=  myWire.read();
    return true;
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
        Serial.print("C["); Serial.print(i); Serial.print("] = ");
        Serial.println(C[i]);
    }
    return true;
}

// ----------------------------
// 計測・補正
// ----------------------------

// MS5611 から気圧と温度を読み取り、キャリブレーション係数で補正した値を返す。
// データシート第 4.9 章の「Compensated pressure and temperature values」式を実装している。
//
// 変数名の意味（データシートに準拠）:
//   D1: 気圧 ADC 生値
//   D2: 温度 ADC 生値
//   dT: 実測温度と基準温度の差 = D2 - C[5]*256
//   tempd: 補正前温度 (100倍値) = 2000 + dT*C[6]/2^23
//   OFF:  ゼロ点オフセット     = C[2]*2^16 + C[4]*dT/2^7
//   SENS: 感度                 = C[1]*2^15 + C[3]*dT/2^8
//
// 低温補正（Second Order Temperature Compensation）:
//   tempd < 2000（＝20.00℃未満）のとき T2/OFF2/SENS2 を追加補正する。
//   さらに tempd < -1500（＝-15.00℃未満）では追加の補正項を加える。
//
// 最終気圧: P = (D1*SENS/2^21 - OFF) / 2^15  [単位: 0.01 hPa]
// 戻り値: 成功=true。temperature は ℃、pressure は hPa。
bool ms5611_read(float &tempderature, float &pressure) {
    uint32_t D1, D2;
    if (!ms5611_read_adc(CMD_CONVERT_D1, D1)) return false;
    if (!ms5611_read_adc(CMD_CONVERT_D2, D2)) return false;

    int32_t dT   = (int32_t)D2 - (int32_t)C[5] * 256L;
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
    OFF  -= OFF2;
    SENS -= SENS2;

    int32_t P = ((int64_t)D1 * SENS / 2097152LL - OFF) / 32768LL;

    tempderature = tempd / 100.0f;   // 0.01℃単位 → ℃
    pressure    = P    / 100.0f;    // 0.01hPa単位 → hPa
    return true;
}

// 気圧から気圧高度を計算する（国際標準大気モデル）。
// 式: altitude = 44330 * (1 - (P / P0)^(1/5.255))
//   P0=1013.25hPa（標準海面気圧）、指数 0.1902949 = 1/5.255。
// sea_level_hpa をその日の QNH に設定するとより正確な高度が得られる。
float pressure_to_altitude(float pressure_hpa, float sea_level_hpa = 1013.25f) {
    return 44330.0f * (1.0f - pow(pressure_hpa / sea_level_hpa, 0.1902949f));
}

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

    Serial.println("myWire init done");

    if (!ms5611_init()) {
        Serial.println("MS5611 init FAILED. Halting.");
        while (1) delay(1000);
    }
    Serial.println("MS5611 init OK!");
}

// MS5611 から 1 回計測して結果を Serial に出力するテスト関数。
// 読み取り失敗時は I2C をリセットして次回呼び出しに備える。
// 現在は開発・デバッグ用途のみ。本番時は GPS 高度との比較・補完に活用予定。
void airdata_test() {
    float temperature, pressure;
    if (ms5611_read(temperature, pressure)) {
        float altitude = pressure_to_altitude(pressure);
        Serial.print("Temp: "); Serial.print(temperature, 2);
        Serial.print(" C  |  Press: "); Serial.print(pressure, 2);
        Serial.print(" hPa  |  Alt: "); Serial.print(altitude, 1);
        Serial.println(" m");
    } else {
        Serial.println("Read failed, retrying...");
        delay(500);
        myWire.begin();  // I2Cリセット試行
    }
    delay(500);
}