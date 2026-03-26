// ============================================================
// File    : airdata.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 大気データ取得モジュールのヘッダー（開発中）。
//           気圧センサー（MS5611）から高度・気圧・気温を取得する。
//           airdata_update() をループから毎回呼ぶ非ブロッキング方式。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/23
// ============================================================

#ifndef AIRDATA_H
  #define AIRDATA_H
  #include <Arduino.h>
  #include <Wire.h>

  // MS5611 が使用する I2C バス（i2c0, GPIO32=SDA, GPIO33=SCL）。
  // BNO085 など同じバスに繋ぐセンサーはこれを extern で参照する。
  extern TwoWire myWire;

  // 初期化・テスト
  void airdata_wire_begin(); // I2C バス初期化（imu_setup() より前に呼ぶ）
  void airdata_setup();      // MS5611 初期化（imu_setup() の後に呼ぶ）
  void airdata_test();

  // 非ブロッキング計測（ループから毎回呼ぶ）
  // 1サイクル（気圧＋温度）完了時に true を返す（約20msごと）
  bool airdata_update();

  // MS5611 が正常に接続・初期化されているか（airdata_setup() 後に確定）
  bool  get_airdata_ok();

  // 最新計測値のゲッター（airdata_update() が true を返した後に更新）
  float get_airdata_altitude();          // 気圧高度 [m]（起動地点からの相対高度、トリム平均）
  float get_airdata_pressure();          // 気圧 [hPa]
  float get_airdata_temperature();       // 気温 [℃]
  float get_airdata_vspeed();            // 鉛直速度 [m/s]（トリム平均差分）
  int   get_airdata_win_samples();       // 直前ウィンドウのトリム後有効サンプル数（診断用）
  float get_airdata_win_hz();            // 直前ウィンドウの更新レート [Hz]（診断用）

  // 起動時グランドレベル情報（Vario 詳細画面の高度 KF セクションで使用）
  float get_airdata_ground_pressure();   // 起動時の気圧 [hPa]（"0m 基準" に使用した気圧）
  float get_airdata_ground_altitude();   // 起動時の標準大気高度 [m]（ISA 1013.25hPa 基準）

  // 気圧→高度変換（QNH 指定可能）
  float pressure_to_altitude(float pressure_hpa, float sea_level_hpa = 1013.25f);

  // GNSS補正によるグランドレベル微調整（imu_kalman_gnss_update から呼ぶ）
  // delta_m が正 → ground_alt_abs が上がり → バロ相対高度が下がる
  // delta_m が負 → ground_alt_abs が下がり → バロ相対高度が上がる
  void airdata_adjust_ground_alt(float delta_m);
#endif
