// ============================================================
// File    : airdata.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 大気データ取得モジュールのヘッダー（開発中）。
//           気圧センサー（MS5611）から高度・気圧を取得する予定。
//           現時点は setup / test 関数の宣言のみ。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================

#ifndef AIRDATA_H
  #define AIRDATA_H
  #include <Arduino.h>
  void airdata_setup();
  void airdata_test();
#endif