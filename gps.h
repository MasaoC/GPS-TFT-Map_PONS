// ============================================================
// File    : gps.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : GPS受信・解析モジュールのヘッダー。
//           衛星データ構造体(SatelliteData)、NMEAバッファ定義、
//           位置・速度・高度・時刻取得関数のプロトタイプ宣言。
//           リプレイモード・デモモード切替関数も含む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================

#ifndef GPS_H
  #define GPS_H
  #include <TinyGPS++.h>
  // Define satellite types
  #define SATELLITE_TYPE_GPS 1
  #define SATELLITE_TYPE_GLONASS 2
  #define SATELLITE_TYPE_GALILEO 3
  #define SATELLITE_TYPE_BEIDOU 4
  #define SATELLITE_TYPE_QZSS 5
  #define SATELLITE_TYPE_UNKNOWN 0

  #define MAX_LAST_NMEA 16
  #define NMEA_MAX_CHAR 100
  #define MAX_SATELLITES 64  // 実測で48超えが発生したため64に拡大（SAM-M10Qは5星座で最大55衛星程度が可視）

  struct SatelliteData {
    int PRN = 0;
    int elevation = 0;
    int azimuth = 0;
    int SNR = 0;
    int satelliteType = SATELLITE_TYPE_UNKNOWN;
    unsigned long lastReceived = 0;
  };
  extern SatelliteData satellites[MAX_SATELLITES];
  extern char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];
  extern int stored_nmea_index;
  extern unsigned long time_lastnmea;
  extern bool newcourse_arrived;

  void utcToJst(int *year, int *month, int *day, int *hour);
  void parseGSV(char *nmea);
  void parseGSA(char *nmea);
  char* get_gps_nmea(int i);
  unsigned long get_gps_nmea_time(int i);
  void gps_setup();
  void gps_loop(int id);
  void try_enque_savecsv();
  
  bool gps_new_location_arrived();
  void set_new_location_off();

  void gps_getposition_mode();
  void gps_constellation_mode();
  bool get_gps_fix();
  bool get_gps_connection();
  int get_gps_numsat();
  double get_gps_mps();
  double get_gps_truetrack();
  double get_gps_magtrack();
  uint32_t get_gps_baudrate();  // GPS シリアルの現在ボーレート
  int   get_gps_fixtype();   // GSA フィックスタイプ (1=No Fix, 2=2D, 3=3D)
  float get_gps_pdop();      // PDOP（Position DOP）
  float get_gps_hdop();      // HDOP（Horizontal DOP）
  float get_gps_vdop();      // VDOP（Vertical DOP）
  int   get_gsa_numsat();    // 測位使用衛星数
  int   get_gsa_prn(int i);  // 測位使用衛星 PRN (i=0..11)
  double get_gps_lat();
  double get_gps_lon();
  double get_gps_altitude();

  TinyGPSDate get_gpsdate();
  TinyGPSTime get_gpstime();

  // 最大 G/S 取得関数
  float get_maxgs();            // 全時間最大 G/S [m/s]
  int   get_maxgs_hour();       // 全時間最大 G/S の記録時刻（JST 時）
  int   get_maxgs_min();        // 全時間最大 G/S の記録時刻（JST 分）
  float get_maxgs_5min();       // 5分保持最大 G/S [m/s]
  int   get_maxgs_5min_hour();  // 5分保持最大 G/S の記録時刻（JST 時）
  int   get_maxgs_5min_min();   // 5分保持最大 G/S の記録時刻（JST 分）

  void toggle_demo_biwako();
  bool get_demo_biwako();
  void set_demo_biwako(bool biwakomode);
  void toggleReplayMode();
  bool getReplayMode();
  void set_replaymode(bool replaymode);

#endif