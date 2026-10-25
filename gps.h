// ============================================================
// File    : gps.h
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : GPS（u-blox SAM-M10Q）の受信と解析。UBX バイナリ（NAV-PVT / NAV-SAT）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
#ifndef GPS_H
  #define GPS_H
  #include <Arduino.h>

  // Define satellite types
  #define SATELLITE_TYPE_GPS 1
  #define SATELLITE_TYPE_GLONASS 2
  #define SATELLITE_TYPE_GALILEO 3
  #define SATELLITE_TYPE_BEIDOU 4
  #define SATELLITE_TYPE_QZSS 5
  #define SATELLITE_TYPE_UNKNOWN 0

  // NAV-SAT は全衛星系ぶん来るので 32 では足りない（衛星番号も衛星系ごとに 1 から振られる）。
  #define MAX_SATELLITES 64

  // GPS 詳細画面 2 の「受けた UBX フレーム」の一覧（以前は NMEA の生の文）
  #define GPS_FRAMELOG_LINES 11
  #define GPS_FRAMELOG_CHARS 48

  struct SatelliteData {
    int PRN = 0;           // 衛星番号（衛星系ごと。同じ番号が GPS と Galileo にありうる）
    int elevation = 0;
    int azimuth = 0;
    int SNR = 0;           // C/N0 [dBHz]
    int satelliteType = SATELLITE_TYPE_UNKNOWN;
    unsigned long lastReceived = 0;
  };
  extern SatelliteData satellites[MAX_SATELLITES];
  extern unsigned long time_last_gps_frame;  // UBX のフレームを最後に受けた時刻（受信中の表示用）

  // UTC の日付と時刻（NAV-PVT から）。呼び出し側は TinyGPS++ の頃と同じ名前で読む。
  struct GpsDate {
    bool valid = false;
    int y = 0, mo = 0, d = 0;
    bool isValid() const { return valid; }
    int year() const { return y; }
    int month() const { return mo; }
    int day() const { return d; }
  };
  struct GpsTime {
    bool valid = false;
    int h = 0, mi = 0, s = 0;
    bool isValid() const { return valid; }
    int hour() const { return h; }
    int minute() const { return mi; }
    int second() const { return s; }
  };

  void utcToJst(int *year, int *month, int *day, int *hour);
  const char* get_gps_framelog(int i);   // i=0 が最新
  void gps_setup();
  bool gps_loop();                        // 新しい NAV-PVT を受けた回だけ true（その直後に描くため）
  void gps_getposition_mode();            // 地図に戻るとき（NAV-SAT を 10 秒に 1 回に）
  void gps_constellation_mode();          // GPS 詳細画面を開くとき（NAV-SAT を 1Hz に）
  bool get_gps_fix();
  bool get_gps_connection();
  bool get_gps_live();
  int get_gps_fix_age_s();
  bool get_gps_fix3d();          // 3D 測位かつ gnssFixOK（本番前チェック。v7 と同じ条件）
  int get_gps_fixtype();         // u-blox の fixType（0=なし 2=2D 3=3D）
  float get_gps_hacc_m();        // 水平精度の目安 [m]（NAV-PVT の hAcc）。測位していなければ -1
  float get_gps_sacc_mps();      // 速度精度の目安 [m/s]（NAV-PVT の sAcc）。測位していなければ -1
  float get_gps_pdop();
  int get_gps_ana_data_svs();    // AssistNow Autonomous の予測データを持つ衛星数（NAV-SAT）
  int get_gps_ana_orbit_svs();   // いま予測軌道で測っている衛星数（NAV-SAT）
  float get_maxgs_all();         // 起動からの最大 G/S [m/s]
  int get_maxgs_all_hhmm();      // その時刻（JST の 時*100+分）。まだ無ければ -1
  float get_maxgs_recent();      // 直近 MAXGS_WINDOW_MS の最大 G/S [m/s]
  extern int gps_cfg_acked;      // 直近の gps_setup() で ACK が返った UBX の設定の数（GPS 詳細画面）
  extern int gps_cfg_total;
  int get_gps_numsat();
  double get_gps_mps();
  double get_gps_truetrack();   // 方位はすべて真方位（磁方位の get_gps_magtrack() は 0.503 で廃止）
  double get_gps_lat();
  double get_gps_lon();

  GpsDate get_gpsdate();
  GpsTime get_gpstime();

  void toggle_demo_biwako();
  bool get_demo_biwako();

  // リプレイ（SD の飛行 CSV を本体で再生する）。設定画面で選び、設定画面を抜けるときに反映する。
  bool replay_active();
  int replay_get_speed();                 // 1 / 4 / 16 倍
  void replay_cycle_speed();
  void replay_select_next();              // OFF → 1 つ目 → … → 最後 → OFF
  void replay_select_none();
  const char* replay_selected_name();     // 選んでいるファイル（OFF なら nullptr）
  bool replay_selected_is_playing();
  void replay_apply_selection();          // 選んだファイルを再生する／OFF なら止める
  void replay_set_paused(bool paused);    // 地図画面以外で一時停止する
  bool replay_is_paused();

#endif
