// ============================================================
// File    : gps.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : GPS受信・解析モジュールのヘッダー（Quectel LC86G・NMEA）。
//           衛星データ構造体(SatelliteData)、受信バッファ定義、
//           位置・速度・高度・時刻取得関数のプロトタイプ宣言。
//           リプレイモード切替、地点選択式デモ飛行(demo_site_t)の関数、
//           およびリプレイ中に電圧を CSV の値へ差し替える関数も含む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================

#ifndef GPS_H
  #define GPS_H
  #include <TinyGPS++.h>
  // GpsDate / GpsTime: get_gpsdate() / get_gpstime() の戻り値型
  // 実 GPS は NMEA の RMC から更新、リプレイモードは SD の飛行 CSV から更新する。
  struct GpsDate {
    uint16_t _year  = 0; uint8_t _month = 0; uint8_t _day = 0; bool _valid = false;
    bool     isValid() const { return _valid;  }
    uint16_t year()    const { return _year;   }
    uint8_t  month()   const { return _month;  }
    uint8_t  day()     const { return _day;    }
  };
  struct GpsTime {
    uint8_t _hour=0; uint8_t _min=0; uint8_t _sec=0; uint8_t _cs=0; bool _valid=false;
    bool    isValid()     const { return _valid; }
    uint8_t hour()        const { return _hour;  }
    uint8_t minute()      const { return _min;   }
    uint8_t second()      const { return _sec;   }
    uint8_t centisecond() const { return _cs;    }
  };
  // Define satellite types
  #define SATELLITE_TYPE_GPS 1
  #define SATELLITE_TYPE_GLONASS 2
  #define SATELLITE_TYPE_GALILEO 3
  #define SATELLITE_TYPE_BEIDOU 4
  #define SATELLITE_TYPE_QZSS 5
  #define SATELLITE_TYPE_UNKNOWN 0

  #define MAX_LAST_NMEA 16
  #define NMEA_MAX_CHAR 100
  #define MAX_SATELLITES 64  // v6 の実測で 48 超えが発生したため 64（5 星座で最大 55 衛星程度が可視）

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
  extern bool newcourse_arrived;

  void utcToJst(int *year, int *month, int *day, int *hour);
  void jstToUtc(int *year, int *month, int *day, int *hour);

  // リプレイ再生中の値の差し替え（電池表示）。
  // havebit には mysd.h の RHAVE_* を渡す。CSV にその列が無ければ false を返し、
  // 呼び出し側は実際の値をそのまま使う。
  bool  replay_has_value(uint16_t havebit);
  float replay_get_voltage();

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
  bool get_gps_cfg_ok();  // 起動時の設定コマンド（PAIR062 の GSA・GSV）が通ったか（起動画面の進捗表示用）
  int get_gps_numsat();
  double get_gps_mps();
  double get_gps_truetrack();
  uint32_t get_gps_baudrate();  // GPS シリアルの現在ボーレート
  int   get_gps_fixtype();   // GSA フィックスタイプ (1=No Fix, 2=2D, 3=3D)。GSA が来ていなければ測位の有無から推す
  float get_gps_pdop();      // PDOP（Position DOP）
  float get_gps_hdop();      // HDOP（Horizontal DOP）
  float get_gps_vdop();      // VDOP（Vertical DOP）
  int   get_gsa_numsat();    // 測位使用衛星数
  int   get_gsa_prn(int i);  // 測位使用衛星 PRN (i=0..11)
  double get_gps_lat();
  double get_gps_lon();
  double get_gps_altitude();

  // 水平精度（GST の緯度・経度の 1σ から作る、mm 単位）。
  // ★ get_gps_hacc_valid() が false のときは「分からない」（GST が来ていない・リプレイ・デモ）。
  //   値が 0 でも「完璧」とは読まないこと。
  uint32_t get_gps_hacc_mm();
  bool     get_gps_hacc_valid();
  bool     get_gps_gnssFixOK();   // 有効な測位か（RMC が 'A' かつ GGA の品質 > 0、2D 以上）
  // 起動時の設定コマンドの結果（$PAIR001 の結果。0=成功 3=非対応 -1=応答なし）。GPS DETAIL 用。
  int      get_gps_easy_result(); // PAIR490（EASY＝軌道の自己予測）
  int      get_gps_gst_result();  // PAIR062,8（GST の出力）

  GpsDate get_gpsdate();
  GpsTime get_gpstime();

  // 最大 G/S 取得関数
  float get_maxgs();            // 全時間最大 G/S [m/s]
  int   get_maxgs_hour();       // 全時間最大 G/S の記録時刻（JST 時）
  int   get_maxgs_min();        // 全時間最大 G/S の記録時刻（JST 分）
  float get_maxgs_5min();       // 5分保持最大 G/S [m/s]
  int   get_maxgs_5min_hour();  // 5分保持最大 G/S の記録時刻（JST 時）
  int   get_maxgs_5min_min();   // 5分保持最大 G/S の記録時刻（JST 分）

  // ---- デモ飛行 ----
  // 実 GPS を使わず仮想的に飛ばすモード。地点を選べるようにしてあるので、
  // 各フライト地点の地図表示を実機で確認するのにも使える。
  enum demo_site_t : uint8_t {
    DEMO_OFF = 0,
    DEMO_BIWAKO,
    DEMO_SHIRAHAMA,
    DEMO_KASAOKA,
    DEMO_FUJIGAWA,
    DEMO_TOKYO,
    DEMO_OSAKA,
    DEMO_SITE_COUNT
  };
  demo_site_t get_demo_site();
  void set_demo_site(demo_site_t site);
  void next_demo_site();           // 設定画面で 1 段階進める（末尾で OFF に戻る）
  const char* get_demo_site_name(demo_site_t site);
  bool is_demo_active();           // いずれかの地点でデモ飛行中か
  void set_demo_off();
  bool getReplayMode();
  void set_replaymode(bool replaymode);

  uint32_t get_gps_fix_millis();
  uint32_t get_gps_jst_yyyymmdd();   // JST の今日の日付 YYYYMMDD。0 = 時刻がまだ無い

#endif