// ============================================================
// File    : gnss.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS受信・解析モジュールのヘッダー。
//           衛星データ構造体(SatelliteData)、受信バッファ定義、
//           位置・速度・高度・時刻取得関数のプロトタイプ宣言。
//           リプレイモード切替、地点選択式デモ飛行(demo_site_t)の関数、
//           およびリプレイ中にセンサ値を CSV の値へ差し替える関数も含む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================

#ifndef GNSS_H
  #define GNSS_H
  // GnssDate / GnssTime: get_gnss_date() / get_gnss_time() の戻り値型
  // 実 GNSS は UBX NAV-PVT から更新、リプレイモードは SD の飛行 CSV から更新する。
  struct GnssDate {
    uint16_t _year  = 0; uint8_t _month = 0; uint8_t _day = 0; bool _valid = false;
    bool     isValid() const { return _valid;  }
    uint16_t year()    const { return _year;   }
    uint8_t  month()   const { return _month;  }
    uint8_t  day()     const { return _day;    }
  };
  struct GnssTime {
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
  #define MAX_SATELLITES 64  // 実測で48超えが発生したため64に拡大（SAM-M10Qは5星座で最大55衛星程度が可視）

  struct SatelliteData {
    int PRN = 0;
    int elevation = 0;
    int azimuth = 0;
    int SNR = 0;
    // 仰角・方位が分かっているか。false の間 elevation / azimuth は意味の無い 0 で、
    // 星座画面は図に描かずフッターに並べる（handle_navsat() のコメント）。
    bool posKnown = false;
    int satelliteType = SATELLITE_TYPE_UNKNOWN;
    unsigned long lastReceived = 0;
  };
  extern SatelliteData satellites[MAX_SATELLITES];
  extern char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];
  extern int stored_nmea_index;
  extern bool newcourse_arrived;

  void utcToJst(int *year, int *month, int *day, int *hour);
  void jstToUtc(int *year, int *month, int *day, int *hour);

  // リプレイ再生中のセンサ値オーバーライド（imu.cpp / airdata.cpp から参照）。
  // havebit には mysd.h の RHAVE_* を渡す。CSV にその列が無ければ false を返し、
  // 呼び出し側は実センサの値をそのまま使う。
  bool  replay_has_value(uint16_t havebit);
  float replay_get_kf_altitude();
  float replay_get_kf_vspeed();
  float replay_get_pressure();
  float replay_get_voltage();

  char* get_gnss_nmea(int i);
  unsigned long get_gnss_nmea_time(int i);
  void gnss_setup();
  void gnss_loop(int id);
  // UBX のフレームを 1 つ GNSS へ書く（チェックサムは付ける）。航法データベース（gnss_dbd.cpp）のポールと試験用。
  // 書きかけの DBD のフレームがあれば先に書き切る（gnss_dbd_finish_tx）。
  void gnss_ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
  // 受信がうまくいっているかの数（gnss_dbd.cpp の "GNSS DBD IO" の行が前後の差を出す）。
  uint32_t gnss_ubx_ck_errors();    // チェックサムが合わなかった UBX フレームの数（起動から）
  uint32_t gnss_rx_overflows();     // 受信 FIFO があふれた回数（起動から）
  int      gnss_rx_backlog_peak_take();   // gnss_loop() に入った時点の FIFO のたまりの最大 [B]。取ると 0 に戻る
  void try_enque_savecsv();
  
  bool gnss_new_location_arrived();
  void set_new_location_off();

  void gnss_getposition_mode();
  void gnss_constellation_mode();
  bool get_gnss_fix();
  bool get_gnss_connection();
  // 一度でも GNSS から受信したか（起動直後と「抜けた」の区別に使う）。
  // ★ これ無しで `!get_gnss_connection()` を警報の条件にすると、**起動直後の
  //   まだ 1 バイトも来ていない間に発報する**（PONS Link の GNSS 断線ビットで踏んだ）。
  bool gnss_ever_connected();
  bool get_gnss_cfg_ok();  // 起動時 UBX 設定コマンドが全部 ACK されたか（起動画面の進捗表示用）
  // 起動直後の NAV-PVT に有効な日時が入っていたか（GNSS のバックアップ電池 BT1 が時計を保っていた）。
  bool gnss_rtc_alive_on_boot();
  bool gnss_rtc_boot_checked();   // 起動後最初の NAV-PVT を見て、上の判定が決まったか
  // 捕捉衛星数。分からないときは GNSS_NUMSAT_UNKNOWN（リプレイで CSV に列が無い・ミラー中に回転枠 3 をまだ受けていない）。
  // ★ 0 は「衛星が 0 個」という本物の値。不明を 0 で返すと、受信を始めた直後の受信機が
  //   機体は測位しているのに 0sats（赤の点滅・GS の赤線・Weak GNSS Signal）を出す（2026-11-07 に直した）。
  int get_gnss_numsat();
  #define GNSS_NUMSAT_UNKNOWN 99
  double get_gnss_mps();
  double get_gnss_truetrack();
  uint32_t get_gnss_baudrate();  // GNSS シリアルの現在ボーレート
  int   get_gnss_fixtype();   // GSA フィックスタイプ (1=No Fix, 2=2D, 3=3D)
  float get_gnss_pdop();      // PDOP（Position DOP）
  float get_gnss_hdop();      // HDOP（Horizontal DOP）
  float get_gnss_vdop();      // VDOP（Vertical DOP）
  int   get_gsa_numsat();    // 測位使用衛星数
  int   get_gsa_prn(int i);  // 測位使用衛星 PRN (i=0..11)
  // ★ ミラーもデモも通さない、この機体自身の測位。
  //   受信モードでは get_gnss_*() が機体の値に置き換わるので、
  //   ボート自身を地図に出すにはこちらを使う。fix が無ければ false。
  bool gnss_get_own_fix(double &lat, double &lon, double &gs, double &track);

  double get_gnss_lat();
  double get_gnss_lon();
  double get_gnss_altitude();

  uint32_t get_gnss_hacc_mm();     // 水平精度推定値（NAV-PVT hAcc、mm 単位）
  uint32_t get_gnss_vacc_mm();     // 垂直精度推定値（NAV-PVT vAcc、mm 単位）
  uint32_t get_gnss_sacc_mmps();   // 速度精度推定値（NAV-PVT sAcc、mm/s 単位）
  float    get_gnss_veld_mps();    // GNSS 垂直速度（NAV-PVT velD、上昇正、m/s）
  // 測位エポック（NAV-PVT iTOW、GPS 週内時刻 ms）。
  // ★ 「その測位をもう使ったか」の判定に使う。millis() ではなく**測位の時刻**で
  //   判定しないと、loop() が速いときに同じ観測を何度も融合してしまう
  //   （imu.cpp の imu_kalman_gnss_vel_update のコメント参照）。
  uint32_t get_gnss_itow_ms();
  // AssistNow Autonomous（GNSS が自分で予測した衛星軌道）の状態。GNSS DETAIL 3 の表示用。
  // ★ どれも**自機の GNSS** の値（ミラー中も受信機自身のもの）。
  int  get_gnss_ana_state();      // -1=状態がまだ届いていない / 0=無効 / 1=有効（GNSS 自身の申告）
  bool get_gnss_ana_busy();       // 予測を計算中か（この間は GNSS の消費電流が増える）
  int  get_gnss_ana_data_svs();   // 予測データを持っている衛星数
  int  get_gnss_ana_orbit_svs();  // いま予測軌道を使っている衛星数
  // NED 水平速度。姿勢 ESKF の速度観測用（旋回中の遠心加速度を分離するのに必要）。
  bool     get_gnss_fixok();   // NAV-PVT gnssFixOK フラグ（有効な GNSS フィックスか）

  GnssDate get_gnss_date();
  GnssTime get_gnss_time();
  uint32_t get_gnss_jst_yyyymmdd();   // JST の今日の日付 YYYYMMDD。0 = 時刻がまだ無い

  // 最大 G/S 取得関数
  void  reset_maxgs();          // 最大 G/S を捨てる（供給元が入れ替わるとき）
  float get_maxgs();            // 全時間最大 G/S [m/s]
  int   get_maxgs_hour();       // 全時間最大 G/S の記録時刻（JST 時）
  int   get_maxgs_min();        // 全時間最大 G/S の記録時刻（JST 分）
  float get_maxgs_5min();       // 5分保持最大 G/S [m/s]
  int   get_maxgs_5min_hour();  // 5分保持最大 G/S の記録時刻（JST 時）
  int   get_maxgs_5min_min();   // 5分保持最大 G/S の記録時刻（JST 分）

  // ---- デモ飛行 ----
  // 実 GNSS を使わず仮想的に飛ばすモード。地点を選べるようにしてあるので、
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
  // リプレイ中の姿勢 [度]（飛行 CSV・受信ログの roll / pitch / yaw … 列の記録由来）。
  // 実機の ESKF ではないので、表示側は getReplayMode() で参照先を切り替えること。
  bool get_replay_attitude(float &roll, float &pitch);
  // 以下はその列が CSV にあり、値が入っている行だけ true を返す（姿勢の列の無い古い CSV では false）。
  bool get_replay_pitch_avg(float &avg);
  bool get_replay_roll_trim(float &trim);
  bool get_replay_yaw(float &yaw, float &acc95);
  bool get_replay_wind(float &speed_mps, float &dir_to_deg);
  // 推定 TAS と大きい枠の中身（飛行 CSV の tas_mps / hdr 列。列の無い古い CSV では false）。
  // 表示は直接読まず tas.h の tas_header() を通す。
  bool get_replay_tas(float &mps);
  bool get_replay_hdr(int &hdr);
  void set_replaymode(bool replaymode);

  uint32_t get_gnss_fix_millis();  // 最後にGNSS時刻を受信したときのmillis()（時刻推定用）

#endif