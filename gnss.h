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
  #define SATELLITE_TYPE_SBAS 6   // 静止衛星の補強信号（2026-11-14〜。番号は PRN 120〜158。日本では MSAS 129・137＝みちびきの静止衛星、GAGAN 128 など）
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
  // いま画面に出している位置の供給元（ミラー中は機体、デモ、リプレイ、自機）に fix が無いか（2026-11-14〜）。
  // 地図の「Scanning GNSS Signal」の箱（display_tft.cpp の draw_nomapdata）とユーザー LED の点滅（.ino の loop_userled）が
  // 同じ答えを出すためにここにまとめた。条件を呼び出し側に書き写さないこと
  bool gnss_shown_no_fix();
  bool get_gnss_connection();
  // 起動時点で GNSS 内部 RTC が有効な日時を保持していたか（バックアップ電池 MS621FE の生存判定用）
  bool gnss_rtc_alive_on_boot();
  bool gnss_rtc_boot_checked();   // 起動後最初の NAV-PVT を見て、上の判定が決まったか
  // 起動時の衛星系の設定（CFG-SIGNAL）の確認と書き直し（QZSS を入れる・GNSS_BDS_B1I なら BeiDou を B1I にして GLONASS を切る）が
  // 済んだか（2026-11-13〜）。書き直すと GNSS が測位をやり直すので、ANA の設定・航法データベースの戻し・外からの時刻入れはこれを待つ
  bool gnss_signal_cfg_settled();
  // 一度でも GNSS から受信したか（起動直後と「抜けた」の区別に使う）。
  // ★ これ無しで `!get_gnss_connection()` を警報の条件にすると、**起動直後の
  //   まだ 1 バイトも来ていない間に発報する**（PONS Link の GNSS 断線ビットで踏んだ）。
  bool gnss_ever_connected();
  bool get_gnss_cfg_ok();  // 起動時 UBX 設定コマンドが全部 ACK されたか（起動画面の進捗表示用）
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
  // 自機の GNSS の fix と高度（ミラー・デモの差し替えを通さない。リプレイ中は呼ぶ側が除く）。link_assist.cpp が使う
  bool   gnss_own_fixok();
  double gnss_own_altitude();
  uint32_t gnss_own_tacc_ns();   // 自機の GNSS の時刻の精度 [ns]（NAV-PVT tAcc。fix の前でも RTC の時刻なら入る）
  // 自機の GNSS の時刻を仲間へ渡してよいか（2026-11-14〜）。true になるのは
  //   ・起動時に GNSS の時計（RTC）が生きていた（BT1 が保っていた時刻）か、
  //   ・この起動で fix した（fix で時刻が確かめられた）とき。
  // 外から入れた時刻（LoRa・PC）や、fix の前に GNSS が自分で読んだ時刻は、fix するまで false。
  // fix していないときに時刻が飛んだ（GNSS_TIME_JUMP_MS）・時刻が消えたときも false へ戻す（次の fix で true）。
  // 11/14 に、LoRa でもらった時刻が fix の前に約 +17 分飛んだため（settings.h の GNSS_TIME_JUMP_MS）。link_assist.cpp が見る
  bool gnss_own_time_trusted();

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
  bool get_gnss_ana_off_cold();
  bool gnss_ana_off_for_cold_test();  // PONS:DBD,COLD / COLDONLY の前に、その起動の間 ANA を切る（2026-11-14〜。gnss.cpp）   // この起動はコールドスタート（BT1 が切れていた）なので ANA を切った（2026-11-13〜）
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
  // 地点を選ぶのは REPLAY/DEMO 画面（GPS_TFT_map.ino の startDemo）。
  void set_demo_site(demo_site_t site);
  const char* get_demo_site_name(demo_site_t site);
  bool is_demo_active();           // いずれかの地点でデモ飛行中か
  void set_demo_off();
  bool getReplayMode();
  // 方位を測った時刻 [ms]。リプレイ中は CSV の行の時刻（再生の仮想時刻）、それ以外は millis()。
  // 旋回角速度の窓はこれで数える（millis() だと再生倍率の分だけ deg/s が大きく出る）。
  uint32_t get_gnss_course_ms();
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

  // ---- UBX-MON-RF (0x0A 0x38) 関連（debug/v7-noise-test から移した）----
  struct GnssMonRf {
    bool     valid;          // 一度でも受信できたか
    uint8_t  jamInd;         // CW干渉・妨害インジケータ (0..255, 0=なし)
    uint16_t agcCnt;         // AGCカウント値 (0..8191)
    uint16_t noisePerMS;     // 雑音レベル
    int8_t   ofsI;           // I成分 DCオフセット
    uint8_t  magI;           // I成分 振幅
    int8_t   ofsQ;           // Q成分 DCオフセット
    uint8_t  magQ;           // Q成分 振幅
    uint8_t  jammingState;   // 0=unknown, 1=OK, 2=warning, 3=critical
    uint32_t lastReceivedMs; // 受信時刻 (millis)
    uint32_t count;          // 受信フレーム数
  };
  GnssMonRf get_gnss_mon_rf();
  // GNSS DETAIL 3 に出す直近の受信状態（2026-11-14〜）。log.txt の `grf` 行（1 分まとめ）と同じ物差しを、直近 60 秒で出す。
  // 値はすべて自機の GNSS（ミラー中も送信機の値ではない）。リプレイ中は実 GNSS を読まないので valid が落ちる。
  struct GnssRfNow {
    bool     valid;   // MON-RF が 5 秒以内に届いているか
    uint8_t  jam;     // いまの jamInd（CW 妨害の指標 0..255）
    uint8_t  jam50;   // 直近 n 回（MON-RF は 1 秒に 1 回、最大 60）のうち jamInd >= 50 だった数（grf の jam50 と同じ物差し）
    uint8_t  n;
    uint16_t noise;   // noisePerMS
    uint16_t agc;     // agcCnt
    float    cno4;    // C/N0 上位 4 の平均 [dB-Hz]（12 秒以内に NAV-SAT に載った衛星）
    float    cnoall;  // C/N0 全体の平均 [dB-Hz]
    uint8_t  trk;     // 追尾している衛星（C/N0 > 0）の数
  };
  GnssRfNow gnss_rf_now();
  void      gnss_enable_mon_rf(uint8_t rate = 1);
  // 1 分ぶんの受信状態を log.txt の `grf` 行にする（60 秒ログから呼ぶ。debug/v7-noise-test と同じ行）
  void      gnss_log_rf_minute();

#endif