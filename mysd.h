// ============================================================
// File    : mysd.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : SDカード操作とCore1タスクキューのヘッダー。
//           タスク種別(TaskType)・タスク構造体・キュー定義と、
//           設定保存/読込・CSVフライトログ・起動ロゴ読込・
//           音声再生タスク生成関数のプロトタイプ宣言。
//           リプレイ再生の共有データ構造(ReplayRow/ReplayCol)と
//           選択画面の項目モデルもここで定義する。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================

#ifndef MYSD_H
  #define MYSD_H
  #include "display_tft.h"
  #include <Arduino.h>
  #include <limits.h>   // INT_MAX（removeDuplicateTask の既定引数）
  #include "lora_link/link_proto.h"

  typedef enum {
      TASK_NONE,
      TASK_INIT_SD,
      TASK_LOG_SD,
      TASK_LOG_SDF,
      TASK_SAVE_CSV,
      TASK_PLAY_MULTITONE,
      TASK_PLAY_WAV,
      TASK_STOP_WAV,      // 排他グループの音声を止める（案内の画面を抜けたとき。playWavArgs.excl_group）
      TASK_SAVE_SETTINGS,
      TASK_BROWSE_SD,
      TASK_BROWSE_REPLAY, // リプレイ選択画面用の CSV ファイル列挙
      TASK_LOAD_REPLAY,
      TASK_INIT_REPLAY,
      TASK_FLUSH_IMULOG,  // 生 IMU ログの二重バッファ片側を Core1 で SD へ書き出す
      TASK_SAVE_RXCSV,    // 無線で受信したテレメトリを received/ へ書き出す
      TASK_LATCH_BATT_OVERDISCHARGE, // 過放電（3.0V 未満）ラッチを Core1 で本体フラッシュへ記録する
      TASK_SERIAL_LIST,   // Web Serial: SD 内の CSV ファイル一覧をシリアル出力
      TASK_SERIAL_GET,    // Web Serial: 指定 CSV ファイルをシリアルへストリーミング送出
      TASK_SAVE_RTCBK,    // GNSS のバックアップ電池の推定（rtc_backup.cpp）を SD の rtc_backup.txt へ書く
      TASK_SAVE_DBD,      // GNSS の航法データベース（gnss_dbd.cpp が RAM にためたもの）を SD の GNSS_DBD_FILE へ書く
      TASK_LOAD_DBD       // 同じファイルを RAM へ読み直す（机の上の試験 PONS:DBD,COLD。起動時は setup1() から直接）
  } TaskType;


    // Structure to hold setting information
    typedef struct  {
        const char* id;                    // Setting identifier
        void (*setter)(const char*);       // Function to set the value
        void (*getter)(char*, size_t);     // Function to get the value
    } SDSetting;


    void setup_sd(int trycount, bool load_settings = true);
    bool get_sd_use_spi();    // 現在 SPI モードで動作中なら true（通常は SDIO = false）
    int  get_sd_setup_count(); // setup_sd() の累計呼び出し回数

    // ===== リプレイ再生（飛行CSVの直接再生） =====
    // CSV から取り出す列。replay_col_names[] の並びと一致させること。
    typedef enum {
        RCOL_LAT = 0, RCOL_LON, RCOL_GS, RCOL_TTRACK, RCOL_GNSSALT,
        RCOL_KFALT, RCOL_KFVS, RCOL_PRESS, RCOL_DATE, RCOL_TIME,
        RCOL_NUMSAT, RCOL_VOLT,
        // ★ 姿勢・風。自機飛行 CSV（data/）および受信ログ（received/）の両方に入る。
        RCOL_ROLL, RCOL_PITCH, RCOL_YAW,
        RCOL_PAVG, RCOL_RTRIM, RCOL_YAWACC, RCOL_WSPD, RCOL_WDIR,
        // 推定 TAS と大きい枠の中身（2026-11-07〜。docs/pons_tas.md §6）
        RCOL_TAS, RCOL_HDR,
        REPLAY_COL_COUNT
    } ReplayCol;

    // ReplayRow.have のビット。その列が CSV に存在し値が読めた場合に立つ。
    // 立っていない項目は再生時に実センサ値へフォールバックする（v5データは高度等を持たない）。
    // ★ RHAVE_* の定義は lora_link/link_proto.h へ移した（上で include 済み）。
    //   電波に載せるテレメトリでも同じビットを使うため、実体は共有ヘッダに 1 つだけ置く。
    //   2 か所に書くと必ずずれて、無線とリプレイで意味が食い違う。

    // CSV 1行分のリプレイデータ。Core1 が生成し Core0 が消費する。
    typedef struct {
        double   lat, lon;
        float    gs, ttrack;
        float    gnss_altitude, kf_altitude, kf_vspeed, pressure, voltage;
        // 姿勢ログ由来の値 [度]。有効かどうかは RHAVE_ATT* のビットで判断する。
        float    roll, pitch, yaw;
        float    pitch_avg, roll_trim, yaw_acc95;
        float    wind_mps, wind_dir;
        float    tas;    // 推定 TAS [m/s]（RHAVE_TAS）
        int8_t   hdr;    // 大きい枠の中身（tas.h の TasHdr）。-1 = 列が無い
        int      numsat;
        uint16_t have;   // RHAVE_* のビットマスク
        int      year, month, day, hour, minute, second, centisecond;
        uint32_t t_ms;   // CSV 先頭データ行からの経過ミリ秒
    } ReplayRow;

    // リプレイ選択画面の項目種別
    typedef enum {
        RITEM_NONE = 0,     // 空行（そのページに項目が無い）
        RITEM_OFF,          // リプレイもデモも止めて通常 GNSS に戻す
        RITEM_DEMO,         // デモ飛行の地点（選ぶとその地点で始める。replay_menu_demo_site）
        RITEM_FLIGHTONLY,   // 静止区間をスキップするか（YES/NO トグル）
        RITEM_SPEED,        // 再生速度の倍率（x1 / x2 / x?? トグル）
        RITEM_SOURCE,       // 再生元フォルダ（FLIGHT = 自機 / RECEIVED = 無線で受けた機体）
        RITEM_FILE,         // SD 上の飛行 CSV
        RITEM_RETURN        // 設定画面に戻る
    } ReplayItemType;

    void init_replay();
    void load_replay();
    bool browse_replay_files(int start_index);
    int  replay_menu_total_items();
    int  replay_menu_page_of(int index);
    int  replay_menu_page_count();
    int  replay_menu_file_start_for_page(int page);
    ReplayItemType replay_menu_item(int index, int page, char* label, size_t labelsize, int* filesize);
    int  replay_menu_demo_site(int index);   // デモの行なら地点（demo_site_t）、それ以外は 0（DEMO_OFF）
    // REPLAY/DEMO 画面の固定項目の通し番号（並びの説明は mysd.cpp の「項目モデル」）
    #define REPLAY_IDX_OFF         0
    #define REPLAY_IDX_DEMO_FIRST  1                                        // ここから REPLAY_DEMO_ROWS 行
    #define REPLAY_IDX_FLIGHTONLY  (REPLAY_IDX_DEMO_FIRST + REPLAY_DEMO_ROWS)
    #define REPLAY_IDX_SPEED       (REPLAY_IDX_FLIGHTONLY + 1)
    #define REPLAY_IDX_SOURCE      (REPLAY_IDX_SPEED + 1)
    static_assert(REPLAY_IDX_SOURCE + 1 == REPLAY_FIXED_COUNT, "REPLAY_IDX_* and REPLAY_FIXED_COUNT disagree");
    // 以下は Core0（gnss.cpp）から呼ぶリングバッファ操作
    bool     replay_available();
    uint32_t replay_peek_t_ms();
    bool     replay_pop(ReplayRow* out);
    bool     replay_buffer_has_space();
    const char* get_replay_filename();
    void     set_replay_filename(const char* name);
    bool     get_replay_flight_only();
    void     set_replay_flight_only(bool on);
    int      get_replay_speed();     // 再生速度の倍率（1 / 2 / REPLAY_SPEED_FAST）
    void     cycle_replay_speed();   // 倍率を次の候補へ切り替える
    // ★ 再生元フォルダ。一覧に出すのは常にどちらか一方だけ。
    //   こうしておくとページ計算が「1 フォルダぶん」のままで済み、
    //   今のインデックス計算に一切手を入れずに受信ログを扱える。
    void     toggle_replay_from_received();
    const char* replay_source_dir();          // "data" または "received"
    const char* replay_source_prefix();       // "data/" または "received/"
    const char* replay_filename_base();       // 再生中ファイルのフォルダを除いた部分
    bool        replay_filename_is(const char* name);  // 一覧の name が再生中か（フルパスで比較）
    void     replay_set_paused(bool paused);  // 再生の一時停止（設定画面表示中など）

    bool browse_sd(int page);
    void log_sd(const char* text);
    void log_sdf(const char* format, ...);

    // Forward declarations of example getter/setter functions
    void setVolume(const char* value);
    void getVolume(char* buffer, size_t bufferSize);
    void setVarioVolume(const char* value);
    void getVarioVolume(char* buffer, size_t bufferSize);
    void setDestination(const char* value);
    void getDestination(char* buffer, size_t bufferSize);
    void setNavigationMode(const char* value);
    void getNavigationMode(char* buffer, size_t bufferSize);
    void setScaleIndex(const char* value);
    void getScaleIndex(char* buffer, size_t bufferSize);
    void setUpwardMode(const char* value);
    void getUpwardMode(char* buffer, size_t bufferSize);
    void setVarioInhibit(const char* value);
    void getVarioInhibit(char* buffer, size_t bufferSize);
    void setKfQVel(const char* value);
    void getKfQVel(char* buffer, size_t bufferSize);
    void setKfQBias(const char* value);
    void getKfQBias(char* buffer, size_t bufferSize);
    void setKfR(const char* value);
    void getKfR(char* buffer, size_t bufferSize);
    // 機体ゼロ点（マウント基準）のオフセット [度]。据え付け後に一度較正して永続化する。
    void setLevelRoll(const char* value);
    void getLevelRoll(char* buffer, size_t bufferSize);
    void setLevelPitch(const char* value);
    void getLevelPitch(char* buffer, size_t bufferSize);
    // 較正時に申告するピッチ角（IMU/ESKF 画面の SET PITCH 行の値）
    void setPitchTarget(const char* value);
    void getPitchTarget(char* buffer, size_t bufferSize);
    // Roll/Pitch/Yaw 機能のマスタースイッチ
    void setRpyFunctions(const char* value);
    void getRpyFunctions(char* buffer, size_t bufferSize);
    // 直進中のロール自動トリムの有効/無効
    void setAutoRollTrim(const char* value);
    void getAutoRollTrim(char* buffer, size_t bufferSize);
    // 風推定の有効/無効
    void setWindEstimate(const char* value);
    void getWindEstimate(char* buffer, size_t bufferSize);
    void setHeaderTas(const char* value);    // ヘッダー右の大きい枠（0 = 平均ピッチ、1 = 推定 TAS）
    void getHeaderTas(char* buffer, size_t bufferSize);
    // APPLY 時に申告するロール角
    void setRollTarget(const char* value);
    void getRollTarget(char* buffer, size_t bufferSize);
    // マウントから外されたまま APPLY されていない状態
    void setNeedsApply(const char* value);
    void getNeedsApply(char* buffer, size_t bufferSize);
    // 最後に APPLY した日（JST の YYYYMMDD、0 = 不明）
    void setCalibDate(const char* value);
    void getCalibDate(char* buffer, size_t bufferSize);
    // 対気速度モデル V(θ)=V0*sqrt(K/(K+θ)) の係数（機体依存。既定は settings.h）
    void setAirspeedV0(const char* value);
    void getAirspeedV0(char* buffer, size_t bufferSize);
    void setAirspeedK(const char* value);
    void getAirspeedK(char* buffer, size_t bufferSize);
    void setAirspeedMin(const char* value);
    void getAirspeedMin(char* buffer, size_t bufferSize);
    void setAirspeedMax(const char* value);
    void getAirspeedMax(char* buffer, size_t bufferSize);
    // 自動ロールトリムの累積補正量（度）
    void setRollTrim(const char* value);
    void getRollTrim(char* buffer, size_t bufferSize);
    // AUTO10K の折返しフェーズ（"AWAY" / "INTO"）
    void setAuto10kStatus(const char* value);
    void getAuto10kStatus(char* buffer, size_t bufferSize);
    bool loadSettings();
    bool saveSettings();


  // ピーク時の見積り: コース警告(4トーン+WAV=5) + AUTO10K折返し(5) + バンク警告(2)
  // が近接し、背景で CSV(2Hz) + 姿勢ログ(5Hz) + テキストログが流れる。Core1 が SD 書き込みで
  // 数百ms 止まると 20 本では溢れ得た。溢れると音（＝一番鳴ってほしい警告）が捨てられるため 40 にする。
  // sizeof(Task)=264B（logSdfArgs の char[256] が最大メンバ）なので 40 本で 10.3KB。
  // 20 本(5.2KB)からの増加は +5.2KB。RP2350 の SRAM 520KB に対し約 1%。
  // ※ 1 本あたりが大きいのは logSdfArgs のバッファのため。さらに増やしたい場合は
  //   本数より先にそのバッファを見直す方が効率が良い。
  #define TASK_QUEUE_SIZE 40


  typedef struct {
      TaskType type;
      union {
          int pagenum;                         //For browsesd
          const char* logText;               // For log_sd
          struct {                           // For log_sdf
              const char* format;
              char buffer[256];
          } logSdfArgs;
          struct {                           // For saveCSV
              float latitude;
              float longitude;
              float gs;
              int ttrack;  // 真方位（true track）を格納
              float gnss_altitude;
              float kf_altitude;  // KF推定高度 [m]（気圧基準）
              float kf_vspeed;   // KF推定上昇率 [m/s]
              float pressure;
              float voltage;      // バッテリー電圧 [V]（リプレイで当時の電池表示を再現するため）
              int numsat;         // 測位に使用した衛星数（同上）
              int year, month, day, hour, minute, second, centisecond;
              // ---- 飛行 CSV だけの追加列（不具合の検証用。v7.1 の照度センサーとバックライト）----
              float    lux;       // 照度 [lx]。センサーが応答していないときは負（空欄）
              int8_t   bl_pct;    // バックライトの点灯率 [%]
              // ---- 受信ログ（received/）用の追加列。TASK_SAVE_RXCSV でのみ使う ----
              int      rssi;      // 受信強度 [dBm]
              uint16_t seq;       // 送信側の連番。取りこぼしの解析に使う
              uint16_t age_ms;    // 受信から書き出しまでの経過
              // ★ 姿勢・風。自機飛行 CSV（data/）と受信ログ（received/）の両方で使う。
              //   未収束の項目は have ビットを立てず、書き出し側で空欄にする。
              float    roll, pitch, yaw;
              float    pitch_avg, roll_trim, yaw_acc95;
              float    wind_mps, wind_dir;
              // 推定 TAS（RHAVE_TAS）と、大きい枠に何を出していたか（tas.h の TasHdr）。
              // 受信ログでは送信機の推定 TAS と、ボートの大きい枠に出していたもの。
              float    tas;
              uint8_t  hdr;
              uint16_t have;      // RHAVE_ATT* / RHAVE_TAS のどれが入っているか
              // ★ 受信した瞬間の**ボート自身**の位置と、機体までの距離。
              //   これがあると飛行後に「距離 vs RSSI」がそのまま引ける。
              //   要求 200m の裏取り、SF を上げるべきかの判断、アンテナ位置の
              //   比較が 1 つの CSV で完結する。own_valid=false なら空欄で書く。
              bool     own_valid;
              float    own_lat, own_lon;
              float    dist_m;    // 機体までの距離 [m]
              // 受けた 32 バイトのフレームの回転枠の番号（0〜7。lora_link/link_wire2.h）。
              //   回転枠の項目（日付・平均ピッチ・風・電圧・気圧など）は、その枠の行で更新された値。
              uint8_t  slot;
          } saveCsvArgs;
          struct {
              int freq;
              int duration;
              int counter;
              int priority;
              int min_volume;  // 最低保証ボリューム（0=制限なし）
              bool solo_play;  // true のとき WAV との同時再生を禁止する
          } playMultiToneArgs;
          struct{
              const char* wavfilename;
              int priority;
              int min_volume;  // 最低保証ボリューム（0=制限なし）
              int excl_group;  // 排他グループ（settings.h の WAV_EXCL_*）。0=排他なし
          }playWavArgs;
          struct {                           // For imulog_write_buffer
              int  bufidx;                   // 書き出す二重バッファの索引（0 or 1）
              int  year, month, day, hour, minute, second;  // ファイルタイムスタンプ用
              char filename[24];             // "imuraw/20260316.bin" = 19文字
          } imuLogArgs;
          struct {                           // For TASK_SAVE_RTCBK
              float    hours;                // あと何時間もつか（推定）
              uint32_t utc;                  // その値の時刻（UTC、1970 年からの秒）
          } rtcBkArgs;
          struct {                           // For TASK_SERIAL_GET
              char filename[64];             // 送信対象ファイル名（相対パス含む）
              uint32_t tail;                 // 0 = 全部。それ以外は末尾のこのバイト数だけ（行の頭から。PONS:GETTAIL）
          } serialGetArgs;
      };
  } Task;


  typedef struct {
      Task tasks[TASK_QUEUE_SIZE];
      volatile int head;
      volatile int tail;
  } TaskQueue;


  Task createSaveSettingTask();
  Task createLogSdTask(const char* logText);
  Task createLogSdfTask(const char* format, ...) __attribute__((format(printf, 1, 2)));
  Task createSaveCsvTask(float latitude, float longitude, float gs, int ttrack, float gnss_altitude, float kf_altitude, float kf_vspeed, float pressure, float voltage, int numsat, int year, int month, int day, int hour, int minute, int second, int centisecond);

  // 無線で受信したテレメトリを received/ へ書き出すタスク。
  // 列は自機のフライト CSV と同じ並びにし、末尾に rssi/seq/age/src を足す。
  // 揃えてあるので、既存のリプレイ機能と解析ツールがそのまま使える（docs/pons_link.md §5）。
  // slot = 32 バイトのフレームの回転枠の番号（分からなければ 0xFF）
  Task createSaveRxCsvTask(const LinkTelem* t, int rssi, uint16_t age_ms, uint8_t slot);
  // 飛行 CSV の 1 行を書く。引数は saveRxCSV と同じく Task そのもの
  //（17 個並べると、項目を足したとき呼び出し側とずれても気づけないため）。
  void saveCSV(const Task& tk);
  void saveRxCSV(const Task& tk);
  Task createPlayMultiToneTask(int freq, int duration, int count,int priority=1,int min_volume=0,bool solo_play=false);
  Task createPlayWavTask(const char* filename,int priority=1,int min_volume=0,
                         int excl_group=WAV_EXCL_NONE);
  // 排他グループ excl_group（WAV_EXCL_*）の音声を、再生中のものも待っているものも止める。
  // 案内の画面を抜けたときに、そのページの読み上げを後に残さないため（TAS の案内）。
  Task createStopWavTask(int excl_group);
  Task createBrowseSDTask(int page);
  Task createBrowseReplayTask(int start_index);
  Task createLoadReplayTask();
  Task createInitReplayTask();
  Task createFlushImuLogTask(int bufidx, const char* filename,
                             int year, int month, int day, int hour, int minute, int second);
  Task createLatchBattTask();
  Task createSaveRtcBackupTask(float hours, uint32_t utc);
  // GNSS のバックアップ電池の推定の記録（SD の RTC_BACKUP_FILE。Core1 専用）。
  // load は起動時の setup_sd() の直後に 1 回。読めなくても rtc_backup_set_loaded_core1(false, ...) を必ず呼ぶ。
  void rtc_backup_load_sd();
  void rtc_backup_save_sd(float hours, uint32_t utc);
  // GNSS の航法データベース（SD の GNSS_DBD_FILE。Core1 専用。中身の受け渡しは gnss_dbd.h の *_core1）。
  // load は起動時の setup_sd() の直後に 1 回と、試験の TASK_LOAD_DBD。読めなくても gnss_dbd_loaded_core1(false, ...) を必ず呼ぶ。
  Task createSaveDbdTask();
  Task createLoadDbdTask();
  void gnss_dbd_load_sd();
  void gnss_dbd_save_sd();
  Task createSerialListTask();
  Task createSerialGetTask(const char* filename, uint32_t tail = 0);   // tail: 末尾だけ送る [B]（0 = 全部）
  void serial_list_csv_files();
  void serial_send_csv_file(const char* filename, uint32_t tail = 0);

  // Web Serial 通信状態フラグ（Core0 / Core1 共有）
  extern volatile bool webserial_connected;
  extern volatile bool webserial_cancel_requested;

  // Functions to handle the queue (declarations)
  // 戻り値: キューに入れられたら true、満杯で捨てたら false。
  // 捨てられたタスクは実行されない。後始末が要る呼び出し側は戻り値を見ること。
  // 設定ファイル（settings.txt）を読み終えたか。Core1 が立て、Core0 が読む。
  // 通常は startup_demo_tft() の sd_setup_complete 待ちで true になってから
  // link_setup() が走る。あの待ちが 5 秒で切れたときだけ false のまま来るので、
  // link_setup() がログに残して後から分かるようにしてある。
  extern volatile bool settings_loaded;
  bool enqueueTask(Task task);
  // type が同じで、かつ優先度が max_priority 以下のタスクをキューから消す。
  // 多重トーン以外は優先度を見ない（置き場所が種別ごとに違うため）。
  void removeDuplicateTask(TaskType type, int max_priority = INT_MAX);
  bool enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
  void sd_note_write_error();   // 開いたファイルへの書き込みが失敗した → SD の異常にする（Core1。BUG-119）
  bool isTaskRunning(int taskType);
  bool isTaskInQueue(int taskType);
  void clearCurrentTask();  // Core1 がタスク完了時に呼ぶ（currentTask.type = TASK_NONE）




  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern volatile bool sd_setup_complete;

  // リプレイ再生用の共有状態（Core1 が生成 / Core0 が消費）
  extern volatile ReplayRow replay_rows[REPLAY_BUF_SIZE];
  extern volatile uint8_t replay_head, replay_tail;
  extern volatile bool replay_eof;
  // リプレイを続けられない。Core0 が見て解除し、理由に応じたログを残す。
  // ★ bool ではなく理由を持たせてある。「ファイルが壊れている」と
  //   「SD が読めない」は現場での対処が違うのに、ログが同じだと区別できない。
  enum ReplayBadReason : uint8_t {
      REPLAY_BAD_NONE = 0,   // 問題なし
      REPLAY_BAD_FILE,       // 開けない / ヘッダが無い / 空 / 未選択
      REPLAY_BAD_NOSD,       // SD が使えない（未初期化・書き込みエラー・カード抜け）
  };
  extern volatile uint8_t replay_file_bad;
  extern volatile uint32_t replay_init_seq;  // init_replay() のたびに加算（再生時計のリセット通知）
  extern char replay_filename[REPLAY_FILENAME_LEN];

  // リプレイ選択画面のファイル一覧（browse_replay_files() が更新）
  extern char replayfiles[REPLAY_LIST_ROWS][32];
  extern int  replayfiles_size[REPLAY_LIST_ROWS];
  extern volatile int  replayfiles_count;  // 現在 replayfiles[] に入っている件数（Core1 が更新）
  extern volatile int  replayfiles_total;  // SD 上の対象 CSV の総数（Core1 が更新）

#endif