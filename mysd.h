// ============================================================
// File    : mysd.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : SDカード操作とCore1タスクキューのヘッダー。
//           タスク種別(TaskType)・タスク構造体・キュー定義と、
//           設定保存/読込・CSVフライトログ・起動ロゴ読込・
//           音声再生タスク生成関数のプロトタイプ宣言。
//           リプレイ再生の共有データ構造(ReplayRow/ReplayCol)と
//           選択画面の項目モデルもここで定義する。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================

#ifndef MYSD_H
  #define MYSD_H
  #include "display_tft.h"
  #include <Arduino.h>
  #include "lora_link/link_proto.h"

  typedef enum {
      TASK_NONE,
      TASK_INIT_SD,
      TASK_LOG_SD,
      TASK_LOG_SDF,
      TASK_SAVE_CSV,
      TASK_PLAY_MULTITONE,
      TASK_PLAY_WAV,
      TASK_SAVE_SETTINGS,
      TASK_BROWSE_SD,
      TASK_BROWSE_REPLAY, // リプレイ選択画面用の CSV ファイル列挙
      TASK_LOAD_REPLAY,
      TASK_INIT_REPLAY,
      TASK_LOG_IMUREPLAY,
      TASK_FLUSH_IMULOG,  // 生 IMU ログの二重バッファ片側を Core1 で SD へ書き出す
      TASK_SAVE_RXCSV     // 無線で受信したテレメトリを received/ へ書き出す
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
        // ★ 姿勢・風。**飛行 CSV には存在しない**（別ファイル imu_replaydata/ にある）。
        //   受信ログ received/ にだけ入っていて、列名は imu_replaydata/ と揃えてある。
        //   無い列は -1 のままなので、飛行 CSV の再生は今までと 1 ミリも変わらない。
        RCOL_ROLL, RCOL_PITCH, RCOL_YAW,
        RCOL_PAVG, RCOL_RTRIM, RCOL_YAWACC, RCOL_WSPD, RCOL_WDIR,
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
        int      numsat;
        uint16_t have;   // RHAVE_* のビットマスク
        int      year, month, day, hour, minute, second, centisecond;
        uint32_t t_ms;   // CSV 先頭データ行からの経過ミリ秒
    } ReplayRow;

    // リプレイ選択画面の項目種別
    typedef enum {
        RITEM_NONE = 0,     // 空行（そのページに項目が無い）
        RITEM_OFF,          // リプレイ解除（通常 GPS に戻す）
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
    // 以下は Core0（gps.cpp）から呼ぶリングバッファ操作
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
    void saveCSV(float latitude, float longitude, float gs, int ttrack, float gnss_altitude, float kf_altitude, float kf_vspeed, float pressure, float voltage, int numsat, int year, int month, int day, int hour, int minute, int second, int centisecond);
    // リプレイで画面を再現するための ESKF 結果ログ（imu_replaydata/YYYYMMDD.txt, 5Hz）。
    // pitch_avg は 平均が溜まるまで無効。valid=false のときは空欄で書く。
    void save_imu_replaydata(int h, int m, int s, int cs,
                             float roll, float pitch, float yaw,
                             float pitch_avg, bool pitch_avg_valid,
                             float roll_trim, float yaw_acc95,
                             float wind_mps, float wind_dir, bool wind_valid,
                             const char* filename, int year, int month, int day);

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
    // APPLY 時に申告するロール角
    void setRollTarget(const char* value);
    void getRollTarget(char* buffer, size_t bufferSize);
    // マウントから外されたまま APPLY されていない状態
    void setNeedsApply(const char* value);
    void getNeedsApply(char* buffer, size_t bufferSize);
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
              // ---- 受信ログ（received/）用の追加列。TASK_SAVE_RXCSV でのみ使う ----
              int      rssi;      // 受信強度 [dBm]
              uint16_t seq;       // 送信側の連番。取りこぼしの解析に使う
              uint16_t age_ms;    // 受信から書き出しまでの経過
              // ★ 姿勢・風。機体の SD が死んでもボート側に姿勢が残るようにする。
              //   列名と「未収束は空欄」の流儀は imu_replaydata/ に合わせてあるので、
              //   ESKF の解析ツールをそのまま流用できる。
              float    roll, pitch, yaw;
              float    pitch_avg, roll_trim, yaw_acc95;
              float    wind_mps, wind_dir;
              uint16_t have;      // RHAVE_ATT* のどれが入っているか
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
          }playWavArgs;
          struct {                           // For save_imu_replaydata
              int hour, minute, second, centisecond;
              int year, month, day;          // ファイルタイムスタンプ設定用
              float roll, pitch, yaw;
              float pitch_avg, roll_trim, yaw_acc95;
              float wind_mps, wind_dir;
              bool  pitch_avg_valid, wind_valid;
              // "imu_replaydata/20260316.txt" = 27文字 + NUL。24 だと溢れるので 32。
              char filename[32];
          } imuReplayArgs;
          struct {                           // For imulog_write_buffer
              int  bufidx;                   // 書き出す二重バッファの索引（0 or 1）
              int  year, month, day, hour, minute, second;  // ファイルタイムスタンプ用
              char filename[24];             // "imuraw/20260316.bin" = 19文字
          } imuLogArgs;
      };
  } Task;


  typedef struct {
      Task tasks[TASK_QUEUE_SIZE];
      volatile int head;
      volatile int tail;
  } TaskQueue;


  Task createSaveSettingTask();
  Task createLogSdTask(const char* logText);
  Task createLogSdfTask(const char* format, ...);
  Task createSaveCsvTask(float latitude, float longitude, float gs, int ttrack, float gnss_altitude, float kf_altitude, float kf_vspeed, float pressure, float voltage, int numsat, int year, int month, int day, int hour, int minute, int second, int centisecond);

  // 無線で受信したテレメトリを received/ へ書き出すタスク。
  // 列は自機のフライト CSV と同じ並びにし、末尾に rssi/seq/age/src を足す。
  // 揃えてあるので、既存のリプレイ機能と解析ツールがそのまま使える（docs/pons_link.md §5）。
  Task createSaveRxCsvTask(const LinkTelem* t, int rssi, uint16_t age_ms);
  void saveRxCSV(const Task& tk);
  Task createPlayMultiToneTask(int freq, int duration, int count,int priority=1,int min_volume=0,bool solo_play=false);
  Task createPlayWavTask(const char* filename,int priority=1,int min_volume=0);
  Task createBrowseSDTask(int page);
  Task createBrowseReplayTask(int start_index);
  Task createLoadReplayTask();
  Task createInitReplayTask();
  Task createLogImuReplayTask(int h, int m, int s, int cs,
                              float roll, float pitch, float yaw,
                              float pitch_avg, bool pitch_avg_valid,
                              float roll_trim, float yaw_acc95,
                              float wind_mps, float wind_dir, bool wind_valid,
                              const char* filename, int year, int month, int day);
  Task createFlushImuLogTask(int bufidx, const char* filename,
                             int year, int month, int day, int hour, int minute, int second);

  // Functions to handle the queue (declarations)
  // 戻り値: キューに入れられたら true、満杯で捨てたら false。
  // 捨てられたタスクは実行されない。後始末が要る呼び出し側は戻り値を見ること。
  bool enqueueTask(Task task);
  bool enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
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