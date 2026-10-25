// ============================================================
// File    : mysd.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : SDカード操作とCore1タスクキューのヘッダー。
//           タスク種別(TaskType)・タスク構造体・キュー定義と、
//           設定保存/読込・CSVフライトログ・起動ロゴ読込・
//           音声再生タスク生成関数のプロトタイプ宣言。
//           リプレイ再生の共有データ構造(ReplayRow/ReplayCol)と
//           選択画面の項目モデルもここで定義する。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================

#ifndef MYSD_H
  #define MYSD_H
  #include "display_tft.h"
  #include <Arduino.h>
  #include <limits.h>   // INT_MAX（removeDuplicateTask の既定引数）

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
      TASK_LOAD_LOGO      // 起動時ロゴ BMP を Core1 で SD 読み込みするタスク
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
        RCOL_DATE, RCOL_TIME, RCOL_NUMSAT, RCOL_VOLT,
        REPLAY_COL_COUNT
    } ReplayCol;

    // ReplayRow.have のビット。その列が CSV に存在し値が読めた場合に立つ。
    // 立っていない項目は再生時に実際の値へフォールバックする（古い v5 データは高度などを持たない）。
    // ※ v6 / v7 の CSV の気圧・KF の列は読まない（v5 には表示する場所が無い）。
    #define RHAVE_GS     0x0001
    #define RHAVE_TTRACK 0x0002
    #define RHAVE_GNSSALT    0x0004
    #define RHAVE_DATE   0x0040
    #define RHAVE_NUMSAT 0x0080
    #define RHAVE_VOLT   0x0100

    // CSV 1行分のリプレイデータ。Core1 が生成し Core0 が消費する。
    typedef struct {
        double   lat, lon;
        float    gs, ttrack;
        float    gnss_altitude, voltage;
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
        RITEM_2025,         // 固定項目: 2025 大会データ
        RITEM_2026,         // 固定項目: 2026 大会データ
        RITEM_FILE,         // SD 上の飛行 CSV
        RITEM_RETURN        // 設定画面に戻る
    } ReplayItemType;

    void init_replay();
    void load_replay();
    bool browse_replay_files(int start_index);
    // 一覧の先頭に並ぶ固定項目数（3〜REPLAY_FIXED_COUNT）。
    // 大会データ（2025/2026）は SD 上に実在するときだけ数に入る。
    int  replay_menu_fixed_count();
    // 大会データが SD 上にあるか。browse_replay_files() が更新する。
    extern volatile bool replay_have_2025;
    extern volatile bool replay_have_2026;
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
    void     replay_set_paused(bool paused);  // 再生の一時停止（設定画面表示中など）

    bool browse_sd(int page);
    void log_sd(const char* text);
    void log_sdf(const char* format, ...);
    void saveCSV(float latitude, float longitude, float gs, int ttrack, float gnss_altitude, float hacc_m, float voltage, int numsat, int year, int month, int day, int hour, int minute, int second, int centisecond);

    // Forward declarations of example getter/setter functions
    void setVolume(const char* value);
    void getVolume(char* buffer, size_t bufferSize);
    void setDestination(const char* value);
    void getDestination(char* buffer, size_t bufferSize);
    void setNavigationMode(const char* value);
    void getNavigationMode(char* buffer, size_t bufferSize);
    void setScaleIndex(const char* value);
    void getScaleIndex(char* buffer, size_t bufferSize);
    void setUpwardMode(const char* value);
    void getUpwardMode(char* buffer, size_t bufferSize);
    // AUTO10K の折返しフェーズ（"AWAY" / "INTO"）
    void setAuto10kStatus(const char* value);
    void getAuto10kStatus(char* buffer, size_t bufferSize);
    bool loadSettings();
    bool saveSettings();


  // ピーク時の見積り（v6 の値）: コース警告(4トーン+WAV=5) + AUTO10K折返し(5) + 電源の警報
  // が近接し、背景で CSV + テキストログが流れる。Core1 が SD 書き込みで
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
              float hacc_m;       // GST の水平精度推定 [m]。負 = 分からない（空欄で書く）
              float voltage;      // バッテリー電圧 [V]（リプレイで当時の電池表示を再現するため）
              int numsat;         // 測位に使用した衛星数（同上）
              int year, month, day, hour, minute, second, centisecond;
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
  Task createSaveCsvTask(float latitude, float longitude, float gs, int ttrack, float gnss_altitude, float hacc_m, float voltage, int numsat, int year, int month, int day, int hour, int minute, int second, int centisecond);
  Task createPlayMultiToneTask(int freq, int duration, int count,int priority=1,int min_volume=0,bool solo_play=false);
  Task createPlayWavTask(const char* filename,int priority=1,int min_volume=0);
  Task createBrowseSDTask(int page);
  Task createBrowseReplayTask(int start_index);
  Task createLoadReplayTask();
  Task createInitReplayTask();
  Task createLoadLogoTask();

  // Functions to handle the queue (declarations)
  // 戻り値: キューに入れられたら true、満杯で捨てたら false。
  // 捨てられたタスクは実行されない。後始末が要る呼び出し側は戻り値を見ること。
  // 設定ファイル（settings.txt）を読み終えたか。Core1 が立て、Core0 が読む。
  extern volatile bool settings_loaded;
  // キュー満杯で捨てたタスクの累計（v5 は USERLED が無いので、件数を後から log.txt に残す）。
  extern volatile uint32_t task_drop_count;
  bool enqueueTask(Task task);
  // type が同じで、かつ優先度が max_priority 以下のタスクをキューから消す。
  // 多重トーン以外は優先度を見ない（置き場所が種別ごとに違うため）。
  void removeDuplicateTask(TaskType type, int max_priority = INT_MAX);
  bool enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
  bool isTaskRunning(int taskType);
  bool isTaskInQueue(int taskType);
  void clearCurrentTask();  // Core1 がタスク完了時に呼ぶ（currentTask.type = TASK_NONE）
  void load_push_logo();




  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern volatile bool sd_setup_complete;
  extern volatile bool logo_ready;
  extern TFT_eSprite logo_sprite;

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