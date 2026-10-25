
#ifndef MYSD_H
  #define MYSD_H
  #include "display_tft.h"
  #include <Arduino.h>

  typedef enum {
      TASK_NONE,
      TASK_INIT_SD,
      TASK_LOG_SD,
      TASK_LOG_SDF,
      TASK_SAVE_CSV,
      TASK_SAVE_SETTINGS,
      TASK_REPLAY_INIT,     // リプレイのファイルを開き直す（名前が空なら閉じる）
      TASK_REPLAY_LOAD      // リプレイの行を読み足す
  } TaskType;

  // SD書き込み遅延時にログ・CSV・設定保存が重なっても溢れないよう 40 に拡張（v7 と同じ）
  #define TASK_QUEUE_SIZE 40

  // 飛行 CSV の 1 行（gps.cpp が作る）。列名は v7 にそろえてある（v7 のツールとリプレイが列名で読む）。
  struct CsvRow {
      double latitude, longitude;
      float gs, truetrack, altitude;
      float voltage;          // 電池電圧。読めないとき（SW OFF のまま USB・電源の異常）は NAN（空欄）
      int numsat;
      float hacc, sacc;       // NAV-PVT の水平精度 [m] / 速度精度 [m/s]
      int year, month, day, hour, minute, second;   // JST
  };

  // ===== リプレイ（SD の飛行 CSV を本体で再生する。v7 の 0.92）=====
  // Core1 が CSV を読んで 1 行ずつリングに積み、Core0（gps.cpp）が時刻どおりに取り出して GPS の値として使う。
  // リングは producer（Core1）と consumer（Core0）が 1 つずつなので mutex は要らない
  // （スロットへ書いてから head を進める順を守る）。
  #define REPLAY_MAX_FILES 30
  #define REPLAY_NAME_LEN 24          // "YYYY-MM-DD_HHMM.csv" は 19 文字
  #define REPLAY_BUF_SIZE 16          // 2 の累乗
  struct ReplayRow {
      double latitude, longitude;
      float gs, truetrack, altitude, hacc, sacc;   // 列が無いものは NAN
      int numsat;                                  // 列が無ければ -1
      uint32_t t_ms;                               // 先頭行からの経過 [ms]（長い途切れは詰めてある）
      int year, month, day, hour, minute, second;  // CSV のまま（JST）。日付が無ければ year=0
  };
  extern ReplayRow replay_rows[REPLAY_BUF_SIZE];
  extern volatile uint8_t replay_head, replay_tail;   // head は Core1、tail は Core0 だけが進める
  extern volatile bool replay_eof;                    // ファイルの終わりまで読んだ
  extern volatile bool replay_bad;                    // 開けない・列が足りない（再生できない）
  extern volatile bool replay_load_pending;           // TASK_REPLAY_LOAD を積んでまだ終わっていない
  extern volatile uint32_t replay_init_seq;           // TASK_REPLAY_INIT を終えるたびに増える
  int replay_file_count();                            // SD のルートにある飛行 CSV の数（起動時に数える）
  const char* replay_file_name(int i);

  // settings.txt に残す設定（v7 と同じ "id:value" 形式）。Core0 がその時点の値を写して積み、
  // Core1 がそのまま書く（Core1 から Core0 の変数を読まない）。デモは残さない（電源を入れ直せば本番の設定に戻る）。
  struct SavedSettings {
      char destination[24];   // 目的地の名前（SD の目的地は並びが変わりうるので名前で持つ）
      int navigation_mode;    // DMODE_*（読めなかったら -1）
      int auto10k_status;     // AUTO10K_*。実際の位置でのフェーズ（読めなかったら -1）
      int upward_mode;        // 0=TRACK UP, 1=NORTH UP（読めなかったら -1）
      int scaleindex;         // 地図の縮尺（読めなかったら -1）
  };


  typedef struct {
      TaskType type;
      union {
          const char* logText;               // For log_sd
          struct {                           // For log_sdf
              const char* format;
              char buffer[256];
          } logSdfArgs;
          CsvRow saveCsvArgs;                // For saveCSV
          SavedSettings saveSettingsArgs;    // For saveSettings
          char replayName[REPLAY_NAME_LEN];  // For replay_init
      };
  } Task;


  typedef struct {
      Task tasks[TASK_QUEUE_SIZE];
      volatile int head;
      volatile int tail;
  } TaskQueue;


  Task createLogSdTask(const char* logText);
  Task createSaveCsvTask(const CsvRow& row);
  Task createSaveSettingsTask();   // Core0 で呼ぶ（その時点の設定を写す）
  // 起動時に SD から読んだ設定を 1 回だけ受け取る（Core0 で反映するため）。まだ読んでいなければ false。
  bool take_loaded_settings(SavedSettings* out);
  Task createReplayInitTask(const char* name);   // name が空なら閉じるだけ

  // Functions to handle the queue (declarations)
  bool enqueueTask(const Task& task);   // 積めたら true（キューが満杯なら捨てて false）。Task は大きいので参照で渡す
  bool enqueueLogSdf(const char* format, ...);   // 書式つきのログを積む（Task を呼び出し側のスタックに置かない）
  void enqueueTaskWithAbortCheck(const Task& task);
  bool dequeueTask(Task *task);

  bool good_sd();
  bool isTaskRunning(int taskType);

  /*
  void log_sd(const char* text);
  void log_sdf(const char* format, ...);
  void saveCSV(const CsvRow& row);
  */


  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern volatile unsigned long loop1counter;
  extern volatile int loop1pos;
  extern volatile int loop0pos;
#endif