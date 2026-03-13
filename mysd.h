// ============================================================
// File    : mysd.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : SDカード操作とCore1タスクキューのヘッダー。
//           タスク種別(TaskType)・タスク構造体・キュー定義と、
//           設定保存/読込・CSVフライトログ・BMP地図画像読込・
//           音声再生タスク生成関数のプロトタイプ宣言。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================

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
      TASK_LOAD_MAPIMAGE,
      TASK_PLAY_MULTITONE,
      TASK_PLAY_WAV,
      TASK_SAVE_SETTINGS,
      TASK_BROWSE_SD,
      TASK_LOAD_REPLAY,
      TASK_INIT_REPLAY
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

    void init_replay();
    void load_replay();
    bool browse_sd(int page);
    void log_sd(const char* text);
    void log_sdf(const char* format, ...);
    void saveCSV(float latitude, float longitude,float gs,int ttrack, float altitude, float pressure, int numsat, float voltage, int year, int month, int day, int hour, int minute, int second, int centisecond);
    void load_mapimage(double center_lat, double center_lon,int zoomlevel);

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
    bool loadSettings();
    bool saveSettings();


  #define TASK_QUEUE_SIZE 20  // ピーク時（コース警告4トーン+WAV+地図+CSV+ログ）で約10タスク。余裕を持たせて20。RAM増加は+2.6KBのみ。


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
              int mtrack;
              float altitude;
              float pressure;
              int numsats;
              float voltage;
              int year, month, day, hour, minute, second, centisecond;
          } saveCsvArgs;
          struct {                           // For load_mapimage
              double center_lat;
              double center_lon;
              int zoomlevel;
          } loadMapImageArgs;
          struct {
              int freq;
              int duration;
              int counter;
              int priority;
              int min_volume;  // 最低保証ボリューム（0=制限なし）
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
  Task createSaveCsvTask(float latitude, float longitude, float gs, int mtrack, float altitude, float pressure, int numsats, float voltage, int year, int month, int day, int hour, int minute, int second, int centisecond);
  Task createLoadMapImageTask(double center_lat, double center_lon, int zoomlevel);
  Task createPlayMultiToneTask(int freq, int duration, int count,int priority=1,int min_volume=0);
  Task createPlayWavTask(const char* filename,int priority=1,int min_volume=0);
  Task createBrowseSDTask(int page);
  Task createLoadReplayTask();
  Task createInitReplayTask();

  // Functions to handle the queue (declarations)
  void enqueueTask(Task task);
  void enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
  bool isTaskRunning(int taskType);
  bool isTaskInQueue(int taskType);
  void load_push_logo();



  double pixelsPerDegreeLat(int zoom,double latitude);

  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern TFT_eSprite gmap_sprite;
  extern char replay_nmea[128];
  extern volatile unsigned long replay_seekpos;
  extern volatile bool loaded_replay_nmea;
  extern volatile bool gmap_loaded_active;
  extern volatile bool new_gmap_ready;
#endif