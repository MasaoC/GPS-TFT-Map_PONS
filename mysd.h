
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
      TASK_BROWSE_SD
  } TaskType;


    // Structure to hold setting information
    typedef struct  {
        const char* id;                    // Setting identifier
        void (*setter)(const char*);       // Function to set the value
        void (*getter)(char*, size_t);     // Function to get the value
    } SDSetting;


    void setup_sd(int trycount);

    bool browse_sd(int page);
    void log_sd(const char* text);
    void log_sdf(const char* format, ...);
    void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second);
    void load_mapimage(double center_lat, double center_lon,int zoomlevel);

    // Forward declarations of example getter/setter functions
    void setVolume(const char* value);
    void getVolume(char* buffer, size_t bufferSize);
    void setDestination(const char* value);
    void getDestination(char* buffer, size_t bufferSize);
    void setDestinationMode(const char* value);
    void getDestinationMode(char* buffer, size_t bufferSize);
    void setScaleIndex(const char* value);
    void getScaleIndex(char* buffer, size_t bufferSize);
    bool loadSettings();
    bool saveSettings();


  #define TASK_QUEUE_SIZE 10


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
              int year, month, day, hour, minute, second;
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
          } playMultiToneArgs;
          struct{
              const char* wavfilename;
              int priority;
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
  Task createSaveCsvTask(float latitude, float longitude, float gs, int mtrack, int year, int month, int day, int hour, int minute, int second);
  Task createLoadMapImageTask(double center_lat, double center_lon, int zoomlevel);
  Task createPlayMultiToneTask(int freq, int duration, int count);
  Task createPlayWavTask(const char* logText,int priority=1);
  Task createBrowseSDTask(int page);

  // Functions to handle the queue (declarations)
  void enqueueTask(Task task);
  void enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
  bool isTaskRunning(int taskType);
  bool isTaskInQueue(int taskType);
  /*
  void log_sd(const char* text);
  void log_sdf(const char* format, ...);
  void saveCSV(float latitude, float longitude,float gs,int mtrack, int year, int month, int day, int hour, int minute, int second) ;
  void load_mapimage(double center_lat, double center_lon,int zoomlevel);
  */
  void load_push_logo();



  double pixelsPerDegreeLat(int zoom,double latitude);

  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern TFT_eSprite gmap_sprite;
  extern volatile bool gmap_loaded_active;
  extern volatile bool new_gmap_ready;
  extern volatile unsigned long loop1counter;
  extern volatile int restartcount;
#endif