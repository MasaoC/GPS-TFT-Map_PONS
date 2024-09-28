
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
      TASK_LOAD_MAPIMAGE
  } TaskType;

  #define TASK_QUEUE_SIZE 10


  typedef struct {
      TaskType type;
      union {
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
      };
  } Task;


  typedef struct {
      Task tasks[TASK_QUEUE_SIZE];
      volatile int head;
      volatile int tail;
  } TaskQueue;


  Task createLogSdTask(const char* logText);
  Task createLogSdfTask(const char* format, ...);
  Task createSaveCsvTask(float latitude, float longitude, float gs, int mtrack, int year, int month, int day, int hour, int minute, int second);
  Task createLoadMapImageTask(double center_lat, double center_lon, int zoomlevel);

  // Functions to handle the queue (declarations)
  void enqueueTask(Task task);
  void enqueueTaskWithAbortCheck(Task task);
  bool dequeueTask(Task *task);

  bool good_sd();
  bool isTaskRunning(int taskType);

  /*
  void log_sd(const char* text);
  void log_sdf(const char* format, ...);
  void saveCSV(float latitude, float longitude,float gs,int mtrack, int year, int month, int day, int hour, int minute, int second) ;
  void load_mapimage(double center_lat, double center_lon,int zoomlevel);
  */


  extern Task currentTask;
  extern mutex_t taskQueueMutex;
  extern TFT_eSprite gmap_sprite;
  extern volatile bool gmap_loaded;
  extern volatile bool new_gmap_loaded;
  extern volatile unsigned long loop1counter;
  extern volatile int loop1pos;
  extern volatile int loop0pos;
  extern volatile int restartcount;
#endif