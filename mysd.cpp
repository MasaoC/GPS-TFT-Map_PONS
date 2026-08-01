// ============================================================
// File    : mysd.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : SDカード操作の実装（全処理はCore1で実行）。
//           SdFatライブラリによるファイル読み書き、
//           設定ファイル保存/読込、CSVフライトログ追記、
//           飛行CSVのリプレイ再生（列名索引パーサ・先読みリングバッファ）、
//           Googleマップ画像(BMP)のロード、
//           Core1タスクキューのエンキュー/デキュー管理。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/07/31
// ============================================================
// SD card read and write programs.
// All process regarding SD card access are done in Core1.(#2 core)

#include <SPI.h>

#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "imu.h"
//#define DISABLE_FS_H_WARNING
#include "SdFat.h"
#include "sound.h"

#define MAX_SETTING_LENGTH 32
#define MAX_LINE_LENGTH (MAX_SETTING_LENGTH * 2 + 2) // ID:value ペア + 区切り文字・改行のバッファサイズ


SdFs SD;                     // SdFat ライブラリのファイルシステムオブジェクト
volatile bool sdInitialized = false;  // SD カードの初期化が完了しているか（Core0/Core1 両方から参照）
volatile bool sdError = false;        // SD アクセス中にエラーが発生したか（Core0/Core1 両方から参照）
volatile bool sd_setup_complete = false;  // Core1 の setup_sd() が完了したことを示すフラグ（Core0 が待機に使用）
volatile bool logo_ready = false;  // Core1 でロゴ BMP 読み込みが完了したら true（Core0 が pushSprite に使用）
#define LOGFILE_NAME "log.txt"

// ---- 処理時間計測変数（BNO085配置コア決定用、デバッグビルドのみ） ----
#ifndef RELEASE
TimingStat ts_load_mapimage = TSTAT_INIT("C1_load_mapimage");
TimingStat ts_savecsv_flush = TSTAT_INIT("C1_savecsv_flush");
extern volatile bool c0_is_redrawing;  // Core0 描画中フラグ（重複検出用）
uint32_t _c1_overlap_count = 0;        // Core0描画中にCore1重処理が重なった回数
#endif

unsigned long lasttrytime_sd = 0; // SD エラー時の最後の再試行時刻（10秒ごとにリトライする）
bool headerWritten = false;       // CSV ファイルにヘッダ行を書いたか（1フライト1回だけ書く）

// CSV ファイル名の生成に使う起動時の日時（初回書き込み時に確定する）
int fileyear = 0;//意図的に0で初期化して、最初のGPS時刻取得時に確定させる。変えるな。
int filemonth;
int fileday;
int filehour;
int fileminute;
int filesecond;


// ============================================================
// Core0↔Core1 間タスクキュー（ミューテックス保護）
//
// Core0（表示・GPS・ボタン）から Core1（SD・音声）へ
// 非同期で処理を依頼するためのリングバッファ。
//   - enqueueTask(): Core0 がタスクを積む
//   - dequeueTask(): Core1 がタスクを取り出して実行
//   - taskQueueMutex でアトミック操作を保証
// ============================================================
volatile bool abortTask = false;   // 実行中タスクに中断を要求するフラグ（ズームレベル変更時など）
TaskQueue taskQueue;               // タスクのリングバッファ本体
mutex_t taskQueueMutex;            // タスクキューへのアクセスを保護するミューテックス
Task currentTask;                  // Core1 が現在実行中のタスク（isTaskRunning() で参照）


void load_mapimage(double center_lat, double center_lon,int zoomlevel);
void log_sd(const char* text);
void log_sdf(const char* format, ...);
void dateTime(uint16_t* date, uint16_t* time);



// ============================================================
// SD カード設定ファイル（settings.txt）の管理
//
// ファイル形式: テキスト、1 行に 1 項目、"id:value\n" の形式
//   例:
//     volume:50
//     destination:PLATHOME
//     navigation_mode:INTO
//     scaleindex:2
//
// SDSetting 構造体が id（識別子文字列）と setter/getter を持ち、
// loadSettings() / saveSettings() がこのテーブルを走査して一括処理する。
// 新しい設定項目を追加するときはこの配列に 1 行追加するだけでよい。
// ============================================================
SDSetting settings[] = {
  {"volume",          setVolume,         getVolume},
  {"vario_volume",    setVarioVolume,    getVarioVolume},
  {"vario_inhibit",   setVarioInhibit,   getVarioInhibit},
  {"destination",     setDestination,    getDestination},
  {"navigation_mode", setNavigationMode, getNavigationMode},
  {"scaleindex",      setScaleIndex,     getScaleIndex},
  {"upward_mode",     setUpwardMode,     getUpwardMode},
  {"kf_q_vel",        setKfQVel,         getKfQVel},
  {"kf_q_bias",       setKfQBias,        getKfQBias},
  {"kf_R",            setKfR,            getKfR}
};
const int numSettings = sizeof(settings) / sizeof(settings[0]);
extern volatile int sound_volume;
extern volatile int vario_volume;
extern volatile bool vario_inhibit;
extern int destination_mode;
extern int scaleindex;
extern double scalelist[6];
extern double scale;
extern int upward_mode;  // display_tft.cpp で定義。0=TRACKUP, 1=NORTHUP


// 全設定を settings.txt に保存する。
// 一度ファイルを削除してから新規書き込みすることで、
// 古い項目が残らないようにしている。
bool saveSettings() {
  SD.remove("settings.txt"); // 古いファイルを消してから書き直す
  FsFile file = SD.open("settings.txt", FILE_WRITE);
  if (!file) {
    DEBUGW_PLN(20250401,"Failed to open settings.txt for writing");
    return false;
  }

  char value[MAX_SETTING_LENGTH];
  for (int i = 0; i < numSettings; i++) {
    settings[i].getter(value, MAX_SETTING_LENGTH);
    file.print(settings[i].id);
    file.print(":");
    file.print(value);
    file.print("\n");//
  }

  file.close();
  DEBUG_PLN(20250401,"Settings saved to settings.txt");
  return true;
}


// settings.txt から設定を読み込み、各 setter を呼んで反映する。
// ファイルを 1 文字ずつ読んで行を組み立て、"id:value" 形式でパースする。
// ファイル末尾に改行がない場合も処理できるよう、ループ後に残余バッファをチェックする。
bool loadSettings() {
  FsFile file = SD.open("settings.txt", FILE_READ);
  if (!file) {
    DEBUGW_PLN(20250401,"Failed to open settings.txt for reading");
    return false;
  }

  char line[MAX_LINE_LENGTH];
  char settingId[MAX_SETTING_LENGTH];
  char value[MAX_SETTING_LENGTH];
  int linePos = 0;

  while (file.available()) {
    char c = file.read();
    if (c == '\n' || linePos >= MAX_LINE_LENGTH - 1) {
      line[linePos] = '\0'; // Null-terminate the line
      linePos = 0;

      // Parse line into settingId and value
      char* colon = strchr(line, ':');
      if (colon) {
        *colon = '\0'; // Split at colon
        strncpy(settingId, line, MAX_SETTING_LENGTH);
        settingId[MAX_SETTING_LENGTH - 1] = '\0';
        strncpy(value, colon + 1, MAX_SETTING_LENGTH);
        value[MAX_SETTING_LENGTH - 1] = '\0';

        // Find and apply the setting
        for (int i = 0; i < numSettings; i++) {
          if (strcmp(settings[i].id, settingId) == 0) {
            settings[i].setter(value);
            break;
          }
        }
      }
    } else {
      line[linePos++] = c;
    }
  }



  // Handle last line if not newline-terminated
  if (linePos > 0) {
    line[linePos] = '\0';
    char* colon = strchr(line, ':');
    if (colon) {
      *colon = '\0';
      strncpy(settingId, line, MAX_SETTING_LENGTH);
      settingId[MAX_SETTING_LENGTH - 1] = '\0';
      strncpy(value, colon + 1, MAX_SETTING_LENGTH);
      value[MAX_SETTING_LENGTH - 1] = '\0';

      for (int i = 0; i < numSettings; i++) {
        if (strcmp(settings[i].id, settingId) == 0) {
          settings[i].setter(value);
          break;
        }
      }
    }
  }

  file.close();
  DEBUG_PLN(20250508,"Settings loaded from settings.txt");
  return true;
}



// ============================================================
// 設定項目ごとの setter / getter の実装
// setter: 文字列を受け取り、該当するグローバル変数に変換・格納する。
// getter: グローバル変数を文字列に変換して buffer に書き込む。
// ============================================================

// 音量: 文字列を int に変換して sound_volume に反映
void setVolume(const char* value) {
  sound_volume = atoi(value);
  DEBUG_P(20250508,"Set volume to: ");
  DEBUG_PLN(20250508,sound_volume);
}

void getVolume(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%d", sound_volume);
}

// バリオメーター音量: 文字列を int に変換して vario_volume に反映
void setVarioVolume(const char* value) {
  vario_volume = atoi(value);
}

void getVarioVolume(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%d", vario_volume);
}

// バリオ抑止フラグ: "true" なら vario を完全無効化する（SDカード手動設定のみ）
void setVarioInhibit(const char* value) {
  vario_inhibit = (strcmp(value, "true") == 0);
}
void getVarioInhibit(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%s", vario_inhibit ? "true" : "false");
}

// ナビゲーションモード: "INTO" / "AWAY" / "AUTO10K" を destination_mode 定数に変換
void setNavigationMode(const char *value){
  if(strcmp(value,"INTO") == 0){
    destination_mode = DMODE_FLYINTO;
  }
  else if(strcmp(value,"AWAY") == 0){
    destination_mode = DMODE_FLYAWAY;
  }
  else if(strcmp(value,"AUTO10K") == 0){
    destination_mode = DMODE_AUTO10K;
  }else{
    DEBUGW_PLN(20250509,"ERR MODE");
  }
}
void getNavigationMode(char* buffer, size_t bufferSize) {
  if(bufferSize <= 0)
    return;
  if(destination_mode == DMODE_FLYINTO)
    strncpy(buffer,"INTO", bufferSize);
  else if(destination_mode == DMODE_FLYAWAY)
    strncpy(buffer,"AWAY", bufferSize);
  else if(destination_mode == DMODE_AUTO10K)
    strncpy(buffer,"AUTO10K", bufferSize);
  buffer[bufferSize - 1] = '\0';
}

// 目的地: 名前で extradestinations を線形探索して currentdestination インデックスを設定する。
// 見つからない場合はエラーログを残す。
void setDestination(const char* value) {
  for(int i = 0; i < destinations_count; i++){
    if(strcmp(extradestinations[i].name,value) == 0){
      currentdestination = i;
      return;
    }
  }
  DEBUGW_P(20250508,"ERR DEST:");
  DEBUGW_PLN(20250508,value);
  enqueueTask(createLogSdfTask("ERR setDestination(%s)",value));
}


void getScaleIndex(char* buffer, size_t bufferSize) {
  if(bufferSize <= 0)
    return;
  snprintf(buffer, bufferSize, "%d", scaleindex);
}

void setScaleIndex(const char* value) {
  int indexofsetting = atoi(value);
  if(indexofsetting >= 0 && indexofsetting < (sizeof(scalelist) / sizeof(scalelist[0]))){
    scaleindex = indexofsetting;
    scale = scalelist[scaleindex];
  }else{
    DEBUGW_P(20250508,"ERR scale index:");
    DEBUGW_PLN(20250508,indexofsetting);
    enqueueTask(createLogSdfTask("ERR scale index(%s)",value));
  }
}

// 地図方向モード（NORTHUP=1 / TRACKUP=0）の getter / setter
void getUpwardMode(char* buffer, size_t bufferSize) {
  if (bufferSize <= 0) return;
  snprintf(buffer, bufferSize, "%d", upward_mode);
}

void setUpwardMode(const char* value) {
  int mode = atoi(value);
  if (mode >= 0 && mode < 2) {  // 0=TRACKUP, 1=NORTHUP
    upward_mode = mode;
  }
}

// Kalman フィルター係数: 速度プロセスノイズ（デフォルト KF_Q_VEL）
void setKfQVel(const char* value) {
  float v = atof(value);
  if (v > 0.0f) imu_set_kf_params(v, get_imu_kf_q_bias(), get_imu_kf_R());
}
void getKfQVel(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%.6g", get_imu_kf_q_vel());
}

// Kalman フィルター係数: バイアスプロセスノイズ（デフォルト KF_Q_BIAS）
void setKfQBias(const char* value) {
  float v = atof(value);
  if (v > 0.0f) imu_set_kf_params(get_imu_kf_q_vel(), v, get_imu_kf_R());
}
void getKfQBias(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%.6g", get_imu_kf_q_bias());
}

// Kalman フィルター係数: 気圧高度観測ノイズ [m²]（デフォルト KF_R）
void setKfR(const char* value) {
  float v = atof(value);
  if (v > 0.0f) imu_set_kf_params(get_imu_kf_q_vel(), get_imu_kf_q_bias(), v);
}
void getKfR(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%.6g", get_imu_kf_R());
}

void getDestination(char* buffer, size_t bufferSize) {
  if(bufferSize <= 0)
    return;
  strncpy(buffer, extradestinations[currentdestination].name, bufferSize);
  buffer[bufferSize - 1] = '\0';
}


// 現在 Core1 が実行中のタスクが指定タイプかを確認する
bool isTaskRunning(int taskType) {
  return currentTask.type == taskType;
}

// キュー内に指定タイプのタスクが存在するかを確認する（head〜tail を線形探索）
bool isTaskInQueue(int taskType){
    int current = taskQueue.head;
    while (current != taskQueue.tail) {
        if (taskQueue.tasks[current].type == taskType) {
            return true;
        }
        current = (current + 1) % TASK_QUEUE_SIZE;
    }
    return false;
}

// ============================================================
// タスク生成ファクトリ関数（create〜Task）
//
// 各タスクは Task 構造体として表現され、
// type フィールドで種別を識別し、union のフィールドで引数を渡す。
// Core0 がこれらを呼んで enqueueTask() に渡し、Core1 がデキューして実行する。
// ============================================================

// 設定ファイル保存タスクを生成する
Task createSaveSettingTask(){
  Task task;
  task.type = TASK_SAVE_SETTINGS;
  return task;
}


// テキストをログファイルに書き込むタスクを生成する
Task createLogSdTask(const char* logText) {
  Task task;
  task.type = TASK_LOG_SD;
  task.logText = logText;
  return task;
}

// printf 形式のフォーマット文字列でログタスクを生成する（可変引数対応）
Task createLogSdfTask(const char* format, ...) {
  Task task;
  task.type = TASK_LOG_SDF;
  va_list args;
  va_start(args, format);
  vsnprintf(task.logSdfArgs.buffer, sizeof(task.logSdfArgs.buffer), format, args);
  va_end(args);
  task.logSdfArgs.format = "%s";
  return task;
}


// 1 フレーム分の GPS データを CSV ログに書き込むタスクを生成する
Task createSaveCsvTask(float latitude, float longitude, float gs, int ttrack, float altitude, float kf_altitude, float kf_vspeed, float pressure, int year, int month, int day, int hour, int minute, int second, int centisecond) {
  Task task;
  task.type = TASK_SAVE_CSV;
  task.saveCsvArgs.latitude = latitude;
  task.saveCsvArgs.longitude = longitude;
  task.saveCsvArgs.gs = gs;
  task.saveCsvArgs.ttrack = ttrack;
  task.saveCsvArgs.altitude = altitude;
  task.saveCsvArgs.kf_altitude = kf_altitude;
  task.saveCsvArgs.kf_vspeed  = kf_vspeed;
  task.saveCsvArgs.pressure = pressure;
  task.saveCsvArgs.year = year;
  task.saveCsvArgs.month = month;
  task.saveCsvArgs.day = day;
  task.saveCsvArgs.hour = hour;
  task.saveCsvArgs.minute = minute;
  task.saveCsvArgs.second = second;
  task.saveCsvArgs.centisecond = centisecond;
  return task;
}

// Google マップ BMP 画像の読み込みタスクを生成する（中心座標とズームレベルを指定）
Task createLoadMapImageTask(double center_lat, double center_lon, int zoomlevel) {
  Task task;
  task.type = TASK_LOAD_MAPIMAGE;
  task.loadMapImageArgs.center_lat = center_lat;
  task.loadMapImageArgs.center_lon = center_lon;
  task.loadMapImageArgs.zoomlevel = zoomlevel;
  return task;
}

Task createPlayMultiToneTask(int freq, int duration, int count,int priority,int min_volume,bool solo_play){
  Task task;
  task.type = TASK_PLAY_MULTITONE;
  task.playMultiToneArgs.freq = freq;
  task.playMultiToneArgs.duration = duration;
  task.playMultiToneArgs.counter = count;
  task.playMultiToneArgs.priority = priority;
  task.playMultiToneArgs.min_volume = min_volume;
  task.playMultiToneArgs.solo_play = solo_play;
  return task;
}

Task createPlayWavTask(const char* filename, int priority, int min_volume){
  Task task;
  task.type = TASK_PLAY_WAV;
  task.playWavArgs.wavfilename = filename;
  task.playWavArgs.priority = priority;
  task.playWavArgs.min_volume = min_volume;
  return task;
}


Task createBrowseSDTask(int page){
  Task task;
  task.type = TASK_BROWSE_SD;
  task.pagenum = page;
  return task;
}


// リプレイ選択画面のファイル一覧取得タスク。
// browse_sd() と違い「ページ番号」ではなく「対象CSVの通し番号の開始位置」を渡す
// （リストの先頭に固定項目が並ぶため、ページ境界と一覧のインデックスがずれる）。
Task createBrowseReplayTask(int start_index){
  Task task;
  task.type = TASK_BROWSE_REPLAY;
  task.pagenum = start_index;
  return task;
}


Task createLoadReplayTask(){
  Task task;
  task.type = TASK_LOAD_REPLAY;
  return task;
}

Task createInitReplayTask(){
  Task task;
  task.type = TASK_INIT_REPLAY;
  return task;
}


// キュー内の指定タイプのタスクを全て除去する（重複防止用）。
// in-place 削除: read ポインタで走査し、対象でない要素だけ write 位置に詰める。
void removeDuplicateTask(TaskType type) {
    mutex_enter_blocking(&taskQueueMutex);

    if (taskQueue.head == taskQueue.tail) {
        mutex_exit(&taskQueueMutex);
        return;  // キューが空
    }

    int read = taskQueue.head;
    int write = taskQueue.head;

    while (read != taskQueue.tail) {
        if (taskQueue.tasks[read].type != type) {
            if (write != read) {
                taskQueue.tasks[write] = taskQueue.tasks[read];
            }
            write = (write + 1) % TASK_QUEUE_SIZE;
        }
        read = (read + 1) % TASK_QUEUE_SIZE;
    }

    taskQueue.tail = write;  // 新しい末尾位置（削除後）
    mutex_exit(&taskQueueMutex);
}

// 中断チェック付きでタスクをキューに追加する。
// 地図画像ロード (TASK_LOAD_MAPIMAGE) に特化した特殊処理を行う:
//   - 同じズームレベルの地図を既にロード中なら、重複追加しない（待つ）
//   - ズームレベルが変わった場合は abortTask フラグを立てて現在のロードを中断し、
//     キュー内の重複を削除して新しいタスクを追加する
void enqueueTaskWithAbortCheck(Task newTask) {
  if (isTaskRunning(newTask.type) && newTask.type == TASK_LOAD_MAPIMAGE) {
    if(newTask.loadMapImageArgs.zoomlevel == currentTask.loadMapImageArgs.zoomlevel){
      // 同じズームの地図を既にロード中 → 重複追加しない
      DEBUGW_P(20250429,"NOT enque the task:");
      DEBUGW_PLN(20250429,newTask.type);
      return;
    }
    // ズームレベルが変わった → 実行中のロードを中断して新しいタスクを優先する
    abortTask = true;
  }
  removeDuplicateTask(newTask.type);  // キュー内の同種タスクを削除してから追加
  enqueueTask(newTask);
}

extern volatile bool userled_forced_on;  // GPS_TFT_map.ino で定義

// タスクをリングバッファに追加する（ミューテックス保護）。
// バッファが満杯の場合は追加せずにスキップする（タスクロスト）。
// タスクロスト時は USERLED_PIN を永続点灯してエラーを通知する。
void enqueueTask(Task task) {
  mutex_enter_blocking(&taskQueueMutex);
  int nextTail = (taskQueue.tail + 1) % TASK_QUEUE_SIZE;
  if (nextTail != taskQueue.head) {  // キューに空きがある場合だけ追加
    taskQueue.tasks[taskQueue.tail] = task;
    taskQueue.tail = nextTail;
  } else {
    userled_forced_on = true;         // 永続点灯フラグを立てる（loop_userled がフラッシュで消さないよう保護）
    digitalWrite(USERLED_PIN, HIGH);  // キュー満杯によるタスクロスト → LED 永続点灯
    // ※ mutex 保持中のため enqueueTask() の再帰呼び出し不可。Serial 出力のみ。
    Serial.println("ERR: task queue full, task dropped");
  }
  mutex_exit(&taskQueueMutex);
}

// タスクをリングバッファから取り出す（ミューテックス保護）。
// 取り出せた場合は *task に格納して true を返す、空なら false を返す。
bool dequeueTask(Task* task) {
  bool success = false;
  mutex_enter_blocking(&taskQueueMutex);
  if (taskQueue.head != taskQueue.tail) {  // キューが空でない
    *task = taskQueue.tasks[taskQueue.head];
    taskQueue.head = (taskQueue.head + 1) % TASK_QUEUE_SIZE;
    success = true;
  }
  mutex_exit(&taskQueueMutex);
  return success;
}

// mapdata.csv の 1 行を解析して extramaps[] に追加する。
// CSV 形式: "ポリゴン名,頂点数,lon1,lat1,lon2,lat2,..."
void process_mapcsv_line(String line) {
  int index = 0;
  int commaIndex = line.indexOf(',');
  String name = line.substring(index, commaIndex);
  index = commaIndex + 1;

  commaIndex = line.indexOf(',', index);
  int size = line.substring(index, commaIndex).toInt();
  index = commaIndex + 1;

  // Allocate memory for the coordinates
  double (*cords)[2] = new double[size][2];

  for (int i = 0; i < size; i++) {
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      delete[] cords;
      return;
    }
    cords[i][0] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      delete[] cords;
      return;
    }
    cords[i][1] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
  }

  extramaps[mapdata_count].id = current_id++;
  extramaps[mapdata_count].name = strdup(name.c_str()); // Duplicate string to allocate memory
  extramaps[mapdata_count].size = size;
  extramaps[mapdata_count].cords = cords;
  mapdata_count++;
}


// destinations.csv の 1 行を解析して extradestinations[] に追加する。
// CSV 形式: "目的地名,lon,lat"（頂点は 1 点だけ）
void process_destinationcsv_line(String line) {
  int index = 0;
  int commaIndex = line.indexOf(',');
  String name = line.substring(index, commaIndex);
  index = commaIndex + 1;

  int size = 1;
  // Allocate memory for the coordinates
  double (*cords)[2] = new double[size][2];

  for (int i = 0; i < size; i++) {
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      delete[] cords;
      return;
    }
    cords[i][0] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      delete[] cords;
      return;
    }
    cords[i][1] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
  }

  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup(name.c_str()); // Duplicate string to allocate memory
  extradestinations[destinations_count].size = size;
  extradestinations[destinations_count].cords = cords;
  destinations_count++;
}

// SD カードの mapdata.csv から地図ポリゴンを読み込み extramaps[] に格納する。
// メモリリーク防止のため 1 回だけ実行する（mapdatainitialized フラグで管理）。
bool mapdatainitialized = false;
void init_mapdata() {
  if(mapdatainitialized){
    DEBUG_PLN(20241006,"Map already initialized.");
    return; // 2 回目以降はスキップ（重複ロードによるメモリリークを防ぐ）
  }
  FsFile myFile = SD.open("mapdata.csv");
  if (!myFile) {
    return;
  }
  while (myFile.available() && mapdata_count < MAX_MAPDATAS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      process_mapcsv_line(line);
    }
  }
  myFile.close();
  mapdatainitialized = true;
}


// SD カードの destinations.csv から目的地を読み込み extradestinations[] に追加する。
// init_destinations() で固定目的地を登録した後に呼ぶことで、SD 側の追加目的地を上書きせず後ろに追加できる。
void load_destinations(){
  FsFile myFile = SD.open("destinations.csv");
  if (!myFile) {
    return;
  }
  while (myFile.available() && destinations_count < MAX_DESTINATIONS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      process_destinationcsv_line(line);
    }
  }
  myFile.close();
}

// SD カードを初期化し、地図・目的地・設定を読み込む。
// trycount: 1回の呼び出しで SDIO を試みる最大回数。失敗時は 100ms 待って再試行する。
// 成功時: FAT タイムスタンプコールバック登録 → ログ記録 → init_mapdata / load_destinations / loadSettings を実行。
// 失敗時: sdInitialized=false のまま処理を戻す（Core1 タスクはエラーをユーザーに報告する）。
// SDIO / SPI フォールバック状態
// setup_sd() 自体が SDIO 失敗を 5 回繰り返した場合に SPI に切り替える（呼び出し回数カウント方式）。
// SDIO が復活した場合は自動的に SDIO に戻る。
static bool sd_use_spi = false;
static int sdio_fail_count = 0;  // setup_sd() が SDIO 失敗した累計呼び出し回数
static int sd_setup_count = 0;   // setup_sd() の累計呼び出し回数

bool get_sd_use_spi()    { return sd_use_spi; }
int  get_sd_setup_count(){ return sd_setup_count; }

void setup_sd(int trycount, bool load_settings){
  sd_setup_count++;
  sdInitialized = false;

  // --- SDIO 試行（SPI モード中はスキップ）---
  if (!sd_use_spi) {
    for (int i = 0; i < trycount; i++) {  // trycount 回だけ試行
      sdInitialized = SD.begin(SdioConfig(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO));
      if (sdInitialized) break;
      delay(100);
    }
  }

  if (sdInitialized) {
    sdio_fail_count = 0;  // SDIO 成功 → カウンターリセット
    sd_use_spi = false;
  } else {
    sdio_fail_count++;    // SDIO 失敗 → 呼び出し回数をカウント
    DEBUG_PLN(20260309, "SDIO failed.");

    // --- setup_sd が 5 回連続 SDIO 失敗したら SPI にフォールバック ---
    if (sdio_fail_count >= 5) {
      DEBUG_PLN(20260309, "5 SDIO failures. Trying SPI fallback...");
      SPI.setRX(RP_DAT0_GPIO);   // GPIO4 = DAT0 → MISO
      SPI.setTX(RP_CMD_GPIO);    // GPIO3 = CMD  → MOSI
      SPI.setSCK(RP_CLK_GPIO);   // GPIO2 = CLK  → SCK
      SPI.begin();
      sdInitialized = SD.begin(SdSpiConfig(SD_CS_SPI_PIN, DEDICATED_SPI, SD_SCK_MHZ(16)));
      if (sdInitialized) {
        sd_use_spi = true;
        sdio_fail_count = 0;  // SPI 成功 → カウンターリセット
        DEBUG_PLN(20260309, "SD SPI fallback OK.");
      }
    }
  }

  if (!sdInitialized) {
    DEBUG_PLN(20260208, "SD init failed.");
    sd_setup_complete = true;  // 失敗でも完了を通知（Core0 が無限待機しないように）
    return;
  }

  sdError = false;
  SdFile::dateTimeCallback(dateTime);
  log_sd(sd_use_spi ? "SD INIT SPI" : "SD INIT");
  init_mapdata();
  load_destinations();
  if (load_settings) {
    if (!loadSettings()) {
      DEBUGW_PLN(20250508, "Error loading settings.");
    }
  }
  sd_setup_complete = true;  // 全 SD 初期化処理完了を Core0 に通知
}

// ============================================================
// リプレイ再生（PONS の飛行 CSV を直接再生する）
//
// 旧実装は Python で NMEA に変換した replay.csv 専用だったが、v6 では
// SD に保存されている飛行 CSV（YYYY-MM-DD_HHMM.csv, 2Hz）をそのまま再生する。
// CSV のヘッダを「列名」で解釈するため、v5（6列）～v6（10列）まで無加工で扱える。
//
// 役割分担:
//   Core1 (このファイル) … ファイルを開きっぱなしにして行を読み、パースしてリングに積むだけ
//   Core0 (gps.cpp)      … リングから「再生時刻に達した行」を取り出して stored_* に反映する
// 単一 producer / 単一 consumer のリングなので mutex は不要。
// （スロットへ書き込んでから head を進める順序を必ず守ること）
// ============================================================

// ===== リプレイ再生用変数 =====
volatile ReplayRow replay_rows[REPLAY_BUF_SIZE];  // 先読みした CSV 行のリングバッファ
volatile uint8_t replay_head = 0;                 // Core1 が書き込む位置
volatile uint8_t replay_tail = 0;                 // Core0 が読み出す位置
volatile bool replay_eof = false;                 // ファイル末尾に到達した
volatile unsigned long replay_start_time = 0;     // 再生開始時刻（millis()）。t_ms との比較基準
char replay_filename[REPLAY_FILENAME_LEN] = "";   // 再生対象ファイル（"" = 未選択）
static bool replay_flight_only = true;            // PLAY FLIGHT ONLY（静止区間をスキップ）。既定 YES

// 飛行 CSV のファイルハンドル。SD.open()/close() を毎回呼ぶのではなく静的 FsFile を
// 使い回すことで open/close 時の SDIO ハングリスクを最小化する（実処理は saveCSV()）。
// init_replay() からも flush するため、ここで定義している。
static FsFile csvFileStatic;  // セッション中開きっぱなし。SDエラー時のみ close。

static FsFile replayFileStatic;      // セッション中は開きっぱなし（SDIOハングリスク低減のため）
static int8_t replay_col[REPLAY_COL_COUNT];  // 列名 → CSV上の列インデックス（-1 = その列は無い）
static uint32_t replay_csv_t0_ms = 0;        // CSV 先頭データ行の時刻（0時からのms）
static uint32_t replay_prev_ms = 0;          // 直前行の時刻（日跨ぎ検出用）
static uint32_t replay_day_offset_ms = 0;    // 日跨ぎ補正の累積 [ms]
static bool replay_t0_valid = false;         // 先頭行の時刻を確定済みか

// --- PLAY FLIGHT ONLY 用のタイムライン圧縮 ---
// 静止行を捨てるだけでは、残った行が元の時刻を保持しているため再生が静止区間で
// 待ち続けてしまう。スキップした区間の長さを累積し、以降の行の再生時刻から
// 差し引くことで、静止区間を詰めて再生する。
static uint32_t replay_skipped_ms = 0;         // スキップした区間の累計 [ms]
static uint32_t replay_leadin_until_abs = 0;   // この絶対時刻までは静止していても再生する（助走区間）

// 助走区間の開始位置を探すためのリングバッファ（静止行のファイル位置と時刻）
static uint32_t replay_leadin_pos[REPLAY_LEADIN_SLOTS];
static uint32_t replay_leadin_abs[REPLAY_LEADIN_SLOTS];

// CSV ヘッダで探す列名。並びは ReplayCol の enum と一致させること。
static const char* const replay_col_names[REPLAY_COL_COUNT] = {
  "latitude", "longitude", "gs", "truetrack", "altitude",
  "kf_altitude", "kf_vspeed", "pressure", "date", "time",
  "numsat", "voltage"
};

// 大文字小文字を無視した文字列比較（strcasecmp 相当）。前後の空白も無視する。
static bool replay_name_match(const char* token, const char* name) {
  while (*token == ' ' || *token == '\t') token++;
  while (*token && *name) {
    char a = *token, b = *name;
    if (a >= 'A' && a <= 'Z') a += 32;
    if (b >= 'A' && b <= 'Z') b += 32;
    if (a != b) return false;
    token++; name++;
  }
  // token 側の末尾に空白・CR・LF が残っていても一致とみなす
  // （ヘッダ行の最終列は fgets が付けた改行を含むため、これを無視しないと
  //   "time" が最終列にある CSV で列を見つけられなくなる）
  while (*token == ' ' || *token == '\t' || *token == '\r' || *token == '\n') token++;
  return (*token == '\0' && *name == '\0');
}

// CSV 1行から n 番目（0始まり）のフィールドを取り出して out にコピーする。
// 戻り値: 取り出せたら true。フィールドが存在しない場合は false。
static bool replay_get_field(const char* line, int index, char* out, size_t outsize) {
  if (index < 0) return false;
  int field = 0;
  const char* p = line;
  while (field < index) {
    p = strchr(p, ',');
    if (p == nullptr) return false;
    p++;
    field++;
  }
  const char* end = strchr(p, ',');
  size_t len = (end == nullptr) ? strlen(p) : (size_t)(end - p);
  if (len >= outsize) len = outsize - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  // 末尾の CR / 空白を除去
  while (len > 0 && (out[len-1] == '\r' || out[len-1] == '\n' || out[len-1] == ' ')) {
    out[--len] = '\0';
  }
  return (len > 0);
}

// 指定した列の値を float として取り出す。列が無い / 空なら false を返し val は変更しない。
static bool replay_get_float(const char* line, ReplayCol col, float* val) {
  char buf[24];
  if (!replay_get_field(line, replay_col[col], buf, sizeof(buf))) return false;
  *val = atof(buf);
  return true;
}

// "HH:MM:SS" または "HH:MM:SS.cc" を 0時からのミリ秒に変換する。
// 戻り値: 変換できたら true。cs には centisecond（0-99）を返す。
static bool replay_parse_time(const char* s, uint32_t* ms, int* h, int* m, int* sec, int* cs) {
  int hh = 0, mm = 0, ss = 0, cc = 0;
  int n = sscanf(s, "%d:%d:%d.%d", &hh, &mm, &ss, &cc);
  if (n < 3) return false;                   // v5 は ".cc" が無いので n==3 で正常
  if (n < 4) cc = 0;
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 60) return false;
  *ms  = ((uint32_t)hh * 3600UL + (uint32_t)mm * 60UL + (uint32_t)ss) * 1000UL + (uint32_t)cc * 10UL;
  *h = hh; *m = mm; *sec = ss; *cs = cc;
  return true;
}

// "YYYY-MM-DD" を分解する。
static bool replay_parse_date(const char* s, int* y, int* mo, int* d) {
  return (sscanf(s, "%d-%d-%d", y, mo, d) == 3);
}

// リングバッファの空きスロット数を返す（Core1 側から使用）。
static uint8_t replay_free_slots() {
  uint8_t used = (uint8_t)((replay_head - replay_tail) & (REPLAY_BUF_SIZE - 1));
  return (REPLAY_BUF_SIZE - 1) - used;  // 1 スロットは満杯/空の区別用に常に空けておく
}

// Core0 から呼ぶ: 未消費の行があるか。
bool replay_available() {
  return replay_head != replay_tail;
}

// Core0 から呼ぶ: 次に再生すべき行の再生時刻 [ms]。行が無い場合の戻り値は不定。
uint32_t replay_peek_t_ms() {
  return replay_rows[replay_tail].t_ms;
}

// Core0 から呼ぶ: 先頭の行を取り出す。取り出せたら true。
bool replay_pop(ReplayRow* out) {
  if (replay_head == replay_tail) return false;
  // volatile 配列からのコピー（構造体代入は volatile 不可なのでメンバ単位でコピーする）
  volatile ReplayRow* src = &replay_rows[replay_tail];
  out->lat = src->lat;               out->lon = src->lon;
  out->gs = src->gs;                 out->ttrack = src->ttrack;
  out->altitude = src->altitude;     out->kf_altitude = src->kf_altitude;
  out->kf_vspeed = src->kf_vspeed;   out->pressure = src->pressure;
  out->voltage = src->voltage;       out->numsat = src->numsat;
  out->have = src->have;             out->t_ms = src->t_ms;
  out->year = src->year;             out->month = src->month;   out->day = src->day;
  out->hour = src->hour;             out->minute = src->minute; out->second = src->second;
  out->centisecond = src->centisecond;
  replay_tail = (replay_tail + 1) & (REPLAY_BUF_SIZE - 1);
  return true;
}

// Core0 から呼ぶ: バッファに空きがあるか（補充依頼を出すかの判断用）。
bool replay_buffer_has_space() {
  uint8_t used = (uint8_t)((replay_head - replay_tail) & (REPLAY_BUF_SIZE - 1));
  return used < (REPLAY_BUF_SIZE - 1);
}

// 選択中のファイル名を返す（設定画面のラベル表示用）。
const char* get_replay_filename() {
  return replay_filename;
}

// 再生対象ファイルを設定する。実際の読み込み開始は init_replay() で行う。
void set_replay_filename(const char* name) {
  strlcpy(replay_filename, name, sizeof(replay_filename));
}

// PLAY FLIGHT ONLY（静止区間をスキップして再生するか）の取得／設定。
bool get_replay_flight_only() { return replay_flight_only; }
void set_replay_flight_only(bool on) { replay_flight_only = on; }

// CSV のヘッダ行を解釈して replay_col[] を構築する。
// 列名は大文字小文字を無視して照合し、未知の列や空の列名（2026大会データに存在）は無視する。
static void replay_parse_header(const char* header) {
  for (int i = 0; i < REPLAY_COL_COUNT; i++) replay_col[i] = -1;

  int index = 0;
  const char* p = header;
  while (p != nullptr && *p != '\0') {
    const char* end = strchr(p, ',');
    char token[24];
    size_t len = (end == nullptr) ? strlen(p) : (size_t)(end - p);
    if (len >= sizeof(token)) len = sizeof(token) - 1;
    memcpy(token, p, len);
    token[len] = '\0';

    for (int c = 0; c < REPLAY_COL_COUNT; c++) {
      if (replay_col[c] == -1 && replay_name_match(token, replay_col_names[c])) {
        replay_col[c] = index;
        break;
      }
    }
    index++;
    p = (end == nullptr) ? nullptr : end + 1;
  }
}

// リプレイ再生をファイル先頭から開始（またはループ再生のため再開）する。
// ファイルを開き直し、ヘッダから列マップを構築し、リングバッファを空にする。
void init_replay(){
  if (replayFileStatic.isOpen()) replayFileStatic.close();

  replay_head = 0;
  replay_tail = 0;
  replay_eof = false;
  replay_t0_valid = false;
  replay_csv_t0_ms = 0;
  replay_prev_ms = 0;
  replay_day_offset_ms = 0;
  replay_skipped_ms = 0;
  replay_leadin_until_abs = 0;
  replay_start_time = millis();

  // 飛行 CSV は 2 秒に 1 回しか flush していないため、リプレイに入って saveCSV() が
  // 呼ばれなくなると、直前の実飛行データ最大 2 秒分が SdFat のバッファに残ったままになる。
  // リプレイ中に電源が切られると失われるので、ここで確実に書き出しておく。
  // （ファイルは閉じない。リプレイ終了後は同じファイルに追記を続ける）
  if (csvFileStatic.isOpen()) csvFileStatic.flush();

  if (replay_filename[0] == '\0') {
    DEBUGW_PLN(20260731, "REPLAY: no file selected");
    replay_eof = true;
    return;
  }
  if (!good_sd()) {
    replay_eof = true;
    return;
  }

  replayFileStatic = SD.open(replay_filename, FILE_READ);
  if (!replayFileStatic) {
    DEBUGW_P(20260731, "REPLAY: cannot open ");
    DEBUGW_PLN(20260731, replay_filename);
    log_sdf("ERR REPLAY open failed: %s", replay_filename);
    replay_eof = true;
    return;
  }

  // 1行目 = ヘッダ。列名から列マップを作る。
  char header[160];
  int n = replayFileStatic.fgets(header, sizeof(header));
  if (n <= 0) {
    replay_eof = true;
    return;
  }
  replay_parse_header(header);

  // 最低限 latitude / longitude / time が無ければ再生できない
  if (replay_col[RCOL_LAT] < 0 || replay_col[RCOL_LON] < 0 || replay_col[RCOL_TIME] < 0) {
    DEBUGW_PLN(20260731, "REPLAY: header missing lat/lon/time");
    log_sdf("ERR REPLAY bad header: %s", replay_filename);
    replayFileStatic.close();
    replay_eof = true;
    return;
  }
  log_sdf("REPLAY start: %s", replay_filename);
}


// 静止区間の先頭行に来たときに呼ぶ（PLAY FLIGHT ONLY 有効時のみ）。
// 区間がどこで終わる（＝機体が動き出す）かを先読みし、
//   ・静止が REPLAY_LEADIN_MS 未満  → 一切スキップせずそのまま再生する
//   ・静止が REPLAY_LEADIN_MS 以上  → 動き出しの REPLAY_LEADIN_MS 手前まで飛ばす
// いずれの場合もファイル位置を再生を再開すべき行まで seek で巻き戻して戻る。
// 動き出しの手前を「助走」として残すのは、いきなり動き出すと表示や警告音が
// 実際の飛行開始と食い違って見えるため。
//
// run_start_pos / run_start_abs: 静止区間の先頭行のファイル位置と絶対時刻
// 戻り値: false なら静止のままファイル末尾に到達した
static bool replay_handle_stationary_run(uint32_t run_start_pos, uint32_t run_start_abs) {
  // 走査中に進んでしまう日跨ぎ判定用の状態を保存しておき、seek で戻すときに復元する
  uint32_t saved_prev_ms   = replay_prev_ms;
  uint32_t saved_day_offset = replay_day_offset_ms;

  // 区間の先頭を最初の候補として登録する
  replay_leadin_pos[0] = run_start_pos;
  replay_leadin_abs[0] = run_start_abs;
  uint8_t head = 1, count = 1;

  char line[200], buf[24];
  uint32_t move_abs = 0;
  uint32_t last_abs = run_start_abs;
  bool found_move = false;
  int scanned = 0;

  while (scanned < REPLAY_SCAN_MAX) {
    uint32_t pos = (uint32_t)replayFileStatic.position();
    int n = replayFileStatic.fgets(line, sizeof(line));
    if (n <= 0) {
      // 静止したままファイル末尾。残りは再生する意味がないので終了扱いにする。
      return false;
    }
    scanned++;

    uint32_t tod; int hh, mm, ss, cc;
    if (!replay_get_field(line, replay_col[RCOL_TIME], buf, sizeof(buf))) continue;
    if (!replay_parse_time(buf, &tod, &hh, &mm, &ss, &cc)) continue;
    // 日跨ぎ補正（本体のループと同じ処理）
    if (tod + 60000UL < replay_prev_ms) replay_day_offset_ms += 86400000UL;
    replay_prev_ms = tod;
    uint32_t abs_ms = tod + replay_day_offset_ms;
    last_abs = abs_ms;

    float gs_val;
    if (replay_get_float(line, RCOL_GS, &gs_val) && gs_val > REPLAY_MIN_GS) {
      move_abs = abs_ms;      // ここで動き出した
      found_move = true;
      break;
    }

    // まだ静止 → 直近 REPLAY_LEADIN_SLOTS 行のファイル位置を保持する
    replay_leadin_pos[head] = pos;
    replay_leadin_abs[head] = abs_ms;
    head = (head + 1) % REPLAY_LEADIN_SLOTS;
    if (count < REPLAY_LEADIN_SLOTS) count++;
  }

  if (!found_move) {
    // 走査上限に達した（非常に長い静止区間）。ここまでを飛ばしたことにして
    // 現在位置からそのまま続行する（seek で戻さないので日跨ぎ判定の状態も進めたままにする）。
    // 次の行もまた静止なら改めて先読みが走るので、Core1 を長時間占有せずに少しずつ処理が進む。
    replay_skipped_ms += last_abs - run_start_abs;
    return true;
  }

  // ここから先は seek でファイル位置を巻き戻すので、走査中に進めた
  // 日跨ぎ判定用の状態も区間先頭の時点まで戻す（戻した行を読み直すため）
  replay_prev_ms       = saved_prev_ms;
  replay_day_offset_ms = saved_day_offset;

  // 再生を再開する位置を決める
  uint32_t resume_pos, resume_abs;
  if (move_abs - run_start_abs < REPLAY_LEADIN_MS) {
    // 短い静止 → スキップしない（区間の先頭から再生する）
    resume_pos = run_start_pos;
    resume_abs = run_start_abs;
  } else {
    // 動き出しの REPLAY_LEADIN_MS 手前以降で、最も古い行を探す
    uint32_t target = move_abs - REPLAY_LEADIN_MS;
    uint8_t oldest = (uint8_t)((head + REPLAY_LEADIN_SLOTS - count) % REPLAY_LEADIN_SLOTS);
    resume_pos = replay_leadin_pos[oldest];   // 保持している中で最も古い行（既定値）
    resume_abs = replay_leadin_abs[oldest];
    for (uint8_t i = 0; i < count; i++) {
      uint8_t k = (uint8_t)((oldest + i) % REPLAY_LEADIN_SLOTS);
      if (replay_leadin_abs[k] >= target) {
        resume_pos = replay_leadin_pos[k];
        resume_abs = replay_leadin_abs[k];
        break;
      }
    }
  }

  // 飛ばした分だけ再生タイムラインを詰める
  replay_skipped_ms += resume_abs - run_start_abs;
  // 動き出しまでは静止していても再生する（助走区間）
  replay_leadin_until_abs = move_abs;
  replayFileStatic.seek(resume_pos);
  return true;
}


// リングバッファの空きスロットを埋められるだけ埋める（Core1 で実行）。
// パースに失敗した行は読み飛ばして次へ進む（旧実装のような無限ループにはならない）。
void load_replay() {
  if (replay_eof) return;
  if (!replayFileStatic.isOpen()) return;

  char line[200];

  while (replay_free_slots() > 0) {
    uint32_t line_pos = (uint32_t)replayFileStatic.position();  // 静止区間の巻き戻しに使う
    int n = replayFileStatic.fgets(line, sizeof(line));
    if (n <= 0) {
      // ファイル末尾。Core0 側が残りを再生し終えたら init_replay() でループ再生する。
      replay_eof = true;
      return;
    }

    // --- 時刻（必須） ---
    char buf[24];
    if (!replay_get_field(line, replay_col[RCOL_TIME], buf, sizeof(buf))) continue;
    uint32_t tod_ms;
    int hh, mm, ss, cc;
    if (!replay_parse_time(buf, &tod_ms, &hh, &mm, &ss, &cc)) continue;

    // 日跨ぎ補正: 時刻が前の行より大きく戻ったら 1 日進んだとみなす
    if (replay_t0_valid && tod_ms + 60000UL < replay_prev_ms) {
      replay_day_offset_ms += 86400000UL;
    }
    replay_prev_ms = tod_ms;
    if (!replay_t0_valid) {
      replay_csv_t0_ms = tod_ms;
      replay_t0_valid = true;
    }
    uint32_t abs_ms = tod_ms + replay_day_offset_ms;
    if (abs_ms < replay_csv_t0_ms) continue;  // 念のための保険

    // --- 対地速度（PLAY FLIGHT ONLY の判定に使う） ---
    float gs_val;
    bool has_gs = replay_get_float(line, RCOL_GS, &gs_val);

    // PLAY FLIGHT ONLY: 静止（GS が閾値以下）の区間の先頭に来たら、
    // 区間の長さを先読みして「飛ばすか / そのまま再生するか」を決める。
    // 助走区間（動き出し直前の REPLAY_LEADIN_MS）の中にいる間は判定しない。
    // gs 列を持たない CSV は判定できないので全行再生する。
    if (replay_flight_only && has_gs && gs_val <= REPLAY_MIN_GS &&
        abs_ms >= replay_leadin_until_abs) {
      if (!replay_handle_stationary_run(line_pos, abs_ms)) {
        replay_eof = true;   // 静止のままファイル末尾に到達
        return;
      }
      continue;  // ファイル位置は seek で調整済み。次の行から読み直す。
    }

    // --- 緯度経度（必須） ---
    // 緯度経度は float では精度が足りないので atof() の double をそのまま使う
    double dlat, dlon;
    if (!replay_get_field(line, replay_col[RCOL_LAT], buf, sizeof(buf))) continue;
    dlat = atof(buf);
    if (!replay_get_field(line, replay_col[RCOL_LON], buf, sizeof(buf))) continue;
    dlon = atof(buf);
    if (dlat < -90.0 || dlat > 90.0 || dlon < -180.0 || dlon > 180.0) continue;

    // --- スロットに書き込む ---
    volatile ReplayRow* row = &replay_rows[replay_head];
    row->have = 0;
    row->lat = dlat;
    row->lon = dlon;
    // スキップした静止区間の長さを差し引いた再生時刻
    row->t_ms = (abs_ms - replay_csv_t0_ms) - replay_skipped_ms;
    row->hour = hh; row->minute = mm; row->second = ss; row->centisecond = cc;

    float v;
    if (has_gs)                                   { row->gs = gs_val; row->have |= RHAVE_GS; }
    else                                          { row->gs = 0; }
    if (replay_get_float(line, RCOL_TTRACK, &v)) { row->ttrack = v; row->have |= RHAVE_TTRACK; }
    else                                          { row->ttrack = 0; }
    if (replay_get_float(line, RCOL_ALT, &v))    { row->altitude = v;    row->have |= RHAVE_ALT; }
    else                                          { row->altitude = 0; }
    if (replay_get_float(line, RCOL_KFALT, &v))  { row->kf_altitude = v; row->have |= RHAVE_KFALT; }
    else                                          { row->kf_altitude = 0; }
    if (replay_get_float(line, RCOL_KFVS, &v))   { row->kf_vspeed = v;   row->have |= RHAVE_KFVS; }
    else                                          { row->kf_vspeed = 0; }
    if (replay_get_float(line, RCOL_PRESS, &v))  { row->pressure = v;    row->have |= RHAVE_PRESS; }
    else                                          { row->pressure = 0; }
    if (replay_get_float(line, RCOL_VOLT, &v))   { row->voltage = v;     row->have |= RHAVE_VOLT; }
    else                                          { row->voltage = 0; }
    if (replay_get_float(line, RCOL_NUMSAT, &v)) { row->numsat = (int)v; row->have |= RHAVE_NUMSAT; }
    else                                          { row->numsat = 0; }

    // --- 日付（あれば） ---
    int yy = 0, mo = 0, dd = 0;
    if (replay_get_field(line, replay_col[RCOL_DATE], buf, sizeof(buf)) &&
        replay_parse_date(buf, &yy, &mo, &dd)) {
      row->year = yy; row->month = mo; row->day = dd;
      row->have |= RHAVE_DATE;
    } else {
      row->year = 0; row->month = 0; row->day = 0;
    }

    // 内容を書き終えてから head を進める（Core0 が中途半端な行を読まないようにする）
    replay_head = (replay_head + 1) & (REPLAY_BUF_SIZE - 1);
  }
}

// ===== リプレイ選択画面のファイル一覧 =====
char replayfiles[REPLAY_LIST_ROWS][32];  // 現在表示中のCSVファイル名
int  replayfiles_size[REPLAY_LIST_ROWS]; // 対応するファイルサイズ [byte]
int  replayfiles_count = 0;              // replayfiles[] に入っている件数
int  replayfiles_total = 0;              // SD ルート上の対象CSVの総数

extern volatile bool loading_replaylist;

// リプレイ対象として一覧に出さないシステムCSV。
static const char* const replay_excluded[] = { "mapdata.csv", "destinations.csv", "replay.csv" };

// ファイル名が拡張子 .csv（大小無視）で終わるか。
static bool has_csv_ext(const char* name) {
  size_t len = strlen(name);
  if (len < 5) return false;  // "x.csv" が最短
  const char* ext = name + len - 4;
  return (ext[0] == '.' &&
          (ext[1] == 'c' || ext[1] == 'C') &&
          (ext[2] == 's' || ext[2] == 'S') &&
          (ext[3] == 'v' || ext[3] == 'V'));
}

// リプレイ対象のCSVか判定する（隠しファイル・システムCSVを除外）。
static bool is_replay_candidate(const char* name) {
  if (name[0] == '.') return false;                 // macOS の ._ ファイル等
  if (!has_csv_ext(name)) return false;
  for (unsigned i = 0; i < sizeof(replay_excluded)/sizeof(replay_excluded[0]); i++) {
    if (replay_name_match(name, replay_excluded[i])) return false;  // 大小無視で比較
  }
  return true;
}

// SD ルートのリプレイ対象CSVを列挙し replayfiles[] に格納する（Core1 で実行）。
// start_index: 対象CSVの通し番号（0始まり）のうち、どこから格納するか。
// 戻り値: 1 件以上格納できたら true。
// フォルダは対象外（固定の大会データは replay/ 配下だが一覧には出さず固定項目として扱う）。
bool browse_replay_files(int start_index) {
  if (start_index < 0) start_index = 0;

  for (int i = 0; i < REPLAY_LIST_ROWS; i++) {
    replayfiles[i][0] = '\0';
    replayfiles_size[i] = 0;
  }
  replayfiles_count = 0;
  replayfiles_total = 0;

  FsFile root = SD.open("/");
  if (!root || !root.isDirectory()) {
    DEBUGW_PLN(20260731, "REPLAY: failed to open root");
    if (root) root.close();
    loading_replaylist = false;
    return false;
  }

  while (true) {
    FsFile entry = root.openNextFile();
    if (!entry) break;

    char name[32];
    entry.getName(name, sizeof(name));
    bool isdir = entry.isDirectory();
    uint32_t fsize = isdir ? 0 : entry.size();
    entry.close();

    if (isdir || !is_replay_candidate(name)) continue;

    // 該当ページの範囲内なら格納する
    if (replayfiles_total >= start_index && replayfiles_count < REPLAY_LIST_ROWS) {
      strlcpy(replayfiles[replayfiles_count], name, 32);
      replayfiles_size[replayfiles_count] = (int)fsize;
      replayfiles_count++;
    }
    replayfiles_total++;
  }

  root.close();
  DEBUG_P(20260731, "REPLAY files total: ");
  DEBUG_PLN(20260731, replayfiles_total);
  loading_replaylist = false;
  return replayfiles_count > 0;
}


// ===== リプレイ選択画面の項目モデル =====
// 一覧の通し番号（index）は次の並びになっている:
//   0                          : Replay OFF
//   1                          : PLAY FLIGHT ONLY: YES/NO（トグル）
//   2                          : 2025 Taikai
//   3                          : 2026 Taikai
//   4 .. 4+replayfiles_total-1 : SD ルート上の飛行 CSV
//   4+replayfiles_total        : Return
// 先頭の固定項目数は REPLAY_FIXED_COUNT。
// 描画側もボタン処理側もこのモデルを共有する。

// 一覧の総項目数（固定項目 + ファイル数 + Return）。
int replay_menu_total_items() {
  return REPLAY_FIXED_COUNT + replayfiles_total + 1;
}

// 通し番号が属するページ番号（0始まり）。
int replay_menu_page_of(int index) {
  if (index < 0) return 0;
  return index / REPLAY_LIST_ROWS;
}

// 総ページ数。
int replay_menu_page_count() {
  int total = replay_menu_total_items();
  return (total <= 0) ? 1 : ((total - 1) / REPLAY_LIST_ROWS) + 1;
}

// そのページを描画するために browse_replay_files() に渡すべき「ファイルの開始通し番号」。
// ページ 0 は先頭に固定項目が入る分だけファイルの開始位置が手前にずれる。
int replay_menu_file_start_for_page(int page) {
  int start = page * REPLAY_LIST_ROWS - REPLAY_FIXED_COUNT;
  return (start < 0) ? 0 : start;
}

// 通し番号 index の項目種別を返し、表示用ラベル（ファイルなら実ファイル名）を label に格納する。
// filesize には該当ファイルのサイズ [byte] を返す（ファイル以外は 0）。
// page: 現在 replayfiles[] に読み込まれているページ番号。
ReplayItemType replay_menu_item(int index, int page, char* label, size_t labelsize, int* filesize) {
  if (filesize) *filesize = 0;
  if (label && labelsize > 0) label[0] = '\0';

  if (index < 0 || index >= replay_menu_total_items()) return RITEM_NONE;

  if (index == 0) { strlcpy(label, "Replay OFF (Normal GPS)", labelsize); return RITEM_OFF; }
  if (index == 1) {
    snprintf(label, labelsize, "PLAY FLIGHT ONLY: %s", replay_flight_only ? "YES" : "NO");
    return RITEM_FLIGHTONLY;
  }
  if (index == 2) { strlcpy(label, REPLAY_2025_LABEL, labelsize);         return RITEM_2025; }
  if (index == 3) { strlcpy(label, REPLAY_2026_LABEL, labelsize);         return RITEM_2026; }

  if (index == REPLAY_FIXED_COUNT + replayfiles_total) {
    strlcpy(label, "Return", labelsize);
    return RITEM_RETURN;
  }

  // ファイル項目: 現在読み込まれているページ内の何番目かを求める
  int local = (index - REPLAY_FIXED_COUNT) - replay_menu_file_start_for_page(page);
  if (local < 0 || local >= replayfiles_count) return RITEM_NONE;  // 別ページ = 未読み込み
  strlcpy(label, replayfiles[local], labelsize);
  if (filesize) *filesize = replayfiles_size[local];
  return RITEM_FILE;
}


// ===== SD ブラウザ用変数 =====
char sdfiles[20][32];  // 現在のページに表示するファイル/フォルダ名（最大20エントリ）。フォルダは先頭に'/'を付与
int sdfiles_size[20];  // 対応するファイルのサイズ（バイト）。フォルダは 0
int max_page = -1;     // SD ルートの総ファイル数から計算した最大ページ番号（0-based）

extern bool loading_sddetail;

// SD カードのルートディレクトリをページ単位で列挙し sdfiles[] に格納する。
// page: 0-based のページ番号。1 ページあたり最大 20 エントリ。
// 戻り値: エントリが 1 件以上あれば true、ゼロまたはエラーなら false。
// - 隠しファイル（'.'または"/."で始まるもの）はスキップする（macOS の ._ ファイル除外）。
// - フォルダは名前の先頭に '/' を付けて区別する。
// - 処理完了後に loading_sddetail=false をセットし、表示側に完了を通知する。
bool browse_sd(int page) {
    // Input validation
    if (page < 0) {
        DEBUGW_PLN(20250508,"Invalid page number");
        return false;
    }

    // Clear the array
    for (int i = 0; i < 20; i++) {
        sdfiles[i][0] = '\0';
        sdfiles_size[i] = 0;
    }


    FsFile root = SD.open("/");
    if (!root || !root.isDirectory()) {
        DEBUGW_PLN(20250508,"Failed to open root directory");

        if (root) root.close();
        return false;
    }

    int count = 0;          // Total files/folders processed
    int matched = 0;        // Entries stored in sdfiles
    int total_valid = 0;    // Total non-hidden files/folders

    // Process all entries
    while (true) {
        FsFile entry = root.openNextFile();
        if (!entry) break;

        // Skip hidden files and macOS-specific files starting with "." or "/."
        char name[32];
        entry.getName(name, 32);
        if (name[0] == '.' || (name[0] == '/' && name[1] == '.')) {
            entry.close();
            continue;
        }

        // Count valid entries for max_page calculation
        total_valid++;

        // Store entries for the requested page
        if (count >= page * 20 && matched < 20) {
            char* target = sdfiles[matched];
            
            if (entry.isDirectory()) {
                target[0] = '/';
                strlcpy(target + 1, name, 31); // Leave space for prefix
            } else {
                strlcpy(target, name, 32);
                sdfiles_size[matched] = entry.size();
            }

            // Ensure truncation at 60 chars (including prefix for folders)
            if (strlen(target) > 30) {
                target[30] = '\0';
            }

            matched++;
        }
        
        count++;
        entry.close();
    }

    root.close();

    // Calculate and store maximum page number (0-based)
    max_page = total_valid > 0 ? (total_valid - 1) / 20 : 0;

    // Print results for debugging
    for (int i = 0; i < matched; i++) {
        DEBUG_PLN(20250508,sdfiles[i]);
    }
    DEBUG_P(20250508,"Max page: ");
    DEBUG_PLN(20250508,max_page);
    loading_sddetail = false;
    return matched > 0; // Return true if any entries were found
}




// SdFat ライブラリのファイルタイムスタンプコールバック関数。
// setup_sd() 内で SdFile::dateTimeCallback(dateTime) として登録しておくと、
// ファイル作成・更新時にこの関数が呼ばれて FAT タイムスタンプが書き込まれる。
// fileyear 等は saveCSV() 内で GPS 時刻が最初に取得された瞬間に初期化される。
void dateTime(uint16_t* date, uint16_t* time) {
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(fileyear, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}










// SD の状態に応じて 10秒クールダウン付きで setup_sd(1) による回復を試みる。
// 複数の SD 操作関数から共通で呼ぶヘルパー。
// ケース1: 未初期化かつエラーなし（起動直後に setup_sd が失敗した状態）→ 即時リトライ
// ケース2: sdError 中 → 10秒ごとにリトライ
// どちらも lasttrytime_sd でクールダウンを管理する。
// load_settings=false にすることで、復旧時に設定値（scaleindex 等）を上書きしない。
static void try_sd_recovery() {

  // SD カードが物理的に挿入されていない場合はリトライしない
  if (digitalRead(SD_DETECT)){
    DEBUG_PLN(20250508,"SD card not detected. Skipping SD recovery.");
    return;
  }

  if (!sdInitialized && !sdError) {
    if (millis() - lasttrytime_sd > 10000) {
      lasttrytime_sd = millis();
      setup_sd(1, false);  // 復旧時は設定を再読み込みしない（ユーザー操作中の設定値を保護）
    }
  } else if (sdError) {
    if (millis() - lasttrytime_sd > 10000) {
      lasttrytime_sd = millis();
      setup_sd(1, false);  // 復旧時は設定を再読み込みしない（ユーザー操作中の設定値を保護）
    }
  }

}

// SD カードが正常に使用できる状態かどうかを返す。
// sdInitialized（初期化成功）かつ sdError（書き込みエラーなし）の両方が満たされているとき true。
// sdError 中は try_sd_recovery() で回復を試み、10秒ごとにリトライする。
bool good_sd(){

  // SD リカバリ（setup_sd 再呼出し）は Core1 でのみ実行する。
  // Core0 から try_sd_recovery() を呼ぶと SDIO バスを両コアが同時にアクセスし、
  // バスハングでフリーズする原因になる。
  if (get_core_num() == 1) {
    try_sd_recovery();
  }
  return sdInitialized && !sdError;
}


// log.txt に「起動からの経過秒:テキスト」の形式で 1 行追記する。
// good_sd() で回復チェックも兼ねて使用可否を確認する。
// 書き込み失敗時は sdError=true にして以降の書き込みを無効化する。
//
// SD.open()/close() を毎回呼ぶのではなく、静的 FsFile を使い回すことで
// open/close 時のSDIOハングリスクを最小化する。書き込み後は flush() で反映する。
static FsFile logFileStatic;  // セッション中開きっぱなし。SDエラー時のみ close。

void log_sd(const char* text){
  if (!good_sd()) {
    if (logFileStatic.isOpen()) logFileStatic.close();  // SDエラー時はファイルをリセット
    return;
  }

  // ファイルが未オープンの場合のみ open する（毎回 open/close しない）
  if (!logFileStatic.isOpen()) {
    logFileStatic = SD.open(LOGFILE_NAME, FILE_WRITE);
  }
  if(!logFileStatic){
    DEBUGW_PLN(20250508,"ERR LOG. SD FAIL");
    sdError = true;
    return;
  }

  char logtext[100];   // array to hold the result.
  snprintf(logtext, sizeof(logtext), "%d:%s",(int)(millis()/1000),text);
  logFileStatic.println(logtext);
  logFileStatic.flush();  // close() の代わりに flush() でデータを確定する
}


// printf 書式でフォーマットした文字列を log.txt に追記する。
// 内部で vsnprintf によりバッファに文字列化してから log_sd() を呼ぶ。最大 256 バイト。
void log_sdf(const char* format, ...){
  char buffer[256]; // Temporary buffer for formatted text
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return log_sd(buffer);
}

// GPS の飛行データを CSV ファイルに 1 行追記するフライトログ関数。
// 列: latitude, longitude, gs(m/s), TrueTrack(°), Altitude(m), KF_Altitude(m), KF_Vspeed(m/s), pressure, date, time
//
// ファイル名は最初に GPS 時刻が取得された瞬間に確定し、
// 以降は同じファイルに追記し続ける（例: 2025-05-08_1230.csv）。
// ヘッダ行は headerWritten フラグで 1 回だけ書く。
// SD エラー時は 10 秒ごとに setup_sd(1) でリトライを試みる。
//
// SD.open()/close() を毎回呼ぶのではなく、静的 FsFile を使い回すことで
// open/close 時のSDIOハングリスクを最小化する。書き込み後は flush() で反映する。
// （実体は init_replay() から flush するためファイル前方で定義してある）

void saveCSV(float latitude, float longitude, float gs, int ttrack, float altitude, float kf_altitude, float kf_vspeed, float pressure, int year, int month, int day, int hour, int minute, int second, int centisecond) {
  // 未初期化・エラー時は good_sd() 内の try_sd_recovery() が10秒クールダウン付きで回復を試みる
  if (!good_sd()) {
    if (csvFileStatic.isOpen()) csvFileStatic.close();  // SDエラー時はファイルをリセット
    return;
  }

  //Run only once.
  if(fileyear == 0 && year != 0){
    fileyear = year;
    filemonth = month;
    fileday = day;
    filehour = hour;
    fileminute = minute;
    filesecond = second;
  }

  char csv_filename[30];
  snprintf(csv_filename, sizeof(csv_filename), "%04d-%02d-%02d_%02d%02d.csv", fileyear, filemonth, fileday,filehour,fileminute);

  // ファイルが未オープンの場合のみ open する（毎回 open/close しない）
  if (!csvFileStatic.isOpen()) {
    csvFileStatic = SD.open(csv_filename, FILE_WRITE);
  }

  if (csvFileStatic) {
    if(!headerWritten){
      csvFileStatic.println("latitude,longitude,gs,TrueTrack,Altitude,KF_Altitude,KF_Vspeed,pressure,date,time");
      headerWritten = true;
    }

    csvFileStatic.print(latitude, 6);
    csvFileStatic.print(",");
    csvFileStatic.print(longitude, 6);
    csvFileStatic.print(",");
    csvFileStatic.print(gs, 1);
    csvFileStatic.print(",");
    csvFileStatic.print(ttrack);
    csvFileStatic.print(",");
    csvFileStatic.print(altitude,2);
    csvFileStatic.print(",");
    csvFileStatic.print(kf_altitude,2);  // KF推定高度 [m]（気圧基準）
    csvFileStatic.print(",");
    csvFileStatic.print(kf_vspeed,2);   // KF推定上昇率 [m/s]
    csvFileStatic.print(",");
    csvFileStatic.print(pressure,2);
    csvFileStatic.print(",");

    // Format date as YYYY-MM-DD
    char date[11];
    snprintf(date, sizeof(date), "%04d-%02d-%02d", year, month, day);
    csvFileStatic.print(date);
    csvFileStatic.print(",");

    // Format time as HH:MM:SS.ss（スプレッドシートで時刻値として読み込める形式）
    char time[12];
    snprintf(time, sizeof(time), "%02d:%02d:%02d.%02d", hour, minute, second, centisecond);
    csvFileStatic.println(time);

    // flush() は SD への物理書き込みを伴うため消費電力が大きい。
    // 2秒に1回だけ flush し、それ以外は SdFat の内部バッファに留めることで消費電力を抑制する。
    // 2Hz×2秒 = 4行×約80バイト ≈ 320バイト → 512バイトブロックバッファに収まるため安全。
    { static unsigned long last_flush_ms = 0;
      if (millis() - last_flush_ms >= 2000) {
        TIMING_START(csv_fl);
        csvFileStatic.flush();
        TIMING_END(ts_savecsv_flush, csv_fl);
        last_flush_ms = millis();
      }
    }
    sdError = false; // Reset SD error flag after successful write
  } else {
    if (!sdError) {
      sdError = true;
    }
    sdInitialized = false; // Mark SD card as not initialized for the next attempt
  }
}

// ===== Euler角ログ（BNO085 ROTATION_VECTOR 5Hz） =====
// eulerフォルダに JST 日付のファイルを追記モードで書き込む。
// 50レコードごとに sync() してファイルサイズ・更新タイムスタンプをFATに反映する。
static FsFile eulerFileStatic;
static char eulerOpenedFilename[24] = "";  // 現在開いているファイル名
static int eulerWriteCount = 0;            // sync() タイミング管理カウンタ

void saveEuler(int h, int m, int s, int cs, float roll, float pitch, float yaw, const char* filename, int year, int month, int day) {
  if (!good_sd()) return;

  // ファイル名が変わった場合（日付変更等）は sync してから閉じて開き直す
  if (eulerFileStatic.isOpen() && strcmp(eulerOpenedFilename, filename) != 0) {
    eulerFileStatic.sync();
    eulerFileStatic.close();
    eulerOpenedFilename[0] = '\0';
    eulerWriteCount = 0;
  }

  if (!eulerFileStatic.isOpen()) {
    // eulerフォルダがなければ作成
    if (!SD.exists("euler")) {
      SD.mkdir("euler");
    }
    // 既存ファイルかどうか確認（ヘッダー書き込み判定用）
    bool fileExisted = SD.exists(filename);

    // 追記モードで開く
    if (!eulerFileStatic.open(filename, O_RDWR | O_CREAT | O_APPEND)) return;
    strncpy(eulerOpenedFilename, filename, sizeof(eulerOpenedFilename) - 1);
    eulerOpenedFilename[sizeof(eulerOpenedFilename) - 1] = '\0';

    // GPS 日時でファイルタイムスタンプを設定（作成・アクセス・更新の全3種）
    eulerFileStatic.timestamp(T_CREATE | T_ACCESS | T_WRITE, year, month, day, h, m, s);

    // 新規ファイルのみヘッダー書き込み
    if (!fileExisted) {
      eulerFileStatic.println("time,roll,pitch,yaw");
    }
  }

  // HH:MM:SS.cc,roll,pitch,yaw 形式で書き込み
  char line[64];
  snprintf(line, sizeof(line), "%02d:%02d:%02d.%02d,%.2f,%.2f,%.2f",
           h, m, s, cs, roll, pitch, yaw);
  eulerFileStatic.println(line);

  // 50レコードごと（約10秒@5Hz）に sync() → FATのファイルサイズ・タイムスタンプを更新
  if (++eulerWriteCount >= 50) {
    eulerFileStatic.timestamp(T_WRITE, year, month, day, h, m, s);  // 更新タイムスタンプを現在時刻に
    eulerFileStatic.sync();
    eulerWriteCount = 0;
  }
}

Task createLogEulerTask(int h, int m, int s, int cs, float roll, float pitch, float yaw, const char* filename, int year, int month, int day) {
  Task t;
  t.type = TASK_LOG_EULER;
  t.logEulerArgs.hour        = h;
  t.logEulerArgs.minute      = m;
  t.logEulerArgs.second      = s;
  t.logEulerArgs.centisecond = cs;
  t.logEulerArgs.year        = year;
  t.logEulerArgs.month       = month;
  t.logEulerArgs.day         = day;
  t.logEulerArgs.roll        = roll;
  t.logEulerArgs.pitch       = pitch;
  t.logEulerArgs.yaw         = yaw;
  strncpy(t.logEulerArgs.filename, filename, sizeof(t.logEulerArgs.filename) - 1);
  t.logEulerArgs.filename[sizeof(t.logEulerArgs.filename) - 1] = '\0';
  return t;
}

// 起動時ロゴ BMP 読み込みタスクを生成する（引数なし、ファイル名はハードコード）
Task createLoadLogoTask(){
  Task task;
  task.type = TASK_LOAD_LOGO;
  return task;
}

// ===== BMP ファイルヘッダー構造体 =====
// Windows Bitmap（BMP）のバイナリ構造に対応した構造体定義。
// BITMAPFILEHEADER と BITMAPINFOHEADER を分けて定義し、SdFat の read() で直接読み込む。
// このプロジェクトでは 640×640 ピクセル、16-bit RGB565 の BMP を前提としている。
struct bmp_file_header_t {
  uint16_t signature;       // 2 bytes: should be 'BM'
  uint32_t file_size;       // 4 bytes: total file size
  uint16_t reserved[2];     // 4 bytes: reserved, should be 0
  uint32_t image_offset;    // 4 bytes: offset to image data
};

struct bmp_image_header_t {
  uint32_t header_size;         // 4 bytes: header size (40 bytes for BITMAPINFOHEADER)
  uint32_t image_width;         // 4 bytes: width of the image in pixels
  uint32_t image_height;        // 4 bytes: height of the image in pixels
  uint16_t color_planes;        // 2 bytes: number of color planes (should be 1)
  uint16_t bits_per_pixel;      // 2 bytes: bits per pixel (1, 4, 8, 16, 24, 32)
  uint32_t compression_method;  // 4 bytes: compression method (0 for no compression)
  uint32_t image_size;          // 4 bytes: size of the image data
  uint32_t horizontal_resolution;  // 4 bytes: horizontal resolution (pixels per meter)
  uint32_t vertical_resolution;    // 4 bytes: vertical resolution (pixels per meter)
  uint32_t colors_in_palette;      // 4 bytes: number of colors in the palette
  uint32_t important_colors;       // 4 bytes: number of important colors
};




// ⚠️ 赤道上のみで正確。緯度補正なし。
// 指定ズームレベルでの 1 経度度あたりのピクセル数を返す（赤道基準）。
// Google Maps タイル仕様: ズーム0で全世界が 256px の 1 タイルに収まり、
// ズームが 1 増えるごとにピクセル数は 2 倍になる（256 * 2^zoom / 360）。
double pixelsPerDegreeEQ(int zoom) {
  // Google Maps API approximation: pixels per degree at the given zoom level
  // Zoom 5 is used as the base reference in this example
  return 256 * pow(2, zoom) / 360.0;
}

// ⚠️ 赤道上のみで正確。緯度補正なし。
// 指定ズームレベルでの 1km あたりのピクセル数を返す（赤道基準）。
// 単位変換: px/deg ÷ km/deg = px/km （KM_PER_DEG_LAT ≈ 111.0 km/deg）
double pixelsPerKMEQ_zoom(int zoom){
  return pixelsPerDegreeEQ(zoom)/KM_PER_DEG_LAT;//px/deg / (km/deg) = px /km
}

// 指定緯度・ズームレベルでの緯度方向 1 度あたりのピクセル数を返す。
// Mercator 投影では高緯度ほど地図が伸びる（1 度あたりのピクセルが増える）。
// cos(latitude) で赤道基準値を現地スケールに補正する。緯度 90° に近づくと無限大になるため注意。
double pixelsPerDegreeLat(int zoom,double latitude) {
  // Calculate pixels per degree for latitude
  return pixelsPerDegreeEQ(zoom) / cos(radians(latitude)); // Reference latitude
}


// 値 value を x 度単位のグリッドに丸める汎用ユーティリティ。
// 地図ファイル名に使うグリッド座標の計算に使用する。
// 例: roundToNearestXDegrees(0.2, 35.123) → 35.0（zoom11 の 0.2° グリッド）
double roundToNearestXDegrees(double x, double value) {
  return round(value / x) * x;
}

// ===== 地図スプライト管理変数 =====
// gmap_sprite     : 240×240 ピクセルの地図画像を保持する TFT スプライト（描画バッファ）。
// gmap_loaded_active : スプライトに有効な地図データが入っているかどうか（display_tft.cpp が参照）。
//                     BMP 読み込み中は false に落とし、完了後に true にする。
// new_gmap_ready  : 新しい地図データが準備できたことを display_tft.cpp に通知するフラグ。
// lastsprite_id   : 現在スプライトに読み込まれている BMP の識別子文字列
//                   （ズームレベル + 座標 × 100 + start_x/y を連結した文字列）。
// last_start_x/y  : 前回の地図切り出し開始座標（スクロール差分計算に使う）。
TFT_eSprite gmap_sprite = TFT_eSprite(&tft);
volatile bool gmap_loaded_active = false;
volatile bool new_gmap_ready = false;
char lastsprite_id[20] = "\0";
int last_start_x = 0, last_start_y = 0;

// 指定した緯度・経度・ズームレベルに対応する BMP ファイルを SD から読み込み、
// gmap_sprite に 240×240 ピクセルの地図画像として展開する。
//
// ファイル名規則: z{zoom}/{lat*100}_{lon*100}_z{zoom}.bmp
//   例: z13/3512_13570_z13.bmp  → zoom=13, lat=35.12°, lon=135.70°
//
// ズームレベルとグリッド間隔（BMP 1 枚がカバーする度数）の対応:
//   zoom5=12°, zoom7=3°, zoom9=0.8°, zoom11=0.2°, zoom13=0.05°
//
// 【スクロール最適化】
//   同一 BMP ファイルを使いつつ中心座標が少しずれた場合（start_x/y の変化のみ）、
//   スプライトを scroll() でシフトし、露出した端の列/行だけを BMP から差分再読み込みする。
//   全ピクセル再読み込みを避けることで SD 読み込み時間を大幅に短縮できる。
//
// 【中断対応】
//   abortTask フラグが true になるとピクセルループを途中で抜ける。
//   BMP 読み込み中は gmap_loaded_active=false にし、完了後に true に戻す。
void load_mapimage(double center_lat, double center_lon,int zoomlevel) {
  // Core1 スタック使用量を計測。
  // load_mapimage() は SD からの BMP 読み込み + gmap_sprite への描画を行うため
  // Core1 で最もスタックを消費する（5 秒に 1 回出力）。
  DEBUG_STACK_C1("load_mapimage");
  DEBUG_PLN(20250417,"load mapimage begin");
  TIMING_START(lmi);
#ifndef RELEASE
  if (c0_is_redrawing) _c1_overlap_count++;  // Core0描画と重なったらカウント
#endif
  // ズームレベルに対応したグリッド間隔（度数）を決定する。
  // BMP ファイル 1 枚が round_degrees 度単位のグリッドに配置されているため、
  // center_lat/lon を round_degrees 単位に丸めて対応するファイルを特定する。
  double round_degrees = 0.0;
  if(zoomlevel == 5) round_degrees = 12.0;
  else if(zoomlevel == 7) round_degrees = 3.0;
  else if(zoomlevel == 9) round_degrees = 0.8;
  else if(zoomlevel == 11) round_degrees = 0.2;
  else if(zoomlevel == 13) round_degrees = 0.05;

  //Invalid zoomleel
  if(round_degrees == 0.0){
    gmap_loaded_active = false;
    return;
  }

  double map_lat,map_lon;
  // BMP ファイルの基準座標（グリッドに丸めた lat/lon）を算出する
  map_lat = roundToNearestXDegrees(round_degrees, center_lat);
  map_lon = roundToNearestXDegrees(round_degrees, center_lon);

  // BMP 画像（640×640px）の中心が map_lat/map_lon に対応するため、
  // center_lat/lon の差分をピクセルオフセットに変換して BMP 上の現在位置を求める。
  // BMP の中心は (320, 320)。緯度方向は上が北なので符号が負になる。
  int center_x = (int)(320.0 + (center_lon - map_lon) * pixelsPerDegreeEQ(zoomlevel));
  int center_y = (int)(320.0 - (center_lat - map_lat) * pixelsPerDegreeLat(zoomlevel,center_lat));
  // TFT 画面（240×240）は BMP の一部を切り出して表示する。
  // start_x/y は BMP 上の切り出し左上座標（現在位置が画面中央になるよう計算）。
  int start_x = center_x - 120;
  int start_y = center_y - 120;

  // スプライト識別子を生成する。
  // フォーマット: "zoom(2桁) + lat*100(4桁) + lon*100(5桁) + start_x(3桁) + start_y(3桁)"
  // 同じ識別子なら既にスプライトが最新状態なのでスキップできる。
  // 最初の 11 文字（zoom+lat+lon）が一致すれば同一 BMP ファイルを示す（スクロール判定に使う）。
  char current_sprite_id[36];
  int maplat4 = round(map_lat*100);
  int maplon5 = round(map_lon*100);
  snprintf(current_sprite_id, sizeof(current_sprite_id), "%2d%4d%5d%3d%3d",zoomlevel,maplat4,maplon5,start_x,start_y);

  //Already loaded.
  if(strcmp(current_sprite_id,lastsprite_id) == 0 && gmap_loaded_active){
    //if(rotation != 0){
      //tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
      //gmap_sprite.pushRotated(-rotation);
    //}
    //else
    //  gmap_sprite.pushSprite(0, 40);
    return;
  }
  
  // 識別子の先頭 11 文字（ズーム+緯度+経度）が一致するか確認する。
  // 一致 → 同じ BMP ファイルで切り出し位置だけが変わった（スクロール可能）。
  // 不一致 → 異なる BMP ファイルが必要（全面再描画）。
  bool samefile = true;
  for(int i = 0; i< 11; i++){
    if(current_sprite_id[i] != lastsprite_id[i]){
      samefile = false;
      break;
    }
  }
  int scrollx = 0;
  int scrolly = 0;
  // 同一 BMP かつ既に読み込み済みの場合のみ差分スクロール計算を行う。
  // scrollx/y はスプライトをシフトするピクセル量。正なら右/下へ移動。
  if(samefile && gmap_loaded_active){
    scrollx = (start_x-last_start_x);
    scrolly = (start_y-last_start_y);
  }

  strcpy(lastsprite_id,current_sprite_id);

  char filename[36];
  snprintf(filename, sizeof(filename), "z%d/%04d_%05d_z%d.bmp", zoomlevel,maplat4,maplon5,zoomlevel);


  // Open BMP file
  FsFile bmpImage = SD.open(filename, FILE_READ);
  if (!bmpImage) {
    gmap_loaded_active = false;
    return;
  }

  // Read the file header
  bmp_file_header_t fileHeader;
  bmpImage.read((uint8_t*)&fileHeader.signature, sizeof(fileHeader.signature));
  bmpImage.read((uint8_t*)&fileHeader.file_size, sizeof(fileHeader.file_size));
  bmpImage.read((uint8_t*)fileHeader.reserved, sizeof(fileHeader.reserved));
  bmpImage.read((uint8_t*)&fileHeader.image_offset, sizeof(fileHeader.image_offset));


  // Check signature
  if (fileHeader.signature != 0x4D42) { // 'BM' in little-endian
    bmpImage.close();
    gmap_loaded_active = false;
    return;
  }

  // Image header (assuming 640x640, 16-bit RGB565 BMP file)
  bmp_image_header_t imageHeader;
  bmpImage.read((uint8_t*)&imageHeader.header_size, sizeof(imageHeader.header_size));
  bmpImage.read((uint8_t*)&imageHeader.image_width, sizeof(imageHeader.image_width));
  bmpImage.read((uint8_t*)&imageHeader.image_height, sizeof(imageHeader.image_height));
  bmpImage.read((uint8_t*)&imageHeader.color_planes, sizeof(imageHeader.color_planes));
  bmpImage.read((uint8_t*)&imageHeader.bits_per_pixel, sizeof(imageHeader.bits_per_pixel));
  bmpImage.read((uint8_t*)&imageHeader.compression_method, sizeof(imageHeader.compression_method));
  bmpImage.read((uint8_t*)&imageHeader.image_size, sizeof(imageHeader.image_size));
  bmpImage.read((uint8_t*)&imageHeader.horizontal_resolution, sizeof(imageHeader.horizontal_resolution));
  bmpImage.read((uint8_t*)&imageHeader.vertical_resolution, sizeof(imageHeader.vertical_resolution));
  bmpImage.read((uint8_t*)&imageHeader.colors_in_palette, sizeof(imageHeader.colors_in_palette));
  bmpImage.read((uint8_t*)&imageHeader.important_colors, sizeof(imageHeader.important_colors));


  if (imageHeader.image_width != 640 || imageHeader.image_height != 640 || imageHeader.bits_per_pixel != 16) {
    bmpImage.close();
    gmap_loaded_active = false;
    return;
  }

  // Create the 240x240 sprite
  if(!gmap_sprite.created()){
    gmap_sprite.setColorDepth(16);
    gmap_sprite.createSprite(240, 240);
    // ★ RAM最大消費ポイント: backscreen(115KB)+header_footer(24KB)+vsi_sprite(2.4KB)+audioBuffer(32KB)+gmap_sprite(115KB)が同時確保された直後
    DEBUG_P(20260311, "[RAMピーク] gmap_sprite生成後 FreeHeap=");
    DEBUG_PN(20260311, rp2040.getFreeHeap(), DEC);
    DEBUG_P(20260311, " / Total=");
    DEBUG_PNLN(20260311, rp2040.getTotalHeap(), DEC);
  }
  DEBUG_STACK_C1("after_createSprite");  // スプライト生成直後のスタック使用量
  int tloadbmp_start = millis();

  // この時点からスプライトの書き換えが始まるため、表示側が参照しないよう gmap_loaded_active を落とす。
  gmap_loaded_active = false;

  // ===== スクロール差分描画 =====
  // scrollx/y が 0 以外なら同一 BMP の切り出し位置がずれた → スクロール最適化を使う。
  if(scrollx != 0 || scrolly != 0){
    // スプライトは既にロード済みなので、scroll() でシフトして端だけ補充する。
    // Step 1: 水平スクロール
    // scroll(-scrollx, 0) でスプライトを左右にシフトし、露出した縦帯を BMP から読み込む。
    if (scrollx != 0) {
        gmap_sprite.setScrollRect(0, 0, 240, 240);
        gmap_sprite.scroll(-scrollx, 0);

        // 水平補充: スクロール後に露出した縦帯の x 範囲を決める。
        // scrollx > 0（右へ移動）→ 右端が露出。scrollx < 0（左へ移動）→ 左端が露出。
        int x_start = scrollx > 0 ? 240 - scrollx : 0;
        int x_end = scrollx > 0 ? 240 : -scrollx;

        for (int y = 0; y < 240; y++) {
            loop_tone(); // SDロード中もtone終了判定を維持する
            int bmp_y = last_start_y + y;//Y must be old start_y since we have not scrolled vertically yet.
            if (bmp_y < 0 || bmp_y >= 640) continue; // Skip out-of-bound rows
            if(abortTask)
              break;
            bmpImage.seek(fileHeader.image_offset + (640 - bmp_y - 1) * 640 * 2 + start_x * 2 + x_start * 2);
            for (int x = x_start; x < x_end; x++) {
                int bmp_x = start_x + x;
                if (bmp_x < 0 || bmp_x >= 640) continue; // Skip out-of-bound columns

                // Read pixel data byte by byte
                uint16_t color = bmpImage.read(); // Read low byte
                color |= (bmpImage.read() << 8);  // Read high byte
                gmap_sprite.drawPixel(x, y, color);
            }
        }
    }

    // Step 2: 垂直スクロール
    // scroll(0, -scrolly) でスプライトを上下にシフトし、露出した横帯を BMP から読み込む。
    if (scrolly != 0) {
        gmap_sprite.scroll(0, -scrolly);

        // 垂直補充: スクロール後に露出した横帯の y 範囲を決める。
        // scrolly > 0（下へ移動）→ 下端が露出。scrolly < 0（上へ移動）→ 上端が露出。
        int y_start = scrolly > 0 ? 240 - scrolly : 0;
        int y_end = scrolly > 0 ? 240 : -scrolly;

        for (int y = y_start; y < y_end; y++) {
            loop_tone(); // SDロード中もtone終了判定を維持する
            if(abortTask)
              break;
            int bmp_y = start_y + y;
            if (bmp_y < 0 || bmp_y >= 640) continue; // Skip out-of-bound rows
            bmpImage.seek(fileHeader.image_offset + (640 - bmp_y - 1) * 640 * 2 + start_x * 2);
            for (int x = 0; x < 240; x++) {
                int bmp_x = start_x + x;
                if (bmp_x < 0 || bmp_x >= 640) continue; // Skip out-of-bound columns

                // Read pixel data byte by byte
                uint16_t color = bmpImage.read(); // Read low byte
                color |= (bmpImage.read() << 8);  // Read high byte
                gmap_sprite.drawPixel(x, y, color);
            }
        }
    }
    DEBUG_P(20240815,"scroll x/y/load time=");
    DEBUG_P(20240815,scrollx);
    DEBUG_P(20240815,"/");
    DEBUG_P(20240815,scrolly);
    DEBUG_P(20240815,"/");
    DEBUG_P(20240815,millis()-tloadbmp_start);
    DEBUG_PLN(20240815,"ms");
  }
  // scrollx/y がともに 0 = 新しい BMP ファイルを全面読み込みする。
  // BMP は bottom-up 格納（row 0 が画像の最下行）なので、
  // seek 計算で行を逆順に参照する: row = (640 - bmp_y - 1)。
  else{
    for (int y = 0; y < 240; y++) {
      loop_tone(); // SDロード中もtone終了判定を維持する
      if(abortTask)
        break;
      int bmp_y = start_y + y;
      if (bmp_y < 0 || bmp_y >= 640) continue; // Skip out-of-bound rows
      // BMP の该当行の start_x 列目にシーク
      bmpImage.seek(fileHeader.image_offset + (640 - bmp_y - 1) * 640 * 2 + start_x * 2);
      DEBUG_STACK_C1("bmp_seek");  // SdFat seek 呼び出し直後のスタック使用量（深い呼び出しのピーク付近）
      for (int x = 0; x < 240; x++) {
        int bmp_x = start_x + x;
        if (bmp_x < 0 || bmp_x >= 640) continue; // Skip out-of-bound columns
        // Read pixel data byte by byte
        uint16_t color = bmpImage.read();           // Read low byte
        color |= (bmpImage.read() << 8);            // Read high byte
        gmap_sprite.drawPixel(x, y, color);
      }
    }
  }

  DEBUG_P(20240815,"bmp load time=");
  DEBUG_P(20240815,millis()-tloadbmp_start);
  DEBUG_PLN(20240815,"ms");
  
  bmpImage.close();
  
  // abortTask が立っていた場合は中断扱いとし、スプライトを無効にして終了する。
  // display_tft.cpp 側は gmap_loaded_active=false のままなのでこのスプライトは使わない。
  if(abortTask){
    DEBUG_PLN(20240828,"aborted task! gmap unloaded");
    gmap_loaded_active = false;
    abortTask = false;
    TIMING_END(ts_load_mapimage, lmi);  // 中断時もここまでの実時間を計測
    return;
  }

  // 正常完了: 切り出し位置を記憶して次回のスクロール差分計算に備える。
  // new_gmap_ready=true で display_tft.cpp に「新しい地図が使える」ことを通知する。
  last_start_x = start_x;
  last_start_y = start_y;
  gmap_loaded_active = true;
  new_gmap_ready = true;
  TIMING_END(ts_load_mapimage, lmi);

  DEBUG_PLN(20250424,"gmap_loaded_active! new_gmap_ready!");
}



// SD カードの logo.bmp を読み込んで TFT 画面の左上（0, 0）に直接描画する。
// 主に起動時のスプラッシュロゴ表示に使用する。
// 期待するフォーマット: 240×52 ピクセル、16-bit RGB565 BMP。
// gmap_sprite に一時展開してから pushSprite(0,0) で TFT に転送する。
// シグネチャ不一致またはサイズ不一致の場合は描画をスキップする。
void load_push_logo(){
    // Open BMP file
  FsFile bmpImage = SD.open("logo.bmp", FILE_READ);
  if (!bmpImage) {
    gmap_loaded_active = false;
    return;
  }
  const int sizey = 52; // ロゴ画像の高さ（ピクセル）
  // Read the file header
  bmp_file_header_t fileHeader;
  bmpImage.read((uint8_t*)&fileHeader.signature, sizeof(fileHeader.signature));
  bmpImage.read((uint8_t*)&fileHeader.file_size, sizeof(fileHeader.file_size));
  bmpImage.read((uint8_t*)fileHeader.reserved, sizeof(fileHeader.reserved));
  bmpImage.read((uint8_t*)&fileHeader.image_offset, sizeof(fileHeader.image_offset));
  // Check signature
  if (fileHeader.signature != 0x4D42) { // 'BM' in little-endian
    bmpImage.close();
    gmap_loaded_active = false;
    return;
  }
  // Image header (assuming 640x640, 16-bit RGB565 BMP file)
  bmp_image_header_t imageHeader;
  bmpImage.read((uint8_t*)&imageHeader.header_size, sizeof(imageHeader.header_size));
  bmpImage.read((uint8_t*)&imageHeader.image_width, sizeof(imageHeader.image_width));
  bmpImage.read((uint8_t*)&imageHeader.image_height, sizeof(imageHeader.image_height));
  bmpImage.read((uint8_t*)&imageHeader.color_planes, sizeof(imageHeader.color_planes));
  bmpImage.read((uint8_t*)&imageHeader.bits_per_pixel, sizeof(imageHeader.bits_per_pixel));
  bmpImage.read((uint8_t*)&imageHeader.compression_method, sizeof(imageHeader.compression_method));
  bmpImage.read((uint8_t*)&imageHeader.image_size, sizeof(imageHeader.image_size));
  bmpImage.read((uint8_t*)&imageHeader.horizontal_resolution, sizeof(imageHeader.horizontal_resolution));
  bmpImage.read((uint8_t*)&imageHeader.vertical_resolution, sizeof(imageHeader.vertical_resolution));
  bmpImage.read((uint8_t*)&imageHeader.colors_in_palette, sizeof(imageHeader.colors_in_palette));
  bmpImage.read((uint8_t*)&imageHeader.important_colors, sizeof(imageHeader.important_colors));
  if (imageHeader.image_width != 240 || imageHeader.image_height != sizey || imageHeader.bits_per_pixel != 16) {
    bmpImage.close();
    return;
  }
  // Create the 240x240 sprite
  if(!gmap_sprite.created()){
    gmap_sprite.setColorDepth(16);
    gmap_sprite.createSprite(240, 240);
    // ★ RAM最大消費ポイント: backscreen(115KB)+header_footer(24KB)+vsi_sprite(2.4KB)+audioBuffer(32KB)+gmap_sprite(115KB)が同時確保された直後
    DEBUG_P(20260311, "[RAMピーク] gmap_sprite生成後 FreeHeap=");
    DEBUG_PN(20260311, rp2040.getFreeHeap(), DEC);
    DEBUG_P(20260311, " / Total=");
    DEBUG_PNLN(20260311, rp2040.getTotalHeap(), DEC);
  }
  int tloadbmp_start = millis();

  for (int y = 0; y < sizey; y++) {
      for (int x = 0; x < 240; x++) {
          bmpImage.seek(fileHeader.image_offset + 240*(sizey-y-1) * 2 + x * 2);
          uint16_t color = bmpImage.read(); // Read low byte
          color |= (bmpImage.read() << 8);  // Read high byte
          gmap_sprite.drawPixel(x, y, color);
      }
  }
  bmpImage.close();

  logo_ready = true;  // Core0 に読み込み完了を通知（pushSprite は Core0 が行う）
}

