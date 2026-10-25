// SD card read and write programs.
// All process regarding SD card access are done in Core1.(#2 core)

#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "SdFat.h"
#include <SPI.h>

extern volatile bool quick_redraw;

SdFat32 SD;
bool sdInitialized = false;
bool sdError = false;
#define LOGFILE_NAME "log.txt"

unsigned long lasttrytime_sd = 0;
bool headerWritten = false;
int fileyear = 0;
int filemonth;
int fileday;
int filehour;
int fileminute;
int filesecond;



mutex_t sdMutex;
volatile bool core0NeedsAccess = false;
TaskQueue taskQueue;
mutex_t taskQueueMutex;
Task currentTask;


void saveCSV(const CsvRow& row);
void log_sd(const char* text);
void log_sdf(const char* format, ...);
void dateTime(uint16_t* date, uint16_t* time);




bool isTaskRunning(int taskType) {
  loop0pos = 42;
  return currentTask.type == taskType;
}


// Initializer for TASK_LOG_SD
Task createLogSdTask(const char* logText) {
  loop0pos = 41;
  Task task;
  task.type = TASK_LOG_SD;
  task.logText = logText;
  return task;
}

// Initializer for TASK_LOG_SDF
Task createLogSdfTask(const char* format, ...) {
  loop0pos = 40;
  Task task;
  task.type = TASK_LOG_SDF;
  va_list args;
  va_start(args, format);
  vsnprintf(task.logSdfArgs.buffer, sizeof(task.logSdfArgs.buffer), format, args);
  va_end(args);
  task.logSdfArgs.format = "%s";
  return task;
}


// createLogSdfTask() を積むのと同じことを、Task を呼び出し側のスタックに置かずに行う。
// ★ Core0 のスタックは 4KB しかない（SCRATCH_Y。あふれると Core1 のスタックを壊す）。Task は 264 バイトあり、
//   enqueueTask(createLogSdfTask(...)) を 1 つの関数に何か所も書くと戻り値の分がそれぞれ積まれて、
//   gps_loop() だけで 2.8KB になっていた。ログはこの関数（noinline）の中でだけ Task を作る。
__attribute__((noinline)) bool enqueueLogSdf(const char* format, ...) {
  loop0pos = 40;
  Task task;
  task.type = TASK_LOG_SDF;
  va_list args;
  va_start(args, format);
  vsnprintf(task.logSdfArgs.buffer, sizeof(task.logSdfArgs.buffer), format, args);
  va_end(args);
  task.logSdfArgs.format = "%s";
  return enqueueTask(task);
}


// Initializer for TASK_SAVE_CSV
Task createSaveCsvTask(const CsvRow& row) {
  loop0pos = 39;
  Task task;
  task.type = TASK_SAVE_CSV;
  task.saveCsvArgs = row;
  return task;
}


// キューに積まれている（まだ実行していない）同じ種類のタスクを消す（v7 と同じ実装）。
// ★ 以前は tail から head へ、つまり「空いている側」（実行済みの古いスロット）を走査して head を戻していた。
//   古いスロットに同じ種類が残っていると、先頭のタスクが 2 回実行されたり、満杯近くでは積んだタスクが
//   全部消えたりしえた（設定の保存を積むたびに起こりうる）。
void removeDuplicateTask(TaskType type) {
  loop0pos = 29;
  mutex_enter_blocking(&taskQueueMutex);
  loop0pos = 30;
  int read = taskQueue.head;
  int write = taskQueue.head;
  while (read != taskQueue.tail) {
    loop0pos = 31;
    if (taskQueue.tasks[read].type != type) {
      if (write != read) taskQueue.tasks[write] = taskQueue.tasks[read];
      write = (write + 1) % TASK_QUEUE_SIZE;
    }
    read = (read + 1) % TASK_QUEUE_SIZE;
  }
  taskQueue.tail = write;  // 消した分だけ末尾を詰める
  loop0pos = 37;
  mutex_exit(&taskQueueMutex);
}

void enqueueTaskWithAbortCheck(const Task& newTask) {
  loop0pos = 26;
  //Serial.println("enq task");
  loop0pos = 27;
  removeDuplicateTask(newTask.type);
  loop0pos = 28;
  enqueueTask(newTask);  // Enqueue the new task
}

bool enqueueTask(const Task& task) {
  loop0pos = 17;
  bool queued = false;
  mutex_enter_blocking(&taskQueueMutex);
  loop0pos = 18;
  int nextTail = (taskQueue.tail + 1) % TASK_QUEUE_SIZE;
  loop0pos = 19;
  if (nextTail != taskQueue.head) {  // Queue not full
    loop0pos = 20;
    taskQueue.tasks[taskQueue.tail] = task;
    loop0pos = 21;
    taskQueue.tail = nextTail;
    loop0pos = 22;
    queued = true;
  }
  loop0pos = 23;
  mutex_exit(&taskQueueMutex);
  loop0pos = 24;
  return queued;
}

bool dequeueTask(Task* task) {
  loop0pos = 11;
  bool success = false;
  mutex_enter_blocking(&taskQueueMutex);
  loop0pos = 12;
  if (taskQueue.head != taskQueue.tail) {  // Queue not empty
    loop0pos = 13;
    *task = taskQueue.tasks[taskQueue.head];
    loop0pos = 14;
    taskQueue.head = (taskQueue.head + 1) % TASK_QUEUE_SIZE;
    loop0pos = 15;
    success = true;
  }
  loop0pos = 16;
  mutex_exit(&taskQueueMutex);
  loop0pos = 25;
  return success;
}

//
// mapdata.csv の 1 行（名前,点の数,経度1,緯度1,経度2,緯度2,...）を地図の線に足す。
// 戻り値: 登録したら true、行を捨てたら false。
bool process_mapcsv_line(String line) {
  int index = 0;
  int commaIndex = line.indexOf(',');
  if (commaIndex <= 0) return false;             // コンマが無い / 名前が空
  String name = line.substring(index, commaIndex);
  index = commaIndex + 1;

  commaIndex = line.indexOf(',', index);
  if (commaIndex < 0) return false;              // 座標が無い
  int size = line.substring(index, commaIndex).toInt();
  index = commaIndex + 1;

  // 点の数は CSV の値をそのまま使うので、確保する前に必ず確かめる（v7 と同じ）。
  // 座標 1 組は短くても "0,0," の 4 文字なので、行の長さから上限が決まる。
  // ★ 以前は確かめていなかった。壊れた mapdata.csv（巨大な値・負の値）だと new が失敗し、
  //   例外の無いビルドでは abort() して起動しなくなる（SD を抜くまで直らない）。
  if (size <= 0 || size > (int)(line.length() / 4)) {
    log_sdf("ERR mapdata.csv bad size %d: %.20s", size, name.c_str());
    return false;
  }

  // Allocate memory for the coordinates
  double (*cords)[2] = new double[size][2];
  if (cords == nullptr) return false;            // nothrow 版の new に備える

  for (int i = 0; i < size; i++) {
    commaIndex = line.indexOf(',', index);
    // 最後の点の経度より前でコンマが切れたら、点の数と合わない行として捨てる
    if (commaIndex == -1) {
      delete[] cords;
      return false;
    }
    cords[i][0] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      delete[] cords;
      return false;
    }
    cords[i][1] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
  }

  char* dupname = strdup(name.c_str()); // Duplicate string to allocate memory
  if (dupname == nullptr) {
    delete[] cords;
    return false;
  }
  extramaps[mapdata_count].id = current_id++;
  extramaps[mapdata_count].name = dupname;
  extramaps[mapdata_count].size = size;
  extramaps[mapdata_count].cords = cords;
  mapdata_count++;
  return true;
}


// destinations.csv の 1 行（名前,緯度,経度）を目的地に足す。v7 と同じく壊れた行は捨てる。
// ★ 以前はコンマの無い行が「名前＝行全体・座標 (0,0)」の目的地として登録されていた
//   （String::substring() に -1 を渡すと行全体が返るため）。選ぶと誘導がギニア湾を指す。
// 戻り値: 登録したら true、行を捨てたら false。
bool process_destinationcsv_line(String line) {
  const int c1 = line.indexOf(',');
  if (c1 <= 0) return false;                    // コンマが無い / 名前が空
  const int c2 = line.indexOf(',', c1 + 1);
  if (c2 < 0) return false;                     // 経度が無い

  String name = line.substring(0, c1);
  String slat = line.substring(c1 + 1, c2);
  String slon = line.substring(c2 + 1);
  name.trim(); slat.trim(); slon.trim();
  if (name.length() == 0 || slat.length() == 0 || slon.length() == 0) return false;

  // 明らかな打ち間違いを弾く（toDouble() は数値でない文字列でも 0 を返す）
  const double lat = slat.toDouble();
  const double lon = slon.toDouble();
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
  if (lat == 0.0 && lon == 0.0) return false;

  // 目的地は 1 点だけ持つ。cords[0][0]=緯度 / cords[0][1]=経度（地図ポリゴンとは順序が逆）
  double (*cords)[2] = new double[1][2];
  cords[0][0] = lat;
  cords[0][1] = lon;

  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup(name.c_str()); // Duplicate string to allocate memory
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = cords;
  destinations_count++;
  return true;
}


void init_mapdata() {
  #ifdef DISABLE_SD
    return;
  #endif
  // setup_sd() は SD の復旧時にも呼ばれる。読めたら 2 回目からは読まない（地図を二重に足さない）。
  static bool loaded = false;
  if (loaded) {
    return;
  }
  File32 myFile = SD.open("mapdata.csv");
  if (!myFile) {
    return;
  }
  loaded = true;
  int added = 0, rejected = 0;
  while (myFile.available() && mapdata_count < MAX_MAPDATAS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (process_mapcsv_line(line)) added++;
      else rejected++;
    }
  }
  myFile.close();
  log_sdf("MAPDATA: %d added, %d ignored", added, rejected);
}

// 内蔵の目的地（PLATHOME など）は navdata.cpp で最初から入っている。ここは SD の分を後ろに足すだけ。
// ★ setup_sd() は SD の復旧時にも呼ばれるので、読めたら 2 回目からは読まない（二重に足さない）。
//   以前は呼ぶたびに目的地を作り直していて、選んでいた目的地も PLATHOME に戻っていた。
void init_destinations(){
  static bool loaded = false;
  if (loaded) {
    return;
  }
  File32 myFile = SD.open("destinations.csv");
  if (!myFile) {
    return;
  }
  loaded = true;
  int added = 0, rejected = 0;
  while (myFile.available() && destinations_count < MAX_DESTINATIONS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (process_destinationcsv_line(line)) added++;
      else rejected++;
    }
  }
  myFile.close();
  log_sdf("DESTINATIONS: %d added, %d ignored", added, rejected);
}

// ===== パイロン座標の上書き（override_pilon_coordinate.csv。v7 と同じ）=====
// パイロンの座標は毎年変わり得るが、大会直前にファームウェアを再ビルドするのは難しい。
// SD のトップに override_pilon_coordinate.csv があれば、起動時にその値で上書きする。
//   # で始まる行と空行は無視。NAME,緯度,経度 の 1 行 1 地点。書かれた地点だけが上書きされる。
//   NAME: PLATHOME / N_PILON / S_PILON / PILON_1KM / TAKESHIMA
// 上書きするのは navdata.cpp の course_*。内蔵の目的地・地図のパイロンと基準線・折り返しの円が
// すべてこれを見る。Auto 10km の折り返しは「選んでいる目的地からの距離」なので PLATHOME だけが効く。
static bool apply_pilon_override(const char* name, double lat, double lon) {
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
  if (lat == 0.0 && lon == 0.0) return false;
  double (*p)[2] = nullptr;
  if      (!strcmp(name, "PLATHOME"))  p = course_pla;
  else if (!strcmp(name, "N_PILON"))   p = course_npilon;
  else if (!strcmp(name, "S_PILON"))   p = course_spilon;
  else if (!strcmp(name, "PILON_1KM")) p = course_1km;
  else if (!strcmp(name, "TAKESHIMA")) p = course_takeshima;
  else return false;
  p[0][0] = lat;
  p[0][1] = lon;
  log_sdf("PILON OVERRIDE %s %.6f %.6f", name, lat, lon);
  return true;
}

void load_pilon_override() {
  static bool loaded = false;  // setup_sd() は SD の復旧時にも呼ばれる
  if (loaded) return;
  File32 f = SD.open("override_pilon_coordinate.csv");
  if (!f) return;
  loaded = true;
  int applied = 0, rejected = 0;
  char line[80];
  while (f.fgets(line, sizeof(line)) > 0) {
    char* p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '#' || *p == '\r' || *p == '\n' || *p == '\0') continue;
    char name[16];
    double lat = 0.0, lon = 0.0;
    if (sscanf(p, "%15[^,],%lf,%lf", name, &lat, &lon) != 3) { rejected++; continue; }
    for (int i = (int)strlen(name) - 1; i >= 0 && (name[i] == ' ' || name[i] == '\t'); i--) name[i] = '\0';
    if (apply_pilon_override(name, lat, lon)) applied++;
    else rejected++;
  }
  f.close();
  if (applied > 0) course_override_loaded = true;
  log_sdf("PILON OVERRIDE: %d applied, %d ignored", applied, rejected);
}

// ===== 設定の保存（settings.txt。v7 と同じ "id:value" 形式）=====
// 書くのは Core1（SD は Core1 専用）。中身は Core0 が createSaveSettingsTask() で写したもの。
// 読むのは起動時に 1 回。読んだ値はここでは反映せず、Core0 が take_loaded_settings() で受け取って
// 反映する（地図の縮尺などを Core0 が描いている最中に Core1 から書き換えないため）。
#define SETTINGS_FILE "settings.txt"
extern int destination_mode;   // GPS_TFT_map.ino
extern int scaleindex;         // GPS_TFT_map.ino


static SavedSettings loaded_settings;
static volatile bool loaded_settings_ready = false;

Task createSaveSettingsTask() {
  Task task;
  task.type = TASK_SAVE_SETTINGS;
  SavedSettings& st = task.saveSettingsArgs;
  memset(&st, 0, sizeof(st));
  if (currentdestination >= 0 && currentdestination < destinations_count)
    strncpy(st.destination, extradestinations[currentdestination].name, sizeof(st.destination) - 1);
  st.navigation_mode = destination_mode;
  st.auto10k_status = auto10k_status_flight;
  st.upward_mode = get_upward_mode();
  st.scaleindex = scaleindex;
  return task;
}

static const char* navmode_name(int m) {
  return m == DMODE_AUTO10K ? "AUTO10K" : m == DMODE_FLYAWAY ? "AWAY" : "INTO";
}

static void saveSettings(const SavedSettings& st) {
  #ifdef DISABLE_SD
    return;
  #endif
  if (!good_sd()) return;
  // ★ 読んだ設定を Core0 がまだ反映していない間は書かない。起動直後に縮尺ボタンなどで保存が走ると、
  //   反映前の既定値で settings.txt を上書きして、保存していた設定が消えてしまうため。
  if (loaded_settings_ready) return;
  SD.remove(SETTINGS_FILE);  // 古い項目が残らないよう、消してから書き直す
  File32 f = SD.open(SETTINGS_FILE, FILE_WRITE);
  if (!f) {
    log_sd("ERR settings.txt write");
    return;
  }
  f.print("destination:");     f.print(st.destination); f.print("\n");
  f.print("navigation_mode:"); f.print(navmode_name(st.navigation_mode)); f.print("\n");
  f.print("auto10k_status:");  f.print(st.auto10k_status == AUTO10K_INTO ? "INTO" : "AWAY"); f.print("\n");
  f.print("upward_mode:");     f.print(st.upward_mode == 0 ? "TRACKUP" : "NORTHUP"); f.print("\n");
  f.print("scaleindex:");      f.print(st.scaleindex); f.print("\n");
  f.close();
}

static void parse_setting_line(char* line, SavedSettings& st) {
  char* colon = strchr(line, ':');
  if (!colon) return;
  *colon = '\0';
  const char* key = line;
  const char* v = colon + 1;
  if      (!strcmp(key, "destination"))     { strncpy(st.destination, v, sizeof(st.destination) - 1); st.destination[sizeof(st.destination) - 1] = '\0'; }
  else if (!strcmp(key, "navigation_mode")) st.navigation_mode = !strcmp(v, "AUTO10K") ? DMODE_AUTO10K : !strcmp(v, "AWAY") ? DMODE_FLYAWAY : !strcmp(v, "INTO") ? DMODE_FLYINTO : -1;
  else if (!strcmp(key, "auto10k_status"))  st.auto10k_status = !strcmp(v, "INTO") ? AUTO10K_INTO : !strcmp(v, "AWAY") ? AUTO10K_AWAY : -1;
  else if (!strcmp(key, "upward_mode"))     st.upward_mode = !strcmp(v, "TRACKUP") ? 0 : !strcmp(v, "NORTHUP") ? 1 : -1;
  else if (!strcmp(key, "scaleindex"))      st.scaleindex = (v[0] >= '0' && v[0] <= '9') ? atoi(v) : -1;
}

static void loadSettings() {
  static bool loaded = false;  // setup_sd() は SD の復旧時にも呼ばれる。反映は起動時の 1 回だけ
  if (loaded) return;
  File32 f = SD.open(SETTINGS_FILE, FILE_READ);
  if (!f) return;
  loaded = true;
  SavedSettings st;
  memset(&st, 0, sizeof(st));
  st.navigation_mode = st.auto10k_status = st.upward_mode = st.scaleindex = -1;
  // ★ CR も行の終わりとして扱う（PC のエディタが CRLF で保存すると、値の末尾に \r が残って
  //   文字列の比較が全部外れる。v7 で実際に起きた）。
  char line[48];
  int n = 0;
  while (true) {
    const int c = f.read();
    if (c < 0 || c == '\n' || c == '\r') {
      line[n] = '\0';
      if (n > 0) parse_setting_line(line, st);
      n = 0;
      if (c < 0) break;
    } else if (n < (int)sizeof(line) - 1) {
      line[n++] = (char)c;
    }
  }
  f.close();
  loaded_settings = st;
  __sync_synchronize();  // 中身を書き終えてから「読んだ」を立てる（Core0 が読む）
  loaded_settings_ready = true;
  log_sdf("SETTINGS loaded: dest=%s mode=%d 10k=%d up=%d scale=%d", st.destination,
          st.navigation_mode, st.auto10k_status, st.upward_mode, st.scaleindex);
}

bool take_loaded_settings(SavedSettings* out) {
  if (!loaded_settings_ready) return false;
  __sync_synchronize();
  *out = loaded_settings;
  loaded_settings_ready = false;
  return true;
}

// ===== リプレイ（SD の飛行 CSV を本体で再生する。v7 の 0.92 を v3 の大きさにしたもの）=====
// ★ CSV は列名で解釈する（v3 0.502 までの 6 列、0.503 の 11 列、v7 の CSV のどれでも読める）。
//   要るのは latitude / longitude / time。ほかの列は有れば使う。
ReplayRow replay_rows[REPLAY_BUF_SIZE];
volatile uint8_t replay_head = 0, replay_tail = 0;
volatile bool replay_eof = false;
volatile bool replay_bad = false;
volatile bool replay_load_pending = false;
volatile uint32_t replay_init_seq = 0;

static char replay_files[REPLAY_MAX_FILES][REPLAY_NAME_LEN];
static volatile int replay_files_n = 0;
int replay_file_count() { return replay_files_n; }
const char* replay_file_name(int i) { return (i >= 0 && i < replay_files_n) ? replay_files[i] : nullptr; }

Task createReplayInitTask(const char* name) {
  Task task;
  task.type = TASK_REPLAY_INIT;
  strncpy(task.replayName, name ? name : "", REPLAY_NAME_LEN - 1);
  task.replayName[REPLAY_NAME_LEN - 1] = '\0';
  return task;
}

// 飛行 CSV の名前か（"YYYY-MM-DD_HHMM.csv"）。設定や地図の CSV を再生しないため。
static bool is_flight_csv_name(const char* n) {
  if (strlen(n) != 19) return false;
  for (int i = 0; i < 19; i++) {
    const char c = n[i];
    if (i == 4 || i == 7) { if (c != '-') return false; }
    else if (i == 10) { if (c != '_') return false; }
    else if (i == 15) { if (c != '.') return false; }
    else if (i > 15) continue;
    else if (c < '0' || c > '9') return false;
  }
  return strcasecmp(n + 15, ".csv") == 0;
}

// SD のルートの飛行 CSV を数える（起動時に 1 回。並びはディレクトリの登録順＝書き込まれた順。v7 と同じ）
static void scan_replay_files() {
  static bool scanned = false;
  if (scanned) return;
  File32 root = SD.open("/");
  if (!root) return;
  scanned = true;
  File32 f;
  int n = 0;
  while (n < REPLAY_MAX_FILES && f.openNext(&root, O_RDONLY)) {
    char name[32];
    f.getName(name, sizeof(name));
    if (!f.isDir() && is_flight_csv_name(name)) {
      strncpy(replay_files[n], name, REPLAY_NAME_LEN - 1);
      replay_files[n][REPLAY_NAME_LEN - 1] = '\0';
      n++;
    }
    f.close();
  }
  root.close();
  __sync_synchronize();  // 名前を書き終えてから数を公開する（Core0 が読む）
  replay_files_n = n;
  log_sdf("REPLAY files: %d", n);
}

enum { RC_LAT, RC_LON, RC_GS, RC_TT, RC_ALT, RC_NUMSAT, RC_HACC, RC_SACC, RC_DATE, RC_TIME, RC_COUNT };
static const char* const replay_col_names[RC_COUNT] = {
  "latitude", "longitude", "gs", "truetrack", "gnss_altitude", "numsat", "hacc", "sacc", "date", "time"
};
static int8_t replay_col[RC_COUNT];
static File32 replay_file;
static uint32_t replay_t0 = 0, replay_prev_tod = 0, replay_day_ms = 0, replay_gap_ms = 0, replay_prev_abs = 0, replay_last_t = 0;
static bool replay_t0_valid = false;

// CSV の 1 行から index 番目の欄を取り出す（前後の空白・改行は落とす）
static bool replay_field(const char* line, int index, char* out, size_t n) {
  if (index < 0) return false;
  const char* p = line;
  for (int k = 0; k < index; k++) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
  }
  const char* e = strchr(p, ',');
  size_t len = e ? (size_t)(e - p) : strlen(p);
  if (len >= n) len = n - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  while (len > 0 && (out[len - 1] == '\r' || out[len - 1] == '\n' || out[len - 1] == ' ')) out[--len] = '\0';
  return len > 0;
}

static float replay_float(const char* line, int col) {
  char buf[24];
  if (!replay_field(line, replay_col[col], buf, sizeof(buf))) return NAN;
  return atof(buf);
}

static void replay_init(const char* name) {
  if (replay_file) replay_file.close();
  replay_head = 0;
  replay_tail = 0;
  replay_eof = false;
  replay_bad = false;
  replay_t0_valid = false;
  replay_day_ms = 0;
  replay_gap_ms = 0;
  replay_last_t = 0;
  if (name[0] != '\0') {
    replay_file = SD.open(name, FILE_READ);
    char header[200];
    const int n = replay_file ? replay_file.fgets(header, sizeof(header)) : 0;
    for (int c = 0; c < RC_COUNT; c++) replay_col[c] = -1;
    if (n > 0) {
      // 列名を大文字小文字を区別せずに探す
      char tok[24];
      for (int idx = 0; replay_field(header, idx, tok, sizeof(tok)); idx++) {
        for (int c = 0; c < RC_COUNT; c++) if (strcasecmp(tok, replay_col_names[c]) == 0) replay_col[c] = idx;
        if (strcasecmp(tok, "altitude") == 0 && replay_col[RC_ALT] < 0) replay_col[RC_ALT] = idx;  // v7 の 0.923 以前の名前
      }
    }
    if (!replay_file || n <= 0 || replay_col[RC_LAT] < 0 || replay_col[RC_LON] < 0 || replay_col[RC_TIME] < 0) {
      log_sdf("ERR REPLAY cannot play: %s", name);
      if (replay_file) replay_file.close();
      replay_eof = true;
      replay_bad = true;
    } else {
      log_sdf("REPLAY start: %s", name);
    }
  }
  __sync_synchronize();
  replay_init_seq = replay_init_seq + 1;   // Core0 はこれが変わったのを見て再生の時計を 0 に戻す
}

// リングの空きを埋められるだけ埋める。読めない行は飛ばす。
static void replay_load() {
  while (replay_file && !replay_eof) {
    const uint8_t used = (uint8_t)((replay_head - replay_tail) & (REPLAY_BUF_SIZE - 1));
    if (used >= REPLAY_BUF_SIZE - 1) break;   // 満杯（1 スロットは空けておく）
    char line[200];
    if (replay_file.fgets(line, sizeof(line)) <= 0) { replay_eof = true; break; }
    char buf[24];
    int hh, mm, ss;
    if (!replay_field(line, replay_col[RC_TIME], buf, sizeof(buf)) || sscanf(buf, "%d:%d:%d", &hh, &mm, &ss) != 3) continue;
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 60) continue;
    char la[24], lo[24];
    if (!replay_field(line, replay_col[RC_LAT], la, sizeof(la)) || !replay_field(line, replay_col[RC_LON], lo, sizeof(lo))) continue;
    ReplayRow& r = replay_rows[replay_head];
    r.latitude = atof(la);
    r.longitude = atof(lo);
    if (r.latitude == 0.0 && r.longitude == 0.0) continue;
    const uint32_t tod = ((uint32_t)hh * 3600UL + mm * 60UL + ss) * 1000UL;
    if (replay_t0_valid && tod + 60000UL < replay_prev_tod) replay_day_ms += 86400000UL;  // 日付をまたいだ
    replay_prev_tod = tod;
    const uint32_t abs_ms = tod + replay_day_ms;
    if (!replay_t0_valid) { replay_t0 = abs_ms; replay_prev_abs = abs_ms; replay_t0_valid = true; }
    // 記録が途切れた区間（電源を切った・測位を失った）は 1 秒に詰める（何分も止まったまま再生しない）
    if (abs_ms > replay_prev_abs + 5000UL) replay_gap_ms += abs_ms - replay_prev_abs - 1000UL;
    replay_prev_abs = abs_ms;
    // 時刻が少し戻った行（記録の乱れ）は前の行と同じ時刻にする。
    // ★ そのまま引くと 0 より前になって uint32_t が回り込み、そこで再生が止まったままになる。
    const int32_t t = (int32_t)(abs_ms - replay_t0 - replay_gap_ms);
    r.t_ms = (t < (int32_t)replay_last_t) ? replay_last_t : (uint32_t)t;
    replay_last_t = r.t_ms;
    r.gs = replay_float(line, RC_GS);
    r.truetrack = replay_float(line, RC_TT);
    r.altitude = replay_float(line, RC_ALT);
    r.hacc = replay_float(line, RC_HACC);
    r.sacc = replay_float(line, RC_SACC);
    const float ns = replay_float(line, RC_NUMSAT);
    r.numsat = isnan(ns) ? -1 : (int)ns;
    r.year = 0;
    if (replay_field(line, replay_col[RC_DATE], buf, sizeof(buf))) sscanf(buf, "%d-%d-%d", &r.year, &r.month, &r.day);
    r.hour = hh; r.minute = mm; r.second = ss;
    __sync_synchronize();   // 行を書き終えてから head を進める（Core0 が読む）
    replay_head = (replay_head + 1) & (REPLAY_BUF_SIZE - 1);
  }
  replay_load_pending = false;
}

void setup_sd(){
  #ifdef DISABLE_SD
    return;
  #endif

  SPI.setRX(SD_RX);
  SPI.setSCK(SD_SCK);
  SPI.setTX(SD_TX);
  #ifdef RP2040_ZERO
  SPI.setCS(SD_CS_PIN);
  SPI.begin();
  #endif
  #ifdef RP2040_PICO
  SPI.begin(true);
  #endif
  
  
  sdInitialized = SD.begin(SD_CS_PIN,SD_SCK_MHZ(50));

  if (!sdInitialized) {
    return;
  }else{
    sdError = false;
    // set date time callback function
    SdFile::dateTimeCallback(dateTime);
    log_sd("SD INIT");
    init_mapdata();
    load_pilon_override();   // 目的地より先（内蔵の目的地は course_* を指しているので、順序は表示だけの問題）
    init_destinations();
    loadSettings();          // 目的地の後（目的地を名前で探すのは Core0 だが、SD の目的地が出そろってから渡す）
    scan_replay_files();
  }
}


volatile unsigned long loop1counter = 0;
volatile int loop1pos = 0;
volatile int loop0pos = 0;

unsigned long last_loop1counter = 0;
unsigned long time_loop1counter_updated = 0;

void setup1(void){
  mutex_init(&taskQueueMutex);
  setup_sd();
}

void loop1() {
  loop1pos = 0;
  loop1counter++;
  loop1pos = 1;
  if (dequeueTask(&currentTask)) {
    loop1pos = 2;
    switch (currentTask.type) {
      case TASK_INIT_SD:
        loop1pos = 3;
        setup_sd();
        break;
      case TASK_LOG_SD:
        loop1pos = 4;
        log_sd(currentTask.logText);
        break;
      case TASK_LOG_SDF:
        loop1pos = 5;
        log_sdf(currentTask.logSdfArgs.format, currentTask.logSdfArgs.buffer);
        break;
      case TASK_SAVE_CSV:
        loop1pos = 6;
        saveCSV(currentTask.saveCsvArgs);
        break;
      case TASK_REPLAY_INIT:
        loop1pos = 12;
        replay_init(currentTask.replayName);
        break;
      case TASK_REPLAY_LOAD:
        loop1pos = 13;
        replay_load();
        break;
      case TASK_SAVE_SETTINGS:
        loop1pos = 11;
        saveSettings(currentTask.saveSettingsArgs);
        break;
      case TASK_NONE:
        break;
    }
    loop1pos = 8;
    currentTask.type = TASK_NONE;
  } else {
    loop1pos = 9;
    // No tasks, optionally sleep or yield
    delay(10);
  }
  loop1pos = 10;
}

void dateTime(uint16_t* date, uint16_t* time) {
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(fileyear, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}










bool good_sd(){
  return sdInitialized && !sdError;
}


File32 logFile;
void log_sd(const char* text){
  loop1pos = 100;
  #ifdef DISABLE_SD
    return;
  #endif

  File32 logFile = SD.open("log2.txt", FILE_WRITE);
  if(!logFile){
    Serial.println("ERR LOG");
    sdError = true;
    return;
  }

  loop1pos = 101;
  // if the file opened okay, write to it:
  if (logFile) {
    loop1pos = 102;
    // log_sdf() は 256 バイトまで整形してくるので、それより大きくして snprintf で切る
    // （以前は 100 バイトに sprintf していて、長い行で Core1 のスタックを壊しえた）。
    char logtext[280];
    loop1pos = 103;
    snprintf(logtext, sizeof(logtext), "%d:%s",(int)(millis()/1000),text);
    loop1pos = 104;
    logFile.println(logtext);
    loop1pos = 105;
    // close the file:
    logFile.close();
    loop1pos = 106;
  }
}


void log_sdf(const char* format, ...){
  loop1pos = 107;

  #ifdef DISABLE_SD
    return;
  #endif

  char buffer[256]; // Temporary buffer for formatted text
  loop1pos = 112;
  va_list args;
  loop1pos = 111;
  va_start(args, format);
  loop1pos = 108;
  vsnprintf(buffer, sizeof(buffer), format, args);
  loop1pos = 109;
  va_end(args);
  loop1pos = 110;
  return log_sd(buffer);
}






void saveCSV(const CsvRow& r) {
  loop1pos = 200;
  #ifdef DISABLE_SD
    return;
  #endif

  if (!sdInitialized && !sdError) {
    sdInitialized = SD.begin(SD_CS_PIN);
    if (!sdInitialized) {
      sdError = true;
      return;
    }
  }
  loop1pos = 201;

  if(sdError){
    if(millis()-lasttrytime_sd > 10000){//10秒以上たっていたら、リトライ
      lasttrytime_sd = millis();
      setup_sd();
    }
    return;
  }
  loop1pos = 202;

  //Run only once.
  if(fileyear == 0 && r.year != 0){
    fileyear = r.year;
    filemonth = r.month;
    fileday = r.day;
    filehour = r.hour;
    fileminute = r.minute;
    filesecond = r.second;
  }
  loop1pos = 203;

  char csv_filename[30];
  snprintf(csv_filename, sizeof(csv_filename), "%04d-%02d-%02d_%02d%02d.csv", fileyear, filemonth, fileday,filehour,fileminute);
  // ファイル名は起動ごとに 1 つなので、記録も最初の 1 回だけ（以前は毎秒 log2.txt に書いていた）
  static bool filename_logged = false;
  if (!filename_logged) {
    filename_logged = true;
    log_sdf("CSV %s", csv_filename);
  }
  loop1pos = 204;

  File32 csvFile = SD.open(csv_filename, FILE_WRITE);
  loop1pos = 205;

  if (csvFile) {
    if(!headerWritten){
      // 列名は v7 にそろえてある（v7 のリプレイ・ツールは列名で読む）。hAcc / sAcc は v3 で足した列。
      csvFile.println("latitude,longitude,gs,TrueTrack,GNSS_Altitude,voltage,numsat,date,time,hAcc,sAcc");
      headerWritten = true;
    }
    loop1pos = 206;

    char line[160];
    char volt[8] = "";
    if (!isnan(r.voltage)) snprintf(volt, sizeof(volt), "%.2f", r.voltage);
    snprintf(line, sizeof(line), "%.7f,%.7f,%.2f,%.1f,%.1f,%s,%d,%04d-%02d-%02d,%02d:%02d:%02d,%.1f,%.2f",
             r.latitude, r.longitude, r.gs, r.truetrack, r.altitude, volt, r.numsat,
             r.year, r.month, r.day, r.hour, r.minute, r.second, r.hacc, r.sacc);
    csvFile.println(line);

    loop1pos = 208;
    csvFile.close();
    sdError = false; // Reset SD error flag after successful write
  } else {
    if (!sdError) {
      sdError = true;
    }
    sdInitialized = false; // Mark SD card as not initialized for the next attempt
  }

  loop1pos = 209;
}

// 地図画像（SD の BMP）の読み込みは 0.503 で廃止した。地図はフラッシュ内蔵のベクタ地図（src/vectormap.cpp）。


