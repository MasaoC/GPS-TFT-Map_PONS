// SD card read and write programs.
// All process regarding SD card access are done in Core1.(#2 core)

#include <SPI.h>

#include "mysd.h"
#include "navdata.h"
#include "settings.h"
//#define DISABLE_FS_H_WARNING
#include "SdFat.h"
#include "sound.h"

#define MAX_SETTING_LENGTH 32
#define MAX_LINE_LENGTH (MAX_SETTING_LENGTH * 2 + 2) // ID:value + separator and newline


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
volatile bool abortTask = false;
TaskQueue taskQueue;
mutex_t taskQueueMutex;
Task currentTask;


void load_mapimage(double center_lat, double center_lon,int zoomlevel);
void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second);
void log_sd(const char* text);
void log_sdf(const char* format, ...);
void dateTime(uint16_t* date, uint16_t* time);



//====================SD settings.txt=========
// Array of settings (extend this as needed)
SDSetting settings[] = {
  {"volume", setVolume, getVolume},
  {"destination", setDestination, getDestination},
  {"navigation_mode", setNavigationMode, getNavigationMode},
  {"scaleindex", setScaleIndex, getScaleIndex}
};
const int numSettings = sizeof(settings) / sizeof(settings[0]);
extern volatile int sound_volume;
extern int destination_mode;
extern int scaleindex;
extern double scalelist[6];
extern double scale;


// Save all settings to settings.txt
bool saveSettings() {
  SD.remove("settings.txt"); // Remove old file to start fresh
  File32 file = SD.open("settings.txt", FILE_WRITE);
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


// Load settings from settings.txt
bool loadSettings() {
  File32 file = SD.open("settings.txt", FILE_READ);
  if (!file) {
    ("Failed to open settings.txt for reading");
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



// Example setter and getter functions
void setVolume(const char* value) {
  sound_volume = atoi(value);
  DEBUG_P(20250508,"Set volume to: ");
  DEBUG_PLN(20250508,sound_volume);
}

void getVolume(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%d", sound_volume);
}

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

void getDestination(char* buffer, size_t bufferSize) {
  if(bufferSize <= 0)
    return;
  strncpy(buffer, extradestinations[currentdestination].name, bufferSize);
  buffer[bufferSize - 1] = '\0';
}


bool isTaskRunning(int taskType) {
  return currentTask.type == taskType;
}

bool isTaskInQueue(int taskType){
    int current = taskQueue.head;
    // Iterate from head to tail-1
    while (current != taskQueue.tail) {
        if (taskQueue.tasks[current].type == taskType) {
            return true;
        }
        current = (current + 1) % TASK_QUEUE_SIZE;
    }
    return false;
}

Task createSaveSettingTask(){
  Task task;
  task.type = TASK_SAVE_SETTINGS;
  return task;
}


// Initializer for TASK_LOG_SD
Task createLogSdTask(const char* logText) {
  Task task;
  task.type = TASK_LOG_SD;
  task.logText = logText;
  return task;
}

// Initializer for TASK_LOG_SDF
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


// Initializer for TASK_SAVE_CSV
Task createSaveCsvTask(float latitude, float longitude, float gs, int mtrack, int year, int month, int day, int hour, int minute, int second) {
  Task task;
  task.type = TASK_SAVE_CSV;
  task.saveCsvArgs.latitude = latitude;
  task.saveCsvArgs.longitude = longitude;
  task.saveCsvArgs.gs = gs;
  task.saveCsvArgs.mtrack = mtrack;
  task.saveCsvArgs.year = year;
  task.saveCsvArgs.month = month;
  task.saveCsvArgs.day = day;
  task.saveCsvArgs.hour = hour;
  task.saveCsvArgs.minute = minute;
  task.saveCsvArgs.second = second;
  return task;
}

// Initializer for TASK_LOAD_MAPIMAGE
Task createLoadMapImageTask(double center_lat, double center_lon, int zoomlevel) {
  Task task;
  task.type = TASK_LOAD_MAPIMAGE;
  task.loadMapImageArgs.center_lat = center_lat;
  task.loadMapImageArgs.center_lon = center_lon;
  task.loadMapImageArgs.zoomlevel = zoomlevel;
  return task;
}

Task createPlayMultiToneTask(int freq, int duration, int count,int priority){
  Task task;
  task.type = TASK_PLAY_MULTITONE;
  task.playMultiToneArgs.freq = freq;
  task.playMultiToneArgs.duration = duration;
  task.playMultiToneArgs.counter = count;
  task.playMultiToneArgs.priority = priority;
  return task;
}

Task createPlayWavTask(const char* filename, int priority){
  Task task;
  task.type = TASK_PLAY_WAV;
  task.playWavArgs.wavfilename = filename;
  task.playWavArgs.priority = priority;
  return task;
}


Task createBrowseSDTask(int page){
  Task task;
  task.type = TASK_BROWSE_SD;
  task.pagenum = page;
  return task;
}


void removeDuplicateTask(TaskType type) {
    mutex_enter_blocking(&taskQueueMutex);

    if (taskQueue.head == taskQueue.tail) {
        mutex_exit(&taskQueueMutex);
        return;
    }

    int read = taskQueue.head;
    int write = taskQueue.head;

    // Loop through the queue from head to tail
    while (read != taskQueue.tail) {
        if (taskQueue.tasks[read].type != type) {
            if (write != read) {
                taskQueue.tasks[write] = taskQueue.tasks[read];
            }
            write = (write + 1) % TASK_QUEUE_SIZE;
        }
        read = (read + 1) % TASK_QUEUE_SIZE;
    }

    taskQueue.tail = write;

    mutex_exit(&taskQueueMutex);
}

void enqueueTaskWithAbortCheck(Task newTask) {
  if (isTaskRunning(newTask.type) && newTask.type == TASK_LOAD_MAPIMAGE) {  // Implement this check based on your task handling
    if(newTask.loadMapImageArgs.zoomlevel == currentTask.loadMapImageArgs.zoomlevel){
      //Same zoom level. Meaning mapimage loading in progress, just be patient and dont add another task of loading image.
      DEBUGW_P(20250429,"NOT enque the task:");
      DEBUGW_PLN(20250429,newTask.type);
      return;
    }
    
    // Abort the old task with different zoomlevel = meaning user changed zoom level while loading image.
    abortTask = true;  // Notify the running task to abort
  }
  removeDuplicateTask(newTask.type);
  enqueueTask(newTask);  // Enqueue the new task
}

void enqueueTask(Task task) {
  mutex_enter_blocking(&taskQueueMutex);
  int nextTail = (taskQueue.tail + 1) % TASK_QUEUE_SIZE;
  if (nextTail != taskQueue.head) {  // Queue not full
    taskQueue.tasks[taskQueue.tail] = task;
    taskQueue.tail = nextTail;
  }
  mutex_exit(&taskQueueMutex);
}

bool dequeueTask(Task* task) {
  bool success = false;
  mutex_enter_blocking(&taskQueueMutex);
  if (taskQueue.head != taskQueue.tail) {  // Queue not empty
    *task = taskQueue.tasks[taskQueue.head];
    taskQueue.head = (taskQueue.head + 1) % TASK_QUEUE_SIZE;
    success = true;
  }
  mutex_exit(&taskQueueMutex);
  return success;
}

//
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

bool mapdatainitialized = false;
void init_mapdata() {
  #ifdef DISABLE_SD
    return;
  #endif
  if(mapdatainitialized){
    DEBUG_PLN(20241006,"Map already initialized.");
    return;//only once to avoid possible memory leak.
  }
  File32 myFile = SD.open("mapdata.csv");
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


void load_destinations(){
  File32 myFile = SD.open("destinations.csv");
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

void setup_sd(int trycount){
  for(int i = 0; i < trycount; i++){
    sdInitialized = SD.begin(SdioConfig(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO));
    if(sdInitialized)
      break;
    delay(100);
  }
  DEBUG_PLN(20250508, "SUCCESS SD setup");
  
  if (!sdInitialized) {
    return;
  }else{
    sdError = false;
    // set date time callback function
    SdFile::dateTimeCallback(dateTime);
    log_sd("SD INIT");
    init_mapdata();
    load_destinations();
    if(!loadSettings()){
      DEBUGW_PLN(20250508,"Error loading settings.");
    }
  }
}

char sdfiles[20][32]; 
int sdfiles_size[20]; 
int max_page = -1;
extern bool loading_sddetail;
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


    File32 root = SD.open("/");
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
        File32 entry = root.openNextFile();
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




void dateTime(uint16_t* date, uint16_t* time) {
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(fileyear, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}










bool good_sd(){
  return sdInitialized && !sdError;
}


void log_sd(const char* text){
  #ifdef DISABLE_SD
    return;
  #endif

  File32 logFile = SD.open("log2.txt", FILE_WRITE);
  if(!logFile){
    DEBUGW_PLN(20250508,"ERR LOG");
    sdError = true;
    return;
  }

  // if the file opened okay, write to it:
  if (logFile) {
    char logtext[100];   // array to hold the result.
    sprintf(logtext,"%d:%s",(int)(millis()/1000),text);
    logFile.println(logtext);
    // close the file:
    logFile.close();
  }
}


void log_sdf(const char* format, ...){
  #ifdef DISABLE_SD
    return;
  #endif

  char buffer[256]; // Temporary buffer for formatted text
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return log_sd(buffer);
}






void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second) {
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

  if(sdError){
    if(millis()-lasttrytime_sd > 10000){//10秒以上たっていたら、リトライ
      lasttrytime_sd = millis();
      setup_sd(1);
    }
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
  sprintf(csv_filename, "%04d-%02d-%02d_%02d%02d.csv", fileyear, filemonth, fileday,filehour,fileminute);
  log_sdf("%d:%s",millis(),csv_filename);

  File32 csvFile = SD.open(csv_filename, FILE_WRITE);

  if (csvFile) {
    if(!headerWritten){
      csvFile.println("latitude,longitude,gs,TrueTrack,date,time");
      headerWritten = true;
    }

    csvFile.print(latitude, 6);
    csvFile.print(",");
    csvFile.print(longitude, 6);
    csvFile.print(",");
    csvFile.print(gs, 1);
    csvFile.print(",");
    csvFile.print(ttrack);
    csvFile.print(",");

    // Format date as YYYY-MM-DD
    char date[11];
    sprintf(date, "%04d-%02d-%02d", year, month, day);
    csvFile.print(date);
    csvFile.print(",");

    // Format time as HH:MM:SS
    char time[9];
    sprintf(time, "%02d:%02d:%02d", hour, minute, second);
    csvFile.println(time);

    csvFile.close();
    sdError = false; // Reset SD error flag after successful write
  } else {
    if (!sdError) {
      sdError = true;
    }
    sdInitialized = false; // Mark SD card as not initialized for the next attempt
  }
}

// Define BMP header structures
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




// Warning Only applicable at equator.
double pixelsPerDegreeEQ(int zoom) {
  // Google Maps API approximation: pixels per degree at the given zoom level
  // Zoom 5 is used as the base reference in this example
  return 256 * pow(2, zoom) / 360.0;
}

// Warning Only applicable at equator.
double pixelsPerKMEQ_zoom(int zoom){
  return pixelsPerDegreeEQ(zoom)/KM_PER_DEG_LAT;//px/deg / (km/deg) = px /km
}

double pixelsPerDegreeLat(int zoom,double latitude) {
  // Calculate pixels per degree for latitude
  return pixelsPerDegreeEQ(zoom) / cos(radians(latitude)); // Reference latitude
}


// Function to round a value to the nearest 5 degrees
double roundToNearestXDegrees(double x, double value) {
  return round(value / x) * x;
}

TFT_eSprite gmap_sprite = TFT_eSprite(&tft);
bool init_gmap = false;
volatile bool gmap_loaded_active = false;
volatile bool new_gmap_ready = false;
char lastsprite_id[20] = "\0";
int last_start_x,last_start_y = 0;

void load_mapimage(double center_lat, double center_lon,int zoomlevel) {
  DEBUG_PLN(20250417,"load mapimage begin");
  //Setting up resolution of bmp image according to zoomlevel.
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
  // Round latitude and longitude to the nearest 5 degrees
  map_lat = roundToNearestXDegrees(round_degrees, center_lat);
  map_lon = roundToNearestXDegrees(round_degrees, center_lon);
  
  // Calculate pixel coordinates for given latitude and longitude
  int center_x = (int)(320.0 + (center_lon - map_lon) * pixelsPerDegreeEQ(zoomlevel));
  int center_y = (int)(320.0 - (center_lat - map_lat) * pixelsPerDegreeLat(zoomlevel,center_lat));
  // Calculate top-left corner of 240x240 region
  int start_x = center_x - 120;
  int start_y = center_y - 120;

  char current_sprite_id[36];
  int maplat4 = round(map_lat*100);
  int maplon5 = round(map_lon*100);
  sprintf(current_sprite_id,"%2d%4d%5d%3d%3d",zoomlevel,maplat4,maplon5,start_x,start_y);

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
  
  //Check to scroll bmp
  bool samefile = true;
  for(int i = 0; i< 11; i++){
    if(current_sprite_id[i] != lastsprite_id[i]){
      samefile = false;
      break;
    }
  }
  int scrollx = 0;
  int scrolly = 0;
  //If exact same file already loaded.
  if(samefile && gmap_loaded_active){
    scrollx = (start_x-last_start_x);
    scrolly = (start_y-last_start_y);
  }

  strcpy(lastsprite_id,current_sprite_id);

  char filename[36];
  sprintf(filename, "z%d/%04d_%05d_z%d.bmp", zoomlevel,maplat4,maplon5,zoomlevel);


  // Open BMP file
  File32 bmpImage = SD.open(filename, FILE_READ);
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
  }
  int tloadbmp_start = millis();

  //From nowon, gmap under edit. so unload gmap.
  gmap_loaded_active = false;

  //Scroll
  if(scrollx != 0 || scrolly != 0){
    //gmap_sprite is already loaded.
    // Step 1: Horizontal Scroll

    if (scrollx != 0) {
        gmap_sprite.setScrollRect(0, 0, 240, 240);
        gmap_sprite.scroll(-scrollx, 0);

        // Horizontal filling
        int x_start = scrollx > 0 ? 240 - scrollx : 0;
        int x_end = scrollx > 0 ? 240 : -scrollx;

        for (int y = 0; y < 240; y++) {
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

    // Step 2: Vertical Scroll
    if (scrolly != 0) {
        gmap_sprite.scroll(0, -scrolly);

        // Vertical filling
        int y_start = scrolly > 0 ? 240 - scrolly : 0;
        int y_end = scrolly > 0 ? 240 : -scrolly;

        for (int y = y_start; y < y_end; y++) {
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
  // scrollx and scrolly is zero = New file loading.
  else{
    
    for (int y = 0; y < 240; y++) {
      if(abortTask)
        break;
      int bmp_y = start_y + y;
      if (bmp_y < 0 || bmp_y >= 640) continue; // Skip out-of-bound rows
      bmpImage.seek(fileHeader.image_offset + (640 - bmp_y - 1) * 640 * 2 + start_x * 2);
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
  
  if(abortTask){
    DEBUG_PLN(20240828,"aborted task! gmap unloaded");
    gmap_loaded_active = false;
    abortTask = false;
    return;
  }

  last_start_x = start_x;
  last_start_y = start_y;
  gmap_loaded_active = true;
  new_gmap_ready = true;

  DEBUG_PLN(20250424,"gmap_loaded_active! new_gmap_ready!");
}



void load_push_logo(){
    // Open BMP file
  File32 bmpImage = SD.open("logo.bmp", FILE_READ);
  if (!bmpImage) {
    gmap_loaded_active = false;
    return;
  }
  const int sizey = 52;
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

  gmap_sprite.pushSprite(0,0);
}

