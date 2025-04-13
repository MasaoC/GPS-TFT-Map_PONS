// SD card read and write programs.
// All process regarding SD card access are done in Core1.(#2 core)

#include "mysd.h"
#include "navdata.h"
#include "settings.h"
//#define DISABLE_FS_H_WARNING
#include <SD.h>
#include <SPI.h>
#include "sound.h"

extern volatile bool quick_redraw;

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


// Initializer for TASK_SAVE_CSV
Task createSaveCsvTask(float latitude, float longitude, float gs, int mtrack, int year, int month, int day, int hour, int minute, int second) {
  loop0pos = 39;
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
  loop0pos = 38;
  Task task;
  task.type = TASK_LOAD_MAPIMAGE;
  task.loadMapImageArgs.center_lat = center_lat;
  task.loadMapImageArgs.center_lon = center_lon;
  task.loadMapImageArgs.zoomlevel = zoomlevel;
  return task;
}

Task createPlayMultiToneTask(double freq, double duration, int count){
  loop0pos = 39;
  Task task;
  task.type = TASK_PLAY_MULTITONE;
  task.playMultiToneArgs.freq = freq;
  task.playMultiToneArgs.duration = duration;
  task.playMultiToneArgs.counter = count;
  return task;
}

void removeDuplicateTask(TaskType type) {
    mutex_enter_blocking(&taskQueueMutex);
    
    // If queue is empty, nothing to do
    if (taskQueue.head == taskQueue.tail) {
        mutex_exit(&taskQueueMutex);
        return;
    }


    Serial.println("removeDup()");
    Serial.print(taskQueue.head);
    Serial.print(":");
    Serial.println(taskQueue.tail);

    int current = taskQueue.head;
    
    // Iterate from head to tail-1
    while (current != taskQueue.tail) {
        if (taskQueue.tasks[current].type == type) {
            // Found a match, shift tasks forward
            int next = (current + 1) % TASK_QUEUE_SIZE;
            while (next != taskQueue.tail) {
                taskQueue.tasks[current] = taskQueue.tasks[next];
                current = next;
                next = (next + 1) % TASK_QUEUE_SIZE;
            }
            // Update tail to reflect removal
            taskQueue.tail = (taskQueue.tail - 1 + TASK_QUEUE_SIZE) % TASK_QUEUE_SIZE;
            break; // Remove only the first matching task
        }
        current = (current + 1) % TASK_QUEUE_SIZE;
    }
    
    Serial.print(taskQueue.head);
    Serial.print(":");
    Serial.println(taskQueue.tail);

    mutex_exit(&taskQueueMutex);
}

void enqueueTaskWithAbortCheck(Task newTask) {
  loop0pos = 26;
  if (isTaskRunning(newTask.type) && newTask.type == TASK_LOAD_MAPIMAGE) {  // Implement this check based on your task handling
    if(newTask.loadMapImageArgs.zoomlevel == currentTask.loadMapImageArgs.zoomlevel){
      //Same zoom level. Meaning mapimage loading in progress, just be patient and dont add another task of loading image.
      Serial.println("NOT enque the task.");
      return;
    }
    
    // Abort the old task with different zoomlevel = meaning user changed zoom level while loading image.
    abortTask = true;  // Notify the running task to abort
  }
  loop0pos = 27;
  removeDuplicateTask(newTask.type);
  loop0pos = 28;
  enqueueTask(newTask);  // Enqueue the new task
}

void enqueueTask(Task task) {

  if(task.type == TASK_PLAY_MULTITONE)
      Serial.println("et");
  loop0pos = 17;
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
  }
  loop0pos = 23;
  mutex_exit(&taskQueueMutex);
  loop0pos = 24;
}

bool dequeueTask(Task* task) {
  loop0pos = 11;
  bool success = false;
  mutex_enter_blocking(&taskQueueMutex);
  loop0pos = 12;
  if (taskQueue.head != taskQueue.tail) {  // Queue not empty
    loop0pos = 13;
    *task = taskQueue.tasks[taskQueue.head];
    Serial.print("dequetask");
    Serial.print(taskQueue.head);
    Serial.print(":");
    Serial.print(taskQueue.tail);
    Serial.print(":");
    Serial.print(task->type);
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
  File myFile = SD.open("mapdata.csv");
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

void init_destinations(){
  destinations_count = 0;
  currentdestination = 0;
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("PLATHOME");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PLA_LAT, PLA_LON} };
  destinations_count++;
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("N_PILON");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PILON_NORTH_LAT, PILON_NORTH_LON} };
  destinations_count++;
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("W_PILON");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PILON_WEST_LAT, PILON_WEST_LON} };
  destinations_count++;
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("TAKESHIMA");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {TAKESHIMA_LAT, TAKESHIMA_LON} };
  destinations_count++;
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("SHINURA");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {SHINURA_LAT, SHINURA_LON} };
  destinations_count++;
}

void load_destinations(){
  File myFile = SD.open("destinations.csv");
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

void setup_sd(){
  #ifdef DISABLE_SD
    return;
  #endif

  
  //sdInitialized = SD.begin(SD_CS_PIN,SD_SCK_MHZ(50));
  sdInitialized = SD.begin(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO);


  if (!sdInitialized) {
    return;
  }else{
    sdError = false;
    // set date time callback function
    SdFile::dateTimeCallback(dateTime);
    log_sd("SD INIT");
    init_mapdata();
    load_destinations();
  }
}


volatile unsigned long loop1counter = 0;
volatile int loop1pos = 0;
volatile int loop0pos = 0;

unsigned long last_loop1counter = 0;
unsigned long time_loop1counter_updated = 0;

void setup1(void){
  mutex_init(&taskQueueMutex);
  init_destinations();
  setup_sd();

  setup_sound();
}

void loop1() {
  loop1pos = 0;
  loop1counter++;
  loop1pos = 1;
  loop_sound();
  if (dequeueTask(&currentTask)) {
    loop1pos = 2;
    switch (currentTask.type) {
      case TASK_PLAY_MULTITONE:
        playTone(currentTask.playMultiToneArgs.freq,currentTask.playMultiToneArgs.duration,currentTask.playMultiToneArgs.counter);
        break;
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
        saveCSV(
          currentTask.saveCsvArgs.latitude, currentTask.saveCsvArgs.longitude,
          currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.mtrack,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second
        );
        break;
      case TASK_LOAD_MAPIMAGE:
        loop1pos = 7;
        load_mapimage(
          currentTask.loadMapImageArgs.center_lat, currentTask.loadMapImageArgs.center_lon,
          currentTask.loadMapImageArgs.zoomlevel
        );
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


File logFile;
void log_sd(const char* text){
  loop1pos = 100;
  #ifdef DISABLE_SD
    return;
  #endif

  File logFile = SD.open("log2.txt", FILE_WRITE);
  if(!logFile){
    Serial.println("ERR LOG");
    sdError = true;
    return;
  }

  loop1pos = 101;
  // if the file opened okay, write to it:
  if (logFile) {
    loop1pos = 102;
    char logtext[100];   // array to hold the result.
    loop1pos = 103;
    sprintf(logtext,"%d:%s",(int)(millis()/1000),text);
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






void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second) {
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
  if(fileyear == 0 && year != 0){
    fileyear = year;
    filemonth = month;
    fileday = day;
    filehour = hour;
    fileminute = minute;
    filesecond = second;
  }
  loop1pos = 203;

  char csv_filename[30];
  sprintf(csv_filename, "%04d-%02d-%02d_%02d%02d.csv", fileyear, filemonth, fileday,filehour,fileminute);
  log_sdf("%d:%s",millis(),csv_filename);
  loop1pos = 204;

  File csvFile = SD.open(csv_filename, FILE_WRITE);
  loop1pos = 205;

  if (csvFile) {
    if(!headerWritten){
      csvFile.println("latitude,longitude,gs,TrueTrack,date,time");
      headerWritten = true;
    }
    loop1pos = 206;

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

    loop1pos = 207;

    // Format time as HH:MM:SS
    char time[9];
    sprintf(time, "%02d:%02d:%02d", hour, minute, second);
    csvFile.println(time);

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




double pixelsPerDegree(int zoom) {
  // Google Maps API approximation: pixels per degree at the given zoom level
  // Zoom 5 is used as the base reference in this example
  return 256 * pow(2, zoom) / 360.0;
}

double pixelsPerKM_zoom(int zoom){
  return pixelsPerDegree(zoom)/KM_PER_DEG_LAT;//px/deg / (km/deg) = px /km
}

double pixelsPerDegreeLat(int zoom,double latitude) {
  // Calculate pixels per degree for latitude
  return pixelsPerDegree(zoom) / cos(radians(latitude)); // Reference latitude
}


// Function to round a value to the nearest 5 degrees
double roundToNearestXDegrees(double x, double value) {
  return round(value / x) * x;
}

TFT_eSprite gmap_sprite = TFT_eSprite(&tft);
bool init_gmap = false;
volatile bool gmap_loaded = false;
volatile bool new_gmap_loaded = false;
char lastsprite_id[20] = "\0";
int last_start_x,last_start_y = 0;

void load_mapimage(double center_lat, double center_lon,int zoomlevel) {
  loop1pos = 300;
  DEBUG_PLN(20240828,"load mapimage begin");
  //Setting up resolution of bmp image according to zoomlevel.
  double round_degrees = 0.0;
  if(zoomlevel == 5) round_degrees = 12.0;
  else if(zoomlevel == 7) round_degrees = 3.0;
  else if(zoomlevel == 9) round_degrees = 0.8;
  else if(zoomlevel == 11) round_degrees = 0.2;
  else if(zoomlevel == 13) round_degrees = 0.05;

  loop1pos = 301;
  //Invalid zoomleel
  if(round_degrees == 0.0){
    gmap_loaded = false;
    return;
  }
  loop1pos = 302;

  double map_lat,map_lon;
  // Round latitude and longitude to the nearest 5 degrees
  map_lat = roundToNearestXDegrees(round_degrees, center_lat);
  map_lon = roundToNearestXDegrees(round_degrees, center_lon);
  
  loop1pos = 303;
  // Calculate pixel coordinates for given latitude and longitude
  int center_x = (int)(320.0 + (center_lon - map_lon) * pixelsPerDegree(zoomlevel));
  int center_y = (int)(320.0 - (center_lat - map_lat) * pixelsPerDegreeLat(zoomlevel,center_lat));
  // Calculate top-left corner of 240x240 region
  int start_x = center_x - 120;
  int start_y = center_y - 120;

  loop1pos = 304;

  char current_sprite_id[36];
  int maplat4 = round(map_lat*100);
  int maplon5 = round(map_lon*100);
  sprintf(current_sprite_id,"%2d%4d%5d%3d%3d",zoomlevel,maplat4,maplon5,start_x,start_y);

  loop1pos = 305;

  //Already loaded.
  if(strcmp(current_sprite_id,lastsprite_id) == 0 && gmap_loaded){
    loop1pos = 306;
    //if(rotation != 0){
      //tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
      //gmap_sprite.pushRotated(-rotation);
    //}
    //else
    //  gmap_sprite.pushSprite(0, 40);
    return;
  }

  loop1pos = 307;
  
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
  if(samefile && gmap_loaded){
    scrollx = (start_x-last_start_x);
    scrolly = (start_y-last_start_y);
  }

  loop1pos = 308;
  strcpy(lastsprite_id,current_sprite_id);

  char filename[36];
  sprintf(filename, "z%d/%04d_%05d_z%d.bmp", zoomlevel,maplat4,maplon5,zoomlevel);

  loop1pos = 309;

  // Open BMP file
  File bmpImage = SD.open(filename, FILE_READ);
  if (!bmpImage) {
    gmap_loaded = false;
    return;
  }
  loop1pos = 310;

  // Read the file header
  bmp_file_header_t fileHeader;
  bmpImage.read((uint8_t*)&fileHeader.signature, sizeof(fileHeader.signature));
  bmpImage.read((uint8_t*)&fileHeader.file_size, sizeof(fileHeader.file_size));
  bmpImage.read((uint8_t*)fileHeader.reserved, sizeof(fileHeader.reserved));
  bmpImage.read((uint8_t*)&fileHeader.image_offset, sizeof(fileHeader.image_offset));

  loop1pos = 311;

  // Check signature
  if (fileHeader.signature != 0x4D42) { // 'BM' in little-endian
    bmpImage.close();
    gmap_loaded = false;
    return;
  }

  loop1pos = 312;

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


  loop1pos = 313;

  if (imageHeader.image_width != 640 || imageHeader.image_height != 640 || imageHeader.bits_per_pixel != 16) {
    bmpImage.close();
    gmap_loaded = false;
    return;
  }

  loop1pos = 314;

  // Create the 240x240 sprite
  if(!gmap_sprite.created()){
    gmap_sprite.setColorDepth(16);
    gmap_sprite.createSprite(240, 240);
  }
  int tloadbmp_start = millis();

  loop1pos = 315;

  //From nowon, gmap under edit. so unload gmap.
  gmap_loaded = false;

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

  loop1pos = 318;
  DEBUG_P(20240815,"bmp load time=");
  DEBUG_P(20240815,millis()-tloadbmp_start);
  DEBUG_PLN(20240815,"ms");
  
  bmpImage.close();
  
  loop1pos = 319;
  if(abortTask){
    DEBUG_PLN(20240828,"aborted task! gmap unloaded");
    gmap_loaded = false;
    abortTask = false;
    return;
  }

  last_start_x = start_x;
  last_start_y = start_y;
  gmap_loaded = true;
  new_gmap_loaded = true;

  loop1pos = 320;

  DEBUG_PLN(20240828,"gmap_loaded! new_gmap_loaded!");
}


