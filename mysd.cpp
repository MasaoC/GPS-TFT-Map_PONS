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
volatile bool abortTask = false;
volatile TaskType currentTaskType = TASK_NONE;
TaskQueue taskQueue;
mutex_t taskQueueMutex;

void load_mapimage(double center_lat, double center_lon,int zoomlevel);
void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second);
void log_sd(const char* text);
void log_sdf(const char* format, ...);
void dateTime(uint16_t* date, uint16_t* time);




bool isTaskRunning(int taskType) {
    return currentTaskType == taskType;
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

void removeDuplicateTask(TaskType type) {
  mutex_enter_blocking(&taskQueueMutex);
  int newTail = taskQueue.tail;
  int originalHead = taskQueue.head;

  // Iterate through the queue from tail to head
  while (newTail != originalHead) {
    // Check if the task at the current position matches the type to be removed
    if (taskQueue.tasks[newTail].type == type) {
        Serial.println("remove Dup task");
        // Shift the remaining tasks forward in the queue
        int current = newTail;
        while (current != originalHead) {
            int next = (current + 1) % TASK_QUEUE_SIZE;
            taskQueue.tasks[current] = taskQueue.tasks[next];
            current = next;
        }
        // Update the head pointer to reflect the removal
        taskQueue.head = (taskQueue.head - 1 + TASK_QUEUE_SIZE) % TASK_QUEUE_SIZE;
        originalHead = taskQueue.head; // Update the new head after the removal
    } else {
        // Move to the next task in the queue
        newTail = (newTail + 1) % TASK_QUEUE_SIZE;
    }
  }
  mutex_exit(&taskQueueMutex);
}

void enqueueTaskWithAbortCheck(Task newTask) {
  if (isTaskRunning(newTask.type)) {  // Implement this check based on your task handling
    abortTask = true;  // Notify the running task to abort
    Serial.print("abortTask = true");
  }
  removeDuplicateTask(newTask.type);
  enqueueTask(newTask);  // Enqueue the new task
}

void enqueueTask(Task task) {
  Serial.print("Enqued task:");
  Serial.println(task.type);
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
void process_csv_line(String line) {
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


void read_sd() {
  #ifdef DISABLE_SD
    return;
  #endif
  File32 myFile = SD.open("mapdata.csv");
  if (!myFile) {
    return;
  }

  while (myFile.available() && mapdata_count < MAX_MAPDATAS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      process_csv_line(line);
    }
  }
  myFile.close();
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
    read_sd();
  }
}

void setup1(void){
  mutex_init(&taskQueueMutex);
  setup_sd();
}

void loop1() {
  Task currentTask;
  while (true) {
      if (dequeueTask(&currentTask)) {
          currentTaskType = currentTask.type;
          switch (currentTask.type) {
              case TASK_INIT_SD:
                  setup_sd();
                  break;
              case TASK_LOG_SD:
                  log_sd(currentTask.logText);
                  break;
              case TASK_LOG_SDF:
                  log_sdf(currentTask.logSdfArgs.format, currentTask.logSdfArgs.buffer);
                  break;
              case TASK_SAVE_CSV:
                  saveCSV(
                      currentTask.saveCsvArgs.latitude, currentTask.saveCsvArgs.longitude,
                      currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.mtrack,
                      currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
                      currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
                      currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second
                  );
                  break;
              case TASK_LOAD_MAPIMAGE:
                  load_mapimage(
                      currentTask.loadMapImageArgs.center_lat, currentTask.loadMapImageArgs.center_lon,
                      currentTask.loadMapImageArgs.zoomlevel
                  );
                  break;
          }
          currentTaskType = TASK_NONE;
      } else {
          // No tasks, optionally sleep or yield
          tight_loop_contents();
      }
  }
}

void dateTime(uint16_t* date, uint16_t* time) {
 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(fileyear, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}






void core1Task(){
 while (true) {
      // Check if Core 0 needs access
      if (core0NeedsAccess) {
          // Release the mutex if held
          if (mutex_try_enter(&sdMutex, NULL)) {
              mutex_exit(&sdMutex);
          }
          // Wait until Core 0 no longer needs access
          while (core0NeedsAccess) {
              tight_loop_contents();  // Optionally, yield control or sleep
          }
      }
      
      // Core 1's long process, periodically checking the flag
      if (mutex_enter_timeout_ms(&sdMutex, 10)) {  // Try to acquire mutex with a timeout
          // Critical section: access shared resources
          // ...

          mutex_exit(&sdMutex);  // Release the mutex
      }

      // Perform non-critical work (outside mutex-protected area)
      // ...
  }
}




bool good_sd(){
  return sdInitialized && !sdError;
}



void log_sd(const char* text){
  #ifdef DISABLE_SD
    return;
  #endif

  File32 myFile = SD.open(LOGFILE_NAME, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    char logtext[100];   // array to hold the result.
    sprintf(logtext,"%d:%s",millis(),text);
    myFile.println(logtext);
    // close the file:
    myFile.close();
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
      setup_sd();
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
  DEBUG_PLN(20240828,"load mapimage begin");
  //Setting up resolution of bmp image according to zoomlevel.
  double round_degrees = 0.0;
  if(zoomlevel == 5) round_degrees = 12.0;
  else if(zoomlevel == 7) round_degrees = 3.0;
  else if(zoomlevel == 9) round_degrees = 0.8;
  else if(zoomlevel == 11) round_degrees = 0.2;
  else if(zoomlevel == 13) round_degrees = 0.05;

  //Invalid zoomleel
  if(round_degrees == 0.0){
    gmap_loaded = false;
    return;
  }

  double map_lat,map_lon;
  // Round latitude and longitude to the nearest 5 degrees
  map_lat = roundToNearestXDegrees(round_degrees, center_lat);
  map_lon = roundToNearestXDegrees(round_degrees, center_lon);
  
  // Calculate pixel coordinates for given latitude and longitude
  int center_x = (int)(320.0 + (center_lon - map_lon) * pixelsPerDegree(zoomlevel));
  int center_y = (int)(320.0 - (center_lat - map_lat) * pixelsPerDegreeLat(zoomlevel,center_lat));
  // Calculate top-left corner of 240x240 region
  int start_x = center_x - 120;
  int start_y = center_y - 120;

  char current_sprite_id[36];
  int maplat4 = round(map_lat*100);
  int maplon5 = round(map_lon*100);
  sprintf(current_sprite_id,"%2d%4d%5d%3d%3d",zoomlevel,maplat4,maplon5,start_x,start_y);

  //Already loaded.
  if(strcmp(current_sprite_id,lastsprite_id) == 0 && gmap_loaded){
    //if(rotation != 0){
      //tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
      //gmap_sprite.pushRotated(-rotation);
    //}
    //else
    //  gmap_sprite.pushSprite(0, 40);
    Serial.println("gmap No change");
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
  if(samefile){;
    scrollx = (start_x-last_start_x);
    scrolly = (start_y-last_start_y);
  }

  strcpy(lastsprite_id,current_sprite_id);

  char filename[36];
  sprintf(filename, "z%d/%04d_%05d_z%d.bmp", zoomlevel,maplat4,maplon5,zoomlevel);


  // Open BMP file
  File32 bmpImage = SD.open(filename, FILE_READ);
  if (!bmpImage) {
    gmap_loaded = false;
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
    gmap_loaded = false;
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
    gmap_loaded = false;
    return;
  }

  // Create the 240x240 sprite
  if(!gmap_sprite.created()){
    gmap_sprite.setColorDepth(16);
    gmap_sprite.createSprite(240, 240);
  }
  int tloadbmp_start = millis();

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
    DEBUG_P(20240815,"bmp load time=");
    DEBUG_P(20240815,millis()-tloadbmp_start);
    DEBUG_PLN(20240815,"ms");
  }

  //if(rotation != 0){
    //tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
    //gmap_sprite.setPivot(120, 120);
    //gmap_sprite.pushRotated(-rotation);
  //}
  //else
    //gmap_sprite.pushSprite(0, 40);


  // Close the BMP file
  bmpImage.close();
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

  DEBUG_PLN(20240828,"gmap_loaded! new_gmap_loaded!");
}


