#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include <SD.h>


File myFile;
File csvFile;

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


bool good_sd(){
  return sdInitialized && !sdError;
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


void log_sd(const char* text){

  #ifdef DISABLE_SD
    return;
  #endif

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(LOGFILE_NAME, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    char logtext[100];   // array to hold the result.
    sprintf(logtext,"%d:%s",millis(),text);
    myFile.println(logtext);
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening log");
  }
}


void process_line(String line) {
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
      Serial.println("Error: Invalid format.");
      delete[] cords;
      return;
    }
    cords[i][0] = line.substring(index, commaIndex).toDouble();
    index = commaIndex + 1;
    commaIndex = line.indexOf(',', index);
    if (commaIndex == -1 && i < size - 1) {
      Serial.println("Error: Invalid format.");
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

  File myFile = SD.open("mapdata.csv");
  if (!myFile) {
    Serial.println("error opening mapdata.csv");
    return;
  }

  Serial.println("mapdata.csv:");

  while (myFile.available() && mapdata_count < MAX_MAPDATAS) {
    String line = myFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      process_line(line);
    }
  }

  myFile.close();

  /*. FOR DEBUG
  Serial.println("extramap");
  // For debugging: print loaded map data
  for (int i = 0; i < mapdata_count; i++) {
    Serial.print("ID: "); Serial.println(extramaps[i].id);
    Serial.print("Name: "); Serial.println(extramaps[i].name);
    Serial.print("Size: "); Serial.println(extramaps[i].size);
    for (int j = 0; j < extramaps[i].size; j++) {
      Serial.print("Lat: "); Serial.print(extramaps[i].cords[j][0]);
      Serial.print(", Lon: "); Serial.println(extramaps[i].cords[j][1]);
    }
  }
  */
}


void utcToJst(int *year, int *month, int *day, int *hour) {
    if(*month <= 0 || *month > 12){
      Serial.print("month invalid:");
      Serial.println(*month);
      log_sdf("month invalid:%d",month);
      return;
    }
    // Add 9 hours to convert UTC to JST
    *hour += 9;
    // Handle overflow of hours (24-hour format)
    if (*hour >= 24) {
        *hour -= 24;
        (*day)++;
    }
    // Handle overflow of days in each month
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    // Check for leap year
    bool isLeapYear = ((*year % 4 == 0 && *year % 100 != 0) || (*year % 400 == 0));
    if (isLeapYear) {
        daysInMonth[1] = 29;
    }
    if (*day > daysInMonth[*month - 1]) {
        *day = 1;
        (*month)++;
    }
    // Handle overflow of months
    if (*month > 12) {
        *month = 1;
        (*year)++;
    }
}

void saveCSV(float latitude, float longitude,float gs,int ttrack, int year, int month, int day, int hour, int minute, int second) {

  #ifdef DISABLE_SD
    return;
  #endif

  utcToJst(&year,&month,&day,&hour);
  if (!sdInitialized && !sdError) {
    sdInitialized = SD.begin(SD_CS_PIN);
    if (!sdInitialized) {
      Serial.println("SD Card initialization failed!");
      sdError = true;
      return;
    }
  }
  if(sdError){
    Serial.println("SD FAIL");
    if(millis()-lasttrytime_sd > 10000){
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
  Serial.println(csv_filename);
  csvFile = SD.open(csv_filename, FILE_WRITE);
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
      Serial.println("Failed to open CSV file for appending");
      sdError = true;
    }
    sdInitialized = false; // Mark SD card as not initialized for the next attempt
  }
}

void dateTime(uint16_t* date, uint16_t* time) {
 //sprintf(timestamp, "%02d:%02d:%02d %2d/%2d/%2d \n", filehour,fileminute,filesecond,filemonth,fileday,fileyear-2000);
 //Serial.println(timestamp);
 // return date using FAT_DATE macro to format fields
 Serial.print("Callback: FileYear is ");
 Serial.println(fileyear);
 *date = FAT_DATE(fileyear, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}

void setup_sd(){
  #ifdef DISABLE_SD
    return;
  #endif
  Serial.print("Initializing SD card...");

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
  
  sdInitialized = SD.begin(SD_CS_PIN);


  if (!sdInitialized) {
    Serial.println("SD initialization failed!");
    return;
  }else{
    Serial.println("SD initialization done.");
    sdError = false;
    // set date time callback function
    SdFile::dateTimeCallback(dateTime);
    log_sd("SD INIT");
    read_sd();
  }
}