
#include <SD.h>
File myFile;
File csvFile;

bool sdInitialized = false;
bool sdError = false;
#define SD_CS_PIN 1
#define LOGFILE_NAME "log.txt"

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


void log_sd(char* text){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(LOGFILE_NAME, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    myFile.println(text);
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening log");
  }
}

void read_sd(){
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = SD.open("test.txt");

  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void saveCSV(float latitude, float longitude, int year, int month, int day, int hour, int minute, int second) {
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
    return;
  }
  //Run only once.
  if(fileyear == 0){
    fileyear = year + 2000;
    filemonth = month;
    fileday = day;
    filehour = hour;
    fileminute = minute;
    filesecond = second;
  }
  char csv_filename[20];
  sprintf(csv_filename, "%04d-%02d-%02d_%02d%02d.csv", fileyear, filemonth, fileday,filehour,fileminute);
  csvFile = SD.open(csv_filename, FILE_WRITE);
  if (csvFile) {
    if(!headerWritten){
      csvFile.println("latitude,longitude,date,time");
      headerWritten = true;
    }
    csvFile.print(latitude, 6);
    csvFile.print(",");
    csvFile.print(longitude, 6);
    csvFile.print(",");

    // Format date as YYYY-MM-DD
    char date[11];
    sprintf(date, "%04d-%02d-%02d", year+2000, month, day);
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
 Serial.println("year is");
 Serial.println(fileyear);
 *date = FAT_DATE(fileyear-1980, filemonth, fileday);
 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(filehour, fileminute, filesecond);
}

void setup_sd(){
  Serial.print("Initializing SD card...");

  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin();
  
  // set date time callback function
  SdFile::dateTimeCallback(dateTime);
  
  sdInitialized = SD.begin(SD_CS_PIN);

  if (!sdInitialized) {
    Serial.println("SD initialization failed!");
    return;
  }else{
    Serial.println("SD initialization done.");
  }
}