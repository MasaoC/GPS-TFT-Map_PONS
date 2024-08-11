#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include <SD.h>
#include "display_tft.h"



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

#include <SD.h>
#include <SPI.h>

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

double pixelsPerDegreeLat(int zoom) {
  // Calculate pixels per degree for latitude
  return pixelsPerDegree(zoom) / cos(radians(37.00)); // Reference latitude
}

// Function to round a value to the nearest 5 degrees
float roundToNearestFiveDegrees(float value) {
  return round(value / 5.0) * 5.0;
}

TFT_eSprite gmap_sprite = TFT_eSprite(&tft);
bool init_gmap = false;

/*void bmp_open() {
  int starttime = millis();
  // Open BMP file
  File bmpImage = SD.open("output_rgb565_fixed.bmp", FILE_READ);
  if (!bmpImage) {
    Serial.println("Error opening BMP file");
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
    Serial.println("Not a BMP file");
    bmpImage.close();
    return;
  }
  
  // Read the image header
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


  // Print file header
  Serial.println("File Header:");
  Serial.print("Signature: ");
  Serial.println(fileHeader.signature, HEX);
  Serial.print("File Size: ");
  Serial.println(fileHeader.file_size);
  Serial.print("Reserved: ");
  Serial.print(fileHeader.reserved[0]);
  Serial.print(", ");
  Serial.println(fileHeader.reserved[1]);
  Serial.print("Image Offset: ");
  Serial.println(fileHeader.image_offset);

  // Print image header
  Serial.println("Image Header:");
  Serial.print("Header Size: ");
  Serial.println(imageHeader.header_size);
  Serial.print("Image Width: ");
  Serial.println(imageHeader.image_width);
  Serial.print("Image Height: ");
  Serial.println(imageHeader.image_height);
  Serial.print("Color Planes: ");
  Serial.println(imageHeader.color_planes);
  Serial.print("Bits Per Pixel: ");
  Serial.println(imageHeader.bits_per_pixel);
  Serial.print("Compression Method: ");
  Serial.println(imageHeader.compression_method);
  Serial.print("Image Size: ");
  Serial.println(imageHeader.image_size);
  Serial.print("Horizontal Resolution: ");
  Serial.println(imageHeader.horizontal_resolution);
  Serial.print("Vertical Resolution: ");
  Serial.println(imageHeader.vertical_resolution);
  Serial.print("Colors In Palette: ");
  Serial.println(imageHeader.colors_in_palette);
  Serial.print("Important Colors: ");
  Serial.println(imageHeader.important_colors);

  if (imageHeader.bits_per_pixel != 16) {
    Serial.println("BMP is not 16-bit!");
    bmpImage.close();
    return;
  }
  
  // Create a sprite with a reduced color depth
  uint16_t spriteWidth = imageHeader.image_width;
  uint16_t spriteHeight = imageHeader.image_height;
  
  // Set color depth to 16 bits per pixel
  gmap_sprite.setColorDepth(16);
  gmap_sprite.createSprite(spriteWidth, spriteHeight);

  // Locate the pixels
  bmpImage.seek(fileHeader.image_offset);

  // Read pixel data into the sprite
  for (int16_t y = spriteHeight - 1; y >= 0; y--) {
    for (int16_t x = 0; x < spriteWidth; x++) {
      // Read a pixel (16-bit BMP assumed)
      uint16_t color = bmpImage.read() | (bmpImage.read() << 8);
      gmap_sprite.drawPixel(x, y, color);
    }
  }

  gmap_sprite.pushSprite(0, 0);

  // Delete the sprite to free up memory
  gmap_sprite.deleteSprite();

  // Close the BMP file
  bmpImage.close();
  Serial.print(millis()-starttime);
  Serial.println("ms for sd read");
  delay(5000);
  delay(5000);
}*/



void display_region5(double center_lat, double center_lon) {
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  int starttime = millis();

  // Round latitude and longitude to the nearest 5 degrees
  double map_lat = roundToNearestFiveDegrees(center_lat);
  double map_lon = roundToNearestFiveDegrees(center_lon);
  // Calculate the file name based on the center latitude and longitude
  int zoomlevel = 7;
  
  char filename[30];
  sprintf(filename, "z%d/%04d_%05d_z%d.bmp", zoomlevel,(int)(map_lat*100),(int)(map_lon*100),zoomlevel);
  DEBUG_P(filename);

  // Open BMP file
  File bmpImage = SD.open(filename, FILE_READ);
  if (!bmpImage) {
    Serial.println("Error opening BMP file");
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
    Serial.println("Not a BMP file");
    bmpImage.close();
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
    Serial.println("BMP image dimensions or format are not as expected!");
    bmpImage.close();
    return;
  }

  // Calculate pixel coordinates for given latitude and longitude
  int center_x = (int)(320.0 + (center_lon - map_lon) * pixelsPerDegree(zoomlevel));
  int center_y = (int)(320.0 - (center_lat - map_lat) * pixelsPerDegreeLat(zoomlevel));

  // Calculate top-left corner of 240x240 region
  int start_x = center_x - 120;
  int start_y = center_y - 120;

  // Create the 240x240 sprite
  if(!gmap_sprite.created()){
    gmap_sprite.setColorDepth(16);
    gmap_sprite.createSprite(240, 240);
    gmap_sprite.fillSprite(COLOR_WHITE);
  }

  for (int y = 0; y < 240; y++) {
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


  // Push the sprite to the TFT display at (0, 40)
  gmap_sprite.pushSprite(0, 40);

  Serial.print(millis()-starttime);
  Serial.println("ms to pushsrpite");


  // Close the BMP file
  bmpImage.close();
  delay(4000);
}


