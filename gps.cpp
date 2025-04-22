#include <Arduino.h>

#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "mysd.h"
#include "display_tft.h"
#include "gps.h"

// Create a UBLOX instance
TinyGPSPlus gps;

//mediatek
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"

//quectel
#define PAIR_SET_38400 "$PAIR864,0,0,38400*23"
#define PAIR_DISABLE_GSV "$PAIR062,3,0*3D"
#define PAIR_ENABLE_GSV "$PAIR062,3,1*3C"
#define PAIR_DISABLE_GSA "$PAIR062,2,0*3C"
#define PAIR_ENABLE_GSA "$PAIR062,2,1*3D"



double stored_longitude, stored_latitude, stored_truetrack, stored_altitude, stored_fixtype, stored_gs;
int stored_numsats;
bool gps_connection = false;
bool demo_biwako = false;

SatelliteData satellites[32];  // Array to hold data for up to 32 satellites
char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];
int stored_nmea_index = 0;

int readfail_counter = 0;

void utcToJst(int *year, int *month, int *day, int *hour) {
  if(*month <= 0 || *month > 12){
    Serial.print("month invalid:");
    Serial.println(*month);
    enqueueTask(createLogSdfTask("month invalid:%d",*month));
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

char* get_gps_nmea(int i){
  int index = (stored_nmea_index-1-i)%MAX_LAST_NMEA;
  if(index < 0){
    index += MAX_LAST_NMEA;
  }
  return last_nmea[index];
}


void removeStaleSatellites() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < 32; i++) {
    if (satellites[i].PRN != 0 && (currentMillis - satellites[i].lastReceived > 30000)) {
      satellites[i].PRN = 0;  // Mark as stale and remove
    }
  }
}

void parseGSV(char *nmea) {
  // Print the received NMEA sentence for debugging
  #ifdef DEBUG_NMEA
  Serial.print("Received NMEA: ");
  Serial.println(nmea);
  #endif


  // Determine the satellite type based on the NMEA sentence identifier
  int satelliteType = SATELLITE_TYPE_UNKNOWN;
  if (strstr(nmea, "$GPGSV")) {
    satelliteType = SATELLITE_TYPE_GPS;
  } else if (strstr(nmea, "$GLGSV")) {
    satelliteType = SATELLITE_TYPE_GLONASS;
  } else if (strstr(nmea, "$GAGSV")) {
    satelliteType = SATELLITE_TYPE_GALILEO;
  } else if (strstr(nmea, "$GBGSV")) {//$BDGSV
    satelliteType = SATELLITE_TYPE_BEIDOU;
  } else if (strstr(nmea, "$GQGSV")) {//GQGSVQZGSV
    satelliteType = SATELLITE_TYPE_QZSS;
  }


  #ifdef DEBUG_NMEA
  // Print the satellite type for debugging
  Serial.print("Satellite Type: ");
  Serial.println(satelliteType);
  #endif

  
  // Example NMEA GSV sentence: $GPGSV,4,4,14,194,,,,195,,,*7D
  char *p = nmea;

  // Skip past the initial part of the sentence
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;

  for (int i = 0; i < 4; i++) {
    if (*p == '*' || *p == '\0') break; // End of sentence or no more data

    // Read satellite PRN number
    int prn = atoi(p);
    p = strchr(p, ','); if (!p) break; p++;

    // Read elevation
    int elevation = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    // Read azimuth
    int azimuth = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    // Read SNR (Signal to Noise Ratio)
    int snr = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    #ifdef DEBUG_NMEA
    // Debugging: Print parsed values
    Serial.print("PRN: "); Serial.print(prn);
    Serial.print(", Elevation: "); Serial.print(elevation);
    Serial.print(", Azimuth: "); Serial.print(azimuth);
    Serial.print(", SNR: "); Serial.println(snr);
    #endif

    // Validate parsed values and update satellite data
    if (prn > 0 && prn < 200) {
      for (int j = 0; j < 32; j++) {
        if (satellites[j].PRN == prn || satellites[j].PRN == 0) {
          satellites[j].PRN = prn;
          satellites[j].elevation = (elevation >= 0 && elevation <= 90) ? elevation : satellites[j].elevation;
          satellites[j].azimuth = (azimuth >= 0 && azimuth < 360) ? azimuth : satellites[j].azimuth;
          satellites[j].SNR = (snr >= 0) ? snr : satellites[j].SNR;
          satellites[j].satelliteType = satelliteType;
          satellites[j].lastReceived = millis();
          break;
        }
      }
    } else {
      Serial.println("Invalid PRN parsed");
      Serial.print("PRN: "); Serial.print(prn);
    }
  }
}



//Due compability
void gps_getposition_mode() {
  Serial.println("POS MODE");

  #ifdef QUECTEL_GPS
    GPS_SERIAL.println(PAIR_DISABLE_GSV);
    delay(3);
    GPS_SERIAL.println(PAIR_DISABLE_GSA);
  #endif
  #ifdef MEDIATEK_GPS
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  #endif
}
//Due compability
void gps_constellation_mode() {
  Serial.println("CONST MODE");
  #ifdef QUECTEL_GPS
    GPS_SERIAL.println(PAIR_ENABLE_GSV);
    delay(3);
    GPS_SERIAL.println(PAIR_ENABLE_GSA);
  #endif
  #ifdef MEDIATEK_GPS
    GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_GSVONLY);
    GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);
  #endif
}

int setupcounter = 1;

// Try to establish connection with GPS module.  Either ublox or mediatek.
void gps_setup() {
  Serial.println("GPS SETUP");
  Serial.print("setup");
  Serial.println(setupcounter);
  
  readfail_counter = 0;
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);// Initialize GNSS

  //初回SETUP
  if(setupcounter == 1){

    #ifdef QUECTEL_GPS
      GPS_SERIAL.setFIFOSize(256);//LC86GPAMD Bufferサイズ、128では不足するケースあり。
      GPS_SERIAL.begin(115200);
    #elif MEADIATEK_GPS
      GPS_SERIAL.println(PMTK_ENABLE_SBAS);
      gps_getposition_mode();
      delay(100);//（Do not delete without care.)
      GPS_SERIAL.println("$PMTK251,38400*27");
      delay(100);//（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//（Do not delete without care.)
      GPS_SERIAL.begin(38400);
    #elif UBLOX_GPS
      // Configure GPS baud rate
      const unsigned char UBLOX_INIT_38400[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x83,0x90,};
      delay (50);// なぜか必要（Do not delete without care.)
      for (int i = 0; i < sizeof(UBLOX_INIT_38400); i++) {
        GPS_SERIAL.write(UBLOX_INIT_38400[i]);
      }
      delay (100);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.begin(38400);
    #else
      GPS_SERIAL.begin(9600);
    #endif
    
  }else{
    //2nd try
    if(setupcounter%3 == 2){
      Serial.println("115200 to 38400 LC86G try ");
      GPS_SERIAL.setFIFOSize(128);
      GPS_SERIAL.begin(115200);
      GPS_SERIAL.println(PAIR_SET_38400);//Need restart of LC86G module.
      delay (100);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.begin(38400);
    }else if(setupcounter%3 == 1){
      //4th try
      GPS_SERIAL.begin(38400);
    }else{
      //3rd try
      GPS_SERIAL.begin(9600);
    }
  }

  for(int i = 0; i < MAX_LAST_NMEA;i++){
    last_nmea[i][0] = 0;
  }

  setupcounter++;
}

void toggle_demo_biwako() {
  demo_biwako = !demo_biwako;
}

bool get_demo_biwako() {
  return demo_biwako;
}

unsigned long last_latlon_manager = 0;
unsigned long last_gps_save_time = 0;
unsigned long last_gps_time = 0;// last position update time.
unsigned long time_lastnmea = 0;//last nmea time

const int NMEA_BUFFER_SIZE = 256;
int index_buffer1 = 0;
char nmea_buffer1[NMEA_BUFFER_SIZE];


TinyGPSDate get_gpsdate(){
  return gps.date;
}
TinyGPSTime get_gpstime(){
  return gps.time;
}


// return true if all the gps msgs received. Hardware Serialの受信を行う。
// UART受信とTFTの描画が同時に発生すると、バッファーオーバーフローでデータ受信が失敗する恐れがある。
// そのため画面描画する際に true　を返す。
bool drawallow_once = false;
bool gps_loop() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    if(GPS_SERIAL.overflow()){
      Serial.print("!!WARNING!! Buffer Overflow ");
      Serial.println(GPS_SERIAL.available());
      Serial.println(c);
      Serial.println((int)c);
    }
    if((int)c >= 128){
      readfail_counter++;
      if(readfail_counter > 10){
        Serial.print("Read Failed 10 times, non ascii char:");
        Serial.println((int)c);
        gps_setup();
      }
      continue;
    }
    gps.encode(c);
    gps_connection = true;

    if(c == '$')
      index_buffer1 = 0;
    
    nmea_buffer1[index_buffer1++] = c;
    if(index_buffer1 >= (NMEA_BUFFER_SIZE-1) || c == '\n'){
      if(index_buffer1 >= 2){
        time_lastnmea = millis();
        drawallow_once = true;
        nmea_buffer1[index_buffer1-1] = '\0';
        strcpy(last_nmea[stored_nmea_index],nmea_buffer1);
        stored_nmea_index = (stored_nmea_index+1)%MAX_LAST_NMEA;
        #ifdef DEBUG_NMEA
        Serial.println(nmea_buffer1);
        #endif
        // Usually last message of ublox is GNGLL for ublox M10Q.  
        // This will be the key message to start drawing TFT (So we dont miss hardware serial RX with overflow while long wait for TFT update).
        //if(strstr(nmea_buffer1, "$GNGLL")){
        //  return true;
        //}
        if(strstr(nmea_buffer1, "GSV")){
          parseGSV(nmea_buffer1);
        }
      }
      index_buffer1 = 0;
    }
  }

    
  if (gps.location.isUpdated()) {
    last_gps_time = millis();
    stored_latitude = gps.location.lat();
    stored_longitude = gps.location.lng();

    #ifdef DEBUG_NMEA
    Serial.print(F("Location: "));
    Serial.print(stored_latitude, 6);
    Serial.print(F(", "));
    Serial.println(stored_longitude, 6);
    #endif
    bool all_valid = true;
    if (gps.location.isValid()) {
      // Check if the location fix is valid
      if (gps.satellites.isValid()) {
        stored_fixtype = 2;
      } else {
        stored_fixtype = 1;
      }
    } else {
      Serial.println("Location: Not Available");
      all_valid = false;
      stored_fixtype = 0;
    }
    

    // Print date
    if (!gps.date.isValid()) {
      Serial.println("Date: Not Available");
      all_valid = false;
    }
    
    // Print time
    if (!gps.time.isValid()) {
      Serial.println("Time: Not Available");
      all_valid = false;
    }
    int tracklog_interval = constrain(50000/(1.0+stored_gs/2.0), 1000, 15000);//約50mおきに一回記録するような計算となる。
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    utcToJst(&year,&month,&day,&hour);
    if(all_valid && millis() - last_latlon_manager > tracklog_interval){
      latlon_manager.addCoord({ (float)stored_latitude, (float)stored_longitude });
      last_latlon_manager = millis();
    }
    if(all_valid && millis() - last_gps_save_time > 1000){
      enqueueTask(createSaveCsvTask(stored_latitude, stored_longitude, stored_gs, stored_truetrack, year, month, day, hour, gps.time.minute(), gps.time.second()));
      last_gps_save_time = millis();
    }
  }


  //simulated.
  #ifndef RELEASE_GPS
    int tracklog_interval = constrain(50000/(1+gps.speed.mps()), 1000, 15000);//約50mおきに一回記録するような計算となる。
    if(millis() - last_latlon_manager > tracklog_interval){
      latlon_manager.addCoord({ (float)get_gps_lat(), (float)get_gps_lon() });
      last_latlon_manager = millis();
    }
  #endif

  if (gps.altitude.isUpdated()) {
    #ifdef DEBUG_NMEA
    Serial.print(F("Altitude: "));
    Serial.println(stored_altitude);
    #endif
    stored_altitude = gps.altitude.meters();
  }

  if (gps.speed.isUpdated()) {
    stored_gs = gps.speed.mps();
    #ifdef DEBUG_NMEA
    Serial.print(F("Speed: "));
    Serial.print(stored_gs);  // Speed in meters per second
    Serial.println(F(" mps"));
    #endif
  }

  if (gps.course.isUpdated()) {
    #ifdef DEBUG_NMEA
    Serial.print(F("Course: "));
    Serial.println(stored_truetrack);
    #endif
    stored_truetrack = gps.course.deg();
    if(stored_truetrack < 0 || stored_truetrack > 360){
      Serial.print("ERROR:MT");
      Serial.println(stored_truetrack);
      stored_truetrack = 0;
    }
  }

  if (gps.satellites.isUpdated()) {
    // Remove satellites not received for 60 seconds
    removeStaleSatellites();
    stored_numsats = gps.satellites.value();
    #ifdef DEBUG_NMEA
    Serial.print(F("Satellites: "));
    Serial.println(stored_numsats);
    #endif
  }


  //with 38400, usually nmea sentence will be sent within 10ms.
  if(millis()-time_lastnmea > 20 && drawallow_once){
    drawallow_once = false;
    return true;
  }
  return false;
}






double get_gps_lat() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LAT + timeelapsed / 16000.0 / 1000.0 - 0.002;
  }
#ifdef DEBUG_GPS_SIM_BIWAKO
  return PLA_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA
  int timeelapsed = ((int)(millis()/1000)) % 20000;
  return SHINURA_LAT + timeelapsed / 1600.0;
#endif
#ifdef DEBUG_GPS_SIM_SAPPORO
  return SAPPORO_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHISHI
  return SHISHI_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LAT + stored_latitude - SHINURA_LAT;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LAT + stored_latitude - OSAKA_LAT;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LAT + stored_latitude - SHINURA_LAT;
#endif

  return stored_latitude;
}

double get_gps_lon() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LON - timeelapsed / 1600.0 / 1000.0 + 0.025;
  }

#ifdef DEBUG_GPS_SIM_BIWAKO
  return PLA_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA
  int timeelapsed = ((int)(millis()/1000)) % 20000;
  return SHINURA_LON + timeelapsed / 16000.0;
#endif
#ifdef DEBUG_GPS_SIM_SAPPORO
    return SAPPORO_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHISHI
    return SHISHI_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LON + stored_longitude - SHINURA_LON;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LON + stored_longitude - OSAKA_LON;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LON + stored_longitude - SHINURA_LON;
#endif

  return stored_longitude;
}


double get_gps_mps() {
  #ifndef RELEASE_GPS
    return 8 + 4 * sin(millis() / 1500.0);
  #endif
  if (demo_biwako) {
    return 4 + 2 * sin(millis() / 1500.0);
  }
  return stored_gs;
}


bool get_gps_connection() {
  return gps_connection;
}
bool get_gps_fix() {
  if(stored_fixtype >= 1){
    return true;
  }
  return false;
}

double get_gps_altitude() {
  return stored_altitude;
}


double get_gps_truetrack() {
  #ifdef DEBUG_GPS_SIM_SHINURA
    return 40 + (38.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  #endif
  if (demo_biwako) {
    return 280 + (8.5 + 2*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+30*sin(millis() / 10000.0);
  }
  return stored_truetrack;     // Heading in degrees;
}


double get_gps_magtrack() {
  double temp = get_gps_truetrack() + 8.0;
  if (temp > 360.0) {
    temp -= 360.0;
  }
  return temp;
}

int get_gps_numsat() {
  return stored_numsats;
}

