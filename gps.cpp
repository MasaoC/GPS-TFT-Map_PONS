// Handle GNSS modules. Currently optimized for LC86GPAMD.
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
#define PAIR_SET_38400 "$PAIR864,0,0,38400*23"
//#define PQTM_OFF "$PQTMCFGMSGRATE,W,PQTMANTENNASTATUS,0,2*39"


double stored_longitude, stored_latitude, stored_truetrack, stored_altitude, stored_fixtype, stored_gs;
int stored_numsats;
bool gps_connection = false;
bool demo_biwako = false;

SatelliteData satellites[32];  // Array to hold data for up to 32 satellites
char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];
unsigned long last_nmea_time[MAX_LAST_NMEA];
int stored_nmea_index = 0;
int readfail_counter = 0;
bool new_location_arrived = false;
bool newcourse_arrived = false;

void utcToJst(int *year, int *month, int *day, int *hour) {
  if(*month <= 0 || *month > 12){
    DEBUGW_P(20250424,"month invalid:");
    DEBUGW_PLN(20250424,*month);
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

unsigned long get_gps_nmea_time(int i){
  int index = (stored_nmea_index-1-i)%MAX_LAST_NMEA;
  if(index < 0){
    index += MAX_LAST_NMEA;
  }
  return last_nmea_time[index];
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
  DEBUG_P(20250508,"Received NMEA: ");
  DEBUG_PLN(20250508,nmea);
  #endif


  // Determine the satellite type based on the NMEA sentence identifier
  int satelliteType = SATELLITE_TYPE_UNKNOWN;
  if (strstr(nmea, "$GPGSV")) {
    satelliteType = SATELLITE_TYPE_GPS;
    //satelliteType = SATELLITE_TYPE_QZSS;//Undistinguishable from the GPGSV.
  } else if (strstr(nmea, "$GLGSV")) {
    satelliteType = SATELLITE_TYPE_GLONASS;
  } else if (strstr(nmea, "$GAGSV")) {
    satelliteType = SATELLITE_TYPE_GALILEO;
  } else if (strstr(nmea, "$GBGSV")) {//$BDGSV
    satelliteType = SATELLITE_TYPE_BEIDOU;
  }


  #ifdef DEBUG_NMEA
  // Print the satellite type for debugging
  DEBUG_P(20250508,"Satellite Type: ");
  DEBUG_PLN(20250508,satelliteType);
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
    DEBUG_P(20250508,"PRN: "); DEBUG_P(20250508,prn);
    DEBUG_P(20250508,", Elevation: "); DEBUG_P(20250508,elevation);
    DEBUG_P(20250508,", Azimuth: "); DEBUG_P(20250508,azimuth);
    DEBUG_P(20250508,", SNR: "); DEBUG_P(20250508,snr);
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
          if(193 <= prn && prn  <= 199)
            satellites[j].satelliteType = SATELLITE_TYPE_QZSS;
          satellites[j].lastReceived = millis();
          break;
        }
      }
    } else {
      DEBUGW_P(20250508,"Invalid PRN parsed");
      DEBUGW_P(20250508,"PRN: ");
      DEBUGW_PLN(20250508,prn);
    }
  }
}



//Due compability
void gps_getposition_mode() {
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
  DEBUG_P(20250429,"GPS SETUP:");
  DEBUG_PLN(20250429,setupcounter);
  
  readfail_counter = 0;
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);// Initialize GNSS

  //初回SETUP
  if(setupcounter == 1){

    #ifdef QUECTEL_GPS
      GPS_SERIAL.setFIFOSize(1024);//LC86GPAMD Bufferサイズ、128では不足するケースあり。
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
      DEBUG_PLN(20250508,"115200 to 38400 LC86G try ");
      GPS_SERIAL.setFIFOSize(1024);
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
    last_nmea_time[i] = millis();
  }

  //効果なし 調査中
  //GPS_SERIAL.println(PQTM_OFF);

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

unsigned long last_demo_gpsupdate = 0;
double demo_biwako_lat = PLA_LAT;
double demo_biwako_lon = PLA_LON;
double demo_biwako_mps = 0;
double demo_biwako_truetrack = 280;

void add_latlon_track(float lat,float lon){
  int tracklog_interval = constrain(50000/(1.0+stored_gs/2.0), 1000, 15000);//約50mおきに一回記録するような計算となる。
  if(get_demo_biwako()){
    tracklog_interval = 900;
  }
  if(millis() - last_latlon_manager > tracklog_interval){
    latlon_manager.addCoord({ lat, lon});
    last_latlon_manager = millis();
  }
}

bool gps_new_location_arrived(){
  if(get_demo_biwako()){
    if(millis() > last_demo_gpsupdate + 1000){
      int biwa_spd = 10;
      demo_biwako_lat += 0.00005*biwa_spd*cos(radians(get_gps_truetrack()));
      if(calculateDistanceKm(demo_biwako_lat,demo_biwako_lon,PLA_LAT,PLA_LON) > 15){
        demo_biwako_lat = PLA_LAT;
        demo_biwako_lon = PLA_LON;
        latlon_manager.reset();
      }
      demo_biwako_lon += 0.00005*biwa_spd*sin(radians(get_gps_truetrack()));
      last_demo_gpsupdate = millis();
      new_location_arrived = true;
      demo_biwako_mps = 7 + sin(millis() / 1500.0);
      
      int basetrack = 280;
      if(destination_mode == DMODE_AUTO10K && auto10k_status == AUTO10K_INTO)
        basetrack = 100;

      
      int target_angle = basetrack+(20 + 10*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);

      int demo_steer_angle = target_angle - demo_biwako_truetrack;
      if(demo_steer_angle < -180){
        demo_steer_angle += 360;
      }else if(demo_steer_angle > 180){
        demo_steer_angle -= 360;
      }
      demo_biwako_truetrack += demo_steer_angle*0.4*(max(0,0.7+sin(millis() / 10000.0)));//basetrack + (10 + 5*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);
      if(demo_biwako_truetrack > 360)
        demo_biwako_truetrack -= 360;
      else if(demo_biwako_truetrack < 0)
        demo_biwako_truetrack += 360;

      newcourse_arrived = true;
      add_latlon_track(demo_biwako_lat, demo_biwako_lon);
    }
  }
  return new_location_arrived;
}

void set_new_location_off(){
  new_location_arrived = false;
}

// GNSSモジュールからのHardware Serialの受信を行う。
void gps_loop(int id) {
  if(GPS_SERIAL.available() > 256){
    DEBUGW_P(20250424,"ID=");
    DEBUGW_P(20250424,id);
    DEBUGW_P(20250424," Caution, remaining FIFO buffer. avail=");
    DEBUGW_PLN(20250424,GPS_SERIAL.available());
  }

  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    if(GPS_SERIAL.overflow()){
      DEBUGW_P(20250424,"!!WARNING!! Buffer Overflow ");
      DEBUGW_PLN(20250424,GPS_SERIAL.available());
    }
    if((int)c >= 128){
      readfail_counter++;
      if(readfail_counter > 10){
        DEBUGW_P(20250424,"Read Failed 10 times, non ascii char:");
        DEBUGW_P(20250424,(int)c);
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
        nmea_buffer1[index_buffer1-1] = '\0';
        strcpy(last_nmea[stored_nmea_index],nmea_buffer1);
        last_nmea_time[stored_nmea_index] = millis();
        stored_nmea_index = (stored_nmea_index+1)%MAX_LAST_NMEA;
        #ifdef DEBUG_NMEA
        DEBUG_PLN(20250508,nmea_buffer1);
        #endif
        if(strstr(nmea_buffer1, "GSV")){
          parseGSV(nmea_buffer1);
        }
      }
      index_buffer1 = 0;
    }
  }

    
  if (gps.location.isUpdated()) {
    last_gps_time = millis();
    if(stored_latitude != gps.location.lat() || stored_longitude != gps.location.lng()){
      if(!get_demo_biwako())
        new_location_arrived = true;
    }
    stored_latitude = gps.location.lat();
    stored_longitude = gps.location.lng();
    

    #ifdef DEBUG_NMEA
    DEBUG_P(20250508,F("Location: "));
    DEBUG_P(20250508,stored_latitude, 6);
    DEBUG_P(20250508,F(", "));
    DEBUG_PLN(20250508,stored_longitude, 6);
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
      DEBUG_PLN(20250508,"Location: Not Available");
      all_valid = false;
      stored_fixtype = 0;
    }
    

    // Print date
    if (!gps.date.isValid()) {
      DEBUG_PLN(20250508,"Date: Not Available");
      all_valid = false;
    }
    
    // Print time
    if (!gps.time.isValid()) {
      DEBUG_PLN(20250508,"Time: Not Available");
      all_valid = false;
    }

    if(all_valid && ! get_demo_biwako()){
      add_latlon_track(get_gps_lat(),get_gps_lon());
    }

    if(all_valid && millis() - last_gps_save_time > 1000){
      int year = gps.date.year();
      int month = gps.date.month();
      int day = gps.date.day();
      int hour = gps.time.hour();
      utcToJst(&year,&month,&day,&hour);
      enqueueTask(createSaveCsvTask(stored_latitude, stored_longitude, stored_gs, stored_truetrack, year, month, day, hour, gps.time.minute(), gps.time.second()));
      last_gps_save_time = millis();
    }
  }


  if (gps.altitude.isUpdated()) {
    #ifdef DEBUG_NMEA
    DEBUG_P(20250508,F("Altitude: "));
    DEBUG_PLN(20250508,stored_altitude);
    #endif
    stored_altitude = gps.altitude.meters();
  }

  if (gps.speed.isUpdated()) {
    stored_gs = gps.speed.mps();
    #ifdef DEBUG_NMEA
    DEBUG_P(20250508,F("Speed: "));
    DEBUG_P(20250508,stored_gs);  // Speed in meters per second
    DEBUG_PLN(20250508,F(" mps"));
    #endif
  }

  if (gps.course.isUpdated()) {
    #ifdef DEBUG_NMEA
    DEBUG_P(20250508,F("Course: "));
    DEBUG_PLN(20250508,stored_truetrack);
    #endif
    if(!get_demo_biwako())
      newcourse_arrived = true;
    stored_truetrack = gps.course.deg();
    if(stored_truetrack < 0 || stored_truetrack > 360){
      DEBUGW_P(20250508,"ERROR:MT");
      DEBUGW_PLN(20250508,stored_truetrack);
      stored_truetrack = 0;
    }
  }

  if (gps.satellites.isUpdated()) {
    // Remove satellites not received for 60 seconds
    removeStaleSatellites();
    stored_numsats = gps.satellites.value();
    #ifdef DEBUG_NMEA
    DEBUG_P(20250508,F("Satellites: "));
    DEBUG_PLN(20250508,stored_numsats);
    #endif
  }
}






double get_gps_lat() {
  if (demo_biwako) {
    return demo_biwako_lat;
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
  return PLA_LAT + stored_latitude - SHINURA_LAT+0.02;
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
    return demo_biwako_lon;
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
  return PLA_LON + stored_longitude - SHINURA_LON-0.03;
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
  if (demo_biwako) {
    return demo_biwako_mps;
  }
  return stored_gs;
}


bool get_gps_connection() {
  return gps_connection;
}
bool get_gps_fix() {
  if(demo_biwako){
    return get_gps_numsat() != 0;
  }
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
    return demo_biwako_truetrack;
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
  if(get_demo_biwako()){
    return (int)(20.0*sin(millis()/5000))+20;
  }
  return stored_numsats;
}

