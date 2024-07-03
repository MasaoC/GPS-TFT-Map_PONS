#include "gps_functions.h"
#include "navdata.h"
#include "settings.h"


#ifdef XIAO_ESP32S3
#include <HardwareSerial.h>
HardwareSerial MySerial0(0);
Adafruit_GPS GPS(&MySerial0);
#else
Adafruit_GPS GPS(&Serial1);
#endif

bool gps_connection = false;
bool demo_biwako = false;


void gps_setup() {
#ifdef XIAO_ESP32S3
  // Configure MySerial0 on pins TX=D6 and RX=D7 (-1, -1 means use the default)
  MySerial0.begin(9600, SERIAL_8N1, -1, -1);
#else
  Serial1.begin(9600);
#endif
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);  // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
}

void toggle_demo_biwako(){
  demo_biwako = !demo_biwako;
}

bool get_demo_biwako(){
  return demo_biwako;
}


void gps_loop() {
  // Read data from the GPS module
  char c = GPS.read();
  //Serial.print(c);
  // If a new NMEA sentence is available
  if (GPS.newNMEAreceived()) {
    gps_connection = true;
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
}
float get_gps_lat() {
  if(demo_biwako){
    int timeelapsed = millis()%200000;
    return PLA_LAT+timeelapsed/16000.0/1000.0-0.002;
  }
  #ifdef DEBUG_GPS_SIM_SHINURA2BIWA
    return PLA_LAT +GPS.latitudeDegrees- SHINURA_LAT;
  #endif
  #ifdef DEBUG_GPS_SIM_OSAKA2BIWA
    return PLA_LAT+GPS.latitudeDegrees-OSAKA_LAT;
  #endif 

  #ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
    return OSAKA_LAT+GPS.latitudeDegrees-SHINURA_LAT;
  #endif 
  Serial.println(GPS.latitude);
  Serial.println(GPS.latitude_fixed);
  return GPS.latitudeDegrees;
}

float get_gps_long() {
  if(demo_biwako){
    int timeelapsed = millis()%200000;
    return PLA_LON-timeelapsed/1600.0/1000.0+0.025;
  }
  #ifdef DEBUG_GPS_SIM_SHINURA2BIWA
    return PLA_LON +GPS.longitudeDegrees- SHINURA_LON;
  #endif
  #ifdef DEBUG_GPS_SIM_OSAKA2BIWA
    return PLA_LON+GPS.longitudeDegrees-OSAKA_LON;
  #endif 

  #ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
    return OSAKA_LON+GPS.longitudeDegrees-SHINURA_LON;
  #endif 

  return GPS.longitudeDegrees;
}
double get_gps_kts() {
  if(demo_biwako){
    return 20+5*sin(millis()/1500.0);
  }
  return GPS.speed;
}

double get_gps_mps(){
  if(demo_biwako){
    return get_gps_kts()*0.514444;
  }
  return GPS.speed*0.514444;
}


bool get_gps_connection(){
  return gps_connection;
}
bool get_gps_fix(){
  return GPS.fix;
}

float get_gps_altitude() {
  return GPS.altitude;// + GPS.geoidheight;
}


float get_gps_truetrack() {
  if(demo_biwako){
    return 280+(9.1+sin(millis()/2100.0))*sin(millis()/3000.0);
  }
  return GPS.angle;
}


float get_gps_magtrack() {
  float temp = get_gps_truetrack()+8.0;
  if(temp > 360.0){
    temp -= 360.0;
  }
  return temp;
}

int get_gps_numsat() {
  return GPS.satellites;
}


float get_gps_pdop(){
  return GPS.PDOP;
}
/*
    if (isnan(GPS.speed)) {
      display.print("N/A");
    } else {
      // Convert speed from knots to m/s (1 knot = 0.514444 m/s) and display with one decimal place
      float speed_m_s = GPS.speed * 0.514444;
      display.print(speed_m_s, 1);
      display.print("m/s");
    }

    // Set a larger text size
    display.setTextSize(5);
    // Display Track Made Good
    display.setCursor(24, 24);
    if (isnan(GPS.angle)) {
      display.print("N/A");
    } else {
      char buf[4];
      sprintf( buf, "%03d", (int)GPS.angle );
      display.print(buf); // Display as an integer
    }

    // Display the results
    display.display();
  }
}
*/