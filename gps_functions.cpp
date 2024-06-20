#include "gps_functions.h"

//#define ESP32S3

#ifdef ESP32S3
  #include <HardwareSerial.h>
  HardwareSerial MySerial0(0);
  Adafruit_GPS GPS(&MySerial0);
#else
  Adafruit_GPS GPS(&Serial1);
#endif

void gps_setup() {
  #ifdef ESP32S3
  // Configure MySerial0 on pins TX=D6 and RX=D7 (-1, -1 means use the default)
  MySerial0.begin(9600, SERIAL_8N1, -1, -1);
  #else
  Serial1.begin(9600);
  #endif
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
}


void gps_loop() {
  // Read data from the GPS module
  char c = GPS.read();
  //Serial.print(c);
  // If a new NMEA sentence is available
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
}
float get_gps_lat(){
  return GPS.latitudeDegrees;
}

float get_gps_long(){
  return GPS.longitudeDegrees;

}
double get_gps_speed(){
  return GPS.speed;
}

float get_gps_altitude(){
  return GPS.altitude+GPS.geoidheight;
}


float get_gps_truetrack(){
  return GPS.angle;
}

int get_gps_numsat(){
  return GPS.satellites;
}

float get_gps_magvar(){
  return GPS.magvariation;
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