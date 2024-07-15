#include <Adafruit_GPS.h>
#include "gps_functions.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "mysd.h"
#include "latlon.h"

Adafruit_GPS GPS(&Serial2);


bool gps_connection = false;
bool demo_biwako = false;

GNGGAData gnggaData;
GNRMCData gnrmcData;
//AdaGPS GPS;


// Define a buffer to store incoming data
char nmeaBuffer[256];
int bufferIndex = 0;

SatelliteData satellites[32];  // Array to hold data for up to 32 satellites


#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"

void gps_getposition_mode() {
  Serial.println("POS MODE");
  Serial2.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Serial2.println(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
}


void gps_constellation_mode() {
  Serial.println("CONST MODE");
  Serial2.println(PMTK_SET_NMEA_OUTPUT_GSVONLY);
  Serial2.println(PMTK_SET_NMEA_UPDATE_1HZ);
}


void gps_setup() {
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(9600);

  GPS.begin(9600);
  gps_getposition_mode();
  GPS.sendCommand(PGCMD_ANTENNA);
}

void parseGSV(char *nmea) {
  // Print the received NMEA sentence for debugging
  Serial.print("Received NMEA: ");
  Serial.println(nmea);


  // Determine the satellite type based on the NMEA sentence identifier
  int satelliteType = SATELLITE_TYPE_UNKNOWN;
  if (strstr(nmea, "$GPGSV")) {
    satelliteType = SATELLITE_TYPE_GPS;
  } else if (strstr(nmea, "$GLGSV")) {
    satelliteType = SATELLITE_TYPE_GLONASS;
  } else if (strstr(nmea, "$GAGSV")) {
    satelliteType = SATELLITE_TYPE_GALILEO;
  } else if (strstr(nmea, "$BDGSV")) {
    satelliteType = SATELLITE_TYPE_BEIDOU;
  } else if (strstr(nmea, "$QZGSV")) {
    satelliteType = SATELLITE_TYPE_QZSS;
  }

  // Print the satellite type for debugging
  Serial.print("Satellite Type: ");
  Serial.println(satelliteType);

  
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

    // Debugging: Print parsed values
    Serial.print("PRN: "); Serial.print(prn);
    Serial.print(", Elevation: "); Serial.print(elevation);
    Serial.print(", Azimuth: "); Serial.print(azimuth);
    Serial.print(", SNR: "); Serial.println(snr);

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
    }
  }
}


void printGSVData() {
  Serial.println(F("Satellite Data:"));
  for (int i = 0; i < 32; i++) {
    if (satellites[i].PRN == 0) continue; // Skip empty entries

    Serial.print(F("PRN: "));
    Serial.print(satellites[i].PRN);
    Serial.print(F(", Elevation: "));
    Serial.print(satellites[i].elevation);
    Serial.print(F(", Azimuth: "));
    Serial.print(satellites[i].azimuth);
    Serial.print(F(", SNR: "));
    Serial.println(satellites[i].SNR);
  }
}


void toggle_demo_biwako() {
  demo_biwako = !demo_biwako;
}

bool get_demo_biwako() {
  return demo_biwako;
}


void removeStaleSatellites() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < 32; i++) {
    if (satellites[i].PRN != 0 && (currentMillis - satellites[i].lastReceived > 30000)) {
      satellites[i].PRN = 0;  // Mark as stale and remove
    }
  }
}


unsigned long last_latlon_manager = 0;
void gps_loop(bool constellation_mode) {

  // Read data from the GPS module
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    gps_connection = true;
    Serial.println(GPS.lastNMEA());
    if (constellation_mode && strstr(GPS.lastNMEA(), "GSV")) {
      parseGSV(GPS.lastNMEA());
      GPS.parse(GPS.lastNMEA());
    } else if (GPS.parse(GPS.lastNMEA())) {
      // GNGGA と GNRMC が毎秒くるので、片方のみ=1秒おきに保存、
      if (GPS.fix && strstr(GPS.lastNMEA(), "$GNRMC")) {
        saveCSV(GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
        int tracklog_interval = constrain(12000 - get_gps_mps() * 1000, 1000, 10000);
        //Interval from 1sec to 12sec. When mps is 0, 12sec interval.  When mps is 7, log 35m apart.
        //When mps is 5, save every 7sec and log 35m apart.  When mps is 3, save every 9 sec and 27m apart. so on...
        if (millis() - last_latlon_manager > tracklog_interval) {
          latlon_manager.addCoord({ GPS.latitudeDegrees, GPS.longitudeDegrees });
          last_latlon_manager = millis();
        }
      }
    } else {
      Serial.print("Failed Parsing: ");
      Serial.println(GPS.lastNMEA());
    }
  }

  // Remove satellites not received for 60 seconds
  removeStaleSatellites();
}


double get_gps_lat() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LAT + timeelapsed / 16000.0 / 1000.0 - 0.002;
  }
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LAT + GPS.latitudeDegrees - SHINURA_LAT;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LAT + GPS.latitudeDegrees - OSAKA_LAT;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LAT + GPS.latitudeDegrees - SHINURA_LAT;
#endif

  return GPS.latitudeDegrees;
}

double get_gps_long() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LON - timeelapsed / 1600.0 / 1000.0 + 0.025;
  }
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LON + GPS.longitudeDegrees - SHINURA_LON;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LON + GPS.longitudeDegrees - OSAKA_LON;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LON + GPS.longitudeDegrees - SHINURA_LON;
#endif

  return GPS.longitudeDegrees;
}
double get_gps_kts() {
  if (demo_biwako) {
    return 20 + 5 * sin(millis() / 1500.0);
  }
  return GPS.speed;
}

double get_gps_mps() {
  if (demo_biwako) {
    return get_gps_kts() * 0.514444;
  }
  return GPS.speed * 0.514444;
}


bool get_gps_connection() {
  return gps_connection;
}
bool get_gps_fix() {
  return GPS.fix;
}

double get_gps_altitude() {
  return GPS.altitude;  // + GPS.geoidheight;
}


double get_gps_truetrack() {
  if (demo_biwako) {
    return 280 + (8.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  }
  return GPS.angle;
}


double get_gps_magtrack() {
  double temp = get_gps_truetrack() + 8.0;
  if (temp > 360.0) {
    temp -= 360.0;
  }
  return temp;
}

int get_gps_numsat() {
  return GPS.satellites;
}


double get_gps_pdop() {
  return GPS.PDOP;
}