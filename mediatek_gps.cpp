
#ifdef MEDIATEK_GPS

#include <Adafruit_GPS.h>
#include "mediatek_gps.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "mysd.h"
#include "display_tft.h"

Adafruit_GPS GPS(&GPS_SERIAL);

bool gps_connection = false;
bool demo_biwako = false;



// Define a buffer to store incoming data
char nmeaBuffer[256];
int bufferIndex = 0;

SatelliteData satellites[32];  // Array to hold data for up to 32 satellites


#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"


// RMC/GGAを受信した判定。(57600bpsだと通常9ms間隔,9600だと74ms以下となるので、80msでもOK。)
#define TIME_NMEA_GROUP 500 //不具合の可能性があり、一時的に500msにセット。

void gps_getposition_mode() {
  Serial.println("POS MODE");
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
}


void gps_constellation_mode() {
  Serial.println("CONST MODE");
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_GSVONLY);
  GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);
}


char* get_gps_nmea(){
  return GPS.lastNMEA();
}

void gps_setup() {
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);

  GPS.begin(9600);
  
  //Somehow 57600　NG on RP2040_ZERO
  // For RP2040_PICO, unable change bps for some unknown reason. Keep commented out for now.
  //GPS.sendCommand("$PMTK251,38400*27");
  //GPS.begin(38400);

  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PGCMD_ANTENNA);
  gps_getposition_mode();

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
  } else if (strstr(nmea, "$GQGSV")) {//QZGSV
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
unsigned long last_gga = 0;
unsigned long last_rmc = 0;
bool gps_loop(bool constellation_mode) {
  // Read data from the GPS module
  char c = GPS.read();
  bool GPS_updated = false;

  if (GPS.newNMEAreceived()) {
    gps_connection = true;

    Serial.println(GPS.lastNMEA());
    if(strstr(GPS.lastNMEA(), "GGA"))
      last_gga = millis();
    else if(strstr(GPS.lastNMEA(), "RMC"))
      last_rmc = millis();
    int timedif = last_rmc-last_gga;
    Serial.println(timedif);
    if(abs(timedif) < TIME_NMEA_GROUP)
      GPS_updated = true;

    if (constellation_mode && strstr(GPS.lastNMEA(), "GSV")) {
      parseGSV(GPS.lastNMEA());
      //GPS.parse(GPS.lastNMEA());
    } else if (GPS.parse(GPS.lastNMEA())) {
      // GNGGA と GNRMC が毎秒くるので、片方のみ=1秒おきに保存、
      if (GPS.fix && strstr(GPS.lastNMEA(), "$GNRMC")) {
        if(GPS.year < 24){// Somehow, error data has GPS.year of 01 or 2002 ish.  By eliminating GPS data with year < 2024, this will avoid displaying errorneous track.
          Serial.println("ERROR GPS DATA");
          log_sd("ERROR GPS DATA");
          log_sd(GPS.lastNMEA());
        }else{
          // save to SD card
          saveCSV(GPS.latitudeDegrees, GPS.longitudeDegrees,GPS.speed,get_gps_truetrack() , GPS.year+2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
          int tracklog_interval = constrain(50000/(1+get_gps_mps()), 1000, 15000);//約50mおきに一回記録するような計算となる。
          if (millis() - last_latlon_manager > tracklog_interval) {
            latlon_manager.addCoord({ GPS.latitudeDegrees, GPS.longitudeDegrees });
            last_latlon_manager = millis();
          }
        }
      }
    } else {
      Serial.print("Failed Parsing: ");
      Serial.println(GPS.lastNMEA());
    }
  }

  // Remove satellites not received for 60 seconds
  removeStaleSatellites();

  return GPS_updated;
}


double get_gps_lat() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LAT + timeelapsed / 16000.0 / 1000.0 - 0.002;
  }

#ifdef DEBUG_GPS_SIM_SHINURA
    return SHINURA_LAT;
#endif
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
#ifdef DEBUG_GPS_SIM_SHINURA
    return SHINURA_LON;
#endif
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
#ifndef RELEASE_GPS
  return true;
#endif
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





LatLonManager::LatLonManager() : currentIndex(0), count(0) {}

void LatLonManager::addCoord(Coordinate position) {
  coords[currentIndex] = position;
  currentIndex = (currentIndex + 1) % MAX_TRACK_CORDS;
  if (count < MAX_TRACK_CORDS) {
    count++;
  }
}

int LatLonManager::getCount() {
  return count;
}

void LatLonManager::printData() {
  for (int i = 0; i < count; i++) {
    Serial.print("Coordinate ");
    Serial.print(i + 1);
    Serial.print(": Latitude = ");
    Serial.print(coords[i].latitude, 6);
    Serial.print(", Longitude = ");
    Serial.println(coords[i].longitude, 6);
  }
}

Coordinate LatLonManager::getData(int newest_index) {
  if (newest_index >= count) {
    Serial.println("Invalid index");
    return {0, 0};
  }
  int index = (currentIndex - 1 - newest_index + MAX_TRACK_CORDS) % MAX_TRACK_CORDS;
  return coords[index];
}

LatLonManager latlon_manager;


// Function to convert latitude and longitude to x, y coordinates on the TFT screen
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection) {
  // Convert map center latitude and longitude to radians
  float centerLatRad = mapCenterLat * DEG_TO_RAD;
  float centerLonRad = mapCenterLon * DEG_TO_RAD;

  // Convert point latitude and longitude to radians
  float latRad = lat * DEG_TO_RAD;
  float lonRad = lon * DEG_TO_RAD;

  // Calculate the differences
  float dLat = latRad - centerLatRad;
  float dLon = lonRad - centerLonRad;

  // Calculate x and y distances
  float xDist = dLon * cos(centerLatRad) * 111320.0; // Approx distance per degree longitude in meters
  float yDist = dLat * 110540.0; // Approx distance per degree latitude in meters

  // Apply scale factor
  xDist *= mapScale;
  yDist *= mapScale;

  // Apply rotation for map up direction
  float angleRad = mapUpDirection * DEG_TO_RAD;
  float rotatedX = xDist * cos(angleRad) - yDist * sin(angleRad);
  float rotatedY = xDist * sin(angleRad) + yDist * cos(angleRad);

  // Translate to screen coordinates
  return cord_tft{(SCREEN_WIDTH / 2) + (int)rotatedX,
    (SCREEN_HEIGHT / 2) - (int)rotatedY}; // Y is inverted on the screen
}

// Function to convert x, y coordinates on the TFT screen to latitude and longitude
Coordinate xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown) {
    // Translate screen coordinates to map coordinates
    float screenX = x - (SCREEN_WIDTH / 2);
    float screenY = (SCREEN_HEIGHT / 2) - y + mapshiftdown;

    // Apply rotation inverse for map up direction
    float angleRad = mapUpDirection * DEG_TO_RAD;
    float rotatedX = screenX * cos(angleRad) + screenY * sin(angleRad);
    float rotatedY = -screenX * sin(angleRad) + screenY * cos(angleRad);

    // Convert map distances to degrees
    float lonDist = rotatedX / (111320.0 * cos(mapCenterLat * DEG_TO_RAD) * mapScale);
    float latDist = rotatedY / (110540.0 * mapScale);

    // Calculate latitude and longitude
    float newLon = mapCenterLon + (lonDist * RAD_TO_DEG);
    float newLat = mapCenterLat + (latDist * RAD_TO_DEG);

    return Coordinate{newLat, newLon};
}

bool out_of_bounds(int x1,int y1,int x2,int y2){
  int newx = (x1+x2)/2;
  int newy = (y1+y2)/2;
  return (newx < 0 || newx > SCREEN_WIDTH) && (newy <0 || newy > SCREEN_HEIGHT);
}



#define EARTH_RADIUS 6371.0 // Earth's radius in kilometers


// Function to calculate the distance between two points (latitude and longitude) using an optimized formula
double fastDistance(float lat1, float lon1, float lat2, float lon2) {
    // Calculate differences in latitude and longitude in degrees
    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;

    // Convert degree differences to meters
    double dLatMeters = dLat * KM_PER_DEG_LAT;
    double dLonMeters = dLon * KM_PER_DEG_LON(lat1);

    // Use Pythagorean theorem to calculate the distance
    return sqrt(dLatMeters*dLatMeters + dLonMeters*dLonMeters); 
}


bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2){
  return (abs(lat1-lat2) < latdif) && (abs(lon1-lon2) < londif);
}


bool check_maybe_inside_draw(Coordinate mapcenter, float checklat, float checklon, float scale){
  double dist = fastDistance(mapcenter.latitude,mapcenter.longitude, checklat, checklon);
  float radius_scrn = 146.6f*scale;
  return dist < radius_scrn;
}


#endif