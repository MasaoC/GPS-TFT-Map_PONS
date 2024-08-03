#include <TinyGPS++.h>
#include "ublox_gps.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "mysd.h"
#include "display_tft.h"

// Create a UBLOX instance
TinyGPSPlus gps;


double stored_longitude, stored_latitude, stored_truetrack, stored_altitude, stored_fixtype, stored_gs;
int stored_numsats;

bool gps_connection = false;
bool demo_biwako = false;

SatelliteData satellites[32];  // Array to hold data for up to 32 satellites



// Send UBX command to change baud rate to 38400
byte setBaud38400[] = {
  0xB5, 0x62, // UBX header
  0x06, 0x00, // CFG-PRT
  0x14, 0x00, // Length
  0x01,       // Port ID (1 for UART1)
  0x00,       // Reserved
  0x00, 0x00, 0x00, 0x00, // txReady (ignored)
  0xD0, 0x08, 0x00, 0x00, // Mode (0x000008D0)
  0x80, 0x96, 0x00, 0x00, // Baud rate (38400)
  0x07, 0x00, // InProtoMask (0x07: UBX+NMEA+RTCM)
  0x03, 0x00, // OutProtoMask (0x03: UBX+NMEA)
  0x00, 0x00, // Reserved
  0x13, 0xA0  // Checksum (to be calculated)
};

// Calculate UBX checksum (provided for completeness, not used here)
void calcChecksum(byte *ubxMsg, byte len, byte &ckA, byte &ckB) {
  ckA = 0;
  ckB = 0;
  for (int i = 2; i < len - 2; i++) {
    ckA += ubxMsg[i];
    ckB += ckA;
  }
}



void sendUBX(byte *ubxMsg, byte len,bool slow = false) {
  for (int i = 0; i < len; i++) {
    GPS_SERIAL.write(ubxMsg[i]);
    if(slow)
      delay(5);
  }
}




void disableGSV() {
  byte configNMEA[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // Class: CFG, ID: NMEA
    0x24, 0x00, // Length: 36 bytes
    0x00, 0x00, // Reserved
    0x00, 0x00, // Version
    0x00, 0x00, // Reserved
    0x00, 0x00, // Mask for NMEA output (default)
    0x00, 0x00, // Reserved
    0x00, 0x00, // Reserved
    0x00, 0x00, // Reserved
    0x00, 0x00, // Checksum (to be calculated)
  };

  // Set the output mask to disable GSV messages
  configNMEA[10] = 0xFF; // All messages disabled
  configNMEA[11] = 0xFF; // All messages disabled

  // Enable specific messages (bit positions as needed)
  // For example: 0x01 for GNGGA, 0x02 for GNGRMC
  // Example: Enable only GNGGA and GNGRMC (assumes bits 0 and 1)
  configNMEA[10] = 0x03; // Enable GNGGA and GNGRMC

  // Calculate and update checksum
  byte ckA, ckB;
  calcChecksum(configNMEA, sizeof(configNMEA), ckA, ckB);
  configNMEA[sizeof(configNMEA) - 2] = ckA;
  configNMEA[sizeof(configNMEA) - 1] = ckB;

  // Send the UBX command
  sendUBX(configNMEA, sizeof(configNMEA));
}


// RMC/GGAを受信した判定。(57600bpsだと通常9ms間隔,9600だと74ms以下となるので、80msでもOK。)
#define TIME_NMEA_GROUP 500 //不具合の可能性があり、一時的に500msにセット。


void gps_getposition_mode() {
  Serial.println("POS MODE");

  
}


void gps_constellation_mode() {
  Serial.println("CONST MODE");

}


void gps_setup() {
  GPS_SERIAL.begin(9600);
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);// Initialize GNSS

  // Configure GPS baud rate
  const unsigned char UBLOX_INIT_38400[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x83,0x90,};
  delay (1500);
  for (int i = 0; i < sizeof(UBLOX_INIT_38400); i++) {
    GPS_SERIAL.write(UBLOX_INIT_38400[i]);
  }
  delay (1500);
  GPS_SERIAL.end();
  delay(200);
  GPS_SERIAL.begin(38400);

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
unsigned long last_gps_save = 0;
unsigned long last_gps_time = 0;
bool draw_allowed = false;
// ここでは、Hardware Serialの受信を行う。この受信とTFTの描画が同時に発生すると、バッファーオーバーフローでデータ受信が失敗する恐れがある。
// そのため画面描画する際に true　を返す。
// M10Qの初期設定のデータ量では、最初の座標取得からメッセージを全て受信するまで38400bpsでおよそ270msである。そのため、500ms経過したら描画許可を出すこととする。(ので、データ受信完了してから画面描画は500ms遅延する。)

bool gps_loop(bool constellation_mode) {

  if(GPS_SERIAL.available() > 10){
    Serial.print("!!!!!B");
    Serial.println(GPS_SERIAL.available());
  }

  if(millis()-last_gps_time > 500 && !draw_allowed){
    draw_allowed = true;
    return true;
  }
  
  while (GPS_SERIAL.available() > 0) {
    //Serial.println(millis()-last_gps_time);
    //Serial.println((char)GPS_SERIAL.peek());
    gps.encode(GPS_SERIAL.read());
    gps_connection = true;

    if (gps.location.isUpdated()) {
      draw_allowed = false;
      last_gps_time = millis();
      Serial.print(F("Location: "));
      stored_latitude = gps.location.lat();
      stored_longitude = gps.location.lng();
      Serial.print(stored_latitude, 6);
      Serial.print(F(", "));
      Serial.println(stored_longitude, 6);
      bool all_valid = true;
      if (gps.location.isValid()) {
        // Check if the location fix is valid
        if (gps.satellites.isValid()) {
          stored_fixtype = 2;
        } else {
          stored_fixtype = 1;
        }
      } else {
        all_valid = false;
        stored_fixtype = 0;
      }
      int year,month,day,hour,minute,second;

      // Print date
      if (gps.date.isValid()) {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();
        Serial.print("Date: ");
        Serial.print(year);
        Serial.print("-");
        Serial.print(month);
        Serial.print("-");
        Serial.print(day);
        Serial.println();
      } else {
        Serial.println("Date: Not Available");
        all_valid = false;
      }
      
      // Print time
      if (gps.time.isValid()) {
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        Serial.print("Time: ");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.print(second);
        Serial.println();
      } else {
        Serial.println("Time: Not Available");
        all_valid = false;
      }
      if(all_valid && millis() - last_gps_save > 1000){
        saveCSV(stored_latitude, stored_longitude, stored_gs, stored_truetrack, year, month, day, hour, minute, second);
        last_gps_save = millis();
      }
    }

    if (gps.altitude.isUpdated()) {
      Serial.print(F("Altitude: "));
      stored_altitude = gps.altitude.meters();
      Serial.println(stored_altitude);
    }

    if (gps.speed.isUpdated()) {
      Serial.print(F("Speed: "));
      stored_gs = gps.speed.mps();
      Serial.print(stored_gs);  // Speed in meters per second
      Serial.println(F(" mps"));
    }

    if (gps.course.isUpdated()) {
      Serial.print(F("Course: "));
      stored_truetrack = gps.course.deg();
      Serial.println(stored_truetrack);
    }

    if (gps.satellites.isUpdated()) {
      Serial.print(F("Satellites: "));
      stored_numsats = gps.satellites.value();
      Serial.println(stored_numsats);
      if(millis() - last_gps_time > 1000)
        return true;
    }
  }
  return false;
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

double get_gps_long() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LON - timeelapsed / 1600.0 / 1000.0 + 0.025;
  }
#ifdef DEBUG_GPS_SIM_SHINURA
    return SHINURA_LON;
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
  if (demo_biwako) {
    return 2 + 2 * sin(millis() / 1500.0);
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
  if (demo_biwako) {
    return 280 + (8.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
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
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown) {
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
    (SCREEN_HEIGHT / 2) - (int)rotatedY + mapshiftdown}; // Y is inverted on the screen
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
#define AVG_LATITUDE 35.5 // Average latitude of the specific range in degrees
#define COS_AVG_LATITUDE cos(AVG_LATITUDE * DEG_TO_RAD) // Cosine of the average latitude in radians

// Precompute meters per degree for the specific latitude
const double METERS_PER_DEG_LAT = 110540.0; // Approximate meters per degree of latitude
const double METERS_PER_DEG_LON = 111320.0 * COS_AVG_LATITUDE; // Approximate meters per degree of longitude

// Function to calculate the distance between two points (latitude and longitude) using an optimized formula
double fastDistance(float lat1, float lon1, float lat2, float lon2) {
    // Calculate differences in latitude and longitude in degrees
    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;

    // Convert degree differences to meters
    double dLatMeters = dLat * METERS_PER_DEG_LAT;
    double dLonMeters = dLon * METERS_PER_DEG_LON;

    // Use Pythagorean theorem to calculate the distance
    double distance = sqrt(dLatMeters*dLatMeters + dLonMeters*dLonMeters) / 1000.0; // Convert to kilometers

    return distance;
}


bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2){
  return (abs(lat1-lat2) < latdif) && (abs(lon1-lon2) < londif);
}


bool check_maybe_inside_draw(Coordinate mapcenter, float checklat, float checklon, float scale){
  double dist = fastDistance(mapcenter.latitude,mapcenter.longitude, checklat, checklon);
  float radius_scrn = 146.6f*scale;
  return dist < radius_scrn;
}