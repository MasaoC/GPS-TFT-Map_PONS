
#include "latlon.h"
#include "display_tft.h"


LatLonManager::LatLonManager() : currentIndex(0), count(0) {}

void LatLonManager::addCoord(Coordinate position) {
  coords[currentIndex] = position;
  currentIndex = (currentIndex + 1) % 180;
  if (count < 180) {
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
  int index = (currentIndex - 1 - newest_index + 180) % 180;
  return coords[index];
}

LatLonManager latlon_manager;


// Function to convert latitude and longitude to x, y coordinates on the TFT screen
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown) {
  // Convert map center latitude and longitude to radians
  float centerLatRad = mapCenterLat * PI / 180.0;
  float centerLonRad = mapCenterLon * PI / 180.0;

  // Convert point latitude and longitude to radians
  float latRad = lat * PI / 180.0;
  float lonRad = lon * PI / 180.0;

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
  float angleRad = mapUpDirection * PI / 180.0;
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
    float angleRad = mapUpDirection * PI / 180.0;
    float rotatedX = screenX * cos(angleRad) + screenY * sin(angleRad);
    float rotatedY = -screenX * sin(angleRad) + screenY * cos(angleRad);

    // Convert map distances to degrees
    float lonDist = rotatedX / (111320.0 * cos(mapCenterLat * PI / 180.0) * mapScale);
    float latDist = rotatedY / (110540.0 * mapScale);

    // Calculate latitude and longitude
    float newLon = mapCenterLon + (lonDist * 180.0 / PI);
    float newLat = mapCenterLat + (latDist * 180.0 / PI);

    return Coordinate{newLat, newLon};
}

bool out_of_bounds(int x1,int y1,int x2,int y2){
  int newx = (x1+x2)/2;
  int newy = (y1+y2)/2;
  return (newx < 0 || newx > SCREEN_WIDTH) && (newy <0 || newy > SCREEN_HEIGHT);
}



#define EARTH_RADIUS 6371.0 // Earth's radius in kilometers
#define AVG_LATITUDE 35.5 // Average latitude of the specific range in degrees
//#define DEG_TO_RAD (PI / 180.0) // Conversion factor from degrees to radians
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