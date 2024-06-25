
#include "latlon.h"
#include "display_tft.h"


#define MAX_STROKES 10

struct Point {
  byte x;
  byte y;
};

struct Stroke {
  Point* points;
  int pointCount;
  int maxPoints;
};

// Array to store strokes
Stroke strokes[MAX_STROKES];
int strokeCount = 0;

// Function to add a new stroke with a specified maximum number of points
bool addStroke(int maxPoints) {
  if (strokeCount >= MAX_STROKES) {
    return false; // No more space for new strokes
  }
  strokes[strokeCount].points = (Point*)malloc(maxPoints * sizeof(Point));
  if (strokes[strokeCount].points == nullptr) {
    return false; // Memory allocation failed
  }
  strokes[strokeCount].pointCount = 0;
  strokes[strokeCount].maxPoints = maxPoints;
  strokeCount++;
  return true;
}

// Function to add a point to the last stroke
bool addPointToStroke(byte x, byte y) {
  if (strokeCount == 0) {
    return false; // No strokes to add points to
  }
  Stroke* stroke = &strokes[strokeCount - 1];
  if (stroke->pointCount >= stroke->maxPoints) {
    return false; // No more space for new points in this stroke
  }
  stroke->points[stroke->pointCount].x = x;
  stroke->points[stroke->pointCount].y = y;
  stroke->pointCount++;
  return true;
}

// Function to remove the last stroke
bool removeStroke() {
  if (strokeCount == 0) {
    return false; // No strokes to remove
  }
  strokeCount--;
  free(strokes[strokeCount].points); // Free allocated memory
  strokes[strokeCount].points = nullptr;
  strokes[strokeCount].pointCount = 0;
  strokes[strokeCount].maxPoints = 0;
  return true;
}

// Function to print strokes for debugging purposes
void printStrokes() {
  for (int i = 0; i < strokeCount; i++) {
    Serial.print("Stroke ");
    Serial.print(i);
    Serial.println(":");
    for (int j = 0; j < strokes[i].pointCount; j++) {
      Serial.print("(");
      Serial.print(strokes[i].points[j].x);
      Serial.print(", ");
      Serial.print(strokes[i].points[j].y);
      Serial.print(") ");
    }
    Serial.println();
  }
}


// Function to convert latitude and longitude to x, y coordinates on the TFT screen
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection) {
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
    (SCREEN_HEIGHT / 2) - (int)rotatedY + MAP_SHIFT_DOWN}; // Y is inverted on the screen
}

// Function to convert x, y coordinates on the TFT screen to latitude and longitude
cord_map xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection) {
    // Translate screen coordinates to map coordinates
    float screenX = x - (SCREEN_WIDTH / 2);
    float screenY = (SCREEN_HEIGHT / 2) - y + MAP_SHIFT_DOWN;

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

    return cord_map{newLat, newLon};
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



bool check_maybe_inside_draw(cord_map mapcenter, float checklat, float checklon, float scale){
  double dist = fastDistance(mapcenter.lat,mapcenter.lon, checklat, checklon);
  float radius_scrn = 146.6f*scale;
  return dist < radius_scrn;
}