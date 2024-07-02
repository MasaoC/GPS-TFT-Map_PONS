
#include "display_tft.h"

struct cord_map{
  float lat;
  float lon;
};



cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection, int mapshiftdown);