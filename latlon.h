
#include "display_tft.h"

struct cord_map{
  float lat;
  float lon;
};

bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);

cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection, int mapshiftdown);