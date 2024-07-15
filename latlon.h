
#include "display_tft.h"


#define MAX_TRACK_CORDS 800


bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection, int mapshiftdown);

#ifndef LATLONMANAGER_H
#define LATLONMANAGER_H


struct Coordinate {
  float latitude;
  float longitude;
};



class LatLonManager {
private:
  Coordinate coords[MAX_TRACK_CORDS];
  int currentIndex;
  int count;

public:
  LatLonManager();
  void addCoord(Coordinate position);
  int getCount();
  void printData();
  Coordinate getData(int newest_index);
};

extern LatLonManager latlon_manager;

#endif // LATLONMANAGER_H