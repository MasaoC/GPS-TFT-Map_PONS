#include "settings.h"


#ifndef NAVDATA_H
#define NAVDATA_H

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
    void reset();
    Coordinate getData(int newest_index);
  };
  extern LatLonManager latlon_manager;


  // Function to calculate y-coordinate in Mercator projection
  double latitudeToMercatorY(double latitude);
  bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);





  #define ROW_FILLDATA 28
  #define COL_FILLDATA 28


  #define PLA_LAT 35.294353
  #define PLA_LON 136.25418
  #define SHINURA_LAT 35.650433188178
  #define SHINURA_LON 139.913699521520
  #define OSAKA_LAT 34.8227376
  #define OSAKA_LON 135.5213544
  #define SAPPORO_LAT 43.05989937316593
  #define SAPPORO_LON 141.37769461180662

  #define SHISHI_LAT 36.4734161//36.44641973
  #define SHISHI_LON 136.9234498//136.64713865

  #define PILON_NORTH_LAT 35.41640778478595
  #define PILON_NORTH_LON 136.1183001762145
  #define PILON_WEST_LAT 35.23295479141404
  #define PILON_WEST_LON 136.0493286559818
  #define TAKESHIMA_LAT 35.296584352454964
  #define TAKESHIMA_LON 136.1780537684742



  #define KM_PER_DEG_LAT (111.321)
  #define KM_PER_DEG_LON(LAT_DEG) (cos(LAT_DEG * DEG_TO_RAD) * 111.321)


  struct mapdata {
    int id;
    char* name;
    int size;
    double (*cords)[2]; // Pointer to an array of 2-element arrays
  };



  #define MAX_MAPDATAS 100
  extern mapdata extramaps[MAX_MAPDATAS];
  extern int current_id;
  extern int mapdata_count;

  #define MAX_DESTINATIONS 100
  extern mapdata extradestinations[MAX_DESTINATIONS];
  extern int destinations_count;
  extern int currentdestination;



  extern mapdata map_shinura;
  extern mapdata map_takeshima;
  extern mapdata map_chikubushima;
  extern mapdata map_biwako;
  extern mapdata map_okishima;

  extern mapdata map_handaioutside;
  extern mapdata map_handaihighway;
  extern mapdata map_handaihighway2;
  extern mapdata map_handaiinside1;
  extern mapdata map_handaiinside2;
  extern mapdata map_handaiinside3;
  extern mapdata map_handaiinside4;
  extern mapdata map_handaiinside5;
  extern mapdata map_handairailway;
  extern mapdata map_handaicafe;

  extern mapdata map_japan1;
  extern mapdata map_japan2;
  extern mapdata map_japan3;
  extern mapdata map_japan4;


  extern const bool filldata[ROW_FILLDATA][COL_FILLDATA];

#endif