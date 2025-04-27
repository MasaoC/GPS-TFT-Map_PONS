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





  #define RADIUS_EARTH_KM 6371.0  // Earth's radius in kilometers

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


  #define PILON_NORTH_LAT 35.36437425956881
  #define PILON_NORTH_LON 136.1763690764263
  #define PILON_WEST_LAT 35.26178541248602
  #define PILON_WEST_LON 136.145446500099


/*. old
  #define PILON_NORTH_LAT 35.41640778478595
  #define PILON_NORTH_LON 136.1183001762145
  #define PILON_WEST_LAT 35.23295479141404
  #define PILON_WEST_LON 136.0493286559818
*/

  #define TAKESHIMA_LAT 35.296584352454964
  #define TAKESHIMA_LON 136.1780537684742



  #define SCALE_EXLARGE_GMAP 52.32994872   //zoom13
  #define SCALE_LARGE_GMAP 13.08248718     //zoom11
  #define SCALE_MEDIUM_GMAP 3.2706218      //zoom9
  #define SCALE_SMALL_GMAP 0.81765545      //zoom7
  #define SCALE_EXSMALL_GMAP 0.2044138625  //zoom5


  #define KM_PER_DEG_LAT (111.321)
  #define KM_PER_DEG_LON(LAT_DEG) (cos(LAT_DEG * DEG_TO_RAD) * 111.321)


  struct mapdata {
    int id;
    char* name;
    int size;
    double (*cords)[2]; // Pointer to an array of 2-element arrays
  };

  float deg2rad(float degrees);
  double rad2deg(double rad);
  extern int magc;
  extern float dest_dist;



  double calculateDistance(double lat1, double lon1, double lat2, double lon2);
  double calculateTrueCourseRad(double lat1, double lon1, double lat2, double lon2);
  void nav_update();

  #define MAX_MAPDATAS 100
  extern mapdata extramaps[MAX_MAPDATAS];
  extern int current_id;
  extern int mapdata_count;

  #define MAX_DESTINATIONS 100
  extern mapdata extradestinations[MAX_DESTINATIONS];
  extern int destinations_count;
  extern int currentdestination;

  //destination mode
  #define DMODE_FLYINTO 0
  #define DMODE_FLYAWAY 1
  #define DMODE_AUTO10K 2
  extern int destination_mode;


  extern int auto10k_status;
  #define AUTO10K_AWAY 0
  #define AUTO10K_INTO 1
  extern int auto10k_status;



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