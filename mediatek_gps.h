
#ifdef MEDIATEK_GPS
  #include "display_tft.h"

  void gps_setup();
  bool gps_loop(bool constellation_mode);
  void gps_getposition_mode();
  void gps_constellation_mode();
  bool get_gps_fix();
  bool get_gps_connection();
  char* get_gps_nmea();
  int get_gps_numsat();
  double get_gps_kts();
  double get_gps_mps();
  double get_gps_truetrack();
  double get_gps_magtrack();
  double get_gps_pdop();
  double get_gps_lat();
  double get_gps_long();
  double get_gps_altitude();

  void toggle_demo_biwako();
  bool get_demo_biwako();

  #ifndef GPS_FUNCT
  #define GPS_FUNCT
    // Define satellite types
    #define SATELLITE_TYPE_GPS 1
    #define SATELLITE_TYPE_GLONASS 2
    #define SATELLITE_TYPE_GALILEO 3
    #define SATELLITE_TYPE_BEIDOU 4
    #define SATELLITE_TYPE_QZSS 5
    #define SATELLITE_TYPE_UNKNOWN 0

    #define MAX_TRACK_CORDS 360


    // Structure to store parsed data from GNGGA message
    struct GNGGAData {
      char time[11];        // HHMMSS.SSS
      double latitude;
      char latDirection;
      double longitude;
      char lonDirection;
      int fixQuality;
      int numSatellites;
      double hdop;
      double altitude;
      char altitudeUnits;
      double geoidHeight;
      char geoidUnits;
    };

    // Structure to store parsed data from GNRMC message
    struct GNRMCData {
      char time[11];        // HHMMSS.SSS
      char status;
      double latitude;
      char latDirection;
      double longitude;
      char lonDirection;
      double speed;
      double course;
      char date[7];         // DDMMYY
      char mode;
    };


    struct SatelliteData {
      int PRN = 0;
      int elevation = 0;
      int azimuth = 0;
      int SNR = 0;
      int satelliteType = SATELLITE_TYPE_UNKNOWN;
      unsigned long lastReceived = 0;
    };

    struct AdaGPS{
      double latitudeDegrees;
      double longitudeDegrees;
      int satellites;
      int angle;
      double PDOP;
      double speed;
      double altitude;
      bool fix;
    };


    extern SatelliteData satellites[32];

    bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);
    cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection, int mapshiftdown);

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
#endif