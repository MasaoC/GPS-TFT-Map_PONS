
void gps_setup();
void gps_loop(bool redraw,bool constellation_mode);

void toggle_demo_biwako();
bool get_demo_biwako();


void gps_getposition_mode();
void gps_constellation_mode();

bool get_gps_fix();
bool get_gps_connection();
double get_gps_kts();
double get_gps_mps();
double get_gps_truetrack();
double get_gps_magtrack();
int get_gps_numsat();
double get_gps_pdop();

double get_gps_lat();
double get_gps_long();
double get_gps_altitude();


#ifndef GPS_FUNCT
#define GPS_FUNCT
  // Define satellite types
  #define SATELLITE_TYPE_GPS 1
  #define SATELLITE_TYPE_GLONASS 2
  #define SATELLITE_TYPE_GALILEO 3
  #define SATELLITE_TYPE_BEIDOU 4
  #define SATELLITE_TYPE_QZSS 5
  #define SATELLITE_TYPE_UNKNOWN 0



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


#endif
