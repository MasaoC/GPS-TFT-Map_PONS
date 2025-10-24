
#ifndef GPS_H
  #define GPS_H
  #include <TinyGPS++.h>
  // Define satellite types
  #define SATELLITE_TYPE_GPS 1
  #define SATELLITE_TYPE_GLONASS 2
  #define SATELLITE_TYPE_GALILEO 3
  #define SATELLITE_TYPE_BEIDOU 4
  #define SATELLITE_TYPE_QZSS 5
  #define SATELLITE_TYPE_UNKNOWN 0

  #define MAX_LAST_NMEA 16
  #define NMEA_MAX_CHAR 100

  struct SatelliteData {
    int PRN = 0;
    int elevation = 0;
    int azimuth = 0;
    int SNR = 0;
    int satelliteType = SATELLITE_TYPE_UNKNOWN;
    unsigned long lastReceived = 0;
  };
  extern SatelliteData satellites[32];
  extern char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];
  extern int stored_nmea_index;
  extern unsigned long time_lastnmea;
  extern bool newcourse_arrived;

  void utcToJst(int *year, int *month, int *day, int *hour);
  void parseGSV(char *nmea);
  char* get_gps_nmea(int i);
  unsigned long get_gps_nmea_time(int i);
  void gps_setup();
  void gps_loop(int id);
  void try_enque_savecsv();
  
  bool gps_new_location_arrived();
  void set_new_location_off();

  void gps_getposition_mode();
  void gps_constellation_mode();
  bool get_gps_fix();
  bool get_gps_connection();
  int get_gps_numsat();
  double get_gps_mps();
  double get_gps_truetrack();
  double get_gps_magtrack();
  double get_gps_pdop();
  double get_gps_lat();
  double get_gps_lon();
  double get_gps_altitude();

  TinyGPSDate get_gpsdate();
  TinyGPSTime get_gpstime();

  void toggle_demo_biwako();
  bool get_demo_biwako();
  void set_demo_biwako(bool biwakomode);
  void toggleReplayMode();
  bool getReplayMode();
  void set_replaymode(bool replaymode);

#endif