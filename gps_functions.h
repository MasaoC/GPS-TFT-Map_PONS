#include <Adafruit_GPS.h>


void gps_setup();
void gps_loop();


bool get_gps_fix();
double get_gps_speed();
float get_gps_truetrack();
int get_gps_numsat();
float get_gps_pdop();

float get_gps_lat();
float get_gps_long();
float get_gps_altitude();
float get_gps_magvar();
