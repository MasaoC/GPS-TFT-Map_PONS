#include <Adafruit_GPS.h>


void gps_setup();
void gps_loop();

void toggle_demo_biwako();
bool get_demo_biwako();

bool get_gps_fix();
bool get_gps_connection();
double get_gps_speed();
float get_gps_truetrack();
float get_gps_magtrack();
int get_gps_numsat();
float get_gps_pdop();

float get_gps_lat();
float get_gps_long();
float get_gps_altitude();
