#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "settings.h"
#include "navdata.h"




#define SCREEN_WIDTH 240
#define HEADERFOOTER_HEIGHT 50
#define SCREEN_HEIGHT 320
#define BACKSCREEN_SIZE 240

// For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
#ifdef PNP_BL
  #define BRIGHTNESS(brt) (255-brt)
#endif
#ifdef NPN_BL
  #define BRIGHTNESS(brt) (brt)
#endif

#define BL_PWM_FRQ 1000   //1000Hz



#ifndef DISPLAY_TFT_DEFINED
  #define DISPLAY_TFT_DEFINED

  struct cord_tft{
    int x;
    int y;

    //xr_offset は、画面右端を狭めるオプション。これによって改行してはいけない状況での、isOutsideTftを実行可能。
    bool isOutsideTft(){
      return x < 0 || x > SCREEN_WIDTH || y < 0 || y > SCREEN_HEIGHT;
    }
  };

  enum stroke_group{
    STRK_PILONLINE,STRK_MAP1,STRK_SEALAND,STRK_OTHER,STRK_TRACK,STRK_TARGETLINE,STRK_TARGETLINE2
  };  

  enum text_id{
    SETTING_SETDESTINATION,SETTING_DESTINATIONMODE,SETTING_TITLE,SETTING_BRIGHTNESS,SETTING_DEMOBIWA,SETTING_REPLAY,SETTING_UPWARD,SETTING_GPSDETAIL,SETTING_MAPDETAIL,SETTING_VOLUME,SETTING_EXIT,
    ND_MPS,ND_MPS_LGND,ND_SATS,ND_MT,ND_DIST_PLAT,ND_DESTNAME,ND_TEMP,ND_TIME,ND_DESTMODE,ND_MC_PLAT,ND_LAT,ND_LON,ND_DEGPERSEC_VAL,ND_DEGPERSEC_TEX,ND_BATTERY,
    ND_SEARCHING,ND_GPSDOTS,ND_GPSCOND,COUNTER,SETTING_SD_DETAIL
  };

  #define COLOR_ORANGE TFT_ORANGE
  #define COLOR_WHITE TFT_WHITE
  #define COLOR_BLUE TFT_BLUE
  #define COLOR_GREEN TFT_DARKGREEN
  #define COLOR_RED TFT_RED
  #define COLOR_DARKORANGE TFT_OLIVE
  #define COLOR_MAGENTA TFT_MAGENTA
  #define COLOR_BLACK TFT_BLACK
  #define COLOR_CYAN TFT_CYAN
  //https://rgbcolorpicker.com/565
  #define COLOR_GRAY 0x7bcf
  #define COLOR_BRIGHTGRAY 0xc618
  #define COLOR_LIGHT_BLUE 0x3dbf
  #define COLOR_PINK TFT_PINK
  #define COLOR_YELLOW TFT_YELLOW

  extern TFT_eSPI tft;
  extern bool fresh_display;
  extern int screen_brightness;
  
#endif

// Function to convert x, y coordinates on the TFT screen to latitude and longitude

cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection);
Coordinate xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown);
// Function to calculate the distance between two points (latitude and longitude) using an optimized formula



void draw_headertext(double degpersecond);
void draw_loading_image();
void draw_nogmap(double scale);
bool draw_gmap(float drawupward_direction);
void draw_degpersec(double degpersecond);
void draw_map_footer();
void setup_tft();
void draw_strokes();

void draw_header();
void draw_footer();



void tft_change_brightness(int increment);
void toggle_mode();
bool is_trackupmode();
bool is_northupmode();


void draw_compass(float truetrack, uint16_t col);
void draw_nomapdata();


//mode draws
void draw_setting_mode(int selectedLine, int cursorLine);
void draw_gpsdetail(int page);
void draw_sddetail(int page);
void draw_maplist_mode(int maplist_page);


void clean_backscreen();
void push_backscreen();


void draw_flyinto(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness);
void draw_flyinto2(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness);
void draw_flyawayfrom(double dest_lat,double dest_lon, double center_lat, double center_lon, float scale, float up);
void draw_track(double center_lat,double center_lon,float scale,float up);
void draw_ExtraMaps(double center_lat,double center_lon,float scale,float up);
void draw_Japan(double center_lat,double center_lon,float scale,float up);
void draw_Shinura(double center_lat,double center_lon,float scale,float up);
void draw_Biwako(double center_lat,double center_lon,float scale,float up,bool gmap_drawed);
void draw_Osaka(double center_lat,double center_lon,float scale,float up);
bool try_draw_km_distance(float scale, float km);
void draw_km_distances(float scale);
void startup_demo_tft();
void draw_demo_biwako();
void draw_map(stroke_group id, float mapUpDirection, double center_lat, double center_lon,float mapScale, const mapdata* mp,uint16_t color);
void fill_sea_land(double mapcenter_lat, double mapcenter_lon,float scale, float upward);
void draw_triangle(int ttrack,int steer_angle);
void draw_course_warning(int steer_angle);
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon,float scale, float upward);



int mod( int x, int y );

