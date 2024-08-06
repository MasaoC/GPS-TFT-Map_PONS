#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "settings.h"
#include "navdata.h"




#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
#ifdef PNP_BL
  #define BRIGHTNESS(brt) (255-brt)
#endif
#ifdef NPN_BL
  #define BRIGHTNESS(brt) (brt)
#endif

#define BL_PWM_FRQ 1000   //1000Hz


#define PX_PER_KM(SCALE) (17.3596112311*SCALE)//(1789+140)/60/1852=0.01735961123 pixel/m/scale

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
    STRK_PILONLINE,STRK_MAP1,STRK_SEALAND,STRK_OTHER,STRK_TRACK,STRK_COMPASS
  };  

  enum text_id{
    SETTING_TITLE,SETTING_BRIGHTNESS,SETTING_DEMOBIWA,SETTING_UPWARD,SETTING_GPSDETAIL,SETTING_MAPDETAIL,SETTING_EXIT,
    ND_MPS,ND_MPS_LGND,ND_SATS,ND_MT,ND_DIST_PLAT,ND_LAT,ND_LON,ND_DEGPERSEC_VAL,ND_DEGPERSEC_TEX,ND_BATTERY,
    ND_SEARCHING,ND_GPSDOTS,ND_GPSCOND
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

  extern TFT_eSPI tft;
  extern bool fresh_display;
  extern int screen_brightness;
#endif


void draw_sdinfo();
void draw_gpsinfo();
void setup_tft();
void clean_display();
void clean_map();

void tft_change_brightness(int increment);
void toggle_mode();
bool is_trackupmode();
bool is_northupmode();


void redraw_compass(float up,int col,int bgcolor);
void draw_nomapdata();
void draw_gpsdetail(bool redraw,int page);
void draw_maplist_mode(bool redraw,int maplist_page);
void draw_setting_mode(bool redraw, int selectedLine, int cursorLine);
void draw_bankwarning();
void draw_degpersecond(float degpersecond);

void draw_track(double center_lat,double center_lon,float scale,float up);
void draw_ExtraMaps(double center_lat,double center_lon,float scale,float up);
void draw_Japan(double center_lat,double center_lon,float scale,float up);
void draw_Shinura(double center_lat,double center_lon,float scale,float up);
void draw_Biwako(double center_lat,double center_lon,float scale,float up);
void draw_Osaka(double center_lat,double center_lon,float scale,float up);
bool draw_circle_km(float scale, float km);
void draw_km_circle(float scale);
void startup_demo_tft();
void draw_map(stroke_group id, float mapUpDirection, double center_lat, double center_lon,float mapScale, const mapdata* mp,uint16_t color);
void fill_sea_land(double mapcenter_lat, double mapcenter_lon,float scale, float upward);
void erase_triangle();
void draw_triangle();
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon,float scale, float upward);


int mod( int x, int y );

