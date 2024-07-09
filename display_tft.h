#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "settings.h"
#include "navdata.h"


#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define TFT_BL        7
/*
#define TFT_RST        4
#define TFT_CS        5
#define TFT_DC         6
#define TFT_MOSI         3
#define TFT_SCLK         2
*/



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
    SETTING_TITLE,SETTING_BRIGHTNESS,SETTING_DEMOBIWA,SETTING_UPWARD,SETTING_GPSCONST,SETTING_EXIT,
    ND_MPS,ND_SATS,ND_MT,ND_DIST_PLAT,ND_LAT,ND_LON,ND_DEGPERSEC_VAL,ND_DEGPERSEC_TEX,ND_BATTERY,
    ND_SEARCHING,ND_GPSDOTS,ND_GPSCOND
  };

  #if defined(TFT_USE_ST7789) || defined(TFT_USE_ST7735)
    #define COLOR_ORANGE ST77XX_ORANGE
    #define COLOR_WHITE ST77XX_WHITE
    #define COLOR_BLUE ST77XX_NAVY
    #define COLOR_GREEN ST77XX_DARKGREEN
    #define COLOR_RED ST77XX_RED
    #define COLOR_MAGENTA ST77XX_YELLOW
    #define COLOR_MAGENTA ST77XX_MAGENTA
    #define COLOR_BLACK ST77XX_BLACK
    #define COLOR_CYAN ST77XX_CYAN
    //https://rgbcolorpicker.com/565
    #define COLOR_GRAY 0x7bcf
    #define COLOR_BRIGHTGRAY 0xc618
    #define COLOR_LIGHT_BLUE 0x3dbf
  #endif
  #ifdef TFT_USE_ILI9341
    #define COLOR_ORANGE ILI9341_ORANGE
    #define COLOR_WHITE ILI9341_WHITE
    #define COLOR_BLUE ILI9341_NAVY
    #define COLOR_YELLOW ILI9341_YELLOW
    #define COLOR_PURPLE ILI9341_PURPLE
    #define COLOR_PINK ILI9341_PINK
    #define COLOR_MAGENTA ILI9341_MAGENTA
    #define COLOR_BLACK ILI9341_BLACK
    #define COLOR_CYAN ILI9341_DARKCYAN
    #define COLOR_GRAY 0x7bcf
    #define COLOR_BRIGHTGRAY 0xc618
    #define COLOR_LIGHT_BLUE 0x3dbf
    #define COLOR_RED ILI9341_RED
    #define COLOR_GREEN ILI9341_DARKGREEN
  #endif

  
  extern TFT_eSPI tft;

  extern bool fresh_display;



#endif


void draw_sdinfo();
void draw_gpsinfo();
void setup_tft();
void blacken_display(bool& redraw);

void tft_increment_brightness();
void toggle_mode();
bool is_trackupmode();
bool is_northupmode();


void redraw_compass(bool redraw, float up,int col,int bgcolor);
void draw_nomapdata(bool redraw);
void draw_ConstellationDiagram(bool redraw);
void draw_setting_mode(bool redraw, int selectedLine, int cursorLine);
void draw_bankwarning();
void draw_degpersecond(float degpersecond);
void draw_Japan(bool redraw, double center_lat,double center_lon,float scale,float up);
void draw_Shinura(bool redraw, double center_lat,double center_lon,float scale,float up);
void draw_Biwako(bool redraw, double center_lat,double center_lon,float scale,float up);
void draw_Osaka(bool redraw, double center_lat,double center_lon,float scale,float up);
bool draw_circle_km(float scale, float km);
void draw_km_circle(float scale);
void startup_demo_tft();
void draw_map(stroke_group id, float mapUpDirection, double center_lat, double center_lon,float mapScale, const mapdata* mp,uint16_t color);
void fill_sea_land(double mapcenter_lat, double mapcenter_lon,float scale, float upward);
void draw_triangle();
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon,float scale, float upward);



