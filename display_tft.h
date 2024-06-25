#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
//#include <SPI.h>

#include "navdata.h"

#define TFT_RST        D0
#define TFT_DC         D1
#define TFT_BL      D9

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 160


#define MAP_SHIFT_DOWN 60   //80+60=140 is centerY.
#define PX_PER_KM(SCALE) (17.3596112311*SCALE)//(1789+140)/60/1852=0.01735961123 pixel/m/scale

#ifndef DISPLAY_TFT_DEFINED
  #define DISPLAY_TFT_DEFINED

  struct cord_tft{
    int x;
    int y;

    bool isOutsideTft(){
      return x < 0 || x > SCREEN_WIDTH || y < 0 || y > SCREEN_HEIGHT;
    }
  };

  enum stroke_group{
    STRK_PILONLINE,STRK_MAP1,STRK_SEALAND
  };
  

  extern  Adafruit_ST7735 tft;
  extern bool fresh_display;

#endif

//https://rgbcolorpicker.com/565
#define ST77XX_GRAY 0x7bcf
#define ST77XX_BRIGHTGRAY 0xc618

void show_gpsinfo();
void setup_tft();

void draw_bankwarning();
void draw_degpersecond(float degpersecond);
void drawShinura(bool redraw, float center_lat,float center_lon,float scale,float up);
void drawBiwako(bool redraw, float center_lat,float center_lon,float scale,float up);
void drawOsaka(bool redraw, float center_lat,float center_lon,float scale,float up);
bool draw_circle_km(float scale, float km);
void draw_km_circle(float scale);
void startup_demo_tft();
void drawmap(stroke_group id, float mapUpDirection, float center_lat, float center_lon,float mapScale, const mapdata* mp,uint16_t color);
void fill_sea_land(float mapcenter_lat, float mapcenter_lon,float scale, float upward);

void draw_pilon_line(float mapcenter_lat, float mapcenter_lon,float scale, float upward);
