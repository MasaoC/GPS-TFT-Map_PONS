#include <Adafruit_GFX.h>    // Core graphics library
#include "settings.h"
#include "navdata.h"


#ifdef TFT_USE_ST7789
  #include <Adafruit_ST7789.h> // Hardware-specific library for ST7735
  #define SCREEN_WIDTH 240
  #define SCREEN_HEIGHT 320
  #define TFT_BL        D9
  #define TFT_CS        -1
#endif
#ifdef TFT_USE_ST7735
  #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 160
  #define TFT_BL        D9
  #define TFT_CS        -1
#endif
#ifdef TFT_USE_ILI9341
  #include <Adafruit_ILI9341.h>
  #define SCREEN_WIDTH 240
  #define SCREEN_HEIGHT 320
  #define TFT_BL        D5    //D9 used for MISO
  #define TFT_CS        -1
  //#define TFT_MOSI      D10
  //#define TFT_CLK       D8
#endif

#define TFT_RST        D0
#define TFT_DC         D1

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
    STRK_PILONLINE,STRK_MAP1,STRK_SEALAND,STRK_OTHER
  };

  #if defined(TFT_USE_ST7789) || defined(TFT_USE_ST7735)
    #define COLOR_ORANGE ST77XX_ORANGE
    #define COLOR_BLACK ST77XX_BLACK
    #define COLOR_BLUE ST77XX_BLUE
    #define COLOR_GREEN ST77XX_GREEN
    #define COLOR_RED ST77XX_RED
    #define COLOR_YELLOW ST77XX_YELLOW
    #define COLOR_MAGENTA ST77XX_MAGENTA
    #define COLOR_WHITE ST77XX_WHITE
    #define COLOR_CYAN ST77XX_CYAN
    //https://rgbcolorpicker.com/565
    #define COLOR_GRAY 0x7bcf
    #define COLOR_BRIGHTGRAY 0xc618
    #define COLOR_LIGHT_BLUE 0x3dbf
  #endif
  #ifdef TFT_USE_ILI9341
    #define COLOR_ORANGE ILI9341_ORANGE
    #define COLOR_BLACK ILI9341_BLACK
    #define COLOR_BLUE ILI9341_BLUE
    #define COLOR_GREEN ILI9341_GREEN
    #define COLOR_RED ILI9341_RED
    #define COLOR_YELLOW ILI9341_YELLOW
    #define COLOR_WHITE ILI9341_WHITE
    #define COLOR_CYAN ILI9341_CYAN
    #define COLOR_GRAY 0x7bcf
    #define COLOR_BRIGHTGRAY 0xc618
    #define COLOR_LIGHT_BLUE 0x3dbf
    #define COLOR_MAGENTA ILI9341_PINK
  #endif

  
  #ifdef TFT_USE_ST7789
    extern  Adafruit_ST7789 tft;
  #endif
  #ifdef TFT_USE_ST7735
    extern  Adafruit_ST7735 tft;
  #endif
  #ifdef TFT_USE_ILI9341
    extern Adafruit_ILI9341 tft;
  #endif

  extern bool fresh_display;



#endif


void draw_gpsinfo();
void setup_tft();

void tft_increment_brightness();
void toggle_mode();
bool is_trackupmode();
bool is_headingupmode();
bool is_northupmode();


void draw_setting_mode(bool& redraw, int selectedLine, int cursorLine);
void draw_bankwarning();
void draw_degpersecond(float degpersecond);
void drawJapan(bool& redraw, float center_lat,float center_lon,float scale,float up);
void drawShinura(bool& redraw, float center_lat,float center_lon,float scale,float up);
void drawBiwako(bool& redraw, float center_lat,float center_lon,float scale,float up);
void drawOsaka(bool& redraw, float center_lat,float center_lon,float scale,float up);
bool draw_circle_km(float scale, float km);
void draw_km_circle(float scale);
void startup_demo_tft();
void drawmap(stroke_group id, float mapUpDirection, float center_lat, float center_lon,float mapScale, const mapdata* mp,uint16_t color);
void fill_sea_land(float mapcenter_lat, float mapcenter_lon,float scale, float upward);
void draw_headingupmode();
void draw_triangle();
void draw_pilon_takeshima_line(float mapcenter_lat, float mapcenter_lon,float scale, float upward);



