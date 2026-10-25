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



#ifndef DISPLAY_TFT_DEFINED
  #define DISPLAY_TFT_DEFINED

  struct Setting {
      int id;
      std::string (*getLabel)(bool selected);
      void (*CallbackEnter)();
      void (*CallbackToggle)();
  };


  extern Setting settings[];

  enum text_id{
    SETTING_SETDESTINATION,SETTING_DESTINATIONMODE,SETTING_TITLE,SETTING_BRIGHTNESS,SETTING_DEMOBIWA,SETTING_UPWARD,SETTING_GPSDETAIL,SETTING_MAPDETAIL,SETTING_EXIT,
    SETTING_MAPSCALE,SETTING_REPLAY,SETTING_REPLAYSPEED,
    ND_MPS,ND_SATS,ND_MT,ND_DIST_PLAT,ND_DESTNAME,ND_TIME,ND_DESTMODE,ND_MC_PLAT,ND_LAT,ND_LON,ND_DEGPERSEC_VAL,ND_DEGPERSEC_TEX,ND_BATTERY,
    ND_MAXGS,ND_ACC,COUNTER,
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

  // 目的地へ「向かう（INTO）」か「離れる（AWAY）」かの文字の色（v7 と同じ）。
  // AWAY は誘導の向きを 180 度反転させるので、取り違えると矢印が真逆を指す。文字だけだと似ていて
  // 見分けにくいので色を分ける。Auto 10km と通常のモードで同じ配色にして「青なら目的地の方へ」と読める。
  // ★ 白の背景なので明るいオレンジ（COLOR_ORANGE）は使わない（ほとんど見えない）。
  #define COLOR_NAV_INTO COLOR_BLUE
  #define COLOR_NAV_AWAY COLOR_DARKORANGE

  extern TFT_eSPI tft;
  extern int screen_brightness;

#endif


void draw_loading_image();
void draw_sdinfo();
void draw_gpsinfo();
void setup_tft();
void clean_display();


void tft_change_brightness(int increment);
void toggle_mode();
bool is_trackupmode();
bool is_northupmode();
void set_upward_mode(int mode);   // 0=TRACK UP, 1=NORTH UP（SD の設定を戻すとき）
int get_upward_mode();


bool draw_gpsdetail(bool redraw,int page);         // 描き直したら true
bool draw_maplist_mode(bool redraw,int maplist_page); // 描き直したら true
void draw_setting_mode(bool redraw, int selectedLine, int cursorLine);

// 電源の知らせ（電源経路の異常・高温での充電。power.cpp が判断、ここは描くだけ）。
#define POWER_NOTICE_Y_MAP 86                    // 地図: バンク警告（46〜82）のすぐ下
#define POWER_NOTICE_Y_SUB (SCREEN_HEIGHT - 64)  // 設定・GPS 詳細・地図一覧: 下端（GPS 詳細の日時の行より上）
void draw_power_notice(int y, bool force);
void draw_power_status(bool force);              // 設定画面の最下行に電源の生の値
void draw_readiness(bool force);                 // 設定画面の右上に本番前チェック（readiness.cpp）
// Auto 10km の切り替わりの知らせ。地図の上端（ヘッダのすぐ下）に出す。TRACK UP の自機は下寄りなので、
// 進行方向の地図の真ん中をふさがない位置。バンク警告と同じ段だが、両方が同時に出ることはまず無い。
#define AUTO10K_NOTICE_Y 52
bool draw_auto10k_notice(bool force);            // 時間が終わって消す必要があれば true
void draw_bankwarning();
void startup_demo_tft();


int mod( int x, int y );
