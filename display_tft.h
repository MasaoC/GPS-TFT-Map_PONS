// ============================================================
// File    : display_tft.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : TFTディスプレイ描画モジュールのヘッダー。
//           画面サイズ・カラー定数・座標構造体・enum定義と、
//           マップ/コンパス/ヘッダー/フッター/設定画面/
//           リプレイ選択画面など全描画関数のプロトタイプ宣言。
//           多角形塗りつぶし・線分クリップなど描画共通部品の宣言も含む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================
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
    SETTING_SETDESTINATION,SETTING_DESTINATIONMODE,SETTING_TITLE,SETTING_BRIGHTNESS,SETTING_DEMOBIWA,SETTING_REPLAY,SETTING_UPWARD,SETTING_GPSDETAIL,SETTING_MAPDETAIL,SETTING_VOLUME,SETTING_VARIO_VOLUME,SETTING_EXIT,
    ND_MPS,ND_MPS_LGND,ND_SATS,ND_MT,ND_DIST_PLAT,ND_DESTNAME,ND_TEMP,ND_TIME,ND_DESTMODE,ND_MC_PLAT,ND_LAT,ND_LON,ND_DEGPERSEC_VAL,ND_DEGPERSEC_TEX,ND_BATTERY,
    ND_SEARCHING,ND_GPSDOTS,ND_GPSCOND,COUNTER,SETTING_SD_DETAIL,SETTING_VARIO_DETAIL,SETTING_SCALE
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
  extern TFT_eSprite backscreen;  // マップ描画用 (240×240px, 16bit)
  extern TFT_eSprite vsi_sprite;  // VSIインジケーター (5×240px, 16bit)
  extern bool fresh_display;
  extern int screen_brightness;
  
#endif

// Function to convert x, y coordinates on the TFT screen to latitude and longitude

cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection);
Coordinate xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown);
// Function to calculate the distance between two points (latitude and longitude) using an optimized formula



void draw_gs_track();
void draw_loading_image();
void draw_degpersec(double degpersecond);
void draw_map_footer();
void setup_tft();
void draw_strokes();

void draw_header();
void draw_footer();
float get_input_voltage();



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
void draw_replayselect(int page, int cursor);
extern volatile bool loading_replaylist;   // Core1 でリプレイ用ファイル一覧を取得中
extern bool replay_loading_displayed;      // "Stand by..." 表示中（読み込み完了時の再描画判定用）
void draw_variodetail(int page);
void draw_maplist_mode(int maplist_page);


void clean_backscreen();
void push_backscreen();
void draw_vsi();


void draw_flyinto(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness);
void draw_flyinto2(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness);
void draw_flyawayfrom(double dest_lat,double dest_lon, double center_lat, double center_lon, float scale, float up);
void draw_track(double center_lat,double center_lon,float scale,float up);
uint16_t mapdata_color(const char* name);  // 地図名の先頭文字 → 描画色
void draw_FlashMaps(double center_lat,double center_lon,float scale,float up);  // 内蔵ポリゴン（SD不要）
void draw_ExtraMaps(double center_lat,double center_lon,float scale,float up);
bool try_draw_km_distance(float scale, float km);
void draw_km_distances(float scale);
void startup_demo_tft();
void draw_demo_biwako();
void draw_replay_indicator();
// 線分を backscreen の矩形にクリップする。完全に画面外なら false を返す。
// TFT_eSPI の drawWedgeLine / drawLine は画面外の端点をそのまま走査するため、
// 高倍率で端点が数千px 外に出ると 1 フレームに数百ms かかる。描画前に必ず通すこと。
bool clip_line_to_screen(int* x0, int* y0, int* x1, int* y1);
// 地図座標から作った線を描く（クリップ済み・非アンチエイリアスの太線）。
// 道路やトラックのように本数が多い描画に使う。drawWideLine は画素ごとに
// readPixel を呼ぶため本数が多いと極端に遅い（実装のコメント参照）。
void draw_map_line(int x0, int y0, int x1, int y1, int w, uint16_t col);
void draw_map(stroke_group id, float mapUpDirection, double center_lat, double center_lon,float mapScale, const mapdata* mp,uint16_t color);
// 複数リングをまとめて even-odd 規則で塗りつぶす（島＝穴が自動的に抜ける）。
// ring_start は要素数 nrings+1 で、リング i の点は [ring_start[i], ring_start[i+1])。
// 辺数が上限を超える場合は何も描かず false を返す。
bool fill_polygon_evenodd(const int16_t* xs, const int16_t* ys,
                          const uint16_t* ring_start, uint8_t nrings, uint16_t color);
void draw_nofix_cross();                              // GPS fix なし時のグレー × 描画
void draw_hacc_circle(double scale, uint32_t hacc_mm); // hAcc 不良・gnssFixOK=false 時の不確かさ円描画
void draw_triangle(int ttrack,int steer_angle);
void draw_course_warning(int steer_angle);
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon,float scale, float upward);
void draw_pilon_takeshima_marks(double mapcenter_lat, double mapcenter_lon,float scale, float upward);



int mod( int x, int y );

