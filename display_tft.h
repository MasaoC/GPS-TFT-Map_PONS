// ============================================================
// File    : display_tft.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : TFTディスプレイ描画モジュールのヘッダー。
//           画面サイズ・カラー定数・座標構造体・enum定義と、
//           マップ/コンパス/ヘッダー/フッター/設定画面/
//           リプレイ選択画面など全描画関数のプロトタイプ宣言。
//           多角形塗りつぶし・線分クリップなど描画共通部品の宣言も含む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
// ★ この先頭ガードが無いと、mysd.h 経由と直接 include の二重読み込みで
//   関数プロトタイプの再宣言までは黙って通るが、enum の再定義は
//   コンパイルエラーになる（BootCheckState を足すときに入れた。v7 と同じ）。
#ifndef DISPLAY_TFT_H
#define DISPLAY_TFT_H
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "settings.h"
#include "navdata.h"




#define SCREEN_WIDTH 240
#define HEADERFOOTER_HEIGHT 50
#define SCREEN_HEIGHT 320
#define BACKSCREEN_SIZE 240

// ※ v5 / v6 基板にはバックライト制御線が無い（v5 はスイッチで入り切りするだけ）ため、
//    輝度調整機能（BRIGHTNESS_SETTING_AVAIL / TFT_BL / PNP_BL / NPN_BL / BL_PWM_FRQ）は
//    v6 の 0.947 で削除した。復活させる場合は git 履歴を参照。



#ifndef DISPLAY_TFT_DEFINED
  #define DISPLAY_TFT_DEFINED

  struct cord_tft{
    int x;
    int y;

    // cord_tft は latLonToXY() が返す **backscreen（240x240）内の座標**なので、
    // 判定も backscreen の大きさで行う。以前は SCREEN_HEIGHT(320) と比べていたため、
    // y が 240〜320 の点を「画面内」と判定してから TFT_eSPI 側で捨てていた。
    bool isOutsideTft(){
      return x < 0 || x > BACKSCREEN_SIZE || y < 0 || y > BACKSCREEN_SIZE;
    }
  };

  // 設定画面のメニュー項目 ID（button.cpp の menu_settings[].id）。
  // 以前は地図画面の各テキストにも ID を振って差分更新していたが（ND_* / COUNTER）、
  // その仕組み（TextManager）ごと v0.947 で廃止したのでメニュー分だけ残っている。
  enum text_id{
    SETTING_SETDESTINATION,SETTING_DESTINATIONMODE,SETTING_DEMOBIWA,SETTING_REPLAY,
    SETTING_UPWARD,SETTING_GPSDETAIL,SETTING_MAPDETAIL,SETTING_VOLUME,
    SETTING_EXIT,SETTING_SD_DETAIL,SETTING_SCALE
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

#endif

// 緯度経度を TFT スクリーン上のピクセル座標へ変換する。
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection);
// Function to calculate the distance between two points (latitude and longitude) using an optimized formula



void draw_gs_track();
void draw_degpersec(double degpersecond);
void draw_map_footer();
void setup_tft();

void draw_header();
void draw_footer();
float get_input_voltage();      // 実測のバッテリー電圧 [V]（ピーク追跡込み）
int battery_percent(float v);    // 電池電圧 → 残量 [%]（BAT_ZERO_VOLTAGE〜BAT_FULL_VOLTAGE）
float get_display_voltage();    // 画面表示用。リプレイ中は CSV の電圧を返す
bool  replay_voltage_active();  // リプレイの電圧を表示中か（USB 判定の抑止に使う）



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
void draw_maplist_mode(int maplist_page);


void push_backscreen();

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
void draw_map(float mapUpDirection, double center_lat, double center_lon,float mapScale, const mapdata* mp,uint16_t color);
// 複数リングをまとめて even-odd 規則で塗りつぶす（島＝穴が自動的に抜ける）。
// ring_start は要素数 nrings+1 で、リング i の点は [ring_start[i], ring_start[i+1])。
// 辺数が上限を超える場合は何も描かず false を返す。
bool fill_polygon_evenodd(const int16_t* xs, const int16_t* ys,
                          const uint16_t* ring_start, uint8_t nrings, uint16_t color);
void draw_nofix_cross();                              // GPS fix なし時のグレー × 描画
void draw_hacc_circle(double scale, uint32_t hacc_mm); // hAcc（GST）不良時の不確かさ円描画
void draw_triangle(int ttrack,float steer_angle);   // steer_angle は小数のまま渡す（しきい値 15/55/100 度を正確に判定するため）
void draw_course_warning(float steer_angle);   // 左右の判定だけに使う。int だと |ズレ|<1度 で左右が逆になる
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon,float scale, float upward);

// ---- 起動画面（v7 の 0.975 / 0.979）----
void draw_boot_title();   // 起動直後のタイトル（白画面を埋めるため setup() からも呼ぶ）
// 起動時チェックの進捗表示（draw_boot_progress() 用）。まだ確認できていない項目は PENDING。
enum BootCheckState : uint8_t { BOOT_CHECK_PENDING, BOOT_CHECK_OK, BOOT_CHECK_NG };
// setup() の間、GPS の設定が終わったら OK/NG を出す（v5 にはセンサーが無いので GPS だけ）。
// startup_demo_tft() が最終結果（SD 含む）を出すまで「INITIALIZING ...」のまま
// 何も変わらず固まって見える時間を減らすための中間表示。
void draw_boot_progress(BootCheckState gps);
void draw_pilon_takeshima_marks(double mapcenter_lat, double mapcenter_lon,float scale, float upward);

#endif  // DISPLAY_TFT_H
