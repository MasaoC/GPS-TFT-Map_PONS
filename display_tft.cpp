// ============================================================
// File    : display_tft.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : TFTディスプレイ描画の実装。
//           ポリゴン地図（内蔵/SD）、コンパス、飛行コース矢印、
//           ヘッダー/フッター、設定画面、
//           GPSDetail/SDDetail/マップリスト/リプレイ選択画面など全UI描画。
//           地図背景そのものは vectormap.cpp が描く。
//           描画の共通部品として、多角形の塗りつぶし（スキャンラインeven-odd）と
//           線分の画面クリップ・非アンチエイリアス太線もここに置く。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================
// Updates TFT display using TFT-eSPI library.

#include <cstring>  // for strlen and strcpy
#include <string>
#include <cstdlib>  // for malloc and free

#include "settings.h"
#include "display_tft.h"
#include "attitude.h"
#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "font_data.h"
#include "sound.h"
#include "button.h"
#include "airdata.h"
#include "imu.h"
#include "vectormap.h"

// フォント定義: TFT_eSPI のカスタムフォント（PROGMEM 格納）
#define AA_FONT_SMALL NotoSansBold15   // アンチエイリアス付き小サイズフォント（地図テキスト等）
#define NM_FONT_MEDIUM Arial_Black22   // 中サイズフォント（ESKF のロール・ピッチ表示）
#define NM_FONT_LARGE Arial_Black56    // 大サイズフォント（ヘッダーの速度・真方位表示）

// ---- 処理時間計測変数（BNO085配置コア決定用、デバッグビルドのみ） ----
#ifndef RELEASE
TimingStat ts_draw_header     = TSTAT_INIT("C0_draw_header");
TimingStat ts_push_backscreen = TSTAT_INIT("C0_push_backscreen");
#endif

// ===== TFT スプライト =====
// tft        : TFT_eSPI 本体（240×320px）。直接描画は非推奨で、スプライト経由が基本。
// backscreen : メインの 240×240 マップ描画用スプライト。描画後 pushSprite(0,50) で転送。
// header_footer: ヘッダー（50px）・フッター（30px）共有スプライト。pushSprite で位置を変えて転送。
TFT_eSPI tft = TFT_eSPI();                 // Invoke custom library
TFT_eSprite backscreen = TFT_eSprite(&tft);
TFT_eSprite header_footer = TFT_eSprite(&tft);
TFT_eSprite vsi_sprite = TFT_eSprite(&tft);  // VSIインジケーター (5×240px, 16bit, 2,400 Byte)

// ===== 画面輝度 =====
int screen_brightness = 255;                                // 現在の輝度値（PWMデューティ 0-255）
const int brightnessLevels[] = { 10, 100, 150, 200, 255 };  // 選択可能な輝度ステップ
int brightnessIndex = 4;                                    // デフォルト: 最大輝度（255）

// ===== 地図方向モード =====
// NORTHUP: 地図は常に北が上。TRACKUP: 進行方向が上に来るように回転する。
#define MODE_TRACKUP 0
#define MODE_NORTHUP 1
#define MODE_SIZE 2
int upward_mode = MODE_NORTHUP;




extern volatile int sound_volume;
extern volatile int vario_volume;
extern volatile bool vario_inhibit;
extern int screen_mode;
extern int destination_mode;


// 地図座標から作った線を backscreen に描く際は、必ずこの 2 つを使うこと。
// 緯度経度由来の座標は高倍率で画面外へ大きく飛び出し、そのまま TFT_eSPI に
// 渡すと画面外を延々と走査して 1 フレームに数百 ms かかる
// （理由は clip_line_to_screen のコメント）。
static inline void draw_clipped_line(int x0, int y0, int x1, int y1, uint16_t col) {
  if (!clip_line_to_screen(&x0, &y0, &x1, &y1)) return;
  backscreen.drawLine(x0, y0, x1, y1, col);
}
static inline void draw_clipped_wideline(int x0, int y0, int x1, int y1, int w, uint16_t col) {
  if (!clip_line_to_screen(&x0, &y0, &x1, &y1)) return;
  backscreen.drawWideLine(x0, y0, x1, y1, w, col);
}

// 地図座標から作った線を、本数が多い箇所で描くための関数。
//
// TFT_eSPI の drawWideLine は bg_color の既定値が 0x00FFFFFF で、この経路では
// アンチエイリアス画素ごとに readPixel() を呼んで setWindow をやり直し、さらに
// wedgeLineDistance() の浮動小数点距離計算を画素ごとに行う（drawWedgeLine 参照）。
// 見た目は綺麗だが単価が高く、地図の道路や 500 点のトラックのように線が数百〜数千本
// になる描画では支配的なコストになる。
// 240px の画面ではアンチエイリアスの効果も小さいので、平行な drawLine を重ねて太らせる。
//
// 誘導線やパイロン基準線のように本数が少なく目立つ線は、従来どおり
// draw_clipped_wideline（アンチエイリアス付き）のままにしてある。
void draw_map_line(int x0, int y0, int x1, int y1, int w, uint16_t col) {
  if (!clip_line_to_screen(&x0, &y0, &x1, &y1)) return;
  backscreen.drawLine(x0, y0, x1, y1, col);
  if (w <= 1) return;
  // 線の主方向に対して垂直側へずらして重ねる（斜めでも太さがほぼ揃う）
  const bool steep = abs(y1 - y0) > abs(x1 - x0);
  for (int o = 1; o <= (w - 1) / 2; o++) {
    if (steep) {
      backscreen.drawLine(x0 - o, y0, x1 - o, y1, col);
      backscreen.drawLine(x0 + o, y0, x1 + o, y1, col);
    } else {
      backscreen.drawLine(x0, y0 - o, x1, y1 - o, col);
      backscreen.drawLine(x0, y0 + o, x1, y1 + o, col);
    }
  }
  if (w % 2 == 0) {  // 偶数幅は片側にもう 1 本足す
    if (steep) backscreen.drawLine(x0 + w / 2, y0, x1 + w / 2, y1, col);
    else backscreen.drawLine(x0, y0 + w / 2, x1, y1 + w / 2, col);
  }
}

// 地図方向モードを NORTHUP ↔ TRACKUP で切り替える。
void toggle_mode() {
  upward_mode = (upward_mode + 1) % MODE_SIZE;
}

bool is_northupmode() { return upward_mode == MODE_NORTHUP; }
bool is_trackupmode() { return upward_mode == MODE_TRACKUP; }

// TRACKUP モードでは自機を画面下寄り（BACKSCREEN_SIZE*3/4）に表示し、前方視界を広げる。
// NORTHUP モードでは従来通り中央（BACKSCREEN_SIZE/2）を返す。
inline int get_self_cy() {
  return is_trackupmode() ? (BACKSCREEN_SIZE * 3 / 4) : (BACKSCREEN_SIZE / 2);
}



// 緯度経度をスプライト（backscreen）上のピクセル座標に変換する。
// Mercator 投影で X（経度差）・Y（緯度差）を km 距離に変換し、
// mapScale（px/km）を掛けてピクセル量にする。
// mapUpDirection（度）だけ回転させることで TRACKUP モードに対応する。
// スプライト中心が現在位置（mapCenterLat/Lon）に対応する。
// 戻り値の Y 座標は TFT 座標系（下が正）のため yDist を反転させる。
cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection) {
  // Calculate x distance (longitude) = Approx distance per degree longitude in km with Mercator projection.
  float xDist = (lon - mapCenterLon) * 111.321;  // 1 degree longitude = ~111.321 km
  // Calculate y-coordinates in Mercator projection
  double yA = latitudeToMercatorY(mapCenterLat);
  double yB = latitudeToMercatorY(lat);
  // Calculate the distance on the y-axis
  double yDist = (yB - yA)* 6378.22347118;// 1 radian latitude（Mercator Y軸）= 111.321 *180/3.1415 = 6378.22347118 km（地球半径に相当）
  // Apply scale factor
  xDist *= mapScale;
  yDist *= mapScale;
  // Apply rotation for map up direction
  float angleRad = mapUpDirection * DEG_TO_RAD;
  float rotatedX = xDist * cos(angleRad) - yDist * sin(angleRad);
  float rotatedY = xDist * sin(angleRad) + yDist * cos(angleRad);
  // Translate to screen coordinates
  // TRACKUP: 自機は get_self_cy()=180 を基準、NORTHUP: 120（中央）
  return cord_tft{
    (BACKSCREEN_SIZE / 2) + (int)rotatedX,
    get_self_cy() - (int)rotatedY  // Y is inverted on the screen
  };
}


// TFT スクリーン上のピクセル座標 (x, y) を緯度経度に逆変換する（latLonToXY の逆関数）。
// mapshiftdown: 地図表示領域の Y 軸オフセット（ヘッダー分のシフト量）。
// 主にタッチ入力や目的地座標の逆引きに使用する。
Coordinate xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown) {
    // Translate screen coordinates to map coordinates
    // TRACKUP: 自機は (HEADERFOOTER_HEIGHT + get_self_cy()) に表示されるため Y 基準を更新
    float screenX = x - (SCREEN_WIDTH / 2);
    float screenY = (HEADERFOOTER_HEIGHT + get_self_cy()) - y + mapshiftdown;

    // Apply rotation inverse for map up direction
    float angleRad = mapUpDirection * DEG_TO_RAD;
    float rotatedX = screenX * cos(angleRad) + screenY * sin(angleRad);
    float rotatedY = -screenX * sin(angleRad) + screenY * cos(angleRad);

    // Convert map distances to degrees
    float lonDist = rotatedX / (111320.0 * cos(mapCenterLat * DEG_TO_RAD) * mapScale);
    float latDist = rotatedY / (110540.0 * mapScale);

    // Calculate latitude and longitude
    float newLon = mapCenterLon + (lonDist * RAD_TO_DEG);
    float newLat = mapCenterLat + (latDist * RAD_TO_DEG);

    return Coordinate{newLat, newLon};
}




// TFT ディスプレイと描画スプライトを初期化する。
// 1. GPIO27(WR) / GPIO28(DC) を LOW にしてから TFT を起動する。
// 2. VERTICAL_FLIP マクロでパネル向きを切り替える（setRotation）。
// 3. backscreen（240×BACKSCREEN_SIZE）と header_footer（240×HEADERFOOTER_HEIGHT）スプライトを作成する。
// スプライトは一度だけ作成すれば再利用できるため、created() で二重作成を防ぐ。
void setup_tft() {

  //TFT_WR
  gpio_init(27);
  gpio_set_dir(27, GPIO_OUT);
  gpio_put(27, 0);
  //TFT_DC
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);
  gpio_put(28, 0);


  tft.begin();

#ifdef VERTICAL_FLIP
  tft.setRotation(0);           //set 0 for newhaven
#else
  tft.setRotation(2);           //set 2 for normal（VERTICAL_FLIP 未定義時）
#endif

  tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  tft.fillScreen(COLOR_WHITE);



  if(!backscreen.created()){
    backscreen.setColorDepth(16);
    backscreen.createSprite(SCREEN_WIDTH, BACKSCREEN_SIZE);
    backscreen.loadFont(AA_FONT_SMALL);
  }
  if(!header_footer.created()){
    header_footer.setColorDepth(16);
    header_footer.createSprite(SCREEN_WIDTH, HEADERFOOTER_HEIGHT);
    header_footer.loadFont(NM_FONT_LARGE);
  }
  // VSIスプライト: 5px幅 × 240px高さ、16bit色深度（RAM消費 2,400 Byte）
  if(!vsi_sprite.created()){
    vsi_sprite.setColorDepth(16);
    vsi_sprite.createSprite(5, BACKSCREEN_SIZE);
  }
}



// TFT スクリーン上のピクセル座標を表すクラス。
// isOutsideTft() で画面外判定を行い、不要な drawLine 呼び出しを省略するために使う。
class Point {
public:
  int x, y;
  Point(int x = 0, int y = 0)
    : x(x), y(y) {};

  //xr_offset は、画面右端を狭めるオプション。これによって改行してはいけない状況での、isOutsideTftを実行可能。
  bool isOutsideTft(){
    return x < 0 || x > SCREEN_WIDTH || y < 0 || y > SCREEN_HEIGHT;
  }
};

// 画面上のテキスト要素を表すクラス。
// id で識別し、同一 id が既に表示されている場合は旧テキストを白で上書きしてから新テキストを描画する
//（ちらつきなしに動的テキストを更新するための仕組み）。
// textchar は heap に動的確保され、デストラクタで解放される。
class Text {
private:
  int id;
  int size;
  Point cord;
  char* textchar;

public:
  Text()
    : id(0), size(0), cord(0, 0), textchar(NULL) {}

  Text(int id, int size, int x, int y, const char* text_in)
    : id(id), size(size), cord(x, y), textchar(NULL) {
    setText(text_in);
  }

  ~Text() {
    if (textchar != NULL) {
      free(textchar);  // Free dynamically allocated memory
    }
  }

  // Copy constructor
  Text(const Text& other)
    : id(other.id), size(other.size), cord(other.cord), textchar(NULL) {
    setText(other.textchar);  // Use setText to allocate new memory
  }

  // Assignment operator (handles self-assignment and memory leak prevention)
  Text& operator=(const Text& other) {
    if (this != &other) {  // Check for self-assignment
      if (textchar != NULL) {
        free(textchar);  // Free the existing memory
      }
      id = other.id;
      size = other.size;
      cord = other.cord;
      setText(other.textchar);  // Allocate and copy the new string
    }
    return *this;
  }

  bool setText(const char* text_in) {
    if (textchar != NULL) {
      free(textchar);
      textchar = NULL;
    }
    if (text_in == NULL) {
      return false;  // null ポインタは受け付けない
    }
    size_t length = strlen(text_in);
    textchar = (char*)malloc((length + 1) * sizeof(char));
    if (textchar == NULL) {
      return false;  // メモリ確保失敗
    }
    memcpy(textchar, text_in, length + 1);  // strcpy の代わりに長さ付きコピー
    return true;
  }

  int getId() const {
    return id;
  }
  int getSize() const {
    return size;
  }
  Point getCord() const {
    return cord;
  }
  const char* getTextChar() const {
    return textchar;
  }

  friend class TextManager;
};

// Text オブジェクトの配列を管理し、ID ベースの差分更新描画を提供するクラス。
// drawText()/drawTextf() を呼ぶと、同 ID の古いテキストを自動消去してから新しいテキストを描画する。
// 最大 50 テキスト（textmanager(50) で確保）。
class TextManager {
private:
  Text** draw_texts;  // Array of pointers to Text objects
  int textCount;
  int maxtext;

  Text* createNewText(int id, int size, int x, int y, const char* text_in) {
    if (textCount >= maxtext) {
      return nullptr;
    }
    Text* newText = new Text(id, size, x, y, text_in);
    draw_texts[textCount++] = newText;
    return newText;
  }

public:
  TextManager(int maxTexts)
    : textCount(0), maxtext(maxTexts) {
    draw_texts = new Text*[maxTexts];
    for (int i = 0; i < maxTexts; ++i) {
      draw_texts[i] = nullptr;
    }
  }

  ~TextManager() {
    for (int i = 0; i < textCount; ++i) {
      delete draw_texts[i];
    }
    delete[] draw_texts;
  }

  bool drawText(int id, int size, int x, int y, uint16_t col, const char* text_in) {
    Text* foundText = nullptr;
    // Search for text with the same id.
    for (int i = 0; i < textCount; i++) {
      if (draw_texts[i]->getId() == id) {
        foundText = draw_texts[i];
        break;
      }
    }
    // If found, overwrite text with white.
    if (foundText != nullptr) {
      tft.setCursor(foundText->getCord().x, foundText->getCord().y);
      tft.setTextColor(COLOR_WHITE);
      tft.setTextSize(foundText->getSize());
      tft.print(foundText->getTextChar());
      // Update value.
      foundText->cord.x = x;
      foundText->cord.y = y;
      foundText->size = size;
      foundText->setText(text_in);  // Update textchar
    }
    // If not found, create new Text with the id
    if (foundText == nullptr) {
      foundText = createNewText(id, size, x, y, text_in);
      if (foundText == nullptr) {
        return false;
      }
    }
    // Print the text.
    tft.setCursor(foundText->cord.x, foundText->cord.y);
    tft.setTextColor(col, COLOR_WHITE);
    tft.setTextSize(foundText->size);
    tft.print(foundText->getTextChar());
    return true;
  }


  bool drawTextf(int id, int size, int x, int y, uint16_t col, const char* format, ...) {
    char buffer[256];  // Temporary buffer for formatted text

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    return drawText(id, size, x, y, col, buffer);
  }
};

TextManager textmanager(50);





// 指定した km 値のスケールバーを backscreen の右下端に描画する。
// 画面に収まるサイズ（140px 以内）のときのみ描画し、true を返す。
// 緯度補正（cos(lat)）を加えることで Mercator 歪みに対応したスケールを表示する。
bool try_draw_km_distance(float scale, float km) {
  double latnow = get_gps_lat();
  if(latnow <-80 || latnow > 80){
    latnow = 35;
  }
  int distance_px =  scale*km/cos(radians(latnow));//scale * km;  //scale is px/km
  if(distance_px > 140)
    return false;

  int ypos = 240-2;
  int xpos = 240-2-5;  // VSI（右端5px）と重ならないよう5px左にオフセット

  backscreen.drawFastVLine(xpos, ypos-distance_px, distance_px, COLOR_BLACK);
  backscreen.drawFastHLine(xpos-5, ypos, 5, COLOR_BLACK);
  backscreen.drawFastHLine(xpos-5, ypos-distance_px, 5, COLOR_BLACK);
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_BLACK);
  if(km >= 100){
    backscreen.setCursor(xpos-18,ypos-distance_px-8);
    backscreen.printf("%d",(int)km);
  }
  else if(km < 1.0 ){
    backscreen.setCursor(xpos-18,ypos-distance_px-8);
    backscreen.printf("%.1f",km);
  }else{
    backscreen.setCursor(xpos-11,ypos-distance_px-8);
    backscreen.printf("%d",(int)km);
  }
  backscreen.setCursor(xpos-12,ypos-distance_px);
  backscreen.print("km");
  return true;
}


// コンパスの N/E/S/W ラベルを backscreen 上に描画する。
// truetrack: 現在の真方位（度）。N/E/S/W の位置は truetrack 方向を基準に算出する。
// NORTHUP モードでは dist=100px（広め）、TRACKUP モードでは dist=73px（狭め）にして
// 進行方向矢印（draw_triangle）と重ならないようにする。
// N/E/S/W は真方位。磁気コンパスを載せない方針になったので偏角補正はしない。
void draw_compass(float truetrack, uint16_t col) {
  backscreen.setTextWrap(false);
  // 半径は元の 2/3。地図左下のピッチ・ロール表示と N/E/S/W が重なって
  // 読みにくかったため縮めてある（針の長さ get_needle_len() は変えていない）。
  int dist = 73 * 2 / 3 - 4;
  if (is_northupmode()) {
#ifdef TFT_USE_ST7735
    dist = 50 * 2 / 3 - 4;
#else
    dist = 100 * 2 / 3 - 4;
#endif
  }
  int centerx = BACKSCREEN_SIZE / 2;
  int centery = get_self_cy();  // TRACKUP=180, NORTHUP=120
  // 偏角は加えない（真方位表示）
  float radian = deg2rad(truetrack);
  float radian45offset = deg2rad(truetrack + 45);
  backscreen.setTextColor(col, COLOR_WHITE);
  backscreen.setTextSize(2);
  backscreen.loadFont(AA_FONT_SMALL);
  

  cord_tft n = { int(centerx + sin(-radian) * dist), int(centery - cos(radian) * dist) };
  cord_tft e = { int(centerx + cos(radian) * dist), int(centery - sin(radian) * dist) };
  cord_tft s = { int(centerx + sin(radian) * dist), int(centery + cos(radian) * dist) };
  cord_tft w = { int(centerx - cos(-radian) * dist), int(centery - sin(-radian) * dist) };
  if (!n.isOutsideTft()) {
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);  // N は赤
    backscreen.setCursor(n.x - 5, n.y - 5);
    backscreen.print("N");
    backscreen.setTextColor(col, COLOR_WHITE);         // 残りは元の色に戻す
  }
  if (!e.isOutsideTft()) {
    backscreen.setCursor(e.x - 5, e.y - 5);
    backscreen.print("E");
  }
  if (!s.isOutsideTft()) {
    backscreen.setCursor(s.x - 5, s.y - 5);
    backscreen.print("S");
  }
  if (!w.isOutsideTft()) {
    backscreen.setCursor(w.x - 5, w.y - 5);
    backscreen.print("W");
  }

  cord_tft nw = { int(centerx + sin(-radian45offset) * dist), int(centery - cos(radian45offset) * dist) };
  cord_tft ne = { int(centerx + cos(radian45offset) * dist), int(centery - sin(radian45offset) * dist) };
  cord_tft se = { int(centerx + sin(radian45offset) * dist), int(centery + cos(radian45offset) * dist) };
  cord_tft sw = { int(centerx - cos(-radian45offset) * dist), int(centery - sin(-radian45offset) * dist) };

  /*
  backscreen.setTextSize(1);
  if (!nw.isOutsideTft()) {
    backscreen.setCursor(nw.x - 5, nw.y - 2);
    backscreen.print("NW");
  }
  if (!ne.isOutsideTft()) {
    backscreen.setCursor(ne.x - 5, ne.y - 2);
    backscreen.print("NE");
  }
  if (!se.isOutsideTft()) {
    backscreen.setCursor(se.x - 5, se.y - 2);
    backscreen.print("SE");
  }
  if (!sw.isOutsideTft()) {
    backscreen.setCursor(sw.x - 5, sw.y - 2);
    backscreen.print("SW");
  }
  */
  backscreen.setTextWrap(true);
}




// 現在のスケールに適した距離スケールバーを最大 2 本描画する。
// 候補リストを大きい順に試し、140px 以内に収まる最初の 2 候補を表示する。
// 2 本描画したら終了（それ以上は画面が煩雑になるため）。
void draw_km_distances(float scale) {
  const float distances[] = {400, 200, 100, 50, 20, 10, 5, 2, 1, 0.4, 0.2};
  int draw_counter = 0;

  backscreen.unloadFont();

  for (float d : distances) {
    if (try_draw_km_distance(scale, d)) {
      draw_counter++;
      if (draw_counter >= 2)
        break;
    }
  }

  // スケールバーが 2 本に届かなくても必ずフォントを戻す。
  // 戻し忘れると以降に描く [GS]/[m/s]/[MT] やコース警告が内蔵フォントになり、
  // 表示が崩れる（スケールによって発生したりしなかったりする不具合の原因）。
  backscreen.loadFont(AA_FONT_SMALL);
}

// ===== マップ描画管理変数 =====
bool fresh = false;         // 地図の強制再描画フラグ（スケール変更時などに true にする）
float last_scale = 1.0;     // 前回描画時のスケール（変化を検知するための記憶値）
float last_up = 0;          // 前回描画時の mapUpDirection







#ifdef TFT_USE_ST7735
#define TRIANGLE_HWIDTH 4
#define TRIANGLE_SIZE 12
#else
#define TRIANGLE_HWIDTH 6
#define TRIANGLE_SIZE 18
#endif





extern float last_tone_tt;
extern unsigned long trackwarning_until;



// コース逸脱警告ボックスを backscreen に描画する。
// 1 秒おきに点滅（(millis()/1000)%2 == 0 のときだけ描画）し、
// steer_angle の符号で "Turn RIGHT" / "Turn LEFT" を切り替える。
// 呼び出しはメインの draw ループから angle_diff > 15° のときに行われる。
void draw_course_warning(int steer_angle){
  if((millis()/1000)%2 == 0){
    // TRACKUP: 自機アイコン尾翼（cy+9=189）の下にボックスを配置する
    int box_y = is_trackupmode() ? (get_self_cy() + 12) : 175;  // TRACKUP=192, NORTHUP=175
    backscreen.fillRect(5, box_y, SCREEN_WIDTH-5*2, 25, COLOR_WHITE);
    backscreen.drawRect(5, box_y, SCREEN_WIDTH-5*2, 25, COLOR_RED);
    backscreen.drawRect(6, box_y+1, SCREEN_WIDTH-5*2-2, 25-2, COLOR_RED);
    backscreen.setTextColor(COLOR_RED);
    if(steer_angle > 0){
      backscreen.setCursor(23, box_y+5);
      backscreen.print("Turn RIGHT! Turn RIGHT!");
    }
    else{
      backscreen.setCursor(30, box_y+5);
      backscreen.print("Turn LEFT! Turn LEFT!");
    }
  }
}

extern float course_warning_index;
const int NEEDLE_LEN_NORTHUP = 120;  // 針の長さ（px）。スプライト半径の最大値に合わせてある。
const int NEEDLE_LEN_TRACKUP = 180;  // TRACKUP モード用: 自機が下寄りのため前方スペースが広く、長めにする。
const int NEEDLE_LEN = NEEDLE_LEN_NORTHUP;  // 後方互換用。実際の使用箇所では get_needle_len() を推奨。
inline int get_needle_len() { return is_trackupmode() ? NEEDLE_LEN_TRACKUP : NEEDLE_LEN_NORTHUP; }

// 機体位置マーカー（飛行機アイコン）とコース逸脱表示を backscreen に描画する。
//
// NORTHUP モード（自機マーカー + コース針）:
//   - 機体アイコン: 主翼の横棒、尾翼の横棒、胴体線の 3 本で表現（進行方向 = ttrack）。
//   - True Track 維持針(三角形): 長さ NEEDLE_LEN の太線（last_tone_tt 方向へ伸びる）。
//   - コース逸脱 arc: steer_angle に比例した赤い弧で偏差を視覚化。
//   - 誘導三角形: steer_angle > 15°, 55°, 100° の 3 段階で赤い三角形を追加表示。
//     ※ trackwarning_until で警告持続時間を管理し、時間切れ後は黒枠のみに戻る。
//
// TRACKUP モード（固定マーカー）:
//   - 進行方向は常に上なので、中心の垂直線 + 翼横棒の固定アイコンのみ描画する。

// GPS fix なし時に飛行機マークの代わりにグレーの × を中央に描画する。
// × のサイズは飛行機アイコンと同程度（腕の長さ 12px）。
void draw_nofix_cross() {
  const int cx   = BACKSCREEN_SIZE / 2;  // = 120（X は常に中央）
  const int cy   = get_self_cy();        // TRACKUP=180, NORTHUP=120
  const int arm  = 12;                   // × の腕の長さ [px]
  // TRACKUP: GNSS 精度に関わらず、進行方向を示す縦線は常に表示する
  if (is_trackupmode()) {
    const int shortening = 15;
    backscreen.drawFastVLine(cx,     shortening, cy - shortening - 5, COLOR_BLACK);
    backscreen.drawFastVLine(cx + 1, shortening, cy - shortening - 5, COLOR_BLACK);
  }
  backscreen.drawWideLine(cx - arm, cy - arm, cx + arm, cy + arm, 3, COLOR_GRAY);
  backscreen.drawWideLine(cx + arm, cy - arm, cx - arm, cy + arm, 3, COLOR_GRAY);
}

// hAcc 不良時に飛行機マークの代わりに自機位置の不確かさ円を描画する。
// gnssFixOK=false の場合は常にこの関数が呼ばれる。
// 動作:
//   1. 中心に青い塗りつぶし小円（HDOP_CENTER_DOT_RADIUS px）で位置を示す。
//   2. 誤差半径 = hacc_mm [mm] → [m] → [km] → ピクセル換算して青い輪郭円を描画。
//   3. 輪郭円半径 < HDOP_MIN_CIRCLE_RADIUS の場合は輪郭円を省略し、精度数値をテキストで表示する。
void draw_hacc_circle(double scale, uint32_t hacc_mm) {
  const int cx = BACKSCREEN_SIZE / 2;  // = 120（X は常に中央）
  const int cy = get_self_cy();        // TRACKUP=180, NORTHUP=120

  // TRACKUP: GNSS 精度に関わらず、進行方向を示す縦線は常に表示する
  if (is_trackupmode()) {
    const int shortening = 15;
    backscreen.drawFastVLine(cx,     shortening, cy - shortening - 5, COLOR_BLACK);
    backscreen.drawFastVLine(cx + 1, shortening, cy - shortening - 5, COLOR_BLACK);
  }

  // 誤差半径: [mm] → [m] → [km] → ピクセル（四捨五入で正確な円サイズに）
  float accuracy_m = hacc_mm / 1000.0f;
  int r = (int)roundf((accuracy_m / 1000.0f) * (float)scale);

  // 中心位置を示す小さい塗りつぶし青円
  backscreen.fillCircle(cx, cy, HDOP_CENTER_DOT_RADIUS, COLOR_BLUE);

  // 円半径がバックスクリーン半分を超えると画面外にはみ出して青い点だけに見えるため、
  // 大きすぎる場合もテキスト表示にフォールバックする。
  const int MAX_CIRCLE_R = BACKSCREEN_SIZE / 2;  // 120px

  if (r >= HDOP_MIN_CIRCLE_RADIUS && r <= MAX_CIRCLE_R) {
    // 不確かさ範囲を示す輪郭青円
    backscreen.drawCircle(cx, cy, r, COLOR_BLUE);
  } else {
    // 円が小さすぎ／大きすぎる場合: 精度数値をテキストで表示（例: "Acc:8m" / "Acc:1.2km"）
    char buf[16];
    if (accuracy_m < 1000.0f)
      snprintf(buf, sizeof(buf), "Acc:%.0fm", accuracy_m);
    else
      snprintf(buf, sizeof(buf), "Acc:%.1fkm", accuracy_m / 1000.0f);
    backscreen.setTextSize(1);
    backscreen.setTextColor(COLOR_BLUE, COLOR_BLACK);
    backscreen.setCursor(cx + HDOP_CENTER_DOT_RADIUS + 2, cy - 4);
    backscreen.print(buf);
  }
}

// ESKF の姿勢を画面に出してよいかを判定する。
// リプレイ中は CSV に IMU 生データ（.bin）が入っていないため ESKF には
// 「いま机の上にある本体」の姿勢しか流れてこない。再生中の飛行と全く無関係な
// ロール・ピッチ・ヨーが出てしまい誤解を招くので、リプレイ中はまとめて非表示にする。
// （将来 .bin まで再生できるようにしたら、この判定を外せばよい）
// デモモード用の疑似風。表示の確認だけが目的で、実機の推定（attitude.cpp）には触れない。
// 風速 0〜6 m/s を 24 秒周期で往復させ、風向は 30 秒で 1 回転させる。
// これで 3 色すべてと全方位を机の上で一通り確認できる。
// ※ ESKF が収束していないと draw_eskf_attitude() 自体が描かれないので、
//   GNSS の入る場所（屋外・窓際）でデモを動かすこと。
static bool demo_wind(float &speed_mps, float &dir_to_deg) {
  const uint32_t ms = millis();
  speed_mps  = 3.0f * (1.0f - cosf((float)(ms % 24000u) * (2.0f * (float)M_PI / 24000.0f)));
  dir_to_deg = (float)(ms % 30000u) * (360.0f / 30000.0f);
  return true;
}

// デモ用の機首方位（真方位）。デモの疑似風と対地速度から偏流角を作るので、
// 風の矢印の向きと機首の振れが連動して見え、デモとして自然になる。
//   対気速度ベクトル = 対地速度ベクトル - 風ベクトル、その向きが機首方位。
// 実際の偏流角は横風が強いと 60 度近くになるが、デモの見た目としては大きすぎるので
// DEMO_CRAB_MAX_DEG に制限する。
static bool demo_heading(float &yaw_deg) {
  float ws, wd;
  if (!demo_wind(ws, wd)) return false;
  const float trk = (float)get_gps_truetrack();
  float vg = (float)get_gps_mps();
  if (vg < 1.0f) vg = 1.0f;
  const float te = vg * sinf(deg2rad(trk)) - ws * sinf(deg2rad(wd));
  const float tn = vg * cosf(deg2rad(trk)) - ws * cosf(deg2rad(wd));
  float crab = atan2f(te, tn) * 180.0f / (float)M_PI - trk;
  while (crab >  180.0f) crab -= 360.0f;
  while (crab < -180.0f) crab += 360.0f;
  if (crab >  DEMO_CRAB_MAX_DEG) crab =  DEMO_CRAB_MAX_DEG;
  if (crab < -DEMO_CRAB_MAX_DEG) crab = -DEMO_CRAB_MAX_DEG;
  yaw_deg = trk + crab;
  return true;
}

// 表示に使うヨー（真方位）と、その 95% 精度。
// デモは疑似値、リプレイは記録値、通常は ESKF の推定値。ここに集約して、
// 自機アイコンの向き・ヨー数値・偏流点線・信頼度判定が食い違わないようにする。
static bool eskf_display_yaw(float &yaw_deg) {
  if (is_demo_active()) return demo_heading(yaw_deg);
  if (getReplayMode())  { float a; return get_replay_yaw(yaw_deg, a); }
  if (!attitude_ready()) return false;
  float r, p;
  attitude_get_euler(r, p, yaw_deg);
  return true;
}
static bool eskf_display_yaw_acc(float &acc95) {
  if (is_demo_active()) { acc95 = DEMO_YAW_ACC95_DEG; return true; }
  if (getReplayMode())  { float y; return get_replay_yaw(y, acc95); }
  if (!attitude_ready()) return false;
  acc95 = attitude_get_yaw_acc95_deg();
  return true;
}

bool eskf_display_enabled() {
  // マスタースイッチ。OFF なら地図上の姿勢表示は一切出さない（リプレイ含む）。
  // 表示系はすべてこの関数を通るので、ここ 1 か所で塞げる。
  if (!attitude_get_rpy_enabled()) return false;
  // リプレイ中は IMU 生データ（.bin）を再生しないので実機 ESKF の値は使えないが、
  // 同じ日の姿勢ログ（imu_replaydata/ か旧 euler/）があれば当時の値を再現できる。
  if (getReplayMode()) {
    float r, p;
    return get_replay_attitude(r, p);
  }
  return attitude_ready();
}

// ESKF のヨー（真方位）を表示・アイコン回転に使ってよいかを判定する。
// ヨーは水平加速度がある間しか可観測にならず、等速直進が続くと際限なく漂うため、
// 推定精度がしきい値以下のときだけ「機首方向」として信用する。
// 判定は 95%(2σ) で行う。1σ のまま「これ以下なら安心」と扱うと 3 回に 1 回は外れる。
bool eskf_calib_allowed() {
  if (!attitude_ready()) return false;
  // 通常は対地速度で「地上にいる」ことを確認する。
  if (!getReplayMode() && !is_demo_active())
    return get_gps_mps() <= LEVEL_CALIB_MAX_MPS;
  // デモ・リプレイ中は get_gps_mps() が再生データの速度を返すため、実機が机の上で
  // 止まっていても「飛行中」と判定されて APPLY できなくなる。
  // GPS が偽物なので、代わりに IMU 由来の静止判定を使う。これは再生データでは
  // 偽装できない実機の物理状態なので、飛行中に誤って較正する危険も無い。
  return attitude_is_static();
}

bool eskf_yaw_reliable() {
  // リプレイは記録された 95% 値で当時と同じ判定を再現する。しきい値は現在の設定を
  // 使うので、後からしきい値を変えて過去フライトを見直すこともできる。
  float acc95;
  if (!eskf_display_enabled() || !eskf_display_yaw_acc(acc95)) return false;
  return acc95 < ESKF_YAW_TRUST_95_DEG;
}

// ヨーが「著しく信頼できる」か（95%値 < ESKF_YAW_DRIFT_95_DEG）。
// 偏流角を示す点線を出してよいかの判定に使う。eskf_yaw_reliable() より厳しい。
static bool eskf_yaw_high_confidence() {
  float acc95;
  if (!eskf_display_enabled() || !eskf_display_yaw_acc(acc95)) return false;
  return acc95 < ESKF_YAW_DRIFT_95_DEG;
}

// 自機アイコンの色。ESKF のヨーで機首を向けているときは黒、
// 単に対地進路（トラック）を向けているだけのときはグレー。
// 「いま見えている向きが機首なのかトラックなのか」を色で区別できるようにする。
// ※ ヨーの情報源が無い場合（BNO085 非搭載機、姿勢ログの無いリプレイ）は
//   区別する意味が無いので従来どおり黒。ここを入れないと IMU の無い個体で
//   アイコンが永久にグレーになってしまう。
static uint16_t plane_icon_color() {
  bool have_yaw_source;
  if (getReplayMode()) { float y, a; have_yaw_source = get_replay_yaw(y, a); }
  else                 { have_yaw_source = get_imu_ok(); }
  if (!have_yaw_source) return COLOR_BLACK;
  return eskf_yaw_reliable() ? COLOR_BLACK : COLOR_GRAY;
}

// 自機アイコン（主翼・尾翼・胴体の 3 本＝「士」の形）を任意の向きで描く。
// screen_deg: 画面の上を 0 とした時計回りの角度 [度]。
//   NORTHUP は画面上＝北なので、そのまま方位を渡せばよい。
//   TRACKUP は画面上＝対地進路なので (方位 - トラック) を渡す（＝偏流角）。
// ここを機首方位で呼べばアイコンが実際の機首を向き、トラックで呼べば従来どおり。
static void draw_plane_icon(int cx, int cy, float screen_deg, uint16_t col) {
  const float a = deg2rad(screen_deg);
  const float ca = cos(a), sa = sin(a);

  int left_wing_x  = cx - TRIANGLE_HWIDTH * 2 * ca;
  int left_wing_y  = cy - TRIANGLE_HWIDTH * 2 * sa;
  int right_wing_x = cx + TRIANGLE_HWIDTH * 2 * ca;
  int right_wing_y = cy + TRIANGLE_HWIDTH * 2 * sa;
  int left_tail_x  = cx + (-TRIANGLE_HWIDTH / 2) * ca - (0.5 * TRIANGLE_SIZE) * sa;
  int left_tail_y  = cy + (-TRIANGLE_HWIDTH / 2) * sa + (0.5 * TRIANGLE_SIZE) * ca;
  int right_tail_x = cx + (TRIANGLE_HWIDTH / 2) * ca - (0.5 * TRIANGLE_SIZE) * sa;
  int right_tail_y = cy + (TRIANGLE_HWIDTH / 2) * sa + (0.5 * TRIANGLE_SIZE) * ca;
  int body_tail_x  = cx - TRIANGLE_SIZE / 2 * sa;
  int body_tail_y  = cy + TRIANGLE_SIZE / 2 * ca;

  backscreen.drawWideLine(left_wing_x, left_wing_y, right_wing_x, right_wing_y, 3, col);
  backscreen.drawWideLine(left_tail_x, left_tail_y, right_tail_x, right_tail_y, 2, col);
  backscreen.drawWideLine(cx, cy, body_tail_x, body_tail_y, 2, col);

  // ヨーが著しく信頼できるときだけ、機首の先へ点線を伸ばす。
  // TRACKUP では画面上＝トラックなので、この点線の傾きがそのまま偏流角になる。
  // NORTHUP では針（トラック）との開きが偏流角。
  // 前方は screen_deg=0 で画面上向き＝(+sa, -ca)。
  if (eskf_yaw_high_confidence()) {
    const int step = ESKF_DRIFT_LINE_DASH_PX + ESKF_DRIFT_LINE_SKIP_PX;  // 実線 + 空白
    for (int d = ESKF_DRIFT_LINE_GAP_PX;
         d < ESKF_DRIFT_LINE_GAP_PX + ESKF_DRIFT_LINE_LEN_PX; d += step) {
      int x0 = cx + (int)lroundf(sa * d);
      int y0 = cy - (int)lroundf(ca * d);
      int x1 = cx + (int)lroundf(sa * (d + ESKF_DRIFT_LINE_DASH_PX));
      int y1 = cy - (int)lroundf(ca * (d + ESKF_DRIFT_LINE_DASH_PX));
      backscreen.drawWideLine(x0, y0, x1, y1, 2, col);
    }
  }
}

// 自機アイコンを向けるべき画面上の角度 [度]（画面の上＝0、時計回り）。
// ESKF のヨーが信頼できるときだけ機首方位に合わせる。横風があるとトラックと機首は
// 偏流角のぶんずれるので、信頼できないヨーで回すと逆に誤解を招く。
// 信頼できないときは従来どおり対地進路（NORTHUP=ttrack、TRACKUP=画面上固定）。
static float plane_icon_screen_deg(int ttrack) {
  float y;
  if (!eskf_yaw_reliable() || !eskf_display_yaw(y))
    return is_trackupmode() ? 0.0f : (float)ttrack;
  return is_trackupmode() ? (y - (float)ttrack) : y;
}

// ヘッダー右の大きい枠が平均ピッチを出しているか（定義は draw_header の直前）。
static bool header_shows_pitch();

// TRACKUP 専用の半径縮小量 [px]。画面中央上部に "TT xxx" を出すようになり、
// 従来の半径（針の長さ nlen = 180）だと三角形の頂点が文字に届いてしまうため。
// STEER 側も一緒に縮めないと、内側へ寄った 30 度幅の三角形と誘導矢印が重なる。
#define TRACKUP_TONE_SHRINK   26
#define TRACKUP_STEER_SHRINK  24

void draw_triangle(int ttrack,int steer_angle) {
  // Core0 スタック残量を計測。
  // draw_triangle() は drawWideLine を最も多く呼ぶ関数であり、
  // TFT_eSPI の drawWideLine が大量のスタックを消費するため、
  // その直前がスタック消費のピーク（5 秒に 1 回出力）。
  DEBUG_STACK_C0("draw_triangle");
  float tt_radians = deg2rad(ttrack);
  const int nlen = get_needle_len();  // NORTHUP=120, TRACKUP=180
  if (upward_mode == MODE_NORTHUP) {
    // 最後のtrack音声が流れた時の、true track　方向を基準に、15度左・右・中央の3点を計算して三角形を描画する。
    // あくまでtrue trackの変化に気づくための表示であって、目標方位ではない。
    float tone_left = deg2rad(last_tone_tt-15);
    float tone_right = deg2rad(last_tone_tt+15);
    float tone_center = deg2rad(last_tone_tt);
    float x1 = (nlen-6) * sin(tone_left) + BACKSCREEN_SIZE/2;
    float y1 = (nlen-6) * -cos(tone_left) + BACKSCREEN_SIZE/2;
    float x2 = (nlen-6) * sin(tone_right) + BACKSCREEN_SIZE/2;
    float y2 = (nlen-6) * -cos(tone_right) + BACKSCREEN_SIZE/2;
    float x3 = (nlen) * sin(tone_center) + BACKSCREEN_SIZE/2;
    float y3 = (nlen) * -cos(tone_center) + BACKSCREEN_SIZE/2;
    if(trackwarning_until < millis()){
      backscreen.drawTriangle(x1,y1,x2,y2,x3,y3, COLOR_BLACK);
    }else{
      backscreen.fillTriangle(x1,y1,x2,y2,x3,y3, (millis()/1000)%2==0?COLOR_RED:COLOR_BLACK);
    }
    
    // 機体アイコン。NORTHUP は画面上＝北なので、そのまま方位を渡す。
    // ヨーが信頼できるときは機首方位、できないときは従来どおり ttrack で描かれる。
    draw_plane_icon(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2,
                    plane_icon_screen_deg(ttrack), plane_icon_color());


    if((millis()/1000)%3 != 0){
      float arc_factor = course_warning_index/900.0;
      if(steer_angle > 0)
        backscreen.drawArc(240/2, 240/2, (nlen-21), (nlen-23), (ttrack+180)%360, (ttrack+180+(int)(arc_factor*steer_angle))%360, COLOR_RED, COLOR_WHITE);
      else
        backscreen.drawArc(240/2, 240/2, (nlen-21), (nlen-23), (ttrack+180+(int)(arc_factor*steer_angle))%360,(ttrack+180)%360, COLOR_RED, COLOR_WHITE);

      //約10度以上の方位違いがある場合に、指示三角形を描画する。
      if(abs(steer_angle) > 15){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-0.1:0.1);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-0.35:0.35);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if(abs(steer_angle) > 55){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-0.7:0.7);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-0.95:0.95);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if(abs(steer_angle) > 100){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-1.3:1.3);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-1.55:1.55);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
    }
    
    // Needle (True Track)
    backscreen.drawWideLine(240/2+sin(tt_radians)*9, 240/2-cos(tt_radians)*9, 120+sin(tt_radians)*nlen, 120-cos(tt_radians)*nlen, 5,COLOR_BLACK);
    
  }
  if (upward_mode == MODE_TRACKUP) {
    // TRACKUP モードでは自機Y位置を画面下寄り（3/4）に表示する
    const int cy = BACKSCREEN_SIZE * 3 / 4;  // = 180（X は常に BACKSCREEN_SIZE/2 = 120）

    // ── 進行方向の縦線（常に画面上＝対地進路）──
    // TRACKUP では地図が回転して対地進路が常に画面上を向く。この線は進路そのものなので
    // 機首が振れても回さない（機首とのズレ＝偏流角がアイコンとの角度差で見える）。
    // 画面中央上部の "TT xxx"（NM_FONT_MEDIUM、高さ約24px）の下から始める。
    // ヘッダーがトラックを出しているときは中央の TT が無いので、その分だけ上へ伸ばせる。
    int shortening = header_shows_pitch() ? 28 : 16;
    backscreen.drawFastVLine(240 / 2,     shortening, cy - shortening - 5, COLOR_BLACK);
    backscreen.drawFastVLine(240 / 2 + 1, shortening, cy - shortening - 5, COLOR_BLACK);

    // ── 機体アイコン ──
    // ヨーが信頼できるときは (機首方位 - トラック) だけ傾けて実際の機首方向に合わせる。
    // 信頼できないときは 0（画面上向き）になり、従来の固定表示と同じになる。
    draw_plane_icon(BACKSCREEN_SIZE / 2, cy, plane_icon_screen_deg(ttrack), plane_icon_color());

    // ── True Track 維持針（last_tone_tt: 最後にトーンが鳴った時点のTT方向）──
    // TRACKUPでは機首=画面上=0°なので、last_tone_tt の画面上の角度は (last_tone_tt - ttrack)
    // 画面中央上部の "TT xxx" と重ならないよう、TRACKUP では半径を縮める
    const int tlen = nlen - TRACKUP_TONE_SHRINK;
    float screen_tone = last_tone_tt - ttrack;
    float tone_left   = deg2rad(screen_tone - 15);
    float tone_right  = deg2rad(screen_tone + 15);
    float tone_center = deg2rad(screen_tone);
    float tx1 = (tlen-6) * sin(tone_left)   + BACKSCREEN_SIZE/2;
    float ty1 = (tlen-6) * -cos(tone_left)  + cy;
    float tx2 = (tlen-6) * sin(tone_right)  + BACKSCREEN_SIZE/2;
    float ty2 = (tlen-6) * -cos(tone_right) + cy;
    float tx3 = tlen     * sin(tone_center) + BACKSCREEN_SIZE/2;
    float ty3 = tlen     * -cos(tone_center)+ cy;
    if (trackwarning_until < millis()) {
      backscreen.drawTriangle(tx1,ty1,tx2,ty2,tx3,ty3, COLOR_BLACK);
    } else {
      backscreen.fillTriangle(tx1,ty1,tx2,ty2,tx3,ty3, (millis()/1000)%2==0?COLOR_RED:COLOR_BLACK);
    }

    // ── コース逸脱アーク + 誘導三角形 ──
    // drawArcの座標系: 0°=6時方向（画面下）、時計回り。機首方向（画面上）= drawArc 180°
    // NORTHUPの (ttrack+180)%360 は方位角→drawArc角への変換。TRACKUPは機首固定なので 180 を使う。
    if ((millis()/1000)%3 != 0) {
      // 上で縮めた 30 度幅の三角形と重ならないよう、こちらも内側へ寄せる
      const int slen = nlen - TRACKUP_STEER_SHRINK;
      float arc_factor = course_warning_index / 900.0;
      if (steer_angle > 0)
        backscreen.drawArc(240/2, cy, (slen-21), (slen-23), 180, (180 + (int)(arc_factor*steer_angle)) % 360, COLOR_RED, COLOR_WHITE);
      else
        backscreen.drawArc(240/2, cy, (slen-21), (slen-23), (360 + 180 + (int)(arc_factor*steer_angle)) % 360, 180, COLOR_RED, COLOR_WHITE);

      // 誘導三角形: 機首=0ラジアン（画面上）を基準に offset。NORTHUPの tt_radians に相当
      if (abs(steer_angle) > 15) {
        double steer_triangle_start_rad = (steer_angle<0?-0.1:0.1);
        double steer_triangle_end_rad   = (steer_angle<0?-0.35:0.35);
        float x1 = (slen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (slen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (slen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (slen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (slen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (slen-22) * -cos(steer_triangle_end_rad) + cy;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if (abs(steer_angle) > 55) {
        double steer_triangle_start_rad = (steer_angle<0?-0.7:0.7);
        double steer_triangle_end_rad   = (steer_angle<0?-0.95:0.95);
        float x1 = (slen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (slen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (slen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (slen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (slen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (slen-22) * -cos(steer_triangle_end_rad) + cy;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if (abs(steer_angle) > 100) {
        double steer_triangle_start_rad = (steer_angle<0?-1.3:1.3);
        double steer_triangle_end_rad   = (steer_angle<0?-1.55:1.55);
        float x1 = (slen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (slen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (slen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (slen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (slen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (slen-22) * -cos(steer_triangle_end_rad) + cy;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
    }
  }
}



// A→B 方向の B 側を distance km 延長した点 C を計算する（球面上の Vincenty 近似）。
// B で見た A→B の方位を反転させて（+PI）B 基準の正確な方向を求める。
// 目的地の「手前」や「飛び越した先」の座標計算に使用する。
void calculatePointC(double lat1, double lon1, double lat2, double lon2, double distance, double& lat3, double& lon3) {
  const double R = 6371.0;  // Radius of the Earth in kilometers
  lat1 = deg2rad(lat1);
  lon1 = deg2rad(lon1);
  lat2 = deg2rad(lat2);
  lon2 = deg2rad(lon2);

  double d = distance / R;  // Distance in radians
  //Note  PointA and PointB should be reversed and PI be added in order to accurately calculate angle at pointB since bearing is not the same at pointA and pointB if they are far apart.
  //And we want the bearing at pointB.
  double bearing = calculateTrueCourseRad(lat2, lon2,lat1, lon1)+PI;
  lat3 = asin(sin(lat2) * cos(d) + cos(lat2) * sin(d) * cos(bearing));
  lon3 = lon2 + atan2(sin(bearing) * sin(d) * cos(lat2), cos(d) - sin(lat2) * sin(lat3));

  lat3 = rad2deg(lat3);
  lon3 = rad2deg(lon3);

}


// A から A→B 方向へ distance km 進んだ点 D を計算する（calculatePointC とは始点が異なる）。
// A と B が同一点の場合は方位が未定義のためスキップする。
// 目的地が画面外にある場合に画面端付近の仮想点を計算し、矢印を描画するために使う。
void calculatePointD(double lat1, double lon1, double lat2, double lon2, double distance, double& lat3, double& lon3) {
  const double R = 6371.0;  // Radius of the Earth in kilometers
  lat1 = deg2rad(lat1);
  lon1 = deg2rad(lon1);
  lat2 = deg2rad(lat2);
  lon2 = deg2rad(lon2);

  double d = distance / R;  // Distance in radians

  if (lat1 == lat2 && lon1 == lon2) {
    DEBUG_PLN(20250424,"Error: Points A and B are the same. Bearing is undefined.");
    return;
  }
  double bearing = calculateTrueCourseRad(lat1, lon1, lat2, lon2);

  lat3 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bearing));
  lon3 = lon1 + atan2(sin(bearing) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat3));


  lat3 = rad2deg(lat3);
  lon3 = rad2deg(lon3);

}

// 始点 (lat1,lon1) から方位 bearing_rad（北基準・時計回り・ラジアン）へ
// distance km 進んだ点を計算する。calculatePointD の「方位を外から与える版」。
// 2 点から方位を求めるのではなく方位そのものを指定したい場合（センターライン等）に使う。
void calculatePointFromBearing(double lat1, double lon1, double bearing_rad, double distance, double& lat2, double& lon2) {
  const double R = 6371.0;  // Radius of the Earth in kilometers
  lat1 = deg2rad(lat1);
  lon1 = deg2rad(lon1);

  double d = distance / R;  // Distance in radians

  lat2 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bearing_rad));
  lon2 = lon1 + atan2(sin(bearing_rad) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat2));

  lat2 = rad2deg(lat2);
  lon2 = rad2deg(lon2);
}

// FLYAWAY モード用の描画。目的地から「離れていく」方向への矢印を描く。
// 1. draw_flyinto() で目的地への基本矢印を描画する。
// 2. 目的地から「自機方向→その先」へのマゼンタ線を追加し、「飛び越した先」を示す。
void draw_flyawayfrom(double dest_lat,double dest_lon, double center_lat, double center_lon, float scale, float up) {
  draw_flyinto(dest_lat,dest_lon,center_lat,center_lon,scale,up,1);

  double lat3, lon3;
  cord_tft dest = latLonToXY(dest_lat, dest_lon, center_lat, center_lon, scale, up);
  double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
  calculatePointC(dest_lat, dest_lon, center_lat, center_lon, distance, lat3, lon3);
  cord_tft targetpoint = latLonToXY(lat3, lon3, center_lat, center_lon, scale, up);
  // 自機位置からtargetpoint（目的地と逆方向・画面外）へ太線を引く。
  // 細線は draw_flyinto() が目的地→自機間を描画済み。
  draw_clipped_wideline(BACKSCREEN_SIZE/2, get_self_cy(), targetpoint.x, targetpoint.y, 5, COLOR_MAGENTA);
}

// FLYINTO 拡張版。目的地が画面外の場合は仮想点を計算して矢印の終点に使う。
// 目的地が画面内の場合は目的地の「先」まで線を延長して通過線も描画する（draw_flyinto と異なる点）。
// 画面外オーバーフロー対策: goal の座標が int 範囲を超えると処理が重くなるため、
//   distance = 200/scale px 相当の仮想点を calculatePointD で求めて代替する。
void draw_flyinto2(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness) {
  cord_tft goal = latLonToXY(dest_lat, dest_lon, center_lat, center_lon, scale, up);
  if (goal.isOutsideTft()) {
    //scaleがとても大きい場合、goal.x,goal.yがオーバーフローする。
    //そのまま描画すると処理落ちするので、適度な座標を計算し直す必要がある。
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    calculatePointD(center_lat, center_lon, dest_lat, dest_lon, distance, newlat, newlon);
    goal = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
    draw_clipped_wideline(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
  }else{
    draw_clipped_wideline(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    //
    calculatePointC(dest_lat, dest_lon, center_lat, center_lon, -distance, newlat, newlon);
    cord_tft outside_tft = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
    draw_clipped_wideline(goal.x, goal.y, outside_tft.x, outside_tft.y, 2, COLOR_MAGENTA);
  }
}

// 自機位置（画面中央）から目的地への誘導線（マゼンタの太線）を描画する。
// 目的地が画面外の場合: calculatePointD で画面端付近の仮想点を求めて矢印の向きを示す。
// thickness パラメータで線の太さを調整できる（通常 = 1、強調時 = 5）。
void draw_flyinto(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness) {
  cord_tft goal = latLonToXY(dest_lat, dest_lon, center_lat, center_lon, scale, up);
  if (goal.isOutsideTft()) {
    //scaleがとても大きい場合、goal.x,goal.yがオーバーフローする。
    //そのまま描画すると処理落ちするので、適度な座標を計算し直す必要がある。
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    calculatePointD(center_lat, center_lon, dest_lat, dest_lon, distance, newlat, newlon);
    goal = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
  }
  draw_clipped_wideline(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
}


// ★グローバル変数として宣言（スタック節約のため）★
// Core0 のスタックは 4KB しかなく、MAX_TRACK_CORDS>=300 のときにローカル宣言すると
// スタックオーバーフローになるため、RAM（グローバル領域）に置いてある。
cord_tft points[MAX_TRACK_CORDS];

// LatLonManager（latlon_manager）に蓄積されたフライトトラックを backscreen に描画する。
// 最新座標（getData(0)）から自機位置（画面中央）への緑線を引き、
// その後 getData(i) → getData(i+1) を順に繋いでトラック履歴を表示する。
void draw_track(double center_lat, double center_lon, float scale, float up) {
  int sizetrack = latlon_manager.getCount();
  if(sizetrack <= 0){
    return;
  }
  //latest
  Coordinate c0 = latlon_manager.getData(0);
  cord_tft p0 = latLonToXY(c0.latitude, c0.longitude, center_lat, center_lon, scale, up);
  

  if(p0.x != BACKSCREEN_SIZE/2 || p0.y != get_self_cy()){
    draw_map_line(p0.x,p0.y,BACKSCREEN_SIZE/2,get_self_cy(),2,COLOR_GREEN);
  }

  for (int i = 0; i < sizetrack-1; i++) {
    Coordinate c0 = latlon_manager.getData(i);
    Coordinate c1 = latlon_manager.getData(i+1);
    if((c0.latitude == 0 && c0.longitude == 0) || (c1.latitude == 0 && c1.longitude == 0)){
      DEBUGW_PLN(20250510,"ERR lat lon 0");
      break;
    }
    
    cord_tft p0 = latLonToXY(c0.latitude, c0.longitude, center_lat, center_lon, scale, up);
    cord_tft p1 = latLonToXY(c1.latitude, c1.longitude, center_lat, center_lon, scale, up);
    //Only if cordinates are different.
    if (p0.x != p1.x || p0.y != p1.y) {
      draw_map_line(p0.x,p0.y,p1.x,p1.y,2,COLOR_GREEN);
    }
    
  }
}


// SD カードの mapdata.csv から読み込んだ追加ポリゴン（extramaps[]）を描画する。
// ポリゴンの最初の座標点が現在位置から 1°×1° 以内にある場合のみ描画する（遠方の不要描画を省略）。
// ポリゴンの描画色はマップ名の先頭文字で決まる:
//   r=赤, o=オレンジ, g=明るいグレー, m=マゼンタ, c=シアン, b=青, その他=緑
// 地図名の先頭 1 文字で描画色を決める。SD の mapdata.csv と内蔵ポリゴンで
// 同じ規則を使うため、ここに集約してある。
//   r=赤 / o=オレンジ / g=グレー / m=マゼンタ / c=シアン / b=青 / それ以外=緑
uint16_t mapdata_color(const char* name) {
  switch (name[0]) {
    case 'r': return COLOR_RED;
    case 'o': return COLOR_ORANGE;
    case 'g': return COLOR_BRIGHTGRAY;
    case 'm': return COLOR_MAGENTA;
    case 'c': return COLOR_CYAN;
    case 'b': return COLOR_BLUE;
    default:  return COLOR_GREEN;
  }
}

// フラッシュ内蔵のポリゴン地図（mapdatas[]）を描画する。
// SD の mapdata.csv 由来（draw_ExtraMaps）と同じ規則で描くが、
// こちらは SD が無くても・抜けても必ず表示される点が違う。
// 飛行に必須の注記はこちらに置くこと。
void draw_FlashMaps(double center_lat, double center_lon, float scale, float up) {
  for (int i = 0; i < flashmap_count; i++) {
    const mapdata* mp = flashmaps[i];
    if (mp->size <= 1) continue;
    // 現在地から離れた地図は描かない（extramaps と同じ 1 度四方の判定）
    if (!check_within_latlon(1, 1, mp->cords[0][1], get_gps_lat(),
                             mp->cords[0][0], get_gps_lon())) continue;
    draw_map(up, center_lat, center_lon, scale, mp, mapdata_color(mp->name));
  }
}

void draw_ExtraMaps(double center_lat, double center_lon, float scale, float up) {
  for (int i = 0; i < mapdata_count; i++) {
    if (extramaps[i].size <= 1) {
      continue;
    }
    double lon1 = extramaps[i].cords[0][0];
    double lat1 = extramaps[i].cords[0][1];
    if (check_within_latlon(1, 1, lat1, get_gps_lat(), lon1, get_gps_lon())) {
      int col = mapdata_color(extramaps[i].name);
      draw_map(up, center_lat, center_lon, scale, &extramaps[i], col);
        }
  }
}




// 起動アニメーションの地図背景。実際の地図と同じ描画関数を使う。
static void draw_startup_map(double lat, double lon, float scale) {
  draw_vectormap(lat, lon, scale, 0);
}

// backscreen の左下にソフトウェアバージョン文字列を、右上にバージョンテキストを描画する。
// 主に startup_demo_tft() のデモ画面で使用する。
void draw_version_backscreen(){
  // OpenStreetMap の帰属表示。ODbL 4.3 と OSMF Attribution Guidelines により、
  // 組み込みデバイスでも画面上への表示が必要（スプラッシュ画面での表示が認められている）。
  // 起動アニメーションの間ずっと表示されるので、この位置で要件を満たす。
  backscreen.unloadFont();
  backscreen.setTextColor(COLOR_GRAY);
  backscreen.setCursor(10, 240-34);
  backscreen.print("Map data (c) OpenStreetMap");
  backscreen.setCursor(10, 240-24);
  backscreen.print("contributors - ODbL 1.0");
  backscreen.setCursor(10, 240-12);
  backscreen.unloadFont();
  backscreen.setTextColor(COLOR_BLACK);
  backscreen.print("SOFTWARE:");
  backscreen.print(BUILDVERSION);
  backscreen.print("(Build ");
  backscreen.print(BUILDDATE);
  backscreen.print(")");

  backscreen.setCursor(160,0);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.println(VERSION_TEXT);
}

// Arduino の map() 関数の float 版。x を [in_min, in_max] から [out_min, out_max] に線形変換する。
// startup_demo_tft() のアニメーション（スケール・座標の補間）に使用する。
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 起動時のスプラッシュアニメーションを表示する。
// 1. ロゴ BMP を SD から読み込んで表示。SD 状態を画面下部に表示。
// 2. 琵琶湖マップをズームイン（遠→近）しながら PLA 位置に移動するアニメーション（約 40 フレーム）。
// 3. 続いてズームアウト（近→遠）しながら日本地図へ引くアニメーション。
// mapf() で中心座標とスケールを線形補間してスムーズなアニメーションを実現している。
void startup_demo_tft() {
  tft.fillScreen(COLOR_WHITE);
  tft.setTextColor(COLOR_RED, COLOR_WHITE);
  tft.setCursor(1, 0);
  tft.println(" Pilot Oriented");
  tft.setCursor(5, 14);
  tft.println("   Navigation System  for HPA");

  
  // Core1 の setup_sd() が完了するまで待機（最大5秒）。
  // Core1 が SD を初期化中に Core0 が SD にアクセスすると SDIO バス競合でフリーズする。
  {
    unsigned long t = millis();
    while (!sd_setup_complete && millis() - t < 5000) {
      gps_loop(7);  // 待機中も GPS FIFO を読み捨てて overflow 防止
      delay(10);
    }
  }


  if (good_sd()) {
    // ロゴ BMP 読み込みを Core1 に依頼する（SD アクセスは Core1 の責務）
    enqueueTask(createLoadLogoTask());    // SD 正常: 起動音を再生
    enqueueTask(createPlayWavTask("wav/opening.wav")); 
  } else {
    enqueueTask(createPlayMultiToneTask(500, 150, 10));    // SD エラー: 警告ビープ（500Hz を 10 回）
  }

  tft.setCursor(20, SCREEN_HEIGHT - 28);//320-292=28
  if(good_sd()){
    tft.setTextColor(COLOR_GREEN, COLOR_WHITE);
    tft.print("SD OK! MAP COUNT: ");
    tft.print(mapdata_count);
  }else{
    tft.setTextColor(COLOR_RED, COLOR_WHITE);
    tft.print("[ERROR: CHECK SD CARD !]");
  }
  // MS5611 / BNO085 接続状態（SD表示の1行下）
  tft.setCursor(10, SCREEN_HEIGHT - 16);
  tft.setTextColor(get_airdata_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  tft.print(get_airdata_ok() ? "MS5611:OK" : "MS5611:NG");
  tft.print("  ");
  tft.setTextColor(get_imu_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  tft.print(get_imu_ok() ? "BNO085:OK" : "BNO085:NG");
  float center_lat = 35.2334225841915;
  float center_lon = 136.091056306493;

  // 起動アニメーションは自機表示なし。TRACKUP 時の get_self_cy()=180 が適用されると
  // 地図が 60px 下にずれるため、アニメーション全体を強制的に NORTHUP 基準で描画する。
  // ※ここより前で sd_setup_complete 待ちが完了しているため upward_mode は SD 読み込み済みの値。
  int saved_upward_mode = upward_mode;
  upward_mode = MODE_NORTHUP;

  backscreen.fillScreen(COLOR_WHITE);
  draw_startup_map(center_lat, center_lon, 2.5);
  draw_pilon_takeshima_line(center_lat, center_lon, 2.5, 0);
  draw_pilon_takeshima_marks(center_lat, center_lon, 2.5, 0);
  draw_version_backscreen();
  backscreen.pushSprite(0,52);


  // Core1 がロゴ BMP を logo_sprite に読み込むのを待ち、完了したら pushSprite で表示する。
  // 残り時間は delay で消費して起動タイミングを維持する。
  {
    const int wait_timeout_ms = 1900;
    unsigned long wait_start = millis();
    while (!logo_ready && millis() - wait_start < wait_timeout_ms) {
      gps_loop(7);  // 待機中も GPS FIFO を読み捨てて overflow 防止
      delay(10);
    }
    if (logo_ready) {
      logo_sprite.pushSprite(0, 0);  // ロゴを画面上部 (0,0) に表示
      backscreen.pushSprite(0, 52);  // ロゴの黒エリア(y=52~239)で上書きされた琵琶湖地図を復元
      logo_ready = false;
    }
    while (millis() - wait_start < wait_timeout_ms) {
      gps_loop(7);  // 残り待機中も GPS FIFO を読み捨てて overflow 防止
      delay(10);
    }
  }

  float zoomin_speedfactor = 1.0f;
  int countermax = 40/zoomin_speedfactor;
  float scalenow = 0;
  for (int i = 0; i <= countermax; i++) {
    backscreen.fillScreen(COLOR_WHITE);
    scalenow = 2.5 + i * 0.25*zoomin_speedfactor;
    draw_startup_map(mapf(i,0,countermax,center_lat,pla_lat), mapf(i,0,countermax,center_lon,pla_lon), scalenow);
    draw_pilon_takeshima_line(mapf(i,0,countermax,center_lat,pla_lat), mapf(i,0,countermax,center_lon,pla_lon), scalenow, 0);
    draw_pilon_takeshima_marks(mapf(i,0,countermax,center_lat,pla_lat), mapf(i,0,countermax,center_lon,pla_lon), scalenow, 0);
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }
  delay(10);


  float zoomout_speedfactor = 1.0f;
  countermax = 40/zoomout_speedfactor;

  for (int i = 0; i < countermax; i++) {
    //scalenow *= 0.78;//for zoomout_speedfactor=2. 
    scalenow *= 0.89;// for zoomout_speedfactor=1
    if(scalenow < 0.08)
      scalenow = 0.08;
    else{
      center_lat += 0.005*i;// for zoomout_speedfactor=1
      //center_lat += 0.01*i;// for zoomout_speedfactor=2
    }

    bool islast = i == countermax-1;
    backscreen.fillScreen(COLOR_WHITE);
    {
      draw_startup_map(mapf(i,0,countermax,pla_lat,center_lat), mapf(i,0,countermax,pla_lon,center_lon), scalenow);
      draw_pilon_takeshima_line(mapf(i,0,countermax,pla_lat,center_lat), mapf(i,0,countermax,pla_lon,center_lon), scalenow, 0);
      draw_pilon_takeshima_marks(mapf(i,0,countermax,pla_lat,center_lat), mapf(i,0,countermax,pla_lon,center_lon), scalenow, 0);
    }
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }

  // OpenStreetMap の帰属表示（draw_version_backscreen が描いている）を
  // 読める時間だけ残す。OSMF の Attribution Guidelines はスプラッシュ画面での
  // 表示を認めているが、一瞬で消えると「easily readable」の要件を満たさない。
  // 最終フレームは既に表示済みなので、そのまま 2 秒保持する。
  {
    const unsigned long hold_ms = 3500;
    unsigned long t0 = millis();
    while (millis() - t0 < hold_ms) {
      gps_loop(7);  // 待機中も GPS FIFO を読み捨てて overflow 防止
      delay(10);
    }
  }

  upward_mode = saved_upward_mode;  // TRACKUP/NORTHUP 設定を復元
}
// デモ飛行中の通知ボックスを backscreen に描画する。
// 選択中の地点名を出すので、どの地点を表示しているか画面で分かる。
void draw_demo_biwako(){
  if((millis()/3000)%2 == 0){  // 6秒周期（3秒点灯・3秒消灯）で点滅
    backscreen.fillRect(5, 195, SCREEN_WIDTH-5*2, 15+5*2, COLOR_WHITE);
    backscreen.drawRect(5, 195, SCREEN_WIDTH-5*2, 15+5*2, COLOR_ORANGE);
    backscreen.setCursor(25,200);
    backscreen.setTextColor(COLOR_ORANGE);
    backscreen.print("DEMO: ");
    backscreen.print(get_demo_site_name(get_demo_site()));
  }
}


// リプレイモード中であることを backscreen に赤枠付きで通知する。
// draw_demo_biwako と同スタイル。画面上部に配置して常に視認できるようにする。
void draw_replay_indicator(){
  if((millis()/1000)%2 == 0){
    backscreen.fillRect(5, 195, SCREEN_WIDTH-5*2, 20, COLOR_WHITE);
    backscreen.drawRect(5, 195, SCREEN_WIDTH-5*2, 20, COLOR_RED);
    backscreen.drawRect(6, 196, SCREEN_WIDTH-5*2-2, 18, COLOR_RED);
    backscreen.setCursor(95,199);
    backscreen.setTextColor(COLOR_RED);
    backscreen.loadFont(AA_FONT_SMALL);
    backscreen.print("REPLAY");
  }
}

// backscreen スプライトを白でクリアする（毎フレームの描画前に呼ぶ）。
// VSIスプライト（5×240px）を鉛直速度に応じて描画する。
// 中心 Y=120 が 0 m/s の基準線（白）。上昇=緑バー、下降=シアンバー。
// 不感域なし。最大バー長 120px（±1.5 m/s で振り切り）。
// バーの上にグレー線（バリオ閾値 KF:±0.3 / MS5611単独:±0.6 m/s）と白線（±0.5 m/s, ±1.0 m/s）を重ねて描画。
void draw_vsi() {
    // BNO085 が使えるときは Kalman 融合済みの上昇率を使う。
    // BNO085 非接続時は MS5611 単独の上昇率にフォールバックする。
    float vspeed = get_imu_ok() ? get_imu_vspeed() : get_airdata_vspeed();
    vsi_sprite.fillScreen(TFT_BLACK);
    vsi_sprite.drawFastHLine(0, 120, 5, TFT_WHITE);  // 0 m/s 基準線
    if (!get_airdata_ok()) return;
    const float MAX_VSPEED = 1.5f;
    const int   BAR_MAX_PX = 120;
    if (vspeed > 0) {
        // 上昇: 中心から上方向に緑バー（1.5 m/s で振り切り）
        int bar = (int)(min(vspeed, MAX_VSPEED) / MAX_VSPEED * BAR_MAX_PX);
        vsi_sprite.fillRect(0, 120 - bar, 5, bar, TFT_GREEN);
    } else if (vspeed < 0) {
        // 下降: 中心から下方向にシアンバー（1.5 m/s で振り切り）
        int bar = (int)(min(-vspeed, MAX_VSPEED) / MAX_VSPEED * BAR_MAX_PX);
        vsi_sprite.fillRect(0, 121, 5, bar, TFT_CYAN);
    }
    // 閾値ラインをバーの上に重ねて描画（バーがなくても常に表示）
    // グレー線: バリオ音デッドバンド閾値。sound.cpp と同じ定数から px を計算するので、
    // 値を変えても線と音がずれない（以前は px を直書きしていて二重管理だった）。
    //   KF融合中(BNO085+MS5611): ±0.25 m/s = ±20px / MS5611単独: ±0.60 m/s = ±48px
    const float db_mps = (get_imu_ok() && get_airdata_ok()) ? VARIO_DEADBAND_KF_MPS
                                                            : VARIO_DEADBAND_BARO_MPS;
    const int db_px = (int)(db_mps / MAX_VSPEED * BAR_MAX_PX + 0.5f);
    vsi_sprite.drawFastHLine(0, 120 - db_px, 5, TFT_DARKGREY);
    vsi_sprite.drawFastHLine(0, 120 + db_px, 5, TFT_DARKGREY);
    // 白線: ±0.5 m/s = ±40px, ±1.0 m/s = ±80px  (v/1.5*120)
    vsi_sprite.drawFastHLine(0, 120 - 40, 5, TFT_WHITE);     // +0.5 m/s
    vsi_sprite.drawFastHLine(0, 120 + 40, 5, TFT_WHITE);     // -0.5 m/s
    vsi_sprite.drawFastHLine(0, 120 - 80, 5, TFT_WHITE);     // +1.0 m/s
    vsi_sprite.drawFastHLine(0, 120 + 80, 5, TFT_WHITE);     // -1.0 m/s
}

// backscreen スプライトを TFT の (0, 50) に転送する（ヘッダー 50px の下から表示）。
// 転送前に VSI を backscreen の右端 X=235 に合成する。
// ============================================================
// draw_eskf_attitude(): 地図上に姿勢を 3 行で重ねる
// ============================================================
// 背景は敷かず地図へ直接上書きする。リプレイ中は姿勢ログがある日のみ。
// 位置は左下。最下行(JST 時刻)のすぐ上を最下段とし、下から上へ
// Rtrim → R → P → 風/Y95 と積む。
// 平均ピッチはヘッダー右の大きい枠へ移した（同じ情報を 2 か所に出さない）。
// DEBUG_ESKF の有無でこの位置と大きさを変えないこと。
//
//   Avg**s P : 平均ピッチ。大きい字（NM_FONT_MEDIUM）＝一番見るべき値。
//   P / R    : 瞬時値。参考なので小さい字（AA_FONT_SMALL）。
//
// ロールは BANK_WARN_DEG を超えたら赤にする（バンク超過の気付き用）。
// 左端に寄せてあるのは、TRACKUP のとき自機アイコン（cy=180）の真下に
// draw_eskf_yaw() のヨー数値が入るため、中央付近を空けておく必要があるから。
// 自動ロールトリムの累積補正量の表示色。
// 正しく取り付いていれば実測上ほぼ 0 のままなので、大きな累積は
// 「取り付けか機体に異常があり、バンク警報の基準がずれている」ことを意味する。
// 表示は 8px と小さいが、色なら周辺視野でも気づける。
static uint16_t roll_trim_color(float trim) {
  const float a = fabsf(trim);
  if (a >= ROLL_TRIM_ALERT_DEG) return COLOR_RED;
  if (a >= ROLL_TRIM_WARN_DEG)  return COLOR_ORANGE;
  return COLOR_GRAY;
}

// 姿勢の値が「ありえない」範囲か（設置ミス・較正忘れの疑い）。
// true のとき数値を灰色にして、機体の姿勢としては信用できないことを示す。
static inline bool attitude_implausible(float deg) {
  return fabsf(deg) >= ATTITUDE_IMPLAUSIBLE_DEG;
}

// 風速に応じた矢印の色。しきい値は settings.h。
static uint16_t wind_arrow_color(float mps) {
  if (mps <= WIND_COL_LOW_MAX) return COLOR_GREEN;        // 濃い緑
  if (mps <= WIND_COL_MID_MAX) return COLOR_WIND_PURPLE;    // 青
  return COLOR_RED;                               // 紫
}


// 風ベクトルの矢印を (cx,cy) を中心に描く。長さは固定で、強さは色で表す。
// screen_deg: 画面の上を 0 とした時計回りの角度。前方は (+sin, -cos)。
// 矢印は「風が吹いていく向き」を指す（気象通報の風向とは逆なので注意）。
static void draw_wind_arrow(int cx, int cy, float screen_deg, uint16_t col) {
  const float a = deg2rad(screen_deg);
  const float ca = cosf(a), sa = sinf(a);
  const int h = WIND_ARROW_LEN_PX / 2;
  const int tipx  = cx + (int)lroundf(sa * h),  tipy  = cy - (int)lroundf(ca * h);
  const int tailx = cx - (int)lroundf(sa * h),  taily = cy + (int)lroundf(ca * h);
  backscreen.drawWideLine(tailx, taily, tipx, tipy, WIND_ARROW_WIDTH_PX, col);
  // 矢じり。先端から WIND_ARROW_HEAD_PX 手前を底辺の中心にする。
  const int hw = WIND_ARROW_HEAD_PX;
  const int bx = cx + (int)lroundf(sa * (h - hw)), by = cy - (int)lroundf(ca * (h - hw));
  const int px = (int)lroundf(ca * hw * 0.55f),    py = (int)lroundf(sa * hw * 0.55f);
  backscreen.fillTriangle(tipx, tipy, bx + px, by + py, bx - px, by - py, col);
}

#define ESKF_LABEL_X        2
#define ESKF_VALUE_RIGHT_X  50    // P / R の値の右端（ここへ右揃え。桁が揃う）
#define ESKF_ROW_GAP        1     // 各行の隙間 [px]
// 最下段(Rtrim, 内蔵フォント8px)の下端から最下行(JST 時刻)までの余白 [px]。
// sAcc と MaxGS を他所へ移して空いたので、ブロック全体を下へ寄せてある。
#define ESKF_BOTTOM_GAP     2
void draw_eskf_attitude() {
  // 機能そのものが OFF なら何も出さない（ESKF 導入前と同じ見た目に戻す）。
  if (!attitude_get_rpy_enabled()) return;

  // 機能は ON なのに姿勢が出せない状態。何も描かないと「表示が壊れた」のか
  // 「まだ準備中」なのか区別できないので、理由だけ 1 行出す。
  // 屋外で GNSS 速度が入れば 0.5 秒ほどで READY になるので、通常は一瞬しか見えない。
  // BNO085 非搭載機は姿勢機能自体が無いので対象外（出しっぱなしになってしまう）。
  if (!getReplayMode() && get_imu_ok() && !attitude_ready()) {
    backscreen.setTextWrap(false);
    backscreen.loadFont(AA_FONT_SMALL);
    const int fh = backscreen.fontHeight();
    backscreen.setTextColor(COLOR_ORANGE);
    backscreen.setCursor(ESKF_LABEL_X,
                         BACKSCREEN_SIZE - 9 - 8 - ESKF_BOTTOM_GAP - fh - ESKF_ROW_GAP);
    backscreen.print("ESKF Not ready");
    backscreen.setTextColor(COLOR_BLACK);
    return;
  }

  if (!eskf_display_enabled()) return;  // リプレイで姿勢ログが無い日

  float r, p, y = 0.0f;
  if (getReplayMode()) {
    if (!get_replay_attitude(r, p)) return;  // その日の姿勢ログが無い
  } else {
    attitude_get_euler(r, p, y);
  }

  char rbuf[8], pbuf[8];
  snprintf(rbuf, sizeof(rbuf), "%+5.1f", r);
  snprintf(pbuf, sizeof(pbuf), "%+5.1f", p);

  // 平均ピッチだけを NM_FONT_MEDIUM の大きい字にする。
  // 瞬時ピッチは周期 2〜6.7 秒の振動が主で std 1.23 度あり操縦の指標にならない。
  // 平均なら std 0.37 度まで落ちて 1 度の変化が有意に読める（窓長は PITCH_AVG_SEC）。
  // 瞬時の P / R は参考値なので AA_FONT_SMALL に落とす。
  // リプレイでは実機（机の上）の平均ではなく、記録された当時の値を出す。
  backscreen.setTextWrap(false);
  backscreen.loadFont(AA_FONT_SMALL);
  const int fh_s = backscreen.fontHeight();

  // 下から積む。最下段は Rtrim（内蔵フォント 8px）で、その下は JST 時刻の行。
  const int yrtrim = BACKSCREEN_SIZE - 9 - 8 - ESKF_BOTTOM_GAP;
  const int yroll  = yrtrim - fh_s - ESKF_ROW_GAP;                 // ロール
  const int ypit   = yroll  - fh_s - ESKF_ROW_GAP;                 // ピッチ

  // ---- P の上: ヨー精度（常時）と風ベクトル ----
  // 風は「対気速度の向き＝機首方位」を前提にするので、ヨーが著しく信頼できて、かつ
  // 直進が累計 WIND_LPF_SEC 秒たまるまで出ない。出ていない間も理由が分かるよう
  // ヨー精度の 95% 値は常に出す（風が出ているときも隠さない）。
  // リプレイでは記録された値をそのまま出す。
  {
    const int yacc95 = ypit - ESKF_ROW_GAP - fh_s;   // ヨー精度の行（P のすぐ上）

    float wspd = 0.0f, wdir = 0.0f;
    float acc95 = 0.0f;
    bool  has_wind, has_acc;
    has_acc = eskf_display_yaw_acc(acc95);
    if (is_demo_active()) {
      // デモは表示確認用。実機の推定条件（直進 10 秒など）を待たずに矢印を出す。
      has_wind = attitude_get_wind_enabled() && demo_wind(wspd, wdir);
    } else if (getReplayMode()) {
      has_wind = get_replay_wind(wspd, wdir);
    } else {
      has_wind = eskf_yaw_high_confidence() && attitude_get_wind_enabled() &&
                 attitude_get_wind(wspd, wdir);
    }

    // σ はフォント(NotoSansBold15)に無いので "Y95" と書く。意味は 95% 値（=2σ）。
    if (has_acc) {
      backscreen.setTextColor(COLOR_GRAY);
      backscreen.setCursor(ESKF_LABEL_X, yacc95);
      backscreen.printf("Y95%%%.0f\xC2\xB0", acc95);
    }

    int yblock_top = yacc95;
    if (has_wind) {
      // 数値だけ NM_FONT_MEDIUM。Arial_Black22 は数字と +-. しか持たないので、
      // "W" と "m/s" は AA_FONT_SMALL のまま描いて数字だけ大きくする。
      char nbuf[8];
      snprintf(nbuf, sizeof(nbuf), "%.1f", wspd);
      const int wlw = backscreen.textWidth("W");
      backscreen.loadFont(NM_FONT_MEDIUM);
      const int fh_m = backscreen.fontHeight();
      const int nw   = backscreen.textWidth(nbuf);
      const int ywtxt = yacc95 - ESKF_ROW_GAP - fh_m;
      const int ylbl  = ywtxt + (fh_m - fh_s) / 2;   // 小文字は数値の高さの中央へ
      backscreen.setTextColor(COLOR_BLACK);
      backscreen.setCursor(ESKF_LABEL_X + wlw + 3, ywtxt);
      backscreen.print(nbuf);
      backscreen.loadFont(AA_FONT_SMALL);
      backscreen.setCursor(ESKF_LABEL_X, ylbl);
      backscreen.print("W");
      backscreen.setCursor(ESKF_LABEL_X + wlw + 3 + nw + 2, ylbl);
      backscreen.print("m/s");

      // 矢印は地図と同じ向き合わせ（TRACKUP は画面上＝トラック、NORTHUP は北）
      const float wscreen = is_trackupmode()
                            ? (wdir - (float)get_gps_truetrack()) : wdir;
      draw_wind_arrow(ESKF_LABEL_X + WIND_ARROW_LEN_PX / 2 + 2,
                      ywtxt - ESKF_ROW_GAP - WIND_ARROW_LEN_PX / 2,
                      wscreen, wind_arrow_color(wspd));
      yblock_top = ywtxt - ESKF_ROW_GAP - WIND_ARROW_LEN_PX;
    }

    // ---- 較正のやり直しが必要（マウントから外された形跡がある）----
    // この状態では表示している姿勢そのものが信用できないので、設定画面まで
    // 行かないと分からないのでは遅い。地図上でも赤字で出す。
    if (attitude_needs_apply()) {
      backscreen.setTextColor(COLOR_RED);
      backscreen.setCursor(ESKF_LABEL_X, yblock_top - ESKF_ROW_GAP - fh_s);
      backscreen.print("ESKF CALIBRATION REQUIRED!");
      backscreen.setTextColor(COLOR_BLACK);
    }
  }

  // ---- 中段: P ----
  // ピッチは色を変えない。ピッチ単独では失速余裕を判定できないため
  // （本来は迎角が要る。実測でも 平均ピッチと対気速度の相関は -0.14 と弱く、
  //   迎角推定も対地経路角の 平均がほぼゼロで補正にならなかった）。
  backscreen.setTextColor(COLOR_GRAY);
  backscreen.setCursor(ESKF_LABEL_X, ypit);
  backscreen.print("P");
  backscreen.setTextColor(attitude_implausible(p) ? COLOR_GRAY : COLOR_BLACK);
  backscreen.setCursor(ESKF_VALUE_RIGHT_X - backscreen.textWidth(pbuf), ypit);
  backscreen.print(pbuf);

  // ---- 最下段: R ----
  backscreen.setTextColor(COLOR_GRAY);
  backscreen.setCursor(ESKF_LABEL_X, yroll);
  backscreen.print("R");
  // ありえない値なら灰色（設置ミスの疑い）。それ以外はバンク超過だけ赤。
  // 地図の上に直接描くので通常色は黒（フッターと同じ）。
  backscreen.setTextColor(attitude_implausible(r) ? COLOR_GRAY
                          : (fabsf(r) > BANK_WARN_DEG ? COLOR_RED : COLOR_BLACK));
  backscreen.setCursor(ESKF_VALUE_RIGHT_X - backscreen.textWidth(rbuf), yroll);
  backscreen.print(rbuf);
  backscreen.setTextColor(COLOR_BLACK);

  // ---- R の下: 自動ロールトリムの累積補正量（内蔵フォント 8px = 最小）----
  // どれだけ自動補正が入っているかを飛行中にも確認できるようにする。
  // リプレイでは記録された当時の値を出す（旧 euler 形式には無いので非表示）。
  float trim_val = 0.0f;
  bool  show_trim;
  if (getReplayMode()) show_trim = get_replay_roll_trim(trim_val);
  else { show_trim = true; trim_val = attitude_get_roll_trim_deg(); }
  if (show_trim) {
    backscreen.unloadFont();
    backscreen.setTextSize(1);
    backscreen.setTextColor(roll_trim_color(trim_val));
    backscreen.setCursor(ESKF_LABEL_X, yrtrim);
    backscreen.printf("Rtrim%+.2f", trim_val);
    backscreen.setTextColor(COLOR_BLACK);
  }

  // この後に描く コース警告 / [GS] [m/s] [MT] と同じフォントで抜けること。
  backscreen.loadFont(AA_FONT_SMALL);
}


// ============================================================
// draw_eskf_yaw(): 自機アイコンのすぐ下に ESKF のヨー角（真方位）を描く
// ============================================================
// フォントは [GS] [m/s] [MT] と同じ AA_FONT_SMALL。
// 色で信頼度を表す:
//   黒     = 信頼できる（95%値 < ESKF_YAW_TRUST_95_DEG）。このときアイコンも機首方向へ回る。
//   グレー = 信頼できない。アイコンは回らず対地進路（トラック）向きのまま。
// ロール・ピッチ行より後に呼ぶこと（TRACKUP では真下がその行に近いため）。
void draw_eskf_yaw() {
  if (!eskf_display_enabled()) return;

  // 旧 euler 形式のヨーは BNO085 由来で、実測で真方位から -30〜-65 度ずれていた。
  // 当時これを機首方位として表示していないので、リプレイでも再現しない
  // （eskf_display_yaw() が新形式のときだけ true を返す）。
  float y;
  if (!eskf_display_yaw(y)) return;

  char buf[8];
  snprintf(buf, sizeof(buf), "%03d", ((int)lroundf(y) % 360 + 360) % 360);

  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextWrap(false);
  backscreen.setTextColor(eskf_yaw_reliable() ? COLOR_BLACK : COLOR_GRAY);
  // アイコン（尾翼の下端は cy+TRIANGLE_SIZE/2）のすぐ下に中央揃えで置く
  backscreen.setCursor(BACKSCREEN_SIZE / 2 - backscreen.textWidth(buf) / 2,
                       get_self_cy() + TRIANGLE_SIZE / 2 + 3);
  backscreen.print(buf);
  backscreen.setTextColor(COLOR_BLACK);
}


#ifdef DEBUG_ESKF
// ============================================================
// draw_eskf_debug(): 比較用の詳細表示（DEBUG_ESKF 有効時のみ）
// ============================================================
// 姿勢推定の精度を実機で見比べるための評価用。地図上部に被せる。
// draw_eskf_attitude() が使う左下の領域には触れないこと。
//
// 見方: 旋回中・加減速中に ESKF と BNO085 が開いていれば、その差が BNO085 の誤差。
//   BNO085 は比力を鉛直とみなすため、定常旋回でロールを過小評価し、加減速でピッチがずれる。
// ヨーは σ（推定標準偏差）で信頼度を示す。水平加速度が無いと可観測にならず育つ。
void draw_eskf_debug() {
  const int x = 0, y = 2, w = 240, h = 46;

  backscreen.unloadFont();
  backscreen.fillRect(x, y, w, h, COLOR_BLACK);
  backscreen.drawRect(x, y, w, h, COLOR_GRAY);

  if (!attitude_ready()) {
    backscreen.setTextSize(2);
    backscreen.setTextColor(COLOR_YELLOW, COLOR_BLACK);
    backscreen.setCursor(x + 6, y + 6);
    backscreen.print("ESKF INIT...");
    backscreen.setTextSize(1);
    backscreen.setTextColor(COLOR_BRIGHTGRAY, COLOR_BLACK);
    backscreen.setCursor(x + 6, y + 28);
    backscreen.printf("static %.1fs / need %.1fs  %s",
                      attitude_get_static_secs(),
                      ESKF_STATIC_INIT_US / 1000000.0f,
                      attitude_is_static() ? "hold still" : "waiting");
    backscreen.loadFont(AA_FONT_SMALL);  // 内蔵フォントのまま抜けない
    return;
  }

  float er, ep, ey;  attitude_get_euler(er, ep, ey);
  float br, bp, by;  get_imu_euler(br, bp, by);
  float yacc = attitude_get_yaw_acc95_deg();   // 95%(2σ) 表記

  // 見出し
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_GRAY, COLOR_BLACK);
  backscreen.setCursor(x + 44,  y + 2); backscreen.print("ROLL");
  backscreen.setCursor(x + 110, y + 2); backscreen.print("PITCH");
  backscreen.setCursor(x + 182, y + 2); backscreen.print("YAW");

  // ESKF（ヨーだけ信頼度で色を変える。ロール・ピッチと同確度だと誤解させないため）
  backscreen.setTextColor(COLOR_CYAN, COLOR_BLACK);
  backscreen.setCursor(x + 2, y + 14); backscreen.print("ESKF");
  backscreen.setTextSize(2);
  backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK);
  backscreen.setCursor(x + 36,  y + 12); backscreen.printf("%+5.1f", er);
  backscreen.setCursor(x + 108, y + 12); backscreen.printf("%+5.1f", ep);
  backscreen.setTextColor(yacc < 10.0f ? COLOR_WHITE
                        : yacc < 40.0f ? COLOR_YELLOW : COLOR_GRAY, COLOR_BLACK);
  backscreen.setCursor(x + 186, y + 12); backscreen.printf("%4.0f", ey);

  // BNO085（比較用）
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_ORANGE, COLOR_BLACK);
  backscreen.setCursor(x + 2, y + 32); backscreen.print("BNO");
  backscreen.setTextColor(COLOR_BRIGHTGRAY, COLOR_BLACK);
  backscreen.setCursor(x + 36,  y + 32); backscreen.printf("%+6.1f", br);
  backscreen.setCursor(x + 108, y + 32); backscreen.printf("%+6.1f", bp);
  backscreen.setCursor(x + 186, y + 32); backscreen.printf("%4.0f", by);

  // ヨーの収束状態: M=地磁気で初期化済み / -=収束待ち、数値は 95%(2σ) の精度 [度]
  backscreen.setTextColor(COLOR_GRAY, COLOR_BLACK);
  backscreen.setCursor(x + 142, y + 32);
  backscreen.printf("%c%3.0f", attitude_yaw_from_mag() ? 'M' : '-', yacc);

  backscreen.setTextSize(1);
  // 内蔵フォントのまま抜けると以降の描画が崩れるので必ず戻す（draw_eskf_attitude と同様）
  backscreen.loadFont(AA_FONT_SMALL);
}
#endif // DEBUG_ESKF


// ============================================================
// IMU / 姿勢 ESKF 画面（2 ページ構成）
// ============================================================
// ページ1: 姿勢の比較と、機体ゼロ点の較正操作
// ページ2: センサーの生値・ESKF の内部状態（診断用）
//
// 操作: 短押し = カーソル移動 / 長押し = 実行（Exit 項目で設定画面へ戻る）
// 他の設定画面と同じ操作系に揃えてある。
extern int imu_cursor;   // GPS_TFT_map.ino で定義。ページ1のカーソル位置


// タイトル右端の状態ドット。緑になる条件は次の 3 つを全て満たすとき。
//   ・バンク警告が ON
//   ・自動ロールトリムが ON
//   ・自動トリムの累積が小さい（ROLL_TRIM_WARN_DEG 未満）
// 3 つ目を入れているのは、累積が大きい＝取り付けがずれていて
// バンク警報の基準が信用できない状態だから。飛行前点検で気づけるようにする。
// 右上の状態ドットが緑になる条件を 1 か所にまとめる。
// 各メニュー行の丸もここを参照するので、条件と表示が食い違わない。
//   緑になるのは次の 3 つを全て満たすとき:
//     1. ESKF が収束している（READY）        … ページ1の ESKF 行 / ページ2の READY 行
//     2. Roll/Pitch/Yaw func が ON          … ページ1
//     3. Roll trim during flight が ON      … ページ2
//     4. 自動トリムの累積が ROLL_TRIM_WARN_DEG 未満  … ページ1の Rtrim 表示
// 1 は「まだ姿勢が出せていない」状態なので、これを緑にすると準備完了と誤解する。
static bool imu_ok_ready() { return attitude_ready(); }
static bool imu_ok_rpy()   { return attitude_get_rpy_enabled(); }
static bool imu_ok_trim()  { return attitude_get_roll_trim_enabled(); }
static bool imu_ok_rtrim() { return fabsf(attitude_get_roll_trim_deg()) < ROLL_TRIM_WARN_DEG; }

// メニュー行の右端に状態の丸を描く（行の縦中央）。x は全行で揃える。
static void draw_imu_row_dot(int y, uint16_t col) {
  backscreen.fillCircle(232, y + 7, 4, col);
}
static uint16_t imu_row_col(bool ok) { return ok ? COLOR_GREEN : COLOR_ORANGE; }

static void draw_imu_status_dot() {
  // BNO085 非搭載機。姿勢機能そのものが無いので緑（＝準備OK）にはしない。
  if (!get_imu_ok()) { header_footer.fillCircle(228, 18, 6, COLOR_GRAY); return; }
  // マウントから外されたあと APPLY していない＝較正が無効。これだけは赤にする。
  // 姿勢の基準そのものが失われている状態で、他の設定の良し悪しより深刻なため。
  if (attitude_needs_apply() && attitude_get_rpy_enabled()) {
    header_footer.fillCircle(228, 18, 6, COLOR_RED);
    return;
  }
  const bool ok = imu_ok_ready() && imu_ok_rpy() && imu_ok_trim() && imu_ok_rtrim();
  header_footer.fillCircle(228, 18, 6, ok ? COLOR_GREEN : COLOR_ORANGE);
}

static void draw_imu_page1() {
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);
  header_footer.print("IMU / ESKF  1/2");
  draw_imu_status_dot();
  header_footer.pushSprite(0, -10);

  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextWrap(false);   // 長い項目名が 2 行に折り返さないように

  int y = 2;
  const int lh = 12;
  bool ready = attitude_ready();

  // ---- 姿勢の比較 ----
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y); backscreen.print("      ROLL   PITCH    YAW"); y += lh;
  float er, ep, ey, br, bp, by;
  attitude_get_euler(er, ep, ey);
  get_imu_euler(br, bp, by);
  backscreen.setTextColor(COLOR_BLUE, COLOR_WHITE);
  backscreen.setCursor(2, y);
  // 未初期化・IMU 途絶時は数値を出さない（単位クォータニオンのままだと
  // pitch=-90 というもっともらしい値が出て有効値と誤読される）
  if (ready) backscreen.printf("ESKF %+6.1f %+6.1f %6.1f", er, ep, ey);
  else       backscreen.print("ESKF   ---    ---    ---");
  draw_imu_row_dot(y - 2, imu_row_col(imu_ok_ready()));
  y += lh;
  backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("BNO  %+6.1f %+6.1f %6.1f", br, bp, by);
  y += lh;
  // 巡航のトリム状態は瞬時ピッチではなく 平均で見る（瞬時値は std 1.23 度の振動があるため）。
  // 直進中のロール自動トリムの累積量も併記して、どれだけ補正が入ったか分かるようにする。
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y);
  if (attitude_pitch_avg_valid())
    backscreen.printf("P avg%.0fs %+5.1f  ", PITCH_AVG_SEC, attitude_get_pitch_avg_deg());
  else
    backscreen.printf("P avg%.0fs   --    ", PITCH_AVG_SEC);
  // Rtrim は緑条件の 3 つ目。しきい値を超えていることが分かるよう色を変える。
  backscreen.setTextColor(imu_ok_rtrim() ? COLOR_GRAY : COLOR_ORANGE, COLOR_WHITE);
  backscreen.printf("Rtrim %+.2f", attitude_get_roll_trim_deg());
  draw_imu_row_dot(y - 2, imu_row_col(imu_ok_rtrim()));
  y += lh + 6;

  // ---- 注意書き ----
  // 色で意味を分ける。赤は「異常＝対処が要る」、青は「操作前に確認すること」。
  // Note: 〜 を赤にしていたが、エラーが起きているように見えるので青にした。
  backscreen.setCursor(2, y);
  if (attitude_needs_apply() && attitude_get_rpy_enabled()) {
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
    // マウントから外された形跡がある。較正をやり直すまで姿勢は信用できない。
    backscreen.print("!! REMOVED FROM MOUNT"); y += lh;
    backscreen.setCursor(2, y);
    backscreen.print("   APPLY required"); y += lh + 6;
  } else {
    backscreen.setTextColor(COLOR_BLUE, COLOR_WHITE);
    backscreen.print("Note: Make sure actual aircraft"); y += lh;
    backscreen.setCursor(2, y);
    backscreen.print("  attitude matches SET attitude"); y += lh + 6;
  }

  // ---- 操作メニュー ----
  // 項目数 6。8 項目のときは 15 まで詰めていたが、2 ページ目へ移して余裕ができた。
  const int mh = 17;
  bool on_ground = eskf_calib_allowed();
  for (int i = 0; i < IMU_MENU_COUNT; i++) {
    bool sel = (imu_cursor == i);
    backscreen.setCursor(2, y);
    backscreen.setTextColor(sel ? COLOR_MAGENTA : COLOR_BLACK, COLOR_WHITE);
    backscreen.print(sel ? ">" : " ");
    backscreen.setCursor(14, y);
    switch (i) {
      case IMU_MENU_SETPITCH:
        backscreen.printf("SET PITCH = %+.1f deg", attitude_get_pitch_target());
        break;
      case IMU_MENU_SETROLL:
        // 通常は 0。0 以外は「傾けたまま較正する」宣言なので目立たせる。
        // 0 以外は「傾けたまま較正する」宣言なので、カーソル位置でなくても目立たせる
        if (!sel && attitude_get_roll_target() != 0.0f)
          backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
        backscreen.printf("SET ROLL  = %+.1f deg", attitude_get_roll_target());
        break;
      case IMU_MENU_APPLY:
        // 実行可否をそのまま出す。飛行中は通さない。
        if (attitude_calib_pending())
                             backscreen.print("APPLY  ... hold still");
        else if (!ready)     backscreen.print("APPLY  (ESKF not ready)");
        else if (!on_ground) backscreen.print((getReplayMode() || is_demo_active())
                                              ? "APPLY  (hold still)" : "APPLY  (moving)");
        else                 backscreen.print("APPLY above=calibrate now");
        // 較正が必要なときは赤丸。画面右上の赤丸だけでは「何をすれば消えるのか」が
        // 分からないので、対処すべき項目に同じ色の印を並べて対応付ける。
        if (attitude_needs_apply() && attitude_get_rpy_enabled())
          draw_imu_row_dot(y, COLOR_RED);
        break;
      case IMU_MENU_RPYFUNC:
        // 姿勢表示・各種警報・自動トリム・風推定・姿勢ログを一括で ON/OFF する
        backscreen.printf("Roll/Pitch/Yaw func: %s",
                          attitude_get_rpy_enabled() ? "ON " : "OFF");
        draw_imu_row_dot(y, imu_row_col(imu_ok_rpy()));
        break;
      case IMU_MENU_NEXTPAGE:
        backscreen.print("Next page (Detail) >");
        // ページ2 で見る項目の状態をここに集約して見せる。
        // 緑にならない原因がページ2 にあるとき、ページ1 だけ見ても分かるようにする。
        draw_imu_row_dot(y, imu_row_col(imu_ok_trim() && imu_ok_ready()));
        break;
      case IMU_MENU_EXIT:     backscreen.print("Exit >>");              break;
    }
    y += mh;
  }

  // ---- 現在保存されているオフセット ----
  y += 4;
  float lr, lp;
  attitude_get_level_offset(lr, lp);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("stored offset  R%+.1f  P%+.1f", lr, lp);

  backscreen.unloadFont();
  backscreen.pushSprite(0, 40);
}


// ページ2: センサー生値と ESKF 内部状態（診断用）
static void draw_imu_page2() {
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);
  header_footer.print("IMU / ESKF  2/2");
  draw_imu_status_dot();
  header_footer.pushSprite(0, -10);

  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

  int y = 2;
  const int lh = 12;
  const float R2D = 180.0f / (float)M_PI;
  float g[3], a[3];
  get_imu_raw_gyro(g);
  get_imu_raw_accel(a);
  float gn = sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]) * R2D;
  float an = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);

  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y); backscreen.print("-- Gyro raw [deg/s] --"); y += lh;
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("X%+7.3f Y%+7.3f Z%+7.3f", g[0]*R2D, g[1]*R2D, g[2]*R2D); y += lh;
  backscreen.setCursor(2, y);
  // 静止中ならこの大きさが残留バイアス＋ノイズ。BNO085 は校正済みなので本来小さい。
  backscreen.setTextColor(gn < 0.5f ? COLOR_GREEN : COLOR_ORANGE, COLOR_WHITE);
  backscreen.printf("|w| %.3f deg/s", gn); y += lh;

  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y); backscreen.print("-- Accel raw [m/s2] --"); y += lh;
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("X%+7.3f Y%+7.3f Z%+7.3f", a[0], a[1], a[2]); y += lh;
  backscreen.setCursor(2, y);
  backscreen.printf("|a| %.3f  (g=9.807)", an); y += lh + 2;

  bool st = attitude_is_static();
  backscreen.setTextColor(st ? COLOR_GREEN : COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("STATIC: %s  %.1fs", st ? "YES" : "no ", attitude_get_static_secs());
  y += lh + 2;

  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y); backscreen.print("-- ESKF --"); y += lh;
  bool ready = attitude_ready();
  backscreen.setTextColor(ready ? COLOR_GREEN : COLOR_ORANGE, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("%s   GNSS upd:%lu", ready ? "READY" : "INIT...",
                    (unsigned long)attitude_get_gnss_updates());
  backscreen.fillCircle(232, y + 6, 4, imu_row_col(imu_ok_ready()));
  y += lh;

  float bg[3], ba[3];
  attitude_get_gyro_bias(bg);
  attitude_get_accel_bias(ba);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.setCursor(2, y);
  // 上限（1.0deg/s）に張り付いていたら初期姿勢誤差を吸っている疑い
  backscreen.printf("bg %+5.2f %+5.2f %+5.2f d/s", bg[0]*R2D, bg[1]*R2D, bg[2]*R2D); y += lh;
  backscreen.setCursor(2, y);
  backscreen.printf("ba %+5.2f %+5.2f %+5.2f m/s2", ba[0], ba[1], ba[2]); y += lh + 2;

  // ヨーの信頼度。水平加速度がある間しか可観測にならず、等速直進では育つ。
  // 95%(2σ) で表記する。1σ のままだと「これ以下なら安心」と誤読されやすい。
  float yacc = attitude_get_yaw_acc95_deg();
  backscreen.setTextColor(yacc < 10.0f ? COLOR_GREEN
                        : yacc < 40.0f ? COLOR_ORANGE : COLOR_RED, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("yaw acc +-%.0f deg(95%%) init:%s",
                    yacc, attitude_yaw_from_mag() ? "MAG" : "none");
  y += lh + 2;

  float lr, lp;
  attitude_get_level_offset(lr, lp);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.printf("level offset R%+.1f P%+.1f", lr, lp); y += lh + 4;

  // ---- 調整メニュー ----
  // 日常の運用では触らない項目。実飛行での検証がまだなので、片方だけ切って
  // 切り分けられるよう残してある（ページ1のマスタースイッチとは別物）。
  y += 2;
  const int mh2 = 15;
  for (int i = 0; i < IMU2_MENU_COUNT; i++) {
    const bool sel = (imu2_cursor == i);
    backscreen.setCursor(2, y);
    backscreen.setTextColor(sel ? COLOR_MAGENTA : COLOR_BLACK, COLOR_WHITE);
    backscreen.print(sel ? ">" : " ");
    backscreen.setCursor(14, y);
    switch (i) {
      case IMU2_MENU_AUTOROLL:
        backscreen.printf("Roll trim during flight: %s",
                          attitude_get_roll_trim_enabled() ? "ON " : "OFF");
        backscreen.fillCircle(232, y + 6, 4, imu_row_col(imu_ok_trim()));
        break;
      case IMU2_MENU_WIND:
        backscreen.printf("Wind estimate: %s",
                          attitude_get_wind_enabled() ? "ON " : "OFF");
        break;
      case IMU2_MENU_BACK:
        backscreen.print("< Back to page 1");
        break;
    }
    y += mh2;
  }

  backscreen.unloadFont();
  backscreen.pushSprite(0, 40);
}


void draw_imudetail(int page) {
  if (page % 2 == 0) draw_imu_page1();
  else               draw_imu_page2();
}




void push_backscreen(){
  TIMING_START(push_bs);
  if (vario_volume > 0 && !vario_inhibit) {
    draw_vsi();
    vsi_sprite.pushToSprite(&backscreen, 235, 0);  // Sprite→Sprite 合成
  }
  backscreen.pushSprite(0, 50);
  TIMING_END(ts_push_backscreen, push_bs);
}

// backscreen の最上部に小さなテキスト（GS・m/s・TT ラベル）を描画する。
// trackwarning_until が有効な間は「TT」ラベルを赤/黒で点滅させてコース警告を示す。
// TT = True Track。機体に磁気コンパスを載せない方針になったので、表示は全て真方位。
// 画面に出す真方位の 3 桁表示値。ヘッダーと地図内で必ず同じ値になるよう
// 1 か所に集約する（以前ヘッダーは切り捨て、地図内は四捨五入で 1 度ずれていた）。
// 四捨五入した上で 0〜359 に丸め込む（359.7 度 → 360 ではなく 000）。
static int truetrack_display_deg() {
  return ((int)lroundf(get_gps_truetrack()) % 360 + 360) % 360;
}

void draw_gs_track(){
  backscreen.setTextWrap(false);
  // ヘッダー右の大きい枠が何を出しているかに合わせてラベルを変える。
  // 同じ位置に別の量が出るので、ラベル無しでは機体間で取り違える危険がある。
  const bool hdr_pitch = header_shows_pitch();
  const char* hdr_label = hdr_pitch ? "Pavg" : "TT";
  const int   hdr_lx    = SCREEN_WIDTH - (hdr_pitch ? 52 : 40);
  // コース警報の点滅はトラック表示のときだけ（ピッチ表示中は無関係なので点滅させない）
  if(trackwarning_until > millis() && !hdr_pitch){
    backscreen.setTextColor((millis()/1000)%2==0?COLOR_RED:COLOR_BLACK, TFT_WHITE);
    backscreen.drawString(hdr_label, hdr_lx, 1);
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
  }else{
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
    backscreen.drawString(hdr_label, hdr_lx, 1);
  }
  backscreen.drawString("GS", 1, 1);

  // "m/s" はヘッダーへ縦書き("m p s")で移動した（draw_header 参照）。
  // 空いた画面中央上部に真方位を出す。ヘッダーがピッチを出しているときだけで、
  // ヘッダーが真方位のときは重複するので出さない。
  // NORTHUP でも出す（地図が回らないぶん数値の方位が要るため）。
  // 数値は NM_FONT_MEDIUM。ただし Arial_Black22 は数字と +-. しか持たないので、
  // ラベル "TT" は AA_FONT_SMALL のまま描いて数字だけ大きくする。
  if (hdr_pitch) {
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
    const int lw = backscreen.textWidth("TT");
    const int lh = backscreen.fontHeight();
    char buf[8];
    snprintf(buf, sizeof(buf), "%03d", truetrack_display_deg());
    backscreen.loadFont(NM_FONT_MEDIUM);
    const int nw = backscreen.textWidth(buf);
    const int nh = backscreen.fontHeight();
    const int x0 = (SCREEN_WIDTH - (lw + 4 + nw)) / 2;   // ラベル＋数値で中央寄せ
    backscreen.setCursor(x0 + lw + 4, 1);
    backscreen.print(buf);
    backscreen.loadFont(AA_FONT_SMALL);
    backscreen.setCursor(x0, 1 + (nh - lh) / 2);         // ラベルは数値の高さの中央
    backscreen.print("TT");
  }

  // sAcc（速度精度）は GS のすぐ右。下に置くと旋回角速度(deg/s)の表示と重なるため。
  // 常時見るものではないので内蔵フォントの最小サイズ(8px)で控えめに出す。
  {
    // 幅と高さは AA_FONT_SMALL が読み込まれているうちに取る（unloadFont の後だと
    // 内蔵フォントの値になってしまい、中央合わせが効かない）。
    const int gsw = backscreen.textWidth("GS");
    const int gsh = backscreen.fontHeight();
    const float sacc_mps = get_gps_sacc_mmps() / 1000.0f;
    backscreen.unloadFont();
    backscreen.setTextSize(1);
    // 背景は塗らない（1引数の setTextColor）。地図の上に白い箱が出ないようにする。
    backscreen.setTextColor((sacc_mps < 1.0f) ? COLOR_GREEN
                          : (sacc_mps < 3.0f) ? COLOR_ORANGE : COLOR_RED);
    // GS(AA_FONT_SMALL) の行の高さの中央に合わせる
    backscreen.setCursor(1 + gsw + 4, 1 + (gsh - 8) / 2);
    backscreen.printf("sAcc%.2f", sacc_mps);
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
    backscreen.loadFont(AA_FONT_SMALL);   // 呼び出し側は地図用フォントを期待している
  }
}






// ===== バッテリー監視変数 =====
unsigned long last_maxadr_time = 0;           // max_adreading が最後に更新された時刻
int max_adreading = 0;                        // AD 読み値のピーク保持値（ノイズ対策のためピーク追跡）
unsigned long last_battery_warning_time = 0;  // 最後のバッテリー低下警告を再生した時刻

// バッテリー入力電圧を読み取って返す（単位: V、上限 4.3V）。
// ピーク追跡方式でノイズを除去する:
//   - 急激な低下（200カウント以上）はリセット（USB が外れた瞬間の値落ち対策）。
//   - 上昇時はすぐにピークを更新し、時刻を記録する。
//   - ピーク更新から 3 秒経過したら指数移動平均（α=0.2）でゆっくり追従させる。
// BATTERY_MULTIPLYER() マクロで AD 値を電圧に変換している（settings.h 参照）。
float get_input_voltage(){
  int adreading = analogRead(BATTERY_PIN);
  if (adreading < max_adreading - 200) {//急激に低下=USBが外れた時。 0.4V相当。
    max_adreading = adreading;
  }
  else if (adreading > max_adreading) {
    DEBUG_P(20250508,"bat");
    DEBUG_PLN(20250508,BATTERY_MULTIPLYER(max_adreading));
    max_adreading = adreading;
    last_maxadr_time = millis();
  } else if (millis() - last_maxadr_time > 3000L) {
    max_adreading += (adreading - max_adreading) * 0.2;
  }
  return min(BATTERY_MULTIPLYER(max_adreading),4.3);
}

// 旋回角速度（deg/s）を backscreen の左端または右端に表示する。
// - 0.5 deg/s 未満は非表示。
// - 正（右旋回）: 右端に表示。負（左旋回）: 左端に表示。
// - GPS 速度 2.0 m/s 以上のとき色分け: 1 deg/s 超=緑（適切な右旋回）、-1 deg/s 未満=青（左旋回）。
// - 3 deg/s 超: 背景を塗りつぶして強調（背景色 = 旋回方向色）。
// - 4 deg/s 超: さらに赤枠で警告強調。
void draw_degpersec(double degpersecond){
  if(abs(degpersecond) < 0.5)
    return;
  backscreen.loadFont(AA_FONT_SMALL);  // Must load the font first
  int col = COLOR_GRAY;
  if(get_gps_mps() > 2.0){
    if (degpersecond > 1.0)
      col = COLOR_GREEN;
    if (degpersecond < -1.0)
      col = COLOR_BLUE;
    if(abs(degpersecond) > 3.0){
      backscreen.setTextColor(COLOR_WHITE,col);
      if(degpersecond < 0)
        backscreen.fillRect(0, 15, 48, 35, col);
      else
        backscreen.fillRect(SCREEN_WIDTH-48, 15, 48, 35, col);
      
      if(abs(degpersecond) > 4.0){
        if(degpersecond > 0){
          backscreen.drawRect(SCREEN_WIDTH-48, 15, 48, 35, COLOR_RED);
          backscreen.drawRect(SCREEN_WIDTH-47, 16, 46, 33, COLOR_RED);
        }else{
          backscreen.drawRect(0, 15, 48, 35, COLOR_RED);
          backscreen.drawRect(1, 16, 46, 33, COLOR_RED);
        }
      }
    }else{
      backscreen.setTextColor(col);
    }
  }else{
    backscreen.setTextColor(col);
  }
  int posx;
  if(degpersecond > 0)
    posx = SCREEN_WIDTH - 45;
  else
    posx = 5;
  backscreen.setTextWrap(false);
  backscreen.setCursor(posx, 17);
  backscreen.printf("%+.1f",degpersecond);
  backscreen.setCursor(posx - 2, 31);
  backscreen.print("deg/s");
}


// ヘッダー右の大きい枠に平均ピッチを出すか（出さないときは従来どおり真方位）。
// BNO085 非搭載機・機能 OFF・平均がまだ溜まっていない間は真方位のまま。
static bool header_shows_pitch() {
  // リプレイでは imu_replaydata に記録された平均ピッチを再現する
  // （実機の平均ではなく当時の画面を出す。地図側の表示を廃止したのでここが唯一の出口）。
  if (getReplayMode()) { float a; return get_replay_pitch_avg(a); }
  return get_imu_ok() && attitude_get_rpy_enabled() && attitude_pitch_avg_valid();
}

// ヘッダーに出す平均ピッチの値。リプレイでは記録値。
static float header_pitch_value() {
  if (getReplayMode()) { float a = 0.0f; get_replay_pitch_avg(a); return a; }
  return attitude_get_pitch_avg_deg();
}

// ヘッダー右に平均ピッチを描く。
// NM_FONT_LARGE(Arial_Black56) は数字と小数点しか持っておらず、符号を出せない。
// また字送りが 数字37px / 点19px / プラス38px(22ptからの換算) で、
// "+3.5" を字形だけで組むと 131px となり枠(111px)に入らない。
// そこで符号と小数点だけ塗り矩形で描き、数字だけフォントに任せる。
//   符号25 + 数字37 + 点10 + 数字37 = 109px（枠 111px をほぼ使い切る）
// 絶対値が 10 度以上なら小数を落として "±dd"（25+37+37 = 99px）にする。
// 符号の枠を 18→25px に広げて横棒を長くしてある。プラスとマイナスは横棒の有無だけで
// 見分けるので、棒が短いと遠目に区別できない。
static void draw_header_pitch(int x, float deg, uint16_t col) {
  const int SIGN_W = 25, DIGIT_W = 37, DOT_W = 10;
  const int ytop = 3;               // 数字の描画開始 y（従来の方位表示と同じ）
  const bool neg = (deg < 0.0f);
  float a = fabsf(deg);
  if (a > 99.9f) a = 99.9f;

  // 符号。マイナスは横棒、プラスは十字。数字の高さの中央に合わせる。
  const int bar_w = 21, bar_t = 7, cx = x + SIGN_W / 2, cy = ytop + 21;
  header_footer.fillRect(cx - bar_w/2, cy - bar_t/2, bar_w, bar_t, col);
  if (!neg) header_footer.fillRect(cx - bar_t/2, cy - bar_w/2, bar_t, bar_w, col);
  x += SIGN_W;

  header_footer.setCursor(x, ytop);
  if (a < 10.0f) {
    const int i = (int)a, f = (int)((a - i) * 10.0f + 0.5f);
    header_footer.printf("%d", i);
    x += DIGIT_W;
    header_footer.fillRect(x + 2, ytop + 36, 6, 6, col);   // 小数点
    x += DOT_W;
    header_footer.setCursor(x, ytop);
    header_footer.printf("%d", f > 9 ? 9 : f);
  } else {
    header_footer.printf("%02d", (int)(a + 0.5f));
  }
}

// 画面上部 50px のヘッダーを描画して TFT の (0, 0) に転送する。
// 左: 対地速度 m/s（GPS 速度 < 2m/s のときは灰色）。
// 右半分: 真方位 3 桁。コース警告中は赤/白反転で点滅する。
// GNSS 衛星数 = 0 のときは赤い横線で「GPS 無効」を示す。
// 区切り縦線（mtvx）でスピードエリアと方位エリアを分けている。
//Height 50px
void draw_header() {
  TIMING_START(hdr);
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextWrap(false);
  header_footer.loadFont(NM_FONT_LARGE);  // Must load the font first
  int sel_col = get_gps_mps()<2?COLOR_GRAY:COLOR_BLACK;
  header_footer.setTextColor(sel_col, TFT_WHITE);


  header_footer.setCursor(-1, 3);
  // 100m/s以上（3桁）は小数点を省略して整数表示（右側のMT表示との重なり防止）
  float _mps = get_gps_mps();
  header_footer.printf(_mps >= 100.0f ? "%4.0f" : "%4.1f", _mps);


  const int mtvx = SCREEN_WIDTH - 111;
  header_footer.drawFastVLine(mtvx, 0, HEADERFOOTER_HEIGHT, COLOR_GRAY);

  // 速度の単位を縦書き("m","p","s")で中央縦線のすぐ左に置く。
  // 横書き "m/s" を地図面に置いていたが、姿勢表示の場所を確保するため
  // ヘッダーの余白へ移した。右下寄せ（縦線より左）。
  {
    header_footer.loadFont(AA_FONT_SMALL);
    header_footer.setTextColor(COLOR_GRAY, COLOR_WHITE);
    const int uh = header_footer.fontHeight()-2;  // 文字の高さ（縦書きの行間に使う）
    static const char* const kUnit[3] = {"m", "p", "s"};
    for (int i = 0; i < 3; i++) {
      const int w = header_footer.textWidth(kUnit[i]);
      header_footer.setCursor(mtvx - 2 - w - i*2, HEADERFOOTER_HEIGHT - 1 - uh * (3 - i));
      header_footer.print(kUnit[i]);
    }
    header_footer.loadFont(NM_FONT_LARGE);   // 方位の3桁表示へ戻す
  }
  if (header_shows_pitch()) {
    // ---- 平均ピッチ ----
    // パイロットは対地進路を地図から読めるが、平均ピッチは他に得る手段が無い。
    // 一番目立つ枠はこちらに使う（パイロットからのフィードバックによる）。
    // 対地速度が遅いと sel_col が灰色になるが、それは速度表示の都合。
    // ピッチの信頼性とは無関係なので巻き込まない。灰色にするのは「ありえない値」のときだけ。
    const float pv = header_pitch_value();
    const uint16_t pcol = attitude_implausible(pv) ? COLOR_GRAY : COLOR_BLACK;
    header_footer.setTextColor(pcol, TFT_WHITE);
    draw_header_pitch(mtvx, pv, pcol);
  } else {
    // ---- 真方位 ----
    header_footer.setCursor(mtvx, 3);
    if(trackwarning_until > millis()){
      if((millis()/1000)%2==0){
        header_footer.setTextColor(COLOR_RED, TFT_WHITE);
      }else{
        header_footer.fillRect(mtvx,0,111,3,COLOR_RED);
        header_footer.setTextColor(COLOR_WHITE, TFT_RED,true);
      }
      header_footer.printf("%03d", truetrack_display_deg());
      header_footer.setTextColor(sel_col, TFT_WHITE);
    }else{
      header_footer.setTextColor(sel_col, TFT_WHITE);
      header_footer.printf("%03d", truetrack_display_deg());
    }
  }

  header_footer.drawFastHLine(0,49,240,COLOR_BLACK);

  if(get_gps_numsat() == 0){
    // この赤線は「GNSS が無いので値が無効」を示すもの。対地速度と真方位は無効になるが、
    // 平均ピッチは GNSS が無くても（ESKF が生きていれば）意味を持つので線を引かない。
    const int len = header_shows_pitch() ? (mtvx - 5 - 2) : (SCREEN_WIDTH - 10);
    for(int i = 0; i < 4; i++)
      header_footer.drawFastHLine(5, 22+i, len, COLOR_RED);
  }

  header_footer.pushSprite(0,0);
  TIMING_END(ts_draw_header, hdr);
}

extern float course_warning_index;


// backscreen の最下部に地図サポート情報を描画する（地図モード専用フッター）。
// 最下行に JST 時刻と Max G/S を横に並べるだけ。sAcc は左上（GS の下）へ移した。
// 緯度・経度は v0.94 で削除した（飛行中に読む場面が無く、姿勢表示の場所を圧迫していたため）。
void draw_map_footer(){
  //JST Time
  GpsTime time = get_gpstime();
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_BLACK);

  if (time.isValid()) {
    backscreen.setCursor(1, BACKSCREEN_SIZE-9);
    backscreen.printf("%02d:%02d:%02dJST", (time.hour()+9)%24, time.minute(), time.second());
  }

  // 最大 G/S（JST 時刻のすぐ右）。
  // 5分保持と全時間が同値の場合は片側のみ表示。異なる場合は両方表示。
  backscreen.setCursor(75, BACKSCREEN_SIZE-9);
  if (get_maxgs_5min() == get_maxgs()) {
    backscreen.printf("MaxGS %.1f(%02d:%02d)",
      get_maxgs(), get_maxgs_hour(), get_maxgs_min());
  } else {
    backscreen.printf("MaxGS%.1f(%02d:%02d)%.1f(%02d:%02d)",
      get_maxgs_5min(), get_maxgs_5min_hour(), get_maxgs_5min_min(),
      get_maxgs(),      get_maxgs_hour(),      get_maxgs_min());
  }
  backscreen.loadFont(AA_FONT_SMALL);
}

// 画面下部 30px のフッターを描画して TFT の (0, 290) に転送する（HEIGHT: 30px）。
// 左: TC（真方位コース）と目的地までの距離 km。その下にナビモード名と目的地名。
// 右: バッテリー電圧（USB 接続時は "USB"。低電圧時は赤点滅 + 音声警告 60 秒ごと）。
// 右中: GNSS 衛星数（5 未満=赤背景、10 未満=オレンジ、それ以上=緑）。
// 右端: SD カード状態（緑=正常、赤=エラー）。
//HEIGHT: 30px. The footer.
void draw_footer(){
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0,0,240,COLOR_BLACK);

  // ====Navigation.==== Distance to plathome.
  if(currentdestination != -1 && currentdestination < destinations_count){
    header_footer.setCursor(1, 1);
    header_footer.setTextColor(COLOR_MAGENTA);
    header_footer.printf("TC%3d", truec);


    header_footer.setCursor(60, 1);
    header_footer.setTextColor(COLOR_BLACK);
    if(!get_gps_fix() && !is_demo_active())
      header_footer.print("---km");
    else
      header_footer.printf(dest_dist>1000?"%.0fkm":dest_dist>100?"%.1fkm":"%.2fkm", dest_dist);
    

    if(currentdestination != -1 && currentdestination < destinations_count){
      header_footer.setTextColor(COLOR_MAGENTA);
      header_footer.setCursor(1, 17);
      if(destination_mode == DMODE_FLYAWAY)
        header_footer.print("FLY AWAY");
      if(destination_mode == DMODE_FLYINTO)
        header_footer.print("FLY INTO");
      if(destination_mode == DMODE_AUTO10K){
        if(auto10k_status == AUTO10K_INTO)
          header_footer.print("10K INTO");
        if(auto10k_status == AUTO10K_AWAY)
          header_footer.print("10K AWAY");
      }

      header_footer.setCursor(80, 17);
      header_footer.setTextWrap(false);
      header_footer.print(extradestinations[currentdestination].name);
    }
  }

  // ====Battery====

  header_footer.setCursor(SCREEN_WIDTH - 37, 1);
  header_footer.setTextColor(COLOR_GREEN);
  if (digitalRead(USB_DETECT)) {
    header_footer.print("USB");
  } else {
    header_footer.setCursor(SCREEN_WIDTH - 45, 1);
    float input_voltage = get_input_voltage();
    int bat_pct = constrain((int)((input_voltage - BAT_ZERO_VOLTAGE) / (4.2f - BAT_ZERO_VOLTAGE) * 100.0f), 0, 100);
    if (input_voltage <= BAT_LOW_VOLTAGE) {
      if((millis()/1000)%2 != 0){
        header_footer.setTextColor(COLOR_RED);
      }else{
        header_footer.fillRect(SCREEN_WIDTH-47,0, 47,15, COLOR_RED);
        header_footer.setTextColor(COLOR_WHITE,COLOR_RED);
      }
      header_footer.printf("%d%%", bat_pct);
      //最後のバッテリー警告から60秒以上経過。
      if(millis() > last_battery_warning_time+60*1000){
        last_battery_warning_time = millis();
        if(good_sd()){
          // SD認識済み: WAVファイルを再生（最低volume60保証）
          enqueueTask(createPlayWavTask("wav/battery_low.wav", 1, 60));
        } else {
          // SD未認識: 高音ビープ3回で代替警告（最低volume60保証）
          enqueueTask(createPlayMultiToneTask(2637, 150, 1, 1, 60));
          enqueueTask(createPlayMultiToneTask(2637, 150, 1, 1, 60));
          enqueueTask(createPlayMultiToneTask(2637, 400, 1, 1, 60));
        }
        enqueueTask(createLogSdfTask("Battery low: %d%% (%.2fV)", bat_pct, input_voltage));
      }
    } else if (input_voltage < BAT_HALF_VOLTAGE) {  
      header_footer.setTextColor(COLOR_MAGENTA);
      header_footer.printf("%d%%", bat_pct);
    } else {  // 50%以上
      header_footer.setTextColor(COLOR_GREEN);
      header_footer.printf("%d%%", bat_pct);
    }
  }
  // ====GNSS====
  int col = COLOR_GREEN;
  if (get_gps_numsat() < 5) {
    header_footer.setTextColor(COLOR_WHITE,COLOR_RED);
    header_footer.fillRect(SCREEN_WIDTH-101,1, 44,15, COLOR_RED);
  } else if (get_gps_numsat() < 10) {
    header_footer.setTextColor(COLOR_DARKORANGE);
  }else{
    header_footer.setTextColor(COLOR_GREEN);
  }
  header_footer.setCursor(SCREEN_WIDTH-100,1);
  header_footer.printf("%dsats", get_gps_numsat());

  

  // ====SD==== draw
  if (good_sd()) {
    header_footer.fillRect(SCREEN_WIDTH - 22, 15, 22, 15, COLOR_GREEN);
    header_footer.setTextColor(COLOR_WHITE, COLOR_GREEN);
  } else {
    header_footer.fillRect(SCREEN_WIDTH - 22, 15, 22, 15, COLOR_RED);
    header_footer.setTextColor(COLOR_WHITE, COLOR_RED);
  }
  header_footer.setCursor(SCREEN_WIDTH - 20,16);
  header_footer.print("SD");

  header_footer.pushSprite(0,290);
}

// 画面下部に「HDG UP」テキストを直接 TFT に描画する（TRACKUP モードを示す補助表示）。
void draw_headingupmode() {
  tft.setCursor(SCREEN_WIDTH / 2 - 18, SCREEN_HEIGHT - 21);
  tft.setTextColor(COLOR_BLACK);  // Highlight selected line
  tft.println("HDG UP");
}


// mapdata 構造体が持つポリゴン座標列を backscreen に折れ線（drawLine）で描画する。
// 各座標を latLonToXY() でスクリーン座標に変換し、両端いずれかが画面内の辺のみ描画する（画面外スキップ最適化）。
void draw_map(float mapUpDirection, double center_lat, double center_lon, float mapScale, const mapdata* mp, uint16_t color) {
  if (mp->size < 2) return;

  // 隣り合う 2 点しか使わないので、直前の点だけ持って回す。
  // 以前は全点を可変長配列（cord_tft points[mapsize]）へ展開していたが、
  // SD の mapdata.csv は点数に上限が無いため、大きなポリゴンを入れると
  // スタックを圧迫した（8 Byte/点。5000 点なら 40KB）。
  cord_tft prev = latLonToXY(mp->cords[0][1], mp->cords[0][0],
                             center_lat, center_lon, mapScale, mapUpDirection);
  for (int i = 1; i < mp->size; i++) {
    cord_tft cur = latLonToXY(mp->cords[i][1], mp->cords[i][0],
                              center_lat, center_lon, mapScale, mapUpDirection);
    // 画面外の判定は draw_clipped_line に任せる。
    // 以前は「両端とも画面外なら描かない」としていたが、画面を横切る長い線分は
    // 両端とも画面外になるため、高倍率で辺が消えていた。
    draw_clipped_line(prev.x, prev.y, cur.x, cur.y, color);
    prev = cur;
  }
}


// ============================================================
// スキャンライン多角形塗りつぶし（even-odd / 偶奇規則）
// ------------------------------------------------------------
// TFT_eSPI には fillTriangle / fillRect / fillCircle しかなく、琵琶湖のような
// 凹凸のある形や「島＝穴」を塗れないため自前で実装する。
//
// 原理: 画面を 1 行ずつ横に走査し、その行を横切る辺との交点を全部求めて
//       x でソートし、2 つずつペアにした区間を drawFastHLine で塗る。
//
//    y=100 の行 ──────────────────────────────
//               ╱‾‾‾‾‾‾‾‾‾╲
//              ╱    ╱▲╲    ╲        ▲ = 竹島（湖の中の島）
//    ─────────┤    ╱   ╲    ├────────
//             x1  x2    x3   x4      → [x1,x2] と [x3,x4] を塗る
//
// even-odd を使う理由:
//   「半直線との交差回数が奇数なら内側」という規則なので、リングの回転方向
//   （時計回り/反時計回り）を一切気にしなくてよい。もう一方の流儀である
//   nonzero winding rule だと島リングを逆向きに格納する前処理が必要になるが、
//   OSM データも既存の map_biwako も向きが保証されていない。
//   湖リングと島リングをまとめて 1 回で渡すだけで、島が自動的に穴として抜ける。
// ============================================================

// ============================================================
// 線分の画面クリップ（Cohen–Sutherland）
// ------------------------------------------------------------
// TFT_eSPI の drawWedgeLine は境界ボックスを clipWindow() で切るものの、
// 走査開始行 ys に「クリップ前の端点 y」をそのまま使っている:
//
//     if (!clipWindow(&x0, &y0, &x1, &y1)) return;
//     int32_t ys = ay;                      // ← クリップ前の端点
//     for (int32_t yp = ys; yp <= y1; yp++) // ← 画面外の分だけ空回りする
//
// このため端点が画面から数千 px 外にあると、外側ループがその回数だけ回り、
// 1 フレームに数百 ms かかる（最大拡大で実測 380ms）。drawLine の Bresenham も
// 同様に画面外を 1px ずつ歩く。
// 描画前にここで画面矩形へ切っておけば、どの縮尺でも端点は必ず画面付近に収まる。
// ============================================================
#define CLIP_MARGIN 4  // 太線の端キャップが画面端で欠けないよう少し外側で切る

// 座標は最大倍率で ±数万 px になるため、交点計算は double で行う。
// float だと桁落ちでクリップ後の線が元の線から最大 6px ずれる（実測）。
static inline int clip_outcode(double x, double y) {
  const double lo = -CLIP_MARGIN, hi = BACKSCREEN_SIZE - 1 + CLIP_MARGIN;
  int c = 0;
  if (x < lo) c |= 1; else if (x > hi) c |= 2;
  if (y < lo) c |= 4; else if (y > hi) c |= 8;
  return c;
}

// 線分を画面矩形にクリップする。完全に画面外なら false（描画不要）。
bool clip_line_to_screen(int* px0, int* py0, int* px1, int* py1) {
  const double lo = -CLIP_MARGIN, hi = BACKSCREEN_SIZE - 1 + CLIP_MARGIN;
  double x0 = *px0, y0 = *py0, x1 = *px1, y1 = *py1;
  int c0 = clip_outcode(x0, y0);
  int c1 = clip_outcode(x1, y1);

  for (int guard = 0; guard < 8; guard++) {
    if (!(c0 | c1)) break;        // 両端とも画面内
    if (c0 & c1) return false;    // 同じ側に両端がある = 交差しない
    const int c = c0 ? c0 : c1;
    double x = 0, y = 0;
    if (c & 8) { x = x0 + (x1 - x0) * (hi - y0) / (y1 - y0); y = hi; }
    else if (c & 4) { x = x0 + (x1 - x0) * (lo - y0) / (y1 - y0); y = lo; }
    else if (c & 2) { y = y0 + (y1 - y0) * (hi - x0) / (x1 - x0); x = hi; }
    else { y = y0 + (y1 - y0) * (lo - x0) / (x1 - x0); x = lo; }
    if (c == c0) { x0 = x; y0 = y; c0 = clip_outcode(x0, y0); }
    else { x1 = x; y1 = y; c1 = clip_outcode(x1, y1); }
  }
  // 切り捨てではなく四捨五入。切り捨てだと負側で外へ、正側で内へ寄り、
  // クリップ前の線から余計にずれる。
  *px0 = (int)lround(x0); *py0 = (int)lround(y0);
  *px1 = (int)lround(x1); *py1 = (int)lround(y1);
  return true;
}


#define VM_MAX_EDGES 2048  // 同時に扱える辺の数（12 Byte/辺 = 24KB）

// スキャンライン用の辺。ET/AET のどちらか一方にしか属さないので next を共用できる。
struct scan_edge {
  float x;        // 現在のスキャンラインにおける交点 x
  float dxdy;     // 1 行進むごとの x の増分（= 1/傾き）
  int16_t y_bot;  // この y に達したら除外する（半開区間 [y_top, y_bot)）
  int16_t next;   // 連結リストの次の辺 index（-1 = 終端）
};

static scan_edge s_edges[VM_MAX_EDGES];
static int16_t s_et[BACKSCREEN_SIZE];  // Edge Table: 各行から開始する辺のリスト先頭
static int16_t s_aet_buf[VM_MAX_EDGES / 4];  // Active Edge Table のソート用一時配列

// 複数リングをまとめて even-odd で塗りつぶす。
//   xs/ys      : 全リングの点を連結したスクリーン座標の配列
//   ring_start : リング i の点は [ring_start[i], ring_start[i+1]) の範囲。要素数は nrings+1
//   nrings     : リング数（湖本体 + 島、など）
// 各リングは暗黙に閉じている（最後の点から最初の点へ辺を張る）。
// 辺数が VM_MAX_EDGES を超える場合は中途半端な塗りを避けるため何も描かず false を返す。
bool fill_polygon_evenodd(const int16_t* xs, const int16_t* ys,
                          const uint16_t* ring_start, uint8_t nrings, uint16_t color) {
  const int H = BACKSCREEN_SIZE;
  const int W = BACKSCREEN_SIZE;

  if (nrings == 0) return true;
  if (ring_start[nrings] > VM_MAX_EDGES) return false;  // 辺の総数は点の総数と等しい

  for (int i = 0; i < H; i++) s_et[i] = -1;

  int nedges = 0;
  int y_min = H, y_max = 0;

  // ---- Edge Table の構築 ----
  for (uint8_t r = 0; r < nrings; r++) {
    const uint16_t beg = ring_start[r];
    const uint16_t end = ring_start[r + 1];
    const int n = end - beg;
    if (n < 3) continue;  // 3 点未満は面にならない

    for (int i = 0; i < n; i++) {
      const int j = (i + 1 == n) ? 0 : (i + 1);  // 最後の点は最初の点へ閉じる
      int x0 = xs[beg + i], y0 = ys[beg + i];
      int x1 = xs[beg + j], y1 = ys[beg + j];

      if (y0 == y1) continue;  // 水平な辺は交点を作らないので無視
      if (y0 > y1) {           // 常に上から下へ向くように揃える
        int t = x0; x0 = x1; x1 = t;
        t = y0; y0 = y1; y1 = t;
      }
      if (y1 <= 0 || y0 >= H) continue;  // 画面の上下に完全に外れている

      const float dxdy = (float)(x1 - x0) / (float)(y1 - y0);
      float x_top = (float)x0;
      int y_top = y0;
      if (y_top < 0) {  // 画面上端でクリップし、その行での x を求め直す
        x_top += dxdy * (float)(0 - y_top);
        y_top = 0;
      }
      const int y_bot = (y1 > H) ? H : y1;  // 半開区間なので下端は含まない
      if (y_top >= y_bot) continue;

      if (nedges >= VM_MAX_EDGES) return false;
      s_edges[nedges].x = x_top;
      s_edges[nedges].dxdy = dxdy;
      s_edges[nedges].y_bot = (int16_t)y_bot;
      s_edges[nedges].next = s_et[y_top];
      s_et[y_top] = (int16_t)nedges;
      nedges++;

      if (y_top < y_min) y_min = y_top;
      if (y_bot > y_max) y_max = y_bot;
    }
  }
  if (nedges == 0) return true;

  // ---- 走査 ----
  // ポリゴンが画面の一部にしか無い場合に全 240 行を回らないよう y_min..y_max に限定する。
  int aet_n = 0;
  for (int y = y_min; y < y_max; y++) {
    // この行から始まる辺を AET に追加
    for (int e = s_et[y]; e != -1; e = s_edges[e].next) {
      if (aet_n < (int)(sizeof(s_aet_buf) / sizeof(s_aet_buf[0]))) s_aet_buf[aet_n++] = (int16_t)e;
    }
    // 下端に達した辺を除外（半開区間なので y_bot == y で抜ける）
    int w = 0;
    for (int i = 0; i < aet_n; i++) {
      if (s_edges[s_aet_buf[i]].y_bot > y) s_aet_buf[w++] = s_aet_buf[i];
    }
    aet_n = w;

    // x で挿入ソート（アクティブな辺は通常ごく少数なので十分速い）
    for (int i = 1; i < aet_n; i++) {
      const int16_t key = s_aet_buf[i];
      const float kx = s_edges[key].x;
      int j = i - 1;
      while (j >= 0 && s_edges[s_aet_buf[j]].x > kx) {
        s_aet_buf[j + 1] = s_aet_buf[j];
        j--;
      }
      s_aet_buf[j + 1] = key;
    }

    // 交点を 2 つずつペアにして塗る（奇数番目の区間＝内側）
    for (int i = 0; i + 1 < aet_n; i += 2) {
      // ピクセル中心規則: 隣接ポリゴンで同じ画素を二重に塗らないよう ceil / ceil-1 で取る
      int xa = (int)ceilf(s_edges[s_aet_buf[i]].x);
      int xb = (int)ceilf(s_edges[s_aet_buf[i + 1]].x) - 1;
      if (xb < 0 || xa >= W || xa > xb) continue;
      if (xa < 0) xa = 0;
      if (xb >= W) xb = W - 1;
      backscreen.drawFastHLine(xa, y, xb - xa + 1, color);
    }

    // 次の行に向けて交点を進める
    for (int i = 0; i < aet_n; i++) s_edges[s_aet_buf[i]].x += s_edges[s_aet_buf[i]].dxdy;
  }
  return true;
}




// 琵琶湖 HPA 競技コースの緑の基準線を描画する:
//   - PLA（スタート）から N パイロン・W パイロン・竹島 への緑の基準線
//   - PLA を中心とした 10.975km 円（公式ルール 2025: 第 1 レグ往路距離）
//   - PLA を中心とした 1.0km 円（公式ルール 2025: 折り返し後の距離）
// 描画順の都合でパイロンのアイコン（draw_pilon_takeshima_marks）とは分けてある。
// 線はマゼンタの誘導ラインより先に描いて下のレイヤーに、アイコンは後に描いて上のレイヤーにする。
#define PILON_LINE_WIDTH 3          // PLA→パイロン基準線の太さ [px]
#define CENTERLINE_WIDTH 2          // PLA センターライン（N/W パイロンの中間方位線）の太さ [px]
#define CENTERLINE_INNER_KM 1.0     // センターラインの描き始め（1km 円上）
#define CENTERLINE_OUTER_KM 10.975  // センターラインの描き終わり（10.975km 円上）
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(pla_lat, pla_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(pilon_north_lat, pilon_north_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft s_pilon = latLonToXY(pilon_south_lat, pilon_south_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft takeshima = latLonToXY(takeshima_lat_v, takeshima_lon_v, mapcenter_lat, mapcenter_lon, scale, upward);


  // PLA → 北/南パイロンの線は飛行中に最も見る基準線なので 3px 幅で太く描く（竹島線は 1px のまま）
  draw_clipped_wideline(pla.x, pla.y, n_pilon.x, n_pilon.y, PILON_LINE_WIDTH, COLOR_BLUE);
  draw_clipped_line(pla.x, pla.y, takeshima.x, takeshima.y, COLOR_GREEN);
  draw_clipped_wideline(pla.x, pla.y, s_pilon.x, s_pilon.y, PILON_LINE_WIDTH, COLOR_BLUE);

  // 2本のパイロン基準線の「角度的な中間」を通るセンターライン（オレンジ）。
  // 方位はパイロン座標から実行時に計算するので、パイロンが移動しても自動で追従する。
  // 1km 円〜10.975km 円の間だけ描き、PLA 付近のスタート台・1km パイロンのアイコンを隠さない。
  double centerline_bearing = pla_centerline_bearing_rad();
  double inner_lat, inner_lon, outer_lat, outer_lon;
  calculatePointFromBearing(pla_lat, pla_lon, centerline_bearing, CENTERLINE_INNER_KM, inner_lat, inner_lon);
  calculatePointFromBearing(pla_lat, pla_lon, centerline_bearing, CENTERLINE_OUTER_KM, outer_lat, outer_lon);
  cord_tft cl_inner = latLonToXY(inner_lat, inner_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft cl_outer = latLonToXY(outer_lat, outer_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  draw_clipped_wideline(cl_inner.x, cl_inner.y, cl_outer.x, cl_outer.y, CENTERLINE_WIDTH, COLOR_ORANGE);

  // 公式ルール2025 10.975km for first leg outbound.
  backscreen.drawCircle(pla.x, pla.y, scale*10.975f/cos(radians(35)),COLOR_GREEN);
  // 公式ルール2025 1.0km リターンフライト。
  backscreen.drawCircle(pla.x, pla.y, scale*1.0f/cos(radians(35)),COLOR_GREEN);
}

// 琵琶湖 HPA 競技コースのアイコンを描画する:
//   - N パイロン・W パイロン・1km 地点: オレンジ塗りつぶし三角形（パイロンマーク）
//   - PLA: 青地に黄色の小三角形 + 赤線（スタート台の簡易表示）
// マゼンタの誘導ラインより後に呼び、アイコンが隠れないようにする。
void draw_pilon_takeshima_marks(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(pla_lat, pla_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(pilon_north_lat, pilon_north_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft s_pilon = latLonToXY(pilon_south_lat, pilon_south_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft pilon_1km = latLonToXY(pilon_1km_lat, pilon_1km_lon, mapcenter_lat, mapcenter_lon, scale, upward);

  if(!n_pilon.isOutsideTft()){
    backscreen.fillTriangle(n_pilon.x-3,n_pilon.y,n_pilon.x+3,n_pilon.y,n_pilon.x,n_pilon.y-9, COLOR_ORANGE);
    backscreen.drawTriangle(n_pilon.x-3,n_pilon.y,n_pilon.x+3,n_pilon.y,n_pilon.x,n_pilon.y-9, COLOR_BLACK);
  }
  if(!s_pilon.isOutsideTft()){
    backscreen.fillTriangle(s_pilon.x-3,s_pilon.y,s_pilon.x+3,s_pilon.y,s_pilon.x,s_pilon.y-9, COLOR_ORANGE);
    backscreen.drawTriangle(s_pilon.x-3,s_pilon.y,s_pilon.x+3,s_pilon.y,s_pilon.x,s_pilon.y-9, COLOR_BLACK);
  }
  if(!pilon_1km.isOutsideTft()){
    backscreen.fillTriangle(pilon_1km.x-3,pilon_1km.y,pilon_1km.x+3,pilon_1km.y,pilon_1km.x,pilon_1km.y-9, COLOR_ORANGE);
    backscreen.drawTriangle(pilon_1km.x-3,pilon_1km.y,pilon_1km.x+3,pilon_1km.y,pilon_1km.x,pilon_1km.y-9, COLOR_BLACK);
  }
  if(!pla.isOutsideTft()){
    backscreen.fillRect(pla.x-3, pla.y-2, 6, 8, COLOR_BLUE);
    backscreen.fillTriangle(pla.x-2, pla.y-1, pla.x+1, pla.y-1, pla.x, pla.y+3, COLOR_YELLOW);
    backscreen.drawLine(pla.x-2, pla.y-2, pla.x+2, pla.y-2, COLOR_RED);
    backscreen.drawLine(pla.x-2, pla.y-3, pla.x+2, pla.y-3, COLOR_RED);
  }

}



// 負の値にも対応した剰余演算（C++ の % は負の結果になる場合がある）。
// brightnessIndex のループ計算（tft_change_brightness）で使用する。
int mod(int x, int y) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

// 画面輝度を brightnessLevels[] 配列から 1 ステップ進める（輝度設定の切り替え）。
// increment = +1 で明るく、-1 で暗くなる（リングバッファ方式でループする）。
// BRIGHTNESS_SETTING_AVAIL が定義されていない場合は何もしない。
void tft_change_brightness(int increment) {
#ifdef BRIGHTNESS_SETTING_AVAIL
  brightnessIndex = mod(brightnessIndex + increment, sizeof(brightnessLevels) / sizeof(brightnessLevels[0]));
  screen_brightness = brightnessLevels[brightnessIndex];
  analogWrite(TFT_BL, BRIGHTNESS(screen_brightness));  // For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
#endif
}


// 衛星の方位角・仰角をスカイプロット（円形の衛星配置図）上の X 座標に変換する。
// 仰角 0°（水平）→ 外周、90°（天頂）→ 中心。方位角が X/Y の方向を決める。
int calculateGPS_X(float azimuth, float elevation) {
  const int shift_left = 10;
  const int radius = SCREEN_WIDTH / 2 - shift_left;
  return (SCREEN_WIDTH / 2) + (int)(cos(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) - shift_left;
}

// 衛星の方位角・仰角をスカイプロット上の Y 座標に変換する（calculateGPS_X の Y 版）。
int calculateGPS_Y(float azimuth, float elevation) {
  const int height = SCREEN_WIDTH;
  const int radius = SCREEN_WIDTH / 2 - 20;
  const int shift_down = 0;
  return (height / 2) - (int)(sin(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) + shift_down;
}


// GPS 状態に応じたステータスメッセージを backscreen に表示する（地図データなし時のみ呼ばれる）。
// 優先順位:
//   1. GPS モジュール未接続 → "NO GNSS connection !!" 表示して終了。
//   2. GPS 未フィックス → "Scanning GNSS..." + ドットアニメーション。
//   3. 衛星数 = 0 → "Weak GNSS Signal" + スキャン中表示。
void draw_nomapdata() {

  if (get_gps_connection()) {
    //"GPS Module connected."
  } else {
    if (!getReplayMode() && !is_demo_active()) {  // リプレイ中・デモ中は NO GNSS 警告を表示しない
      backscreen.setCursor(3,50);
      backscreen.setTextColor(COLOR_MAGENTA);
      backscreen.println("NO GNSS connection !!");
      backscreen.println(" Try power off then on.");
      backscreen.println(" Please contact developer.");
      return;
    }
  }

  int col = COLOR_BLACK;
  // TRACKUP: 自機アイコン尾翼（cy+9=189）の下にボックスを配置する
  int box_y  = is_trackupmode() ? (get_self_cy() + 12) : 145;  // TRACKUP=192, NORTHUP=145
  int text_y1 = is_trackupmode() ? (box_y + 5)  : 150;          // TRACKUP=197, NORTHUP=150
  int text_y2 = is_trackupmode() ? (box_y + 20) : 165;          // TRACKUP=212, NORTHUP=165

  if (!get_gps_fix()) {
    backscreen.fillRect(5, box_y, SCREEN_WIDTH-5*2, 30+5*2, COLOR_WHITE);
    backscreen.drawRect(5, box_y, SCREEN_WIDTH-5*2, 30+5*2, COLOR_RED);
    backscreen.drawRect(6, box_y+1, SCREEN_WIDTH-5*2-2, 30+5*2-2, COLOR_RED);
    backscreen.setCursor(23, text_y1);
    backscreen.setTextColor(COLOR_ORANGE);
    backscreen.loadFont(AA_FONT_SMALL);
    backscreen.print("Scanning GNSS/GPS Signal");
    char text[28];
    int dotCount = (millis() / 900) % 10;
    // Safely create the string with snprintf
    snprintf(text, sizeof(text), "Stand by for fix%.*s", dotCount, "..........");
    backscreen.setCursor(45, text_y2);
    backscreen.print(text);
  }
  else if (get_gps_numsat() == 0 && !is_demo_active()) {
    backscreen.fillRect(5, box_y, SCREEN_WIDTH-5*2, 30+5*2, COLOR_WHITE);
    backscreen.drawRect(5, box_y, SCREEN_WIDTH-5*2, 30+5*2, COLOR_RED);
    backscreen.drawRect(6, box_y+1, SCREEN_WIDTH-5*2-2, 30+5*2-2, COLOR_RED);
    backscreen.setCursor(30, text_y1);
    backscreen.setTextColor(COLOR_RED);
    backscreen.print("Weak GNSS/GPS Signal");
    char text[28];
    int dotCount = (millis() / 900) % 10;
    // Safely create the string with snprintf
    snprintf(text, sizeof(text), "Scanning signal%.*s", dotCount, "..........");
    backscreen.setCursor(45, text_y2);
    backscreen.print(text);
  }
}

unsigned long lastdrawn_sddetail = 0;
extern char sdfiles[20][32];
extern int sdfiles_size[20]; 
extern volatile int max_page;  // Core1 が更新する（実体は mysd.cpp・volatile）
volatile bool loading_sddetail = true;
bool sd_detail_loading_displayed = false;

// SD カードのファイル一覧画面を描画する（screen_mode == MODE_SDDETAIL 時）。
// browse_sd() の結果（sdfiles[]）を使って 20 エントリ / ページで表示する。
// loading_sddetail=true の間は "Stand by..." または "No SD card" を表示。
// ファイルサイズは KB 単位で右端に表示する。
// 描画完了後に sd_detail_loading_displayed フラグを更新する。
void draw_sddetail(int page) {
  lastdrawn_sddetail = millis();
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);  // +10px（variodetail と同じオフセット）
  if(max_page >= 0 && !loading_sddetail)
    header_footer.printf("SD DETAIL  %d/%d",page%(max_page+1)+1,max_page+1);
  else
    header_footer.printf("SD DETAIL  loading...");

  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に合わせ backscreen との重複を解消
  
  backscreen.fillScreen(COLOR_WHITE);
  
  if(!loading_sddetail){
    for(int i= 0; i< 20; i++){
      backscreen.setCursor(10,i*12);
      backscreen.print(sdfiles[i]);
    }

    backscreen.unloadFont();
    backscreen.setTextSize(1);
    for(int i= 0; i< 20; i++){
      if(sdfiles_size[i] != 0){
        backscreen.setCursor(SCREEN_WIDTH-40,i*12+4);
        backscreen.printf("%dKB",(int)(sdfiles_size[i]/1024)+1);
      }
    }
    backscreen.loadFont(AA_FONT_SMALL);
    sd_detail_loading_displayed = false;
  }else{
    sd_detail_loading_displayed = true;
    backscreen.setCursor(80,100);
    if(digitalRead(SD_DETECT)){
      backscreen.print("No SD card ...");
    }else
      backscreen.print("Stand by ...");
  }
  backscreen.pushSprite(0,40);

  // フッター: setup回数（左） / SD検出状態（中央） / SDIO or SPI 状態（右）
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0, 0, SCREEN_WIDTH, COLOR_BLACK);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setCursor(1, 1);
  header_footer.printf("SETUP: %d", get_sd_setup_count());
  header_footer.setCursor(SCREEN_WIDTH / 2 - 30, 1);
  if (!digitalRead(SD_DETECT)) {
    header_footer.setTextColor(COLOR_GREEN, COLOR_WHITE);
    header_footer.print("DET:OK");
  } else {
    header_footer.setTextColor(COLOR_RED, COLOR_WHITE);
    header_footer.print("DET:NO");
  }
  header_footer.setCursor(SCREEN_WIDTH - 55, 1);
  if (get_sd_use_spi()) {
    header_footer.setTextColor(COLOR_RED, COLOR_WHITE);
    header_footer.print("SPI");
  } else {
    header_footer.setTextColor(COLOR_GREEN, COLOR_WHITE);
    header_footer.print("SDIO");
  }
  header_footer.pushSprite(0, SCREEN_HEIGHT - 40);
}


// ===== リプレイ選択画面 =====
volatile bool loading_replaylist = true;   // Core1 でファイル一覧を取得中
bool replay_loading_displayed = false;     // "Stand by..." を表示中か（読み込み完了時の再描画判定用）

// リプレイする対象を選ぶ画面を描画する（screen_mode == MODE_REPLAYSELECT 時）。
// 1ページ 20 行（12px ピッチ）。先頭に固定項目（Replay OFF / 2025 / 2026）、
// 続けて SD 上の飛行 CSV、最後に Return が並ぶ。項目モデルは mysd.cpp の
// replay_menu_item() が管理しており、ボタン処理側と共有している。
// cursor: 現在のカーソル位置（通し番号）。マゼンタで表示する。
void draw_replayselect(int page, int cursor) {
  int pagecount = replay_menu_page_count();

  // --- ヘッダー ---
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);  // +10px（sddetail と同じオフセット）
  if (!loading_replaylist)
    header_footer.printf("REPLAY  %d/%d", page + 1, pagecount);
  else
    header_footer.printf("REPLAY  loading...");
  // 状態ドット。設定メニューの REPLAY 行と同じ意味（緑＝通常 GPS、赤＝再生中）。
  header_footer.fillCircle(228, 18, 6, getReplayMode() ? COLOR_RED : COLOR_GREEN);
  header_footer.pushSprite(0, -10);

  // --- リスト本体 ---
  backscreen.fillScreen(COLOR_WHITE);
  // variodetail 等から戻ると unloadFont() された状態のことがあるため明示的にロードする
  backscreen.loadFont(AA_FONT_SMALL);

  if (!loading_replaylist) {
    char label[40];
    int  fsize = 0;
    for (int r = 0; r < REPLAY_LIST_ROWS; r++) {
      int index = page * REPLAY_LIST_ROWS + r;
      ReplayItemType type = replay_menu_item(index, page, label, sizeof(label), &fsize);
      if (type == RITEM_NONE) continue;

      uint16_t col = COLOR_BLACK;
      if (index == cursor) {
        col = COLOR_MAGENTA;            // カーソル位置
      } else if (type == RITEM_OFF || type == RITEM_RETURN ||
                 type == RITEM_FLIGHTONLY || type == RITEM_SPEED) {
        col = COLOR_BLUE;               // 操作項目はファイル名と区別する
      } else if (getReplayMode() && strcmp(get_replay_filename(), label) == 0) {
        col = COLOR_GREEN;              // 現在再生中のファイル
      }
      // 固定項目は現在再生中かどうかをパスで判定する
      if (getReplayMode() && index != cursor) {
        if ((type == RITEM_2025 && strcmp(get_replay_filename(), REPLAY_2025_FILE) == 0) ||
            (type == RITEM_2026 && strcmp(get_replay_filename(), REPLAY_2026_FILE) == 0))
          col = COLOR_GREEN;
      }

      backscreen.setTextColor(col, COLOR_WHITE);
      backscreen.setCursor(10, r * 12);
      backscreen.print(label);
    }

    // ファイルサイズは小さい内蔵フォントで右端に描く（sddetail と同じ流儀）
    backscreen.unloadFont();
    backscreen.setTextSize(1);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    for (int r = 0; r < REPLAY_LIST_ROWS; r++) {
      int index = page * REPLAY_LIST_ROWS + r;
      if (replay_menu_item(index, page, label, sizeof(label), &fsize) == RITEM_FILE && fsize != 0) {
        backscreen.setCursor(SCREEN_WIDTH - 40, r * 12 + 4);
        backscreen.printf("%dKB", (int)(fsize / 1024) + 1);
      }
    }
    backscreen.loadFont(AA_FONT_SMALL);
    replay_loading_displayed = false;
  } else {
    replay_loading_displayed = true;
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    backscreen.setCursor(80, 100);
    if (digitalRead(SD_DETECT))
      backscreen.print("No SD card ...");
    else
      backscreen.print("Stand by ...");
  }
  backscreen.pushSprite(0, 40);

  // --- フッター: 現在の再生状態 ---
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0, 0, SCREEN_WIDTH, COLOR_BLACK);
  header_footer.setCursor(1, 1);
  if (getReplayMode()) {
    header_footer.setTextColor(COLOR_RED, COLOR_WHITE);
    // この画面を開いている間は再生を一時停止しているので PAUSED と表示する
    header_footer.printf("PAUSED x%d: %s", get_replay_speed(), get_replay_filename());
  } else {
    header_footer.setTextColor(COLOR_GREEN, COLOR_WHITE);
    header_footer.print("NOW: normal GPS");
  }
  header_footer.pushSprite(0, SCREEN_HEIGHT - 40);
}




// ============================================================
// Vario 詳細画面を描画する（screen_mode == MODE_VARIODETAIL 時）。
// page % 2 == 0: センサー接続状況・VSI 全比較・MS5611 気圧データ・GNSS 垂直速度。
// page % 2 == 1: BNO085 IMU データ（線形加速度・Euler 角）・Kalman 設定・GNSS VSI 融合。
// ヘッダーは GPS DETAIL 同様に 10px 上にずらして pushSprite(0,-10) で転送。
// VSI バーは GPS_TFT_map.ino のループ内で airdata_updated タイミングに更新される。
void draw_variodetail(int page) {
  // ---- ヘッダー: GPS DETAIL と同様に 10px 上ずらし ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);

  const char* page_titles[] = {
    "VARIO 1:STATUS+VSI",
    "VARIO 2:IMU+KALMAN"
  };
  header_footer.setCursor(1, 11);  // +10px（GPS DETAIL と同じオフセット）
  header_footer.print(page_titles[page % 2]);
  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に合わせる

  // ---- バックスクリーン（AA_FONT_SMALL を使用）----
  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

  int y = 2;
  const int line_height = 12;
  const int nextcolumn_height = 14;

  if (page % 2 == 0) {
    // ---- Page 1: センサー接続状況 + VSI 全比較 + MS5611 + GNSS 垂直 ----

    // センサー接続状況
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- Sensor Status --");
    y += line_height;

    // MS5611 と BNO085 を 1 行にまとめる
    backscreen.setCursor(2, y);
    backscreen.setTextColor(get_airdata_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
    backscreen.printf("MS5611:%s", get_airdata_ok() ? "OK" : "NG");
    backscreen.setCursor(84, y);
    backscreen.setTextColor(get_imu_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
    backscreen.printf("BNO085:%s", get_imu_ok() ? "OK" : "NG");
    y += line_height;

    // GNSS fix 状態
    bool _gnss3d = get_gps_gnssFixOK() && get_gps_fixtype() >= 3;
    bool _gnssAny = get_gps_gnssFixOK();
    backscreen.setCursor(2, y);
    backscreen.setTextColor(_gnss3d ? COLOR_GREEN : _gnssAny ? COLOR_ORANGE : COLOR_RED, COLOR_WHITE);
    backscreen.printf("GNSS:%s fix=%d %dsats",
      _gnss3d ? "3D-OK" : _gnssAny ? "2D" : "NOFIX",
      get_gps_fixtype(), get_gps_numsat());
    y += line_height; 

    // 動作モード（GNSS VSI 融合状況を含む）
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    if      (get_imu_ok() && get_airdata_ok() && _gnss3d) backscreen.print("KF: Baro+IMU+GNSS");
    else if (get_imu_ok() && get_airdata_ok())              backscreen.print("KF: Baro+IMU");
    else if (get_airdata_ok() && _gnss3d)                   backscreen.print("KF: Baro+GNSS");
    else if (get_airdata_ok())                              backscreen.print("KF: Baro only");
    else if (get_imu_ok())                                  backscreen.print("Mode: BNO085 only");
    else                                                     backscreen.print("Mode: No sensors");
    y += nextcolumn_height;

    // VSI 全比較（Baro / GNSS / KF）
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- VSI Comparison --");
    y += line_height;

    if (get_airdata_ok()) {
      float baro_vsi = get_airdata_vspeed();

      backscreen.setCursor(2, y);
      backscreen.setTextColor((baro_vsi > 0.1f) ? COLOR_GREEN : (baro_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("Baro:%+.2fm/s  Alt:%.1fm", baro_vsi, get_airdata_altitude());
      y += line_height;

      // GNSS VSI — sAcc ゲート状態を色で示す
      float _gnss_vsi = get_gps_veld_mps();
      float _sacc_now = get_gps_sacc_mmps() / 1000.0f;
      bool  _gate_ok  = _gnss3d && (_sacc_now < GNSS_VSI_SACC_MAX_MPS);
      backscreen.setCursor(2, y);
      backscreen.setTextColor(_gate_ok ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("GNSS:%+.2fm/s %s", _gnss_vsi, _gate_ok ? "[KF]" : "[--]");
      y += line_height;

      // KF VSI（BNO085 なし時も Baro+GNSS KF から出力）
      float kf_vsi = get_imu_vspeed();
      backscreen.setCursor(2, y);
      backscreen.setTextColor((kf_vsi > 0.1f) ? COLOR_GREEN : (kf_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("KF  :%+.2fm/s", kf_vsi);
      y += nextcolumn_height;

      // 高度 KF まとめ（各センサーと KF MSL 出力を一覧表示）
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("-- Altitude KF (MSL) --");
      y += line_height;

      // 現在の気圧・気温 ＋ 起動時の基準気圧
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("Pres:%.1fhPa Temp:%.1fC", get_airdata_pressure(), get_airdata_temperature());
      y += line_height;

      // 起動時の気圧（これを 0m 基準に使用）と起動地のISA絶対高度
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("GND:%.1fhPa ISA:%.1fm", get_airdata_ground_pressure(), get_airdata_ground_altitude());
      y += line_height;

      // GNSS MSL 高度と KF MSL 高度（gnss_kf_offset 確定後は GNSS に収束）
      bool _msl_ready = get_imu_gnss_offset_ready();
      float _kf_msl   = get_imu_altitude_msl();
      float _gnss_msl = get_gps_altitude();
      backscreen.setCursor(2, y);
      backscreen.setTextColor(_msl_ready ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("GNSS:%.1fm  KF:%.1fm%s", _gnss_msl, _kf_msl, _msl_ready ? "" : "(-)");
      y += line_height;

      // B.S.L. = MSL - ELEVATION_GEOID_OFFSET_M（琵琶湖面 0m 基準）
      float _kf_bsl   = _kf_msl   - ELEVATION_GEOID_OFFSET_M;
      float _gnss_bsl = _gnss_msl - ELEVATION_GEOID_OFFSET_M;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(_msl_ready ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("BSL GNSS:%.1fm KF:%.1fm", _gnss_bsl, _kf_bsl);
      y += nextcolumn_height;

    } else {
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.print("MS5611: no data");
      y += nextcolumn_height;
    }

    // GNSS 垂直詳細
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- GNSS Vertical --");
    y += line_height;

    float gnss_vsi = get_gps_veld_mps();
    float vacc_m   = get_gps_vacc_mm()   / 1000.0f;
    float sacc_mps = get_gps_sacc_mmps() / 1000.0f;
    float gnss_alt = get_gps_altitude();
    float r_vel    = sacc_mps * sacc_mps * GNSS_VSI_R_SCALE;
    if (r_vel < 0.001f) r_vel = 0.001f;  // 下限クランプ（imu.cpp と同じ）
    bool  _sacc_ok = sacc_mps < GNSS_VSI_SACC_MAX_MPS;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(_sacc_ok ? (sacc_mps > GNSS_VSI_SACC_MAX_MPS * 0.7f ? COLOR_ORANGE : COLOR_BLACK)
                                     : COLOR_RED, COLOR_WHITE);
    backscreen.printf("sAcc:%.3fm/s(<%.1f)%s  vAcc:%.2fm",
      sacc_mps, GNSS_VSI_SACC_MAX_MPS, _sacc_ok ? "OK" : "NG", vacc_m);
    y += line_height;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("R_vel:%.4fm2  Alt:%.1fm(MSL)", r_vel, gnss_alt);
    y += line_height;

    if (get_airdata_ok()) {
      float dalt = gnss_alt - get_airdata_altitude();
      backscreen.setCursor(2, y);
      backscreen.setTextColor(fabsf(dalt) < 20.0f ? COLOR_BLACK : COLOR_ORANGE, COLOR_WHITE);
      backscreen.printf("AltDiff:%+.1fm(GNSS-Baro)", dalt);
      y += line_height;
    }

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("fix=%d gnssOK=%s sats=%d",
      get_gps_fixtype(), get_gps_gnssFixOK() ? "Y" : "N", get_gps_numsat());

  } else {
    // ---- Page 2: BNO085 IMU データ + Kalman 設定 + GNSS VSI 融合 ----

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- BNO085 IMU --");
    y += line_height;

    if (get_imu_ok()) {
      // イベント受信レート
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("GRV:%.1fHz LACC:%.1fHz RV:%.1fHz",
                        get_imu_grv_hz(), get_imu_lacc_hz(), get_imu_rv_hz());
      y += line_height;

      float az = get_imu_az();
      backscreen.setCursor(2, y);
      backscreen.setTextColor((az > 0.2f) ? COLOR_GREEN : (az < -0.2f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("VertAccel:%+.3fm/s2", az);
      y += line_height;

      backscreen.setCursor(2, y);
      backscreen.setTextColor(get_imu_gnss_offset_ready() ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("KF MSL:%.1fm  VSI:%+.2fm/s", get_imu_altitude_msl(), get_imu_vspeed());
      y += nextcolumn_height;

      // 線形加速度（ボディフレーム）
      float lax, lay, laz;
      get_imu_linaccel(lax, lay, laz);
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("-- LinAccel[body] --");
      y += line_height;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("X:%+.3f Y:%+.3f Z:%+.3fm/s2", lax, lay, laz);
      y += line_height;

      // Euler 角（Roll/Pitch: GAME RV、Yaw: RV+Mag）
      float roll, pitch, yaw;
      get_imu_euler(roll, pitch, yaw);
      float mag_acc = get_imu_mag_accuracy_deg();
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("-- Euler Angles --");
      y += line_height;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("R:%+.1f P:%+.1f (GAME RV)", roll, pitch);
      y += line_height;
      backscreen.setCursor(2, y);
      if (mag_acc >= 0.0f) {
        backscreen.setTextColor(mag_acc < 5.0f ? COLOR_GREEN : mag_acc < 15.0f ? COLOR_ORANGE : COLOR_RED, COLOR_WHITE);
        backscreen.printf("Yaw:%+.1f(mag) acc:+/-%.1fdeg", yaw, mag_acc);
      } else {
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.printf("Yaw:%+.1f(game,no mag)", yaw);
      }
      y += nextcolumn_height;

    } else {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.setCursor(2, y);
      backscreen.print("BNO085 not connected.");
      y += line_height;
      if (get_airdata_ok()) {
        backscreen.setCursor(2, y);
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.printf("Baro VSI:%+.2fm/s (only)", get_airdata_vspeed());
        y += line_height;
      }
      y += 3;
    }

    // Kalman フィルター設定
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- Kalman Config --");
    y += line_height;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    backscreen.printf("Q_vel:%.4f  Q_bias:%.4f", get_imu_kf_q_vel(), get_imu_kf_q_bias());
    y += line_height;

    backscreen.setCursor(2, y);
    backscreen.printf("R_baro:%.4fm2", get_imu_kf_R());
    y += line_height;

    {
      float _ha   = get_imu_horiz_accel();
      float _qeff = get_imu_kf_q_vel() + KF_HORIZ_ACCEL_GAIN * _ha * _ha;
      backscreen.setCursor(2, y);
      // Q_vel_eff が基準値より大きいとき（水平加速中）はオレンジで強調
      backscreen.setTextColor(_ha > 1.0f ? COLOR_ORANGE : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("HorizA:%.2f gain:%.3f Q_eff:%.4f", _ha, KF_HORIZ_ACCEL_GAIN, _qeff);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    }
    y += nextcolumn_height;

    // GNSS VSI KF セクション
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- GNSS VSI Fusion --");
    y += line_height;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    backscreen.printf("sAcc gate:<%.1fm/s  R_scale:%.1f", GNSS_VSI_SACC_MAX_MPS, GNSS_VSI_R_SCALE);
    y += line_height;

    float _sacc_mps = get_gps_sacc_mmps() / 1000.0f;
    float _r_vel    = _sacc_mps * _sacc_mps * GNSS_VSI_R_SCALE;
    if (_r_vel < 0.001f) _r_vel = 0.001f;  // 下限クランプ（imu.cpp と同じ）

    backscreen.setCursor(2, y);
    // sAcc が閾値に近づくとオレンジ、超えると赤で警告
    backscreen.setTextColor(_sacc_mps < GNSS_VSI_SACC_MAX_MPS * 0.7f ? COLOR_BLACK
                          : _sacc_mps < GNSS_VSI_SACC_MAX_MPS        ? COLOR_ORANGE
                          : COLOR_RED, COLOR_WHITE);
    backscreen.printf("sAcc:%.3fm/s  R_vel:%.4fm2", _sacc_mps, _r_vel);
    y += line_height;

    bool _gnss3d2 = get_gps_gnssFixOK() && get_gps_fixtype() >= 3;
    bool _sacc_ok2 = (_sacc_mps < GNSS_VSI_SACC_MAX_MPS);
    bool _gate2    = _gnss3d2 && _sacc_ok2;
    backscreen.setCursor(2, y);
    if (_gate2) {
      backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
      backscreen.printf("Gate:PASS sAcc=%.3f", _sacc_mps);
    } else {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      if (!_gnss3d2) backscreen.print("Gate:FAIL no 3D fix");
      else           backscreen.printf("Gate:FAIL sAcc%.3f>=%.1f", _sacc_mps, GNSS_VSI_SACC_MAX_MPS);
    }
    y += line_height;

    backscreen.setCursor(2, y);
    bool _gnss3d_kf = get_gps_gnssFixOK() && get_gps_fixtype() >= 3;
    if (get_imu_ok() && get_airdata_ok() && _gnss3d_kf) {
      backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
      backscreen.print("KF: Active (Baro+IMU+GNSS)");
    } else if (get_imu_ok() && get_airdata_ok()) {
      backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
      backscreen.print("KF: Active (Baro+IMU)");
    } else if (get_airdata_ok() && _gnss3d_kf) {
      backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      backscreen.print("KF: Active (Baro+GNSS)");
    } else if (get_airdata_ok()) {
      backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      backscreen.print("KF: Active (Baro only)");
    } else {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.print("KF: Inactive (no baro)");
    }
  }

  backscreen.pushSprite(0, 40);

  // ---- フッター: ページ番号 ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0, 0, SCREEN_WIDTH, COLOR_BLACK);
  header_footer.setTextColor(COLOR_GRAY, COLOR_WHITE);
  header_footer.setCursor(2, 5);
  header_footer.printf("Page %d/2  (short: next / long: back)", page % 2 + 1);
  header_footer.pushSprite(0, SCREEN_HEIGHT - 40);
}


//==================MODE DRAWS===============

// GPS 詳細画面を描画する（screen_mode == MODE_GPSDETAIL 時）。
// page % 3 == 0: スカイプロット画面（全衛星の方位・仰角を円形に描画）。
//   - 衛星種別（GPS=シアン, GLONASS=緑, GALILEO=青, QZSS=赤, BeiDou=オレンジ）で色分け。
//   - SNR=0 の衛星は小さい丸（追跡中だが信号弱い）。
//   - フッターに UTC 時刻と衛星数・フィックス状態を表示。
// page % 3 == 1: 最新 MAX_LAST_NMEA 件の UBX フレームラベルを表示。
//   1 秒以内に受信したものは黒、古いものは灰色で表示（鮮度の視覚化）。
// page % 3 == 2: GSA 情報画面（フィックス種別・DOP・緯度経度・使用衛星 PRN リスト）。
void draw_gpsdetail(int page) {

  header_footer.fillScreen(COLOR_WHITE);

  if (page % 3 == 1) {
    tft.fillRect(0, 20, 240, 320-20, COLOR_WHITE);
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    header_footer.setTextSize(2);
    header_footer.setCursor(1, 1);
    header_footer.println("GPS DETAIL 2: UBX frames");
    header_footer.pushSprite(0,0);

    tft.unloadFont();
    tft.setTextSize(1);
    int posy = 0;
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    for (int i = 0; i < MAX_LAST_NMEA; i++) {
      posy += 18;
      tft.setCursor(1, posy);
      if(get_gps_nmea_time(i) < millis()-1000){
        tft.setTextColor(COLOR_GRAY);
      }else{
        tft.setTextColor(COLOR_BLACK);
      }
      tft.println(get_gps_nmea(i));
    }
    tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  }
  if (page % 3 == 0) {
    // ヘッダー（凡例）: y を +10 してから pushSprite(0,-10) で push
    // → ヘッダー底辺が screen y=39 になり backscreen(y=40) との重複10px が解消する
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    header_footer.setTextSize(2);
    header_footer.setCursor(1, 11);  // +10
    header_footer.println("GPS DETAIL 1:CONSTELLATION");
    header_footer.setTextColor(COLOR_CYAN,   COLOR_WHITE); header_footer.setCursor(0, 28); header_footer.print("GPS ");  // +10
    header_footer.setTextColor(COLOR_GREEN,  COLOR_WHITE); header_footer.print("GLO ");
    header_footer.setTextColor(COLOR_BLUE,   COLOR_WHITE); header_footer.print("GAL ");
    header_footer.setTextColor(COLOR_RED,    COLOR_WHITE); header_footer.print("QZS ");
    header_footer.setTextColor(COLOR_ORANGE, COLOR_WHITE); header_footer.print("BEI ");
    header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に

    // フッター（UTC・衛星数）
    header_footer.fillSprite(COLOR_WHITE);
    header_footer.setCursor(23, 5);
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    GpsDate date = get_gpsdate();
    GpsTime time = get_gpstime();
    if (date.isValid() && time.isValid()) {
      header_footer.printf("%d.%d.%d %02d:%02d:%02d UTC", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second());
    }
    header_footer.setCursor(23, 17);
    header_footer.printf("%d sats, Fix=%s",get_gps_numsat(),get_gps_fix()?"yes":"no");
    header_footer.pushSprite(0,SCREEN_HEIGHT-40);

    backscreen.fillSprite(COLOR_BLACK);                         // 暗い背景
    backscreen.drawCircle(120, 120, 118, COLOR_GRAY);           // 外周円（地平線 0°）
    backscreen.drawCircle(120, 120, 79, COLOR_BRIGHTGRAY);      // 仰角 30° リング
    backscreen.drawCircle(120, 120, 39, COLOR_BRIGHTGRAY);      // 仰角 60° リング

    // N/E/S/W ラベルを外周に描画（N は赤）
    backscreen.setTextSize(1);
    backscreen.setTextColor(COLOR_RED,   COLOR_BLACK); backscreen.setCursor(116, 4);   backscreen.print("N");
    backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(116, 228); backscreen.print("S");
    backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(228, 116); backscreen.print("E");
    backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(4,   116); backscreen.print("W");


    for (int i = 0; i < MAX_SATELLITES; i++) {
      if (satellites[i].PRN == 0) continue;  // Skip empty entries
      //if(satellites[i].SNR == 0) continue; //Skip unknown signal strength.

      // Calculate position on display based on azimuth and elevation
      float azimuth = satellites[i].azimuth;
      float elevation = satellites[i].elevation;

      int x = calculateGPS_X(azimuth, elevation);
      int y = calculateGPS_Y(azimuth, elevation);

      uint16_t color;
      if (satellites[i].satelliteType == SATELLITE_TYPE_QZSS)        color = COLOR_RED;
      else if (satellites[i].satelliteType == SATELLITE_TYPE_GPS)     color = COLOR_CYAN;
      else if (satellites[i].satelliteType == SATELLITE_TYPE_GLONASS) color = COLOR_GREEN;
      else if (satellites[i].satelliteType == SATELLITE_TYPE_GALILEO) color = COLOR_BLUE;
      else if (satellites[i].satelliteType == SATELLITE_TYPE_BEIDOU)  color = COLOR_ORANGE;
      else                                                             color = COLOR_GRAY;

      // SNR あり: 塗りつぶし円、SNR なし: 枠線のみ（信号未確認の衛星）
      if (satellites[i].SNR > 0) {
        backscreen.fillCircle(x, y, 5, color);
      } else {
        backscreen.drawCircle(x, y, 4, color);
      }

      int textx = constrain(x + 6, 0, SCREEN_WIDTH - 20);
      if (satellites[i].PRN >= 10)
        textx -= 10;
      if (satellites[i].PRN >= 100)
        textx -= 10;
      backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK);
      backscreen.setCursor(textx, y - 3);
      backscreen.print(satellites[i].PRN);
    }
    backscreen.pushSprite(0,40);
  }

  // ---- page % 3 == 2: GSA 情報画面 ----------------------------------------
  // フィックス種別・DOP値・緯度経度・UTC時刻・測位使用衛星 PRN リストを表示する。
  if (page % 3 == 2) {
    tft.fillScreen(COLOR_WHITE);

    // ヘッダー
    header_footer.fillSprite(COLOR_WHITE);
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    header_footer.setTextSize(2);
    header_footer.setCursor(1, 1);
    header_footer.println("GPS DETAIL 3: GSA INFO");
    header_footer.pushSprite(0, 0);

    tft.unloadFont();
    tft.setTextSize(1);
    int y = 22;

    // ---- Fix Status ----
    int fixtype = get_gps_fixtype();
    const char* fixstr;
    uint16_t fixcolor;
    if      (fixtype == 3) { fixstr = "3D Fix"; fixcolor = COLOR_GREEN; }
    else if (fixtype == 2) { fixstr = "2D Fix"; fixcolor = COLOR_ORANGE; }
    else                   { fixstr = "No Fix"; fixcolor = COLOR_RED; }

    tft.setTextSize(2);
    tft.setTextColor(fixcolor, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.printf("Fix: %s", fixstr);
    y += 20;

    // ---- 衛星使用数 ----
    tft.setTextSize(1);
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.printf("Sats used: %d / total: %d", get_gsa_numsat(), get_gps_numsat());
    y += 14;

    // ---- DOP ----
    // DOP 値: 1未満=非常に良好(緑), 2未満=良好(黒), 5未満=やや悪い(オレンジ), 5以上=悪い(赤)
    auto dopColor = [](float d) -> uint16_t {
      if (d < 1.0f) return COLOR_GREEN;
      if (d < 2.0f) return COLOR_BLACK;
      if (d < 5.0f) return COLOR_ORANGE;
      return COLOR_RED;
    };
    float pdop = get_gps_pdop();
    float hdop = get_gps_hdop();
    float vdop = get_gps_vdop();

    tft.setCursor(1, y);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("PDOP:");
    tft.setTextColor(dopColor(pdop), COLOR_WHITE);
    tft.printf("%.2f  ", pdop);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("HDOP:");
    tft.setTextColor(dopColor(hdop), COLOR_WHITE);
    tft.printf("%.2f", hdop);
    y += 14;

    tft.setCursor(1, y);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("VDOP:");
    tft.setTextColor(dopColor(vdop), COLOR_WHITE);
    tft.printf("%.2f", vdop);
    y += 18;

    // ---- 緯度・経度 ----
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.printf("Lat: %.6f", get_gps_lat());
    y += 12;
    tft.setCursor(1, y);
    tft.printf("Lon: %.6f", get_gps_lon());
    y += 18;

    // ---- UTC 時刻・日付 ----
    GpsDate date = get_gpsdate();
    GpsTime time = get_gpstime();
    tft.setCursor(1, y);
    if (date.isValid() && time.isValid()) {
      tft.printf("UTC %04d-%02d-%02d  %02d:%02d:%02d",
        date.year(), date.month(), date.day(),
        time.hour(), time.minute(), time.second());
    } else {
      tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
      tft.print("UTC: --");
    }
    y += 18;

    // ---- GPS ボーレート ----
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.printf("Baud: %lu bps", get_gps_baudrate());
    y += 14;

    // ---- 測位使用衛星 PRN リスト ----
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.print("PRNs in fix:");
    y += 12;
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    tft.setCursor(1, y);
    int col = 0;
    for (int i = 0; i < 12; i++) {
      int prn = get_gsa_prn(i);
      if (prn == 0) continue;
      if (col > 0) tft.print(" ");
      tft.printf("%3d", prn);
      col++;
      if (col == 8) { y += 12; tft.setCursor(1, y); col = 0; }
    }

    // ---- UBX NAV-PVT 精度情報 ----------------------------------------
    // gnssFixOK・hAcc・vAcc・sAcc を色分け表示する。
    // 閾値: hAcc <5m=緑 / <25m=橙 / ≥25m=赤、vAcc <10m=緑 / <50m=橙 / ≥50m=赤
    //        sAcc <1m/s=緑 / <3m/s=橙 / ≥3m/s=赤
    y += 16;
    tft.drawFastHLine(0, y - 4, SCREEN_WIDTH, COLOR_GRAY);  // 区切り線

    // gnssFixOK
    tft.setCursor(1, y);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("gnssFixOK:");
    bool fixOK = get_gps_gnssFixOK();
    tft.setTextColor(fixOK ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
    tft.print(fixOK ? "YES" : "NO");
    y += 12;

    // hAcc / vAcc を横並び表示
    float hacc_m = get_gps_hacc_mm() / 1000.0f;
    float vacc_m = get_gps_vacc_mm() / 1000.0f;
    uint16_t haccColor = (hacc_m < 5.0f)  ? COLOR_GREEN : (hacc_m < 25.0f) ? COLOR_ORANGE : COLOR_RED;
    uint16_t vaccColor = (vacc_m < 10.0f) ? COLOR_GREEN : (vacc_m < 50.0f) ? COLOR_ORANGE : COLOR_RED;

    tft.setCursor(1, y);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("hAcc:");
    tft.setTextColor(haccColor, COLOR_WHITE);
    tft.printf("%.1fm  ", hacc_m);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("vAcc:");
    tft.setTextColor(vaccColor, COLOR_WHITE);
    tft.printf("%.1fm", vacc_m);
    y += 12;

    // sAcc（速度精度）
    float sacc_mps = get_gps_sacc_mmps() / 1000.0f;
    uint16_t saccColor = (sacc_mps < 1.0f) ? COLOR_GREEN : (sacc_mps < 3.0f) ? COLOR_ORANGE : COLOR_RED;
    tft.setCursor(1, y);
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("sAcc:");
    tft.setTextColor(saccColor, COLOR_WHITE);
    tft.printf("%.2fm/s (%.1fkm/h)", sacc_mps, sacc_mps * 3.6f);
  }
}


// フラッシュ（ROM）に書き込まれた地図ポリゴンと SD から読み込んだ追加マップの一覧を表示する。
// "MAPLIST FLSH:N/SD:M (page/total)" 形式でヘッダーに表示。
// フラッシュマップはページ 0 に全表示、SD マップは 30 エントリ / ページでページング表示。
// MAPLIST の 1 ページに入る行数。
// backscreen(240px) を y=40 に貼るので、10px/行で 24 行が上限。
// 以前は 30 行想定で、SD 地図が増えると下がはみ出して切れていた。
#define MAPLIST_ROWS 24

// ページ 0 の先頭に出す OSM 内訳が何行になるかを返す。
// 実際の描画（下の draw_maplist_mode）と同じ数え方をすること。
// 固定値にすると、収録クラス数が変わったときに枠計算とズレる。
static int maplist_osm_rows() {
  int cls = 0;
  for (int c = 0; c < VM_CLASS_COUNT; c++) if (vm_class_points[c] != 0) cls++;
  return 2                            // 日付 + 合計
       + (VM_LOD_COUNT + 1) / 2       // LOD は 2 個ずつ 1 行
       + 1                            // "pts x1000:" の見出し
       + (cls + 2) / 3;               // クラスは 3 個ずつ 1 行
}

void draw_maplist_mode(int maplist_page) {

  mapdata** mapdatas = flashmaps;   // navdata.cpp の共有配列（draw_FlashMaps と同じもの）
  int sizeof_mapflash = flashmap_count;

  const int osm_rows = maplist_osm_rows();
  int pagetotal = 1 + (osm_rows + sizeof_mapflash + mapdata_count) / MAPLIST_ROWS;
  int pagenow = maplist_page % pagetotal;

  // ヘッダー: スプライト経由で描画（tft 直接描画によるちかちかを防ぐ）
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);  // +10px（variodetail と同じオフセット）
  header_footer.printf("MAPLIST FLSH:%d/SD:%d (%d/%d)", sizeof_mapflash, mapdata_count, pagenow + 1, pagetotal);
  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に合わせる

  // コンテンツ: backscreen スプライト経由で描画
  backscreen.fillScreen(COLOR_WHITE);
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  int posy = 0;
  backscreen.setCursor(1, posy);

  if (pagenow == 0) {
    // ---- 内蔵ベクタ地図（OpenStreetMap 由来）の内訳 ----
    // どの範囲がどれだけの容量で入っているかを実機で確認できるようにする。
    // 数値は vectormap_data.cpp が生成時に埋め込んだもの。
    backscreen.setTextColor(COLOR_BLUE, COLOR_WHITE);
    backscreen.printf("OSM(ODbL) %s", vm_data_source_date);
    posy += 10; backscreen.setCursor(1, posy);
    backscreen.printf(" total %luKB / %u tiles",
                      (unsigned long)(vm_blob_size / 1024), vm_tile_count);
    posy += 10; backscreen.setCursor(1, posy);
    // LOD ごとに「使用する最小 scale・タイル数・容量」を 2 行に詰めて出す
    for (int l = 0; l < VM_LOD_COUNT; l += 2) {
      for (int k = l; k < l + 2 && k < VM_LOD_COUNT; k++) {
        if (vm_lod_min_scale[k] > 0)
          backscreen.printf(" L%d>%.3g %ut %luK", k, vm_lod_min_scale[k],
                            vm_lod_tiles[k], (unsigned long)(vm_lod_bytes[k] / 1024));
        else
          backscreen.printf(" L%d all %ut %luK", k,
                            vm_lod_tiles[k], (unsigned long)(vm_lod_bytes[k] / 1024));
      }
      posy += 10; backscreen.setCursor(1, posy);
    }
    // 地物クラス別の点数（千点単位。点が多いほどその地物が細かい）。
    // 画面幅は 240px = 約 40 文字しかないので、クラス名は 4 文字に切って 3 個ずつ折り返す。
    backscreen.print(" pts x1000:");
    posy += 10; backscreen.setCursor(1, posy);
    int printed = 0;
    for (int c = 0; c < VM_CLASS_COUNT; c++) {
      if (vm_class_points[c] == 0) continue;   // 収録していないクラスは出さない
      backscreen.printf(" %.4s%lu", vm_class_names[c], (unsigned long)(vm_class_points[c] / 1000));
      if (++printed % 3 == 0) { posy += 10; backscreen.setCursor(1, posy); }
    }
    if (printed % 3 != 0) { posy += 10; backscreen.setCursor(1, posy); }

    // ---- コース座標（パイロン等）----
    // SD の override_pilon_coordinate.csv で上書きできるので、実際に使われている
    // 値をここで確認できるようにする。上書きが効いていれば見出しが OVERRIDE になる。
    backscreen.setTextColor(pilon_override_loaded ? COLOR_MAGENTA : COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("COURSE %s", pilon_override_loaded ? "SD OVERRIDE" : "built-in");
    posy += 10; backscreen.setCursor(1, posy);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    {
      const struct { const char* n; double la, lo; } pts[] = {
        { "PLA ", pla_lat,         pla_lon         },
        { "NPIL", pilon_north_lat, pilon_north_lon },
        { "SPIL", pilon_south_lat, pilon_south_lon },
        { "1KM ", pilon_1km_lat,   pilon_1km_lon   },
        { "TAKE", takeshima_lat_v, takeshima_lon_v },
      };
      for (unsigned i = 0; i < sizeof(pts)/sizeof(pts[0]); i++) {
        backscreen.printf(" %s %10.6f,%11.6f", pts[i].n, pts[i].la, pts[i].lo);
        posy += 10; backscreen.setCursor(1, posy);
      }
    }

    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

    for (int i = 0; i < sizeof_mapflash; i++) {
      backscreen.printf("FLSH %d: %s,%4.2f,%4.2f,%d", mapdatas[i]->id, mapdatas[i]->name, mapdatas[i]->cords[0][0], mapdatas[i]->cords[0][1], mapdatas[i]->size);
      posy += 10;
      backscreen.setCursor(1, posy);
    }
  }

  // このページに表示する SD 地図の範囲を決める（sd_start_index 以上 sd_end_index 未満）。
  // 終端を決めずに mapdata_count まで回すと、どのページでも残り全件を描いてしまい
  // ページを送っても同じ内容が続いて見える（添字自体は範囲内なので表示上の問題のみ）。
  int sd_start_index = 0;
  int sd_rows = MAPLIST_ROWS;
  if (pagenow > 0) {
    sd_start_index = MAPLIST_ROWS * pagenow - sizeof_mapflash - osm_rows;
    if (sd_start_index < 0) sd_start_index = 0;
  } else {
    sd_rows = MAPLIST_ROWS - sizeof_mapflash - osm_rows;  // ページ 0 は OSM 内訳とフラッシュ地図の分だけ枠が減る
    if (sd_rows < 0) sd_rows = 0;
  }
  int sd_end_index = sd_start_index + sd_rows;
  if (sd_end_index > mapdata_count) {
    sd_end_index = mapdata_count;
  }

  for (int i = sd_start_index; i < sd_end_index; i++) {
    if (extramaps[i].size <= 1) {
      continue;
    }
    double lon1 = extramaps[i].cords[0][0];
    double lat1 = extramaps[i].cords[0][1];
    char name_firstchar = extramaps[i].name[0];

    backscreen.printf("SD %d:%s,%4.2f,%4.2f,%d,%c", i, extramaps[i].name, lat1, lon1, extramaps[i].size, name_firstchar);
    posy += 10;
    backscreen.setCursor(1, posy);
  }

  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.pushSprite(0, 40);  // ヘッダー底辺(y=39)の直下から描画

}




// CPU 温度の判定しきい値 [℃]
#define CPU_TEMP_WARN_C 50.0f  // これを超えるとオレンジ（要注意）
#define CPU_TEMP_HOT_C  55.0f  // これを超えると赤（高温）

// 設定画面フッターの電池アイコン色を返す。
// NAV 画面ヘッダーのバッテリー残量表示（draw_header 内）と同じ配色に合わせている:
//   USB 接続中           : 緑（"USB" 表示が緑）
//   BAT_LOW_VOLTAGE 以下 : 赤（残量警告）
//   BAT_HALF_VOLTAGE 未満: マゼンタ（50%未満）
//   それ以上             : 緑
uint16_t battery_status_color() {
  if (digitalRead(USB_DETECT))
    return COLOR_GREEN;
  float input_voltage = get_input_voltage();
  if (input_voltage <= BAT_LOW_VOLTAGE)
    return COLOR_RED;
  else if (input_voltage < BAT_HALF_VOLTAGE)
    return COLOR_MAGENTA;
  else
    return COLOR_GREEN;
}

// 設定画面フッターの CPU 温度アイコン色を返す。
uint16_t cpu_temp_status_color(float cpu_temp) {
  if (cpu_temp >= CPU_TEMP_HOT_C)
    return COLOR_RED;
  else if (cpu_temp >= CPU_TEMP_WARN_C)
    return COLOR_ORANGE;
  else
    return COLOR_GREEN;
}

// 設定画面を描画する（screen_mode == MODE_SETTING 時）。
// selectedLine: 現在値が変更中の行（ダイヤルを押している間）→ 赤色表示。
// cursorLine  : カーソル位置の行 → マゼンタ色表示。その他は黒。
// menu_settings[].getLabel() で現在の設定値を含むラベル文字列を取得して表示する。
// iconColor() が返す色の丸アイコンを各行の右端に描画する（NULL の場合は描画しない）。
// フッターにはバッテリー残量推定・CPU 温度・空きヒープ（デバッグビルド時のみ）を表示し、
// 電池・CPU 温度の行にはメニューと同じ丸アイコンを右寄せで表示する。
void draw_setting_mode(int selectedLine, int cursorLine) {
  // 行間 16px。項目数 × separation が backscreen の高さ (BACKSCREEN_SIZE=240) を超えると
  // 最終行（Save & Exit）が表示されなくなるため、項目を増やす場合はここも調整すること。
  // 現在の有効項目は 14（BRIGHTNESS は BRIGHTNESS_SETTING_AVAIL 未定義のため無効）。
  //   14 x 16 = 224px < 240px。18px のままだと 252px となり Save & Exit が切れる。
  const int separation = 16;
  tft.loadFont(AA_FONT_SMALL);
  textmanager.drawText(SETTING_TITLE, 2, 5, 5, COLOR_BLUE, "SETTINGS");
  tft.setTextColor(COLOR_BLACK);
  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);  // variodetail から戻った時に unloadFont() されている場合があるため明示的にロード

  for (int i = 0; i < setting_size; ++i) {
    uint16_t col = COLOR_BLACK;
    if (cursorLine == i) {
      col = (selectedLine == i) ? COLOR_RED : COLOR_MAGENTA;
    }
    backscreen.setCursor(10,i * separation);
    backscreen.setTextColor(col,COLOR_WHITE);

    if(menu_settings[i].iconColor != nullptr)
      backscreen.fillCircle(SCREEN_WIDTH-7,i * separation+7, 5, menu_settings[i].iconColor());

    backscreen.print(menu_settings[i].getLabel(selectedLine == i).c_str());
    //textmanager.drawTextf(menu_settings[i].id, 2, 10, startY + i * separation, col, menu_settings[i].getLabel(selectedLine == i).c_str());
  }
  backscreen.pushSprite(0,30);

  header_footer.unloadFont();
  header_footer.fillScreen(COLOR_WHITE);

  header_footer.setTextSize(1);


  header_footer.setCursor(2, 5);
  header_footer.setTextColor(COLOR_GRAY);
  if (digitalRead(USB_DETECT)) {
    double input_voltage = get_input_voltage();
    header_footer.printf("Battery: Charging %.2fV", input_voltage);
  }else{
    double input_voltage = get_input_voltage();
    int battery_minutes = max(0,(input_voltage-3.4)/(4.2-3.4)*60*4);
    header_footer.printf("Battery Time:Approx. %dh %dm (%.2fV)",battery_minutes/60,((int)(battery_minutes%60)/10)*10, input_voltage);
  }
  // 電池残量の丸アイコン（メニュー項目と同じ x 位置・大きさで右寄せ）
  header_footer.fillCircle(SCREEN_WIDTH-7, 9, 5, battery_status_color());

  header_footer.setCursor(2, 18);
  {
    float cpu_temp = analogReadTemp();
    header_footer.setTextColor(cpu_temp >= CPU_TEMP_HOT_C ? COLOR_RED : COLOR_GRAY);
    header_footer.printf("CPU %.1fC", cpu_temp);
    if (get_airdata_ok()) {
      float sensor_temp = get_airdata_temperature();
      header_footer.setTextColor(sensor_temp >= 45.0f ? COLOR_RED : COLOR_GRAY);
      header_footer.printf("  Sensor %.1fC", sensor_temp);
    }
    // CPU 温度の丸アイコン（電池の丸の真下）
    header_footer.fillCircle(SCREEN_WIDTH-7, 22, 5, cpu_temp_status_color(cpu_temp));
  }

  #ifndef RELEASE
  header_footer.setCursor(145, 18);
  header_footer.printf("FreeHeap %d%% ",rp2040.getFreeHeap()*100/rp2040.getTotalHeap());
  #endif

  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.pushSprite(0,SCREEN_HEIGHT-40);
}