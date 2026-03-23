// ============================================================
// File    : display_tft.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : TFTディスプレイ描画の実装。
//           地図（ポリゴン・Googleマップ画像）、コンパス、
//           飛行コース矢印、ヘッダー/フッター、設定画面、
//           GPSDetail/SDDetail/マップリスト画面など全UI描画。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================
// Updates TFT display using TFT-eSPI library.

#include <cstring>  // for strlen and strcpy
#include <string>
#include <cstdlib>  // for malloc and free

#include "settings.h"
#include "display_tft.h"
#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "font_data.h"
#include "sound.h"
#include "button.h"
#include "airdata.h"
#include "imu.h"

// フォント定義: TFT_eSPI のカスタムフォント（PROGMEM 格納）
#define AA_FONT_SMALL NotoSansBold15   // アンチエイリアス付き小サイズフォント（地図テキスト等）
#define NM_FONT_MEDIUM Arial_Black22   // 中サイズフォント（未使用）
#define NM_FONT_LARGE Arial_Black56    // 大サイズフォント（ヘッダーの速度・磁方位表示）

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

// ポリラインの描画と消去を管理するクラス。
// addStroke() でストロークを追加し、addPointToStroke() で頂点を追加する。
// drawCurrentStroke() で最後に追加したストロークだけを描画、
// drawAllStrokes() で全ストロークを白で上書き（消去）する。
// backscreen スプライトへ drawLine / drawWideLine で線を引く。
class StrokeManager {

  struct Stroke {
    stroke_group id;      // ストロークの種類識別子（stroke_group 列挙型）
    Point* points;        // 頂点座標配列（heap 確保）
    int pointCount;       // 現在の頂点数
    int maxPoints;        // 最大頂点数（addStroke 時に確保）
    int thickness;        // 線の太さ（1=drawLine, >1=drawWideLine）
  };

  Stroke* strokes;
  int strokeCount;
  int maxStrokes;

public:
  StrokeManager(int maxStrokes) {
    this->maxStrokes = maxStrokes;
    this->strokes = new Stroke[maxStrokes];
    this->strokeCount = 0;
  }

  ~StrokeManager() {
    for (int i = 0; i < strokeCount; i++) {
      free(strokes[i].points);
    }
    delete[] strokes;
  }

  bool addStroke(stroke_group id, int maxPoints, int thickness = 1) {
    if (strokeCount >= maxStrokes) {
      DEBUGW_PLN(20240912, id);
      DEBUGW_PLN(20240912, maxPoints);
      DEBUGW_PLN(20240912, "ERR max stroke reached");
      enqueueTask(createLogSdfTask("ERR max stroke reached(id,max) %d,%d", id, maxPoints));
      return false;  // No more space for new strokes
    }
    strokes[strokeCount].points = (Point*)malloc(maxPoints * sizeof(Point));
    
    if (strokes[strokeCount].points == nullptr) {
      DEBUG_P(20240912, id);
      DEBUG_PLN(20240912, "malloc fail");
      enqueueTask(createLogSdfTask("malloc failed:adding stroke:ID=%d", id));
      return false;  // Memory allocation failed
    }
    strokes[strokeCount].pointCount = 0;
    strokes[strokeCount].maxPoints = maxPoints;
    strokes[strokeCount].id = id;
    strokes[strokeCount].thickness = thickness;
    strokeCount++;
    return true;
  }


  bool addPointToStroke(int x, int y) {
    if (strokeCount == 0) {
      return false;  // No strokes to add points to
    }
    if (strokes[strokeCount - 1].pointCount >= strokes[strokeCount - 1].maxPoints) {
      return false;  // No more space for new points in this stroke
    }
    strokes[strokeCount - 1].points[strokes[strokeCount - 1].pointCount].x = x;
    strokes[strokeCount - 1].points[strokes[strokeCount - 1].pointCount].y = y;
    strokes[strokeCount - 1].pointCount++;
    return true;
  }

  void drawCurrentStroke(int col){
    int strkid = strokeCount - 1;
    if(strokeCount == 0 || strokes[strkid].pointCount < 2){
      return;
    }
    for(int i = 0; i < strokes[strkid].pointCount-1; i++){
      if (!strokes[strkid].points[i].isOutsideTft() || !strokes[strkid].points[i+1].isOutsideTft()) {
        if (strokes[strkid].thickness == 1)
          backscreen.drawLine(strokes[strkid].points[i].x, strokes[strkid].points[i].y, strokes[strkid].points[i + 1].x, strokes[strkid].points[i + 1].y, col);
        else
          backscreen.drawWideLine(strokes[strkid].points[i].x, strokes[strkid].points[i].y, strokes[strkid].points[i + 1].x, strokes[strkid].points[i + 1].y, strokes[strkid].thickness, col);
      }
    }
  }


  void removeAllStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      if(strokes[i].points != nullptr){
        free(strokes[i].points);  // Free allocated memory for each stroke
      }
      strokes[i].points = nullptr;
      strokes[i].pointCount = 0;
      strokes[i].maxPoints = 0;
    }
    strokeCount = 0;
  }


  void drawAllStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        if (strokes[i].thickness == 1)
          backscreen.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, COLOR_WHITE);
        else
          backscreen.drawWideLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, strokes[i].thickness, COLOR_WHITE);
      }
    }
  }
};




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
// 偏差補正: +8.0° を加えて磁北を表示する（日本の平均的な磁気偏差）。
void draw_compass(float truetrack, uint16_t col) {
  backscreen.setTextWrap(false);
  int dist = 73;
  if (is_northupmode()) {
#ifdef TFT_USE_ST7735
    dist = 50;
#else
    dist = 100;
#endif
  }
  int centerx = BACKSCREEN_SIZE / 2;
  int centery = get_self_cy();  // TRACKUP=180, NORTHUP=120
  //Varietion 8 degrees.
  float radian = deg2rad(truetrack + 8.0);
  float radian45offset = deg2rad(truetrack + 8.0 + 45);
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
      if (draw_counter >= 2) {
        backscreen.loadFont(AA_FONT_SMALL);
        return;
      }
    }
  }
}

// ===== マップ描画管理変数 =====
bool fresh = false;         // 地図の強制再描画フラグ（スケール変更時などに true にする）
float last_scale = 1.0;     // 前回描画時のスケール（変化を検知するための記憶値）
float last_up = 0;          // 前回描画時の mapUpDirection
bool nomap_drawn = true;    // いずれのマップポリゴンも描画されていない場合 true（テキスト表示の切り替えに使用）







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

extern int course_warning_index;
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
  backscreen.drawWideLine(cx - arm, cy - arm, cx + arm, cy + arm, 3, COLOR_GRAY);
  backscreen.drawWideLine(cx + arm, cy - arm, cx - arm, cy + arm, 3, COLOR_GRAY);
}

// HDOP 不良時に飛行機マークの代わりに自機位置の不確かさ円を描画する。
// 動作:
//   1. 中心 (120,120) に青い塗りつぶし小円（HDOP_CENTER_DOT_RADIUS px）で位置を示す。
//   2. 誤差半径 = hdop × GPS_BASE_ACCURACY_M [m] をピクセル換算して青い輪郭円を描画。
//   3. 輪郭円半径 < HDOP_MIN_CIRCLE_RADIUS の場合は輪郭円を省略し、"H5.2" 形式のテキストで HDOP 値を表示する。
void draw_hdop_circle(double scale, float hdop) {
  const int cx = BACKSCREEN_SIZE / 2;  // = 120（X は常に中央）
  const int cy = get_self_cy();        // TRACKUP=180, NORTHUP=120

  // 誤差半径: [m] → [km] → ピクセル（四捨五入で正確な円サイズに）
  int r = (int)roundf((hdop * GPS_BASE_ACCURACY_M / 1000.0f) * (float)scale);

  // 中心位置を示す小さい塗りつぶし青円
  backscreen.fillCircle(cx, cy, HDOP_CENTER_DOT_RADIUS, COLOR_BLUE);

  if (r >= HDOP_MIN_CIRCLE_RADIUS) {
    // 不確かさ範囲を示す輪郭青円
    backscreen.drawCircle(cx, cy, r, COLOR_BLUE);
  } else {
    // 円が小さすぎて見えないため、HDOP 数値をテキストで表示（例: "H5.2"）
    char buf[10];
    snprintf(buf, sizeof(buf), "H%.1f", hdop);
    backscreen.setTextSize(1);
    backscreen.setTextColor(COLOR_BLUE, COLOR_BLACK);
    backscreen.setCursor(cx + HDOP_CENTER_DOT_RADIUS + 2, cy - 4);
    backscreen.print(buf);
  }
}

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
    
    int left_wing_tip_x = BACKSCREEN_SIZE / 2 - TRIANGLE_HWIDTH*2 * cos(tt_radians);
    int left_wing_tip_y = BACKSCREEN_SIZE / 2 - TRIANGLE_HWIDTH*2 * sin(tt_radians);
    int right_wing_tip_x = BACKSCREEN_SIZE / 2 + TRIANGLE_HWIDTH*2 * cos(tt_radians);
    int right_wing_tip_y = BACKSCREEN_SIZE / 2 + TRIANGLE_HWIDTH*2 * sin(tt_radians);
    int left_tail_tip_x = BACKSCREEN_SIZE / 2 + (-TRIANGLE_HWIDTH / 2) * cos(tt_radians) - (0.5 * TRIANGLE_SIZE) * sin(tt_radians);
    int left_tail_tip_y = BACKSCREEN_SIZE / 2 + (-TRIANGLE_HWIDTH / 2) * sin(tt_radians) + (0.5 * TRIANGLE_SIZE) * cos(tt_radians);
    int right_tail_tip_x = BACKSCREEN_SIZE / 2 + (TRIANGLE_HWIDTH / 2) * cos(tt_radians) - (0.5 * TRIANGLE_SIZE) * sin(tt_radians);
    int right_tail_tip_y = BACKSCREEN_SIZE / 2 + (TRIANGLE_HWIDTH / 2) * sin(tt_radians) + (0.5 * TRIANGLE_SIZE) * cos(tt_radians);
    int body_tail_tip_x = BACKSCREEN_SIZE / 2 - TRIANGLE_SIZE/2 * sin(tt_radians);
    int body_tail_tip_y = BACKSCREEN_SIZE / 2 + TRIANGLE_SIZE/2 * cos(tt_radians);
    backscreen.drawWideLine(left_wing_tip_x, left_wing_tip_y, right_wing_tip_x, right_wing_tip_y, 3, COLOR_BLACK);
    backscreen.drawWideLine(left_tail_tip_x, left_tail_tip_y, right_tail_tip_x, right_tail_tip_y, 2, COLOR_BLACK);
    backscreen.drawWideLine(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2, body_tail_tip_x, body_tail_tip_y, 2, COLOR_BLACK);
    

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

    // ── 機体アイコン（固定・常に上向き）──
    // TRACKUPでは地図が回転して機首が常に画面上を向くため、アイコンは固定座標で描く
    int shortening = 15;
    backscreen.drawFastVLine(240 / 2,     shortening, cy - shortening - 5, COLOR_BLACK);
    backscreen.drawFastVLine(240 / 2 + 1, shortening, cy - shortening - 5, COLOR_BLACK);
    backscreen.drawWideLine(BACKSCREEN_SIZE / 2 - TRIANGLE_HWIDTH*2, cy, BACKSCREEN_SIZE / 2 + TRIANGLE_HWIDTH*2, cy, 3, COLOR_BLACK);
    backscreen.drawWideLine(BACKSCREEN_SIZE / 2 - TRIANGLE_HWIDTH/2, cy + TRIANGLE_SIZE/2, BACKSCREEN_SIZE/2 + TRIANGLE_HWIDTH/2, cy + TRIANGLE_SIZE/2, 2, COLOR_BLACK);
    backscreen.drawWideLine(BACKSCREEN_SIZE / 2, cy, BACKSCREEN_SIZE / 2, cy + TRIANGLE_SIZE/2, 2, COLOR_BLACK);

    // ── True Track 維持針（last_tone_tt: 最後にトーンが鳴った時点のTT方向）──
    // TRACKUPでは機首=画面上=0°なので、last_tone_tt の画面上の角度は (last_tone_tt - ttrack)
    float screen_tone = last_tone_tt - ttrack;
    float tone_left   = deg2rad(screen_tone - 15);
    float tone_right  = deg2rad(screen_tone + 15);
    float tone_center = deg2rad(screen_tone);
    float tx1 = (nlen-6) * sin(tone_left)   + BACKSCREEN_SIZE/2;
    float ty1 = (nlen-6) * -cos(tone_left)  + cy;
    float tx2 = (nlen-6) * sin(tone_right)  + BACKSCREEN_SIZE/2;
    float ty2 = (nlen-6) * -cos(tone_right) + cy;
    float tx3 = nlen     * sin(tone_center) + BACKSCREEN_SIZE/2;
    float ty3 = nlen     * -cos(tone_center)+ cy;
    if (trackwarning_until < millis()) {
      backscreen.drawTriangle(tx1,ty1,tx2,ty2,tx3,ty3, COLOR_BLACK);
    } else {
      backscreen.fillTriangle(tx1,ty1,tx2,ty2,tx3,ty3, (millis()/1000)%2==0?COLOR_RED:COLOR_BLACK);
    }

    // ── コース逸脱アーク + 誘導三角形 ──
    // drawArcの座標系: 0°=6時方向（画面下）、時計回り。機首方向（画面上）= drawArc 180°
    // NORTHUPの (ttrack+180)%360 は方位角→drawArc角への変換。TRACKUPは機首固定なので 180 を使う。
    if ((millis()/1000)%3 != 0) {
      float arc_factor = course_warning_index / 900.0;
      if (steer_angle > 0)
        backscreen.drawArc(240/2, cy, (nlen-21), (nlen-23), 180, (180 + (int)(arc_factor*steer_angle)) % 360, COLOR_RED, COLOR_WHITE);
      else
        backscreen.drawArc(240/2, cy, (nlen-21), (nlen-23), (360 + 180 + (int)(arc_factor*steer_angle)) % 360, 180, COLOR_RED, COLOR_WHITE);

      // 誘導三角形: 機首=0ラジアン（画面上）を基準に offset。NORTHUPの tt_radians に相当
      if (abs(steer_angle) > 15) {
        double steer_triangle_start_rad = (steer_angle<0?-0.1:0.1);
        double steer_triangle_end_rad   = (steer_angle<0?-0.35:0.35);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + cy;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if (abs(steer_angle) > 55) {
        double steer_triangle_start_rad = (steer_angle<0?-0.7:0.7);
        double steer_triangle_end_rad   = (steer_angle<0?-0.95:0.95);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + cy;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if (abs(steer_angle) > 100) {
        double steer_triangle_start_rad = (steer_angle<0?-1.3:1.3);
        double steer_triangle_end_rad   = (steer_angle<0?-1.55:1.55);
        float x1 = (nlen-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (nlen-12) * -cos(steer_triangle_start_rad) + cy;
        float x2 = (nlen-32) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (nlen-32) * -cos(steer_triangle_start_rad) + cy;
        float x3 = (nlen-22) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (nlen-22) * -cos(steer_triangle_end_rad) + cy;
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
  backscreen.drawWideLine(BACKSCREEN_SIZE/2, get_self_cy(), targetpoint.x, targetpoint.y, 5, COLOR_MAGENTA);
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
    backscreen.drawWideLine(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
  }else{
    backscreen.drawWideLine(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    //
    calculatePointC(dest_lat, dest_lon, center_lat, center_lon, -distance, newlat, newlon);
    cord_tft outside_tft = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
    backscreen.drawLine(goal.x, goal.y,outside_tft.x,outside_tft.y, COLOR_MAGENTA);
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
  backscreen.drawWideLine(240 / 2, get_self_cy(), goal.x, goal.y, thickness, COLOR_MAGENTA);
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
    backscreen.drawWideLine(p0.x,p0.y,BACKSCREEN_SIZE/2,get_self_cy(),2,COLOR_GREEN);
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
      backscreen.drawWideLine(p0.x,p0.y,p1.x,p1.y,2,COLOR_GREEN);
    }
    
  }
}

// 日本列島の概略ポリゴン（map_japan1〜4）を緑で描画する。広域表示時に使用。
void draw_Japan(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan1, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan2, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan3, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan4, COLOR_GREEN);
  nomap_drawn = false;
}

// SD カードの mapdata.csv から読み込んだ追加ポリゴン（extramaps[]）を描画する。
// ポリゴンの最初の座標点が現在位置から 1°×1° 以内にある場合のみ描画する（遠方の不要描画を省略）。
// ポリゴンの描画色はマップ名の先頭文字で決まる:
//   r=赤, o=オレンジ, g=明るいグレー, m=マゼンタ, c=シアン, b=青, その他=緑
void draw_ExtraMaps(double center_lat, double center_lon, float scale, float up) {
  for (int i = 0; i < mapdata_count; i++) {
    if (extramaps[i].size <= 1) {
      continue;
    }
    double lon1 = extramaps[i].cords[0][0];
    double lat1 = extramaps[i].cords[0][1];
    if (check_within_latlon(1, 1, lat1, get_gps_lat(), lon1, get_gps_lon())) {
      int col = COLOR_GREEN;
      char name_firstchar = extramaps[i].name[0];
      if (name_firstchar == 'r') {
        col = COLOR_RED;
      } else if (name_firstchar == 'o') {
        col = COLOR_ORANGE;
      } else if (name_firstchar == 'g') {
        col = COLOR_BRIGHTGRAY;
      } else if (name_firstchar == 'm') {
        col = COLOR_MAGENTA;
      } else if (name_firstchar == 'c') {
        col = COLOR_CYAN;
      } else if (name_firstchar == 'b') {
        col = COLOR_BLUE;
      }
      draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &extramaps[i], col);
      nomap_drawn = false;
    }
  }
}

void draw_Shinura(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_shinura, COLOR_GREEN);

  nomap_drawn = false;
}

// 琵琶湖エリアのポリゴン（湖岸・島）と 10km コースの基準線/円を描画する。
// gmap_drawed=false のときは fill_sea_land() で水面・陸地の色塗りも行う
//（BMP 地図画像がない状態でも水面を水色、陸地をオレンジで色分けするため）。
// gps_loop(2/3) を fill_sea_land の前後で呼んでいるのは、色塗りに時間がかかるためその間も GPS を処理するため。
void draw_Biwako(double center_lat, double center_lon, float scale, float up, bool gmap_drawed) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_biwako, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_takeshima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_chikubushima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_okishima, COLOR_GREEN);
  if(!gmap_drawed){//if (! && !isTaskInQueue(TASK_LOAD_MAPIMAGE))
    //Run gps_loop 
    gps_loop(2);
    fill_sea_land(center_lat, center_lon, scale, up);
    gps_loop(3);
  }
  // draw_pilon_takeshima_line はマゼンタライン描画後に呼ぶため、ここでは描画しない。
  // GPS_TFT_map.ino のレイヤー4（目的地ライン）直後で呼ばれる。
  nomap_drawn = false;
}

// 大阪（阪大エリア）のポリゴン群を描画する。
// 外周（シアン）、高速道路（マゼンタ）、構内エリア（グレー）、鉄道（オレンジ）、カフェ（緑）を色分け表示。
void draw_Osaka(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaioutside, COLOR_CYAN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway, COLOR_MAGENTA);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway2, COLOR_MAGENTA);

  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside1, COLOR_BRIGHTGRAY);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside2, COLOR_BRIGHTGRAY);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside3, COLOR_BRIGHTGRAY);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside4, COLOR_BRIGHTGRAY);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside5, COLOR_BRIGHTGRAY);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handairailway, COLOR_ORANGE);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaicafe, COLOR_GREEN);

  nomap_drawn = false;
}

// backscreen の左下にソフトウェアバージョン文字列を、右上にバージョンテキストを描画する。
// 主に startup_demo_tft() のデモ画面で使用する。
void draw_version_backscreen(){
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
  draw_Biwako(center_lat,center_lon,2.5, 0,false);
  draw_pilon_takeshima_line(center_lat, center_lon, 2.5, 0);
  draw_version_backscreen();
  backscreen.pushSprite(0,52);


  // Core1 がロゴ BMP を gmap_sprite に読み込むのを待ち、完了したら pushSprite で表示する。
  // 残り時間は delay で消費して起動タイミングを維持する。
  {
    const int wait_timeout_ms = 1900;
    unsigned long wait_start = millis();
    while (!logo_ready && millis() - wait_start < wait_timeout_ms) {
      delay(10);
    }
    if (logo_ready) {
      gmap_sprite.pushSprite(0, 0);  // ロゴを画面上部 (0,0) に表示
      backscreen.pushSprite(0, 52);  // ロゴの黒エリア(y=52~239)で上書きされた琵琶湖地図を復元
      logo_ready = false;
    }
    unsigned long elapsed = millis() - wait_start;
    if (elapsed < wait_timeout_ms) {
      delay(wait_timeout_ms - elapsed);
    }
  }

  float zoomin_speedfactor = 1.0f;
  int countermax = 40/zoomin_speedfactor;
  float scalenow = 0;
  for (int i = 0; i <= countermax; i++) {
    backscreen.fillScreen(COLOR_WHITE);
    scalenow = 2.5 + i * 0.25*zoomin_speedfactor;
    draw_Biwako(mapf(i,0,countermax,center_lat,PLA_LAT), mapf(i,0,countermax,center_lon,PLA_LON), scalenow, 0, false);
    draw_pilon_takeshima_line(mapf(i,0,countermax,center_lat,PLA_LAT), mapf(i,0,countermax,center_lon,PLA_LON), scalenow, 0);
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }
  delay(10);


  float zoomout_speedfactor = 1.0f;
  countermax = 40/zoomout_speedfactor;

  for (int i = 0; i < countermax; i++) {
    //scalenow *= 0.78;//for zoomout_speedfactor=2. 
    scalenow *= 0.88;// for zoomout_speedfactor=1
    if(scalenow < 0.07)
      scalenow = 0.07;
    else{
      center_lat += 0.005*i;// for zoomout_speedfactor=1
      //center_lat += 0.01*i;// for zoomout_speedfactor=2
    }

    bool islast = i == countermax-1;
    backscreen.fillScreen(COLOR_WHITE);
    if(scalenow < 0.5){
      draw_Japan(mapf(i,0,countermax,PLA_LAT,center_lat), mapf(i,0,countermax,PLA_LON,center_lon),scalenow, 0);
      draw_map(STRK_MAP1, 0, center_lat, center_lon, scalenow, &map_biwako, COLOR_GREEN);
    }
    else{
      draw_Biwako(mapf(i,0,countermax,PLA_LAT,center_lat), mapf(i,0,countermax,PLA_LON,center_lon),scalenow, 0, false);
      draw_pilon_takeshima_line(mapf(i,0,countermax,PLA_LAT,center_lat), mapf(i,0,countermax,PLA_LON,center_lon), scalenow, 0);
    }
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }
  delay(500);
  upward_mode = saved_upward_mode;  // TRACKUP/NORTHUP 設定を復元
}
// デモモード（GPS 固定前に琵琶湖をデモ飛行）中の通知ボックスを backscreen に描画する。
// 「LAKE BIWA DEMO x5 SPD」と設定変更の案内を表示する。
void draw_demo_biwako(){
  if((millis()/3000)%2 == 0){  // 6秒周期（3秒点灯・3秒消灯）で点滅
    backscreen.fillRect(5, 195, SCREEN_WIDTH-5*2, 15+5*2, COLOR_WHITE);
    backscreen.drawRect(5, 195, SCREEN_WIDTH-5*2, 15+5*2, COLOR_ORANGE);
    backscreen.setCursor(25,200);
    backscreen.setTextColor(COLOR_ORANGE);
    backscreen.print("LAKE BIWA DEMO x5 SPD");
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
void clean_backscreen(){
  backscreen.fillScreen(COLOR_WHITE);
}
// VSIスプライト（5×240px）を鉛直速度に応じて描画する。
// 中心 Y=120 が 0 m/s の基準線（白）。上昇=緑バー、下降=シアンバー。
// 不感域なし。最大バー長 120px（±2.0 m/s で振り切り）。
// バーの上にグレー線（バリオ閾値 KF:±0.3 / MS5611単独:±0.6 m/s）と白線（±1.0 m/s）を重ねて描画。
void draw_vsi() {
    // BNO085 が使えるときは Kalman 融合済みの上昇率を使う。
    // BNO085 非接続時は MS5611 単独の上昇率にフォールバックする。
    float vspeed = get_imu_ok() ? get_imu_vspeed() : get_airdata_vspeed();
    vsi_sprite.fillScreen(TFT_BLACK);
    vsi_sprite.drawFastHLine(0, 120, 5, TFT_WHITE);  // 0 m/s 基準線
    if (!get_airdata_ok()) return;
    const float MAX_VSPEED = 2.0f;
    const int   BAR_MAX_PX = 120;
    if (vspeed > 0) {
        // 上昇: 中心から上方向に緑バー（2.0 m/s で振り切り）
        int bar = (int)(min(vspeed, MAX_VSPEED) / MAX_VSPEED * BAR_MAX_PX);
        vsi_sprite.fillRect(0, 120 - bar, 5, bar, TFT_GREEN);
    } else if (vspeed < 0) {
        // 下降: 中心から下方向にシアンバー（2.0 m/s で振り切り）
        int bar = (int)(min(-vspeed, MAX_VSPEED) / MAX_VSPEED * BAR_MAX_PX);
        vsi_sprite.fillRect(0, 121, 5, bar, TFT_CYAN);
    }
    // 閾値ラインをバーの上に重ねて描画（バーがなくても常に表示）
    // グレー線: バリオ音デッドバンド閾値（sound.cpp の dead_band と一致させる）
    //   KF融合中（BNO085+MS5611）: ±0.3 m/s = ±18px
    //   MS5611単独              : ±0.6 m/s = ±36px
    int db_px = (get_imu_ok() && get_airdata_ok()) ? 18 : 36;
    vsi_sprite.drawFastHLine(0, 120 - db_px, 5, TFT_DARKGREY);
    vsi_sprite.drawFastHLine(0, 120 + db_px, 5, TFT_DARKGREY);
    // 白線: ±1.0 m/s = ±60px
    vsi_sprite.drawFastHLine(0, 120 - 60, 5, TFT_WHITE);     // +1.0 m/s
    vsi_sprite.drawFastHLine(0, 120 + 60, 5, TFT_WHITE);     // -1.0 m/s
}

// backscreen スプライトを TFT の (0, 50) に転送する（ヘッダー 50px の下から表示）。
// 転送前に VSI を backscreen の右端 X=235 に合成する。
void push_backscreen(){
  TIMING_START(push_bs);
  if (vario_volume > 0 && !vario_inhibit) {
    draw_vsi();
    vsi_sprite.pushToSprite(&backscreen, 235, 0);  // Sprite→Sprite 合成
  }
  backscreen.pushSprite(0, 50);
  TIMING_END(ts_push_backscreen, push_bs);
}

// backscreen の最上部に小さなテキスト（GS・m/s・MT ラベル）を描画する。
// trackwarning_until が有効な間は「MT」ラベルを赤/黒で点滅させてコース警告を示す。
void draw_gs_track(){
  backscreen.setTextWrap(false);
  if(trackwarning_until > millis()){
    backscreen.setTextColor((millis()/1000)%2==0?COLOR_RED:COLOR_BLACK, TFT_WHITE);
    backscreen.drawString("MT", SCREEN_WIDTH - 40, 1);
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
  }else{
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
    backscreen.drawString("MT", SCREEN_WIDTH - 40, 1);
  }
  backscreen.drawString("GS", 1, 1);
  backscreen.drawString("m/s", 100, 1);
}


// 地図 BMP 画像が表示できないときのステータスメッセージを backscreen に描画する。
// - scale > SCALE_EXLARGE_GMAP: このスケールでは地図画像なし（灰色文字で静的表示）。
// - タスクキューに TASK_LOAD_MAPIMAGE がある: "Loading image..." を灰色で表示。
// - GPS 固定済みで画像なし: "No map image available." をオレンジで表示（ファイルが見つからない警告）。
void draw_nogmap(double scale) {
  backscreen.setCursor(5, BACKSCREEN_SIZE - 36);  // hPa 表示(BACKSCREEN_SIZE-18)との重複を避けて上にずらす
  if(scale > SCALE_EXLARGE_GMAP){
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("No map image at this scale.");
  }else{
    if(!isTaskInQueue(TASK_LOAD_MAPIMAGE) && !isTaskRunning(TASK_LOAD_MAPIMAGE)){
      if(get_gps_fix() && !new_gmap_ready){//描画中に新しいimageがload完了している場合がある。
        backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
        backscreen.print("No map image available.");
      }
    }else{
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("Loading image...");
    }
  }
}

// gmap_sprite に読み込まれている地図画像を backscreen に転送する。
// TRACKUP モード: pushRotated() で -drawupward_direction 度回転させて転送（進行方向が上になる）。
// NORTHUP モード: pushToSprite() でそのまま (0,0) に転送。
// gmap_loaded_active=false の場合は何もしないで false を返す。
bool draw_gmap(float drawupward_direction){
  if (gmap_loaded_active) {
    if(is_trackupmode()) {
      // gmap_sprite のpivot（デフォルト=スプライト中心=自機GPS位置）が
      // backscreen の pivot 位置に着地し、その点を軸に回転する。
      // → gmap 中心（自機位置）が backscreen 上の自機描画位置 (cx, cy) に正確に一致する。
      backscreen.setPivot(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE * 3 / 4);  // (120, 180)
      gmap_sprite.pushRotated(&backscreen, -drawupward_direction);
      backscreen.setPivot(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2);       // 後続処理のためリセット
    } else
      gmap_sprite.pushToSprite(&backscreen,0,0);
    DEBUG_PLN(20240828, "pushed gmap");
    return true;
  }
  return false;
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


// 画面上部 50px のヘッダーを描画して TFT の (0, 0) に転送する。
// 左: 対地速度 m/s（GPS 速度 < 2m/s のときは灰色）。
// 右半分: 磁方位 3 桁。コース警告中は赤/白反転で点滅する。
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
  header_footer.printf("%4.1f", get_gps_mps());


  const int mtvx = SCREEN_WIDTH - 111;
  header_footer.drawFastVLine(mtvx, 0, HEADERFOOTER_HEIGHT, COLOR_GRAY);
  header_footer.setCursor(mtvx, 3);
  if(trackwarning_until > millis()){
    if((millis()/1000)%2==0){
      header_footer.setTextColor(COLOR_RED, TFT_WHITE);
    }else{
      header_footer.fillRect(mtvx,0,111,3,COLOR_RED);
      header_footer.setTextColor(COLOR_WHITE, TFT_RED,true);
    }
    header_footer.printf("%03d", (int)get_gps_magtrack());
    header_footer.setTextColor(sel_col, TFT_WHITE);
  }else{
    header_footer.setTextColor(sel_col, TFT_WHITE);
    header_footer.printf("%03d", (int)get_gps_magtrack());
  }

  header_footer.drawFastHLine(0,49,240,COLOR_BLACK);

  if(get_gps_numsat() == 0){
    for(int i = 0; i < 4; i++)
      header_footer.drawFastHLine(5, 22+i, SCREEN_WIDTH-10, COLOR_RED);
  }

  header_footer.pushSprite(0,0);
  TIMING_END(ts_draw_header, hdr);
}

extern int course_warning_index;


// backscreen の最下部に地図サポート情報を描画する（地図モード専用フッター）。
// 左: GPS 時刻（UTC+9 = JST 変換済み）。
// 中央〜右: 緯度（〜N/S）・経度（〜E/W）を 5 桁精度で表示。
void draw_map_footer(){
  //JST Time
  TinyGPSTime time = get_gpstime();
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_BLACK);

  // Magnetic Heading 表示（Max G/S の 1 行上）
  if (get_imu_ok()) {
    float roll, pitch, yaw;
    get_imu_euler(roll, pitch, yaw);
    backscreen.setCursor(1, BACKSCREEN_SIZE-27);
    backscreen.printf("MH%03d", (int)yaw);
  }

  // 最大 G/S 表示（JST 時刻の 1 行上）
  // 5分保持と全時間が同値の場合は片側のみ表示。異なる場合は両方表示。
  backscreen.setCursor(1, BACKSCREEN_SIZE-18);
  if (get_maxgs_5min() == get_maxgs()) {
    // 同じ値なので全時間最大のみ表示
    backscreen.printf("Max GS %.1fm/s(%02d:%02d)",
      get_maxgs(), get_maxgs_hour(), get_maxgs_min());
  } else {
    backscreen.printf("Max GS %.1fm/s(%02d:%02d) %.1fm/s(%02d:%02d)",
      get_maxgs_5min(), get_maxgs_5min_hour(), get_maxgs_5min_min(),
      get_maxgs(),      get_maxgs_hour(),      get_maxgs_min());
  }

  if (time.isValid()) {
    backscreen.setCursor(1, BACKSCREEN_SIZE-9);
    backscreen.printf("%02d:%02d:%02d JST", (time.hour()+9)%24, time.minute(), time.second());
  }

  // 5 decimal places latitude, longitude print.
  backscreen.setCursor(115,BACKSCREEN_SIZE-9);
  backscreen.printf("%.5f", get_gps_lat());
  if(get_gps_lat() >= 0)
    backscreen.print("N");
  else
    backscreen.print("S");

  backscreen.setCursor(170,BACKSCREEN_SIZE-9);
  backscreen.printf("%.5f", get_gps_lon());
  if(get_gps_lon() >= 0)
    backscreen.print("E");
  else
    backscreen.print("W");
  backscreen.loadFont(AA_FONT_SMALL);
}

// 画面下部 30px のフッターを描画して TFT の (0, 290) に転送する（HEIGHT: 30px）。
// 左: MC（磁方位コース）と目的地までの距離 km。その下にナビモード名と目的地名。
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
    header_footer.printf("MC%3d", magc);


    header_footer.setCursor(60, 1);
    header_footer.setTextColor(COLOR_BLACK);
    if(!get_gps_fix() && !get_demo_biwako())
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
    } else if (input_voltage < 3.8) {  // 50%未満 (4.2-3.4=0.8V の半分は0.4Vなので4.2-0.4=3.8Vが50%の目安)
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
void draw_map(stroke_group strokeid, float mapUpDirection, double center_lat, double center_lon, float mapScale, const mapdata* mp, uint16_t color) {
  int mapsize = mp->size;
  cord_tft points[mapsize];

  for (int i = 0; i < mapsize; i++) {
    float lat1 = mp->cords[i][1];  // Example latitude
    float lon1 = mp->cords[i][0];  // Example longitude
    points[i] = latLonToXY(lat1, lon1, center_lat, center_lon, mapScale, mapUpDirection);
  }

  for (int i = 0; i < mapsize - 1; i++) {
    if (!points[i].isOutsideTft() || !points[i + 1].isOutsideTft()) {
      backscreen.drawLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, color);
    }
  }
}




// 琵琶湖 HPA 競技コース関連のシンボルを描画する:
//   - PLA（スタート）から N パイロン・W パイロン・竹島 への緑の基準線
//   - PLA を中心とした 10.975km 円（公式ルール 2025: 第 1 レグ往路距離）
//   - PLA を中心とした 1.0km 円（公式ルール 2025: 折り返し後の距離）
//   - N パイロン・W パイロン・1km 地点: オレンジ塗りつぶし三角形（パイロンマーク）
//   - PLA: 青地に黄色の小三角形 + 赤線（スタート台の簡易表示）
void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(PILON_NORTH_LAT, PILON_NORTH_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft w_pilon = latLonToXY(PILON_WEST_LAT, PILON_WEST_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft pilon_1km = latLonToXY(PILON_1KM_LAT, PILON_1KM_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft takeshima = latLonToXY(TAKESHIMA_LAT, TAKESHIMA_LON, mapcenter_lat, mapcenter_lon, scale, upward);

  
  backscreen.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, COLOR_GREEN);
  backscreen.drawLine(pla.x, pla.y, takeshima.x, takeshima.y, COLOR_GREEN);
  backscreen.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, COLOR_GREEN);
  // 公式ルール2025 10.975km for first leg outbound.
  backscreen.drawCircle(pla.x, pla.y, scale*10.975f/cos(radians(35)),COLOR_GREEN);
  // 公式ルール2025 1.0km リターンフライト。
  backscreen.drawCircle(pla.x, pla.y, scale*1.0f/cos(radians(35)),COLOR_GREEN);

  if(!n_pilon.isOutsideTft()){
    backscreen.fillTriangle(n_pilon.x-3,n_pilon.y,n_pilon.x+3,n_pilon.y,n_pilon.x,n_pilon.y-9, COLOR_ORANGE);
    backscreen.drawTriangle(n_pilon.x-3,n_pilon.y,n_pilon.x+3,n_pilon.y,n_pilon.x,n_pilon.y-9, COLOR_BLACK);
  }
  if(!w_pilon.isOutsideTft()){
    backscreen.fillTriangle(w_pilon.x-3,w_pilon.y,w_pilon.x+3,w_pilon.y,w_pilon.x,w_pilon.y-9, COLOR_ORANGE);
    backscreen.drawTriangle(w_pilon.x-3,w_pilon.y,w_pilon.x+3,w_pilon.y,w_pilon.x,w_pilon.y-9, COLOR_BLACK);
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


// navdata.cpp の filldata[][] ビットマップを使って琵琶湖エリアの水面・陸地を色分けする。
// filldata は 0.02° グリッドの 2D 配列で、true=水面、false=陸地。
// 各グリッド点に水色（水面）またはオレンジ（陸地）の + 記号を描画する。
// lenbar はスケールに応じた + の長さ（拡大率が高いほど長くして隙間を埋める）。
//
// scale > 36 のとき（高ズーム）は近傍グリッドをサブ補完して塗りつぶし密度を上げる。
// ただし陸海が混在しているグリッドセルは補完しない（誤塗りを防ぐため）。
void fill_sea_land(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {

  int lenbar = 5;
  if (scale > 1.8) lenbar = 7;
  if (scale > 7.2) lenbar = 15;
  if (scale > 36) lenbar = 20;
  if (scale > 90) lenbar = 24;

  //fill
  for (int lat_i = 0; lat_i < ROW_FILLDATA; lat_i++) {
    float latitude = 35.0 + lat_i * 0.02;
    for (int lon_i = 0; lon_i < COL_FILLDATA; lon_i++) {
      float longitude = 135.8 + lon_i * 0.02;
      bool is_sea = filldata[lat_i][lon_i];
      int indexfillp = lat_i * COL_FILLDATA + lon_i;
      cord_tft pos = latLonToXY(latitude, longitude, mapcenter_lat, mapcenter_lon, scale, upward);
      if (pos.isOutsideTft()) {
        continue;
      } else {
        int col = is_sea ? COLOR_LIGHT_BLUE : COLOR_ORANGE;
        backscreen.drawFastHLine(pos.x - lenbar, pos.y, 1 + lenbar * 2, col);

        if (scale > 9) {
          backscreen.drawFastVLine(pos.x, pos.y - lenbar, 1 + lenbar * 2, col);
        }
      }
    }
  }

  if (scale > 36) {
    // + マークのNavデータが登録されていない場所の+をfillする。 要するに拡大すると真っ暗にならないように対策。
    //Big zoom
    int latitude_index = int((mapcenter_lat - 35.0) / 0.02);
    int longitude_index = int((mapcenter_lon - 135.8) / 0.02);

    for (int x = latitude_index - 3; x < latitude_index + 3; x++) {
      for (int y = longitude_index - 3; y < longitude_index + 3; y++) {
        if (x >= 0 && x < ROW_FILLDATA - 1 && y >= 0 && y < COL_FILLDATA - 1) {  // ループ変数 x/y で境界チェック（latitude_index/longitude_index では不十分）
          bool origin_sealand = filldata[x][y];
          if (origin_sealand != filldata[x + 1][y] || origin_sealand != filldata[x][y + 1] || origin_sealand != filldata[x + 1][y + 1]) {
            //陸海の曖昧なエリアは + マークを追加しない。
            continue;
          }
          int dx_size = 2;
          int dy_size = 2;
          if (scale > 90) {
            dx_size = 8;
            dy_size = 8;
          }
          int col = origin_sealand ? COLOR_LIGHT_BLUE : COLOR_ORANGE;
          for (int dx = 0; dx < dx_size; dx++) {
            for (int dy = 0; dy < dy_size; dy++) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              double lat_dx = 35.0 + x * 0.02 + 0.02 * dx / dx_size;
              double lat_dy = 135.8 + y * 0.02 + 0.02 * dy / dy_size;
              cord_tft dpos = latLonToXY(lat_dx, lat_dy, mapcenter_lat, mapcenter_lon, scale, upward);
              if (!dpos.isOutsideTft()) {
                backscreen.drawFastVLine(dpos.x, dpos.y - lenbar, 1 + lenbar * 2, col);
                backscreen.drawFastHLine(dpos.x - lenbar, dpos.y, 1 + lenbar * 2, col);
              }
            }
          }
        }
      }
    }
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
//   4. nomap_drawn=false → 地図は描画済みなので何も表示しない。
void draw_nomapdata() {

  if (get_gps_connection()) {
    //"GPS Module connected."
  } else {
    if (!getReplayMode() && !get_demo_biwako()) {  // リプレイ中・デモ中は NO GNSS 警告を表示しない
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
  else if (get_gps_numsat() == 0 && !get_demo_biwako()) {
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
  } else if (nomap_drawn) {
    //"NO MAPDATA.GPS Fixed."
  }
}

unsigned long lastdrawn_sddetail = 0;
extern char sdfiles[20][32];
extern int sdfiles_size[20]; 
extern int max_page;     // Global variable to store maximum page number
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




// ============================================================
// Vario 詳細画面を描画する（screen_mode == MODE_VARIODETAIL 時）。
// page % 3 == 0: センサー接続状況 + MS5611 気圧データ。
// page % 3 == 1: BNO085 IMU データ（垂直加速度・Kalman 推定値）。
// page % 3 == 2: Kalman フィルター設定パラメーター。
// ヘッダーは GPS DETAIL 同様に 10px 上にずらして pushSprite(0,-10) で転送。
// VSI バーは GPS_TFT_map.ino のループ内で airdata_updated タイミングに更新される。
void draw_variodetail(int page) {
  // ---- ヘッダー: GPS DETAIL と同様に 10px 上ずらし ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);

  const char* page_titles[] = {
    "VARIO DETAIL 1:SENSORS",
    "VARIO DETAIL 2:IMU",
    "VARIO DETAIL 3:KALMAN"
  };
  header_footer.setCursor(1, 11);  // +10px（GPS DETAIL と同じオフセット）
  header_footer.print(page_titles[page % 3]);
  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に合わせる

  // ---- バックスクリーン ----
  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

  int y = 2;

  if (page % 3 == 0) {
    // ---- Page 1: センサー接続状況 + MS5611 気圧データ ----

    // センサー接続状況
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("--- Sensor Status ---");
    y += 16;

    // MS5611
    backscreen.setCursor(2, y);
    backscreen.setTextColor(get_airdata_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
    backscreen.printf("MS5611 : %s", get_airdata_ok() ? "OK" : "NOT CONNECTED");
    y += 14;

    // BNO085
    backscreen.setCursor(2, y);
    backscreen.setTextColor(get_imu_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
    backscreen.printf("BNO085 : %s", get_imu_ok() ? "OK" : "NOT CONNECTED");
    y += 14;

    // 動作モード
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    if      (get_imu_ok() && get_airdata_ok()) backscreen.print("Mode: Kalman Fusion");
    else if (get_airdata_ok())                  backscreen.print("Mode: MS5611 only");
    else if (get_imu_ok())                      backscreen.print("Mode: BNO085 only");
    else                                         backscreen.print("Mode: No sensors");
    y += 20;

    // MS5611 データ
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("--- MS5611 Data ---");
    y += 16;

    if (get_airdata_ok()) {
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

      backscreen.setCursor(2, y);
      backscreen.printf("Pressure: %.2f hPa", get_airdata_pressure());
      y += 14;

      backscreen.setCursor(2, y);
      backscreen.printf("Altitude: %.1f m", get_airdata_altitude());
      y += 14;

      backscreen.setCursor(2, y);
      backscreen.printf("Temp    : %.1f C", get_airdata_temperature());
      y += 14;

      float baro_vsi = get_airdata_vspeed();
      backscreen.setCursor(2, y);
      backscreen.setTextColor((baro_vsi > 0.1f) ? COLOR_GREEN : (baro_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("VSI(baro): %+.2f m/s", baro_vsi);
      y += 14;

      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("n=%d  %.1f Hz", get_airdata_win_samples(), get_airdata_win_hz());
      y += 20;

      // VSI Comparison（BNO085 も接続済みのとき）
      if (get_imu_ok()) {
        float kf_vsi  = get_imu_vspeed();
        float baro_vsi = get_airdata_vspeed();
        backscreen.setCursor(2, y);
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.print("--- VSI Comparison ---");
        y += 16;
        backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
        backscreen.setCursor(2, y);
        backscreen.printf("Baro VSI : %+.2f m/s", baro_vsi);
        y += 14;
        backscreen.setCursor(2, y);
        backscreen.printf("KF   VSI : %+.2f m/s", kf_vsi);
        y += 14;
        backscreen.setCursor(2, y);
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.printf("Delta    : %+.2f m/s", kf_vsi - baro_vsi);
      }
    } else {
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.print("(no data)");
    }

  } else if (page % 3 == 1) {
    // ---- Page 2: BNO085 IMU データ ----

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("--- BNO085 IMU Data ---");
    y += 16;

    if (get_imu_ok()) {
      // イベント受信レート
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("GRV:%.1fHz LACC:%.1fHz RV:%.1fHz",
                        get_imu_grv_hz(), get_imu_lacc_hz(), get_imu_rv_hz());
      y += 16;
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

      float az = get_imu_az();
      backscreen.setCursor(2, y);
      backscreen.setTextColor((az > 0.2f) ? COLOR_GREEN : (az < -0.2f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("Vert Accel : %+.3f m/s2", az);
      y += 14;

      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("KF Altitude: %.1f m", get_imu_altitude());
      y += 14;

      float kf_vsi = get_imu_vspeed();
      backscreen.setCursor(2, y);
      backscreen.setTextColor((kf_vsi > 0.1f) ? COLOR_GREEN : (kf_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("KF VSI     : %+.2f m/s", kf_vsi);
      y += 20;

      // ---- 線形加速度（ボディフレーム）----
      float lax, lay, laz;
      get_imu_linaccel(lax, lay, laz);
      y += 8;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("--- Linear Accel [body] ---");
      y += 15;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("X: %+.3f   Y: %+.3f", lax, lay);
      y += 14;
      backscreen.setCursor(2, y);
      backscreen.printf("Z: %+.3f m/s2", laz);
      y += 18;

      // ---- Euler 角（Roll/Pitch: GAME RV、Yaw: RV+Mag）----
      float roll, pitch, yaw;
      get_imu_euler(roll, pitch, yaw);
      float mag_acc = get_imu_mag_accuracy_deg();
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("--- Euler Angles ---");
      y += 15;
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.setCursor(2, y);
      backscreen.printf("Roll : %+.1f deg  (GAME RV)", roll);
      y += 14;
      backscreen.setCursor(2, y);
      backscreen.printf("Pitch: %+.1f deg  (GAME RV)", pitch);
      y += 14;
      backscreen.setCursor(2, y);
      if (mag_acc >= 0.0f) {
        // ROTATION_VECTOR 受信済み: 磁北基準のヨーと精度を表示
        backscreen.printf("Yaw  : %+.1f deg  (mag)", yaw);
        y += 14;
        backscreen.setCursor(2, y);
        backscreen.setTextColor(mag_acc < 5.0f ? COLOR_GREEN : mag_acc < 15.0f ? COLOR_ORANGE : COLOR_RED, COLOR_WHITE);
        backscreen.printf("  Heading acc: +/-%.1f deg", mag_acc);
      } else {
        // 未受信: GAME RV のヨーをグレーでフォールバック表示
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.printf("Yaw  : %+.1f deg  (game, no mag)", yaw);
      }

    } else {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.setCursor(2, y);
      backscreen.print("BNO085 not connected.");
      y += 14;

      if (get_airdata_ok()) {
        backscreen.setCursor(2, y);
        backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
        backscreen.print("Using baro VSI only.");
        y += 14;

        float baro_vsi = get_airdata_vspeed();
        backscreen.setCursor(2, y);
        backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
        backscreen.printf("Baro VSI : %+.2f m/s", baro_vsi);
      }
    }

  } else {
    // ---- Page 3: Kalman フィルター設定 ----

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("--- Kalman Filter Config ---");
    y += 16;

    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

    backscreen.setCursor(2, y);
    backscreen.print("Process Noise (Q):");
    y += 14;

    backscreen.setCursor(2, y);
    backscreen.printf("  Q_vel  : %.4f", KF_Q_VEL);
    y += 13;

    backscreen.setCursor(2, y);
    backscreen.printf("  Q_bias : %.4f", KF_Q_BIAS);
    y += 20;

    backscreen.setCursor(2, y);
    backscreen.print("Observation Noise (R):");
    y += 14;

    backscreen.setCursor(2, y);
    backscreen.printf("  R      : %.4f m2", KF_R);
    y += 20;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("State: x = [z, z_dot, b]");
    y += 14;

    backscreen.setCursor(2, y);
    if (get_imu_ok() && get_airdata_ok()) {
      backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
      backscreen.print("KF: Active (fused)");
    } else if (get_imu_ok()) {
      backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      backscreen.print("KF: BNO085 only (baro absent)");
    } else {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.print("KF: Inactive");
    }
  }

  backscreen.pushSprite(0, 40);

  // ---- フッター: ページ番号 ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0, 0, SCREEN_WIDTH, COLOR_BLACK);
  header_footer.setTextColor(COLOR_GRAY, COLOR_WHITE);
  header_footer.setCursor(2, 5);
  header_footer.printf("Page %d/3  (short: next / long: back)", page % 3 + 1);
  header_footer.pushSprite(0, SCREEN_HEIGHT - 40);
}


//==================MODE DRAWS===============

// GPS 詳細画面を描画する（screen_mode == MODE_GPSDETAIL 時）。
// page % 3 == 0: スカイプロット画面（全衛星の方位・仰角を円形に描画）。
//   - 衛星種別（GPS=シアン, GLONASS=緑, GALILEO=青, QZSS=赤, BeiDou=オレンジ）で色分け。
//   - SNR=0 の衛星は小さい丸（追跡中だが信号弱い）。
//   - フッターに UTC 時刻と衛星数・フィックス状態を表示。
// page % 3 == 1: 生 NMEA 文字列の最新 MAX_LAST_NMEA 件を表示。
//   1 秒以内に受信した文字列は黒、古いものは灰色で表示（鮮度の視覚化）。
// page % 3 == 2: GSA 情報画面（フィックス種別・DOP・緯度経度・使用衛星 PRN リスト）。
void draw_gpsdetail(int page) {

  header_footer.fillScreen(COLOR_WHITE);

  if (page % 3 == 1) {
    tft.fillRect(0, 20, 240, 320-20, COLOR_WHITE);
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    header_footer.setTextSize(2);
    header_footer.setCursor(1, 1);
    header_footer.println("GPS DETAIL 2: raw NMEAs");
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
    TinyGPSDate date = get_gpsdate();
    TinyGPSTime time = get_gpstime();
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
    TinyGPSDate date = get_gpsdate();
    TinyGPSTime time = get_gpstime();
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
  }
}


// フラッシュ（ROM）に書き込まれた地図ポリゴンと SD から読み込んだ追加マップの一覧を表示する。
// "MAPLIST FLSH:N/SD:M (page/total)" 形式でヘッダーに表示。
// フラッシュマップはページ 0 に全表示、SD マップは 30 エントリ / ページでページング表示。
void draw_maplist_mode(int maplist_page) {

  mapdata* mapdatas[] = { &map_shinura, &map_okishima, &map_takeshima, &map_chikubushima, &map_biwako, &map_handaioutside, &map_handaihighway, &map_handaihighway2, &map_handaiinside1, &map_handaiinside2, &map_handaiinside3,
                          &map_handaiinside4, &map_handaiinside5, &map_handairailway, &map_handaicafe, &map_japan1, &map_japan2, &map_japan3, &map_japan4 };
  int sizeof_mapflash = sizeof(mapdatas) / sizeof(mapdatas[0]);

  int pagetotal = 1 + (sizeof_mapflash + mapdata_count) / 30;
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
    for (int i = 0; i < sizeof_mapflash; i++) {
      backscreen.printf("FLSH %d: %s,%4.2f,%4.2f,%d", mapdatas[i]->id, mapdatas[i]->name, mapdatas[i]->cords[0][0], mapdatas[i]->cords[0][1], mapdatas[i]->size);
      posy += 10;
      backscreen.setCursor(1, posy);
    }
  }

  int sd_start_index = 0;
  if (pagenow > 0) {
    sd_start_index = 30 * pagenow - sizeof_mapflash;
  }

  for (int i = sd_start_index; i < mapdata_count; i++) {
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




// 設定画面を描画する（screen_mode == MODE_SETTING 時）。
// selectedLine: 現在値が変更中の行（ダイヤルを押している間）→ 赤色表示。
// cursorLine  : カーソル位置の行 → マゼンタ色表示。その他は黒。
// menu_settings[].getLabel() で現在の設定値を含むラベル文字列を取得して表示する。
// iconColor() が返す色の丸アイコンを各行の右端に描画する（NULL の場合は描画しない）。
// フッターにはバッテリー残量推定・CPU 温度・空きヒープ（デバッグビルド時のみ）を表示する。
void draw_setting_mode(int selectedLine, int cursorLine) {
  const int separation = 20;
  tft.loadFont(AA_FONT_SMALL);
  textmanager.drawText(SETTING_TITLE, 2, 5, 5, COLOR_BLUE, "SETTINGS");
  tft.setTextColor(COLOR_BLACK);
  backscreen.fillScreen(COLOR_WHITE);

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

  header_footer.setCursor(2, 18);
  {
    float cpu_temp = analogReadTemp();
    header_footer.setTextColor(cpu_temp >= 55.0f ? COLOR_RED : COLOR_GRAY);
    header_footer.printf("CPU %.1fC", cpu_temp);
    if (get_airdata_ok()) {
      float sensor_temp = get_airdata_temperature();
      header_footer.setTextColor(sensor_temp >= 45.0f ? COLOR_RED : COLOR_GRAY);
      header_footer.printf("  Sensor %.1fC", sensor_temp);
    }
  }

  #ifndef RELEASE
  header_footer.setCursor(145, 18);
  header_footer.printf("FreeHeap %d%% ",rp2040.getFreeHeap()*100/rp2040.getTotalHeap());
  #endif

  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.pushSprite(0,SCREEN_HEIGHT-40);
}