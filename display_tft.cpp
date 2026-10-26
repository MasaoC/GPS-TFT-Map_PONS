#include "settings.h"
#include "display_tft.h"
#include "gps.h"
#include "mysd.h"
#include "power.h"
#include "readiness.h"
#include "map_draw.h"
#include "src/vectormap.h"
#include "font_data.h"
#include <cstring>
#include <string>
#include <cstdlib>
#include "hardware/adc.h"

#define AA_FONT_SMALL NotoSansBold15
#define NM_FONT_MEDIUM Arial_Black22
#define NM_FONT_LARGE Arial_Black56


TFT_eSPI tft = TFT_eSPI();                 // Invoke custom library


int screen_brightness = 255;                                // Example brightness value
const int brightnessLevels[] = { 10, 100, 150, 200, 255 };  // Example brightness levels
int brightnessIndex = 4;                                    // Default to 60

#define MODE_TRACKUP 0
#define MODE_NORTHUP 1
#define MODE_SIZE 2
// 既定は TRACK UP（v7 と同じ。SD の settings.txt が読めれば、そこに残した向きに戻る）
int upward_mode = MODE_TRACKUP;


extern int screen_mode;
extern int destination_mode;


void toggle_mode() {
  upward_mode = (upward_mode + 1) % MODE_SIZE;
}

bool is_northupmode() {
  return upward_mode == MODE_NORTHUP;
}
bool is_trackupmode() {
  return upward_mode == MODE_TRACKUP;
}
void set_upward_mode(int mode) {
  if (mode == MODE_TRACKUP || mode == MODE_NORTHUP) upward_mode = mode;
}
int get_upward_mode() { return upward_mode; }


void setup_tft() {
  gpio_init(27);
  gpio_set_dir(27, GPIO_OUT);
  gpio_put(27, 0);  // or gpio_put(28, 1);
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);
  gpio_put(28, 0);  // or gpio_put(28, 1);


  // Set GPIO26 as an ADC input and disable pull-up/pull-down resistors
  adc_gpio_init(BATTERY_PIN);  // Initialize GPIO26 as ADC


  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);

#ifdef BRIGHTNESS_SETTING_AVAIL
  // Initialize backlight control pin
  pinMode(TFT_BL, OUTPUT);
  analogWriteFreq(BL_PWM_FRQ);  // 1000Hz
  analogWrite(TFT_BL, BRIGHTNESS(0));
  analogWrite(TFT_BL, BRIGHTNESS(screen_brightness));
#endif

  tft.begin();


  tft.setRotation(2);           //set 0 for newhaven
  tft.setTextWrap(false);       // 文字列のはみ出しによる次行折り返しを防止
  tft.loadFont(AA_FONT_SMALL);  // Must load the font first


  tft.fillScreen(COLOR_WHITE);
  map_setup(AA_FONT_SMALL);   // 地図のスプライト（map_draw.cpp）
}


class Point {
public:
  int x, y;
  Point(int x = 0, int y = 0)
    : x(x), y(y) {};
};

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
      free(textchar);  // Free previously allocated memory
    }
    size_t length = strlen(text_in);
    textchar = (char*)malloc((length + 1) * sizeof(char));  // Allocate memory for the new string
    if (textchar == NULL) {
      return false;  // Memory allocation failed
    }
    strcpy(textchar, text_in);  // Copy the string
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
      tft.setTextWrap(false);
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
    tft.setTextWrap(false);
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


void clean_display() {
  tft.fillScreen(COLOR_WHITE);
}


// 起動画面。内蔵ベクタ地図を琵琶湖拡大から日本列島全景へ対数ズームアウトして表示する。
// ★ 地図データの帰属表示（OpenStreetMap、ODbL）をここに出す（v7 と同じ。LICENSE.ODbL）。
void startup_demo_tft() {
  tft.fillScreen(COLOR_WHITE);

  // タイトル等の静的テキストは初回に1回だけ描画（毎フレーム再描画による負荷・ちらつきを防止）
  tft.setTextColor(COLOR_RED, COLOR_WHITE);
  tft.setCursor(5, 2);
  tft.println(" Pilot Oriented");
  tft.println("   Navigation System   for HPA");
  tft.println("========== PONS ==========");

  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  tft.setCursor(1, MAP_Y + MAP_H + 20);
  tft.print("SOFT VER:");
  tft.print(BUILDVERSION);
  tft.print("(b");
  tft.print(BUILDDATE);
  tft.print(")");

  // 琵琶湖（zoom9相当: scale 3.0）から日本海沖・日本列島全景（zoom5相当: scale 0.15）への対数ズームアウト
  struct BootFrame {
    float lat;
    float lon;
    float scale;
  };
  static const BootFrame frames[] = {
    { 35.2334f, 136.0911f, 3.00f }, // 0: 琵琶湖拡大（パイロン・スタート台）
    { 35.6000f, 136.2500f, 1.80f }, // 1: 近畿〜中部
    { 36.1500f, 136.4500f, 1.00f }, // 2: 中部・本州
    { 36.8000f, 136.7500f, 0.55f }, // 3: 東日本〜西日本
    { 37.4000f, 137.0500f, 0.32f }, // 4: 日本列島広域
    { 37.8000f, 137.1500f, 0.20f }, // 5: 日本全国
    { 38.0000f, 137.2000f, 0.15f }, // 6: 日本列島全景（画面中央に収まる）
  };
  const int num_frames = sizeof(frames) / sizeof(frames[0]);

  int last_count = -1;
  for (int i = 0; i < num_frames; i++) {
    draw_startup_map(frames[i].lat, frames[i].lon, frames[i].scale);
    backscreen.setTextColor(COLOR_BLACK);
    backscreen.setCursor(4, MAP_H - 18);
    backscreen.print("Map data (c) OpenStreetMap");
    backscreen.pushSprite(0, MAP_Y);

    if (mapdata_count != last_count) {
      last_count = mapdata_count;
      tft.fillRect(1, MAP_Y + MAP_H + 2, 200, 16, COLOR_WHITE);
      tft.setCursor(1, MAP_Y + MAP_H + 2);
      tft.print("SD MAP COUNT:  ");
      tft.print(mapdata_count);
    }
    gps_loop();
  }

  // 最終フレーム（日本列島全景）を約1.2秒キープし、視認できるようにする
  // この間も gps_loop() を回して UART バッファ溢れを防ぎ、SD MAP COUNT の進行を表示する
  const uint32_t hold_start = millis();
  while (millis() - hold_start < 1200) {
    gps_loop();
    if (mapdata_count != last_count) {
      last_count = mapdata_count;
      tft.fillRect(1, MAP_Y + MAP_H + 2, 200, 16, COLOR_WHITE);
      tft.setCursor(1, MAP_Y + MAP_H + 2);
      tft.print("SD MAP COUNT:  ");
      tft.print(mapdata_count);
    }
    delay(20);
  }
}


bool bankwarning_flipflop = true;
void draw_bankwarning() {
  int starty = 46;
  bankwarning_flipflop = !bankwarning_flipflop;
  int bgcolor = COLOR_BLACK;
  int tcolor = COLOR_WHITE;
  if (bankwarning_flipflop) {
    bgcolor = COLOR_WHITE;
    tcolor = COLOR_BLACK;
  }
  tft.fillRect(0, starty, SCREEN_WIDTH, 36, bgcolor);
  int x = SCREEN_WIDTH / 2 - 37;
  tft.setCursor(x, starty + 18);
  tft.setTextColor(COLOR_RED, bgcolor);
  tft.setTextSize(4);
  tft.print("!  BANK  !");
  tft.setCursor(x, starty + 2);
  tft.setTextColor(tcolor, bgcolor);
  tft.print("!  BANK  !");
  tft.setTextSize(1);
}


// 電池電圧 → 表示色（v7 と同じ。BAT_LOW_VOLTAGE 未満は赤、50% 未満はマゼンタ）。
uint16_t battery_color(float v) {
  if (v < BAT_LOW_VOLTAGE)  return COLOR_RED;
  if (v < BAT_HALF_VOLTAGE) return COLOR_MAGENTA;
  return COLOR_GREEN;
}

// 電源の知らせ（power.cpp が判断）。消せない・点滅させない（v7 と同じ見た目）。
//   電源経路の異常 : POWER FAULT !! / SW OFF + UNPLUG USB
//   高温での充電   : TOO HOT TO CHARGE 46C / UNPLUG USB AND COOL DOWN（異常のあいだは出さない）
// 音の無い v3 では画面が唯一の知らせ方なので、どの画面でも出す。
// force = その画面を描き直した直後（上書きされているので必ず描く）。それ以外は 1 秒ごとに描き直す。
unsigned long last_power_notice_draw = 0;
void draw_power_notice(int y, bool force) {
  const bool fault = power_fault_active();
  if (!fault && !power_charge_hot_active()) return;
  if (!force && millis() - last_power_notice_draw < 1000) return;
  last_power_notice_draw = millis();
  tft.fillRect(5, y, SCREEN_WIDTH - 10, 40, COLOR_RED);
  tft.loadFont(AA_FONT_SMALL);
  tft.setTextColor(COLOR_WHITE, COLOR_RED);
  tft.setCursor(12, y + 5);
  if (fault) {
    tft.print("POWER FAULT !!");
    tft.setCursor(12, y + 21);
    tft.print("SW OFF + UNPLUG USB");
  } else {
    tft.printf("TOO HOT TO CHARGE %.0fC", power_cpu_temp());
    tft.setCursor(12, y + 21);
    tft.print("UNPLUG USB AND COOL DOWN");
  }
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
}

// 設定画面の下 2 行。上は電池と CPU 温度（v7 の設定画面フッターと同じ中身）、
// 下は電源の生の値（ADC_VREF_V の確かめ用。USB をつないで VSYS が 4.7V 前後なら正しい）。
unsigned long last_power_status_draw = 0;
void draw_power_status(bool force) {
  if (!force && millis() - last_power_status_draw < 1000) return;
  last_power_status_draw = millis();
  char line1[48];
  uint16_t col1 = COLOR_GRAY;
  switch (power_battery_state()) {
    case BATT_FAULT:    col1 = COLOR_RED; snprintf(line1, sizeof(line1), "POWER FAULT! SW OFF+UNPLUG USB"); break;
    case BATT_OFF:      col1 = COLOR_RED; snprintf(line1, sizeof(line1), "Battery: SW OFF (not charging)"); break;
    case BATT_CHARGING: snprintf(line1, sizeof(line1), "Battery: Charging (USB)"); break;
    default: {
      col1 = battery_color(power_battery_volts());
      const int pct = power_battery_percent();
      const int mins = (int)((float)pct / 100.0f * BAT_FULL_RUNTIME_MIN);
      snprintf(line1, sizeof(line1), "Battery: %d%% (%.2fV) ~%dh%02dm", pct, power_battery_volts(), mins / 60, mins % 60);
      break;
    }
  }
  char line2[64];
  power_format_status(line2, sizeof(line2));
  const float t = power_cpu_temp();
  tft.unloadFont();
  tft.setTextSize(1);
  tft.fillRect(0, SCREEN_HEIGHT - 22, SCREEN_WIDTH, 22, COLOR_WHITE);
  tft.setTextColor(col1, COLOR_WHITE);
  tft.setCursor(2, SCREEN_HEIGHT - 20);
  tft.print(line1);
  tft.setTextColor(power_charge_hot_active() ? COLOR_RED : COLOR_GRAY, COLOR_WHITE);
  tft.setCursor(SCREEN_WIDTH - 54, SCREEN_HEIGHT - 20);
  if (isnan(t)) tft.print("CPU --C");
  else          tft.printf("CPU %2.0fC", t);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.setCursor(2, SCREEN_HEIGHT - 9);
  tft.print(line2);
  tft.loadFont(AA_FONT_SMALL);
}

// 設定画面の右上に本番前チェックの結果（readiness.cpp）。そろえば緑の READY、欠けていれば
// 赤の NOT READY と足りない項目。1 秒ごとに描き直す。
unsigned long last_readiness_draw = 0;
void draw_readiness(bool force) {
  if (!force && millis() - last_readiness_draw < 1000) return;
  last_readiness_draw = millis();
  char missing[40];
  const bool ready = readiness_check(missing, sizeof(missing));
  tft.unloadFont();
  tft.setTextSize(1);
  // 欠けた項目が全部並ぶと 23 文字（"GNSS REPLAY USB BATT SD" = 138px）。x=112 からだと右端で
  // 折り返すので、長いときだけ左へ寄せる（左の "SETTINGS" は x=80 くらいまで）。
  const int mx = min(112, SCREEN_WIDTH - 2 - 6 * (int)strlen(missing));
  tft.fillRect(98, 0, SCREEN_WIDTH - 98, 24, COLOR_WHITE);
  tft.setTextColor(ready ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  tft.setCursor(112, 3);
  tft.print(ready ? "READY" : "NOT READY");
  tft.setCursor(mx, 13);
  tft.print(missing);
  tft.loadFont(AA_FONT_SMALL);
}

// Auto 10km のフェーズが切り替わった知らせ（GPS_TFT_map.ino の apply_auto10k_status() が時刻を入れる）。
// v3 は音が無いので、地図に AUTO10K_NOTICE_MS のあいだ出す。
// 戻り値: 知らせの時間が終わった（地図を描き直して消す必要がある）なら true。
extern unsigned long auto10k_notice_since;
bool draw_auto10k_notice(bool force) {
  static bool shown = false;
  static unsigned long last_draw = 0;
  if (auto10k_notice_since == 0) return false;
  if (millis() - auto10k_notice_since >= AUTO10K_NOTICE_MS) {
    auto10k_notice_since = 0;
    const bool was_shown = shown;
    shown = false;
    return was_shown;
  }
  if (!force && shown && millis() - last_draw < 1000) return false;
  last_draw = millis();
  shown = true;
  const int y = AUTO10K_NOTICE_Y;
  const bool into = auto10k_status == AUTO10K_INTO;
  // INTO は青、AWAY は暗いオレンジ（下端の 10K INTO / 10K AWAY と同じ色。display_tft.h）
  const uint16_t bg = into ? COLOR_NAV_INTO : COLOR_NAV_AWAY;
  tft.fillRect(5, y, SCREEN_WIDTH - 10, 24, bg);
  tft.loadFont(AA_FONT_SMALL);
  tft.setTextColor(COLOR_WHITE, bg);
  tft.setCursor(12, y + 5);
  tft.print(into ? "10K INTO: HEAD BACK" : "10K AWAY: HEAD OUT");
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  return false;
}

void draw_sdinfo() {
  if (good_sd()) {
    tft.fillRect(SCREEN_WIDTH - 22, SCREEN_HEIGHT - 15, 22, 15, COLOR_GREEN);
  } else {
    tft.fillRect(SCREEN_WIDTH - 22, SCREEN_HEIGHT - 15, 22, 15, COLOR_RED);
  }
  tft.setTextColor(COLOR_WHITE, COLOR_GREEN);
  tft.setCursor(SCREEN_WIDTH - 20, SCREEN_HEIGHT - 14);
  tft.print("SD");
}



unsigned long last_loading_image = 0;
void draw_loading_image() {
  if (millis() - last_loading_image > 15) {  //50Hz
    last_loading_image = millis();

    int x = (last_loading_image / 20) % SCREEN_WIDTH;
    int color = COLOR_GRAY;
    if (isTaskRunning(TASK_LOG_SD) || isTaskRunning(TASK_LOG_SDF) || isTaskRunning(TASK_SAVE_CSV)) {
      color = COLOR_RED;
    }

    if (last_loading_image - time_last_gps_frame < 15) {
      color = COLOR_BLUE;
    }
    tft.drawFastVLine(x, SCREEN_HEIGHT / 2 + 240 / 2, 4, color);
    tft.drawFastVLine((x + 1) % SCREEN_WIDTH, SCREEN_HEIGHT / 2 + 240 / 2, 4, COLOR_WHITE);
    tft.drawFastVLine((x + 2) % SCREEN_WIDTH, SCREEN_HEIGHT / 2 + 240 / 2, 4, COLOR_WHITE);
  }
  #ifndef RELEASE
  tft.unloadFont();
  textmanager.drawTextf(COUNTER, 1, 0, SCREEN_HEIGHT-35, COLOR_BLACK, "%d|%d|%d|%d", loop1counter, currentTask.type, loop0pos, loop1pos);
  tft.loadFont(AA_FONT_SMALL);
  #endif
}


void draw_gpsinfo() {
  tft.setTextWrap(false);
  // GPS が止まった・測位を失ったら、残っている速度と方位をグレーにする（今の値と見誤らないため）。
  int col = get_gps_live() ? COLOR_BLACK : COLOR_GRAY;
  const int mtvx = 120;   // 中央区切り線（画面幅 240px の中央。左: 速度、右: 方位）

  // 中央縦線とヘッダー下部境界線
  tft.drawFastVLine(mtvx, 0, 50, COLOR_GRAY);
  tft.drawFastHLine(0, 49, SCREEN_WIDTH, COLOR_BLACK);

  // 速度の単位を縦書き("m","p","s")で中央縦線のすぐ左に置く（v7 と同じ省スペース配置）
  tft.unloadFont();
  tft.loadFont(AA_FONT_SMALL);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  const int uh = tft.fontHeight() - 2;
  static const char* const kUnit[3] = { "m", "p", "s" };
  for (int i = 0; i < 3; i++) {
    const int w = tft.textWidth(kUnit[i]);
    tft.setCursor(mtvx - 2 - w - i * 2, 49 - uh * (3 - i));
    tft.print(kUnit[i]);
  }

  // 大フォント（Arial_Black56 56pt）で速度と方位を表示
  tft.loadFont(NM_FONT_LARGE);
  // 対地速度（2.0m/s 未満はグレー）
  const float mps = get_gps_mps();
  const uint16_t spd_col = mps < 2.0f ? COLOR_GRAY : col;
  textmanager.drawTextf(ND_MPS, 1, 1, 3, spd_col, mps >= 100.0f ? "%4.0f" : "%4.1f", mps);

  // 真方位（3桁）。mtvx+2(122px)から開始し、右端(240px)まで118pxの余裕を確保して折り返しを防止
  textmanager.drawTextf(ND_MT, 1, mtvx + 2, 3, col, "%03d", (int)lround(get_gps_truetrack()) % 360);
  tft.loadFont(AA_FONT_SMALL);

  // 測位が無効なときは速度と方位に取り消し線（赤線4本）を引いて明示（v7 と同じ）
  if (!get_gps_live() || get_gps_numsat() == 0) {
    for (int i = 0; i < 4; i++) {
      tft.drawFastHLine(5, 23 + i, SCREEN_WIDTH - 10, COLOR_RED);
    }
  }

  // 電池（判定は power.cpp。ここは描くだけ）。CHARGE は「USB から充電中」、BATOFF は「USB はあるが SW OFF」。
  switch (power_battery_state()) {
    case BATT_FAULT:
      textmanager.drawText(ND_BATTERY, 1, SCREEN_WIDTH - 67, SCREEN_HEIGHT - 28, COLOR_RED, "PWR NG");
      break;
    case BATT_OFF:
      textmanager.drawText(ND_BATTERY, 1, SCREEN_WIDTH - 67, SCREEN_HEIGHT - 28, COLOR_RED, "BATOFF");
      break;
    case BATT_CHARGING:
      textmanager.drawText(ND_BATTERY, 1, SCREEN_WIDTH - 67, SCREEN_HEIGHT - 28, COLOR_GREEN, "CHARGE");
      break;
    case BATT_LOW: {
      // ★ 電池低下視覚アラート: 赤ベタ塗り反転点滅（500ms周期）。v3 は音が無いので操船中でも絶対に気づくよう強調。
      const bool blink_on = (millis() / 500) % 2;
      tft.fillRect(SCREEN_WIDTH - 68, SCREEN_HEIGHT - 29, 66, 15, blink_on ? COLOR_RED : COLOR_WHITE);
      tft.drawRect(SCREEN_WIDTH - 68, SCREEN_HEIGHT - 29, 66, 15, COLOR_RED);
      tft.setTextColor(blink_on ? COLOR_WHITE : COLOR_RED, blink_on ? COLOR_RED : COLOR_WHITE);
      tft.drawString("BATLOW", SCREEN_WIDTH - 64, SCREEN_HEIGHT - 27);
      break;
    }
    default:
      textmanager.drawTextf(ND_BATTERY, 1, SCREEN_WIDTH - 45, SCREEN_HEIGHT - 28,
                            battery_color(power_battery_volts()), "%d%%", power_battery_percent());
      break;
  }

  // Distance to plathome.
  if(currentdestination != -1 && currentdestination < destinations_count){
    // ★ 測位していない（GPS が止まった・測位を失った）ときは TC と距離を出さない（v7 の 0.965）。
    //   以前は起動直後に緯度経度 (0,0) からの、それらしい 3 桁の方位と 1 万 km 台の距離が出ていた。
    if (get_gps_live()) {
      double destlat = extradestinations[currentdestination].cords[0][0];
      double destlon = extradestinations[currentdestination].cords[0][1];
      double dist = calculateDistance(get_gps_lat(), get_gps_lon(), destlat, destlon);
      // 目的地への True Course（真方位）。反転計算後に四捨五入する（v7 と同じ）。
      double tc_raw = true_course_deg(get_gps_lat(), get_gps_lon(), destlat, destlon);
      if(nav_effective_mode() == DMODE_FLYAWAY){
        tc_raw += 180.0;
      }
      int tc = (int)lround(tc_raw) % 360;
      int posx_km = dist>1000?51:53;
      textmanager.drawTextf(ND_MC_PLAT, 2, 1, SCREEN_HEIGHT - 28, COLOR_MAGENTA, "TC%03d", tc);
      textmanager.drawTextf(ND_DIST_PLAT, 2, posx_km, SCREEN_HEIGHT - 28, COLOR_BLACK, dist>1000?"%.0fkm":dist>100?"%.1fkm":"%.2fkm", dist);
    } else {
      textmanager.drawText(ND_MC_PLAT, 2, 1, SCREEN_HEIGHT - 28, COLOR_GRAY, "TC---");
      textmanager.drawText(ND_DIST_PLAT, 2, 53, SCREEN_HEIGHT - 28, COLOR_GRAY, "---km");
    }

    tft.unloadFont(); 
    // INTO は青、AWAY は暗いオレンジ（display_tft.h の COLOR_NAV_*。v7 と同じ）
    textmanager.drawText(ND_DESTMODE, 1, 120, SCREEN_HEIGHT - 29,
                         nav_effective_mode() == DMODE_FLYAWAY ? COLOR_NAV_AWAY : COLOR_NAV_INTO, nav_mode_text());
    textmanager.drawText(ND_DESTNAME, 1, 120, SCREEN_HEIGHT - 21, COLOR_MAGENTA,extradestinations[currentdestination].name);
    tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  }

  // 時刻（JST）・最大 G/S・水平精度の行（v7 と同じ中身。以前は UTC で、目的地が無いと出なかった）
  tft.unloadFont();
  GpsTime time = get_gpstime();
  if (time.isValid()) {
    textmanager.drawTextf(ND_TIME, 1, 170, SCREEN_HEIGHT - 35, COLOR_BLACK,"%02d:%02d:%02dJST", (time.hour() + 9) % 24, time.minute(), time.second());
  }
  // 最大 G/S: 起動からの最大（その時刻）と直近 5 分の最大
  int mx_hhmm = get_maxgs_all_hhmm();
  if (get_maxgs_all() > 0.0f && mx_hhmm >= 0) {
    textmanager.drawTextf(ND_MAXGS, 1, 1, SCREEN_HEIGHT - 35, COLOR_BLACK, "MAX%4.1f(%02d:%02d)5m%4.1f",
                          get_maxgs_all(), mx_hhmm / 100, mx_hhmm % 100, get_maxgs_recent());
  }
  // 水平精度が悪いときだけ赤で出す（v7 の「Acc: XXm」と同じ基準）
  float hacc = get_gps_hacc_m();
  if (hacc >= GPS_ACC_SHOW_M) {
    textmanager.drawTextf(ND_ACC, 1, 126, SCREEN_HEIGHT - 35, COLOR_RED, "Acc%3.0fm", hacc);
  } else {
    textmanager.drawText(ND_ACC, 1, 126, SCREEN_HEIGHT - 35, COLOR_WHITE, "");
  }
  tft.loadFont(AA_FONT_SMALL);  // Must load the font first

  // 5 decimal places latitude, longitude print.
  col = COLOR_GREEN;
  if (get_gps_numsat() < 5) {
    col = COLOR_RED;
  } else if (get_gps_numsat() < 10) {
    col = COLOR_DARKORANGE;
  }
  textmanager.drawTextf(ND_SATS, 1, 1, SCREEN_HEIGHT - 14, col, "%dsats", get_gps_numsat());
  textmanager.drawTextf(ND_LAT, 1, 60, SCREEN_HEIGHT - 14, COLOR_BLACK, "%.5f", get_gps_lat());
  textmanager.drawTextf(ND_LON, 1, 140, SCREEN_HEIGHT - 14, COLOR_BLACK, "%.5f", get_gps_lon());
}


int mod(int x, int y) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

void tft_change_brightness(int increment) {
#ifdef BRIGHTNESS_SETTING_AVAIL
  brightnessIndex = mod(brightnessIndex + increment, sizeof(brightnessLevels) / sizeof(brightnessLevels[0]));
  screen_brightness = brightnessLevels[brightnessIndex];
  analogWrite(TFT_BL, BRIGHTNESS(screen_brightness));  // For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
#endif
}


int calculateGPS_X(float azimuth, float elevation) {
  const int shift_left = 10;
  const int radius = SCREEN_WIDTH / 2 - shift_left;

  return (SCREEN_WIDTH / 2) + (int)(cos(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) - shift_left;
}

int calculateGPS_Y(float azimuth, float elevation) {
  const int height = SCREEN_HEIGHT;
  const int radius = SCREEN_WIDTH / 2 - 20;
  const int shift_down = 0;
  return (height / 2) - (int)(sin(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) + shift_down;
}


unsigned long lastdrawn_const = 0;
bool draw_gpsdetail(bool redraw, int page) {
  if (millis() - lastdrawn_const > 1000) {
    redraw = true;
  }
  bool redraw_done = redraw;
  if (redraw) {
    lastdrawn_const = millis();
    redraw = false;
    bool aru = false;
    for (int i = 0; i < MAX_SATELLITES; i++) {
      if (satellites[i].PRN != 0) aru = true;  // Skip empty entries
    }

    tft.fillScreen(COLOR_WHITE);

    if (page % 2 == 1) {
      tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
      tft.setTextSize(2);
      tft.setCursor(1, 1);
      tft.println("GPS DETAIL 2: UBX frames");

      tft.unloadFont();
      tft.setTextSize(1);
      int posy = 10;
      tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
      for (int i = 0; i < GPS_FRAMELOG_LINES; i++) {
        posy += 20;
        tft.setCursor(1, posy);
        tft.println(get_gps_framelog(i));
      }
      tft.loadFont(AA_FONT_SMALL);  // Must load the font first
    }
    if (page % 2 == 0) {
      tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
      tft.setTextSize(2);
      tft.setCursor(1, 1);
      tft.println("GPS DETAIL 1:CONSTELLATION");

      tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, SCREEN_WIDTH / 2 - 2, COLOR_BLACK);
      tft.setTextColor(COLOR_CYAN, COLOR_WHITE);
      tft.print("GPS ");
      tft.setTextColor(COLOR_GREEN, COLOR_WHITE);
      tft.print("GLO ");
      tft.setTextColor(COLOR_WHITE, COLOR_BLUE, true);
      tft.print("GAL ");
      tft.setTextColor(COLOR_RED, COLOR_WHITE);
      tft.print("QZS ");
      tft.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      tft.print("BEI ");

      tft.setCursor(23, SCREEN_HEIGHT - 20);
      tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
      GpsDate date = get_gpsdate();
      GpsTime time = get_gpstime();
      if (date.isValid() && time.isValid()) {
        tft.printf("%d.%d.%d %02d:%02d:%02d UTC", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second());
      }
      // NAV-PVT の測位の状態と精度、GPS の設定（gps_setup() の UBX）のうち ACK が返った数、
      // AssistNow Autonomous（予測軌道で測っている衛星数 / 予測データを持つ衛星数）
      tft.unloadFont();
      tft.setTextSize(1);
      tft.setCursor(23, SCREEN_HEIGHT - 44);
      const float hacc = get_gps_hacc_m();
      const float sacc = get_gps_sacc_mps();
      tft.printf("fix%d %s pDOP%.1f ", get_gps_fixtype(), get_gps_fix3d() ? "OK" : "--", get_gps_pdop());
      if (hacc >= 0.0f) tft.printf("hAcc%.1fm sAcc%.2f", hacc, sacc);
      else              tft.print("hAcc -- sAcc --");
      tft.setCursor(23, SCREEN_HEIGHT - 34);
      tft.printf("CFG %d/%d %dHz  ANA %d/%d", gps_cfg_acked, gps_cfg_total, GPS_RATE_HZ,
                 get_gps_ana_orbit_svs(), get_gps_ana_data_svs());
      tft.loadFont(AA_FONT_SMALL);

      if (!aru)
        return true;

      tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
      for (int i = 0; i < MAX_SATELLITES; i++) {
        if (satellites[i].PRN == 0) continue;  // Skip empty entries
        //if(satellites[i].SNR == 0) continue; //Skip unknown signal strength.

        // Calculate position on display based on azimuth and elevation
        float azimuth = satellites[i].azimuth;
        float elevation = satellites[i].elevation;

        int x = calculateGPS_X(azimuth, elevation);
        int y = calculateGPS_Y(azimuth, elevation);
        int size = 5;
        if (satellites[i].SNR <= 0) {
          size = 2;
        }

        if (satellites[i].satelliteType == SATELLITE_TYPE_QZSS)
          tft.fillCircle(x, y, size, COLOR_RED);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GPS)
          tft.fillCircle(x, y, size, COLOR_CYAN);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GLONASS)
          tft.fillCircle(x, y, size, COLOR_GREEN);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GALILEO)
          tft.drawCircle(x, y, size, COLOR_BLUE);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_BEIDOU)
          tft.drawCircle(x, y, size, COLOR_ORANGE);
        else
          tft.fillCircle(x, y, size, COLOR_BLACK);

        int textx = constrain(x + 6, 0, SCREEN_WIDTH - 20);
        if (satellites[i].PRN >= 10)
          textx -= 10;
        if (satellites[i].PRN >= 100)
          textx -= 10;
        tft.setCursor(textx, y - 3);
        tft.print(satellites[i].PRN);
      }
    }
  }
  return redraw_done;
}


// 地図一覧。1 行ずつの項目を通し番号で並べ、ページごとに MAPLIST_LINES 行ずつ出す。
// ★ 以前はページの終わりを決めておらず、1 ページ目から SD の地図を最後まで描いて画面の外へはみ出していた（v7 の 0.923）。
#define MAPLIST_LINES 27
static const mapdata* const flash_maps[] = { &map_handaioutside, &map_handaihighway, &map_handaihighway2, &map_handaiinside1, &map_handaiinside2, &map_handaiinside3,
                                             &map_handaiinside4, &map_handaiinside5, &map_handairailway, &map_handaicafe };
#define FLASH_MAP_COUNT ((int)(sizeof(flash_maps) / sizeof(flash_maps[0])))
#define MAPLIST_HEAD_LINES 8   // ベクタ地図 2 行 + コース 6 行

// i 番目の行の文字と色。範囲外なら false。
static bool maplist_entry(int i, char* buf, size_t n, uint16_t* col) {
  *col = COLOR_BLACK;
  if (i == 0) {
    snprintf(buf, n, "VECTORMAP OSM %s LOD%d %dtiles %lupts", vm_data_source_date, vm_select_lod(map_frame.scale),
             vm_last_tiles, (unsigned long)vm_last_points);
    return true;
  }
  if (i == 1) {
    snprintf(buf, n, "MAP FRAME last %lums max %lums", (unsigned long)map_frame_ms_last, (unsigned long)map_frame_ms_max);
    return true;
  }
  if (i == 2) {
    // コースの座標（SD の override_pilon_coordinate.csv で上書きしたかどうかも）
    *col = course_override_loaded ? COLOR_MAGENTA : COLOR_GRAY;
    snprintf(buf, n, "%s", course_override_loaded ? "COURSE SD OVERRIDE" : "COURSE built-in");
    return true;
  }
  if (i < MAPLIST_HEAD_LINES) {
    static const char* const cname[] = { "PLA ", "NPIL", "SPIL", "1KM ", "TAKE" };
    double (*const cpt[])[2] = { course_pla, course_npilon, course_spilon, course_1km, course_takeshima };
    const int k = i - 3;
    *col = course_override_loaded ? COLOR_MAGENTA : COLOR_GRAY;
    snprintf(buf, n, " %s %.6f, %.6f", cname[k], cpt[k][0][0], cpt[k][0][1]);
    return true;
  }
  i -= MAPLIST_HEAD_LINES;
  if (i < FLASH_MAP_COUNT) {
    const mapdata* m = flash_maps[i];
    snprintf(buf, n, "FLSH %d: %s,%4.2f,%4.2f,%d", m->id, m->name, m->cords[0][1], m->cords[0][0], m->size);
    return true;
  }
  i -= FLASH_MAP_COUNT;
  if (i < mapdata_count) {
    const mapdata* m = &extramaps[i];
    snprintf(buf, n, "SD %d:%s,%4.2f,%4.2f,%d", i, m->name, m->size > 0 ? m->cords[0][1] : 0.0, m->size > 0 ? m->cords[0][0] : 0.0, m->size);
    return true;
  }
  return false;
}

bool draw_maplist_mode(bool redraw, int maplist_page) {
  if (millis() - lastdrawn_const > 10000L) {
    redraw = true;
  }
  bool redraw_done = redraw;
  if (redraw) {
    lastdrawn_const = millis();
    redraw = false;
    const int total = MAPLIST_HEAD_LINES + FLASH_MAP_COUNT + mapdata_count;
    const int pagetotal = (total + MAPLIST_LINES - 1) / MAPLIST_LINES;
    const int pagenow = mod(maplist_page, pagetotal);

    tft.fillScreen(COLOR_WHITE);
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    tft.setCursor(1, 1);
    tft.printf("MAPLIST FLSH:%d/SD:%d (%d/%d)", FLASH_MAP_COUNT, mapdata_count, pagenow + 1, pagetotal);

    tft.unloadFont();
    tft.setTextSize(1);
    char buf[64];
    uint16_t col;
    for (int line = 0; line < MAPLIST_LINES; line++) {
      if (!maplist_entry(pagenow * MAPLIST_LINES + line, buf, sizeof(buf), &col)) break;
      tft.setTextColor(col, COLOR_WHITE);
      tft.setCursor(1, 20 + line * 10);
      tft.print(buf);
    }
    tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  }
  return redraw_done;
}

void reset_degpersecond();
// GPS_TFT_map.ino
void change_scale(int step);
const char* scale_name();
void exit_setting();

Setting settings[] = {
  { SETTING_SETDESTINATION,
    [](bool selected) -> std::string {
      char buff[48];  // temporary buffer
      // 目的地名は SD の destinations.csv から来て長さが決まらないので snprintf で切る
      // （以前は 32 バイトに sprintf していて、14 文字ほどの名前でスタックを壊しうる）。
      if(currentdestination != -1 && currentdestination < destinations_count){
        snprintf(buff, sizeof(buff), selected ? " Set destination: %s(%d)" : "Set destination: %s(%d)", extradestinations[currentdestination].name, currentdestination);
      }else
        snprintf(buff, sizeof(buff), selected ? " Set destination: %d/%d" : "Set destination: %d/%d", currentdestination,destinations_count);
        
      return std::string(buff);  // return as std::string
    },nullptr,
    []() {
      if(destinations_count > 0){
        currentdestination++;
        if(currentdestination >= destinations_count){
          currentdestination = 0;
        }
      }
      // パイロン・竹島を目的地にしたまま Auto 10km にはしない（v7 と同じ。そこから 10km 離れる誘導は意味が無い）
      if(destination_mode == DMODE_AUTO10K && currentdestination >= 0 && currentdestination < destinations_count
         && is10k_not_allowed(extradestinations[currentdestination].name)){
        destination_mode = DMODE_FLYINTO;
      }
    } },
  { SETTING_DESTINATIONMODE,
    [](bool selected) -> std::string {
      char buff[40];  // temporary buffer
      const char* m = destination_mode == DMODE_AUTO10K ? "Auto 10km" : destination_mode == DMODE_FLYAWAY ? "Fly away" : "Fly into";
      snprintf(buff, sizeof(buff), selected ? " Destination Mode: %s" : "Destination Mode: %s", m);
      return std::string(buff);  // return as std::string
    },nullptr,
    []() {
      // Fly into → Fly away → Auto 10km → Fly into。Auto 10km にできない目的地では飛ばす。
      if(destination_mode == DMODE_FLYINTO)
        destination_mode = DMODE_FLYAWAY;
      else if(destination_mode == DMODE_FLYAWAY){
        bool ng = currentdestination < 0 || currentdestination >= destinations_count
                  || is10k_not_allowed(extradestinations[currentdestination].name);
        destination_mode = ng ? DMODE_FLYINTO : DMODE_AUTO10K;
      }
      else
        destination_mode = DMODE_FLYINTO;
    } },
  { SETTING_MAPSCALE,
    // 地図の縮尺。地図画面ではボタンのダブルクリックでも変わる（1 回押しでは変わらない。v7 の 0.921）。
    [](bool selected) -> std::string {
      char buff[32];
      snprintf(buff, sizeof(buff), selected ? " Map scale: %s" : "Map scale: %s", scale_name());
      return std::string(buff);
    },
    nullptr,
    []() {
      change_scale(1);
    } },
  { SETTING_UPWARD,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      snprintf(buff, sizeof(buff), selected ? " Upward: %s" : "Upward: %s", is_trackupmode() ? "TRACK UP" : "NORTH UP");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      toggle_mode();
    } },
#ifdef BRIGHTNESS_SETTING_AVAIL
  { SETTING_BRIGHTNESS,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      snprintf(buff, sizeof(buff), selected ? " Brightness: %03d" : "Brightness: %03d", screen_brightness);
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      tft_change_brightness(1);
    } },
#endif
  { SETTING_DEMOBIWA,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      snprintf(buff, sizeof(buff), selected ? " DEMO BIWA: %s" : "DEMO BIWA: %s", get_demo_biwako() ? "YES" : "NO");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      latlon_manager.reset();
      toggle_demo_biwako();
      reset_degpersecond();
      // デモ中の Auto 10km のフェーズは仮想の位置で動くので、入るときも抜けるときも実際の値に戻す
      auto10k_status = auto10k_status_flight;
      // デモとリプレイは両方とも「仮想の位置」なので同時には使わない（デモを入れたらリプレイの選択を外す）
      if (get_demo_biwako()) replay_select_none();
    } },
  { SETTING_REPLAY,
    // SD の飛行 CSV を本体で再生する（v7 の 0.92）。選ぶのはここ、始めるのは設定画面を抜けたとき。
    [](bool selected) -> std::string {
      char buff[48];
      const char* name = replay_selected_name();
      if (name == nullptr) snprintf(buff, sizeof(buff), selected ? " Replay: OFF" : "Replay: OFF");
      else snprintf(buff, sizeof(buff), selected ? " Replay: %s%s" : "Replay: %s%s", name, replay_selected_is_playing() ? " *" : "");
      return std::string(buff);
    },
    nullptr,
    []() {
      replay_select_next();
    } },
  { SETTING_REPLAYSPEED,
    [](bool selected) -> std::string {
      char buff[32];
      snprintf(buff, sizeof(buff), selected ? " Replay speed: x%d" : "Replay speed: x%d", replay_get_speed());
      return std::string(buff);
    },
    nullptr,
    []() {
      replay_cycle_speed();
    } },
  { SETTING_GPSDETAIL,
    [](bool selected) -> std::string {
      return "Show GPS detail >";
    },
    []() {
      DEBUG_P(20240801, "GPS CONST MODE");
      gps_constellation_mode();
      screen_mode = MODE_GPSDETAIL;
    },
    nullptr },
  { SETTING_MAPDETAIL,
    [](bool selected) -> std::string {
      return "Maplist detail >";
    },
    []() {
      DEBUG_P(20240801, "MAP DETAIL MODE");
      screen_mode = MODE_MAPLIST;
    },
    nullptr },
  { SETTING_EXIT,
    [](bool selected) -> std::string {
      return "Exit setting";
    },
    []() {
      // 設定を SD に残し、リプレイの選択を反映して地図画面へ（GPS_TFT_map.ino。放置で戻るときも同じ）
      exit_setting();
    },
    nullptr }
};


int setting_size = sizeof(settings) / sizeof(settings[0]);

void draw_setting_mode(bool redraw, int selectedLine, int cursorLine) {
  // 10 項目が電源の知らせ（下端から 64px）と最下行の電池の表示に掛からない間隔
  const int separation = 21;
  const int startY = 35;

  if (redraw) {
    redraw = false;
    textmanager.drawText(SETTING_TITLE, 2, 5, 5, COLOR_BLUE, "SETTINGS");
    tft.setTextColor(COLOR_BLACK);


    for (int i = 0; i < setting_size; ++i) {
      uint16_t col = COLOR_BLACK;
      if (cursorLine == i) {
        col = (selectedLine == i) ? COLOR_RED : COLOR_MAGENTA;
      }

      textmanager.drawTextf(settings[i].id, 2, 10, startY + i * separation, col, settings[i].getLabel(selectedLine == i).c_str());
    }
  }
}