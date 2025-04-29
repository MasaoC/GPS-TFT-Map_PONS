#include "settings.h"
#include "display_tft.h"
#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "font_data.h"
#include "sound.h"
#include "hardware/adc.h"
#include "button.h"
#include <cstring>  // for strlen and strcpy
#include <string>
#include <cstdlib>  // for malloc and free

#define AA_FONT_SMALL NotoSansBold15
#define NM_FONT_MEDIUM Arial_Black22
#define NM_FONT_LARGE Arial_Black46


TFT_eSPI tft = TFT_eSPI();                 // Invoke custom library
TFT_eSprite needle = TFT_eSprite(&tft);    // Sprite object for needle
TFT_eSprite backscreen = TFT_eSprite(&tft);
TFT_eSprite header_footer = TFT_eSprite(&tft);


int screen_brightness = 255;                                // Example brightness value
const int brightnessLevels[] = { 10, 100, 150, 200, 255 };  // Example brightness levels
int brightnessIndex = 4;                                    // Default to 60



#define MODE_TRACKUP 0
#define MODE_NORTHUP 1
#define MODE_SIZE 2
int upward_mode = MODE_NORTHUP;




extern int sound_volume;
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



cord_tft latLonToXY(float lat, float lon, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection) {
  // Calculate x distance (longitude) = Approx distance per degree longitude in km with Mercator projection.
  float xDist = (lon - mapCenterLon) * 111.321;  // 1 degree longitude = ~111.321 km
  // Calculate y-coordinates in Mercator projection
  double yA = latitudeToMercatorY(mapCenterLat);
  double yB = latitudeToMercatorY(lat);
  // Calculate the distance on the y-axis
  double yDist = (yB - yA)* 6378.22347118;// 1 radian longitude = 111.321 *180/3.1415 = 6378.22347118 km
  // Apply scale factor
  xDist *= mapScale;
  yDist *= mapScale;
  // Apply rotation for map up direction
  float angleRad = mapUpDirection * DEG_TO_RAD;
  float rotatedX = xDist * cos(angleRad) - yDist * sin(angleRad);
  float rotatedY = xDist * sin(angleRad) + yDist * cos(angleRad);
  // Translate to screen coordinates
  return cord_tft{
    (BACKSCREEN_SIZE / 2) + (int)rotatedX,
    (BACKSCREEN_SIZE / 2) - (int)rotatedY  // Y is inverted on the screen
  };
}


// Function to convert x, y coordinates on the TFT screen to latitude and longitude
Coordinate xyToLatLon(int x, int y, float mapCenterLat, float mapCenterLon, float mapScale, float mapUpDirection,int mapshiftdown) {
    // Translate screen coordinates to map coordinates
    float screenX = x - (SCREEN_WIDTH / 2);
    float screenY = (SCREEN_HEIGHT / 2) - y + mapshiftdown;

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

const int NEEDLE_LEN = 120;
void createNeedle(void) {
  needle.setColorDepth(8);
  needle.createSprite(4, NEEDLE_LEN);  // create the needle Sprite 3x120

  needle.fillSprite(TFT_BLACK);  // Fill with black
  needle.fillRect(0, 10, 4, 12, COLOR_RED);

  // Define needle pivot point
  uint16_t piv_x = needle.width() / 2;   // x pivot of Sprite (middle)
  uint16_t piv_y = needle.height() - 2;  // y pivot of Sprite (10 pixels from bottom)
  needle.setPivot(piv_x, piv_y);         // Set pivot point in this Sprite

  //needle.drawLine(0, 10, 0, 120, COLOR_GRAY);

  // Draw needle centre boss
  //needle.drawPixel( piv_x, piv_y, TFT_WHITE);     // Mark needle pivot point with a white pixel
}



void setup_tft() {

  //TFT_WR
  gpio_init(27);
  gpio_set_dir(27, GPIO_OUT);
  gpio_put(27, 0);
  //TFT_DC
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);
  gpio_put(28, 0);


  adc_gpio_init(BATTERY_PIN);  // Initialize GPIO26 as ADC

  pinMode(24,INPUT);
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

#ifdef VERTICAL_FLIP
  tft.setRotation(0);           //set 0 for newhaven
#else
  tft.setRotation(2);           //set 0 for newhaven
#endif

  tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  //tft.setTextFont(2);


  tft.fillScreen(COLOR_WHITE);
  createNeedle();
  if(!backscreen.created()){
    backscreen.setColorDepth(16);
    backscreen.createSprite(SCREEN_WIDTH, BACKSCREEN_SIZE);
    backscreen.loadFont(AA_FONT_SMALL);
  }
  if(!header_footer.created()){
    header_footer.setColorDepth(16);
    header_footer.createSprite(SCREEN_WIDTH, 40);
    header_footer.loadFont(NM_FONT_LARGE);
  }

  DEBUG_P(20250430, "SETUP C0 free/used heap and free stack/Pointer:");
  DEBUG_P(20250430, rp2040.getFreeHeap());
  DEBUG_P(20250430, "/");
  DEBUG_P(20250430, rp2040.getUsedHeap());
  DEBUG_P(20250430, "/");
  DEBUG_P(20250430, rp2040.getFreeStack());
  DEBUG_P(20250430, "/");
  Serial.println(rp2040.getStackPointer(),HEX);
}




struct line {
  int x1, y1, x2, y2;
};


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

class StrokeManager {

  struct Stroke {
    stroke_group id;
    Point* points;
    int pointCount;
    int maxPoints;
    int thickness;
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
      DEBUG_PLN(20240912, id);
      DEBUG_PLN(20240912, maxPoints);
      DEBUG_PLN(20240912, "ERR max stroke reached");
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

/*
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
  */

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
          drawThickLine(strokes[strkid].points[i].x, strokes[strkid].points[i].y, strokes[strkid].points[i + 1].x, strokes[strkid].points[i + 1].y, strokes[strkid].thickness, col);
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

  void printStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      Serial.print("Stroke ");
      Serial.print(i);
      Serial.print(":type");
      Serial.print(strokes[i].id);
      Serial.print(":maxP");
      Serial.print(strokes[i].maxPoints);
      Serial.print(":count");
      Serial.print(strokes[i].pointCount);
      Serial.print(":thickness");
      Serial.print(strokes[i].thickness);
      Serial.println(":");
      for (int j = 0; j < strokes[i].pointCount; j++) {
        Serial.print("(");
        Serial.print(strokes[i].points[j].x);
        Serial.print(", ");
        Serial.print(strokes[i].points[j].y);
        Serial.print(") ");
      }
      Serial.println();
    }
  }


  void drawAllStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        if (strokes[i].thickness == 1)
          backscreen.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, COLOR_WHITE);
        else
          drawThickLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, strokes[i].thickness, COLOR_WHITE);
      }
    }
  }
};




bool try_draw_km_distance(float scale, float km) {
  double latnow = get_gps_lat();
  if(latnow <-80 || latnow > 80){
    latnow = 35;
  }
  int distance_px =  scale*km/cos(radians(latnow));//scale * km;  //scale is px/km
  if(distance_px > 140)
    return false;

  int ypos = 240-2;
  int xpos = 240-2;

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
  /*

  if (ypos > 30) {
    backscreen.drawCircle(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2, radius, COLOR_PINK);
    backscreen.setCursor(BACKSCREEN_SIZE / 2 + 2, ypos + 1);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    backscreen.setTextSize(1);
    
    if (km > 0.999f) {
      backscreen.print(int(km));
    } else {
      backscreen.print(km, 1);
    }
    
    backscreen.println("km");
    return true;
  } else {
    return false;
  }
  */
}

void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  // Calculate the direction of the line
  float angle = atan2(y1 - y0, x1 - x0);
  float halfThickness = thickness / 2.0;

  // Calculate the perpendicular offset for the thickness
  float offsetX = halfThickness * sin(angle);
  float offsetY = halfThickness * -cos(angle);

  // Draw parallel lines to create the thick effect
  for (float t = -halfThickness; t <= halfThickness; t++) {
    int dx = round(t * sin(angle));
    int dy = round(t * -cos(angle));
    backscreen.drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, color);
  }
}

// draw magnetic north east west south according to truetrack, where truetrack is direction of upward display.
// However direction of y-axis is downward for tft display.
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
  int centery = BACKSCREEN_SIZE / 2;
  //Varietion 8 degrees.
  float radian = deg2rad(truetrack + 8.0);
  float radian45offset = deg2rad(truetrack + 8.0 + 45);
  backscreen.setTextColor(col, COLOR_WHITE);
  backscreen.setTextSize(2);
  

  cord_tft n = { int(centerx + sin(-radian) * dist), int(centery - cos(radian) * dist) };
  cord_tft e = { int(centerx + cos(radian) * dist), int(centery - sin(radian) * dist) };
  cord_tft s = { int(centerx + sin(radian) * dist), int(centery + cos(radian) * dist) };
  cord_tft w = { int(centerx - cos(-radian) * dist), int(centery - sin(-radian) * dist) };
  if (!n.isOutsideTft()) {
    backscreen.setCursor(n.x - 5, n.y - 5);
    backscreen.print("N");
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

bool fresh = false;
float last_scale = 1.0;
float last_up = 0;
bool nomap_drawn = true;







#ifdef TFT_USE_ST7735
#define TRIANGLE_HWIDTH 4
#define TRIANGLE_SIZE 12
#else
#define TRIANGLE_HWIDTH 6
#define TRIANGLE_SIZE 18
#endif





extern float last_tone_tt;
extern unsigned long trackwarning_until;



void draw_course_warning(int steer_angle){ 
  if((millis()/1000)%2 == 0){
    backscreen.fillRect(5, 175, SCREEN_WIDTH-5*2, 25, COLOR_WHITE);
    backscreen.drawRect(5, 175, SCREEN_WIDTH-5*2, 25, COLOR_RED);
    backscreen.drawRect(6, 176, SCREEN_WIDTH-5*2-2, 25-2, COLOR_RED);
    backscreen.setTextColor(COLOR_RED);
    if(steer_angle > 0){  
      backscreen.setCursor(23,180);
      backscreen.print("Turn RIGHT! Turn RIGHT!");
    }
    else{
      backscreen.setCursor(30,180);
      backscreen.print("Turn LEFT! Turn LEFT!");
    }
  }
}

void draw_triangle(int ttrack,int steer_angle) {
  float tt_radians = deg2rad(ttrack);
  if (upward_mode == MODE_NORTHUP) {
    float tone_left = deg2rad(last_tone_tt-15);
    float tone_right = deg2rad(last_tone_tt+15);
    float tone_center = deg2rad(last_tone_tt);
    float x1 = (NEEDLE_LEN-10) * sin(tone_left) + BACKSCREEN_SIZE/2;
    float y1 = (NEEDLE_LEN-10) * -cos(tone_left) + BACKSCREEN_SIZE/2;
    float x2 = (NEEDLE_LEN-10) * sin(tone_right) + BACKSCREEN_SIZE/2;
    float y2 = (NEEDLE_LEN-10) * -cos(tone_right) + BACKSCREEN_SIZE/2;
    float x3 = (NEEDLE_LEN-4) * sin(tone_center) + BACKSCREEN_SIZE/2;
    float y3 = (NEEDLE_LEN-4) * -cos(tone_center) + BACKSCREEN_SIZE/2;
    if(trackwarning_until < millis()){
      backscreen.drawTriangle(x1,y1,x2,y2,x3,y3, COLOR_BLACK);
    }else{
      backscreen.fillTriangle(x1,y1,x2,y2,x3,y3, (millis()/1000)%2==0?COLOR_RED:COLOR_BLACK);
    }
    backscreen.setPivot(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2);
    needle.pushRotated(&backscreen,ttrack);
    //needle.pushRotated(ttrack);

    //Center black triangle (my position)
    int rb_x_new = -TRIANGLE_HWIDTH * cos(tt_radians) - TRIANGLE_SIZE * sin(tt_radians);
    int rb_y_new = -TRIANGLE_HWIDTH * sin(tt_radians) + TRIANGLE_SIZE * cos(tt_radians);
    int lb_x_new = TRIANGLE_HWIDTH * cos(tt_radians) - TRIANGLE_SIZE * sin(tt_radians);
    int lb_y_new = TRIANGLE_HWIDTH * sin(tt_radians) + TRIANGLE_SIZE * cos(tt_radians);
    backscreen.fillTriangle(BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2, BACKSCREEN_SIZE / 2 + rb_x_new, BACKSCREEN_SIZE / 2 + rb_y_new, 240 / 2 + lb_x_new, 240 / 2 + lb_y_new, COLOR_BLACK);


    if((millis()/1000)%2==0){
      //約10度以上の方位違いがある場合に、指示三角形を描画する。
      if(abs(steer_angle) > 15){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-0.1:0.1);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-0.35:0.35);
        float x1 = (NEEDLE_LEN-10) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (NEEDLE_LEN-10) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (NEEDLE_LEN-30) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (NEEDLE_LEN-30) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (NEEDLE_LEN-20) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (NEEDLE_LEN-20) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if(abs(steer_angle) > 55){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-0.7:0.7);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-0.95:0.95);
        float x1 = (NEEDLE_LEN-10) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (NEEDLE_LEN-10) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (NEEDLE_LEN-30) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (NEEDLE_LEN-30) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (NEEDLE_LEN-20) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (NEEDLE_LEN-20) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
      if(abs(steer_angle) > 100){
        double steer_triangle_start_rad = tt_radians + (steer_angle<0?-1.3:1.3);
        double steer_triangle_end_rad = tt_radians + (steer_angle<0?-1.55:1.55);
        float x1 = (NEEDLE_LEN-12) * sin(steer_triangle_start_rad) + 240/2;
        float y1 = (NEEDLE_LEN-12) * -cos(steer_triangle_start_rad) + 240/2;
        float x2 = (NEEDLE_LEN-30) * sin(steer_triangle_start_rad) + 240/2;
        float y2 = (NEEDLE_LEN-30) * -cos(steer_triangle_start_rad) + 240/2;
        float x3 = (NEEDLE_LEN-20) * sin(steer_triangle_end_rad) + 240/2;
        float y3 = (NEEDLE_LEN-20) * -cos(steer_triangle_end_rad) + 240/2;
        backscreen.fillTriangle(x1,y1,x2,y2,x3,y3,COLOR_RED);
      }
    }
  }
  if (upward_mode == MODE_TRACKUP) {
    int shortening = 30;
    backscreen.drawFastVLine(240 / 2, shortening, 240 / 2 - shortening, COLOR_BLACK);
    backscreen.drawFastVLine(240 / 2 + 1, shortening, 240 / 2 - shortening, COLOR_BLACK);
    backscreen.fillTriangle(240 / 2, 240 / 2, 240 / 2 - TRIANGLE_HWIDTH, 240 / 2 + TRIANGLE_SIZE, 240 / 2 + TRIANGLE_HWIDTH, 240 / 2 + TRIANGLE_SIZE, COLOR_BLACK);
  }
}



// point A -> B -(distance)> C
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


// Point A -(distance)> D -> B
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

void draw_flyawayfrom(double dest_lat,double dest_lon, double center_lat, double center_lon, float scale, float up) {
  draw_flyinto(dest_lat,dest_lon,center_lat,center_lon,scale,up,1);

  int thickness = 2;
  double lat3, lon3;
  cord_tft dest = latLonToXY(dest_lat, dest_lon, center_lat, center_lon, scale, up);
  double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
  calculatePointC(dest_lat, dest_lon, center_lat, center_lon, distance, lat3, lon3);
  cord_tft targetpoint = latLonToXY(lat3, lon3, center_lat, center_lon, scale, up);
  drawThickLine(240/2, 240/2, targetpoint.x, targetpoint.y, thickness, COLOR_MAGENTA);
}

void draw_flyinto2(double dest_lat, double dest_lon, double center_lat, double center_lon, float scale, float up,int thickness) {
  cord_tft goal = latLonToXY(dest_lat, dest_lon, center_lat, center_lon, scale, up);
  if (goal.isOutsideTft()) {
    //scaleがとても大きい場合、goal.x,goal.yがオーバーフローする。
    //そのまま描画すると処理落ちするので、適度な座標を計算し直す必要がある。
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    calculatePointD(center_lat, center_lon, dest_lat, dest_lon, distance, newlat, newlon);
    goal = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
    drawThickLine(240 / 2, 240 / 2, goal.x, goal.y, thickness, COLOR_MAGENTA);
  }else{
    drawThickLine(240 / 2, 240 / 2, goal.x, goal.y, thickness, COLOR_MAGENTA);
    double distance = 200 / scale;  //画面外に出ればよいので適当な距離を設定。
    double newlat, newlon;
    calculatePointC(center_lat, center_lon, dest_lat, dest_lon, distance, newlat, newlon);
    goal = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
    backscreen.drawLine(240 / 2, 240 / 2,goal.x, goal.y, COLOR_MAGENTA);
  }
}

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
  drawThickLine(240 / 2, 240 / 2, goal.x, goal.y, thickness, COLOR_MAGENTA);
}


// Im making this variable global so that it will be stored in RAM instead of Stack.
// Core0 is running out of stack with 4KB with MAX_TRACK_CORDS>=300, when making this local variable(Stack) we need to do bool core1_separate_stack = true;
// However, we have enough RAM so, lets make points variable global for now.
cord_tft points[MAX_TRACK_CORDS];

void draw_track(double center_lat, double center_lon, float scale, float up) {
  DEBUG_P(20250430, "C0 free/used heap and free stack/Pointer:");
  DEBUG_P(20250430, rp2040.getFreeHeap());
  DEBUG_P(20250430, "/");
  DEBUG_P(20250430, rp2040.getUsedHeap());
  DEBUG_P(20250430, "/");
  DEBUG_P(20250430, rp2040.getFreeStack());
  DEBUG_P(20250430, "/");
  Serial.println(rp2040.getStackPointer(),HEX);
  
  delay(10);
  
  int sizetrack = latlon_manager.getCount();
  if(sizetrack <= 0){
    return;
  }
  //latest
  Coordinate c0 = latlon_manager.getData(0);
  cord_tft p0 = latLonToXY(c0.latitude, c0.longitude, center_lat, center_lon, scale, up);
  Serial.printf("Heap %d%% FrStk %dKB",rp2040.getFreeHeap()*100/rp2040.getTotalHeap(),rp2040.getFreeStack()/1000);
  

  if(p0.x != 120 || p0.y != 120){
    //backscreen.drawWideLine(p0.x,p0.y,120,120,2,COLOR_GREEN);
    drawThickLine(p0.x,p0.y,120,120,2,COLOR_GREEN);
  }

  for (int i = 0; i < sizetrack-1; i++) {
    Coordinate c0 = latlon_manager.getData(i);
    Coordinate c1 = latlon_manager.getData(i+1);
    if(c0.latitude == 0 && c1.longitude == 0){
      DEBUG_PLN(20250424,"ERR lat lon 0");
      break;
    }
    
    cord_tft p0 = latLonToXY(c0.latitude, c0.longitude, center_lat, center_lon, scale, up);
    cord_tft p1 = latLonToXY(c1.latitude, c1.longitude, center_lat, center_lon, scale, up);
    //Only if cordinates are different.
    if (p0.x != p1.x || p0.y != p1.y) {
      //backscreen.drawWideLine(p0.x,p0.y,p1.x,p1.y,2,COLOR_GREEN);
      drawThickLine(p0.x,p0.y,p1.x,p1.y,2,COLOR_GREEN);
    }
    
  }
}

void draw_Japan(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan1, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan2, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan3, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan4, COLOR_GREEN);
  nomap_drawn = false;
}

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

void draw_Biwako(double center_lat, double center_lon, float scale, float up, bool gmap_drawed) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_biwako, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_takeshima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_chikubushima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_okishima, COLOR_GREEN);
  draw_pilon_takeshima_line(center_lat, center_lon, scale, up);
  if(!gmap_drawed){//if (! && !isTaskInQueue(TASK_LOAD_MAPIMAGE))
    //Run gps_loop 
    gps_loop(2);
    fill_sea_land(center_lat, center_lon, scale, up);
    gps_loop(3);
  }
  nomap_drawn = false;
}

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

void draw_version_backscreen(){
  backscreen.setCursor(10, 240-12);
  backscreen.unloadFont();
  backscreen.setTextColor(COLOR_BLACK);
  backscreen.print("SOFTWARE:");
  backscreen.print(BUILDVERSION);
  backscreen.print("(Build ");
  backscreen.print(BUILDDATE);
  backscreen.print(")");

  backscreen.setCursor(150,0);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.println(" Version 5 ");
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void startup_demo_tft() {
  tft.fillScreen(COLOR_WHITE);
  tft.setTextColor(COLOR_RED, COLOR_WHITE);
  tft.setCursor(1, 0);
  tft.println(" Piolot Oriented");
  tft.setCursor(5, 14);
  tft.println("   Navigation System  for HPA");

  //320-292=28

  if(good_sd()){
    load_push_logo();//240x52
  }

  if(good_sd()){
    tft.setTextColor(COLOR_GREEN, COLOR_WHITE);
    tft.setCursor(20, SCREEN_HEIGHT - 28);
    tft.print("SD OK! MAP COUNT: ");
    tft.print(mapdata_count);
  }else{
    tft.setTextColor(COLOR_RED, COLOR_WHITE);
    tft.setCursor(10, SCREEN_HEIGHT - 28);
    tft.print("[ERROR: CHECK SD CARD !]");
  }
  float center_lat = 35.2334225841915;
  float center_lon = 136.091056306493;

  backscreen.fillScreen(COLOR_WHITE);
  draw_Biwako(center_lat,center_lon,2, 0,false);
  draw_version_backscreen();
  backscreen.pushSprite(0,52);
  delay(3200);

  for (int i = 0; i <= 40; i++) {
    backscreen.fillScreen(COLOR_WHITE);
    draw_Biwako(mapf(i,0,40,center_lat,PLA_LAT), mapf(i,0,40,center_lon,PLA_LON), 2 + i * 0.25, 0, false);
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }
  delay(10);

  
  for (int i = 0; i < 40; i++) {
    backscreen.fillScreen(COLOR_WHITE);
    draw_Biwako(mapf(i,0,100,PLA_LAT,center_lat), mapf(i,0,100,PLA_LON,center_lon), (2+40*0.25)*(40.0 - i)/40.0, 0, false);
    draw_version_backscreen();
    backscreen.pushSprite(0,52);
  }
}
void draw_demo_biwako(){
  backscreen.fillRect(5, 165, SCREEN_WIDTH-5*2, 25+5*2, COLOR_WHITE);
  backscreen.drawRect(5, 165, SCREEN_WIDTH-5*2, 25+5*2, COLOR_ORANGE);
  backscreen.setCursor(25,170);
  backscreen.setTextColor(COLOR_ORANGE);
  backscreen.print("LAKE BIWA DEMO x10 SPD");
  backscreen.setCursor(17,185);
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_BLACK);
  backscreen.print("Change setting to turn on/off demo.");
  backscreen.loadFont(AA_FONT_SMALL);
}


void clean_backscreen(){
  backscreen.fillScreen(COLOR_WHITE);
}
void push_backscreen(){
  backscreen.pushSprite(0, 40);
}

void draw_headertext(double degpersecond){
  backscreen.setTextWrap(false);
  if(trackwarning_until > millis()){
    backscreen.setTextColor((millis()/1000)%2==0?COLOR_RED:COLOR_BLACK, TFT_WHITE);
    backscreen.drawString("MT", SCREEN_WIDTH - 40, 1);
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
  }else{
    backscreen.setTextColor(TFT_BLACK, TFT_WHITE);
    backscreen.drawString("MT", SCREEN_WIDTH - 40, 1);
  }
  backscreen.drawString("GS      m/s", 18, 1);
}

/*
bool bankwarning_flipflop = true;
void draw_bankwarning() {
  int starty = 55;
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
*/

void draw_nogmap(double scale) {
  backscreen.setCursor(5, BACKSCREEN_SIZE - 15);
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

bool draw_gmap(float drawupward_direction){
  if (gmap_loaded_active) {
    if(is_trackupmode())
      gmap_sprite.pushRotated(&backscreen,-drawupward_direction);
    else
      gmap_sprite.pushToSprite(&backscreen,0,0);
    DEBUG_PLN(20240828, "pushed gmap");
    return true;
  }
  return false;
}


unsigned long last_loading_image = 0;
void draw_loading_image() {
  if (millis() - last_loading_image > 15) {  //50Hz
    last_loading_image = millis();

    int x = (last_loading_image / 20) % SCREEN_WIDTH;
    int color = isTaskRunning(TASK_LOAD_MAPIMAGE) ? COLOR_ORANGE : COLOR_GRAY;
    if (isTaskRunning(TASK_LOG_SD) || isTaskRunning(TASK_LOG_SDF) || isTaskRunning(TASK_SAVE_CSV)) {
      color = COLOR_RED;
    }

    if (last_loading_image - time_lastnmea < 15) {
      color = COLOR_BLUE;
    }
    /*
    tft.drawFastVLine(x, SCREEN_HEIGHT -30, 2, color);
    tft.drawFastVLine((x + 1) % SCREEN_WIDTH, SCREEN_HEIGHT -30, 2, COLOR_WHITE);
    tft.drawFastVLine((x + 2) % SCREEN_WIDTH, SCREEN_HEIGHT -30, 2, COLOR_WHITE);
    */
  }
}

int last_written_mh = 0;
unsigned long last_maxadr_time = 0;
int max_adreading = 0;



void draw_header(double degpersecond) {
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);  // Must load the font first

  int col = COLOR_GRAY;
  int posx = SCREEN_WIDTH / 2 - 8;
  
  if(get_gps_mps() > 2.0){
    if (degpersecond > 1.0)
      col = COLOR_GREEN;
    if (degpersecond < -1.0)
      col = COLOR_BLUE;
    if(abs(degpersecond) > 3.0){
      header_footer.setTextColor(COLOR_WHITE,col);
      header_footer.fillRect(102, 0, 48, 40, col);
      if(abs(degpersecond) > 10.0){
        header_footer.drawRect(102, 0, 48, 40, COLOR_RED);
        header_footer.drawRect(103, 1, 46, 38, COLOR_RED);
      }
    }else{
      header_footer.setTextColor(col);
    }
  }else{
    header_footer.setTextColor(col);
  }
  if (degpersecond != 0)
    posx -= 5;

  header_footer.setCursor(posx, 8);
  header_footer.printf("%+.1f",degpersecond);
  header_footer.drawString("deg/s", SCREEN_WIDTH / 2 - 15, 23);


  //drawbar(degpersecond, -1);


  const int mtlx = SCREEN_WIDTH - 40;
  const int mtvx = SCREEN_WIDTH - 92;
  header_footer.setTextWrap(false);
  header_footer.loadFont(NM_FONT_LARGE);  // Must load the font first
  int sel_col = get_gps_mps()<2?COLOR_GRAY:COLOR_BLACK;

  header_footer.setCursor(mtvx, 3);
  if(trackwarning_until > millis()){
    if((millis()/1000)%2==0){
      header_footer.setTextColor(COLOR_RED, TFT_WHITE);
    }else{
      header_footer.fillRect(mtvx,0,92,3,COLOR_RED);
      header_footer.setTextColor(COLOR_WHITE, TFT_RED,true);
    }
    header_footer.printf("%03d", (int)get_gps_magtrack());
    header_footer.setTextColor(sel_col, TFT_WHITE);
  }else{
    header_footer.setTextColor(sel_col, TFT_WHITE);
    header_footer.printf("%03d", (int)get_gps_magtrack());
  }

  header_footer.setCursor(1, 3);
  header_footer.printf("%4.1f", get_gps_mps());

  header_footer.drawFastHLine(0,39,240,COLOR_BLACK);

  if(get_gps_numsat() == 0){
    header_footer.drawFastHLine(5, 19, SCREEN_WIDTH-10, COLOR_RED);
    header_footer.drawFastHLine(5, 20, SCREEN_WIDTH-10, COLOR_RED);
  }

  header_footer.pushSprite(0,0);
}

extern int course_warning_index;

void draw_footer(){
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0,0,240,COLOR_BLACK);

  // ====Navigation.==== Distance to plathome.
  if(currentdestination != -1 && currentdestination < destinations_count){
    header_footer.setCursor(1, 12);
    header_footer.setTextColor(COLOR_MAGENTA);
    header_footer.printf("MC%3d", magc);


    header_footer.setCursor(60, 12);
    header_footer.setTextColor(COLOR_BLACK);
    if(!get_gps_fix() && !get_demo_biwako())
      header_footer.print("---km");
    else
      header_footer.printf(dest_dist>1000?"%.0fkm":dest_dist>100?"%.1fkm":"%.2fkm", dest_dist);
    

    if(currentdestination != -1 && currentdestination < destinations_count){
      header_footer.setTextColor(COLOR_MAGENTA);
      header_footer.setCursor(1, 27);
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

      header_footer.setCursor(80, 27);
      header_footer.setTextWrap(false);
      header_footer.print(extradestinations[currentdestination].name);
    }
    TinyGPSTime time = get_gpstime();
    header_footer.unloadFont();
    if (time.isValid()) {
      header_footer.setCursor(1, 2);
      header_footer.setTextSize(1);
      header_footer.setTextColor(COLOR_BLACK);
      header_footer.printf("%02d:%02d:%02d JST", (time.hour()+9)%24, time.minute(), time.second());
      header_footer.printf("%d",course_warning_index);
    }
    header_footer.loadFont(AA_FONT_SMALL);  // Must load the font first
  }

  // ====Battery====
  int adreading = analogRead(BATTERY_PIN);

  if (adreading > max_adreading) {
    max_adreading = adreading;
    last_maxadr_time = millis();
  } else if (millis() - last_maxadr_time > 3000L) {
    max_adreading += (adreading - max_adreading) * 0.4;
  }
  double input_voltage = BATTERY_MULTIPLYER(max_adreading);
  header_footer.setCursor(SCREEN_WIDTH - 37, 13);
  header_footer.setTextColor(COLOR_GREEN);
  if (digitalRead(24)) {
    header_footer.print("USB");
  } else {
    if (input_voltage > 4.25) {
      header_footer.setCursor(SCREEN_WIDTH - 45, 13);
      header_footer.setTextColor(COLOR_RED);
      header_footer.printf("%.2fV", input_voltage);
    } else if (input_voltage < BAT_LOW_VOLTAGE) {
      header_footer.setCursor(SCREEN_WIDTH - 67, 13);
      header_footer.setTextColor(COLOR_RED);
      header_footer.print("BATLOW");
    } else if (input_voltage < 3.75) {
      header_footer.setCursor(SCREEN_WIDTH - 45, 13);
      header_footer.setTextColor(COLOR_MAGENTA);
      header_footer.printf("%.2fV", input_voltage);
    } else {//between 3.75-4.25
      header_footer.setCursor(SCREEN_WIDTH - 45, 13);
      header_footer.setTextColor(COLOR_GREEN);
      header_footer.printf("%.2fV", input_voltage);
    }
  }


  // ====GNSS====
  int col = COLOR_GREEN;
  if (get_gps_numsat() < 5) {
    header_footer.setTextColor(COLOR_WHITE,COLOR_RED);
    header_footer.fillRect(SCREEN_WIDTH-101,12, 44,15, COLOR_RED);
  } else if (get_gps_numsat() < 10) {
    header_footer.setTextColor(COLOR_DARKORANGE);
  }
  header_footer.setCursor(SCREEN_WIDTH-100,13);
  header_footer.printf("%dsats", get_gps_numsat());

  // 5 decimal places latitude, longitude print.
  header_footer.unloadFont();
  header_footer.setTextColor(COLOR_BLACK);
  header_footer.setCursor(115,2);
  header_footer.printf("%.5f", get_gps_lat());
  if(get_gps_lat() >= 0)
    header_footer.print("N");
  else
    header_footer.print("S");

  header_footer.setCursor(175,2);
  header_footer.printf("%.5f", get_gps_lon());
  if(get_gps_lon() >= 0)
    header_footer.print("E");
  else
    header_footer.print("W");
  header_footer.loadFont(AA_FONT_SMALL);
  

  // ====SD==== draw
  if (good_sd()) {
    header_footer.fillRect(SCREEN_WIDTH - 22, 25, 22, 15, COLOR_GREEN);
    header_footer.setTextColor(COLOR_WHITE, COLOR_GREEN);
  } else {
    header_footer.fillRect(SCREEN_WIDTH - 22, 25, 22, 15, COLOR_RED);
    header_footer.setTextColor(COLOR_WHITE, COLOR_RED);
  }
  header_footer.setCursor(SCREEN_WIDTH - 20,26);
  header_footer.print("SD");

  header_footer.pushSprite(0,280);
}

void draw_headingupmode() {
  tft.setCursor(SCREEN_WIDTH / 2 - 18, SCREEN_HEIGHT - 21);
  tft.setTextColor(COLOR_BLACK);  // Highlight selected line
  tft.println("HDG UP");
}


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




void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(PILON_NORTH_LAT, PILON_NORTH_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft w_pilon = latLonToXY(PILON_WEST_LAT, PILON_WEST_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft takeshima = latLonToXY(TAKESHIMA_LAT, TAKESHIMA_LON, mapcenter_lat, mapcenter_lon, scale, upward);

  
  backscreen.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, COLOR_GREEN);
  backscreen.drawLine(pla.x, pla.y, takeshima.x, takeshima.y, COLOR_GREEN);
  backscreen.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, COLOR_GREEN);
  backscreen.drawCircle(pla.x, pla.y, scale*10.55f/cos(radians(35)),COLOR_GREEN);
}


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
        if (latitude_index >= 0 && latitude_index < ROW_FILLDATA - 1 && longitude_index >= 0 && longitude_index < COL_FILLDATA - 1) {
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
  const int height = SCREEN_WIDTH;
  const int radius = SCREEN_WIDTH / 2 - 20;
  const int shift_down = 0;
  return (height / 2) - (int)(sin(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) + shift_down;
}


void draw_nomapdata() {
  int posy = SCREEN_HEIGHT - 130;
  
  if (get_gps_connection()) {
    //"GPS Module connected."
  } else {
    backscreen.setCursor(3,150);
    backscreen.setTextColor(COLOR_MAGENTA);
    backscreen.print("GPS connection not found !! Check connection, or try reset.");
    return;
  }

  int col = COLOR_BLACK;
  if (!get_gps_fix()) {
    backscreen.fillRect(5, 145, SCREEN_WIDTH-5*2, 30+5*2, COLOR_WHITE);
    backscreen.drawRect(5, 145, SCREEN_WIDTH-5*2, 30+5*2, COLOR_RED);
    backscreen.drawRect(6, 146, SCREEN_WIDTH-5*2-2, 30+5*2-2, COLOR_RED);
    backscreen.setCursor(23,150);
    backscreen.setTextColor(COLOR_ORANGE);
    backscreen.print("Scanning GNSS/GPS Signal");
    char text[28];
    int dotCount = (millis() / 900) % 10;
    // Safely create the string with snprintf
    snprintf(text, sizeof(text), "Stand by for fix%.*s", dotCount, "..........");
    backscreen.setCursor(45,165);
    backscreen.print(text);
  }
  else if (get_gps_numsat() == 0 && !get_demo_biwako()) {
    backscreen.fillRect(5, 145, SCREEN_WIDTH-5*2, 30+5*2, COLOR_WHITE);
    backscreen.drawRect(5, 145, SCREEN_WIDTH-5*2, 30+5*2, COLOR_RED);
    backscreen.drawRect(6, 146, SCREEN_WIDTH-5*2-2, 30+5*2-2, COLOR_RED);
    backscreen.setCursor(30,150);
    backscreen.setTextColor(COLOR_RED);
    backscreen.print("Weak GNSS/GPS Signal");
    char text[28];
    int dotCount = (millis() / 900) % 10;
    // Safely create the string with snprintf
    snprintf(text, sizeof(text), "Scanning signal%.*s", dotCount, "..........");
    backscreen.setCursor(45,165);
    backscreen.print(text);
  } else if (nomap_drawn) {
    //"NO MAPDATA.GPS Fixed."
  }
}

unsigned long lastdrawn_sddetail = 0;
extern char sdfiles[20][32];
extern int sdfiles_size[20]; 
extern int max_page;     // Global variable to store maximum page number
bool loading_sddetail = true;

void draw_sddetail(bool redraw_screen, int page) {
  if(redraw_screen){
    loading_sddetail = true;
    if(max_page <= 0)
      enqueueTask(createBrowseSDTask(0));
    else
      enqueueTask(createBrowseSDTask(page%(max_page+1)));
  }
  if(redraw_screen || millis() - lastdrawn_sddetail > 500) {

    lastdrawn_sddetail = millis();
    header_footer.fillScreen(COLOR_WHITE);
    header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
    header_footer.setTextSize(2);
    header_footer.setCursor(1, 1);
    if(max_page >= 0 && !loading_sddetail)
      header_footer.printf("SD DETAIL  %d/%d",page%(max_page+1)+1,max_page+1);
    else
      header_footer.printf("SD DETAIL  loading...");
    header_footer.pushSprite(0,0);
    backscreen.fillScreen(COLOR_WHITE);
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
    backscreen.pushSprite(0,40);
    header_footer.fillScreen(COLOR_WHITE);
    header_footer.pushSprite(0,SCREEN_HEIGHT-40);
    /*
    // Print available files on SD card
    Serial.println("Available WAV files:");
    File root = SD.open("wav/");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
            String filename = entry.name();
            if (filename.endsWith(".wav") || filename.endsWith(".WAV")) {
                Serial.println(entry.name());
            }
        }
        entry.close();
    }
    root.close();
    */
  }
}



//==================MODE DRAWS===============
unsigned long lastdrawn_const = 0;
void draw_gpsdetail(bool redraw_screen, int page) {
  if (redraw_screen || millis() - lastdrawn_const > 1000) {
    lastdrawn_const = millis();
    bool aru = false;
    for (int i = 0; i < 32; i++) {
      if (satellites[i].PRN != 0) aru = true;  // Skip empty entries
    }

    header_footer.fillScreen(COLOR_WHITE);

    if (page % 2 == 1) {
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
    if (page % 2 == 0) {
      header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
      header_footer.setTextSize(2);
      header_footer.setCursor(1, 1);
      header_footer.println("GPS DETAIL 1:CONSTELLATION");

      header_footer.setTextColor(COLOR_CYAN, COLOR_WHITE);
      header_footer.setCursor(0,18);
      header_footer.print("GPS ");
      header_footer.setTextColor(COLOR_GREEN, COLOR_WHITE);
      header_footer.print("GLO ");
      header_footer.setTextColor(COLOR_BLUE, COLOR_WHITE);
      header_footer.print("GAL ");
      header_footer.setTextColor(COLOR_RED, COLOR_WHITE);
      header_footer.print("QZS ");
      header_footer.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      header_footer.print("BEI ");
      header_footer.pushSprite(0,0);

      

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



      backscreen.fillSprite(COLOR_WHITE);
      backscreen.drawCircle(240 / 2, 240 / 2, 240 / 2 - 2, COLOR_BLACK);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

      if (!aru){
        backscreen.pushSprite(0,40);
        return;
      }


      for (int i = 0; i < 32; i++) {
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
          backscreen.fillCircle(x, y, size, COLOR_RED);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GPS)
          backscreen.fillCircle(x, y, size, COLOR_CYAN);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GLONASS)
          backscreen.fillCircle(x, y, size, COLOR_GREEN);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_GALILEO)
          backscreen.fillCircle(x, y, size, COLOR_BLUE);
        else if (satellites[i].satelliteType == SATELLITE_TYPE_BEIDOU)
          backscreen.fillCircle(x, y, size, COLOR_ORANGE);
        else
          backscreen.fillCircle(x, y, size, COLOR_BLACK);

        int textx = constrain(x + 6, 0, SCREEN_WIDTH - 20);
        if (satellites[i].PRN >= 10)
          textx -= 10;
        if (satellites[i].PRN >= 100)
          textx -= 10;
        backscreen.setCursor(textx, y - 3);
        backscreen.print(satellites[i].PRN);
      }
      backscreen.pushSprite(0,40);
    }
  }
}


void draw_maplist_mode(bool redraw_screen, int maplist_page) {
  if (redraw_screen || millis() - lastdrawn_const > 10000L) {
    lastdrawn_const = millis();

    mapdata* mapdatas[] = { &map_shinura, &map_okishima, &map_takeshima, &map_chikubushima, &map_biwako, &map_handaioutside, &map_handaihighway, &map_handaihighway2, &map_handaiinside1, &map_handaiinside2, &map_handaiinside3,
                            &map_handaiinside4, &map_handaiinside5, &map_handairailway, &map_handaicafe, &map_japan1, &map_japan2, &map_japan3, &map_japan4 };
    int sizeof_mapflash = sizeof(mapdatas) / sizeof(mapdatas[0]);
    tft.fillScreen(COLOR_WHITE);
    tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
    tft.setCursor(1, 1);


    int pagetotal = 1 + (sizeof_mapflash + mapdata_count) / 30;
    int pagenow = maplist_page % pagetotal;

    tft.printf("MAPLIST FLSH:%d/SD:%d (%d/%d)", sizeof_mapflash, mapdata_count, pagenow + 1, pagetotal);

    tft.unloadFont();
    tft.setTextSize(1);
    int posy = 20;
    tft.setCursor(1, posy);
    tft.setTextColor(COLOR_BLACK);
    if (pagenow == 0) {
      for (int i = 0; i < sizeof_mapflash; i++) {
        tft.printf("FLSH %d: %s,%4.2f,%4.2f,%d", mapdatas[i]->id, mapdatas[i]->name, mapdatas[i]->cords[0][0], mapdatas[i]->cords[0][1], mapdatas[i]->size);
        posy += 10;
        tft.setCursor(1, posy);
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

      tft.printf("SD %d:%s,%4.2f,%4.2f,%d,%c", i, extramaps[i].name, lat1, lon1, extramaps[i].size, name_firstchar);
      posy += 10;
      tft.setCursor(1, posy);
    }

    tft.loadFont(AA_FONT_SMALL);  // Must load the font first
  }
}




void draw_setting_mode(bool redraw_screen, int selectedLine, int cursorLine) {
  if(redraw_screen){
    const int separation = 25;
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
      backscreen.print(menu_settings[i].getLabel(selectedLine == i).c_str());
      //textmanager.drawTextf(menu_settings[i].id, 2, 10, startY + i * separation, col, menu_settings[i].getLabel(selectedLine == i).c_str());
    }
    backscreen.pushSprite(0,30);

    header_footer.unloadFont();
    header_footer.fillScreen(COLOR_WHITE);

    header_footer.setTextSize(1);
    header_footer.setCursor(2, 15);
    header_footer.setTextColor(COLOR_GRAY);
    header_footer.printf("CPU temp %.1fC",analogReadTemp());

    #ifdef RELEASE
    header_footer.setCursor(100,15);
    header_footer.printf("Heap %d%% FrStk %dKB",rp2040.getFreeHeap()*100/rp2040.getTotalHeap(),rp2040.getFreeStack()/1000);
    #endif

    header_footer.loadFont(AA_FONT_SMALL);
    header_footer.pushSprite(0,SCREEN_HEIGHT-40);
  }
}