#include "display_tft.h"
#include "latlon.h"
#include "navdata.h"
#include "gps_functions.h"
#include "settings.h"
#include "compass.h"

#ifdef TFT_USE_ST7789
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  // TFT_CS must be actually connected to ST7789 Module CS pin for some reason.
  // Library is not accepting -1 CS or my module that I have. thus not working.
#endif
#ifdef TFT_USE_ST7735
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#endif

#ifdef TFT_USE_ILI9341
  //default pin setting
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);//CS can be -1 but now, RST must be connected to actual pin. somehow...
  //Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI,TFT_CLK,-1,-1);   Does not work....
#endif



int screen_brightness = 255; // Example brightness value
const int brightnessLevels[] = {30, 100, 150, 200, 255}; // Example brightness levels
int brightnessIndex = 4; // Default to 60

#define MODE_TRACKUP 0
#define MODE_HEADINGUP 1
#define MODE_NORTHUP 2
int upward_mode = MODE_NORTHUP;


int map_shift_down = 0;
#ifdef TFT_USE_ST7735
  #define MAP_SHIFT_DOWN_DEFAULT 40
#else
  #define MAP_SHIFT_DOWN_DEFAULT 60   //80+60=140 is centerY.
#endif

void toggle_mode(){
  upward_mode = (upward_mode +1)%3;
  if(upward_mode == MODE_TRACKUP || upward_mode == MODE_HEADINGUP){
    map_shift_down = MAP_SHIFT_DOWN_DEFAULT;
  }else if(upward_mode == MODE_NORTHUP){
    map_shift_down = 0;
  }
}
bool is_headingupmode(){
  return upward_mode == MODE_HEADINGUP;
}
bool is_northupmode(){
  return upward_mode == MODE_NORTHUP;
}
bool is_trackupmode(){
  return upward_mode == MODE_TRACKUP;
}


void setup_tft() {

  // Initialize backlight control pin
  pinMode(TFT_BL, OUTPUT);
  #ifdef XIAO_RP2040
  analogWriteFreq(BL_PWM_FRQ); // 1000Hz
  #endif
  analogWrite(TFT_BL,255-screen_brightness);// For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.



  #ifdef TFT_USE_ST7789
    tft.init(240, 320);           // Init ST7789 320x240
  #endif
  #ifdef TFT_USE_ST7735
    tft.initR(INITR_BLACKTAB);
  #endif
  #ifdef TFT_USE_ILI9341
    tft.begin();
  #endif
  //tft.invertDisplay(true);

/* not working... for now.
  #ifdef XIAO_RP2040
    #define SPI_CLOCK_SPEED 40000000
    tft.setSPISpeed(SPI_CLOCK_SPEED);
  #endif
*/

  Serial.println(F("Initialized"));
  tft.fillScreen(COLOR_BLACK);

}




struct line {
  int x1, y1, x2, y2;
};


class StrokeManager {
  struct Point {
    int x;
    int y;
  };

  struct Stroke {
    stroke_group id;
    Point* points;
    int pointCount;
    int maxPoints;
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

  bool addStroke(stroke_group id, int maxPoints) {
    if (strokeCount >= maxStrokes) {
      Serial.println("ERR max stroke reached");
      return false;  // No more space for new strokes
    }
    strokes[strokeCount].points = (Point*)malloc(maxPoints * sizeof(Point));
    if (strokes[strokeCount].points == nullptr) {
      return false;  // Memory allocation failed
    }
    strokes[strokeCount].pointCount = 0;
    strokes[strokeCount].maxPoints = maxPoints;
    strokes[strokeCount].id = id;
    strokeCount++;
    return true;
  }

  bool addPointToStroke(int x, int y) {
    if (strokeCount == 0) {
      return false;  // No strokes to add points to
    }
    Stroke* stroke = &strokes[strokeCount - 1];
    if (stroke->pointCount >= stroke->maxPoints) {
      return false;  // No more space for new points in this stroke
    }
    stroke->points[stroke->pointCount].x = x;
    stroke->points[stroke->pointCount].y = y;
    stroke->pointCount++;
    return true;
  }


  void removeAllStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      free(strokes[i].points);  // Free allocated memory for each stroke
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
      Serial.print(":size");
      Serial.print(strokes[i].pointCount);
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

  void drawStroke(int id) {
    for (int i = 0; i < strokeCount; i++) {
      if (id != strokes[i].id) {
        continue;
      }
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        tft.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, COLOR_BLACK);
      }
    }
  }


  void drawAllStroke() {
    for (int i = 0; i < strokeCount; i++) {
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        tft.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, COLOR_BLACK);
      }
    }
    Serial.print("stroke black ");
    Serial.println(strokeCount);
  }
};

#ifdef XIAO_SAMD21
  #define MAX_STROKES 300
#else
  #define MAX_STROKES 800
#endif
StrokeManager strokeManager(MAX_STROKES);


// Convert degrees to radians
float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}


double rad2deg(double rad) {
    return rad * (180.0 / PI);
}


#define RADIUS_EARTH_KM 6371.0 // Earth's radius in kilometers


// Function to calculate distance using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = degreesToRadians(lat1);
  lon1 = degreesToRadians(lon1);
  lat2 = degreesToRadians(lat2);
  lon2 = degreesToRadians(lon2);
  
  // Haversine formula
  double dlon = lon2 - lon1;
  double dlat = lat2 - lat1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = RADIUS_EARTH_KM * c;
  
  return distance;
}




bool draw_circle_km(float scale, float km) {
  int radius = PX_PER_KM(scale) * km;
  int ypos = SCREEN_HEIGHT / 2 + map_shift_down - radius;
  if (ypos > 16) {
    tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, radius, COLOR_RED);
    tft.setCursor(SCREEN_WIDTH / 2 + 2, ypos - 2);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    if (km > 0.999f) {
      tft.print(int(km));
    } else {
      tft.print(km, 1);
    }
    tft.println("km");
    return true;
  } else {
    return false;
  }
}


// draw magnetic north east west south according to truetrack, where truetrack is direction of upward display.
// However direction of y-axis is downward for tft display.
void draw_compass(float truetrack, uint16_t col) {
  tft.setTextWrap(false);
  int dist = 73;
  if(is_northupmode()){
    dist = 50;
  }
  int centerx = SCREEN_WIDTH / 2;
  int centery = SCREEN_HEIGHT / 2 + map_shift_down;
  //Varietion 8 degrees.
  float radian = degreesToRadians(truetrack+8.0);
  float radian45offset = degreesToRadians(truetrack+8.0 + 45);
  tft.setTextColor(col);
  tft.setTextSize(2);

  cord_tft n = { int(centerx + sin(-radian) * dist), int(centery - cos(radian) * dist) };
  cord_tft e = { int(centerx + cos(radian) * dist), int(centery - sin(radian) * dist) };
  cord_tft s = { int(centerx + sin(radian) * dist), int(centery + cos(radian) * dist) };
  cord_tft w = { int(centerx - cos(-radian) * dist), int(centery - sin(-radian) * dist) };
  if (!n.isOutsideTft()) {
    tft.setCursor(n.x-5, n.y);
    tft.print("N");
  }
  if (!e.isOutsideTft()) {
    tft.setCursor(e.x-5, e.y);
    tft.print("E");
  }
  if (!s.isOutsideTft()) {
    tft.setCursor(s.x-5, s.y);
    tft.print("S");
  }
  if (!w.isOutsideTft()) {
    tft.setCursor(w.x-5, w.y);
    tft.print("W");
  }

  cord_tft nw = { int(centerx + sin(-radian45offset) * dist), int(centery - cos(radian45offset) * dist) };
  cord_tft ne = { int(centerx + cos(radian45offset) * dist), int(centery - sin(radian45offset) * dist) };
  cord_tft se = { int(centerx + sin(radian45offset) * dist), int(centery + cos(radian45offset) * dist) };
  cord_tft sw = { int(centerx - cos(-radian45offset) * dist), int(centery - sin(-radian45offset) * dist) };

  tft.setTextSize(1);
  if (!nw.isOutsideTft()) {
    tft.setCursor(nw.x-5, nw.y);
    tft.print("NW");
  }
  if (!ne.isOutsideTft()) {
    tft.setCursor(ne.x-5, ne.y);
    tft.print("NE");
  }
  if (!se.isOutsideTft()) {
    tft.setCursor(se.x-5, se.y);
    tft.print("SE");
  }
  if (!sw.isOutsideTft()) {
    tft.setCursor(sw.x-5, sw.y);
    tft.print("SW");
  }
  tft.setTextWrap(true);
}





void draw_km_circle(float scale) {
  if (!draw_circle_km(scale, 400)) {
    if (!draw_circle_km(scale, 20)) {
      if (!draw_circle_km(scale, 10)) {
        if (!draw_circle_km(scale, 5)) {
          if (!draw_circle_km(scale, 2)) {
            if (!draw_circle_km(scale, 1)) {
              draw_circle_km(scale, 0.4);
              draw_circle_km(scale, 0.2);
            } else {
              draw_circle_km(scale, 0.5);
            }
          } else {
            draw_circle_km(scale, 1);
          }
        } else {
          draw_circle_km(scale, 2);
        }
      } else {
        draw_circle_km(scale, 5);
      }
    } else {
      draw_circle_km(scale, 10);
    }
  }
  else {
    draw_circle_km(scale, 200);
  }
}

bool fresh = false;
float last_scale = 1.0;
unsigned long lastfresh_millis = 0;
float last_up = 0;
float last_truetrack = 0;

void blacken_display(bool redraw) {
  fresh = false;
  if (millis() - lastfresh_millis > 20000 || redraw) {
    lastfresh_millis = millis();
    fresh = true;
  }
  if (fresh) {
    tft.fillScreen(COLOR_BLACK);
  } else {
    strokeManager.drawAllStroke();
  }
  strokeManager.removeAllStrokes();
}


int rb_x_old,rb_y_old,lb_x_old,lb_y_old;
void draw_triangle(){
  if(upward_mode == MODE_NORTHUP){
    int magtrack = get_gps_truetrack();
    float radians = degreesToRadians(magtrack);
    int x_track =  SCREEN_HEIGHT * sin(radians);
    int y_track =  SCREEN_HEIGHT * -cos(radians);

    tft.drawLine(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + x_track, SCREEN_HEIGHT / 2 + map_shift_down + y_track, COLOR_WHITE);
    strokeManager.addStroke(STRK_OTHER, 2);
    strokeManager.addPointToStroke(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down);
    strokeManager.addPointToStroke(SCREEN_WIDTH / 2 + x_track, SCREEN_HEIGHT / 2 + map_shift_down + y_track);

    int rb_x_new = -4 * cos(radians) - 12 * sin(radians);
    int rb_y_new = -4 * sin(radians) + 12 * cos(radians);
    int lb_x_new = 4 * cos(radians) - 12 * sin(radians);
    int lb_y_new = 4 * sin(radians) + 12 * cos(radians);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + rb_x_old, SCREEN_HEIGHT / 2 + map_shift_down + rb_y_old, SCREEN_WIDTH / 2 + lb_x_old, SCREEN_HEIGHT / 2 + map_shift_down + lb_y_old, COLOR_BLACK);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + rb_x_new, SCREEN_HEIGHT / 2 + map_shift_down + rb_y_new, SCREEN_WIDTH / 2 + lb_x_new, SCREEN_HEIGHT / 2 + map_shift_down + lb_y_new, COLOR_WHITE);
    rb_x_old = rb_x_new;
    rb_y_old = rb_y_new;
    lb_x_old = lb_x_new;
    lb_y_old = lb_y_new;
  }
  if(upward_mode == MODE_TRACKUP){
    #if defined(COMPASS_QMC5883L) || defined(COMPASS_BNO055)
      tft.drawFastVLine(SCREEN_WIDTH / 2, 20, map_shift_down + SCREEN_HEIGHT / 2 - 22, COLOR_WHITE);
      int heading = get_magnetic_heading();
      int magtrack = get_gps_magtrack();
      int difangle = heading-magtrack;
      int rightbottomx = 4;int rightbottomy = +10;
      int leftbottomx = -4;int leftbottomy = +10;
      float radians = degreesToRadians(difangle);

      int rb_x_new = -4 * cos(radians) - 12 * sin(radians);
      int rb_y_new = -4 * sin(radians) + 12 * cos(radians);
      int lb_x_new = 4 * cos(radians) - 12 * sin(radians);
      int lb_y_new = 4 * sin(radians) + 12 * cos(radians);
      tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + rb_x_old, SCREEN_HEIGHT / 2 + map_shift_down + rb_y_old, SCREEN_WIDTH / 2 + lb_x_old, SCREEN_HEIGHT / 2 + map_shift_down + lb_y_old, COLOR_BLACK);
      tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + rb_x_new, SCREEN_HEIGHT / 2 + map_shift_down + rb_y_new, SCREEN_WIDTH / 2 + lb_x_new, SCREEN_HEIGHT / 2 + map_shift_down + lb_y_new, COLOR_WHITE);
      rb_x_old = rb_x_new;
      rb_y_old = rb_y_new;
      lb_x_old = lb_x_new;
      lb_y_old = lb_y_new;
    #else
      tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 - 4, SCREEN_HEIGHT / 2 + map_shift_down + 12, SCREEN_WIDTH / 2 + 4, SCREEN_HEIGHT / 2 + map_shift_down + 12, COLOR_RED);
    #endif
  }else if(upward_mode == MODE_HEADINGUP){
    //heading up mode
    tft.drawFastVLine(SCREEN_WIDTH / 2, 20, map_shift_down + SCREEN_HEIGHT / 2 - 22, COLOR_GREEN);
    int heading = get_magnetic_heading();
    int magtrack = get_gps_magtrack();
    int difangle = magtrack-heading;
    float radians = degreesToRadians(difangle);

    int x_track =  - SCREEN_HEIGHT * sin(radians);
    int y_track =  SCREEN_HEIGHT * cos(radians);

    tft.drawLine(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 + x_track, SCREEN_HEIGHT / 2 + map_shift_down + y_track, COLOR_WHITE);
    strokeManager.addStroke(STRK_OTHER, 2);
    strokeManager.addPointToStroke(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down);
    strokeManager.addPointToStroke(SCREEN_WIDTH / 2 + x_track, SCREEN_HEIGHT / 2 + map_shift_down + y_track);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + map_shift_down, SCREEN_WIDTH / 2 - 4, SCREEN_HEIGHT / 2 + map_shift_down + 12, SCREEN_WIDTH / 2 + 4, SCREEN_HEIGHT / 2 + map_shift_down + 12, COLOR_WHITE);
  }
}

void drawJapan(bool& redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan1, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan2, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan3, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_japan4, COLOR_GREEN);
  draw_km_circle(scale);
  draw_compass(last_truetrack, COLOR_BLACK);
  draw_compass(up, COLOR_WHITE);
  last_truetrack = up;
  redraw = false;
}

void drawShinura(bool& redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_shinura, COLOR_GREEN);
  draw_km_circle(scale);
  draw_compass(last_truetrack, COLOR_BLACK);
  draw_compass(up, COLOR_WHITE);

  last_truetrack = up;
  redraw = false;
}

void drawBiwako(bool& redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_biwako, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_takeshima, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_chikubushima, COLOR_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_okishima, COLOR_GREEN);
  draw_km_circle(scale);
  draw_pilon_takeshima_line(center_lat, center_lon, scale, up);
  fill_sea_land(center_lat, center_lon, scale, up);
  draw_compass(last_truetrack, COLOR_BLACK);
  draw_compass(up, COLOR_WHITE);
  last_truetrack = up;
  redraw = false;
}

void drawOsaka(bool& redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaioutside, COLOR_CYAN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway, COLOR_YELLOW);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway2, COLOR_YELLOW);

  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside1, COLOR_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside2, COLOR_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside3, COLOR_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside4, COLOR_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside5, COLOR_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handairailway, COLOR_ORANGE);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaicafe, COLOR_GREEN);


  draw_km_circle(scale);
  draw_compass(last_truetrack, COLOR_BLACK);
  draw_compass(up, COLOR_WHITE);

  last_truetrack = up;
  redraw = false;
}


void startup_demo_tft() {
  for (int i = 0; i <= 360; i += 30) {
    float scale = 0.15;
    float center_lat = 35.3034225841915;
    float center_lon = 136.1461056306493;
    bool redraw = false;
    drawBiwako(redraw, center_lat, center_lon, scale, i);
    draw_degpersecond(30.0 / (300.0 / 1000));
    draw_gpsinfo();
    tft.setCursor(SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 - 40);
    tft.setTextSize(3);
    tft.print("DEMO");
    delay(100);
  }
}

void draw_degpersecond(float degpersecond) {
  tft.fillRect(SCREEN_WIDTH - 28, 0, 28, 8, COLOR_BLACK);

  if (degpersecond > 0) {
    tft.setTextColor(COLOR_GREEN);
  }
  if (degpersecond < 0) {
    tft.setTextColor(COLOR_LIGHT_BLUE);
  }
  if (degpersecond == 0) {
    tft.setTextColor(COLOR_BRIGHTGRAY);
  }

  tft.setTextSize(1);
  tft.setCursor(SCREEN_WIDTH - 28, 0);
  tft.print(degpersecond, 1);
  tft.setCursor(SCREEN_WIDTH - 30, 8);
  tft.print("deg/s");

  int barposy = 22;
  #ifdef TFT_USE_ILI9341
    barposy = 25;
  #endif
  tft.drawFastHLine(0, barposy - 1, SCREEN_WIDTH, COLOR_BLACK);
  tft.drawFastHLine(0, barposy, SCREEN_WIDTH, COLOR_BLACK);
  tft.drawFastHLine(0, barposy + 1, SCREEN_WIDTH, COLOR_BLACK);
  int barwidth = (abs(degpersecond) * SCREEN_WIDTH / 2) / 3.0;
  if (degpersecond > 0) {
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy - 1, barwidth, COLOR_GREEN);
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy, barwidth, COLOR_GREEN);
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy + 1, barwidth, COLOR_GREEN);
  } else {
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy - 1, barwidth, COLOR_LIGHT_BLUE);
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy, barwidth, COLOR_LIGHT_BLUE);
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy + 1, barwidth, COLOR_LIGHT_BLUE);
  }
}

void draw_bankwarning() {
  tft.fillRect(SCREEN_WIDTH / 2 - 50, SCREEN_HEIGHT / 2 - 23, 100, 30, COLOR_BLACK);
  tft.setCursor(SCREEN_WIDTH / 2 - 50, SCREEN_HEIGHT / 2 - 20);
  tft.setTextColor(COLOR_RED);
  tft.setTextSize(3);
  tft.print("!BANK!");
  tft.setCursor(SCREEN_WIDTH / 2 - 50 - 1, SCREEN_HEIGHT / 2 - 20 - 2);
  tft.setTextColor(COLOR_WHITE);
  tft.print("!BANK!");
  tft.setTextSize(1);
}

int last_written_mh = 0;
void draw_gpsinfo() {
  #ifdef TFT_USE_ILI9341
    tft.fillRect(SCREEN_WIDTH / 2 - 40, 0, 76, 24, COLOR_BLACK);
    if(is_trackupmode() || is_northupmode()){
      tft.setCursor(SCREEN_WIDTH / 2 - 40, 0);
      tft.setTextColor(COLOR_CYAN);
      tft.setTextSize(2);
      tft.print("MT"); 
      tft.setTextSize(3);
      tft.printf("%03d", (int)get_gps_magtrack());
    }else if(is_headingupmode()){//heading up mode
      tft.setCursor(SCREEN_WIDTH / 2 - 40, 9);
      tft.setTextColor(COLOR_GREEN);
      tft.setTextSize(2);
      tft.print("MH");  // Display as an integer
      tft.setTextSize(3);
      tft.printf("%03d", get_magnetic_heading());
    }
  #else
    tft.fillRect(SCREEN_WIDTH / 2 - 25, 0, 46, 14, COLOR_BLACK);
    if(is_trackupmode() || is_northupmode()){
      tft.setCursor(SCREEN_WIDTH / 2 - 25, 0);
      tft.setTextColor(COLOR_CYAN);
      tft.setTextSize(1);
      tft.print("MT"); 
      tft.setTextSize(2);
      tft.printf("%03d", (int)get_gps_magtrack());
    }else if(is_headingupmode()){
      //heading up mode.
      tft.setCursor(SCREEN_WIDTH / 2 - 25, 8);
      tft.setTextColor(COLOR_GREEN);
      tft.setTextSize(1);
      tft.print("MH");  // Display as an integer
      tft.setCursor(SCREEN_WIDTH / 2 - 13, 0);
      tft.setTextSize(2);
      tft.printf("%03d",get_raw_magnetic_heading());
    }
  #endif

  tft.setTextSize(1);

  tft.setCursor(0, 0);
  tft.fillRect(0, 0, 39, 8, COLOR_BLACK);

  tft.setTextColor(COLOR_GREEN);
  // Convert speed from knots to m/s (1 knot = 0.514444 m/s) and display with one decimal place
  if(get_gps_fix()){
    double speed_m_s = get_gps_speed() * 0.514444;
    tft.print(speed_m_s, 1);
  }else{
    tft.print("N/A");
  }
  tft.println("m/s");
  
  tft.fillRect(0, 8, 36, 8, COLOR_BLACK);
  if(get_gps_numsat() < 5){
    tft.setTextColor(COLOR_YELLOW);
  }
  tft.print(get_gps_numsat());
  tft.println("sats");
  tft.setTextColor(COLOR_GREEN);
  
  tft.setCursor(0, 27);
  tft.fillRect(0, 27, 32, 8, COLOR_BLACK);
  double input_voltage = analogRead(A3)/1024.0*3.3*2;
  Serial.println(input_voltage);
  if(input_voltage < 3.75){
    tft.setTextColor(COLOR_YELLOW);
    tft.println("BATTERY LOW");
    tft.setTextColor(COLOR_GREEN);
  }else{
    tft.print(input_voltage,1);
    tft.println("V");
  }

  tft.setCursor(0, 35);
  tft.fillRect(0, 35, 35, 8, COLOR_BLACK);
  tft.print("RMH");
  tft.println(get_raw_magnetic_heading());

  #ifdef TFT_USE_ILI9341
    tft.fillRect(0, SCREEN_HEIGHT - 24, 48, 24, COLOR_BLACK);
    tft.fillRect(0, SCREEN_HEIGHT - 9, SCREEN_WIDTH, 9, COLOR_BLACK);
    tft.setTextSize(2);
    double dist = calculateDistance(get_gps_lat(),get_gps_long(),PLA_LAT,PLA_LON);
    tft.setCursor(0, SCREEN_HEIGHT - 24);
    tft.print(dist,1);
    tft.println("km");

    tft.setTextSize(1);
    tft.println(get_gps_lat(),6);
    tft.setCursor(SCREEN_WIDTH/2, SCREEN_HEIGHT - 8);
    tft.println(get_gps_long(),6);
  #else
    tft.fillRect(0, SCREEN_HEIGHT - 18, 24, 9, COLOR_BLACK);
    tft.fillRect(0, SCREEN_HEIGHT - 9, SCREEN_WIDTH, 9, COLOR_BLACK);


    tft.setTextSize(1);
    double dist = calculateDistance(get_gps_lat(),get_gps_long(),PLA_LAT,PLA_LON);
    tft.setCursor(0, SCREEN_HEIGHT - 16);
    tft.print(dist,1);
    tft.println("km");
    tft.println(get_gps_lat(),6);
    tft.setCursor(SCREEN_WIDTH/2, SCREEN_HEIGHT - 8);
    tft.println(get_gps_long(),6);
  #endif
}

void draw_headingupmode(){
  tft.setCursor(SCREEN_WIDTH/2-18, SCREEN_HEIGHT - 21);
  tft.setTextColor(COLOR_WHITE); // Highlight selected line
  tft.println("HDG UP");
}


void drawmap(stroke_group strokeid, float mapUpDirection, float center_lat, float center_lon, float mapScale, const mapdata* mp, uint16_t color) {
  int tstart_calc = millis();
  //float mapScale = 1.5;  // Example scale

  //cord_map mapcenter = xyToLatLon(SCREEN_WIDTH/2,SCREEN_HEIGHT/2,center_lat,center_lon,mapScale,mapUpDirection);
  int mapsize = mp->size;
  cord_tft points[mapsize];

  //bool drawbool[BIWAKO_DATA];
  if(!strokeManager.addStroke(strokeid, mapsize)){
    return;
  }
  for (int i = 0; i < mapsize; i++) {
    float lat1 = mp->cords[i][1];  // Example latitude
    float lon1 = mp->cords[i][0];  // Example longitude
                                   //drawbool[i] = check_maybe_inside_draw(mapcenter, lat1, lon1, mapScale);
                                   //if(drawbool[i]){
    points[i] = latLonToXY(lat1, lon1, center_lat, center_lon, mapScale, mapUpDirection, map_shift_down);
    strokeManager.addPointToStroke(points[i].x, points[i].y);

    //}else{
    //}
  }
  //Serial.print(millis() - tstart_calc);
  //Serial.println("ms to calculate map");
  //tft.startWrite();
  int tstartline = millis();

  for (int i = 0; i < mapsize - 1; i++) {
    //if(drawbool[i] && drawbool[i+1]){
    if (!points[i].isOutsideTft() || !points[i + 1].isOutsideTft()) {
      tft.drawLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, color);
      //strokeManager.addPointToStroke(points[i].x, points[i].y);
      //strokeManager.addPointToStroke(points[i+1].x, points[i+1].y);
    }

    //}
  }
  //Serial.print(millis() - tstartline);
  //Serial.println("ms to draw lines");
  //tft.endWrite();
  //Serial.print(millis() - tstart);
  //Serial.println("ms to draw complete map");
}


void calculatePointC(double lat1, double lon1, double lat2, double lon2, double distance, double &lat3, double &lon3) {
    const double R = 6371.0; // Radius of the Earth in kilometers
    lat1 = degreesToRadians(lat1);
    lon1 = degreesToRadians(lon1);
    lat2 = degreesToRadians(lat2);
    lon2 = degreesToRadians(lon2);

    double d = distance / R; // Distance in radians
    double bearing = atan2(sin(lon2 - lon1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1));

    lat3 = asin(sin(lat2) * cos(d) + cos(lat2) * sin(d) * cos(bearing));
    lon3 = lon2 + atan2(sin(bearing) * sin(d) * cos(lat2), cos(d) - sin(lat2) * sin(lat3));

    lat3 = rad2deg(lat3);
    lon3 = rad2deg(lon3);
}


void draw_pilon_takeshima_line(float mapcenter_lat, float mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);
  cord_tft n_pilon = latLonToXY(PILON_NORTH_LAT, PILON_NORTH_LON, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);
  cord_tft w_pilon = latLonToXY(PILON_WEST_LAT, PILON_WEST_LON, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);
  cord_tft takeshima = latLonToXY(TAKESHIMA_LAT, TAKESHIMA_LON, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);

  if(!strokeManager.addStroke(STRK_PILONLINE, 2)) return;
  tft.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, COLOR_MAGENTA);
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(n_pilon.x, n_pilon.y);


  if(!strokeManager.addStroke(STRK_PILONLINE, 2)) return;
  tft.drawLine(pla.x, pla.y, takeshima.x, takeshima.y, COLOR_MAGENTA);
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(takeshima.x, takeshima.y);


  if(!strokeManager.addStroke(STRK_PILONLINE, 2)) return;
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(w_pilon.x, w_pilon.y);
  tft.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, COLOR_MAGENTA);



  if(!strokeManager.addStroke(STRK_PILONLINE, 2)) return;

  double lat3,lon3;
  calculatePointC(PLA_LAT,PLA_LON,get_gps_lat(),get_gps_long(),5,lat3,lon3);
  cord_tft targetpoint = latLonToXY(lat3, lon3, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);

  tft.drawLine(pla.x, pla.y, targetpoint.x, targetpoint.y, COLOR_YELLOW);
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(targetpoint.x, targetpoint.y);
}


void fill_sea_land(float mapcenter_lat, float mapcenter_lon, float scale, float upward) {
  #ifdef XIAO_SAMD21
    //Due to memory shortage.
    return;
  #endif
  //fill
  for (int lat_i = 0; lat_i < ROW_FILLDATA; lat_i++) {
    float latitude = 35.0 + lat_i * 0.02;
    for (int lon_i = 0; lon_i < COL_FILLDATA; lon_i++) {
      float longitude = 135.8 + lon_i * 0.02;
      bool is_sea = filldata[lat_i][lon_i];
      int indexfillp = lat_i * COL_FILLDATA + lon_i;
      cord_tft pos = latLonToXY(latitude, longitude, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);
      if (pos.isOutsideTft()) {
        continue;
      } else {
        if (is_sea) {
          if (scale > 0.5) {
            int lenbar = 2;
            if(scale > 2) lenbar = 3;
            if(scale > 5) lenbar = 4;
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x, pos.y - lenbar);
            strokeManager.addPointToStroke(pos.x, pos.y + lenbar);
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x - lenbar, pos.y);
            strokeManager.addPointToStroke(pos.x + lenbar, pos.y);

            tft.drawFastVLine(pos.x, pos.y - lenbar, 1+lenbar*2, COLOR_LIGHT_BLUE);
            tft.drawFastHLine(pos.x - lenbar, pos.y, 1+lenbar*2, COLOR_LIGHT_BLUE);
          } else {
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x - 1, pos.y);
            strokeManager.addPointToStroke(pos.x + 1, pos.y);
            tft.drawFastHLine(pos.x - 1, pos.y, 3, COLOR_LIGHT_BLUE);
          }
        } else {
          if (scale > 0.5) {
            int lenbar = 2;
            if(scale > 2) lenbar = 3;
            if(scale > 5) lenbar = 4;
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x, pos.y - lenbar);
            strokeManager.addPointToStroke(pos.x, pos.y + lenbar);
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x - lenbar, pos.y);
            strokeManager.addPointToStroke(pos.x + lenbar, pos.y);

            tft.drawFastVLine(pos.x, pos.y - lenbar, 1+lenbar*2, COLOR_ORANGE);
            tft.drawFastHLine(pos.x - lenbar, pos.y, 1+lenbar*2, COLOR_ORANGE);
          } else {
            if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
            strokeManager.addPointToStroke(pos.x - 1, pos.y);
            strokeManager.addPointToStroke(pos.x + 1, pos.y);
            tft.drawFastHLine(pos.x - 1, pos.y, 3, COLOR_ORANGE);
          }
        }
      }
    }
  }
  
  if(scale > 2){
    // + マークのNavデータが登録されていない場所の+をfillする。 要するに拡大すると真っ暗にならないように対策。
    //Big zoom
    int latitude_index = int((mapcenter_lat-35.0)/0.02);
    int longitude_index = int((mapcenter_lon-135.8)/0.02);
    for(int x = latitude_index-3; x < latitude_index+3;x++){
      for(int y = longitude_index-3; y < longitude_index+3;y++){
        if(latitude_index >= 0 && latitude_index < ROW_FILLDATA-1 && longitude_index >= 0 && longitude_index < COL_FILLDATA-1){
          bool origin = filldata[x][y];
          if(origin != filldata[x+1][y] || origin != filldata[x][y+1] || origin != filldata[x+1][y+1]){
            //陸海の曖昧なエリアは + マークを追加しない。
            continue;
          }
          int dx_size = 2;
          int dy_size = 2;
          if(scale > 5){
            dx_size = 8;
            dy_size = 8;
          }
          for(int dx = 0; dx < dx_size; dx++){
            for(int dy= 0; dy < dy_size; dy++){
              if(dx == 0 && dy == 0){
                continue;
              }
              double lat_dx = 35.0+x*0.02+0.02*dx/dx_size;
              double lat_dy = 135.8+y*0.02+0.02*dy/dy_size;
              cord_tft dpos = latLonToXY(lat_dx, lat_dy, mapcenter_lat, mapcenter_lon, scale, upward, map_shift_down);
              int lenbar = 3;
              if (!dpos.isOutsideTft()) {
                if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
                strokeManager.addPointToStroke(dpos.x, dpos.y - lenbar);
                strokeManager.addPointToStroke(dpos.x, dpos.y + lenbar);
                if(!strokeManager.addStroke(STRK_SEALAND, 2)) return;
                strokeManager.addPointToStroke(dpos.x - lenbar, dpos.y);
                strokeManager.addPointToStroke(dpos.x + lenbar, dpos.y);
                if(origin){
                  tft.drawFastVLine(dpos.x, dpos.y - lenbar, 1+lenbar*2, COLOR_LIGHT_BLUE);
                  tft.drawFastHLine(dpos.x - lenbar, dpos.y, 1+lenbar*2, COLOR_LIGHT_BLUE);
                }else{
                  tft.drawFastVLine(dpos.x, dpos.y - lenbar, 1+lenbar*2, COLOR_ORANGE);
                  tft.drawFastHLine(dpos.x - lenbar, dpos.y, 1+lenbar*2, COLOR_ORANGE);
                }
              }
            }
          }
        }
      }
    }
  }
}


void tft_increment_brightness(){
  brightnessIndex = (brightnessIndex+1)% (sizeof(brightnessLevels)/sizeof(brightnessLevels[0]));
  screen_brightness = brightnessLevels[brightnessIndex];
  analogWrite(TFT_BL,255-screen_brightness);// For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
}

void draw_setting_mode(bool& redraw, int selectedLine, int cursorLine){
  if(redraw){
    tft.fillScreen(COLOR_BLACK);
    redraw = false;
  }

  tft.setTextColor(COLOR_WHITE);
  if(SCREEN_WIDTH <=128){
    tft.setTextSize(1);
  }else{
    tft.setTextSize(2);
  }
  // Line 1: Screen Brightness
  if (cursorLine == 0){
    if(selectedLine == 0){
      tft.setTextColor(COLOR_MAGENTA); // Highlight selected line
    }else{
      tft.setTextColor(COLOR_YELLOW); // Highlight selected line
    }
  }
  tft.setCursor(10, 30);
  tft.print(selectedLine == 0 ? " Brightness: ":"Brightness: ");
  tft.println(screen_brightness);
  tft.setTextColor(COLOR_WHITE);

  // Line 2: Placeholder for future settings
  if (cursorLine == 1){
    if(selectedLine == 1){
      tft.setTextColor(COLOR_MAGENTA); // Highlight selected line
    }else{
      tft.setTextColor(COLOR_YELLOW); // Highlight selected line
    }
  }
  tft.setCursor(10, 60);
  tft.print(selectedLine == 1 ? " DEMO BIWA: ":"DEMO BIWA: ");
  if(get_demo_biwako()){
    tft.println("YES");
  }else{
    tft.println("NO");
  }
  tft.setTextColor(COLOR_WHITE);

  // Line 3: Placeholder for future settings
  if (cursorLine == 2){
    if(selectedLine == 2){
      tft.setTextColor(COLOR_MAGENTA); // Highlight selected line
    }else{
      tft.setTextColor(COLOR_YELLOW); // Highlight selected line
    }
  }
  tft.setCursor(10, 90);
  tft.print(selectedLine == 2 ? " UPWARD: ":"UPWARD: ");
  if(is_trackupmode()){
    tft.println("TRACK UP");
  }else if(is_headingupmode()){
    tft.println("HEADING UP");
  }else if(is_northupmode()){
    tft.println("NORTH UP");
  }
  tft.setTextColor(COLOR_WHITE);

  // Line 4: Exit
  if (cursorLine == 3){
    if(selectedLine == 3){
      tft.setTextColor(COLOR_MAGENTA); // Highlight selected line
    }else{
      tft.setTextColor(COLOR_YELLOW); // Highlight selected line
    }
  }
  tft.setCursor(10, 120);
  tft.println("Exit");
  tft.setTextColor(COLOR_WHITE);
}