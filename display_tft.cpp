#include "settings.h"
#include "display_tft.h"
#include "ublox_gps.h"
#include "mysd.h"
#include "font_data.h"
#include "hardware/adc.h"
#include <cstring> // for strlen and strcpy
#include <string>
#include <cstdlib> // for malloc and free


#define AA_FONT_SMALL NotoSansBold15
#define NM_FONT_LARGE Arial_Black32


TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite needle = TFT_eSprite(&tft); // Sprite object for needle
TFT_eSprite needle_w = TFT_eSprite(&tft); // Sprite object for deleting needle with white


int screen_brightness = 255; // Example brightness value
const int brightnessLevels[] = {10, 100, 150, 200, 255}; // Example brightness levels
int brightnessIndex = 4; // Default to 60

#define MODE_TRACKUP 0
#define MODE_NORTHUP 1
#define MODE_SIZE 2
int upward_mode = MODE_NORTHUP;



#define RADIUS_EARTH_KM 6371.0 // Earth's radius in kilometers
#define MAX_STROKES_MAP 100
#define MAX_STROKES_SEALAND 1000




void toggle_mode(){
  upward_mode = (upward_mode +1)%MODE_SIZE;
}

bool is_northupmode(){
  return upward_mode == MODE_NORTHUP;
}
bool is_trackupmode(){
  return upward_mode == MODE_TRACKUP;
}


void createNeedle(void)
{
  needle.setColorDepth(8);
  needle_w.setColorDepth(8);
  needle.createSprite(3, 120); // create the needle Sprite 3x120
  needle_w.createSprite(3, 120);

  needle.fillSprite(TFT_BLACK);          // Fill with black
  //needle.drawFastVLine(0, 0, 120, COLOR_RED);
  needle.drawFastVLine(2, 0, 120, COLOR_RED);
  needle.fillRect(0, 5, 3, 9, COLOR_RED);
  needle_w.fillSprite(TFT_WHITE);

  // Define needle pivot point
  uint16_t piv_x = needle.width() / 2;   // x pivot of Sprite (middle)
  uint16_t piv_y = needle.height() - 2; // y pivot of Sprite (10 pixels from bottom)
  needle.setPivot(piv_x, piv_y);         // Set pivot point in this Sprite
  needle_w.setPivot(piv_x, piv_y);         // Set pivot point in this Sprite

  //needle.drawLine(0, 10, 0, 120, COLOR_GRAY);

  // Draw needle centre boss
  //needle.drawPixel( piv_x, piv_y, TFT_WHITE);     // Mark needle pivot point with a white pixel
}



void setup_tft() {
  gpio_init(27);
  gpio_set_dir(27, GPIO_OUT);
  gpio_put(27, 0);  // or gpio_put(28, 1);
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);
  gpio_put(28, 0);  // or gpio_put(28, 1);


  // Set GPIO26 as an ADC input and disable pull-up/pull-down resistors
  adc_gpio_init(BATTERY_PIN); // Initialize GPIO26 as ADC


  pinMode(BATTERY_PIN,INPUT);
  analogReadResolution(12);

  #ifdef BRIGHTNESS_SETTING_AVAIL
  // Initialize backlight control pin
  pinMode(TFT_BL, OUTPUT);
  analogWriteFreq(BL_PWM_FRQ); // 1000Hz
  analogWrite(TFT_BL,BRIGHTNESS(0));
  analogWrite(TFT_BL,BRIGHTNESS(screen_brightness));
  #endif

  tft.begin();


  tft.setRotation(2);//set 0 for newhaven
  tft.loadFont(AA_FONT_SMALL);    // Must load the font first
  //tft.setTextFont(2);


  tft.fillScreen(COLOR_WHITE);
  createNeedle();
}




struct line {
  int x1, y1, x2, y2;
};


class Point {
public:
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
};

class Text {
private:
    int id;
    int size;
    Point cord;
    char* textchar;

public:
    Text() : id(0), size(0), cord(0, 0), textchar(NULL) {}

    Text(int id, int size, int x, int y, const char* text_in) : id(id), size(size), cord(x, y), textchar(NULL) {
        setText(text_in);
    }

    ~Text() {
        free(textchar);
    }

    // Copy constructor
    Text(const Text& other) : id(other.id), size(other.size), cord(other.cord), textchar(NULL) {
        setText(other.textchar);
    }

    // Assignment operator
    Text& operator=(const Text& other) {
        if (this != &other) {
            id = other.id;
            size = other.size;
            cord = other.cord;
            setText(other.textchar);
        }
        return *this;
    }

    bool setText(const char* text_in) {
        if (textchar != NULL) {
            free(textchar);
        }
        size_t length = strlen(text_in);
        textchar = (char*)malloc((length + 1) * sizeof(char));
        if (textchar == NULL) {
            return false; // Memory allocation failed
        }
        strcpy(textchar, text_in);
        return true;
    }

    int getId() const { return id; }
    int getSize() const { return size; }
    Point getCord() const { return cord; }
    const char* getTextChar() const { return textchar; }

    friend class TextManager;
};

class TextManager {
private:
    Text** draw_texts; // Array of pointers to Text objects
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
    TextManager(int maxTexts) : textCount(0), maxtext(maxTexts) {
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
          foundText->setText(text_in); // Update textchar
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
      tft.setTextColor(col,COLOR_WHITE);
      tft.setTextSize(foundText->size);
      tft.print(foundText->getTextChar());
      return true;
  }


    bool drawTextf(int id, int size, int x, int y, uint16_t col, const char* format, ...) {
        char buffer[256]; // Temporary buffer for formatted text

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

  bool addStroke(stroke_group id, int maxPoints,int thickness = 1) {
    if (strokeCount >= maxStrokes) {
      DEBUG_P(20240827,id);
      DEBUG_PLN(20240827,"ERR max stroke reached");
      enqueueTask(createLogSdfTask("ERR max stroke reached(id,max) %d,%d",id,maxPoints));


      return false;  // No more space for new strokes
    }
    strokes[strokeCount].points = (Point*)malloc(maxPoints * sizeof(Point));
    if (strokes[strokeCount].points == nullptr) {
      DEBUG_P(20240827,id);
      DEBUG_PLN(20240827,"malloc fail");
      enqueueTask(createLogSdfTask("malloc failed:adding stroke:ID=%d",id));
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


  void drawAllStrokes() {
    for (int i = 0; i < strokeCount; i++) {
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        if(strokes[i].thickness == 1)
          tft.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, COLOR_WHITE);
        else
          drawThickLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, strokes[i].thickness, COLOR_WHITE);
      }
    }
  }
};



StrokeManager mapStrokeManager(MAX_STROKES_MAP);
StrokeManager sealandStrokeManager(MAX_STROKES_SEALAND);


// Convert degrees to radians
float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}


double rad2deg(double rad) {
    return rad * (180.0 / PI);
}



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
  int radius = scale * km;//scale is px/km
  int ypos = SCREEN_HEIGHT / 2 - radius;
  if (ypos > 30) {
    tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, radius, COLOR_PINK);
    tft.setCursor(SCREEN_WIDTH / 2 + 2, ypos + 1);
    tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
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
    tft.drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, color);
  }
}

// draw magnetic north east west south according to truetrack, where truetrack is direction of upward display.
// However direction of y-axis is downward for tft display.
void draw_compass(float truetrack, uint16_t col) {
  tft.setTextWrap(false);
  int dist = 73;
  if(is_northupmode()){
    #ifdef TFT_USE_ST7735
      dist = 50;
    #else
      dist = 100;
    #endif
  }
  int centerx = SCREEN_WIDTH / 2;
  int centery = SCREEN_HEIGHT / 2;
  //Varietion 8 degrees.
  float radian = degreesToRadians(truetrack+8.0);
  float radian45offset = degreesToRadians(truetrack+8.0 + 45);
  tft.setTextColor(col,COLOR_WHITE);
  tft.setTextSize(2);

  cord_tft n = { int(centerx + sin(-radian) * dist), int(centery - cos(radian) * dist) };
  cord_tft e = { int(centerx + cos(radian) * dist), int(centery - sin(radian) * dist) };
  cord_tft s = { int(centerx + sin(radian) * dist), int(centery + cos(radian) * dist) };
  cord_tft w = { int(centerx - cos(-radian) * dist), int(centery - sin(-radian) * dist) };
  if (!n.isOutsideTft()) {
    tft.setCursor(n.x-5, n.y-5);
    tft.print("N");
  }
  if (!e.isOutsideTft()) {
    tft.setCursor(e.x-5, e.y-5);
    tft.print("E");
  }
  if (!s.isOutsideTft()) {
    tft.setCursor(s.x-5, s.y-5);
    tft.print("S");
  }
  if (!w.isOutsideTft()) {
    tft.setCursor(w.x-5, w.y-5);
    tft.print("W");
  }

  cord_tft nw = { int(centerx + sin(-radian45offset) * dist), int(centery - cos(radian45offset) * dist) };
  cord_tft ne = { int(centerx + cos(radian45offset) * dist), int(centery - sin(radian45offset) * dist) };
  cord_tft se = { int(centerx + sin(radian45offset) * dist), int(centery + cos(radian45offset) * dist) };
  cord_tft sw = { int(centerx - cos(-radian45offset) * dist), int(centery - sin(-radian45offset) * dist) };

  tft.setTextSize(1);
  if (!nw.isOutsideTft()) {
    tft.setCursor(nw.x-5, nw.y-2);
    tft.print("NW");
  }
  if (!ne.isOutsideTft()) {
    tft.setCursor(ne.x-5, ne.y-2);
    tft.print("NE");
  }
  if (!se.isOutsideTft()) {
    tft.setCursor(se.x-5, se.y-2);
    tft.print("SE");
  }
  if (!sw.isOutsideTft()) {
    tft.setCursor(sw.x-5, sw.y-2);
    tft.print("SW");
  }
  tft.setTextWrap(true);
}





void draw_km_circle(float scale) {
  int draw_counter = 0;

  if(draw_circle_km(scale, 400)) draw_counter++;
  if(draw_circle_km(scale, 200)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 100)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 50)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 20)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 10)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 5)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 2)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 1)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 0.4)) draw_counter++;
  if(draw_counter >= 2) return;
  if(draw_circle_km(scale, 0.2)) draw_counter++;
  if(draw_counter >= 2) return;
}

bool fresh = false;
float last_scale = 1.0;
float last_up = 0;
float last_truetrack = 0;
bool nomap_drawn = true;

void redraw_compass(float up,int forecolor,int bgcolor){
  draw_compass(last_truetrack, bgcolor);
  draw_compass(up, forecolor);
  last_truetrack = up;
}

void clean_display(){
  tft.fillScreen(COLOR_WHITE);
  mapStrokeManager.removeAllStrokes();
  nomap_drawn = true;
}

void clean_map(){
  mapStrokeManager.drawAllStrokes();
  mapStrokeManager.removeAllStrokes();
  nomap_drawn = true;
}




int rb_x_old,rb_y_old,lb_x_old,lb_y_old;
#ifdef TFT_USE_ST7735
  #define TRIANGLE_HWIDTH 4
  #define TRIANGLE_SIZE 12
#else
  #define TRIANGLE_HWIDTH 6
  #define TRIANGLE_SIZE 20
#endif


int oldmagtrack = 0;

void erase_triangle(){
  if(upward_mode == MODE_NORTHUP){
    tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
    needle_w.pushRotated(oldmagtrack);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, SCREEN_WIDTH / 2 + rb_x_old, SCREEN_HEIGHT / 2 + rb_y_old, SCREEN_WIDTH / 2 + lb_x_old, SCREEN_HEIGHT / 2 + lb_y_old, COLOR_WHITE);
  }
}

void draw_triangle(){
  if(upward_mode == MODE_NORTHUP){
    int magtrack = get_gps_truetrack();
    float radians = degreesToRadians(magtrack);
    int x_track =  SCREEN_HEIGHT * sin(radians);
    int y_track =  SCREEN_HEIGHT * -cos(radians);

    tft.setPivot(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
    needle.pushRotated(magtrack);
    oldmagtrack = magtrack;

    int rb_x_new = -TRIANGLE_HWIDTH * cos(radians) - TRIANGLE_SIZE * sin(radians);
    int rb_y_new = -TRIANGLE_HWIDTH * sin(radians) + TRIANGLE_SIZE * cos(radians);
    int lb_x_new = TRIANGLE_HWIDTH * cos(radians) - TRIANGLE_SIZE * sin(radians);
    int lb_y_new = TRIANGLE_HWIDTH * sin(radians) + TRIANGLE_SIZE * cos(radians);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, SCREEN_WIDTH / 2 + rb_x_new, SCREEN_HEIGHT / 2 + rb_y_new, SCREEN_WIDTH / 2 + lb_x_new, SCREEN_HEIGHT / 2 + lb_y_new, COLOR_BLACK);
    rb_x_old = rb_x_new;
    rb_y_old = rb_y_new;
    lb_x_old = lb_x_new;
    lb_y_old = lb_y_new;
  }
  if(upward_mode == MODE_TRACKUP){
    int shortening = 30;
    tft.drawFastVLine(SCREEN_WIDTH / 2, shortening, SCREEN_HEIGHT / 2 -shortening, COLOR_BLACK);
    tft.drawFastVLine(SCREEN_WIDTH / 2+1, shortening, SCREEN_HEIGHT / 2 -shortening, COLOR_BLACK);
    tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 , SCREEN_WIDTH / 2 - TRIANGLE_HWIDTH, SCREEN_HEIGHT / 2 + TRIANGLE_SIZE, SCREEN_WIDTH / 2 + TRIANGLE_HWIDTH, SCREEN_HEIGHT / 2 + TRIANGLE_SIZE, COLOR_BLACK);
  }
}



// point A -> B -(distance)> C
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

// Point A -(distance)> D -> B
void calculatePointD(double lat1, double lon1, double lat2, double lon2, double distance, double &lat3, double &lon3) {
  const double R = 6371.0; // Radius of the Earth in kilometers
  lat1 = degreesToRadians(lat1);
  lon1 = degreesToRadians(lon1);
  lat2 = degreesToRadians(lat2);
  lon2 = degreesToRadians(lon2);

  double d = distance / R; // Distance in radians
  double deltaLon = lon2 - lon1;
  double bearing;

  if (lat1 == lat2 && lon1 == lon2) {
      Serial.println("Error: Points A and B are the same. Bearing is undefined.");
      return;
  }

  bearing = atan2(sin(deltaLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon));

  lat3 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bearing));
  lon3 = lon1 + atan2(sin(bearing) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat3));


  lat3 = rad2deg(lat3);
  lon3 = rad2deg(lon3);
}

void draw_flyawayfrom(double center_lat,double center_lon,float scale, float up){
  int thickness = 2;
  double lat3,lon3;
  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, center_lat, center_lon, scale, up);
  calculatePointC(PLA_LAT,PLA_LON,center_lat,center_lon,25,lat3,lon3);
  cord_tft targetpoint = latLonToXY(lat3, lon3, center_lat, center_lon, scale, up);
  mapStrokeManager.addStroke(STRK_TARGETLINE, 2);
  mapStrokeManager.addPointToStroke(pla.x, pla.y);
  mapStrokeManager.addPointToStroke(targetpoint.x, targetpoint.y);
  drawThickLine(pla.x,pla.y,targetpoint.x,targetpoint.y, thickness, COLOR_MAGENTA);  
}

void draw_flyinto(double center_lat,double center_lon,float scale, float up){
  int thickness = 2;
  double target_lat = PLA_LAT;
  double target_lon = PLA_LON;
  cord_tft goal = latLonToXY(target_lat, target_lon, center_lat, center_lon, scale, up);
  if(goal.isOutsideTft()){
    //scaleがとても大きい場合、goal.x,goal.yがオーバーフローする。
    //そのまま描画すると処理落ちするので、適度な座標を計算し直す必要がある。
    double distance = 200/scale;//画面外に出ればよいので適当な距離を設定。
    double newlat,newlon;
    calculatePointD(center_lat,center_lon,target_lat,target_lon,distance,newlat,newlon);
    goal = latLonToXY(newlat, newlon, center_lat, center_lon, scale, up);
  }
  mapStrokeManager.addStroke(STRK_TARGETLINE, 2, thickness);
  mapStrokeManager.addPointToStroke(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
  mapStrokeManager.addPointToStroke(goal.x, goal.y);
  drawThickLine(SCREEN_WIDTH/2, SCREEN_HEIGHT/2,goal.x,goal.y, thickness, COLOR_MAGENTA);  
}

void draw_track(double center_lat,double center_lon,float scale, float up){
  int sizetrack = latlon_manager.getCount();
  cord_tft points[sizetrack];
  int thickness = 2;
  mapStrokeManager.addStroke(STRK_TRACK, sizetrack,thickness);
  int old_x = 0;
  int old_y = 0;
  for(int i = 0;i  < sizetrack;i++){
    Coordinate tempc = latlon_manager.getData(i);
    points[i] = latLonToXY(tempc.latitude,tempc.longitude,center_lat,center_lon,scale,up);
    //Only if cordinates are different.
    if(i < 2 || old_x !=  points[i].x || old_y !=  points[i].y){
      old_x = points[i].x;
      old_y = points[i].y;
      mapStrokeManager.addPointToStroke(points[i].x,points[i].y);
    }
  }
  for (int i = 0; i < sizetrack - 1; i++) {
    if (!points[i].isOutsideTft() || !points[i + 1].isOutsideTft()) {
      //tft.drawLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, COLOR_GREEN);
      drawThickLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y,thickness, COLOR_GREEN);
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
  for(int i = 0; i < mapdata_count;i++){
    if(extramaps[i].size <= 1){
      continue;
    }
    double lon1 = extramaps[i].cords[0][0];
    double lat1 = extramaps[i].cords[0][1];
    if(check_within_latlon(1,1,lat1,get_gps_lat(),lon1,get_gps_long())){
      int col = COLOR_GREEN;
      char name_firstchar = extramaps[i].name[0];
      if(name_firstchar == 'r'){
        col = COLOR_RED;
      }else if(name_firstchar == 'o'){
        col = COLOR_ORANGE;
      }else if(name_firstchar == 'g'){
        col = COLOR_BRIGHTGRAY;
      }else if(name_firstchar == 'm'){
        col = COLOR_MAGENTA;
      }else if(name_firstchar == 'c'){
        col = COLOR_CYAN;
      }else if(name_firstchar == 'b'){
        col = COLOR_BLUE;
      }
      draw_map(STRK_MAP1,up,center_lat,center_lon,scale,&extramaps[i],col);
    }

  }
}

void draw_Shinura(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_shinura, COLOR_GREEN);
  
  nomap_drawn = false;
}

void draw_Biwako(double center_lat, double center_lon, float scale, float up) {
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_biwako, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_takeshima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_chikubushima, COLOR_GREEN);
  draw_map(STRK_MAP1, up, center_lat, center_lon, scale, &map_okishima, COLOR_GREEN);
  draw_pilon_takeshima_line(center_lat, center_lon, scale, up);
  if(!gmap_loaded)
    fill_sea_land(center_lat, center_lon, scale, up);
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


void startup_demo_tft() {
  float scale = 3.0;
  float center_lat = 35.2334225841915;
  float center_lon = 136.091056306493;
  bool redraw = false;
  //draw_degpersecond(30.0 / (300.0 / 1000));
  //draw_gpsinfo();
  for(int i = 0; i < 10; i++){
    clean_map();
    sealandStrokeManager.drawAllStrokes();
    sealandStrokeManager.removeAllStrokes();
    draw_Biwako(center_lat, center_lon, scale-i*0.19, 0);

    tft.setTextColor(COLOR_RED,COLOR_WHITE);
    tft.setCursor(5, 5);
    tft.println(" Piolot Oriented");
    tft.println("   Navigation System   for HPA");
    tft.println("========== PONS ==========");


    tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
    tft.setCursor(1, SCREEN_HEIGHT / 2 + 110);
    tft.print("SD MAP COUNT:  ");
    tft.print(mapdata_count);
    tft.setCursor(1, SCREEN_HEIGHT / 2 + 135);
    tft.print("SOFT VERSION:");
    tft.print(BUILDVERSION);
    tft.print("(b");
    tft.print(BUILDDATE);
    tft.print(")");


    for(int j = 0; j < 250; j++){
      gps_loop();
      delay(1);
    }
  }
}

void drawbar(float degpersecond, int col){
  if(col == -1){
    if (degpersecond > 0) {
      col = COLOR_GREEN;
    }else{
      col = COLOR_LIGHT_BLUE;
    }
  }
  int barposy = 28;
  float abs_degpersecond = abs(degpersecond);
  int absint_degpersecond = abs_degpersecond;
  int barwidth = (abs_degpersecond * (SCREEN_WIDTH/2-20)) / 3.0;//3deg = max width
  int thickness = constrain(1+absint_degpersecond*absint_degpersecond,2,13);
  if (degpersecond > 0) {
    for(int i = 0;i < thickness; i++){
      tft.drawFastHLine(SCREEN_WIDTH / 2+20, barposy+i, barwidth, col);
    }
  } else {
    for(int i = 0;i < thickness; i++){
      tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth-20, barposy+i, barwidth, col);
    }
  }
}

float last_degpersecond;

void draw_degpersecond(float degpersecond) {
  int col = COLOR_GRAY;
  if (degpersecond > 0) {
    col = COLOR_GREEN;
  }
  if (degpersecond < 0) {
    col = COLOR_BLUE;
  }
  drawbar(last_degpersecond,COLOR_WHITE);
  drawbar(degpersecond,-1);
  last_degpersecond = degpersecond;

  textmanager.drawTextf(ND_DEGPERSEC_VAL,1,SCREEN_WIDTH/2 - 20, 27,COLOR_BLACK,"%.1f",degpersecond);
  textmanager.drawText(ND_DEGPERSEC_TEX,1,SCREEN_WIDTH -100, 27,col,"deg/s");
}

bool bankwarning_flipflop = true;
void draw_bankwarning() {
  int starty = 46;
  bankwarning_flipflop = !bankwarning_flipflop;
  int bgcolor = COLOR_BLACK;
  int tcolor = COLOR_WHITE;
  if(bankwarning_flipflop){
    bgcolor = COLOR_WHITE;
    tcolor = COLOR_BLACK;
  }
  tft.fillRect(0, starty, SCREEN_WIDTH, 36, bgcolor);
  int x = SCREEN_WIDTH / 2 - 40;
  tft.setCursor(x, starty+2);
  tft.setTextColor(COLOR_RED,bgcolor);
  tft.setTextSize(4);
  tft.print("!  BANK  !");
  tft.setCursor(x - 1, starty+4);
  tft.setTextColor(tcolor,bgcolor);
  tft.print("!  BANK  !");
  tft.setTextSize(1);
}

void draw_nogmap(){
  tft.setTextColor(COLOR_ORANGE,COLOR_WHITE);
  tft.setCursor(1, SCREEN_HEIGHT-58);
  tft.print("No map image available.");
}

void draw_sdinfo(){
  if(good_sd()){
    tft.fillRect(SCREEN_WIDTH-22, SCREEN_HEIGHT-15, 22, 15, COLOR_GREEN);
  }else{
    tft.fillRect(SCREEN_WIDTH-22, SCREEN_HEIGHT-15, 22, 15, COLOR_RED);
  }
  tft.setTextColor(COLOR_WHITE,COLOR_GREEN);
  tft.setCursor(SCREEN_WIDTH-20, SCREEN_HEIGHT-14);
  tft.print("SD");
}

const int numPoints = 14;
const int adReadings[numPoints] = {1600, 1584, 1569, 1549, 1535, 1525, 1508, 1490, 1473, 1295, 995, 746, 555, 473};
const double voltages[numPoints] = {4.25, 4.15, 4.05, 3.95, 3.85, 3.75, 3.65, 3.55, 3.45, 3.35, 3.25, 3.15, 3.05, 2.95};
double volts_interpolate(int adreading) {
    // Handle readings above the highest data point
    if (adreading >= adReadings[0]) {
        return voltages[0] + (adreading - adReadings[0]) * (voltages[0] - voltages[1]) / (adReadings[0] - adReadings[1]);
    }    
    // Handle readings below the lowest data point
    if (adreading <= adReadings[numPoints - 1]) {
        return voltages[numPoints - 1] + (adreading - adReadings[numPoints - 1]) * (voltages[numPoints - 1] - voltages[numPoints - 2]) / (adReadings[numPoints - 1] - adReadings[numPoints - 2]);
    }
    // Handle readings within the data range using linear interpolation
    for (int i = 0; i < numPoints - 1; i++) {
        if (adreading >= adReadings[i + 1] && adreading <= adReadings[i]) {
            return voltages[i] + (adreading - adReadings[i]) * (voltages[i + 1] - voltages[i]) / (adReadings[i + 1] - adReadings[i]);
        }
    }
    // If something goes wrong, return a default value
    return 0.0;
}


unsigned long last_loading_image = 0;
void draw_loading_image(){
  if(millis()-last_loading_image > 15){//50Hz
    last_loading_image = millis();

    int x = (millis()/20)%SCREEN_WIDTH;
    int color = (TASK_LOAD_MAPIMAGE == currentTaskType)?COLOR_ORANGE:COLOR_GRAY;
    if(currentTaskType == TASK_LOG_SD || currentTaskType == TASK_LOG_SDF || currentTaskType == TASK_SAVE_CSV){
      color = COLOR_RED;
    }

    if(millis() - time_lastnmea < 15){
      color = COLOR_BLUE;
    }
    tft.drawFastVLine(x, SCREEN_HEIGHT/2+240/2, 4, color);
    tft.drawFastVLine((x+1)%SCREEN_WIDTH, SCREEN_HEIGHT/2+240/2, 4, COLOR_WHITE);
    tft.drawFastVLine((x+2)%SCREEN_WIDTH, SCREEN_HEIGHT/2+240/2, 4, COLOR_WHITE);
  }
}

int last_written_mh = 0;
unsigned long last_maxadr_time = 0;
int max_adreading = 0;
unsigned long last_bigvolarity_time = 0;

void draw_gpsinfo() {
  int col = COLOR_BLACK;
  const int mtlx = SCREEN_WIDTH - 100;
  const int mtvx = mtlx+25;
  tft.setTextColor(TFT_BLACK, TFT_WHITE, true);
  tft.drawString("MT",mtlx, 1);
  tft.drawString("m/s",75, 13);

  tft.loadFont(NM_FONT_LARGE);    // Must load the font first
  textmanager.drawTextf(ND_MT,1,mtvx, 1,col,"%03d", (int)get_gps_magtrack());

  textmanager.drawTextf(ND_MPS,1,1,1,COLOR_BLACK,"%4.1f",get_gps_mps());
  tft.loadFont(AA_FONT_SMALL);    // Must load the font first

  col = COLOR_GREEN;
  if(get_gps_numsat() < 5){
    col = COLOR_RED;
  }else if(get_gps_numsat() < 10){
    col = COLOR_DARKORANGE;
  }
  textmanager.drawTextf(ND_SATS,1,SCREEN_WIDTH/2,SCREEN_HEIGHT-28,col,"%dsats",get_gps_numsat());
  
  tft.setCursor(0, 27);

  
  int adreading = analogRead(BATTERY_PIN);
  if(abs(max_adreading-adreading) > 100 && max_adreading != 0){
    //100以上ずれたら異常=mcp73831が充電池がつながっているかどうかを検出しようとしている＝電池がつながっていない。
    last_bigvolarity_time = millis();
  }

  if(adreading > max_adreading){
    max_adreading = adreading;
    last_maxadr_time = millis();
  }else if(millis() - last_maxadr_time > 3000){
    max_adreading += (adreading-max_adreading)*0.4;
  }

  double input_voltage = BATTERY_MULTIPLYER(max_adreading);

  
  if(millis() - last_bigvolarity_time < 5000){
    textmanager.drawText(ND_BATTERY,1,SCREEN_WIDTH - 70, SCREEN_HEIGHT-28,COLOR_RED,"BAT_OFF");
  }else{
    if(input_voltage > 4.25){
      input_voltage = 4.25;
      textmanager.drawText(ND_BATTERY,1,SCREEN_WIDTH - 70, SCREEN_HEIGHT-28,COLOR_GREEN,"CHARGE");
    }
    else if(input_voltage < BAT_LOW_VOLTAGE){
      textmanager.drawText(ND_BATTERY,1,SCREEN_WIDTH - 70, SCREEN_HEIGHT-28,COLOR_RED,"BAT_LOW");
    }else if(input_voltage < 3.75){
      textmanager.drawTextf(ND_BATTERY,1,SCREEN_WIDTH - 45, SCREEN_HEIGHT-28,COLOR_MAGENTA,"%.2fV",input_voltage);
    }else{
      textmanager.drawTextf(ND_BATTERY,1,SCREEN_WIDTH - 45, SCREEN_HEIGHT-28,COLOR_GREEN,"%.2fV",input_voltage);
    }
  }

  // Distance to plathome.
  double dist = calculateDistance(get_gps_lat(),get_gps_long(),PLA_LAT,PLA_LON);
  textmanager.drawTextf(ND_DIST_PLAT,2,0,SCREEN_HEIGHT - 30,COLOR_BLACK,"%.1fkm", dist);

  // 5 decimal places latitude, longitude print.
  textmanager.drawTextf(ND_LAT,1,1,SCREEN_HEIGHT - 14,COLOR_BLACK,"%.6f", get_gps_lat());
  textmanager.drawTextf(ND_LON,1,SCREEN_WIDTH/2,SCREEN_HEIGHT - 14,COLOR_BLACK,"%.6f", get_gps_long());
}

void draw_headingupmode(){
  tft.setCursor(SCREEN_WIDTH/2-18, SCREEN_HEIGHT - 21);
  tft.setTextColor(COLOR_BLACK); // Highlight selected line
  tft.println("HDG UP");
}


void draw_map(stroke_group strokeid, float mapUpDirection, double center_lat, double center_lon, float mapScale, const mapdata* mp, uint16_t color) {
  int tstart_calc = millis();
  int mapsize = mp->size;
  cord_tft points[mapsize];

  //bool drawbool[BIWAKO_DATA];
  if(!mapStrokeManager.addStroke(strokeid, mapsize)){
    return;
  }
  for (int i = 0; i < mapsize; i++) {
    float lat1 = mp->cords[i][1];  // Example latitude
    float lon1 = mp->cords[i][0];  // Example longitude
                                   //drawbool[i] = check_maybe_inside_draw(mapcenter, lat1, lon1, mapScale);
                                   //if(drawbool[i]){
    points[i] = latLonToXY(lat1, lon1, center_lat, center_lon, mapScale, mapUpDirection);
    mapStrokeManager.addPointToStroke(points[i].x, points[i].y);
  }
  int tstartline = millis();

  for (int i = 0; i < mapsize - 1; i++) {
    if (!points[i].isOutsideTft() || !points[i + 1].isOutsideTft()) {
      tft.drawLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, color);
    }
  }
}




void draw_pilon_takeshima_line(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(PILON_NORTH_LAT, PILON_NORTH_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft w_pilon = latLonToXY(PILON_WEST_LAT, PILON_WEST_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft takeshima = latLonToXY(TAKESHIMA_LAT, TAKESHIMA_LON, mapcenter_lat, mapcenter_lon, scale, upward);

  if(!mapStrokeManager.addStroke(STRK_PILONLINE, 2)) return;
  tft.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, COLOR_GREEN);
  mapStrokeManager.addPointToStroke(pla.x, pla.y);
  mapStrokeManager.addPointToStroke(n_pilon.x, n_pilon.y);


  if(!mapStrokeManager.addStroke(STRK_PILONLINE, 2)) return;
  tft.drawLine(pla.x, pla.y, takeshima.x, takeshima.y, COLOR_GREEN);
  mapStrokeManager.addPointToStroke(pla.x, pla.y);  
  mapStrokeManager.addPointToStroke(takeshima.x, takeshima.y);


  if(!mapStrokeManager.addStroke(STRK_PILONLINE, 2)) return;
  mapStrokeManager.addPointToStroke(pla.x, pla.y);
  mapStrokeManager.addPointToStroke(w_pilon.x, w_pilon.y);
  tft.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, COLOR_GREEN);
}


void fill_sea_land(double mapcenter_lat, double mapcenter_lon, float scale, float upward) {
  sealandStrokeManager.drawAllStrokes();
  sealandStrokeManager.removeAllStrokes();

  int lenbar = 5;
  if(scale > 1.8) lenbar = 7;
  if(scale > 7.2) lenbar = 15;
  if(scale > 36) lenbar = 20;
  if(scale > 90) lenbar = 24;
      
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
        int col = is_sea? COLOR_LIGHT_BLUE : COLOR_ORANGE;
        if(!sealandStrokeManager.addStroke(STRK_SEALAND, 2)) return;
        sealandStrokeManager.addPointToStroke(pos.x - lenbar, pos.y);
        sealandStrokeManager.addPointToStroke(pos.x + lenbar, pos.y);
        tft.drawFastHLine(pos.x - lenbar, pos.y, 1+lenbar*2, col);

        if (scale > 9) {
          if(!sealandStrokeManager.addStroke(STRK_SEALAND, 2)) return;
          sealandStrokeManager.addPointToStroke(pos.x, pos.y - lenbar);
          sealandStrokeManager.addPointToStroke(pos.x, pos.y + lenbar);
          tft.drawFastVLine(pos.x, pos.y-lenbar, 1+lenbar*2, col);
          //sealandStrokeManager.addPointToStroke(pos.x - lenbar, pos.y+20 );
          //sealandStrokeManager.addPointToStroke(pos.x + lenbar, pos.y+20 );
          //tft.drawFastHLine(pos.x - lenbar, pos.y+20, 1+lenbar*2, col);
        }
      }
    }
  }
  
  if(scale > 36){
    // + マークのNavデータが登録されていない場所の+をfillする。 要するに拡大すると真っ暗にならないように対策。
    //Big zoom
    int latitude_index = int((mapcenter_lat-35.0)/0.02);
    int longitude_index = int((mapcenter_lon-135.8)/0.02);

    for(int x = latitude_index-3; x < latitude_index+3;x++){
      for(int y = longitude_index-3; y < longitude_index+3;y++){
        if(latitude_index >= 0 && latitude_index < ROW_FILLDATA-1 && longitude_index >= 0 && longitude_index < COL_FILLDATA-1){
          bool origin_sealand = filldata[x][y];
          if(origin_sealand != filldata[x+1][y] || origin_sealand != filldata[x][y+1] || origin_sealand != filldata[x+1][y+1]){
            //陸海の曖昧なエリアは + マークを追加しない。
            continue;
          }
          int dx_size = 2;
          int dy_size = 2;
          if(scale > 90){
            dx_size = 8;
            dy_size = 8;
          }
          int col = origin_sealand?COLOR_LIGHT_BLUE:COLOR_ORANGE;
          for(int dx = 0; dx < dx_size; dx++){
            for(int dy= 0; dy < dy_size; dy++){
              if(dx == 0 && dy == 0){
                continue;
              }
              double lat_dx = 35.0+x*0.02+0.02*dx/dx_size;
              double lat_dy = 135.8+y*0.02+0.02*dy/dy_size;
              cord_tft dpos = latLonToXY(lat_dx, lat_dy, mapcenter_lat, mapcenter_lon, scale, upward);
              if (!dpos.isOutsideTft()) {
                if(!sealandStrokeManager.addStroke(STRK_SEALAND, 2)) return;
                sealandStrokeManager.addPointToStroke(dpos.x, dpos.y - lenbar);
                sealandStrokeManager.addPointToStroke(dpos.x, dpos.y + lenbar);
                //sealandStrokeManager.addPointToStroke(dpos.x - lenbar, dpos.y);
                //sealandStrokeManager.addPointToStroke(dpos.x + lenbar, dpos.y);
                if(!sealandStrokeManager.addStroke(STRK_SEALAND, 2)) return;
                sealandStrokeManager.addPointToStroke(dpos.x - lenbar, dpos.y);
                sealandStrokeManager.addPointToStroke(dpos.x + lenbar, dpos.y);

                tft.drawFastVLine(dpos.x, dpos.y - lenbar, 1+lenbar*2, col);
                tft.drawFastHLine(dpos.x - lenbar, dpos.y, 1+lenbar*2, col);
                //tft.drawFastHLine(dpos.x - lenbar, dpos.y+20, 1+lenbar*2, col);
              }
            }
          }
        }
      }
    }
  }
}

int mod( int x, int y ){
  return x<0 ? ((x+1)%y)+y-1 : x%y;
}

void tft_change_brightness(int increment){
  #ifdef BRIGHTNESS_SETTING_AVAIL
  brightnessIndex = mod(brightnessIndex+increment,sizeof(brightnessLevels)/sizeof(brightnessLevels[0]));
  screen_brightness = brightnessLevels[brightnessIndex];
  analogWrite(TFT_BL,BRIGHTNESS(screen_brightness));// For PNP transistor. 255= No backlight, 0=always on. Around 200 should be enough for lighting TFT.
  #endif
}


int calculateGPS_X(float azimuth, float elevation) {
  const int shift_left = 10;
  const int radius = SCREEN_WIDTH/2 - shift_left;

  return (SCREEN_WIDTH / 2) + (int)(cos(radians(azimuth)) * (radius) * (1 - elevation / 90.0))-shift_left;
}

int calculateGPS_Y(float azimuth, float elevation) {
  const int height = SCREEN_HEIGHT;
  const int radius = SCREEN_WIDTH/2-20;
  const int shift_down = 0;
  return (height / 2) - (int)(sin(radians(azimuth)) * (radius) * (1 - elevation / 90.0))+shift_down;
}


unsigned long lastdrawtime_nomapdata = 0;
void draw_nomapdata(){
  int posy = SCREEN_HEIGHT-130;
  if(!nomap_drawn && get_gps_fix()){
    //Remove old draw.
    textmanager.drawText(ND_GPSCOND,1,0,0,COLOR_WHITE,"");
    textmanager.drawText(ND_SEARCHING,2,3, posy+15,COLOR_WHITE,"");
    textmanager.drawText(ND_GPSDOTS,2,3, posy+30,COLOR_WHITE,"");
    return;
  }
  //Map is not written or gps_fix not obtained.
  if(millis() - lastdrawtime_nomapdata > 500){
    lastdrawtime_nomapdata = millis();
    if(get_gps_connection()){
      textmanager.drawText(ND_GPSCOND,1,3, posy,COLOR_GREEN,"GPS Module connected.");
    }else{
      textmanager.drawText(ND_GPSCOND,2,3, posy,COLOR_MAGENTA,"GPS connection not found !! Check connection, or try reset.");
      return;
    }

    int col = COLOR_BLACK;
    if(!get_gps_fix()){
      textmanager.drawText(ND_SEARCHING,2,3, posy+15,col,"Weak Signal. Fixing GPS.");
      int dotcounter = (millis() / 900) % 10;
      char text[20] = "Scanning";
      int i = 8;
      for (; i < 8 + dotcounter && i < 20; i++) {
        text[i] = '.';
      }
      text[i] = '\0';
      textmanager.drawText(ND_GPSDOTS,2,3, posy+30,col,text);
    }
    else if(nomap_drawn){
      textmanager.drawText(ND_SEARCHING,2,3, posy+15,col,"NO MAPDATA.GPS Fixed.");
    }
  }
}
unsigned long lastdrawn_const = 0;
void draw_gpsdetail(bool redraw, int page){
  if(millis()-lastdrawn_const > 1000){
    redraw = true;
  }
  if(redraw){
    lastdrawn_const = millis();
    redraw = false;
    bool aru = false;
    for (int i = 0; i < 32; i++) {
      if (satellites[i].PRN != 0) aru = true; // Skip empty entries
    }

    tft.fillScreen(COLOR_WHITE);

    if(page%2 == 1){
      tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
      tft.setTextSize(2);
      tft.setCursor(1, 1);
      tft.println("GPS DETAIL 2: raw NMEAs");

      tft.unloadFont();
      tft.setTextSize(1);
      int posy = 10;
      tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
      for(int i = 0; i < MAX_LAST_NMEA; i++){
        posy += 20;
        tft.setCursor(1,posy);
        tft.println(get_gps_nmea(i));
      }
      tft.loadFont(AA_FONT_SMALL);    // Must load the font first
    }
    if(page%2 == 0){
      tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
      tft.setTextSize(2);
      tft.setCursor(1, 1);
      tft.println("GPS DETAIL 1:CONSTELLATION");

      tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 , SCREEN_WIDTH/2-2, COLOR_BLACK);
      tft.setTextColor(COLOR_CYAN,COLOR_WHITE);
      tft.print("GPS ");
      tft.setTextColor(COLOR_GREEN,COLOR_WHITE);
      tft.print("GLO ");
      tft.setTextColor(COLOR_WHITE,COLOR_BLUE,true);
      tft.print("GAL ");
      tft.setTextColor(COLOR_RED,COLOR_WHITE);
      tft.print("QZS ");
      tft.setTextColor(COLOR_ORANGE,COLOR_WHITE);
      tft.print("BEI ");

      tft.setCursor(0,SCREEN_HEIGHT-15);
      tft.unloadFont();
      tft.setTextSize(1);
      tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
      TinyGPSDate date = get_gpsdate();
      TinyGPSTime time = get_gpstime();
      if(date.isValid() && time.isValid()){
        tft.printf("DateTime:%d.%d.%d %d:%d:%d UTC",date.year(),date.month(),date.day(),time.hour(),time.minute(),time.second());
      }
      tft.loadFont(AA_FONT_SMALL);    // Must load the font first

      if(!aru)
        return;

      tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
      for (int i = 0; i < 32; i++) {
        if (satellites[i].PRN == 0) continue; // Skip empty entries
        //if(satellites[i].SNR == 0) continue; //Skip unknown signal strength.

        // Calculate position on display based on azimuth and elevation
        float azimuth = satellites[i].azimuth;
        float elevation = satellites[i].elevation;

        int x = calculateGPS_X(azimuth, elevation);
        int y = calculateGPS_Y(azimuth, elevation);
        int size = 5;
        if(satellites[i].SNR <= 0){
          size = 2;
        }

        if(satellites[i].satelliteType == SATELLITE_TYPE_QZSS)
          tft.fillCircle(x, y, size, COLOR_RED);
        else if(satellites[i].satelliteType == SATELLITE_TYPE_GPS)
          tft.fillCircle(x, y, size, COLOR_CYAN);
        else if(satellites[i].satelliteType == SATELLITE_TYPE_GLONASS)
          tft.fillCircle(x, y, size, COLOR_GREEN);
        else if(satellites[i].satelliteType == SATELLITE_TYPE_GALILEO)
          tft.drawCircle(x, y, size, COLOR_BLUE);
        else if(satellites[i].satelliteType == SATELLITE_TYPE_BEIDOU)
          tft.drawCircle(x, y, size, COLOR_ORANGE);
        else
          tft.fillCircle(x, y, size, COLOR_BLACK);
        
        int textx = constrain(x+6,0,SCREEN_WIDTH-20);
        if(satellites[i].PRN >= 10)
          textx -= 10;
        if(satellites[i].PRN >= 100)
          textx -= 10;
        tft.setCursor(textx, y - 3);
        tft.print(satellites[i].PRN);
      }
    }
  }
}


void draw_maplist_mode(bool redraw, int maplist_page){
  if(millis()-lastdrawn_const > 10000){
    redraw = true;
  }
  if(redraw){
    lastdrawn_const = millis();
    redraw = false;

    mapdata* mapdatas[] = {&map_shinura,&map_okishima,&map_takeshima,&map_chikubushima,&map_biwako,&map_handaioutside,&map_handaihighway,&map_handaihighway2,&map_handaiinside1,&map_handaiinside2,&map_handaiinside3,
    &map_handaiinside4,&map_handaiinside5,&map_handairailway,&map_handaicafe,&map_japan1,&map_japan2,&map_japan3,&map_japan4};
    int sizeof_mapflash = sizeof(mapdatas)/sizeof(mapdatas[0]);
    tft.fillScreen(COLOR_WHITE);
    tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
    tft.setCursor(1, 1);


    int pagetotal = 1+(sizeof_mapflash+mapdata_count)/30;
    int pagenow = maplist_page%pagetotal;
    
    tft.printf("MAPLIST FLSH:%d/SD:%d (%d/%d)",sizeof_mapflash,mapdata_count,pagenow+1,pagetotal);

    tft.unloadFont();
    tft.setTextSize(1);
    int posy = 20;
    tft.setCursor(1,posy);
    tft.setTextColor(COLOR_BLACK);
    if(pagenow == 0){
      for(int i = 0; i < sizeof_mapflash;i++){
        tft.printf("FLSH %d: %s,%4.2f,%4.2f,%d",mapdatas[i]->id,mapdatas[i]->name,mapdatas[i]->cords[0][0],mapdatas[i]->cords[0][1],mapdatas[i]->size);
        posy += 10;
        tft.setCursor(1,posy);
      }
    }

    int sd_start_index = 0;
    if(pagenow > 0){
      sd_start_index = 30*pagenow-sizeof_mapflash;
    }

    for(int i = sd_start_index; i < mapdata_count;i++){
      if(extramaps[i].size <= 1){
        continue;
      }
      double lon1 = extramaps[i].cords[0][0];
      double lat1 = extramaps[i].cords[0][1];
      char name_firstchar = extramaps[i].name[0];
      
      tft.printf("SD %d:%s,%4.2f,%4.2f,%d,%c",i,extramaps[i].name,lat1,lon1,extramaps[i].size,name_firstchar);
      posy += 10;
      tft.setCursor(1,posy);
    }

    tft.loadFont(AA_FONT_SMALL);    // Must load the font first
    
  }
}

void reset_degpersecond();
extern int screen_mode;
int t_counter = 0;

Setting settings[] = {
    {SETTING_SETDESTINATION,
        [](bool selected) -> std::string {
            char buff[32];   // temporary buffer
            sprintf(buff, selected ? " Set destination: %03d" : "Set destination >: %03d", t_counter);
            return std::string(buff);  // return as std::string
        },
        []() {
            Serial.println("hi");
        },
        []() {
            t_counter++;
        }
    },
    #ifdef BRIGHTNESS_SETTING_AVAIL    
        {SETTING_BRIGHTNESS, 
            [](bool selected) -> std::string {
                char buff[32];   // temporary buffer
                sprintf(buff, selected ? " Brightness: %03d" : "Brightness: %03d", screen_brightness);
                return std::string(buff);  // return as std::string
            }, nullptr,
            []() {
                tft_change_brightness(1);
            }
        },
    #endif 
    {SETTING_DEMOBIWA, 
        [](bool selected) -> std::string {
            char buff[32];   // temporary buffer
            sprintf(buff, selected ? " DEMO BIWA: %s" : "DEMO BIWA: %s", get_demo_biwako() ? "YES" : "NO");
            return std::string(buff);  // return as std::string
        }, nullptr,
        []() {
            toggle_demo_biwako();
            reset_degpersecond();
        }
    },
    {SETTING_UPWARD, 
        [](bool selected) -> std::string {
            char buff[32];   // temporary buffer
            sprintf(buff, selected ? " Upward: %s" : "Upward: %s", is_trackupmode() ? "TRACK UP" : "NORTH UP");
            return std::string(buff);  // return as std::string
        }, nullptr,
        []() {
            toggle_mode();
        }
    },
    {SETTING_GPSDETAIL,
        [](bool selected) -> std::string {
            return "Show GPS detail >";
        },
        []() {
            DEBUG_P(20240801,"GPS CONST MODE");
            gps_constellation_mode();
            screen_mode = MODE_GPSDETAIL;
        },
        nullptr
    },
    {SETTING_MAPDETAIL,
        [](bool selected) -> std::string {
            return "Maplist detail >";
        },
        []() {
            DEBUG_P(20240801,"MAP DETAIL MODE");
            screen_mode = MODE_MAPLIST;
        },
        nullptr
    },
    {SETTING_EXIT,
        [](bool selected) -> std::string {
            return "Exit setting";
        },
        []() {
            screen_mode = MODE_MAP;
        },
        nullptr
    }
};


int setting_size = sizeof(settings)/sizeof(settings[0]);

void draw_setting_mode(bool redraw, int selectedLine, int cursorLine) {
  const int separation = 25;
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