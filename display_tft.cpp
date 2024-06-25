#include "display_tft.h"
#include "latlon.h"
#include "navdata.h"
#include "gps_functions.h"

Adafruit_ST7735 tft = Adafruit_ST7735(-1, TFT_DC, TFT_RST);


void setup_tft() {


  tft.initR(INITR_BLACKTAB);
  //tft.setSPISpeed(24000000);

  Serial.println(F("Initialized"));

  tft.fillScreen(ST77XX_BLACK);

  // Initialize backlight control pin
  pinMode(TFT_BL, OUTPUT);
  //digitalWrite(TFT_BL, HIGH);  // Turn on the backlight
  analogWrite(TFT_BL,255);
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
      Serial.println("ERR max stroke");
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
        tft.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, ST77XX_BLACK);
      }
    }
  }


  void drawAllStroke() {
    for (int i = 0; i < strokeCount; i++) {
      for (int j = 0; j < strokes[i].pointCount - 1; j++) {
        tft.drawLine(strokes[i].points[j].x, strokes[i].points[j].y, strokes[i].points[j + 1].x, strokes[i].points[j + 1].y, ST77XX_BLACK);
      }
    }
    Serial.print("stroke black ");
    Serial.println(strokeCount);
  }
};

#define MAX_STROKES 600

StrokeManager strokeManager(MAX_STROKES);




bool draw_circle_km(float scale, float km) {
  int radius = PX_PER_KM(scale) * km;
  int ypos = SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN - radius;
  if (ypos > 16) {
    tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN, radius, ST77XX_RED);
    tft.setCursor(SCREEN_WIDTH / 2 + 2, ypos - 2);
    tft.setTextColor(ST77XX_WHITE);
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

// Convert degrees to radians
float degreesToRadians(float degrees) {
  return degrees * PI / 180.0;
}


// draw north east west south according to truetrack, where truetrack is direction of upward display.
// However direction of y-axis is downward for tft display.
void draw_news(float truetrack, uint16_t col) {
  const int dist = 103;
  int centerx = SCREEN_WIDTH / 2;
  int centery = SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN;
  float radian = degreesToRadians(truetrack);
  float radian45offset = degreesToRadians(truetrack + 45);
  tft.setTextColor(col);
  tft.setTextSize(2);

  cord_tft n = { int(centerx + sin(-radian) * dist), int(centery - cos(radian) * dist) };
  cord_tft e = { int(centerx + cos(radian) * dist), int(centery - sin(radian) * dist) };
  cord_tft s = { int(centerx + sin(radian) * dist), int(centery + cos(radian) * dist) };
  cord_tft w = { int(centerx - cos(-radian) * dist), int(centery - sin(-radian) * dist) };
  if (!n.isOutsideTft()) {
    tft.setCursor(n.x, n.y);
    tft.print("N");
  }
  if (!e.isOutsideTft()) {
    tft.setCursor(e.x, e.y);
    tft.print("E");
  }
  if (!s.isOutsideTft()) {
    tft.setCursor(s.x, s.y);
    tft.print("S");
  }
  if (!w.isOutsideTft()) {
    tft.setCursor(w.x, w.y);
    tft.print("W");
  }

  cord_tft nw = { int(centerx + sin(-radian45offset) * dist), int(centery - cos(radian45offset) * dist) };
  cord_tft ne = { int(centerx + cos(radian45offset) * dist), int(centery - sin(radian45offset) * dist) };
  cord_tft se = { int(centerx + sin(radian45offset) * dist), int(centery + cos(radian45offset) * dist) };
  cord_tft sw = { int(centerx - cos(-radian45offset) * dist), int(centery - sin(-radian45offset) * dist) };

  tft.setTextSize(1);
  if (!nw.isOutsideTft()) {
    tft.setCursor(nw.x, nw.y);
    tft.print("NW");
  }
  if (!ne.isOutsideTft()) {
    tft.setCursor(ne.x, ne.y);
    tft.print("NE");
  }
  if (!se.isOutsideTft()) {
    tft.setCursor(se.x, se.y);
    tft.print("SE");
  }
  if (!sw.isOutsideTft()) {
    tft.setCursor(sw.x, sw.y);
    tft.print("SW");
  }
}





void draw_km_circle(float scale) {

  tft.drawFastVLine(SCREEN_WIDTH / 2, 20, MAP_SHIFT_DOWN + SCREEN_HEIGHT / 2 - 22, ST77XX_GRAY);
  if (!draw_circle_km(scale, 20)) {
    if (!draw_circle_km(scale, 10)) {
      if (!draw_circle_km(scale, 5)) {
        if (!draw_circle_km(scale, 2)) {
          if (!draw_circle_km(scale, 1)) {
            draw_circle_km(scale, 0.5);
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
    tft.fillScreen(ST77XX_BLACK);
  } else {
    strokeManager.drawAllStroke();
  }
  strokeManager.removeAllStrokes();
}



void drawShinura(bool redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_shinura, ST77XX_GREEN);
  draw_km_circle(scale);
  draw_news(last_truetrack, ST77XX_BLACK);
  draw_news(up, ST77XX_WHITE);
  last_truetrack = up;
}

void drawBiwako(bool redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_biwako, ST77XX_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_takeshima, ST77XX_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_chikubushima, ST77XX_GREEN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_okishima, ST77XX_GREEN);
  fill_sea_land(center_lat, center_lon, scale, up);
  draw_km_circle(scale);
  draw_pilon_line(center_lat, center_lon, scale, up);
  draw_news(last_truetrack, ST77XX_BLACK);
  draw_news(up, ST77XX_WHITE);
  last_truetrack = up;
}

void drawOsaka(bool redraw, float center_lat, float center_lon, float scale, float up) {
  blacken_display(redraw);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaioutside, ST77XX_CYAN);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway, ST77XX_YELLOW);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaihighway2, ST77XX_YELLOW);

  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside1, ST77XX_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside2, ST77XX_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside3, ST77XX_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside4, ST77XX_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaiinside5, ST77XX_BRIGHTGRAY);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handairailway, ST77XX_ORANGE);
  drawmap(STRK_MAP1, up, center_lat, center_lon, scale, &map_handaicafe, ST77XX_GREEN);


  draw_km_circle(scale);
  draw_news(last_truetrack, ST77XX_BLACK);
  draw_news(up, ST77XX_WHITE);
  last_truetrack = up;
}


void startup_demo_tft() {
  for (int i = 0; i <= 360; i += 30) {
    float scale = 0.15;
    float center_lat = 35.3034225841915;
    float center_lon = 136.1461056306493;

    drawBiwako(false, center_lat, center_lon, scale, i);
    show_gpsinfo();
    draw_degpersecond(30.0 / (300.0 / 1000));
    tft.setCursor(SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 - 55);
    tft.setTextSize(3);
    tft.print("DEMO");
    delay(100);
  }
}

void draw_degpersecond(float degpersecond) {
  tft.fillRect(SCREEN_WIDTH - 28, 0, 28, 8, ST77XX_BLACK);

  if (degpersecond > 0) {
    tft.setTextColor(ST77XX_GREEN);
  }
  if (degpersecond < 0) {
    tft.setTextColor(ST77XX_BLUE);
  }
  tft.setCursor(SCREEN_WIDTH - 28, 0);
  tft.print(degpersecond, 1);
  tft.setCursor(SCREEN_WIDTH - 30, 8);
  tft.print("deg/s");

  int barposy = 22;
  tft.drawFastHLine(0, barposy - 1, SCREEN_WIDTH, ST77XX_BLACK);
  tft.drawFastHLine(0, barposy, SCREEN_WIDTH, ST77XX_BLACK);
  tft.drawFastHLine(0, barposy + 1, SCREEN_WIDTH, ST77XX_BLACK);
  int barwidth = (abs(degpersecond) * SCREEN_WIDTH / 2) / 3.0;
  if (degpersecond > 0) {
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy - 1, barwidth, ST77XX_GREEN);
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy, barwidth, ST77XX_GREEN);
    tft.drawFastHLine(SCREEN_WIDTH / 2, barposy + 1, barwidth, ST77XX_GREEN);
  } else {
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy - 1, barwidth, ST77XX_BLUE);
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy, barwidth, ST77XX_BLUE);
    tft.drawFastHLine(SCREEN_WIDTH / 2 - barwidth, barposy + 1, barwidth, ST77XX_BLUE);
  }
}

void draw_bankwarning() {
  tft.fillRect(SCREEN_WIDTH / 2 - 50, SCREEN_HEIGHT / 2 - 23, 100, 30, ST77XX_BLACK);
  tft.setCursor(SCREEN_WIDTH / 2 - 50, SCREEN_HEIGHT / 2 - 20);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.print("!BANK!");
  tft.setCursor(SCREEN_WIDTH / 2 - 50 - 1, SCREEN_HEIGHT / 2 - 20 - 2);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("!BANK!");
  tft.setTextSize(1);
}

void show_gpsinfo() {

  tft.fillRect(SCREEN_WIDTH / 2 - 14, 0, 34, 14, ST77XX_BLACK);
  tft.setCursor(SCREEN_WIDTH / 2 - 14, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  if(get_gps_fix()){
    char buf[4];
    sprintf(buf, "%03d", (int)get_gps_truetrack());
    tft.println(buf);  // Display as an integer
  }else{
    tft.println("N/A");  // Display as an integer
  }

  tft.setTextSize(1);

  tft.setCursor(0, 0);
  tft.fillRect(0, 0, 42, 8, ST77XX_BLACK);

  tft.setTextColor(ST77XX_GREEN);
  // Convert speed from knots to m/s (1 knot = 0.514444 m/s) and display with one decimal place
  if(get_gps_fix()){
    double speed_m_s = get_gps_speed() * 0.514444;
    tft.print(speed_m_s, 1);
  }else{
    tft.print("N/A");
  }
  tft.println("m/s");

  //tft.println(get_gps_lat(),4);
  //tft.println(get_gps_long(),4);

  //tft.fillRect(0, 8, 42, 8, ST77XX_BLACK);
  //tft.println(get_gps_altitude(),1);


  tft.fillRect(0, SCREEN_HEIGHT - 8, 16, 8, ST77XX_BLACK);
  tft.setCursor(0, SCREEN_HEIGHT - 8);
  tft.println(get_gps_numsat());
  //tft.print(get_gps_altitude());
}




void drawmap(stroke_group strokeid, float mapUpDirection, float center_lat, float center_lon, float mapScale, const mapdata* mp, uint16_t color) {


  int tstart_calc = millis();
  //float mapScale = 1.5;  // Example scale

  //cord_map mapcenter = xyToLatLon(SCREEN_WIDTH/2,SCREEN_HEIGHT/2,center_lat,center_lon,mapScale,mapUpDirection);
  cord_tft points[mp->mapsize];

  //bool drawbool[BIWAKO_DATA];
  strokeManager.addStroke(strokeid, mp->mapsize);
  for (int i = 0; i < mp->mapsize; i++) {
    float lat1 = mp->cords[i][1];  // Example latitude
    float lon1 = mp->cords[i][0];  // Example longitude
                                   //drawbool[i] = check_maybe_inside_draw(mapcenter, lat1, lon1, mapScale);
                                   //if(drawbool[i]){
    points[i] = latLonToXY(lat1, lon1, center_lat, center_lon, mapScale, mapUpDirection);
    strokeManager.addPointToStroke(points[i].x, points[i].y);

    //}else{
    //}
  }
  //Serial.print(millis() - tstart_calc);
  //Serial.println("ms to calculate map");
  //tft.startWrite();
  int tstartline = millis();

  for (int i = 0; i < mp->mapsize - 1; i++) {
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
  tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN, SCREEN_WIDTH / 2 - 4, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN + 8, SCREEN_WIDTH / 2 + 4, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN + 8, ST77XX_RED);

  //Serial.print(millis() - tstart);
  //Serial.println("ms to draw complete map");
}


void draw_pilon_line(float mapcenter_lat, float mapcenter_lon, float scale, float upward) {
  float north_lat = 35.41640778478595;
  float north_lon = 136.1183001762145;
  float west_lat = 35.23295479141404;
  float west_lon = 136.0493286559818;

  cord_tft pla = latLonToXY(PLA_LAT, PLA_LON, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft n_pilon = latLonToXY(north_lat, north_lon, mapcenter_lat, mapcenter_lon, scale, upward);
  cord_tft w_pilon = latLonToXY(west_lat, west_lon, mapcenter_lat, mapcenter_lon, scale, upward);

  strokeManager.addStroke(STRK_PILONLINE, 2);
  tft.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, ST77XX_MAGENTA);
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(n_pilon.x, n_pilon.y);


  strokeManager.addStroke(STRK_PILONLINE, 2);
  strokeManager.addPointToStroke(pla.x, pla.y);
  strokeManager.addPointToStroke(w_pilon.x, w_pilon.y);
  tft.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, ST77XX_MAGENTA);
  //Serial.println("added strokes");
  //strokeManager.printStrokes();
}


void fill_sea_land(float mapcenter_lat, float mapcenter_lon, float scale, float upward) {
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
        if (is_sea) {
          if (scale > 0.5) {
            int lenbar = 2;
            if(scale > 2) lenbar = 3;
            if(scale > 5) lenbar = 4;
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x, pos.y - lenbar);
            strokeManager.addPointToStroke(pos.x, pos.y + lenbar);
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x - lenbar, pos.y);
            strokeManager.addPointToStroke(pos.x + lenbar, pos.y);

            tft.drawFastVLine(pos.x, pos.y - lenbar, 1+lenbar*2, ST77XX_BLUE);
            tft.drawFastHLine(pos.x - lenbar, pos.y, 1+lenbar*2, ST77XX_BLUE);
          } else {
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x - 1, pos.y);
            strokeManager.addPointToStroke(pos.x + 1, pos.y);
            tft.drawFastHLine(pos.x - 1, pos.y, 3, ST77XX_BLUE);
          }
        } else {
          if (scale > 0.5) {
            int lenbar = 2;
            if(scale > 2) lenbar = 3;
            if(scale > 5) lenbar = 4;
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x, pos.y - lenbar);
            strokeManager.addPointToStroke(pos.x, pos.y + lenbar);
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x - lenbar, pos.y);
            strokeManager.addPointToStroke(pos.x + lenbar, pos.y);

            tft.drawFastVLine(pos.x, pos.y - lenbar, 1+lenbar*2, ST77XX_ORANGE);
            tft.drawFastHLine(pos.x - lenbar, pos.y, 1+lenbar*2, ST77XX_ORANGE);
          } else {
            strokeManager.addStroke(STRK_SEALAND, 2);
            strokeManager.addPointToStroke(pos.x - 1, pos.y);
            strokeManager.addPointToStroke(pos.x + 1, pos.y);
            tft.drawFastHLine(pos.x - 1, pos.y, 3, ST77XX_ORANGE);
          }
        }
      }
    }
  }
}