#include "display_tft.h"
#include "latlon.h"
#include "gps_functions.h"

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


void setup_tft(){

    // Initialize backlight control pin
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // Turn on the backlight
  //analogWrite(TFT_BL,255);

  tft.initR(INITR_BLACKTAB);
  //tft.setSPISpeed(24000000);

  Serial.println(F("Initialized"));

  tft.fillScreen(ST77XX_BLACK);

  startup_demo_tft();

}

bool draw_circle_km(float scale, int km) {
  int radius = PX_PER_KM(scale) * km;
  int ypos = SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN - radius;
  if (ypos > 16) {
    tft.drawCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN, radius, ST77XX_RED);
    tft.setCursor(SCREEN_WIDTH / 2+2, ypos - 2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.print(km);
    tft.println("km");
    return true;
  } else {
    return false;
  }
}



void draw_km_circle(float scale) {

  tft.drawFastVLine(SCREEN_WIDTH/2, 20, MAP_SHIFT_DOWN+SCREEN_HEIGHT/2-22, ST77XX_GRAY);

  if (!draw_circle_km(scale, 10)) {
    if (!draw_circle_km(scale, 5)) {
      if (!draw_circle_km(scale, 2)) {
        draw_circle_km(scale, 1);
      } else {
        draw_circle_km(scale, 1);
      }
    } else {
      draw_circle_km(scale, 2);
    }
  } else {
    draw_circle_km(scale, 5);
  }
}


void draw_pilon_line(float mapcenter_lat,float mapcenter_lon,float scale,float upward){
  float plat_lat = 35.29491494;
  float plat_lon = 136.255252;
  float north_lat = 35.41640778478595;
  float north_lon = 136.1183001762145;
  float west_lat = 35.23295479141404;
  float west_lon =  136.0493286559818;

  cord_tft pla = latLonToXY(plat_lat,plat_lon,mapcenter_lat,mapcenter_lon,scale,upward);
  cord_tft n_pilon = latLonToXY(north_lat,north_lon,mapcenter_lat,mapcenter_lon,scale,upward);
  cord_tft w_pilon = latLonToXY(west_lat,west_lon,mapcenter_lat,mapcenter_lon,scale,upward);

  tft.drawLine(pla.x, pla.y, n_pilon.x, n_pilon.y, ST77XX_MAGENTA);
  tft.drawLine(pla.x, pla.y, w_pilon.x, w_pilon.y, ST77XX_MAGENTA);
}

void startup_demo_tft() {
  for (int i = 0; i <= 360; i += 30) {

    //float scale = 1.5;
    //float center_lat = 35.63006541;
    //float center_lon = 139.9210862;
    //drawmap(true, i, center_lat, center_lon, scale, &map_shinura);
    //drawmap(false, i, center_lat, center_lon, scale, &map_toyosu);
    //draw_km_circle(scale);

    
    float scale = 0.15;
    float center_lat = 35.3034225841915;
    float center_lon = 136.1461056306493;
    drawmap(true,i,center_lat,center_lon,scale,&map_biwako);
    drawmap(false,i,center_lat,center_lon,scale,&map_takeshima);
    drawmap(false,i,center_lat,center_lon,scale,&map_chikubushima);
    drawmap(false,i,center_lat,center_lon,scale,&map_okishima);
    draw_km_circle(scale);
    fill_sea_land(center_lat, center_lon, scale, i);
    tft.setCursor(SCREEN_WIDTH/2-30, SCREEN_HEIGHT/2-15);
    tft.setTextSize(3);
    tft.print("DEMO");
    

    delay(50);
  }
}

unsigned long last_newtrack_time = 0;
float old_track = 0;

void show_gpsinfo(){
  tft.fillRect(SCREEN_WIDTH/2-14, 0, 34, 14, ST77XX_BLACK);
  tft.setCursor(SCREEN_WIDTH/2-14, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  char buf[4];
  sprintf( buf, "%03d", (int)get_gps_truetrack() );
  tft.println(buf); // Display as an integer
  
  tft.setTextSize(1);

  unsigned long timedif = millis() - last_newtrack_time;
  if(!timedif == 0){
    float degchange = get_gps_truetrack()-old_track;
    if(degchange<-180){
      degchange += 360;
    }else if(degchange >180){
      degchange -= 360;
    }
    float degpersecond = degchange/(timedif/1000);
    old_track = get_gps_truetrack();
    tft.fillRect(SCREEN_WIDTH-28, 0, 28, 8, ST77XX_BLACK);
    tft.setCursor(SCREEN_WIDTH-28, 0);
    tft.print(degpersecond,1);
    tft.setCursor(SCREEN_WIDTH-30, 8);
    tft.print("deg/s");
    if(abs(degpersecond) > 3.0 || abs(degchange) > 15){
      tft.invertDisplay(true);
    }else{
      tft.invertDisplay(false);
    }
  }
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.fillRect(0, 0, 18, 16, ST77XX_BLACK);

  
  // Convert speed from knots to m/s (1 knot = 0.514444 m/s) and display with one decimal place
  double speed_m_s = get_gps_speed() * 0.514444;
  tft.print(speed_m_s, 1);
  tft.println("m/s");

  tft.fillRect(0, SCREEN_HEIGHT-8, 18, 8, ST77XX_BLACK);
  tft.setCursor(0, SCREEN_HEIGHT-8);
  tft.print(get_gps_numsat());

  /*
  tft.println(get_gps_lat(),4);
  tft.println(get_gps_long(),4);
  tft.println(get_gps_altitude());
  tft.println(get_gps_magvar());

  */

}

struct line{
  int x1,y1,x2,y2;
};

#define MAX_MAPDATA 150
int last_drawn_points = 0;
cord_tft points[MAX_MAPDATA];
float last_scale = 1.0;
int lastfresh_millis = 0;


void drawmap(bool erasePreviousLines, float mapUpDirection, float center_lat, float center_lon, float mapScale, const mapdata* mp) {
  int tstart = millis();
  if (erasePreviousLines) {
    bool fresh = last_scale != mapScale;
    if(millis() - lastfresh_millis > 10000){
      fresh = true;
      lastfresh_millis = millis();
    }
    last_scale = mapScale;
    if (fresh) {
      tft.fillScreen(ST77XX_BLACK);
      show_gpsinfo();
    } else {
      for (int i = 0; i < last_drawn_points; i++) {
        //if(drawbool[i] && drawbool[i+1]){
        tft.drawLine(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, ST77XX_BLACK);
        //}
      }
    }
    Serial.print(millis() - tstart);
    Serial.println(" ms to fill screen");
    last_drawn_points = 0;
  }
  if (last_drawn_points + mp->mapsize >= MAX_MAPDATA) {
    Serial.println("map overflow");

    tft.setCursor(0, SCREEN_HEIGHT / 2 - 20);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.println("OVERFLOW");
    return;
  }

  int tstart_calc = millis();
  //float mapScale = 1.5;  // Example scale

  //cord_map mapcenter = xyToLatLon(SCREEN_WIDTH/2,SCREEN_HEIGHT/2,center_lat,center_lon,mapScale,mapUpDirection);

  //bool drawbool[BIWAKO_DATA];
  for (int i = 0; i < mp->mapsize; i++) {
    float lat1 = mp->cords[i][1];  // Example latitude
    float lon1 = mp->cords[i][0];  // Example longitude
                                   //drawbool[i] = check_maybe_inside_draw(mapcenter, lat1, lon1, mapScale);
                                   //if(drawbool[i]){
    points[i + last_drawn_points] = latLonToXY(lat1, lon1, center_lat, center_lon, mapScale, mapUpDirection);
    //}else{
    //}
  }
  Serial.print(millis() - tstart_calc);
  Serial.println("ms to calculate map");
  //tft.startWrite();
  int tstartline = millis();

  for (int i = 0; i < mp->mapsize - 1; i++) {
    //if(drawbool[i] && drawbool[i+1]){
    int index = last_drawn_points + i;
    tft.drawLine(points[index].x, points[index].y, points[index + 1].x, points[index + 1].y, ST77XX_GREEN);
    //}
  }
  last_drawn_points += mp->mapsize;
  Serial.print(millis() - tstartline);
  Serial.println("ms to draw lines");
  //tft.endWrite();
  tft.fillTriangle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN, SCREEN_WIDTH / 2 - 4, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN + 8, SCREEN_WIDTH / 2 + 4, SCREEN_HEIGHT / 2 + MAP_SHIFT_DOWN + 8, ST77XX_RED);

  Serial.print(millis() - tstart);
  Serial.println("ms to draw complete map");
}


cord_tft fill_points[ROW_FILLDATA*COL_FILLDATA];

void fill_sea_land(float mapcenter_lat, float mapcenter_lon, float scale, float upward) {

  for(int i = 0; i < ROW_FILLDATA*COL_FILLDATA;i++){
    tft.drawFastHLine(fill_points[i].x-1, fill_points[i].y, 3, ST77XX_BLACK);
  }

  for (int lat_i = 0; lat_i < ROW_FILLDATA; lat_i++) {
    float latitude = 35.0 + lat_i * 0.03;
    for (int lon_i = 0; lon_i < COL_FILLDATA; lon_i++) {
      float longitude = 135.8 + lon_i * 0.03;
      bool is_sea = filldata[lat_i][lon_i];
      int indexfillp = lat_i*COL_FILLDATA+lon_i;
      fill_points[indexfillp] = latLonToXY(latitude, longitude, mapcenter_lat, mapcenter_lon, scale, upward);
      //tft.setCursor(fill_points[indexfillp].x, fill_points[indexfillp].y);
      tft.setTextSize(1);
      if (is_sea) {
        tft.drawFastHLine(fill_points[indexfillp].x-1, fill_points[indexfillp].y, 3, ST77XX_BLUE);
      } else {
        tft.drawFastHLine(fill_points[indexfillp].x-1, fill_points[indexfillp].y, 3, ST77XX_ORANGE);
      }
    }
  }
}