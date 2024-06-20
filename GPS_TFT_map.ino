#include "gps_functions.h"
#include "navdata.h"
#include "display_oled.h"
#include "display_tft.h"


void setup(void) {
  Serial.begin(38400);
  Serial.print(F("SETUP"));
  delay(5000);
  Serial.print(F("SETUP"));
  
  gps_setup();
  setup_oled();
  setup_tft();
  
}



float track_now = 0;
float lat_now = 0;


void debug_print(char* inp){
  //Serial.println(inp);
}


int last_time_gpsdata = 0;
void loop() {
  
  
  debug_print("gps loop");
  gps_loop();
  debug_print("loop begin");

  float newtrack = get_gps_truetrack();
  float new_lat = get_gps_lat();

  debug_print("show gps");


  if(millis() - last_time_gpsdata > 1000){
    last_time_gpsdata = millis();
    show_gpsinfo();
  }


  if(newtrack != track_now || new_lat != lat_now){
    track_now = newtrack;
    lat_now = new_lat;
    float new_long = get_gps_long();
    debug_print("drawmap begin");


    //float scale = 2.5;
    //drawmap(true, newtrack,new_lat,new_long, scale, &map_shinura);
    //drawmap(false,newtrack,new_lat,new_long,scale, &map_toyosu);
    
    float scale = 1.5;//;0.7;
    float lat = 35.29491494;
    float lon = 136.255252;
    drawmap(true,newtrack,lat,lon, scale, &map_biwako);
    drawmap(false,newtrack,lat,lon, scale, &map_takeshima);
    drawmap(false,newtrack,lat,lon, scale, &map_chikubushima);
    drawmap(false,newtrack,lat,lon, scale, &map_okishima);
    fill_sea_land(lat,lon,scale,newtrack);
    draw_pilon_line(lat,lon,scale,newtrack);
    draw_km_circle(scale);
    debug_print("draw km circle");
    
  }
  debug_print("show oled");
  show_oled();
  delay(10);
}

