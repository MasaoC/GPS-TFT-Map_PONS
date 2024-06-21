#include "gps_functions.h"
#include "navdata.h"
#include "display_oled.h"
#include "display_tft.h"



#define RP2040

#ifdef RP2040
  #include <Adafruit_NeoPixel.h>
  int Power = 11;
  int PIN  = 12;
  #define NUMPIXELS 1
  Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#endif

#define SCREEN_INTERVAL 500 

int scaleindex = 3;
float scale = 2.0;
const float scalelist[] = {0.2,0.4,1.0,2.5,10.0};


unsigned long last_newtrack_time = 0;
float old_track = 0;
float track_now = 0;
float lat_now = 0;
int last_time_gpsdata = 0;
bool redraw = false;
bool quick_redraw = false;
float degpersecond = 0;
bool bank_warning = false;

const int switchPin = D2;  // Pin where the switch is connected
unsigned long pressTime = 0;   // Time when the switch is pressed
unsigned long debounceTime = 50; // Debounce time in milliseconds
unsigned long longPressDuration = 1000; // Duration to detect a long press in milliseconds
bool switchState = HIGH;  // Current state of the switch
bool lastSwitchState = HIGH; // Previous state of the switch
bool longPressHandled = false; // Whether the long press was already handled



void setup(void) {
  Serial.begin(38400);
  Serial.print(F("SETUP"));

  pixels.begin();
  pinMode(Power,OUTPUT);
  digitalWrite(Power, HIGH);

  pinMode(switchPin,INPUT_PULLUP);

  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(15, 25, 205));
  pixels.show();
  
  gps_setup();
  setup_oled();
  setup_tft();
  startup_demo_tft();

  tft.fillScreen(ST77XX_BLACK);
  pixels.clear();
  pixels.show();

  redraw = true;
  quick_redraw = true;
}




void check_bankwarning(){
  unsigned long timedif = millis() - last_newtrack_time;
  last_newtrack_time = millis();
  float degchange = get_gps_truetrack()-old_track;
  
  if(degchange<-180){
    degchange += 360;
  }else if(degchange >180){
    degchange -= 360;
  }
  
  if(timedif >0){
    degpersecond = degchange/(timedif/1000.0);
  }
  old_track = get_gps_truetrack();


  //Do not trigger bank warning if speed is below ...
  if(get_gps_speed() < 1.0){
    if(bank_warning){
      redraw = true;
      pixels.clear();
      pixels.show();
      bank_warning = false;
    }
    return;
  }
  if(abs(degpersecond) > 3.0 || abs(degchange) > 15){
    bank_warning = true;
  }else{
    if(bank_warning){
      redraw = true;
      pixels.clear();
      pixels.show();
      bank_warning = false;
    }
  }
}


void debug_print(const char* inp){
  Serial.println(inp);
}


void switch_handling(){
  int reading = digitalRead(switchPin); // Read the current state of the switch

  // Check for switch state change
  if (reading != lastSwitchState) {
    if (reading == LOW) {
      pressTime = millis(); // Record the time when the switch is pressed
      longPressHandled = false;
    } else {
      // When the switch is released
      unsigned long pressDuration = millis() - pressTime;
      if (pressDuration < longPressDuration && pressDuration > debounceTime) {
        Serial.println("short press");
        scale = scalelist[scaleindex++%(sizeof(scalelist) / sizeof(scalelist[0]))];
        quick_redraw = true;
        redraw = true;
      }
    }
    delay(debounceTime); // Debounce delay
  } else if (reading == LOW && !longPressHandled) {
    // Check for long press
    if (millis() - pressTime >= longPressDuration) {
        Serial.println("long press");
    }
  }

  lastSwitchState = reading; // Update the switch state
}


bool err_nomap = false;

void loop() {
  switch_handling();
  gps_loop();

  if(bank_warning){
    if((millis()/100)%3 == 0){
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    }
    if((millis()/100)%3 == 1){
      pixels.clear();
      pixels.show();
    }
  }

  if(millis() - last_time_gpsdata > SCREEN_INTERVAL || quick_redraw){
    float newtrack = get_gps_truetrack();
    float new_lat = get_gps_lat();
    Serial.println(new_lat);
    last_time_gpsdata = millis();
    quick_redraw = false;

    check_bankwarning();

    if(!bank_warning){
      //1.0deg per second から　3.0 deg per second をLEDの0-255に変換。
      byte led_g = constrain(map(degpersecond*100,100,300,0,255),0,255);
      byte led_b = constrain(map(-degpersecond*100,100,300,0,255),0,255);
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, led_g, led_b));
      pixels.show();
    }
    
    if(newtrack != track_now || new_lat != lat_now || redraw){
      track_now = newtrack;
      lat_now = new_lat;
      float new_long = get_gps_long();
      debug_print("drawmap begin");


      if(abs(new_long - PLA_LON) < 0.6){
        drawBiwako(redraw,new_lat,new_long, scale, newtrack);
        redraw = false;
      }
      else if(abs(new_long - OSAKA_LON) < 0.6){
        drawOsaka(redraw, new_lat,new_long, scale, newtrack);
        redraw = false;
      }
      else if(abs(new_long - SHINURA_LON) < 0.6){
        drawShinura(redraw,new_lat, new_long, scale, newtrack);
        redraw = false;
      }else{
        Serial.println("ERR NO map around this area");
        tft.fillRect(0, SCREEN_HEIGHT/2-50, SCREEN_WIDTH, 100, ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(2);
        tft.setCursor(0, SCREEN_HEIGHT/2-50);
        tft.println("NO MAPDATA AT");
        tft.print("");
        tft.print("LAT:");
        tft.println(new_lat);
        tft.print("LON:");
        tft.println(new_long);
        tft.print("Searching GPS");
        int dotcouter = (millis()/1000)%5;
        for(int i = 0; i < dotcouter;i++){
          tft.print(".");
        }
        redraw = true;
      }
    }
    show_gpsinfo();
    draw_degpersecond(degpersecond);
    

    if(bank_warning){
      draw_bankwarning();
    }
    debug_print("show oled");
    show_oled();
  }
  
}

