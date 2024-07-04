#include "gps_functions.h"
#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "gps_functions.h"

#ifdef USE_OLED
  #include "display_oled.h"
#endif





unsigned long last_newtrack_time = 0;
float truetrack_now = 0;
float lat_now = 0;
unsigned long last_time_gpsdata = 0;    //GPSを最後に受信した時間 millis()
bool redraw = false;          //次の描画では、黒塗りして新たに画面を描画する
bool quick_redraw = false;    //次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画する
bool bank_warning = false;

const int sampleInterval = 500; // Sample interval in milliseconds
const int numSamples = 6;       // Number of samples to take over 3 seconds
float upward_samples[numSamples]; // Array to store truetrack values
unsigned long lastSampleTime = 0; // Time when the last sample was taken
int sampleIndex = 0; // Index to keep track of current sample

float degpersecond = 0; // The calculated average differential


//スイッチ関連
#ifdef RP2040_ZERO
  const int switchPin = 7;  // Pin where the switch is connected
#else
  const int switchPin = D2;  // Pin where the switch is connected
#endif
unsigned long pressTime = 0;   // Time when the switch is pressed
unsigned long debounceTime = 50; // Debounce time in milliseconds
unsigned long longPressDuration = 1000; // Duration to detect a long press in milliseconds
bool switchState = HIGH;  // Current state of the switch
bool lastSwitchState = HIGH; // Previous state of the switch
bool longPressHandled = false; // Whether the long press was already handled


// Settings mode variable
bool show_setting = false;


#ifdef XIAO_RP2040
  #include <Adafruit_NeoPixel.h>
  int Power = 11;
  int PIN  = 12;
  #define NUMPIXELS 1
  Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#endif

#define SCALE_JAPAN 0.008f


int scaleindex = 3;
const float scalelist[] = {SCALE_JAPAN,0.2f,0.4f,1.0f,2.5f,10.0f};
float scale = scalelist[scaleindex];



void setup(void) {
  Serial.begin(38400);
  Serial.print(F("SETUP"));



  #ifdef XIAO_RP2040
    pixels.begin();
    pinMode(Power,OUTPUT);
    digitalWrite(Power, HIGH);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(15, 25, 205));
    pixels.show();
  #endif



  pinMode(switchPin,INPUT_PULLUP);

  gps_setup();

  #ifdef USE_OLED
    setup_oled();
  #endif
  setup_tft();


  startup_demo_tft();

  tft.fillScreen(COLOR_WHITE);


  #ifdef XIAO_RP2040
    pixels.clear();
    pixels.show();
  #endif

  redraw = true;
  quick_redraw = true;
  
}

void update_degpersecond(){
  unsigned long currentTime = millis();
  // Check if it's time to take a new sample
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    // Update the truetrack value
    // This is where you would get the new truetrack value from your sensor
    upward_samples[sampleIndex] = get_gps_truetrack();
    // Calculate the differential if we have enough samples
    if (sampleIndex >= numSamples - 1) {
      float totalDifference = 0;
      // Calculate the differences between consecutive samples
      for (int i = 1; i < numSamples; i++) {
        float degchange =  (upward_samples[i] - upward_samples[i - 1]) ;
        if(degchange<-180){
          degchange += 360;
        }else if(degchange >180){
          degchange -= 360;
        }
        float difference = degchange / (sampleInterval / 1000.0); // deg/s
        totalDifference += difference;
      }
      // Calculate the average differential
      degpersecond = totalDifference / (numSamples - 1);
      // Print the average differential
      Serial.print("Average Differential (deg/s): ");
      Serial.println(degpersecond);
      // Shift samples to make room for new ones
      for (int i = 1; i < numSamples; i++) {
        upward_samples[i - 1] = upward_samples[i];
      }
      // Adjust the sample index to the last position
      sampleIndex = numSamples - 2;
    }
    // Move to the next sample
    sampleIndex++;
  }
}

void check_bankwarning(){
  unsigned long currentTime = millis();
  last_newtrack_time = currentTime;

  //Do not trigger bank warning if speed is below ...
  if(get_gps_mps() < 2.0){
    if(bank_warning){
      redraw = true;
      #ifdef XIAO_RP2040
        pixels.clear();
        pixels.show();
      #endif
      tft.invertDisplay(false);
      bank_warning = false;
    }
    return;
  }

  
  if(abs(degpersecond) > 3.0 && get_gps_mps() > 2){
    bank_warning = true;
  }else{
    if(bank_warning){
      redraw = true;
      #ifdef XIAO_RP2040
        pixels.clear();
        pixels.show();
      #endif
      tft.invertDisplay(false);
      bank_warning = false;
    }
  }
}


void debug_print(const char* inp){
  Serial.println(inp);
}


// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
#define SETTING_LINES 4


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
        if(show_setting){//Setting mode
          if(selectedLine == -1){//No active selected line.
            cursorLine = (cursorLine+1)%SETTING_LINES;
          }else if(selectedLine == 0){
            tft_increment_brightness();
            redraw = true;
          }else if(selectedLine == 1){
            toggle_demo_biwako();  
            redraw = true;
          }else{
            toggle_mode();
            redraw = true;
          }
        }else{
          scale = scalelist[scaleindex++%(sizeof(scalelist) / sizeof(scalelist[0]))];
          quick_redraw = true;
          redraw = true;
        }
      }
    }
    delay(debounceTime); // Debounce delay
  } else if (reading == LOW && !longPressHandled) {
    // Check for long press
    if (millis() - pressTime >= longPressDuration) {
        Serial.println("long press");
        longPressHandled = true;
        if(!show_setting){        
          show_setting = true;
          cursorLine = 0;
          selectedLine = -1;
          redraw = true;
        }else{
          //exit
          if(cursorLine == SETTING_LINES-1){
            show_setting = false;
            quick_redraw = true;
            redraw = true;
          }else{
            if(selectedLine == -1){
              //Entering changing value mode.
              selectedLine = cursorLine;
              redraw = true;
            }else{
              //exiting changing value mode.
              selectedLine = -1;
              redraw = true;
            }
          }
        }
    }
  }

  lastSwitchState = reading; // Update the switch state
}


bool err_nomap = false;

void loop() {
  switch_handling();
  gps_loop();
  
  if (show_setting) {
    draw_setting_mode(redraw, selectedLine, cursorLine);
    return;
  }


  if(bank_warning){
    if((millis()/100)%3 == 0){
      #ifdef XIAO_RP2040
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.show();
      #endif
      tft.invertDisplay(false);
    }
    if((millis()/100)%3 == 1){
      #ifdef XIAO_RP2040
        pixels.clear();
        pixels.show();
      #endif
      tft.invertDisplay(true);
    }
  }


  update_degpersecond();

  if(millis() - last_time_gpsdata > SCREEN_INTERVAL || quick_redraw){
    float new_truetrack = get_gps_truetrack();
    float new_lat = get_gps_lat();
    Serial.println(new_lat);
    last_time_gpsdata = millis();
    quick_redraw = false;

    check_bankwarning();

    #ifdef XIAO_RP2040
      if(!bank_warning){
        //1.0deg per second から　3.0 deg per second をLEDの0-255に変換。
        byte led_g = constrain(map(degpersecond*100,100,300,0,255),0,255);
        byte led_b = constrain(map(-degpersecond*100,100,300,0,255),0,255);
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(0, led_g, led_b));
        pixels.show();
      }
    #endif
    
    if(new_truetrack != truetrack_now || new_lat != lat_now || redraw){
      truetrack_now = new_truetrack;
      lat_now = new_lat;

      float drawupward_direction = truetrack_now;
      if(is_northupmode()){
        drawupward_direction = 0;
      }
      float new_long = get_gps_long();

      if(scale <= SCALE_JAPAN){
        drawJapan(redraw,new_lat,new_long,scale,drawupward_direction);
      }else{
        if(abs(new_long - PLA_LON) < 0.6){
          drawBiwako(redraw,new_lat,new_long, scale, drawupward_direction);
        }
        else if(abs(new_long - OSAKA_LON) < 0.6){
          drawOsaka(redraw, new_lat,new_long, scale, drawupward_direction);
        }
        else if(abs(new_long - SHINURA_LON) < 0.6){
          drawShinura(redraw,new_lat, new_long, scale, drawupward_direction);
          redraw = false;
        }else{
          tft.fillRect(0, SCREEN_HEIGHT/2-50, SCREEN_WIDTH, 120, COLOR_WHITE);
          tft.setTextColor(COLOR_BLACK);
          tft.setTextSize(2);
          tft.setCursor(0, SCREEN_HEIGHT/2-30);
          
          if(!get_gps_fix()){
            tft.println("Searching");
            tft.print("GPS");
            int dotcouter = (millis()/1000)%5;
            for(int i = 0; i < dotcouter;i++){
              tft.print(".");
            }
            tft.println("");
            tft.setTextSize(1);
            if(get_gps_connection()){
              tft.setTextColor(COLOR_GREEN);
              tft.print("GPS connection found");
            }else{
              tft.setTextColor(COLOR_MAGENTA);
              tft.println("No GPS found !!");
              tft.print("Check connection, or try reset.");
            }
          }else{
            tft.println("NO MAPDATA");
            tft.println("GPS Fixed.");
          }
          redraw = true;
        }
      }

      if(get_demo_biwako()){
        tft.setCursor(SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2 - 35);
        tft.setTextSize(2);
        tft.print("DEMO");
      }
    }
    draw_triangle();
    draw_degpersecond(degpersecond);
    draw_gpsinfo();
    

    if(bank_warning){
      draw_bankwarning();
    }

    #ifdef USE_OLED
      show_oled();
    #endif
  }
  
}

