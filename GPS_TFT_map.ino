#include "gps_latlon.h"
#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"


#define SCALE_JAPAN 0.008f
#define SETTING_LINES 6

unsigned long last_newtrack_time = 0;
float truetrack_now = 0;
float lat_now = 0;
unsigned long last_time_gpsdata = 0;  //GPSを最後に受信した時間 millis()
bool redraw_screen = false;                  //次の描画では、黒塗りして新たに画面を描画する
bool quick_redraw = false;            //次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画する
bool bank_warning = false;

const int sampleInterval = 500;    // Sample interval in milliseconds
const int numSamples = 6;          // Number of samples to take over 3 seconds
float upward_samples[numSamples];  // Array to store truetrack values
unsigned long lastSampleTime = 0;  // Time when the last sample was taken
int sampleIndex = 0;               // Index to keep track of current sample
float degpersecond = 0;  // The calculated average differential

const int MODE_SETTING = 1;
const int MODE_MAP = 2;
const int MODE_GPSCONST = 3;
const int MODE_MAPLIST = 4;
int screen_mode = MODE_MAP;
int scaleindex = 3;
const float scalelist[] = { SCALE_JAPAN, 0.2f, 0.5f, 1.0f, 2.5f, 10.0f };
float scale = scalelist[scaleindex];
// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
bool err_nomap = false;
unsigned long lastfresh_millis = 0;


// Callback function for short press
void shortPressCallback() {
  quick_redraw = true;
  redraw_screen = true;
  Serial.println("short press");

  #ifdef SINGLE_SWITCH
    if (screen_mode == MODE_SETTING) {  //Setting mode
      if (selectedLine == -1) {         //No active selected line.
        cursorLine = (cursorLine + 1) % SETTING_LINES;
      } else if (selectedLine == 0) {
        tft_change_brightness(1);
      } else if (selectedLine == 1) {
        toggle_demo_biwako();
        reset_degpersecond();
      } else if (selectedLine == 2) {
        toggle_mode();
      } else if (selectedLine == 3) {
        screen_mode = MODE_GPSCONST;
      }
    } else {
      scale = scalelist[++scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
    }
  #else
    if (screen_mode != MODE_SETTING) {
      if (screen_mode == MODE_GPSCONST)
        gps_getposition_mode();
      screen_mode = MODE_SETTING;
      cursorLine = 0;
      selectedLine = -1;
      tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
    } else {
      //exit
      if (cursorLine == SETTING_LINES - 1) {
        screen_mode = MODE_MAP;
      } else if (cursorLine == 3) {
        Serial.println("GPS CONST MODE");
        gps_constellation_mode();
        screen_mode = MODE_GPSCONST;
      } else if (cursorLine == 4) {
        Serial.println("MAPLIST MODE");
        screen_mode = MODE_MAPLIST;
      } else {
        if (selectedLine == -1) {
          //Entering changing value mode.
          selectedLine = cursorLine;
        } else {
          //exiting changing value mode.
          selectedLine = -1;
        }
      }
    }
  #endif
}
// Callback function for short press
void shortPressCallback_up() {
  quick_redraw = true;
  redraw_screen = true;
  Serial.println("short press up");
  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = (cursorLine + 1) % SETTING_LINES;
    } else if (selectedLine == 0) {
      tft_change_brightness(1);
    } else if (selectedLine == 1) {
      toggle_demo_biwako();
      reset_degpersecond();
    } else if (selectedLine == 2) {
      toggle_mode();
    } else if (selectedLine == 3) {
      screen_mode = MODE_GPSCONST;
    }
  } else {
    scale = scalelist[++scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
  }
}


// Callback function for short press
void shortPressCallback_down() {
  quick_redraw = true;
  redraw_screen = true;
  Serial.println("short press down");
  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = mod(cursorLine - 1,SETTING_LINES);
    } else if (selectedLine == 0) {
      tft_change_brightness(-1);
    } else if (selectedLine == 1) {
      toggle_demo_biwako();
      reset_degpersecond();
    } else if (selectedLine == 2) {
      toggle_mode();
    } else if (selectedLine == 3) {
      screen_mode = MODE_GPSCONST;
    }
  } else {
    scale = scalelist[--scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
  }
}
// Callback function for long press
void longPressCallback() {
  quick_redraw = true;
  redraw_screen = true;

  Serial.println("long press");
  #ifdef SINGLE_SWITCH
    if (screen_mode != MODE_SETTING) {
      if (screen_mode == MODE_GPSCONST)
        gps_getposition_mode();
      screen_mode = MODE_SETTING;
      cursorLine = 0;
      selectedLine = -1;
      tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
    } else {
      //exit
      if (cursorLine == SETTING_LINES - 1) {
        screen_mode = MODE_MAP;
      } else if (cursorLine == 3) {
        Serial.println("GPS CONST MODE");
        gps_constellation_mode();
        screen_mode = MODE_GPSCONST;
      } else if (cursorLine == 4) {
        Serial.println("MAP DETAIL MODE");
        screen_mode = MODE_MAPLIST;
      } else {
        if (selectedLine == -1) {
          //Entering changing value mode.
          selectedLine = cursorLine;
        } else {
          //exiting changing value mode.
          selectedLine = -1;
        }
      }
    }
  #endif
}

// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback);
Button sw_up(SW_UP, shortPressCallback_up);
Button sw_down(SW_DOWN, shortPressCallback_down);

void setup_switch() {
    pinMode(sw_push.getPin(), INPUT_PULLUP); // This must be after setup tft for some reason of library TFT_eSPI.
    pinMode(sw_up.getPin(), INPUT_PULLUP); // This must be after setup tft for some reason of library TFT_eSPI.
    pinMode(sw_down.getPin(), INPUT_PULLUP); // This must be after setup tft for some reason of library TFT_eSPI.
}

void switch_handling(){
  sw_push.read();
  sw_up.read();
  sw_down.read();
}

void setup(void) {
  Serial.begin(38400);
  Serial.print(F("SETUP INIT"));



  setup_switch();
  Serial.print(F("GPS SETUP"));
  gps_setup();
  setup_sd();//sd init must be before tft for somereason of library TFT_eSPI
  setup_tft();



  startup_demo_tft();


  redraw_screen = true;
  quick_redraw = true;
  const char* inittext = "INIT";
  log_sd(inittext);
  Serial.print(F("SETUP DONE"));
}

void reset_degpersecond(){
  float track = get_gps_truetrack();
  for (int i = 1; i < numSamples; i++) {
    upward_samples[i] = track;
  }
  degpersecond = 0;
  bank_warning = false;
}
void update_degpersecond() {
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
        float degchange = (upward_samples[i] - upward_samples[i - 1]);
        if (degchange < -180) {
          degchange += 360;
        } else if (degchange > 180) {
          degchange -= 360;
        }
        float difference = degchange / (sampleInterval / 1000.0);  // deg/s
        totalDifference += difference;
      }
      // Calculate the average differential
      degpersecond = totalDifference / (numSamples - 1);
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

void check_bankwarning() {
  unsigned long currentTime = millis();
  last_newtrack_time = currentTime;

  //Do not trigger bank warning if speed is below ...
  if (get_gps_mps() < 2.0) {
    if (bank_warning) {
      redraw_screen = true;
      bank_warning = false;
    }
    return;
  }


  if (abs(degpersecond) > 3.0 && get_gps_mps() > 2) {
    bank_warning = true;
  } else {
    if (bank_warning) {
      redraw_screen = true;
      bank_warning = false;
    }
  }
}

void debug_print(const char* inp) {
  Serial.println(inp);
}


void loop() {
  switch_handling();
  bool newmsg_received = gps_loop(screen_mode == MODE_GPSCONST);

  if (screen_mode == MODE_SETTING) {
    draw_setting_mode(redraw_screen, selectedLine, cursorLine);
    redraw_screen = false;
    return;
  }
  if (screen_mode == MODE_GPSCONST) {
    draw_ConstellationDiagram(redraw_screen);
    redraw_screen = false;
    return;
  }

  if (screen_mode == MODE_MAPLIST) {
    draw_maplist_mode(redraw_screen);
    redraw_screen = false;
    return;
  }


  update_degpersecond();

  // (GPSの受信が完了したタイミング or GPSが不作動) && (画面更新インターバルが経過している or 強制描画タイミング)
  // TFT のSPI 通信とGPS module RX Interruptのタイミング競合によりGPS受信失敗するので、GPS受信直後にTFT更新を限定している。
  bool redraw_map = ((newmsg_received || !get_gps_connection()) && (millis() - last_time_gpsdata > SCREEN_INTERVAL)) || quick_redraw;
  if (redraw_map) {
    float new_truetrack = get_gps_truetrack();
    float new_lat = get_gps_lat();
    float new_long = get_gps_long();
    last_time_gpsdata = millis();
    quick_redraw = false;
    //char logdata[30];
    //sprintf(logdata,"%d,%03d,%.1f,%s",millis(),new_truetrack,get_gps_mps(),get_gps_fix()?"fix":"nil");
    //log_sd(logdata);
    

    //画面上の方向設定
    float drawupward_direction = truetrack_now;
    if (is_northupmode()) {
      drawupward_direction = 0;
    }

    //redraw_map = 古い線を白色で上書きして、新しく書き直す。redraw_screen = 画面全てを白で塗りつぶしたあとに塗り直す。
    bool redraw_map = redraw_screen;
    if(new_truetrack != truetrack_now || new_lat != lat_now){
      check_bankwarning();
      redraw_map = true;
    }


    if (millis() - lastfresh_millis > SCREEN_FRESH_INTERVAL || redraw_screen) {
      lastfresh_millis = millis();
      clean_display();
      redraw_screen = true;
    }

    if (redraw_map || redraw_screen) {
      truetrack_now = new_truetrack;
      lat_now = new_lat;
      //古い線の削除。必要に応じて、白塗りつぶしを行った場合は、redraw_screen = true で上書きして帰ってくる。
      //clean_display(redraw_screen);

      clean_map();

      erase_triangle();
      


      if(scale > SCALE_JAPAN){
        if (check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
          draw_Biwako(new_lat, new_long, scale, drawupward_direction);
        } else if (check_within_latlon(0.6, 0.6, new_lat, OSAKA_LAT, new_long, OSAKA_LON)) {
          draw_Osaka(new_lat, new_long, scale, drawupward_direction);
        } else if (check_within_latlon(0.6, 0.6, new_lat, SHINURA_LAT, new_long, SHINURA_LON)) {
          draw_Shinura(new_lat, new_long, scale, drawupward_direction);
        }
        draw_ExtraMaps(new_lat, new_long, scale, drawupward_direction);
      }else{
        //near japan.
        if(check_within_latlon(20,40,new_lat, 35, new_long, 138)){
          draw_Japan(new_lat, new_long, scale, drawupward_direction);
        }
      }

      if (get_demo_biwako()) {
        tft.setTextColor(COLOR_BLACK,COLOR_WHITE);
        tft.setCursor(SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2 + 20);
        tft.setTextSize(3);
        tft.print("DEMO");
      }

      draw_km_circle(scale);
      draw_triangle();
      redraw_compass(drawupward_direction,COLOR_BLACK, COLOR_WHITE);
    }

    //地図が更新されていない時でも、更新するもの。

    if (bank_warning) {
      draw_bankwarning();
    }
    draw_nomapdata();
    draw_degpersecond(degpersecond);
    draw_gpsinfo();
    draw_sdinfo();

    //更新終了
    redraw_screen = false;
    Serial.print("Redraw time ms:");
    Serial.println(millis()-last_time_gpsdata);
  }
}