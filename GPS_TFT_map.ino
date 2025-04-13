#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "sound.h"

#define SCALE_EXLARGE_GMAP 52.32994872   //zoom13
#define SCALE_LARGE_GMAP 13.08248718     //zoom11
#define SCALE_MEDIUM_GMAP 3.2706218      //zoom9
#define SCALE_SMALL_GMAP 0.81765545      //zoom7
#define SCALE_EXSMALL_GMAP 0.2044138625  //zoom5

unsigned long last_newtrack_time = 0;
float truetrack_now = 0;
float lat_now = 0;
unsigned long last_time_gpsdata = 0;  //GPSを最後に受信した時間 millis()
bool redraw_screen = false;           //次の描画では、黒塗りして新たに画面を描画する
volatile bool quick_redraw = false;   //次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画する
bool bank_warning = false;

const int sampleInterval = 500;    // Sample interval in milliseconds
const int numSamples = 4;          // Number of samples to take over 2 seconds
float upward_samples[numSamples];  // Array to store truetrack values
unsigned long lastSampleTime = 0;  // Time when the last sample was taken
int sampleIndex = 0;               // Index to keep track of current sample
float degpersecond = 0;            // The calculated average differential


int destination_mode = DMODE_FLYAWAY;

int screen_mode = MODE_MAP;
int detail_page = 0;
int scaleindex = 3;
const double scalelist[] = { SCALE_EXSMALL_GMAP, SCALE_SMALL_GMAP, SCALE_MEDIUM_GMAP, SCALE_LARGE_GMAP, SCALE_EXLARGE_GMAP, 180.0f };
double scale = scalelist[scaleindex];

// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
unsigned long lastfresh_millis = 0;

int sound_len = 50;

extern int setting_size;


double lastload_lat;
double lastload_lon;
int lastload_zoomlevel;

// Callback function for short press
void shortPressCallback() {
  quick_redraw = true;
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");


#ifdef SINGLE_SWITCH
  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = (cursorLine + 1) % setting_size;
    } else {
      settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  } else {
    scale = scalelist[++scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
    gmap_loaded = false;
  }
#else
  if (screen_mode != MODE_SETTING) {
    if (screen_mode == MODE_GPSDETAIL)
      gps_getposition_mode();
    screen_mode = MODE_SETTING;
    cursorLine = 0;
    selectedLine = -1;
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
  } else {
    //exit
    if (cursorLine == setting_size - 1) {
      screen_mode = MODE_MAP;
    } else if (cursorLine == 3) {
      detail_page = 0;
      DEBUG_PLN(20240801, "GPS DETAIL MODE");
      gps_constellation_mode();
      screen_mode = MODE_GPSDETAIL;
    } else if (cursorLine == 4) {
      detail_page = 0;
      DEBUG_PLN(20240801, "MAPLIST MODE");
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
  DEBUG_PLN(20240801, "short press up");
  if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  } else if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {                //No active selected line.
      cursorLine = (cursorLine + 1) % setting_size;
    } else if (selectedLine == 0) {
      tft_change_brightness(1);
    } else if (selectedLine == 1) {
      toggle_demo_biwako();
      reset_degpersecond();
    } else if (selectedLine == 2) {
      toggle_mode();
    } else if (selectedLine == 3) {
      screen_mode = MODE_GPSDETAIL;
    }
  } else {
    scale = scalelist[++scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
    gmap_loaded = false;
  }
}


// Callback function for short press
void shortPressCallback_down() {
  quick_redraw = true;
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press down");
  if (screen_mode == MODE_MAPLIST) {
    detail_page--;
  } else if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {                //No active selected line.
      cursorLine = mod(cursorLine - 1, setting_size);
    } else if (selectedLine == 0) {
      tft_change_brightness(-1);
    } else if (selectedLine == 1) {
      toggle_demo_biwako();
      reset_degpersecond();
    } else if (selectedLine == 2) {
      toggle_mode();
    } else if (selectedLine == 3) {
      screen_mode = MODE_GPSDETAIL;
    }
  } else {
    scale = scalelist[--scaleindex % (sizeof(scalelist) / sizeof(scalelist[0]))];
    gmap_loaded = false;
  }
}
// Callback function for long press
void longPressCallback() {
  quick_redraw = true;
  redraw_screen = true;

  DEBUG_PLN(20240801, "long press");
#ifdef SINGLE_SWITCH
  if (screen_mode != MODE_SETTING) {
    if (screen_mode == MODE_GPSDETAIL)
      gps_getposition_mode();
    screen_mode = MODE_SETTING;
    cursorLine = 0;
    selectedLine = -1;
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
  } else {
    if (selectedLine == -1 && settings[cursorLine].CallbackEnter != nullptr) {
      settings[cursorLine].CallbackEnter();
      selectedLine = cursorLine;
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
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  pinMode(sw_up.getPin(), INPUT_PULLUP);    // This must be after setup tft for some reason of library TFT_eSPI.
  pinMode(sw_down.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
}

void switch_handling() {
  sw_push.read();
  sw_up.read();
  sw_down.read();
}


void setup(void) {
  Serial.begin(38400);

  #ifndef RELEASE
  while (!Serial){
    //max wait for 5 seconds.
    if(millis() > 5000){
      break;
    }
  };
  #endif

  setup_switch();

  // Enqueue init_sd task
  //Task task;
  //task.type = TASK_INIT_SD;
  //enqueueTask(task);

  //setup_sd();//sd init must be before tft for somereason of library TFT_eSPI
  setup_tft();

  gps_setup();
  startup_demo_tft();

  redraw_screen = true;
  quick_redraw = true;


  // Enqueue log_sd task
  Task task;
  task.type = TASK_LOG_SD;
  task.logText = "SETUP DONE";
  enqueueTask(task);

  DEBUG_PLN(20240801, "SETUP DONE");
}

void reset_degpersecond() {
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
  if (get_gps_mps() <= 2.0) {
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




void loop() {
  switch_handling();
  bool longdraw_allowed = gps_loop();
  #ifndef RELEASE
  if(Serial.available()){
    GPS_SERIAL.write(Serial.read());
  }
  #endif

  if (screen_mode == MODE_SETTING) {
    draw_setting_mode(redraw_screen, selectedLine, cursorLine);
    redraw_screen = false;
    return;
  }
  if (screen_mode == MODE_GPSDETAIL) {
    draw_gpsdetail(redraw_screen, detail_page);
    redraw_screen = false;
    return;
  }

  if (screen_mode == MODE_MAPLIST) {
    draw_maplist_mode(redraw_screen, detail_page);
    redraw_screen = false;
    return;
  }
  update_degpersecond();
  check_bankwarning();

  draw_loading_image();

  // (GPSの受信が完了したタイミング or GPSが不作動) && (画面更新インターバルが経過している) or 強制描画タイミング(スイッチ操作など)
  // TFT のSPI 通信とGPS module RX Interruptのタイミング競合によりGPS受信失敗するので、GPS受信直後にTFT更新を限定している。
  bool redraw_map = ((longdraw_allowed || !get_gps_connection()) && (millis() - last_time_gpsdata > SCREEN_INTERVAL)) || quick_redraw;
  if (new_gmap_loaded && longdraw_allowed) {
    redraw_map = true;
  }
  if (redraw_map) {
    DEBUG_PLN(20240828, "redraw map");
    quick_redraw = false;
    new_gmap_loaded = false;
    float new_truetrack = get_gps_truetrack();
    double new_lat = get_gps_lat();
    double new_long = get_gps_lon();
    last_time_gpsdata = millis();

    //画面上の方向設定
    float drawupward_direction = truetrack_now;
    if (is_northupmode()) {
      drawupward_direction = 0;
    }

    //redraw_screen = 古い線を白色で上書きして、新しく書き直す。redraw_screen = 画面全てを白で塗りつぶしたあとに塗り直す。
    if (new_truetrack != truetrack_now || new_lat != lat_now) {
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

      clean_map();

      //google map load成功している間は、古いものを削除する必要はない。
      if (!gmap_loaded) {
        erase_triangle();
      }


      int zoomlevel = 0;
      if (scale == SCALE_EXSMALL_GMAP) zoomlevel = 5;
      if (scale == SCALE_SMALL_GMAP) zoomlevel = 7;
      if (scale == SCALE_MEDIUM_GMAP) zoomlevel = 9;
      if (scale == SCALE_LARGE_GMAP) zoomlevel = 11;
      if (scale == SCALE_EXLARGE_GMAP) zoomlevel = 13;
      //load_mapimage(new_lat,new_long,zoomlevel);
      if (gmap_loaded) {
        if(is_trackupmode())
          gmap_sprite.pushRotated(-drawupward_direction);
        else
          gmap_sprite.pushSprite(0, 40);
        DEBUG_PLN(20240828, "pushed gmap");
      }
      //If loadImagetask already running in Core1, abort.
      if(lastload_lat != new_lat || lastload_lon != new_long || lastload_zoomlevel != zoomlevel){
        lastload_lat = new_lat;
        lastload_lon = new_long;
        lastload_zoomlevel = zoomlevel;
        enqueueTaskWithAbortCheck(createLoadMapImageTask(new_lat, new_long, zoomlevel));
      }


      if (scale > SCALE_SMALL_GMAP) {
        if (check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
          draw_Biwako(new_lat, new_long, scale, drawupward_direction);
        } else if (check_within_latlon(0.6, 0.6, new_lat, OSAKA_LAT, new_long, OSAKA_LON)) {
          draw_Osaka(new_lat, new_long, scale, drawupward_direction);
        } else if (check_within_latlon(0.6, 0.6, new_lat, SHINURA_LAT, new_long, SHINURA_LON)) {
          draw_Shinura(new_lat, new_long, scale, drawupward_direction);
        }
        draw_ExtraMaps(new_lat, new_long, scale, drawupward_direction);
      } else {
        //near japan.
        if (check_within_latlon(20, 40, new_lat, 35, new_long, 138)) {
          draw_Japan(new_lat, new_long, scale, drawupward_direction);
        }
      }

      draw_track(new_lat, new_long, scale, drawupward_direction);

      if(currentdestination != -1 && currentdestination < destinations_count){
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        if(destination_mode == DMODE_FLYINTO) 
          draw_flyinto(destlat, destlon, new_lat, new_long, scale, drawupward_direction,2);
        else if(destination_mode == DMODE_FLYAWAY) 
          draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
      }

      if (get_demo_biwako()) {
        tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
        tft.setCursor(SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2 + 20);
        tft.setTextSize(3);
        tft.print("DEMO");
      }

      draw_km_circle(scale);

      //google map load成功している間は、古いものを削除する必要はない。
      if (!gmap_loaded) {
        redraw_compass(drawupward_direction, COLOR_BLACK, COLOR_WHITE);
      } else {
        draw_compass(drawupward_direction, COLOR_BLACK);
      }
      draw_triangle();
    }

    //地図が更新されていない時でも、更新するもの。

    if (bank_warning) {
      draw_bankwarning();
    }

    #ifdef PIN_TONE
    update_tone(degpersecond,sound_len);
    #endif
    
    draw_nomapdata();
    draw_degpersecond(degpersecond);  //after gpsinfo preferred
    draw_gpsinfo();
    draw_sdinfo();
    if (!gmap_loaded && !isTaskRunning(TASK_LOAD_MAPIMAGE)) {
      draw_nogmap();
    }

    //更新終了
    redraw_screen = false;

    DEBUG_P(20250413, "C0 free/used heap and free stack/Pointer:");
    DEBUG_P(20250413, rp2040.getFreeHeap());
    DEBUG_P(20250413, "/");
    DEBUG_P(20250413, rp2040.getUsedHeap());
    DEBUG_P(20250413, "/");
    DEBUG_P(20250413, rp2040.getFreeStack());
    DEBUG_P(20250413, "/");
    DEBUG_PLN(20250413, rp2040.getStackPointer());
  }
}