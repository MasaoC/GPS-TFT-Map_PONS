// GPS_TFT_map. PONS v5 ( Pilot oriented navigation system for HPA )
// Author: MasaoC  (@masao_mobile)
// Last update date: 2025/07/29

#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "sound.h"
#include "hardware/adc.h"

// we need to do bool core1_separate_stack = true; to avoid stack running out.
// (Likely due to drawWideLine from TFT-eSPI consuming alot of stack.)
bool core1_separate_stack = true;  //DO NOT REMOVE THIS LINE.

unsigned long screen_update_time = 0;  // 最後に画面更新した時間 millis()
bool redraw_screen = false;            //次の描画では、黒塗りして新たに画面を描画する

const int NUM_SAMPLES = 4;             // Number of samples for deg/s
float truetrack_samples[NUM_SAMPLES];  // Array to store truetrack values
int sampleIndex = 0;                   // Index to keep track of current sample
float degpersecond = 0;                // The calculated average differential

int screen_mode = MODE_MAP;
int detail_page = 0;
double scalelist[6];
double scale;

// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
int lastload_zoomlevel;
int course_warning_index = 0;                    //From 0 to 900
unsigned long last_course_warning_time = 0;      //millis
unsigned long last_destination_toofar_time = 0;  //millis
double steer_angle = 0.0;                        //-180 to 180.

//次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画するフラグ。BMPロード完了時にフラグが立つ。
volatile int scaleindex = 3;
volatile int sound_volume = 50;

extern volatile bool loading_sddetail;
extern bool sd_detail_loading_displayed;

void reset_degpersecond();
void update_degpersecond(int true_track);
void check_destination_toofar();
void update_course_warning(float degpersecond);
void shortPressCallback();
void longPressCallback();
// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback);

//===============SET UP CORE0=================
void setup(void) {
  Serial.begin(38400);

#ifndef RELEASE
  while (!Serial) {
    delay(10);
  }
#endif

  //setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  setup_tft();

  adc_gpio_init(BATTERY_PIN);
  pinMode(USB_DETECT, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);

  startup_demo_tft();
  gps_setup();  //To avoid overflow, after tft setup recommend.

  scalelist[0] = SCALE_EXSMALL_GMAP;  //pixelsPerDegreeLat(5,35)/KM_PER_DEG_LAT;
  scalelist[1] = SCALE_SMALL_GMAP;    //pixelsPerDegreeLat(7,35)/KM_PER_DEG_LAT;
  scalelist[2] = SCALE_MEDIUM_GMAP;   //pixelsPerDegreeLat(9,35)/KM_PER_DEG_LAT;
  scalelist[3] = SCALE_LARGE_GMAP;    //pixelsPerDegreeLat(11,35)/KM_PER_DEG_LAT;
  scalelist[4] = SCALE_EXLARGE_GMAP;  //pixelsPerDegreeLat(13,35)/KM_PER_DEG_LAT;
  scalelist[5] = 200.0;
  scale = scalelist[scaleindex];
  redraw_screen = true;
  enqueueTask(createLogSdTask("SETUP DONE"));
}


//===============SET UP CORE1=================
void setup1(void) {
  Serial.begin(38400);
  mutex_init(&taskQueueMutex);
  init_destinations();
  setup_sound();

  //====core1_separate_stack = true の時、Serial.printがないとSDが動かない。詳細不明だが、消すな！
  Serial.println("");  //消すな
  // call to Serial.println() might force the Arduino runtime or the RP2350’s FreeRTOS (used in the RP2040/RP2350 Arduino core for dual-core support) to fully initialize Core1’s context,
  // ensuring that the SD library’s internal state to set up.
  // Without this call, Core1 might attempt to initialize the SD card before its runtime environment is fully ready, especially with a separate stack.
  //====

  setup_sd(5);

  if (good_sd()) {
    enqueueTask(createPlayWavTask("wav/opening.wav"));
  } else {
    enqueueTask(createPlayMultiToneTask(500, 150, 10));
  }
}



//===============MAIN LOOP CORE0=================
void loop() {
  //switch handling
  sw_push.read();
  gps_loop(0);

#ifndef RELEASE
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
#endif



  if ((millis() - screen_update_time > SCREEN_FRESH_INTERVAL)) {
    redraw_screen = true;
  }


  if (screen_mode == MODE_SETTING) {
    if (redraw_screen) {
      draw_setting_mode(selectedLine, cursorLine);
    }
  } else if (screen_mode == MODE_GPSDETAIL) {
    if (redraw_screen)
      draw_gpsdetail(detail_page);
  } else if (screen_mode == MODE_SDDETAIL) {
    if (sd_detail_loading_displayed && !loading_sddetail)
      redraw_screen = true;
    if (redraw_screen)
      draw_sddetail(detail_page);
  } else if (screen_mode == MODE_MAPLIST) {
    if (redraw_screen)
      draw_maplist_mode(detail_page);
  } else if (screen_mode == MODE_MAP) {
    bool new_gps_info = gps_new_location_arrived();
    if (new_gps_info) {
      // Automatic destination change for AUTO 10KM mode.
      if (destination_mode == DMODE_AUTO10K) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        double distance_frm_destination = calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon);
        if (auto10k_status == AUTO10K_AWAY) {

          if (distance_frm_destination > 10.475) {  //10.975 だけど、500mの誤差を引いておく。
            auto10k_status = AUTO10K_INTO;
            enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 2));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
            enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
          }
        }
        if (auto10k_status == AUTO10K_INTO && distance_frm_destination < 1.5) {  //折り返し用。だけど、500mの誤差を引いておく。
          auto10k_status = AUTO10K_AWAY;
          enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 2));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
          enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
          enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
        }
      }
      check_destination_toofar();
    }

    // Called only once per second.
    if (newcourse_arrived) {


      int ttrack = get_gps_truetrack();
      update_degpersecond(ttrack);
      update_tone(degpersecond);
      update_course_warning(degpersecond);
      steer_angle = (magc - 8) - ttrack;
      if (steer_angle < -180) {
        steer_angle += 360;
      } else if (steer_angle > 180) {
        steer_angle -= 360;
      }

      newcourse_arrived = false;
    }


    // BMPロード完了 or GPSの情報が更新された。
    if (new_gmap_ready || new_gps_info) {
      redraw_screen = true;
    }

    // GPSデータを受信した時と、bmpロード完了した時、両方でcallされるので毎秒二回は redraw_screen = true になる。
    if (redraw_screen) {
      float new_truetrack = get_gps_truetrack();
      double new_lat = get_gps_lat();
      double new_long = get_gps_lon();
      set_new_location_off();

      //画面上の方向設定
      float drawupward_direction = new_truetrack;
      if (is_northupmode()) {
        drawupward_direction = 0;
      }

      nav_update();  // MC や dist を更新する。
      draw_header();

      new_gmap_ready = false;

      clean_backscreen();


      int zoomlevel = 0;
      if (scaleindex == 0) zoomlevel = 5;   //SCALE_EXSMALL_GMAP
      if (scaleindex == 1) zoomlevel = 7;   //SCALE_SMALL_GMAP
      if (scaleindex == 2) zoomlevel = 9;   //SCALE_MEDIUM_GMAP
      if (scaleindex == 3) zoomlevel = 11;  //SCALE_LARGE_GMAP
      if (scaleindex == 4) zoomlevel = 13;  //SCALE_EXLARGE_GMAP

      bool gmap_drawed = false;
      if (gmap_loaded_active)
        gmap_drawed = draw_gmap(drawupward_direction);

      // If new gps
      if (new_gps_info || lastload_zoomlevel != zoomlevel) {
        lastload_zoomlevel = zoomlevel;
        //If loadImagetask already running in Core1, abort.
        enqueueTaskWithAbortCheck(createLoadMapImageTask(new_lat, new_long, zoomlevel));
      }

      if (scale > SCALE_SMALL_GMAP) {
        if (check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
          draw_Biwako(new_lat, new_long, scale, drawupward_direction, gmap_drawed);
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

      gps_loop(4);

      draw_track(new_lat, new_long, scale, drawupward_direction);

      if (currentdestination != -1 && currentdestination < destinations_count) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        if (destination_mode == DMODE_FLYINTO)
          draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction, 5);
        else if (destination_mode == DMODE_FLYAWAY)
          draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
        else if (destination_mode == DMODE_AUTO10K) {
          if (auto10k_status == AUTO10K_AWAY)
            draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
          else if (auto10k_status == AUTO10K_INTO)
            draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction, 5);
        }
      }
      gps_loop(5);
      draw_compass(drawupward_direction, COLOR_BLACK);
      draw_degpersec(degpersecond);
      if (get_demo_biwako()) {
        draw_demo_biwako();
      }
      draw_km_distances(scale);

      draw_triangle(new_truetrack, steer_angle);

      //10秒間 warning 表示
      if (millis() - last_course_warning_time < 10000 && millis() > 10000) {
        draw_course_warning(steer_angle);
      }

      if (!gmap_drawed) {
        draw_nogmap(scale);
      }

      draw_headertext(degpersecond);
      draw_map_footer();
      draw_nomapdata();
      gps_loop(6);
      push_backscreen();
      draw_footer();
    }
  } else {
    DEBUGW_PLN(20250510, "ERR screen mode");
  }

  if (redraw_screen) {
    //更新終了
    redraw_screen = false;
    screen_update_time = millis();
  }
}


//===============MAIN LOOP CORE1=================
void loop1() {
  loop_sound();
  if (dequeueTask(&currentTask)) {
    switch (currentTask.type) {
      case TASK_SAVE_SETTINGS:
        saveSettings();
        break;
      case TASK_PLAY_WAV:
        startPlayWav(currentTask.playWavArgs.wavfilename, currentTask.playWavArgs.priority);
        break;
      case TASK_PLAY_MULTITONE:
        playTone(currentTask.playMultiToneArgs.freq, currentTask.playMultiToneArgs.duration, currentTask.playMultiToneArgs.counter, currentTask.playMultiToneArgs.priority);
        break;
      case TASK_INIT_SD:
        setup_sd(1);
        break;
      case TASK_BROWSE_SD:
        browse_sd(currentTask.pagenum);
        break;
      case TASK_LOAD_REPLAY:
        load_replay();
        break;
      case TASK_INIT_REPLAY:
        init_replay();
        break;
      case TASK_LOG_SD:
        log_sd(currentTask.logText);
        break;
      case TASK_LOG_SDF:
        log_sdf(currentTask.logSdfArgs.format, currentTask.logSdfArgs.buffer);
        break;
      case TASK_SAVE_CSV:
        saveCSV(
          currentTask.saveCsvArgs.latitude, currentTask.saveCsvArgs.longitude,
          currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.mtrack, currentTask.saveCsvArgs.altitude,
          currentTask.saveCsvArgs.numsats, currentTask.saveCsvArgs.voltage,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second);
        break;
      case TASK_LOAD_MAPIMAGE:
        load_mapimage(
          currentTask.loadMapImageArgs.center_lat, currentTask.loadMapImageArgs.center_lon,
          currentTask.loadMapImageArgs.zoomlevel);
        break;
    }
    currentTask.type = TASK_NONE;
  } else {
    // No tasks, optionally sleep or yield
    delay(10);
  }
}
extern int max_page;  // Global variable to store maximum page number

//==========BUTTON==========
//Callback function for short press
void shortPressCallback() {
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");


  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = (cursorLine + 1) % setting_size;
    } else {
      menu_settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_SDDETAIL) {
    detail_page++;
    loading_sddetail = true;
    if (max_page <= 0)
      enqueueTask(createBrowseSDTask(0));
    else
      enqueueTask(createBrowseSDTask(detail_page % (max_page + 1)));
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  } else {
    gmap_loaded_active = false;
    scaleindex = (scaleindex + 1) % (sizeof(scalelist) / sizeof(scalelist[0]));
    scale = scalelist[scaleindex];
  }
}

// Callback function for long press
void longPressCallback() {
  redraw_screen = true;

  if (screen_mode != MODE_SETTING) {
    if (screen_mode == MODE_GPSDETAIL)
      gps_getposition_mode();
    screen_mode = MODE_SETTING;
    cursorLine = 0;
    selectedLine = -1;
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
  } else {
    if (selectedLine == -1 && menu_settings[cursorLine].CallbackEnter != nullptr) {
      menu_settings[cursorLine].CallbackEnter();
      selectedLine = cursorLine;
    } else {
      if (selectedLine == -1) {
        //Entering changing value mode.
        selectedLine = cursorLine;
      } else {
        //exiting changing value mode.
        if (menu_settings[cursorLine].CallbackExit != nullptr)
          menu_settings[cursorLine].CallbackExit();
        selectedLine = -1;
      }
    }
  }
}

// reset deg per second to 0.
void reset_degpersecond() {
  float track = get_gps_truetrack();
  for (int i = 1; i < NUM_SAMPLES; i++) {
    truetrack_samples[i] = track;
  }
  degpersecond = 0;
}

// deg per second being updated. call this once a second.
void update_degpersecond(int true_track) {
  // Update the truetrack value
  // This is where you would get the new truetrack value from your sensor
  truetrack_samples[sampleIndex] = true_track;
  // Calculate the differential if we have enough samples
  if (sampleIndex >= NUM_SAMPLES - 1) {
    float totalDifference = 0;
    // Calculate the differences between consecutive samples
    for (int i = 1; i < NUM_SAMPLES; i++) {
      float degchange = (truetrack_samples[i] - truetrack_samples[i - 1]);
      if (degchange < -180) {
        degchange += 360;
      } else if (degchange > 180) {
        degchange -= 360;
      }
      float difference = degchange;  // deg/s
      totalDifference += difference;
    }
    // Calculate the average differential
    degpersecond = totalDifference / (NUM_SAMPLES - 1);
    // Shift samples to make room for new ones
    for (int i = 1; i < NUM_SAMPLES; i++) {
      truetrack_samples[i - 1] = truetrack_samples[i];
    }
    // Adjust the sample index to the last position
    sampleIndex = NUM_SAMPLES - 2;
  }
  // Move to the next sample
  sampleIndex++;
}



void check_destination_toofar() {
  double destlat = extradestinations[currentdestination].cords[0][0];
  double destlon = extradestinations[currentdestination].cords[0][1];
  if (calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon) > 100.0) {
    if (millis() - last_destination_toofar_time > 1000 * 60) {  //60sec
      last_destination_toofar_time = millis();
      enqueueTask(createPlayWavTask("wav/destination_toofar.wav", 1));
    }
  }
}

//一秒に一回まで。
void update_course_warning(float degpersecond) {
  //移動していない時,GPSロスト時は発動しない。
  if (get_gps_mps() < 2 || get_gps_numsat() == 0) {
    course_warning_index = 0;
  }
  //正しい方向に変化している時はindexを減らす。
  else if ((degpersecond > 0.5 && steer_angle > 0) || (degpersecond < -0.5 && steer_angle < 0)) {
    course_warning_index -= 15 * abs(degpersecond);  //修正速度に応じ、7以上のindexが減る。3deg/sの時、indexは45減る。

  }
  //正しい方向に修正されていない、かつ15度以上ずれている。
  else if (abs(steer_angle) > 15) {
    course_warning_index += min(abs(steer_angle), 90);  //15以上、90を最大値として、indexに加算する。
  }
  //15度未満のズレ、index 0 reset。
  else {
    course_warning_index = 0;
  }

  // range 0-900
  if (course_warning_index > 900)
    course_warning_index = 900;
  else if (course_warning_index < 0)
    course_warning_index = 0;

  //15度の修正されないズレは、15/900=60秒でwarning。90度以上の修正されないズレは、10秒でwarning。ただしwarningは、30秒に一度をmax回数とする。
  if (course_warning_index >= 900 && millis() - last_course_warning_time > 30000) {
    last_course_warning_time = millis();
    course_warning_index = 0;
    enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 2));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
    enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2));
    if (steer_angle > 0)
      enqueueTask(createPlayWavTask("wav/course_right.wav", 2));
    else
      enqueueTask(createPlayWavTask("wav/course_left.wav", 2));
  }
}