// GPS_TFT_map. PONS v5
// Author: Masao Chiguchi @C 2025

#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "sound.h"
#include "hardware/adc.h"

// we need to do bool core1_separate_stack = true; for stack running out.
bool core1_separate_stack = true;//DO NOT REMOVE THIS LINE.

unsigned long last_newtrack_time = 0;
float truetrack_now = 0;
unsigned long last_screen_update = 0;  //GPSを最後に受信した時間 millis()
bool redraw_screen = false;           //次の描画では、黒塗りして新たに画面を描画する
volatile bool quick_redraw = false;   //次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画する
bool bank_warning = false;

const int NUM_SAMPLES = 4;          // Number of samples to take over 2 seconds
float upward_samples[NUM_SAMPLES];  // Array to store truetrack values
unsigned long lastSampleTime = 0;  // Time when the last sample was taken
int sampleIndex = 0;               // Index to keep track of current sample
float degpersecond = 0;            // The calculated average differential

int screen_mode = MODE_MAP;
int detail_page = 0;
volatile int scaleindex = 3;
double scalelist[6];
double scale;

// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
unsigned long lastfresh_millis = 0;
volatile int sound_volume = 50;
int lastload_zoomlevel;


void shortPressCallback();
void longPressCallback();

// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback);

int course_warning_index = 0;
unsigned long last_course_warning_time = 0;

double steer_angle = 0.0;

void reset_degpersecond();
void update_degpersecond();
void update_course_warning(float degpersecond);

extern int setting_size;
extern bool newcourse_arrived;

//===============SET UP CORE0=================
void setup(void) {
  Serial.begin(38400);

  #ifndef RELEASE
  while(!Serial){
    delay(10);
  }
  #endif

  //setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  setup_tft();

  adc_gpio_init(BATTERY_PIN);  // Initialize GPIO26 as ADC
  pinMode(24,INPUT);
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);

  gps_setup();


  startup_demo_tft();


  scalelist[0] = SCALE_EXSMALL_GMAP;//pixelsPerDegreeLat(5,35)/KM_PER_DEG_LAT;
  scalelist[1] = SCALE_SMALL_GMAP;//pixelsPerDegreeLat(7,35)/KM_PER_DEG_LAT;
  scalelist[2] = SCALE_MEDIUM_GMAP;//pixelsPerDegreeLat(9,35)/KM_PER_DEG_LAT;
  scalelist[3] = SCALE_LARGE_GMAP;//pixelsPerDegreeLat(11,35)/KM_PER_DEG_LAT;
  scalelist[4] = SCALE_EXLARGE_GMAP;//pixelsPerDegreeLat(13,35)/KM_PER_DEG_LAT;
  scalelist[5] = 200.0;
  scale = scalelist[scaleindex];
  redraw_screen = true;
  quick_redraw = true;
  enqueueTask(createLogSdTask("SETUP DONE"));
}


//===============SET UP CORE1=================
void setup1(void){
  Serial.begin(38400);
  mutex_init(&taskQueueMutex);
  init_destinations();
  setup_sound();

  //====core1_separate_stack = true の時、Serial.printがないとSDが動かない。詳細不明だが、消すな！
  Serial.println("");//消すな
  // call to Serial.println() might force the Arduino runtime or the RP2350’s FreeRTOS (used in the RP2040/RP2350 Arduino core for dual-core support) to fully initialize Core1’s context,
  // ensuring that the SD library’s internal state to set up.
  // Without this call, Core1 might attempt to initialize the SD card before its runtime environment is fully ready, especially with a separate stack.
  //====
  
  setup_sd(5);

  if(good_sd()){
      enqueueTask(createPlayWavTask( "wav/opening.wav"));
  }else{
      enqueueTask(createPlayMultiToneTask(500, 150, 10));
  }
}

//===============MAIN LOOP CORE0=================
void loop() {
  //switch handling
  sw_push.read();
  gps_loop(0);
  
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

  if (screen_mode == MODE_SDDETAIL) {
    draw_sddetail(redraw_screen, detail_page);
    redraw_screen = false;
    return;
  }
  if (screen_mode == MODE_MAPLIST) {
    draw_maplist_mode(redraw_screen, detail_page);
    redraw_screen = false;
    return;
  }

  bool new_gps_info = gps_newdata_arrived();

  if(new_gps_info && destination_mode == DMODE_AUTO10K){
    double distance_frm_plathome = calculateDistance(get_gps_lat(), get_gps_lon(), PLA_LAT, PLA_LON);
    if(auto10k_status == AUTO10K_AWAY){
      if(distance_frm_plathome > 10.0){//10.54 だけど、540mの誤差を引いておく。
        auto10k_status = AUTO10K_INTO;
        enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793,500,1));
        enqueueTask(createPlayMultiToneTask(3136,500,1));
        enqueueTask(createPlayMultiToneTask(2793,500,1));
        enqueueTask(createPlayMultiToneTask(3136,500,1));
        enqueueTask(createPlayWavTask("wav/destination_change.wav",3));
      }
    }
    if(auto10k_status == AUTO10K_INTO && distance_frm_plathome < 1.0){//折り返し用。
      auto10k_status = AUTO10K_AWAY;
      enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793,500,1));
      enqueueTask(createPlayMultiToneTask(3136,500,1));
      enqueueTask(createPlayMultiToneTask(2793,500,1));
      enqueueTask(createPlayMultiToneTask(3136,500,1));
      enqueueTask(createPlayWavTask("wav/destination_change.wav",3));
    }
  }


  if(newcourse_arrived){
    update_degpersecond();
    update_tone(degpersecond);
    update_course_warning(degpersecond);
    newcourse_arrived = false;

    int ttrack = get_gps_truetrack();
    float tt_radians = deg2rad(ttrack);
    steer_angle = (magc-8) - ttrack;
    if(steer_angle < -180){
      steer_angle += 360;
    }else if(steer_angle > 180){
      steer_angle -= 360;
    }
  }

  if(!redraw_screen){
    // (GPSの受信が完了したタイミング or GPSが不作動) && (画面更新インターバルが経過している) or 強制描画タイミング(スイッチ操作など)
    // TFT のSPI 通信とGPS module RX Interruptのタイミング競合によりGPS受信失敗するので、GPS受信直後にTFT更新を限定している。
    redraw_screen = (( !get_gps_connection()) && (millis() - last_screen_update > SCREEN_INTERVAL)) || quick_redraw;
    // 新しいgmapのロードが完了していて、GPS受信中ではない(longdraw_allowed)の場合。
    if (new_gmap_ready || new_gps_info) {
      redraw_screen = true;
    }
    if (millis() - lastfresh_millis > SCREEN_FRESH_INTERVAL) {
      redraw_screen = true;
    }
  }

  
  // GPSデータを受信した時と、bmpロード完了した時、両方でcallされるので毎秒二回は redraw_screen = true になる。
  if (redraw_screen) {
    DEBUG_PLN(20250424, "redraw map");
    quick_redraw = false;
    float new_truetrack = get_gps_truetrack();
    double new_lat = get_gps_lat();
    double new_long = get_gps_lon();
    last_screen_update = millis();
    set_newdata_off();

    //画面上の方向設定
    float drawupward_direction = truetrack_now;
    if (is_northupmode()) {
      drawupward_direction = 0;
    }

    nav_update();// MC や dist を更新する。
    draw_header(degpersecond);

    truetrack_now = new_truetrack;
    new_gmap_ready = false;

    //clean_map();


    clean_backscreen();


    int zoomlevel = 0;
    if (scaleindex == 0) zoomlevel = 5;//SCALE_EXSMALL_GMAP
    if (scaleindex == 1) zoomlevel = 7;//SCALE_SMALL_GMAP
    if (scaleindex == 2) zoomlevel = 9;//SCALE_MEDIUM_GMAP
    if (scaleindex == 3) zoomlevel = 11;//SCALE_LARGE_GMAP
    if (scaleindex == 4) zoomlevel = 13;//SCALE_EXLARGE_GMAP

    bool gmap_drawed = false;
    if(gmap_loaded_active)
      gmap_drawed = draw_gmap(drawupward_direction);
    
    

    //If loadImagetask already running in Core1, abort.
    if(new_gps_info || lastload_zoomlevel != zoomlevel){
      lastload_zoomlevel = zoomlevel;
      enqueueTaskWithAbortCheck(createLoadMapImageTask(new_lat, new_long, zoomlevel));
    }

    if (scale > SCALE_SMALL_GMAP) {
      if (check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
        draw_Biwako(new_lat, new_long, scale, drawupward_direction,gmap_drawed);
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

    if(currentdestination != -1 && currentdestination < destinations_count){
      double destlat = extradestinations[currentdestination].cords[0][0];
      double destlon = extradestinations[currentdestination].cords[0][1];
      if(destination_mode == DMODE_FLYINTO) 
        draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction,3);
      else if(destination_mode == DMODE_FLYAWAY) 
        draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
      else if(destination_mode == DMODE_AUTO10K){
        if(auto10k_status == AUTO10K_AWAY)
          draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
        else if(auto10k_status == AUTO10K_INTO)
          draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction,3);
      }
    }
    gps_loop(5);
    draw_compass(drawupward_direction, COLOR_BLACK);
    if (get_demo_biwako()) {
      draw_demo_biwako();
    }
    draw_km_distances(scale);
    //google map load成功している間は、古いものを削除する必要はない。
    //if (!gmap_loaded_active) {
    //  redraw_compass(drawupward_direction, COLOR_BLACK, COLOR_WHITE);
    //} else {
    //  draw_compass(drawupward_direction, COLOR_BLACK);
    //}
    draw_triangle(new_truetrack, steer_angle);

    //10秒間 warning 表示
    if(millis() - last_course_warning_time < 10000 && millis()  > 10000){
      draw_course_warning(steer_angle);
    }
    
    if (!gmap_drawed) {
      draw_nogmap(scale);
    }
    draw_headertext(degpersecond);
    draw_nomapdata();
    gps_loop(6);
    push_backscreen();
    draw_footer();
    lastfresh_millis = millis();
    

    //地図が更新されていない時でも、更新するもの。

    /*
    if (bank_warning) {
      draw_bankwarning();
    }*/


    //更新終了
    redraw_screen = false;

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
        startPlayWav(currentTask.playWavArgs.wavfilename,currentTask.playWavArgs.priority);
        break;
      case TASK_PLAY_MULTITONE:
        playTone(currentTask.playMultiToneArgs.freq,currentTask.playMultiToneArgs.duration,currentTask.playMultiToneArgs.counter);
        break;
      case TASK_INIT_SD:
        setup_sd(1);
        break;
      case TASK_BROWSE_SD:
        browse_sd(currentTask.pagenum);
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
          currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.mtrack,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second
        );
        break;
      case TASK_LOAD_MAPIMAGE:
        load_mapimage(
          currentTask.loadMapImageArgs.center_lat, currentTask.loadMapImageArgs.center_lon,
          currentTask.loadMapImageArgs.zoomlevel
        );
        break;
    }
    currentTask.type = TASK_NONE;
  } else {
    // No tasks, optionally sleep or yield
    delay(10);
  }
}


//==========BUTTON==========
//Callback function for short press
void shortPressCallback() {
  quick_redraw = true;
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");


  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = (cursorLine + 1) % setting_size;
    } else {
      menu_settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL || screen_mode == MODE_SDDETAIL) {
    detail_page++;
  } else {
    gmap_loaded_active = false;
    scaleindex = (scaleindex+1) % (sizeof(scalelist) / sizeof(scalelist[0]));
    scale = scalelist[scaleindex];
  }
}

// Callback function for long press
void longPressCallback() {
  quick_redraw = true;
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
        if(menu_settings[cursorLine].CallbackExit != nullptr)
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
    upward_samples[i] = track;
  }
  degpersecond = 0;
  bank_warning = false;
}

// deg per second being updated. call this once a second.
void update_degpersecond() {
  lastSampleTime = millis();
  // Update the truetrack value
  // This is where you would get the new truetrack value from your sensor
  upward_samples[sampleIndex] = get_gps_truetrack();
  // Calculate the differential if we have enough samples
  if (sampleIndex >= NUM_SAMPLES - 1) {
    float totalDifference = 0;
    // Calculate the differences between consecutive samples
    for (int i = 1; i < NUM_SAMPLES; i++) {
      float degchange = (upward_samples[i] - upward_samples[i - 1]);
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
      upward_samples[i - 1] = upward_samples[i];
    }
    // Adjust the sample index to the last position
    sampleIndex = NUM_SAMPLES - 2;
  }
  // Move to the next sample
  sampleIndex++;
}

//一秒に一回まで。
void update_course_warning(float degpersecond){
  //移動していない時,GPSロスト時は発動しない。
  if(get_gps_mps() < 2 || get_gps_numsat() == 0){
    course_warning_index = 0;
  }
  //正しい方向に変化している時はindexを減らす。
  else if((degpersecond > 0.5 && steer_angle > 0) || (degpersecond < -0.5 && steer_angle < 0)){
    course_warning_index -= 15*abs(degpersecond);//修正速度に応じ、7以上のindexが減る。3deg/sの時、indexは45減る。
    
  }
  //正しい方向に修正されていない、かつ15度以上ずれている。
  else if(abs(steer_angle) > 15){
      course_warning_index += min(abs(steer_angle),90);//15以上、90を最大値として、indexに加算する。
  }
  //15度未満のズレ、index 0 reset。
  else{
    course_warning_index = 0;
  }
  
  // range 0-900
  if(course_warning_index > 900)
    course_warning_index = 900;
  else if(course_warning_index < 0)
    course_warning_index = 0;
  
  //15度の修正されないズレは、15/900=60秒でwarning。90度以上の修正されないズレは、10秒でwarning。ただしwarningは、30秒に一度をmax回数とする。
  if(course_warning_index >= 900 && millis() - last_course_warning_time > 30000){
    last_course_warning_time = millis();
    course_warning_index = 0;
    enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793,500,1));
    enqueueTask(createPlayMultiToneTask(3136,500,1));
    enqueueTask(createPlayMultiToneTask(2793,500,1));
    enqueueTask(createPlayMultiToneTask(3136,500,1));
    if(steer_angle > 0)
      enqueueTask(createPlayWavTask("wav/course_right.wav",2));
    else
      enqueueTask(createPlayWavTask("wav/course_left.wav",2));
  }
}