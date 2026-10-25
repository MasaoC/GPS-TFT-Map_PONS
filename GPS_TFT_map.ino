// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : メインエントリポイント。
//           Core0: 画面描画・GPS処理・ボタン入力・コース警告
//           Core1: SDカード操作
//           地図背景はフラッシュ内蔵のベクタ地図（vectormap.cpp）を使う。
//           SDカード上のBMPタイル方式は廃止済み。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================

#include "navdata.h"
#include "display_tft.h"
#include "map_draw.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "power.h"

unsigned long last_time_gpsdata = 0;  //地図を最後に描き直した時間 millis()
bool redraw_screen = false;           //次の描画では、黒塗りして新たに画面を描画する
volatile bool quick_redraw = false;   //次の描画時間を待たずに、次のループで黒塗りして新たに画面を描画する
bool bank_warning = false;

// 旋回角速度 [deg/s]。真方位を NUM_SAMPLES 個の窓で持ち、窓の両端の時刻の差で割る（v7 と同じ。
// 測位の周期によらない）。2Hz なら約 1.5 秒の平均。
const int NUM_SAMPLES = 4;
float truetrack_samples[NUM_SAMPLES];
uint32_t truetrack_sample_times[NUM_SAMPLES];
int sampleIndex = 0;
float degpersecond = 0;

// 既定は Auto 10km（v7 の 0.984 と同じ。SD の settings.txt が読めれば、そこに残したモードに戻る）。
// SD が無い・壊れた機体でも、大会で使う設定のまま動く。既定の目的地 PLATHOME は Auto 10km にできる目的地。
int destination_mode = DMODE_AUTO10K;

int screen_mode = MODE_MAP;
int detail_page = 0;
int scaleindex = 3;

const double scalelist[] = { SCALE_EXSMALL_GMAP, SCALE_SMALL_GMAP, SCALE_MEDIUM_GMAP, SCALE_LARGE_GMAP, SCALE_EXLARGE_GMAP, 180.0f };
#define SCALE_COUNT ((int)(sizeof(scalelist) / sizeof(scalelist[0])))
static const char* const scale_names[SCALE_COUNT] = { "zoom5", "zoom7", "zoom9", "zoom11", "zoom13", "zoom15" };
double scale = scalelist[scaleindex];

// 地図の縮尺を step 段変える。scaleindex は 0〜SCALE_COUNT-1 に収める
// （以前は ++ し続けて % で引いていて、-- 側では負の添字になりえた。SD にも残すので収めておく）。
void change_scale(int step) {
  scaleindex = ((scaleindex + step) % SCALE_COUNT + SCALE_COUNT) % SCALE_COUNT;
  scale = scalelist[scaleindex];
  enqueueTaskWithAbortCheck(createSaveSettingsTask());  // 次の起動で同じ縮尺に戻す
}
const char* scale_name() { return scale_names[scaleindex]; }

// Variables for setting selection
int selectedLine = -1;
int cursorLine = 0;
unsigned long lastfresh_millis = 0;
unsigned long last_user_input_ms = 0;   // 最後にボタンを操作した時刻（設定画面の放置タイムアウト用）
extern int setting_size;

// ----- 誘導（nav_tick() が更新し、地図の描画が読む） -----
float steer_angle = 0;                  // 目的地の方位 − 対地進路（-180〜+180。正 = 右へ）
float course_warning_index = 0;         // 0〜COURSE_WARN_INDEX_MAX。外れたまま修正しない積算（度・秒）
unsigned long last_course_warning_time = 0;  // 「Turn RIGHT/LEFT」を出した時刻（0 = まだ）
float last_tone_tt = 0;                 // 30 度の範囲表示の基準の進路
unsigned long trackwarning_until = 0;   // 範囲を外れた直後、この時刻まで三角を点滅する

// 設定を SD に残し、リプレイの選択を反映して地図画面へ戻る。設定画面の Exit と放置タイムアウトの両方から呼ぶ。
void exit_setting() {
  screen_mode = MODE_MAP;
  selectedLine = -1;
  replay_apply_selection();
  enqueueTaskWithAbortCheck(createSaveSettingsTask());
  redraw_screen = true;
  quick_redraw = true;
}

// Callback function for short press
void shortPressCallback() {
  last_user_input_ms = millis();
  DEBUG_PLN(20240801, "short press");


#ifdef SINGLE_SWITCH
  if (screen_mode == MODE_MAP) {
    // ★ 地図画面の 1 回押しでは何もしない。縮尺はダブルクリックで変える（v7 の 0.921）。
    //   以前は 1 回押すごとに縮尺が変わり、飛行中・航行中に意図せず押して地図が変わっていた。
    return;
  }
  quick_redraw = true;
  redraw_screen = true;
  if (screen_mode == MODE_SETTING) {  //Setting mode
    if (selectedLine == -1) {         //No active selected line.
      cursorLine = (cursorLine + 1) % setting_size;
    } else {
      settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  }
#else
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
    if (selectedLine == -1) {
      //Entering changing value mode.
      selectedLine = cursorLine;
    } else {
      //exiting changing value mode.
      selectedLine = -1;
    }
  }
#endif
}

// ダブルクリック: 地図画面で縮尺を 1 段変える。ほかの画面では何もしない（1 回ごとの短押しはもう処理済み）。
void doublePressCallback() {
  last_user_input_ms = millis();
  if (screen_mode != MODE_MAP) return;
  change_scale(1);
  quick_redraw = true;
}

// Callback function for short press（3 ボタンの基板用。v3 は 1 ボタンなので使わない）
void shortPressCallback_up() {
  last_user_input_ms = millis();
  quick_redraw = true;
  redraw_screen = true;
  if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  } else if (screen_mode == MODE_SETTING) {
    if (selectedLine == -1) cursorLine = (cursorLine + 1) % setting_size;
    else settings[selectedLine].CallbackToggle();
  } else {
    change_scale(1);
  }
}

void shortPressCallback_down() {
  last_user_input_ms = millis();
  quick_redraw = true;
  redraw_screen = true;
  if (screen_mode == MODE_MAPLIST) {
    detail_page--;
  } else if (screen_mode == MODE_SETTING) {
    if (selectedLine == -1) cursorLine = mod(cursorLine - 1, setting_size);
    else settings[selectedLine].CallbackToggle();
  } else {
    change_scale(-1);
  }
}

// Callback function for long press
void longPressCallback() {
  last_user_input_ms = millis();
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
      // ★ 地図画面へ戻る項目（Exit）のときは選択状態にしない
      selectedLine = (screen_mode == MODE_MAP) ? -1 : cursorLine;
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
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback, doublePressCallback);
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
  setup_switch();
  setup_tft();   // SD は Core1 が初期化する（mysd.cpp の setup1()）
  power_setup();  // setup_tft() の analogReadResolution(12) より後

  gps_setup();
  startup_demo_tft();

  redraw_screen = true;
  quick_redraw = true;
  last_user_input_ms = millis();


  // Enqueue log_sd task
  Task task;
  task.type = TASK_LOG_SD;
  task.logText = "SETUP DONE";
  enqueueTask(task);

  DEBUG_PLN(20240801, "SETUP DONE");
}

void reset_degpersecond() {
  sampleIndex = 0;
  degpersecond = 0;
  bank_warning = false;
}

// 新しい進路のサンプルを入れて旋回角速度を更新する（nav_tick から）。v7 の update_degpersecond と同じ。
static void update_degpersecond(float true_track) {
  const uint32_t now = millis();
  truetrack_samples[sampleIndex] = true_track;
  truetrack_sample_times[sampleIndex] = now;
  if (sampleIndex >= NUM_SAMPLES - 1) {
    float total = 0;
    for (int i = 1; i < NUM_SAMPLES; i++) {
      float d = truetrack_samples[i] - truetrack_samples[i - 1];
      if (d < -180) d += 360;
      else if (d > 180) d -= 360;
      total += d;
    }
    const uint32_t elapsed = truetrack_sample_times[NUM_SAMPLES - 1] - truetrack_sample_times[0];
    degpersecond = elapsed > 0 ? total * 1000.0f / (float)elapsed : 0;
    for (int i = 1; i < NUM_SAMPLES; i++) {
      truetrack_samples[i - 1] = truetrack_samples[i];
      truetrack_sample_times[i - 1] = truetrack_sample_times[i];
    }
    sampleIndex = NUM_SAMPLES - 2;
  }
  sampleIndex++;
}

void check_bankwarning() {
  //Do not trigger bank warning if speed is below ... （GPS が止まった・測位を失ったときも出さない）
  const bool on = get_gps_live() && get_gps_mps() > 2.0 && abs(degpersecond) > 3.0;
  if (bank_warning && !on) redraw_screen = true;   // 警告の帯を消す
  bank_warning = on;
}




// ===== 起動時の設定の反映 =====
// SD の settings.txt は Core1 が起動時に読む（mysd.cpp）。反映は Core0 のここで、1 回だけ。
static void apply_loaded_settings() {
  SavedSettings st;
  if (!take_loaded_settings(&st)) return;
  if (st.destination[0] != '\0') {
    for (int i = 0; i < destinations_count; i++) {
      if (strcmp(extradestinations[i].name, st.destination) == 0) {
        currentdestination = i;
        break;
      }
    }
  }
  if (st.navigation_mode >= DMODE_FLYINTO && st.navigation_mode <= DMODE_AUTO10K) destination_mode = st.navigation_mode;
  // パイロン・竹島を目的地にしたまま Auto 10km にはしない（設定画面と同じ約束）
  if (destination_mode == DMODE_AUTO10K && currentdestination >= 0 && currentdestination < destinations_count
      && is10k_not_allowed(extradestinations[currentdestination].name)) {
    destination_mode = DMODE_FLYINTO;
  }
  if (st.auto10k_status == AUTO10K_AWAY || st.auto10k_status == AUTO10K_INTO) {
    auto10k_status_flight = st.auto10k_status;
    if (!get_demo_biwako() && !replay_active()) auto10k_status = st.auto10k_status;
  }
  if (st.upward_mode == 0 || st.upward_mode == 1) set_upward_mode(st.upward_mode);
  if (st.scaleindex >= 0 && st.scaleindex < SCALE_COUNT) {
    scaleindex = st.scaleindex;
    scale = scalelist[scaleindex];
  }
  redraw_screen = true;
  quick_redraw = true;
}

// 位置が仮想か（デモ・リプレイ）。仮想の位置で出た Auto 10km のフェーズは SD に残さない。
static bool virtual_position() {
  return get_demo_biwako() || replay_active();
}

// ===== Auto 10km（v7 の nav_alarm_tick() の 1 と同じ判定）=====
// 目的地（通常は PLATHOME）から AUTO10K_TURN_KM を超えたら INTO、戻って AUTO10K_RETURN_KM 以内で AWAY。
unsigned long auto10k_notice_since = 0;  // 切り替わった時刻（地図に知らせを出す。0 = 出さない）

// フェーズを変える。
// ★ デモ・リプレイ中は表示だけ変え、SD には残さない。仮想の位置で出たフェーズを焼くと、
//   次の起動が復路から始まる（v7 と同じ）。デモ・リプレイを抜けると実際の値に戻る（gps.cpp / 設定画面）。
static void apply_auto10k_status(int st) {
  auto10k_status = st;
  auto10k_notice_since = millis();
  const bool virt = virtual_position();
  if (!virt) {
    auto10k_status_flight = st;
    // 飛行中に再起動しても復路の向きが逆にならないよう、切り替わるたびに保存する（1 周に数回だけ）
    enqueueTaskWithAbortCheck(createSaveSettingsTask());
  }
  enqueueLogSdf("AUTO10K -> %s%s", st == AUTO10K_INTO ? "INTO" : "AWAY", virt ? " (virtual)" : "");
  quick_redraw = true;
}

// ★ 保存した INTO を戻した状態でプラットホーム付近にいると、起動直後に AWAY へ切り替わって知らせが出る。
//   黙って向きが変わるより、知らせて確認を促す方が安全という v7 と同じ判断。
static void auto10k_check(double lat, double lon) {
  if (destination_mode != DMODE_AUTO10K) return;
  if (currentdestination < 0 || currentdestination >= destinations_count) return;
  const double d = calculateDistance(lat, lon, extradestinations[currentdestination].cords[0][0],
                                     extradestinations[currentdestination].cords[0][1]);
  if (auto10k_status == AUTO10K_AWAY && d > AUTO10K_TURN_KM) {
    apply_auto10k_status(AUTO10K_INTO);
  } else if (auto10k_status == AUTO10K_INTO && d < AUTO10K_RETURN_KM) {
    apply_auto10k_status(AUTO10K_AWAY);
  }
}

// 修正しないまま外れ続けたことの積算（v7 の update_course_warning と同じ式。音の代わりに枠を出す）
static void update_course_warning() {
  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  const float dt = last_ms ? (now - last_ms) / 1000.0f : 0;
  last_ms = now;
  if (dt <= 0 || dt > 3.0f) return;   // 初回と、画面の切り替えなどで間が空いた回は積まない
  if (get_gps_mps() < 2 || get_gps_numsat() == 0) {
    course_warning_index = 0;           // 動いていない・GPS が無い
  } else if ((degpersecond > 0.5f && steer_angle > 0) || (degpersecond < -0.5f && steer_angle < 0)) {
    course_warning_index -= 15 * fabsf(degpersecond) * dt;   // 正しい向きに旋回中
  } else if (fabsf(steer_angle) > COURSE_WARN_MIN_DEG) {
    course_warning_index += fminf(fabsf(steer_angle), 90) * dt;
  } else {
    course_warning_index = 0;
  }
  course_warning_index = constrain(course_warning_index, 0.0f, COURSE_WARN_INDEX_MAX);
  // 15 度のズレを修正しないと 60 秒、90 度以上なら 10 秒で出す。出すのは 30 秒に 1 回まで。
  if (course_warning_index >= COURSE_WARN_INDEX_MAX && (last_course_warning_time == 0 || now - last_course_warning_time > COURSE_WARN_REPEAT_MS)) {
    last_course_warning_time = now;
    course_warning_index = 0;
    enqueueLogSdf("COURSE WARNING %s (steer %.0f)", steer_angle > 0 ? "RIGHT" : "LEFT", steer_angle);
  }
}

// ===== 誘導の判断（v7 の nav_alarm_tick に当たる）=====
// ★ 画面によらず、新しい位置が来るたびに loop() から呼ぶ。描画は一切しない。
//   以前は旋回角速度などが地図画面の分岐の中にあり、設定画面を開いている間は止まっていた（v7 の 0.965）。
static void nav_tick() {
  const bool live = get_gps_live();
  const double lat = get_gps_lat(), lon = get_gps_lon();
  const float tt = get_gps_truetrack();
  if (live) auto10k_check(lat, lon);

  // 目的地へのズレ角。測位していない・目的地が無いときは 0（円弧も三角も出さない）
  steer_angle = 0;
  if (live && currentdestination >= 0 && currentdestination < destinations_count) {
    float tc = true_course_deg(lat, lon, extradestinations[currentdestination].cords[0][0],
                               extradestinations[currentdestination].cords[0][1]);
    if (nav_effective_mode() == DMODE_FLYAWAY) tc += 180;
    float d = fmodf(tc - tt, 360.0f);
    if (d < -180) d += 360;
    else if (d > 180) d -= 360;
    steer_angle = d;
  }
  update_degpersecond(tt);
  update_course_warning();
  check_bankwarning();

  // 30 度の範囲表示（v7 の update_tone の警告 1。v3 は音の代わりに三角を点滅）
  if (live) {
    float rel = tt - last_tone_tt;
    if (rel > 180) rel -= 360;
    if (rel < -180) rel += 360;
    if (fabsf(rel) > TRACK_RANGE_HALF_DEG) {
      last_tone_tt = tt;
      if (get_gps_mps() > 2.0) trackwarning_until = millis() + TRACK_WARN_MS;
    }
  }
}

// 設定画面（と GPS 詳細・地図一覧）を放置したら地図へ戻す（v7 の 0.965。出したまま出てしまったときの保険）
static void setting_timeout_tick() {
  if (screen_mode == MODE_MAP) return;
  if (millis() - last_user_input_ms <= SETTING_IDLE_TIMEOUT_MS) return;
  enqueueLogSdf("SETTING SCREEN TIMEOUT -> MAP (%lus idle)", (unsigned long)(SETTING_IDLE_TIMEOUT_MS / 1000UL));
  if (screen_mode == MODE_GPSDETAIL) gps_getposition_mode();
  exit_setting();
  last_user_input_ms = millis();
}

// 地図の 1 フレームに渡す状態を作る
static MapView make_map_view() {
  MapView v = {};
  v.lat = get_gps_lat();
  v.lon = get_gps_lon();
  v.scale = scale;
  v.ttrack = get_gps_truetrack();
  v.up = is_northupmode() ? 0 : v.ttrack;
  v.live = get_gps_live();
  v.demo = get_demo_biwako();
  v.replay = replay_active();
  v.replay_speed = replay_get_speed();
  v.hacc_m = get_gps_hacc_m();
  // 三角の自機を出してよいか。精度が悪い（hAcc が HACC_CIRCLE_M 以上）ときは精度の円にする（v7 と同じ）。
  // デモとリプレイは実際の精度と関係ないので、いつも三角。
  v.marker_exact = v.demo || v.replay || (v.hacc_m >= 0 && v.hacc_m < HACC_CIRCLE_M);
  v.has_destination = currentdestination >= 0 && currentdestination < destinations_count;
  if (v.has_destination) {
    v.dest_lat = extradestinations[currentdestination].cords[0][0];
    v.dest_lon = extradestinations[currentdestination].cords[0][1];
  }
  v.fly_into = nav_effective_mode() == DMODE_FLYINTO;
  v.steer = steer_angle;
  v.course_warning_index = course_warning_index;
  v.course_warning_box = last_course_warning_time != 0 && millis() - last_course_warning_time < COURSE_WARN_SHOW_MS;
  v.tone_tt = last_tone_tt;
  v.tone_warning = millis() < trackwarning_until;
  v.gps_connected = get_gps_connection();
  v.fix_age_s = get_gps_fix_age_s();
  v.degpersecond = degpersecond;
  return v;
}

void loop() {
  switch_handling();
  // リプレイ再生中、地図画面以外（設定画面・GPS詳細・地図一覧）では進行を一時停止
  replay_set_paused(screen_mode != MODE_MAP);
  // gps_loop() は新しい位置（NAV-PVT かリプレイの行）を受けた回だけ true を返す
  bool new_position = gps_loop();
  // デモは GPS と無関係に動くので、2Hz で新しい位置とみなす
  static unsigned long last_demo_tick = 0;
  if (get_demo_biwako() && millis() - last_demo_tick >= 500) {
    last_demo_tick = millis();
    new_position = true;
  }
  // 判断はここ（画面ごとの分岐より前）に置く。どの画面を出していても動かすため。描画は混ぜない。
  power_tick();
  apply_loaded_settings();
  if (new_position) nav_tick();
  setting_timeout_tick();

  if (screen_mode == MODE_SETTING) {
    draw_setting_mode(redraw_screen, selectedLine, cursorLine);
    draw_readiness(redraw_screen);
    draw_power_status(redraw_screen);
    draw_power_notice(POWER_NOTICE_Y_SUB, redraw_screen);
    redraw_screen = false;
    return;
  }
  if (screen_mode == MODE_GPSDETAIL) {
    bool drew = draw_gpsdetail(redraw_screen, detail_page);
    draw_power_notice(POWER_NOTICE_Y_SUB, drew);
    redraw_screen = false;
    return;
  }

  if (screen_mode == MODE_MAPLIST) {
    bool drew = draw_maplist_mode(redraw_screen, detail_page);
    draw_power_notice(POWER_NOTICE_Y_SUB, drew);
    redraw_screen = false;
    return;
  }

  draw_loading_image();

  // 地図は新しい位置を受けた直後に描き直す（そこから次の測位まで約 0.5 秒あく）。
  // GPS が止まっている・測位していないときも、状態の文字を出すため 1 秒ごとに描き直す。
  const unsigned long since = millis() - last_time_gpsdata;
  bool redraw_map = quick_redraw || (new_position && since > SCREEN_INTERVAL) || (!get_gps_live() && since > 1000);
  if (redraw_map) {
    quick_redraw = false;
    last_time_gpsdata = millis();

    if (millis() - lastfresh_millis > SCREEN_FRESH_INTERVAL || redraw_screen) {
      lastfresh_millis = millis();
      clean_display();
      redraw_screen = true;
    }

    // 地図の部分（ベクタ地図・航跡・誘導線・自機・修正矢印）はスプライトに描いてから一度に送る（map_draw.cpp）
    draw_map_screen(make_map_view());

    //地図が更新されていない時でも、更新するもの。
    if (bank_warning) {
      draw_bankwarning();
    }
    draw_gpsinfo();
    draw_sdinfo();

    //更新終了
    redraw_screen = false;

    // 地図 1 フレームの時間を 60 秒ごとに残す（2Hz で描き切れているかを実機で確かめるため）
    static unsigned long last_frame_log = 0;
    if (millis() - last_frame_log >= 60000UL) {
      last_frame_log = millis();
      enqueueLogSdf("MAP frame %lums (max %lums)", (unsigned long)map_frame_ms_last, (unsigned long)map_frame_ms_max);
      map_frame_ms_max = 0;
    }
  }

  // 地図を描き直した回は上書きされているので必ず描く。それ以外は 1 秒ごと
  // （GPS が止まって地図の描き直しが止まっても出し続けるため、redraw_map の外に置く）。
  draw_power_notice(POWER_NOTICE_Y_MAP, redraw_map);
  // Auto 10km の切り替わりの知らせ。時間が終わったら地図を描き直して消す。
  if (draw_auto10k_notice(redraw_map)) {
    redraw_screen = true;
    quick_redraw = true;
  }
}
