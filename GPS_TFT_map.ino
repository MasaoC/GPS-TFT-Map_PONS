// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : メインエントリポイント。
//           Core0: 画面描画・GPS処理・ボタン入力・コース警告
//           Core1: SDカード操作・音声再生（タスクキュー経由）
//           地図背景はフラッシュ内蔵のベクタ地図（vectormap.cpp）を使う。
//           SDカード上のBMPタイル方式は廃止済み。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================

#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "sound.h"
#include "hardware/adc.h"
#include "vectormap.h"
#include "power.h"
#include "readiness.h"  // 本番前チェック（設定画面の右上）

// we need to do bool core1_separate_stack = true; to avoid stack running out.
// (Likely due to drawWideLine from TFT-eSPI consuming alot of stack.)
bool core1_separate_stack = true;  //DO NOT REMOVE THIS LINE.

// --- 画面更新タイミング管理 ---
unsigned long screen_update_time = 0;  // 最後に画面更新した時間 millis()
bool redraw_screen = false;            // true にすると次のループで画面を再描画する

// --- 旋回角速度 (deg/s) 計算用 ---
// GPS の真方位 (truetrack) を NUM_SAMPLES 個のスライディングウィンドウで保持し、
// 連続サンプル間の変化量の合計を「最古サンプルと最新サンプルの経過時間」で割ることで
// GPS レート（1Hz / 2Hz）に依存しない真の deg/s を算出する。
const int NUM_SAMPLES = 4;             // スライディングウィンドウのサンプル数
float truetrack_samples[NUM_SAMPLES];  // 過去の真方位サンプル配列
uint32_t truetrack_sample_times[NUM_SAMPLES];  // 各サンプルの millis() タイムスタンプ
int sampleIndex = 0;                   // 次に書き込むサンプルのインデックス
float degpersecond = 0;                // 算出された旋回角速度 [deg/s]

// --- 画面モード・スケール管理 ---
int screen_mode = MODE_MAP;  // 現在の画面モード（MODE_MAP / MODE_SETTING など）
int detail_page = 0;         // サブ画面（GPSDetail / SDDetail）のページ番号
int replay_cursor = 0;       // リプレイ選択画面のカーソル位置（項目の通し番号）
int replay_list_page = 0;    // リプレイ選択画面で現在表示・読み込み済みのページ
double scalelist[6];         // 選択可能なスケール値リスト（ズームレベルに対応）
double scale;                // 現在のマップスケール [pixels/km]

// --- 設定画面カーソル状態 ---
// selectedLine == -1: 値変更モードでない（カーソル移動のみ）
// selectedLine >= 0:  その行の値を変更中
int selectedLine = -1;
int cursorLine = 0;

// --- コース警告 ---
// course_warning_index: 0〜900 の積算値（単位: 度・秒）。
// コースから外れているほど増加し、900 に達すると音声警告を発する。
// dt 乗算で小数加算が発生するため float 型で保持する。
float course_warning_index = 0;
unsigned long last_course_warning_time = 0;      // 直近の警告発報時刻 [millis]
unsigned long last_destination_toofar_time = 0;  // 直近の「目的地が遠すぎる」警告時刻 [millis]
double steer_angle = 0.0;  // 現在針路と目的地方位の差 (-180〜+180 度。正=右、負=左)

// ---- 処理時間計測変数（デバッグビルドのみ） ----
#ifndef RELEASE
TimingStat ts_redraw = TSTAT_INIT("C0_redraw");
volatile bool c0_is_redrawing = false;  // Core1側の重複検出用フラグ
#endif

// --- Core1 スタック残量計測用ベース SP ---
// setup1() の先頭で現在の SP をキャプチャし、以降の使用量計測の基準にする。
// core1_separate_stack=true のため Core1 のスタックはヒープから動的確保されており、
// リンカシンボルでは下限がわからないため、この方法で代替する。
volatile uint32_t _core1_base_sp = 0;

volatile int scaleindex = 3;    // scalelist のインデックス（初期値 3 = SCALE_LARGE_GMAP）
volatile int sound_volume  = 50; // 音量 0〜100

extern volatile bool loading_sddetail;
extern bool sd_detail_loading_displayed;

void reset_degpersecond();
void update_degpersecond(float true_track);
void check_destination_toofar();
static void apply_auto10k_status(int st);
void update_course_warning(float degpersecond);
static void nav_alarm_tick(bool new_gps_info);
static void batt_alarm_tick();
static void setting_idle_tick();
static void gnss_conn_tick();
// 最後にボタンが操作された時刻。設定画面の放置タイムアウト（SETTING_IDLE_TIMEOUT_MS）用。
// 3 つのコールバックの先頭で更新するので、押し方の種類によらず必ず記録される。
static unsigned long last_user_input_ms = 0;
void shortPressCallback();
void longPressCallback();
void doublePressCallback();
void next_scaleindex();
// 設定画面のラベル用に、現在のスケールで画面横幅(240px)が何 km に相当するかを返す。
// 以前は Google タイルのズーム番号(zoom5〜13)を表示していたが、ベクタ地図に移行して
// タイルとの対応が無くなったため、実際の距離で表すようにした。
float scale_screen_km(int index);
// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback, doublePressCallback);

//===============SET UP CORE0=================
// Core0 の初期化処理。
// 初期化順の注意:
//   1. TFT を先に初期化しないとボタンピン設定が正常に動作しない（TFT_eSPI の制約）。
//   2. GPS は TFT 初期化後に setup する（バッファオーバーフロー防止のため）。
void setup(void) {
  Serial.begin(38400);
  #ifndef RELEASE
    // デバッグ時はシリアルモニタが繋がるまで待機（RELEASE では即開始）
    while (!Serial) {
      delay(10);
    }
  #endif

  // バッテリー電圧監視用 ADC の初期化
  adc_gpio_init(BATTERY_PIN);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(SD_DETECT, INPUT_PULLUP);  // SDカード検出ピン（挿入時 LOW）
  analogReadResolution(12);  // 12bit = 0〜4095
  power_setup();             // USB 判定と電源経路の異常の見張り（analogReadResolution の後）

  //setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  setup_tft();
  // ★ **ここでタイトルを出しておく**（v7 の 0.975）。これが無いと起動画面が始まるまで画面が真っ白になる。
  //   白のままの区間は gps_setup() で、**GPS 未接続だと設定コマンドの応答待ち（1 つ 600ms）が
  //   空振りして約 2.4 秒**かかる（log.txt の "GPS CFG NG"）。**固まっているわけではない。**
  draw_boot_title();

  // ★★ **log.txt の最初の行はビルドの身元にする**（v7 の 0.991）。
  //   これが無いと、後からログを読んだときに**どのファームが出したものか分からない**。
  //   バージョンだけでは足りない:
  //     ・どの基板向けのビルドか（v5 / v6 / v7 は同じ番号帯で別のファーム）
  //     ・RELEASE でないビルドは GPS シミュレーションやデバッグ出力が入りうる
  //   ★ 基板名は VERSION_TEXT を使う（settings.h。ここで書き直すと二重管理になる）。
  //   ★ 置き場所はここ。setup() の冒頭に出すと、Core1 の mutex_init より先に
  //     enqueueTask を呼ぶ窓に入る（setup1() の冒頭コメント参照）。
#ifdef RELEASE
  const char* build_rel = "";
#else
  const char* build_rel = " [NOT RELEASE]";
#endif
  enqueueTask(createLogSdfTask("BOOT PONS %s build %lu board=%s%s",
                               BUILDVERSION, (unsigned long)BUILDDATE, VERSION_TEXT, build_rel));

  gps_setup();
  draw_boot_progress(get_gps_cfg_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG);

  startup_demo_tft();

  // スケールリストの初期化（Google Map のズームレベルに対応する pixel/km 値）
  scalelist[0] = SCALE_EXSMALL_GMAP;  //pixelsPerDegreeLat(5,35)/KM_PER_DEG_LAT; 最広域（日本全体スケール）
  scalelist[1] = SCALE_SMALL_GMAP;    //pixelsPerDegreeLat(7,35)/KM_PER_DEG_LAT;
  scalelist[2] = SCALE_MEDIUM_GMAP;   //pixelsPerDegreeLat(9,35)/KM_PER_DEG_LAT;
  scalelist[3] = SCALE_LARGE_GMAP;    //pixelsPerDegreeLat(11,35)/KM_PER_DEG_LAT;
  scalelist[4] = SCALE_EXLARGE_GMAP;  //pixelsPerDegreeLat(13,35)/KM_PER_DEG_LAT;
  scalelist[5] = 200.0;               // 最大拡大
  scale = scalelist[scaleindex];
  redraw_screen = true;
  {
    // get_input_voltage() のピーク追跡（max_adreading）を通した値を記録する。
    // analogRead() の 1 発読みだと ADC のノイズがそのまま乗り、USB を抜いた直後などに
    // 実際より低い値がログに残ることがあった。
    // 起動直後は max_adreading が 0 なので、数サンプル回して追跡を立ち上げてから読む。
    // ここは setup() の末尾なので、16ms 程度の待ちは起動時間に影響しない。
    // 併せて、この後の最初のヘッダー描画も落ち着いた値から始められる。
    float startup_voltage = 0.0f;
    for (int i = 0; i < 16; i++) {
      startup_voltage = get_input_voltage();
      delay(1);
    }
    enqueueTask(createLogSdfTask("SETUP DONE Battery: %.2fV", startup_voltage));
  }

}


//===============SET UP CORE1=================
// Core1 の初期化処理。タスクキュー・音声・SD を準備する。
// Core0 と独立して動作し、重いSD処理・音声再生を引き受けることで
// Core0 の描画ループをブロックしない設計になっている。
void setup1(void) {
  // タスクキューの mutex は何よりも先に初期化する。
  // arduino-pico の main() は multicore_launch_core1() を呼んでから setup() を呼ぶため、
  // Core0 の setup() と この setup1() は並行に走る。Core0 は setup() 内で enqueueTask() を
  // 呼ぶので、mutex_init が遅れると未初期化の mutex を掴む危険がある。
  // 特に下の while(!Serial) は USB 接続まで待つため、その後ろに置くと窓が極端に長くなる。
  mutex_init(&taskQueueMutex);  // Core0/Core1 間のタスクキュー排他制御用 mutex を初期化

  Serial.begin(38400);
  #ifndef RELEASE
    while (!Serial) {
      delay(10);
    }
  #endif

  init_destinations();           // 目的地リストを SD から読み込む
  setup_sound();                 // スピーカー・アンプ・PWM を初期化

  //====core1_separate_stack = true の時、Serial.printがないとSDが動かない。詳細不明だが、消すな！
  Serial.println("");  //消すな
  // call to Serial.println() might force the Arduino runtime or the RP2350’s FreeRTOS (used in the RP2040/RP2350 Arduino core for dual-core support) to fully initialize Core1’s context,
  // ensuring that the SD library’s internal state to set up.
  // Without this call, Core1 might attempt to initialize the SD card before its runtime environment is fully ready, especially with a separate stack.
  //====

  setup_sd(1);  // 起動時も1回のみ。接触不良時の長時間ブロック防止。失敗後は try_sd_recovery() が10秒ごとにリトライ。
  // 起動音（opening.wav）は startup_demo_tft() 内でエンキューする（重複防止）
}



// その年月の日数。うるう年を考慮する（下の get_jst_now() の日付繰り上げ用）。
static int days_in_month_of(int mo, int y) {
  static const uint8_t dim[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  if (mo < 1 || mo > 12) return 31;   // GPS の月が壊れていても配列外を読まない
  if (mo == 2 && (y % 4 == 0) && (y % 100 != 0 || y % 400 == 0)) return 29;
  return dim[mo];
}

// ============================================================
// get_jst_now(): GPS の UTC 日時から現在の JST 日時を求める
// ============================================================
// GPS パケット受信時刻からの millis() 経過分を足して現在時刻を推定し、
// UTC+9 の JST に変換する。日をまたぐ場合は日付を繰り上げる（うるう年考慮）。
//
// 戻り値: GPS 日時が有効なら true。false のとき出力引数の内容は不定。
static bool get_jst_now(int &y, int &mo, int &d, int &h, int &mi, int &s, int &cs) {
  if (!get_gpsdate().isValid() || !get_gpstime().isValid()) return false;

  // millis() オフセットで UTC 時刻を推定し JST（UTC+9）に変換
  uint32_t elapsed_ms = millis() - get_gps_fix_millis();
  int utc_cs = get_gpstime().centisecond() + (int)(elapsed_ms / 10);
  int utc_s  = get_gpstime().second()      + utc_cs / 100;
  int utc_m  = get_gpstime().minute()      + utc_s  / 60;
  int utc_h  = get_gpstime().hour()        + utc_m  / 60;
  cs = utc_cs % 100;
  s  = utc_s  % 60;
  mi = utc_m  % 60;
  int jst_total_h = utc_h + 9;
  h = jst_total_h % 24;

  // JST 日付計算。★ 繰り上げは 1 日ぶんとは限らない（v7 の 0.963）。
  //   utc_h は「GPS パケットの時刻 + millis() の経過」なので、GPS が長時間
  //   止まったまま日時だけ有効に残ると 24 時間を超えて積み上がる。
  //   以前は +1 日しか見ていなかったため、そこから先は日付がずれていた。
  //   また月が壊れていると days_in_month[mo] が配列外を読んでいた。
  y  = get_gpsdate().year();
  mo = get_gpsdate().month();
  d  = get_gpsdate().day();
  for (int add_days = jst_total_h / 24; add_days > 0; add_days--) {
    d++;
    if (d > days_in_month_of(mo, y)) { d = 1; if (++mo > 12) { mo = 1; y++; } }
  }
  return true;
}


//===============MAIN LOOP CORE0=================
// Core0 のメインループ。以下の処理を毎ループ実行する:
//   1. ボタン状態の読み取り
//   2. GPS データの受信・解析（gps_loop は描画中にも分散して呼ぶ）
//   3. 画面モードに応じた描画（地図 / 設定 / GPS詳細 など）
// gps_loop(id) の id はデバッグ用の呼び出し箇所識別子。
void loop() {
  //switch handling
  sw_push.read();
  // 地図画面以外（設定画面・リプレイ選択画面・各詳細画面）を開いている間は
  // リプレイを一時停止する。戻ってきたら続きから再生される。
  replay_set_paused(screen_mode != MODE_MAP);
  gps_loop(0);  // ループ先頭で GPS データを受信
  // GPS モジュールの断線／復帰（音とログ。表示は display_tft.cpp）
  gnss_conn_tick();
  // 電源経路の異常・高温での充電（power.cpp。内部で 250ms に間引く）。
  // ★ 画面モードによらずここで回す。判断と警報は描画関数の中に置かない。
  power_tick();
  // 本番前チェック（readiness.cpp。内部で 1 秒に 1 回。表示は設定画面の右上）
  readiness_tick();

  // GPS Fix 取得時に音声で通知（false→true の立ち上がりエッジを検出）
  // 音声は本体 FLASH に内蔵しているので SD の有無に関係なく鳴る。
  {
    static bool prev_gps_fix = false;
    bool cur_fix = get_gps_fix();
    if (!prev_gps_fix && cur_fix) {
      enqueueTask(createPlayWavTask("wav/fixed.wav", 4));  // 優先度4: AUTO10Kトーン(p=3)より高くして埋もれないよう
      enqueueTask(createLogSdTask("GPS FIX acquired"));
    }
    prev_gps_fix = cur_fix;
  }

  // ---- 電池電圧の低下警告（batt_alarm_tick）----
  batt_alarm_tick();

#ifndef RELEASE
  // デバッグ時: PC シリアルから GPS モジュールへコマンドを転送可能
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
#endif

  // GPS 更新がなくても、一定間隔で強制再描画する
  // （時刻表示など時間経過で変わる表示の更新保証）
  if (millis() - screen_update_time > SCREEN_FRESH_INTERVAL) {
    redraw_screen = true;
  }

  // 60秒ごとに電圧と JST 時刻をテキストログへ記録（どの画面モードでも実行）
  {
    static unsigned long last_volt_log_ms = 0;
    if (millis() - last_volt_log_ms >= 60000UL) {
      last_volt_log_ms = millis();
      // ★ 時刻は **get_jst_now() の判定を通ったときだけ**書く。
      //   以前は get_gpstime() の生の値を無条件に使っていたため、測位できていない間は
      //   _hour/_min が 0 のままで「09:00 JST」と記録されていた。ログ上は
      //   正しい時刻に見えるので、あとから見たときに嘘に気づけない。
      //   （get_jst_now() は millis() の経過ぶんも足してあり、日跨ぎも処理済み。）
      int jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs;
      const bool jst_valid = get_jst_now(jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs);
      char jst_str[8];
      if (jst_valid) snprintf(jst_str, sizeof(jst_str), "%02d:%02d", log_h, log_m);
      else           strlcpy(jst_str, "--:--", sizeof(jst_str));
      // CPU 温度は power.cpp が 2 秒ごとに測って平滑化した値（高温充電の判定と同じもの）。
      enqueueTask(createLogSdfTask("volt=%.2fV cpu=%.1fC %s JST",
                                   get_input_voltage(), power_cpu_temp(), jst_str));

      // ★ タスクキューが溢れて捨てたタスクがあれば残す。v5 には USERLED が無い
      //   （v6 は溢れた瞬間に LED を点けっぱなしにしていた）ので、ここで件数を書く。
      //   溢れている最中はこのログ自体も積めないので、空いてから 1 行出る。
      static uint32_t logged_drops = 0;
      const uint32_t drops = task_drop_count;
      if (drops != logged_drops) {
        if (enqueueTask(createLogSdfTask("!!ERROR!! task queue overflow: %lu tasks dropped (total)",
                                         (unsigned long)drops)))
          logged_drops = drops;
      }
    }
  }

  // ---- ナビゲーションと警報 ----
  // ★ **画面モードによらず必ずここで走らせる。**
  //   以前はこの下の MODE_MAP の分岐の中に置いていたため、設定画面や各詳細画面を
  //   開いている間は Auto10km の折返し判定・コース逸脱警報・旋回角速度の更新が
  //   すべて止まっていた。とくに INTO→AWAY（目的地から 1.5km 以内）は「そのとき
  //   近くにいるか」で判定するので、地図画面を離れている間に通り過ぎると
  //   二度と成立せず、復路のまま戻らなくなる（v7 の 0.965）。
  bool new_gps_info = gps_new_location_arrived();
  nav_alarm_tick(new_gps_info);

  // ---- 設定画面の放置タイムアウト ----
  setting_idle_tick();

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
  } else if (screen_mode == MODE_REPLAYSELECT) {
    if (replay_loading_displayed && !loading_replaylist)
      redraw_screen = true;  // Core1 のファイル一覧取得が完了したので描き直す
    if (redraw_screen)
      draw_replayselect(replay_list_page, replay_cursor);
  } else if (screen_mode == MODE_MAPLIST) {
    if (redraw_screen)
      draw_maplist_mode(detail_page);
  } else if (screen_mode == MODE_MAP) {
    if (new_gps_info) {
      redraw_screen = true;
    }

    // GPS 更新のたびに redraw_screen が立つため、
    // 通常は毎秒 1 回程度（LC86G は 1Hz）この描画ブロックが実行される。
    if (redraw_screen) {
      TIMING_START(redraw);
      #ifndef RELEASE
      c0_is_redrawing = true;
      #endif
      float new_truetrack = get_gps_truetrack();
      double new_lat = get_gps_lat();
      double new_long = get_gps_lon();
      set_new_location_off();  // GPS の「新位置フラグ」をクリア

      // 画面の上方向を設定: トラックアップ時は機首方向、ノースアップ時は 0（北）
      float drawupward_direction = new_truetrack;
      if (is_northupmode()) {
        drawupward_direction = 0;
      }

      nav_update();   // 目的地への真方位(TC)・目的地距離(dist)を最新 GPS 位置で再計算
      draw_header();  // ヘッダー（速度・衛星数など）を TFT に直接描画


      // 描画はすべてバックスクリーンに対して行い、最後に push_backscreen() で
      // 一括転送することでちらつきを防ぐ。
      // 背景のクリアは draw_vectormap() が行うので、ここでは何もしない。

      // ---- レイヤー 1: 地図背景 ----
      // フラッシュ内蔵のベクタ地図（OpenStreetMap 由来）を backscreen へ直接描画する。
      // 回転は latLonToXY と同じ座標変換に吸収されるため、TRACKUP でも画面四隅まで埋まる。
      // SD カードは不要で、位置が動いても I/O は発生しない。
      draw_vectormap(new_lat, new_long, scale, drawupward_direction);

      // ---- レイヤー 2: ポリゴン地図（滑走路・基準線などの注記）----
      // 内蔵（フラッシュ）と SD の mapdata.csv 由来の両方を描く。
      // 内蔵は SD が抜けていても必ず表示されるので、飛行に必須の注記はそちらに置く。
      if (scale > SCALE_SMALL_GMAP) {
        draw_FlashMaps(new_lat, new_long, scale, drawupward_direction);
        draw_ExtraMaps(new_lat, new_long, scale, drawupward_direction);
      }

      gps_loop(4);  // 描画の合間に GPS データを受信（取りこぼし防止）

      // ---- レイヤー 3: 飛行軌跡 ----
      draw_track(new_lat, new_long, scale, drawupward_direction);

      // ---- レイヤー 3.5: パイロンへの基準線・コース円 ----
      // マゼンタラインより先に描画して、重なったときは基準線が隠れるようにする。
      bool draw_pilon = (scale > SCALE_SMALL_GMAP && check_within_latlon(0.6, 0.6, new_lat, pla_lat, new_long, pla_lon));
      if (draw_pilon) {
        draw_pilon_takeshima_line(new_lat, new_long, scale, drawupward_direction);
      }

      // ---- レイヤー 4: 目的地ライン ----
      // fix 取得前は自機位置が不明なため、誘導線は描画しない
      if (get_gps_fix() && currentdestination != -1 && currentdestination < destinations_count) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        if (destination_mode == DMODE_FLYINTO)
          draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction, 5);
        else if (destination_mode == DMODE_FLYAWAY)
          draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
        else if (destination_mode == DMODE_AUTO10K) {
          // AUTO10K: 折り返しの状態に応じて AWAY/INTO を自動切り替え
          if (auto10k_status == AUTO10K_AWAY)
            draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale, drawupward_direction);
          else if (auto10k_status == AUTO10K_INTO)
            draw_flyinto2(destlat, destlon, new_lat, new_long, scale, drawupward_direction, 5);
        }
      }
      gps_loop(5);  // 描画の合間に GPS データを受信

      // ---- レイヤー 4.5: パイロン・PLA アイコン ----
      // マゼンタラインより後に描画してアイコンが隠れないようにする。
      if (draw_pilon) {
        draw_pilon_takeshima_marks(new_lat, new_long, scale, drawupward_direction);
      }

      // ---- レイヤー 5: オーバーレイ（速度グラフ・スケールバーなど）----
      draw_degpersec(degpersecond);
      if (is_demo_active()) {
        draw_demo_biwako();  // 琵琶湖デモ表示（見た目や警告音などに慣れるための練習用）
      }
      if (getReplayMode()) {
        draw_replay_indicator();  // リプレイモード中であることを赤枠付きで通知
      }
      draw_km_distances(scale);  // 画面左下のスケールバー

      // GPS fix 状態と hAcc（GST）に応じて自機位置マーカーを切り替える
      // リプレイ・デモモードは実際の精度と無関係なので、常に通常の飛行機マーカーを表示する
      // ★ hAcc が分からない（GST が来ていない）ときは円を出さない。0 m と取り違えて
      //   「精度良好」と言うのでも、大きい円で「精度不良」と言うのでもなく、普通の自機マーカーにする。
      const bool cur_hacc_ok = get_gps_hacc_valid();
      const float cur_hacc_m = get_gps_hacc_mm() / 1000.0f;  // mm → m
      if (!get_gps_fix() && !is_demo_active() && !getReplayMode()) {
        draw_nofix_cross();                              // fix なし（通常モードのみ）: グレーの ×
      } else if (!is_demo_active() && !getReplayMode() &&
                 cur_hacc_ok && cur_hacc_m >= HACC_THRESHOLD_M) {
        draw_hacc_circle(scale, get_gps_hacc_mm());     // hAcc 不良: 青い不確かさ円
      } else {
        draw_triangle(new_truetrack, steer_angle);       // 精度良好、またはリプレイ/デモ: 飛行機マーカー
      }

      // コンパス(N/E/S/W)は自機マーカー・進行方向縦線より後に描いて上のレイヤーにする。
      draw_compass(drawupward_direction, COLOR_BLACK);

      // コース警告表示（警告発報から 10 秒間だけ表示する）
      if (millis() - last_course_warning_time < 10000 && millis() > 10000) {
        draw_course_warning(steer_angle);
      }


      draw_gs_track();  // ヘッダーに速度・コースなどのテキストを描画
      draw_map_footer();
      draw_nomapdata();
      gps_loop(6);        // 描画の合間に GPS データを受信
      push_backscreen();  // バックスクリーンを TFT に一括転送
      draw_footer();      // フッターは TFT に直接描画（バックスクリーン外）
      #ifndef RELEASE
      c0_is_redrawing = false;
      #endif
      TIMING_END(ts_redraw, redraw);
    }
  } else {
    DEBUGW_PLN(20250510, "ERR screen mode");
  }

  if (redraw_screen) {
    //更新終了
    redraw_screen = false;
    screen_update_time = millis();
  }

#ifndef RELEASE
  // 30秒ごとに各処理の最大・平均・回数を Serial 出力する
  { static uint32_t _last_timing_report = 0;
    if (millis() - _last_timing_report >= 30000) {
      _last_timing_report = millis();
      Serial.println("===== TIMING REPORT =====");
      TIMING_REPORT(ts_redraw);
      extern TimingStat ts_draw_header, ts_push_backscreen;
      TIMING_REPORT(ts_draw_header);
      TIMING_REPORT(ts_push_backscreen);
      extern TimingStat ts_savecsv_flush;
      TIMING_REPORT(ts_savecsv_flush);
      extern volatile uint32_t _c1_overlap_count;
      Serial.print("[TIME] C1_overlap_count="); Serial.println(_c1_overlap_count);
      Serial.println("=========================");
    }
  }
#endif
}


//===============MAIN LOOP CORE1=================
// Core1 のメインループ。タスクキューからタスクを取り出して順番に実行するディスパッチャー。
// SD アクセス・音声再生はすべて Core1 で行い、Core0 の描画をブロックしない。
// loop_sound() は毎ループ呼んでPWM音声の継続再生を維持する。
void loop1() {
  // core1_separate_stack=true の場合、setup1() とは別スタックで loop1() が動く。
  // そのため loop1() 最初の1回だけ SP をキャプチャしてベース値にする。
  if (_core1_base_sp == 0) {
    uint32_t _sp_; asm volatile ("mov %0, sp" : "=r" (_sp_)); _core1_base_sp = _sp_;
    DEBUG_P(20260312, "[C1 loop1 base SP]=0x"); DEBUG_PNLN(20260312, _core1_base_sp, HEX);
  }
  loop_sound();  // 音声の継続再生処理（WAV 送出など）
  loop_tone();   // トーンバッファの非ブロッキング再生管理
  if (dequeueTask(&currentTask)) {
    switch (currentTask.type) {
      case TASK_SAVE_SETTINGS:
        saveSettings();
        break;
      case TASK_PLAY_WAV:
        startPlayWav(currentTask.playWavArgs.wavfilename, currentTask.playWavArgs.priority, currentTask.playWavArgs.min_volume);
        break;
      case TASK_PLAY_MULTITONE:
        playTone(currentTask.playMultiToneArgs.freq, currentTask.playMultiToneArgs.duration, currentTask.playMultiToneArgs.counter, currentTask.playMultiToneArgs.priority, currentTask.playMultiToneArgs.min_volume, currentTask.playMultiToneArgs.solo_play);
        break;
      case TASK_INIT_SD:
        setup_sd(1);
        break;
      case TASK_BROWSE_SD:
        browse_sd(currentTask.pagenum);
        break;
      case TASK_BROWSE_REPLAY:
        // pagenum にはページ番号ではなく「ファイルの開始通し番号」が入っている
        browse_replay_files(currentTask.pagenum);
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
          currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.ttrack, currentTask.saveCsvArgs.gnss_altitude,
          currentTask.saveCsvArgs.hacc_m,
          currentTask.saveCsvArgs.voltage,
          currentTask.saveCsvArgs.numsat,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second,
          currentTask.saveCsvArgs.centisecond);
        break;
      case TASK_LOAD_LOGO:
        load_push_logo();  // SD からロゴ BMP を logo_sprite に読み込む（pushSprite は Core0 が行う）
        break;
    }
    clearCurrentTask();  // mutex 内で TASK_NONE にする（Core0 の isTaskRunning() と整合させるため）
  } else {
    // No tasks, optionally sleep or yield
    delay(10);
  }
}
extern volatile int max_page;  // Core1 が更新する（実体は mysd.cpp・volatile）

//==========BUTTON==========
// マップスケール（ズームレベル）を次の段階へ切り替える。
// 地図画面でのダブルクリックと、設定画面の Map scale 項目の両方から呼ばれる。
void next_scaleindex() {
  scaleindex = (scaleindex + 1) % (sizeof(scalelist) / sizeof(scalelist[0]));
  scale = scalelist[scaleindex];
}

// 画面横幅 240px が何 km に相当するかを返す（設定画面のスケール表示用）。
float scale_screen_km(int index) {
  if (index < 0 || index >= (int)(sizeof(scalelist) / sizeof(scalelist[0]))) return 0;
  return (float)(BACKSCREEN_SIZE / scalelist[index]);
}

// 短押しコールバック: 画面モードごとに動作が変わる。
//   MAP モード:      何もしない（誤操作防止のためスケール変更はダブルクリックに変更）
//   SETTING モード:  カーソル移動 or 値のトグル変更
//   SDDETAIL:       次のページを SD から読み込む
//   MAPLIST/GPSDETAIL: 次のページへ
// 操作音は Button クラスではなくここで鳴らす。何も起きない画面では鳴らさないため。
void shortPressCallback() {
  last_user_input_ms = millis();
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");

  // MAP モードでは短押しは何もしない（スケール変更はダブルクリックへ移した）。
  // 何も起きないのに音だけ鳴ると紛らわしいので、音も鳴らさずに抜ける。
  if (screen_mode == MODE_MAP)
    return;

  enqueueTask(createPlayMultiToneTask(1046, 80, 1));  // 短押し音

  if (screen_mode == MODE_SETTING) {
    if (selectedLine == -1) {  // 値変更モードでない → カーソルを次の行へ
      cursorLine = (cursorLine + 1) % setting_size;
    } else {                   // 値変更モード中 → 現在行の値をトグル
      menu_settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_SDDETAIL) {
    detail_page++;
    loading_sddetail = true;
    if (max_page <= 0)
      enqueueTask(createBrowseSDTask(0));
    else
      enqueueTask(createBrowseSDTask(detail_page % (max_page + 1)));  // ページをループ
  } else if (screen_mode == MODE_REPLAYSELECT) {
    // カーソルを次の項目へ（末尾まで行ったら先頭に戻る）
    int total = replay_menu_total_items();
    replay_cursor = (replay_cursor + 1) % total;
    int newpage = replay_menu_page_of(replay_cursor);
    if (newpage != replay_list_page) {
      // ページを跨いだので、そのページ分のファイル一覧を Core1 に取り直してもらう
      replay_list_page = newpage;
      loading_replaylist = true;
      enqueueTask(createBrowseReplayTask(replay_menu_file_start_for_page(newpage)));
    }
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL) {
    detail_page++;
  }
}

// ダブルクリックコールバック: 地図画面でのみスケール（ズームレベル）を切り替える。
// 短押しは毎回発火するため、ここでは短押しと重複しない処理だけを行う。
// 操作音は shortPressCallback() と同じ理由でここで鳴らす（効かない画面では鳴らさない）。
void doublePressCallback() {
  last_user_input_ms = millis();
  DEBUG_PLN(20240801, "double press");

  // MAP モード以外（設定画面や各詳細画面）では短押しがページ送り／カーソル移動を、
  // 長押しが実行を担っているのでダブルクリックには意味がない。音も鳴らさずに抜ける。
  if (screen_mode != MODE_MAP)
    return;

  enqueueTask(createPlayMultiToneTask(1568, 80, 1));  // ダブルクリック音（短押しより高い音）

  redraw_screen = true;
  next_scaleindex();  // MAP モード: スケールを次のレベルに切り替える
}

// 設定画面に戻る（リプレイ選択画面から抜けるとき用）。
// 汎用の「設定画面へ戻る」パスを通らないので、カーソル状態は自分でリセットする。
static void backToSettingFromReplay() {
  screen_mode = MODE_SETTING;
  selectedLine = -1;   // 値変更モードを解除（REPLAY 項目に入った時に立っている）
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);  // 画面を白でクリア
}

// 指定したファイルのリプレイ再生を開始し、地図画面へ戻る。
static void startReplay(const char* filename) {
  set_replay_filename(filename);
  set_replaymode(true);
  set_demo_off();       // デモとリプレイは同時使用不可 → デモを無効化
  latlon_manager.reset();       // 位置履歴をクリア
  reset_degpersecond();         // 旋回角速度をリセット
  enqueueTask(createInitReplayTask());  // Core1: 選択したファイルを開いて読み込み開始
  selectedLine = -1;
  exit_setting();               // 設定を保存して地図画面へ
}

// リプレイ選択画面での長押し（決定）を処理する。
static void handleReplaySelect() {
  char label[40];
  int  fsize = 0;
  ReplayItemType type = replay_menu_item(replay_cursor, replay_list_page, label, sizeof(label), &fsize);

  switch (type) {
    case RITEM_OFF:
      // リプレイを解除して通常の GPS に戻す
      set_replaymode(false);
      set_replay_filename("");
      latlon_manager.reset();
      reset_degpersecond();
      backToSettingFromReplay();
      break;
    case RITEM_FLIGHTONLY:
      // 静止区間をスキップするかを切り替える。画面はそのまま（続けて再生対象を選べる）
      set_replay_flight_only(!get_replay_flight_only());
      break;
    case RITEM_SPEED:
      // 再生速度を x1 → x2 → x?? と切り替える。再生中でも時刻は飛ばない
      cycle_replay_speed();
      break;
    case RITEM_2025:
      startReplay(REPLAY_2025_FILE);
      break;
    case RITEM_2026:
      startReplay(REPLAY_2026_FILE);
      break;
    case RITEM_FILE:
      startReplay(label);  // label には SD ルート上のファイル名が入っている
      break;
    case RITEM_RETURN:
      backToSettingFromReplay();
      break;
    default:
      break;  // 空行を選んでいる場合は何もしない
  }
}

// 長押しコールバック: 設定画面への出入り、または設定項目の確定/解除。
// 状態遷移:
//   MAP/他 → 長押し → SETTING 画面へ移行
//   SETTING（未選択）→ 長押し → 現在行を「値変更モード」に入る
//   SETTING（変更中）→ 長押し → 値変更モードを終了し確定する
void longPressCallback() {
  last_user_input_ms = millis();
  redraw_screen = true;

  // リプレイ選択画面での長押し = 「決定」。
  // 下の汎用パス（設定画面以外 → 設定画面へ戻る）より先に処理しないと項目を選べない。
  if (screen_mode == MODE_REPLAYSELECT) {
    handleReplaySelect();
    return;
  }

  if (screen_mode != MODE_SETTING) {
    // 設定画面以外から長押し → 設定画面へ遷移
    if (screen_mode == MODE_GPSDETAIL)
      gps_getposition_mode();  // GPS 詳細画面での位置取得モード解除
    screen_mode = MODE_SETTING;
    cursorLine = 0;
    selectedLine = -1;
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);  // 画面を白でクリア
  } else {
    // 設定画面内での長押し
    if (selectedLine == -1 && menu_settings[cursorLine].CallbackEnter != nullptr) {
      // CallbackEnter がある項目（サブ画面に入る項目）: Enter を実行して選択状態へ
      menu_settings[cursorLine].CallbackEnter();
      selectedLine = cursorLine;
    } else {
      if (selectedLine == -1) {
        // 値変更モードへ入る
        selectedLine = cursorLine;
      } else {
        // 値変更モードから抜ける（確定）
        if (menu_settings[cursorLine].CallbackExit != nullptr)
          menu_settings[cursorLine].CallbackExit();
        selectedLine = -1;
      }
    }
  }
}

// 旋回角速度をリセットする。GPS 受信開始時や停止時などに呼ぶ。
// サンプル配列を現在の真方位と現在時刻で埋めることで、急激な deg/s の跳ね上がりを防ぐ。
void reset_degpersecond() {
  float track = get_gps_truetrack();
  uint32_t now = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    truetrack_samples[i] = track;
    truetrack_sample_times[i] = now;
  }
  degpersecond = 0;
}

// 旋回角速度を更新する。GPS コース更新時（newcourse_arrived）に呼ぶ。
// アルゴリズム: スライディングウィンドウの方位変化合計を経過時間で正規化
//   1. 新しい真方位と現在時刻を配列に追加
//   2. 配列が満杯になったら連続サンプル間の差分を合計
//   3. 合計を「最古サンプルと最新サンプルの時間差（秒）」で割って真の deg/s を算出
//   4. 配列を1つずらして古いサンプルを捨てる
//   ※ 360度またぎ（例: 359→1度）を -180〜+180 に正規化して計算する
//   ※ 時間正規化により GPS レート（1Hz / 2Hz）に依存しない
void update_degpersecond(float true_track) {
  uint32_t now = millis();
  truetrack_samples[sampleIndex] = true_track;
  truetrack_sample_times[sampleIndex] = now;

  // サンプルが揃ったら真の deg/s を計算する
  if (sampleIndex >= NUM_SAMPLES - 1) {
    float totalDifference = 0;
    for (int i = 1; i < NUM_SAMPLES; i++) {
      float degchange = (truetrack_samples[i] - truetrack_samples[i - 1]);
      // 360度またぎを正規化（例: -359 → +1, +359 → -1）
      if (degchange < -180) {
        degchange += 360;
      } else if (degchange > 180) {
        degchange -= 360;
      }
      totalDifference += degchange;
    }
    // 最古サンプルと最新サンプルの時間差[ms]で正規化 → 真の deg/s
    uint32_t elapsed_ms = truetrack_sample_times[NUM_SAMPLES - 1] - truetrack_sample_times[0];
    if (elapsed_ms > 0) {
      degpersecond = totalDifference * 1000.0f / (float)elapsed_ms;
    } else {
      degpersecond = 0;
    }

    // 配列を1つ前にシフト（最古のサンプルを捨てる）
    for (int i = 1; i < NUM_SAMPLES; i++) {
      truetrack_samples[i - 1] = truetrack_samples[i];
      truetrack_sample_times[i - 1] = truetrack_sample_times[i];
    }
    sampleIndex = NUM_SAMPLES - 2;  // 次回は末尾に書き込む位置に調整
  }
  sampleIndex++;
}




// AUTO10K の折返しフェーズを変更する。
// ★ **リプレイ中とデモ中は表示だけ変え、SD には保存しない**（v7 の 0.986）。どちらも位置が
//   仮想なので、そこで出たフェーズを焼くと**次の起動が復路から始まる**。
//   デモは必ず踏む: 琵琶湖のデモは PLA を中心に 15km 走るので 10.475km を越え、
//   他の地点は PLATHOME から 100km 以上あるので**開始直後の 1 回目で成立する**。
// ★ 表示側の auto10k_status は動かすこと。抜けるときの復帰は
//   auto10k_restore_flight_phase()（gps.cpp の set_demo_site / set_replaymode）。
static void apply_auto10k_status(int st) {
  if (getReplayMode() || is_demo_active()) {
    auto10k_status = st;
    return;
  }
  set_auto10k_status_flight(st);
  // 飛行中に再起動しても復路のナビ方位が逆にならないよう、遷移のたびに保存する。
  // 遷移は 1 周あたり数回しか起きないので SD 書き込みの頻度は問題にならない。
  enqueueTask(createSaveSettingTask());
}

// ナビゲーションの判断と警報。**画面モードによらず loop() から毎回呼ぶこと**（v7 の 0.965）。
// 描画は一切しない（描画は MODE_MAP の分岐が担当する）。
//
// 内訳:
//   1. Auto10km の折返しフェーズ判定（GPS 位置が変わったときだけ）
//   2. 目的地が遠すぎる警告（同上）
//   3. 針路誤差・旋回角速度・旋回音・コース逸脱警報（GPS コース更新時だけ）
static void nav_alarm_tick(bool new_gps_info) {
  // 位置そのものをキーにして「本当に動いたときだけ」評価する。
  // new_gps_info のフラグを落とすのは MODE_MAP の描画ブロック（set_new_location_off()）
  // なので、地図画面以外ではフラグが立ちっぱなしになる。毎ループ Haversine を
  // 回さないよう、入力の変化で判定する。
  static double last_nav_lat = NAN, last_nav_lon = NAN;
  if (new_gps_info) {
    const double nav_lat = get_gps_lat();
    const double nav_lon = get_gps_lon();
    if (nav_lat != last_nav_lat || nav_lon != last_nav_lon) {
      last_nav_lat = nav_lat;
      last_nav_lon = nav_lon;

      // Automatic destination change for AUTO 10KM mode.
      // currentdestination の有効性も確認する。init_destinations() は Core1 の setup1() で走るため、
      // それが終わる前に Core0 がここへ来ると -1（未選択）のまま配列外を読んでしまう。
      if (destination_mode == DMODE_AUTO10K && currentdestination != -1 && currentdestination < destinations_count) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        double distance_frm_destination = calculateDistanceKm(nav_lat, nav_lon, destlat, destlon);
        // フェーズが変わったときは、起動直後・リプレイ切替直後であっても必ず鳴らす。
        // 保存した INTO を復元した状態でプラットホーム付近にいると、下の
        // 「1.5km 以内なら AWAY」が即座に成立して起動直後に鳴ることがあるが、
        // 黙って向きが変わるより、鳴らして確認を促す方が安全という判断。
        if (auto10k_status == AUTO10K_AWAY) {
          if (distance_frm_destination > 10.475) {  // 公式ルール 10.975km が折り返し地点だが、実際には潮流などの影響が影響があるため、500mの誤差を引いておく。
            apply_auto10k_status(AUTO10K_INTO);
            // ★ 最後の引数 true = solo_play。**下の INTO→AWAY と必ず揃えること。**
            //   solo_play にすると、4 音を鳴らし切ってから destination_change.wav が鳴る
            //   （通常トーンだと音声と重なって鳴る。docs/pons_sound.md §4）。
            //   往路→復路も復路→往路も同じ「目的地が切り替わった」通知なので、
            //   聞こえ方が違うと本番で取り違える。折り返しは 10km 飛んだ直後の
            //   一番集中している場面なので、トーンで注意を引いてから音声を流す。
            //
            // ★★ **ここを enqueueTaskWithAbortCheck にしないこと。**
            //   あれはキューの多重トーンを消してから積む。消す側に回ると、
            //   このチャイムの 2 秒の間にコース逸脱警報が発報したときに
            //   **向こうの 4 音だけが消えて course_right.wav だけが残る**
            //   （逆向きも同じ）。どちらの警報も消してよいものではないので、
            //   消さずに後ろへ並べる。Core0 は単スレッドなので 5 つは必ず連続して入る。
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
            enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
          }
        }
        if (auto10k_status == AUTO10K_INTO && distance_frm_destination < 1.5) {  //折り返し地点用。再度の折り返しは 1km だが、500mの誤差を足しておく。
          apply_auto10k_status(AUTO10K_AWAY);
          // solo_play。上の AWAY→INTO と同じ鳴り方にしてある（理由は上のコメント。
          // 打ち消さずに積むところも同じ）。
          enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
          enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
        }
      }
      check_destination_toofar();
    }
  }

  // GPS コース更新時（1Hz または 2Hz）にのみ実行する処理
  if (newcourse_arrived) {
    // ★ float で受けること。以前は int で、切り捨て誤差が窓の両端に残り
    //   deg/s に最大 ±0.67（誤差1度 ÷ 窓1.5秒）の量子化ノイズが乗っていた。
    //   着色 1.0 / 強調 3.0 / コース警報の「修正中」判定 0.5 deg/s という
    //   しきい値に対して無視できない大きさだった。
    //   truetrack_samples[] は元から float、GPS 側も NMEA の RMC の方位（小数 2 桁）
    //   なので、捨てていたのはここだけ。
    float ttrack = get_gps_truetrack();

    // ★ **この順番を入れ替えないこと。**
    //   nav_update() → steer_angle → update_course_warning() の順でなければ、
    //   警報は 1 周期古いズレ角で判定する。以前はまさにそうなっていて、
    //   Auto10km でフェーズが反転した直後の約 1 秒、警報が反転前のコースを見ていた。
    //   nav_update() は MODE_MAP の描画ブロックでも呼ぶが、これは Haversine と
    //   atan2 が 1 回ずつなので、2 度呼んでも負荷は問題にならない。
    nav_update();                       // truec（目的地への真方位）を最新 GPS 位置で更新

    // 針路誤差 (steer_angle) の計算:
    // truec はナビが指示する真方位コース、ttrack は GPS 実測の真方位。
    // どちらも小数を保った値なので、結果もそのまま使える（表示だけ丸める）。
    // 結果を -180〜+180 に正規化する。
    steer_angle = truec - ttrack;
    if (steer_angle < -180) {
      steer_angle += 360;
    } else if (steer_angle > 180) {
      steer_angle -= 360;
    }

    update_degpersecond(ttrack);        // 旋回角速度 [deg/s] を更新
    update_tone(degpersecond);          // 旋回音の音程を更新
    update_course_warning(degpersecond); // コース逸脱警告の積算値を更新（steer_angle を読む）

    newcourse_arrived = false;
  }
}

// GPS モジュールの断線／復帰を音と log.txt で知らせる（v7 の 0.979）。
// ★ 表示は draw_gnss_disconnect_warning()（地図の上にマゼンタの枠）だが、
//   **画面を見ていなければ気づけない**。飛行 CSV には「位置が止まった」ことしか
//   残らないので、原因が断線だったのかフィックス喪失だったのかを後から
//   切り分けられるようにする。判定は get_gps_connection()（gps.cpp）。
// ★ 起動直後（まだ 1 バイトも来ていない）では鳴らさない。一度でも繋がってから
//   切れたときだけ知らせる。最初の接続そのものは基準を置くだけで記録しない。
// ★ リプレイ中も実 GPS のバイトは読み捨てながら見ている（gps_loop）ので、判定は同じに働く。
static void gnss_conn_tick() {
  static bool     ever_conn = false, prev_conn = false;
  static uint32_t lost_alert_ms = 0;
  const bool conn = get_gps_connection();
  if (!ever_conn) {
    if (conn) { ever_conn = true; prev_conn = true; }
    return;
  }
  if (prev_conn != conn) {
    prev_conn = conn;
    if (conn) {
      enqueueTask(createLogSdTask("GNSS connection restored"));
      // ★ 復帰は断線の**逆向き**（392 → 784）。同じ 2 音の上昇と下降で対にする。
      enqueueTask(createPlayMultiToneTask(392, 120, 1, 2));
      enqueueTask(createPlayMultiToneTask(784, 160, 1, 2));
    } else {
      enqueueTask(createLogSdTask("!!ERROR!! GNSS connection LOST"));
    }
    lost_alert_ms = 0;   // 0 にしておくと下の繰り返しが即座に 1 回目を鳴らす
  }
  // ★ 切れている間は GNSS_LOST_ALERT_MS ごとに繰り返す。
  //   **下降 2 音（784 → 392、1 オクターブ下降）**。他の警報はすべて上昇か
  //   同音の連打なので、下降は「何かが失われた」専用の形になる。
  if (!conn && (lost_alert_ms == 0 || millis() - lost_alert_ms >= GNSS_LOST_ALERT_MS)) {
    lost_alert_ms = millis();
    enqueueTask(createPlayMultiToneTask(784, 200, 1, 3));
    enqueueTask(createPlayMultiToneTask(392, 300, 1, 3));
  }
}

// 電池電圧の低下警告。**loop() の最上位から毎回呼ぶこと**（v7 の 0.963）。
// ★ **警報を描画関数の中に置かないこと。** 元は draw_footer() の中にあったため、
//   draw_footer() を呼ばない画面（設定画面・各詳細画面）を開いている間は
//   電圧が下がっても鳴らなかった。
//   毎秒 1 回だけ読むのは、ADC の連打を避けるためと、get_input_voltage() の
//   ピーク追跡をどの画面でも回し続けるため（地図画面以外で誰も呼ばないと追跡が止まり、
//   戻ったときに古い値が出る）。
static void batt_alarm_tick() {
  static uint32_t last_batt_check_ms = 0;
  static uint32_t last_batt_warn_ms  = 0;
  if (millis() - last_batt_check_ms < 1000) return;
  last_batt_check_ms = millis();
  const float bv = get_input_voltage();
  // 充電中は鳴らさない。リプレイ中も鳴らさない（過去の電圧で今の警告を出しても
  // 意味がなく、SD のテキストログにも当時の値が「今の警告」として混ざるため）。
  // ★ USB の判定は usb_power_present()。壊れた充電 IC が VBUS を持ち上げると
  //   USB_DETECT が High になり、以前はここで警告まで止まっていた（power.cpp）。
  // ★ 電源経路の異常のあいだも鳴らさない（v7 の 0.993）。電圧が低いのは電池ではなく故障のせいで、
  //   「バッテリー低下」と言うと交換で済むと誤解させる（power_tick が別の音で鳴らす）。
  if (usb_power_present() || power_fault_active() || replay_voltage_active() ||
      bv > BAT_LOW_VOLTAGE) return;
  if (last_batt_warn_ms == 0 ? millis() < BAT_WARN_INTERVAL_MS
                             : millis() - last_batt_warn_ms < BAT_WARN_INTERVAL_MS) return;
  last_batt_warn_ms = millis();
  // 最低 volume 60 保証。音声は本体 FLASH 内蔵なので SD の有無に関係なく鳴る。
  enqueueTask(createPlayWavTask("wav/battery_low.wav", 1, 60));
  enqueueTask(createLogSdfTask("Battery low: %d%% (%.2fV)", battery_percent(bv), bv));
}

// 設定画面の放置タイムアウト（v7 の 0.965 / 0.992）。
// 設定画面を出したまま離陸してしまった場合の保険。nav_alarm_tick() が
// 画面によらず動くようになったので警報は鳴るが、地図が見えないままなのは変わらない。
// ボタン操作が一定時間無ければ、通常の「Save & Exit」と同じ経路で地図画面へ戻す。
static void setting_idle_tick() {
  if (screen_mode != MODE_SETTING) return;
  if (millis() - last_user_input_ms <= SETTING_IDLE_TIMEOUT_MS) return;
  enqueueTask(createLogSdfTask("SETTING SCREEN TIMEOUT -> MAP (%lus idle)",
                               (unsigned long)(SETTING_IDLE_TIMEOUT_MS / 1000UL)));
  // ★ 値変更モードのまま時間切れになった場合は、**必ず CallbackExit を通す。**
  //   長押しで抜ける通常経路（longPressCallback の「値変更モードから抜ける（確定）」）と
  //   同じ処理。ここには Auto10km と目的地の組み合わせチェックが入っている。
  //   飛ばすと、設定画面で禁止の組み合わせを作りかけたまま地図へ戻ることになる。
  if (selectedLine != -1 && menu_settings[cursorLine].CallbackExit != nullptr)
    menu_settings[cursorLine].CallbackExit();
  selectedLine = -1;
  // ★ **別れの音声は鳴らさない**（farewell=false）。操作していないのに声が鳴ると
  //   何が起きたのか分からない。代わりに専用の短い降下ペアを鳴らす。
  //   ボタン操作音とは別の音にしてある（あちらは「押した」、こちらは「勝手に閉じた」）。
  exit_setting(false);                                   // 設定を保存して MODE_MAP へ
  enqueueTask(createPlayMultiToneTask(659, 100, 1, 2));  // ミ
  enqueueTask(createPlayMultiToneTask(523, 100, 1, 2));  // ド
  last_user_input_ms = millis();
  // 画面クリアは不要。地図の再描画（ヘッダ＋backscreen＋フッタ）が 240x320 を
  // 埋め尽くすため、通常の「Save & Exit」でもクリアしていない。
  redraw_screen = true;
}

// 目的地が 100km 以上離れている場合に警告音を鳴らす。
// 120秒に1回に制限して、繰り返し鳴らしすぎないようにしている。
void check_destination_toofar() {
  // 目的地が未選択（起動直後は -1）なら配列外アクセスになるため何もしない
  if (currentdestination == -1 || currentdestination >= destinations_count) {
    return;
  }
  double destlat = extradestinations[currentdestination].cords[0][0];
  double destlon = extradestinations[currentdestination].cords[0][1];
  if (calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon) > 100.0) {
    if (millis() - last_destination_toofar_time > 1000 * 120) {  // 120秒クールダウン
      last_destination_toofar_time = millis();
      enqueueTask(createPlayWavTask("wav/destination_toofar.wav", 1));
    }
  }
}

// コース逸脱の警告を管理する（GPS コース更新時に呼ぶ、GPS レート非依存）。
// 積算型アルゴリズム（単位: 度・秒）:
//   - 前回呼び出しからの経過時間 dt[秒] を計測し、積算量に乗算することで GPS レート
//     （1Hz / 2Hz）に依存しない実時間ベースの累積を行う
//   - コースのズレ角 (steer_angle) × dt を course_warning_index に加算
//   - 正しい方向に修正中は 15 × |degpersecond| × dt を減算
//   - index が 900 に達したら音声警告を発報し、30秒のクールダウンに入る
//   - 低速・GPS ロスト時は発動しない（誤警告防止）
//   - 設定画面等からの復帰時に dt が過大になった場合はスキップ（誤発報防止）
void update_course_warning(float degpersecond) {
  // GPS レート非依存のため、前回呼び出しからの経過時間 dt[秒] を計算する
  static uint32_t last_course_warning_update_ms = 0;
  uint32_t now = millis();
  float dt = 0;
  if (last_course_warning_update_ms != 0) {
    dt = (now - last_course_warning_update_ms) / 1000.0f;
  }
  last_course_warning_update_ms = now;
  // 初回呼び出し or 設定画面/リプレイ切替等で長いギャップが発生した場合は
  // 次回から改めて積算するためスキップ（巨大 dt による誤発報防止）
  if (dt <= 0 || dt > 3.0f) {
    return;
  }

  //移動していない時,GPSロスト時は発動しない。
  if (get_gps_mps() < 2 || get_gps_numsat() == 0) {
    course_warning_index = 0;
  }
  //正しい方向に変化している時はindexを減らす。
  else if ((degpersecond > 0.5 && steer_angle > 0) || (degpersecond < -0.5 && steer_angle < 0)) {
    course_warning_index -= 15 * abs(degpersecond) * dt;  //修正速度×dt に比例して減算（1Hz相当で 3deg/s の時 45/秒）

  }
  //正しい方向に修正されていない、かつ15度以上ずれている。
  else if (abs(steer_angle) > 15) {
    course_warning_index += min(abs(steer_angle), 90) * dt;  //15以上、90を最大値として、dt を乗算して加算する。
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

  //15度の修正されないズレは、900/15=60秒でwarning。90度以上の修正されないズレは、10秒でwarning。ただしwarningは、30秒に一度をmax回数とする。
  if (course_warning_index >= 900 && millis() - last_course_warning_time > 30000) {
    last_course_warning_time = millis();
    course_warning_index = 0;
    // ★ ここも打ち消さない（Auto10km 側のコメント参照）。30 秒のクールダウンが
    //   あるので、積み上がって鳴り続けることは無い。
    enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2, 0, true));
    if (steer_angle > 0)
      enqueueTask(createPlayWavTask("wav/course_right.wav", 2));
    else
      enqueueTask(createPlayWavTask("wav/course_left.wav", 2));
  }
}