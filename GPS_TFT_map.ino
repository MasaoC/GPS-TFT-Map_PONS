// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : メインエントリポイント。
//           Core0: 画面描画・GPS処理・ボタン入力・コース警告
//           Core1: SDカード操作・音声再生（タスクキュー経由）
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/23
// ============================================================

#include "navdata.h"
#include "display_tft.h"
#include "settings.h"
#include "mysd.h"
#include "button.h"
#include "gps.h"
#include "sound.h"
#include "hardware/adc.h"
#include "airdata.h"
#include "imu.h"

// we need to do bool core1_separate_stack = true; to avoid stack running out.
// (Likely due to drawWideLine from TFT-eSPI consuming alot of stack.)
bool core1_separate_stack = true;  //DO NOT REMOVE THIS LINE.

// --- 画面更新タイミング管理 ---
unsigned long screen_update_time = 0;  // 最後に画面更新した時間 millis()
bool redraw_screen = false;            // true にすると次のループで画面を再描画する

// --- 旋回角速度 (deg/s) 計算用 ---
// GPS の真方位 (truetrack) を NUM_SAMPLES 個のスライディングウィンドウで保持し、
// 連続サンプル間の変化量の平均として degpersecond を算出する（GPS コース更新時＝newcourse_arrived 時に実行。GPS が毎秒更新なら結果的に毎秒更新）。
const int NUM_SAMPLES = 4;             // スライディングウィンドウのサンプル数
float truetrack_samples[NUM_SAMPLES];  // 過去の真方位サンプル配列
int sampleIndex = 0;                   // 次に書き込むサンプルのインデックス
float degpersecond = 0;                // 算出された旋回角速度 [deg/s]

// --- 画面モード・スケール管理 ---
int screen_mode = MODE_MAP;  // 現在の画面モード（MODE_MAP / MODE_SETTING など）
int detail_page = 0;         // サブ画面（GPSDetail / SDDetail）のページ番号
double scalelist[6];         // 選択可能なスケール値リスト（ズームレベルに対応）
double scale;                // 現在のマップスケール [pixels/km]

// --- 設定画面カーソル状態 ---
// selectedLine == -1: 値変更モードでない（カーソル移動のみ）
// selectedLine >= 0:  その行の値を変更中
int selectedLine = -1;
int cursorLine = 0;
int lastload_zoomlevel;  // 前回 BMP ロードを要求したズームレベル（変化検知用）

// --- コース警告 ---
// course_warning_index: 0〜900 の積算値。
// コースから外れているほど増加し、900 に達すると音声警告を発する。
int course_warning_index = 0;
unsigned long last_course_warning_time = 0;      // 直近の警告発報時刻 [millis]
unsigned long last_destination_toofar_time = 0;  // 直近の「目的地が遠すぎる」警告時刻 [millis]
double steer_angle = 0.0;  // 現在針路と目的地方位の差 (-180〜+180 度。正=右、負=左)

// --- 大気データ シリアル出力タイミング管理 ---
unsigned long last_airdata_print_time = 0;  // 最後に気温・気圧を Serial 出力した時間 [millis]
bool airdata_updated = false;               // airdata_update() が true を返した直後フラグ（VSI 更新トリガー）

// --- ユーザーLED 制御 ---
// userled_forced_on: タスクキュー溢れなど重篤エラー時に永続点灯させるフラグ（一度 true になったらリセットまで戻らない）
volatile bool userled_forced_on = false;

// ---- 処理時間計測変数（BNO085配置コア決定用、デバッグビルドのみ） ----
#ifndef RELEASE
TimingStat ts_redraw = TSTAT_INIT("C0_redraw");
volatile bool c0_is_redrawing = false;  // Core1側の重複検出用フラグ
#endif

// --- Core1 スタック残量計測用ベース SP ---
// setup1() の先頭で現在の SP をキャプチャし、以降の使用量計測の基準にする。
// core1_separate_stack=true のため Core1 のスタックはヒープから動的確保されており、
// リンカシンボルでは下限がわからないため、この方法で代替する。
volatile uint32_t _core1_base_sp = 0;

// Core1 からの BMP ロード完了を受けて次ループで即再描画させるための volatile フラグ
// （Core1 が書き込み、Core0 が読む。volatile で最適化を防ぐ）
volatile int scaleindex = 3;    // scalelist のインデックス（初期値 3 = SCALE_LARGE_GMAP）
volatile int sound_volume  = 50; // 音量 0〜100
volatile int vario_volume  = 10; // バリオメーター音量 0〜100（設定画面から変更可）
volatile bool vario_inhibit = false; // true のとき vario は完全無効（SDカード settings.txt でのみ設定可）

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

// USERLED フラッシュ制御（Core0 のループから毎回呼ぶ）。
// 条件ごとの LED 動作:
//   永続点灯 (userled_forced_on): タスクキュー溢れなどの致命エラー → 消灯しない
//   フラッシュ: 0衛星 or SDエラー → 1秒に1回、20ms だけ点灯
//   正常時: LED 消灯
void loop_userled() {
  if (userled_forced_on) return;  // 致命エラー時は永続点灯のまま

  bool should_flash = (get_gps_numsat() == 0) || !good_sd();

  if (!should_flash) {
    digitalWrite(USERLED_PIN, LOW);
    return;
  }

  // 1秒周期で 20ms だけ点灯
  static unsigned long last_flash_time = 0;
  unsigned long now = millis();
  if (now - last_flash_time >= 1000) {
    last_flash_time = now;
    digitalWrite(USERLED_PIN, HIGH);
  } else if (now - last_flash_time >= 20) {
    digitalWrite(USERLED_PIN, LOW);
  }
}

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

  // ユーザーLED 初期化（起動時は消灯）
  pinMode(USERLED_PIN, OUTPUT);
  digitalWrite(USERLED_PIN, LOW);
  // バッテリー電圧監視用 ADC の初期化
  adc_gpio_init(BATTERY_PIN);
  pinMode(USB_DETECT, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(SD_DETECT, INPUT_PULLUP);  // SDカード検出ピン（挿入時 LOW）
  analogReadResolution(12);  // 12bit = 0〜4095

  //setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  setup_tft();
  // BNO085 の NRST を早期に HIGH に固定する（内蔵プルアップ）。
  // RP2350 GPIO は電源投入直後フローティングになるため、imu_setup() が呼ばれるまでの間に
  // NRST が偶然 LOW になって BNO085 がリセット状態に入るのを防ぐ。
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
  pinMode(IMU_RST_PIN, INPUT_PULLUP);
#endif

  // I2C バス初期化（MS5611・BNO085 共用。両 setup より先に呼ぶ）
  airdata_wire_begin();
  // BNO085 を先に初期化する（リセット後のブート時間を確保するため）
  imu_setup();
  // MS5611 初期化（BNO085 の後に呼ぶ）
  airdata_setup();

  // 両センサーの初期化完了後にセンサー組み合わせをログ出力する
  if (!get_imu_ok() && !get_airdata_ok()) {
    enqueueTask(createLogSdTask("VARIO: NO SENSORS - BNO085 and MS5611 both absent"));
  } else if (get_imu_ok() && !get_airdata_ok()) {
    enqueueTask(createLogSdTask("VARIO: BNO085 only - Kalman inactive, V/S=0"));
  } else if (!get_imu_ok() && get_airdata_ok()) {
    enqueueTask(createLogSdTask("VARIO: MS5611 only - baro-only fallback"));
  } else {
    enqueueTask(createLogSdTask("VARIO: BNO085+MS5611 Kalman fusion enabled"));
  }

  gps_setup();
  
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
    float startup_voltage = min(BATTERY_MULTIPLYER(analogRead(BATTERY_PIN)), 4.3f);
    enqueueTask(createLogSdfTask("SETUP DONE Battery: %.2fV", startup_voltage));
  }

}


//===============SET UP CORE1=================
// Core1 の初期化処理。タスクキュー・音声・SD を準備する。
// Core0 と独立して動作し、重いSD処理・音声再生を引き受けることで
// Core0 の描画ループをブロックしない設計になっている。
void setup1(void) {
  Serial.begin(38400);
  #ifndef RELEASE
    while (!Serial) {
      delay(10);
    }
  #endif


    
  mutex_init(&taskQueueMutex);  // Core0/Core1 間のタスクキュー排他制御用 mutex を初期化
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



//===============MAIN LOOP CORE0=================
// Core0 のメインループ。以下の処理を毎ループ実行する:
//   1. ボタン状態の読み取り
//   2. GPS データの受信・解析（gps_loop は描画中にも分散して呼ぶ）
//   3. 画面モードに応じた描画（地図 / 設定 / GPS詳細 など）
// gps_loop(id) の id はデバッグ用の呼び出し箇所識別子。
void loop() {
  //switch handling
  sw_push.read();
  gps_loop(0);  // ループ先頭で GPS データを受信
  loop_userled();  // USERLED フラッシュ制御（0衛星・SDエラー時）

  // GPS Fix 取得時に音声で通知（false→true の立ち上がりエッジを検出）
  // SDカードが使えれば "wav/fixed.wav" を再生、使えなければチャイム音（ド・ミ・ソの上昇 3音）で代替する。
  {
    static bool prev_gps_fix = false;
    bool cur_fix = get_gps_fix();
    if (!prev_gps_fix && cur_fix) {
      if (good_sd()) {
        enqueueTask(createPlayWavTask("wav/fixed.wav", 4));  // 優先度4: AUTO10Kトーン(p=3)より高くして埋もれないよう
      } else {
        enqueueTask(createPlayMultiToneTask(523, 150, 1, 1));  // ド
        enqueueTask(createPlayMultiToneTask(659, 150, 1, 1));  // ミ
        enqueueTask(createPlayMultiToneTask(784, 300, 1, 1));  // ソ
      }
      enqueueTask(createLogSdTask("GPS FIX acquired"));
    }
    prev_gps_fix = cur_fix;
  }

  // 大気データ更新（非ブロッキングのステートマシン。毎ループ呼ぶことで約50Hzで計測）
  airdata_updated = airdata_update();

  // IMU 更新（H_INT フラグ確認 → データ読み出し → Kalman predict。毎ループ呼んでよい）
  imu_update();

  // ROTATION_VECTOR 更新時（5Hz）かつ GPS 日時有効時 → Euler角を JST 時刻で SD ログ
  if (get_imu_rv_updated() && get_gpsdate().isValid() && get_gpstime().isValid()) {
    // millis() オフセットで UTC 時刻を推定し JST（UTC+9）に変換
    uint32_t elapsed_ms = millis() - get_gps_fix_millis();
    int utc_cs = get_gpstime().centisecond() + (int)(elapsed_ms / 10);
    int utc_s  = get_gpstime().second()      + utc_cs / 100;
    int utc_m  = get_gpstime().minute()      + utc_s  / 60;
    int utc_h  = get_gpstime().hour()        + utc_m  / 60;
    int log_cs = utc_cs % 100;
    int log_s  = utc_s  % 60;
    int log_m  = utc_m  % 60;
    int jst_total_h = utc_h + 9;
    bool next_day   = (jst_total_h >= 24);
    int log_h       = jst_total_h % 24;

    // JST 日付計算（日をまたぐ場合に翌日へ繰り上げ）
    int jst_year  = get_gpsdate().year();
    int jst_month = get_gpsdate().month();
    int jst_day   = get_gpsdate().day() + (next_day ? 1 : 0);
    static const uint8_t days_in_month[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    int max_day = days_in_month[jst_month];
    if (jst_month == 2 && (jst_year % 4 == 0) && (jst_year % 100 != 0 || jst_year % 400 == 0))
      max_day = 29;  // うるう年
    if (jst_day > max_day) {
      jst_day = 1;
      jst_month++;
      if (jst_month > 12) { jst_month = 1; jst_year++; }
    }

    // ファイルパス生成: euler/yyyymmdd.txt
    char euler_fname[24];
    snprintf(euler_fname, sizeof(euler_fname), "euler/%04d%02d%02d.txt",
             jst_year, jst_month, jst_day);

    // Euler 角取得（参照渡し: roll, pitch, yaw [度]）
    float euler_roll, euler_pitch, euler_yaw;
    get_imu_euler(euler_roll, euler_pitch, euler_yaw);

    // モノトニック保証: GPS 2Hz パケット間の処理遅延の揺れでタイムスタンプが
    // 逆転することがあるため、前回より小さい場合はこの読み取りをスキップする。
    static int32_t prev_jst_cs = -1;
    int32_t this_jst_cs = (int32_t)log_h * 360000L + log_m * 6000 + log_s * 100 + log_cs;
    if (prev_jst_cs < 0 || this_jst_cs > prev_jst_cs) {
      prev_jst_cs = this_jst_cs;
      enqueueTask(createLogEulerTask(log_h, log_m, log_s, log_cs,
                                     euler_roll, euler_pitch, euler_yaw, euler_fname,
                                     jst_year, jst_month, jst_day));
    }
  }

  // 気圧高度が更新されたタイミングで Kalman 観測ステップを実行する
  // （predict は 50Hz で走り、update は気圧の更新レート ~40Hz で走る）
  if (airdata_updated) {
    imu_kalman_baro_update(get_airdata_altitude());
  }

  // GNSS高度による気圧基準補正（3Dフィックス有効時のみ、内部で1秒レート制限）
  // Vertical speed は変更せず、高度の長期ドリフトをゆっくり修正する。
  if (get_gps_gnssFixOK() && get_gps_fixtype() >= 3) {
    imu_kalman_gnss_update(
      (float)get_gps_altitude(),
      get_gps_vacc_mm() / 1000.0f
    );
    // GNSS 垂直速度を KF 速度観測として追加（vAcc ゲート、sAcc で R 計算）
    // BNO085 の有無によらず動作し、加減速中や BNO085 なし時のバリオ精度を補完する。
    imu_kalman_gnss_vel_update(
      get_gps_veld_mps(),
      get_gps_vacc_mm()   / 1000.0f,
      get_gps_sacc_mmps() / 1000.0f
    );
  }

  // バリオメーター音更新（内部で 100ms レート制限。毎ループ呼んでよい）
  update_vario();


#ifndef RELEASE
  // デバッグ時: PC シリアルから GPS モジュールへコマンドを転送可能
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
#endif

  // GPS 更新や BMP ロードがなくても、一定間隔で強制再描画する
  // （時刻表示など時間経過で変わる表示の更新保証）
  if ((millis() - screen_update_time > SCREEN_FRESH_INTERVAL)) {
    redraw_screen = true;
  }

  // 60秒ごとに電圧と JST 時刻をテキストログへ記録（どの画面モードでも実行）
  {
    static unsigned long last_volt_log_ms = 0;
    if (millis() - last_volt_log_ms >= 60000UL) {
      last_volt_log_ms = millis();
      GpsTime t = get_gpstime();
      int jst_h = (t._hour + 9) % 24;
      enqueueTask(createLogSdfTask("volt=%.2fV %02d:%02d JST", get_input_voltage(), jst_h, t._min));
    }
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
  } else if (screen_mode == MODE_VARIODETAIL) {
    if (redraw_screen) {
      draw_variodetail(detail_page);
      draw_vsi();                      // 全画面再描画直後に即座に VSI を上書き（白フリッカー防止）
      vsi_sprite.pushSprite(235, 40);
    }
    // airdata 更新タイミングで VSI バーを直接転写（メイン画面と同じ頻度）
    if (airdata_updated) {
      draw_vsi();
      vsi_sprite.pushSprite(235, 40);  // backscreen は y=40 から開始
    }
  } else if (screen_mode == MODE_MAP) {
    bool new_gps_info = gps_new_location_arrived();
    if (new_gps_info) {
      // Automatic destination change for AUTO 10KM mode.
      if (destination_mode == DMODE_AUTO10K) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        double distance_frm_destination = calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon);
        if (auto10k_status == AUTO10K_AWAY) {

          if (distance_frm_destination > 10.475) {  // 公式ルール 10.975km が折り返し地点だが、実際には潮流などの影響が影響があるため、500mの誤差を引いておく。
            auto10k_status = AUTO10K_INTO;
            enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 3));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3));
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3));
            enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
          }
        }
        if (auto10k_status == AUTO10K_INTO && distance_frm_destination < 1.5) {  //折り返し地点用。再度の折り返しは 1km だが、500mの誤差を足しておく。
          auto10k_status = AUTO10K_AWAY;
          enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
          enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
          enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
        }
      }
      check_destination_toofar();
    }

    // 毎秒1回: GPS からコース更新が届いた時のみ実行する処理
    if (newcourse_arrived) {
      int ttrack = get_gps_truetrack();

      update_degpersecond(ttrack);        // 旋回角速度 [deg/s] を更新
      update_tone(degpersecond);          // 旋回音の音程を更新
      update_course_warning(degpersecond); // コース逸脱警告の積算値を更新

      // 針路誤差 (steer_angle) の計算:
      // magc はナビが指示する磁気コース、-8 は機体取り付け角補正オフセット、
      // ttrack は GPS 実測の真方位。結果を -180〜+180 に正規化する。
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

    // GPS 更新と BMP ロード完了の両方で redraw_screen が立つため、
    // 通常は毎秒 2 回程度この描画ブロックが実行される。
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

      nav_update();   // 磁気コース(MC)・目的地距離(dist)を最新 GPS 位置で再計算
      draw_header();  // ヘッダー（速度・衛星数など）を TFT に直接描画

      new_gmap_ready = false;

      // ---- バックスクリーン（ダブルバッファ）を黒でクリア ----
      // 描画はすべてバックスクリーンに対して行い、最後に push_backscreen() で
      // 一括転送することでちらつきを防ぐ。
      clean_backscreen();

      // scaleindex → Google Map ズームレベルの変換
      int zoomlevel = 0;
      if (scaleindex == 0) zoomlevel = 5;   //SCALE_EXSMALL_GMAP
      if (scaleindex == 1) zoomlevel = 7;   //SCALE_SMALL_GMAP
      if (scaleindex == 2) zoomlevel = 9;   //SCALE_MEDIUM_GMAP
      if (scaleindex == 3) zoomlevel = 11;  //SCALE_LARGE_GMAP
      if (scaleindex == 4) zoomlevel = 13;  //SCALE_EXLARGE_GMAP

      // ---- レイヤー 1: Google Map 画像（BMP）を背景として描画 ----
      bool gmap_drawed = false;
      if (gmap_loaded_active)
        gmap_drawed = draw_gmap(drawupward_direction);

      // GPS 位置が変わった or ズームレベルが変わった時に新しい BMP を Core1 に要求
      if (new_gps_info || lastload_zoomlevel != zoomlevel) {
        lastload_zoomlevel = zoomlevel;
        // 前回のロードタスクがまだ実行中なら中断してから新タスクを追加する
        enqueueTaskWithAbortCheck(createLoadMapImageTask(new_lat, new_long, zoomlevel));
      }

      // ---- レイヤー 2: ベクターポリゴン地図 ----
      // 詳細ズーム時は現在地周辺のエリア別地図を描画。
      // 広域ズーム時は日本全体の海岸線ポリゴンを描画。
      if (scale > SCALE_SMALL_GMAP) {
        if (check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
          draw_Biwako(new_lat, new_long, scale, drawupward_direction, gmap_drawed);
        } else if (check_within_latlon(0.6, 0.6, new_lat, OSAKA_LAT, new_long, OSAKA_LON)) {
          draw_Osaka(new_lat, new_long, scale, drawupward_direction);
        } else if (check_within_latlon(0.6, 0.6, new_lat, SHINURA_LAT, new_long, SHINURA_LON)) {
          draw_Shinura(new_lat, new_long, scale, drawupward_direction);
        }
        draw_ExtraMaps(new_lat, new_long, scale, drawupward_direction);  // SD から読んだカスタム地図
      } else {
        // 広域スケール: 日本全国の海岸線を描画
        if (check_within_latlon(20, 40, new_lat, 35, new_long, 138)) {
          draw_Japan(new_lat, new_long, scale, drawupward_direction);
        }
      }

      gps_loop(4);  // 描画の合間に GPS データを受信（取りこぼし防止）

      // ---- レイヤー 3: 飛行軌跡 ----
      draw_track(new_lat, new_long, scale, drawupward_direction);

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
      if (scale > SCALE_SMALL_GMAP && check_within_latlon(0.6, 0.6, new_lat, PLA_LAT, new_long, PLA_LON)) {
        draw_pilon_takeshima_line(new_lat, new_long, scale, drawupward_direction);
      }

      // ---- レイヤー 5: オーバーレイ（コンパス・速度グラフ・スケールバーなど）----
      draw_compass(drawupward_direction, COLOR_BLACK);
      draw_degpersec(degpersecond);
      if (get_demo_biwako()) {
        draw_demo_biwako();  // 琵琶湖デモ表示（見た目や警告音などに慣れるための練習用）
      }
      if (getReplayMode()) {
        draw_replay_indicator();  // リプレイモード中であることを赤枠付きで通知
      }
      draw_km_distances(scale);  // 画面左下のスケールバー

      // GPS fix 状態と hAcc/gnssFixOK に応じて自機位置マーカーを切り替える
      // リプレイ・デモモードは実際の精度と無関係なので、常に通常の飛行機マーカーを表示する
      bool   cur_fix_ok = get_gps_gnssFixOK();
      float  cur_hacc_m = get_gps_hacc_mm() / 1000.0f;  // mm → m
      if (!get_gps_fix() && !get_demo_biwako() && !getReplayMode()) {
        draw_nofix_cross();                              // fix なし（通常モードのみ）: グレーの ×
      } else if (!get_demo_biwako() && !getReplayMode() &&
                 (!cur_fix_ok || cur_hacc_m >= HACC_THRESHOLD_M)) {
        draw_hacc_circle(scale, get_gps_hacc_mm());     // gnssFixOK=false または hAcc 不良: 青い不確かさ円
      } else {
        draw_triangle(new_truetrack, steer_angle);       // 精度良好、またはリプレイ/デモ: 飛行機マーカー
      }

      // コース警告表示（警告発報から 10 秒間だけ表示する）
      if (millis() - last_course_warning_time < 10000 && millis() > 10000) {
        draw_course_warning(steer_angle);
      }

      if (!gmap_drawed) {
        draw_nogmap(scale);  // BMP がない時は「地図なし」インジケータを表示
      }

      draw_gs_track();  // ヘッダーに速度・コースなどのテキストを描画
      draw_map_footer();
      draw_nomapdata();
      gps_loop(6);        // 描画の合間に GPS データを受信
      push_backscreen();  // バックスクリーンを TFT に一括転送（VSI合成済み）
      draw_footer();      // フッターは TFT に直接描画（バックスクリーン外）
      #ifndef RELEASE
      c0_is_redrawing = false;
      #endif
      TIMING_END(ts_redraw, redraw);
    }

    // backscreen 非更新時、airdata が更新されたタイミングで VSI を TFT へ直接転写
    // バリオ音量が 0 の場合は非表示
    if (!redraw_screen && vario_volume > 0 && !vario_inhibit && airdata_updated) {
      draw_vsi();
      vsi_sprite.pushSprite(235, 50);  // TFT座標: X=235, Y=50（backscreenオフセット）
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
  // 30秒ごとに各処理の最大・平均・回数を Serial 出力する（BNO085配置コア決定用）
  // 判断基準: max が 10000us (10ms) 未満 → そのコアで BNO085 割り込みを受けられる
  { static uint32_t _last_timing_report = 0;
    if (millis() - _last_timing_report >= 30000) {
      _last_timing_report = millis();
      Serial.println("===== TIMING REPORT =====");
      TIMING_REPORT(ts_redraw);
      extern TimingStat ts_draw_header, ts_push_backscreen;
      TIMING_REPORT(ts_draw_header);
      TIMING_REPORT(ts_push_backscreen);
      extern TimingStat ts_load_mapimage, ts_savecsv_flush;
      TIMING_REPORT(ts_load_mapimage);
      TIMING_REPORT(ts_savecsv_flush);
      extern uint32_t _c1_overlap_count;
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
          currentTask.saveCsvArgs.gs, currentTask.saveCsvArgs.ttrack, currentTask.saveCsvArgs.altitude,
          currentTask.saveCsvArgs.kf_altitude,
          currentTask.saveCsvArgs.kf_vspeed,
          currentTask.saveCsvArgs.pressure,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second,
          currentTask.saveCsvArgs.centisecond);
        break;
      case TASK_LOAD_MAPIMAGE:
        load_mapimage(
          currentTask.loadMapImageArgs.center_lat, currentTask.loadMapImageArgs.center_lon,
          currentTask.loadMapImageArgs.zoomlevel);
        break;
      case TASK_LOG_EULER:
        saveEuler(currentTask.logEulerArgs.hour,
                  currentTask.logEulerArgs.minute,
                  currentTask.logEulerArgs.second,
                  currentTask.logEulerArgs.centisecond,
                  currentTask.logEulerArgs.roll,
                  currentTask.logEulerArgs.pitch,
                  currentTask.logEulerArgs.yaw,
                  currentTask.logEulerArgs.filename,
                  currentTask.logEulerArgs.year,
                  currentTask.logEulerArgs.month,
                  currentTask.logEulerArgs.day);
        break;
      case TASK_LOAD_LOGO:
        load_push_logo();  // SD からロゴ BMP を gmap_sprite に読み込む（pushSprite は Core0 が行う）
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
// 短押しコールバック: 画面モードごとに動作が変わる。
//   MAP モード:      スケール（ズームレベル）を順番に切り替える
//   SETTING モード:  カーソル移動 or 値のトグル変更
//   SDDETAIL:       次のページを SD から読み込む
//   MAPLIST/GPSDETAIL: 次のページへ
void shortPressCallback() {
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");

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
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL || screen_mode == MODE_VARIODETAIL) {
    detail_page++;
  } else {
    // MAP モード: スケールを次のレベルに切り替える
    gmap_loaded_active = false;  // 旧 BMP を無効化（新スケールで再ロードする）
    scaleindex = (scaleindex + 1) % (sizeof(scalelist) / sizeof(scalelist[0]));
    scale = scalelist[scaleindex];
  }
}

// 長押しコールバック: 設定画面への出入り、または設定項目の確定/解除。
// 状態遷移:
//   MAP/他 → 長押し → SETTING 画面へ移行
//   SETTING（未選択）→ 長押し → 現在行を「値変更モード」に入る
//   SETTING（変更中）→ 長押し → 値変更モードを終了し確定する
void longPressCallback() {
  redraw_screen = true;

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
// サンプル配列を現在の真方位で埋めることで、急激な deg/s の跳ね上がりを防ぐ。
void reset_degpersecond() {
  float track = get_gps_truetrack();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    truetrack_samples[i] = track;
  }
  degpersecond = 0;
}

// 旋回角速度を更新する。毎秒1回呼ぶこと。
// アルゴリズム: スライディングウィンドウ平均
//   1. 新しい真方位を配列に追加
//   2. 配列が満杯になったら連続サンプル間の差分を合計し平均を出す
//   3. 配列を1つずらして古いサンプルを捨てる
//   ※ 360度またぎ（例: 359→1度）を -180〜+180 に正規化して計算する
void update_degpersecond(int true_track) {
  truetrack_samples[sampleIndex] = true_track;

  // サンプルが揃ったら平均 deg/s を計算する
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
    degpersecond = totalDifference / (NUM_SAMPLES - 1);

    // 配列を1つ前にシフト（最古のサンプルを捨てる）
    for (int i = 1; i < NUM_SAMPLES; i++) {
      truetrack_samples[i - 1] = truetrack_samples[i];
    }
    sampleIndex = NUM_SAMPLES - 2;  // 次回は末尾に書き込む位置に調整
  }
  sampleIndex++;
}



// 目的地が 100km 以上離れている場合に警告音を鳴らす。
// 120秒に1回に制限して、繰り返し鳴らしすぎないようにしている。
void check_destination_toofar() {
  double destlat = extradestinations[currentdestination].cords[0][0];
  double destlon = extradestinations[currentdestination].cords[0][1];
  if (calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon) > 100.0) {
    if (millis() - last_destination_toofar_time > 1000 * 120) {  // 120秒クールダウン
      last_destination_toofar_time = millis();
      enqueueTask(createPlayWavTask("wav/destination_toofar.wav", 1));
    }
  }
}

// コース逸脱の警告を管理する（毎秒1回呼ぶこと）。
// 積算型アルゴリズム:
//   - コースのズレ角 (steer_angle) に比例して course_warning_index を増加させる
//   - 正しい方向に修正中はインデックスを減らす
//   - index が 900 に達したら音声警告を発報し、30秒のクールダウンに入る
//   - 低速・GPS ロスト時は発動しない（誤警告防止）
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
    enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(2793, 500, 1, 2, 0, true));
    enqueueTask(createPlayMultiToneTask(3136, 500, 1, 2, 0, true));
    if (steer_angle > 0)
      enqueueTask(createPlayWavTask("wav/course_right.wav", 2));
    else
      enqueueTask(createPlayWavTask("wav/course_left.wav", 2));
  }
}