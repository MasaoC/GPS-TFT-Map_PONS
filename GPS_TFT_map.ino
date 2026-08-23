// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : メインエントリポイント。
//           Core0: 画面描画・GPS処理・ボタン入力・コース警告
//           Core1: SDカード操作・音声再生（タスクキュー経由）
//           地図背景はフラッシュ内蔵のベクタ地図（vectormap.cpp）を使う。
//           SDカード上のBMPタイル方式は廃止済み。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
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
#include "imulog.h"
#include "attitude.h"
#include "vectormap.h"

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
// IMU/ESKF 画面（ページ1）のカーソル位置。display_tft.cpp の IMU_MENU_* と対応。
int imu_cursor = 0;
int imu2_cursor = 0;   // IMU/ESKF ページ2 のカーソル位置
// バンク角警告の有効/無効（IMU/ESKF 画面で切替、SD に保存）
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
void doublePressCallback();
static void imu_execute();
void next_scaleindex();
// 設定画面のラベル用に、現在のスケールで画面横幅(240px)が何 km に相当するかを返す。
// 以前は Google タイルのズーム番号(zoom5〜13)を表示していたが、ベクタ地図に移行して
// タイルとの対応が無くなったため、実際の距離で表すようにした。
float scale_screen_km(int index);
// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback, doublePressCallback);

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
  attitude_setup();   // 姿勢 ESKF（センサー入力は imu.cpp/gps.cpp から流し込む）
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



// ============================================================
// get_jst_now(): GPS の UTC 日時から現在の JST 日時を求める
// ============================================================
// GPS パケット受信時刻からの millis() 経過分を足して現在時刻を推定し、
// UTC+9 の JST に変換する。日をまたぐ場合は日付を繰り上げる（うるう年考慮）。
// Euler 角ログと生 IMU ログの両方が同じ日付のファイル名を使うため関数化している。
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
  bool next_day   = (jst_total_h >= 24);
  h = jst_total_h % 24;

  // JST 日付計算（日をまたぐ場合に翌日へ繰り上げ）
  y  = get_gpsdate().year();
  mo = get_gpsdate().month();
  d  = get_gpsdate().day() + (next_day ? 1 : 0);
  static const uint8_t days_in_month[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  int max_day = days_in_month[mo];
  if (mo == 2 && (y % 4 == 0) && (y % 100 != 0 || y % 400 == 0))
    max_day = 29;  // うるう年
  if (d > max_day) {
    d = 1;
    mo++;
    if (mo > 12) { mo = 1; y++; }
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
  // 生 IMU ログを止めるのはリプレイ中だけ
  //（Euler ログと同じ理由: 再生日時のファイルを実センサ値で汚さない）。
  //
  // ※ GPS 日時の有効性では止めないこと。
  //   以前は日時未確定でも止めていたが、それだと屋内など測位できない場所で
  //   IMU の生ログが一切取れず（wrote=0）、ベンチでの検証ができなかった。
  //   日時が無いときは下でファイル名を imuraw/nofix.bin にフォールバックする。
  //   レコードはすべてホスト時刻 t_us を持つので、日付が無くても解析はできる。
  imulog_set_paused(getReplayMode());
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
  // リプレイ中は記録しない。リプレイ中の日時は CSV 由来のため、そのまま記録すると
  // 「再生した飛行の日付」のファイルに現在の（静止した）IMU 値を追記してしまい、
  // 実際の飛行記録を汚してしまう。下の prev_jst_cs による単調増加チェックも
  // 再生日時に引きずられて、通常モードに戻ったあと記録が止まる原因になる。
  int jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs;
  bool jst_valid = get_jst_now(jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs);

  // ---- リプレイ用 ESKF 結果ログ（画面の再現に使う）----
  // ESKF が未収束の間は書かない。行が無い＝そのとき画面にも出ていなかった、を意味する。
  // 契機は BNO085 のレポート到着ではなく固定周期にする。ESKF の出力は連続なので、
  // レポートの到着ゆらぎで間隔がばらつくと再生が不均一になるため。
  // 保険として実測の GNSS フィックスも要求する。日時が再生由来のまま残っていると
  // 過去の日付のファイルへ机の上の値を書き込んでしまうため（set_replaymode() 参照）。
  static uint32_t last_replaydata_ms = 0;
  if (!getReplayMode() && jst_valid && get_gps_gnssFixOK() && attitude_ready() &&
      attitude_get_rpy_enabled() &&
      (millis() - last_replaydata_ms) >= IMU_REPLAYDATA_INTERVAL_MS) {

    // モノトニック保証: GPS パケット間の処理遅延の揺れでタイムスタンプが
    // 逆転することがあるため、前回より小さい場合はこの読み取りをスキップする。
    static int32_t prev_jst_cs = -1;
    int32_t this_jst_cs = (int32_t)log_h * 360000L + log_m * 6000 + log_s * 100 + log_cs;
    if (prev_jst_cs < 0 || this_jst_cs > prev_jst_cs) {
      prev_jst_cs = this_jst_cs;
      last_replaydata_ms = millis();

      char rd_fname[32];   // "imu_replaydata/20260316.txt" = 27文字
      snprintf(rd_fname, sizeof(rd_fname), "%s/%04d%02d%02d.txt",
               IMU_REPLAYDATA_DIR, jst_year, jst_month, jst_day);

      // 画面に出ているのと同じ値を残す（マウント補正・ゼロ点・自動トリム適用後）
      float a_roll, a_pitch, a_yaw;
      attitude_get_euler(a_roll, a_pitch, a_yaw);

      // 風は推定できていない区間があるので、有効フラグ付きで渡す（無効なら空欄で書かれる）
      float a_wspd = 0.0f, a_wdir = 0.0f;
      const bool a_wind_ok = attitude_get_wind(a_wspd, a_wdir);

      enqueueTask(createLogImuReplayTask(log_h, log_m, log_s, log_cs,
                                         a_roll, a_pitch, a_yaw,
                                         attitude_get_pitch_avg_deg(),
                                         attitude_pitch_avg_valid(),
                                         attitude_get_roll_trim_deg(),
                                         attitude_get_yaw_acc95_deg(),
                                         a_wspd, a_wdir, a_wind_ok,
                                         rd_fname,
                                         jst_year, jst_month, jst_day));
    }
  }

  // ---- 生 IMU ログ: 満杯になったバッファを Core1 へ引き渡す ----
  // imulog_take_pending() が 0/1 を返したら、そのバッファは「Core1 へ引き渡し済み」に
  // マークされているため、必ずタスクを enqueue しないとバッファが解放されず Core0 が詰まる。
  // リプレイ中は記録しない（Euler ログと同じ理由: 再生日時のファイルを実センサ値で汚さない）。
  if (!getReplayMode()) {
    int imulog_buf = imulog_take_pending();
    if (imulog_buf >= 0) {
      char imu_fname[24];
      bool queued;
      if (jst_valid) {
        snprintf(imu_fname, sizeof(imu_fname), "imuraw/%04d%02d%02d.bin",
                 jst_year, jst_month, jst_day);
        queued = enqueueTask(createFlushImuLogTask(imulog_buf, imu_fname,
                                                  jst_year, jst_month, jst_day,
                                                  log_h, log_m, log_s));
      } else {
        // GPS 未測位（屋内テスト等）。日付が決まらないので固定名へ書く。
        // レコードはホスト時刻 t_us を持つので、これでも解析はできる。
        // ファイルのタイムスタンプは SdFat の下限（1980 年以降）を満たす固定値にする。
        snprintf(imu_fname, sizeof(imu_fname), "imuraw/nofix.bin");
        queued = enqueueTask(createFlushImuLogTask(imulog_buf, imu_fname,
                                                  2020, 1, 1, 0, 0, 0));
      }
      // キュー満杯でタスクが捨てられたらバッファを戻す。
      // 戻さないと busy のままになり、2 枚とも失った時点で記録が恒久停止する。
      if (!queued) imulog_release_pending(imulog_buf);
    }
  }

  // 気圧高度が更新されたタイミングで Kalman 観測ステップを実行する
  // ※ update の実レートは約 3.7Hz。~40Hz ではない（2026-08-18 に実ログで確認）。
  //   MS5611 自体は ~40Hz でサンプルするが、airdata_update() が true を返すのは
  //   VSPEED_WINDOW_MS(250ms) のトリム平均ウィンドウが完了したときだけのため
  //   （プロペラ干渉 ~4.5Hz を除去する意図的な設計。airdata.cpp 参照）。
  //   したがって predict:update の比はおよそ 33:3.7 ≒ 9:1 になる。
  if (airdata_updated) {
    imu_kalman_baro_update(get_airdata_altitude());
    // 生 IMU ログにも気圧を残す（ESKF の検証と高度の突き合わせ用）
    imulog_push(IMULOG_ID_BARO, 0, IMULOG_ACC_NONE,
                get_airdata_pressure(), get_airdata_altitude(), get_airdata_temperature());
  }

  // GNSS高度による気圧基準補正（3Dフィックス有効時のみ、内部で1秒レート制限）
  // Vertical speed は変更せず、高度の長期ドリフトをゆっくり修正する。
  //
  // リプレイ中は実行しない。リプレイ中の GNSS 高度は CSV 由来（再生した飛行場所の高度）
  // であり、GNSS 垂直速度に至っては更新されず直前の実測値のまま残る。これらを Kalman に
  // 入れると airdata_adjust_ground_alt() が実際の気圧基準を書き換え、_gnss_kf_offset も
  // ずれてしまう。どちらもリプレイ終了後まで残り、通常モードに戻っても V/S が
  // 再生中のように動き続ける原因になる。
  if (!getReplayMode() && get_gps_gnssFixOK() && get_gps_fixtype() >= 3) {
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

  // ---- バンク角の警告 ----
  // 実測（2026-07-26 琵琶湖 45分）では 4 度超えが 31 回あったが、中央継続は 0.2 秒。
  // 継続条件を付けないとチカチカ鳴るだけなので 1 秒以上続いたときだけ発報する
  // （同条件で実測 3 回）。さらに一度鳴ったら 60 秒は繰り返さない。
  // 姿勢が壊れているときに鳴り続けるのを防ぐのが主目的。
  if (attitude_get_rpy_enabled() && attitude_ready() && !getReplayMode()) {
    static uint32_t bank_over_since_ms = 0;
    static uint32_t bank_last_warn_ms  = 0;
    // 発報済みフラグ。解除条件（バンクが戻る＋時間経過）を満たすまで鳴らさない。
    // ★ true で始める。起動時点で既に 4 度を超えていた場合（マウントのズレ、
    //   傾いた場所に置いてある、較正前など）に、一度も水平に戻らないまま
    //   鳴り出すのを防ぐため。一度でも 3 度未満に戻れば通常動作に入る。
    static bool     bank_warned        = true;
    float wr, wp, wy;
    attitude_get_euler(wr, wp, wy);
    uint32_t now_ms = millis();
    const float bank_abs = fabsf(wr);

    // 再武装。バンクが BANK_WARN_CLEAR_DEG 未満まで戻り、かつ発報から
    // BANK_WARN_REARM_MS 以上経ってから。バンクが超過したままなら何時間でも鳴らさない
    // （センサーのズレで超過が続く不具合を想定しているため、時間だけでは解除しない）。
    if (bank_warned && bank_abs < BANK_WARN_CLEAR_DEG &&
        (now_ms - bank_last_warn_ms) >= BANK_WARN_REARM_MS) {
      bank_warned = false;
    }

    if (bank_abs > BANK_WARN_DEG) {
      if (bank_over_since_ms == 0) bank_over_since_ms = now_ms;
      if (!bank_warned && (now_ms - bank_over_since_ms) >= BANK_WARN_HOLD_MS) {
        bank_last_warn_ms = now_ms;
        bank_warned = true;
        enqueueTask(createPlayWavTask("wav/bank_warning.wav", 3));
        enqueueTask(createLogSdfTask("BANK WARN roll=%+.1f deg", wr));
      }
    } else {
      bank_over_since_ms = 0;   // 一度でも下回ったら継続時間をリセット
    }
  }

  // ---- 直進中のロール自動トリムが入ったらログに残す ----
  // 事後検証できるよう、1 回の補正量と累積量を記録する。
  // ---- 地上待機中のバンクチェック ----
  // 自動ロールトリムは対地速度 3m/s 超でしか働かないので地上では動かない。
  // しかしマウントのズレを直せるのは地上だけなので、ここで別建てに見張る。
  // 静止していることを条件に入れて、運搬中・組み立て中の傾きを除外する。
  if (attitude_get_rpy_enabled() && attitude_ready() && !getReplayMode() &&
      get_gps_mps() <= LEVEL_CALIB_MAX_MPS && attitude_is_static()) {
    static uint32_t gnd_over_since_ms = 0;
    static uint32_t gnd_last_warn_ms  = 0;
    static uint32_t gndp_over_since_ms = 0;
    static uint32_t gndp_last_warn_ms  = 0;
    float gr, gp, gy;
    attitude_get_euler(gr, gp, gy);
    const uint32_t now_ms = millis();

    // ロール: 0（または SET ROLL で申告した値）からのズレ
    if (fabsf(gr - attitude_get_roll_target()) > GROUND_ROLL_WARN_DEG) {
      if (gnd_over_since_ms == 0) gnd_over_since_ms = now_ms;
      if ((now_ms - gnd_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
          (gnd_last_warn_ms == 0 ||
           (now_ms - gnd_last_warn_ms) >= GROUND_ROLL_WARN_INTERVAL_MS)) {
        gnd_last_warn_ms = now_ms;
        enqueueTask(createPlayWavTask("wav/roll_check.wav", 3));
        enqueueTask(createLogSdfTask("GROUND ROLL %+.1f deg (check mount / APPLY)", gr));
      }
    } else {
      gnd_over_since_ms = 0;   // 一度でも下回ったら継続時間をリセット
    }

    // ピッチ: 申告値 SET PITCH からのズレで見る。
    // 地上の機体ピッチは場所によって変わる（プラットホーム -3.5 度、平地 0 度）ので、
    // ロールのように絶対値 0 を基準にはできない。
    if (fabsf(gp - attitude_get_pitch_target()) > GROUND_PITCH_WARN_DEG) {
      if (gndp_over_since_ms == 0) gndp_over_since_ms = now_ms;
      if ((now_ms - gndp_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
          (gndp_last_warn_ms == 0 ||
           (now_ms - gndp_last_warn_ms) >= GROUND_ROLL_WARN_INTERVAL_MS)) {
        gndp_last_warn_ms = now_ms;
        enqueueTask(createPlayWavTask("wav/pitch_check.wav", 3));
        enqueueTask(createLogSdfTask("GROUND PITCH %+.1f deg (target %+.1f)",
                                     gp, attitude_get_pitch_target()));
      }
    } else {
      gndp_over_since_ms = 0;
    }
  }

  // APPLY の待ち時間は「指を離してから」数える。
  // 長押しコールバックは閾値に達した時点で呼ばれるので、押しっぱなしのまま
  // 実行されてしまう（3 秒押すと押下中に焼き付いていた）。押している間は
  // カウントダウンを進めず、離した瞬間から改めて 1 秒待つ。
  {
    static bool calib_btn_held = false;
    if (attitude_calib_pending()) {
      if (digitalRead(SW_PUSH) == LOW) {        // 押下中（アクティブ LOW）
        attitude_hold_calibrate();
        calib_btn_held = true;
      } else if (calib_btn_held) {
        calib_btn_held = false;
        // 離した合図。ここから 1 秒じっとしていれば較正される。
        enqueueTask(createPlayMultiToneTask(2093, 60, 1));
      }
    } else {
      calib_btn_held = false;
    }
  }

  // 予約していた APPLY が実行された
  if (attitude_take_calib_done()) {
    enqueueTask(createSaveSettingTask());                  // 次回起動でも効くよう保存
    // 成功トーン（上がり 2 音）。SD に eskf_apply_done.wav が無くても、
    // 較正が実行されたことが分かるようにする。WAV の直前に鳴らす。
    enqueueTask(createPlayMultiToneTask(1568, 70, 1));
    enqueueTask(createPlayMultiToneTask(2093, 90, 1));
    // 較正できたことを音声で知らせる。優先度 3 = 案内音声と同等。
    // 較正の成否は取り違えると危険なので埋もれさせない。
    enqueueTask(createPlayWavTask("wav/eskf_apply_done.wav", 3));
    enqueueTask(createLogSdfTask("ESKF APPLY done"));
  }

  {
    float applied, total;
    if (attitude_take_roll_trim_event(applied, total)) {
      enqueueTask(createLogSdfTask("ROLL TRIM %+.2f deg (total %+.2f)", applied, total));
    }
  }


#ifndef RELEASE
  // デバッグ時: PC シリアルから GPS モジュールへコマンドを転送可能
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
#endif

  // GPS 更新や BMP ロードがなくても、一定間隔で強制再描画する
  // （時刻表示など時間経過で変わる表示の更新保証）
  //
  // VARIO 詳細はセンサー値を読むための画面で、他に再描画トリガーが無いため、
  // この間隔がそのまま表示の更新レートになる。地図画面と同じ 1050ms だと
  // 約 0.95Hz しか出ないので、この画面だけ短い間隔を使う。
  {
    unsigned long fresh_interval =
        (screen_mode == MODE_VARIODETAIL || screen_mode == MODE_IMUDETAIL)
        ? SCREEN_FRESH_INTERVAL_DETAIL
        : SCREEN_FRESH_INTERVAL;
    if (millis() - screen_update_time > fresh_interval) {
      redraw_screen = true;
    }
  }

  // 60秒ごとに電圧と JST 時刻をテキストログへ記録（どの画面モードでも実行）
  {
    static unsigned long last_volt_log_ms = 0;
    if (millis() - last_volt_log_ms >= 60000UL) {
      last_volt_log_ms = millis();
      GpsTime t = get_gpstime();
      int jst_h = (t._hour + 9) % 24;
      enqueueTask(createLogSdfTask("volt=%.2fV cpu=%.1fC %02d:%02d JST", get_input_voltage(), analogReadTemp(), jst_h, t._min));

      // センサー受信レートも 60 秒ごとに残す（RELEASE ビルドでもシリアルなしで確認できる）。
      // 生レポートのレート要求を変えたら必ずこの行を確認すること:
      //   GRV=15.0 LACC=15.0 RV=5.0 なら正常。
      //   これらが設定値より低ければ → BNO085 の配信飢餓。要求レートが過大で、
      //     古い加速度を繰り返し積分してバリオが暴れる（2026-08-17 の回帰）。
      //   stale が増えていれば → 上記の飢餓が実際に起きて predict を止めた回数。
      //   MS5611 が ~40Hz から落ちていれば → 気圧観測レートの低下。
      //   drop が増えていれば → SD 書き出しが追いつかず生ログを取りこぼしている。
      // ※ 2 行に分けること。log_sd() は logtext[128] に "<起動秒>:" を前置するため、
      //   1 行にまとめると稼働時間が延びるほど末尾が切り詰められる（実際 wrote= が消えた）。
      enqueueTask(createLogSdfTask("rate GRV=%.1f LACC=%.1f RV=%.1f MS5611=%.1f Hz",
                                   get_imu_grv_hz(), get_imu_lacc_hz(), get_imu_rv_hz(),
                                   get_airdata_win_hz()));
      enqueueTask(createLogSdfTask("raw GYR=%.1f ACC=%.1f MAG=%.1f Hz stale=%lu drop=%lu wrote=%lu",
                                   get_imu_gyro_hz(), get_imu_accel_hz(), get_imu_mag_hz(),
                                   (unsigned long)get_imu_lacc_stale_skips(),
                                   (unsigned long)imulog_get_dropped(),
                                   (unsigned long)imulog_get_written()));
      // ヨーの不確かさを飛行後に確認できるよう残す。
      // 生ログには σ そのものは入っていないが、tools/imulog/eskf.py を流せば
      // 50Hz 全分解能で再現できる。こちらは 60 秒ごとの粗い突き合わせ用。
      {
        float lvr, lvp;
        attitude_get_level_offset(lvr, lvp);
        enqueueTask(createLogSdfTask("eskf ready=%d yawsig=%.1f mag=%d lvl R%+.1f P%+.1f",
                                     (int)attitude_ready(), attitude_get_yaw_sigma_deg(),
                                     (int)attitude_yaw_from_mag(), lvr, lvp));
      }
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
  } else if (screen_mode == MODE_REPLAYSELECT) {
    if (replay_loading_displayed && !loading_replaylist)
      redraw_screen = true;  // Core1 のファイル一覧取得が完了したので描き直す
    if (redraw_screen)
      draw_replayselect(replay_list_page, replay_cursor);
  } else if (screen_mode == MODE_MAPLIST) {
    if (redraw_screen)
      draw_maplist_mode(detail_page);
  } else if (screen_mode == MODE_IMUDETAIL) {
    if (redraw_screen)
      draw_imudetail(detail_page);
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
      // currentdestination の有効性も確認する。init_destinations() は Core1 の setup1() で走るため、
      // それが終わる前に Core0 がここへ来ると -1（未選択）のまま配列外を読んでしまう。
      if (destination_mode == DMODE_AUTO10K && currentdestination != -1 && currentdestination < destinations_count) {
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

    // GPS コース更新時（1Hz または 2Hz）にのみ実行する処理
    if (newcourse_arrived) {
      int ttrack = get_gps_truetrack();

      update_degpersecond(ttrack);        // 旋回角速度 [deg/s] を更新
      update_tone(degpersecond);          // 旋回音の音程を更新
      update_course_warning(degpersecond); // コース逸脱警告の積算値を更新

      // 針路誤差 (steer_angle) の計算:
      // truec はナビが指示する真方位コース、ttrack は GPS 実測の真方位。
      // 結果を -180〜+180 に正規化する。
      // ※ 以前は (magc - 8) と書いていたが、magc は真方位に +8 した値だったので
      //   -8 はそれを打ち消していただけ。つまり計算は元から真方位同士で、
      //   真方位へ統一しても操縦指示の挙動は変わらない。
      steer_angle = truec - ttrack;
      if (steer_angle < -180) {
        steer_angle += 360;
      } else if (steer_angle > 180) {
        steer_angle -= 360;
      }

      newcourse_arrived = false;
    }


    if (new_gps_info) {
      redraw_screen = true;
    }

    // GPS 更新のたびに redraw_screen が立つため、
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

      // GPS fix 状態と hAcc/gnssFixOK に応じて自機位置マーカーを切り替える
      // リプレイ・デモモードは実際の精度と無関係なので、常に通常の飛行機マーカーを表示する
      bool   cur_fix_ok = get_gps_gnssFixOK();
      float  cur_hacc_m = get_gps_hacc_mm() / 1000.0f;  // mm → m
      if (!get_gps_fix() && !is_demo_active() && !getReplayMode()) {
        draw_nofix_cross();                              // fix なし（通常モードのみ）: グレーの ×
      } else if (!is_demo_active() && !getReplayMode() &&
                 (!cur_fix_ok || cur_hacc_m >= HACC_THRESHOLD_M)) {
        draw_hacc_circle(scale, get_gps_hacc_mm());     // gnssFixOK=false または hAcc 不良: 青い不確かさ円
      } else {
        draw_triangle(new_truetrack, steer_angle);       // 精度良好、またはリプレイ/デモ: 飛行機マーカー
      }

      // コンパス(N/E/S/W)は自機マーカー・進行方向縦線より後に描いて上のレイヤーにする。
      draw_compass(drawupward_direction, COLOR_BLACK);

      // ESKF のロール・ピッチ（リプレイ中を除き常時表示）。
      // ※ 各種ポップアップより先に描くこと。コース警告のボックス（y=175 または 192 から
      //   高さ25）は ESKF 行と重なるため、後から描かれる側が上になる。
      //   状態通知のほうが優先度が高いので、ESKF は下のレイヤーに置く。
      // push の前に呼んでバックスクリーンへ合成する（TFT へ直接描くとちらつくため）。
      draw_eskf_attitude();
      // ヨー角は自機アイコンの真下。TRACKUP ではロール・ピッチ行と近いので、
      // 必ず draw_eskf_attitude() の後に描いてこちらを上のレイヤーにする。
      draw_eskf_yaw();
#ifdef DEBUG_ESKF
      draw_eskf_debug();   // 比較用の詳細。上の表示位置は変えない
#endif

      // コース警告表示（警告発報から 10 秒間だけ表示する）
      if (millis() - last_course_warning_time < 10000 && millis() > 10000) {
        draw_course_warning(steer_angle);
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
          currentTask.saveCsvArgs.kf_altitude,
          currentTask.saveCsvArgs.kf_vspeed,
          currentTask.saveCsvArgs.pressure,
          currentTask.saveCsvArgs.year, currentTask.saveCsvArgs.month,
          currentTask.saveCsvArgs.day, currentTask.saveCsvArgs.hour,
          currentTask.saveCsvArgs.minute, currentTask.saveCsvArgs.second,
          currentTask.saveCsvArgs.centisecond);
        break;
      case TASK_LOG_IMUREPLAY:
        save_imu_replaydata(currentTask.imuReplayArgs.hour,
                            currentTask.imuReplayArgs.minute,
                            currentTask.imuReplayArgs.second,
                            currentTask.imuReplayArgs.centisecond,
                            currentTask.imuReplayArgs.roll,
                            currentTask.imuReplayArgs.pitch,
                            currentTask.imuReplayArgs.yaw,
                            currentTask.imuReplayArgs.pitch_avg,
                            currentTask.imuReplayArgs.pitch_avg_valid,
                            currentTask.imuReplayArgs.roll_trim,
                            currentTask.imuReplayArgs.yaw_acc95,
                            currentTask.imuReplayArgs.wind_mps,
                            currentTask.imuReplayArgs.wind_dir,
                            currentTask.imuReplayArgs.wind_valid,
                            currentTask.imuReplayArgs.filename,
                            currentTask.imuReplayArgs.year,
                            currentTask.imuReplayArgs.month,
                            currentTask.imuReplayArgs.day);
        break;
      case TASK_LOAD_LOGO:
        load_push_logo();  // SD からロゴ BMP を logo_sprite に読み込む（pushSprite は Core0 が行う）
        break;
      case TASK_FLUSH_IMULOG:
        // 生 IMU ログの二重バッファ片側を SD へ書き出す（約 4KB / 0.45 秒分）。
        // 書き出し後にバッファを解放するので、この処理が滞ると Core0 側で記録が捨てられる。
        imulog_write_buffer(currentTask.imuLogArgs.bufidx,
                            currentTask.imuLogArgs.filename,
                            currentTask.imuLogArgs.year,
                            currentTask.imuLogArgs.month,
                            currentTask.imuLogArgs.day,
                            currentTask.imuLogArgs.hour,
                            currentTask.imuLogArgs.minute,
                            currentTask.imuLogArgs.second);
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
  } else if (screen_mode == MODE_IMUDETAIL) {
    // IMU / ESKF 画面: 短押しはカーソル移動のみ。実行はダブルクリック。
    // 較正は「今の姿勢を何度として記録するか」を選んでから行うので、
    // 誤操作で意図しない値が焼き付かないよう 2 段階にしてある。
    if (detail_page % 2 == 0) imu_cursor  = (imu_cursor  + 1) % IMU_MENU_COUNT;
    else                      imu2_cursor = (imu2_cursor + 1) % IMU2_MENU_COUNT;
    redraw_screen = true;
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GPSDETAIL || screen_mode == MODE_VARIODETAIL) {
    detail_page++;
  }
}

// IMU / ESKF 画面のカーソル位置に応じた実行処理（長押しから呼ばれる）。
// 設定画面と同じく「短押し=カーソル移動 / 長押し=実行」の操作感に揃えてある。
// この画面を出るのは Exit 項目からのみ（長押しで戻る動作は持たせていない）。
static void imu_execute() {
  redraw_screen = true;

  if (detail_page % 2 != 0) {     // ページ2 の調整メニュー
    switch (imu2_cursor) {
      case IMU2_MENU_AUTOROLL:
        attitude_set_roll_trim_enabled(!attitude_get_roll_trim_enabled());
        enqueueTask(createSaveSettingTask());
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
        break;
      case IMU2_MENU_WIND:
        attitude_set_wind_enabled(!attitude_get_wind_enabled());
        enqueueTask(createSaveSettingTask());
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
        break;
      case IMU2_MENU_BACK:
        detail_page = 0;
        imu2_cursor = 0;
        enqueueTask(createPlayMultiToneTask(1568, 80, 1));
        break;
    }
    return;
  }

  switch (imu_cursor) {
    case IMU_MENU_SETPITCH:
      // 「いま機体は何度か」の申告値を 0.5 度ずつ巡回させる
      attitude_cycle_pitch_target();
      enqueueTask(createPlayMultiToneTask(1568, 60, 1));
      break;

    case IMU_MENU_SETROLL: {
      // 横風でウィングロー保持のまま較正する場合の申告値。通常は 0。
      const bool was_zero = (attitude_get_roll_target() == 0.0f);
      attitude_cycle_roll_target();
      // 0 から離れた瞬間だけ注意喚起する。0.5 度刻みで巡回するたびに音声を流すと
      // 耳障りなうえキューに溜まるので、状態が変わった 1 回に絞る。
      if (was_zero && attitude_get_roll_target() != 0.0f)
        enqueueTask(createPlayWavTask("wav/change_setroll_caution.wav", 3));
      else
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
      break;
    }

    case IMU_MENU_APPLY:
      // ★ 地上でのみ通す。飛行中に実行すると傾いた姿勢を基準として焼き付けてしまい、
      //   バンク角警告が機能しなくなる。
      if (eskf_calib_allowed()) {
        // すぐには実行しない。スイッチを押す力でデバイスが傾くので、指を離して
        // 落ち着いてから attitude 側が実行する（ESKF_APPLY_DELAY_US）。
        attitude_request_calibrate(attitude_get_pitch_target(), attitude_get_roll_target());
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));  // 受け付けた合図
      } else {
        enqueueTask(createPlayMultiToneTask(440, 200, 1));  // 実行不可（低い音）
      }
      break;

    case IMU_MENU_RPYFUNC:
      attitude_set_rpy_enabled(!attitude_get_rpy_enabled());
      enqueueTask(createSaveSettingTask());
      enqueueTask(createPlayMultiToneTask(1568, 60, 1));
      break;

    case IMU_MENU_NEXTPAGE:
      detail_page = 1;
      enqueueTask(createPlayMultiToneTask(1568, 80, 1));
      break;

    case IMU_MENU_EXIT:
      enqueueTask(createSaveSettingTask());
      screen_mode = MODE_MAP;
      enqueueTask(createPlayMultiToneTask(1568, 80, 1));
      break;
  }
}


// ダブルクリックコールバック: 地図画面でのみスケール（ズームレベル）を切り替える。
// 短押しは毎回発火するため、ここでは短押しと重複しない処理だけを行う。
// 操作音は shortPressCallback() と同じ理由でここで鳴らす（効かない画面では鳴らさない）。
void doublePressCallback() {
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
  redraw_screen = true;

  // リプレイ選択画面での長押し = 「決定」。
  // 下の汎用パス（設定画面以外 → 設定画面へ戻る）より先に処理しないと項目を選べない。
  if (screen_mode == MODE_REPLAYSELECT) {
    handleReplaySelect();
    return;
  }

  // IMU / ESKF 画面での長押し = 「実行」。
  // 設定画面が「短押し=移動 / 長押し=実行」なので、そちらに操作感を合わせる。
  // この画面には Exit 項目があるため、長押しで設定画面へ戻る動作は持たせない。
  if (screen_mode == MODE_IMUDETAIL) {
    imu_execute();
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
void update_degpersecond(int true_track) {
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