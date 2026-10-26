// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
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
#include "airdata.h"
#include "imu.h"
#include "imulog.h"
#include "attitude.h"
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
// IMU/ESKF 画面（ページ1）のカーソル位置。display_tft.cpp の IMU_MENU_* と対応。
int imu_cursor = 0;
int imu2_cursor = 0;   // IMU/ESKF ページ2/3 のカーソル位置（BNO085 と校正）
int imu3_cursor = 0;   // IMU/ESKF ページ3/3 のカーソル位置（ESKF の調整）
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
// 地上の姿勢チェックが「しきい値を超え続けている」か（readiness.cpp が読む）。
// 下の地上チェックのブロックが毎周回落としてから立て直す。
bool ground_roll_warn_active  = false;
bool ground_pitch_warn_active = false;
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

// USERLED フラッシュ制御（Core0 のループから毎回呼ぶ）。v7 の 0.980 から移植。
// 条件ごとの LED 動作:
//   永続点灯 (userled_forced_on): タスクキュー溢れなどの致命エラー → 消灯しない
//   2 回点滅: SD エラー   → 1 秒に 2 回、各 20ms
//   1 回点滅: 衛星 0      → 1 秒に 1 回、20ms
//   正常時:   消灯
// **両方成立したら SD を優先する。** 衛星 0 は屋内なら普通に起きるうえ、直れば
// 何も失われない。SD エラーは「飛行が一切記録されていない」ことを意味し、
// 後から取り返せないので、そちらを見せる。
// （画面ヘッダーには sats バッジと SD バッジの両方が出ている。LED はそれを
//   見ていないとき用の手段なので、画面に出る／出ないは優先順位の理由にならない。）
//
// ★ 時間待ちをしない。delay() を入れると Core0 が止まり、BNO085 のレポートが溜まる。
// ★ **「いま位相が点灯区間か」で判定してはいけない。** Core0 の 1 周は地図描画中に
//   数十 ms になる。20ms の点灯区間をサンプリングで当てに行くと**ほとんど素通りして光らない**。
//   そのため「周期が来たら点ける → 20ms 経ったら消す」の**立ち上がりラッチ方式**にしてある
//   （0.949 も 1 回点滅はこの方式だった。2 発目も同じ方式で打つ）。
//   こうすると 1 周が 20ms を超えても、点灯が長引くだけで発数は必ず出る。
void loop_userled() {
  // 致命エラー時は永続点灯のまま。**ここで毎回 HIGH を打ち直すこと。**
  // 立てるのは Core1 の enqueueTask()（mysd.cpp）で、フラグを立ててから
  // digitalWrite(HIGH) する。Core0 がその直前にフラグを false と読んでいると、
  // 下の消灯がこの HIGH を**後から上書きして LED が消えたまま戻らない**
  // （以降は毎回この return で抜けるので二度と点かない）。打ち直せば次の周回で治る。
  if (userled_forced_on) {
    digitalWrite(USERLED_PIN, HIGH);
    return;
  }

  // 時刻はすべて符号なしの引き算で比べる（millis() の折り返しでも正しい）。
  static unsigned long cycle_start = 0;  // いまの点滅周期の開始時刻
  static unsigned long pulse_start = 0;  // いま出している 1 発の開始時刻
  static uint8_t       pulses_left = 0;  // この周期であと何発打つか
  static bool          led_on      = false;

  const bool sd_ng  = !good_sd();               // Core0 から呼ぶと状態を読むだけ（復旧は Core1 側）
  const bool no_sat = (get_gps_numsat() == 0);

  if (!sd_ng && !no_sat) {
    digitalWrite(USERLED_PIN, LOW);
    led_on = false;  // 消したことを状態にも反映する（次にエラーへ落ちたとき点け直せるように）
    // 打ち残しも捨てる。残したまま同じ周期内でエラーへ戻ると、GAP を過ぎている分
    // **2 発目だけが即座に出て**、衛星 0（1 回点滅）が SD 異常（2 回点滅）に見える。
    pulses_left = 0;
    return;
  }

  const unsigned long now = millis();

  if (now - cycle_start >= USERLED_BLINK_PERIOD_MS) {
    // 新しい周期の先頭。ここで発数を決める（周期の途中で SD が壊れても、
    // パターンが変わるのは次の周期から。1 秒以内なので実用上の遅れは無い）。
    cycle_start = now;
    pulses_left = sd_ng ? 2 : 1;
    pulse_start = now;
    pulses_left--;
    led_on = true;
  } else if (led_on) {
    // 点灯中: 規定時間が過ぎたら消す。1 周が 20ms より長いと点灯もその分伸びるが、
    // 「光った」ことは必ず見える。
    if (now - pulse_start >= USERLED_BLINK_PULSE_MS) led_on = false;
  } else if (pulses_left > 0 && now - cycle_start >= USERLED_BLINK_GAP_MS) {
    // 2 発目（SD エラーのときだけ残っている）。
    pulse_start = now;
    pulses_left--;
    led_on = true;
  }

  digitalWrite(USERLED_PIN, led_on ? HIGH : LOW);
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
  pinMode(BATTERY_PIN, INPUT);
  pinMode(SD_DETECT, INPUT_PULLUP);  // SDカード検出ピン（挿入時 LOW）
  analogReadResolution(12);  // 12bit = 0〜4095
  power_setup();             // USB 判定と電源経路の異常の見張り（analogReadResolution の後）

  //setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP);  // This must be after setup tft for some reason of library TFT_eSPI.
  setup_tft();
  // ★ **ここでタイトルを出しておく**（v7 の 0.975）。これが無いと起動画面が始まるまで画面が真っ白になる。
  //   白のままの区間は imu_setup()（β 機は BNO085 のリセット〜ブートで約 0.4 秒）と
  //   gps_setup() で、**GPS 未接続だと後者が数秒**かかる
  //   （CFG の ACK 待ちが空振りする。log.txt の "GPS CFG NO ACK"）。
  //   GPS を繋げば ACK が即返るので 1 秒以下に縮む。**固まっているわけではない。**
  draw_boot_title();
  // BNO085 の NRST を早期に HIGH に固定する（内蔵プルアップ）。
  // RP2350 GPIO は電源投入直後フローティングになるため、imu_setup() が呼ばれるまでの間に
  // NRST が偶然 LOW になって BNO085 がリセット状態に入るのを防ぐ。
#if defined(IMU_RST_PIN) && IMU_RST_PIN >= 0
  pinMode(IMU_RST_PIN, INPUT_PULLUP);
#endif

  // ★★ **setup() の間は生 IMU ログを止める**（v7 の 0.975）。
  //   バッファの排出 imulog_take_pending() は **loop() からしか呼ばれない**ので、
  //   setup() 中に貯めるとバッファ 2 枚（IMULOG_RECS_PER_BUF × 2）で満杯になり、
  //   **それ以降は全部捨てられる**。捨てても seq は進むため、PC 側には
  //   「取りこぼし」として残り、decode_imulog.py のセッション分割を誤らせる。
  //   起動画面の間の生データは机上の値で使い道が無いので、記録しない。
  //   ★ 解除は明示的に書かない。loop() 冒頭の
  //     imulog_set_paused(getReplayMode()) が毎周回セットし直すので、
  //     loop() の 1 回目で自動的に解ける（リプレイ中なら正しく止まったまま）。
  imulog_set_paused(true);

  // I2C バス初期化（MS5611・BNO085 共用。両 setup より先に呼ぶ）
  airdata_wire_begin();
  // BNO085 を先に初期化する（リセット後のブート時間を確保するため）
  imu_setup();
  attitude_setup();   // 姿勢 ESKF（センサー入力は imu.cpp/gps.cpp から流し込む）
  // ★ ここで一度描く。BNO085 の結果は分かった時点ですぐ出す（v7 の 0.979）
  //   （startup_demo_tft() まで待つと「INITIALIZING SENSORS ...」のまま固まって見える）。
  BootCheckState imu_check = get_imu_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG;
  draw_boot_progress(imu_check, BOOT_CHECK_PENDING, BOOT_CHECK_PENDING);

  // MS5611 初期化（BNO085 の後に呼ぶ）
  airdata_setup();
  BootCheckState airdata_check = get_airdata_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG;
  draw_boot_progress(imu_check, airdata_check, BOOT_CHECK_PENDING);

  // ★★ **log.txt の最初の行はビルドの身元にする**（v7 の 0.991）。
  //   これが無いと、後からログを読んだときに**どのファームが出したものか分からない**。
  //   バージョンだけでは足りない:
  //     ・どの基板向けのビルドか（v6 と v7 は同じ番号帯で別のファーム）
  //     ・RELEASE でないビルドは GPS シミュレーションやデバッグ出力が入りうる
  //   ★ 基板名は VERSION_TEXT を使う（settings.h。ここで書き直すと二重管理になる）。
  //   ★ BNO085 の有無も残す（v6 は β 機だけが載せている。見つからなかった機体は "none"）。
  //   ★ 置き場所はここ。setup() の冒頭に出すと、Core1 の mutex_init より先に
  //     enqueueTask を呼ぶ窓に入る（setup1() の冒頭コメント参照）。
  //     imu_setup() / airdata_setup() のログ（"BNO085 init OK" など）が数行先に並ぶが、
  //     どの起動でもその直後に出るので、電源投入の区切りとして探せる。
#ifdef RELEASE
  const char* build_rel = "";
#else
  const char* build_rel = " [NOT RELEASE]";
#endif
  enqueueTask(createLogSdfTask("BOOT PONS %s build %lu board=%s imu=%s%s",
                               BUILDVERSION, (unsigned long)BUILDDATE, VERSION_TEXT,
                               get_imu_present() ? (get_imu_ok() ? "BNO085" : "BNO085(NG)") : "none",
                               build_rel));

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
  draw_boot_progress(imu_check, airdata_check, get_gps_cfg_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG);

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


// 警告音を抑制するか。**充電中 かつ マウントから外れた姿勢**のときだけ true（v7 の 0.979）。
//
// ★ なぜこの 2 条件か:
//   ・机に置いて充電すると、ロール +78 度・ピッチ -80 度のような姿勢になり、
//     バンク警告・地上ロール/ピッチ警告・較正の催促が延々と鳴る。
//     人力飛行機が地上でその姿勢になることはないので、**姿勢だけでも
//     「機体に載っていない」と判定できる**（settings.h の OFF_MOUNT_DEG）。
//   ・**USB を AND 条件にしてあるのは、机の上で意図的に回して音を確認したいため。**
//     充電せずに置けば従来どおり鳴るので、試験の手段が残る。
//   ・機体に載せたまま充電しても、水平ならマウント外れ判定に入らないので
//     **飛行前点検としての警告は保たれる**。
// ★ **抑制するのは音だけ。** 表示もログも残す。ログには (MUTED) を付けて、
//   「条件は成立していたが鳴らさなかった」ことが後から分かるようにする。
//   画面には draw_warn_mute_banner()（display_tft.cpp）が「なぜ静かなのか」を出す。
// ★ **解除操作を持たせない。** 機体に載せて水平に戻した瞬間に自動で復活し、
//   それはまさにリマインダーが必要なタイミングでもある。
//   覚えて解除しなければならないミュートは、当日に忘れると全ての警報が無音になる。
bool warn_muted_off_mount() {
  if (getReplayMode() || replay_voltage_active()) return false;   // 電池表示と同じ流儀
  // ★ off_mount の判定は attitude_on_gyro() の中でしか更新されないので、
  //   BNO085 が死ぬと**前回の値のまま固まる**。その状態で「抑制中」と名乗ると
  //   画面が嘘になる。抑制対象はいずれも attitude_ready() を条件に持つので、ここで揃えても漏れない。
  if (!attitude_ready()) return false;
  // USB の判定は usb_power_present()（壊れた充電 IC の「USB に見える」を除いたもの。power.cpp）
  return usb_power_present() && attitude_is_off_mount();
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
  // GPS モジュールの断線／復帰（音とログ。表示は display_tft.cpp）
  gnss_conn_tick();
  // 電源経路の異常・高温での充電（power.cpp。内部で 250ms に間引く）。
  // ★ 画面モードによらずここで回す。判断と警報は描画関数の中に置かない。
  power_tick();
  // 本番前チェック（readiness.cpp。内部で 1 秒に 1 回。表示は設定画面の右上）
  readiness_tick();

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
        snprintf(imu_fname, sizeof(imu_fname), IMULOG_DIR "/%04d%02d%02d.bin",
                 jst_year, jst_month, jst_day);
        queued = enqueueTask(createFlushImuLogTask(imulog_buf, imu_fname,
                                                  jst_year, jst_month, jst_day,
                                                  log_h, log_m, log_s));
      } else {
        // GPS 未測位（屋内テスト等）。日付が決まらないので合言葉を渡す。
        // Core0 から SD は触れないので、実ファイル名（imuraw/nofixNNN.bin）の決定は
        // Core1 側の imulog_write_buffer() がやる。番号を振るのは、固定名 1 つだと
        // 複数日・複数起動が 1 ファイルに混ざって切り分けられなくなるため。
        // レコードはホスト時刻 t_us と BOOT レコードを持つので、これでも解析はできる。
        // ファイルのタイムスタンプは SdFat の下限（1980 年以降）を満たす固定値にする。
        // 書式文字列としてではなく %s で渡す（パスに % が混ざっても壊れないように）
        snprintf(imu_fname, sizeof(imu_fname), "%s", IMULOG_NOFIX_REQUEST);
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
        const bool muted = warn_muted_off_mount();
        if (!muted) enqueueTask(createPlayWavTask("wav/bank_warning.wav", 3));
        enqueueTask(createLogSdfTask("BANK WARN roll=%+.1f deg%s", wr,
                                     muted ? " (MUTED: charging + off mount)" : ""));
      }
    } else {
      bank_over_since_ms = 0;   // 一度でも下回ったら継続時間をリセット
    }
  }

  // ---- 電池電圧の低下警告（batt_alarm_tick）----
  batt_alarm_tick();

  // ---- 直進中のロール自動トリムが入ったらログに残す ----
  // 事後検証できるよう、1 回の補正量と累積量を記録する。
  // ---- 地上待機中のバンクチェック ----
  // 自動ロールトリムは対地速度 3m/s 超でしか働かないので地上では動かない。
  // しかしマウントのズレを直せるのは地上だけなので、ここで別建てに見張る。
  // 静止していることを条件に入れて、運搬中・組み立て中の傾きを除外する。
  // ★ 本番前チェックへ出す発報状態は、ここで毎周回落としてから下で立て直す
  //   （ブロックの条件から外れたら＝動き出した・姿勢機能 OFF などでは立たない）。
  ground_roll_warn_active = ground_pitch_warn_active = false;
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
      // 本番前チェックへ出す発報状態。運搬中のミュート（充電中かつマウント外れ）も効かせる（v7 と同じ）。
      ground_roll_warn_active = (now_ms - gnd_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
                                !warn_muted_off_mount();
      if ((now_ms - gnd_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
          (gnd_last_warn_ms == 0 ||
           (now_ms - gnd_last_warn_ms) >= GROUND_ROLL_WARN_INTERVAL_MS)) {
        gnd_last_warn_ms = now_ms;
        const bool muted = warn_muted_off_mount();
        if (!muted) enqueueTask(createPlayWavTask("wav/roll_check.wav", 3));
        enqueueTask(createLogSdfTask("GROUND ROLL %+.1f deg (check mount / APPLY)%s", gr,
                                     muted ? " (MUTED: charging + off mount)" : ""));
      }
    } else {
      gnd_over_since_ms = 0;   // 一度でも下回ったら継続時間をリセット
    }

    // ピッチ: 申告値 SET PITCH からのズレで見る。
    // 地上の機体ピッチは場所によって変わる（プラットホーム -3.5 度、平地 0 度）ので、
    // ロールのように絶対値 0 を基準にはできない。
    //
    // ただしプラットホーム近傍だけは例外。台の上と台の手前の地面は装置から
    // 区別がつかないため、基準を一点ではなく PITCH_TARGET_MIN_DEG 〜 0 度の
    // 区間とみなす（平地で APPLY してから台へ上げても鳴らないようにする）。
    // 測位できていないときは近傍とみなさず、従来どおりの厳しい判定を使う。
    //
    // ※ 距離計算（haversine, double）は loop() 毎に回すには重いので 1 秒に 1 回だけ
    //   更新して結果を使い回す。60 秒継続してから発報する判定なので粒度は足りる。
    static uint32_t gndp_geo_ms   = 0;
    static bool     gndp_near_pla = false;
    if (gndp_geo_ms == 0 || (now_ms - gndp_geo_ms) >= PLATFORM_NEAR_RECHECK_MS) {
      gndp_geo_ms   = now_ms;
      gndp_near_pla = get_gps_fix() &&
          calculateDistanceKm(get_gps_lat(), get_gps_lon(), pla_lat, pla_lon) <= PLATFORM_NEAR_KM;
    }
    const bool near_platform = gndp_near_pla;
    bool pitch_bad = near_platform
        ? (gp > GROUND_PITCH_WARN_DEG ||
           gp < PITCH_TARGET_MIN_DEG - GROUND_PITCH_WARN_DEG)
        : (fabsf(gp - attitude_get_pitch_target()) > GROUND_PITCH_WARN_DEG);
    if (pitch_bad) {
      if (gndp_over_since_ms == 0) gndp_over_since_ms = now_ms;
      ground_pitch_warn_active = (now_ms - gndp_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
                                 !warn_muted_off_mount();
      if ((now_ms - gndp_over_since_ms) >= GROUND_ROLL_WARN_HOLD_MS &&
          (gndp_last_warn_ms == 0 ||
           (now_ms - gndp_last_warn_ms) >= GROUND_ROLL_WARN_INTERVAL_MS)) {
        gndp_last_warn_ms = now_ms;
        const bool muted = warn_muted_off_mount();
        if (!muted) enqueueTask(createPlayWavTask("wav/pitch_check.wav", 3));
        enqueueTask(createLogSdfTask("GROUND PITCH %+.1f deg (target %+.1f, nearPLA %d)%s",
                                     gp, attitude_get_pitch_target(), (int)near_platform,
                                     muted ? " (MUTED: charging + off mount)" : ""));
      }
    } else {
      gndp_over_since_ms = 0;
    }

    // ---- 較正のやり直しが必要（マウントから外された形跡がある）----
    // 画面には赤字で出しているが、**地上では画面を見ていないことが多い**。
    // 気づかないまま飛ぶと、姿勢表示・バンク角警告・自動ロールトリムが
    // まとめて信用できない状態のまま飛ぶことになるので、音でも促す。
    //
    // ★ このブロックの条件（地上・静止・リプレイ/ミラー以外）をそのまま使う。
    //   **飛行中は絶対に鳴らさない。**飛行中に較正はできないし、
    //   対処のしようがない警告で注意をそらすほうが危険だから。
    //   SD が無い機体でも気づけるよう、WAV が鳴らせないときは低いトーンで代替する。
    //   最低保証音量は付けない。地上で静止しているとき＝人が機体のそばにいるときにしか
    //   鳴らないので、音量を絞る判断を上書きする理由が無い（60 は電池切れ専用）。
    {
      static uint32_t eskf_apply_last_warn_ms = 0;
      if (attitude_needs_apply()) {
        if (eskf_apply_last_warn_ms == 0 ||
            (now_ms - eskf_apply_last_warn_ms) >= ESKF_APPLY_WARN_INTERVAL_MS) {
          eskf_apply_last_warn_ms = now_ms;
          const bool muted = warn_muted_off_mount();
          if (!muted) {
            if (good_sd()) enqueueTask(createPlayWavTask("wav/eskf_calib_required.wav", 3));
            else           enqueueTask(createPlayMultiToneTask(262, 250, 3, 3));
          }
          enqueueTask(createLogSdfTask("ESKF CALIBRATION REQUIRED (on ground)%s",
                                       muted ? " (MUTED: charging + off mount)" : ""));
        }
      } else {
        eskf_apply_last_warn_ms = 0;   // 較正し直したら次回は即座に知らせる
      }
    }

    // ---- 較正した日から日付をまたいでいる ----
    // OFF MOUNT 判定（attitude.cpp）は ESKF が回っている間しか働かないので、
    // **電源を切ったまま外して付け直した**ケースだけは検出できない。
    // 地上ロール/ピッチチェックが拾えるのは 3 度を超えるズレまでで、
    // それ未満のズレは再 APPLY 以外に直す手段が無い。そこを埋めるのがこれ。
    //
    // ★ これは推測であって断定ではない。本当にマウントに付けっぱなしのまま
    //   日をまたぐこともある。だから needs_apply は立てない。
    //   立てると、ヘッダのピッチ数字に「読むな」の赤い取り消し線が入る。
    //   根拠のない推測でそこまでやらない。
    //   画面は設定画面（IMU/ESKF 1/2）の注意書きだけ、音はここで一度だけ。
    //
    // ★ 繰り返さないこと。2 分おきに鳴らすと、プラットホームの上で鳴ったときに
    //   「止めるために APPLY する」人が出る。台の上で APPLY すると、正しかった
    //   較正が台の姿勢で上書きされてしまう（APPLY は静止していれば通る）。
    //   一度伝えたら、あとは人の判断に任せる。
    {
      static bool calib_day_warned = false;
      const uint32_t today_jst = get_gps_jst_yyyymmdd();
      if (!calib_day_warned && !attitude_needs_apply() &&
          attitude_calib_date_stale(today_jst)) {
        calib_day_warned = true;
        // WAV が無い機体・SD が無い機体でも気づけるよう、先に 2 音鳴らしてから
        // WAV を積む（APPLY 成功音と同じ作り）。eskf_calib_required.wav は
        // 流用しない。「較正が必要」と断定する文言になってしまうため。
        enqueueTask(createPlayMultiToneTask(1319, 90, 2));
        if (good_sd()) enqueueTask(createPlayWavTask("wav/eskf_calib_oldday.wav", 3));
        enqueueTask(createLogSdfTask("ESKF CALIB from another day (calib %lu, today %lu)",
                                     (unsigned long)attitude_get_calib_date(),
                                     (unsigned long)today_jst));
      }
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
    // 較正した日を刻む。測位していなければ 0（不明）が入り、日付判定は働かない。
    // ★ createSaveSettingTask() より先に呼ぶ。保存は Core1 が後から実行するので
    //   どちらの順でも実害は無いが、「保存する値を全部そろえてから保存を頼む」
    //   という読み方ができるようにしておく。
    attitude_set_calib_date(get_gps_jst_yyyymmdd());
    enqueueTask(createSaveSettingTask());                  // 次回起動でも効くよう保存
    // 成功トーン（上がり 2 音）。SD に eskf_apply_done.wav が無くても、
    // 較正が実行されたことが分かるようにする。WAV の直前に鳴らす。
    enqueueTask(createPlayMultiToneTask(1568, 70, 1));
    enqueueTask(createPlayMultiToneTask(2093, 90, 1));
    // 較正できたことを音声で知らせる。優先度 3 = 案内音声と同等。
    // 較正の成否は取り違えると危険なので埋もれさせない。
    enqueueTask(createPlayWavTask("wav/eskf_apply_done.wav", 3));
    // 適用したオフセットと、そのときの GNSS 日時を残す。
    // log.txt の行頭は「起動からの秒数」でしかないため、これが無いと
    // 「いつ較正したか」は同じファイル中の GNSS TIME 行と突き合わせないと
    // 分からない（測位できなかった起動ではその行自体が出ない）。
    // 日時は UTC。GNSS TIME 行と揃えてある。未測位なら ---- を出す。
    {
      float lvr, lvp;
      attitude_get_level_offset(lvr, lvp);
      GpsDate d = get_gpsdate();
      GpsTime t = get_gpstime();
      if (d.isValid() && t.isValid())
        enqueueTask(createLogSdfTask("ESKF APPLY done R%+.2f P%+.2f at %04d-%02d-%02d %02d:%02d:%02d UTC",
                                     lvr, lvp, d.year(), d.month(), d.day(),
                                     t.hour(), t.minute(), t.second()));
      else
        enqueueTask(createLogSdfTask("ESKF APPLY done R%+.2f P%+.2f at ---- (no GNSS time)",
                                     lvr, lvp));
    }
  }

  {
    float applied, total;
    if (attitude_take_roll_trim_event(applied, total)) {
      enqueueTask(createLogSdfTask("ROLL TRIM %+.2f deg (total %+.2f)", applied, total));
      // 次回起動でも効くよう保存する。設定画面を開かない運用（パイロットに預けたまま）
      // でも残す必要があるため、ここで保存する。
      // 補正は最短でも ROLL_TRIM_WINDOW_S × ROLL_TRIM_CONFIRM_WINDOWS 間隔でしか
      // 起きないので、SD 書き込みの頻度は問題にならない。
      enqueueTask(createSaveSettingTask());
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
      // ★ 時刻は **get_jst_now() の判定を通ったときだけ**書く。
      //   以前は get_gpstime() の生の値を無条件に使っていたため、測位できていない間は
      //   _hour/_min が 0 のままで「09:00 JST」と記録されていた。ログ上は
      //   正しい時刻に見えるので、あとから見たときに嘘に気づけない。
      //   （ループ先頭で毎回求めている jst_valid / log_h / log_m をそのまま使う。
      //     こちらは millis() の経過ぶんも足してあり、日跨ぎも処理済み。）
      char jst_str[8];
      if (jst_valid) snprintf(jst_str, sizeof(jst_str), "%02d:%02d", log_h, log_m);
      else           strlcpy(jst_str, "--:--", sizeof(jst_str));
      enqueueTask(createLogSdfTask("volt=%.2fV cpu=%.1fC %s JST",
                                   get_input_voltage(), analogReadTemp(), jst_str));

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
      // ★ BNO085 の申告精度と動的校正の設定（v7 の 0.975）。加速度が 3 未満の間は
      //   静止時 |a| が数 % ずれる（v7 の実測 -4.6%）ので、バリオの鉛直加速度も同じだけずれる。
      enqueueTask(createLogSdfTask("cal cfg=0x%02X acc=%u gyr=%u mag=%u",
                                   get_imu_cal_cfg(), get_imu_acc_accuracy(),
                                   get_imu_gyr_accuracy(), get_imu_mag_accuracy()));
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
          currentTask.saveCsvArgs.kf_altitude,
          currentTask.saveCsvArgs.kf_vspeed,
          currentTask.saveCsvArgs.pressure,
          currentTask.saveCsvArgs.voltage,
          currentTask.saveCsvArgs.numsat,
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
  } else if (screen_mode == MODE_IMUDETAIL) {
    // IMU / ESKF 画面: 短押しはカーソル移動のみ。実行はダブルクリック。
    // 較正は「今の姿勢を何度として記録するか」を選んでから行うので、
    // 誤操作で意図しない値が焼き付かないよう 2 段階にしてある。
    if      (detail_page % 3 == 0) imu_cursor  = (imu_cursor  + 1) % IMU_MENU_COUNT;
    else if (detail_page % 3 == 1) imu2_cursor = (imu2_cursor + 1) % IMU2_MENU_COUNT;
    else                           imu3_cursor = (imu3_cursor + 1) % IMU3_MENU_COUNT;
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

  if (detail_page % 3 == 1) {     // ページ2/3: BNO085 と校正
    switch (imu2_cursor) {
      case IMU2_MENU_SAVECAL: {
        // ★ 条件を満たさないなら**何もしない**。途中の悪い校正を不揮発へ焼くと
        //   電源を切っても戻らないので、ここは低い音で断る。
        //   BNO085 を載せていない機体も imu_cal_ready() が false なのでここで断る。
        if (!imu_cal_ready()) {
          enqueueTask(createPlayMultiToneTask(440, 200, 1));
          break;
        }
        // ★ RTC が無いので日付は GNSS 由来。屋内なら取れないので 0（日付不明）で保存する。
        //   日付が無くても「保存した」ことは残る。それが画面の NEVER を消す条件。
        //   リプレイ中・デモ中の GPS の日付は今日ではないので、そのときも 0 にする。
        const uint32_t today = (getReplayMode() || is_demo_active()) ? 0 : get_gps_jst_yyyymmdd();
        if (imu_save_calibration(today)) {
          // 較正完了と同じ流儀の上昇 2 音
          enqueueTask(createPlayMultiToneTask(1568, 60, 1, 2));
          enqueueTask(createPlayMultiToneTask(2093, 90, 1, 2));
        } else {
          enqueueTask(createPlayMultiToneTask(440, 300, 2));
        }
        break;
      }
      case IMU2_MENU_NEXTPAGE:
        detail_page = 2;
        imu3_cursor = 0;
        enqueueTask(createPlayMultiToneTask(1568, 80, 1));
        break;
      case IMU2_MENU_BACK:
        detail_page = 0;
        imu2_cursor = 0;
        enqueueTask(createPlayMultiToneTask(1568, 80, 1));
        break;
    }
    return;
  }

  if (detail_page % 3 == 2) {     // ページ3/3: ESKF の調整メニュー
    switch (imu3_cursor) {
      case IMU3_MENU_AUTOROLL:
        attitude_set_roll_trim_enabled(!attitude_get_roll_trim_enabled());
        enqueueTask(createSaveSettingTask());
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
        break;
      case IMU3_MENU_WIND:
        attitude_set_wind_enabled(!attitude_get_wind_enabled());
        enqueueTask(createSaveSettingTask());
        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
        break;
      case IMU3_MENU_BACK:
        detail_page = 0;
        imu3_cursor = 0;
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
    //   truetrack_samples[] は元から float、GNSS 側も NAV-PVT の headMotion
    //   （分解能 1e-5 度）なので、捨てていたのはここだけ。
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
  if (good_sd()) {
    // SD 認識済み: WAV を再生（最低 volume 60 保証）
    enqueueTask(createPlayWavTask("wav/battery_low.wav", 1, 60));
  } else {
    // SD 未認識: 高音ビープ 3 回で代替警告（最低 volume 60 保証）
    enqueueTask(createPlayMultiToneTask(2637, 150, 1, 1, 60));
    enqueueTask(createPlayMultiToneTask(2637, 150, 1, 1, 60));
    enqueueTask(createPlayMultiToneTask(2637, 400, 1, 1, 60));
  }
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