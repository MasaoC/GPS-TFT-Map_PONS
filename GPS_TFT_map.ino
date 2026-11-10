// ============================================================
// File    : GPS_TFT_map.ino
// Project : PONS v7.1 (Pilot Oriented Navigation System for HPA)
// Role    : メインエントリポイント。
//           Core0: 画面描画・GNSS処理・ボタン入力・コース警告
//           Core1: SDカード操作・音声再生（タスクキュー経由）
//           地図背景はフラッシュ内蔵のベクタ地図（vectormap.cpp）を使う。
//           SDカード上のBMPタイル方式は廃止済み。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================
//
// ■ src/ に置いてあるもの
//   Arduino IDE はスケッチ直下のファイルを全部タブに出すので、
//   ファイルが増えるとタブが埋まって目的のものを探しづらくなる。
//   **仕様が固まっていて普段いじらないモジュール**は src/ へ移した。
//   src/ 配下もコンパイルはされるが、IDE のタブには出ない。
//     src/button.*     ボタン入力・メニュー
//     src/sound.*      音声出力（WAV・トーン）
//     src/vectormap.*  ベクタ地図の描画
//     src/imulog.*     生 IMU ロガー（姿勢 ESKF のオフライン開発用）
//     src/flashdata/   FLASH に置く大きな定数データ
//
//   ★ src/ の中からルート側のヘッダを参照するときは "../mysd.h" のように
//     1 段戻ること。スケッチ直下は include パスに入っていない。
// ============================================================

#include "airdata.h"
#include "attitude.h"
#include "tas.h"         // 推定 TAS: 地上と飛行中の判定（tas_tick）・大きい枠の設定
#include "backlight.h" // v7.1 のバックライト（輝度スイッチ・照度センサー）
#include "bootwatch.h" // 起動の見張り（起動中に固まったら再起動し、止まった段階を log.txt に残す）
#include "display_tft.h"
#include "gnss.h"
#include "gnss_dbd.h"  // GNSS の航法データベース（USB シリアルの PONS:DBD,... と、デバッグ中継の前の書き切り）
#include "hardware/adc.h"
#include "imu.h"
#include "imu_sensor.h" // imu_sensor_name()
#include "link.h"       // PONS Link（機体⇄ボート無線）
#include "mysd.h"
#include "navdata.h"
#include "power.h"     // USB 判定・電源経路の異常
#include "readiness.h" // 本番前チェック（そろったら「準備完了」）
#include "settings.h"
#include "src/button.h"
#include "src/imulog.h"
#include "src/sound.h"
#include "src/vectormap.h"
#include "src/flashdata/imu_guide_data.h" // IMU_GUIDE_PAGES（IMU/ESKF 画面の入口の案内）
#include "src/flashdata/tas_guide_data.h" // TAS_GUIDE_PAGES（推定 TAS の案内の画像）

// we need to do bool core1_separate_stack = true; to avoid stack running out.
// (Likely due to drawWideLine from TFT-eSPI consuming alot of stack.)
bool core1_separate_stack = true; // DO NOT REMOVE THIS LINE.

// --- 画面更新タイミング管理 ---
unsigned long screen_update_time = 0; // 最後に画面更新した時間 millis()
bool redraw_screen = false; // true にすると次のループで画面を再描画する

// --- 旋回角速度 (deg/s) 計算用 ---
// GNSS の真方位 (truetrack) を NUM_SAMPLES
// 個のスライディングウィンドウで保持し、
// 連続サンプル間の変化量の合計を「最古サンプルと最新サンプルの経過時間」で割ることで
// GNSS レート（1Hz / 2Hz）に依存しない真の deg/s を算出する。
const int NUM_SAMPLES = 4;            // スライディングウィンドウのサンプル数
float truetrack_samples[NUM_SAMPLES]; // 過去の真方位サンプル配列
uint32_t
    truetrack_sample_times[NUM_SAMPLES]; // 各サンプルの millis() タイムスタンプ
int sampleIndex = 0;                     // 次に書き込むサンプルのインデックス
float degpersecond = 0;                  // 算出された旋回角速度 [deg/s]

// --- 画面モード・スケール管理 ---
int screen_mode = MODE_MAP; // 現在の画面モード（MODE_MAP / MODE_SETTING など）
int detail_page = 0;        // サブ画面（GNSSDetail / SDDetail）のページ番号
int replay_cursor = 0;      // リプレイ選択画面のカーソル位置（項目の通し番号）
int replay_list_page = 0;   // リプレイ選択画面で現在表示・読み込み済みのページ
// IMU/ESKF 画面（ページ1）のカーソル位置。display_tft.cpp の IMU_MENU_*
// と対応。
int imu_cursor = 0;
int imu2_cursor = 0; // IMU/ESKF ページ2/2 のカーソル位置（IMU の生値と ESKF の状態。行は戻る / Exit）
// IMU/ESKF 画面の入口の案内（画像 IMU_GUIDE_PAGES 枚）の何枚目を出しているか。-1 = 出していない。
// 設定画面から入るたびに 0 から通る（imu_guide_start）。押すたびに次の 1 枚へ進み、
// 最後の次でページ1（較正の画面）へ移る（imu_guide_next）。
int imu_guide_page = -1;
// 推定 TAS の案内（docs/pons_tas.md §4.4）の何画面目を出しているか。-1 = 出していない。
// IMU/ESKF 画面 1 ページ目で「Header」を TAS にしたときだけ 0 から通る（tas_guide_start）。
// 0 = 説明の画像、1 = 対気速度モデルの曲線（ファームが描く）。最後の次でページ1へ戻る。
int tas_guide_page = -1;
// バンク角警告の有効/無効（IMU/ESKF 画面で切替、SD に保存）
double scalelist[6]; // 選択可能なスケール値リスト（ズームレベルに対応）
double scale;        // 現在のマップスケール [pixels/km]

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
// バンク角警告でミラー値を受けるための一時変数（判定式の中で使う）
static float bank_roll_tmp = 0.0f, bank_pitch_tmp = 0.0f;
unsigned long last_course_warning_time = 0; // 直近の警告発報時刻 [millis]
unsigned long last_destination_toofar_time =
    0; // 直近の「目的地が遠すぎる」警告時刻 [millis]
double steer_angle =
    0.0; // 現在針路と目的地方位の差 (-180〜+180 度。正=右、負=左)

// --- 大気データ シリアル出力タイミング管理 ---
unsigned long last_airdata_print_time =
    0; // 最後に気温・気圧を Serial 出力した時間 [millis]
bool airdata_updated =
    false; // airdata_update() が true を返した直後フラグ（VSI 更新トリガー）

// --- ユーザーLED 制御 ---
// userled_forced_on:
// タスクキュー溢れなど重篤エラー時に永続点灯させるフラグ（一度 true
// になったらリセットまで戻らない）
volatile bool userled_forced_on = false;

// ---- 処理時間計測変数（IMU
// の読み出しを飢えさせない調査用、デバッグビルドのみ） ----
#ifndef RELEASE
TimingStat ts_redraw = TSTAT_INIT("C0_redraw");
volatile bool c0_is_redrawing = false; // Core1側の重複検出用フラグ
#endif

// --- Core1 スタック残量計測用ベース SP ---
// setup1() の先頭で現在の SP をキャプチャし、以降の使用量計測の基準にする。
// core1_separate_stack=true のため Core1
// のスタックはヒープから動的確保されており、
// リンカシンボルでは下限がわからないため、この方法で代替する。
volatile uint32_t _core1_base_sp = 0;

volatile int scaleindex =
    3; // scalelist のインデックス（初期値 3 = SCALE_LARGE_GMAP）
volatile int sound_volume = 50; // 音量 0〜100
volatile int vario_volume =
    10; // バリオメーター音量 0〜100（設定画面から変更可）
volatile bool vario_inhibit =
    false; // true のとき vario は完全無効（SDカード settings.txt でのみ設定可）

extern volatile bool loading_sddetail;
extern bool sd_detail_loading_displayed;

void reset_degpersecond();
void update_degpersecond(float true_track);
void check_destination_toofar();
static void apply_auto10k_status(int st);
void update_course_warning(float degpersecond);
static void nav_alarm_tick(bool new_gnss_info);
// 最後にボタンが操作された時刻。設定画面の放置タイムアウト（下の
// SETTING_IDLE_TIMEOUT_MS）用。 3
// つのコールバックの先頭で更新するので、押し方の種類によらず必ず記録される。
static unsigned long last_user_input_ms = 0;
// 地上の姿勢チェックが「いま発報している」状態。PONS Link の status に載せて、
// **受信機でも同じ音声を鳴らす**ために公開する（link.cpp が読む）。
// ★ 立てるのは下の地上ブロックだけ。**毎周回クリアしてから立て直す**ので、
//   離陸して条件から外れた瞬間に落ちる。落とさないと status
//   が立ったままになり、
//   受信機が飛行中ずっと鳴らし続ける（送信機は飛行中は黙る）。
bool ground_roll_warn_active = false;
bool ground_pitch_warn_active = false;
bool ground_calib_warn_active = false;
// 上のロール・ピッチの判定に使った許容範囲
// [度]。**画面に「何度なら良いか」を出すため**に公開する （display_tft.cpp
// が読む）。音声が鳴っても、どちらの値が何度ずれているのか画面で分からなかった。
// 範囲を画面側で計算し直さないこと（プラットホーム近傍の緩和など、判定と食い違う）。
float ground_roll_ok_lo = 0.0f, ground_roll_ok_hi = 0.0f;
float ground_pitch_ok_lo = 0.0f, ground_pitch_ok_hi = 0.0f;
void shortPressCallback();
void longPressCallback();
void doublePressCallback();
static void imu_execute();
static void imu_guide_next();
static void tas_guide_start();
static void tas_guide_next();
void next_scaleindex();
// 設定画面のラベル用に、現在のスケールで画面横幅(240px)が何 km
// に相当するかを返す。 以前は Google
// タイルのズーム番号(zoom5〜13)を表示していたが、ベクタ地図に移行して
// タイルとの対応が無くなったため、実際の距離で表すようにした。
float scale_screen_km(int index);
// Create Button objects
Button sw_push(SW_PUSH, shortPressCallback, longPressCallback,
               doublePressCallback);

// USERLED フラッシュ制御（Core0 のループから毎回呼ぶ）。
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
// ★ 時間待ちをしない。delay() を入れると Core0 が止まり、IMU のサンプルを
//   取りこぼす（CLAUDE.md「IMU の読み出しを飢えさせない」）。
// ★ **「いま位相が点灯区間か」で判定してはいけない。** Core0 の 1
// 周は地図描画中に
//   数十 ms になる（60 秒ログの gapmax= が実測値。40ms 前後）。20ms
//   の点灯区間を サンプリングで当てに行くと**ほとんど素通りして光らない**。
//   そのため「周期が来たら点ける → 20ms
//   経ったら消す」の**立ち上がりラッチ方式**にしてある。 こうすると 1 周が 20ms
//   を超えても、点灯が長引くだけで発数は必ず出る。
void loop_userled() {
  // 致命エラー時は永続点灯のまま。**ここで毎回 HIGH を打ち直すこと。**
  // 立てるのは Core1 の enqueueTask()（mysd.cpp）で、フラグを立ててから
  // digitalWrite(HIGH) する。Core0 がその直前にフラグを false と読んでいると、
  // 下の消灯がこの HIGH を**後から上書きして LED が消えたまま戻らない**
  // （以降は毎回この return
  // で抜けるので二度と点かない）。打ち直せば次の周回で治る。
  if (userled_forced_on) {
    digitalWrite(USERLED_PIN, HIGH);
    return;
  }

  // 時刻はすべて符号なしの引き算で比べる（millis() の折り返しでも正しい）。
  static unsigned long cycle_start = 0; // いまの点滅周期の開始時刻
  static unsigned long pulse_start = 0; // いま出している 1 発の開始時刻
  static uint8_t pulses_left = 0;       // この周期であと何発打つか
  static bool led_on = false;

  const bool sd_ng =
      !good_sd(); // Core0 から呼ぶと状態を読むだけ（復旧は Core1 側）
  const bool no_sat = (get_gnss_numsat() == 0);

  if (!sd_ng && !no_sat) {
    digitalWrite(USERLED_PIN, LOW);
    led_on =
        false; // 消したことを状態にも反映する（次にエラーへ落ちたとき点け直せるように）
    // 打ち残しも捨てる。残したまま同じ周期内でエラーへ戻ると、GAP
    // を過ぎている分
    // **2 発目だけが即座に出て**、衛星 0（1 回点滅）が SD 異常（2
    // 回点滅）に見える。
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
    // 点灯中: 規定時間が過ぎたら消す。1 周が 20ms
    // より長いと点灯もその分伸びるが、 「光った」ことは必ず見える。
    if (now - pulse_start >= USERLED_BLINK_PULSE_MS)
      led_on = false;
  } else if (pulses_left > 0 && now - cycle_start >= USERLED_BLINK_GAP_MS) {
    // 2 発目（SD エラーのときだけ残っている）。
    pulse_start = now;
    pulses_left--;
    led_on = true;
  }

  digitalWrite(USERLED_PIN, led_on ? HIGH : LOW);
}

// SD 裏コマンド（batt_unlock.txt）で過放電ラッチが解除されたときの完了画面を 2
// 秒表示する。
static void show_overdischarge_cleared_and_hold() {
  enqueueTask(createPlayMultiToneTask(1568, 80, 1, 3, 60));
  enqueueTask(createPlayMultiToneTask(2093, 120, 1, 3, 60));
  draw_overdischarge_cleared_screen();
  const uint32_t t0 = millis();
  while (millis() - t0 < 2000UL) {
    power_tick();
    airdata_update();
    backlight_tick();
    gnss_loop(7);
    imu_service_if_due();
    loop_userled();
    delay(10);
  }
  sw_push.reset();
  tft.fillScreen(COLOR_WHITE);
}

// 過放電（3.0V 未満）ラッチの全画面警告／完全ロックループ。
//   - USB 接続中（power_overdischarge_usb_active() == true）:
//       ボタン操作で抜けられない完全ロックとし、30
//       秒おきに警告トーン＋音声を鳴らす。
//   - 電池単体（power_overdischarge_usb_active() == false）:
//       警告トーン＋音声を 1
//       回鳴らしたあと、ボタン押下→解放で抜けてナビ画面へ進める。
//   - 画面表示中に解除用
//   SD（batt_unlock.txt）が認識されてラッチが消えた場合も即座に抜ける
//     （別カードで起動した状態から抜き差しして解除用 SD
//     を入れた場合も検知して再マウントする）。
static void run_overdischarge_warning_loop(bool from_loop) {
  backlight_set_saver(
      BL_SAVER_OFF); // 警告画面中は点滅省電力を止めて常時読めるようにする
  if (from_loop) {
    imulog_set_paused(true);
    if (link_chscan_running())
      link_chscan_stop(); // チャンネル調査中に USB
                          // を挿されたら元の無線設定へ戻す
    update_vario(true);
  }
  power_overdischarge_screen_enter();

  const int btn_pin = sw_push.getPin();
  const uint32_t enter_ms = millis();
  uint32_t last_draw_ms = 0;
  uint32_t last_sd_try_ms = enter_ms;
  uint32_t sd_high_since = 0;
  uint32_t sd_low_since = 0;
  bool sd_card_swapped = false;
  uint32_t usb_clear_ms =
      power_overdischarge_usb_active() ? 0 : (enter_ms ? enter_ms : 1);
  bool prev_usb = power_overdischarge_usb_active();
  bool prev_dismiss = false;
  bool seen_released = false;
  bool pressed_down = false;
  uint32_t press_down_ms = 0;
  uint32_t release_high_ms = 0;

  while (power_overdischarge_latched()) {
    const uint32_t now = millis();
    power_tick();
    airdata_update();
    if (from_loop)
      update_vario(true); // update_vario 内部の 100ms
                          // 間引きを跨いで確実に停止エッジを通す
    backlight_tick();
    gnss_loop(7);
    imu_service_if_due();
    loop_userled();

    // ロック画面中に解除用 SD
    // カードを挿した場合・既存カードを抜いて書き換え後に挿し直した場合の検出。
    // SD_DETECT が HIGH（未挿入）を 50ms 以上経てから LOW（挿入）が 500ms
    // 安定したら即座に再マウントし、 初期化失敗中（!good_sd()）は 5
    // 秒おきにリトライする。
    const bool sd_present = (digitalRead(SD_DETECT) == LOW);
    if (!sd_present) {
      sd_low_since = 0;
      if (sd_high_since == 0)
        sd_high_since = now ? now : 1;
      else if (now - sd_high_since >= 50UL)
        sd_card_swapped = true;
    } else {
      sd_high_since = 0;
      if (sd_low_since == 0)
        sd_low_since = now ? now : 1;
      if (now - sd_low_since >= 500UL &&
          (sd_card_swapped || (!good_sd() && now - last_sd_try_ms >= 5000UL)) &&
          !isTaskInQueue(TASK_INIT_SD) && !isTaskRunning(TASK_INIT_SD)) {
        sd_card_swapped = false;
        last_sd_try_ms = now;
        Task t;
        t.type = TASK_INIT_SD;
        enqueueTask(t);
      }
    }

    if (!power_overdischarge_latched())
      break;

    const bool usb_locked = power_overdischarge_usb_active();
    bool can_dismiss = false;

    if (usb_locked) {
      // USB 接続中は完全ロック: ボタン状態を毎回リセットして絶対に抜けさせない
      usb_clear_ms = 0;
      seen_released = false;
      pressed_down = false;
      release_high_ms = 0;
    } else {
      if (usb_clear_ms == 0)
        usb_clear_ms = now ? now : 1;
      // 電源投入直後の押しっぱなしや USB 抜去直後の過渡で誤スキップしないよう
      // 800ms 待つ
      can_dismiss = (now - enter_ms >= 800UL) && (now - usb_clear_ms >= 800UL);
      const bool pin_high = (digitalRead(btn_pin) == HIGH);
      if (!can_dismiss) {
        seen_released = pin_high;
        pressed_down = false;
        release_high_ms = 0;
      } else if (!seen_released) {
        if (pin_high)
          seen_released = true;
      } else if (!pressed_down) {
        if (!pin_high) {
          pressed_down = true;
          press_down_ms = now;
          release_high_ms = 0;
        }
      } else if (!pin_high) {
        // 押下継続中（または離し始めのチャタリングで LOW
        // に戻った場合は解放タイマをやり直す）
        release_high_ms = 0;
      } else if (now - press_down_ms < 25UL) {
        // 25ms 未満の短いノイズは押下とみなさない
        pressed_down = false;
        release_high_ms = 0;
      } else {
        // HIGH → LOW（25ms 以上）→ HIGH（25ms 連続安定）を確認してから抜ける。
        // 解放チャタリングが収まるまで待つことで、直後の loop() の
        // sw_push.read() への誤入力を防ぐ。
        if (release_high_ms == 0) {
          release_high_ms = now ? now : 1;
        } else if (now - release_high_ms >= 25UL) {
          enqueueTask(createPlayMultiToneTask(1046, 80, 1));
          enqueueTask(
              createLogSdTask("BATT LATCH: warning dismissed on battery"));
          break;
        }
      }
    }

    if (last_draw_ms == 0 || now - last_draw_ms >= 250UL ||
        usb_locked != prev_usb || can_dismiss != prev_dismiss) {
      last_draw_ms = now ? now : 1;
      prev_usb = usb_locked;
      prev_dismiss = can_dismiss;
      draw_overdischarge_warning_screen(usb_locked, can_dismiss);
    }
    delay(10);
  }

  power_overdischarge_screen_exit();
  if (power_take_overdischarge_cleared() || !power_overdischarge_latched()) {
    show_overdischarge_cleared_and_hold();
  }
  if (from_loop) {
    imulog_set_paused(getReplayMode());
  }
  sw_push.reset();
  tft.fillScreen(COLOR_WHITE);
  last_user_input_ms = millis();
  redraw_screen = true;
}

//===============SET UP CORE0=================
// Core0 の初期化処理。
// 初期化順の注意:
//   1. TFT を先に初期化しないとボタンピン設定が正常に動作しない（TFT_eSPI
//   の制約）。
//   2. GNSS は TFT 初期化後に setup する（バッファオーバーフロー防止のため）。
void setup(void) {
  // ★ bootwatch_stage() は起動の見張りの目印（bootwatch.h）。"BOOT PONS" までに止まると、
  //   Core1 の見張りか watchdog が再起動し、最後の目印を log.txt に残す。
  bootwatch_stage(BW_STAGE_SETUP);
  Serial.begin(38400);
  bootwatch_stage(BW_STAGE_SERIAL);
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
  pinMode(SD_DETECT, INPUT_PULLUP); // SDカード検出ピン（挿入時 LOW）
  analogReadResolution(12);         // 12bit = 0〜4095
  bootwatch_stage(BW_STAGE_PINS);
  power_setup();
  bootwatch_stage(BW_STAGE_POWER);
  // v7.1 のバックライトは、ここで PWM を設定するまでは GPIO のリセット時の
  // プルダウンでほぼ最大に点いている（backlight.cpp の ■ ハード）。setup_tft()
  // より前に呼んで、起動画面から輝度スイッチどおりの明るさにする。照度センサーは
  // i2c0 の後（下の backlight_als_setup）。
  backlight_setup();
  bootwatch_stage(BW_STAGE_BACKLIGHT);

  // setup switch
  pinMode(sw_push.getPin(), INPUT_PULLUP); // This must be after setup tft for
                                           // some reason of library TFT_eSPI.
  setup_tft();
  bootwatch_stage(BW_STAGE_TFT);
  // ★
  // **ここでタイトルを出しておく。**これが無いと起動画面が始まるまで画面が真っ白になる。
  //   白のままの区間は imu_setup()（SCH16T のリセット〜起動）と
  //   gnss_setup() で、**GNSS 未接続だと後者が約 5 秒**かかる
  //   （CFG の ACK 待ちが 7 コマンドぶん空振りする。log.txt の "GNSS CFG NO
  //   ACK"）。 GNSS を繋げば ACK が即返るので 1
  //   秒以下に縮む。**固まっているわけではない。**
  draw_boot_title();
  // ★★ **setup() の間は生 IMU ログを止める。**
  //   バッファの排出 imulog_take_pending() は **loop()
  //   からしか呼ばれない**ので、 setup() 中に貯めるとバッファ 2
  //   枚（IMULOG_RECS_PER_BUF × 2）で満杯になり、
  //   **それ以降は全部捨てられる**。捨てても seq は進むため、PC 側には
  //   「取りこぼし」として残り、decode_imulog.py のセッション分割を誤らせる。
  //   起動画面の間の生データは机上の値で使い道が無いので、記録しない。
  //   ★ 解除は明示的に書かない。loop() 冒頭の
  //     imulog_set_paused(getReplayMode()) が毎周回セットし直すので、
  //     loop() の 1 回目で自動的に解ける（リプレイ中なら正しく止まったまま）。
  imulog_set_paused(true);

  // i2c0 の初期化（MS5611・照度センサー VEML7700 共用。両 setup より先に呼ぶ）
  airdata_wire_begin();
  backlight_als_setup(); // 照度センサー（i2c0。ケースカバー基板）
  // IMU を先に初期化する（リセット後の起動時間を確保するため）
  imu_setup();
  attitude_setup(); // 姿勢 ESKF（センサー入力は imu.cpp/gnss.cpp から流し込む）
  // ★ ここで一度描く。IMU の結果は分かった時点ですぐ出す
  //   （draw_boot_diag(0) まで待つと「INITIALIZING SENSORS
  //   ...」のまま約1秒固まって見える）。
  BootCheckState imu_check = get_imu_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG;
  draw_boot_progress(imu_check, BOOT_CHECK_PENDING, BOOT_CHECK_PENDING);

  // MS5611 初期化（IMU の後に呼ぶ）
  airdata_setup();
  BootCheckState airdata_check =
      get_airdata_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG;
  draw_boot_progress(imu_check, airdata_check, BOOT_CHECK_PENDING);

  // ★★ **log.txt の最初の行はビルドの身元にする**（0.991 で追加）。
  //   これが無いと、後からログを読んだときに**どのファームが出したものか分からない**。
  //   バージョンだけでは足りない:
  //     ・どの基板向けのビルドか（v7 と v7.1 はブランチごと別のファーム）
  //     ・RELEASE でないビルドは GNSS シミュレーションやデバッグ出力が入りうる
  //   ★ 基板名は VERSION_TEXT
  //   を使う（settings.h。ここで書き直すと二重管理になる）。 ★
  //   置き場所はここ。setup() の冒頭に出すと、Core1 の mutex_init より先に
  //     enqueueTask を呼ぶ窓に入る（setup1() の冒頭コメント参照）。
  //     既存のログ呼び出しと同じ区間に置けば、**ログの 1 行目**にはなる。
#ifdef RELEASE
  const char *build_rel = "";
#else
  const char *build_rel = " [NOT RELEASE]";
#endif
  enqueueTask(createLogSdfTask("BOOT PONS %s build %lu board=%s imu=%s%s",
                               BUILDVERSION, (unsigned long)BUILDDATE,
                               VERSION_TEXT, imu_sensor_name(), build_rel));
  // 起動の見張りの watchdog を外し、前回が途中で止まっていたらログに出す（bootwatch.h）。
  bootwatch_boot_done_core0();

  // 両センサーの初期化完了後にセンサー組み合わせをログ出力する
  if (!get_imu_ok() && !get_airdata_ok()) {
    enqueueTask(createLogSdfTask(
        "VARIO: NO SENSORS - %s and MS5611 both absent", imu_sensor_name()));
  } else if (get_imu_ok() && !get_airdata_ok()) {
    enqueueTask(createLogSdfTask("VARIO: %s only - Kalman inactive, V/S=0",
                                 imu_sensor_name()));
  } else if (!get_imu_ok() && get_airdata_ok()) {
    enqueueTask(createLogSdTask("VARIO: MS5611 only - baro-only fallback"));
  } else {
    enqueueTask(createLogSdfTask("VARIO: %s+MS5611 Kalman fusion enabled",
                                 imu_sensor_name()));
  }

  gnss_setup();
  draw_boot_progress(imu_check, airdata_check,
                     get_gnss_cfg_ok() ? BOOT_CHECK_OK : BOOT_CHECK_NG);

  // ★ SD から読んだ設定が揃うのを待つ。**この下の 2 つが両方それを使う。**
  //     link_setup()       … CH / SF / 送受信モード
  //     startup_demo_tft() … 地図データ件数 / upward_mode
  //   Core1 が SD を初期化している間に Core0 が SD を触ると SDIO バスが競合して
  //   固まるので、待ちは必須。5 秒で打ち切るのは SD
  //   不良の機体でも起動させるため （打ち切った場合は log.txt の LINK SETUP の
  //   settings=0 で後から判別できる）。
  {
    unsigned long t = millis();
    while (!sd_setup_complete && millis() - t < 5000) {
      gnss_loop(7);         // 待機中も GNSS FIFO を読み捨てて overflow 防止
      imu_service_if_due(); // ★ IMU も引き取る（CLAUDE.md「飢えさせない」）
      delay(10);
    }
  }
  // Core1 の初期フラッシュ読込完了を念のため確認（通常は setup1 冒頭で数 ms
  // 以内に完了済み）
  {
    unsigned long t = millis();
    while (!power_latch_ready() && millis() - t < 200) {
      delay(1);
    }
  }

  // ---- 過放電（3.0V 未満）ラッチの起動時判定 ----
  // SD 裏コマンド（batt_unlock.txt）は上の sd_setup_complete 待ちの間に Core1
  // で処理済み。 ラッチが残っている場合は startup_demo_tft()（約 8
  // 秒）より前に即座に警告／ロック画面へ入る。
  bool skip_startup_splash = false;
  if (power_take_overdischarge_cleared()) {
    show_overdischarge_cleared_and_hold();
  } else if (power_overdischarge_boot_latched()) {
    enqueueTask(createLogSdfTask("BATT LATCH active at boot (usb=%d)",
                                 power_overdischarge_usb_active() ? 1 : 0));
    run_overdischarge_warning_loop(false);
    // 電池単体でボタン操作により警告を抜けた場合は、そのままナビ画面へ進める（SD
    // 解除時は通常起動演出へ）
    if (power_overdischarge_latched()) {
      skip_startup_splash = true;
    }
  }

  // ★ **起動画面より先に無線を開く。** 起動画面の下段に E220 の結果を出すため
  //   （ここより後だと、結果が出る頃には loop() が画面を描き替えてしまう）。
  //   読み上げと送信前チェックはここではやらない。setup() 末尾を見ること。
  link_setup();

  if (skip_startup_splash) {
    // startup_demo_tft() を飛ばす場合も、SD 異常を音で知らせる 500Hz×10
    // ビープだけは残す
    if (!good_sd()) {
      enqueueTask(createPlayMultiToneTask(500, 150, 10));
    }
  } else {
    startup_demo_tft();
  }

  // スケールリストの初期化（Google Map のズームレベルに対応する pixel/km 値）
  scalelist[0] = SCALE_EXSMALL_GMAP; // pixelsPerDegreeLat(5,35)/KM_PER_DEG_LAT;
                                     // 最広域（日本全体スケール）
  scalelist[1] = SCALE_SMALL_GMAP;   // pixelsPerDegreeLat(7,35)/KM_PER_DEG_LAT;
  scalelist[2] = SCALE_MEDIUM_GMAP;  // pixelsPerDegreeLat(9,35)/KM_PER_DEG_LAT;
  scalelist[3] = SCALE_LARGE_GMAP; // pixelsPerDegreeLat(11,35)/KM_PER_DEG_LAT;
  scalelist[4] =
      SCALE_EXLARGE_GMAP; // pixelsPerDegreeLat(13,35)/KM_PER_DEG_LAT;
  scalelist[5] = 200.0;   // 最大拡大
  scale = scalelist[scaleindex];
  redraw_screen = true;
  {
    // get_input_voltage() のピーク追跡（max_adreading）を通した値を記録する。
    // analogRead() の 1 発読みだと ADC のノイズがそのまま乗り、USB
    // を抜いた直後などに 実際より低い値がログに残ることがあった。 起動直後は
    // max_adreading が 0 なので、数サンプル回して追跡を立ち上げてから読む。
    // ここは setup() の末尾なので、16ms 程度の待ちは起動時間に影響しない。
    // 併せて、この後の最初のヘッダー描画も落ち着いた値から始められる。
    float startup_voltage = 0.0f;
    for (int i = 0; i < 16; i++) {
      startup_voltage = get_input_voltage();
      delay(1);
    }
    char pst[56];
    power_format_status(pst, sizeof(pst));
    enqueueTask(
        createLogSdfTask("SETUP DONE Battery: %.2fV %s", startup_voltage, pst));
  }

  // 無線モードの読み上げ。起動音(opening.wav)の後に鳴らしたいので、
  // link_setup() を前へ移したあともここに残してある。
  link_announce_mode();

  // ★ 送信前チェックの監視窓は**ここで張り直す。必ず loop() の直前で。**
  //   窓(5 秒)は millis() 基準なので、link_setup() の中で開始したままだと
  //   startup_demo_tft() のアニメーション(約 8 秒)の間に窓が過ぎてしまい、
  //   雑音を 1 サンプルも取れずに NOT MEASURED で終わる。
  link_preflight_restart();
}

//===============SET UP CORE1=================
// Core1 の初期化処理。タスクキュー・音声・SD を準備する。
// Core0 と独立して動作し、重いSD処理・音声再生を引き受けることで
// Core0 の描画ループをブロックしない設計になっている。
void setup1(void) {
  // タスクキューの mutex は何よりも先に初期化する。
  // arduino-pico の main() は multicore_launch_core1() を呼んでから setup()
  // を呼ぶため、 Core0 の setup() と この setup1() は並行に走る。Core0 は
  // setup() 内で enqueueTask() を 呼ぶので、mutex_init が遅れると未初期化の
  // mutex を掴む危険がある。 特に下の while(!Serial) は USB
  // 接続まで待つため、その後ろに置くと窓が極端に長くなる。
  mutex_init(
      &taskQueueMutex); // Core0/Core1 間のタスクキュー排他制御用 mutex を初期化
  bootwatch_stage1(BW_STAGE1_SETUP1); // 起動の見張りの目印（bootwatch.h）
  // 過放電ラッチを本体フラッシュ（EEPROM）から読み込む。
  // ★ 下の while(!Serial)
  // より前に置くこと（デバッグビルドの電池単体起動で止まらないようにするため）。
  // ★ ここでは log.txt へ何も積まない（setup() の "BOOT PONS ..." を必ず 1
  // 行目に保つため）。
  power_latch_init_core1();
  bootwatch_stage1(BW_STAGE1_LATCH);

  Serial.begin(38400);
#ifndef RELEASE
  while (!Serial) {
    delay(10);
  }
#endif

  init_destinations(); // 目的地リストを SD から読み込む
  bootwatch_stage1(BW_STAGE1_DEST);
  setup_sound();       // スピーカー・アンプ・PWM を初期化
  bootwatch_stage1(BW_STAGE1_SOUND);

  //====core1_separate_stack = true
  // の時、Serial.printがないとSDが動かない。詳細不明だが、消すな！
  Serial.println(""); // 消すな
  // call to Serial.println() might force the Arduino runtime or the RP2350’s
  // FreeRTOS (used in the RP2040/RP2350 Arduino core for dual-core support) to
  // fully initialize Core1’s context, ensuring that the SD library’s internal
  // state to set up. Without this call, Core1 might attempt to initialize the
  // SD card before its runtime environment is fully ready, especially with a
  // separate stack.
  //====

  // 起動時は SD_BOOT_TRY_COUNT 回まで試す（電源投入直後はカードが応答せず 1
  // 回目で外れることがある）。
  // 回数を絞っているのは接触不良時の長時間ブロック防止。それでも失敗したら
  // try_sd_recovery() が10秒ごとにリトライ。
  // ★ その前に Core0 の tft.begin() が PIO を取り終えるのを待つ（理由は display_tft.h の
  //   tft_pio_ready）。Core0 は setup() の冒頭から数 ms でここを通る。上限を置くのは、
  //   Core0 が手前で止まっても SD だけは使えるようにするため。
  // ★ 待ちが切れたら、SD を準備したあとで Core0 が止まったままかを確かめ、止まっていれば
  //   log.txt に止まった場所を書いて再起動する（起動の見張り。bootwatch.h）。
  bootwatch_wait_core0(&tft_pio_ready, 1000);
  setup_sd(SD_BOOT_TRY_COUNT);
  rtc_backup_load_sd();   // GNSS のバックアップ電池の推定の記録（SD が無くても「無し」を渡す。rtc_backup.cpp）
  gnss_dbd_load_sd();     // GNSS の航法データベース（BT1 が切れていた起動で戻す。SD が無くても「無し」を渡す。gnss_dbd.cpp）
  bootwatch_report_core1(&tft_pio_ready);
  // 起動音（opening.wav）は startup_demo_tft() 内でエンキューする（重複防止）
}

// その年月の日数。うるう年を考慮する（下の get_jst_now() の日付繰り上げ用）。
static int days_in_month_of(int mo, int y) {
  static const uint8_t dim[] = {0,  31, 28, 31, 30, 31, 30,
                                31, 31, 30, 31, 30, 31};
  if (mo < 1 || mo > 12)
    return 31; // GNSS の月が壊れていても配列外を読まない
  if (mo == 2 && (y % 4 == 0) && (y % 100 != 0 || y % 400 == 0))
    return 29;
  return dim[mo];
}

// ============================================================
// get_jst_now(): GNSS の UTC 日時から現在の JST 日時を求める
// ============================================================
// GNSS パケット受信時刻からの millis() 経過分を足して現在時刻を推定し、
// UTC+9 の JST に変換する。日をまたぐ場合は日付を繰り上げる（うるう年考慮）。
// Euler 角ログと生 IMU
// ログの両方が同じ日付のファイル名を使うため関数化している。
//
// 戻り値: GNSS 日時が有効なら true。false のとき出力引数の内容は不定。
static bool get_jst_now(int &y, int &mo, int &d, int &h, int &mi, int &s,
                        int &cs) {
  if (!get_gnss_date().isValid() || !get_gnss_time().isValid())
    return false;

  // millis() オフセットで UTC 時刻を推定し JST（UTC+9）に変換
  uint32_t elapsed_ms = millis() - get_gnss_fix_millis();
  int utc_cs = get_gnss_time().centisecond() + (int)(elapsed_ms / 10);
  int utc_s = get_gnss_time().second() + utc_cs / 100;
  int utc_m = get_gnss_time().minute() + utc_s / 60;
  int utc_h = get_gnss_time().hour() + utc_m / 60;
  cs = utc_cs % 100;
  s = utc_s % 60;
  mi = utc_m % 60;
  int jst_total_h = utc_h + 9;
  h = jst_total_h % 24;

  // JST 日付計算。★ 繰り上げは 1 日ぶんとは限らない。
  //   utc_h は「GNSS パケットの時刻 + millis() の経過」なので、GNSS が長時間
  //   止まったまま日時だけ有効に残ると 24 時間を超えて積み上がる。
  //   以前は next_day（+1
  //   日）しか見ていなかったため、そこから先は日付がずれていた。 実運用（電池
  //   12 時間）では 1 日を超えないが、桁の扱いを条件付きにしておく
  //   理由が無いので、繰り上がる日数ぶん回す。通常はループ 0〜1 回。
  y = get_gnss_date().year();
  mo = get_gnss_date().month();
  d = get_gnss_date().day();
  for (int add_days = jst_total_h / 24; add_days > 0; add_days--) {
    d++;
    if (d > days_in_month_of(mo, y)) {
      d = 1;
      if (++mo > 12) {
        mo = 1;
        y++;
      }
    }
  }
  return true;
}

//===============MAIN LOOP CORE0=================
// 警告音を抑制するか。**充電中 かつ マウントから外れた姿勢**のときだけ true。
//
// ★ なぜこの 2 条件か:
//   ・机に置いて充電すると、ロール +78 度・ピッチ -80 度のような姿勢になり、
//     バンク警告・地上ロール/ピッチ警告・較正の催促が延々と鳴る。
//     人力飛行機が地上でその姿勢になることはないので、**姿勢だけでも
//     「機体に載っていない」と判定できる**（settings.h の OFF_MOUNT_DEG）。
//   ・**USB を AND
//   条件にしてあるのは、机の上で意図的に回して音を確認したいため。**
//     充電せずに置けば従来どおり鳴るので、試験の手段が残る。
//   ・機体に載せたまま充電しても、水平ならマウント外れ判定に入らないので
//     **飛行前点検としての警告は保たれる**。
// ★ **抑制するのは音だけ。** 表示もログも残す。ログには (MUTED) を付けて、
//   「条件は成立していたが鳴らさなかった」ことが後から分かるようにする。
//   画面には draw_warn_mute_banner() が「なぜ静かなのか」を出す。
// ★ **解除操作を持たせない。** 機体に載せて水平に戻した瞬間に自動で復活し、
//   それはまさにリマインダーが必要なタイミングでもある。
//   覚えて解除しなければならないミュートは、当日に忘れると全ての警報が無音になる。
bool warn_muted_off_mount() {
  if (getReplayMode() || replay_voltage_active())
    return false; // 電池表示と同じ流儀
  // ★★ **ミラー中は抑制しない。** ミラー中の BANK WARN は「**機体の**ロール」で
  //   判定しているのに、attitude_is_off_mount() が見ているのは
  //   **ボート自身の姿勢**。これを条件にすると、ボートを机で充電しているだけで
  //   機体の本物のバンク警告を黙らせてしまう。
  if (link_mirror_active())
    return false;
  // ★ off_mount の判定は attitude_on_gyro() の中でしか更新されないので、
  //   IMU が死ぬと**前回の値のまま固まる**。その状態で「抑制中」と名乗ると
  //   画面が嘘になるので、ESKF が動いていないあいだは抑制しない。
  //   ★ 代償: **バリオだけは attitude_ready() を要求しない**（MS5611
  //   単独でも鳴る）
  //     ので、ESKF が収束する前は運搬中でも鳴りうる。off_mount が分からない
  //     あいだは「傾いているか」を判定できないので、ここは諦めている。
  if (!attitude_ready())
    return false;
  return usb_power_present() && attitude_is_off_mount();
}

// 地上の姿勢チェック（ロール／ピッチ）の共通部分。
// 「しきい値を超え**続けたら**、間隔を空けて繰り返し知らせる」。
//   戻り値 true = いま鳴らす。active =
//   無線へ出す発報状態（鳴らす間隔とは別物で、
//   継続時間を満たしていれば立つ）。運搬中のミュートもここで効かせる。
// ★★ **条件が解除されても last_warn は落とさない。** 落とすと、条件が明滅する
//   たびに鳴り直す（0.988 で受信側がこれを踏んだ。link.cpp
//   の同趣旨の注意書き参照）。 落とすのは継続時間（over_since）だけ。
// ★ 較正やり直しの警告は**わざと逆**（解除されたら間隔も落とす。APPLY
// で直ったら
//   次回は即座に知らせたい）なので、ここには入れていない。
// ★ 状態を構造体にしないのは、.ino の自動プロトタイプがファイル先頭へ入るため
//   （独自の型はその時点で未宣言になる）。
static bool ground_warn_due(uint32_t &over_since, uint32_t &last_warn, bool bad,
                            bool &active) {
  const uint32_t now = millis();
  if (!bad) {
    over_since = 0;
    active = false;
    return false;
  }
  if (over_since == 0)
    over_since = now;
  const bool held = (now - over_since) >= GROUND_ROLL_WARN_HOLD_MS;
  active = held && !warn_muted_off_mount();
  if (!held)
    return false;
  if (last_warn != 0 && now - last_warn < GROUND_ROLL_WARN_INTERVAL_MS)
    return false;
  last_warn = now;
  return true;
}

// ============================================================
//  loop() から毎回呼ぶ「刻み」関数
//    ★ **どれも画面モードに依存させない。**
//    警報を描画関数の中に置くと、その画面を
//      出しているときしか鳴らない（CLAUDE.md の約束）。だから loop()
//      の最上位から 順に呼ぶ。**ここに描画を混ぜないこと**（描画は
//      draw_current_screen）。
//    ★ 刻みの順番は loop() の並びがそのまま意味を持つ。入れ替えないこと。
// ============================================================
// GNSS モジュールの断線／復帰と、Fix 取得の通知。
static void gnss_conn_tick() {
  // GNSS モジュールの断線／復帰を log.txt に残す。
  // ★ 表示は draw_gnss_disconnect_warning()（地図の上にマゼンタの枠）だが、
  //   **画面を見ていなければ気づけない**。飛行 CSV
  //   には「位置が止まった」ことしか
  //   残らないので、原因が断線だったのかフィックス喪失だったのかを後から
  //   切り分けられるようにする。判定は get_gnss_connection()（gnss.cpp）。
  // ★ 起動直後（まだ 1 バイトも来ていない）では鳴らさない。一度でも繋がってから
  //   切れたときだけ残す。最初の接続そのものは基準を置くだけで記録しない。
  {
    static bool ever_conn = false, prev_conn = false;
    static uint32_t lost_alert_ms = 0;
    const bool conn = get_gnss_connection();
    if (!ever_conn) {
      if (conn) {
        ever_conn = true;
        prev_conn = true;
      }
    } else if (prev_conn != conn) {
      prev_conn = conn;
      if (conn) {
        enqueueTask(createLogSdTask("GNSS connection restored"));
        // ★ 復帰は断線の**逆向き**（392 → 784）。同じ 2
        // 音の上昇と下降で対にする。
        //   880 → 1318 は使わない。あれは**無線の電波復帰**で既に使っており、
        //   同じ音が 2 つの意味を持つと現場で取り違える（docs/pons_sound.md
        //   §10）。
        enqueueTask(createPlayMultiToneTask(392, 120, 1, 2));
        enqueueTask(createPlayMultiToneTask(784, 160, 1, 2));
        lost_alert_ms = 0;
      } else {
        enqueueTask(createLogSdTask("!!ERROR!! GNSS connection LOST"));
        lost_alert_ms = 0; // 0 にしておくと下の繰り返しが即座に 1 回目を鳴らす
      }
    }
    // ★ 切れている間は GNSS_LOST_ALERT_MS ごとに繰り返す。
    //   **下降 2 音（784 → 392、1 オクターブ下降）**。他の警報はすべて上昇か
    //   同音の連打なので、下降は「何かが失われた」専用の形になる。
    if (ever_conn && !conn &&
        (lost_alert_ms == 0 ||
         millis() - lost_alert_ms >= GNSS_LOST_ALERT_MS)) {
      lost_alert_ms = millis();
      enqueueTask(createPlayMultiToneTask(784, 200, 1, 3));
      enqueueTask(createPlayMultiToneTask(392, 300, 1, 3));
    }
  }

  // GNSS Fix 取得時に音声で通知（false→true の立ち上がりエッジを検出）
  // "wav/fixed.wav" は本体 FLASH にあるので SD の有無に関係なく鳴る
  // （0.982 まではここに SD
  // が無いときのチャイム音（ド・ミ・ソ）の代替があった）。 ★ 一度鳴らしたら
  // GNSS_FIXED_VOICE_GAP_MS は鳴らさない。不安定な所では fix が付いたり
  //   外れたりして毎秒のように連発していた。その間に付き直した分は黙って捨てる（ログには残す）。
  // ★ **測位の出どころが入れ替わった周回は鳴らさない（ログにも残さない）。**
  // get_gnss_fix() は
  //   ミラー中は機体の測位、ミラーが切れると受信機自身の測位を返す。受信機自身が測位できて
  //   いない（車内など）と、電波が戻った瞬間に「未測位 →
  //   測位」に見えて「測位しました」が鳴り、 電波が戻った知らせ（link.cpp
  //   の上昇 3 音）をかき消していた（実機ログで 103 回中 14 回）。
  {
    static bool prev_gnss_fix = false;
    static bool prev_mirror = false;
    static uint32_t fixed_voice_ms =
        0; // 最後に鳴らした時刻。0 = まだ鳴らしていない
    const bool cur_fix = get_gnss_fix();
    const bool cur_mirror = link_mirror_active();
    if (!prev_gnss_fix && cur_fix && cur_mirror == prev_mirror) {
      if (fixed_voice_ms == 0 ||
          millis() - fixed_voice_ms >= GNSS_FIXED_VOICE_GAP_MS) {
        fixed_voice_ms = millis();
        enqueueTask(createPlayWavTask(
            "wav/fixed.wav",
            4)); // 優先度4: AUTO10Kトーン(p=3)より高くして埋もれないよう
      }
      enqueueTask(createLogSdTask("GNSS FIX acquired"));
    }
    prev_gnss_fix = cur_fix;
    prev_mirror = cur_mirror;
  }
}

// バンク角の警告。
static void bank_alarm_tick() {
  // 実測（2026-07-26 琵琶湖 45分）では 4 度超えが 31 回あったが、中央継続は 0.2
  // 秒。 継続条件を付けないとチカチカ鳴るだけなので 1
  // 秒以上続いたときだけ発報する （同条件で実測 3 回）。さらに一度鳴ったら 60
  // 秒は繰り返さない。 姿勢が壊れているときに鳴り続けるのを防ぐのが主目的。 ★
  // ミラー中は「受信した機体のロール」で判定する。
  //   受信側は警告をビットで受け取るのではなく**自分で再計算する**方針なので
  //   （link_proto.h の status
  //   設計を参照）、入力をミラー値に差し替えるだけでよい。 計算が 1
  //   本のままなので、送信側と受信側で挙動がずれる余地が無い。
  // ★ ミラー中は自機の Roll/Pitch/Yaw func を見ない（2026-10-27）。送信機が ON
  // なら機体の画面でも
  //   バンク警報が鳴るので、受信機でも鳴らす。送信機が OFF
  //   なら姿勢が来ないので鳴らない。
  if ((link_mirror_active() || attitude_get_rpy_enabled()) &&
      !getReplayMode() &&
      (link_mirror_active() ? link_get_attitude(bank_roll_tmp, bank_pitch_tmp)
                            : attitude_ready())) {
    static uint32_t bank_over_since_ms = 0;
    static uint32_t bank_last_warn_ms = 0;
    // 発報済みフラグ。解除条件（バンクが戻る＋時間経過）を満たすまで鳴らさない。
    // ★ true で始める。起動時点で既に 4 度を超えていた場合（マウントのズレ、
    //   傾いた場所に置いてある、較正前など）に、一度も水平に戻らないまま
    //   鳴り出すのを防ぐため。一度でも 3 度未満に戻れば通常動作に入る。
    static bool bank_warned = true;
    float wr, wp, wy;
    // ミラー中は受信した機体のロール、それ以外は自機の姿勢。
    // 入力をここで差し替えるだけで、以降の判定ロジックは 1 本のまま使える。
    if (link_mirror_active()) {
      wr = bank_roll_tmp;
      wp = bank_pitch_tmp;
      wy = 0.0f;
    } else
      attitude_get_euler(wr, wp, wy);
    uint32_t now_ms = millis();
    const float bank_abs = fabsf(wr);

    // 再武装。バンクが BANK_WARN_CLEAR_DEG 未満まで戻り、かつ発報から
    // BANK_WARN_REARM_MS
    // 以上経ってから。バンクが超過したままなら何時間でも鳴らさない
    // （センサーのズレで超過が続く不具合を想定しているため、時間だけでは解除しない）。
    if (bank_warned && bank_abs < BANK_WARN_CLEAR_DEG &&
        (now_ms - bank_last_warn_ms) >= BANK_WARN_REARM_MS) {
      bank_warned = false;
    }

    if (bank_abs > BANK_WARN_DEG) {
      if (bank_over_since_ms == 0)
        bank_over_since_ms = now_ms;
      if (!bank_warned && (now_ms - bank_over_since_ms) >= BANK_WARN_HOLD_MS) {
        bank_last_warn_ms = now_ms;
        bank_warned = true;
        const bool muted = warn_muted_off_mount();
        if (!muted)
          enqueueTask(createPlayWavTask("wav/bank_warning.wav", 3));
        enqueueTask(
            createLogSdfTask("BANK WARN roll=%+.1f deg%s", wr,
                             muted ? " (MUTED: charging + off mount)" : ""));
      }
    } else {
      bank_over_since_ms = 0; // 一度でも下回ったら継続時間をリセット
    }
  }
}

// 電池電圧の低下警告。
static void batt_alarm_tick() {
  // ★ **警報を描画関数の中に置かないこと。** 元は draw_footer()
  // の中にあったため、
  //   次の 2 つの穴があった。
  //     1. draw_footer() は地図画面からしか呼ばれない → 設定画面・WIRELESS
  //     画面・
  //        IMU/ESKF 画面などを開いている間は電圧が下がっても鳴らない
  //     2. 受信モードでは電池表示が別の分岐（S/R の 2 台ぶん表示）に入るため、
  //        **一度も鳴らなかった**。追走ボートとプラットフォームは常に受信モードなので、
  //        実質「ボートの電池切れは警告されない」状態だった
  //   他の警報（バンク角・コース・地上チェック）と同じくここへ置き、画面モードにも
  //   無線モードにも依存しないようにする。
  //
  //   毎秒 1 回だけ読むのは、ADC の連打を避けるためと、
  //   get_input_voltage() のピーク追跡をどの画面でも回し続けるため
  //   （地図画面以外では誰も呼ばないと追跡が止まり、戻ったときに古い値が出る）。
  {
    static uint32_t last_batt_check_ms = 0;
    static uint32_t last_batt_warn_ms = 0;
    if (millis() - last_batt_check_ms >= 1000) {
      last_batt_check_ms = millis();
      const float bv = get_input_voltage();
      // 充電中は鳴らさない。リプレイ中も鳴らさない（過去の電圧で今の警告を出しても
      // 意味がなく、SD
      // のテキストログにも当時の値が「今の警告」として混ざるため）。 ★
      // 「充電中」は usb_power_present() で判定する。2026-10-15 は壊れた充電 IC
      // が
      //   USB に見せかけたせいで、VSYS 2.5V でもこの警告が鳴らなかった。
      // ★
      // 電源経路の異常のあいだは鳴らさない。電圧が低いのは電池ではなく故障のせいで、
      //   「バッテリー低下」と言うと交換で済むと誤解させる（power_tick
      //   が別の音で鳴らす）。
      // ★ v7.1 で SW OFF のまま USB
      // で動いているあいだも鳴らさない。電池電圧は読めず
      //   bv は 0V へ下がっていく（power.h）。usb_power_present()
      //   だけに頼ると、 VBUS が一瞬 4.3V を割ったときに 0V
      //   を電池切れと言ってしまう。
      if (!usb_power_present() && !power_sw_off_on_usb() &&
          !power_fault_active() && !replay_voltage_active() &&
          bv <= BAT_LOW_VOLTAGE &&
          (last_batt_warn_ms == 0
               ? millis() >= BAT_WARN_INTERVAL_MS
               : millis() - last_batt_warn_ms >= BAT_WARN_INTERVAL_MS)) {
        last_batt_warn_ms = millis();
        // WAV を再生（最低 volume 60 保証）。音声は本体 FLASH にあるので
        // SD の有無に関係なく鳴る。0.982 までここに「SD 未認識なら高音ビープ
        // 3 回」の代替があったが、SD が無くても喋るようになったので消した。
        enqueueTask(createPlayWavTask("wav/battery_low.wav", 1, 60));
        enqueueTask(createLogSdfTask("Battery low: %d%% (%.2fV)",
                                     battery_percent(bv), bv));
      }
      // ---- 電池低下のときの省電力: バックライトを 2
      // 段階で点滅させる（backlight.cpp）----
      //   第 1 段階（<= 3.50V）: 2s 点灯 /  4s 消灯（復帰 >= 3.65V または USB）
      //   第 2 段階（<= 3.00V）: 2s 点灯 / 10s 消灯（第 1 段階へ戻る閾値は
      //   >= 3.20V）
      // ★ BAT_LOW_VOLTAGE / BAT_CRIT_VOLTAGE
      // そのもので抜けないこと。消灯中は負荷が減って
      //   電圧が戻るため、点滅の往復が起きる（settings.h の *_EXIT_VOLTAGE）。
      {
        static BacklightSaver saver = BL_SAVER_OFF;
        const bool measurable =
            !usb_power_present() && !power_sw_off_on_usb() &&
            !power_fault_active() && !replay_voltage_active();
        if (!measurable ||
            (saver != BL_SAVER_OFF && bv >= BAT_SAVER_EXIT_VOLTAGE)) {
          if (saver != BL_SAVER_OFF) {
            saver = BL_SAVER_OFF;
            backlight_set_saver(BL_SAVER_OFF);
            enqueueTask(
                createLogSdfTask("BATTERY SAVER off (%.2fV%s)", bv,
                                 measurable ? "" : ", USB / not measurable"));
          }
        } else if (millis() >= BAT_WARN_INTERVAL_MS || saver != BL_SAVER_OFF) {
          BacklightSaver next = saver;
          if (bv <= BAT_CRIT_VOLTAGE) {
            next = BL_SAVER_CRIT;
          } else if (saver == BL_SAVER_CRIT) {
            if (bv >= BAT_CRIT_SAVER_EXIT_VOLTAGE)
              next = BL_SAVER_LOW;
          } else if (saver == BL_SAVER_OFF) {
            if (bv <= BAT_LOW_VOLTAGE)
              next = BL_SAVER_LOW;
          }
          if (next != saver) {
            saver = next;
            backlight_set_saver(saver);
            const bool crit = (saver == BL_SAVER_CRIT);
            const unsigned long on_s =
                (crit ? BL_SAVER_CRIT_ON_MS : BL_SAVER_ON_MS) / 1000UL;
            const unsigned long off_s =
                (crit ? BL_SAVER_CRIT_OFF_MS : BL_SAVER_OFF_MS) / 1000UL;
            enqueueTask(createLogSdfTask(
                "BATTERY SAVER stage%d: backlight %lus on / %lus off (%.2fV)",
                (int)saver, on_s, off_s, bv));
          } else if (saver != BL_SAVER_OFF) {
            // 警告画面ループ等で一時的に BL_SAVER_OFF
            // にされていた場合にも状態を揃える
            backlight_set_saver(saver);
          }
        }
      }
    }
  }
}

// 地上での姿勢チェックと較正の催促。中の 4 つは「地上・静止」の条件を共有する。
static void ground_check_tick() {
  // 自動ロールトリムは対地速度 3m/s 超でしか働かないので地上では動かない。
  // しかしマウントのズレを直せるのは地上だけなので、ここで別建てに見張る。
  // 静止していることを条件に入れて、運搬中・組み立て中の傾きを除外する。
  // ミラー中は get_gnss_mps() が機体の対地速度になるため、判定が噛み合わない。
  // ★
  // 無線へ出す発報状態は、ここで毎周回落としてから下で立て直す（上の宣言参照）。
  ground_roll_warn_active = ground_pitch_warn_active =
      ground_calib_warn_active = false;
  if (attitude_get_rpy_enabled() && attitude_ready() && !getReplayMode() &&
      !link_mirror_active() && get_gnss_mps() <= LEVEL_CALIB_MAX_MPS &&
      attitude_is_static()) {
    static uint32_t roll_since = 0,
                    roll_warn = 0; // ground_warn_due() が持つ状態
    static uint32_t pitch_since = 0, pitch_warn = 0;
    float gr, gp, gy;
    attitude_get_euler(gr, gp, gy);
    const uint32_t now_ms = millis();

    // ロール: 0（または SET ROLL で申告した値）からのズレ
    ground_roll_ok_lo = attitude_get_roll_target() - GROUND_ROLL_WARN_DEG;
    ground_roll_ok_hi = attitude_get_roll_target() + GROUND_ROLL_WARN_DEG;
    if (ground_warn_due(roll_since, roll_warn,
                        fabsf(gr - attitude_get_roll_target()) >
                            GROUND_ROLL_WARN_DEG,
                        ground_roll_warn_active)) {
      const bool muted = warn_muted_off_mount();
      if (!muted)
        enqueueTask(createPlayWavTask("wav/roll_check.wav", 3));
      enqueueTask(
          createLogSdfTask("GROUND ROLL %+.1f deg (check mount / APPLY)%s", gr,
                           muted ? " (MUTED: charging + off mount)" : ""));
    }

    // ピッチ: 申告値 SET PITCH からのズレで見る。
    // 地上の機体ピッチは場所によって変わる（プラットホーム -3.5 度、平地 0
    // 度）ので、 ロールのように絶対値 0 を基準にはできない。
    //
    // ただしプラットホーム近傍だけは例外。台の上と台の手前の地面は装置から
    // 区別がつかないため、基準を一点ではなく PITCH_TARGET_MIN_DEG 〜 0 度の
    // 区間とみなす（平地で APPLY してから台へ上げても鳴らないようにする）。
    // 測位できていないときは近傍とみなさず、従来どおりの厳しい判定を使う。
    //
    // ※ 距離計算（haversine, double）は loop() 毎に回すには重いので 1 秒に 1
    // 回だけ
    //   更新して結果を使い回す。60
    //   秒継続してから発報する判定なので粒度は足りる。
    static uint32_t gndp_geo_ms = 0;
    static bool gndp_near_pla = false;
    if (gndp_geo_ms == 0 ||
        (now_ms - gndp_geo_ms) >= PLATFORM_NEAR_RECHECK_MS) {
      gndp_geo_ms = now_ms;
      gndp_near_pla = get_gnss_fix() &&
                      calculateDistanceKm(get_gnss_lat(), get_gnss_lon(),
                                          pla_lat, pla_lon) <= PLATFORM_NEAR_KM;
    }
    const bool near_platform = gndp_near_pla;
    // 許容範囲。下の判定（pitch_bad）と同じ式なので、片方だけ直さないこと。
    ground_pitch_ok_lo =
        near_platform ? PITCH_TARGET_MIN_DEG - GROUND_PITCH_WARN_DEG
                      : attitude_get_pitch_target() - GROUND_PITCH_WARN_DEG;
    ground_pitch_ok_hi =
        near_platform ? GROUND_PITCH_WARN_DEG
                      : attitude_get_pitch_target() + GROUND_PITCH_WARN_DEG;
    bool pitch_bad =
        near_platform
            ? (gp > GROUND_PITCH_WARN_DEG ||
               gp < PITCH_TARGET_MIN_DEG - GROUND_PITCH_WARN_DEG)
            : (fabsf(gp - attitude_get_pitch_target()) > GROUND_PITCH_WARN_DEG);
    if (ground_warn_due(pitch_since, pitch_warn, pitch_bad,
                        ground_pitch_warn_active)) {
      const bool muted = warn_muted_off_mount();
      if (!muted)
        enqueueTask(createPlayWavTask("wav/pitch_check.wav", 3));
      enqueueTask(createLogSdfTask(
          "GROUND PITCH %+.1f deg (target %+.1f, nearPLA %d)%s", gp,
          attitude_get_pitch_target(), (int)near_platform,
          muted ? " (MUTED: charging + off mount)" : ""));
    }

    // ---- 較正のやり直しが必要（マウントから外された形跡がある）----
    // 画面には赤字で出しているが、**地上では画面を見ていないことが多い**。
    // 気づかないまま飛ぶと、姿勢表示・バンク角警告・自動ロールトリムが
    // まとめて信用できない状態のまま飛ぶことになるので、音でも促す。
    //
    // ★ このブロックの条件（地上・静止・リプレイ/ミラー以外）をそのまま使う。
    //   **飛行中は絶対に鳴らさない。**飛行中に較正はできないし、
    //   対処のしようがない警告で注意をそらすほうが危険だから。
    //   最低保証音量は付けない。地上で静止しているとき＝人が機体のそばにいるときにしか
    //   鳴らないので、音量を絞る判断を上書きする理由が無い（60
    //   は電池切れ専用）。 （0.982 まではここに「SD
    //   が無いときは低いトーンで代替」の分岐があった。
    //     音声を FLASH に焼いたので SD の有無に関係なく必ず喋る。）
    {
      static uint32_t eskf_apply_last_warn_ms = 0;
      if (attitude_needs_apply()) {
        // 無線へ。**ここは地上・静止のブロックの中**なので、飛行中は立たない
        // （送信機が飛行中に黙るものを受信機に喋らせないため）。
        // 運搬中のミュートも同じく効かせる（上のロールのコメント参照）。
        ground_calib_warn_active = !warn_muted_off_mount();
        if (eskf_apply_last_warn_ms == 0 ||
            (now_ms - eskf_apply_last_warn_ms) >= ESKF_APPLY_WARN_INTERVAL_MS) {
          eskf_apply_last_warn_ms = now_ms;
          const bool muted = warn_muted_off_mount();
          if (!muted)
            enqueueTask(createPlayWavTask("wav/eskf_calib_required.wav", 3));
          enqueueTask(
              createLogSdfTask("ESKF CALIBRATION REQUIRED (on ground)%s",
                               muted ? " (MUTED: charging + off mount)" : ""));
        }
      } else {
        eskf_apply_last_warn_ms = 0; // 較正し直したら次回は即座に知らせる
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
    //   立てると、ヘッダのピッチ数字に「読むな」の赤い取り消し線が入り、
    //   PONS Link でボート側にも fault として飛ぶ（link.cpp の
    //   LINK_ST_FAULTS_TONE）。根拠のない推測でそこまでやらない。
    //   画面は設定画面（IMU/ESKF 1/2）の注意書きだけ、音はここで一度だけ。
    //
    // ★ 繰り返さないこと。2 分おきに鳴らすと、プラットホームの上で鳴ったときに
    //   「止めるために APPLY する」人が出る。台の上で APPLY すると、正しかった
    //   較正が台の姿勢で上書きされてしまう（APPLY は静止していれば通る）。
    //   一度伝えたら、あとは人の判断に任せる。
    {
      static bool calib_day_warned = false;
      const uint32_t today_jst = get_gnss_jst_yyyymmdd();
      if (!calib_day_warned && !attitude_needs_apply() &&
          attitude_calib_date_stale(today_jst)) {
        calib_day_warned = true;
        // 先に 2 音鳴らしてから WAV を積む（APPLY 成功音と同じ作り）。
        // 2 音は「これから何か言う」という注意の引き方なので、WAV が必ず鳴る
        // ようになった今も残す。eskf_calib_required.wav は流用しない。
        // 「較正が必要」と断定する文言になってしまうため。
        enqueueTask(createPlayMultiToneTask(1319, 90, 2));
        enqueueTask(createPlayWavTask("wav/eskf_calib_oldday.wav", 3));
        enqueueTask(createLogSdfTask(
            "ESKF CALIB from another day (calib %lu, today %lu)",
            (unsigned long)attitude_get_calib_date(),
            (unsigned long)today_jst));
      }
    }
  }
}

// ESKF の APPLY（機体ゼロ点の較正）の実行待ちと、完了のログ。
static void eskf_apply_tick() {
  // APPLY の待ち時間は「指を離してから」数える。
  // 長押しコールバックは閾値に達した時点で呼ばれるので、押しっぱなしのまま
  // 実行されてしまう（3 秒押すと押下中に焼き付いていた）。押している間は
  // カウントダウンを進めず、離した瞬間から改めて 1 秒待つ。
  {
    static bool calib_btn_held = false;
    if (attitude_calib_pending()) {
      if (digitalRead(SW_PUSH) == LOW) { // 押下中（アクティブ LOW）
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
    // 較正した日を刻む。測位していなければ
    // 0（不明）が入り、日付判定は働かない。 ★ createSaveSettingTask()
    // より先に呼ぶ。保存は Core1 が後から実行するので
    //   どちらの順でも実害は無いが、「保存する値を全部そろえてから保存を頼む」
    //   という読み方ができるようにしておく。
    attitude_set_calib_date(get_gnss_jst_yyyymmdd());
    enqueueTask(createSaveSettingTask()); // 次回起動でも効くよう保存
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
      GnssDate d = get_gnss_date();
      GnssTime t = get_gnss_time();
      if (d.isValid() && t.isValid())
        enqueueTask(createLogSdfTask("ESKF APPLY done R%+.2f P%+.2f at "
                                     "%04d-%02d-%02d %02d:%02d:%02d UTC",
                                     lvr, lvp, d.year(), d.month(), d.day(),
                                     t.hour(), t.minute(), t.second()));
      else
        enqueueTask(createLogSdfTask(
            "ESKF APPLY done R%+.2f P%+.2f at ---- (no GNSS time)", lvr, lvp));
    }
  }
}

// 飛行ログのうち Core0 が組むもの（生 IMU ログ）。
// ※ ESKF の姿勢・風（旧 imu_replaydata/YYYYMMDD.txt, 5Hz）は飛行 CSV（data/*.csv, 2Hz）に
//    直接統合したため、ここでの別ファイル書き出しは廃止した。
// ★ JST はここで自分で求める。**loop() から引き回さないこと** — 引き回すと
//   ログを足すたびに loop() の引数が増える。get_jst_now() は四則演算だけで、
//   2 回呼んでも誤差は 1 センチ秒（単調性のガードが吸収する）。
static void flightlog_tick() {
  int jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs;
  const bool jst_valid =
      get_jst_now(jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs);

  // ---- 生 IMU ログ: 満杯になったバッファを Core1 へ引き渡す ----
  // imulog_take_pending() が 0/1 を返したら、そのバッファは「Core1
  // へ引き渡し済み」に マークされているため、必ずタスクを enqueue
  // しないとバッファが解放されず Core0 が詰まる。 リプレイ中は記録しない（Euler
  // ログと同じ理由: 再生日時のファイルを実センサ値で汚さない）。
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
        // GNSS 未測位（屋内テスト等）。日付が決まらないので合言葉を渡す。
        // Core0 から SD
        // は触れないので、実ファイル名（imuraw/nofixNNN.bin）の決定は Core1
        // 側の imulog_write_buffer() がやる。番号を振るのは、固定名 1 つだと
        // 複数日・複数起動が 1 ファイルに混ざって切り分けられなくなるため。
        // レコードはホスト時刻 t_us と BOOT
        // レコードを持つので、これでも解析はできる。 ファイルのタイムスタンプは
        // SdFat の下限（1980 年以降）を満たす固定値にする。
        // 書式文字列としてではなく %s で渡す（パスに %
        // が混ざっても壊れないように）
        snprintf(imu_fname, sizeof(imu_fname), "%s", IMULOG_NOFIX_REQUEST);
        queued = enqueueTask(
            createFlushImuLogTask(imulog_buf, imu_fname, 2020, 1, 1, 0, 0, 0));
      }
      // キュー満杯でタスクが捨てられたらバッファを戻す。
      // 戻さないと busy のままになり、2 枚とも失った時点で記録が恒久停止する。
      if (!queued)
        imulog_release_pending(imulog_buf);
    }
  }
}

// 60 秒ごとのシステムログ（電圧・時刻・センサーのレート）。
// JST は flightlog_tick() と同じ理由でここで求める。
static void periodic_log_tick() {
  int jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs;
  const bool jst_valid =
      get_jst_now(jst_year, jst_month, jst_day, log_h, log_m, log_s, log_cs);
  // 60秒ごとに電圧と JST 時刻をテキストログへ記録（どの画面モードでも実行）
  {
    static unsigned long last_volt_log_ms = 0;
    if (millis() - last_volt_log_ms >= 60000UL) {
      last_volt_log_ms = millis();
      // ★ 時刻は **get_jst_now() の判定を通ったときだけ**書く。
      //   以前は get_gnss_time()
      //   の生の値を無条件に使っていたため、測位できていない間は _hour/_min が
      //   0 のままで「09:00 JST」と記録されていた。ログ上は
      //   正しい時刻に見えるので、あとから見たときに嘘に気づけない。
      //   （get_jst_now() は millis()
      //   の経過ぶんも足してあり、日跨ぎも処理済み。）
      char jst_str[8];
      if (jst_valid)
        snprintf(jst_str, sizeof(jst_str), "%02d:%02d", log_h, log_m);
      else
        strlcpy(jst_str, "--:--", sizeof(jst_str));
      // bst は最長 " bl=100% lux=120794 bright saver2 dim"（37 文字）
      char pst[56], bst[40];   // pst は最長 "usb=1 vbus=4.95V sw=off hot od bk=192.0h"
      power_format_status(pst, sizeof(pst));
      backlight_format_status(
          bst, sizeof(bst)); // " bl=35% lux=12 eco"（自動調光の調整用）
      enqueueTask(createLogSdfTask("volt=%.2fV %s%s cpu=%.1fC %s JST",
                                   get_input_voltage(), pst, bst,
                                   analogReadTemp(), jst_str));
      // GNSS の受信状態の 1 分まとめ（`grf` 行。debug/v7-noise-test と同じ並びで v7 と比べられる。gnss.cpp）
      gnss_log_rf_minute();

      // センサー受信レートも 60 秒ごとに残す（RELEASE
      // ビルドでもシリアルなしで確認できる）。
      //   raw GYR/ACC が 250Hz から大きく落ちていれば → loop() の周回が足りない
      //     （LPF2 の最小推奨 150Hz を下回ると折り返す。settings.h の IMU_POLL_INTERVAL_US）。
      //   gapmax が数十 ms を超えていれば → どこかで Core0 が長く止まっている。
      //   stale が増えていれば → 読み出しが止まり predict を止めた回数。
      //   MS5611 が ~40Hz から落ちていれば → 気圧観測レートの低下。
      //   drop が増えていれば → SD 書き出しが追いつかず生ログを取りこぼしている。
      // ※ 行を分けること。log_sd() は logtext[128] に "<起動秒>:" を前置するため、
      //   1 行にまとめると稼働時間が延びるほど末尾が切り詰められる（実際 wrote=
      //   が消えた）。
      enqueueTask(createLogSdfTask("rate MS5611=%.1f Hz (%s: no fusion)",
                                   get_airdata_win_hz(), imu_sensor_name()));
      enqueueTask(
          createLogSdfTask("imu gapmax=%lums",
                           (unsigned long)(get_imu_poll_gap_max_us() / 1000)));
      enqueueTask(createLogSdfTask(
          "raw GYR=%.1f ACC=%.1f Hz stale=%lu drop=%lu wrote=%lu",
          get_imu_gyro_hz(), get_imu_accel_hz(),
          (unsigned long)get_imu_acc_stale_skips(),
          (unsigned long)imulog_get_dropped(),
          (unsigned long)imulog_get_written()));
      // ヨーの不確かさを飛行後に確認できるよう残す。
      // 生ログには σ そのものは入っていないが、tools/imulog/eskf.py を流せば
      // 50Hz 全分解能で再現できる。こちらは 60 秒ごとの粗い突き合わせ用。
      {
        float lvr, lvp;
        attitude_get_level_offset(lvr, lvp);
        // yawsrc: 0 = 未設定（絶対方位が無い） / 1 = GNSS 航跡から入れた
        // gobs: 重力観測（levelling update）の累計回数。
        // ★ **ここが増えていなければゲートが閉じている。** GNSS 断中に
        //   ロール・ピッチの錨になるのがこの観測なので、0 のままなら
        //   settings.h の ESKF_LEVEL_* を見直す（旋回レートのしきい値が主）。
        // gnd:
        // ヨーが決まった後に、地上での再水平化に入った回数（ESKF_GROUND_*）。
        //   飛行中に増えていたら、地上の判定が飛行中に誤って成立している。
        enqueueTask(createLogSdfTask(
            "eskf ready=%d yawsig=%.1f yawsrc=%d gobs=%lu gnd=%lu lvl R%+.1f "
            "P%+.1f",
            (int)attitude_ready(), attitude_get_yaw_sigma_deg(),
            (int)attitude_get_yaw_source(),
            (unsigned long)attitude_get_level_updates(),
            (unsigned long)attitude_get_ground_relevels(), lvr, lvp));
      }
    }
  }
}

// 画面モードごとの描画。**判断や発報はここに置かない**（警報は loop()
// の最上位の
// *_tick() 群が持つ。CLAUDE.md の約束）。ここは「どの画面を描くか」だけ。
// new_gnss_info: 新しい位置が届いたか。MODE_MAP が再描画の契機に使う。
static void draw_current_screen(bool new_gnss_info) {
  if (screen_mode == MODE_SETTING) {
    if (redraw_screen) {
      draw_setting_mode(selectedLine, cursorLine);
    }
  } else if (screen_mode == MODE_GNSSDETAIL) {
    if (redraw_screen)
      draw_gnssdetail(detail_page);
  } else if (screen_mode == MODE_SDDETAIL) {
    if (sd_detail_loading_displayed && !loading_sddetail)
      redraw_screen = true;
    if (redraw_screen)
      draw_sddetail(detail_page);
  } else if (screen_mode == MODE_REPLAYSELECT) {
    if (replay_loading_displayed && !loading_replaylist)
      redraw_screen = true; // Core1 のファイル一覧取得が完了したので描き直す
    if (redraw_screen)
      draw_replayselect(replay_list_page, replay_cursor);
  } else if (screen_mode == MODE_MAPLIST) {
    if (redraw_screen)
      draw_maplist_mode(detail_page);
  } else if (screen_mode == MODE_IMUDETAIL) {
    if (redraw_screen) {
      if (imu_guide_page >= 0)      draw_imu_guide(imu_guide_page);   // 入口の案内
      else if (tas_guide_page >= 0) draw_tas_guide(tas_guide_page);   // 推定 TAS の案内
      else                          draw_imudetail(detail_page);
    }
  } else if (screen_mode == MODE_WIRELESS) {
    if (redraw_screen)
      draw_wireless(wireless_cursor);
  } else if (screen_mode == MODE_CHSCAN) {
    // チャンネル調査の一覧。進み具合は 1.05 秒の強制再描画で更新する
    if (redraw_screen)
      draw_chscan();
  } else if (screen_mode == MODE_PCCONNECT) {
    if (redraw_screen)
      draw_pcconnect();
  } else if (screen_mode == MODE_VARIODETAIL) {
    if (redraw_screen) {
      draw_variodetail(detail_page);
      draw_vsi(); // 全画面再描画直後に即座に VSI を上書き（白フリッカー防止）
      vsi_sprite.pushSprite(VSI_X, 40);
    }
    // airdata 更新タイミングで VSI バーを直接転写（メイン画面と同じ頻度）
    if (airdata_updated) {
      draw_vsi();
      vsi_sprite.pushSprite(VSI_X, 40); // backscreen は y=40 から開始
    }
  } else if (screen_mode == MODE_MAP) {
    // ★
    // **ミラー中は、機体のフレームを受けた直後だけ描き直す**（link_take_draw_frame）。
    //   受信中のフレームに TFT のバスの雑音を重ねないため（settings.h の
    //   LINK_DRAW_*）。
    //   ミラー中の画面の中身はフレームが届いたときしか変わらないので、見た目はほぼ同じ。
    //   受信機自身の
    //   GNSS（2Hz）では描き直さない（変わるのはボート自身の位置の点だけ）。
    //   強制再描画は loop() の側で link_draw_window_ok() の窓に入れてある。
    if (link_mirror_active()) {
      if (link_take_draw_frame())
        redraw_screen = true;
    } else if (new_gnss_info) {
      redraw_screen = true;
    }

    // GNSS 更新のたびに redraw_screen が立つため、
    // 通常は毎秒 2 回程度この描画ブロックが実行される。
    if (redraw_screen) {
      TIMING_START(redraw);
#ifndef RELEASE
      c0_is_redrawing = true;
#endif
      float new_truetrack = get_gnss_truetrack();
      double new_lat = get_gnss_lat();
      double new_long = get_gnss_lon();
      set_new_location_off(); // GNSS の「新位置フラグ」をクリア

      // ※ 無線へのテレメトリ送出はここでは行わない。
      //   送信の刻み（1Hz）は link_tx_tick() が持っている（link_loop() 参照）。
      //   ここは GNSS 更新のたび（2Hz）に走るので、投げると倍のレートになり、
      //   電波の占有時間と送信電流がそのまま倍になる。

      // 画面の上方向を設定: トラックアップ時は機首方向、ノースアップ時は
      // 0（北）
      float drawupward_direction = new_truetrack;
      if (is_northupmode()) {
        drawupward_direction = 0;
      }

      nav_update();  // 真方位コース(TC)・目的地距離(dist)を最新 GNSS
                     // 位置で再計算
      draw_header(); // ヘッダー（速度・衛星数など）を TFT に直接描画

      // 描画はすべてバックスクリーンに対して行い、最後に push_backscreen() で
      // 一括転送することでちらつきを防ぐ。
      // 背景のクリアは draw_vectormap() が行うので、ここでは何もしない。

      // ---- レイヤー 1: 地図背景 ----
      // フラッシュ内蔵のベクタ地図（OpenStreetMap 由来）を backscreen
      // へ直接描画する。 回転は latLonToXY
      // と同じ座標変換に吸収されるため、TRACKUP でも画面四隅まで埋まる。 SD
      // カードは不要で、位置が動いても I/O は発生しない。
      draw_vectormap(new_lat, new_long, scale, drawupward_direction);

      // ---- レイヤー 2: ポリゴン地図（滑走路・基準線などの注記）----
      // 内蔵（フラッシュ）と SD の mapdata.csv 由来の両方を描く。
      // 内蔵は SD
      // が抜けていても必ず表示されるので、飛行に必須の注記はそちらに置く。
      if (scale > SCALE_SMALL_GMAP) {
        draw_FlashMaps(new_lat, new_long, scale, drawupward_direction);
        draw_ExtraMaps(new_lat, new_long, scale, drawupward_direction);
      }

      gnss_loop(4); // 描画の合間に GNSS データを受信（取りこぼし防止）

      // ---- レイヤー 3: 飛行軌跡 ----
      draw_track(new_lat, new_long, scale, drawupward_direction);

      // ---- レイヤー 3.5: パイロンへの基準線・コース円 ----
      // マゼンタラインより先に描画して、重なったときは基準線が隠れるようにする。
      bool draw_pilon =
          (scale > SCALE_SMALL_GMAP &&
           check_within_latlon(0.6, 0.6, new_lat, pla_lat, new_long, pla_lon));
      if (draw_pilon) {
        draw_pilon_takeshima_line(new_lat, new_long, scale,
                                  drawupward_direction);
      }

      // ---- レイヤー 4: 目的地ライン ----
      // fix 取得前は自機位置が不明なため、誘導線は描画しない
      if (get_gnss_fix() && currentdestination != -1 &&
          currentdestination < destinations_count) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        if (destination_mode == DMODE_FLYINTO)
          draw_flyinto2(destlat, destlon, new_lat, new_long, scale,
                        drawupward_direction, 5);
        else if (destination_mode == DMODE_FLYAWAY)
          draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale,
                           drawupward_direction);
        else if (destination_mode == DMODE_AUTO10K) {
          // AUTO10K: 折り返しの状態に応じて AWAY/INTO を自動切り替え
          if (auto10k_status == AUTO10K_AWAY)
            draw_flyawayfrom(destlat, destlon, new_lat, new_long, scale,
                             drawupward_direction);
          else if (auto10k_status == AUTO10K_INTO)
            draw_flyinto2(destlat, destlon, new_lat, new_long, scale,
                          drawupward_direction, 5);
        }
      }
      gnss_loop(5); // 描画の合間に GNSS データを受信

      // ---- レイヤー 4.5: パイロン・PLA アイコン ----
      // マゼンタラインより後に描画してアイコンが隠れないようにする。
      if (draw_pilon) {
        draw_pilon_takeshima_marks(new_lat, new_long, scale,
                                   drawupward_direction);
      }

      // ---- レイヤー 4.6: ボート自身の位置（受信モードのミラー中のみ）----
      // 地図は機体を中心に描いているので、自分がどこにいるかは別に出さないと分からない。
      // 目的地ラインより後に描かないと、マゼンタの太線に隠れて見えなくなる。
      draw_own_position_marker(new_lat, new_long, scale, drawupward_direction);

      // ---- レイヤー 5: オーバーレイ（速度グラフ・スケールバーなど）----
      draw_degpersec(degpersecond);
      if (is_demo_active()) {
        draw_demo_biwako(); // 琵琶湖デモ表示（見た目や警告音などに慣れるための練習用）
      }
      if (getReplayMode()) {
        draw_replay_indicator(); // リプレイモード中であることを赤枠付きで通知
      }
      draw_km_distances(scale); // 画面左下のスケールバー

      // GNSS fix 状態と hAcc/gnssFixOK に応じて自機位置マーカーを切り替える
      // リプレイ・デモモードは実際の精度と無関係なので、常に通常の飛行機マーカーを表示する
      bool cur_fix_ok = get_gnss_fixok();
      float cur_hacc_m = get_gnss_hacc_mm() / 1000.0f; // mm → m
      if (!get_gnss_fix() && !is_demo_active() && !getReplayMode()) {
        draw_nofix_cross(); // fix なし（通常モードのみ）: グレーの ×
      } else if (!is_demo_active() && !getReplayMode() &&
                 (!cur_fix_ok || cur_hacc_m >= HACC_THRESHOLD_M)) {
        draw_hacc_circle(scale,
                         get_gnss_hacc_mm()); // gnssFixOK=false または hAcc
                                              // 不良: 青い不確かさ円
      } else {
        draw_triangle(
            new_truetrack,
            steer_angle); // 精度良好、またはリプレイ/デモ: 飛行機マーカー
      }

      // コンパス(N/E/S/W)は自機マーカー・進行方向縦線より後に描いて上のレイヤーにする。
      draw_compass(drawupward_direction, COLOR_BLACK);

      // ESKF のロール・ピッチ（リプレイ中を除き常時表示）。
      // ※ 各種ポップアップより先に描くこと。コース警告のボックス（y=175 または
      // 192 から
      //   高さ25）は ESKF 行と重なるため、後から描かれる側が上になる。
      //   状態通知のほうが優先度が高いので、ESKF は下のレイヤーに置く。
      // push の前に呼んでバックスクリーンへ合成する（TFT
      // へ直接描くとちらつくため）。
      draw_eskf_attitude();
      // ヨー角は自機アイコンの真下。TRACKUP ではロール・ピッチ行と近いので、
      // 必ず draw_eskf_attitude() の後に描いてこちらを上のレイヤーにする。
      draw_eskf_yaw();
#ifdef DEBUG_ESKF
      draw_eskf_debug(); // 比較用の詳細。上の表示位置は変えない
#endif

      // コース警告表示（警告発報から 10 秒間だけ表示する）
      if (millis() - last_course_warning_time < 10000 && millis() > 10000) {
        draw_course_warning(steer_angle);
      }

      draw_gs_track(); // ヘッダーに速度・コースなどのテキストを描画
      draw_map_footer();
      // GNSS の状態の箱（Scanning / Weak GNSS Signal）は push_backscreen() の最後で描く
      // （ほかの通知の箱と重ならない段を探すため。display_tft.cpp の draw_nomapdata）。
      gnss_loop(6);      // 描画の合間に GNSS データを受信
      push_backscreen(); // バックスクリーンを TFT に一括転送（VSI合成済み）
      draw_footer();     // フッターは TFT に直接描画（バックスクリーン外）
#ifndef RELEASE
      c0_is_redrawing = false;
#endif
      TIMING_END(ts_redraw, redraw);
    }

    // backscreen 非更新時、airdata が更新されたタイミングで VSI を TFT
    // へ直接転写 バリオ音量が 0 の場合は非表示 ★ ミラー中は転写しない。V/S
    // は機体の値でフレームが届いたときしか変わらず、
    //   受信機自身の気圧計の更新（約 3.7Hz）で同じ絵を送り直していただけだった。
    //   フレームを受けた直後の描き直しで push_backscreen() が合成して出す。
    if (!redraw_screen && !link_mirror_active() && vario_volume > 0 &&
        !vario_inhibit && airdata_updated) {
      draw_vsi();
      vsi_sprite.pushSprite(
          VSI_X, 50); // TFT座標: X=VSI_X, Y=50（backscreenオフセット）
    }
  } else {
    DEBUGW_PLN(20250510, "ERR screen mode");
  }
}

// バリオ用 Kalman filter への観測入力（気圧高度と GNSS
// 高度／垂直速度）。届いたものだけ。
static void kalman_feed_tick() {
  // 気圧高度が更新されたタイミングで Kalman 観測ステップを実行する
  // ※ update の実レートは約 3.7Hz。~40Hz ではない（2026-08-18
  // に実ログで確認）。
  //   MS5611 自体は ~40Hz でサンプルするが、airdata_update() が true を返すのは
  //   VSPEED_WINDOW_MS(250ms) のトリム平均ウィンドウが完了したときだけのため
  //   （プロペラ干渉 ~4.5Hz を除去する意図的な設計。airdata.cpp 参照）。
  //   したがって predict:update の比はおよそ 33:3.7 ≒ 9:1 になる。
  if (airdata_updated) {
    imu_kalman_baro_update(get_airdata_altitude());
    // 生 IMU ログにも気圧を残す（ESKF の検証と高度の突き合わせ用）
    imulog_push(IMULOG_ID_BARO, 0, IMULOG_ACC_NONE, get_airdata_pressure(),
                get_airdata_altitude(), get_airdata_temperature());
  }

  // GNSS高度による気圧基準補正（3Dフィックス有効時のみ、内部で1秒レート制限）
  // Vertical speed は変更せず、高度の長期ドリフトをゆっくり修正する。
  //
  // リプレイ中は実行しない。リプレイ中の GNSS 高度は CSV
  // 由来（再生した飛行場所の高度） であり、GNSS
  // 垂直速度に至っては更新されず直前の実測値のまま残る。これらを Kalman に
  // 入れると airdata_adjust_ground_alt()
  // が実際の気圧基準を書き換え、_gnss_kf_offset も
  // ずれてしまう。どちらもリプレイ終了後まで残り、通常モードに戻っても V/S が
  // 再生中のように動き続ける原因になる。
  // ★ 無線のミラー中も同じ理由で通さない。ミラー中の get_gnss_altitude() は
  //   受信した「機体」の高度なので、これを自機のカルマンに入れると
  //   気圧基準と _gnss_kf_offset
  //   が壊れ、リプレイと同様に受信をやめた後まで残る。
  if (!getReplayMode() && !link_mirror_active() && get_gnss_fixok() &&
      get_gnss_fixtype() >= 3) {
    imu_kalman_gnss_update((float)get_gnss_altitude(),
                           get_gnss_vacc_mm() / 1000.0f);
    // GNSS 垂直速度を KF 速度観測として追加（vAcc ゲート、sAcc で R 計算）
    // IMU の有無によらず動作し、加減速中や IMU なし時のバリオ精度を補完する。
    imu_kalman_gnss_vel_update(get_gnss_veld_mps(),
                               get_gnss_vacc_mm() / 1000.0f,
                               get_gnss_sacc_mmps() / 1000.0f);
  }
}

// 設定画面を出したまま離陸してしまった場合の保険。
static void setting_timeout_tick() {
  // ---- 設定画面の放置タイムアウト ----
  // 設定画面を出したまま離陸してしまった場合の保険。上の nav_alarm_tick() が
  // 画面によらず動くようになったので警報は鳴るが、地図が見えないままなのは変わらない。
  // ボタン操作が一定時間無ければ、通常の「Save &
  // Exit」と同じ経路で地図画面へ戻す。
  // 別れの音声が鳴るので、戻ったことにパイロットが気づける。
  if (screen_mode == MODE_SETTING &&
      millis() - last_user_input_ms > SETTING_IDLE_TIMEOUT_MS) {
    enqueueTask(
        createLogSdfTask("SETTING SCREEN TIMEOUT -> MAP (%lus idle)",
                         (unsigned long)(SETTING_IDLE_TIMEOUT_MS / 1000UL)));
    // ★ 値変更モードのまま時間切れになった場合は、**必ず CallbackExit
    // を通す。**
    //   長押しで抜ける通常経路（longPressCallback
    //   の「値変更モードから抜ける（確定）」）と 同じ処理。ここには Auto10km
    //   と目的地の組み合わせチェック
    //   （is10K_NotAllowed_Destination）や、無線の設定変更を知らせる警告が入っている。
    //   飛ばすと、設定画面で禁止の組み合わせを作りかけたまま地図へ戻ることになる。
    if (selectedLine != -1 && menu_settings[cursorLine].CallbackExit != nullptr)
      menu_settings[cursorLine].CallbackExit();
    selectedLine = -1;
    // ★
    // **別れの音声は鳴らさない**（farewell=false）。操作していないのに声が鳴ると
    //   何が起きたのか分からない（実機 2026-10-14
    //   の「負けんな」事件。src/button.cpp）。
    //   代わりに専用の短い降下ペアを鳴らす。**ボタン操作音（1046/1568Hz
    //   の単発）とは 別の音にすること** —
    //   あちらは「押した」の確認で、こちらは「勝手に閉じた」の通知。
    //   降下は「閉じた」に読める。優先度 2 は Auto10km 禁止の通知と同じ扱い。
    exit_setting(false); // 設定を保存して MODE_MAP へ
    enqueueTask(createPlayMultiToneTask(659, 100, 1, 2)); // ミ
    enqueueTask(createPlayMultiToneTask(523, 100, 1, 2)); // ド
    // 画面クリアは不要。地図の再描画（ヘッダ＋backscreen＋フッタ）が 240x320 を
    // 埋め尽くすため、通常の「Save & Exit」でもクリアしていない。
    redraw_screen = true;
  }
}

// 地図以外の画面を出したまま動き出したら地図へ戻す（2026-11-08〜）。
// ★ 上の放置タイムアウトは設定画面だけで、しかも 3 分かかる。IMU/ESKF・GNSS 詳細・VARIO・WIRELESS の
//   画面を出したまま発進すると、**地図が見えないまま飛ぶ**ことになっていた（警報は鳴るが場所が分からない）。
// ★ 速度は自機の GNSS（gnss_get_own_fix）で見る。デモ・リプレイの仮の速度では戻さない。
// ★ 受信モードと PC 接続の画面は対象外（settings.h の SCREEN_MOVING_EXIT_*）。
// ★ 各画面の「戻る」と同じ後始末をしてから、放置タイムアウトと同じ経路（設定を保存）で地図へ戻す。
//   音も放置タイムアウトと同じ降下ペア（勝手に閉じた、の合図）。
static void screen_moving_exit_tick() {
  static uint32_t s_since = 0;
  double lat, lon, gs = 0.0, trk;
  const bool applies = screen_mode != MODE_MAP && screen_mode != MODE_PCCONNECT &&
                       link_get_mode() != LINK_MODE_RX;
  const bool moving = applies && get_gnss_connection() &&
                      gnss_get_own_fix(lat, lon, gs, trk) && gs >= SCREEN_MOVING_EXIT_GS_MPS;
  if (!moving) { s_since = 0; return; }
  const uint32_t now = millis();
  if (s_since == 0) { s_since = now ? now : 1; return; }
  if (now - s_since < SCREEN_MOVING_EXIT_HOLD_MS) return;
  s_since = 0;

  const int from = screen_mode;
  switch (from) {
  case MODE_SETTING:
    // 値変更モードのままなら確定の処理を通す（放置タイムアウトと同じ理由）
    if (selectedLine != -1 && menu_settings[cursorLine].CallbackExit != nullptr)
      menu_settings[cursorLine].CallbackExit();
    break;
  case MODE_IMUDETAIL:
    // 案内の読み上げを地図の上に残さない
    // ★ IMU の案内は**案内を抜けたあとも止める**。最後の 1 文（guide_eskf_apply.wav。約 10 秒の
    //   「…APPLY してください」）は較正の画面に着いてから流すので、imu_guide_page はもう -1。
    //   ページで見ると止まらず、地図の上で「APPLY してください」と言い続けた（2026-11-10 まで）。
    //   排他グループで止めるので、案内の音声が鳴っていなければ何も起きない。
    enqueueTask(createStopWavTask(WAV_EXCL_ESKF_GUIDE));
    if (tas_guide_page >= 0) enqueueTask(createStopWavTask(WAV_EXCL_TAS_GUIDE));
    imu_guide_page = -1;
    tas_guide_page = -1;
    break;
  case MODE_GNSSDETAIL:
    gnss_getposition_mode();   // 衛星の一覧のために上げていた出力を戻す（長押しで戻るときと同じ）
    break;
  case MODE_WIRELESS:
    link_warn_setting_changed();   // CH / SF / Group を触っていたら知らせる（「戻る」と同じ）
    break;
  case MODE_CHSCAN:
    link_chscan_stop();            // 元の設定へ戻す
    break;
  default:
    break;
  }
  selectedLine = -1;
  exit_setting(false);   // 設定を保存して MODE_MAP へ（別れの音声は鳴らさない）
  enqueueTask(createPlayMultiToneTask(659, 100, 1, 2)); // ミ
  enqueueTask(createPlayMultiToneTask(523, 100, 1, 2)); // ド
  enqueueTask(createLogSdfTask("SCREEN %d -> MAP (moving %.1fm/s for %lus)", from, gs,
                               (unsigned long)(SCREEN_MOVING_EXIT_HOLD_MS / 1000UL)));
  redraw_screen = true;
}

// ===== Web Serial コマンド受信・処理（Core0） =====
static char serial_cmd_buf[128];
static size_t serial_cmd_len = 0;
static int prev_screen_mode = MODE_MAP;
static uint32_t last_serial_rx_ms = 0;

static void handle_serial_command(const char *raw_cmd) {
  last_serial_rx_ms = millis();
  // 前後の空白をトリム
  while (*raw_cmd == ' ')
    raw_cmd++;
  if (*raw_cmd == '\0')
    return;

  char cmd[128];
  strlcpy(cmd, raw_cmd, sizeof(cmd));
  int clen = strlen(cmd);
  while (clen > 0 && (cmd[clen - 1] == ' ' || cmd[clen - 1] == '\r' ||
                      cmd[clen - 1] == '\n')) {
    cmd[--clen] = '\0';
  }

  if (strcmp(cmd, "PONS:CONNECT") == 0) {
    webserial_connected = true;
    webserial_cancel_requested = false;
    // ★ PC 接続のまま CONNECT をもう一度受けたとき（ビューアのページを読み直した等）に、戻り先を
    //   MODE_PCCONNECT で上書きしない（上書きすると抜けたときに元の画面ではなく地図へ戻る）。
    if (screen_mode != MODE_PCCONNECT) prev_screen_mode = screen_mode;
    screen_mode = MODE_PCCONNECT;
    redraw_screen = true;
    enqueueTask(createLogSdTask("PC CONNECTED (WEB SERIAL ACTIVE)"));
    Serial.printf("PONS:READY,%s,%s,%lu\r\n", VERSION_TEXT, BUILDVERSION,
                  (unsigned long)BUILDDATE);
  } else if (strcmp(cmd, "PONS:PING") == 0) {
    // GET タスク待機中または実行中はデータストリームに PONG
    // が混ざるのを防ぐため応答を抑止
    if (!isTaskInQueue(TASK_SERIAL_GET) && !isTaskRunning(TASK_SERIAL_GET)) {
      Serial.println("PONS:PONG");
    }
  } else if (strcmp(cmd, "PONS:LIST") == 0) {
    if (!webserial_connected) {
      Serial.println("PONS:ERROR,NOT_CONNECTED");
      return;
    }
    // 多重実行防止
    if (isTaskInQueue(TASK_SERIAL_LIST) || isTaskRunning(TASK_SERIAL_LIST)) {
      return;
    }
    enqueueTask(createSerialListTask());
  } else if (strncmp(cmd, "PONS:GET,", 9) == 0) {
    if (!webserial_connected) {
      Serial.println("PONS:ERROR,NOT_CONNECTED");
      return;
    }
    // 転送中なら BUSY を返す
    if (isTaskInQueue(TASK_SERIAL_GET) || isTaskRunning(TASK_SERIAL_GET)) {
      Serial.println("PONS:ERROR,BUSY");
      return;
    }
    const char *filename = cmd + 9;
    while (*filename == ' ')
      filename++;
    if (*filename == '\0') {
      Serial.println("PONS:ERROR,INVALID_FILENAME");
      return;
    }
    enqueueTask(createSerialGetTask(filename));
  } else if (strcmp(cmd, "PONS:CANCEL") == 0) {
    webserial_cancel_requested = true;
    // 転送タスク実行中でなければ即座に応答（転送中なら Core1
    // がファイルを閉じた直後に出す）
    if (!isTaskInQueue(TASK_SERIAL_GET) && !isTaskRunning(TASK_SERIAL_GET)) {
      Serial.println("PONS:CANCEL_OK");
    }
  } else if (strcmp(cmd, "PONS:DISCONNECT") == 0) {
    webserial_connected = false;
    webserial_cancel_requested = true;
    Serial.println("PONS:BYE");
    enqueueTask(createLogSdTask("PC DISCONNECTED (BYE) -> RESUME MAP"));
    if (screen_mode == MODE_PCCONNECT) {
      screen_mode =
          (prev_screen_mode != MODE_PCCONNECT) ? prev_screen_mode : MODE_MAP;
      redraw_screen = true;
    }
  } else if (strncmp(cmd, "PONS:DBD,", 9) == 0) {
    // GNSS の航法データベースの机の上の試験（SAVE / COLD / COLDONLY / HOT / STATUS。gnss_dbd.cpp の
    // gnss_dbd_command）。PONS:CONNECT は要らない（Arduino IDE のシリアルモニタから打てるように）。
    gnss_dbd_command(cmd + 9);
  } else {
#ifndef RELEASE
    // PONS コマンド以外は GNSS へ中継（デバッグ用）
    gnss_dbd_finish_tx(); // 航法データベースのフレームを書きかけていたら先に書き切る
    for (size_t i = 0; cmd[i] != '\0'; i++) {
      GNSS_SERIAL.write(cmd[i]);
    }
    GNSS_SERIAL.write('\r');
    GNSS_SERIAL.write('\n');
#endif
  }
}

static void serial_command_service() {
  // USB ケーブル抜去監視: 給電が断たれたら通常画面へ安全自動復帰
  if ((webserial_connected || screen_mode == MODE_PCCONNECT) &&
      !usb_power_present()) {
    webserial_connected = false;
    webserial_cancel_requested = true;
    enqueueTask(
        createLogSdTask("USB DISCONNECTED (CABLE REMOVED) -> RESUME MAP"));
    if (screen_mode == MODE_PCCONNECT) {
      screen_mode =
          (prev_screen_mode != MODE_PCCONNECT) ? prev_screen_mode : MODE_MAP;
      redraw_screen = true;
    }
    return;
  }

  // 転送タスク実行中・待機中は無通信タイマーを更新し続け、転送中の誤切断を防止
  if (isTaskInQueue(TASK_SERIAL_LIST) || isTaskRunning(TASK_SERIAL_LIST) ||
      isTaskInQueue(TASK_SERIAL_GET) || isTaskRunning(TASK_SERIAL_GET)) {
    last_serial_rx_ms = millis();
  }

  // 無通信フェイルセーフ: PC接続モードのまま 60
  // 秒間無通信なら自動で通常航法へ復帰
  if (screen_mode == MODE_PCCONNECT &&
      (millis() - last_serial_rx_ms >= 60000UL)) {
    webserial_connected = false;
    webserial_cancel_requested = true;
    enqueueTask(createLogSdTask("PC TIMEOUT (60s IDLE) -> RESUME MAP"));
    screen_mode =
        (prev_screen_mode != MODE_PCCONNECT) ? prev_screen_mode : MODE_MAP;
    redraw_screen = true;
    return;
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (serial_cmd_len > 0) {
        serial_cmd_buf[serial_cmd_len] = '\0';
        handle_serial_command(serial_cmd_buf);
        serial_cmd_len = 0;
      }
    } else {
      if (serial_cmd_len < sizeof(serial_cmd_buf) - 1) {
        serial_cmd_buf[serial_cmd_len++] = c;
      } else {
        serial_cmd_len = 0; // 溢れたらリセット
      }
    }
  }
}

// Core0 のメインループ。以下の処理を毎ループ実行する:
//   1. ボタン状態の読み取り
//   2. GNSS データの受信・解析（gnss_loop は描画中にも分散して呼ぶ）
//   3. 画面モードに応じた描画（地図 / 設定 / GNSS詳細 など）
// gnss_loop(id) の id はデバッグ用の呼び出し箇所識別子。
void loop() {
  // 無線モジュールとの UART。1Hz・数十バイトなので負荷は無視できる。
  // 無線モジュールが繋がっていなくても、link_module_alive() が false
  // になるだけで無害。
  link_loop();
  // ★
  // チャンネル調査は一覧の画面にいる間だけ。ほかへ移ったら止めて元の設定へ戻す
  //   （調査中は送信も受信も止まっているので、画面を離れたまま走らせない）。
  //   ボタンで戻るときは longPressCallback
  //   が先に止めるので、普通はここを通らない。
  if (link_chscan_running() && screen_mode != MODE_CHSCAN)
    link_chscan_stop();
  // switch handling
  sw_push.read();
  // 地図画面以外（設定画面・リプレイ選択画面・各詳細画面）を開いている間は
  // リプレイを一時停止する。戻ってきたら続きから再生される。
  replay_set_paused(screen_mode != MODE_MAP);
  // 生 IMU ログを止めるのはリプレイ中と PC Web Serial 接続中
  // （Euler ログと同じ理由: 再生日時のファイルを実センサ値で汚さない / SD
  // カード読み出し専用保護）。
  //
  // ※ GNSS 日時の有効性では止めないこと。
  //   以前は日時未確定でも止めていたが、それだと屋内など測位できない場所で
  //   IMU の生ログが一切取れず（wrote=0）、ベンチでの検証ができなかった。
  //   日時が無いときは下でファイル名を imuraw/nofix.bin にフォールバックする。
  //   レコードはすべてホスト時刻 t_us を持つので、日付が無くても解析はできる。
  imulog_set_paused(getReplayMode() || webserial_connected);
  gnss_loop(0);   // ループ先頭で GNSS データを受信
  loop_userled(); // USERLED フラッシュ制御（0衛星・SDエラー時）

  gnss_conn_tick();

  // 大気データ更新（非ブロッキングのステートマシン。毎ループ呼ぶことで約50Hzで計測）
  airdata_updated = airdata_update();

  // IMU 更新（H_INT フラグ確認 → データ読み出し → Kalman
  // predict。毎ループ呼んでよい）
  imu_update();

  flightlog_tick();

  kalman_feed_tick();

  // バリオメーター音更新（内部で 100ms レート制限。毎ループ呼んでよい）
  // ★ バリオも警報と同じ条件で黙らせる（机で充電しながら置くと、傾いた姿勢で
  //   延々と鳴る。画面の `IN MUTE DUE TO CHARGING + OFF MOUNT`
  //   がその説明になる）。
  update_vario(warn_muted_off_mount());

  bank_alarm_tick();

  power_tick(); // 電源経路の異常・高温充電・過放電監視。電池警告より先に判定する
  if (power_overdischarge_usb_lock_due()) {
    enqueueTask(createLogSdTask(
        "BATT LATCH: USB connected while latched -> entering lock screen"));
    run_overdischarge_warning_loop(true);
  }

  backlight_tick(); // v7.1 のバックライト（輝度スイッチ・照度）

  batt_alarm_tick();

  ground_check_tick();

  // 推定 TAS: 地上と飛行中の切り替え（大きい枠の数字の横線。判定は tas.cpp）
  tas_tick();

  // 本番前チェック。上の地上チェックの発報状態（ground_*_warn_active）を読むので、その後に置く。
  readiness_tick();

  eskf_apply_tick();

  // ---- 直進中のロール自動トリムが入ったらログに残す ----
  // 事後検証できるよう、1 回の補正量と累積量を記録する。
  {
    float applied, total;
    if (attitude_take_roll_trim_event(applied, total)) {
      enqueueTask(createLogSdfTask("ROLL TRIM %+.2f deg (total %+.2f)", applied,
                                   total));
      // 次回起動でも効くよう保存する。設定画面を開かない運用（パイロットに預けたまま）
      // でも残す必要があるため、ここで保存する。
      // 補正は最短でも ROLL_TRIM_WINDOW_S × ROLL_TRIM_CONFIRM_WINDOWS
      // 間隔でしか 起きないので、SD 書き込みの頻度は問題にならない。
      enqueueTask(createSaveSettingTask());
    }
  }

  // USB シリアルからの Web Serial コマンド受信処理（非ブロッキング）
  serial_command_service();

  // GNSS 更新や BMP ロードがなくても、一定間隔で強制再描画する
  // （時刻表示など時間経過で変わる表示の更新保証）
  //
  // VARIO 詳細はセンサー値を読むための画面で、他に再描画トリガーが無いため、
  // この間隔がそのまま表示の更新レートになる。地図画面と同じ 1050ms だと
  // 約 0.95Hz しか出ないので、この画面だけ短い間隔を使う。
  {
    // IMU/ESKF の入口の案内は動かない画像なので、200ms ごとに 150KB を送り直さない
    // （上に重なった表示を消すための保険として、地図と同じ間隔だけ描き直す）。
    // 推定 TAS の案内も、画像（0 枚目）は同じ。曲線の画面（1）は本体を傾けると点が動くので短い間隔。
    unsigned long fresh_interval =
        (screen_mode == MODE_VARIODETAIL ||
         (screen_mode == MODE_IMUDETAIL && imu_guide_page < 0 && tas_guide_page != 0))
            ? SCREEN_FRESH_INTERVAL_DETAIL
            : SCREEN_FRESH_INTERVAL;
    // ★
    // ミラー中の地図だけは、受信中のフレームに重ならない時間に限る（link_draw_window_ok）。
    //   フレームを 1 つ落とすと、1.05
    //   秒後の強制再描画がちょうど次のフレームの受信に重なる。
    //   間隔を延ばしても重なる位相は残るので、間隔ではなく時刻（窓）で避ける。
    if (millis() - screen_update_time > fresh_interval &&
        (screen_mode != MODE_MAP || link_draw_window_ok())) {
      redraw_screen = true;
    }
  }

  periodic_log_tick();

  // ---- ナビゲーションと警報 ----
  // ★ **画面モードによらず必ずここで走らせる。**
  //   以前はこの下の MODE_MAP
  //   の分岐の中に置いていたため、設定画面や各詳細画面を 開いている間は
  //   Auto10km の折返し判定・コース逸脱警報・旋回角速度の更新が
  //   すべて止まっていた。とくに INTO→AWAY（目的地から 1.5km 以内）は「そのとき
  //   近くにいるか」で判定するので、地図画面を離れている間に通り過ぎると
  //   二度と成立せず、復路のまま戻らなくなる。
  //   電池警告を loop() の最上位へ移したのと同じ理由（CLAUDE.md 参照）。
  bool new_gnss_info = gnss_new_location_arrived();
  nav_alarm_tick(new_gnss_info);

  setting_timeout_tick();
  screen_moving_exit_tick();   // 地図以外の画面を出したまま動き出したら地図へ

  draw_current_screen(new_gnss_info);

  if (redraw_screen) {
    // 更新終了
    redraw_screen = false;
    screen_update_time = millis();
  }

#ifndef RELEASE
  // 30秒ごとに各処理の最大・平均・回数を Serial 出力する（IMU
  // の読み出しを飢えさせない調査用） 判断基準: max が 10000us (10ms) 未満 →
  // その処理の間も IMU を読み出せる
  {
    static uint32_t _last_timing_report = 0;
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
      Serial.print("[TIME] C1_overlap_count=");
      Serial.println(_c1_overlap_count);
      Serial.println("=========================");
    }
  }
#endif
}

//===============MAIN LOOP CORE1=================
// Core1
// のメインループ。タスクキューからタスクを取り出して順番に実行するディスパッチャー。
// SD アクセス・音声再生はすべて Core1 で行い、Core0 の描画をブロックしない。
// loop_sound() は毎ループ呼んでPWM音声の継続再生を維持する。
void loop1() {
  // core1_separate_stack=true の場合、setup1() とは別スタックで loop1()
  // が動く。 そのため loop1() 最初の1回だけ SP をキャプチャしてベース値にする。
  if (_core1_base_sp == 0) {
    uint32_t _sp_;
    asm volatile("mov %0, sp" : "=r"(_sp_));
    _core1_base_sp = _sp_;
    DEBUG_P(20260312, "[C1 loop1 base SP]=0x");
    DEBUG_PNLN(20260312, _core1_base_sp, HEX);
  }
  loop_sound(); // 音声の継続再生処理（WAV 送出など）
  loop_tone();  // トーンバッファの非ブロッキング再生管理
  if (dequeueTask(&currentTask)) {
    switch (currentTask.type) {
    case TASK_SAVE_SETTINGS:
      saveSettings();
      break;
    case TASK_PLAY_WAV:
      if (strcmp(currentTask.playWavArgs.wavfilename,
                 BATT_OVERDISCHARGE_WAV_FILE) == 0 &&
          !power_overdischarge_latched()) {
        break;
      }
      startPlayWav(currentTask.playWavArgs.wavfilename,
                   currentTask.playWavArgs.priority,
                   currentTask.playWavArgs.min_volume,
                   currentTask.playWavArgs.excl_group);
      break;
    case TASK_STOP_WAV:
      stop_wav_excl_group_core1(currentTask.playWavArgs.excl_group);
      break;
    case TASK_PLAY_MULTITONE:
      if (currentTask.playMultiToneArgs.freq == BATT_OVERDISCHARGE_TONE_FREQ &&
          currentTask.playMultiToneArgs.solo_play &&
          currentTask.playMultiToneArgs.priority ==
              BATT_OVERDISCHARGE_TONE_PRIORITY &&
          !power_overdischarge_latched()) {
        break;
      }
      playTone(currentTask.playMultiToneArgs.freq,
               currentTask.playMultiToneArgs.duration,
               currentTask.playMultiToneArgs.counter,
               currentTask.playMultiToneArgs.priority,
               currentTask.playMultiToneArgs.min_volume,
               currentTask.playMultiToneArgs.solo_play);
      break;
    case TASK_INIT_SD:
      // ★ 実行中の再マウントで操作中の設定値を settings.txt で上書きしないよう
      // false にする
      //   （まだ一度も settings.txt を読めていない起動失敗時だけは setup_sd
      //   側が自動で読む）。
      setup_sd(1, false);
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
      saveCSV(currentTask);
      break;
    case TASK_SAVE_RXCSV:
      saveRxCSV(currentTask);
      break;
    case TASK_FLUSH_IMULOG:
      // 生 IMU ログの二重バッファ片側を SD へ書き出す（約 4KB / 0.45 秒分）。
      // 書き出し後にバッファを解放するので、この処理が滞ると Core0
      // 側で記録が捨てられる。
      imulog_write_buffer(
          currentTask.imuLogArgs.bufidx, currentTask.imuLogArgs.filename,
          currentTask.imuLogArgs.year, currentTask.imuLogArgs.month,
          currentTask.imuLogArgs.day, currentTask.imuLogArgs.hour,
          currentTask.imuLogArgs.minute, currentTask.imuLogArgs.second);
      break;
    case TASK_LATCH_BATT_OVERDISCHARGE:
      power_latch_commit_core1();
      break;
    case TASK_SERIAL_LIST:
      serial_list_csv_files();
      break;
    case TASK_SAVE_RTCBK:
      rtc_backup_save_sd(currentTask.rtcBkArgs.hours, currentTask.rtcBkArgs.utc);
      break;
    case TASK_SAVE_DBD:
      gnss_dbd_save_sd();
      break;
    case TASK_LOAD_DBD:
      gnss_dbd_load_sd();
      break;
    case TASK_SERIAL_GET:
      serial_send_csv_file(currentTask.serialGetArgs.filename);
      break;
    }
    clearCurrentTask(); // mutex 内で TASK_NONE にする（Core0 の isTaskRunning()
                        // と整合させるため）
  } else {
    // No tasks, optionally sleep or yield
    delay(10);
  }
}
extern volatile int max_page; // Core1 が更新する（実体は mysd.cpp・volatile）

//==========BUTTON==========
// マップスケール（ズームレベル）を次の段階へ切り替える。
// 地図画面でのダブルクリックと、設定画面の Map scale 項目の両方から呼ばれる。
void next_scaleindex() {
  scaleindex = (scaleindex + 1) % (sizeof(scalelist) / sizeof(scalelist[0]));
  scale = scalelist[scaleindex];
}

// 指定の縮尺へ切り替える（範囲外は無視）。PONS Link の受信側が
// **送信機の縮尺に合わせる**ために使う（link.cpp）。
// ★ scaleindex と scale は必ず対で更新すること（上と同じ）。
void set_scaleindex(int idx) {
  if (idx < 0 || idx >= (int)(sizeof(scalelist) / sizeof(scalelist[0])))
    return;
  scaleindex = idx;
  scale = scalelist[scaleindex];
}

// 画面横幅 240px が何 km に相当するかを返す（設定画面のスケール表示用）。
float scale_screen_km(int index) {
  if (index < 0 || index >= (int)(sizeof(scalelist) / sizeof(scalelist[0])))
    return 0;
  return (float)(BACKSCREEN_SIZE / scalelist[index]);
}

// 短押しコールバック: 画面モードごとに動作が変わる。
//   MAP モード:
//   何もしない（誤操作防止のためスケール変更はダブルクリックに変更） SETTING
//   モード:  カーソル移動 or 値のトグル変更 SDDETAIL:       次のページを SD
//   から読み込む MAPLIST/GNSSDETAIL: 次のページへ
// 操作音は Button
// クラスではなくここで鳴らす。何も起きない画面では鳴らさないため。
void shortPressCallback() {
  backlight_notify_activity();
  last_user_input_ms = millis();
  redraw_screen = true;
  DEBUG_PLN(20240801, "short press");

  // MAP モードでは短押しは何もしない（スケール変更はダブルクリックへ移した）。
  // 充電ポップアップが表示されている場合は、短押しで 15
  // 秒間スヌーズ（一時非表示）にして地図を確認できるようにする。
  if (screen_mode == MODE_MAP) {
    if (power_charge_popup_active()) {
      power_charge_popup_snooze();
      redraw_screen = true;
    }
    return;
  }

  // PC接続モード中: プッシュスイッチで手動切断して通常画面へ復帰
  if (screen_mode == MODE_PCCONNECT) {
    webserial_connected = false;
    webserial_cancel_requested = true;
    Serial.println("PONS:BYE");
    enqueueTask(createLogSdTask("PC DISCONNECTED (BUTTON) -> RESUME MAP"));
    screen_mode =
        (prev_screen_mode != MODE_PCCONNECT) ? prev_screen_mode : MODE_MAP;
    redraw_screen = true;
    return;
  }

  enqueueTask(createPlayMultiToneTask(1046, 80, 1)); // 短押し音

  // チャンネル調査の一覧: 調べている途中なら止める、終わっていればもう一度。
  // 断られたら（モジュール無応答）低い音。
  if (screen_mode == MODE_CHSCAN) {
    if (link_chscan_running())
      link_chscan_stop();
    else if (!link_chscan_start())
      enqueueTask(createPlayMultiToneTask(440, 200, 1));
    return;
  }

  if (screen_mode == MODE_SETTING) {
    if (selectedLine == -1) { // 値変更モードでない → カーソルを次の行へ
      cursorLine = (cursorLine + 1) % setting_size;
    } else { // 値変更モード中 → 現在行の値をトグル
      menu_settings[selectedLine].CallbackToggle();
    }
  } else if (screen_mode == MODE_SDDETAIL) {
    detail_page++;
    loading_sddetail = true;
    if (max_page <= 0)
      enqueueTask(createBrowseSDTask(0));
    else
      enqueueTask(
          createBrowseSDTask(detail_page % (max_page + 1))); // ページをループ
  } else if (screen_mode == MODE_REPLAYSELECT) {
    // カーソルを次の項目へ（末尾まで行ったら先頭に戻る）
    int total = replay_menu_total_items();
    replay_cursor = (replay_cursor + 1) % total;
    int newpage = replay_menu_page_of(replay_cursor);
    if (newpage != replay_list_page) {
      // ページを跨いだので、そのページ分のファイル一覧を Core1
      // に取り直してもらう
      replay_list_page = newpage;
      loading_replaylist = true;
      enqueueTask(
          createBrowseReplayTask(replay_menu_file_start_for_page(newpage)));
    }
  } else if (screen_mode == MODE_WIRELESS) {
    // 無線画面:
    // 短押しはカーソル移動のみ。変更は長押し（設定画面と同じ操作感）。
    wireless_cursor = (wireless_cursor + 1) % WIRELESS_MENU_COUNT;
    redraw_screen = true;
  } else if (screen_mode == MODE_IMUDETAIL) {
    // IMU / ESKF 画面: 短押しはカーソル移動のみ。実行は長押し。
    // 較正は「今の姿勢を何度として記録するか」を選んでから行うので、
    // 誤操作で意図しない値が焼き付かないよう 2 段階にしてある。
    // 入口の案内を出している間は、押すと次の 1 枚へ（imu_guide_next）。
    if (imu_guide_page >= 0)
      imu_guide_next();
    else if (tas_guide_page >= 0)
      tas_guide_next();   // 推定 TAS の案内も同じ（押すと次の画面へ）
    else if (detail_page % IMU_PAGE_COUNT == 0)
      imu_cursor = (imu_cursor + 1) % IMU_MENU_COUNT;
    else
      imu2_cursor = (imu2_cursor + 1) % IMU2_MENU_COUNT;
    redraw_screen = true;
  } else if (screen_mode == MODE_MAPLIST || screen_mode == MODE_GNSSDETAIL ||
             screen_mode == MODE_VARIODETAIL) {
    detail_page++;
  }
}

// IMU / ESKF 画面のカーソル位置に応じた実行処理（長押しから呼ばれる）。
// 設定画面と同じく「短押し=カーソル移動 / 長押し=実行」の操作感に揃えてある。
// この画面を出るのは Exit 項目からのみ（長押しで戻る動作は持たせていない）。
// Exit はページ1にも 2/2 にもあり、どちらも imu_exit() を通す（動きを食い違わせない）。
static void imu_exit() {
  // 案内の最後の 1 文（guide_eskf_apply.wav）が鳴っていたら止める（地図の上で「APPLY してください」と言わない。
  // screen_moving_exit_tick() と同じ理由）
  enqueueTask(createStopWavTask(WAV_EXCL_ESKF_GUIDE));
  enqueueTask(createSaveSettingTask());
  screen_mode = MODE_MAP;
  enqueueTask(createPlayMultiToneTask(1568, 80, 1));
}

// ---- IMU/ESKF 画面の入口の案内 ----
// 較正の画面（ページ1）へ行く前に、画像で 1 項目ずつ見せる（SET PITCH / SET ROLL / 本体の向き）。
// 画像の原本は tools/imu_guide/、焼き込みは tools/gen_guide_flash.py。
// 1 枚ごとにその項目を読み上げる。音声は同じ排他グループ（WAV_EXCL_ESKF_GUIDE）なので、
// ページを送ると次の音声が前の音声を打ち切る（pending にも残らない。docs/pons_sound.md §3）。
// ★ 汎用の「再生を止める」は無い。最後の 1 文（guide_eskf_apply.wav）を較正の画面に
//   着いたときに流すので、3 枚目の音声もそこで打ち切られ、案内の音が後に残ることは無い。
// ★ 画像と音声は順番で 1 対 1。並べ替えるときは画像も一緒に並べ替える。
static const char *const IMU_GUIDE_WAV[] = {
  "wav/guide_eskf_pitch.wav",    // 1/3 SET PITCH   「セットピッチが現在の地面の傾斜角と…」
  "wav/guide_eskf_roll.wav",     // 2/3 SET ROLL    「セットロールが現在の機体の角度と…」
  "wav/guide_eskf_heading.wav",  // 3/3 本体の向き  「本体の向きが進行方向に向いていて…」
};
static_assert(sizeof(IMU_GUIDE_WAV) / sizeof(IMU_GUIDE_WAV[0]) == IMU_GUIDE_PAGES,
              "IMU_GUIDE_WAV[] must have one voice per guide image (IMU_GUIDE_PAGES)");

// 優先度 3 = コース警告と同等。誤った較正は警告機能そのものを壊すため（従来の案内と同じ）。
static void imu_guide_say(const char *wav) {
  enqueueTask(createPlayWavTask(wav, 3, 0, WAV_EXCL_ESKF_GUIDE));
}

// 設定画面の「IMU / ESKF detail >」から入ったときに呼ぶ（src/button.cpp）。
void imu_guide_start() {
  imu_guide_page = 0;
  tas_guide_page = -1;
  imu_guide_say(IMU_GUIDE_WAV[0]);
  redraw_screen = true;
}

// 短押し・長押しのどちらでも次の 1 枚へ（設定画面の癖で長押ししても止まって見えないように）。
// ★ 案内を飛ばす操作は付けない。「較正の画面へ行く前に必ず一度通る」ための画面。
static void imu_guide_next() {
  imu_guide_page++;
  if (imu_guide_page < IMU_GUIDE_PAGES) {
    imu_guide_say(IMU_GUIDE_WAV[imu_guide_page]);
  } else {
    imu_guide_page = -1;
    detail_page = 0;
    imu_cursor = 0;
    // 案内は全画面の画像。ページ1はヘッダを pushSprite(0, -10) で y=0〜39 に置くなど、
    // 塗らない所があるので、先に白で消しておく（消さないと案内の切れ端が残る）。
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
    imu_guide_say("wav/guide_eskf_apply.wav");   // 「…確認してから、校正を実行。APPLYしてください。」
  }
  redraw_screen = true;
}

// ---- 推定 TAS の案内（docs/pons_tas.md §4.4）----
// 大きい枠を TAS にしたときだけ通す 2 画面。1 画面ごとに読み上げる（画面と音声は順番で 1 対 1）。
// 画像の原本は tools/tas_guide/、焼き込みは tools/gen_guide_flash.py。曲線の画面はファームが描く
// （display_pages.cpp の draw_tas_guide。本体を傾けると点と数字が動く）。
// ★ 起動のたびには出さない。IMU/ESKF 画面の入口の案内にも足さない（2026-11-07 決定）。
// ★ 抜けたら読み上げを止める（TASK_STOP_WAV）。IMU の案内のように最後に流す 1 文が無いので、
//   止めないと曲線の画面の説明がページ1に戻ってからも鳴り続ける。
#define TAS_GUIDE_SCREENS 2
static const char *const TAS_GUIDE_WAV[TAS_GUIDE_SCREENS] = {
  "wav/guide_tas_1.wav",       // 1/2 説明の画像  「青い箱は、ピッチから計算した、推定の対気速度です…地上では数字に横線が入り…」
  "wav/guide_tas_curve.wav",   // 2/2 曲線の画面  「ピッチと対気速度の関係です…」
};
static_assert(TAS_GUIDE_PAGES + 1 == TAS_GUIDE_SCREENS,
              "TAS の案内は画像 TAS_GUIDE_PAGES 枚＋曲線の画面 1 つ（TAS_GUIDE_WAV[] と数を合わせる）");

// 優先度 2。誤った較正のように警告機能を壊す話ではないので、IMU の案内（3）より下げる。
static void tas_guide_say(const char *wav) {
  enqueueTask(createPlayWavTask(wav, 2, 0, WAV_EXCL_TAS_GUIDE));
}

static void tas_guide_start() {
  tas_guide_page = 0;
  tas_guide_say(TAS_GUIDE_WAV[0]);
  redraw_screen = true;
}

// 短押し・長押しのどちらでも次の画面へ（IMU の案内と同じ操作）。
static void tas_guide_next() {
  tas_guide_page++;
  if (tas_guide_page < TAS_GUIDE_SCREENS) {
    tas_guide_say(TAS_GUIDE_WAV[tas_guide_page]);
  } else {
    tas_guide_page = -1;
    enqueueTask(createStopWavTask(WAV_EXCL_TAS_GUIDE));
    // 案内は全画面。ページ1は塗らない所があるので先に白で消す（imu_guide_next と同じ理由）。
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
    enqueueTask(createPlayMultiToneTask(1568, 80, 1));
  }
  redraw_screen = true;
}

static void imu_execute() {
  redraw_screen = true;

  if (detail_page % IMU_PAGE_COUNT == 1) { // ページ2/2: 見るだけ（行は戻る / Exit）
    switch (imu2_cursor) {
    case IMU2_MENU_BACK:
      detail_page = 0;
      imu2_cursor = 0;
      enqueueTask(createPlayMultiToneTask(1568, 80, 1));
      break;
    case IMU2_MENU_EXIT:
      imu2_cursor = 0;
      imu_exit();
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
    // ★
    // 地上でのみ通す。飛行中に実行すると傾いた姿勢を基準として焼き付けてしまい、
    //   バンク角警告が機能しなくなる。
    if (eskf_calib_allowed()) {
      // すぐには実行しない。スイッチを押す力でデバイスが傾くので、指を離して
      // 落ち着いてから attitude 側が実行する（ESKF_APPLY_DELAY_US）。
      attitude_request_calibrate(attitude_get_pitch_target(),
                                 attitude_get_roll_target());
      enqueueTask(createPlayMultiToneTask(1568, 60, 1)); // 受け付けた合図
    } else {
      enqueueTask(createPlayMultiToneTask(440, 200, 1)); // 実行不可（低い音）
    }
    break;

  case IMU_MENU_RPYFUNC:
    attitude_set_rpy_enabled(!attitude_get_rpy_enabled());
    enqueueTask(createSaveSettingTask());
    enqueueTask(createPlayMultiToneTask(1568, 60, 1));
    break;

  case IMU_MENU_AUTOROLL: // 直進中のロール自動トリム（2026-11-05 に 3/3 から移した）
    attitude_set_roll_trim_enabled(!attitude_get_roll_trim_enabled());
    enqueueTask(createSaveSettingTask());
    enqueueTask(createPlayMultiToneTask(1568, 60, 1));
    break;

  case IMU_MENU_WIND: // 風推定（同上）
    attitude_set_wind_enabled(!attitude_get_wind_enabled());
    enqueueTask(createSaveSettingTask());
    enqueueTask(createPlayMultiToneTask(1568, 60, 1));
    break;

  case IMU_MENU_HEADER: { // ヘッダー右の大きい枠: 平均ピッチ ⇔ 推定 TAS（docs/pons_tas.md §4.1）
    const bool to_tas = !tas_setting_get();
    tas_setting_set(to_tas);
    enqueueTask(createSaveSettingTask());
    // TAS にしたときだけ仕組みの案内を通す。Pavg に戻すときは合図だけ。
    if (to_tas) tas_guide_start();
    else        enqueueTask(createPlayMultiToneTask(1568, 60, 1));
    break;
  }

  case IMU_MENU_NEXTPAGE:
    detail_page = 1;
    imu2_cursor = 0;
    enqueueTask(createPlayMultiToneTask(1568, 80, 1));
    break;

  case IMU_MENU_EXIT:
    imu_exit();
    break;
  }
}

// ダブルクリックコールバック:
// 地図画面でのみスケール（ズームレベル）を切り替える。
// 短押しは毎回発火するため、ここでは短押しと重複しない処理だけを行う。
// 操作音は shortPressCallback()
// と同じ理由でここで鳴らす（効かない画面では鳴らさない）。
void doublePressCallback() {
  power_charge_popup_snooze();
  backlight_notify_activity();
  last_user_input_ms = millis();
  DEBUG_PLN(20240801, "double press");

  // MAP
  // モード以外（設定画面や各詳細画面）では短押しがページ送り／カーソル移動を、
  // 長押しが実行を担っているのでダブルクリックには意味がない。音も鳴らさずに抜ける。
  if (screen_mode != MODE_MAP)
    return;

  enqueueTask(createPlayMultiToneTask(
      1568, 80, 1)); // ダブルクリック音（短押しより高い音）

  redraw_screen = true;
  next_scaleindex(); // MAP モード: スケールを次のレベルに切り替える
}

// 設定画面に戻る（リプレイ選択画面から抜けるとき用）。
// 汎用の「設定画面へ戻る」パスを通らないので、カーソル状態は自分でリセットする。
static void backToSettingFromReplay() {
  screen_mode = MODE_SETTING;
  selectedLine = -1; // 値変更モードを解除（REPLAY 項目に入った時に立っている）
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT,
               COLOR_WHITE); // 画面を白でクリア
}

// 指定したファイルのリプレイ再生を開始し、地図画面へ戻る。
static void startReplay(const char *filename) {
  set_replay_filename(filename);
  set_replaymode(true);
  set_demo_off();         // デモとリプレイは同時使用不可 → デモを無効化
  latlon_manager.reset(); // 位置履歴をクリア
  reset_degpersecond();   // 旋回角速度をリセット
  enqueueTask(
      createInitReplayTask()); // Core1: 選択したファイルを開いて読み込み開始
  selectedLine = -1;
  exit_setting(); // 設定を保存して地図画面へ
}

// 指定した地点でデモ飛行を始め、地図画面へ戻る（REPLAY/DEMO 画面の DEMO の行から）。
// 2026-11-05 までは設定画面の DEMO の行のトグルで地点を回し、値変更モードを抜けた時点で始めていた。
static void startDemo(demo_site_t site) {
  // デモとリプレイは同時使用不可 → リプレイを無効化（RITEM_OFF と同じ止め方）。
  // ★ 再生していないときは set_replaymode(false) を呼ばない。日時を無効にする副作用がある
  //   （gnss.cpp の set_replaymode。実 GNSS の次の NAV-PVT まで日時が --- になる）。
  if (getReplayMode()) {
    set_replaymode(false);
    set_replay_filename("");
  }
  set_demo_site(site);  // 仮想機体を地点の中心へ置き、位置履歴を消す
  reset_degpersecond(); // 旋回角速度をリセット
  selectedLine = -1;
  exit_setting(); // 設定を保存して地図画面へ
}

// REPLAY/DEMO 画面での長押し（決定）を処理する。
static void handleReplaySelect() {
  char label[40];
  int fsize = 0;
  ReplayItemType type = replay_menu_item(replay_cursor, replay_list_page, label,
                                         sizeof(label), &fsize);

  switch (type) {
  case RITEM_OFF:
    // リプレイもデモも解除して通常の GNSS に戻す
    set_replaymode(false);
    set_replay_filename("");
    set_demo_off();
    latlon_manager.reset();
    reset_degpersecond();
    backToSettingFromReplay();
    break;
  case RITEM_DEMO:
    startDemo((demo_site_t)replay_menu_demo_site(replay_cursor));
    break;
  case RITEM_FLIGHTONLY:
    // 静止区間をスキップするかを切り替える。画面はそのまま（続けて再生対象を選べる）
    set_replay_flight_only(!get_replay_flight_only());
    break;
  case RITEM_SPEED:
    // 再生速度を x1 → x2 → x?? と切り替える。再生中でも時刻は飛ばない
    cycle_replay_speed();
    break;
  case RITEM_SOURCE:
    // 再生元フォルダを切り替える。一覧の中身が丸ごと入れ替わるので、
    // カーソルとページを先頭へ戻してから取り直す
    // （戻さないと、前のフォルダの件数を前提にした位置を指したままになる）。
    toggle_replay_from_received();
    replay_cursor = REPLAY_IDX_SOURCE; // SOURCE 行に留まる（続けて切り替えられる）
    replay_list_page = 0;
    loading_replaylist = true;
    enqueueTask(createBrowseReplayTask(0));
    break;
  case RITEM_FILE: {
    // label はファイル名のみ。受信ログは received/ の下にあるので前置する。
    char path[REPLAY_FILENAME_LEN];
    snprintf(path, sizeof(path), "%s%s", replay_source_prefix(), label);
    startReplay(path);
    break;
  }
  case RITEM_RETURN:
    backToSettingFromReplay();
    break;
  default:
    break; // 空行を選んでいる場合は何もしない
  }
}

// 長押しコールバック: 設定画面への出入り、または設定項目の確定/解除。
// 状態遷移:
//   MAP/他 → 長押し → SETTING 画面へ移行
//   SETTING（未選択）→ 長押し → 現在行を「値変更モード」に入る
//   SETTING（変更中）→ 長押し → 値変更モードを終了し確定する
void longPressCallback() {
  power_charge_popup_snooze();
  backlight_notify_activity();
  last_user_input_ms = millis();
  redraw_screen = true;

  // PC接続モード中: プッシュスイッチ長押しでも手動切断して通常画面へ安全復帰
  if (screen_mode == MODE_PCCONNECT) {
    webserial_connected = false;
    webserial_cancel_requested = true;
    Serial.println("PONS:BYE");
    enqueueTask(createLogSdTask("PC DISCONNECTED (BUTTON LONG) -> RESUME MAP"));
    screen_mode =
        (prev_screen_mode != MODE_PCCONNECT) ? prev_screen_mode : MODE_MAP;
    redraw_screen = true;
    return;
  }

  // リプレイ選択画面での長押し = 「決定」。
  // 下の汎用パス（設定画面以外 →
  // 設定画面へ戻る）より先に処理しないと項目を選べない。
  if (screen_mode == MODE_REPLAYSELECT) {
    handleReplaySelect();
    return;
  }

  // チャンネル調査の一覧からの長押し = WIRELESS
  // 画面へ戻る（調べている途中なら止めて元の設定へ）。 ★
  // 下の汎用パス（設定画面以外 → 設定画面へ）より先に見ること。
  if (screen_mode == MODE_CHSCAN) {
    link_chscan_stop();
    screen_mode = MODE_WIRELESS;
    return;
  }

  // IMU / ESKF 画面での長押し = 「実行」。
  // 設定画面が「短押し=移動 / 長押し=実行」なので、そちらに操作感を合わせる。
  // この画面には Exit 項目があるため、長押しで設定画面へ戻る動作は持たせない。
  if (screen_mode == MODE_WIRELESS) {
    switch (wireless_cursor) {
    case 0: // モードを OFF → SENDER → RECEIVER と回す。
            // 3 択のトグルなので「送受同時
            // ON」を表現できない（docs/pons_link.md §1）。
      link_set_mode((link_get_mode() + 1) % 3);
      break;
    case 1: // チャネルを CH0 → 3 → 6 → 9 → 12 → 15 → 21 → 27 → 0
            // と回す（920.8-926.2MHz）。 帯域 500kHz
            // が重ならないものだけにしてある（link_proto.h の
            // LINK_RADIO_CHS）。 CH15
            // から上はアッパーバンド（休止に合わせた間隔で送り、SF11
            // は使わない。docs/pons_link.md §2）。
      link_set_radio_ch(link_radio_ch_next(link_get_radio_ch()));
      break;
    case 2: // 拡散率 SF7→SF8→…→SF11→SF7 と回す（帯域幅 500kHz は固定）。
            // SF を上げるほど届くが、送信時間＝送信電流が倍々に増える。
            // アッパーバンドの CH では SF11
            // を飛ばす（link_next_radio_profile）。
      link_set_radio_profile(link_next_radio_profile(link_get_radio_profile()));
      break;
    case 3: // グループ（A〜D）。電波の設定ではないのでモジュールへの書き込みは不要
            // （ペイロードの中身なので即座に反映される）。
            // ★ 長押し 1 回で A→B→C→D→A。**普段は SD の settings.txt で決めるもの**で、
            //   ここは「現地で確認する」「既定の A に戻ってしまったのを戻す」ための手段。
            //   4 つしか無いので、どのグループにも 3 回以内で届く。
      link_set_group((uint8_t)((link_get_group() + 1) % (LINK_GROUP_MAX + 1)));
      break;
    case 4: // 送信前チェックを手で走らせ直す。
            // ★ ブリングアップでは「アンテナを動かして測り直す」を何度もやる。
            //   従来は CH を変えて戻す遠回りしか手段が無かった。
            //   TX 以外では走らないので、断られたら低い音で知らせる
            //  （設定画面の「いま実行できない操作」と同じ 440Hz）。
      if (!link_preflight_restart())
        enqueueTask(createPlayMultiToneTask(440, 200, 1));
      break;
    case 5: // 選べるチャンネルを全部聴いて一覧にする（link_chscan_*）。
            // モジュールが応答しないなら走らないので、断られたら低い音。
            // ★ チャンネルは変えない。一覧を見てから、この画面の Ch
            // で人が選ぶ。
      if (link_chscan_start())
        screen_mode = MODE_CHSCAN;
      else
        enqueueTask(createPlayMultiToneTask(440, 200, 1));
      break;
    default: // 戻る
      // ★ CH / SF / Group を触ったまま出るときは必ず知らせる。
      //   この 3 つは **3 台すべてで一致していないと通信できない**のに、
      //   変えたのが 1 台だけで連絡を忘れる、という事故が起こりうる。
      //   症状は「無信号」なので、現場では距離や故障と区別が付かない。
      //
      //   ★★ **送信機・受信機のどちらでも鳴らすこと。** ここに mode の分岐を
      //     入れてはいけない。受信機側を触って戻し忘れる事故も同じだけ起こるし、
      //     受信機は 2 台あるぶん取り違えやすい。
      link_warn_setting_changed();
      screen_mode = MODE_SETTING;
      // WIRELESS に入った時に CallbackEnter 経由で selectedLine が立ったまま
      // （値変更モード）になっている。WIRELESS 項目には CallbackToggle が無い
      // (nullptr) ため、戻さずに戻ると次の短押しで nullptr
      // 呼び出し→フリーズする。
      selectedLine = -1;
      // ★ 画面を白で消してから戻る（ほかの「設定画面へ戻る」経路と同じ）。
      //   設定画面はタイトルを tft へ字の背景色だけで上書きし、本体を
      //   y=30〜269・ フッタを y=280〜
      //   に置くので、**塗らない所が残る**。WIRELESS 画面は ヘッダを
      //   y=0〜39・本体を y=40〜279 に置くので、戻ると `WIRELESS LINK` の
      //   タイトルの一部と、本体の下端（フッタのすぐ上の帯）が古いまま残っていた。
      tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
      break;
    }
    redraw_screen = true;
    return;
  }

  if (screen_mode == MODE_IMUDETAIL) {
    if (imu_guide_page >= 0)      imu_guide_next();   // 入口の案内: 長押しも「次へ」
    else if (tas_guide_page >= 0) tas_guide_next();   // 推定 TAS の案内も同じ
    else                          imu_execute();
    return;
  }

  if (screen_mode != MODE_SETTING) {
    // 設定画面以外から長押し → 設定画面へ遷移
    if (screen_mode == MODE_GNSSDETAIL)
      gnss_getposition_mode(); // GNSS 詳細画面での位置取得モード解除
    screen_mode = MODE_SETTING;
    cursorLine = 0;
    selectedLine = -1;
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT,
                 COLOR_WHITE); // 画面を白でクリア
  } else {
    // 設定画面内での長押し
    if (selectedLine == -1 &&
        menu_settings[cursorLine].CallbackEnter != nullptr) {
      // CallbackEnter がある項目（サブ画面に入る項目）: Enter
      // を実行して選択状態へ
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

// 旋回角速度をリセットする。GNSS 受信開始時や停止時などに呼ぶ。
// サンプル配列を現在の真方位と現在時刻で埋めることで、急激な deg/s
// の跳ね上がりを防ぐ。
//
// ※ **画面を切り替えたときには意図して呼んでいない。** 窓は 4 サンプルなので、
//   更新が数秒途切れたあとは窓がその空白をまたぎ、経過時間が過大になって deg/s
//   が 実際より小さく出る（3 サンプルぶん＝約 1.5 秒で正常に戻る）。
//   小さく出る側＝「旋回していない」と見える側なので、コース警報が早く鳴ることは
//   あっても鳴らなくなることはない。安全側に外れるため直していない。
void reset_degpersecond() {
  float track = get_gnss_truetrack();
  uint32_t now = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    truetrack_samples[i] = track;
    truetrack_sample_times[i] = now;
  }
  degpersecond = 0;
}

// 旋回角速度を更新する。GNSS コース更新時（newcourse_arrived）に呼ぶ。
// アルゴリズム: スライディングウィンドウの方位変化合計を経過時間で正規化
//   1. 新しい真方位と現在時刻を配列に追加
//   2. 配列が満杯になったら連続サンプル間の差分を合計
//   3. 合計を「最古サンプルと最新サンプルの時間差（秒）」で割って真の deg/s
//   を算出
//   4. 配列を1つずらして古いサンプルを捨てる
//   ※ 360度またぎ（例: 359→1度）を -180〜+180 に正規化して計算する
//   ※ 時間正規化により GNSS レート（1Hz / 2Hz）に依存しない
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
    uint32_t elapsed_ms =
        truetrack_sample_times[NUM_SAMPLES - 1] - truetrack_sample_times[0];
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
    sampleIndex = NUM_SAMPLES - 2; // 次回は末尾に書き込む位置に調整
  }
  sampleIndex++;
}

// AUTO10K の折返しフェーズを変更する。
// ★ **リプレイ中とデモ中は表示だけ変え、SD には保存しない。** どちらも位置が
//   仮想なので、そこで出たフェーズを焼くと**次の起動が復路から始まる**。
//   デモは必ず踏む: `DEMO_BIWAKO` は PLA を中心に 15km 走るので 10.475km
//   を越え、 他の地点は PLATHOME から 100km 以上あるので**開始直後の 1
//   回目で成立する**。
// ★ 表示側の `auto10k_status` は動かすこと。デモの針路生成（gnss.cpp）が
//   これを見て折り返し後に東向きへ変わる。抜けるときの復帰は
//   `auto10k_restore_flight_phase()`（gnss.cpp の set_demo_site /
//   set_replaymode）。
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

// ナビゲーションの判断と警報。**画面モードによらず loop() から毎回呼ぶこと。**
// 描画は一切しない（描画は MODE_MAP の分岐が担当する）。
//
// 内訳:
//   1. Auto10km の折返しフェーズ判定（GNSS 位置が変わったときだけ）
//   2. 目的地が遠すぎる警告（同上）
//   3. 旋回角速度・旋回音・針路誤差・コース逸脱警報（GNSS コース更新時だけ）
static void nav_alarm_tick(bool new_gnss_info) {
  // 位置そのものをキーにして「本当に動いたときだけ」評価する。
  // new_gnss_info のフラグを落とすのは MODE_MAP
  // の描画ブロック（set_new_location_off()）
  // なので、地図画面以外ではフラグが立ちっぱなしになる。毎ループ Haversine を
  // 回さないよう、pla_centerline_bearing_rad() と同じく入力の変化で判定する。
  static double last_nav_lat = NAN, last_nav_lon = NAN;
  if (new_gnss_info) {
    const double nav_lat = get_gnss_lat();
    const double nav_lon = get_gnss_lon();
    if (nav_lat != last_nav_lat || nav_lon != last_nav_lon) {
      last_nav_lat = nav_lat;
      last_nav_lon = nav_lon;

      // Automatic destination change for AUTO 10KM mode.
      // currentdestination の有効性も確認する。init_destinations() は Core1 の
      // setup1() で走るため、 それが終わる前に Core0 がここへ来ると
      // -1（未選択）のまま配列外を読んでしまう。
      if (destination_mode == DMODE_AUTO10K && currentdestination != -1 &&
          currentdestination < destinations_count) {
        double destlat = extradestinations[currentdestination].cords[0][0];
        double destlon = extradestinations[currentdestination].cords[0][1];
        double distance_frm_destination =
            calculateDistanceKm(nav_lat, nav_lon, destlat, destlon);
        // フェーズが変わったときは、起動直後・リプレイ切替直後であっても必ず鳴らす。
        // 保存した INTO を復元した状態でプラットホーム付近にいると、下の
        // 「1.5km 以内なら AWAY」が即座に成立して起動直後に鳴ることがあるが、
        // 黙って向きが変わるより、鳴らして確認を促す方が安全という判断。
        if (auto10k_status == AUTO10K_AWAY) {

          if (distance_frm_destination >
              10.475) { // 公式ルール 10.975km
                        // が折り返し地点だが、実際には潮流などの影響が影響があるため、500mの誤差を引いておく。
            apply_auto10k_status(AUTO10K_INTO);
            // ★ 最後の引数 true = solo_play。**下の INTO→AWAY
            // と必ず揃えること。**
            //   solo_play にすると、4 音を鳴らし切ってから
            //   destination_change.wav が鳴る
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
            //   消さずに後ろへ並べる。Core0 は単スレッドなので 5
            //   つは必ず連続して入る。
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(2793, 500, 1, 3, 0, true));
            enqueueTask(createPlayMultiToneTask(3136, 500, 1, 3, 0, true));
            enqueueTask(createPlayWavTask("wav/destination_change.wav", 3));
          }
        }
        if (auto10k_status == AUTO10K_INTO &&
            distance_frm_destination <
                1.5) { // 折り返し地点用。再度の折り返しは 1km
                       // だが、500mの誤差を足しておく。
          apply_auto10k_status(AUTO10K_AWAY);
          // solo_play。上の AWAY→INTO
          // と同じ鳴り方にしてある（理由は上のコメント。
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

  // GNSS コース更新時（1Hz または 2Hz）にのみ実行する処理
  if (newcourse_arrived) {
    // ★ float で受けること。以前は int で、切り捨て誤差が窓の両端に残り
    //   deg/s に最大 ±0.67（誤差1度 ÷ 窓1.5秒）の量子化ノイズが乗っていた。
    //   着色 1.0 / 強調 3.0 / コース警報の「修正中」判定 0.5 deg/s という
    //   しきい値に対して無視できない大きさだった。
    //   truetrack_samples[] は元から float、GNSS 側も NAV-PVT の headMotion
    //   （分解能 1e-5 度）なので、捨てていたのはここだけ。
    float ttrack = get_gnss_truetrack();

    // ★ **この順番を入れ替えないこと。**
    //   nav_update() → steer_angle → update_course_warning() の順でなければ、
    //   警報は 1 周期古いズレ角で判定する。以前はまさにそうなっていて、
    //   Auto10km でフェーズが反転した直後の約 1
    //   秒、警報が反転前のコースを見ていた。 nav_update() は MODE_MAP
    //   の描画ブロックでも呼ぶが、これは Haversine と atan2 が 1
    //   回ずつなので、2 度呼んでも負荷は問題にならない。
    nav_update(); // truec（目的地への真方位）を最新 GNSS 位置で更新

    // 針路誤差 (steer_angle) の計算:
    // truec はナビが指示する真方位コース、ttrack は GNSS 実測の真方位。
    // どちらも小数を保った値なので、結果もそのまま使える（表示だけ丸める）。
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

    update_degpersecond(ttrack); // 旋回角速度 [deg/s] を更新
    update_tone(degpersecond);   // 旋回音の音程を更新
    update_course_warning(
        degpersecond); // コース逸脱警告の積算値を更新（steer_angle を読む）

    newcourse_arrived = false;
  }
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
  if (calculateDistanceKm(get_gnss_lat(), get_gnss_lon(), destlat, destlon) >
      100.0) {
    if (millis() - last_destination_toofar_time >
        1000 * 120) { // 120秒クールダウン
      last_destination_toofar_time = millis();
      enqueueTask(createPlayWavTask("wav/destination_toofar.wav", 1));
    }
  }
}

// コース逸脱の警告を管理する（GNSS コース更新時に呼ぶ、GNSS レート非依存）。
// 積算型アルゴリズム（単位: 度・秒）:
//   - 前回呼び出しからの経過時間 dt[秒] を計測し、積算量に乗算することで GNSS
//   レート
//     （1Hz / 2Hz）に依存しない実時間ベースの累積を行う
//   - コースのズレ角 (steer_angle) × dt を course_warning_index に加算
//   - 正しい方向に修正中は 15 × |degpersecond| × dt を減算
//   - index が 900 に達したら音声警告を発報し、30秒のクールダウンに入る
//   - 低速・GNSS ロスト時は発動しない（誤警告防止）
//   - 設定画面等からの復帰時に dt が過大になった場合はスキップ（誤発報防止）
void update_course_warning(float degpersecond) {
  // GNSS レート非依存のため、前回呼び出しからの経過時間 dt[秒] を計算する
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

  // 移動していない時,GNSSロスト時は発動しない。
  // ★ fix も見る（get_gnss_fix() はミラー・デモ・リプレイの供給元ごとに答える）。fix が無くても NAV-PVT は
  //   来るので、ここは毎回呼ばれる。衛星数と速度だけでは、衛星 1〜3 個で fix していないときの速度の
  //   ばらつきで積算が進み得た（2026-11-10 まで）。
  if (!get_gnss_fix() || get_gnss_mps() < 2 || get_gnss_numsat() == 0) {
    course_warning_index = 0;
  }
  // 正しい方向に変化している時はindexを減らす。
  else if ((degpersecond > 0.5 && steer_angle > 0) ||
           (degpersecond < -0.5 && steer_angle < 0)) {
    course_warning_index -=
        15 * abs(degpersecond) *
        dt; // 修正速度×dt に比例して減算（1Hz相当で 3deg/s の時 45/秒）

  }
  // 正しい方向に修正されていない、かつ15度以上ずれている。
  else if (abs(steer_angle) > 15) {
    course_warning_index +=
        min(abs(steer_angle), 90) *
        dt; // 15以上、90を最大値として、dt を乗算して加算する。
  }
  // 15度未満のズレ、index 0 reset。
  else {
    course_warning_index = 0;
  }

  // range 0-900
  if (course_warning_index > 900)
    course_warning_index = 900;
  else if (course_warning_index < 0)
    course_warning_index = 0;

  // 15度の修正されないズレは、900/15=60秒でwarning。90度以上の修正されないズレは、10秒でwarning。ただしwarningは、30秒に一度をmax回数とする。
  if (course_warning_index >= 900 &&
      millis() - last_course_warning_time > 30000) {
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