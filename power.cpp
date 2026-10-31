// ============================================================
// File    : power.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 電源まわり。USB 判定と、電源経路の異常（充電 IC の故障）の検出と警報、
//           高温での充電の警告、過放電（3.0V 未満）の本体フラッシュ記録と充電禁止ロック。
//           v7.1 では「SW を OFF にしたまま USB で充電している」の検出と読み上げも。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/29
// ============================================================
//
// ■ 何を見張るか（docs/pons_power.md）
//   2026-10-15、v7 の充電 IC が電池につながったまま内部で壊れ、
//   電池 → 充電 IC → VBUS → D3 → VSYS と回って SW で切れず、画面は「USB」になった。
//   本物の USB なら電圧は 4.3V 以上あるので、「USB に見えるのに電圧が低い」が
//   続いたら異常として、画面・音・ログ・無線で知らせる。判定値は settings.h。
//
// ■ SW を OFF にしたまま USB で充電している（v7.1 のみ）
//   v7.1 は充電 IC が SW（ロードスイッチ Q3）の後ろにあるので、SW OFF のままだと電池へは
//   Q3 のボディダイオード経由になり、満充電の手前で充電が終わる。本体は USB で起動し、
//   充電 LED も消えるので見た目では分からない。SW4 の OFF 側の接点が GPIO40 を GND に
//   落とすので、「本体が動いているのに GPIO40 ≒ 0V」で分かる。地図に注意を出し、1 度だけ読み上げる。
//
// ■ 高温での充電（v7・v7.1 とも）
//   電池の仕様は「充電 0〜45°C」。MCP73831 には温度を入れる端子が無く、夏の車内や日なたでも
//   充電を続ける。USB がつながっている間に MS5611（本体基板）の温度が CHARGE_HOT_C を
//   超え続けたら、地図に出して読み上げる。止める手段（充電 IC の PROG を切る GPIO）は無いので、
//   USB を抜いてもらう。
//
// ■ 過放電（3.0V 未満）の記録と充電禁止ロック（v7・v7.1 とも）
//   LiPo 電池が一度でも 3.0V 未満に入ると負極銅箔の溶出が進み、次回の USB 充電時に内部微小短絡・
//   発火に至る危険がある。ソフトから充電 IC を止める手段は無く、v7.1 では SW OFF でも Q3 の
//   ボディダイオード経由で USB 充電電流が流れるため、本体フラッシュ（EEPROM 領域）にラッチして
//   次回起動以降に全画面で警告する。
//   - フライト中（電池単体で動作中）に 3.0V を割ったその場では新しい警告も音も出さず、電池を
//     使い切るまで航法を続けさせる（1 回だけ Core1 経由でフラッシュへラッチを記録する）。
//   - 次回以降の起動時は、電池単体なら警告画面＋警告トーン・音声の後にボタン操作でナビ画面へ
//     進めるが、USB 接続時はボタンで抜けられない完全ロックとし、30 秒おきに警告トーン＋音声を鳴らす。
//   - 電池単体で起動警告を抜けた後や、同一セッションで 3.0V を割った後に USB を挿された場合も、
//     BATT_LATCH_USB_HOLD_MS 継続で即座に完全ロック画面へ入る。
//   - SD ルートに batt_unlock.txt を置いて起動すると Core1 の setup_sd() がラッチを解除し、
//     消し忘れ防止のためファイルを自動削除する。
//
// ★ 警報はここ（loop() の最上位）で鳴らす。描画は display_tft.cpp が
//   power_fault_active() 等を見て描く（CLAUDE.md「警報を描画関数の中に書かない」）。
#include "power.h"
#include "settings.h"
#include "mysd.h"
#include "display_tft.h"  // get_input_voltage()
#include "gnss.h"         // gnss_get_own_fix()・get_gnss_date/time()・utcToJst()・リプレイ／デモ中か
#include "link.h"         // link_mirror_active()
#include "airdata.h"      // get_airdata_temperature()（高温での充電）
#include "lora_link/link_proto.h"  // link_crc16()
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>

#define POWER_SAMPLE_MS 250UL

static bool     s_fault       = false;
static uint32_t s_cond_since  = 0;     // 異常の条件が成り立ち始めた時刻（0 = 成り立っていない）
static uint32_t s_last_alert  = 0;
static uint32_t s_last_sample = 0;
static float    s_volt        = 0.0f;  // VBUS（生の平均。ピーク追跡は通さない）
static float    s_batt_v      = 0.0f;  // GPIO40（BATPWR）の生の平均。SW OFF なら SW4 が GND に落とすので 0V
static bool     s_swoff       = false; // SW OFF のまま USB（CHARGE_SW_OFF_HOLD_MS 続いて確定）
static uint32_t s_swoff_since = 0;
static bool     s_swoff_said  = false; // この回の SW OFF をもう読み上げたか
static bool     s_moving      = false;
static uint32_t s_still_since = 0;     // 移動中に止まり始めた時刻（0 = まだ止まっていない）
static bool     s_hot         = false; // 高温での充電（CHARGE_HOT_HOLD_MS 続いて確定）
static uint32_t s_hot_since   = 0;
static uint32_t s_hot_alert   = 0;
static float    s_hot_temp    = 0.0f;  // 判定に使った温度 [°C]（表示・ログ用）

// ---- 過放電（3.0V 未満）ラッチ状態 ----
// ★ s_latch_rec の読み書きは taskQueueMutex で短時間（構造体 32B のコピーのみ）保護する。
//   EEPROM.commit() は内部で rp2040.idleOtherCore() を呼ぶため、**ミューテックスを持ったまま
//   EEPROM.put / commit を絶対に呼ばないこと**（Core0 が enqueueTask で同じミューテックスを
//   待っているとデッドロックする）。
static BattLatchRecord s_latch_rec        = {};
static volatile bool   s_latch_ready      = false;  // Core1 の初期読込完了
static volatile bool   s_boot_latched     = false;  // 起動時点でラッチされていたか
static volatile bool   s_runtime_latched  = false;  // 今回の起動中に 3.0V 未満が確定したか
static volatile bool   s_commit_pending   = false;  // Core1 のフラッシュ書込待ち
static volatile bool   s_cleared_notice   = false;  // SD 裏コマンドで解除された通知（1 回限り）
static bool            s_latch_task_queued = false; // Core0 がフラッシュ書込タスクを enqueue 済みか
static uint32_t        s_od_cond_since    = 0;      // 3.0V 未満条件が連続し始めた時刻
static uint32_t        s_od_usb_since     = 0;      // ラッチ中に USB 給電が連続し始めた時刻
static bool            s_od_usb_lock_due  = false;  // 通常画面から完全ロック画面へ入る要求
static bool            s_od_screen_active = false;  // いま全画面の過放電警告／ロックを出しているか
static uint32_t        s_od_last_usb_alert = 0;     // 完全ロック中に最後に警報を鳴らした時刻
static bool            s_od_batt_announced = false; // 電池単体の警告画面で 1 回目を鳴らしたか

static void charge_log_tick(uint32_t now);

// ADC の生の平均を電圧にする。ピーク追跡（get_input_voltage）は高い値を
// 数秒保持するので、ここでは使わない。
static float read_volt(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return BATTERY_MULTIPLYER(sum / 4.0f);
}

static void sample() {
  s_volt   = read_volt(USB_VBUS_ADC_PIN);
  s_batt_v = read_volt(BATTERY_PIN);
}

// 異常の条件（1 回の測定ぶん）。
static bool fault_condition() {
  // ★ SW OFF のあいだは判定しない。電池は Q3 で切り離されていて（ボディダイオードも
  //   電池 → 充電 IC の向きには流さない）、壊れた充電 IC が電池で VBUS を持ち上げることが
  //   できない。このとき VBUS が 2.0〜4.3V なのは**弱い USB**（細いケーブル・へたった
  //   モバイルバッテリー）で、異常と言うと誤報になる。
  if (power_sw_off_on_usb()) return false;
  return s_volt >= POWER_FAULT_VBUS_MIN_V && s_volt < USB_VBUS_PRESENT_V;
}

bool power_fault_active() { return s_fault; }

bool usb_power_present() {
  if (s_fault) return false;
  return s_volt >= USB_VBUS_PRESENT_V;
}

bool power_sw_off_on_usb() {
  // ★ **USB の判定（usb_power_present()）を条件に入れないこと。** 本体が動いているのに
  //   電池が 0V に見えるなら、それだけで SW OFF と言い切れる（SW ON なら GPIO40 は電池の
  //   電圧で、保護回路付きの 1s は 2.5V を切らない。SW OFF なら電源は USB しかない）。
  //   USB を条件に入れると、細いケーブルで VBUS が一瞬 4.3V を割っただけで外れ、
  //   その瞬間に 0V が本物の電池電圧として電池低下の警告・無線・CSV へ流れる。
  // ★ 確定（s_swoff）を待たずに今の測定で答える。確定までの 2 秒間も 0V を本物として
  //   扱わないようにするため。
  return s_batt_v < CHARGE_SW_OFF_BATT_V;
}

// 地図が自機の状態を映しているか。ミラー中は機体、リプレイ中は過去のログ、デモ中は
// 仮想機体を映しているので、自機の充電の話は重ねない。
// （replay_voltage_active() はリプレイ中しか true にならないので getReplayMode() で足りる）
static bool showing_own_state() {
  return !getReplayMode() && !link_mirror_active() && !is_demo_active();
}

// 移動中か。**自機の** GNSS（ミラーを通さない）の対地速度で見る。
// ★ fix が無いときは静止とみなす（屋内で充電している場面が本命なので、fix 待ちで黙らせない）。
// ★ GNSS が抜けると stored_* が凍ったまま fix が残るので、接続も見る。
// ★ 止まってもすぐには「静止」に戻さない（CHARGE_SW_STILL_MS）。信号待ちのたびに出たり消えたりしない。
static void update_moving(uint32_t now) {
  double lat, lon, gs, trk;
  const bool fix = get_gnss_connection() && gnss_get_own_fix(lat, lon, gs, trk);
  if (fix && gs > CHARGE_SW_MOVING_MPS) {
    s_moving      = true;
    s_still_since = 0;
  } else if (s_moving) {
    if (s_still_since == 0) s_still_since = now;
    else if (now - s_still_since >= CHARGE_SW_STILL_MS) s_moving = false;
  }
}

static uint16_t latch_calc_crc(const BattLatchRecord& r) {
  return link_crc16(reinterpret_cast<const uint8_t*>(&r), offsetof(BattLatchRecord, crc16));
}

static bool latch_rec_valid(const BattLatchRecord& r) {
  if (r.magic != BATT_LATCH_EE_MAGIC) return false;
  if (r.version != BATT_LATCH_EE_VERSION) return false;
  return r.crc16 == latch_calc_crc(r);
}

void power_latch_init_core1() {
  EEPROM.begin(BATT_LATCH_EE_TOTAL_SIZE);
  BattLatchRecord raw = {};
  EEPROM.get(BATT_LATCH_EE_ADDR, raw);
  if (!latch_rec_valid(raw)) {
    // 初回起動（0xFF）またはデータ破損時は未ラッチの初期構造体を RAM に用意する。
    // 毎起動の不要なフラッシュ消去を避けるため、ここでは commit しない。
    memset(&raw, 0, sizeof(raw));
    raw.magic   = BATT_LATCH_EE_MAGIC;
    raw.version = BATT_LATCH_EE_VERSION;
    raw.crc16   = latch_calc_crc(raw);
  }
  mutex_enter_blocking(&taskQueueMutex);
  s_latch_rec    = raw;
  s_boot_latched = (raw.latched != 0);
  s_latch_ready  = true;
  mutex_exit(&taskQueueMutex);
}

void power_latch_commit_core1() {
  BattLatchRecord local;
  bool do_commit = false;
  mutex_enter_blocking(&taskQueueMutex);
  if (s_commit_pending) {
    local = s_latch_rec;
    s_commit_pending = false;
    do_commit = true;
  }
  mutex_exit(&taskQueueMutex);
  if (!do_commit) return;

  // ★ taskQueueMutex を解放してから commit する（idleOtherCore とのデッドロック防止）。
  EEPROM.put(BATT_LATCH_EE_ADDR, local);
  const bool ok = EEPROM.commit();

  log_sdf("BATT LATCH committed to flash (ok=%d events=%u v=%.2fV)",
          ok ? 1 : 0, (unsigned)local.event_count, local.min_volt_mv / 1000.0f);
}

bool power_latch_unlock_core1(uint16_t* out_events, uint16_t* out_resets, uint16_t* out_min_mv) {
  BattLatchRecord local;
  bool was_latched = false;

  mutex_enter_blocking(&taskQueueMutex);
  was_latched = (s_latch_rec.latched != 0) || s_boot_latched || s_runtime_latched;
  s_latch_rec.magic   = BATT_LATCH_EE_MAGIC;
  s_latch_rec.version = BATT_LATCH_EE_VERSION;
  if (was_latched) {
    s_latch_rec.latched = 0;
    if (s_latch_rec.reset_count < 0xFFFF) s_latch_rec.reset_count++;
    s_latch_rec.crc16 = latch_calc_crc(s_latch_rec);
    s_cleared_notice  = true;
  }
  s_boot_latched    = false;
  s_runtime_latched = false;
  s_commit_pending  = false;
  local = s_latch_rec;
  mutex_exit(&taskQueueMutex);

  if (out_events) *out_events = local.event_count;
  if (out_resets) *out_resets = local.reset_count;
  if (out_min_mv) *out_min_mv = local.min_volt_mv;

  if (was_latched) {
    EEPROM.put(BATT_LATCH_EE_ADDR, local);
    EEPROM.commit();
  }
  return was_latched;
}

bool power_latch_ready() { return s_latch_ready; }

bool power_overdischarge_latched() {
  return s_boot_latched || s_runtime_latched;
}

bool power_overdischarge_boot_latched() {
  return s_boot_latched;
}

bool power_overdischarge_usb_active() {
  // 1) 通常の USB 給電（VBUS >= 4.3V）。
  //    ★ usb_power_present() は s_fault 確定後 false に張り付くため、ここでは s_volt を直接見る。
  //      接触不良や遅い挿抜で一度 s_fault が立ったあとに 5V USB が供給されているケースでも、
  //      過放電電池への充電は現に進行するためロックを解除してはいけない。
  if (s_volt >= USB_VBUS_PRESENT_V) return true;
  // 2) v7.1 で SW OFF のまま USB 給電中（GPIO40 < 0.5V）。
  //    SW OFF でも Q3 のボディダイオード経由で電池へ充電電流が流れる。
  if (power_sw_off_on_usb()) return true;
  // 3) 細いケーブルや弱い USB 電源で過放電電池を引いて VBUS が 4.3V 未満へ垂れている場合。
  //    MCP73831 は降圧リニア充電 IC（昇圧回路なし）なので、充電 IC 故障による電池からの逆流では
  //    VBUS（s_volt）が電池電圧（s_batt_v）を超えることは物理的にあり得ない。
  //    s_volt >= 2.0V かつ電池より 0.35V 超高いなら外部 USB から充電方向の電位がかかっている。
  if (s_volt >= POWER_FAULT_VBUS_MIN_V && s_volt > s_batt_v + 0.35f) return true;
  return false;
}

bool power_overdischarge_usb_lock_due() {
  return power_overdischarge_latched() && s_od_usb_lock_due;
}

void power_overdischarge_screen_enter() {
  sample();
  s_od_screen_active  = true;
  s_od_usb_lock_due   = false;
  s_od_usb_since      = 0;
  s_od_last_usb_alert = 0;
  s_od_batt_announced = false;
}

void power_overdischarge_screen_exit() {
  s_od_screen_active = false;
  s_od_usb_lock_due  = false;
  s_od_usb_since     = 0;
}

bool power_take_overdischarge_cleared() {
  bool v = false;
  mutex_enter_blocking(&taskQueueMutex);
  if (s_cleared_notice) {
    v = true;
    s_cleared_notice = false;
  }
  mutex_exit(&taskQueueMutex);
  return v;
}

void power_get_overdischarge_record(BattLatchRecord* out) {
  if (!out) return;
  mutex_enter_blocking(&taskQueueMutex);
  *out = s_latch_rec;
  mutex_exit(&taskQueueMutex);
}

float power_get_vbus_voltage() { return s_volt; }

bool power_sw_off_notice_active() {
  // !s_fault / !s_hot / !power_overdischarge_latched():
  // 過放電ラッチ中・電源経路異常・高温充電中は「充電しないで」の状態なので、
  // 「SW を入れて充電して」の案内は絶対に出さない。
  return s_swoff && !s_fault && !s_hot && !power_overdischarge_latched() &&
         !s_moving && showing_own_state();
}

bool  power_charge_hot_active() { return s_hot; }
float power_charge_hot_temp()   { return s_hot_temp; }

void power_format_status(char* buf, size_t n) {
  const char* hot = s_hot ? " hot" : "";
  const char* od  = power_overdischarge_latched() ? " od" : "";
  snprintf(buf, n, "usb=%d vbus=%.2fV%s%s%s", usb_power_present() ? 1 : 0, s_volt,
           power_sw_off_on_usb() ? " sw=off" : "", hot, od);
}

void power_setup() {
  sample();
  s_last_sample = millis();
}

static void enqueue_overdischarge_alert() {
  // 警告トーン（2637Hz 150ms × 4 回、solo_play=true）→ 音声の順で鳴らす。
  // solo_play=true にしておくことで、4 音が鳴り終わるまで音声が pending_wav で待ち、
  // トーンと音声が重ならずに順番に再生される（src/sound.cpp）。
  // ★ 最低音量 60: 過放電電池への充電は発火の恐れがあるため、音量 0 設定でも鳴らす
  //   （power_fault.wav / charge_hot.wav と同じ例外。docs/pons_sound.md §5）。
  enqueueTask(createPlayMultiToneTask(2637, 150, 4, 4, 60, true));
  enqueueTask(createPlayWavTask("wav/batt_overdischarge.wav", 4, 60));
}

// 過放電（3.0V 未満）の監視・フラッシュ記録要求・警告画面中の警報送出。
static void overdischarge_tick(uint32_t now) {
  if (!s_latch_ready) return;

  // キュー満杯で TASK_LATCH_BATT_OVERDISCHARGE の enqueue に失敗していた場合のみ再送する。
  // ★ isTaskInQueue / isTaskRunning の別ロック確認は Core1 の完了タイミングと TOCTOU 競合して
  //   重複書き込みを起こし得るため、Core0 ローカルの s_latch_task_queued で一意に管理する。
  if (!power_overdischarge_latched()) {
    s_latch_task_queued = false;
  } else if (s_commit_pending && !s_latch_task_queued) {
    s_latch_task_queued = enqueueTask(createLatchBattTask());
  }

  // 1) 未ラッチ時の 3.0V 未満連続監視（既にラッチ済みなら再 commit は絶対に行わない）
  if (!power_overdischarge_latched()) {
    const float peak_v = get_input_voltage();
    const bool measurable = !usb_power_present() && !power_sw_off_on_usb() && !power_fault_active();
    // 生 ADC（s_batt_v）とピーク追跡（peak_v）の両方が [1.80V, 3.00V) にあることを要求する:
    //   - 一瞬の重負荷ドロップは peak_v が >=3.00V に残るので除外される
    //   - SW OFF スライド時は s_batt_v が即座に <1.80V（0V）へ落ちるので除外される
    const bool under_3v = measurable &&
                          now >= BAT_OVERDISCHARGE_ARM_MS &&
                          s_batt_v >= BAT_OVERDISCHARGE_MIN_VALID_V &&
                          s_batt_v <  BAT_OVERDISCHARGE_VOLTAGE &&
                          peak_v   >= BAT_OVERDISCHARGE_MIN_VALID_V &&
                          peak_v   <  BAT_OVERDISCHARGE_VOLTAGE;
    if (!under_3v) {
      s_od_cond_since = 0;
    } else if (s_od_cond_since == 0) {
      s_od_cond_since = now ? now : 1;
    } else if (now - s_od_cond_since >= BAT_OVERDISCHARGE_HOLD_MS) {
      s_od_cond_since = 0;

      uint8_t  flags = 0;
      uint32_t jst_date = 0;
      uint32_t jst_time = 0;
      if (!getReplayMode() && s_moving) flags |= BATT_LATCH_FLAG_MOVING;
      if (!getReplayMode()) {
        const GnssDate gd = get_gnss_date();
        const GnssTime gt = get_gnss_time();
        if (gd.isValid() && gt.isValid() && gd.year() >= 2020) {
          int y = gd.year(), mo = gd.month(), d = gd.day(), h = gt.hour();
          utcToJst(&y, &mo, &d, &h);
          jst_date = (uint32_t)y * 10000U + (uint32_t)mo * 100U + (uint32_t)d;
          jst_time = (uint32_t)h * 10000U + (uint32_t)gt.minute() * 100U + (uint32_t)gt.second();
          flags |= BATT_LATCH_FLAG_JST_VALID;
        }
      }
      const uint16_t mv = (uint16_t)lroundf(s_batt_v * 1000.0f);
      const uint32_t up = now / 1000UL;
      uint16_t ev_cnt = 0;

      mutex_enter_blocking(&taskQueueMutex);
      s_latch_rec.magic         = BATT_LATCH_EE_MAGIC;
      s_latch_rec.version       = BATT_LATCH_EE_VERSION;
      s_latch_rec.latched       = 1;
      s_latch_rec.flags         = flags;
      s_latch_rec.min_volt_mv   = mv;
      if (s_latch_rec.event_count < 0xFFFF) s_latch_rec.event_count++;
      ev_cnt                    = s_latch_rec.event_count;
      s_latch_rec.date_yyyymmdd = jst_date;
      s_latch_rec.time_hhmmss   = jst_time;
      s_latch_rec.uptime_sec    = up;
      s_latch_rec.crc16         = latch_calc_crc(s_latch_rec);
      s_runtime_latched         = true;
      s_commit_pending          = true;
      mutex_exit(&taskQueueMutex);

      // ★ フライト中（電池単体）はフラッシュ記録とテキストログのみ。新警告も音声も出さない。
      s_latch_task_queued = enqueueTask(createLatchBattTask());
      enqueueTask(createLogSdfTask(
        "BATT OVERDISCHARGE LATCHED: raw=%.2fV peak=%.2fV events=%u moving=%d date=%lu time=%06lu",
        s_batt_v, peak_v, (unsigned)ev_cnt, (flags & BATT_LATCH_FLAG_MOVING) ? 1 : 0,
        (unsigned long)jst_date, (unsigned long)jst_time));
    }
  }

  // 2) 全画面の警告／完全ロック表示中の警報送出
  if (s_od_screen_active) {
    if (power_overdischarge_usb_active()) {
      // USB 接続中（完全ロック）: 初回は即座に、以降は BATT_LATCH_ALERT_MS（30秒）ごとに繰り返す。
      // ケーブル接触不良で USB 判定が一瞬切れても s_od_last_usb_alert はリセットしない（連打防止）。
      if (s_od_last_usb_alert == 0 || now - s_od_last_usb_alert >= BATT_LATCH_ALERT_MS) {
        s_od_last_usb_alert = now ? now : 1;
        s_od_batt_announced = true;
        enqueue_overdischarge_alert();
      }
    } else if (!s_od_batt_announced) {
      // 電池単体での起動警告画面: 警告トーン＋音声を 1 回だけ鳴らす
      s_od_batt_announced = true;
      enqueue_overdischarge_alert();
    }
    return;
  }

  // 3) 通常画面で動作中にラッチ済みの状態で USB が挿されたら、完全ロック画面へ遷移させる
  //    （電池単体で起動警告を抜けた後に USB を挿した場合・同一セッションで 3.0V 割れ後に挿した場合の両方）
  if (power_overdischarge_latched() && power_overdischarge_usb_active()) {
    if (s_od_usb_since == 0) {
      s_od_usb_since = now ? now : 1;
    } else if (now - s_od_usb_since >= BATT_LATCH_USB_HOLD_MS) {
      s_od_usb_lock_due = true;
    }
  } else {
    s_od_usb_since    = 0;
    s_od_usb_lock_due = false;
  }
}

// SW を OFF にしたまま USB で充電している、の確定と読み上げ。
// ★ 読み上げは**この回に 1 度だけ**。一晩充電する場面で繰り返すとうるさい。画面には出し続ける。
// ★ 読み上げたあとに SW が入ったら charge_sw_on.wav で知らせる。同じ排他グループなので、
//   割り込まれて pending に残っていた「切れています」もここで捨てられる
//   （残すと SW を入れたあとに「切れています」と嘘を喋る。docs/pons_sound.md §3）。
static void sw_off_tick(uint32_t now) {
  if (!getReplayMode()) update_moving(now);   // リプレイ中の速度は過去のログなので見ない

  if (!power_sw_off_on_usb()) {
    s_swoff_since = 0;
    if (s_swoff) {
      // ここへ来るのは電池電圧が読めるようになったとき＝SW が入ったときだけ
      // （SW OFF のまま USB を抜けば本体ごと止まる）。
      s_swoff = false;
      enqueueTask(createLogSdfTask("CHARGE: SW ON (batt=%.2fV)", s_batt_v));
      if (s_swoff_said && !power_overdischarge_latched())
        enqueueTask(createPlayWavTask("wav/charge_sw_on.wav", 1, 0, WAV_EXCL_CHARGE_SW));
    }
    s_swoff_said = false;
    return;
  }
  if (!s_swoff) {
    if (s_swoff_since == 0) { s_swoff_since = now; return; }
    if (now - s_swoff_since < CHARGE_SW_OFF_HOLD_MS) return;
    s_swoff = true;
    enqueueTask(createLogSdTask("CHARGE: SW OFF on USB (battery cut off by Q3, will not charge fully)"));
  }
  // 優先度は最低（1）、最低音量なし。画面に出ているので、音量 0 の機体まで鳴らす話ではない。
  if (!s_swoff_said && now >= CHARGE_SW_OFF_ARM_MS && power_sw_off_notice_active()) {
    s_swoff_said = true;
    enqueueTask(createPlayWavTask("wav/charge_sw_off.wav", 1, 0, WAV_EXCL_CHARGE_SW));
  }
}

// 高温での充電（上の ■）。USB がつながっている間だけ見る。
// ★ 温度は get_airdata_temperature()（MS5611 の実測。リプレイでも差し替わらない）。
// ★ 電源経路の異常のあいだは見ない（usb_power_present() が false になる）。あちらが優先。
// ★ ミラー中・リプレイ中・デモ中も出す。**自機が熱い**という話なので、電源経路の異常と同じく
//   「ミラー中に自機の状態を混ぜない」の例外（docs/pons_power.md）。
static void charge_hot_tick(uint32_t now) {
  const bool watch = usb_power_present() && get_airdata_ok();
  const float t = watch ? get_airdata_temperature() : 0.0f;
  if (!watch) {
    if (s_hot) {
      const char* why = usb_power_present() ? "temperature sensor lost"
                      : (s_fault ? "power fault" : "USB removed");
      enqueueTask(createLogSdfTask("CHARGE HOT cleared (%s)", why));
    }
    s_hot = false;
    s_hot_since = 0;
    return;
  }
  s_hot_temp = t;
  if (!s_hot) {
    if (t < CHARGE_HOT_C) { s_hot_since = 0; return; }
    if (s_hot_since == 0) { s_hot_since = now; return; }
    if (now - s_hot_since < CHARGE_HOT_HOLD_MS) return;
    s_hot = true;
    s_hot_alert = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT: %.1fC (limit %.0fC while charging)", t, CHARGE_HOT_C));
  } else if (t <= CHARGE_HOT_CLEAR_C) {
    s_hot = false;
    s_hot_since = 0;
    enqueueTask(createLogSdfTask("CHARGE HOT cleared: %.1fC", t));
    return;
  }
  // 過放電ロック画面中は優先度 4 で「USB を外して」と鳴らしているので重ねない
  if (!s_od_screen_active &&
      (s_hot_alert == 0 || now - s_hot_alert >= CHARGE_HOT_ALERT_MS)) {
    s_hot_alert = now;
    // 「充電中の温度が高すぎます。USBを外して、涼しい場所で冷ましてください。」
    // ★ 最低音量 60: 電池の仕様外の充電で、発火の恐れにつながるので電源経路の異常と同じ扱い
    //   （docs/pons_sound.md §5）。
    enqueueTask(createPlayWavTask("wav/charge_hot.wav", 3, 60));
  }
}

void power_tick() {
  const uint32_t now = millis();
  if (now - s_last_sample < POWER_SAMPLE_MS) return;
  s_last_sample = now;
  sample();
  sw_off_tick(now);
  charge_hot_tick(now);
  overdischarge_tick(now);
  charge_log_tick(now);

  if (!s_fault) {
    if (now < POWER_FAULT_ARM_MS || !fault_condition()) {
      s_cond_since = 0;
    } else if (s_cond_since == 0) {
      s_cond_since = now;
    } else if (now - s_cond_since >= POWER_FAULT_HOLD_MS) {
      s_fault = true;
      s_last_alert = 0;
      enqueueTask(createLogSdfTask("POWER FAULT: VBUS=%.2fV (not USB, not 0)", s_volt));
    }
  }

  // 過放電の USB 完全ロック画面中は同じ優先度 4 で 30 秒おきに「USB を外して」と鳴らしているため、
  // 同優先度の power_fault.wav が文の途中で割り込んで互いを潰し合わないよう抑止する
  // （電池単体の警告画面中は 1 回しか鳴らさないため、充電 IC 故障があれば power_fault.wav を鳴らす）。
  if (s_fault && !(s_od_screen_active && power_overdischarge_usb_active()) &&
      (s_last_alert == 0 || now - s_last_alert >= POWER_FAULT_ALERT_MS)) {
    s_last_alert = now;
    // 「電源回路に異常があります。すぐにスイッチを切って、電池とUSBを外してください。」
    // ★ 文言は v7 と v7.1 の両方で正しい手順にしてある（v7 は電池を抜くしかなく、v7.1 は
    //   SW で切れる。画面の POWER_FAULT_ACTION_TEXT は基板ごとに違うが、音声はその和）。
    // ★ 最低音量 60 は電池警告だけの約束だが、これは**発火の恐れ**なので例外にする
    //   （音量 0 の機体でも聞こえないと意味が無い）。docs/pons_sound.md §5。
    enqueueTask(createPlayWavTask("wav/power_fault.wav", 4, 60));
  }
}

// ============================================================
//  USB 充電中の残量(%)・残り時間推定（docs/pons_charge_estimation.md）
// ============================================================
// 完全ステートレス物理モデル:
// 1) ロードシェアリング（Q2+D3）により、USB 給電中は本体負荷電流が電池経路に流れない。
// 2) ADC 測定点 BATPWR は F1(~0.25Ω) と Q3/Q4/保護回路(~0.25Ω) の中間点にある。
//    - CC モード (< 4.125V): 下流 IR ドロップ (~75mV) を減算してセル開放電圧を逆算。
//    - CV モード (>= 4.125V): 電流減衰により BATPWR は 4.125V〜4.200V へ上昇。
//      上流・下流が対称抵抗のため BATPWR = (4.20V + V_cell) / 2 となり、
//      V_cell = 2 * BATPWR - 4.20V で完全連続（4.125V で 4.05V、4.200V で 4.20V）に推計。
// 3) タイマーや状態変数を一切使わないため、途中から USB を挿しても再起動しても即座に正しく推定する。
float power_estimated_cell_voltage() {
  const float v = get_input_voltage();
  if (!usb_power_present() || power_sw_off_on_usb()) {
    return v;
  }
  if (v < 4.125f) {
    return v - 0.075f;
  } else {
    const float est = 2.0f * v - 4.20f;
    return (est > 4.20f) ? 4.20f : est;
  }
}

int power_charging_percent() {
  return battery_percent(power_estimated_cell_voltage());
}

int power_charging_remaining_min() {
  const int pct = power_charging_percent();
  if (pct >= 100) return 0;
  if (pct < 80) {
    // CC フェーズ: 80% (CV突入) までの時間 (303mA充電で約1.7分/%) ＋ CV 所要時間 (約45分)
    return (int)((80 - pct) * 1.7f + 45.0f);
  } else {
    // CV フェーズ: 残り 20% 分を 45分かけて指数減衰
    return (int)((100 - pct) * 2.25f);
  }
}

// USB 充電中の状態・残量推定ロジックの妥当性検証ログ（docs/pons_charge_estimation.md §7）
// 後から log.txt を確認して以下の物理特性を検証・チューニングできるように記録する:
//  1) 開始/途中挿し時の電圧・残量・推定残り時間（CHG START）
//  2) 60 秒ごとの推移（電圧上昇カーブ、温度と熱ダレの関係、残り時間カウントダウン）
//  3) CV モード突入点（4.125V）の到達タイミング（CHG CV ENTER）
//  4) 満充電（FULL）判定の到達タイミング（CHG FULL）
//  5) USB 抜去時の IR ドロップ測定値（充電中電圧 - 抜去後開放電圧 = 実測 ΔV）（CHG UNPLUG）
static void charge_log_tick(uint32_t now) {
  static bool     s_chg_active            = false;
  static uint32_t s_chg_start_ms          = 0;
  static uint32_t s_last_periodic_log_ms  = 0;
  static bool     s_cv_logged             = false;
  static bool     s_full_logged           = false;
  static float    s_last_chg_raw_v        = 0.0f;
  static int      s_last_chg_pct          = 0;
  static bool     s_unplug_pending        = false;
  static uint32_t s_unplug_ms             = 0;

  const bool charging_now = usb_power_present() && !power_sw_off_on_usb();

  // A) 充電開始（USB 接続 ＆ SW ON）
  if (charging_now) {
    s_unplug_pending = false; // 抜去待ち中の再接続はキャンセル
    if (!s_chg_active) {
      s_chg_active           = true;
      s_chg_start_ms         = now ? now : 1;
      s_last_periodic_log_ms = now;
      s_cv_logged            = false;
      s_full_logged          = false;
      const float raw_v   = s_batt_v;
      const float cell_v  = power_estimated_cell_voltage();
      const int   pct     = power_charging_percent();
      const int   rem_m   = power_charging_remaining_min();
      const float vbus    = s_volt;
      const float temp    = get_airdata_ok() ? get_airdata_temperature() : 0.0f;
      enqueueTask(createLogSdfTask("CHG START: vbus=%.2fV raw=%.2fV cell=%.2fV soc=%d%% rem=%dm T=%.1fC",
                                   vbus, raw_v, cell_v, pct, rem_m, temp));
    }
  }

  // B) 充電中の状態追跡とログ
  if (s_chg_active) {
    if (charging_now) {
      s_last_chg_raw_v = s_batt_v;
      s_last_chg_pct   = power_charging_percent();

      const float raw_v  = s_batt_v;
      const float cell_v = power_estimated_cell_voltage();
      const int   pct    = s_last_chg_pct;
      const int   rem_m  = power_charging_remaining_min();
      const float temp   = get_airdata_ok() ? get_airdata_temperature() : 0.0f;

      // 1) CV 突入イベント（4.125V 到達）
      if (!s_cv_logged && raw_v >= 4.125f) {
        s_cv_logged = true;
        const uint32_t elap_m = (now - s_chg_start_ms) / 60000UL;
        enqueueTask(createLogSdfTask("CHG CV ENTER: raw=%.2fV cell=%.2fV soc=%d%% rem=%dm t=%lum T=%.1fC",
                                     raw_v, cell_v, pct, rem_m, (unsigned long)elap_m, temp));
      }

      // 2) 満充電到達イベント
      if (!s_full_logged && pct >= 100) {
        s_full_logged = true;
        const uint32_t elap_m = (now - s_chg_start_ms) / 60000UL;
        enqueueTask(createLogSdfTask("CHG FULL: raw=%.2fV cell=%.2fV t=%lum T=%.1fC",
                                     raw_v, cell_v, (unsigned long)elap_m, temp));
      }

      // 3) 60 秒ごとの定周期スナップショット
      if (now - s_last_periodic_log_ms >= 60000UL) {
        s_last_periodic_log_ms = now;
        const uint32_t elap_m = (now - s_chg_start_ms) / 60000UL;
        const char* mode = (pct >= 100) ? "FULL" : (raw_v >= 4.125f ? "CV" : "CC");
        enqueueTask(createLogSdfTask("CHG [%lum]: raw=%.2fV cell=%.2fV soc=%d%% rem=%dm %s T=%.1fC",
                                     (unsigned long)elap_m, raw_v, cell_v, pct, rem_m, mode, temp));
      }
    } else {
      // 充電状態から抜けた（USB 抜去、または SW OFF）
      s_chg_active = false;
      // 5秒以上充電していた場合のみ抜去過渡測定に入る（ノイズやチャタリング防止）
      if (!power_sw_off_on_usb() && now - s_chg_start_ms >= 5000UL) {
        s_unplug_pending = true;
        s_unplug_ms      = now;
      }
    }
  }

  // C) USB 抜去直後の実測 IR ドロップ測定（抜去後 750ms で過渡現象が落ち着いた瞬間にサンプリング）
  if (s_unplug_pending && !power_sw_off_on_usb()) {
    if (now - s_unplug_ms >= 750UL) {
      s_unplug_pending = false;
      const float post_v = s_batt_v; // 抜去後の実測電池電圧（本体負荷接続時）
      const int   post_pct = battery_percent(post_v);
      const int   drop_mv = (int)lroundf((s_last_chg_raw_v - post_v) * 1000.0f);
      const uint32_t elap_m = (s_unplug_ms - s_chg_start_ms) / 60000UL;
      enqueueTask(createLogSdfTask("CHG UNPLUG: chg=%.2fV(%d%%) post=%.2fV(%d%%) dV=%dmV t=%lum",
                                   s_last_chg_raw_v, s_last_chg_pct, post_v, post_pct, drop_mv, (unsigned long)elap_m));
    }
  }
}

