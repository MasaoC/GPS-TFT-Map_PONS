// ============================================================
// File    : rtc_backup.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS のバックアップ電池（BT1 = MS621FE）が「あと何時間もつか」の推定（2026-11-08〜）。
//           v7 と v7.1 で同じ中身（どちらも BT1 は 3.3V から D5 を通して充電し、電圧を測る線は無い）。
//           v7.2（settings.h の PONS_MAINBOARD_REV = 72）は、電源を切っていても主電池が約 3.8V 以上なら主電池から
//           BT1 を充電する回路が付いたので、そのぶんを見込みに足す（下の ■ v7.2）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/16
// ============================================================
//
// ■ 何をするか
//   BT1 の残りを「あと何時間もつか」（H）で持つ。単位を時間にしておくと計算が簡単になる。
//   - 電源を切っている間: u-blox のバックアップ電流（約 28µA）はほぼ一定なので、**空いた時間をそのまま引く**。
//   - 電源を入れている間: 本体の 3.3V から充電されるので、満充電（RTC_BACKUP_FULL_H）へ時定数
//     RTC_BACKUP_TAU_H で近づける（H' = FULL − (FULL − H)·exp(−dt/τ)）。式に記憶が要らないので、
//     途中から何度足しても同じ結果になる。README の BT1 の表（Seiko の充電カーブと 28µA からの見積もり）に
//     合わせてある: 10 分 → 1.5h（表 2h）、1 時間 → 9h（表 10h）、3 時間 → 26h（表 24h）、6 時間 → 48h、12 時間 → 84h。
//   - 記録: SD の RTC_BACKUP_FILE に「H・書いた時刻（UTC）・本体の固有 ID」を、通電中は RTC_BACKUP_SAVE_MS ごとに書き直す
//     （書くのは Core1。mysd.cpp の rtc_backup_save_sd）。次の起動で GNSS の時刻が取れたら、その差を引く。
//     切る直前の最大 5 分ぶんの充電は記録に入らず、そのぶんの放電も多めに引く（安全側にずれる）。
//
// ■ 起動時の手がかり（計算のずれを毎回引き戻す。これがいちばん効く）
//   バックアップ電流は温度で変わり、電池の容量も年々減るので、足し引きだけでは少しずつずれる。
//   起動直後の最初の NAV-PVT（gnss_rtc_alive_on_boot）で、
//   - 時計が死んでいた → BT1 は確実に空。記録にかかわらず 0 から始める。
//   - 時計が生きていた → 少なくとも空ではない。計算が下回っていたら RTC_BACKUP_ALIVE_MIN_H まで引き上げる
//     （生きていても空に近いことがあるので、多くは見込まない）。
//   どちらでも、記録から出した見込みと合っていたかを "RTC BACKUP CHECK:" の行で log.txt に残す
//   （定数をあとで直すため。check_record()）。
//
// ■ 記録を使わないとき
//   SD が無い・記録が無い・別の機体の記録（SD を差し替えた）・壊れていた → 時計が生きていれば
//   RTC_BACKUP_ALIVE_MIN_H、死んでいれば 0 から始める（それまでの目安と同じく安全側）。
//
// ★ 時刻はリプレイ中・デモ中の GNSS の値を使わない（リプレイは CSV の日時を ubx_* へ書き込む）。
//   最初に取れた本物の時刻を millis() の 0 に当たる UTC として覚え、あとは millis() から出す。
// ★ 電圧は測っていない（BT1 は D5 を通して GNSS にだけつながっている）。あくまで推定。
//
// ■ v7.2（PONS_MAINBOARD_REV = 72。2026-11-16〜）
//   電源を切っていても、主電池が約 3.92V を超えると動き出し、約 3.83V まで下がると止まる回路（U6 MIC841・U7 RT9078・D9）が、
//   主電池から 3.3V で BT1 を充電し、GNSS の時計も保つ。回路が動いている間、BT1 は減らずに満充電へ近づく。
//   - 回路が動いているか（s_aon_on）: 本体が読む電池の電圧で、RTC_BACKUP_AON_ON_V 以上なら動いている、
//     RTC_BACKUP_AON_OFF_V 未満なら止まっている、間は直前のまま（回路のヒステリシスと同じ）。
//   - いま切っても回路が動き続ける時間（aon_hours_now()）: 電池の電圧から止まる電圧までの容量 ÷ RTC_BACKUP_AON_DRAIN_MA。
//     容量は典型的なリチウムポリマーの開放電圧と残量の表（soc_pct()）から出す。
//   - SD の記録にこの時間も書き（PONSRTC2 の 4 つめ）、次の起動で「切っていた時間」と比べる:
//       切っていた時間 ≤ その時間 → 回路は動き続けた。BT1 は切っていた間も充電された（charge()）
//       それより長い → 回路が止まるまで充電され、止まってから減った
//   - 充電ポップアップの見込み（rtc_backup_off_hours()）は「回路が動き続ける時間 ＋ そのあと BT1 がもつ時間」。
//   v7.1（と v7）は回路が無いので、この時間はいつも 0 で、今までと同じ動きになる。
//
// ■ 「速く測位できる」期間（2026-11-16〜）
//   時計が保たれていても、速く測位できるのは衛星の軌道のデータ（放送暦 約 4 時間・ANA の予測 約 3 日）が新しい間だけで、
//   その起点は最後に測位した時刻（s_fix_utc）。充電ポップアップの "Fast GPS" は、時計がもつ時間と「最後の測位から
//   RTC_BACKUP_FAST_H」の短いほうを出す（rtc_backup_fast_hours()）。最後に測位した時刻も SD の記録に書く（PONSRTC3 の 5 つめ）。
//   時計が死んだ起動では BBR の軌道のデータも消えているので、記録の値は使わず、その起動で測位するまで「分からない」（0）。
//   v7.2 で時計がそれより長く保たれるときは、時計がもつ見込みを "clock 3mo" のように別に添える（速くはない）。
#include "rtc_backup.h"
#include "settings.h"
#include "mysd.h"   // enqueueTask・createSaveRtcBackupTask・createLogSdfTask・taskQueueMutex
#include "gnss.h"   // gnss_rtc_boot_checked()・gnss_rtc_alive_on_boot()・get_gnss_date()/time()・リプレイ／デモ中か
#include "power.h"  // power_estimated_cell_voltage()・power_sw_off_on_usb()（v7.2 の回路が動いているかを電池の電圧で見る）

// v7.2 の回路があるか。PONS_MAINBOARD_REV が無いときも 0 になる
#if defined(PONS_MAINBOARD_REV) && PONS_MAINBOARD_REV >= 72
  #define RTC_BACKUP_HAS_AON 1
#else
  #define RTC_BACKUP_HAS_AON 0
#endif
#include <math.h>
#include <string.h>

// ---- SD から読んだ記録（Core1 が書き、Core0 が読む。taskQueueMutex で守る）----
static bool     s_load_done = false;
static bool     s_file_ok   = false;
static float    s_file_h    = 0.0f;
static float    s_file_aon  = 0.0f;   // 記録した時点で「切っても主電池で保てる」と見込んだ時間 [h]（v7.2。それ以外は 0）
static uint32_t s_file_utc  = 0;
static uint32_t s_file_fix  = 0;      // 記録にあった「最後に測位した時刻」（UTC 秒。0 = 分からない）

void rtc_backup_set_loaded_core1(bool ok, float hours, float aon_h, uint32_t utc, uint32_t fix_utc) {
  mutex_enter_blocking(&taskQueueMutex);
  s_file_ok   = ok;
  s_file_h    = hours;
  s_file_aon  = (aon_h > 0.0f) ? aon_h : 0.0f;
  s_file_utc  = utc;
  s_file_fix  = fix_utc;
  s_load_done = true;
  mutex_exit(&taskQueueMutex);
}

// ---- Core0 の状態 ----
static bool     s_known        = false;
// ★ double で持つ（2026-11-14）。float のまま 250ms ごとに 192h へ近づけると、1 回の増分が丸めの半分を下回る約 189.7h で
//   止まっていた（float の 1 目盛りは 190h 付近で約 1.5e-5h、増分は約 7.6e-6h）
static double   s_h            = 0.0;
static uint32_t s_last_ms      = 0;
static bool     s_utc_ok       = false;
static uint32_t s_utc0         = 0;   // millis() == 0 に当たる UTC [s]（＝起動した時刻）
static uint32_t s_last_save_ms = 0;
// 時刻が来ないまま待ちを打ち切り、記録を使わずに決めた（"no time"）。あとで時刻が取れたら記録から決め直す（BUG-82）
static bool     s_provisional  = false;

bool  rtc_backup_known() { return s_known; }
float rtc_backup_hours() { return (float)s_h; }

// 最後に自機の GNSS が測位した時刻（UTC 秒）。0 = 分からない。上の ■「速く測位できる」期間
static uint32_t s_fix_utc = 0;

// ---- v7.2 の回路（上の ■ v7.2）----
static bool  s_aon_on = false;   // 回路が動いている（と安全側に見て言える）
static float s_aon_v  = -1.0f;   // 最後に読めた電池の電圧 [V]。負 = まだ読めていない

#if RTC_BACKUP_HAS_AON
// 典型的なリチウムポリマー 1 セルの開放電圧 → 残量 [%]（0.2C 以下の放電、常温）。点のあいだは直線でつなぐ。
// ★ 表示の % とは別物（あちらは BAT_ZERO〜BAT_FULL の直線）。ここは「止まる電圧までに何 mAh 残っているか」を出すためのもの
static float soc_pct(float v) {
  static const float tv[] = { 3.50f, 3.60f, 3.70f, 3.75f, 3.80f, 3.85f, 3.90f, 3.95f, 4.00f, 4.10f, 4.20f };
  static const float tp[] = {  5.0f, 10.0f, 20.0f, 32.0f, 43.0f, 53.0f, 62.0f, 70.0f, 77.0f, 89.0f, 100.0f };
  const int n = (int)(sizeof(tv) / sizeof(tv[0]));
  if (v <= tv[0])     return tp[0];
  if (v >= tv[n - 1]) return tp[n - 1];
  for (int i = 1; i < n; i++) {
    if (v <= tv[i]) return tp[i - 1] + (tp[i] - tp[i - 1]) * (v - tv[i - 1]) / (tv[i] - tv[i - 1]);
  }
  return tp[n - 1];
}

// 電池の電圧を読んで、回路が動いているかを決め直す。
// ★ SW OFF のまま USB をつないでいる間は電池が切り離されていて読めない（power.h）。そのあいだは最後の値のまま
static void aon_update() {
  if (power_sw_off_on_usb()) return;
  const float v = power_estimated_cell_voltage();
  if (v < 2.5f) return;                 // 起動直後でまだ測れていない・電池が付いていない
  s_aon_v = v;
  if (v >= RTC_BACKUP_AON_ON_V)       s_aon_on = true;
  else if (v < RTC_BACKUP_AON_OFF_V)  s_aon_on = false;
}

// いま電源を切ったら、回路があと何時間動き続けるか [h]
static double aon_hours_now() {
  if (!s_aon_on || s_aon_v < 0.0f) return 0.0;
  const double mah = (double)RTC_BACKUP_AON_BATT_MAH *
                     (soc_pct(s_aon_v) - soc_pct(RTC_BACKUP_AON_OFF_V)) / 100.0;
  return (mah > 0.0) ? mah / (double)RTC_BACKUP_AON_DRAIN_MA : 0.0;
}
#else
static void   aon_update()    {}
static double aon_hours_now() { return 0.0; }
#endif

float rtc_backup_aon_hours() { return (float)aon_hours_now(); }

// 1970-01-01 からの日数（グレゴリオ暦。Howard Hinnant の days_from_civil）
static int32_t days_from_civil(int y, unsigned m, unsigned d) {
  y -= (m <= 2);
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

// 最初に取れた本物の時刻を覚える（上の ★）。
static void capture_utc(uint32_t now) {
  if (s_utc_ok || getReplayMode() || is_demo_active()) return;
  const GnssDate gd = get_gnss_date();
  const GnssTime gt = get_gnss_time();
  if (!gd.isValid() || !gt.isValid() || gd.year() < 2026) return;
  const uint32_t epoch = (uint32_t)days_from_civil(gd.year(), gd.month(), gd.day()) * 86400UL +
                         (uint32_t)gt.hour() * 3600UL + (uint32_t)gt.minute() * 60UL + gt.second();
  s_utc0  = epoch - now / 1000UL;
  s_utc_ok = true;
}

static double charge(double h, double dt_h) {
  if (dt_h <= 0.0) return h;
  return (double)RTC_BACKUP_FULL_H - ((double)RTC_BACKUP_FULL_H - h) * exp(-dt_h / (double)RTC_BACKUP_TAU_H);
}

// 電源を gap_h 時間切っていたあとの BT1 の残り [h]。aon_h は切ったときに見込んだ「主電池の回路が動き続ける時間」（v7.1 は 0）。
// 回路が動いている間は充電され（電源を入れているときと同じ 3.3V・220Ω なので同じ式）、止まってからは減る。
static double after_off(double h, double aon_h, double gap_h) {
  if (gap_h <= aon_h) return charge(h, gap_h);
  return charge(h, aon_h) - (gap_h - aon_h);
}

// いま電源を切ったら、GPS の時計が保たれるのはあと何時間か [h]（回路が動き続ける時間 ＋ そのあと BT1 がもつ時間）
float rtc_backup_off_hours() {
  const double a = aon_hours_now();
  return (float)(a + charge(s_h, a));
}

// いま電源を切ったら、入れ直して GPS が速く位置を出せるのはあと何時間か [h]（上の ■「速く測位できる」期間）
float rtc_backup_fast_hours() {
  if (s_fix_utc == 0 || !s_utc_ok) return 0.0f;
  const uint32_t now_utc = s_utc0 + millis() / 1000UL;
  const float age_h = (now_utc > s_fix_utc) ? (float)(now_utc - s_fix_utc) / 3600.0f : 0.0f;
  float left = RTC_BACKUP_FAST_H - age_h;
  const float off = rtc_backup_off_hours();
  if (left > off) left = off;
  return (left > 0.0f) ? left : 0.0f;
}

// 自機の GNSS が測位していれば、最後に測位した時刻をいまにする（リプレイ・デモ・GNSS の途切れは数えない）
static void fix_update(uint32_t now) {
  if (!s_utc_ok || getReplayMode() || is_demo_active() || !get_gnss_connection()) return;
  double lat, lon, gs, trk;
  if (!gnss_get_own_fix(lat, lon, gs, trk)) return;
  s_fix_utc = s_utc0 + now / 1000UL;
}

// 記録にあった「最後に測位した時刻」を使う。時計が生きていた起動だけ（死んでいたら BBR の軌道のデータも消えている）。
// この起動でもう測位していれば、そちらのほうが新しいので触らない
static void adopt_record_fix() {
  if (!gnss_rtc_alive_on_boot() || s_fix_utc != 0) return;
  uint32_t ff;
  mutex_enter_blocking(&taskQueueMutex);
  ff = s_file_fix;
  mutex_exit(&taskQueueMutex);
  s_fix_utc = ff;
}

// 起動したときの H を決める（上の ■ 起動時の手がかり）。決められないうちは false。
static bool resolve_boot(uint32_t now, float& h0, const char*& why, float& gap_h) {
  gap_h = -1.0f;
  if (!gnss_rtc_boot_checked()) return false;          // 最初の NAV-PVT を待つ
  if (!gnss_rtc_alive_on_boot()) { h0 = 0.0f; why = "rtc lost at boot"; return true; }

  bool done, ok; float fh, fa; uint32_t futc;
  mutex_enter_blocking(&taskQueueMutex);
  done = s_load_done; ok = s_file_ok; fh = s_file_h; fa = s_file_aon; futc = s_file_utc;
  mutex_exit(&taskQueueMutex);
  // 意図して直さない（2026-11-14 の監査）: 20 秒は起動からの millis()。起動が長引いて超えても、仮決め（s_provisional）にしてあとで決め直す
  const bool waiting = now < RTC_BACKUP_LOAD_WAIT_MS;
  if (!done && waiting) return false;                   // SD の記録を待つ
  if (ok && !s_utc_ok && waiting) return false;         // 時刻を待つ（時計が生きていればすぐ入る）

  if (ok && s_utc_ok) {
    gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
    h0 = (float)after_off(fh, fa, gap_h);
    // v7.2: 切っていた間に回路が止まらなかったなら、いまも動いている（電池の電圧を読めば決め直す。aon_update）
    if (RTC_BACKUP_HAS_AON && fa > gap_h) s_aon_on = true;
    adopt_record_fix();
    why = "record";
  } else {
    h0 = 0.0f;
    why = ok ? "no time" : "no record";
  }
  if (h0 < RTC_BACKUP_ALIVE_MIN_H) h0 = RTC_BACKUP_ALIVE_MIN_H;   // 生きていたので空ではない
  return true;
}

// ---- 記録と時計の生死の照らし合わせ（あとで定数を直す材料。2026-11-08〜）----
// 記録どおりなら起動したときに残っていたはずの量 est = 記録の H − 切っていた時間 と、実際に時計が
// 生きていたかを比べて、起動ごとに 1 回だけ log.txt に "RTC BACKUP CHECK:" の行を残す。推定そのものは変えない。
//   - 生きていたのに est が −RTC_BACKUP_CHECK_MARGIN_H より下 → "MISMATCH outlived"
//     （空のはずなのに残っていた＝見積もりが辛い。バックアップ電流が 28µA より少ない・充電が速い・容量が大きい）
//   - 死んでいたのに est が 0 より上 → "MISMATCH died early"
//     （残っているはずなのに切れていた＝見積もりが甘い。表示が安全側でなくなるので、こちらのほうが大事）
//   - それ以外は "ok"（合っていた例も、定数を絞り込む材料になる）
// 余裕を「生きていた」側にだけ取るのは、記録が最大 RTC_BACKUP_SAVE_MS 古いぶん est が低めに出るから
// （空から 5 分の充電で約 0.8h）。est が低めに出ても「死んでいた」側の判定は甘くならない。
// 末尾の x は「実際にもった時間 ÷ 記録の H」。生きていたときは「少なくともこれだけ」、死んでいたときは「多くてもこれだけ」。
// 時計が死んでいたときは時刻が衛星から入るまで（fix まで）待つ。記録が無いときは比べるものが無いので残さない。
static bool s_checked = false;

static void check_record() {
  if (s_checked || !gnss_rtc_boot_checked()) return;
  bool done, ok; float fh, fa; uint32_t futc;
  mutex_enter_blocking(&taskQueueMutex);
  done = s_load_done; ok = s_file_ok; fh = s_file_h; fa = s_file_aon; futc = s_file_utc;
  mutex_exit(&taskQueueMutex);
  if (!done) return;
  if (!ok) { s_checked = true; return; }
  if (!s_utc_ok) return;
  s_checked = true;

  const bool  alive = gnss_rtc_alive_on_boot();
  const float gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
  const float est   = (float)after_off(fh, fa, gap_h);
  const char* verdict = "ok";
  if (alive && est < -RTC_BACKUP_CHECK_MARGIN_H) verdict = "MISMATCH outlived";
  if (!alive && est > 0.0f)                      verdict = "MISMATCH died early";
  char ratio[16];
  if (fh >= 0.1f) snprintf(ratio, sizeof(ratio), "%s x%.2f", alive ? ">=" : "<", gap_h / fh);
  else            snprintf(ratio, sizeof(ratio), "x--");
  enqueueTask(createLogSdfTask("RTC BACKUP CHECK: %s (rtc %s, rec %.1fh, aon %.0fh, off %.1fh, est %+.1fh, lasted %s)",
                               verdict, alive ? "alive" : "lost", fh, fa, gap_h, est, ratio));
}

void rtc_backup_tick() {
  const uint32_t now = millis();
  capture_utc(now);
  check_record();
  aon_update();
  fix_update(now);

  if (!s_known) {
    float h0, gap_h; const char* why;
    if (!resolve_boot(now, h0, why, gap_h)) return;
    s_h       = charge(h0, now / 3600000.0f);   // 起動してからいままでの通電ぶんを足す
    s_known   = true;
    s_last_ms = now;
    // 仮に決めた: 時刻が来なかった（"no time"）か、SD の記録をまだ読み終えていなかった（"no record"）。あとで決め直す
    s_provisional = (strcmp(why, "no time") == 0) || (strcmp(why, "no record") == 0);
    char off[16];
    if (gap_h >= 0.0f) snprintf(off, sizeof(off), "%.1fh", gap_h);
    else               snprintf(off, sizeof(off), "?");
    char fixago[16];
    if (s_fix_utc != 0 && s_utc_ok) snprintf(fixago, sizeof(fixago), "%.1fh", (float)((s_utc0 + now / 1000UL) - s_fix_utc) / 3600.0f);
    else                            snprintf(fixago, sizeof(fixago), "?");
    enqueueTask(createLogSdfTask("RTC BACKUP: boot %.1fh (%s, off %s) -> now %.1fh board=%d aon=%s lastfix %s ago",
                                 h0, why, off, s_h, RTC_BACKUP_HAS_AON ? 72 : 71,
                                 RTC_BACKUP_HAS_AON ? (s_aon_on ? "on" : "off") : "-", fixago));
    return;
  }

  // ★ "no time" で仮に決めたあと時刻が取れたら、記録から決め直す（2026-11-13、BUG-82）。
  //   時計は生きていたのに起動から RTC_BACKUP_LOAD_WAIT_MS（20 秒）のうちに時刻が取れなかったとき（起動時のデモ・
  //   日付だけ有効で時刻が無効、など）、以前は 0.5h に決めたまま、時刻が取れた時点で SD の記録をその値で上書きしていた。
  //   書くのは時刻が取れてからなので（下）、この回で決め直せば記録は失われない。
  //   SD の記録を 20 秒のうちに読み終えなかった（"no record"）ときも同じ（2026-11-14）。読み終えて記録が無ければそのまま。
  if (s_provisional) {
    bool done, ok; float fh, fa; uint32_t futc;
    mutex_enter_blocking(&taskQueueMutex);
    done = s_load_done; ok = s_file_ok; fh = s_file_h; fa = s_file_aon; futc = s_file_utc;
    mutex_exit(&taskQueueMutex);
    if (done && !ok) s_provisional = false;          // 記録は無かった。いまの値のままでよい
    else if (done && ok && s_utc_ok) {
      s_provisional = false;
      const float gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
      float h0 = (float)after_off(fh, fa, gap_h);
      if (RTC_BACKUP_HAS_AON && fa > gap_h) s_aon_on = true;
      adopt_record_fix();
      if (h0 < RTC_BACKUP_ALIVE_MIN_H) h0 = RTC_BACKUP_ALIVE_MIN_H;
      s_h       = charge(h0, now / 3600000.0f);
      s_last_ms = now;
      enqueueTask(createLogSdfTask("RTC BACKUP: boot %.1fh (record, time late, off %.1fh) -> now %.1fh",
                                   h0, gap_h, s_h));
      return;
    }
  }

  s_h = charge(s_h, (now - s_last_ms) / 3600000.0f);
  s_last_ms = now;

  // ★ PC 接続中は書きに行かずに待つ。mysd.cpp の rtc_backup_save_sd() は PC 接続中は何もせずに戻る
  //   （SD を読み出し専用に保つため）ので、ここで「書いた」ことにすると次は RTC_BACKUP_SAVE_MS 後になる。
  //   2026-11-11 まではそうなっていて、PC の時刻（PONS:TIME）で時刻が取れた起動では最初の 1 回が必ず
  //   PC 接続中に当たって捨てられ、PC を抜いて 5 分以内に切ると記録が残らなかった
  //   （実機の log.txt で次の起動が `RTC BACKUP record: none`）。待っておけば PC を抜いた直後に書く。
  //   キューが満杯で捨てられたときも書いたことにしない（次の周回でもう一度。power_tick() の周期）。
  // ★ 仮に決めている間は書かない（記録を読む前・時刻を待っている間に、仮の値で記録を上書きしない）
  if (s_utc_ok && !webserial_connected && !s_provisional &&
      (s_last_save_ms == 0 || now - s_last_save_ms >= RTC_BACKUP_SAVE_MS)) {
    if (enqueueTask(createSaveRtcBackupTask((float)s_h, (float)aon_hours_now(), s_utc0 + now / 1000UL, s_fix_utc)))
      s_last_save_ms = now ? now : 1;
  }
}

// 充電ポップアップの 2 行目。「いま電源を切っても、この時間より前に入れ直せば GPS がすぐ位置を出す」を文で出す。
// ★ 2026-11-10 までは "RTC: ~1.5d" で、初めての人には何の時間なのか分からなかった（RTC も d も略語）。
//   箱の内側は 217px（AA_FONT_SMALL）。最長は "Fast GPS if OFF < 23 hours" 付近の約 192px（26 文字）。
// ★ 安全側に切り捨てる。2 時間未満はまとめて "< 1 hour"。72 時間未満は hours、それ以上は days。
// ★ 2026-11-16 から、時間は rtc_backup_fast_hours()（時計がもつ時間と、最後の測位から RTC_BACKUP_FAST_H の短いほう）。
//   それまでは時計（BT1）がもつ時間をそのまま出していて、最長 "< 8 days" と、軌道のデータが切れたあと（約 3 日）も速いと言っていた。
// ★ v7.2 で時計がそれより 1 日以上長く保たれるときは、短い形で時計の見込みを添える: "Fast GPS <2d, clock 3mo"
//   （速いのは 2 日まで。時計は 3 か月保たれる＝速くはないが、時刻を知っているぶんは早い）。
// 期間を短い形にする（"47h" / "2d" / "3mo"）。2 時間未満は "1h"、72 時間未満は h、60 日未満は d、それ以上は mo（12 で頭打ち）
static void fmt_short(char* b, size_t n, float h) {
  if (h < 2.0f)                snprintf(b, n, "1h");
  else if (h < 72.0f)          snprintf(b, n, "%dh", (int)h);
  else if (h < 60.0f * 24.0f)  snprintf(b, n, "%dd", (int)(h / 24.0f));
  else {
    int mo = (int)(h / (30.0f * 24.0f));
    if (mo > RTC_BACKUP_AON_SHOW_MAX_MONTHS) mo = RTC_BACKUP_AON_SHOW_MAX_MONTHS;
    snprintf(b, n, "%dmo", mo);
  }
}

void rtc_backup_format(char* buf, size_t n) {
  if (!s_known) { snprintf(buf, n, "Fast GPS: checking..."); return; }
  const float fast  = rtc_backup_fast_hours();
  const float clock = rtc_backup_off_hours();
  // v7.2 で回路が動いていて、時計が「速い期間」より 1 日以上長く保たれる → 時計の見込みを添える
  if (RTC_BACKUP_HAS_AON && s_aon_on && clock >= fast + 24.0f) {
    char c[8];
    fmt_short(c, sizeof(c), clock);
    if (fast <= 0.0f) {
      snprintf(buf, n, "No fast GPS, clock %s", c);
    } else {
      char f[8];
      fmt_short(f, sizeof(f), fast);
      snprintf(buf, n, "Fast GPS <%s, clock %s", f, c);
    }
    return;
  }
  if (fast <= 0.0f) {
    snprintf(buf, n, "Fast GPS: get a fix first");   // 最後に測位した時刻が分からない・軌道のデータが古い
  } else if (fast < 2.0f) {
    snprintf(buf, n, "Fast GPS if OFF < 1 hour");
  } else if (fast < 72.0f) {
    snprintf(buf, n, "Fast GPS if OFF < %d hours", (int)fast);
  } else {
    snprintf(buf, n, "Fast GPS if OFF < %d days", (int)(fast / 24.0f));
  }
}
