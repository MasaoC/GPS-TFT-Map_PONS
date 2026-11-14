// ============================================================
// File    : rtc_backup.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS のバックアップ電池（BT1 = MS621FE）が「あと何時間もつか」の推定（2026-11-08〜）。
//           v7 と v7.1 で同じ中身（どちらも BT1 は 3.3V から D5 を通して充電し、電圧を測る線は無い）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/11
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
#include "rtc_backup.h"
#include "settings.h"
#include "mysd.h"   // enqueueTask・createSaveRtcBackupTask・createLogSdfTask・taskQueueMutex
#include "gnss.h"   // gnss_rtc_boot_checked()・gnss_rtc_alive_on_boot()・get_gnss_date()/time()・リプレイ／デモ中か
#include <math.h>
#include <string.h>

// ---- SD から読んだ記録（Core1 が書き、Core0 が読む。taskQueueMutex で守る）----
static bool     s_load_done = false;
static bool     s_file_ok   = false;
static float    s_file_h    = 0.0f;
static uint32_t s_file_utc  = 0;

void rtc_backup_set_loaded_core1(bool ok, float hours, uint32_t utc) {
  mutex_enter_blocking(&taskQueueMutex);
  s_file_ok   = ok;
  s_file_h    = hours;
  s_file_utc  = utc;
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

// 起動したときの H を決める（上の ■ 起動時の手がかり）。決められないうちは false。
static bool resolve_boot(uint32_t now, float& h0, const char*& why, float& gap_h) {
  gap_h = -1.0f;
  if (!gnss_rtc_boot_checked()) return false;          // 最初の NAV-PVT を待つ
  if (!gnss_rtc_alive_on_boot()) { h0 = 0.0f; why = "rtc lost at boot"; return true; }

  bool done, ok; float fh; uint32_t futc;
  mutex_enter_blocking(&taskQueueMutex);
  done = s_load_done; ok = s_file_ok; fh = s_file_h; futc = s_file_utc;
  mutex_exit(&taskQueueMutex);
  // 意図して直さない（2026-11-14 の監査）: 20 秒は起動からの millis()。起動が長引いて超えても、仮決め（s_provisional）にしてあとで決め直す
  const bool waiting = now < RTC_BACKUP_LOAD_WAIT_MS;
  if (!done && waiting) return false;                   // SD の記録を待つ
  if (ok && !s_utc_ok && waiting) return false;         // 時刻を待つ（時計が生きていればすぐ入る）

  if (ok && s_utc_ok) {
    gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
    h0 = fh - gap_h;
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
  bool done, ok; float fh; uint32_t futc;
  mutex_enter_blocking(&taskQueueMutex);
  done = s_load_done; ok = s_file_ok; fh = s_file_h; futc = s_file_utc;
  mutex_exit(&taskQueueMutex);
  if (!done) return;
  if (!ok) { s_checked = true; return; }
  if (!s_utc_ok) return;
  s_checked = true;

  const bool  alive = gnss_rtc_alive_on_boot();
  const float gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
  const float est   = fh - gap_h;
  const char* verdict = "ok";
  if (alive && est < -RTC_BACKUP_CHECK_MARGIN_H) verdict = "MISMATCH outlived";
  if (!alive && est > 0.0f)                      verdict = "MISMATCH died early";
  char ratio[16];
  if (fh >= 0.1f) snprintf(ratio, sizeof(ratio), "%s x%.2f", alive ? ">=" : "<", gap_h / fh);
  else            snprintf(ratio, sizeof(ratio), "x--");
  enqueueTask(createLogSdfTask("RTC BACKUP CHECK: %s (rtc %s, rec %.1fh, off %.1fh, est %+.1fh, lasted %s)",
                               verdict, alive ? "alive" : "lost", fh, gap_h, est, ratio));
}

void rtc_backup_tick() {
  const uint32_t now = millis();
  capture_utc(now);
  check_record();

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
    enqueueTask(createLogSdfTask("RTC BACKUP: boot %.1fh (%s, off %s) -> now %.1fh",
                                 h0, why, off, s_h));
    return;
  }

  // ★ "no time" で仮に決めたあと時刻が取れたら、記録から決め直す（2026-11-13、BUG-82）。
  //   時計は生きていたのに起動から RTC_BACKUP_LOAD_WAIT_MS（20 秒）のうちに時刻が取れなかったとき（起動時のデモ・
  //   日付だけ有効で時刻が無効、など）、以前は 0.5h に決めたまま、時刻が取れた時点で SD の記録をその値で上書きしていた。
  //   書くのは時刻が取れてからなので（下）、この回で決め直せば記録は失われない。
  //   SD の記録を 20 秒のうちに読み終えなかった（"no record"）ときも同じ（2026-11-14）。読み終えて記録が無ければそのまま。
  if (s_provisional) {
    bool done, ok; float fh; uint32_t futc;
    mutex_enter_blocking(&taskQueueMutex);
    done = s_load_done; ok = s_file_ok; fh = s_file_h; futc = s_file_utc;
    mutex_exit(&taskQueueMutex);
    if (done && !ok) s_provisional = false;          // 記録は無かった。いまの値のままでよい
    else if (done && ok && s_utc_ok) {
      s_provisional = false;
      const float gap_h = (s_utc0 > futc) ? (float)(s_utc0 - futc) / 3600.0f : 0.0f;
      float h0 = fh - gap_h;
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
    if (enqueueTask(createSaveRtcBackupTask((float)s_h, s_utc0 + now / 1000UL)))
      s_last_save_ms = now ? now : 1;
  }
}

// 充電ポップアップの 2 行目。「いま電源を切っても、この時間より前に入れ直せば GPS がすぐ位置を出す」を文で出す。
// ★ 2026-11-10 までは "RTC: ~1.5d" で、初めての人には何の時間なのか分からなかった（RTC も d も略語）。
//   箱の内側は 217px（AA_FONT_SMALL）。最長は "Fast GPS if OFF < 23 hours"〜"71 hours" 付近の約 192px。
// ★ 安全側に切り捨てる。2 時間未満はまとめて "< 1 hour"（時計が死んだ起動から充電して最初の 10 分ほどだけ）。
//   72 時間未満（2〜71 時間）は hours 表記、72 時間以上（3 日以上）は days 表記にする。
void rtc_backup_format(char* buf, size_t n) {
  if (!s_known) { snprintf(buf, n, "Fast GPS: checking..."); return; }
  const float h = (float)s_h;
  if (h < 2.0f) {
    snprintf(buf, n, "Fast GPS if OFF < 1 hour");
  } else if (h < 72.0f) {
    snprintf(buf, n, "Fast GPS if OFF < %d hours", (int)h);
  } else {
    snprintf(buf, n, "Fast GPS if OFF < %d days", (int)(h / 24.0f));
  }
}
