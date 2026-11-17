// ============================================================
// File    : rtc_backup.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS のバックアップ電池（BT1 = MS621FE）が「あと何時間もつか」の推定。
//           通電した時間で足し、電源を切っていた時間で引く。記録は SD の rtc_backup.txt。
//           充電ポップアップの "Fast GPS if OFF < 36 hours" と 60 秒ログの bk= / aon= に使う。
//           v7.2 基板（settings.h の PONS_MAINBOARD_REV = 72）は、電源を切っていても主電池が BT1 を保つぶんを足す。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/16
// ============================================================
#ifndef RTC_BACKUP_H
#define RTC_BACKUP_H

#include <Arduino.h>

// ---- Core0 ----
void  rtc_backup_tick();     // power_tick() から 250ms ごとに呼ぶ
bool  rtc_backup_known();    // 推定が決まったか（最初の NAV-PVT と SD の記録を待つ間は false）
float rtc_backup_hours();    // BT1 だけで、あと何時間もつか [h]（推定）
// v7.2: いま電源を切っても、主電池の回路が BT1 を保てる時間 [h]（推定。回路が止まっている・v7.1 は 0）
float rtc_backup_aon_hours();
// いま電源を切ったら、GPS がすぐ位置を出せるのはあと何時間か [h]（= 主電池で保てる時間 ＋ そのあと BT1 がもつ時間）。
// v7.1 は rtc_backup_hours() と同じ。
float rtc_backup_off_hours();
// いま電源を切って入れ直したら、GPS が**速く**位置を出せるのはあと何時間か [h]。
// = rtc_backup_off_hours() と「最後に測位してから RTC_BACKUP_FAST_H」の短いほう（軌道のデータが古くなると時計があっても遅い）。
// 最後に測位した時刻が分からない（時計が死んだ起動でまだ測位していない）ときは 0。
float rtc_backup_fast_hours();
// 充電ポップアップの 2 行目の文（1 行まるごと）。安全側に切り捨てる。
// "Fast GPS: checking..."（まだ分からない）/ "Fast GPS: get a fix first"（速く測位できる期間が無い）/
// "Fast GPS if OFF < 1 hour" / "... < 2..71 hours" / "... < 3 days"（rtc_backup_fast_hours()。最長 3 日）。
// v7.2 で時計がそれより 1 日以上長く保たれるときは "Fast GPS <2d, clock 3mo" / "No fast GPS, clock 3mo" の形。
// n は 32 あれば足りる（最長 26 文字）。
void  rtc_backup_format(char* buf, size_t n);

// ---- Core1（mysd.cpp の SD の読み書きから）----
// 起動時に SD の記録を読み終えたら呼ぶ。読めなかった・別の機体の記録・壊れていたときも ok=false で必ず呼ぶ
// （Core0 は待ちの上限 RTC_BACKUP_LOAD_WAIT_MS までこれを待つ）。
// aon_h は記録した時点で「切っても主電池で保てる」と見込んだ時間（v7.1・古い形式の記録は 0）。
// fix_utc は最後に測位した時刻（UTC 秒。0 = 分からない・古い形式の記録）。
void  rtc_backup_set_loaded_core1(bool ok, float hours, float aon_h, uint32_t utc, uint32_t fix_utc);

#endif // RTC_BACKUP_H
