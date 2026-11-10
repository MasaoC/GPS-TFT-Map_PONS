// ============================================================
// File    : rtc_backup.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS のバックアップ電池（BT1 = MS621FE）が「あと何時間もつか」の推定。
//           通電した時間で足し、電源を切っていた時間で引く。記録は SD の rtc_backup.txt。
//           充電ポップアップの "Fast GPS if OFF < 1.5 days" と 60 秒ログの bk= に使う。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/10
// ============================================================
#ifndef RTC_BACKUP_H
#define RTC_BACKUP_H

#include <Arduino.h>

// ---- Core0 ----
void  rtc_backup_tick();     // power_tick() から 250ms ごとに呼ぶ
bool  rtc_backup_known();    // 推定が決まったか（最初の NAV-PVT と SD の記録を待つ間は false）
float rtc_backup_hours();    // あと何時間もつか [h]（推定）
// 充電ポップアップの 2 行目の文（1 行まるごと）。安全側に切り捨てる。
// "Fast GPS: checking..."（まだ分からない）/ "Fast GPS if OFF < 1 hour" / "... < 18 hours" / "... < 1.5 days" / "... < 8 days"。
// n は 32 あれば足りる（最長 26 文字）。
void  rtc_backup_format(char* buf, size_t n);

// ---- Core1（mysd.cpp の SD の読み書きから）----
// 起動時に SD の記録を読み終えたら呼ぶ。読めなかった・別の機体の記録・壊れていたときも ok=false で必ず呼ぶ
// （Core0 は待ちの上限 RTC_BACKUP_LOAD_WAIT_MS までこれを待つ）。
void  rtc_backup_set_loaded_core1(bool ok, float hours, uint32_t utc);

#endif // RTC_BACKUP_H
