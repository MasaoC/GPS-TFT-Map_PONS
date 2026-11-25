// ============================================================
// File    : live.cpp
// Project : PONS v7.1 / v7.2
// Role    : ライブ配信（live.h の説明）。行のリング・止める条件・行の組み立て。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/24
// ============================================================
//
// ■ 流す行（docs/pons_webserial_protocol.md §4 (9)）。**Re:PONS（live.js）が使う欄だけ**を載せる（使わない欄は足さない）
//   PONS:LIVE,OK,ON,<形の版>,<LINK_PROTO_VER>,<sizeof(LinkTelem)>,<group>,<ch>,<sf>,<送る間隔ms>,<BUILDVERSION>
//   PONS:LIVE,DEST,<番号>,<緯度>,<経度>,<名前>                   受信機の目的地の一覧（パイロンの上書きの後）
//   PONS:LIVE,PILON,<PLA 緯度>,<経度>,<北>,<>,<南>,<>,<1km>,<>,<多景島>,<>
//   PONS:LIVE,MAP,<F|X>,<番号>,<点数>,<名前>                     ポリゴン地図（F = 内蔵、X = SD の mapdata.csv）
//   PONS:LIVE,MP,<F|X>,<番号>,<最初の点の番号>,<緯度>,<経度>,...  その点（1 行に 7 点まで）
//   PONS:LIVE,DUMP_END,<目的地の数>,<地図の数>
//   PONS:LT,<ms>,<回転枠>,<受けた回転枠 hex>,<LinkTelem 69 バイト hex>
//   PONS:LN,<seq>,<flags hex>,<hdr>,<strike>,<wait>,<egs>,<marker>,<last_tone_tt>,<steer>,<cwi>,<deg/s>,
//           <deadband>,<目的地>,<ナビモード>,<Auto10km>,<truec>,<目的地まで km>,<MaxGS>,<時>,<分>,<5分の MaxGS>,<時>,<分>
//   PONS:LS,<flags hex>,<最後に受けてからの ms>,<直近 10 秒の RSSI>,<本数>,<受けた数>,<落ちた数>,
//           <機体までの距離 m>,<送る間隔 ms>,<書いた行>,<捨てた行>
//   PONS:LIVE,OFF,<理由>
//
// ■ リング
//   1 行ずつ「長さ 1 バイト＋中身（\r\n 込み）」で積む。溢れたら古い行から捨てる（捨てた数を数える）。
//   書くのは Serial.availableForWrite() に行がまるごと入るときだけ（arduino-pico の write() は入る分しか
//   書かないので、それなら待たない）。目的地と地図の一覧はリングに積まず、リングが空のときに 1 行ずつ書く
//   （一覧は数 KB あってリングに入りきらない。捨てると地図が欠けるので、捨てずに待つ）。
#include "live.h"

#include "settings.h"
#include "link.h"
#include "gnss.h"
#include "tas.h"
#include "display_tft.h"   // live_collect_screen()（画面の判断）・LiveScreen
#include "mysd.h"          // createLogSdfTask / enqueueTask
#include "e220.h"          // e220_profile_to_sf
#include "src/navdata.h"

// ---- .ino と sound.cpp の大域（画面とナビの判断。どれも Core0 が書いて Core0 が読む）----
extern double        steer_angle;
extern float         degpersecond;
extern float         course_warning_index;
extern unsigned long last_course_warning_time;
extern unsigned long trackwarning_until;
extern float         last_tone_tt;
extern volatile int  vario_volume;

#define LIVE_RING_BYTES 2048   // 2KB。1 秒に約 0.5KB 流すので、PC が 3〜4 秒読まないと捨て始める
#define LIVE_LINE_MAX    224   // 1 行の上限（\r\n 込み）。リングの長さは 1 バイトで持つので 255 以下
#define LIVE_MARK_OK_MS 3000   // この間に 1 行でも書けていれば緑（設計書 §4.6）
#define LIVE_LS_MS      1000   // 受信機の状態の行の間隔

static bool     s_on = false;
static char     s_ring[LIVE_RING_BYTES];
static uint16_t s_rHead = 0, s_rTail = 0, s_rUsed = 0;   // 書く位置・読む位置・使っているバイト
static uint32_t s_lines = 0, s_dropped = 0;              // 書いた行・捨てた行（LIVE を始めてから）
static uint32_t s_minLines = 0, s_minDropped = 0;        // この 1 分（60 秒ログ）
static bool     s_minActive = false;                     // この 1 分に動いていたか
static uint16_t s_maxUsed = 0;                           // この 1 分のリングの最大
static uint32_t s_lastWriteMs = 0;                       // 最後に行を書けた時刻
static uint32_t s_lsMs = 0;
static bool     s_lnPending = false;                     // 新しいフレームを受けた（次の画面のあとで LN を作る）
static uint16_t s_lnSeq = 0;
static uint32_t s_portDownMs = 0;                        // `Serial` が false になった時刻（0 = いまはつながっている）

// ---- 目的地・パイロン・地図の一覧（LIVE を始めたときに 1 回流す）----
enum LiveDump : uint8_t { DUMP_NONE, DUMP_DEST, DUMP_PILON, DUMP_FMAP, DUMP_XMAP, DUMP_END };
static LiveDump s_dump = DUMP_NONE;
static int      s_dumpIdx = 0;    // 目的地・地図の番号
static int      s_dumpPt  = -1;   // 地図の中の点（-1 = 頭の MAP の行をまだ書いていない）
static int      s_dumpMaps = 0;   // 流した地図の数

bool live_active() { return s_on; }

// ---- リング ----
static void ring_drop_oldest() {
  if (s_rUsed == 0) return;
  const uint8_t len = (uint8_t)s_ring[s_rTail];
  const uint16_t rec = (uint16_t)(1 + len);
  s_rTail = (uint16_t)((s_rTail + rec) % LIVE_RING_BYTES);
  s_rUsed = (uint16_t)(s_rUsed - rec);
  s_dropped++;
  s_minDropped++;
}

// 1 行を積む（line は \r\n を含まない）。長すぎる行は積まない（作る側のまちがい）。
static void ring_push(const char* line) {
  const size_t n = strlen(line);
  if (n + 2 > LIVE_LINE_MAX) return;
  const uint8_t len = (uint8_t)(n + 2);
  const uint16_t rec = (uint16_t)(1 + len);
  while (LIVE_RING_BYTES - s_rUsed < rec) ring_drop_oldest();
  auto put = [](char c) {
    s_ring[s_rHead] = c;
    s_rHead = (uint16_t)((s_rHead + 1) % LIVE_RING_BYTES);
  };
  put((char)len);
  for (size_t i = 0; i < n; i++) put(line[i]);
  put('\r');
  put('\n');
  s_rUsed = (uint16_t)(s_rUsed + rec);
  if (s_rUsed > s_maxUsed) s_maxUsed = s_rUsed;
}

// 先頭の 1 行を取り出す（buf に \r\n 込みで入れ、長さを返す）。取り出すだけで消さない。
static uint8_t ring_peek(char* buf) {
  if (s_rUsed == 0) return 0;
  const uint8_t len = (uint8_t)s_ring[s_rTail];
  uint16_t p = (uint16_t)((s_rTail + 1) % LIVE_RING_BYTES);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = s_ring[p];
    p = (uint16_t)((p + 1) % LIVE_RING_BYTES);
  }
  return len;
}
static void ring_pop() {
  if (s_rUsed == 0) return;
  const uint8_t len = (uint8_t)s_ring[s_rTail];
  const uint16_t rec = (uint16_t)(1 + len);
  s_rTail = (uint16_t)((s_rTail + rec) % LIVE_RING_BYTES);
  s_rUsed = (uint16_t)(s_rUsed - rec);
}
static void ring_clear() { s_rHead = s_rTail = s_rUsed = 0; }

// 行がまるごと入るときだけ書く（待たない）。書けたら true。
static bool write_whole(const char* buf, size_t len) {
  if ((size_t)Serial.availableForWrite() < len) return false;
  const size_t w = Serial.write((const uint8_t*)buf, len);
  if (w != len) return false;   // ポートが閉じた瞬間など（次の live_tick が止める）
  s_lines++;
  s_minLines++;
  s_lastWriteMs = millis();
  return true;
}

// リングを書けるだけ書く
static void ring_flush() {
  char buf[LIVE_LINE_MAX];
  while (s_rUsed > 0) {
    const uint8_t len = ring_peek(buf);
    if (!write_whole(buf, len)) return;
    ring_pop();
  }
}

static void hex_append(char* out, const uint8_t* p, size_t n) {
  static const char H[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    out[2 * i]     = H[p[i] >> 4];
    out[2 * i + 1] = H[p[i] & 0x0F];
  }
  out[2 * n] = '\0';
}

// ---- 目的地・パイロン・地図の一覧 ----
// 1 行作って書く。書けなければ false（次の周回で同じ行を作り直す）。
static bool dump_write(const char* line) {
  char buf[LIVE_LINE_MAX];
  const size_t n = strlen(line);
  if (n + 2 > sizeof(buf)) return true;   // 長すぎる行は飛ばす（作る側のまちがい。止まらないように）
  memcpy(buf, line, n);
  buf[n] = '\r';
  buf[n + 1] = '\n';
  return write_whole(buf, n + 2);
}

// 名前に区切りのカンマ・改行が入っていたら置き換える（行の形を壊さない）。40 文字まで。
static void safe_name(char* out, size_t n, const char* name) {
  size_t i = 0;
  if (name) {
    for (; name[i] && i + 1 < n && i < 40; i++) {
      const char c = name[i];
      out[i] = (c == ',' || c == '\r' || c == '\n') ? ' ' : c;
    }
  }
  out[i] = '\0';
}

// 地図（F = 内蔵の flashmaps[]、X = SD の extramaps[]）の i 番目。無ければ nullptr。
static const mapdata* dump_map_at(LiveDump kind, int i) {
  if (kind == DUMP_FMAP) return (i < flashmap_count) ? flashmaps[i] : nullptr;
  return (i < mapdata_count) ? &extramaps[i] : nullptr;
}

// 一覧を 1 行ずつ書く。リングが空のときだけ（受けたテレメトリの行を一覧の後ろに長く待たせない）。
static void dump_step() {
  for (int guard = 0; guard < 8 && s_dump != DUMP_NONE; guard++) {
    if (s_rUsed > 0) return;
    char line[LIVE_LINE_MAX];
    switch (s_dump) {
      case DUMP_DEST: {
        // ★ destinations_count は Core1 の init_destinations() が最後に公開する（navdata.cpp）。数え直さずにそのつど読む
        if (s_dumpIdx >= destinations_count) { s_dump = DUMP_PILON; break; }
        const mapdata& d = extradestinations[s_dumpIdx];
        char nm[48];
        safe_name(nm, sizeof(nm), d.name);
        snprintf(line, sizeof(line), "PONS:LIVE,DEST,%d,%.7f,%.7f,%s", s_dumpIdx, d.cords[0][0], d.cords[0][1], nm);
        if (!dump_write(line)) return;
        s_dumpIdx++;
        break;
      }
      case DUMP_PILON:
        snprintf(line, sizeof(line), "PONS:LIVE,PILON,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f",
                 pla_lat, pla_lon, pilon_north_lat, pilon_north_lon, pilon_south_lat, pilon_south_lon,
                 pilon_1km_lat, pilon_1km_lon, takeshima_lat_v, takeshima_lon_v);
        if (!dump_write(line)) return;
        s_dump = DUMP_FMAP; s_dumpIdx = 0; s_dumpPt = -1;
        break;
      case DUMP_FMAP:
      case DUMP_XMAP: {
        const mapdata* mp = dump_map_at(s_dump, s_dumpIdx);
        const char kc = (s_dump == DUMP_FMAP) ? 'F' : 'X';
        if (!mp) {
          if (s_dump == DUMP_FMAP) { s_dump = DUMP_XMAP; s_dumpIdx = 0; s_dumpPt = -1; }
          else s_dump = DUMP_END;
          break;
        }
        if (s_dumpPt < 0) {
          char nm[48];
          safe_name(nm, sizeof(nm), mp->name);
          snprintf(line, sizeof(line), "PONS:LIVE,MAP,%c,%d,%d,%s", kc, s_dumpIdx, mp->size, nm);
          if (!dump_write(line)) return;
          s_dumpPt = 0;
          s_dumpMaps++;
          break;
        }
        if (s_dumpPt >= mp->size) { s_dumpIdx++; s_dumpPt = -1; break; }
        // 1 行に 7 点まで。**地図の点は [経度, 緯度] の順**（display_tft.cpp の draw_map）。行には緯度, 経度の順で書く
        // ★ 1 点ずつ別の箱に書いてから、入るときだけ足す。snprintf の戻り値は「書けたはずの長さ」なので、SD の mapdata.csv に
        //   桁の大きい値（読み違い）があると、戻り値を足し続けて line の外へ書いてしまう。入らない点は次の行へ回す。
        int n = snprintf(line, sizeof(line), "PONS:LIVE,MP,%c,%d,%d", kc, s_dumpIdx, s_dumpPt);
        int k = s_dumpPt;
        for (; k < mp->size && k < s_dumpPt + 7; k++) {
          char pt[64];
          const int m = snprintf(pt, sizeof(pt), ",%.7f,%.7f", mp->cords[k][1], mp->cords[k][0]);
          if (m <= 0 || m >= (int)sizeof(pt) || n + m + 2 >= (int)sizeof(line)) break;
          memcpy(line + n, pt, (size_t)m + 1);
          n += m;
        }
        if (k == s_dumpPt) k++;   // 1 点も入らない（壊れた値）なら、その点は飛ばす（止まらないように）
        else if (!dump_write(line)) return;
        s_dumpPt = k;
        break;
      }
      case DUMP_END:
        snprintf(line, sizeof(line), "PONS:LIVE,DUMP_END,%d,%d", destinations_count, s_dumpMaps);
        if (!dump_write(line)) return;
        s_dump = DUMP_NONE;
        break;
      default:
        s_dump = DUMP_NONE;
        break;
    }
  }
}

// ---- 始める・止める ----
static void send_ok() {
  char line[LIVE_LINE_MAX];
  snprintf(line, sizeof(line), "PONS:LIVE,OK,ON,%d,%d,%u,%u,%u,%u,%lu,%s",
           LIVE_FMT_VER, LINK_PROTO_VER, (unsigned)sizeof(LinkTelem),
           (unsigned)link_get_group(), (unsigned)link_get_radio_ch(),
           (unsigned)e220_profile_to_sf(link_get_radio_profile()), (unsigned long)link_tx_period_ms(), BUILDVERSION);
  ring_push(line);
}

static void live_start() {
  const bool again = s_on;
  ring_clear();
  s_on = true;
  s_minActive = true;
  s_lsMs = millis() - LIVE_LS_MS;   // すぐに 1 行目の状態を出す
  s_lastWriteMs = millis();         // 始めた直後は緑（3 秒書けなければ赤）
  s_lnPending = false;
  s_portDownMs = 0;                 // 前の LIVE の「つながっていない」を持ち越さない
  if (!again) { s_lines = 0; s_dropped = 0; }
  send_ok();
  // 一覧は頭から流し直す（Re:PONS がつなぎ直したときも、もう一度 ON を送ってくる）
  s_dump = DUMP_DEST; s_dumpIdx = 0; s_dumpPt = -1; s_dumpMaps = 0;
  if (!again) enqueueTask(createLogSdTask("LIVE ON"));
}

void live_stop(const char* why) {
  if (!s_on) return;
  s_on = false;
  ring_clear();
  s_dump = DUMP_NONE;
  // 止めた理由を 1 行（入らなければ出さない。ポートが閉じていれば出せない）
  if (Serial) {
    char line[LIVE_LINE_MAX];
    snprintf(line, sizeof(line), "PONS:LIVE,OFF,%s\r\n", why);
    const size_t n = strlen(line);
    if ((size_t)Serial.availableForWrite() >= n) Serial.write((const uint8_t*)line, n);
  }
  enqueueTask(createLogSdfTask("LIVE OFF (%s) lines=%lu dropped=%lu", why,
                               (unsigned long)s_lines, (unsigned long)s_dropped));
}

void live_command(const char* arg) {
  if (strcmp(arg, "ON") == 0) {
    // 受信機だけの機能（設計書 §4.5）。送信機・OFF では流さない
    if (link_get_mode() != LINK_MODE_RX) {
      Serial.println("PONS:LIVE,ERROR,NOT_RX");
      return;
    }
    live_start();
    ring_flush();
  } else if (strcmp(arg, "OFF") == 0) {
    if (!s_on) Serial.println("PONS:LIVE,OFF,CMD");   // 止まっていても返事はする（Re:PONS が待たないように）
    live_stop("CMD");
  } else {
    Serial.println("PONS:LIVE,ERROR,BAD_ARG");
  }
}

// ---- 受けたテレメトリ（LT）----
void live_on_rx_frame(const LinkTelem* t, uint8_t slot, uint8_t known) {
  if (!s_on) return;
  // ★ リプレイ中・デモ中は画面が電波と無関係なので、テレメトリと判断の行は流さない（設計書 §1.3）
  if (link_rf_suppressed()) return;
  char line[LIVE_LINE_MAX];
  const int n = snprintf(line, sizeof(line), "PONS:LT,%lu,%u,%02x,", (unsigned long)millis(),
                         (unsigned)slot, (unsigned)known);
  hex_append(line + n, (const uint8_t*)t, sizeof(LinkTelem));
  ring_push(line);
  s_lnPending = true;
  s_lnSeq = t->seq;
}

// ---- 画面の判断（LN）----
// ★ 中身は**描く関数が読んでいる値をそのまま並べる**（設計書 §3.2）。画面に項目を足したら、ここにも足して
//   Re:PONS の live_screen.js でも同じ値を読む。判断を Re:PONS で計算し直させないこと（設計書 §1.4）。
void live_after_screen() {
  if (!s_on || !s_lnPending) return;
  // ミラー中だけ（受信が切れて自機の画面に戻っている間の数字はボートの値。設計書 §1.3）
  if (!link_mirror_active()) { s_lnPending = false; return; }
  s_lnPending = false;

  LiveScreen sc;
  live_collect_screen(sc);
  uint16_t fl = sc.flags;
  const uint32_t now = millis();
  if (trackwarning_until > now) fl |= LIVESCR_TRACKWARN;
  // コース警告の箱（.ino の地図の描き直し: 警告から 10 秒、起動から 10 秒より後）
  if (now - last_course_warning_time < 10000 && now > 10000) fl |= LIVESCR_CWARN;
  // VSI（display_tft.cpp の push_backscreen。ミラー中は自機のバリオ禁止を見ない）
  if (vario_volume > 0) fl |= LIVESCR_VSI;

  char line[LIVE_LINE_MAX];
  snprintf(line, sizeof(line),
           "PONS:LN,%u,%04x,%u,%u,%u,%u,%u,%.2f,%.2f,%.1f,%.3f,%.2f,%d,%d,%d,%.2f,%.4f,%.2f,%d,%d,%.2f,%d,%d",
           (unsigned)s_lnSeq, (unsigned)fl, (unsigned)sc.hdr, (unsigned)sc.strike,
           (unsigned)sc.wait, (unsigned)sc.egs, (unsigned)sc.marker, (double)last_tone_tt, steer_angle,
           (double)course_warning_index, (double)degpersecond, (double)sc.deadband, currentdestination,
           destination_mode, auto10k_status, (double)truec, (double)dest_dist,
           (double)get_maxgs(), get_maxgs_hour(), get_maxgs_min(),
           (double)get_maxgs_5min(), get_maxgs_5min_hour(), get_maxgs_5min_min());
  ring_push(line);
  ring_flush();
}

// ---- 受信機の状態（LS）----
static void push_ls() {
  uint8_t fl = 0;
  if (link_is_receiving())    fl |= 0x01;   // 受信中（10 秒受けていなければ NO SIGNAL）
  if (link_dup_sender())      fl |= 0x02;   // 送信機が 2 台
  if (link_rf_suppressed())   fl |= 0x04;   // 受信機がリプレイ中・デモ中
  // 送信機の目的地が受信機の一覧に無い（3 台の destinations.csv・パイロン座標の上書きが違う）。受信機は目的地を「未選択」に
  // するので、配信の画面からはパイロットの画面にある目的地の線・TC・距離が消える。その理由を帯に出す（設計書 §10.6）。
  // 受信機の地図の箱（draw_link_overlay の SENDER DEST NOT IN THIS UNIT）と同じ判断。リプレイ中・デモ中は無線の表示を出さないので見ない
  if (!link_rf_suppressed() && link_dest_missing()) fl |= 0x08;
  double olat, olon, ogs, otrk;
  const bool own = gnss_get_own_fix(olat, olon, ogs, otrk);
  // 機体までの距離（received/ の dist_m と同じ計算。mysd.cpp の createSaveRxCsvTask）。分からなければ -1
  long dist_m = -1;
  if (own && link_is_receiving() && link_get_fix_ok())
    dist_m = lround(calculateDistanceKm(olat, olon, link_get_lat(), link_get_lon()) * 1000.0);
  const uint32_t age = link_age_ms();
  char line[LIVE_LINE_MAX];
  snprintf(line, sizeof(line), "PONS:LS,%02x,%ld,%d,%u,%u,%u,%ld,%lu,%lu,%lu",
           (unsigned)fl, (age == 0xFFFFFFFFu) ? -1L : (long)age,
           (int)link_rssi_avg10(), (unsigned)link_rssi_bars(), (unsigned)link_rx_count(),
           (unsigned)link_miss_count(), dist_m, (unsigned long)link_tx_period_ms(),
           (unsigned long)s_lines, (unsigned long)s_dropped);
  ring_push(line);
}

void live_tick() {
  if (!s_on) return;
  // 受信モードでなくなった（設定画面で変えた）。受信機だけの機能（設計書 §4.5）
  if (link_get_mode() != LINK_MODE_RX) { live_stop("MODE"); return; }
  // ポートが閉じた（ページを閉じた・ケーブルを抜いた。DTR が落ちる）。書いても捨てられるので止める。
  // ★ 1 秒続いたときだけ。arduino-pico の `Serial`（operator bool）は、Core1 が USB を使っている瞬間（USB の排他を取れない）にも
  //   false を返すので、1 回で止めると流している途中で黙って止まる。
  const uint32_t now = millis();
  if (!Serial) {
    if (s_portDownMs == 0) s_portDownMs = now ? now : 1;
    else if (now - s_portDownMs > 1000) { s_portDownMs = 0; live_stop("PORT"); return; }
  } else {
    s_portDownMs = 0;
  }
  if (now - s_lsMs >= LIVE_LS_MS) {
    s_lsMs = now;
    push_ls();
  }
  ring_flush();
  if (s_dump != DUMP_NONE) dump_step();
}

uint8_t live_mark_state() {
  if (!s_on) return 0;
  return (millis() - s_lastWriteMs <= LIVE_MARK_OK_MS) ? 1 : 2;
}

void live_periodic_log() {
  if (!s_on && !s_minActive) return;
  enqueueTask(createLogSdfTask("LIVE %s lines=%lu dropped=%lu ring=%u/%u total=%lu/%lu",
                               s_on ? "on" : "off", (unsigned long)s_minLines, (unsigned long)s_minDropped,
                               (unsigned)s_maxUsed, (unsigned)LIVE_RING_BYTES,
                               (unsigned long)s_lines, (unsigned long)s_dropped));
  s_minLines = s_minDropped = 0;
  s_maxUsed = s_rUsed;
  s_minActive = s_on;
}
