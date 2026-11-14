// ============================================================
// File    : tas.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 推定 TAS（対気速度）の表示の判断。ヘッダー右の大きい枠に何を出すかを、
//           設定・地上と飛行中・表示の供給元から決める（tas.h）。仕様は docs/pons_tas.md。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================
//
// ★ 供給元は 4 つ（CLAUDE.md「表示の供給元は 3 系統ある」＋デモ）:
//   ・実センサー: 自機の平均ピッチ・推定 TAS・設定。地上か飛行中かは自機の対地速度から。
//   ・無線ミラー: 送信機の平均ピッチ・推定 TAS・設定（無線で受けた値とビット）。
//     **地上か飛行中かは受信機がミラーされた対地速度から同じ関数で計算する**
//     （link_proto.h「受信側で再計算できるものは載せない」。計算を 1 本にしておけば送信側とずれない）。
//     推定 TAS は受信機の係数で計算し直さない（受信機は送信機の V0・K を知らない）。
//   ・リプレイ: 記録された hdr 列と tas_mps 列をそのまま出す（いまの V0・K で計算し直さない。
//     あとで係数を直すと過去の飛行の数字が変わってしまうため）。列の無い古い CSV は今までどおり。
//   ・デモ: GNSS の値だけ差し替わり、姿勢は実センサー。対地速度が出るので飛行中の判定が通り、
//     机の上で TAS の箱を確かめられる。
//
#include "tas.h"
#include "settings.h"
#include "attitude.h"
#include "gnss.h"
#include "imu.h"
#include "link.h"

static volatile bool s_setting = false;   // 書くのは Core1（SD の読み込み）と Core0（設定画面）

void tas_setting_set(bool tas) { s_setting = tas; }
bool tas_setting_get()         { return s_setting; }

// ============================================================
//  地上と飛行中
// ============================================================
// 供給元が変わったら数え直す。ボートの受信機がミラーを始めた瞬間に、それまでの
// 「ボート自身の対地速度」で数えた状態を機体の話として持ち込まないため。
enum TasSrc : uint8_t { SRC_SENSOR, SRC_MIRROR, SRC_REPLAY, SRC_DEMO };
static TasSrc   s_src      = SRC_SENSOR;
static bool     s_airborne = false;
static bool     s_runOn    = false;   // 切り替えの条件が続いているか
static uint32_t s_runMs    = 0;       // 続き始めた時刻
// 地上のまま、飛び出す側の条件（対地速度 TAS_AIR_GS_MPS 以上）が成り立っているか（2026-11-14〜）。
// GROUND の札だけはこれで先に外す（線は TAS_SWITCH_HOLD_MS 待ってから外す）。tas_leaving_ground()
static bool     s_leaving  = false;

static TasSrc current_src() {
  if (link_mirror_active()) return SRC_MIRROR;
  if (getReplayMode())      return SRC_REPLAY;
  if (is_demo_active())     return SRC_DEMO;
  return SRC_SENSOR;
}

void tas_tick() {
  const TasSrc src = current_src();
  if (src != s_src) {
    s_src = src;
    s_airborne = false;
    s_runOn = false;
    s_leaving = false;
  }
  // ★ GNSS が無いあいだは数えない（対地速度 0 を「止まった」と読まない）。
  //   飛行中に測位が切れても地上の横線に戻さず、いまの状態を保つ。
  //   衛星数が分からないとき（ミラーを始めた直後で回転枠 3 をまだ受けていない）も同じ。
  const int ns = get_gnss_numsat();
  const bool gs_ok = !(ns == 0 || ns == GNSS_NUMSAT_UNKNOWN);
  const float gs = gs_ok ? (float)get_gnss_mps() : 0.0f;
  // リプレイは記録された hdr をそのまま出すので、札を外すかだけを再生中の対地速度から決める
  if (src == SRC_REPLAY) { s_leaving = gs_ok && gs >= TAS_AIR_GS_MPS; return; }
  if (!gs_ok) { s_runOn = false; s_leaving = false; return; }

  // いまの状態から抜ける条件が成り立っているか
  const bool toward = s_airborne ? (gs < TAS_GND_GS_MPS) : (gs >= TAS_AIR_GS_MPS);
  const uint32_t now = millis();
  if (!toward) { s_runOn = false; s_leaving = false; return; }
  if (!s_runOn) { s_runOn = true; s_runMs = now; s_leaving = !s_airborne; return; }
  if (now - s_runMs >= TAS_SWITCH_HOLD_MS) {
    s_airborne = !s_airborne;
    s_runOn = false;
  }
  s_leaving = !s_airborne && s_runOn;
}

bool tas_leaving_ground() { return s_leaving; }

// ============================================================
//  大きい枠
// ============================================================
// 自機の平均ピッチを出してよいか（いままでの header_shows_pitch() と同じ条件）。
static bool local_pavg_ok() {
  return get_imu_ok() && attitude_get_rpy_enabled() && attitude_pitch_avg_valid();
}

bool tas_local(float &mps, bool &clamped) {
  if (!attitude_get_rpy_enabled()) return false;
  return attitude_get_tas(mps, clamped);
}

bool tas_header_pitch_avg(float &avg) {
  if (link_mirror_active()) return link_get_pitch_avg(avg);
  if (getReplayMode())      return get_replay_pitch_avg(avg);
  if (!local_pavg_ok())     return false;
  avg = attitude_get_pitch_avg_deg();
  return true;
}

// TAS を選んでいるときの箱の中身。値が無ければ（送信機・自機とも平均ピッチが無いのと同じ）真方位へ戻す。
static TasHdr tas_box(bool have, bool clamped) {
  if (!have)       return HDR_TT;
  if (!s_airborne) return HDR_TAS_GROUND;
  return clamped ? HDR_TAS_CLAMP : HDR_TAS;
}

TasHdr tas_header() {
  float a;
  if (link_mirror_active()) {
    if (!link_get_pitch_avg(a)) return HDR_TT;
    // ★ ボートは自分の設定ではなく送信機の設定に従う（受信モードは送信機の画面を再現する）
    if (!link_sender_header_tas()) return HDR_PAVG;
    float v; bool c = false;
    const bool have = link_get_tas(v, c);
    return tas_box(have, c);
  }
  if (getReplayMode()) {
    int h;
    if (get_replay_hdr(h) && h >= 0 && h < HDR_COUNT) {
      const TasHdr r = (TasHdr)h;
      // 記録が TAS でも値の列が読めなければ出さない（中身の無い箱にしない。地上の横線も数字を出す）
      float v;
      if (tas_hdr_is_tas(r) && !get_replay_tas(v)) return HDR_TT;
      if (r == HDR_PAVG && !get_replay_pitch_avg(a)) return HDR_TT;
      return r;
    }
    // hdr 列の無い古い CSV: 今までどおり平均ピッチがあれば Pavg
    return get_replay_pitch_avg(a) ? HDR_PAVG : HDR_TT;
  }
  if (!local_pavg_ok()) return HDR_TT;
  if (!s_setting)       return HDR_PAVG;
  float v; bool c = false;
  return tas_box(attitude_get_tas(v, c), c);
}

float tas_header_value() {
  float v = 0.0f; bool c = false;
  if (link_mirror_active())  { link_get_tas(v, c); return v; }
  if (getReplayMode())       { get_replay_tas(v);  return v; }
  attitude_get_tas(v, c);
  return v;
}
