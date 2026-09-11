// ============================================================
// File    : link_icons.h
// Project : PONS v7 — PONS Link
// Role    : 無線の状態表示に使う 1bit マスクの**宣言**。実体は link_icons.cpp。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/08
// ============================================================
//
// ■ なぜ BMP(RGB565) ではなく 1bit マスクなのか
//   役割アイコンは **色を変えて描く必要がある**（送信=アンバー / 受信=ティール）し、
//   受信モードでは点滅させる。RGB565 で焼くと色が固定されるので、
//   色ごとに別画像を持つ羽目になる。形だけを持って描画時に色を与えるほうが正しい。
//   容量が 32x32 で 2048B → 128B になるのは副次的な利点。
//
// ■ 形式
//   1 行あたり (W+7)/8 バイト、MSB が左端。1=描く / 0=透明。
//
// ■ ★飛行機と船は試作品（仮）
//   本番の絵ができたら link_icons.cpp を差し替えること。
//   電波アイコンは確定版（tools/gen_link_icons.py で生成）。
//
#ifndef LINK_ICONS_H
#define LINK_ICONS_H
#include <Arduino.h>

#define ICON_PLANE_W 32
#define ICON_PLANE_H 32
extern const uint8_t ICON_PLANE[];

#define ICON_BOAT_W 32
#define ICON_BOAT_H 32
extern const uint8_t ICON_BOAT[];

// 電波アイコン。受信用と送信用で**絵そのものを分ける**。19x15 で共通。
#define ICON_RSSI_W 19
#define ICON_RSSI_H 15

// ---- 受信モード：アンテナ本数 ----
// **枠 3 本は常に描き、中を 0〜3 本ぶん塗る**（携帯電話と同じ流儀）。
// 塗りだけにすると 0 本・1 本のときに何の絵なのか読めなくなるので枠を残す。
// 本数は link_rssi_bars() が返す。
extern const uint8_t ICON_RSSI0[];   // 圏外（枠だけ）
extern const uint8_t ICON_RSSI1[];
extern const uint8_t ICON_RSSI2[];
extern const uint8_t ICON_RSSI3[];   // 満信号
extern const uint8_t* const ICON_RSSI_BARS[4];

// ---- 送信モード：出ている電波 ----
// ★ 送信側にアンテナ本数を出してはいけない。**受信の手段が無いので強度を
//   測っていない**のに、測ったように見せることになる。
//   右端の発生源から左へ広がる弧にして、右隣の飛行機アイコンから
//   電波が出ているように見せる。弧の数は強度ではなく「今送った」の脈動。
extern const uint8_t ICON_TX0[];     // 発生源だけ（送れていない）
extern const uint8_t ICON_TX1[];     // 待機中
extern const uint8_t ICON_TX2[];
extern const uint8_t ICON_TX3[];     // 送信直後
extern const uint8_t* const ICON_TX_WAVES[4];

// ロスト時に 0 本アイコンの上へ赤で重ねる×印。同じ 19x15。
// 「弱い（0 本）」と「切れている」を色だけで区別させないための強調。
extern const uint8_t ICON_RSSI_X[];

#endif // LINK_ICONS_H
