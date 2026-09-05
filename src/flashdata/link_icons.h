// ============================================================
// File    : link_icons.h
// Project : PONS v7 — PONS Link
// Role    : 無線の状態表示に使う 1bit マスクの**宣言**。実体は link_icons.cpp。
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
// ■ ★これは試作品（仮）
//   本番の絵ができたら link_icons.cpp を差し替えること。
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

#define ICON_RSSI_W 16
#define ICON_RSSI_H 12
extern const uint8_t ICON_RSSI[];

#endif // LINK_ICONS_H
