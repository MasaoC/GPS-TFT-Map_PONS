// ============================================================
// File    : logo_data.h
// Project : PONS v7
// Role    : 起動スプラッシュのロゴ画像（240x52, RGB565）の**宣言**。
//           実体は logo_data.cpp にある。
// ============================================================
//
// ■ なぜ .h に実体を置かないのか
//   C++ では名前空間スコープの const は内部リンケージなので、
//   .h に実体を書くと **include した .cpp ごとに複製される**。
//   インクルードガード（#ifndef）は「同じ翻訳単位で 2 回 include する」のを
//   防ぐだけで、**別の .cpp が include するのは止められない**。
//   しかもリンクエラーにならないので気づけない（24KB が黙って倍になる）。
//   宣言と定義を分けておけば、実体は必ず 1 つ。vectormap_data.cpp と同じ形。
//
#ifndef LOGO_DATA_H
#define LOGO_DATA_H
#include <Arduino.h>

#define LOGO_W 240
#define LOGO_H 52

extern const uint16_t LOGO_DATA[LOGO_W * LOGO_H];

#endif // LOGO_DATA_H
