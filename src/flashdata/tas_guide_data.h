// ============================================================
// File    : tas_guide_data.h
// Project : PONS v7
// Role    : 推定 TAS の案内画像（240x320, RGB565, 1 枚）の**宣言**。
//           実体は tas_guide_data.cpp（tools/gen_guide_flash.py の生成物。
//           原本は tools/tas_guide/tas_guide_*.svg → .png。手で編集しないこと）。
//           案内の最後の画面（対気速度モデルの曲線）はファームが描く（display_pages.cpp）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================
//
// 宣言と定義を分けている理由は imu_guide_data.h と同じ（.h に実体を置くと、include した
// .cpp ごとに画像が黙って複製される）。
// 大きさと枚数の正本はここ。tools/gen_guide_flash.py もここを読む。
//
#ifndef TAS_GUIDE_DATA_H
#define TAS_GUIDE_DATA_H
#include <Arduino.h>

#define TAS_GUIDE_W     240
#define TAS_GUIDE_H     320
// 曲線は案内画像の次に表示する。
#define TAS_GUIDE_PAGES 1

extern const uint16_t TAS_GUIDE_DATA[TAS_GUIDE_PAGES][TAS_GUIDE_W * TAS_GUIDE_H];

#endif // TAS_GUIDE_DATA_H
