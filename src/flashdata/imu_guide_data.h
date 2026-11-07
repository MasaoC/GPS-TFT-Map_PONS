// ============================================================
// File    : imu_guide_data.h
// Project : PONS v7
// Role    : IMU/ESKF 画面の入口の案内画像（240x320, RGB565, 3 枚）の**宣言**。
//           実体は imu_guide_data.cpp（tools/gen_guide_flash.py の生成物。
//           原本は tools/imu_guide/imu_guide_*.svg → .png。手で編集しないこと）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/07
// ============================================================
//
// 宣言と定義を分けている理由は logo_data.h と同じ（.h に実体を置くと、include した
// .cpp ごとに 450KB が黙って複製される）。
// 大きさと枚数の正本はここ。tools/gen_guide_flash.py もここを読む。
//
#ifndef IMU_GUIDE_DATA_H
#define IMU_GUIDE_DATA_H
#include <Arduino.h>

#define IMU_GUIDE_W     240
#define IMU_GUIDE_H     320
// 枚数を変えたら GPS_TFT_map.ino の IMU_GUIDE_WAV[]（1 枚ごとの音声）も合わせる
// （数が合わないとビルドが static_assert で止まる）。
#define IMU_GUIDE_PAGES 3

extern const uint16_t IMU_GUIDE_DATA[IMU_GUIDE_PAGES][IMU_GUIDE_W * IMU_GUIDE_H];

#endif // IMU_GUIDE_DATA_H
