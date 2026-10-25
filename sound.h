// ============================================================
// File    : sound.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : 音声出力モジュールのヘッダー。
//           WAV（本体 FLASH 内蔵）再生・PWM Sinトーン生成・
//           旋回角速度(degpersecond)に連動した音程更新の
//           関数プロトタイプ宣言。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================

#ifndef SOUND_H
  #define SOUND_H
  #include <Arduino.h>
  #include "mysd.h"
  void __not_in_flash_func(loop_sound)();
  void loop_tone();

  void setup_sound();
  void startPlayWav(const char* filename, int priority = 1, int min_volume = 0);
  void update_tone(float degpersecond);
  void playTone(int freq,int duration, int counter,int priority = 1, int min_volume = 0, bool solo_play = false);
#endif