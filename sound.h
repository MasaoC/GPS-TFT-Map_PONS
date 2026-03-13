// ============================================================
// File    : sound.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 音声出力モジュールのヘッダー。
//           WAVファイル再生・PWM Sinトーン生成・
//           旋回角速度(degpersecond)に連動した音程更新の
//           関数プロトタイプ宣言。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================

#ifndef SOUND_H
  #define SOUND_H
  #include <Arduino.h>
  #include "mysd.h"
  void loop_sound();


  void setup_sound();
  void startPlayWav(const char* filename, int priority = 1, int min_volume = 0);
  void update_tone(float degpersecond);
  void playTone(int freq,int duration, int counter,int priority = 1, int min_volume = 0);
  void update_vario();  // バリオメーター音更新（Core0 から毎ループ呼ぶ）
#endif