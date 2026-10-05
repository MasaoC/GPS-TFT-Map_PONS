// ============================================================
// File    : sound.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 音声出力モジュールのヘッダー。
//           WAVファイル再生・PWM Sinトーン生成・
//           旋回角速度(degpersecond)に連動した音程更新の
//           関数プロトタイプ宣言。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/06
// ============================================================

#ifndef SOUND_H
  #define SOUND_H
  #include <Arduino.h>
  #include "../mysd.h"
  void __not_in_flash_func(loop_sound)();
  void loop_tone();

  void setup_sound();
  // excl_group: 音声の排他グループ（settings.h の WAV_EXCL_*）。
  //   同じグループの音声に割り込まれた側は pending に残さず捨てる（言い替えなので）。
  void startPlayWav(const char* filename, int priority = 1, int min_volume = 0,
                    int excl_group = WAV_EXCL_NONE);
  void update_tone(float degpersecond);
  void playTone(int freq,int duration, int counter,int priority = 1, int min_volume = 0, bool solo_play = false);
  void update_vario();  // バリオメーター音更新（Core0 から毎ループ呼ぶ）
#endif