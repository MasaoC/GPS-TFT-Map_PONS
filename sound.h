
#ifndef SOUND_H
  #define SOUND_H
  #include <Arduino.h>
  #include "mysd.h"
  void loop_sound();


  void setup_sound();
  void startPlayWav(const char* filename, int priority = 1);
  void update_tone(float degpersecond);
  void playTone(int freq,int duration, int counter,int priority = 1);
#endif