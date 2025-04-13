
#ifndef SOUND_H
  #define SOUND_H
  #include <Arduino.h>
  #include "mysd.h"
  void loop_sound();

  void setup_sound();
  void update_tone(float degpersecond, int duration);
  void playTone(int freq,int duration, int counter);
#endif