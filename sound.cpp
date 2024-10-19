#include "settings.h"
#include "sound.h"
#include "gps.h"
#include "mysd.h"


float last_tone_tt = 0;

float note_frequencies[] = {
    130.81,  // C3 (Do)
    146.83,  // D3 (Re)
    164.81,  // E3 (Mi)
    174.61,  // F3 (Fa)
    196.00,  // G3 (So)
    220.00,  // A3 (La)
    246.94,  // B3 (Si)
    261.63,  // C4 (Do)
    293.66,  // D4 (Re)
    329.63,  // E4 (Mi)
    349.23,  // F4 (Fa)
    392.00,  // G4 (So)
    440.00,  // A4 (La)
    493.88,  // B4 (Si)
    523.25,  // C5 (Do)
    587.33,  // D5 (Re)
    659.25,  // E5 (Mi)
    698.46,  // F5 (Fa)
    783.99,  // G5 (So)
    880.00,  // A5 (La)
    987.77,  // B5 (Si)
    1046.50, // C6 (Do)
    1174.66, // D6 (Re)
    1318.51, // E6 (Mi)
    1396.91, // F6 (Fa)
    1567.98, // G6 (So)
    1760.00, // A6 (La)
    1975.53, // B6 (Si)
    2093.00, // C7 (Do)
    2349.32, // D7 (Re)
    2637.02, // E7 (Mi)
    2793.83, // F7 (Fa)
    3135.96, // G7 (So)
    3520.00, // A7 (La)
    3951.07, // B7 (Si)
    4186.01  // C8 (Do)
};

// Function to find the nearest frequency using binary search
float nearest_note_frequency(float input_freq) {
    int left = 0;
    int right = sizeof(note_frequencies) / sizeof(note_frequencies[0]) - 1;

    // Handle cases outside the known note frequencies
    if (input_freq <= note_frequencies[left]) {
        return note_frequencies[left];
    }
    if (input_freq >= note_frequencies[right]) {
        return note_frequencies[right];
    }

    // Binary search for the closest frequency
    while (left <= right) {
        int mid = (left + right) / 2;

        if (note_frequencies[mid] == input_freq) {
            return note_frequencies[mid];
        } else if (note_frequencies[mid] < input_freq) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    // After binary search, 'left' will be the smallest number greater than 'input_freq'
    // and 'right' will be the largest number smaller than 'input_freq'.
    // Find the nearest one.
    if (fabs(input_freq - note_frequencies[left]) < fabs(input_freq - note_frequencies[right])) {
        return note_frequencies[left];
    } else {
        return note_frequencies[right];
    }
}


void update_tone(float degpersecond,int duration){
  int angle_diff = abs(get_gps_truetrack()-last_tone_tt);
  //角度変化が大きい時。
  if(angle_diff > 15 && duration > 0){
    last_tone_tt = get_gps_truetrack();
    enqueueTask(createPlayMultiToneTask(3500,50,angle_diff>30?4:2));
    Serial.println("multi");
  }
  //音を出す閾値は、2.0deg/s かつ 2.0 m/s 以上 かつ 音再生設定がある時。
  if(abs(degpersecond) > 2.0 && get_gps_mps() > 2.0 && duration > 0){
    int freq = abs(degpersecond)*300-560;//40Hz以上
    if(freq > 3000){
      freq = 3000;
    }
    //精度が低い状況や角度変化少ない状況では、音は短めにする。
    int newduration = (abs(degpersecond)<3 || get_gps_mps() < 4.0)?duration*0.4:duration;
    enqueueTask(createPlayMultiToneTask(nearest_note_frequency(freq), newduration,1));
  }
}

void playTone(int freq, int duration, int counter){
  tone(PIN_TONE,freq, duration);
  if(counter <= 1){
    return;
  }
  for(int i = counter; i > 0; i--){
    delay(duration);
    noTone(PIN_TONE);
    delay(duration);
    tone(PIN_TONE,freq, duration);
  }
  noTone(PIN_TONE);
}