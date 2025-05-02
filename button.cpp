#include "Button.h"
#include "mysd.h"
#include "gps.h"


extern int sound_volume;
extern int screen_mode;
extern int destination_mode;
extern int detail_page;
void reset_degpersecond();

// Debounce time in milliseconds
const unsigned long debounceTime = 5;
// Duration to detect a long press in milliseconds
const unsigned long longPressDuration = 1000;

// Constructor
Button::Button(int p, void (*shortPressCb)(), void (*longPressCb)()) 
    : pin(p), switchState(HIGH), lastSwitchState(HIGH), pressTime(0), longPressHandled(false), 
      shortPressCallback(shortPressCb), longPressCallback(longPressCb) {}


// Method to read button state
void Button::read() {
    bool currentSwitchState = digitalRead(pin);

    // Check for switch state change
    if (currentSwitchState != lastSwitchState) {
        delay(debounceTime); // Debounce delay
        currentSwitchState = digitalRead(pin); // Read again after debounce delay

        if (currentSwitchState != lastSwitchState) {
            lastSwitchState = currentSwitchState; // Update lastSwitchState

            if (currentSwitchState == LOW) {
                pressTime = millis(); // Record the time when the switch is pressed
                longPressHandled = false;
            } else {
                unsigned long pressDuration = millis() - pressTime;
                if (pressDuration < longPressDuration && pressDuration > debounceTime) {
                    if (shortPressCallback != NULL) {
                        enqueueTask(createPlayMultiToneTask(1046,80,1));
                        shortPressCallback();
                    }
                }
            }
        }
    } else if (currentSwitchState == LOW && !longPressHandled) {
        // Check for long press
        if (millis() - pressTime >= longPressDuration) {
            if (longPressCallback != NULL) {
                enqueueTask(createPlayMultiToneTask(1046,50,2));
                longPressCallback();
            }
            longPressHandled = true;
        }
    }
}

// Method to get the pin number
int Button::getPin() {
    return pin;
}


Setting menu_settings[] = {
  { SETTING_SETDESTINATION,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      if(currentdestination != -1 && currentdestination < destinations_count){
        sprintf(buff, selected ? " Destination: %s(%d)" : "Destination: %s(%d)", extradestinations[currentdestination].name, currentdestination);
      }else
        sprintf(buff, selected ? " Destination: %d/%d" : "Destination: %d/%d", currentdestination,destinations_count);
      return std::string(buff);  // return as std::string
    },nullptr,
    []() {
      if(destinations_count > 0){
        currentdestination++;
        if(currentdestination >= destinations_count){
          currentdestination = 0;
        }
      }
    },
    nullptr
  },
  { SETTING_DESTINATIONMODE,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      if(destination_mode == DMODE_FLYINTO)
        strcpy(buff, selected ? " Navigation Mode: Fly into" : "Navigation Mode: Fly into");
      else if(destination_mode == DMODE_FLYAWAY)
        strcpy(buff, selected ? " Navigation Mode: Fly away" : "Navigation Mode: Fly away");
      else if(destination_mode == DMODE_AUTO10K)
        strcpy(buff, selected ? " Navigation Mode: Auto10km" : "Navigation Mode: Auto10km");
      return std::string(buff);  // return as std::string
    },nullptr,
    []() {
      if(destination_mode == DMODE_FLYINTO)
        destination_mode = DMODE_FLYAWAY;
      else if(destination_mode == DMODE_FLYAWAY)
        destination_mode = DMODE_AUTO10K;
      else if(destination_mode == DMODE_AUTO10K)
        destination_mode = DMODE_FLYINTO;
      else
        destination_mode = DMODE_FLYINTO;
        
    },
    nullptr
    },
#ifdef BRIGHTNESS_SETTING_AVAIL
  { SETTING_BRIGHTNESS,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " Brightness: %03d" : "Brightness: %03d", screen_brightness);
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      tft_change_brightness(1);
    },
    nullptr
  },
#endif
  { SETTING_VOLUME,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " Volume: %d/100" : "Volume: %d/100", sound_volume);
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      if(sound_volume < 40){
        sound_volume += 10;
      }else{
        sound_volume += 20;
      }

      if(sound_volume >= 101)
        sound_volume = 0;
      else if(sound_volume <= 0)
        sound_volume = 0;      

      if(sound_volume>0){
        enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793,200,1));
        enqueueTask(createPlayMultiToneTask(1046,200,1));
        enqueueTask(createPlayMultiToneTask(440,200,1));
      }
    },
    nullptr
  },
  { SETTING_UPWARD,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " Upward: %s" : "Upward: %s", is_trackupmode() ? "TRACK UP" : "NORTH UP");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      toggle_mode();
    },
    nullptr
  },
  { SETTING_DEMOBIWA,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " DEMO mode: %s" : "DEMO mode: %s", get_demo_biwako() ? "YES" : "NO");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      latlon_manager.reset();
      toggle_demo_biwako();
      reset_degpersecond();
      gmap_loaded_active = false;
    },nullptr
  },
  { SETTING_GPSDETAIL,
    [](bool selected) -> std::string {
      return "Show GPS detail >";
    },
    []() {
      DEBUG_P(20240801, "GPS CONST MODE");
      gps_constellation_mode();
      screen_mode = MODE_GPSDETAIL;
    },
    nullptr,
    nullptr
  },
  { SETTING_MAPDETAIL,
    [](bool selected) -> std::string {
      return "Maplist detail >";
    },
    []() {
      DEBUG_P(20240801, "MAP DETAIL MODE");
      screen_mode = MODE_MAPLIST;
    },
    nullptr,
    nullptr
  },
  { SETTING_SD_DETAIL,
    [](bool selected) -> std::string {
      return "SD card detail >";
    },
    []() {
      screen_mode = MODE_SDDETAIL;
      detail_page = 0;
      enqueueTask(createBrowseSDTask(0));
    },
    nullptr,
    nullptr
  },
  { SETTING_EXIT,
    [](bool selected) -> std::string {
      return "Save & Exit >>";
    },
    []() {
      enqueueTask(createSaveSettingTask());
      if((millis()/1000)%3 == 0)
        enqueueTask(createPlayWavTask("wav/matane.wav"));
      else if((millis()/1000)%3 == 1)
        enqueueTask(createPlayWavTask("wav/baibai.wav"));
      else if((millis()/1000)%3 == 2)
        enqueueTask(createPlayWavTask("wav/arigato.wav"));
      screen_mode = MODE_MAP;
    },
    nullptr,
    nullptr
  }
};


int setting_size = sizeof(menu_settings) / sizeof(menu_settings[0]);



