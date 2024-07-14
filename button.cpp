#include "Button.h"

// Debounce time in milliseconds
const unsigned long debounceTime = 10;
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
                        shortPressCallback();
                    }
                }
            }
        }
    } else if (currentSwitchState == LOW && !longPressHandled) {
        // Check for long press
        if (millis() - pressTime >= longPressDuration) {
            if (longPressCallback != NULL) {
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



/*
  // Check for switch state change
  if (reading != lastSwitchState) {
    if (reading == LOW) {
      pressTime = millis();  // Record the time when the switch is pressed
      longPressHandled = false;
    } else {
      // When the switch is released
      unsigned long pressDuration = millis() - pressTime;
      if (pressDuration < longPressDuration && pressDuration > debounceTime) {
        Serial.println("short press");
        quick_redraw = true;
        redraw_screen = true;
        if (screen_mode == MODE_SETTING) {  //Setting mode
          if (selectedLine == -1) {         //No active selected line.
            cursorLine = (cursorLine + 1) % SETTING_LINES;
          } else if (selectedLine == 0) {
            tft_increment_brightness();
          } else if (selectedLine == 1) {
            toggle_demo_biwako();
            reset_degpersecond();
          } else if (selectedLine == 2) {
            toggle_mode();
          } else if (selectedLine == 3) {
            screen_mode = MODE_GPSCONST;
          }
        } else {
          scale = scalelist[scaleindex++ % (sizeof(scalelist) / sizeof(scalelist[0]))];
        }
      }
    }
    delay(debounceTime);  // Debounce delay
  } else if (reading == LOW && !longPressHandled) {
    // Check for long press
    if (millis() - pressTime >= longPressDuration) {
      Serial.println("long press");
      longPressHandled = true;
      redraw_screen = true;
      quick_redraw = true;
      if (screen_mode != MODE_SETTING) {
        if (screen_mode == MODE_GPSCONST)
          gps_getposition_mode();
        screen_mode = MODE_SETTING;
        cursorLine = 0;
        selectedLine = -1;
        tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
      } else {
        //exit
        if (cursorLine == SETTING_LINES - 1) {
          screen_mode = MODE_MAP;
        } else if (cursorLine == 3) {
          Serial.println("GPS CONST MODE");
          gps_constellation_mode();
          screen_mode = MODE_GPSCONST;
        } else {
          if (selectedLine == -1) {
            //Entering changing value mode.
            selectedLine = cursorLine;
          } else {
            //exiting changing value mode.
            selectedLine = -1;
          }
        }
      }
    }
  }

  lastSwitchState = reading;  // Update the switch state
}

*/
