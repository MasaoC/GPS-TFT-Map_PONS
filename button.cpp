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
