#include "button.h"

// Debounce time in milliseconds
const unsigned long debounceTime = 10;
// Duration to detect a long press in milliseconds
const unsigned long longPressDuration = 1000;
// 前の短押しからこの時間内の短押しはダブルクリック（v7 と同じ 500ms）
const unsigned long doubleClickDuration = 500;

// Constructor
Button::Button(int p, void (*shortPressCb)(), void (*longPressCb)(), void (*doublePressCb)())
    : pin(p), switchState(HIGH), lastSwitchState(HIGH), pressTime(0), lastShortPressTime(0), longPressHandled(false),
      shortPressCallback(shortPressCb), longPressCallback(longPressCb), doublePressCallback(doublePressCb) {}

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
                    const unsigned long now = millis();
                    if (shortPressCallback != NULL) {
                        shortPressCallback();
                    }
                    if (doublePressCallback != NULL && lastShortPressTime != 0 &&
                        now - lastShortPressTime < doubleClickDuration) {
                        doublePressCallback();
                        lastShortPressTime = 0;  // 3 回目が続けてダブルクリックにならないように
                    } else {
                        lastShortPressTime = now;
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
