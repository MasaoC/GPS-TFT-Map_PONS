#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

// Button class definition
// 短押し・長押し・ダブルクリック（v7 と同じ判定）。
// ★ 短押しのコールバックは押すたびにすぐ呼ぶ（ダブルクリックかどうかを待たない。設定画面の操作が遅れないように）。
//   2 回目の短押しが前の短押しから doubleClickDuration 以内なら、短押しに続けてダブルクリックも呼ぶ。
class Button {
public:
    Button(int p, void (*shortPressCb)() = NULL, void (*longPressCb)() = NULL, void (*doublePressCb)() = NULL);
    void read();
    int getPin(); // Method to get the pin number

private:
    int pin;
    bool switchState;
    bool lastSwitchState;
    unsigned long pressTime;
    unsigned long lastShortPressTime;  // 直近の短押しの時刻（ダブルクリック判定用。0 = 無し）
    bool longPressHandled;
    void (*shortPressCallback)();
    void (*longPressCallback)();
    void (*doublePressCallback)();
};

#endif
