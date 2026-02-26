// ============================================================
// File    : button.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : ボタン入力処理とメニュー定義のヘッダー。
//           短押し/長押しコールバックを持つ Button クラスと、
//           設定画面の各メニュー項目を表す Setting 構造体を定義。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================
#ifndef BUTTON_H
#define BUTTON_H
#include <Arduino.h>
#include <string>

  struct Setting {
      int id;
      std::string (*getLabel)(bool selected);
      void (*CallbackEnter)();
      void (*CallbackToggle)();
      void (*CallbackExit)();
      int (*iconColor)();
  };
  extern Setting menu_settings[];
  extern const int setting_size;
  // Button class definition
  class Button {
  public:
      Button(int p, void (*shortPressCb)() = NULL, void (*longPressCb)() = NULL);
      void read();
      int getPin(); // Method to get the pin number

  private:
      int pin;
      bool switchState;
      bool lastSwitchState;
      unsigned long pressTime;
      bool longPressHandled;
      void (*shortPressCallback)();
      void (*longPressCallback)();
  };

#endif
