// ============================================================
// File    : button.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : ボタン入力処理とメニュー定義のヘッダー。
//           短押し/長押し/ダブルクリックコールバックを持つ Button クラスと、
//           設定画面の各メニュー項目を表す Setting 構造体を定義。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/07/31
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

  // 設定を保存して設定画面を閉じ、地図画面へ戻る。
  void exit_setting();

  // Button class definition
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
      unsigned long lastShortPressTime;  // 直近の短押しを検出した時刻（ダブルクリック判定用）[ms]
      bool longPressHandled;
      void (*shortPressCallback)();
      void (*longPressCallback)();
      void (*doublePressCallback)();     // 短押し 2 連続（ダブルクリック）時に呼ぶ
  };

#endif
