// ============================================================
// File    : button.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : ボタン入力処理とメニュー定義のヘッダー。
//           短押し/長押し/ダブルクリックコールバックを持つ Button クラスと、
//           設定画面の各メニュー項目を表す Setting 構造体を定義。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/14
// ============================================================
#ifndef BUTTON_H
#define BUTTON_H
#include <Arduino.h>
#include <string>

  // ★ `iconColor()` がこれを返した行は、丸ではなく**ハイフン**を描く。
  //   意味は「この機体ではこの項目を評価できない」。
  //   **色で正常/異常を言えない場面に丸を出してはいけない。** 受信モード（ボート／
  //   ピット）は自機が飛行機ではないので、姿勢の準備状況や「飛行中でも聞こえる音量か」
  //   という判定が成り立たない。それでも丸を出すと**正しい設定なのに警告に見え**、
  //   やがて丸そのものが無視されるようになる。
  //   iconColor が nullptr（丸を出さない）とは別物で、あちらは「状態の概念が無い行」。
  #define MENU_ICON_HYPHEN  (-1)

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
  // ★ farewell=false で**別れの音声を鳴らさない**。放置タイムアウトで閉じるとき用で、
  //   「操作していないのに声が鳴る」のを避けるため（呼び出し側が代わりに短いトーンを鳴らす）。
  void exit_setting(bool farewell = true);

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
