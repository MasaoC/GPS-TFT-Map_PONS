// ============================================================
// File    : button.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : ボタン入力処理と設定メニューの実装。
//           Button クラスによる短押し/長押し判定、
//           設定画面の全メニュー項目（目的地・音量・輝度など）の
//           ラベル生成・値変更コールバック定義。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================
// Handle button updates
#include "Button.h"
#include "mysd.h"
#include "gps.h"


extern int sound_volume;
extern int screen_mode;
extern int destination_mode;
extern int detail_page;
void reset_degpersecond();

const unsigned long debounceTime = 5;       // チャタリング除去のための待機時間 [ms]
const unsigned long longPressDuration = 1000; // 長押しと判定するまでの時間 [ms]

Button::Button(int p, void (*shortPressCb)(), void (*longPressCb)())
    : pin(p), switchState(HIGH), lastSwitchState(HIGH), pressTime(0), longPressHandled(false),
      shortPressCallback(shortPressCb), longPressCallback(longPressCb) {}

// ボタンの状態を毎ループ読み取る。短押し・長押しを判定してコールバックを呼ぶ。
// 判定ロジック:
//   - 押下時 (HIGH → LOW): 押下開始時刻を記録
//   - 離した時 (LOW → HIGH): 押下時間が longPressDuration 未満なら短押し
//   - 押し続けている時: longPressDuration 以上経過で長押し（1 回だけ発火）
// チャタリング対策として 5ms 待ってから再読みしている。
void Button::read() {
    bool currentSwitchState = digitalRead(pin);

    if (currentSwitchState != lastSwitchState) {
        delay(debounceTime);                           // チャタリング除去
        currentSwitchState = digitalRead(pin);

        if (currentSwitchState != lastSwitchState) {
            lastSwitchState = currentSwitchState;

            if (currentSwitchState == LOW) {
                pressTime = millis();        // 押下開始時刻を記録
                longPressHandled = false;
            } else {
                // ボタンを離した → 押下時間で短押し判定
                unsigned long pressDuration = millis() - pressTime;
                if (pressDuration < longPressDuration && pressDuration > debounceTime) {
                    if (shortPressCallback != NULL) {
                        enqueueTask(createPlayMultiToneTask(1046,80,1));  // 短押し音
                        shortPressCallback();
                    }
                }
            }
        }
    } else if (currentSwitchState == LOW && !longPressHandled) {
        // 押し続け中 → 規定時間経過で長押し発火（longPressHandled で 1 回に制限）
        if (millis() - pressTime >= longPressDuration) {
            if (longPressCallback != NULL) {
                enqueueTask(createPlayMultiToneTask(1046,50,2));  // 長押し音
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

// AUTO10K モードで使用できない目的地かどうかを判定する。
// N_PILON / W_PILON / TAKESHIMA は 10km コースの折り返し地点そのものであり、
// そこを「目的地」として AUTO10K に設定すると経路計算がおかしくなるため禁止している。
bool is10K_NotAllowed_Destination(const char *name) {
    return strcmp(name, "N_PILON") == 0 ||
           strcmp(name, "W_PILON") == 0 ||
           strcmp(name, "TAKESHIMA") == 0;
}


// 設定画面を終了してマップ画面に戻る。
// 1. 現在の設定内容を SD カードへ保存するタスクをキューに積む。
// 2. 起動からの経過秒を 3 で割った余りで別れの音声を切り替える（毎回同じにならないランダム感）。
//    0: "またね"  1: "バイバイ"  2: "ありがとう"
// 3. screen_mode を MODE_MAP に変えて設定画面を閉じる。
void exit_setting(){
    enqueueTask(createSaveSettingTask());
    if((millis()/1000)%3 == 0)
      enqueueTask(createPlayWavTask("wav/matane.wav"));
    else if((millis()/1000)%3 == 1)
      enqueueTask(createPlayWavTask("wav/baibai.wav"));
    else if((millis()/1000)%3 == 2)
      enqueueTask(createPlayWavTask("wav/arigato.wav"));
    screen_mode = MODE_MAP;
}

// ============================================================
// menu_settings[]  —  設定画面に表示するメニュー項目の定義テーブル
//
// 各要素は Setting 構造体で以下のフィールドを持つ:
//   id            : 設定項目の識別子（SETTING_xxx）
//   getLabel      : 画面に表示するラベル文字列を返すラムダ。
//                   selected=true のとき先頭にスペースを付けてカーソル位置を示す。
//   CallbackEnter : 項目を「決定（Enter）」したときに呼ぶラムダ（nullptrなら無し）
//   CallbackToggle: 項目を「トグル（Toggle）」したときに呼ぶラムダ（nullptrなら無し）
//   CallbackExit  : トグル後に設定画面を抜けるときに呼ぶラムダ（nullptrなら無し）
//   iconColor     : アイコン色を返すラムダ（nullptrなら無し）
// ============================================================
Setting menu_settings[] = {

  // ----------------------------------------------------------
  // [1] 目的地選択 (SETDESTINATION)
  //   ・Toggle: currentdestination を 0..destinations_count-1 でローテート
  //   ・Exit  : AUTO10K 禁止目的地が選ばれていたら FLYINTO に強制変更し、エラー音を鳴らす
  //   ・アイコン色: 目的地が "PLATHOME"（出発地点）なら緑、それ以外は赤
  // ----------------------------------------------------------
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
      // 目的地インデックスを +1 し、末尾を超えたら 0 に戻す（ループ選択）
      if(destinations_count > 0){
        currentdestination++;
        if(currentdestination >= destinations_count){
          currentdestination = 0;
        }
      }
    },
    []() {
      // 間違い防止のため、10KM禁止の目的地で10KMモードを設定しようとしたら、エラー音を出してINTOに変える。
      if(is10K_NotAllowed_Destination(extradestinations[currentdestination].name)){
        if(destination_mode == DMODE_AUTO10K){
          destination_mode = DMODE_FLYINTO;
          //error tone.
          enqueueTask(createPlayMultiToneTask(294,140,1,2));
          enqueueTask(createPlayMultiToneTask(294,500,1,2));
        }
      }
    },
    [](){
      if(strcmp(extradestinations[currentdestination].name, "PLATHOME") == 0)
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [2] ナビゲーションモード選択 (DESTINATIONMODE)
  //   ・Toggle: FLYINTO → FLYAWAY → AUTO10K → FLYINTO と循環
  //   ・Exit  : AUTO10K 禁止目的地なら FLYINTO に強制変更し、エラー音を鳴らす
  //   ・アイコン色: AUTO10K なら緑（大会モード）、それ以外は赤
  // ----------------------------------------------------------
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
      // モードを 3 段階でループ切り替え
      if(destination_mode == DMODE_FLYINTO)
        destination_mode = DMODE_FLYAWAY;
      else if(destination_mode == DMODE_FLYAWAY)
        destination_mode = DMODE_AUTO10K;
      else if(destination_mode == DMODE_AUTO10K)
        destination_mode = DMODE_FLYINTO;
      else
        destination_mode = DMODE_FLYINTO;

    },
    []() {
      // 間違い防止のため、10KM禁止の目的地で10KMモードを設定しようとしたら、エラー音を出してINTOに変える。
      if(is10K_NotAllowed_Destination(extradestinations[currentdestination].name)){
        if(destination_mode == DMODE_AUTO10K){
          destination_mode = DMODE_FLYINTO;
          //error tone.
          enqueueTask(createPlayMultiToneTask(294,140,1,2));
          enqueueTask(createPlayMultiToneTask(294,500,1,2));
        }
      }
    },
    [](){
      if(destination_mode == DMODE_AUTO10K)
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

#ifdef BRIGHTNESS_SETTING_AVAIL
  // ----------------------------------------------------------
  // [3] 輝度設定 (BRIGHTNESS)  ※ BRIGHTNESS_SETTING_AVAIL 定義時のみ有効
  //   ・Toggle: tft_change_brightness(1) で輝度を 1 ステップ上げる（上限で折り返す）
  // ----------------------------------------------------------
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
    nullptr,
    nullptr,
  },
#endif

  // ----------------------------------------------------------
  // [4] 音量設定 (VOLUME)
  //   ・Toggle: 0→5→10→20→30→40→60→80→100→0 の順に切り替わる。
  //             音量 > 0 のとき確認音（高→中→低の 3 音）を鳴らして現在音量を体感させる。
  //   ・アイコン色: 30 以上なら緑（聞こえる音量）、未満なら赤
  // ----------------------------------------------------------
  { SETTING_VOLUME,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " Volume: %d/100" : "Volume: %d/100", sound_volume);
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      // 音量を段階的に増加させ、100 を超えたら 0（ミュート）に戻す
      // シーケンス: 0 → 5 → 10 → 20 → 30 → 40 → 60 → 80 → 100 → 0
      if(sound_volume == 0){
        sound_volume = 5;
      } else if(sound_volume == 5){
        sound_volume = 10;
      } else if(sound_volume < 40){
        sound_volume += 10;
      } else {
        sound_volume += 20;
      }

      if(sound_volume >= 101)
        sound_volume = 0;
      else if(sound_volume <= 0)
        sound_volume = 0;

      // 音量 > 0 のとき、高音→中音→低音の確認音を鳴らして体感できるようにする
      if(sound_volume>0){
        enqueueTaskWithAbortCheck(createPlayMultiToneTask(2793,200,1));
        enqueueTask(createPlayMultiToneTask(1046,200,1));
        enqueueTask(createPlayMultiToneTask(440,200,1));
      }
    },
    nullptr,
    [](){
      if(sound_volume >= 30)
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [5] マップ向き設定 (UPWARD)
  //   ・Toggle: NORTH UP ↔ TRACK UP を切り替え（toggle_mode() が内部状態を反転）
  //   ・アイコン色: NORTH UP なら緑（安定方向）、TRACK UP なら赤
  // ----------------------------------------------------------
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
    nullptr,
    nullptr
  },

  // ----------------------------------------------------------
  // [6] デモモード（琵琶湖）(DEMOBIWA)
  //   ・Toggle: 位置履歴をリセットし、琵琶湖デモフライトのオン/オフを切り替える。
  //             degpersecond もリセットし、地図キャッシュも無効化する。
  //   ・Exit  : デモがオンになったらリプレイを無効にして設定を閉じる（両立しない）
  //   ・アイコン色: デモがオフなら緑（通常飛行中）、オンなら赤
  // ----------------------------------------------------------
  { SETTING_DEMOBIWA,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " DEMO mode: %s" : "DEMO mode: %s", get_demo_biwako() ? "YES" : "NO");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      latlon_manager.reset();       // 位置履歴をクリア
      toggle_demo_biwako();         // デモオン/オフ切り替え
      reset_degpersecond();         // 旋回角速度をリセット
      gmap_loaded_active = false;   // 地図キャッシュを無効化（再読み込みさせる）
    },[](){
      if(get_demo_biwako()){
        set_replaymode(false);  // デモとリプレイは同時使用不可 → リプレイを無効化
        exit_setting();
      }
    },
    [](){
      if(!get_demo_biwako())
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [7] フライトリプレイモード (REPLAY)
  //   ・Toggle: 位置履歴・degpersecond をリセットし、リプレイモードをオン/オフ切り替え。
  //             地図キャッシュも無効化する。
  //   ・Exit  : リプレイがオンになったらデモを無効化し、
  //             Core1 へリプレイ初期化タスクを送信してから設定を閉じる。
  //   ・アイコン色: リプレイがオフなら緑（通常モード）、オンなら赤
  // ----------------------------------------------------------
  { SETTING_REPLAY,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " REPLAY mode: %s" : "REPLAY mode: %s", getReplayMode() ? "YES" : "NO");
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      latlon_manager.reset();       // 位置履歴をクリア
      reset_degpersecond();         // 旋回角速度をリセット
      toggleReplayMode();           // リプレイオン/オフ切り替え
      gmap_loaded_active = false;   // 地図キャッシュを無効化（再読み込みさせる）
    },[](){
      if(getReplayMode()){
        set_demo_biwako(false);              // デモとリプレイは同時使用不可 → デモを無効化
        enqueueTask(createInitReplayTask()); // Core1: リプレイ用 SD 読み込みを初期化
        exit_setting();
      }
    },
    [](){
      if(!getReplayMode())
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [8] GPS 詳細画面へ (GPSDETAIL)
  //   ・Enter: GPS 星座モードを有効化し、GPS 詳細画面（衛星配置・SNR グラフ）へ遷移
  //   ・Toggle/Exit/アイコン色: なし
  // ----------------------------------------------------------
  { SETTING_GPSDETAIL,
    [](bool selected) -> std::string {
      return "Show GPS detail >";
    },
    []() {
      DEBUG_P(20240801, "GPS CONST MODE");
      gps_constellation_mode();       // GPS 詳細モード（衛星情報収集）を開始
      screen_mode = MODE_GPSDETAIL;   // GPS 詳細画面に切り替え
    },
    nullptr,
    nullptr,
    nullptr
  },

  // ----------------------------------------------------------
  // [9] マップ一覧画面へ (MAPDETAIL)
  //   ・Enter: SD カード上の地図 BMP リスト画面へ遷移
  //   ・Toggle/Exit/アイコン色: なし
  // ----------------------------------------------------------
  { SETTING_MAPDETAIL,
    [](bool selected) -> std::string {
      return "Maplist detail >";
    },
    []() {
      DEBUG_P(20240801, "MAP DETAIL MODE");
      screen_mode = MODE_MAPLIST;   // 地図リスト画面に切り替え
    },
    nullptr,
    nullptr,
    nullptr
  },

  // ----------------------------------------------------------
  // [10] SD カード詳細画面へ (SD_DETAIL)
  //   ・Enter: ページ番号を 0 にリセットし、Core1 へ SD ブラウズタスクを送信して
  //            SD カード詳細画面（ファイル一覧）へ遷移
  //   ・Toggle/Exit/アイコン色: なし
  // ----------------------------------------------------------
  { SETTING_SD_DETAIL,
    [](bool selected) -> std::string {
      return "SD card detail >";
    },
    []() {
      screen_mode = MODE_SDDETAIL;        // SD 詳細画面に切り替え
      detail_page = 0;                    // ページを先頭に戻す
      enqueueTask(createBrowseSDTask(0)); // Core1: SD ファイル一覧の取得を依頼
    },
    nullptr,
    nullptr,
    nullptr
  },

  // ----------------------------------------------------------
  // [11] 保存して終了 (EXIT)
  //   ・Enter: exit_setting() を呼び出し、設定を保存してマップ画面に戻る
  //   ・Toggle/Exit/アイコン色: なし
  // ----------------------------------------------------------
  { SETTING_EXIT,
    [](bool selected) -> std::string {
      return "Save & Exit >>";
    },
    []() {
      exit_setting();
    },
    nullptr,
    nullptr,
    nullptr
  }
};


const int setting_size = sizeof(menu_settings) / sizeof(menu_settings[0]);



