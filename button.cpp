// ============================================================
// File    : button.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : ボタン入力処理と設定メニューの実装。
//           Button クラスによる短押し/長押し/ダブルクリック判定、
//           設定画面の全メニュー項目（目的地・音量・輝度・デモ地点など）の
//           ラベル生成・値変更コールバック定義。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================
// Handle button updates
#include "Button.h"
#include "mysd.h"
#include "gps.h"
#include "attitude.h"


// 実体は GPS_TFT_map.ino で volatile 定義。宣言側も volatile を付ける
// （割り込み・Core1 から参照されるため、修飾子が食い違うと最適化で不整合が起きる）
extern volatile int sound_volume;
extern volatile int vario_volume;
extern volatile bool vario_inhibit;
extern int screen_mode;
extern int destination_mode;
extern int detail_page;
extern int replay_cursor;      // リプレイ選択画面のカーソル位置（通し番号）
extern int replay_list_page;   // リプレイ選択画面の表示中ページ
extern volatile int scaleindex;  // 現在のマップスケール（scalelist のインデックス）
void reset_degpersecond();
void next_scaleindex();          // マップスケールを次の段階へ（GPS_TFT_map.ino）
float scale_screen_km(int index);  // 画面横幅が何 km か（GPS_TFT_map.ino）

const unsigned long debounceTime = 5;       // チャタリング除去のための待機時間 [ms]
const unsigned long longPressDuration = 1000; // 長押しと判定するまでの時間 [ms]
const unsigned long doubleClickDuration = 500; // 前の短押しからこの時間内に短押しでダブルクリック [ms]

Button::Button(int p, void (*shortPressCb)(), void (*longPressCb)(), void (*doublePressCb)())
    : pin(p), switchState(HIGH), lastSwitchState(HIGH), pressTime(0), lastShortPressTime(0), longPressHandled(false),
      shortPressCallback(shortPressCb), longPressCallback(longPressCb), doublePressCallback(doublePressCb) {}

// ボタンの状態を毎ループ読み取る。短押し・長押し・ダブルクリックを判定してコールバックを呼ぶ。
// 判定ロジック:
//   - 押下時 (HIGH → LOW): 押下開始時刻を記録
//   - 離した時 (LOW → HIGH): 押下時間が longPressDuration 未満なら短押し
//   - 押し続けている時: longPressDuration 以上経過で長押し（1 回だけ発火）
//   - 短押しが doubleClickDuration 以内に 2 回続いたら、2 回目でダブルクリックも発火
//     （短押し自体は毎回発火するので、ダブルクリック側の動作は短押しと重複しない処理にすること）
// チャタリング対策として 5ms 待ってから再読みしている。
// 短押し／ダブルクリックの操作音はコールバック側（GPS_TFT_map.ino）で鳴らす。
// 画面によっては何も起きないため、その場合に音だけ鳴るのを避ける目的。長押し音はここで鳴らす。
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
                    unsigned long now = millis();
                    if (shortPressCallback != NULL) {
                        shortPressCallback();
                    }
                    // 前回の短押しから doubleClickDuration 以内ならダブルクリック
                    if (doublePressCallback != NULL && lastShortPressTime != 0 &&
                        (now - lastShortPressTime) < doubleClickDuration) {
                        doublePressCallback();
                        lastShortPressTime = 0;  // 3 回目の短押しが続けてダブル判定にならないようリセット
                    } else {
                        lastShortPressTime = now;
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
      // 目的地名は destinations.csv 由来で長さが読めないため必ず snprintf を使う。
      // 固定部分だけで " Destination: "(14)+"(N)"(3)+NUL(1)=18 バイトあり、
      // sprintf のままだと名前が15文字以上でスタックを破壊する。
      if(currentdestination != -1 && currentdestination < destinations_count){
        snprintf(buff, sizeof(buff), selected ? " Destination: %s(%d)" : "Destination: %s(%d)", extradestinations[currentdestination].name, currentdestination);
      }else
        snprintf(buff, sizeof(buff), selected ? " Destination: %d/%d" : "Destination: %d/%d", currentdestination,destinations_count);
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
      // 目的地が未選択（起動直後は -1）なら配列外アクセスになるため何もしない
      if(currentdestination == -1 || currentdestination >= destinations_count)
        return;
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
      // 目的地が未選択（起動直後は -1）なら配列外アクセスになるため既定色を返す
      if(currentdestination == -1 || currentdestination >= destinations_count)
        return COLOR_RED;
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
      // 目的地が未選択（起動直後は -1）なら配列外アクセスになるため何もしない
      if(currentdestination == -1 || currentdestination >= destinations_count)
        return;
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
      sprintf(buff, selected ? " Volume(SE): %d/100" : "Volume(SE): %d/100", sound_volume);
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
  // [5] バリオメーター音量 (VARIO_VOLUME)
  //   ・Toggle: 0 → 5 → 10 → 20 → 30 → 40 → 60 → 80 → 100 → 0 のステップ
  //   ・アイコン色: 30以上なら緑（飛行中でも聞こえる音量）、それ未満はオレンジ
  // ----------------------------------------------------------
  { SETTING_VARIO_VOLUME,
    [](bool selected) -> std::string {
      if (vario_inhibit)
        return std::string(selected ? " Volume(Vario): Inhibit" : "Volume(Vario): Inhibit");
      char buff[32];
      sprintf(buff, selected ? " Volume(Vario): %d/100" : "Volume(Vario): %d/100", vario_volume);
      return std::string(buff);
    },
    nullptr,
    []() {
      if (vario_inhibit) return;  // inhibit 中は変更不可
      // Volume と同じステップ: 0 → 5 → 10 → 20 → 30 → 40 → 60 → 80 → 100 → 0
      if (vario_volume == 0) {
        vario_volume = 5;
      } else if (vario_volume == 5) {
        vario_volume = 10;
      } else if (vario_volume < 40) {
        vario_volume += 10;
      } else {
        vario_volume += 20;
      }
      if (vario_volume >= 101) vario_volume = 0;
      if (vario_volume <= 0) vario_volume = 0;
    },
    nullptr,
    [](){
      // inhibit 中は音が出ないので、音量値によらずオレンジ
      if(!vario_inhibit && vario_volume >= 30)
        return COLOR_GREEN;
      else
        return COLOR_ORANGE;
    }
  },

  // ----------------------------------------------------------
  // [6] マップ向き設定 (UPWARD)
  //   ・Toggle: NORTH UP ↔ TRACK UP を切り替え（toggle_mode() が内部状態を反転）
  //   ・アイコン色: TRACK UP なら緑（飛行時の推奨設定）、NORTH UP はオレンジ
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
    [](){
      if(is_trackupmode())
        return COLOR_GREEN;
      else
        return COLOR_ORANGE;
    }
  },

  // ----------------------------------------------------------
  // [7] マップ拡大率設定 (SCALE)
  //   ・Toggle: 地図画面のダブルクリックと同じ順序でスケールを 1 段階進める
  //             （広域 → … → 最大拡大 → 広域 とループ）
  //   ・表示は画面横幅の実距離。ベクタ地図に移行して Google タイルのズーム番号との
  //     対応が無くなったため、pilot にとって意味のある距離表示にしてある。
  //   ・アイコン色: scaleindex==3 なら緑（通常使用する拡大率）、それ以外は赤
  // ----------------------------------------------------------
  { SETTING_SCALE,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      float km = scale_screen_km(scaleindex);
      if (km >= 10.0f)
        sprintf(buff, selected ? " Map scale: %dkm" : "Map scale: %dkm", (int)(km + 0.5f));
      else
        sprintf(buff, selected ? " Map scale: %.1fkm" : "Map scale: %.1fkm", km);
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      next_scaleindex();
    },
    nullptr,
    [](){
      if(scaleindex == 3)  // SCALE_LARGE_GMAP（通常使用）
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [8] デモモード (DEMOBIWA)
  //   ・Toggle: デモ地点を 1 つ進める（OFF → BIWAKO → SHIRAHAMA → KASAOKA
  //             → FUJIGAWA → TOKYO → OSAKA → OFF）。地点ごとに仮想機体をその中心へ置き、
  //             位置履歴と旋回角速度をリセットする。
  //   ・各地点の地図表示を実機で確認する用途にも使える。
  //   ・Exit  : デモがオンになったらリプレイを無効にして設定を閉じる（両立しない）
  //   ・アイコン色: デモがオフなら緑（通常飛行中）、オンなら赤
  // ----------------------------------------------------------
  { SETTING_DEMOBIWA,
    [](bool selected) -> std::string {
      char buff[32];  // temporary buffer
      sprintf(buff, selected ? " DEMO: %s" : "DEMO: %s", get_demo_site_name(get_demo_site()));
      return std::string(buff);  // return as std::string
    },
    nullptr,
    []() {
      next_demo_site();             // 地点を 1 つ進める（内部で位置履歴もリセット）
      reset_degpersecond();         // 旋回角速度をリセット
    },[](){
      if(is_demo_active()){
        set_replaymode(false);  // デモとリプレイは同時使用不可 → リプレイを無効化
        exit_setting();
      }
    },
    [](){
      if(!is_demo_active())
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [9] フライトリプレイモード (REPLAY)
  //   ・Enter : リプレイ選択画面へ遷移し、Core1 へファイル一覧の取得を依頼する。
  //             再生の開始／停止はすべて選択画面側で行う。
  //   ・Toggle/Exit: なし
  //   ・アイコン色: リプレイがオフなら緑（通常モード）、オンなら赤
  // ----------------------------------------------------------
  { SETTING_REPLAY,
    [](bool selected) -> std::string {
      char buff[40];  // temporary buffer
      // 再生中は対象ファイル名（パスを除いた部分）を表示する
      const char* label = "OFF";
      if (getReplayMode()) {
        label = get_replay_filename();
        if (strcmp(label, REPLAY_2025_FILE) == 0)      label = REPLAY_2025_LABEL;
        else if (strcmp(label, REPLAY_2026_FILE) == 0) label = REPLAY_2026_LABEL;
      }
      snprintf(buff, sizeof(buff), selected ? " REPLAY: %s >" : "REPLAY: %s >", label);
      return std::string(buff);  // return as std::string
    },
    []() {
      screen_mode = MODE_REPLAYSELECT;  // リプレイ選択画面に切り替え
      replay_cursor = 0;                // カーソルを先頭に戻す
      replay_list_page = 0;
      loading_replaylist = true;
      enqueueTask(createBrowseReplayTask(replay_menu_file_start_for_page(0)));
    },
    nullptr,
    nullptr,
    [](){
      if(!getReplayMode())
        return COLOR_GREEN;
      else
        return COLOR_RED;
    }
  },

  // ----------------------------------------------------------
  // [10] GPS 詳細画面へ (GPSDETAIL)
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
  // [11] マップ一覧画面へ (MAPDETAIL)
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
  // [12] SD カード詳細画面へ (SD_DETAIL)
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
  // [13] Vario 詳細画面へ (VARIO_DETAIL)
  //   ・Enter: ページ番号を 0 にリセットし、Vario 詳細画面へ遷移
  //            （Status+VSI / IMU+Kalman の 2 ページで表示）
  //   ・Toggle/Exit/アイコン色: なし
  // ----------------------------------------------------------
  { SETTING_VARIO_DETAIL,
    [](bool selected) -> std::string {
      return "Vario detail >";
    },
    []() {
      screen_mode = MODE_VARIODETAIL;  // Vario 詳細画面に切り替え
      detail_page = 0;                 // ページを先頭に戻す
    },
    nullptr,
    nullptr,
    nullptr
  },

  // ----------------------------------------------------------
  // [14] IMU / 姿勢 ESKF 詳細 (IMU_DETAIL)
  //   ※ 機体ゼロ点の較正はこの画面内で短押しして実行する（設定メニューには置かない）
  //   ・Enter: IMU/ESKF 画面へ切り替え
  //   ・静止中のジャイロ実測値・推定バイアス・ESKF と BNO085 の姿勢比較を見る
  //   ・アイコン色: バンク警告と自動ロールトリムが両方 ON なら緑、片方でも OFF ならオレンジ
  //     （誤警報対策を切ったまま飛ばないための目印。IMU/ESKF 画面の右上と同じ意味）
  // ----------------------------------------------------------
  { SETTING_IMU_DETAIL,
    [](bool selected) -> std::string {
      return "IMU / ESKF >";
    },
    []() {
      screen_mode = MODE_IMUDETAIL;
      detail_page = 0;
      extern int imu_cursor;
      imu_cursor = 0;
      // 較正の注意点（ロールが 0 であること・飛行中に実行しないこと）を音声で案内する。
      // 優先度 3 = コース警告と同等。誤った較正は警告機能そのものを壊すため。
      enqueueTask(createPlayWavTask("wav/guide_eskf_setting.wav", 3));
    },
    nullptr,
    nullptr,
    [](){
      extern volatile bool bank_warning_enabled;
      if (bank_warning_enabled && attitude_get_roll_trim_enabled())
        return COLOR_GREEN;
      else
        return COLOR_ORANGE;
    }
  },

  // ----------------------------------------------------------
  // [15] 保存して終了 (EXIT)
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



