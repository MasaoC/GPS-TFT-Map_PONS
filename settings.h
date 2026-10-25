// ============================================================
#pragma once
// File    : settings.h
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GPS 選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// リリース時
#define RELEASE

#define BUILDDATE 20261025
// v5 ハード向けの版。0.893 に v6（0.894〜0.949）と v7（0.950〜0.995）の改良のうち
// v5 のハードで動くものを移植したもの（README の「v6・v7 からの移植（2026-10）」）。
// 末尾の "v5" は、v6 / v7 の番号と重ならないようにするため。
#define BUILDVERSION "0.894v5"
#define VERSION_TEXT "Version 5"




//----------GPS---------
// GPS は Quectel LC86G（LC86GPAMD）固定（v5 ハードウェア）。NMEA と PAIR コマンドで話す。
// ★ 測位は 1Hz（このチップの仕様。2Hz にはできない）。
//#define DEBUG_NMEA

//GPSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GPS_SIM_BIWAKO         //琵琶湖固定座標
  //#define DEBUG_GPS_SIM_SAPPORO         //札幌固定座標
  //#define DEBUG_GPS_SIM_SHISHI         //しし固定座標
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換


//---------TFT----------
// パネル種別（ST7789 / ILI9341）の選択は TFT_eSPI の User_Setup で行う。
// このファイルには選択用マクロは無い（コード側に分岐が無く、置いても効かないため）。
// 設定サンプルは TFT_eSPI/CopySetupFile_TFT_eSPI.h を参照。

#define VERTICAL_FLIP

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 1050


// 設定画面を放置したとき、自動で地図画面へ戻るまでの時間 [ms]（v7 の 0.965）。
// 設定画面を出したまま離陸してしまった場合の保険。ボタン操作のたびに計時し直す。
// 3 分にしてあるのは、設定を眺めながら考え込む程度では抜けず、
// かつ離陸前の点検から離陸までの間には確実に戻る長さだから。
#define SETTING_IDLE_TIMEOUT_MS (3UL * 60UL * 1000UL)

#define MAX_TRACK_CORDS 500



// =====Hardware Settings =====

// Hardware Ver5（Raspberry Pi Pico 2 モジュール = RP2350A・Flash 4MB）
#define SW_PUSH 26
#define BATTERY_PIN 29 // ADC3。Pico 2 基板上の VSYS÷3 の分圧
#define GPS_SERIAL Serial1
// GPS の UART 受信 FIFO [バイト]。LC86G の NMEA は 1Hz で約 0.6〜1.5KB/s（衛星表示中は GSV が増える）。
// 0.893 までは 1024（「128 では不足するケースあり」）。地図の描画や SD の待ちで Core0 が長く止まっても
// 取りこぼさないよう v6 と同じ 4096 にした（RAM は全体の 1% 未満）。
#define GPS_SERIAL_FIFO_SIZE  4096
// GPS モジュールからのバイトがこの時間届かなければ「断線」とみなす [ms]（v7 の 0.979）。
// ★ **飛行中に断線したことを検出するためのもの。** 以前は
//   「1 バイトでも受信したら true、以降ずっと true」のラッチだったので、
//   途中でコネクタが抜けても画面は正常なまま航法を続けていた。
// 値の根拠: LC86G は 1Hz で RMC/GGA/GSA/GST を流し続ける。5 秒 = 5 エポックぶん欠けたら断線。
#define GPS_CONN_TIMEOUT_MS   5000UL
// 断線している間、警報トーンを繰り返す間隔 [ms]。
// ★ 1 回きりにしない。**聞き逃したら二度と鳴らない**のは航法が止まる故障には合わない。
#define GNSS_LOST_ALERT_MS    30000UL
// 地図に出す GNSS 断線の枠の縦位置（backscreen の座標。高さ 56 → 44..99）
#define NOTICE_Y_GNSS_LOST    44
// LC86G の EASY（受信した軌道から GPS 自身が数日先まで軌道を予測する機能。u-blox の
// AssistNow Autonomous に当たる）。1 = 使う / 0 = 使わない。起動時に PAIR490 で書き込む。
// ★ 0 にしても「使わない」を**書き込む**（以前の設定が GPS 側に残り続けないように）。
// ★ 予測データはバックアップ電池（基板の MS621F）で保たれる。電池が切れていると毎回コールドスタート。
#define GNSS_USE_EASY             1
#define GPS_TX 0
#define GPS_RX 1
#define USB_DETECT 24  // Pico 2 の VBUS 検出（GPIO24）
#define RP_CLK_GPIO 2 // Set to CLK GPIO
#define RP_CMD_GPIO 3 // Set to CMD GPIO
#define RP_DAT0_GPIO 4 // Set to DAT0 GPIO. DAT1..3 must be consecutively connected. DAT1=5, DAT2=6, DAT3=7
#define SD_CS_SPI_PIN 7 // SPI フォールバック時の CS ピン（DAT3 = GPIO7）
#define SD_DETECT 8     // カード検出（J4 の DET_A。挿入時 LOW）

#define BATTERY_MULTIPLYER(adr) (0.002415812*adr) //VSYS 1/4096*3.3*3
#define BAT_HALF_VOLTAGE 3.8 // 50%未満 (4.2-3.4=0.8V の半分は0.4Vなので4.2-0.4=3.8Vが50%の目安)
#define BAT_LOW_VOLTAGE 3.5
#define BAT_ZERO_VOLTAGE 3.4
#define BAT_FULL_VOLTAGE 4.2 // 100% とみなす電圧。残量 % と残り時間の基準
// ---- 電池電圧のピーク追跡（display_tft.cpp の get_input_voltage。v7 の 0.989）----
// 音声再生や TFT 書き込みで電圧は一瞬落ちる。あれは負荷の影響で電池そのものの
// 電圧ではないので、**上がるときは即追従（ピーク）・下がるときはゆっくり**にする。
// ★ 下降は **時間で**鈍らせること。「呼ばれるたびに 0.2 ずつ寄せる」作りだと
//   **呼び出し元の数で時定数が変わる**（電池低下の判定や本番前チェックが毎秒呼ぶと、
//   そのぶん表示の下がり方が速くなる）。
#define BATTERY_PEAK_HOLD_MS  3000UL   // ピーク更新からこの間は下げない
#define BATTERY_DECAY_TAU_MS 20000UL   // 下降の時定数。長くすると表示が落ち着く
// 満充電からの目安稼働時間 [分]。設定画面フッターの「Battery Time」の計算に使う。
#define BAT_FULL_RUNTIME_MIN 240
// 電池低下警告を繰り返す間隔 [ms]。電池は減り続けるので 1 回きりの通知では足りない。
// 逆に短くすると、本当に聞くべき他の警報（コース警報）が埋もれる。
#define BAT_WARN_INTERVAL_MS 60000UL

// ---- 電源経路の異常（power.cpp。v7 の 0.993 と同じ判定）----
// 2026-10-15、v7 の充電 IC（MCP73831）が電池につながったまま内部で壊れ、
// 電池 → 充電 IC → VBUS → VSYS と回って **SW で切れず**、画面は「USB」になった。
// v5 も同じ考え方の電源回路（充電 IC の VBAT は電源スイッチの手前、VDD は VBUS）。
// 壊れた充電 IC が VBUS を電池電圧まで持ち上げると、Pico 2 の VBUS 検出（GPIO24）は
// High になるが、VSYS は Pico 2 内蔵のダイオード越しの電池電圧（4V 未満）にしかならない。
// 本物の USB なら VSYS は約 4.7V あるので、「USB 判定なのに電圧が低い」が続いたら異常とみなす。
//   USB_DETECT が High なのに VSYS < POWER_FAULT_VSYS_V
#define POWER_FAULT_VSYS_V      3.9f
#define POWER_FAULT_HOLD_MS     3000UL  // この時間続いたら異常（USB を抜いた直後の過渡を除く）
#define POWER_FAULT_ARM_MS      5000UL  // 起動直後は判定しない
#define POWER_FAULT_ALERT_MS   30000UL  // 警報音を繰り返す間隔
// 異常時に画面で頼む操作。v5 は充電 IC が SW の手前なので、SW を切っても電池から切れない。
#define POWER_FAULT_ACTION_TEXT "UNPLUG THE BATTERY"
// ---- 高温での充電の警告（power.cpp。v7 の 0.994 を v5 向けに）----
// 電池 DTP603048 の仕様は「充電 0〜45°C」（放電は −20〜60°C）。
// ★ v5 には温度センサーが無いので、**マイコン（RP2350）内蔵の温度（CPU 温度）**で見る。
//   CPU 温度は電池の温度そのものではなく、マイコン自身の発熱で周囲より数〜10°C 高く出る
//   （＝早めに警告する側）。目安として使う（判定は v3 の 0.503 と同じ値）。
// ★ 温度センサーの電圧を ADC で読むので、ADC の基準電圧（Pico 2 は 3.3V）が 1% ずれると約 4°C ずれる。
//   設定画面の下の CPU 温度が、電源を入れて数分後に室温 +5〜10°C 前後なら合っている。
#define CHARGE_HOT_C           45.0f    // これ以上が CHARGE_HOT_HOLD_MS 続いたら警告
#define CHARGE_HOT_CLEAR_C     42.0f    // これ以下で解除（境目で出たり消えたりしない）
#define CHARGE_HOT_HOLD_MS     30000UL
#define CHARGE_HOT_ALERT_MS    60000UL  // 続いている間、読み上げをくり返す間隔
#define ADC_VREF_V              3.3f    // Pico 2 の ADC_VREF（3V3 から 200Ω と 2.2µF の RC で作られる）
#define CPU_TEMP_SAMPLE_MS    2000UL    // analogReadTemp() は 1 回 1ms ほど止まるので間引く
#define CPU_TEMP_MIN_C       (-20.0f)   // この外の値は読み違いとして使わない（警告もしない）
#define CPU_TEMP_MAX_C         90.0f
// ---- 本番前チェック（readiness.cpp。v7 の 0.994）----
// 飛ぶ前に確かめる項目がそろったら、地上で止まっているときに 1 度だけ「準備完了」と読み上げる。
// そろっていなければ、何が足りないかを設定画面の右上に出す。項目は readiness.h の RDY_*。
#define READY_HACC_MAX_M        5.0f     // GPS の水平精度（GST の推定）。GPS DETAIL で緑になる値と同じ
#define READY_HDOP_MAX          2.0f     // GST が来ていないときは HDOP で見る（2.0 以下 ≒ 良い配置）
#define READY_BATT_MIN_PERCENT  50       // 電池残量 [%]（battery_percent。50% ≒ 3.8V）
#define READY_STILL_MPS         1.0f     // 対地速度がこれ以下なら「地上で止まっている」
#define READY_HOLD_MS           5000UL   // そろった状態がこれだけ続いたら読み上げる（一瞬そろっただけでは言わない）
#define READY_REARM_MS          60000UL  // 欠けた状態がこれだけ続いたら、もう一度読み上げられるようにする

// 地図に出す注意の縦位置（backscreen の座標）。電源経路の異常と高温は同じ段（同時には出さない）。
#define NOTICE_Y_PWR_FAULT   190        // 高さ 40 → 190..229（地図の下端）
#define NOTICE_Y_CHG_HOT     NOTICE_Y_PWR_FAULT
#define PIN_PWMTONE 9
#define PIN_AMP_SD 10 //アンプシャットダウン(HIGHでON)
// ★ v5 には USERLED が無い（基板の LED は充電表示だけ）。SD の異常・衛星 0 は画面と音で知らせる。
// 音量の上限。設定画面のステップ送りと、SD の settings.txt から読むときの
// クランプの両方がこれを見る（片方だけ直して食い違うのを防ぐ）。
// ※ v5 は基板に音量のつまみ（RV1）もある。こちらはソフトの音量で、つまみと掛け算になる。
#define SOUND_VOLUME_MAX 100

#define SIN_VOLUME 0.15f  // Sin波の振幅倍率（0〜1.0f）。WAVと音量を合わせるため小さめにしてある。0.5fで±254、1.0fで±508（±512ヘッドルーム）。

// =====追加設定項目====
// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// hAcc 不確かさ円 設定
// GPS fix があっても hAcc（GST の水平精度推定）が大きい場合、飛行機マークの代わりに青い不確かさ円を表示する。
// GST がまだ届いていないとき（hAcc 不明）は円を出さない（get_gps_hacc_mm() のコメント）。
#define HACC_THRESHOLD_M       10.0f // この値（m）以上で不確かさ円モードに切替え（旧 HDOP=2×5m 相当）
#define HDOP_MIN_CIRCLE_RADIUS 4     // 輪郭円を描画する最小半径 [px]（未満はテキスト表示に切替え）
#define HDOP_CENTER_DOT_RADIUS 3     // 中心位置を示す塗りつぶし小円の半径 [px]



//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GPSDETAIL 3
#define MODE_MAPLIST 4
#define MODE_SDDETAIL 5
#define MODE_REPLAYSELECT 7  // リプレイ再生ファイルの選択画面


//======= リプレイ再生設定 ======
// 固定エントリ（大会データ）の SD 上のパス。
// SD ルートに置くとファイル一覧にも重複表示されるため replay/ サブフォルダに置く。
#define REPLAY_2025_FILE "replay/2025taikai.csv"
#define REPLAY_2026_FILE "replay/2026taikai.csv"
#define REPLAY_2025_LABEL "2025 Taikai"
#define REPLAY_2026_LABEL "2026 Taikai"

#define REPLAY_BUF_SIZE   16   // 先読みするCSV行数（Core1が供給 → Core0が消費するリングバッファ）
                               // ※ インデックス計算にビットマスクを使うので必ず 2 のべき乗にすること
                               // 高速再生(x20)では 2Hz データを毎秒40行消費するため余裕を持たせている
#define REPLAY_LIST_ROWS  20   // リプレイ選択画面の1ページあたり表示行数（20行×12px = 240px）
#define REPLAY_FIXED_COUNT 5   // 一覧の先頭に並ぶ固定項目数の最大値
                               // （Replay OFF / PLAY FLIGHT ONLY / PLAY SPEED / 2025 / 2026）
                               // 2025・2026 は SD 上に実ファイルがあるときだけ表示するため、
                               // 実際の数は replay_menu_fixed_count()（3〜5）を使うこと。
// 再生速度の選択肢。x1 → x2 → x_FAST の順に切り替わる（既定は x1）。
// 高速側の倍率を変えたいときは REPLAY_SPEED_FAST だけ書き換えればよい。
#define REPLAY_SPEED_FAST 20
#define REPLAY_MIN_GS 0.2f     // PLAY FLIGHT ONLY 有効時、この対地速度 [m/s] 以下を「静止」とみなす
#define REPLAY_LEADIN_MS 5000  // 動き出しの手前この時間 [ms] 分は静止していても再生する（助走表示）。
                               // 静止区間がこの長さ未満なら一切スキップしない。
#define REPLAY_LEADIN_SLOTS 40 // 助走区間の先頭を探すために保持する行数（5秒 ÷ 最短サンプル間隔ぶん）
#define REPLAY_SCAN_MAX 2000   // 静止区間の先読み走査で1回に読む最大行数（Core1 を長時間占有しないための上限）
#define REPLAY_REQ_INTERVAL_MS 200  // Core1へのバッファ補充依頼の最短間隔 [ms]
#define REPLAY_FILENAME_LEN 40      // リプレイ対象ファイル名（パス込み）の最大長


#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GPS)
    #warning NOT RELEASE!
  #endif
#endif


//デバッグ用 print マクロ
#define PRINTREVERSEDATE_NUM 10
#ifndef RELEASE
  #define DEBUG_P(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt);
  #define DEBUG_PN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt,num);
  #define DEBUGW_P(date,txt)  Serial.print(txt);
  #define DEBUG_PLN(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt);
  #define DEBUG_PNLN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt,num);
  #define DEBUGW_PLN(date,txt)  Serial.println(txt);
#else
  #define DEBUG_P(date,txt)  
  #define DEBUG_PN(date,txt,num)  
  #define DEBUGW_P(date,txt)  
  #define DEBUG_PLN(date,txt)
  #define DEBUG_PNLN(date,txt,num)   
  #define DEBUGW_PLN(date,txt)  
#endif

// ============================================================
// スタック残量計測マクロ
//
// Core0: リンカシンボル __StackLimit（スタック下限）と現在 SP の差 = 残りバイト数
// Core1: core1_separate_stack=true のため別ヒープから確保。
//        setup1() 先頭でキャプチャした _core1_base_sp との差 = 使用済みバイト数
//
// ※ 5 秒に 1 回だけ出力するのでシリアルが流れすぎない
// ============================================================
extern volatile uint32_t _core1_base_sp;  // GPS_TFT_map.ino で定義


#ifndef RELEASE
  #define DEBUG_STACK_C0(label) do { \
    static unsigned long _t0_; \
    if (millis() - _t0_ >= 5000) { \
      _t0_ = millis(); \
      Serial.print("[C0 stack free:" label "]="); \
      Serial.println((int)rp2040.getFreeStack()); \
    } \
  } while(0)

  // ※ 現在 DEBUG_STACK_C1 の呼び出し箇所は無い。Core1 のスタックを測りたくなったら
  //   loop1() などに挿して使う（RELEASE ビルドでは空マクロになる）。
  #define DEBUG_STACK_C1(label) do { \
    static unsigned long _t1_; \
    if (millis() - _t1_ >= 5000) { \
      _t1_ = millis(); \
      uint32_t _sp_; \
      asm volatile ("mov %0, sp" : "=r" (_sp_)); \
      Serial.print("[C1 stack used:" label "]="); \
      Serial.println((int)(_core1_base_sp > _sp_ ? _core1_base_sp - _sp_ : 0)); \
    } \
  } while(0)

  // ---- 処理時間計測ユーティリティ ----
  // 使い方: TIMING_START(名前) → 重い処理 → TIMING_END(統計変数, 名前)
  // 30秒ごとに TIMING_REPORT で max/avg/count を Serial 出力する。
  struct TimingStat {
    uint32_t max_us;   // ワーストケース（マイクロ秒）
    uint32_t sum_us;   // 累計（平均計算用。オーバーフロー対策として count も管理）
    uint32_t count;    // 呼び出し回数
    const char* label; // 識別ラベル文字列
  };
  #define TSTAT_INIT(label_str) { 0, 0, 0, (label_str) }

  // TIMING_START: ローカル変数にタイムスタンプを記録する
  #define TIMING_START(var)  uint32_t _ts_##var = time_us_32()

  // TIMING_END: 経過時間を計算して統計変数を更新する
  #define TIMING_END(stat, var) do { \
    uint32_t _te_ = time_us_32() - _ts_##var; \
    if (_te_ > (stat).max_us) (stat).max_us = _te_; \
    (stat).sum_us += _te_; \
    (stat).count++; \
  } while(0)

  // TIMING_REPORT: 統計を Serial に出力する（count > 0 の時のみ）
  #define TIMING_REPORT(stat) do { \
    if ((stat).count > 0) { \
      Serial.print("[TIME] "); Serial.print((stat).label); \
      Serial.print(" max="); Serial.print((stat).max_us); \
      Serial.print("us avg="); Serial.print((stat).sum_us / (stat).count); \
      Serial.print("us n="); Serial.println((stat).count); \
    } \
  } while(0)

#else
  #define DEBUG_STACK_C0(label)
  #define DEBUG_STACK_C1(label)
  // RELEASE ビルド: 計測マクロはすべて空にしてオーバーヘッドゼロにする
  #define TIMING_START(var)
  #define TIMING_END(stat, var)
  #define TIMING_REPORT(stat)
#endif

// ============================================================
// 高度表示設定
// ============================================================
// 当デバイスの GPS は NMEA GGA の海抜高度（ジオイド基準）を使用する。
// これは日本の標高（T.P.=東京湾平均海面 基準）とほぼ一致する（差は ±数十cm 程度）。
//
// ※注意: GPS の「楕円体高（WGS84）」とは異なる。
//   楕円体高 = 海抜高度 + ジオイド高 N（日本では N ≈ +36〜38m）
//   GGA は既に海抜高度を出力しているので、-37m 補正は不要。
//
// ELEVATION_GEOID_OFFSET_M: B.S.L. 高度 = GPS 海抜高度 - この値 [m]
//   表示ラベルは "B.S.L."（琵琶湖基準水位面、Biwa Standard Level）。
//   B.S.L. 0m = T.P.（日本の標高） +84.371m（瀬田川洗堰基準・国土交通省設定値）。
//   → 琵琶湖面で B.S.L. ≈ 0m、湖面上 100m 飛行時に B.S.L. ≈ 100m と表示される。
//   標高（T.P.）に戻したい場合は 0.0f に変更する。
#define ELEVATION_GEOID_OFFSET_M  84.371f  // B.S.L.基準補正 [m]（T.P.84.371m = 琵琶湖面）