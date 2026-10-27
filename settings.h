// ============================================================
#pragma once
// File    : settings.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GNSS/TFT種別選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/23
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// ============================================================
//  リリース前に確認するスイッチ
// ============================================================
// ★ RELEASE が無いと setup1() が while(!Serial) で USB 接続を待ち、実機が起動しない。
//   RELEASE_GNSS 以外（DEBUG_GNSS_SIM_*）を選んだままだと GNSS が疑似値になる。
//   どちらかが抜けているとコンパイル時に #warning NOT RELEASE! が出る。これが唯一の保険。
// リリース時
#define RELEASE
//#define DEBUG_ESKF

#define BUILDDATE 20261027
#define BUILDVERSION "0.997v71"

// ============================================================
//  基板（v7.1。SCH16T を J6 の FPC コネクタで外付け）
// ============================================================
// このブランチ（PONSv7.1）は **v7.1 基板専用**。初代 v7 基板（BNO085 搭載）は PONSv7 ブランチで、
// ピン割り当てがまるで違う（2026-10-23 にブランチごと分けた）。
//   v7.1 だけの事情があるピン:
//     GPIO41-44 SPI1（SCH16T の CS / SCK / MOSI / MISO）
//     GPIO45    SCH16T EXTRESN / GPIO46 SCH16T DRY(SYNC)
//     GPIO47    USB_SENS（VBUS の分圧を ADC7 で測る）
//     GPIO31    E220 の M0/M1
//     GPIO34/35 TFT バックライト（34=PWM 出力、35=輝度スイッチ）
//     GPIO40    BATPWR（電池）の分圧（51/151）。SW OFF で 0V（SW4 が GND に落とす。power.h）
// ★ GPIO42/44 を INPUT_PULLUP で停めないこと。v7 で I2C を守るためにやっていた処置で、
//   v7.1 では **SPI1 が死ぬ**。

// 起動画面に出す基板世代の表記
#define VERSION_TEXT "Version 7.1"

//----------GNSS---------
// GNSS は u-blox SAM-M10Q 固定（v6 ハードウェア）。
// v5 以前の Quectel LC86G / Mediatek 対応コードは v0.947 で削除した。
//#define DEBUG_GBX_NMEA

//GNSSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GNSSを選択】
  #define RELEASE_GNSS
  //#define DEBUG_GNSS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GNSS_SIM_BIWAKO         //琵琶湖固定座標
  //#define DEBUG_GNSS_SIM_SAPPORO         //札幌固定座標
  //#define DEBUG_GNSS_SIM_SHISHI         //しし固定座標
  //#define DEBUG_GNSS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GNSS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GNSS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換

#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GNSS)
    #warning NOT RELEASE!
  #endif
#endif

// ============================================================
//  ハードウェア — ピン割り当て
// ============================================================

// Hardware Ver6
#define SW_PUSH 37  // v7 35->37に変更
#define BATTERY_PIN 40 //A0
#define GNSS_SERIAL Serial1

// GNSS の UART 受信リングバッファ [バイト]。既定 32 では全く足りない。
// ★ 大きさの意味は「**Core0 が止まっても取りこぼさない時間**」。
//   実流量は UBX のみで約 550 B/s（NAV-PVT 100B×2Hz ＋ NAV-SAT 約320B×1Hz
//   ＋ NAV-DOP 26B×1Hz ＋ NAV-AOPSTATUS 24B×1Hz）。回線速度 38400bps(=3840 B/s) ではなく**これで割る**。
//     1024B → 約 1.9 秒   4096B → 約 7.4 秒
//   以前のコメントは 1024B ÷ 3840B/s = 267ms と書いていたが、それは
//   **回線が飽和している前提の最悪値**で、実態の 7 倍きつい見積りだった。
//   RAM は 4096B でも全体の 1% 未満。ここを削る理由が無い。
#define GNSS_SERIAL_FIFO_SIZE  4096

// GNSS モジュールからのバイトがこの時間届かなければ「断線」とみなす [ms]。
// ★ **飛行中に断線したことを検出するためのもの。** 以前は
//   「1 バイトでも受信したら true、以降ずっと true」のラッチだったので、
//   途中でコネクタが抜けても画面は正常なまま航法を続けていた。
// 値の根拠: 設定が通っていれば NAV-PVT が 2Hz（500ms 間隔）、通っていなくても
//   既定の NMEA が 1Hz で流れ続ける。一方こちらの読み出しが遅れても FIFO
//   （GNSS_SERIAL_FIFO_SIZE = 4096B ≒ 7 秒ぶん）に溜まるので、描画で数百 ms
//   止まっても取りこぼさない。実測の Core0 の停止は数十 ms（imu gapmax=）。
//   5 秒なら「本当に来ていない」ときだけ立つ。
#define GNSS_CONN_TIMEOUT_MS   5000UL
// 断線している間、警報トーンを繰り返す間隔 [ms]。
// ★ 1 回きりにしない。**聞き逃したら二度と鳴らない**のは航法が止まる故障には合わない。
// ★ 最低保証音量（min_volume）は付けない。あれは電池警告専用で
//   「離れていても気づける」ための仕組み（gnss.cpp の REPLAY canceled のコメント参照）。
//   断線は飛行中に起きるので、パイロットは装置を身につけている。
#define GNSS_LOST_ALERT_MS    30000UL
// 「測位しました」（fixed.wav）を一度鳴らしたら、この間は鳴らさない [ms]。
// GNSS が不安定な所では fix が付いたり外れたりを繰り返し、毎秒のように連発してうるさかった。
// ログ（GNSS FIX acquired）は毎回残す。
#define GNSS_FIXED_VOICE_GAP_MS 10000UL
// GNSS の AssistNow Autonomous（受信した軌道から GNSS 自身が数日先まで軌道を予測する機能）。
// 1 = 使う / 0 = 使わない。効けば前日と同じ時間帯の起動が 23〜28 秒 → 3〜4 秒になる。
// 条件（BT1 が生きている・3 日以内・同じ時間帯）は README の「GNSS の起動を速くする」。
// ★ 0 にしても「使わない」を GNSS へ**書き込む**。この設定は BT1 で保たれる BBR にも
//   書くので、送るのをやめるだけだと以前のファームが書いた「使う」が残り続ける。
#define GNSS_USE_ANA              1
// AssistNow Autonomous の状況（予測データのある衛星数・計算していた時間）を
// log.txt に残す間隔 [ms]。計算中は GNSS の消費電流が増えるので、その割合を見るためのもの。
#define GNSS_ANA_LOG_INTERVAL_MS  600000UL
// ---- 本番前チェック（readiness.cpp）----
// 飛ぶ前に確かめる項目がそろったら、地上で止まっているときに 1 度だけ「準備完了」と読み上げる。
// そろっていなければ、何が足りないかを設定画面の右上に出す。項目は readiness.h の RDY_*。
#define READY_HACC_MAX_M        5.0f     // GNSS の水平精度。GNSS DETAIL 3 で緑になる値と同じ
#define READY_BATT_MIN_PERCENT  50       // 電池残量 [%]（battery_percent。50% ≒ 3.8V）
#define READY_STILL_MPS         1.0f     // 自機の対地速度がこれ以下なら「地上で止まっている」
#define READY_HOLD_MS           5000UL   // そろった状態がこれだけ続いたら読み上げる（一瞬そろっただけでは言わない）
#define READY_REARM_MS          60000UL  // 欠けた状態がこれだけ続いたら、もう一度読み上げられるようにする
#define GNSS_TX 0
#define GNSS_RX 1
// USB の有無（GPIO47 の ADC で VBUS を測る）。
// ★ 直接 digitalRead() しないこと。判定は power.cpp の usb_power_present() だけ。
  #define USB_VBUS_ADC_PIN 47   // R34 10k / R35 5.1k（BATTERY_PIN と同じ比）
  // ★ 分圧を 100k/51k のように弱くしないこと。電池だけのとき D3 の逆漏れ電流で VBUS が
  //   浮き（151kΩ だと 13µA で 2V）、電源異常を誤報する。
#define RP_CLK_GPIO 2 // Set to CLK GPIO
#define RP_CMD_GPIO 3 // Set to CMD GPIO
#define RP_DAT0_GPIO 4 // Set to DAT0 GPIO. DAT1..3 must be consecutively connected. DAT1=5, DAT2=6, DAT3=7
#define SD_CS_SPI_PIN 7 // SPI フォールバック時の CS ピン（DAT3 = GPIO7）
#define SD_DETECT 10  //v7 8->10に変更
// ============================================================
//  電源・バッテリー
// ============================================================
// BATPWR（ロードスイッチ Q3 の後ろ＝電池）を 51/151 で分圧して測る。
#define BATTERY_MULTIPLYER(adr) (0.00238423334*adr) // 1/4096*3.3*(151/51)=0.00238423334
// ---- 電源経路の異常（power.cpp）----
// 2026-10-15、v7 の充電 IC（MCP73831）が電池につながったまま内部で壊れ、
// 電池 → 充電 IC → VBUS → D3 → VSYS と回って **SW で切れず**、画面は「USB」になった
// （VSYS は 2.6V）。本物の USB なら VSYS は 4.3V 以上あるので、
// 「USB 判定なのに電圧が低い」状態が続いたら異常とみなす。
// v7.1 では VBUS が POWER_FAULT_VBUS_MIN_V〜USB_VBUS_PRESENT_V の間に居続けたら異常
//   （故障なら VBUS は電池電圧を超えられない。抜いた後の放電は 1 秒以内に通り過ぎる）
#define USB_VBUS_PRESENT_V      4.3f    // これ以上なら本物の USB
#define POWER_FAULT_VBUS_MIN_V  2.0f    // これ未満は USB なし
#define POWER_FAULT_HOLD_MS     3000UL  // この時間続いたら異常（USB を抜いた直後の過渡を除く）
#define POWER_FAULT_ARM_MS      5000UL  // 起動直後は判定しない
#define POWER_FAULT_ALERT_MS   30000UL  // 警報音を繰り返す間隔
// 異常時に画面で頼む操作。充電 IC が Q3 の後ろなので SW で電池から切れる（USB からの給電は抜けば止まる）。
  #define POWER_FAULT_ACTION_TEXT "SW OFF + UNPLUG USB"
// ---- SW を OFF にしたまま USB で充電している（power.cpp）----
// 充電 IC が Q3（SW で開閉するロードスイッチ）の後ろにあるので、SW OFF のまま
// USB を挿すと電池へは Q3 のボディダイオード経由になり、**4.2V より 0.5V 前後低いところで
// 充電が終わる**。しかも本体は USB で起動し、充電 LED も消えるので、見た目では
// 満充電と区別が付かない。docs/pons_power.md。
// 検出: SW4 の OFF 側の接点が GPIO40（電池電圧の分圧点）を GND に落とす。
//   SW ON なら GPIO40 は電池電圧（保護回路付きの 1s なので 2.5V 以上）、OFF なら 0V。
#define CHARGE_SW_OFF_BATT_V     1.0f    // GPIO40 の読み（電池電圧換算）がこれ未満なら SW OFF
#define CHARGE_SW_OFF_HOLD_MS    2000UL  // この時間続いたら注意と読み上げを出す（スイッチを行き来させている途中で喋らない）
#define CHARGE_SW_OFF_ARM_MS    15000UL  // 起動からこの時間は読み上げない（起動音・無線モードの読み上げと重ねない）
// 「移動中は出さない」の判定。**自機の** GNSS の対地速度で見る。
// ★ fix が無ければ静止とみなす。屋内で充電している場面が本命なので、fix 待ちで黙らせない。
#define CHARGE_SW_MOVING_MPS     1.5f    // これを超えたら移動中（歩く速さ 1.3m/s 前後より少し上）
#define CHARGE_SW_STILL_MS      30000UL  // この時間止まっていたら移動が終わったとみなす（信号待ちで出たり消えたりしない）
// ---- 高温での充電の警告（power.cpp）----
// 電池 DTP603048(PHR) の仕様は「充電 0〜45°C」（放電は −20〜60°C）。
// 温度は MS5611（本体基板）で見る。電池はケースの中で本体基板の隣にあるので、
// 基板の温度を電池の温度の目安にする（基板は充電 IC の熱で少し高めに出る＝早めに警告する側）。
// MS5611 が居ない機体では判定しない。
#define CHARGE_HOT_C           45.0f    // これ以上が CHARGE_HOT_HOLD_MS 続いたら警告。設定画面下の Sensor 温度もこれ以上で赤
#define CHARGE_HOT_CLEAR_C     42.0f    // これ以下で解除（境目で出たり消えたりしない）
#define CHARGE_HOT_HOLD_MS     30000UL
#define CHARGE_HOT_ALERT_MS    60000UL  // 続いている間、読み上げをくり返す間隔
// ---- TFT バックライト（backlight.cpp）----
// Q1（P-ch）のゲートを PWM で駆動する。**Low で点灯**（ゲートは R47 で +3V3 へ
// プルアップしてあるので、PWM を設定するまでは消灯）。
// 周波数は可聴域より上にする。PAM8302 が同じ +3V3 から電源を取っているので、
// 可聴域だとスピーカーからうなりが出る。
//   輝度スイッチ High … 照度によらず TFT_BL_DUTY_BRIGHT
//   輝度スイッチ Low  … 照度センサー VEML7700 で自動（BL_AUTO_*）。センサーが無いときは TFT_BL_DUTY_DIM
  #define TFT_BL_PIN        34
  #define TFT_SW_PIN        35      // 輝度スイッチ。High = 最大（R48 4.7k で外付けプルダウン）
#define TFT_BL_PWM_HZ       25000
#define TFT_BL_DUTY_BRIGHT  1.00f   // 最大
#define TFT_BL_DUTY_DIM     0.05f   // スイッチ Low で照度センサーが無いとき。実機で調整する
// 照度による自動調光。照度の対数に比例させる（明るさの感じ方が対数なので）。
// VEML7700 はケースカバー基板にあり、窓越しに測るので、値は実機で合わせる。
#define BL_AUTO_LUX_LOW     1.0f    // これ以下は BL_AUTO_DUTY_MIN（夜）
#define BL_AUTO_LUX_HIGH    1000.0f // これ以上は最大（屋外）
#define BL_AUTO_DUTY_MIN    0.03f
#define BL_AUTO_TAU_MS      2000UL  // 照度への追従の時定数（手の影や機体の向きでちらつかせない）
// VEML7700 の 1 カウントあたりの照度（ゲイン ×1・積分 100ms）。データシートの版で
// 0.0576〜0.0672 と違うが、上の閾値を実機で合わせるので、どちらでも困らない。
#define VEML7700_LUX_PER_COUNT 0.0576f
// ---- 電池電圧のピーク追跡（display_tft.cpp の get_input_voltage）----
// 音声再生や TFT 書き込みで電圧は一瞬落ちる。あれは負荷の影響で電池そのものの
// 電圧ではないので、**上がるときは即追従（ピーク）・下がるときはゆっくり**にする。
// ★ 下降は **時間で**鈍らせること。0.988 まで「呼ばれるたびに α=0.2」だったので
//   **呼び出し元の数（12 箇所）で時定数が変わっていた**。負荷の谷まで追いかけて
//   しまい、実測で残量表示が 8%↔11% を往復していた。
#define BATTERY_PEAK_HOLD_MS  3000UL   // ピーク更新からこの間は下げない
#define BATTERY_DECAY_TAU_MS 20000UL   // 下降の時定数。長くすると表示が落ち着く
#define BAT_FULL_VOLTAGE 4.2 // 100% とみなす電圧。残量%と色分けの基準（display_tft.cpp の battery_*）
#define BAT_HALF_VOLTAGE 3.8 // 50%未満 (4.2-3.4=0.8V の半分は0.4Vなので4.2-0.4=3.8Vが50%の目安)
#define BAT_LOW_VOLTAGE 3.5
// 電池低下警告を繰り返す間隔 [ms]。自機（GPS_TFT_map.ino）と、
// 受信機が知らせる送信機の電池（link.cpp）の両方がこれを見る。
// 電池は減り続けるので 1 回きりの通知では足りない。逆に短くすると、
// 本当に聞くべき他の警報（コース・バンク角）が埋もれる。
#define BAT_WARN_INTERVAL_MS 60000UL
// ---- 電池低下のときの省電力（GPS_TFT_map.ino の batt_alarm_tick → backlight.cpp）----
// 電池低下の警告と同じ条件（BAT_LOW_VOLTAGE）で、バックライトを点滅させて電池をもたせる。
// ★ 暗くしたままにしないこと。本番では電池を最後まで使ってでも画面を見たいので、
//   「BL_SAVER_ON_MS 点けて BL_SAVER_OFF_MS 消す」をくり返す（点いている間はいつもの明るさ）。
// ★ 抜けるのは USB がつながったときか、電池が BAT_SAVER_EXIT_VOLTAGE まで戻ったとき（交換・充電）。
//   BAT_LOW_VOLTAGE で抜けると、消えている間に負荷が減って電圧が少し戻り、点滅の入り切りをくり返す。
#define BAT_SAVER_EXIT_VOLTAGE 3.65
#define BL_SAVER_ON_MS   2000UL
#define BL_SAVER_OFF_MS  4000UL
#define BAT_ZERO_VOLTAGE 3.4
// 満充電からの目安稼働時間 [分]。設定画面フッターの「Battery Time」の計算に使う。
// 実測は高輝度 約3時間 / 低輝度 約12時間と幅があるので、間を取った粗い目安。
// 輝度はソフトから読めないため、モードによらず 1 つの値で出している。
#define BAT_FULL_RUNTIME_MIN 240

// ============================================================
//  音（スピーカー・音量・バリオ音）
// ============================================================
#define PIN_PWMTONE 38
#define PIN_AMP_SD 39 //アンプシャットダウン(HIGHでON)
#define USERLED_PIN 36 //ユーザーLED（エラー表示用。エラー時 HIGH） v7 34->36に変更

// ユーザーLED の点滅パターン [ms]。実体は GPS_TFT_map.ino の loop_userled()。
// 1 周期 USERLED_BLINK_PERIOD_MS の中で USERLED_BLINK_PULSE_MS だけ点灯する。
// SD 異常のときだけ USERLED_BLINK_GAP_MS 後にもう 1 発打って「2 回連続点滅」にし、
// 衛星 0（1 回点滅）と目で区別できるようにしている。
// GAP + PULSE は PERIOD より十分短いこと。超えると次の周期の開始が優先され、
// **2 発目が一度も出ないまま 1 回点滅に見える**（＝衛星 0 と区別が付かなくなる）。
#define USERLED_BLINK_PERIOD_MS 1000
#define USERLED_BLINK_PULSE_MS    20
#define USERLED_BLINK_GAP_MS     200

// 音量の上限。設定画面のステップ送りと、SD の settings.txt から読むときの
// クランプの両方がこれを見る（片方だけ直して食い違うのを防ぐ）。
#define SOUND_VOLUME_MAX 100

// 音声の「排他グループ」。同じグループの音声は **ひとつの状態を言い替えたもの**で、
// 新しい方が来た時点で古い方はもう事実ではない。src/sound.cpp の pending
// （割り込まれた音声の救済キュー）は、同じグループの古い音声を**救済せず捨てる**。
// ★ 捨てないと機体が嘘を喋る。実例と設計は docs/pons_sound.md §3「排他グループ」。
// ★ 「鳴らさない」指定ではない。割り込まれた側を救済しないだけで、優先度の判定は同じ。
#define WAV_EXCL_NONE      0   // 排他なし（既定）。割り込まれたら従来どおり救済する
#define WAV_EXCL_LINKMODE  1   // 無線モードの読み上げ（sender_mode / receiver_mode / link_off）
#define WAV_EXCL_CHARGE_SW 2   // 電源スイッチの読み上げ（charge_sw_off / charge_sw_on）

#define SIN_VOLUME 0.15f  // Sin波の振幅倍率（0〜1.0f）。WAVと音量を合わせるため小さめにしてあるが、バリオが小さいと感じる場合は上げる。0.5fで±254、1.0fで±508（±512ヘッドルーム）。
#define VARIO_VOL_SCALE 3
// 上昇ビープ（高音）の音量を、下降音（低音）に対して何%にするかの補正。
// 小型スピーカーは低音の音響能率が低く、同じ振幅でも高音ばかり大きく聞こえるため高音側を絞る。
// 下降音側は VARIO_VOL_SCALE=3 で既にほぼ振幅上限（vario_volume 33%程度でクリップ開始）なので、
// バランス調整は原則こちらの値で行う。小さくするほど高音が控えめになる（目安 15〜40）。
#define VARIO_ASCEND_VOL_PCT 35

// ---- 下降音の音量傾斜（0.991）----
// 下降音は 580Hz（沈み始め）→ 430Hz（1.5m/s 以上）と、**深く沈むほど低音になる**。
// 小型スピーカーは低音の能率が落ちるので、同じ振幅でも
// **浅い沈み（-0.25 付近）がうるさく、深い沈み（-1.0 以下）が静かに聞こえる**。
// そこで**浅い側を絞り、深い側を 100% にする**傾斜をかける。
//   デッドバンド付近 = VARIO_DESCEND_VOL_MIN_PCT[%] → VARIO_DESCEND_FULL_MPS で 100%
//
// ★★ **絞る側は「クリップの下」まで落とさないと効かない。**
//   ISR の振幅は (±508) × VARIO_VOL_SCALE × vario_volume/100 × pct/100 で、
//   ヘッドルームは ±512。つまり **vario_volume=100 のときは pct が 34% を超えると
//   どちらもクリップして差が消える**（下降音が VARIO_VOL_SCALE=3 で振幅上限に
//   張り付いているため。上の VARIO_VOL_SCALE のコメント参照）。
//   60% のような浅い絞りでは「音量を上げている機体では何も変わらない」ことになる。
//   だから既定を 30% にしてある（上昇側の 35% と同じ桁）。
// ★ 深い側はクリップしたままでよい。あちらは**出せる限り大きく**が目的で、
//   クリップで矩形波に近づくと倍音が出て低音でも聞こえやすくなる。
// ★ 聴感の値なのでスピーカーを替えたら測り直すこと。目安 20〜40。
#define VARIO_DESCEND_VOL_MIN_PCT 30     // デッドバンド付近の音量 [%]
#define VARIO_DESCEND_FULL_MPS   1.0f    // この沈下率[m/s]以上で 100%

// バリオ音のデッドバンド [m/s]。この範囲では音を出さない。
// KF 融合中（IMU 生存 + MS5611 接続）は推定精度が高いので狭くできる。
// 0.30 → 0.25 に縮小（GNSS 垂直速度の取り込みで精度が上がったため, v0.942）。
// ※ sound.cpp の音の判定と display_tft.cpp の VSI のグレー線が同じ値を使う。
//   片方だけ変えると「線は出ているのに鳴らない」状態になるので必ずここで変える。
#define VARIO_DEADBAND_KF_MPS    0.25f
#define VARIO_DEADBAND_BARO_MPS  0.60f

// ============================================================
//  画面（TFT・更新間隔・画面モード）
// ============================================================
//---------TFT----------
// パネル種別（ST7789 / ILI9341）の選択は TFT_eSPI の User_Setup で行う。
// このファイルには選択用マクロは無い（コード側に分岐が無く、置いても効かないため）。
// 設定サンプルは CopySetupFile_TFT_eSPI.h を参照。

#define VERTICAL_FLIP

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 1050

// センサー値を眺めるための画面（VARIO 詳細）の強制リフレッシュ間隔 [ms]。
// 地図画面は GNSS 更新(2Hz)でも再描画されるので 1050ms で足りるが、
// VARIO 詳細には他の再描画トリガーが無く、この間隔がそのまま表示更新レートになる
// （従来は 1050ms = 約 0.95Hz しか出ていなかった）。
//
// 200ms = 5Hz。この画面ではベクタ地図を描かないため 1 回の再描画は地図画面(平均64ms)より
// ずっと軽く、Core0 に十分な余裕がある。GNSS シリアルも IRQ で 1024B までバッファされるため
// （38400bps で約 266ms 分）取りこぼさない。
// これ以上速くしたい場合は間隔を詰めるのではなく、VSI バーと同じように
// 値の部分だけ部分転送する方式にすること（全画面 115KB 転送が支配的になるため）。
#define SCREEN_FRESH_INTERVAL_DETAIL 200

// 設定画面を放置したとき、自動で地図画面へ戻るまでの時間 [ms]。
// 設定画面を出したまま離陸してしまった場合の保険。ボタン操作のたびに計時し直す。
// 3 分にしてあるのは、設定を眺めながら考え込む程度では抜けず、
// かつ離陸前の点検から離陸までの間には確実に戻る長さだから。
#define SETTING_IDLE_TIMEOUT_MS (3UL * 60UL * 1000UL)


// ---- MS5611（i2c0）の失敗検出と復旧 ----
// ★ これは「気圧センサーが 1 個死んだだけで音響警報が全停止する」のを防ぐためのもの。
//   airdata_update() は毎ループ呼ばれるので、無応答のまま放置すると
//   ms5611_write_cmd() が失敗のたびにログタスクを積み、0.5 秒ほどで
//   タスクキュー(40)が溢れる。溢れると**以降のタスクが全部捨てられる**ので、
//   バンク角警告もコース警報も電池警告も鳴らなくなる（mysd.cpp の enqueueTask 参照）。
// ---- 送信機の重複検出（受信機側）----
// seq の逆行から「同じ CH/SF/Group に送信機が 2 台いる」と判定したあとの扱い。
// ★ 以前は一度立つと電源を切るまで落ちない永久ラッチだった。同じ種類の状態である
//   「別グループを見た」が 10 秒で自然消滅するのに対して非対称で、
//   2 台目を止めて設定を直しても**再起動しないと確認できなかった**。
#define LINK_DUPSENDER_HOLD_MS  30000UL  // 最後の検出からこの時間は「検出中」とみなす
// 音声で知らせる間隔の下限。1 回きりにすると聞き逃したら二度と鳴らないが、
// 毎秒鳴らすのも実害（他の警報を押しのける）。重大なので 1 分ごとに念を押す。
#define LINK_DUPSENDER_ALERT_MS 60000UL

#define MS5611_FAIL_LIMIT            5      // 連続失敗がこの回数を超えたら ms5611_ok=false
#define MS5611_ERRLOG_INTERVAL_MS 5000UL    // I2C エラーログの最短間隔（毎ループ積まない）

// 復旧の再試行間隔 [ms]。
// ★ 値を決めるときに考えたこと:
//   ・1 回の試行は「アドレス応答の確認 1 トランザクション」から始める。
//     応答が無ければそこで諦めるので、不在時のコストは数十 µs で済む
//     （PROM 8 ワードまで読みに行くのは応答があったときだけ）。
//   ・IMU の復旧（IMU_RECOVERY_INTERVAL_MS）とは別の値。こちらは 1 回 1ms 未満なので短くてよい。
//   ・待っている相手は「一瞬のバス擾乱（即復帰）」「結露（数分）」
//     「ハンダ不良（永久に戻らない）」のどれか。30 秒なら一瞬の擾乱を
//     取りこぼさず、永久故障でも 2 分に 4 回の空振りで済む。
//   ・短くしすぎるとバスが固着したときに Wire のタイムアウトを頻繁に踏む。
#define MS5611_RECOVERY_INTERVAL_MS 30000UL

#define MAX_TRACK_CORDS 500
// 航跡の隣り合う点がこれ以上離れていたら、その間に線を引かない [m]（点の記録は続ける）。
// GNSS が不安定な所やトンネルの出入りで fix が遠くへ跳ぶと、画面を横切る線が出ていた。
// 普段の点の間隔は約 50m（速くても 100m 未満。gnss.cpp の add_latlon_track）なので十分大きい。
// トンネルや電波の途切れで本当に 300m 以上動いたときは、航跡がそこで切れて見えるだけ。
#define TRACK_BREAK_M 300.0f

// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// hAcc 不確かさ円 設定
// GNSS fix があっても hAcc（NAV-PVT 水平精度推定）が大きい場合、飛行機マークの代わりに青い不確かさ円を表示する。
// gnssFixOK=false の場合は常に不確かさ円を表示する。
#define HACC_THRESHOLD_M       10.0f // この値（m）以上で不確かさ円モードに切替え（旧 HDOP=2×5m 相当）
#define HDOP_MIN_CIRCLE_RADIUS 4     // 輪郭円を描画する最小半径 [px]（未満はテキスト表示に切替え）
#define HDOP_CENTER_DOT_RADIUS 3     // 中心位置を示す塗りつぶし小円の半径 [px]

//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GNSSDETAIL 3
#define MODE_MAPLIST 4
#define MODE_SDDETAIL 5
#define MODE_VARIODETAIL 6
#define MODE_REPLAYSELECT 7  // リプレイ再生ファイルの選択画面
#define MODE_IMUDETAIL 8     // IMU / 姿勢 ESKF の詳細画面
#define MODE_WIRELESS 9      // PONS Link（機体⇄ボート無線）の設定画面

// ============================================================
//  内蔵ベクタ地図
// ============================================================
// 地図データは 2 種類あり、**コンパイル時にどちらか一方だけ**を焼く。
// Arduino は src/ 配下の .cpp を全部コンパイルするので、両方のファイルは
// この #define で自分を丸ごと無効化する（重複定義を避けるため）。
//
//   無効（既定）: src/flashdata/vectormap_data.cpp        約 1.3MB
//                 FLASH 2MB の設定でビルドでき、書き込みが速い。無線デバッグ中はこちら。
//   有効        : src/flashdata/vectormap_data_hires.cpp  数 MB
//                 **ボード設定を「16MB (no FS)」にしないとリンクで溢れる。**
//                 書き込み時間が素直に伸びる。
//
// ★ 有効にする前に tools/vectormap/README.md の手順でデータを生成すること。
//   未生成のまま有効にすると #error で止まる（リンクエラーで悩まないように）。
//#define VECTORMAP_HIRES
  
// ============================================================
//  SD カードとリプレイ再生
// ============================================================
// ★ SD の使い分け
//     ルート直下 … **設定ファイルだけ**（settings.txt / mapdata.csv /
//                    destinations.csv / override_pilon_coordinate.csv）
//                    ※ 起動ロゴは FLASH（src/flashdata/logo_data.cpp）。SD の logo.bmp は読まない
//     data/      … 自機の飛行 CSV。過去大会のデータもここへ置く
//     received/  … 無線で受け取った機体のログ
//   リプレイ一覧は data/ か received/ しか見ないので、**除外リストが要らない**。
//   設定用の CSV をうっかり再生対象にしてしまう経路が構造的に無くなる。
//   （以前はルートを走査していたため、ルートに CSV を足すたびに除外リストへ
//     追記する必要があり、実際 override_pilon_coordinate.csv が漏れていた）
//
//   一覧の並びはファイル名順ではなく **ディレクトリの登録順**（＝SD へコピーした順）。
//   上に出したいものは先にコピーすること（browse_replay_files 参照）。
#define REPLAY_FLIGHT_DIR   "data"       // 自機の飛行 CSV と過去大会データ
#define REPLAY_RECEIVED_DIR "received"   // 無線で受け取った機体のログ

#define REPLAY_BUF_SIZE   16   // 先読みするCSV行数（Core1が供給 → Core0が消費するリングバッファ）
                               // ※ インデックス計算にビットマスクを使うので必ず 2 のべき乗にすること
                               // 高速再生(x20)では 2Hz データを毎秒40行消費するため余裕を持たせている
#define REPLAY_LIST_ROWS  20   // リプレイ選択画面の1ページあたり表示行数（20行×12px = 240px）
#define REPLAY_FIXED_COUNT 4   // 一覧の先頭に並ぶ固定項目数（常にこの数）
                               // Replay OFF / PLAY FLIGHT ONLY / PLAY SPEED / SOURCE
// 再生速度の選択肢。x1 → x2 → x_FAST の順に切り替わる（既定は x1）。
// 高速側の倍率を変えたいときは REPLAY_SPEED_FAST だけ書き換えればよい。
#define REPLAY_SPEED_FAST 20
#define REPLAY_MIN_GS 0.2f     // PLAY FLIGHT ONLY 有効時、この対地速度 [m/s] 以下を「静止」とみなす
#define REPLAY_LEADIN_MS 5000  // 動き出しの手前この時間 [ms] 分は静止していても再生する（助走表示）。
                               // 静止区間がこの長さ未満なら一切スキップしない。
#define REPLAY_LEADIN_SLOTS 40 // 助走区間の先頭を探すために保持する行数（5秒 ÷ 最短サンプル間隔ぶん）
#define REPLAY_SCAN_MAX 2000   // 静止区間の先読み走査で1回に読む最大行数（Core1 を長時間占有しないための上限）
#define REPLAY_REQ_INTERVAL_MS 200  // Core1へのバッファ補充依頼の最短間隔 [ms]
#define REPLAY_FILENAME_LEN 48      // リプレイ対象ファイル名（パス込み）の最大長
                                    // received/ 前置(9) + ファイル名(最大31) + NUL に足りる長さ

// ============================================================
//  デバッグ出力・計測マクロ
// ============================================================
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
// SD アクセスがどのコアから来たかの見張り（デバッグビルドのみ）
//
// SD へのアクセスは Core1 に閉じる約束だが、**それを強制する仕組みは無い**。
// mutex が守っているのはタスクキューだけで、SDIO バスは規律だけで守られている。
// Core0 から触ると「たいてい動くが、たまにバスハングでフリーズする」という
// 一番たちの悪い壊れ方をするため、開発中に気づけるようにしておく。
//
// 実際に SD を読み書きする関数の先頭に置く。Core0 から good_sd() を呼ぶのは
// 意図的に安全（フラグを読むだけ）なので、そちらには置かないこと。
// RELEASE では何も残らない。
// ※ 呼び出し箇所ごとに 1 秒に 1 回までに絞る（2Hz の CSV 保存などで溢れさせない）。
// ============================================================
#ifndef RELEASE
  #define ASSERT_SD_CORE1(label) do { \
    if (get_core_num() != 1) { \
      static unsigned long _sdc_; \
      if (_sdc_ == 0 || millis() - _sdc_ >= 1000) { \
        _sdc_ = millis(); \
        Serial.print("!!! SD access from Core0: " label); \
        Serial.println(" (see CLAUDE.md: Core0 から SD を直接触らない)"); \
      } \
    } \
  } while(0)
#else
  #define ASSERT_SD_CORE1(label) do {} while(0)
#endif

// ============================================================
// SD 以外の「Core1 専用」の見張り（デバッグビルドのみ）
//
// SD を触らないが Core1 に閉じていなければならない処理のため。
// 0.982 で音声を FLASH へ移して startPlayWav() が SD を触らなくなったが、
// **Core1 専用であることは変わらない**（再生バッファとタイマー割り込みの状態を
// 動かすので、Core0 から呼ぶと loop_sound() と衝突して音が壊れる）。
// ASSERT_SD_CORE1 を流用すると「SD access from Core0」と嘘のメッセージが出て、
// 実機ログを読む人を SD の方へ誤誘導するので分けてある。
// ============================================================
#ifndef RELEASE
  #define ASSERT_CORE1(label) do { \
    if (get_core_num() != 1) { \
      static unsigned long _c1_; \
      if (_c1_ == 0 || millis() - _c1_ >= 1000) { \
        _c1_ = millis(); \
        Serial.print("!!! Core1-only function called from Core0: " label); \
        Serial.println(" (see CLAUDE.md: SD と音声は Core1 専用)"); \
      } \
    } \
  } while(0)
#else
  #define ASSERT_CORE1(label) do {} while(0)
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

  // ---- 処理時間計測ユーティリティ（IMU の読み出しを飢えさせない調査用） ----
  // 使い方: TIMING_START(名前) → 重い処理 → TIMING_END(統計変数, 名前)
  // 30秒ごとに TIMING_REPORT で max/avg/count を Serial 出力する。
  // 閾値: 10ms = 10000 us（IMU の読み出し間隔の目安）
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
//  IMU（SCH16T-K01）
// ============================================================
// ジャイロと加速度しか出さないチップ（融合は無い）。機上の推定（姿勢 ESKF・バリオ KF）は
// この 2 つだけで成り立つ。「何が有るか」の申告は [imu_sensor.h](imu_sensor.h) の imu_caps()。
// ★ **`EOI` を書いた時点でレジスタが全部ロックされる**（データシート Table 80。
//   リセットするまで解除できない）。フィルターやレンジを設定画面から実行中に変えることは
//   原理的にできない。詳細は [docs/imu_sch16t_plan.md](docs/imu_sch16t_plan.md) §5。

// 能力ビットのマスク。imu_caps() の戻り値に AND される（既定は全部通す）。
#define IMU_CAPS_DEBUG_MASK 0xFFFFFFFFu
#if IMU_CAPS_DEBUG_MASK != 0xFFFFFFFFu
  #warning "IMU_CAPS_DEBUG_MASK is not default (capability degradation rehearsal). restore before release."
#endif

// ---- SCH16T-K01-10 の定数（データシート Doc.No.11624 Rev.6 で全件照合済み）----
// ★ **値は tools/sch16t_test/ で実機確認済み**（2026-10-14）。ドライバは
//   imu_sch16t.cpp（0.991 で実装）がここを読む。実測値は
//   docs/imu_sch16t_bringup.md §1。
//
// SPI: mode 0（CPOL=0/CPHA=0）、48bit アウトオブフレーム。
//   ★ **SCK には下限もある（0.095MHz）。** 遅すぎてもフレームが成立しない。
//   ★ **応答は 1 フレーム遅れて返る。** 先頭に捨てフレームが必要。
#define SCH16T_SPI_HZ        5000000UL  // 0.095〜10.5MHz の範囲の中央あたり
#define SCH16T_CRC8_POLY     0x2F       // 初期値 0xFF・XOR out なし・bit47〜8 が対象
#define SCH16T_CRC8_INIT     0xFF
// 感度（20bit 既定設定）。★ レンジ設定 '000' は**書いてはいけない**（禁止値）。
#define SCH16T_LSB_PER_DPS   1600.0f    // ジャイロ [LSB/(deg/s)]
#define SCH16T_LSB_PER_MPS2  3200.0f    // 加速度 ACC1/ACC2 [LSB/(m/s²)]
#define SCH16T_LSB_PER_MPS2_ACC3 1600.0f // 広レンジ補助加速度 ACC3
#define SCH16T_LSB_PER_DEGC  100.0f     // 温度 [LSB/°C]
// 初期化の待ち [ms]。★ **どれも縮めてはいけない。** データシート §5.1.2 の手順
//   （リセット → NVM 読み出し → 設定 → EN_SENSOR → 安定待ち → ステータス 1 周
//    → EOI → ステータス 2 周 → 検証）に対応している。
#define SCH16T_WAIT_NVM_MS   32
#define SCH16T_WAIT_START_MS 215
#define SCH16T_WAIT_EOI_MS   3
// 品種判定は COMP_ID。★ **ASIC_ID はシリコンのリビジョンなので品種判定に使えない。**
#define SCH16T_COMP_ID       0x0023     // SCH16T-K01
//
// ---- フィルターと読み出しレート（2026-10-14 の単独テストで確定）----
// ★★ **読み出しレートはフィルターと紐づく。** LPF ごとに「最小推奨読み出しレート」が
//   あり、下回ると阻止域のノイズが折り返して帯域内に入る（§5.5 Table 16）:
//       LPF2 13Hz → 150Hz / LPF1 30Hz → 200Hz / LPF0 68Hz（既定）→ 500Hz
//   **BNO085 と同じ感覚で 50Hz で読むと折り返す。**
// ★ **LPF2（13Hz）を選んだ理由。** 68/30/13Hz を各 60 秒実測した結果:
//   - 13Hz↔30Hz は白色予測どおりに動き、ノイズ密度も一致した
//     （＝ここがセンサーの床。実測 0.0022 °/s/√Hz）。68Hz の超過分は環境振動
//   - **13Hz の方が sd が 31% 小さい**（0.0080 対 0.0116 °/s）
//   - **最小推奨レートが 150Hz で済む。** これが一番効く — 地図描画は
//     タイル 1 枚で 40〜100ms Core0 を占有するので（gapmax 実測 40ms 前後）、
//     実効レートは 200Hz 前後まで落ちる。LPF1 の 200Hz では余裕が無い
// ★ **群遅延の差は問題にならない。** 単極近似で 13Hz が 12.2ms、30Hz が 5.3ms、
//   3 次なら 37ms 対 16ms。差は 7〜21ms。一方 **GNSS（NAV-PVT）は測位エポックから
//   50〜100ms 遅れて届く**ので、ESKF の補正で効く相対遅延はそちらが支配する。
//   しかも**ジャイロを遅らせる方が GNSS との相対遅延は縮まる**ので不利でもない。
//   パイロットへの通知も、画面更新が数 Hz・バンク警報は持続判定・V/S は MS5611
//   融合でもっと大きなラグがあるので 21ms は現れない。
//   （この群遅延の値はデータシートではなく単極近似。表があれば確認する価値がある）
#define SCH16T_FILT_SEL      0b010     // LPF2 = 13Hz（CTRL_FILT_* の 3bit × 3 軸）
#define SCH16T_MIN_READ_HZ   150       // LPF2 の最小推奨読み出しレート
// デシメーション。DEC5 = F_PRIM/32 が最大で、これが ODR の下限。
// ★ 実測 **721.5Hz**（公称 737.5Hz に対し −2.1%。2 回で 721.0/722.0 と再現）。
//   全数を読む必要はなく、上の最小推奨レート以上で読めばよい。
#define SCH16T_DEC_SEL       0b100     // DEC5 → 公称 737.5Hz
// ダイナミックレンジ。★ **'000' は CTRL_RATE / CTRL_ACC12 では禁止値。**
// ★ RATE に '010' を書くのは、Murata 由来ライブラリが 1600 LSB に '010' を書き、
//   逆変換では '001' を未定義にしているため（'001' と同じレンジだが '010' が正式）。
#define SCH16T_RATE_DYN_SEL  0b010     // ±327.68 °/s  → 1600 LSB/(°/s)
#define SCH16T_ACC12_DYN_SEL 0b001     // ±163.84 m/s² → 3200 LSB/(m/s²)
#define SCH16T_ACC3_DYN_SEL  0b000     // ±260 m/s²（ACC3 では '000' が既定で合法）
// リセット後「まともな値を返す」までの待ちの上限 [ms]。
// ★★ **固定 delay だけに頼らないこと。** リセット直後の SCH16T は
//   「フレーム構造は正しく CRC も合っているが、データ field が全部 0」の応答を返す。
//   SCH16T_WAIT_NVM_MS だけで進むと**設定書き込みが黙って捨てられる**
//   （実機 2026-10-14 で踏んだ。CRC では検出できない）。ドライバは ASIC_ID が
//   0x0000/0xFFFF 以外になるまで待つ。実測はコールド 1ms / 再初期化 0ms。
#define SCH16T_READY_TIMEOUT_MS  1000
// 生ログ（src/imulog.h）へ落とすときの間引き。**1 = 全サンプル。**
// ★★ **読み出しレートをそのまま記録すると SD の予算を超える。**
//   250Hz でジャイロと加速度の 2 本を押すと 500 レコード/秒 = 14kB/s で、
//   BNO085 時代の実測 158 レコード/秒（4.4kB/s）の 3 倍。imulog の二重バッファは
//   146 レコードなので 0.29 秒で一杯になり、Core1 が WAV や CSV で詰まると
//   dropped が増える。4 で間引くと 125 レコード/秒 ≒ 従来の運転点に収まる。
// ★ 代償: **機上の ESKF は 250Hz で回るのに、ログは 62.5Hz** になるので、
//   オフライン再生が機上の結果を厳密には再現しない（BNO085 時代は
//   レポートレート = feed レートだったので一致していた）。生ログで ESKF を
//   作り直すときは 1 に戻して SD の実測を取り直すこと（計画書 §2.4）。
#define SCH16T_IMULOG_DECIM  4
// 温度を読む間隔（サンプル数）。1 サンプルあたりのフレーム数が
// **8 → 7 に減る（12.5%）** ので、描画で Core0 が止まる環境では効く。
// 温度は推定に使っておらず（バイアスの温度係数 約 −0.005 °/s/℃ は ESKF が
// オンラインで吸収する）、ドリフトの監視用に数 Hz あれば足りる。250Hz / 64 ≒ 3.9Hz。
#define SCH16T_TEMP_EVERY    64
//
// ---- ピン（**v7.1 基板で確定。2026-10-07 に PCB のネットリストから確認**）----
// SCH16T は基板実装ではなく **J6（Molex 200528-0080, 8pin FPC）経由の外付け**。
// ★ **本体 J6 とブレイクアウトは番号が逆順。** ブレイクアウトはフレキ基板で、尾の金端子 J3 を
//   J6 へ直接挿すと J6-n ↔ J3-(9-n) が接する（2026-10-19 に本体側を逆順へ直した）。1:1 ではない。
//   ブレイクアウト J3（FPC）/ J1（ピンヘッダ）: 1=+3V3 2=RESET 3=MOSI 4=CS 5=SCK 6=SYNC 7=MISO 8=GND
//   本体 J6                                 : 1=GND 2=MISO 3=SYNC 4=SCK 5=CS 6=MOSI 7=RESET 8=+3V3
//
// ★ 41/42/43/44 は RP2350 の **SPI1 の正規割り当て**（CSn/SCK/TX/RX）なので
//   ハードウェア SPI が使える。**arduino-pico では `SPI1` オブジェクト**（`SPI` は spi0）。
//   GPIO46 は SPI1_SCK の代替機能でもあるので `SPI1.setSCK(42)` を明示すること。
// SCH16T_PIN_UNSET は「ピンが割り当てられていない」を表す値（ドライバが実行時に確かめる）。
#define SCH16T_PIN_UNSET     255
  #define SCH16T_PIN_CS         41
  #define SCH16T_PIN_SCK        42
  #define SCH16T_PIN_MOSI       43
  #define SCH16T_PIN_MISO       44
  #define SCH16T_PIN_EXTRESN    45   // 外部リセット（負論理）。ネット名 SCH16T_RESET
  #define SCH16T_PIN_DRY        46   // データ準備完了（デシメーション出力用）。SCH16T_SYNC

  // ★ 0.991 で imu_sch16t.cpp に中身が入った（骨組みは 0.987）。ただし
  //   **本体基板では一度も動かしていない。** 単独テスト（Pico 2）で裏付いたのは
  //   チップとフレーム層までで、SPI1 のピン・ESKF への接続・バリオ・画面は未確認。
  //   **docs/imu_sch16t_bringup.md の Step 2〜5 を通してからこの #warning を消すこと。**
  #warning "SCH16T driver is implemented but NEVER RUN ON THE BOARD. do docs/imu_sch16t_bringup.md Step 2-5 first. DO NOT FLY."

// ============================================================
//  IMU の生ログと読み出し
// ============================================================
// ---- 生 IMU ロガー（姿勢 ESKF のオフライン開発用）----
// 目的: 生のジャイロ・加速度と GNSS 速度を SD にバイナリ記録し、姿勢 ESKF を PC 上で
//       作り直せるようにする（始まりは BNO085 内蔵フュージョンの誤りを調べるためだった）。
//       機上では推定を行わない（詳細は imulog.h）。
//
// バス負荷: SCH16T は SPI1 を 1 台で使うので、記録してもバスの取り合いは無い。
// SD 負荷 : 上の SCH16T_IMULOG_DECIM で間引いて約 130 レコード/秒（ジャイロ・加速度 各 62.5
//           ＋ GNSS 2 ＋ 気圧 4）× 28B ≒ 3.7kB/s（30 分飛行で約 6.6MB）。
#define IMULOG_DEFAULT_ENABLED  true  // ビルド時固定（実行時・SD からの切替手段は無い）

// ---- 生のジャイロ・加速度を使うか ----
// ★ **v7.1 では 1 しかない。** SCH16T が出すのは生のジャイロと加速度だけなので、
//   0（v7 の BNO085 で「融合出力の GRV/LACC/RV だけを購読する」設定）にすると
//   姿勢 ESKF に入力が 1 つも届かない。下の #error で止める。
//   0 の意味と、BNO085 の配信能力の内側にレートを保つ話は PONSv7 ブランチの同じ場所。
// （#if で判定するため true/false ではなく 1/0 で書く）
#define IMULOG_RAW_REPORTS_ENABLED  1
#if !IMULOG_RAW_REPORTS_ENABLED
  #error "v7.1 (SCH16T) has raw gyro/accel only. IMULOG_RAW_REPORTS_ENABLED must be 1."
#endif

// ジャイロのサンプル周期 [Hz]。attitude.cpp がサンプル間隔の初期値と、測った間隔の
// 妥当性の範囲（0.2〜5 倍）に使う。BNO085 時代の 50Hz のまま。
// ★ **SCH16T は 250Hz（4ms）で届くので、4ms がこの範囲の下限ちょうどになる。**
//   ESKF の「1 サンプルあたり」のゲイン（ESKF_RELEVEL_GAIN など）も 50Hz を前提にしている。
//   v7.1 の実機で、測ったサンプル間隔と時定数を確かめること（docs/imu_sch16t_bringup.md）。
#define IMU_RATE_GYRO_HZ    50

// 読み出し周期 [µs]。4ms（250Hz）。
// ★★ **生ログの有無に関係なくこの値。** 33ms（33Hz）では LPF2(13Hz) の最小推奨 150Hz すら
//   満たせず、阻止域のノイズが折り返して帯域内に入る（SCH16T_MIN_READ_HZ）。
//   SCH16T はこちらが取りに行く方式で、**読み出しレートがそのまま信号品質を決める**。
  #define IMU_POLL_INTERVAL_US   4000

// 加速度サンプルの許容鮮度 [µs]（安全網）。名前の LACC は BNO085 時代の名残で、
// v7.1 では生の加速度（250Hz = 4ms 周期）の鮮度を見ている。150ms は正常時には決して発火しない。
// 発火するのは読み出しが止まったとき（描画などで Core0 が長く止まる・SCH16T の途絶）だけで、
// そのとき predict を止める。predict を止めても imu_kalman_baro_update() が ~40Hz で
// 出力を更新し続けるため、バリオは IMU なし時と同じ気圧ベース動作に劣化するだけで済む
// （古い加速度を繰り返し積分して暴れるより遥かに安全）。
#define IMU_LACC_MAX_AGE_US  150000

// IMU が途絶したときの復旧試行の間隔 [ms]（imu_sch16t.cpp）。
// 短くしても直る見込みは増えないので、1 分で十分。
#define IMU_RECOVERY_INTERVAL_MS  60000UL

// ============================================================
//  バリオメーター（高度・昇降速度の推定）
// ============================================================
// ---- KF に入れる鉛直加速度の作り方 ----
// 1 : 重力込みの生比力を ESKF の姿勢で地球座標系へ回し、重力加速度 GRAVITY_MPS2 を引く。
//     （0.983 までは BNO085 の GAME_ROTATION_VECTOR で回していた）
// ★ **v7.1 では 1 しかない。** 0 は BNO085 が自分で重力を引いた LINEAR_ACCELERATION を
//   使う経路で、SCH16T には LACC が無いので、0 にすると加速度が一度も届かず
//   バリオが黙って気圧だけになる。下の #error で止める（imu.cpp の #else 側は PONSv7 と
//   形をそろえて残してある）。
//
// ■ 1 にした理由（2026-08-19、v7 の BNO085 で tools/imulog/20260819.bin の屋内上げ下げで確認）
//   BNO085 の LINEAR_ACCELERATION は、GAME_ROTATION_VECTOR とは別の「適応的な重力推定」で
//   重力を引いている。数百 ms 続く鉛直加速度をこの重力推定が徐々に吸収してしまうため、
//   加速中は加速度を過小に、動作が終わった後は逆符号の残差を出す。
//   実測（静止 → 上げる → 止める、を繰り返した区間の ∫a_k dt。両端が静止なので期待値は 0）:
//       区間             LINEAR_ACCELERATION   生比力 - g
//        持ち上げ 4秒        -0.98 m/s            -0.10 m/s
//        下ろす   3秒        +0.76 m/s            +0.26 m/s
//        持ち上げ 5秒        -1.02 m/s            -0.34 m/s
//   ＝ 動かした向きと逆に約 1 m/s の速度誤差が入る。これがそのまま
//      「上げ下げを止めた瞬間に反対側の音が鳴る」症状になっていた。
//   77 秒通しての ∫a_k dt は -0.11 m/s なので、定常的なバイアスではなく
//   「動作に相関した誤差」である点が重要（KF のバイアス状態では取り除けない）。
//
//   KF に通した結果（屋内で上げ下げした 3 セッション）:
//       推定上昇率の符号が真値と逆になる時間の割合  18〜26% → 0.8〜6.5%
//       静止しているのに |v|>0.3 で鳴ってしまう割合  8〜16% → 0〜1.6%
//   屋外（車載・ゆっくりした高度変化）では両者ほぼ同等で、悪化はしない。
#define VARIO_USE_RAW_ACCEL  1
#if !VARIO_USE_RAW_ACCEL
  #error "v7.1 (SCH16T) has no LINEAR_ACCELERATION. VARIO_USE_RAW_ACCEL must be 1."
#endif

// 重力加速度 [m/s²]。生比力から鉛直成分を取り出すときに引く値。
// 標準重力。日本の実測値との差 (~0.01 m/s²) は KF のバイアス状態 x[2] が吸収する。
//
// ★ **静止時の VertAccel（Vario 詳細画面）は加速度計の校正ずれがそのまま出る。**
//   世界座標の鉛直成分からこの固定値を引いているので、スケールやバイアスがずれていれば 0 にならない
//   （v7 の BNO085 では -1.58 m/s² が出て校正ずれに気づけた。経緯は PONSv7 ブランチの
//   settings.h の同じ場所と docs/imu_i2c_bringup.md の「加速度計の校正」）。
//   SCH16T は工場校正なので、ここが 0 から大きく外れていたら軸の向き（IMU_MOUNT_Q*）か
//   読み出しを疑う。
#define GRAVITY_MPS2  9.80665f

// バリオ KF の predict 周期 [µs]（生レポート有効時のみ使う）。
// kf_predict() は Q を dt でスケールせず「1 ステップあたり」で加算する
// （imu.cpp の kf_P[1][1] += q_vel_eff）。したがって predict 周期を変えると
// 単位時間あたりのプロセスノイズ注入量が変わり、チューニング済みのバリオが壊れる。
// ポーリングを 5ms に上げてもここで従来の 30ms を維持する。
//
// ※ ポーリングが 30ms のときはこのゲートを使ってはいけない。
//   ポーリング周期とゲート周期が同じだと、わずかなジッタでゲートを 1 回外し、
//   predict が 60ms 間隔になって dt が倍になる回が混ざるため。
#define IMU_KF_PREDICT_INTERVAL_US  30000

// ---- Kalman フィルターのチューニング ----
// VSI = x[1] は 2 経路で動く。
//   predict（約 33Hz・IMU 加速度）: x[1] += (a_k - x[2])*dt,  P[1][1] += KF_Q_VEL
//   update （約 3.7Hz・気圧高度）: K[1] = P[1][0]/(P[0][0]+KF_R),  x[1] += K[1]*残差
//
// ★ update は 40Hz ではなく **約 3.7Hz**。MS5611 は ~40Hz でサンプルするが、
//   airdata_update() が true を返すのは VSPEED_WINDOW_MS(250ms) のトリム平均が
//   完了したときだけ。チューニングはこの比（33 : 3.7）を前提にすること。
//
// 静止時に VSI が揺れる＝ K[1] が大きすぎる。KF_R を大きくするか KF_Q_VEL を
// 小さくすれば減るが、どちらも上昇・下降への追従が 0.5〜2 秒ぶん鈍る。
//
// KF_Q_BIAS を 1/10 にしてある理由: x[2] は「加速度計のゆっくりしたオフセット」を
//   吸収する状態なのに、0.005（毎秒 0.165 (m/s²)² 相当）だと数秒スケールで動けてしまい、
//   機動そのものを吸収していた（実ログで機動に同期して ±0.2 m/s² 振れることを確認）。
//   ※ 「止めた瞬間に反対側へ振れる」主因はこちらではなく加速度ソースのほうだった
//     （VARIO_USE_RAW_ACCEL 参照）。そちらを直せば 0.005 に戻してもほぼ同等
//     （逆符号 6.4% vs 6.5%）。物理的に妥当な方を採る、という意味での 1/10。

#define KF_Q_VEL    0.02f   // 速度プロセスノイズ  (旧 0.05 → 1/2.5。VSIふらつきをさらに抑制)
#define KF_Q_BIAS   0.0005f // バイアスプロセスノイズ (旧 0.005 → 1/10。上記コメント参照。効果は小さい)
#define KF_R       12.0f    // 気圧高度観測ノイズ [m²] (旧 4.0 → 3倍。気圧ノイズのVSI影響を低減)

// ============================================================
// 水平加速度による速度プロセスノイズ動的増幅（imu.cpp kf_predict で使用）
// ============================================================
// 水平加速度が大きいと BNO085 の重力ベクトル推定がずれ、
// 地球座標系の鉛直加速度 a_k に誤差が混入する。
// predict ステップで速度プロセスノイズを下式で増幅することで、
// IMU 加速度への依存を自動的に弱め、気圧・GNSS 観測の重みを上げる。
//
//   q_vel_eff = KF_Q_VEL + KF_HORIZ_ACCEL_GAIN × horiz_accel²
//
// KF_HORIZ_ACCEL_GAIN : 増幅ゲイン [s²/m²]
//   0    → 従来どおり（水平加速度による補正なし）
//   大きい → 少しの水平加速度でも IMU の影響を早く落とす
//
// 参考: gain=0.01・KF_Q_VEL=0.02（既定値）のときの q_vel_eff
//   horiz 0 m/s² → 0.020 (×1.0)
//   horiz 2 m/s² → 0.060 (×3.0)
//   horiz 4 m/s² → 0.180 (×9.0)
//   horiz 8 m/s² → 0.660 (×33.0)
// ※ KF_Q_VEL は SD 設定 kf_q_vel で実行時に変わるので、実際の倍率もそれに追従する。
#define KF_HORIZ_ACCEL_GAIN  0.01f

// ============================================================
// GNSS高度補正パラメーター（imu_kalman_gnss_update で使用）
// ============================================================
// GNSS高度は気圧高度より誤差が大きいが、長期ドリフトのない絶対基準として使える。
// 気圧基準（ground_alt_abs）をゆっくり修正することで、高度をGNSS基準に近づける。
// Vertical speedへの影響は補正レート（最大 GNSS_MAX_DELTA_M m/s）に留まり無視できる。
//
// GNSS_VACC_MAX_M        : vAcc がこれ以上の時は補正しない（カットオフ）[m]
// GNSS_INIT_SAMPLES      : 起動地MSL高度を平均する初期化サンプル数
// GNSS_CORRECT_RATE      : 気圧基準補正ゲイン（イノベーション×quality に掛ける比率）
//                          vAcc が小さい（高精度）ほど quality が高く補正が速くなる。
// GNSS_MAX_DELTA_M       : 1更新あたりの最大補正量 [m]（バリオへの影響上限）
// GNSS_OFFSET_UPDATE_RATE: gnss_kf_offset（起動地MSL高度）の長期更新ゲイン。
//                          MSL絶対高度がGNSSに収束する速さを決める。quality × rate が実効値。
//                          rate=0.005, quality=0.8 → α=0.004/s → 半減期約170秒（約3分）
#define GNSS_VACC_MAX_M        10.0f  // 垂直精度カットオフ [m]（vAcc < 10m の時のみ補正）
#define GNSS_INIT_SAMPLES      10     // 初期化サンプル数
#define GNSS_CORRECT_RATE      0.02f  // 気圧基準補正ゲイン（旧 0.005 → 4倍に増速）
#define GNSS_MAX_DELTA_M       0.05f  // 1回あたりの最大補正量 [m]（旧 0.02 → 2.5倍）
#define GNSS_OFFSET_UPDATE_RATE 0.005f // MSL絶対基準の長期収束ゲイン
//
// ============================================================
// GNSS VSI Kalman 速度観測パラメーター（imu_kalman_gnss_vel_update で使用）
// ============================================================
// GNSS velD（上昇正）を KF の速度観測（H=[0,1,0]）として取り込む。
// 観測ノイズ R_vel は sAcc²（速度精度の二乗）× R_SCALE を使用。
// ゲートは sAcc のみで判断する（vAcc=垂直位置精度 は速度品質の指標として不適切なため使用しない）。
//
// GNSS_VSI_SACC_MAX_MPS: sAcc がこの値以上なら速度観測更新をスキップ [m/s]
//                        sAcc = NAV-PVT Speed Accuracy Estimate（速度精度 1-sigma）。
//                        この値未満では R_vel = sAcc² × R_SCALE の連続曲線が機能する。
// GNSS_VSI_R_SCALE     : 観測ノイズ R_vel の倍率（1.0 = sAcc² そのまま）
//                        大きくするほど GNSS VSI の影響が弱まり、気圧・IMU 主体になる。
//                        K[1] ≈ P/(P+R) なので、倍率を上げるとゲインが大幅に減少する。
//
// ★★ **16 → 64 に上げた（0.992）。sAcc は建物の反射を申告しない。**
//   建物のそばでは反射（マルチパス）で velD が実際より荒れるのに、sAcc は小さいまま
//   なので**ハードゲートも R も効かない**（V/S が暴れる主犯）。重みそのものを 1/4 に
//   落として気圧・IMU 主体へ寄せてある。
//   ★ **本当は GNSS を VSI から外したいが一旦保留。** 外すなら倍率を更に上げるのでは
//     なく、GPS_TFT_map.ino の imu_kalman_gnss_vel_update() の呼び出しを止める方で
//     判断する（倍率で殺すと「効いているつもりの観測」が残って読み手を惑わす）。
//   ★ 下の数値は倍率に連動する。**定数を変えたら作り直すこと**
//     （0.991 まで 16 倍時代の値が残っていた）。
//
// 参考: sAcc² × R_SCALE のカルマンゲイン K[1] ≈ P/(P+R) への影響（P≈0.2・R_SCALE=64）
//   sAcc=0.1 → R=0.64  K≈0.24（緩やかに補正）
//   sAcc=0.2 → R=2.56  K≈0.07（ほとんど効かない）
//   sAcc=0.5 → R=16.0  K≈0.01（実質切れている。R が自動で切ってくれる）
//
// ★ ハードゲートは「桁違いのゴミ」だけを切る値にすること。**0.3 は低すぎた。**
//   実機 2026-09-30 の sAcc は静止・低速で 0.15〜0.38 に散らばり、0.3 を
//   **91 分間に 393 回（毎分 4 回）またいでいた**。またぐたびに GNSS 速度観測が
//   入る／入らないを切り替わるので、V/S の供給元が突然差し替わる。
//   R_vel = sAcc² × R_SCALE の曲線が既に重みを落としているので、
//   この付近をハードゲートで切る意味は無い（sAcc=0.3 で K≈0.03。R_SCALE=64）。
//   1.0 に上げると通る割合は 89.5% → 95.0%、またぐ回数は 393 → 39 回に減り、
//   **本物の受信不良（トンネル: sAcc 2〜27m/s）だけを切る**ようになる。
#define GNSS_VSI_SACC_MAX_MPS  1.0f  // 速度精度ハードゲート [m/s]（以上はスキップ。旧 0.3 → 崖を外した）
#define GNSS_VSI_R_SCALE      64.0f  // R_vel 倍率（旧 16 → 64。反射で荒れた velD の重みを落とす）

// MAX G/S（画面下部の最大 GS）に入れてよい速度精度 sAcc の上限 [m/s]。
// 上の GNSS_VSI_SACC_MAX_MPS と同じく「桁違いのゴミ」（トンネル: sAcc 2〜27m/s）だけを切る値。
// トンネルの出入りで 237m/s が MAX G/S に残っていた（gnss.cpp の feed_maxgs）。
#define MAXGS_SACC_MAX_MPS     1.0f

// ============================================================
// 高度表示設定
// ============================================================
// 当デバイスの GNSS は UBX NAV-PVT の hMSL（EGM96 ジオイド基準）を使用する。
// hMSL は日本の標高（T.P.=東京湾平均海面 基準）とほぼ一致する（差は ±数十cm 程度）。
//
// ※注意: GNSS の「楕円体高（WGS84）」とは異なる。
//   楕円体高 = hMSL + ジオイド高 N（日本では N ≈ +36〜38m）
//   例: 関西で 0m 標高 → hMSL ≈ 0m、楕円体高 ≈ +37m
//   当 GNSS は既に hMSL を出力済みなので、-37m 補正は不要。
//
// ELEVATION_GEOID_OFFSET_M: B.S.L. 高度 = KF_MSL - この値 [m]
//   表示ラベルは "B.S.L."（琵琶湖基準水位面、Biwa Standard Level）。
//   B.S.L. 0m = T.P.（日本の標高） +84.371m（瀬田川洗堰基準・国土交通省設定値）。
//   → 琵琶湖面で B.S.L. ≈ 0m、湖面上 100m 飛行時に B.S.L. ≈ 100m と表示される。
//   標高（T.P.）に戻したい場合は 0.0f に変更する。
#define ELEVATION_GEOID_OFFSET_M  84.371f  // B.S.L.基準補正 [m]（T.P.84.371m = 琵琶湖面）

// ============================================================
//  姿勢 ESKF（GNSS 速度援用）
// ============================================================
// ============================================================
// 実装は attitude.cpp。PC 側の tools/imulog/eskf.py と対になっているので、
// 値を変えたら両方に反映すること（片方だけ変えると比較が成立しなくなる）。

// プロセスノイズ（BNO085 の実力と HPA の運動を想定した初期値）
#define ESKF_SIGMA_G    2e-3f    // ジャイロ白色雑音 [rad/s/sqrt(Hz)]
#define ESKF_SIGMA_A    3e-2f    // 加速度計白色雑音 [m/s²/sqrt(Hz)]
#define ESKF_SIGMA_BG   1e-5f    // ジャイロバイアスのランダムウォーク
#define ESKF_SIGMA_BA   1e-4f    // 加速度バイアスのランダムウォーク

// バイアス推定の物理的な上限。
// BNO085 は校正済みの値を出すので残留バイアスは本来ごく小さい。
// 上限が無いと初期姿勢誤差の行き場としてバイアス状態が使われ、誤った値に固着する。
// 実測（2026-08-18 車載 session 2）では bg が 5.9deg/s に張り付き、
// ESKF が BNO085 と同じ偽ピッチを出すようになった。上限を入れて解消している。
#define ESKF_MAX_GYRO_BIAS   0.01745f  // 1.0 deg/s [rad/s]
#define ESKF_MAX_ACCEL_BIAS  1.0f      // [m/s²]

// GNSS 速度観測の信頼度（u-blox の sAcc をそのまま使う）
#define ESKF_SACC_MIN   0.05f    // これ未満は過信になるので下限を置く [m/s]
#define ESKF_SACC_MAX   1.0f     // これを超える観測は捨てる（実測で 248m/s の暴れ値あり）

// 静止判定（初期姿勢を静止中の GRV 平均で決めるために使う）
#define ESKF_STATIC_GYRO_RADS   0.026f    // 約 1.5 deg/s
#define ESKF_STATIC_ACCEL_TOL   0.30f     // |加速度| と重力の差 [m/s²]
#define ESKF_STATIC_INIT_US     2000000UL // この時間だけ静止が続いたら初期化 [µs]

// 初期姿勢の不確かさ [度]
#define ESKF_INIT_ATT_SIGMA_DEG         3.0f   // 静止平均から初期化した場合
// 静止区間が取れないまま走り出した場合のフォールバック。
// 初期姿勢が信用できないので大きめに取り、GNSS 観測で速く引き戻せるようにする。
#define ESKF_INIT_ATT_SIGMA_MOVING_DEG  30.0f
// ---- 初期ヨー ----
// ★ **0.983 で地磁気（ROTATION_VECTOR）をやめ、GNSS 航跡から入れる方式にした。**
//   理由は 2 つ。
//   (1) 地磁気が当てにならない。実測（2026-08-19 車載 3 セッション）で BNO085 の
//       磁気ヨーは真方位から -31 / -42 / -67 度ずれ、2026-09-30（0.980 のヨー修正後）
//       でも GS 8.1m/s 走行中の GNSS 航跡と **57.8 度**ずれていた。
//       しかも BNO085 自身は「精度 25 度以内」と申告していたので自己申告は使えない。
//   (2) SCH16T には地磁気が無い（→ docs/imu_sch16t_plan.md）。
//   GNSS 航跡は真方位で、偏角補正も要らない。**磁気より素性が良い。**
//
// ★ 接地中に入れるのが要点。しきい値を飛行速度（対気 5.8〜9.0m/s）より十分低く
//   取ってあるので、注入の瞬間に機体は必ず接地している。接地中は横滑りできないので
//   **航跡＝機首方位が厳密に成立し、クラブ角の誤差が原理的に入らない。**
//   2025 大会の実機体ログでは発進から 2〜3 秒で 3m/s を超えていた
//   （tools/olddata/PONSv5_planelog_2025-07-26_1539.csv の 15:42:15〜19）。
#define ESKF_YAW_INIT_MIN_SPEED_MPS  3.0f  // 水平対地速度がこれ以上でヨーを入れる
#define ESKF_YAW_INIT_ARM_FIXES      2     // これだけ連続で満たしたら注入（単発の化け対策）

// 水平化直後（ヨー未知）の初期σ [度]。180 = 全く不明。
// ★ ここを小さくしてはいけない。既存の eskf_yaw_reliable() が yaw_acc95 (=2σ) で
//   表示の可否を決めているので、180 にしておけば画面が自動的に「信頼できない」側に倒れる。
#define ESKF_YAW_UNKNOWN_SIGMA_DEG   180.0f
// GNSS 航跡から入れた直後の初期σ [度]。
// 接地中なのでクラブ角は無いが、GS 3m/s では航跡の分解能が sAcc/GS ≒ 2 度あり、
// 発進中は機首が振れるので余裕を見る。
#define ESKF_YAW_INIT_SIGMA_DEG       15.0f

// ---- 静止中の再水平化 ----
// ★ **これが無いと「プラットホーム上で起動して待つ」運用が壊れる。**
//   ヨーが未設定のあいだは GNSS 速度観測を入れない（ヨー誤差がロール・ピッチへ
//   転嫁されるため）。つまり待機中は観測が 1 つも無く、ロール・ピッチは
//   ジャイロ積分に任される。傾き誤差 = ジャイロオフセット × 経過時間 なので、
//     SCH16T（生オフセット 0.1deg/s typ・0.3 max）→ 10 分で 60〜180 度（致命的）
//   ★ **BNO085 の真のバイアスは測れない。** 静止中の出力は 3 軸とも
//     **厳密に 0.0 に張り付く**（2026-10-01 に実ログで確認: 静止 159 秒 7958 サンプルで
//     異なる値が 1 種類しかない）。さらに量子化 LSB が 1/512 rad/s = **0.112 deg/s** で、
//     SCH16T のオフセット仕様（0.1deg/s typ）とほぼ同じ。
//     **BNO085 にはここで問題にしている量を表現する分解能が無い。**
//     だから「BNO085 なら無害」とは言えず、この対策はセンサーに関係なく必要。
//   静止しているあいだ重力方向へ寄せ続けることで、待機時間の長さに関係なく
//   ロール・ピッチを保つ。**回転軸が必ず水平になるのでヨーは動かない。**
// ゲインはジャイロ 1 サンプルあたり。0.02 × 50Hz → 時定数 1 秒。
#define ESKF_RELEVEL_GAIN            0.02f

// ---- ヨーが決まった後の、地上での静止中の再水平化 ----
// ★ 上の再水平化はヨーが決まるまでしか働かない（ヨーが決まる＝発進した＝飛んでいる、
//   とみなしていた）。しかし屋内の偽の速度や車での移動でもヨーは決まり、そのあと
//   傾きの誤差が 60 度を超えると、重力観測（f_w が下向きなら入れない）も GNSS 速度も
//   戻せず、再起動まで直らなかった（2026-10-27。VertAccel が -18 m/s² になり、
//   バリオが数分沈降で鳴り続けた）。
//   そこで「地上で止まっている」と言えるときは、ヨーが決まった後でも重力へ寄せる。
//   地上の判定:
//     ・静止判定が ESKF_GROUND_STATIC_US 続いている
//     ・GNSS が「動いている」と言っていない（信用できる測位〔sAcc <= ESKF_SACC_MAX〕の
//       対地速度が ESKF_GROUND_MAX_GS_MPS 未満、または信用できる測位が新しくない）
//   人力機は飛行中に対地速度が 1m/s を下回らない。GNSS 断中の飛行で静止判定が
//   2 秒続くことはまれで、仮に寄せても直進中の比力はほぼ鉛直なので害は小さい。
// ★ ヨーには触らない（level_toward_gravity の回転軸は水平。共分散もロール・ピッチの分だけ
//   入れ直す）。ジャイロのゼロ点の学習は、GNSS が対地速度で地上を確かめたときだけ
//   （GNSS 断中の飛行で、ゆっくりした旋回をゼロ点として覚えないため）。
#define ESKF_GROUND_STATIC_US        2000000UL
#define ESKF_GROUND_MAX_GS_MPS       1.0f

// ============================================================
// 重力観測（levelling update）— Phase 2
// ============================================================
// ESKF の観測は GNSS 速度だけ（H = [0 I 0 0]）なので、ロール・ピッチは
// d(dv)/dt = -[Rf]x dtheta の結合を通して**間接的にしか**拘束されない。
// つまり **GNSS が無い間、重力の錨が 1 つも無い**（0.982 までは BNO085 の GRV が
// 内部で常時その錨を提供していた）。実測で GNSS 断 2 分で 2〜4 度ずれ、
// **バンク警報のしきい値 4 度**に届く。
//
// 準定常のとき比力をワールドへ回した f_w は (0,0,g) になるはずなので、
// その水平 2 成分を観測量にして dtheta の水平成分を直す。
//   d f_w_x = +g * dtheta_y,  d f_w_y = -g * dtheta_x
//   → H は 2x12 で H[0][1]=+g, H[1][0]=-g だけが非零。**ヨーには構造的に効かない。**
//
// ★★ **ゲートが本体。** 無条件に入れると協調旋回でバンクを過小評価し、
//   BNO085 の GRV と同じ間違い＝ ESKF が存在する理由を壊す。
//
// ★ **`|f| ≒ g` では人力飛行機のバンクを判別できない。** |f| は g/cos(φ) で二次なので
//   φ=5 度では |f|-g = 0.0375 m/s² しかなく、加速度計のスケール誤差に埋もれる
//   （BNO085 実測 -1% = 0.098 m/s² ＝信号の 2.6 倍 / SCH16T max 0.25% = 0.0245）。
//   `|f|` は持ち上げ・衝撃・落下のような桁違いの事象を切る**粗い健全性チェック**だけ。
//
// ★ **旋回レートが正しい判別量。** φ=5 度で 6.14 deg/s (V=8) あり、ジャイロの
//   オフセット（SCH16T 0.1 typ / 0.3 max）に対して 20〜61 倍の余裕＝ S/N が約 150 倍良い。
//   ★ 決定的なのは**ジャイロだけから作れる**点。この観測はまさに GNSS が無い場面の
//     ためにあるので、GNSS 由来の水平加速度でゲートすると必要なときに限って使えない。
//
// しきい値 1 deg/s の根拠: 水平化の向き誤差を 1 度以下に抑える条件
//   V*psi_dot/g < tan(1deg) → psi_dot < 1.23 deg/s (V=8)。
//   既存の ROLL_TRIM_YAWRATE_DPS と同じ値で、自動ロールトリムは**すでに同じ露出を
//   受け入れている**（直進中にロールを 0 へ寄せる）。
//
// ★ 瞬時値も併せて見る。2 秒 LPF だけだと旋回の入り口で漏れてバンクを過小評価する。
// ★ フゴイド（周期 2〜6.7 秒・ピッチ std 1.23 度）は**ゼロ平均の振動なのでゲートしない**
//   （瞬時値で切ると巡航中ほとんど閉じる）。切るべきは持続的・非ゼロ平均の誤差だけ。
#define ESKF_LEVEL_ENABLED            1
// 観測を入れる周期 [µs]。GNSS 速度観測（2Hz）と同じ桁にして、どちらかが
// 一方的に強くなりすぎないようにする。
#define ESKF_LEVEL_INTERVAL_US        500000UL
// 観測ノイズ σ [m/s²]。実ログでの掃引（GNSS 断の誤差中央 / 5 度バンクの出力）:
//   なし 5.04 / 5.16   σ=0.1 → 2.46 / 4.71   σ=0.3 → 2.36 / 4.92
//   σ=0.5 → 2.44 / 5.03   σ=1.0 → 2.57 / 5.12
// σ=0.5 はドリフト改善がほぼ最大で、バンクの劣化が σ=0.3 の半分。
// ★ 大きめに取るのは、ゲートを通ってしまう残留マヌーバ（1deg/s で水平加速度
//   0.17 m/s²）と、H に入れていない水平加速度バイアスを含めるため。
#define ESKF_LEVEL_SIGMA_MPS2         0.5f
#define ESKF_LEVEL_MAX_YAWRATE_DPS    1.0f   // ワールド系ヨーレートの持続成分
#define ESKF_LEVEL_MAX_YAWRATE_INST   3.0f   // 同・瞬時値（旋回入り口の漏れ対策）
#define ESKF_LEVEL_ACCMAG_TOL_MPS2    0.5f   // ||f|-g| の粗い健全性チェック
// 前後加速度の持続成分 [m/s²]。1 度の誤差に相当するのが 0.17。
// ★ GNSS が無いと作れない。断中はこの条件を省く（ピッチ側の誤差を許容して
//   錨が無いより良しとする。σ で吸収する）。
#define ESKF_LEVEL_MAX_LONGACC_MPS2   0.2f
// 全角速度 |ω| の上限 [deg/s] と、それを下回り続けていなければならない時間 [µs]。
// ★ 上の旋回レートのゲートは**ワールド系の鉛直軸まわりしか見ない**ので、水平な軸まわりの
//   回転（手で本体を回す・起こす）は素通りしていた。回している最中の比力には回転に伴う
//   加速度が乗るので、それを重力として観測すると、相関を通してジャイロのゼロ点 bg_ が
//   1 回で 0.3deg/s 以上動く（2026-10-27 の実機ログ。90 度回す間に 4 回通って
//   |bg| が 0.96deg/s になり、平置きの間に傾きが毎秒 0.9 度ずれた）。
//   人力機の飛行中の角速度は旋回を除けば数 deg/s（フゴイドで約 3）なので、
//   10deg/s を 0.5 秒下回っていることを条件にしても、巡航中のゲートはほぼ閉じない。
#define ESKF_LEVEL_MAX_RATE_DPS       10.0f
#define ESKF_LEVEL_RATE_QUIET_US      500000UL

// ---- 水平化前の「重力方向」の暫定値（attitude.cpp の up_lp_）----
// ★★ **バリオは起動直後から鉛直加速度を要求する。** ESKF の水平化（静止 2 秒）を
//   待つ作りにしたら、手持ちで静止区間が取れないと **バリオが完全に止まった**
//   （2026-10-01 実機。「高度は追従するのに V/S だけ 0 で固まる」）。
//   **機序は imu.cpp の imu_feed_accel() のコメントにある**（predict を 1 回も
//   通らないとゲインが 0 になる）。だから加速度の低域だけで重力方向を常に持つ。
//   ★ 旋回中は比力が鉛直からずれるので ESKF の代わりにはならない。
//     あくまで水平化までの繋ぎで、品質は 0.982 までの GRV と同程度。
// ゲインは加速度 1 サンプルあたり。0.02 × 約 63Hz → 時定数 0.8 秒。
#define ESKF_UP_LP_GAIN      0.02f
// |a| が g からこれ以上外れている間は更新しない（持ち上げ・衝撃・落下）[m/s²]
#define ESKF_UP_LP_ACC_TOL   2.0f

// ---- 静止中のジャイロバイアス学習 ----
// 静止中のジャイロ平均はそのままゼロ点オフセット。これを bg_ の初期値にする。
// ★ 目的は「待機中のドリフト対策」ではない（それは上の再水平化が担う）。
//   **離陸直後のいちばん動的な時期に、ESKF が姿勢とジャイロバイアスを
//   同じ GNSS 速度観測から同時に学ぶのを避ける**ため。BNO085 は内部校正済みで
//   学ぶべきものが無かったが、SCH16T は 0.3deg/s あり得る。
//   データシート §4.4 も「基板実装でオフセットが乗るので実装後のゼロ点合わせを推奨」
//   と書いている（工場校正は実装後のオフセットを知らない）。
// ★ **フラッシュへ保存しないこと。** SCH16T の offset drift velocity は
//   ±0.01(deg/s)/min（0.5K/min 時）で、起動後の昇温（実測 CPU 20→36℃）で動く。
//   一方 bias instability は 0.3〜0.5 deg/h しかないので、
//   **その飛行の中では一度測れば最後まで有効**。毎回測り直すのが正しい。
// ★ クランプが要る。静止判定は |ω| < ESKF_STATIC_GYRO_RADS（1.5deg/s）で、
//   測ろうとしているバイアスの 5〜15 倍のしきい値しかない。ボートのゆっくりした
//   回頭を「静止」と誤認してバイアスとして焼き付ける余地がある。
//   ただし誤学習が起きるのは鉛直軸まわり（ヨー）が主で、ヨーは yaw_acc95 で
//   ゲートされ、ヨー設定後は ESKF が GNSS から学び直すので害は小さい。
//   ロール・ピッチ軸で緩やかな定常回転が続くと傾きが変わるので再水平化が抗う。
#define ESKF_BIAS_LEARN_SAMPLES      250      // 50Hz で 5 秒ぶん
#define ESKF_BIAS_LEARN_MAX_RADS     0.0087f  // 0.5 deg/s（SCH16T max 0.3 に余裕）

// IMU データが途絶したと判断するまでの時間 [µs]。
// これを超えてジャイロが来なければ ESKF を無効扱いにし、GNSS 観測も取り込まない。
//
// これが無いと、BNO085 断線後にジャイロ・加速度が止まっても GNSS 速度だけ入り続け、
// フィルタが速度の食い違いを姿勢誤差のせいにして姿勢を回してしまう。
// 実測（模擬）では水平のまま加速しただけで 60 秒後にロールが +8 度になり、
// バンク警告の閾値 4 度を超えて偽警報が出た。
//
// 500ms は通常の欠測（地図描画による最大 206ms の遅延）より十分大きく、
// かつ異常を素早く捕まえられる値。
#define ESKF_IMU_TIMEOUT_US  500000UL

// 機体ゼロ点較正を許可する対地速度の上限 [m/s]。
// 飛行中に実行されると傾いた姿勢を水平として焼き付け、バンク角警告が
// 機能しなくなるため、地上でしか通さないためのガード。
#define LEVEL_CALIB_MAX_MPS  1.0f
// APPLY をボタン操作から遅らせる時間 [µs]。
// スイッチを押す力でマウント上のデバイスがわずかに傾くため、押した瞬間の姿勢を
// 焼き付けるとゼロ点がずれる。指を離して機体が落ち着いてから実行する。
#define ESKF_APPLY_DELAY_US  1000000UL

// ---- 較正時に申告するピッチ角の範囲 [度] ----
// 「いま機体は何度か」を人が申告して、その値になるよう較正するための選択肢。
// 本番プラットホームは -3.5 度。試験飛行場の地面も水平とは限らない。
// マウントは充電などで頻繁に付け外しするので、その場で直せる必要がある。
#define PITCH_TARGET_MIN_DEG   (-3.5f)
#define PITCH_TARGET_MAX_DEG   ( 2.0f)
#define PITCH_TARGET_STEP_DEG  ( 0.5f)
// APPLY 時に申告するロール角の範囲 [度]。既定は 0。
// 本来ロールは 0 の状態で較正するのが正しいが、横風が強いと機体をウィングローで
// 保持したまま待機せざるを得ないことがある。その場合だけ実際の値を申告して使う。
// ※ 申告値を間違えるとバンク警報の基準が黙ってずれる。水平にできるなら 0 で較正すること。
#define ROLL_TARGET_MIN_DEG    (-3.0f)
#define ROLL_TARGET_MAX_DEG    ( 3.0f)
#define ROLL_TARGET_STEP_DEG   ( 0.5f)

// ---- 地上待機中のバンクチェック ----
// 自動ロールトリムは対地速度 3m/s 超でしか働かないため、地上では一切動かない。
// マウントのズレを直せるのは地上だけなので、待機中は別建てで実測ロールを見張る。
// しきい値 3 度は、横風時のウィングロー待機（2 度程度はよくある）を誤検出しない値。
#define GROUND_ROLL_WARN_DEG         3.0f
// ピッチ側は「申告値 SET PITCH からのズレ」で見る。地上の機体ピッチは場所によって
// 変わる（プラットホーム -3.5 度、平地 0 度）ので絶対値では判定できないため。
// ※ 較正した場所と違う場所で静止していると誤検出する。運用上は
//   プラットホーム上で APPLY してからそのまま待機する流れを想定している。
#define GROUND_PITCH_WARN_DEG        3.0f
// ---- プラットホーム近傍ではピッチ基準を「区間」にする ----
// 台の上（-3.5 度）と台の手前の地面（0 度）は、装置からは区別がつかない。
// 平地で APPLY してから台へ上げると SET PITCH との差が 3.5 度になり、
// マウントは正常なのにピッチ警告が鳴ってしまう。
// そこでプラットホームからこの距離以内では、基準を一点（SET PITCH）ではなく
// PITCH_TARGET_MIN_DEG 〜 0 度の区間とみなし、その外へ GROUND_PITCH_WARN_DEG
// 出たときだけ発報する（実質 -6.5 〜 +3.0 度が許容）。
// ※ SET PITCH は書き換えない。あれは次回 APPLY の申告値も兼ねており、勝手に
//   変えると平地で APPLY したときに姿勢オフセットへ誤差が黙って焼き付くため。
// ※ この緩和が効くのはプラットホーム周辺だけ。試験飛行場では pla_lat/lon が
//   遠いので従来どおりの判定になる。ロール警告は無関係に効いたままなので、
//   マウントずれの検出経路自体は残る。
#define PLATFORM_NEAR_KM             0.1
// 距離判定の再計算間隔 [ms]。haversine は double 演算で重く、毎ループ回す価値がない。
// 地上で静止しているときにしか使わず、発報まで 60 秒かかる判定なので 1 秒で十分。
#define PLATFORM_NEAR_RECHECK_MS   1000UL
#define GROUND_ROLL_WARN_HOLD_MS   60000UL   // この時間continuous に超えたら発報
#define GROUND_ROLL_WARN_INTERVAL_MS 120000UL // 発報間隔の下限（鳴り続けない）
// 「較正のやり直しが必要」を音で促す間隔 [ms]。**地上にいるときだけ鳴らす**
//（飛行中は較正できないので、対処できない警告で注意をそらすほうが危険）。
#define ESKF_APPLY_WARN_INTERVAL_MS 120000UL

// ---- マウントから外されたことの検出 ----
// 機体に付いている限りロールもピッチもこの角度には達しない。超えたら
// 「手に持って外した」とみなし、次に APPLY するまで較正が無効だと警告する。
// 電源を切って充電するケースに備え、この状態は SD に保存して起動をまたいで保持する。
#define OFF_MOUNT_DEG               30.0f
// 右バンクだけ緩める [度]。プラットホームへ機体を上げる際に大きな右バンクを伴うため、
// 30 度のままだと「マウントから外した」と誤判定する。
// ロールの符号は「正 = 右バンク」。2026 年大会の実測で、右旋回(ヨーレート>0)のとき
// ロールが正、左旋回で負になることを確認済み（相関 +0.82）。
#define OFF_MOUNT_RIGHT_ROLL_DEG    40.0f
// 機体ゼロ点オフセット（level_roll / level_pitch）の上限 [度]。
// SD の settings.txt から復元するときのクランプに使う。APPLY は地上でしか通らず、
// マウントに載っている限りこの角度には達しない（超えるならマウント外れ検出が先に働く）。
// 上限を OFF_MOUNT_DEG に揃えてあるのは、「本当の異常は隠さず、
// 壊れた設定ファイルだけを弾く」ため。
#define LEVEL_OFFSET_LIMIT_DEG      OFF_MOUNT_DEG

// ---- IMU のマウント回転（センサー座標 → 機体軸）----
// q_body = q_sensor ⊗ q_mount。実体はセンサー Y 軸まわり -90 度。
//
// ★ **Euler を組み替える方式に戻してはいけない。**
//   v7 の BNO085 の配置（IC が基板裏面・1番ピンが天）では、機体が水平のときセンサーが
//   ちょうど **ジンバルロック（sensor_pitch = ±90°）** に入る。そこでは
//   sensor_roll と sensor_yaw が独立した量ではなくなるので、3 つの角を
//   入れ替えても絶対に正しくならない（実機で roll 0 なのに pitch が +90/-270 を
//   示し、roll 欄にヨーが出る、という壊れ方をした）。
//   クォータニオンの段階で回してから 1 回だけ Euler を出すこと。
//
// ★★ **下の値は v7 基板の BNO085 で測ったもの。SCH16T（IMU 基板）の向きでは測っていない。**
//   v7.1 の実機で同じ 3 点を測り、違っていたらここを差し替える（docs/imu_sch16t_bringup.md）。
//   違ったまま飛ぶと、ロールとピッチが入れ替わったり符号が逆になったりする。
// 2026-09-21 に v7 の実機で測定（静止させて生加速度＝重力ベクトルを読む）:
//   水平          a=(-9.8,   0,   0) → 機体の「下」= センサー +X
//   機首を真下90  a=(  0,   0,-9.8) → 機体の「前」= センサー +Z
//   右翼を真下90  a=(  0,+9.8,   0) → 機体の「右」= センサー -Y
// マウントを変えたら、この 3 点を測り直してここだけ差し替える。
//
// ★ **方位が 90 度ずれても、ここを回して直そうとしないこと。**
//   ヨーの基準（ENU は東が 0、方位は北が 0）は attitude.h の
//   imu_yaw_to_heading_deg() の担当。ここを機体の鉛直軸まわりに 90 度回すと
//   方位は合うが **ロールとピッチが入れ替わる**（バンク 15 度がピッチ -15 度になる）。
#define IMU_MOUNT_QW   0.70710678f
#define IMU_MOUNT_QX   0.0f
#define IMU_MOUNT_QY  (-0.70710678f)
#define IMU_MOUNT_QZ   0.0f

// バンク角の警告閾値 [度]。これを超えたら地図上のロール表示を赤にする。
// HPA は旋回半径が大きく、実運用のバンクは数度程度。
// （対気 7m/s・バンク 5度で旋回半径 約75m）
#define BANK_WARN_DEG  4.0f

// 姿勢表示の「ありえない値」のしきい値 [度]。絶対値がこれ以上なら灰色にする。
// 人力飛行機の実運用ではロールもピッチもここには届かない
// （2026年大会 45 分の実測でロール最大 7.16 度）。それを超えて出ているときは
// 機体の姿勢ではなく PONS の取り付けミス・較正忘れを疑うべきなので、
// 数値を信用しないよう灰色で示す。赤（バンク超過警告）より優先する。
#define ATTITUDE_IMPLAUSIBLE_DEG  10.0f

// ピッチによる警告は行わない。
// ピッチ単独では失速余裕を判定できず（本来は迎角が要る）、実測でも
// 平均ピッチと対気速度の相関は -0.14 と弱かった。
// 迎角推定も検討したが、対地経路角の 平均が中央 -0.01 度・|2度|超えが 0% で
// 補正項がほぼゼロ、ノイズだけ増える（std 0.73 → 0.88 度）ため見送った。
// ピッチは表示のみとし、閾値は実飛行で失速側のデータが蓄積してから決める。

// ---- バンク警告 ----
// 実測（2026-07-26 琵琶湖 45分）での発報回数:
//   閾値4度・継続条件なし → 31回（中央継続 0.2秒。チカチカするだけ）
//   閾値4度・1秒以上継続  → 3回   ← これを採用
#define BANK_WARN_HOLD_MS      1000UL   // この時間continuous に超えたら発報
// 再武装（再度鳴らせる状態に戻す）条件。想定している誤警報は「センサーのズレで
// バンクが 4 度以上のまま延々と続く」形なので、時間経過だけでは解除しない。
// バンクが十分戻り、かつ WAV の再生が終わるだけの時間が経ってから解除する。
#define BANK_WARN_CLEAR_DEG     3.0f    // これ未満まで戻ったら「解除できる」状態
#define BANK_WARN_REARM_MS    10000UL   // 発報からこの時間は解除しない（WAV 再生の待ち）

// ---- 直進中のロール自動トリム ----
// 飛行機は直進中のロールが理論上 0 になるはずなので、そこからのズレを
// ゆっくり戻す。飛行前の較正ミスや飛行中に PONS が傾いた場合の保険。
// 誤警報の連発を防ぐのが目的で、真のバンクを消さないよう条件を厳しくしてある。
#define ROLL_TRIM_YAWRATE_DPS   1.0f    // これ未満の旋回レートを「直進」とみなす
#define ROLL_TRIM_MIN_SPEED     3.0f    // これ以上の対地速度（＝飛行中）でのみ働く
#define ROLL_TRIM_WINDOW_S     60.0f    // この時間ぶん直進が続いたら 1 回評価する
#define ROLL_TRIM_DEADBAND_DEG  0.5f    // ズレがこの値未満なら補正しない
#define ROLL_TRIM_STEP_DEG      0.5f    // 1 回の補正量の上限（徐々に効かせる）
#define ROLL_TRIM_LIMIT_DEG     5.0f    // 自動補正の累積上限（本当の異常を隠さないため）
// 2 窓連続で同符号・不感帯超過のときだけ補正する。
// 実測（2026年大会45分）で 60 秒窓の平均ロールは std 0.27 度ばらつき、不感帯 0.5 度でも
// 約 10% の窓がノイズだけで発動する。1 窓で補正すると 51 分の飛行で ±0.7 度ほど
// ランダムウォークするので、2 窓の一致を要求してこれを約 0.5% まで落とす。
// 代償は収束が 2 倍遅くなること（1 度のズレが約 3.3 分 → 約 6.6 分）。
// 何窓連続で「同符号・不感帯超過」なら補正するか。想定値は 1〜4。
//   1 : 毎窓補正する。収束は速いがノイズだけで動く（実測で 50 分に ±0.7 度の
//       ランダムウォーク）。試験用。
//   2 : 推奨。誤発動を約 10% → 約 0.5% に落とせる。収束は 1 窓の 2 倍遅くなる
//       （1 度のズレが約 3.3 分 → 約 6.6 分）。実運用はこの値を使うこと。
//   3,4: さらに厳しくする。誤発動はほぼ無くなるが、60 秒窓 × N なので
//       短い試験飛行では一度も補正が入らなくなる。
#define ROLL_TRIM_CONFIRM_WINDOWS  2
#if (ROLL_TRIM_CONFIRM_WINDOWS < 1) || (ROLL_TRIM_CONFIRM_WINDOWS > 4)
  #error "ROLL_TRIM_CONFIRM_WINDOWS must be 1, 2, 3 or 4 (2 recommended)"
#endif
// 累積補正量の表示色としきい値 [度]。正しく取り付いていれば実測上ほぼ 0 のままなので、
// 大きな累積は「取り付けか機体に異常がある」ことを意味する。
#define ROLL_TRIM_WARN_DEG      1.5f    // これ以上でオレンジ（ステータス丸も緑から外す）
#define ROLL_TRIM_ALERT_DEG     3.0f    // これ以上で赤

// ---- リプレイ用 ESKF 結果ログ ----
// 「そのとき画面に何が出ていたか」を後から再現するためのログ。
// 旧 euler/ は BNO085 由来だったが、ESKF 導入で役割が変わったので名前を変えた。
// 旧ファイルは再生時のフォールバックとして読む（IMU_REPLAYDATA_LEGACY_DIR）。
#define IMU_REPLAYDATA_DIR         "imu_replaydata"
#define IMU_REPLAYDATA_LEGACY_DIR  "euler"
#define IMU_REPLAYDATA_INTERVAL_MS  200UL   // 5Hz
// 再生中、姿勢ログのサンプルと CSV 行の時刻がこれ以上離れたら「その時刻のデータは
// 無い」とみなして非表示にする。姿勢ログが飛行 CSV より先に終わっている場合に、
// 最後のサンプルを貼り付けたまま固まって見えるのを防ぐ。
#define REPLAY_ATT_MAX_AGE_MS      2000UL

// ---- 平均ピッチ ----
// 瞬時ピッチは振動するため、巡航のトリム状態は平均で見る。
//
// 窓長は実測で決めた（2026 年大会・連続 3018 秒のピッチ）:
//   瞬時ピッチ  平均 +0.79 度 / std 1.23 度 / 範囲 -3.3〜+8.6 度
//   振動のパワー配分  周期 2〜6.7 秒 50% / 6.7〜20 秒 26% / 20〜100 秒 21%
//   移動平均のばらつき  5秒 0.43 / 10秒 0.37 / 15秒 0.34 / 30秒 0.34 / 60秒 0.27 度
//
// 卓越する振動は周期 2〜6.7 秒なので、10 秒窓で既に 1.5〜5 周期を平均できており、
// 30 秒まで延ばしても分解能は 0.37→0.34 度しか改善しない。
// 逆に長すぎると、周期 20〜100 秒の「本当のトリム変化」まで潰してしまう。
// 試験飛行が短いこともあるため、有効になるまでの待ち時間が短い 10 秒を採る。
#define PITCH_AVG_SEC          10.0f

// ヨー（機首方位）を信用してよい推定標準偏差 σ の上限 [度]。
// ESKF のヨーは水平加速度がある間しか可観測にならず、等速直進が続くと σ が育つ。
// この値以下のときだけ
//   ・地図上のヨー数値を黒（信頼できる）で表示する
//   ・自機アイコンを対地進路ではなく機首方位へ回す
// 超えたらグレー表示＋アイコンは対地進路のまま（誤った偏流角を見せないため）。
// 値は 95%（2σ）で表記する。航空分野の精度表記は 95% が慣例で、
// 1σ のまま判断基準に使うと 3 回に 1 回は外れることになるため。
// 20度(95%) は従来の 10度(1σ) と同じ動作。挙動は変えていない。
//
// 目安の根拠: 偏流角（HPA の巡航 7m/s に対し横風 1m/s で約 8度）と同程度までなら
// 機首方位に意味がある。厳しくすればほとんど回らなくなる。
// 実測（模擬）: 直進＋フゴイド±0.3m/s で 5 分後に 2σ=18.8度、10 分後に 14.4度。
//               完全等速だと 15 分でも 37度で閾値を割らない（安全側で正しい）。
#define ESKF_YAW_TRUST_95_DEG  20.0f
// しきい値まわりのばたつきを防ぐヒステリシス幅 [度]。
// 一度「信頼できる」と判定したら、95%値が (しきい値 + この幅) を超えるまでは
// 信頼できる扱いを続ける。逆方向（信頼できない → できる）はしきい値そのままで判定する。
//
// ヨーの 95%値は直進で育ち旋回で縮むため、長い直進脚のある本番では
// しきい値付近を行き来する。ヒステリシスが無いと、自機アイコンの向きが
// 「機首方位」と「対地進路」の間で切り替わり、偏流点線も出たり消えたりする。
// 変化は分オーダーなので高速なちらつきではないが、飛行中に見え方が変わるのは紛らわしい。
//
// 3 度は 20 度に対して 15%。偏流角（巡航 7m/s・横風 1m/s で約 8 度）に比べて
// 十分小さいので、23 度まで許容しても表示の意味は変わらない。
#define ESKF_YAW_TRUST_HYST_DEG 3.0f
// これを下回るときだけ、機首の先に偏流角を示す点線を描く。
// TRUST より厳しくしてあるのは、点線は「トラックとのわずかな差」を読ませる表示なので、
// ヨーの誤差が偏流角そのものと同程度あると意味を成さないため。
// 実測（車載・機動あり）のヨー精度は 95% 値で 2.5 度程度だった。
#define ESKF_YAW_DRIFT_95_DEG  10.0f

// ヨー誤差の分散の上限 [度]（1σ）。
// 等速直進・測位なしではヨーは不可観測なので、P_[2][2] は観測が入るまで
// 単調に育ち続ける。実機で 13 分放置したら 1σ が 750 度まで伸びた。
// **1σ = 180 度は「まったく分からない」と同義**で、それ以上育っても情報は
// 増えない。ログと表示が読めなくなるうえ、カルマンゲインの計算に極端な値が
// 入るだけなので、ここで頭打ちにする。観測が入れば普通に縮む。
#define ESKF_YAW_SIGMA_MAX_DEG 180.0f
// GNSS 速度観測の許容鮮度 [µs]。これを超えたら自動ロールトリムと風推定を止める。
// GNSS は 2Hz なので 3 秒 = 6 回分の欠測。短い遮蔽では止まらず、
// 「そもそも測位していない」状態だけを確実に弾く長さ。
#define ESKF_GNSS_VEL_TIMEOUT_US 3000000UL
#define ESKF_DRIFT_LINE_LEN_PX   40   // 機首の先に伸ばす点線の全長 [px]
#define ESKF_DRIFT_LINE_DASH_PX   5   // 点線 1 本の長さ [px]
#define ESKF_DRIFT_LINE_SKIP_PX   7   // 点線どうしの間隔 [px]
#define ESKF_DRIFT_LINE_GAP_PX    4   // 機首から点線を描き始めるまでの隙間 [px]

// ---- 対気速度の推定（ピッチからの曲線）----
// 「ピッチ 0 度で 7.0m/s で飛ぶ」という設計前提を使い、ピッチから対気速度を引く。
// ピッチが上がる（機首上げ）ほど遅く、下がるほど速い。
//
//     V(θ) = V0 * sqrt( K / (K + θ) )      θ: ピッチ [度]
//
// ★ 区分線形をやめてこの形にした理由。**これは近似曲線ではなく式そのもの**。
//   定常飛行では 揚力 = 重量 なので
//       ½ρV²S・CL = W    →    V ∝ 1/sqrt(CL)
//   CL が迎角に対して直線なら CL(θ) ∝ (θ + K) と書ける。K は
//   **揚力がゼロになるピッチまでの角度**で、K=10 は「ピッチ -10 度で揚力ゼロ」の意味。
//   これを上へ代入しただけ。線形補間と違い、当てはめた範囲の外でも形が崩れない。
//   従来の 3 点（-4度/9.0、0度/7.0、+5度/5.8）を K=10 の曲線は最大 0.17m/s で通る
//   （各点を厳密に通す K は 10.1 と 11.0 で、両端から求めた値もほぼ一致する）。
//
// ※ 機体・重量・重心で変わる値なので、機体が変わったら直すこと。
//   **ここは既定値で、実際の値は SD の settings.txt から上書きできる**
//   （`airspeed_v0` / `airspeed_k` / `airspeed_min` / `airspeed_max`。README 参照）。
//   読み込んだ値は起動時に log.txt の AIRSPEED MODEL 行に残る。
//   使うピッチは瞬時値ではなく平均（瞬時値は std 1.23 度の振動があるため）。
#define AIRSPEED_V0_MPS        7.0f   // V0: 巡航ピッチ 0 度での対気速度 [m/s]
#define AIRSPEED_CURVE_K_DEG  10.0f   // K : 揚力がゼロになるピッチまでの角度 [度]
#define AIRSPEED_MIN_MPS       5.8f   // 推定の下限 [m/s]（この機体で起こり得ない値を出さない）
#define AIRSPEED_MAX_MPS       9.0f   // 推定の上限 [m/s]

// ---- 風の推定 ----
// 風ベクトル = 対地速度ベクトル - 対気速度ベクトル。
// 対気速度の向きは機首方位（横滑り 0 と仮定）なので、ESKF のヨーが信頼できるときしか成立しない。
// 表示条件は ESKF_YAW_DRIFT_95_DEG（点線と同じしきい値）。
// 風の平滑化時定数 [s]。これが「表示されるまでに必要な直進時間」も決める。
// 実測（2026 年大会、直進 12193 サンプル）での風速の変動幅:
//   瞬時 std 1.3 m/s → 5秒 0.38 / 10秒 0.37 / 20秒 0.31 / 30秒 0.28 / 60秒 0.26 m/s
// 10 秒で既に 0.37 m/s まで落ちており、色分けの刻み(2.0 m/s)に対して誤差の桁が違う。
// 30 秒まで延ばしても 0.09 m/s しか改善しないので、表示が出るまでの待ち時間を優先する。
#define WIND_LPF_SEC            10.0f
#define WIND_MIN_SPEED_MPS       2.0f   // これ以上の対地速度でのみ推定する（地上で回さない）
// 矢印の寸法。0.980 までの 44/14/3 px は地図の上で小さく、飛行中に向きが読めなかった。
// 全長は固定（強さは色で表す）。
// ★ 全長だけ 1.5 倍で、矢じり・軸は 1.3 倍。**全長を 57px（1.3 倍）に縮めないこと。**
//   中心に風向の 3 桁（NotoSansBold15 で 27px 幅）が入るので、矢じりの底辺が
//   中心から 27/2 + 余白 だけ離れていないと、横向き・斜めのときに数字と重なる。
//   矢じりの底辺は中心から (LEN/2 - HEAD) = 15px にあり、ここがぎりぎり。
//   軸は数字を避けて中央で切れる（draw_wind_arrow() 参照）。
#define WIND_ARROW_LEN_PX          66   // 矢印の全長 [px]（強さは色で表すので長さは固定）
#define WIND_ARROW_HEAD_PX         18   // 矢じりの長さ [px]
#define WIND_ARROW_WIDTH_PX         4   // 軸の太さ [px]
// GNSS 速度の裏付けが切れている間に、風のブロック（矢印＋風速）へ重ねる × の線の太さ [px]。
// 地図の × （draw_nofix_cross）と同じ 3px。細いと地図の線に紛れて気づけない。
#define WIND_STALE_CROSS_PX         3
// 矢印の色分けのしきい値 [m/s]。地図が白背景なので、いずれも暗い色を使う
// （黄色やオレンジは白地でほぼ見えなかった）。
// ---- デモモードの疑似値（表示確認用。実機の推定には一切影響しない）----
// 機首の振れは疑似風から計算するが、横風が強いと偏流角が 60 度近くになり
// デモの見た目として不自然なのでここで頭打ちにする。
#define DEMO_CRAB_MAX_DEG       12.0f
#define DEMO_YAW_ACC95_DEG       3.0f   // 偏流点線が出る条件を満たす値にしておく

#define WIND_COL_LOW_MAX         2.0f   // これ以下は緑（COLOR_GREEN）
#define WIND_COL_MID_MAX         4.0f   // これ以下は紫（COLOR_WIND_PURPLE）。超えたら赤

// ============================================================
//  PONS Link（機体⇄ボート無線）
// ============================================================
//  送信前チェック（プリフライト）— link.cpp
// ============================================================
// 送信モードに入った直後だけ、数秒 mode 0 で**聴いてから**送り始める。
// 確かめるのは 2 つ:
//   ・選んだチャンネルの雑音レベルが妥当か
//   ・同じチャンネルに他の送信機がいないか（とくに同じ CH/SF/Group の PONS）
// ★ 後者は送信機側から見える唯一の機会。通常運転では送信機は送信の合間 mode 3 で
//   寝ているので、「送信機が 2 台いる」は受信機しか検出できない（link.h 参照）。
// 監視時間。他機は 1Hz 送信なので **3 発ぶんの受信機会**になる。
// ★ 5000 から下げた。同じ CH に送信機が 2 台いる状況は、どちらも同じ格納庫内の
//   至近距離なので RSSI が非常に強く、1 発でもほぼ確実に受かる。3 回あれば足りる。
//   長くしても得られるのは「遠くの他チームの弱い電波」で、それは警告する対象でもない。
#define LINK_PF_WINDOW_MS       3000
// 雑音サンプル間隔。窓 3000ms に対して 9 サンプル取れる（下の FIRST 参照）。
#define LINK_PF_SAMPLE_MS        300
// ★ **1 回目だけは遅らせる。** e220_wake() で mode 3 から戻った直後はモジュールが
//   落ち着いておらず、実機では早すぎる問い合わせが必ず失敗していた
//   （AUX が High でも数百 ms 効かない）。2 回目以降は LINK_PF_SAMPLE_MS 間隔。
#define LINK_PF_FIRST_SAMPLE_MS  500
// 雑音の**中央値**がこれを超えたら「定常的にうるさい」とみなす。
// 実測の環境雑音は BW500kHz で約 -104dBm（docs/pons_link.md §8）なので 4dB の余裕。
#define LINK_PF_NOISE_WARN_DBM  (-100)
// 雑音の**最大値 - 中央値**がこれを超えたら「断続的に誰かが出ている」とみなす。
// 別 SF や他方式の送信はデコードできないので、この差でしか気づけない。
#define LINK_PF_BURST_DB           8
#define LINK_PF_WARN_SHOW_MS   10000   // 地図に警告ポップアップを出しておく時間
// 雑音の取得が**連続でこの回数**失敗したら、そこで点検を打ち切って NOT MEASURED にする。
// ★ 最後まで回してはいけない。1 回の問い合わせは応答が無いと待たされるので、
//   10 回ぶん繰り返すと Core0 が長時間止まり、GNSS の受信を取りこぼす。
//   「静かだった」ではなく「測れなかった」ので、途中で止めても失うものは無い。
// 雑音の問い合わせが連続で失敗したら、そこで**叩くのをやめる**回数。
// ★ 2 では足りなかった。起床直後の取りこぼしが 1〜2 回あるため、2 だと
//   本番のサンプルを 1 つも取らずに諦めてしまう（実機で踏んだ）。
//   Core0 を止めすぎない上限でもあるが、2 回目以降の待ちは 30ms へ落ちるので
//   4 回でも合計 200+30*3 = 290ms に収まる。
#define LINK_PF_NOISE_FAIL_MAX     4

// ---- 無線モジュールの生存確認 [ms]（link.cpp）----
// ★ 送信機は送信の合間 mode 3 で寝ているので、**送信そのものからは生死が分からない**。
//   モジュールが居なくても e220_send() は成功し、送信回数も地図の弧も増え続ける。
//   問い合わせて返事があるかを見るのが唯一の確実な方法なので、この間隔で叩く。
//   生きていれば数 ms で返るので負荷は無視できる。受信機も同じ間隔で確かめる
//   （受信が無いだけなのか、自分のモジュールが死んだのかを区別するため）。
// ---- 送信のノンブロッキング化（link_tx_tick の状態機械）----
// 起床の待ちを諦める時間。mode 3 → mode 0 のセルフチェック（5.4 / 図 36）が
// これを超えても終わらないなら、そのスロットは捨てて寝かせ直す。
// 捨てても次の秒でやり直すので、失うのは 1 サンプルだけ。
#define LINK_TX_WAKE_GIVEUP_MS    100UL
// 書き込んだあと「モジュールが送信を始めた（AUX が Low に落ちた）」のを待つ時間。
// ★★ **AUX の立ち下がりを見てから mode 3 へ落とすこと。** 見る前に落とすと、
//   モジュールがまだ送信を始めていない場合に packet を諦める可能性がある。
// ★★ そして**これが「本当に送信できたか」を確かめる唯一の手段**でもある。
//   AUX が Low に落ちないまま時間切れになったら、その書き込みは
//   モジュールに捨てられている（60 秒ログの noRf= がその回数）。
//   実機 2026-09-27: Sent は毎秒増えるのに電波が出ず、受信機は 300 秒で 2 発しか
//   受けなかった（bad=0・RSSI -34dBm）。この検出が無かったので気づけなかった。
// 値の根拠: UART の吐き出し 5.9ms ＋ ARIB のキャリアセンス数 ms。余裕を見て 50ms。
#define LINK_TX_TXSTART_MS         50UL
// 送信が終わる（AUX が High に戻る）のを待つ上限。
// ★★ **送信の完了を見てから mode 3 へ落とす。** e220_sleep() のコメントは
//   「送信中なら切替はハードが先送りする」としているが、**送信を始める前**に
//   落とした場合にどうなるかは保証が無い。実機 2026-09-27 に、送信側が
//   noRf=0（AUX は落ちている）なのに受信側が 93% 落とす状態を観測した。
//   完了まで待てば、少なくとも「こちらが打ち切った」可能性は消える。
//   ついでに**実測の送信時間**が取れる（docs/pons_link.md §3 の表の検算になる）。
// 値の根拠（**実測で改訂。2026-09-27**）:
//   SF9(prof=2) の AUX Low は **250〜276ms** だった。docs/pons_link.md §3 の表は
//   SF9=180ms なので、**実測はその約 1.5 倍**（差は ARIB のキャリアセンスと
//   モード遷移ぶん）。同じ比率なら SF11(表 600ms)は **900ms 前後**になる。
//   800ms では SF11 で取りこぼすので 1000ms にした。
// ★★ **SF11 は 1 秒周期に対して余裕がほとんど無い。** 表の占有率 60% は
//   キャリアセンスを含んでおらず、実測では 90% 近くになる見込み。
//   SF11 を常用するなら送信周期を見直すこと（docs/pons_link.md §3）。
#define LINK_TX_DONE_MS          1000UL

#define LINK_ALIVE_PROBE_MS    10000UL

// 設定が通っていない（e220_alive() が false）状態から復帰を試みる間隔。
// ★ なぜ要るか: s_alive は e220_reconfigure() の中でしか立たない。
//   link_set_mode() は reconfigure を呼ばないので、**一度落ちるとモードを
//   変えても戻らない**（2026-09-23、設定画面で CH を変えた瞬間に無応答になり、
//   以降どう操作しても復帰しなかった）。
// ★ 復帰は e220_reconfigure() なので **Core0 が最大 0.6 秒止まる**。
//   GNSS の受信バッファは GNSS_SERIAL_FIFO_SIZE で約 7 秒ぶんあるので
//   0.6 秒では溢れないが、他の処理も止まる。だから間隔を長く取り、
//   **ping が通った（＝モジュールは生きている）ときだけ**試みる。
//   無線が死んだまま飛ぶより、1 分に 1 回 0.6 秒を払うほうが良いと判断した。
#define LINK_REVIVE_INTERVAL_MS  60000UL
// ★ 送信直後は問い合わせない。送信中は mode 0 への切替が保留される（AUX が Low の間は
//   切替が効かない）ので、問い合わせが素通りして「無応答」に見える。
//   最長は SF11・65 バイトで約 600ms（docs/pons_link.md §3 の表）。その外側に置く。
//   送信は 1Hz なので、待てば必ず静かな時間帯に当たる（先送りするだけで飛ばさない）。
#define LINK_PROBE_AFTER_TX_MS   700UL

// ---- 送信モードの電波アイコンを膨らませておく時間 [ms]（display_tft.cpp）----
// 地図の再描画は約 2Hz、送信は 1Hz なので、これくらいだと 1 回おきに
// 膨らんで見え、「1 秒に 1 回出ている」が目で分かる。
#define LINK_TX_PULSE_MS      600

// ---- 受信機で鳴らす「送信機の地上チェック」の間隔（link.cpp）----
// ★ 3 つ（ロール／ピッチ／較正やり直し）は**同時に立つ**。同じ優先度なので
//   同時に積むと互いを pending へ追い出し、断片が鳴ってから先頭から鳴り直す。
//   1 つ鳴らしたらこの時間だけ空けて次へ回す。最長の音声が 4.82 秒なので、
//   鳴り終わるまで次を積まない値にしてある。
#define LINK_GROUND_SAY_GAP_MS 6000UL

// ---- 地図に重ねる通知ボックスの縦位置（display_tft.cpp の begin_notice_box）----
// ★ **同時に出るものを重ねないこと。** 括弧内が実際に占める範囲。
//   同時に出うる組: GNSS 断線 ＋ 警報ミュート ＋（無信号 または 送信前チェック）。
//   排他な組: 無信号（受信モード）と送信前チェック（送信モード）→ 同じ段を使う。
//            送信機 2 台（ミラー中のみ）と警報ミュート（ミラー中は出ない）。
#define NOTICE_Y_GNSS_LOST    44   // 高さ 56 → 44..99 （GNSS モジュールの断線）
#define NOTICE_Y_DUP_SENDER   96   // 高さ 44 → 96..139（送信機が 2 台）
#define NOTICE_Y_WARN_MUTE   104   // 高さ 40 → 104..143（警報ミュート中）
#define NOTICE_Y_LINK_STATE  148   // 高さ 44 → 148..191（無信号 / 送信前チェック）
#define NOTICE_Y_PWR_FAULT   190   // 高さ 40 → 190..229（電源経路の異常。2px だけ上と接する）
// SW OFF のまま充電している注意と、高温での充電の警告も電源経路の異常と同じ段。
// 優先は 異常 > 高温 > SW OFF で、下位のものは上位が出ている間は出さない
// （power_sw_off_notice_active() と draw_charge_hot_notice() が見ている）。
#define NOTICE_Y_CHG_SW_OFF  NOTICE_Y_PWR_FAULT
#define NOTICE_Y_CHG_HOT     NOTICE_Y_PWR_FAULT
// ※ GNSS 断線（44..99）と送信機 2 台（96..139）は 4px だけ接する。両方出るのは
//   「受信機の GNSS が抜けていて、かつ送信機が 2 台」のときで、後から描く側が
//   相手の下枠を 4px 消すだけ。文字は隠れないのでそのままにしてある。

// ---- 受信枠の点滅（display_tft.cpp の link_frame_visible）----
// ★ **半周期を再描画間隔より十分長くすること。** 再描画は新しいデータか
//   SCREEN_FRESH_INTERVAL(1050ms) で起きるので、400〜500ms だとトグルと同じ速さで
//   サンプルすることになり、**点滅が止まって見えたり不規則にちらついたりする**
//   （VSI の右辺で踏んだのと同じ罠）。1000ms なら 2Hz 再描画で 1 相 2 回描かれる。
#define LINK_FRAME_STALE_MS   3000UL  // これより古い＝切れかけ／無信号 → 点滅へ
#define LINK_FRAME_BLINK_MS   1000UL  // 点滅の半周期

// ---- ミラー中の描画の窓（link.cpp の link_draw_window_ok）----
// 受信機は、送信機のフレームを受けている最中に TFT を描かない。16bit バスが動くと E220 の
// 雑音の値が最大で +8〜10dB 上がるため（debug/v7-noise-test の雑音試験。普段の描画で受信率が
// 落ちるところまでは確かめられていないので、手間の小さい保険として入れている）。
// 地図はフレームを受けた直後だけ描き直し、強制再描画は次の受信が始まる手前で締め切る。
#define LINK_DRAW_BUDGET_MS      250   // 描き始めから最後の転送までの見込み [ms]（地図 1 枚）
#define LINK_DRAW_MARGIN_MS      100   // 送信周期の揺れの分の余裕 [ms]
#define LINK_DRAW_MIN_WINDOW_MS   50   // 窓の下限 [ms]。SF11 は送信時間が長く、引き算すると窓が無くなる
// 送信周期の推定に使う、続けて受かった 2 発の間隔の範囲 [ms]（送信機は 1000ms + ループの遅れ）
#define LINK_TX_PERIOD_MIN_MS    900
#define LINK_TX_PERIOD_MAX_MS   1300

// ---- 受信モードで出す「ボート自身の位置」マーカー（display_tft.cpp）----
// 機体マーカーより目立たせないこと。主役はあくまでミラーした機体の画面。
#define OWNPOS_DOT_R            1     // 半径 1px = 直径 3px
#define OWNPOS_TRACK_LEN_PX     5     // 進行方向の線の長さ
#define OWNPOS_TRACK_MIN_MPS  2.0f    // これ未満は方位を出さない（停船中の方位は当てにならない）
