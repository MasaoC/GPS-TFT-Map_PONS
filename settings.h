//====== 設定画面 =======

// リリース時 削除
#define BUILDDATE 20261025
#define BUILDVERSION "0.504v3"

#define RELEASE
#define DEBUG_P(num,txt)  if(num >= BUILDDATE-1)Serial.print(txt);
#define DEBUG_PLN(num,txt)  if(num >= BUILDDATE-1)Serial.println(txt);

// ============================================================
// SD アクセスがどのコアから来たかの見張り（デバッグビルドのみ）
//
// SD へのアクセスは Core1 に閉じる規律だが、Core0 から直接触ると
// 偶発的な SDIO/SPI バスハングを引き起こすため、開発中に検知できるようにする。
// Core0 から good_sd() を呼ぶのは安全（フラグを読むだけ）なのでそこには置かない。
// RELEASE では空マクロになりサイズ不変。1秒に1回までに抑制。
// ============================================================
#ifndef RELEASE
  #define ASSERT_SD_CORE1(label) do { \
    if (get_core_num() != 1) { \
      static unsigned long _sdc_; \
      if (_sdc_ == 0 || millis() - _sdc_ >= 1000) { \
        _sdc_ = millis(); \
        Serial.print("!!! SD access from Core0: " label); \
        Serial.println(" (SD operations must be delegated to Core1)"); \
      } \
    } \
  } while(0)
#else
  #define ASSERT_SD_CORE1(label) do {} while(0)
#endif


//----------GPS---------
// GPS は u-blox SAM-M10Q 固定。UBX バイナリ（NAV-PVT / NAV-SAT）で受ける（0.503〜。v7 と同じ）。
// 以前の NMEA（TinyGPS++）・Mediatek 用の分岐は無い。

//GPSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GPS_SIM_BIWAKO         //琵琶湖固定座標
  //#define DEBUG_GPS_SIM_SAPPORO         //札幌固定座標
  //#define DEBUG_GPS_SIM_SHISHI         //しし固定座標
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換

// GPS が止まった・測位を失ったことの判定（gps.cpp）。以前は一度つながると「接続中」のまま、
// 一度測位すると「測位中」のままで、止まった位置と速度を今の値として出し続けていた。
#define GPS_DATA_TIMEOUT_MS  3000UL   // チェックサムの合う UBX のフレームがこの時間来なければ通信断
#define GPS_FIX_TIMEOUT_MS   3000UL   // 有効な位置がこの時間来なければ測位喪失
#define GPS_RESETUP_MS      10000UL   // 通信断がこの時間続いたら gps_setup() をやり直す（以後この間隔）

// 測位レート（gps_setup() が UBX-CFG-RATE で設定する）。v7 と同じ 2Hz。
// 出すのは NAV-PVT（毎回、約 100 バイト）と NAV-SAT（衛星の配置。普段は 10 秒に 1 回、
// GPS 詳細画面を開いている間だけ 1Hz）。NMEA は CFG-PRT で止める。
// ★ 量は 2Hz で約 0.25KB/秒。UART の FIFO（512 バイト）は描画が 2 秒止まってもあふれない
//   （NMEA を全部出していた頃は約 2.2KB/秒で、0.2 秒であふれた）。
#define GPS_RATE_HZ 2
#define GPS_ACK_TIMEOUT_MS 300        // UBX の設定 1 つあたりの ACK 待ち
// AssistNow Autonomous（GPS 自身が数日先までの軌道を予測して保存し、次の起動を速くする。v7 の 0.994〜）。
// ★ 効くのは GPS モジュールにバックアップ電池（V_BCKUP）があるときだけ。無ければ普通の起動に戻るだけで悪くはならない。
//   効いたかは log2.txt の「GPS FIXED ... after boot」（測位までの秒数）と「GPS ORBIT after 1st fix: ANA=..」。
// ★ この設定は GPS のバックアップ領域（BBR）にも書くので、0 にするときも「無効」を送る（v7 と同じ）。
#define GNSS_USE_ANA 1
#define GPS_ACC_SHOW_M 10.0f          // 水平精度（NAV-PVT の hAcc）がこれ以上なら地図に Acc を赤で出す（v7 と同じ）
#define MAXGS_SACC_MAX_MPS 1.0f       // 最大 G/S に入れる速度の精度（NAV-PVT の sAcc）の上限。飛んだ値を拾わない
#define MAXGS_WINDOW_MS 300000UL      // 最大 G/S の「直近」の幅（5 分）
#define GPS_ACC_LOG_MS  60000UL       // 精度（hAcc/sAcc/捕捉数）を log2.txt に残す間隔


//---------TFT----------
// TFT の種類（ST7789 / ILI9341）は TFT_eSPI の User_Setup.h で選ぶ（CopySetupFile_TFT_eSPI.h を写す）。
// ここには選ぶ設定は無い（以前の TFT_USE_ST7789 などはどこからも使われていなかった）。

// 地図を描き直す最小の間隔 [ms]。描き直しは新しい NAV-PVT を受けた直後（gps_loop() が true）だけなので、
// 実際の間隔は測位の周期（2Hz なら 500ms）になる。周期ぴったりにすると、前の描画が長引いて NAV-PVT の
// 処理が少し遅れただけでその回を飛ばして 1Hz に落ちるので、周期の 8 割にしてある。
#define SCREEN_INTERVAL (1000 / GPS_RATE_HZ * 8 / 10)

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 60000
#define BATTERY_PIN 26 //A0


#define MAX_TRACK_CORDS 500



// =====Hardware Settings =====

// TFTのBL調整があれば、BRIGHTNESS_SETTING_AVAIL。また、調整に使われているトランジスタ。NPN or PNP を指定する。（画面の明るさ設定のため）
//#define BRIGHTNESS_SETTING_AVAIL
//#define NPN_BL
//#define PNP_BL



// Hardware Ver1,2
/*
#define MEDIATEK_GPS
#define RP2040_ZERO
#define SW_PUSH   14
#define SW_UP     10
#define SW_DOWN   11
#define TFT_BL     7
#define GPS_SERIAL Serial2
#define GPS_TX 8
#define GPS_RX 9

#define SD_RX 0
#define SD_CS_PIN 1
#define SD_SCK 2
#define SD_TX 3
#define BATTERY_MULTIPLYER(adr) (adr/4096.0*3.3*2)
#define BAT_LOW_VOLTAGE 3.55
*/




// Hardware Ver3
#define RP2040_PICO
#define SW_PUSH 5
#define SW_UP   -1
#define SW_DOWN -1
#define TFT_BL  -1
#define GPS_SERIAL Serial1
#define GPS_TX 0
#define GPS_RX 1
//#define DISABLE_SD
#define SD_RX 4    //MISO
#define SD_CS_PIN -1
#define SD_SCK 2
#define SD_TX 3    //CMD MOSI
#define BATTERY_MULTIPLYER(adr) (0.0088*adr - 0.5357)  //いろいろトラブルシュートしているうちに、Vref2.95, 分圧1/11 になった数値。
#define BAT_LOW_VOLTAGE 3.49
#define SINGLE_SWITCH //外部スイッチ（up,down,push)がない場合。SINGLE。３個ある場合は comment out。

// ----- 電源の監視（充電 IC の故障の検出。power.cpp） -----
// 充電 IC（MCP73831）は SW の後ろにあり、壊れると「電池 → SW → 充電 IC → VBUS」と逆流して、
// USB を挿していないのに VBUS が立つ。Pico 基板に作り込まれた VBUS 検出（GPIO24）と
// VSYS÷3（GPIO29）で見る。v3 基板は Pico の VSYS に何も繋いでいないので、VSYS は
// 「VBUS − Pico の D1」になる。本物の USB なら 4.4V 以上、逆流なら電池電圧より低い。
#define VBUS_DETECT_PIN 24
#define VSYS_ADC_PIN    29
// ADC_VREF の電圧 [V]。v3 基板では Pico の R7（3V3 からの 200Ω）を外し、外付けの LDO（U2）で与えている。
// BATTERY_MULTIPLYER の「Vref2.95, 分圧1/11」と同じ値（その式の傾きから逆算しても 2.87〜2.95V）。
// 回路図の画像は「+2V5」だが、画像は v2 のファイルで電池の分圧（1/2）も実物（1/11）と違う。
// ★ R7 が残っている基板だと約 3.16V になる。その場合、VSYS は低めに出るだけで誤報する側には外れない
//   （USB 4.7V が 4.39V と出る）が、**CPU 温度は約 27°C 高く出て、充電のたびに高温の警告が出る**。
//   CPU 温度は ADC_VREF_V が 1% ずれると約 4°C ずれるので、高温での充電の警告を信用するには
//   テスターで Pico の 35 番ピン（ADC_VREF）を測ってここに書くこと。設定画面の下の CPU が室温 +5°C 前後なら合っている。
#define ADC_VREF_V 2.95f
#define VSYS_MULTIPLYER(adr) ((adr) / 4096.0f * ADC_VREF_V * 3.0f)
#define POWER_FAULT_VSYS_V     3.9f     // VBUS があるのに VSYS がこれ未満なら異常
#define POWER_VBUS_SEEN_VSYS_V 2.0f     // GPIO24 が Low でも VSYS がこれ以上なら VBUS あり（逆流の VBUS は 3V 前後で、GPIO24 では際どい）
#define POWER_FAULT_HOLD_MS    3000UL   // GPIO24 が High のまま VSYS が低い状態がこの時間続いたら異常（USB を抜いた直後の過渡を除く）
#define POWER_FAULT_HOLD_VSYS_MS 10000UL // GPIO24 が Low で VSYS だけで VBUS ありとみた場合はこちら。USB を抜いたあと
                                        // VSYS は Pico 基板のコンデンサで数秒かけて下がるので、それより長くとる
#define POWER_FAULT_ARM_MS     5000UL   // 起動直後は判定しない
#define POWER_LOG_MS          60000UL   // 電圧をログ（Serial と SD の log2.txt）に残す間隔

// ----- 電池の表示（power.cpp が判定、display_tft.cpp が描く。v7 と同じ基準） -----
#define BAT_FULL_VOLTAGE 4.2f   // 100%。満充電でも負荷で 4.2V を切るので 100% に届かないことがあるが正常
#define BAT_HALF_VOLTAGE 3.8f   // これ未満はマゼンタ（50% 未満）
#define BAT_ZERO_VOLTAGE 3.4f   // 0%
// 満充電からの目安稼働時間 [分]。設定画面フッターの電池稼働目安時間の計算に使う。
#define BAT_FULL_RUNTIME_MIN 240
// 表示する電圧は「上がるときは即追従・下がるときはゆっくり」（GPS 受信や描画の負荷の谷を拾わない）。
// ★ 下降は時間で鈍らせる。以前は描画のたびに 0.4 ずつ寄せていて、時定数が描画の回数で変わっていた
//   （v7 で 0.988 に同じ理由で直したもの）。
#define BATTERY_PEAK_HOLD_MS  3000UL
#define BATTERY_DECAY_TAU_MS 20000UL
// SW OFF のまま USB をつなぐと、充電 IC が電池を探して VCC が上下に振れる（数百 mV）。
// USB があるときに 2 秒の間の ADC の幅がこれを超えたら「BATOFF」。
#define BATOFF_SWING_COUNTS 100
#define BATOFF_HOLD_MS      5000UL     // 一度振れたらこの間は BATOFF のまま（表示がちらつかないように）

// ----- 高温での充電の警告（power.cpp） -----
// 電池（マルツの LiPo、保護回路付き）の充電温度は 0〜45°C。MCP73831 には温度を入れる端子が無いので、
// USB がつながっている間に RP2040 の内蔵温度センサーで見て、熱ければ USB を抜くよう画面に出す。
// ★ 測っているのは電池ではなく Pico のチップの温度で、電池より数度高く出る（＝早めに出る側）。
// ★ ADC_VREF_V が 1% ずれると約 4°C ずれる（上の ADC_VREF_V のコメント）。
#define CHARGE_HOT_C        45.0f
#define CHARGE_HOT_CLEAR_C  42.0f      // これ以下で解除（境目で出たり消えたりしない）
#define CHARGE_HOT_HOLD_MS  30000UL
#define CPU_TEMP_SAMPLE_MS   2000UL    // analogReadTemp() は 1 回 1ms 止まるので間引く
#define CPU_TEMP_MIN_C     (-20.0f)    // この外の値は ADC_VREF_V が違う証拠として使わない（警告もしない）
#define CPU_TEMP_MAX_C      90.0f

// ----- Auto 10km モード（GPS_TFT_map.ino の auto10k_check()。v7 と同じ判定） -----
// 目的地（通常は PLATHOME）から離れる向きに誘導し（10K AWAY）、AUTO10K_TURN_KM を超えたら
// 戻る向き（10K INTO）に切り替える。戻ってきて AUTO10K_RETURN_KM 以内に入ったら再び AWAY。
#define AUTO10K_TURN_KM   10.475   // 公式ルールの折り返しは 10.975km。潮流などを見込んで 500m 手前
#define AUTO10K_RETURN_KM 1.5      // 1km パイロンでの折り返しに 500m の誤差を足したもの
#define AUTO10K_NOTICE_MS 10000UL  // 切り替わったとき地図に出す知らせの時間（v3 は音が無いので画面で知らせる）
#define PLA_TURN_CIRCLE_KM 10.975  // 地図に描く折り返しの目安の円（PLATHOME 中心）

// ----- 誘導の表示（GPS_TFT_map.ino の nav_tick()。v7 と同じ判定。v3 は音が無いので表示だけ） -----
// 修正の円弧: 目的地の方位から COURSE_WARN_MIN_DEG 以上ずれたまま修正しないと、ズレ×秒を積算する
// （90 度で頭打ち）。正しい向きに旋回中（0.5deg/s 以上）は減らす。900 に達したら「Turn RIGHT/LEFT」を 10 秒。
#define COURSE_WARN_MIN_DEG   15.0f
#define COURSE_WARN_INDEX_MAX 900.0f
#define COURSE_WARN_SHOW_MS   10000UL
#define COURSE_WARN_REPEAT_MS 30000UL
// 30 度の範囲表示: 進路が基準から 15 度を超えて変わったら、基準を今の進路にして 8 秒点滅する
#define TRACK_RANGE_HALF_DEG  15.0f
#define TRACK_WARN_MS         8000UL
// 精度がこれより悪いときは、自機の三角の代わりに精度の円を出す（v7 の HACC_THRESHOLD_M と同じ）
#define HACC_CIRCLE_M         10.0f

// 設定画面（と GPS 詳細・地図一覧）を放置したとき、自動で地図画面へ戻るまでの時間（v7 と同じ 3 分）。
// 設定画面を出したまま離陸・出航してしまった場合の保険。ボタン操作のたびに数え直す。
#define SETTING_IDLE_TIMEOUT_MS (3UL * 60UL * 1000UL)

// ----- 本番前チェック（readiness.cpp。設定画面の右上に出す） -----
#define READY_HACC_MAX_M       5.0f   // GNSS の水平精度（NAV-PVT の hAcc）。v7 と同じ
#define READY_BATT_MIN_PERCENT 50     // 電池残量 [%]（50% ≒ 3.8V）




// =====追加設定項目====

// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。





//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GPSDETAIL 3
#define MODE_MAPLIST 4


//destination mode
#define DMODE_FLYINTO 0
#define DMODE_FLYAWAY 1
#define DMODE_AUTO10K 2   // Auto 10km（AWAY と INTO を距離で自動で切り替える。GPS_TFT_map.ino の auto10k_check()）

#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GPS)
    #warning NOT RELEASE!
  #endif
#endif