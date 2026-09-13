// ============================================================
// File    : link.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : PONS Link（機体⇄ボート無線）の RP2350 側インターフェース。
//           UART1 で無線モジュール(E220-900T22S)と話し、送信テレメトリの組み立てと
//           受信テレメトリの供給を行う。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/08
// ============================================================
//
// 受信データの供給関数は、意図的に gps.h のリプレイ用関数
// （replay_get_kf_altitude() など）と同じ形にしてある。
// 受信モードは「CSV の代わりに電波からリプレイする」ものとして実装できるので、
// 表示側の分岐を最小にできる。
//
#ifndef LINK_H
  #define LINK_H
  #include <Arduino.h>
  #include "lora_link/link_proto.h"

  // ---- 初期化・周期処理 ----
  void link_setup();          // UART1 を開き、設定を無線モジュールへ送る
  void link_loop();           // Core0 のループから毎回呼ぶ（軽い。1Hz・65Bのみ）

  // ---- 設定 ----
  // 設定画面から変更されたときに呼ぶ。無線モジュールへ即座に反映される。
  // ★ 電波の設定は「数値」でしかない。意味を決めるのは無線層（e220.cpp）。
  //   LoRa では radio_ch = E220 チャネル(CH0-12)、radio_profile = SF プリセット。
  void link_set_mode(uint8_t mode);          // LINK_MODE_OFF / TX / RX
  void link_set_radio_ch(uint8_t ch);        // 0..LINK_RADIO_CH_MAX
  void link_set_radio_profile(uint8_t p);    // 0..LINK_PROFILE_COUNT-1
  // グループ ID。**3 台とも同じ値**にすること。既定 0。
  // モジュールへの書き込みは不要（電波の設定ではなくペイロードの中身）。
  void link_set_group(uint8_t g);
  uint8_t link_get_group();
  // 「設定を変えたまま伝え忘れ」対策。無線画面の入室時と退室時に呼ぶ。
  // CH / SF / Group が変わっていたら音声と SD ログで知らせる。
  void link_remember_setting();
  void link_warn_setting_changed();
  uint8_t link_get_mode();
  uint8_t link_get_radio_ch();
  uint8_t link_get_radio_profile();

  // 設定ファイル(mysd.cpp)から直接読み書きするための実体
  extern uint8_t link_mode_setting;
  extern uint8_t link_radio_ch;
  extern uint8_t link_radio_profile;
  extern uint8_t link_group;

  // ---- 送信側（機体）----
  // ★ テレメトリの組み立てと送出は link.cpp の内部で完結している。
  //   刻み（1Hz）も link_tx_tick() が持つので、外から呼ぶ関数は無い。
  //   GPS 更新（2Hz）に合わせて呼ぶと電波の占有時間と送信電流が倍になるため、
  //   **意図的に公開していない**。

  // ---- 受信側（ボート／ピット）----
  // MIRROR = 電波が来ている / NO_SIGNAL = 10秒以上来ていない
  typedef enum { LINK_DISP_MIRROR = 0, LINK_DISP_NOSIGNAL = 1 } LinkDispState;

  // ミラー中か。各センサ accessor がこれを見て値を差し替える。
  bool link_mirror_active();

  LinkDispState link_display_state();     // 表示状態（受信モード以外では NOSIGNAL）
  bool     link_is_receiving();           // 直近に受信できているか
  uint32_t link_age_ms();                 // 最後の受信からの経過 [ms]
  int8_t   link_rssi();                   // 直近の RSSI [dBm]

  // 受信したテレメトリの生データ。表示側はこれを読む。
  const LinkTelem* link_rx_telem();

  // ---- 受信テレメトリの取り出し ----
  // ★ **LinkTelem の詰め方（スケール）を知ってよいのは link.cpp だけ。**
  //   呼び出し側で link_rx_telem()->kf_alt_dm / 10.0f のように書いてしまうと、
  //   同じ換算が 2 か所に存在することになり、分解能を変えたとき片方だけ直して
  //   「ミラー表示のときだけ桁がずれる」という静かな壊れ方をする。
  //   受信側の値が要るときは必ずこの getter を通すこと。
  //   どれも link_mirror_active() が真のときだけ呼ぶ前提（未受信なら 0 を返す）。
  bool  link_has_value(uint16_t havebit);
  double   link_get_lat();
  double   link_get_lon();
  double   link_get_gs();                 // 対地速度 [m/s]
  double   link_get_truetrack();          // 真方位 [deg]
  double   link_get_gnss_altitude();      // GNSS 高度 [m]
  int      link_get_numsat();
  uint32_t link_get_hacc_mm();            // 水平精度 [mm]
  bool     link_get_fix_ok();             // gnssFixOK
  float link_get_kf_altitude();
  float link_get_kf_vspeed();
  float link_get_pressure();
  float link_get_voltage();               // 送信側（機体）のバッテリー電圧
  bool  link_get_attitude(float &roll, float &pitch);
  bool  link_get_yaw(float &yaw, float &acc95);
  bool  link_get_pitch_avg(float &avg);
  bool  link_get_roll_trim(float &trim);
  bool  link_get_wind(float &speed_mps, float &dir_to_deg);

  // ---- 無線モジュールの状態（設定画面・詳細画面用）----
  // 設定を書いて読み返した照合が通っているか。通らなければモジュール不良か配線不良。
  bool     link_module_alive();
  uint16_t link_rx_count();
  uint16_t link_miss_count();     // seq の飛びの累計＝電波側で落ちた数
  uint16_t link_tx_count();
  uint32_t link_since_tx_ms();    // 直近の送出からの経過[ms]。未送信は 0xFFFFFFFF
  // ★ 通算平均は持たない。序盤の至近距離のパケットに引きずられて、
  //   飛行中の実力とかけ離れた数字が出続けるため（60 秒ごとのログに
  //   その窓の min..max が残るので、推移はそちらで追える）。
  int8_t   link_rssi_avg10();     // 直近 10 秒の平均。0 = 窓に 1 発も無い
  // 表示用の 0〜3 本。**限界からの余裕**で切っているので SF を変えても意味が保たれる
  // （絶対値のしきい値だと SF ごとに手で直すことになる）。実装は link.cpp。
  uint8_t  link_rssi_bars();
  // 環境雑音 [dBm]。0 = 取得できていない。無線設定画面から 1Hz で呼ぶ想定
  // （モジュールへの問い合わせなので飛行中の描画パスからは呼ばないこと）。
  int16_t  link_noise_dbm();
  // 送信機が 2 台いる異常状態。**受信モードでしか true にならない**
  // （送信機は送信の合間 mode 3 で寝ていて受信できない）。
  // 検出は seq の逆行で行う。1 台なら seq は必ず増えるので、
  // 減る・止まるのは 2 台いる証拠になる。
  bool     link_dup_sender();
  // 別グループの送信機を直近に受けたか。**「無信号」とは別物**として出すこと。
  // これが真なら、電波は届いていてグループ ID だけが食い違っている。
  bool     link_other_group_seen();
  uint8_t  link_other_group_id();
  // 送信機の健康状態（LINK_ST_* のビット）。受信側からは知りようがない項目だけ。
  uint16_t link_sender_status();
  // 目的地／ナビモードが送信側と食い違っているか。
  // 食い違うとコース警告だけが静かにずれるので、明示的に出す。
  bool     link_dest_mismatch();

  // ============================================================
  //  送信前チェック（プリフライト）
  // ============================================================
  // 送信モードに入った直後だけ、LINK_PF_WINDOW_MS のあいだ mode 0 で**聴く**。
  // 通常運転の送信機は送信の合間 mode 3 で寝ていて何も聴かないので、
  // 「選んだチャンネルが使えるか」を確かめられるのはここだけになる。
  //
  // ★ 問題が見つかっても**送信は止めない。** 予備機を緊急で TX に切り替えて
  //   載せ替える場面で送信が始まらないほうが危険なため。E220 は ARIB の
  //   キャリアセンスが必須実装なので、混雑時はモジュール側が自動で送信を待つ。
  //   ここは「気づかせる」だけの仕組みで、飛行を止める仕組みではない。
  typedef enum {
      LINK_PF_IDLE = 0,   // 対象外（OFF / RX）または未実行
      LINK_PF_RUNNING,    // 聴取中。この間は送信しない
      LINK_PF_DONE,       // 完了。結果は link_preflight_flags() を見る
  } LinkPreflightState;

  #define LINK_PFF_OK          0x00
  #define LINK_PFF_NOISY       0x01   // 定常雑音が高い（中央値が閾値超え）
  #define LINK_PFF_BURST       0x02   // 断続的な送信がある。別 SF / 他方式でデコードできない
  #define LINK_PFF_PONS_SAME   0x04   // ★ 同じグループの PONS がいる ＝ 送信機が 2 台
  #define LINK_PFF_PONS_OTHER  0x08   // 別グループの PONS が同じ CH/SF にいる
  #define LINK_PFF_SKIPPED     0x10   // モジュール無応答で実行できなかった

  LinkPreflightState link_preflight_state();
  uint8_t  link_preflight_flags();       // LINK_PFF_* のビット。DONE 以外では 0
  uint32_t link_preflight_remain_ms();   // 監視の残り時間。RUNNING 以外では 0
  int16_t  link_preflight_noise_med();   // 監視中に測った雑音の中央値 [dBm]。0 = 未取得
  uint8_t  link_preflight_other_group(); // LINK_PFF_PONS_OTHER のときのグループ ID
  // 見つけた他機の RSSI [dBm]。0 = 見つからなかった。
  // **近いのか遠いのかで対処が変わる**ので出す（隣で出ているのか、遠くの誰かか）。
  int8_t   link_preflight_pons_rssi();
  // 判定が出てからの経過 [ms]。地図の警告ポップアップを引っ込める判断に使う。
  // DONE 以外では 0xFFFFFFFF（＝「出す時間はとうに過ぎている」）。
  uint32_t link_preflight_since_done_ms();

#endif // LINK_H
