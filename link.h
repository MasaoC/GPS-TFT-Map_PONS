// ============================================================
// File    : link.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : PONS Link（機体⇄ボート無線）の RP2350 側インターフェース。
//           UART1 で無線モジュール(E220-900T22S)と話し、送信テレメトリの組み立てと
//           受信テレメトリの供給を行う。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/27
// ============================================================
//
// 受信データの供給関数は、意図的に gnss.h のリプレイ用関数
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
  void link_announce_mode();  // 無線モードを読み上げる。**setup() の末尾で呼ぶ**
  // settings.txt の CH が選べる CH（0/3/6/9/12・アッパーの 15/21/27）でなかったので寄せた、と記録する。
  // 設定の読み込み（mysd.cpp）から呼ぶ。知らせて保存し直すのは link_announce_mode()。
  void link_note_radio_ch_snapped(uint8_t from);
  void link_loop();           // Core0 のループから毎回呼ぶ（軽い。1Hz・32B。アッパーの SF9 以上は間隔が長い）

  // ---- 設定 ----
  // 設定画面から変更されたときに呼ぶ。無線モジュールへ即座に反映される。
  // ★ 電波の設定は「数値」でしかない。意味を決めるのは無線層（e220.cpp）。
  //   LoRa では radio_ch = E220 チャネル、radio_profile = SF プリセット。
  void link_set_mode(uint8_t mode);          // LINK_MODE_OFF / TX / RX
  void link_set_radio_ch(uint8_t ch);        // 0/3/6/9/12・15/21/27 のどれか。それ以外は無視（link_proto.h）
  void link_set_radio_profile(uint8_t p);    // 0..LINK_PROFILE_COUNT-1（アッパーでは SF10 まで）

  // ---- アッパーバンド（CH15 / 21 / 27。docs/pons_link.md §2）----
  // 電波はどの CH でも 32 バイトのフレーム（lora_link/link_wire2.h）。アッパーでは休止に合わせた間隔で送る。
  // 受信機の「無信号」などの時間は、送る間隔に比例して伸ばす（「何発落ちたら」をローワーとそろえる）。
  bool     link_radio_profile_allowed(uint8_t ch, uint8_t prof);  // アッパーでは SF11 を使わない
  uint8_t  link_next_radio_profile(uint8_t prof);  // WIRELESS 画面で長押しのたびに回す次の SF
  uint32_t link_tx_period_ms();              // 送る間隔 [ms]。ローワーは 1000、アッパーは休止の分だけ長い
  uint32_t link_nosignal_ms();               // 無信号とみなすまで [ms]（10 秒 × 間隔 / 1 秒）
  uint32_t link_stale_ms();                  // 受信枠を点滅させ始めるまで [ms]（LINK_FRAME_STALE_MS × 間隔 / 1 秒）
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

  // ---- リプレイ中／デモ中の扱い（docs/pons_link.md §5）----
  // ★★ **リプレイ中とデモ中は無線を「OFF 扱い」にする。** 画面の数字が電波と
  //   無関係なので、混ぜると**そのとき送信機／受信機だったかのような画面**になる。
  //   基準はミラー中と同じ「その表示は、いま画面に出している数字の話か」。
  // ★★ **`link_mode_setting` は書き換えないこと。** SD に保存される設定なので、
  //   デモのために OFF を代入すると**翌日そのまま無線が黙って飛ぶ**。
  //   抑止は振る舞いだけで、理由は WIRELESS 画面の "Hold" 行に出す。
  // ★ 条件を呼び出し側へ書き写さないこと。0.984 までは `getReplayMode()` が
  //   `link_tx_tick()` と `draw_link_icons()` の 2 か所に書かれていて、片方だけ
  //   直せば「送っていないのにアイコンだけ生きている」が起きる形だった。
  // ★ 2 つに分けてあるが条件はいま同じ（実体も 1 つ）。「電波を出すか」と
  //   「画面に出すか」は別の判断で、片方だけ変わる日が来る。
  bool link_rf_suppressed();   // 電波を出してはいけない場面か（送信の抑止）
  bool link_live_ui_ok();      // 無線の「いまの状態」を画面に出してよいか

  // 設定ファイル(mysd.cpp)から直接読み書きするための実体
  extern uint8_t link_mode_setting;
  extern uint8_t link_radio_ch;
  extern uint8_t link_radio_profile;
  extern uint8_t link_group;

  // ---- 送信側（機体）----
  // ★ テレメトリの組み立てと送出は link.cpp の内部で完結している。
  //   刻み（1Hz）も link_tx_tick() が持つので、外から呼ぶ関数は無い。
  //   GNSS 更新（2Hz）に合わせて呼ぶと電波の占有時間と送信電流が倍になるため、
  //   **意図的に公開していない**。

  // ---- 受信側（ボート／ピット）----
  // MIRROR = 電波が来ている / NO_SIGNAL = 10秒以上来ていない
  typedef enum { LINK_DISP_MIRROR = 0, LINK_DISP_NOSIGNAL = 1 } LinkDispState;

  // ミラー中か。各センサ accessor がこれを見て値を差し替える。
  bool link_mirror_active();

  LinkDispState link_display_state();     // 表示状態（受信モード以外では NOSIGNAL）
  bool     link_is_receiving();           // 直近に受信できているか
  // 新しいテレメトリが届いたか。**読むと落ちる**（ミラー中の位置更新の合図）
  bool     link_take_new_frame();
  // 同じ合図の描画用（地図の描き直しをフレームを受けた直後にそろえる。読むと落ちる）
  bool     link_take_draw_frame();
  // ミラー中に TFT を描いてよい時間か（受信中のフレームに描画を重ねない）。ミラー中以外は true
  bool     link_draw_window_ok();
  uint32_t link_age_ms();                 // 最後の受信からの経過 [ms]
  uint32_t link_rx_interval_ms();         // 直近の受信間隔 [ms]（最後の受信の1個前からの経過時間。未計測は0）
  int8_t   link_rssi();                   // 直近の RSSI [dBm]

  // ---- 受信テレメトリの取り出し ----
  // ★ **LinkTelem の詰め方（スケール）を知ってよいのは link.cpp だけ。**
  //   受信した LinkTelem をそのまま外へ渡して kf_alt_dm / 10.0f のように書かせると、
  //   同じ換算が 2 か所に存在することになり、分解能を変えたとき片方だけ直して
  //   「ミラー表示のときだけ桁がずれる」という静かな壊れ方をする。
  //   受信側の値が要るときは必ずこの getter を通すこと。
  //   だから生の LinkTelem を返す関数は置かない（使われていなかった link_rx_telem() は
  //   2026-10-23 に消した）。
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
  int8_t   link_rssi_avg10();
  // 受信中の電波が限界からどれだけ上にいるか [dB]。本数と同じ根拠から出している。
  // 受信できていなければ 0。表示の桁が崩れないよう ±99 に丸めてある。
  int8_t   link_rssi_margin_db();     // 直近 10 秒の平均。0 = 窓に 1 発も無い
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
  bool     link_dest_missing();       // 送信機の目的地が受信機に無い（ID が一致するものが無い）
  uint16_t link_dest_id(int idx);     // 目的地の ID（名前と座標の CRC16。0 = 未選択）

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

  // WIRELESS 画面から送信前チェックを手動で走らせ直す。
  // TX モードでないときは何もせず false を返す。
  bool link_preflight_restart();

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

  // ---- チャンネル調査（WIRELESS 画面の Ch scan → MODE_CHSCAN）----
  // 選べるチャンネル（link_proto.h の LINK_RADIO_CHS。ローワーの CH0 / 3 / 6 / 9 / 12 と、アッパーの
  // CH15 / 21 / 27）を順に聴いて、雑音と他の PONS を一覧にする。
  // 1 チャンネル LINK_SCAN_DWELL_MS ずつ LINK_SCAN_SWEEPS 周して、結果を積み上げる。
  // ★ **調べている間は送信も受信もしない。** モジュールのチャンネルを動かすため。
  //   終わったら（途中で止めても）元のチャンネルと電源状態に戻す。
  // ★ チャンネルは自動では変えない。3 台そろえる必要があるので、選ぶのは人
  //   （WIRELESS 画面の Ch。戻るときに link_warn_setting_changed() が知らせる）。
  typedef enum {
      LINK_SCAN_IDLE = 0,
      LINK_SCAN_RUNNING,  // 調べている。送信・受信は止まっている
      LINK_SCAN_DONE,     // 予定の周回を終えた
      LINK_SCAN_STOPPED,  // 途中で止めた（取れた分は残る）
  } LinkScanState;

  struct LinkScanRow {
      uint8_t  ch;
      uint8_t  n;            // 雑音のサンプル数（0 = まだ／測れなかった）
      int16_t  med, max;     // 雑音の中央値・最大値 [dBm]（n = 0 なら 0）
      uint16_t pons_same;    // 解読できた PONS のフレーム数（自分と同じ Group）
      uint16_t pons_other;   // 同じく別の Group（他チーム）
      int8_t   pons_rssi;    // 解読できた PONS の最大 RSSI [dBm]。0 = 無し
      bool     burst;        // 断続的な電波（最大値 − 中央値 > LINK_PF_BURST_DB）
      bool     measuring;    // いま測っているチャンネル
      bool     upper;        // アッパーバンド（CH13〜）
      bool     usable;       // いまの SF で選べる（アッパーでは SF11 が使えない）
  };
  #define LINK_SCAN_LOWER_COUNT LINK_LOWER_CH_COUNT   // link_proto.h。CH0 / 3 / 6 / 9 / 12
  #define LINK_SCAN_CH_COUNT    LINK_ALL_CH_COUNT     // ローワー 5 つ ＋ アッパー 3 つ

  bool          link_chscan_start();      // モジュールが応答しないなら false（何もしない）
  void          link_chscan_stop();       // 途中で止める。元の設定に戻す。走っていなければ何もしない
  bool          link_chscan_running();
  LinkScanState link_chscan_state();
  uint8_t       link_chscan_sweep();      // いま（終わったなら最後に）何周目か。1 始まり
  bool          link_chscan_row(uint8_t idx, LinkScanRow* out);  // idx 0..LINK_SCAN_CH_COUNT-1（小さい CH から）
  // 一番静かなチャンネルの idx。他チームの PONS がいるものと、自チーム以外の断続的な電波があるもの、
  // いまの SF で使えないもの（アッパーで SF11）は除いて、雑音の中央値が一番低いもの。候補が無ければ -1。
  int8_t        link_chscan_best();                 // ローワーとアッパーの両方から
  int8_t        link_chscan_best_band(bool upper);  // 片方の帯域だけから（一覧で帯域ごとに best を付ける）

#endif // LINK_H
