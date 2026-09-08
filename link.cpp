// ============================================================
// File    : link.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : PONS Link（機体⇄ボート無線）の RP2350 側実装。
//           UART1 のフレーミング、送信テレメトリの組み立て、
//           受信テレメトリの保持と供給。
// Author  : MasaoC (@masao_mobile)
// ============================================================
//
// ■ 設計の要点
//   ・**送信の write() はあえてブロックさせる。** 隙間なく 1 回で流し込まないと
//     E220 が途中でパケットを切ってしまうため（docs/pons_link.md §4）。
//     115200bps・65 バイトで 3.1ms なので、1Hz なら Core0 への影響は無視できる。
//     長く止まる可能性があるのは「モジュールがまだ前の送信を処理中」のときだけで、
//     そこは e220_send() が AUX を見て**そのスロットを捨てる**ことで避けている。
//   ・受信データの供給関数は gps.cpp のリプレイ用と同じ形にしてある。
//     受信モードを既存のリプレイ経路へ相乗りさせるため。
//
#include "link.h"
#include "settings.h"
#include "gps.h"
#include "imu.h"
#include "attitude.h"
#include "airdata.h"
#include "display_tft.h"
#include "mysd.h"
#include "navdata.h"
#include "e220.h"        // 下位ドライバ（UART・モード制御・レジスタ）

// ★ UART とモジュール制御は e220.cpp が持つ。ここは「何を送るか」だけを扱う。
//   AUX(GPIO30) は e220.h の E220_AUX_PIN。送信の刻みは RP2350 が持つ。

// 受信が途切れてから自機表示へ移るまで（docs/pons_link.md §5）。
// ★ RSSI10_WIN_MS と同じ値にしてあること。受信中なら 10 秒窓に必ず 1 発は入る、
//   という前提で link_rssi_bars() が成り立っている。
#define NOSIGNAL_MS      10000

// ---- 設定（SD の設定ファイルから読まれる）----
// ★ 電波の設定は「数値」でしかない。意味を決めるのは無線層（e220.cpp）で、
//   radio_ch = E220 のチャネル、radio_profile = SF のプリセットに対応する。
uint8_t link_mode_setting    = LINK_MODE_OFF;   // 設定ファイルから読むので外部リンケージ
uint8_t link_radio_ch        = 6;
uint8_t link_radio_profile   = 1;   // 既定 = SF8

// ---- 受信状態 ----
static LinkRxTelem   s_rx;                 // 直近に受信したテレメトリ
static bool          s_rxValid   = false;
static uint32_t      s_rxMs      = 0;      // 受信した時刻（RP の millis）
// ---- 送信側 ----
static uint16_t s_seqOut    = 0;    // 送信テレメトリの連番。ロールオーバーしてよい

// ---- 集計（無線モジュールではなく RP2350 が数える）----
static uint32_t s_txCount   = 0;    // 送信回数
static uint16_t s_missTotal = 0;    // seq の飛びの累計＝電波側で落ちた数
static int32_t  s_rssiSum   = 0;    // RSSI 平均用
static uint32_t s_rssiN     = 0;
static bool     s_dupSender = false;
static uint16_t s_seqBack   = 0;    // seq の逆行回数＝送信機が複数いる証拠
static uint32_t s_seqBackMs = 0;    // 直近に逆行を見た時刻
#define SEQBACK_WINDOW_MS  10000    // この間隔が空いたら数え直す（再起動と区別）

// ============================================================
//  警告の鳴らし方
//    SD が無い機体でも必ず何か鳴るようにする。
//      SD あり → 音声（WAV）。ファイルが開けなければ sound.cpp 側の
//                共通フォールバック（500Hz×2）が鳴るので無音にはならない。
//      SD なし → その警告ごとに決めたトーン。音で種類を区別できる。
//    優先度 3 = コース警告と同等。設定ミスや機器異常は飛ぶ前に気づく必要がある。
//    最低音量 60 = 飛行中でも聞こえる大きさを保証する。
// ============================================================
//   wav に nullptr を渡すと常にトーンだけを鳴らす（音声にする必要が無い警告用）。
static void link_alert(const char* wav, int freq, int dur, int count) {
    if (wav && good_sd()) enqueueTask(createPlayWavTask(wav, 3, 60));
    else                  enqueueTask(createPlayMultiToneTask(freq, dur, count, 3, 60));
}

// ---- 通信の統計（60 秒ごとにシステムログへ残す）----
static uint32_t s_rxTotal    = 0;   // 受信できたフレーム数
static int8_t   s_rssiMin    = 0, s_rssiMax = 0;
static bool     s_rssiSeen   = false;
static uint16_t s_seqGaps    = 0;   // seq の飛び（電波側の取りこぼし）
static uint16_t s_lastSeq    = 0;
static bool     s_haveLastSeq = false;

static void link_push_telemetry();   // 実体は下（送信側の組み立て）

// ============================================================
//  受信の取り込み
//    e220.cpp が magic('P','L') と CRC まで確かめた 1 パケットを返す。
//    間に中継 MCU がいないので、UART フレーミング（sync + CRC）は不要。
// ============================================================
static void link_poll_radio() {
    if (link_mode_setting != LINK_MODE_RX) return;

    LinkTelem t;
    int16_t   rssi = 0;
    while (e220_recv((uint8_t*)&t, (uint8_t)sizeof(t), &rssi)) {
        s_rx.t      = t;
        s_rx.rssi   = (int8_t)constrain((int)rssi, -128, 127);
        s_rx.age_ms = 0;             // 直接受け取るので遅延はほぼ無い
        s_rxValid   = true;
        s_rxMs      = millis();

        // 受信ログ（docs/pons_link.md §5）。received/ は無線で得た情報だけを置く。
        enqueueTask(createSaveRxCsvTask(&s_rx.t, s_rx.rssi, s_rx.age_ms));

        // ---- 集計 ----
        s_rxTotal++;
        if (!s_rssiSeen) { s_rssiMin = s_rssiMax = s_rx.rssi; s_rssiSeen = true; }
        else {
            if (s_rx.rssi < s_rssiMin) s_rssiMin = s_rx.rssi;
            if (s_rx.rssi > s_rssiMax) s_rssiMax = s_rx.rssi;
        }
        s_rssiSum += s_rx.rssi; s_rssiN++;

        // ---- seq の解析 ----
        //   送信機が 1 台なら seq は必ず増える。減る・止まるのは異常。
        if (s_haveLastSeq) {
            const uint16_t d = (uint16_t)(t.seq - s_lastSeq);
            if (d > 1 && d < 1000) {
                // 順方向の飛び＝電波側で落ちた数
                s_seqGaps += (d - 1); s_missTotal += (d - 1);
            } else if (d == 0 || d > 60000) {
                // ★ 逆行または同値。1 台の送信機では起きない。
                //   送信機が 2 台いると互いの seq が交互に届くのでほぼ毎回来る。
                //   一方、送信機の再起動でも 1 回だけ起きる。
                //   **短時間に固まって起きたときだけ**「2 台いる」と判定する。
                //   間隔が空いていれば再起動とみなしてカウントをやり直す。
                const uint32_t now = millis();
                if (s_seqBack && (now - s_seqBackMs) > SEQBACK_WINDOW_MS) s_seqBack = 0;
                s_seqBackMs = now;
                s_seqBack++;
            }
        }
        s_lastSeq = t.seq; s_haveLastSeq = true;

        // ---- 送信機の重複検出 ----
        //   送信機が 1 台なら seq は必ず増えるので、逆行は 2 台いる証拠になる。
        //   送信元 ID を載せて比較する案は却下した。既定値が 3 台とも同じに
        //   なるため、「設定を触っていない 2 台がどちらも TX」という
        //   一番ありそうな誤設定で黙って失敗するため。
        if (!s_dupSender && s_seqBack >= 3) {
            s_dupSender = true;
            link_alert("wav/link_dup_sender.wav", 294, 400, 3);
            enqueueTask(createLogSdfTask(
                "LINK ERROR: another sender detected (seq went backwards %u times)",
                (unsigned)s_seqBack));
        }
        // ★ 制約：この検出は**受信機でしか働かない**。送信機は送信の合間
        //   mode 3 で寝ていて受信できないため。2 台とも TX に誤設定した場合、
        //   受信機から見れば「送信機が 2 台いる」として現れる。
    }
}

// ============================================================
//  初期化・周期処理
// ============================================================
void link_setup() {
    // UART・モードピン・レジスタ設定はすべて e220.cpp が面倒を見る。
    // 設定は書いたあと読み返して照合されるので、通らなければ
    // e220_alive() が false になり link_watch_module() が鳴らす。
    e220_setup(link_radio_ch, link_radio_profile);

    // 受信機は常時 mode 0（docs/pons_link.md §3）。
    // 送信機と OFF は最初から寝かせておく（e220_setup は mode 0 で戻ってくる）。
    if (link_mode_setting != LINK_MODE_RX) e220_sleep();

    // ★ 起動時に送信/受信の**どちらも**読み上げる（docs/pons_link.md §1）。
    //   受信側だけ読み上げる案だと「何も言わない＝正常」と受け取られ、
    //   無線が OFF になっていることに気づけない。両方読み上げれば
    //   **無音が「無線 OFF」を意味する**ようになり、3 状態が音だけで判別できる。
    if (link_mode_setting == LINK_MODE_TX)
        enqueueTask(createPlayWavTask("wav/sender_mode.wav", 2));
    else if (link_mode_setting == LINK_MODE_RX)
        enqueueTask(createPlayWavTask("wav/receiver_mode.wav", 2));
}

// ============================================================
//  link_loop() の中身
//    1 つの関数に詰め込むと何をしているのか読めなくなるので、
//    「何を見張っているか」ごとに分ける。どれも状態遷移の検出なので、
//    前回値を static で持って立ち上がりだけを拾う形に揃えてある。
// ============================================================

// MIRROR ⇄ NO SIGNAL の遷移を知らせる（docs/pons_link.md §5 / 7.10）。
// この瞬間、画面上の数値の意味が「機体」と「自機」の間で入れ替わる。
// 誤読の起点になるので、画面を見ていなくても気づけるようにする。
static void link_watch_mirror_transition() {
    if (link_mode_setting != LINK_MODE_RX) return;
    static bool was_mirror = false;
    static bool primed     = false;   // 起動直後の 1 回目は鳴らさない
    const bool now_mirror = link_is_receiving();
    if (primed && now_mirror != was_mirror) {
        if (now_mirror) {
            // 復帰は頻繁に起きうるので短い音で足りる（低→高）。
            enqueueTask(createPlayMultiToneTask(880, 90, 1, 2));
            enqueueTask(createPlayMultiToneTask(1318, 120, 1, 2));
        } else {
            // 失ったほうは表示の意味が変わるので、言葉で知らせる。
            link_alert("wav/link_no_signal.wav", 1318, 250, 3);
        }
        enqueueTask(createLogSdfTask("LINK %s", now_mirror ? "acquired" : "lost"));
    }
    was_mirror = now_mirror;
    primed = true;
}

// 無線モジュールが応答しない＝設定を書いて読み返した照合が通らない。
// 配線・電源・モジュール不良のいずれかなので、人が気づくしかない。
static void link_watch_module() {
    if (link_mode_setting == LINK_MODE_OFF) return;
    static bool was_alive = false, alerted = false;
    const bool alive = link_module_alive();
    if (alive) { was_alive = true; alerted = false; return; }
    if (alerted) return;
    // ★ 「一度も応答していない」場合も鳴らす。
    //   立ち上がりだけを見ていると、最初から死んでいるモジュールを
    //   永久に見逃す（飛ぶ前に気づきたいのはむしろこちら）。
    if (was_alive || millis() > 15000) {
        alerted = true;
        link_alert("wav/link_no_module.wav", 220, 300, 3);
        enqueueTask(createLogSdfTask("LINK ERROR: radio module not responding"));
    }
}

// 目的地／ナビモードを送信側に合わせる（docs/pons_link.md §5）。
// 受信機の目的は「パイロットに出ている画面をそのまま再現すること」なので、
// 食い違ったまま表示しても意味が無い（コース警告だけが静かにずれる）。
// 勝手に設定が変わるのは驚きなので、必ず知らせる。RAM 上だけの上書き。
static void link_sync_destination() {
    if (!link_mirror_active() || !link_dest_mismatch()) return;
    const int8_t di = s_rx.t.dest_index;
    if (di < 0 || di >= destinations_count) return;   // 送信側が未選択なら表示で知らせるに留める
    currentdestination = di;
    destination_mode   = s_rx.t.nav_mode;
    link_alert("wav/link_override_dest.wav", 494, 200, 3);
    enqueueTask(createLogSdfTask("LINK: dest/nav overridden to sender (%d/%d)",
                                 (int)di, (int)s_rx.t.nav_mode));
}

// 送信機側の異常（SD／IMU／較正未適用／電池低下）。飛ぶ前に気づきたい項目。
// 音声にはしない ― 内訳は無線ページに出るので「何かある」と分かれば足りる。
// 音声を増やすほど、本当に聞くべきものが埋もれる。
static void link_watch_sender_faults() {
    static uint16_t prev_st = 0;
    const uint16_t st = link_sender_status();
    if ((st & ~prev_st) != 0) {
        link_alert(nullptr, 370, 250, 2);
        enqueueTask(createLogSdfTask("LINK WARN: sender fault bits 0x%04X", st));
    }
    prev_st = st;
}

// 通信レポート（60 秒ごと）。現場でシリアルを見られない以上、
// SD のシステムログが「あのとき電波はどうだったか」を振り返る唯一の記録になる。
// 出すのは「モジュールが応答したか」「何発受かったか」「seq がいくつ飛んだか」
// 「RSSI の幅」の 4 つ。あとから原因を切り分けられる粒度にしてある。
static void link_periodic_report() {
    if (link_mode_setting == LINK_MODE_OFF) return;
    static uint32_t lastReport = 0;
    if (millis() - lastReport < 60000) return;
    lastReport = millis();

    const char* mod = link_module_alive() ? "OK" : "NORESP";
    if (link_mode_setting == LINK_MODE_TX) {
        enqueueTask(createLogSdfTask(
            "LINK TX: mod=%s ch=%u prof=%u sent=%u%s",
            mod, link_radio_ch, link_radio_profile,
            (unsigned)s_txCount, s_dupSender ? " DUP_SENDER" : ""));
    } else if (s_rssiSeen) {
        enqueueTask(createLogSdfTask(
            "LINK RX: mod=%s ch=%u prof=%u rx=%lu miss=%u gaps=%u "
            "rssi=%d..%d avg=%d%s",
            mod, link_radio_ch, link_radio_profile,
            (unsigned long)s_rxTotal, (unsigned)s_missTotal,
            (unsigned)s_seqGaps, s_rssiMin, s_rssiMax,
            (int)link_rssi_avg(), s_dupSender ? " DUP_SENDER" : ""));
    } else {
        enqueueTask(createLogSdfTask(
            "LINK RX: mod=%s ch=%u prof=%u no reception yet",
            mod, link_radio_ch, link_radio_profile));
    }
    // RSSI の幅は「この 60 秒でどうだったか」を見たいので毎回入れ直す。
    // 受信数・取りこぼし・CRC エラーは累計のまま（推移が読めるほうが役に立つ）。
    s_rssiSeen = false;
    s_seqGaps  = 0;
}

// 送信機の刻み（1Hz）。**時計を持つのは RP2350 側**。
// E220 は自分から催促してこないので、こちらが叩きに行く。
//   ・送信直前にテレメトリを組み立てるので、値は常に新しい
//   ・送り終えたら即 mode 3 へ落とす（高速スリープ。AUX を待たなくてよい）
static void link_tx_tick() {
    if (link_mode_setting != LINK_MODE_TX) return;
    static uint32_t lastTx = 0;
    if (millis() - lastTx < 1000) return;
    lastTx = millis();

    e220_wake();
    link_push_telemetry();      // 中で e220_send() する
    e220_sleep();               // 送信完了を待たずに寝かせてよい（データシート 5.3）
}

void link_loop() {
    link_poll_radio();
    link_watch_mirror_transition();
    link_watch_module();
    link_sync_destination();
    link_watch_sender_faults();
    link_periodic_report();
    link_tx_tick();
}

// ============================================================
//  設定
// ============================================================
void link_set_mode(uint8_t mode) {
    if (mode > LINK_MODE_RX) mode = LINK_MODE_OFF;
    if (mode == link_mode_setting) return;
    link_mode_setting = mode;
    s_rxValid = false;                 // モードが変わったら受信データは無効
    // 送信機は寝かせ、受信機は起こす（受信機は常時 mode 0：docs/pons_link.md §3）
    if (link_mode_setting == LINK_MODE_TX) e220_sleep();
    else                                   e220_wake();
}
// 値の範囲は E220 の制約（CH0-12 / SF7-11）で決まる。
// チャンネル/プロファイルの変更はモジュールへ書き直しが要る。
// mode 3 へ入って書き、読み返して照合し、mode 0 へ戻る（e220.cpp）。
// ★ この間 Core0 が最大 0.6 秒ほど止まる（9600 での往復とタイムアウト）。
//   設定画面はメニューからしか入れないので飛行中には起きないが、
//   **飛行中に呼んではいけない**関数である点は意識しておくこと。
// ★★ 書込みは **RAM のみ（0xC2）**。設定画面で CH を 0→12 と回すだけで
//   13 回になるので、不揮発（0xC0）で書くとモジュールの書換え寿命を無駄に削る。
//   設定の正本は SD の settings.txt 側なので、次回起動時に e220_setup() が
//   同じ値を書き直す（そのときだけ不揮発）。
void link_set_radio_ch(uint8_t ch) {
    if (ch > E220_CH_MAX || ch == link_radio_ch) return;
    link_radio_ch = ch;
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
}
void link_set_radio_profile(uint8_t p) {
    if (p >= LINK_PROFILE_COUNT || p == link_radio_profile) return;
    link_radio_profile = p;
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
}
uint8_t link_get_mode()          { return link_mode_setting; }
uint8_t link_get_radio_ch()      { return link_radio_ch; }
uint8_t link_get_radio_profile() { return link_radio_profile; }

// ============================================================
//  送信側：テレメトリの組み立て
//    刻みは link_tx_tick() が持つ（1Hz）。ここは呼ばれたら最新値で 1 発作る。
// ============================================================
static void link_push_telemetry() {
    if (link_mode_setting != LINK_MODE_TX) return;

    LinkTelem t = {};
    t.magic0 = 'P'; t.magic1 = 'L';
    t.ver = LINK_PROTO_VER; t.type = LINK_TYPE_TELEM;
    // ★ seq は送信のたびに 1 つ進める。**埋め忘れると全パケットが seq=0 になり、
    //   受信側の取りこぼし計数と重複送信機の検出が両方壊れる。**
    t.seq   = s_seqOut++;
    t.tx_us = micros();

    uint16_t have = 0;

    GpsDate d = get_gpsdate();
    GpsTime tm = get_gpstime();
    if (d.isValid()) { t.year = d.year(); t.month = d.month(); t.day = d.day(); have |= RHAVE_DATE; }
    if (tm.isValid()) { t.hour = tm.hour(); t.minute = tm.minute();
                        t.second = tm.second(); t.centi = tm.centisecond(); }

    t.lat_1e7 = (int32_t)lround(get_gps_lat() * 1e7);
    t.lon_1e7 = (int32_t)lround(get_gps_lon() * 1e7);
    t.gs_cms      = (uint16_t)constrain(lround(get_gps_mps() * 100.0), 0L, 65535L);
    t.track_cdeg  = (uint16_t)constrain(lround(get_gps_truetrack() * 100.0), 0L, 36000L);
    have |= RHAVE_GS | RHAVE_TTRACK;

    t.gnss_alt_dm = (int16_t)constrain(lround(get_gps_altitude() * 10.0), -32768L, 32767L);
    have |= RHAVE_GNSSALT;

    t.kf_alt_dm = (int16_t)constrain(lround(get_imu_altitude_msl() * 10.0f), -32768L, 32767L);
    t.kf_vs_cms = (int16_t)constrain(lround(get_imu_vspeed() * 100.0f), -32768L, 32767L);
    have |= RHAVE_KFALT | RHAVE_KFVS;

    t.press_dpa = (uint16_t)constrain(lround(get_airdata_pressure() * 10.0f), 0L, 65535L);
    have |= RHAVE_PRESS;

    float roll, pitch, yaw;
    attitude_get_euler(roll, pitch, yaw);
    if (attitude_ready()) {
        t.roll_cdeg  = (int16_t)constrain(lround(roll  * 100.0f), -32768L, 32767L);
        t.pitch_cdeg = (int16_t)constrain(lround(pitch * 100.0f), -32768L, 32767L);
        t.yaw_cdeg   = (int16_t)constrain(lround(yaw   * 100.0f), -32768L, 32767L);
        have |= RHAVE_ATT;
        float acc95 = attitude_get_yaw_acc95_deg();
        t.yaw_acc95_deg = (uint8_t)constrain(lround(acc95), 0L, 255L);
        have |= RHAVE_ATT_YAW;
    }
    t.roll_trim_cdeg = (int16_t)constrain(lround(attitude_get_roll_trim_deg() * 100.0f),
                                          -32768L, 32767L);
    have |= RHAVE_ATT_TRIM;

    if (attitude_pitch_avg_valid()) {
        t.pitch_avg_cdeg = (int16_t)constrain(lround(attitude_get_pitch_avg_deg() * 100.0f),
                                              -32768L, 32767L);
        have |= RHAVE_ATT_AVG;
    }
    float wspd, wdir;
    if (attitude_get_wind(wspd, wdir)) {
        t.wind_dmps     = (uint8_t)constrain(lround(wspd * 10.0f), 0L, 255L);
        t.wind_dir_cdeg = (uint16_t)constrain(lround(wdir * 100.0f), 0L, 36000L);
        have |= RHAVE_ATT_WIND;
    }

    t.volt_cv = link_pack_volt(get_input_voltage());
    have |= RHAVE_VOLT;

    t.numsat  = (uint8_t)constrain(get_gps_numsat(), 0, 255);
    have |= RHAVE_NUMSAT;
    t.hacc_dm = (uint16_t)constrain(get_gps_hacc_mm() / 100, 0UL, 65535UL);  // mm → 0.1m
    t.fixflags = get_gps_gnssFixOK() ? 0x01 : 0x00;

    // ナビ設定。受信側はこれを自分の設定と突き合わせるためだけに使う。
    t.dest_index = (int8_t)currentdestination;
    t.nav_mode   = (uint8_t)destination_mode;

    t.have = have;

    // status は「受信側からは知りようがない送信機自身の健康状態」だけ。
    // 飛行の警告は受信側がミラー値から再計算するので載せない（link_proto.h 参照）。
    uint16_t st = 0;
    if (!good_sd())                                    st |= LINK_ST_SD_ERROR;
    if (!get_imu_ok())                                 st |= LINK_ST_IMU_ERROR;
    if (attitude_needs_apply() && attitude_get_rpy_enabled()) st |= LINK_ST_NEEDS_APPLY;
    if (get_input_voltage() <= BAT_LOW_VOLTAGE)        st |= LINK_ST_BAT_LOW;
    t.status = st;

    t.crc16 = link_telem_crc(&t);
    // ★ ヘッダとペイロードを 1 回の write() にまとめるのは e220_send() の責任。
    //   分割して書くと区切り時間を超えてパケットが分裂する（docs/pons_link.md §4）。
    if (e220_send((const uint8_t*)&t, (uint8_t)sizeof(t))) s_txCount++;
}

// ============================================================
//  受信側：状態
// ============================================================
uint32_t link_age_ms()      { return s_rxValid ? (millis() - s_rxMs) : 0xFFFFFFFF; }
bool     link_is_receiving(){ return s_rxValid && link_age_ms() < NOSIGNAL_MS; }
int8_t   link_rssi()        { return s_rxValid ? s_rx.rssi : 0; }
const LinkTelem* link_rx_telem() { return s_rxValid ? &s_rx.t : nullptr; }

// ★ ミラー中か。各センサ accessor はこれを見て、自機の値の代わりに
//   受信した機体の値を返す（gps.cpp の is_demo_active() と同じ流儀）。
//
//   accessor で差し込む方式にした理由:
//     ・グローバル（stored_lat 等）を汚さないので、**自機のフライト CSV は
//       ボート自身の位置を記録し続ける**（try_enque_savecsv は生の変数を読む）
//     ・NO SIGNAL に落ちた瞬間、分岐が外れて即座に自機データへ戻る。
//       上書き方式だと古い機体データが残ってしまう
//     ・received/ は受信データ専用のまま保てる
bool link_mirror_active() {
    return link_mode_setting == LINK_MODE_RX && link_is_receiving();
}

LinkDispState link_display_state() {
    if (link_mode_setting != LINK_MODE_RX) return LINK_DISP_NOSIGNAL;
    return link_is_receiving() ? LINK_DISP_MIRROR : LINK_DISP_NOSIGNAL;
}

// モジュールが応答しているか。E220 は設定を読み返して照合できるので、
// 「LINK メッセージが来ているか」ではなく **設定照合が通ったか** で判定する。
bool link_module_alive()   { return e220_alive(); }

uint16_t link_rx_count()   { return (uint16_t)(s_rxTotal & 0xFFFF); }
uint16_t link_miss_count() { return s_missTotal; }
uint16_t link_tx_count()   { return (uint16_t)(s_txCount & 0xFFFF); }

int8_t   link_rssi_avg()   { return s_rssiN ? (int8_t)(s_rssiSum / (int32_t)s_rssiN) : 0; }

// 環境雑音 [dBm]。**到達距離を決めているのはこれ**（docs/pons_link.md §8）。
//   限界 RSSI ≒ 環境雑音 + SF の SNR 閾値 なので、現地でこれを見れば
//   「あとどれだけ余裕があるか」がその場で分かる。
//   熱雑音の理論値は BW500kHz で約 −111dBm。実測 −104dBm なら静かな部類。
//
//   ★ モジュールへの問い合わせなので、無線設定画面を開いている間だけ
//     1 秒に 1 回まで実行する。飛行中の描画パスからは呼ばない。
//     応答は数 ms で返る（無応答なら e220_alive() が先に false になる）。
int16_t link_noise_dbm() {
    static int16_t  cached = 0;
    static uint32_t lastMs = 0;
    if (!e220_alive() || link_mode_setting == LINK_MODE_OFF) return 0;
    if (lastMs && (millis() - lastMs) < 1000) return cached;
    lastMs = millis();
    int16_t n = 0;
    if (e220_read_noise(&n, nullptr)) cached = n;
    return cached;
}

// 重複送信機は受信機側で検出する（link_poll_radio）。
bool     link_dup_sender() { return s_dupSender; }

uint16_t link_sender_status() { return link_is_receiving() ? s_rx.t.status : 0; }

// ★ 目的地／ナビモードが送信側と食い違っていないか。
//   受信側の警告は「ミラーされた値から自分で再計算する」方式なので、
//   目的地が違うと**コース警告だけが静かにずれる**。計算は正しいのに
//   結果が違う、という一番たちの悪い壊れ方をするので、明示的に検出する。
bool link_dest_mismatch() {
    if (!link_is_receiving()) return false;
    return (s_rx.t.dest_index != (int8_t)currentdestination) ||
           (s_rx.t.nav_mode   != (uint8_t)destination_mode);
}

// ---- リプレイ経路と同じ形の供給関数 ----
// 受信できていない、または have ビットが立っていない項目は false / 0 を返す。
// 呼び出し側は必ず link_has_value() で確かめてから使うこと。
bool link_has_value(uint16_t havebit) {
    return link_is_receiving() && (s_rx.t.have & havebit);
}
float link_get_kf_altitude() { return s_rx.t.kf_alt_dm  * 0.1f; }
float link_get_kf_vspeed()   { return s_rx.t.kf_vs_cms  * 0.01f; }
float link_get_pressure()    { return s_rx.t.press_dpa  * 0.1f; }
float link_get_voltage()     { return link_unpack_volt(s_rx.t.volt_cv); }

bool link_get_attitude(float &roll, float &pitch) {
    if (!link_has_value(RHAVE_ATT)) return false;
    roll  = s_rx.t.roll_cdeg  * 0.01f;
    pitch = s_rx.t.pitch_cdeg * 0.01f;
    return true;
}
bool link_get_yaw(float &yaw, float &acc95) {
    if (!link_has_value(RHAVE_ATT_YAW)) return false;
    yaw   = s_rx.t.yaw_cdeg * 0.01f;
    acc95 = (float)s_rx.t.yaw_acc95_deg;
    return true;
}
bool link_get_pitch_avg(float &avg) {
    if (!link_has_value(RHAVE_ATT_AVG)) return false;
    avg = s_rx.t.pitch_avg_cdeg * 0.01f;
    return true;
}
bool link_get_roll_trim(float &trim) {
    if (!link_has_value(RHAVE_ATT_TRIM)) return false;
    trim = s_rx.t.roll_trim_cdeg * 0.01f;
    return true;
}
bool link_get_wind(float &speed_mps, float &dir_to_deg) {
    if (!link_has_value(RHAVE_ATT_WIND)) return false;
    speed_mps  = s_rx.t.wind_dmps * 0.1f;
    dir_to_deg = s_rx.t.wind_dir_cdeg * 0.01f;
    return true;
}
