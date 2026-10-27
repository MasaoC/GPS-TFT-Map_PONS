// ============================================================
// File    : link.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : PONS Link（機体⇄ボート無線）の RP2350 側実装。
//           UART1 のフレーミング、送信テレメトリの組み立て、
//           受信テレメトリの保持と供給。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/27
// ============================================================
//
// ■ 設計の要点
//   ・**送信の write() はあえてブロックさせる。** 隙間なく 1 回で流し込まないと
//     E220 が途中でパケットを切ってしまうため（docs/pons_link.md §4）。
//     115200bps・32 バイト（宛先 3 バイトを足して 35 バイト。ハード FIFO 32B を超えるのは 3 バイト）
//     なので 0.3ms ほどで、1Hz なら Core0 への影響は無視できる（65 バイトの頃は 3.1ms）。
//     長く止まる可能性があるのは「モジュールがまだ前の送信を処理中」のときだけで、
//     そこは e220_send() が AUX を見て**そのスロットを捨てる**ことで避けている。
//   ・受信データの供給関数は gnss.cpp のリプレイ用と同じ形にしてある。
//     受信モードを既存のリプレイ経路へ相乗りさせるため。
//
#include "link.h"
#include "settings.h"
#include "gnss.h"
#include "imu.h"
#include "attitude.h"
#include "airdata.h"
#include "display_tft.h"
#include "mysd.h"
#include "navdata.h"
#include "e220.h"        // 下位ドライバ（UART・モード制御・レジスタ）
#include "lora_link/link_wire2.h"   // 電波に載せる 32 バイトのフレーム
#include "power.h"       // 電源経路の異常・SW OFF のまま USB（送信機の電源の状態をビットで運ぶ）

// ★ UART とモジュール制御は e220.cpp が持つ。ここは「何を送るか」だけを扱う。
//   AUX(GPIO30) は e220.h の E220_AUX_PIN。送信の刻みは RP2350 が持つ。

// ---- 実体が GPS_TFT_map.ino にあるもの（ヘッダが無いのでここで宣言する）----
// 地上の姿勢チェックが「いま発報している」状態。status に載せて、受信機でも
// 同じ音声を鳴らすために読む（飛行中は立たない。.ino 側の宣言のコメント参照）。
extern bool ground_roll_warn_active;
extern bool ground_pitch_warn_active;
extern bool ground_calib_warn_active;
extern volatile int scaleindex;    // 地図の縮尺（scalelist の添字）
void set_scaleindex(int idx);      // 縮尺を合わせる（範囲外は無視される）

// 受信が途切れてから自機表示へ移るまで（docs/pons_link.md §5）。
// ★ RSSI10_WIN_MS と同じ値にしてあること。受信中なら 10 秒窓に必ず 1 発は入る、
//   という前提で link_rssi_bars() が成り立っている。
// ★ アッパーバンドで送る間隔が 1 秒より長いときは、その分だけ伸ばす（link_nosignal_ms）。
#define NOSIGNAL_MS      10000

// ---- 設定（SD の設定ファイルから読まれる）----
// ★ 電波の設定は「数値」でしかない。意味を決めるのは無線層（e220.cpp）で、
//   radio_ch = E220 のチャネル、radio_profile = SF のプリセットに対応する。
uint8_t link_mode_setting    = LINK_MODE_OFF;   // 設定ファイルから読むので外部リンケージ
uint8_t link_radio_ch        = 6;
uint8_t link_radio_profile   = 1;   // 既定 = SF8
// ★ グループ ID。**既定 0 のまま git に置く。** 実際の値は各機の SD にだけ入れる。
//   同じ会場に別チームの PONS がいても取り違えないための識別子（link_proto.h 参照）。
uint8_t link_group           = LINK_GROUP_DEFAULT;

// ---- 受信状態 ----
static LinkRxTelem   s_rx;                 // 直近に受信したテレメトリ
static bool          s_rxValid   = false;
static uint32_t      s_rxMs      = 0;      // 受信した時刻（RP の millis）
// 新しいフレームを受け取ったことの合図。link_take_new_frame() で読むと落ちる。
static bool          s_rxNewFrame = false;
// 同じ合図の**描画用**（link_take_draw_frame）。上はミラーの位置更新（gnss.cpp）が読んで落とすので、
// 描画が同じものを読むと片方が取りこぼす。だから別に持つ。
static bool          s_drawFrame  = false;
// 送信機の送信周期の推定 [ms]（link_draw_window_ok）。送信機は約 1 秒ごとに送るが、
// ループの遅れの分だけ長くなるので、続けて受かったフレームの間隔から求める。
static uint32_t      s_txPeriodEstMs = 1000;
// ---- 送信側 ----
static uint16_t s_seqOut    = 0;    // 送信テレメトリの連番。ロールオーバーしてよい

// ---- 集計（無線モジュールではなく RP2350 が数える）----
static uint32_t s_txCount   = 0;    // 送信回数
static uint32_t s_txNoRf = 0;   // 書いたのに送信が始まらなかった回数（60 秒ログの noRf=）
static uint32_t s_txAirMaxMs = 0;    // 実測の送信時間の最大 [ms]（60 秒ログの airmax=）
static uint32_t s_txAirTimeout = 0;  // 送信完了を待ちきれなかった回数（to=）
static uint32_t s_txMs      = 0;    // 直近に送出できた時刻。地図の電波アイコンを脈動させる
static uint16_t s_missTotal = 0;    // seq の飛びの累計＝電波側で落ちた数
// ---- 直近 10 秒の RSSI ----
//   通算平均は「今アンテナを向け直したら良くなったか」が分からない。
//   現地で向きや場所を試すときに効くのは直近の平均なので、別に持つ。
//   1Hz 受信なので 16 枠あれば 10 秒ぶんは必ず収まる。
// ★ 窓の長さは「無信号」と同じく送る間隔に比例させる（link_nosignal_ms。アッパーの SF9 以上は
//   10 秒より長い）。どの間隔でも窓に入るのは 10 発ぶんなので、16 枠で足りる。
#define RSSI10_SLOTS   16
#define RSSI10_WIN_MS  10000   // ローワー（1 秒おき）での窓の長さ。実際は link_nosignal_ms()
static int8_t   s_r10Val[RSSI10_SLOTS];
static uint32_t s_r10Ms [RSSI10_SLOTS];
static uint8_t  s_r10Head = 0;
// 別グループの送信機を受けたか。**無信号と区別して出す**ための記録。
// SD が飛んで group が 0 に戻ったとき、黙って無反応になると原因に辿り着けない。
static bool     s_otherGroup   = false;
static uint8_t  s_otherGroupId = 0;
static uint32_t s_otherGroupMs = 0;

// 送信機の重複検出。★ bool の永久ラッチにしない（settings.h の LINK_DUPSENDER_* 参照）。
static uint32_t s_dupSenderMs      = 0;  // 最後に検出した時刻（0 = 一度も検出していない）
static uint32_t s_dupSenderAlertMs = 0;  // 最後に音声で知らせた時刻
static uint16_t s_seqBack   = 0;    // seq の逆行回数＝送信機が複数いる証拠
static uint32_t s_seqBackMs = 0;    // 直近に逆行を見た時刻
#define SEQBACK_WINDOW_MS  10000    // この間隔が空いたら数え直す（再起動と区別）

// ============================================================
//  リプレイ中／デモ中の抑止
//    画面の数字が電波と無関係になる場面。理由と約束は link.h に書いてある。
//    ★ **ここが唯一の判定**。呼び出し側に getReplayMode() / is_demo_active() を
//      書き写さないこと（0.984 までの二重記述が食い違いの温床だった）。
//    ★ 受信（RX）の電波そのものは止めない。受信して received/ に書き続けるのは
//      ボートの本来の仕事で、害が無いうえ失うと二度と戻らない。
//      送信（TX）を止めるのは **嘘の電波を出す**から。この非対称には理由がある。
// ============================================================
static bool link_offline_scene() { return getReplayMode() || is_demo_active(); }
bool link_rf_suppressed() { return link_offline_scene(); }
bool link_live_ui_ok()    { return !link_offline_scene(); }

// ============================================================
//  警告の鳴らし方
//    音声（WAV）は本体 FLASH にあるので、**SD の有無に関係なく必ず喋る**
//    （0.982 で SD から FLASH へ移した。それまでは SD が無い機体では
//     警告ごとに決めたトーンで代替していた）。
//    優先度 3 = コース警告と同等。設定ミスや機器異常は飛ぶ前に気づく必要がある。
// ============================================================
//   wav に nullptr を渡すとトーンだけを鳴らす（音声にする必要が無い警告用）。
//   ★ freq/dur/count はその nullptr の呼び出しのために残してある。
//     WAV を持つ呼び出しでは使われないが、**引数ごと消さないこと。**
//     トーン専用の警告（nullptr 渡し）が現に存在する。
//
//   min_vol（最低保証音量）は **既定 0 = 音量設定に従う**。
//   ★ **60 を指定してよいのは電池切れの警告だけ。**
//     この仕組みは「持ち主が機体から離れていても電池切れに気づけるように」という
//     目的だけで入れたもので、他の警告にも付けると音量を絞った意味が無くなる
//     （絞ったのに全部の警告が大音量で鳴る）。
//     以前は link_alert() が全部 60 を付けていたので、無線の警告 8 種が
//     まとめて音量設定を無視していた。
static void link_alert(const char* wav, int freq, int dur, int count, int min_vol = 0) {
    if (wav) enqueueTask(createPlayWavTask(wav, 3, min_vol));
    else     enqueueTask(createPlayMultiToneTask(freq, dur, count, 3, min_vol));
}

// ---- 通信の統計（60 秒ごとにシステムログへ残す）----
static uint32_t s_rxTotal    = 0;   // 受信できたフレーム数
static int8_t   s_rssiMin    = 0, s_rssiMax = 0;
static bool     s_rssiSeen   = false;
static uint16_t s_seqGaps    = 0;   // seq の飛び（電波側の取りこぼし）
static uint16_t s_lastSeq    = 0;
static bool     s_haveLastSeq = false;

typedef enum { LTX_IDLE = 0, LTX_WAKING, LTX_TXSTART, LTX_TXDONE } LinkTxState;
static LinkTxState s_txState     = LTX_IDLE;
static uint32_t    s_txStateMs   = 0;

static void link_push_telemetry();   // 実体は下（送信側の組み立て）
static void link_preflight_start();  // 実体は下（送信前チェック）
static void link_apply_power_state();// 実体は下（モードに応じた mode 0 / mode 3）
static bool link_chscan_holds_radio();// 実体は下（チャンネル調査がモジュールを使っている／終わった直後）

// ============================================================
//  受信の取り込み
//    e220.cpp が印（LINK_W2_MAGIC）と CRC まで確かめた 32 バイトの 1 パケットを返す。
//    間に中継 MCU がいないので、UART フレーミング（sync + CRC）は不要。
// ============================================================
// 電波に載るのは 32 バイトのフレーム（lora_link/link_wire2.h。2026-10-27〜。どの CH でも）。
// 受けたらここで LinkTelem に組み立て直すので、上の層は 65 バイトの LinkTelem のまま扱う。
// ★ 回転枠の値を覚えておく必要があるので、**自分のグループのものだけ**積む（merge_own）。
//   他グループのフレームと送信前チェック・チャンネル調査は核だけ取り出す（group と seq が
//   分かれば足りる。回転枠の値を混ぜない）。
static LinkW2Rx s_w2rx;
static uint8_t  s_lastSlot = 0xFF;   // 最後に受けた自グループのフレームの回転枠の番号（received/ の slot 列）

// 1 フレーム受けて LinkTelem にする。slot には回転枠の番号を返す（要らなければ nullptr）。
static bool link_recv_telem(LinkTelem* t, int16_t* rssi, bool merge_own, uint8_t* slot) {
    uint8_t buf[LINK_W2_LEN];
    if (!e220_recv(buf, (uint8_t)LINK_W2_LEN, rssi)) return false;
    if (slot) *slot = link_w2_slot(buf);
    if (merge_own && link_w2_group(buf) == link_group) {
        link_w2_decode(buf, &s_w2rx, millis(), s_txPeriodEstMs);
        *t = s_w2rx.acc;
    } else {
        LinkW2Rx tmp;
        link_w2_rx_reset(&tmp);
        link_w2_decode(buf, &tmp, millis(), s_txPeriodEstMs);
        *t = tmp.acc;
    }
    return true;
}

// 回転枠で運ぶ項目（have ビットの無い目的地・ナビモード・縮尺）がもう分かっているか。
// ★ 受け始めた直後（と長く途切れた後）は 0 のまま。そのまま使うと、受信機が「送信機は目的地 0・
//   縮尺 0」と思い込んで自分の設定を書き換えてしまう（link_sync_destination / link_sync_scale）。
static bool link_rx_slot_known(uint8_t slot) {
    return link_w2_slot_seen(&s_w2rx, slot);
}

static void link_poll_radio() {
    if (link_mode_setting != LINK_MODE_RX) return;

    LinkTelem t;
    int16_t   rssi = 0;
    uint8_t   slot = 0xFF;
    while (link_recv_telem(&t, &rssi, true, &slot)) {
        // ★ グループが違う＝別チームの PONS。**表示にも記録にも使わない。**
        //   ただし「受信できていない」とは別物なので、そのことは残す
        //   （設定画面に出す。これが無いと SD の設定ミスに気づけない）。
        if (t.group != link_group) {
            s_otherGroup   = true;
            s_otherGroupId = t.group;
            s_otherGroupMs = millis();
            continue;
        }
        s_rx.t      = t;
        s_lastSlot  = slot;
        s_rx.rssi   = (int8_t)constrain((int)rssi, -128, 127);
        s_rx.age_ms = 0;             // 直接受け取るので遅延はほぼ無い
        s_rxValid   = true;
        const uint32_t prev_rx_ms = s_rxMs;   // 送信周期の推定に使う（下の seq の解析）
        s_rxMs      = millis();
        s_rxNewFrame = true;         // ミラー中の「新しい位置が来た」の合図
        s_drawFrame  = true;         // 同じ合図の描画用（link_take_draw_frame）

        // 受信ログ（docs/pons_link.md §5）。received/ は無線で得た情報だけを置く。
        enqueueTask(createSaveRxCsvTask(&s_rx.t, s_rx.rssi, s_rx.age_ms, s_lastSlot));

        // ---- 集計 ----
        s_rxTotal++;
        if (!s_rssiSeen) { s_rssiMin = s_rssiMax = s_rx.rssi; s_rssiSeen = true; }
        else {
            if (s_rx.rssi < s_rssiMin) s_rssiMin = s_rx.rssi;
            if (s_rx.rssi > s_rssiMax) s_rssiMax = s_rx.rssi;
        }
        s_r10Val[s_r10Head] = s_rx.rssi;
        s_r10Ms [s_r10Head] = s_rxMs;
        s_r10Head = (uint8_t)((s_r10Head + 1) % RSSI10_SLOTS);

        // ---- seq の解析 ----
        //   送信機が 1 台なら seq は必ず増える。減る・止まるのは異常。
        if (s_haveLastSeq) {
            const uint16_t d = (uint16_t)(t.seq - s_lastSeq);
            // 続けて受かった 2 発の間隔から送信周期を求める（1/8 の移動平均）。
            // 再起動や電波の途切れをまたいだ値は入れない（範囲外は捨てる）。
            // ★ 受け付ける範囲は送る間隔（link_tx_period_ms）に合わせて動かす（ローワーは 900〜1300ms）。
            if (d == 1) {
                const uint32_t iv  = s_rxMs - prev_rx_ms;
                const uint32_t per = link_tx_period_ms();
                if (iv + (1000 - LINK_TX_PERIOD_MIN_MS) >= per && iv * 1000 <= per * LINK_TX_PERIOD_MAX_MS)
                    s_txPeriodEstMs = (7 * s_txPeriodEstMs + iv) / 8;
            }
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
        if (s_seqBack >= 3) {
            const uint32_t now = millis();
            s_dupSenderMs = now;        // 「いま検出中」を更新（表示の保持に使う）
            // ★ 音声は間隔を空けて繰り返す。1 回きりだと聞き逃したら二度と鳴らず、
            //   毎フレーム鳴らすと他の警報を押しのける。
            if (s_dupSenderAlertMs == 0 ||
                now - s_dupSenderAlertMs >= LINK_DUPSENDER_ALERT_MS) {
                s_dupSenderAlertMs = now;
                link_alert("wav/link_dup_sender.wav", 294, 400, 3);
                enqueueTask(createLogSdfTask(
                    "LINK ERROR: another sender detected (seq went backwards %u times)",
                    (unsigned)s_seqBack));
            }
        }
        // ★ 制約：この検出は**受信機でしか働かない**。送信機は送信の合間
        //   mode 3 で寝ていて受信できないため。2 台とも TX に誤設定した場合、
        //   受信機から見れば「送信機が 2 台いる」として現れる。
    }
}

// ============================================================
//  初期化・周期処理
// ============================================================
// settings.txt の SF を下げたとき（アッパーの CH で SF11）、その元の値。0xFF = 下げていない。
static uint8_t s_profLoweredFrom = 0xFF;

void link_setup() {
    link_w2_rx_reset(&s_w2rx);
    // ★ アッパーの CH と SF11 の組み合わせは使わない（link_radio_profile_allowed）。
    //   settings.txt の CH と SF は別々に読まれるので、そろったここで確かめて SF10 へ下げる
    //   （起動の末尾で知らせる。link_announce_mode）。
    if (!link_radio_profile_allowed(link_radio_ch, link_radio_profile)) {
        s_profLoweredFrom  = link_radio_profile;
        link_radio_profile = LINK_UPPER_PROF_MAX;
    }
    s_txPeriodEstMs = link_tx_period_ms();
    // UART・モードピン・レジスタ設定はすべて e220.cpp が面倒を見る。
    // 設定は書いたあと読み返して照合されるので、通らなければ
    // e220_alive() が false になり link_watch_module() が鳴らす。
    e220_setup(link_radio_ch, link_radio_profile);

    // e220_setup() は mode 0 で戻ってくるので、モードに応じて寝かせ直す。
    link_apply_power_state();

    // ★ **何を書き込んだかを log.txt に残す。**
    //   ここで使う link_mode_setting / ch / profile は Core1 の loadSettings() が
    //   入れる値で、startup_demo_tft() の sd_setup_complete 待ちを挟んでいるので
    //   通常は読み込み済みになっている。ただし**あの待ちは 5 秒で打ち切る**ので、
    //   SD が極端に遅いと既定値（OFF / CH 既定）のまま無線を設定してしまう。
    //   症状は「送信しているつもりで違うチャンネルにいる」で画面には何も出ない。
    //   settings=0 が残っていればそれが起きたと断定できる。
    //   e220_alive() も一緒に残す。false なら以下の値は書けていない。
    enqueueTask(createLogSdfTask("LINK SETUP link=%u ch=%u sf=%u grp=%u alive=%d settings=%d",
                                 link_mode_setting, link_radio_ch,
                                 link_radio_profile, link_group,
                                 (int)e220_alive(), (int)settings_loaded));

    // ★ 読み上げと送信前チェックはここでは行わない。setup() の末尾で
    //   link_announce_mode() / link_preflight_restart() を呼ぶこと。理由は両関数の頭。
}

// settings.txt の CH を寄せたとき、その元の値。0xFF = 寄せていない。
// 読み込み（Core1）で立てて、起動の末尾（Core0）で 1 回だけ知らせて消す。
static volatile uint8_t s_chSnappedFrom = 0xFF;
void link_note_radio_ch_snapped(uint8_t from) { s_chSnappedFrom = from; }

// 起動時に無線モードを読み上げる。**setup() の末尾で呼ぶ。**
// ★ **3 状態すべてを読み上げる。OFF の分岐を無音に戻さないこと**（0.985〜）。
//   「無音＝OFF」では音量 0 や音声の焼き忘れと区別が付かない。docs/pons_link.md §1。
//   範囲外の値は来ない（setLinkMode() が OFF へ丸める）が、else で受けて無音を作らない。
// ★ link_setup() から切り離してある理由:
//   link_setup() は起動画面に E220 の結果を出すためスプラッシュより前へ移した。
//   読み上げもそこへ付いていくと、**起動音(opening.wav)より先に喋ってしまう**。
//   元どおり setup() の末尾で呼ぶことで、鳴る順番は移動前と同じになる。
void link_announce_mode() {
    // ★ 以前の版の settings.txt に 0/3/6/9/12 以外の CH が残っていたので寄せた。
    //   3 台とも同じ版なら同じ CH に揃うが、1 台だけ古い版のままだと CH がずれて
    //   「無信号」になり、現場では距離や故障と区別が付かない。だから黙って寄せない。
    //   ★ モードの読み上げより**先に**積むこと。こちらは優先度 3 なので、後から積むと
    //     読み上げを途中で止めて頭から鳴り直させる（docs/pons_sound.md §3）。
    //   寄せた値で保存し直すので、知らせるのはこの 1 回だけ。
    // ★ アッパーの CH で SF11 だったので SF10 へ下げたときも同じ（SF が 1 台だけ違っても無信号）。
    if (s_chSnappedFrom != 0xFF || s_profLoweredFrom != 0xFF) {
        if (s_chSnappedFrom != 0xFF)
            enqueueTask(createLogSdfTask("LINK ch %u -> %u (only 0/3/6/9/12 and upper 15/21/27 are used)",
                                         (unsigned)s_chSnappedFrom, (unsigned)link_radio_ch));
        if (s_profLoweredFrom != 0xFF)
            enqueueTask(createLogSdfTask("LINK sf %u -> %u (SF11 is not used on upper ch %u)",
                                         (unsigned)e220_profile_to_sf(s_profLoweredFrom),
                                         (unsigned)e220_profile_to_sf(link_radio_profile), (unsigned)link_radio_ch));
        link_alert("wav/link_setting_changed.wav", 880, 200, 3);
        enqueueTask(createSaveSettingTask());
        s_chSnappedFrom   = 0xFF;
        s_profLoweredFrom = 0xFF;
    }
    if      (link_mode_setting == LINK_MODE_TX)
        enqueueTask(createPlayWavTask("wav/sender_mode.wav",   2, 0, WAV_EXCL_LINKMODE));
    else if (link_mode_setting == LINK_MODE_RX)
        enqueueTask(createPlayWavTask("wav/receiver_mode.wav", 2, 0, WAV_EXCL_LINKMODE));
    else
        enqueueTask(createPlayWavTask("wav/link_off.wav",      2, 0, WAV_EXCL_LINKMODE));
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
    // ★★ **リプレイ中・デモ中は鳴らさない。** この音の意味は「いま画面に出ている
    //   数値の意味が機体と自機の間で入れ替わった」で、再生中・デモ中の画面は
    //   電波では一切動かない。鳴らすと言っていることが嘘になる。
    // ★ **前回値を更新せずに戻ること。** 更新すると再生中に起きた遷移が黙って
    //   飲まれ、再生をやめて画面が NO SIGNAL に変わったことを誰も言わなくなる。
    //   更新しないので、再生をやめた直後の 1 回でちゃんと知らせる。
    if (!link_live_ui_ok()) return;
    // ★ チャンネル調査の間と、終わった直後も鳴らさない（前回値は更新しない。上と同じ理由）。
    //   調査中は受信を止めているので「途切れた」に見えるが、電波が途切れたわけではない。
    //   終わった直後も次のフレームが届くまでは古いままなので、少し待ってから見る。
    if (link_chscan_holds_radio()) return;
    static bool was_mirror = false;
    static bool primed     = false;   // 起動直後の 1 回目は鳴らさない
    const bool now_mirror = link_is_receiving();
    if (primed && now_mirror != was_mirror) {
        if (now_mirror) {
            // 復帰は頻繁に起きうるので音声にはせず、上昇 3 音（ラ・ド#・ミ）を 2 回。
            // ★ 以前の 2 音（880 → 1318Hz・計 0.2 秒）は短すぎて気づけなかった
            //   （同時に鳴っていた「測位しました」にもかき消されていた。.ino の gnss_conn_tick で直した）。
            //   GNSS の復帰（392 → 784Hz の 2 音）と取り違えないよう、高い音域の 3 音にしてある。
            for (int rep = 0; rep < 2; rep++) {
                enqueueTask(createPlayMultiToneTask(880,  150, 1, 2));
                enqueueTask(createPlayMultiToneTask(1109, 150, 1, 2));
                enqueueTask(createPlayMultiToneTask(1319, 300, 1, 2));
            }
        } else {
            // 失ったほうは表示の意味が変わるので、言葉で知らせる。
            link_alert("wav/link_no_signal.wav", 1318, 250, 3);
        }
        enqueueTask(createLogSdfTask("LINK %s", now_mirror ? "acquired" : "lost"));
    }
    was_mirror = now_mirror;
    primed = true;
}

// モジュールが今も応答するかを定期的に確かめる。
//
// ★★ これが無いと「起動後に死んだ」を誰も検出できない。e220_alive() の元になる
//   照合は e220_setup() / e220_reconfigure() でしか行われないので、**飛行中に
//   コネクタが抜けても true のまま**になる。そのとき何が起きるか:
//     ・e220_send() は UART へ書き込むだけで成功する。AUX は未実装対策の
//       INPUT_PULLUP なので、断線すると High（アイドル）に見えて素通りする。
//     ・送信回数（Sent）は増え続け、地図の電波アイコンも脈動し続ける。
//       **画面上は完全に正常なまま、電波は一切出ていない。**
//     ・上りが無いので、ボートが無信号になるまで機体側では気づけない。
//   問い合わせて返事があるかを見るのが唯一の確実な方法なので、ここで叩く。
//
// ・環境ノイズ取得を生存確認に流用する。**値は使わない。返事があったかだけが目的**
//   （成否の勘定は e220.cpp が持っている）。
// ・送信機は寝ているので起こしてから聞き、すぐ寝かせる。生きていれば数 ms。
// ・受信機も確かめる。「受信が無いだけ」なのか「自分のモジュールが死んだ」のかは、
//   ボート側では区別が付かないため。ただし受信できている間は叩かない
//   （1 発でも受かれば e220_recv() が生存を記録する。無駄に UART へ割り込まない）。
static void link_probe_module() {
    if (link_mode_setting == LINK_MODE_OFF) return;
    // 点検中は自分で LINK_PF_SAMPLE_MS ごとに測っているので、重ねて叩かない。
    // （実体は下にあるので、宣言済みのアクセサ経由で見る）
    if (link_preflight_state() == LINK_PF_RUNNING) return;
    // 受信できている＝生きている。問い合わせる必要そのものが無い。
    if (link_mode_setting == LINK_MODE_RX && link_is_receiving()) return;
    const bool tx = (link_mode_setting == LINK_MODE_TX);
    // 送信中（WAKING / TXSTART / TXDONE）および直後には叩かない（送信完了ガード）。
    // アイドル（LTX_IDLE）へ戻り、かつ静かになった時点で改めて実行される。
    if (tx && (s_txState != LTX_IDLE || link_since_tx_ms() < LINK_PROBE_AFTER_TX_MS)) return;

    static uint32_t lastProbe = 0;
    const uint32_t now = millis();
    if (now - lastProbe < LINK_ALIVE_PROBE_MS) return;
    lastProbe = now;

    // ★ 生存確認は **レジスタ読み**で行う（e220_ping）。
    //   以前は雑音の問い合わせを流用していたが、あれは mode 0 ではコマンドとして
    //   解釈されず電波が出るだけで、**必ず失敗する**。その失敗が無応答として
    //   数えられ、健全なモジュールを「死んでいる」と誤判定していた。
    //   ping は mode 3 へ約 20ms 潜る。上の early return で
    //   「受信できているとき」と「送信直後」は既に除いてあるので実害は無い。
    if (tx) e220_wake();               // 寝ていると潜る意味が変わるので起こしておく
    const bool responds = e220_ping();

    // ★ **設定が通っていないだけで、モジュールは生きている**場合の復帰。
    //   s_alive は e220_reconfigure() の中でしか立たないため、書込みが一度
    //   失敗すると e220_alive() が false のまま固定され、モードを変えても
    //   戻らない（settings.h の LINK_REVIVE_INTERVAL_MS のコメント参照）。
    //   ping が通るなら通信自体は生きているので、設定を書き直して復帰させる。
    //   ★ reconfigure は Core0 を最大 0.6 秒止めるので、間隔を空けて試みる。
    if (responds && !e220_alive()) {
        static uint32_t lastRevive = 0;
        if (lastRevive == 0 || (now - lastRevive) >= LINK_REVIVE_INTERVAL_MS) {
            lastRevive = now;
            enqueueTask(createLogSdTask("LINK: module responds but unconfigured, re-configuring"));
            e220_reconfigure(link_radio_ch, link_radio_profile, false);
            link_apply_power_state();
            enqueueTask(createLogSdfTask("LINK: revive -> alive=%d", (int)e220_alive()));
        }
    }

    if (tx) e220_sleep();              // 送信機は送信の合間は寝かせる
}

// 無線モジュールが応答しない＝設定の照合が通らない、または問い合わせに返事が無い。
// 配線・電源・モジュール不良のいずれかなので、人が気づくしかない。
// ★ 上の link_probe_module() のおかげで、**起動後に死んだ場合もここに落ちてくる**。
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

// ---- 目的地の照合（docs/pons_link.md §5）----
// 目的地の ID。名前と座標（1e-5 度 ≒ 1m に丸める）の CRC16。0 は「送っていない」に取ってあるので、
// 計算が 0 になったら 1 にする。未選択（範囲外）は 0。
// ★ 番号（dest_index）だけで突き合わせると、3 台の destinations.csv の中身や並び、パイロンの上書き
//   （pilon override）が違うとき、受信機が**黙って別の場所**に合わせる（番号が同じなら食い違いとしても
//   見えない）。中身で照合すれば、それが全部「受信機に無い」として見える。
uint16_t link_dest_id(int idx) {
    if (idx < 0 || idx >= destinations_count) return 0;
    const mapdata& d = extradestinations[idx];
    uint8_t buf[64 + 8];
    size_t n = d.name ? strnlen(d.name, 64) : 0;
    if (n) memcpy(buf, d.name, n);
    const int32_t la = (int32_t)lround(d.cords[0][0] * 1e5);
    const int32_t lo = (int32_t)lround(d.cords[0][1] * 1e5);
    for (int i = 0; i < 4; i++) {
        buf[n + i]     = (uint8_t)((uint32_t)la >> (8 * i));
        buf[n + 4 + i] = (uint8_t)((uint32_t)lo >> (8 * i));
    }
    const uint16_t id = link_crc16(buf, (uint16_t)(n + 8));
    return id ? id : 1;
}

// 送信機の目的地を、受信機の目的地の番号に直したもの（下の LINK_DEST_* か、0 以上の番号）。
#define LINK_DEST_UNSET   (-1)   // 送信機が未選択
#define LINK_DEST_MISSING (-2)   // 受信機に無い（ID が一致するものが無い）
#define LINK_DEST_UNKNOWN (-3)   // まだ分からない（回転枠 6・7 をまだ受けていない）
// ★ **ID で探す。番号は「同じ番号なら速く済む」ための手掛かりにしか使わない。** 番号は回転枠 6、ID は 7 と
//   別のフレームで来るので、送信機が目的地を変えた直後は 1 発だけ食い違う。ID を先に信じれば、
//   どちらの順で届いても合わせ直しは 1 回で済む（知らせる音声が 2 回鳴らない）。
static int link_sender_dest_local() {
    if (!link_rx_slot_known(W2S_HACC) || !link_rx_slot_known(W2S_NAV)) return LINK_DEST_UNKNOWN;
    const int8_t   di = s_rx.t.dest_index;
    const uint16_t id = s_rx.t.dest_id;
    if (di < 0) return LINK_DEST_UNSET;
    // ID を送らない機体（2026-10-26 までの版・debug/v7-noise-test）は、以前どおり番号で合わせる
    if (id == 0) return (di < destinations_count) ? di : LINK_DEST_MISSING;
    if (link_dest_id(di) == id) return di;            // 同じ番号に同じ目的地がある（ふつうはここ）
    // 番号が違う（並びが違う）か、受信機に無い。全部の ID を計算して探す（最大 MAX_DESTINATIONS 件。
    // 送信機の ID と目的地の数が変わらない間は前の答えを使う）。
    static uint16_t c_id = 0;
    static int      c_count = -1, c_res = LINK_DEST_MISSING;
    if (c_id != id || c_count != destinations_count) {
        c_res = LINK_DEST_MISSING;
        for (int i = 0; i < destinations_count; i++)
            if (link_dest_id(i) == id) { c_res = i; break; }
        c_id = id; c_count = destinations_count;
    }
    return c_res;
}

// 送信機の目的地が受信機の目的地に無いか（WIRELESS 画面と地図の箱の文言を分けるため）。
bool link_dest_missing() {
    return link_is_receiving() && link_sender_dest_local() == LINK_DEST_MISSING;
}

// 目的地／ナビモードを送信側に合わせる（docs/pons_link.md §5）。
// 受信機の目的は「パイロットに出ている画面をそのまま再現すること」なので、
// 食い違ったまま表示しても意味が無い（コース警告だけが静かにずれる）。
// 勝手に設定が変わるのは驚きなので、必ず知らせる。RAM 上だけの上書き。
// ★ 送信機の目的地が受信機に無いときは、**受信機を「未選択」にする**（違う目的地でコース警告・誘導線・
//   距離を出さない）。受信を始めて最初に分かったとき（と送信機が別の目的地へ変えたとき）に 1 回だけ知らせる。
//   受信が切れても未選択のまま（合わせた目的地が残るのと同じ）。
static void link_sync_destination() {
    static uint16_t s_missingAlerted = 0;   // 「受信機に無い」を知らせた送信機の目的地 ID（0 = まだ）
    if (!link_mirror_active()) { s_missingAlerted = 0; return; }
    if (!link_dest_mismatch()) return;
    const int li = link_sender_dest_local();
    if (li == LINK_DEST_MISSING) {
        currentdestination = -1;
        destination_mode   = s_rx.t.nav_mode;
        const uint16_t key = s_rx.t.dest_id ? s_rx.t.dest_id : 0xFFFF;   // ID を送らない機体の番号違いも 1 回
        if (s_missingAlerted != key) {
            s_missingAlerted = key;
            // 低い 3 連（294Hz）。音声が無いので音で知らせ、文言は WIRELESS 画面と地図の箱に出す
            link_alert(nullptr, 294, 200, 3);
            enqueueTask(createLogSdfTask("LINK: sender dest not on this unit (sender idx=%d id=%04X) -> none",
                                         (int)s_rx.t.dest_index, (unsigned)s_rx.t.dest_id));
        }
        return;
    }
    if (li < 0) return;   // 送信側が未選択なら表示で知らせるに留める
    s_missingAlerted = 0;
    currentdestination = li;
    destination_mode   = s_rx.t.nav_mode;
    link_alert("wav/link_override_dest.wav", 494, 200, 3);
    enqueueTask(createLogSdfTask("LINK: dest/nav overridden to sender (%d/%d, sender idx=%d id=%04X)",
                                 li, (int)s_rx.t.nav_mode, (int)s_rx.t.dest_index, (unsigned)s_rx.t.dest_id));
}

// 地図の縮尺を送信機に合わせる（0.988）。
// ★ 受信機の目的は「パイロットに出ている画面の再現」なので、縮尺が違うと同じ地図に
//   見えない。送信機が誤った縮尺のまま飛んでいることにも気づける。
// ★ RAM 上の上書きだけ（SD には書かない）。違ったときだけ合わせる。
// ★ **ミラー中はボート側の拡大率操作（ダブルクリック）が効かなくなる。**
//   意図どおり（送信機に合わせるのが主旨）。変えたいときは受信を切る。
//
// ★★ **「送信機が回した」と「ボート側で変えたのを引き戻した」を区別して鳴らす**（0.991）。
//   前者で鳴らすと**送信機が拡大率を回すたびに鳴って騒がしい**（0.988 で音声に
//   しなかった理由はこれ。画面を見れば分かる変化でもある）。
//   後者は**自分の操作が無視された**ことなので、黙っていると
//   「一瞬変わって戻る＝ボタンかソフトが壊れている」と思わせる。トーンだけ鳴らす。
//   区別の方法: 送信機から来た値が前回と同じなのに自分と違う ＝ こちらで変えた。
static void link_sync_scale() {
    static uint8_t s_last_sender_scale = 0xFF;   // 0xFF = まだ送信機の値を見ていない
    if (!link_mirror_active()) { s_last_sender_scale = 0xFF; return; }

    // ★ 縮尺は回転枠 5 で来る。受けるまでは 0 のままなので合わせない（0 に飛ばさない）。
    if (!link_rx_slot_known(W2S_PRESS)) return;
    const uint8_t sender = s_rx.t.scale_index;
    // 受信開始直後（0xFF）とミラー再開も「送信機が動いた」側に入れる。
    // あれは**最初の位置合わせ**で、却下ではないので鳴らさない。
    const bool sender_moved = (sender != s_last_sender_scale);
    s_last_sender_scale = sender;
    if ((int)sender == scaleindex) return;

    // ★ **引き戻しが実際に効いたときだけ鳴らす。** set_scaleindex() は範囲外を
    //   無視するので、呼ぶ前に鳴らすと、送信機が壊れた値を送り続ける間
    //   link_loop() のたびに鳴り続ける。
    const int before = scaleindex;
    set_scaleindex((int)sender);               // 範囲外なら無視される
    if (!sender_moved && scaleindex != before) {
        // 低い 2 連 = 操作を却下した。★ 優先度 3（コース警告と同等）にしないこと。
        //   UI の「できません」が本物の警告を押しのけるのは順序が逆。
        enqueueTask(createPlayMultiToneTask(262, 100, 2, 2));
        enqueueTask(createLogSdfTask("LINK: scale override (local %d -> sender %d)",
                                     before, (int)sender));
    }
}

// 送信機側の異常（SD／IMU）。飛ぶ前に気づきたい項目。
// 音声にはしない ― 内訳は無線ページに出るので「何かある」と分かれば足りる。
// 音声を増やすほど、本当に聞くべきものが埋もれる。
// ★ 電池低下だけは別扱い（下の link_watch_sender_battery）。ここで一緒に見ると
//   「立ち上がりで 1 回鳴るだけ」になるが、電池は減り続けるので繰り返す必要がある。
// ★ **較正未適用（NEEDS_APPLY）はここから外した**（0.988）。地上で発報している間は
//   LINK_ST_CALIB_GND で音声を鳴らすようになったので、残すとトーンと二重になる。
//   飛行中に立った場合は送信機自身が黙る（対処できない警告で気をそらさない）ので、
//   受信機も黙るのが正しい。画面には今までどおり NEEDS_APPLY で出る。
#define LINK_ST_FAULTS_TONE  (LINK_ST_SD_ERROR | LINK_ST_IMU_ERROR)
static void link_watch_sender_faults() {
    static uint16_t prev_st = 0;
    const uint16_t st = link_sender_status() & LINK_ST_FAULTS_TONE;
    if ((st & ~prev_st) != 0) {
        link_alert(nullptr, 370, 250, 2);
        enqueueTask(createLogSdfTask("LINK WARN: sender fault bits 0x%04X", st));
    }
    prev_st = st;
}

// 送信機（機体）の電池低下を、受信側でも音声で知らせる。
//
// ★ 本番では飛行中に電池を替えられないが、**試験飛行では役に立つ**。
//   ボートが先に気づいて降ろす判断ができる。画面では s の残量が赤くなるだけなので、
//   ボートマンが右上を見ていなければ気づけなかった。
//
// ・繰り返す。電池は減り続けるので、立ち上がりで 1 回では足りない。
//   間隔は自機の電池警告と同じ BAT_WARN_INTERVAL_MS にそろえてある。
// ・受信が切れると link_sender_status() が 0 を返すので、
//   古い状態のまま鳴り続けることはない（ロストは別の警告が担当する）。
static void link_watch_sender_battery() {
    static uint32_t last_warn_ms = 0;
    if ((link_sender_status() & LINK_ST_BAT_LOW) == 0) return;
    const uint32_t now = millis();
    if (last_warn_ms != 0 && now - last_warn_ms < BAT_WARN_INTERVAL_MS) return;
    last_warn_ms = now;
    // ★ 最低音量 60 を付けてよい唯一の警告（上の link_alert() のコメント参照）。
    //   本番では機体のバッテリーを交換できないが、試験飛行では役に立つ。
    link_alert("wav/battery_low_sender.wav", 1568, 200, 4, 60);
    enqueueTask(createLogSdfTask("LINK WARN: sender battery low (%.2fV)",
                                 (double)link_get_voltage()));
}

// 送信機の電源経路の異常（LINK_ST_PWR_FAULT）を受信機でも鳴らす。
// 間隔は送信機と同じ（power.cpp）。機体が発熱している恐れがあるので、
// 最低音量 60 も電池警告と同じく付ける（docs/pons_sound.md §5）。
// ★ 送信機は音声（power_fault.wav）だが、受信機は**トーンのまま**にしてある。
//   同じ音声だと「スイッチを切って電池を外して」がボート自身への指示に聞こえる
//   （電池低下に battery_low_sender.wav を分けたのと同じ理由）。
static void link_watch_sender_power() {
    static uint32_t last_warn_ms = 0;
    if ((link_sender_status() & LINK_ST_PWR_FAULT) == 0) return;
    const uint32_t now = millis();
    if (last_warn_ms != 0 && now - last_warn_ms < POWER_FAULT_ALERT_MS) return;
    last_warn_ms = now;
    link_alert(nullptr, 2637, 150, 6, 60);
    enqueueTask(createLogSdTask("LINK WARN: sender POWER FAULT"));
}

// 送信機の地上チェック（ロール／ピッチ／較正やり直し）を受信機でも同じ音声で鳴らす。
// ★ 鳴らす間隔は送信機と同じ定数を使う（しきい値の判定は送信機側だけが持つ）。
//   片方だけ変えると「機体は喋ったのにボートは黙る」になる。
// ★★ **ビットが落ちても last は落とさないこと**（0.988 の不具合）。送信機のビットは
//   `attitude_is_static()` を含むガードの中で立てているので、機体に触ると一瞬落ちる
//   ＝ **1Hz のテレメトリでは明滅して届く**。落としていたため明滅のたびに即・再発報した。
//   送信機側も条件解除では `gnd_last_warn_ms` を落としていないので、これで揃う。
// ★★ **1 回につき 1 つだけ鳴らす**（見つけたら return）。3 つは同時に立つので、
//   同時に積むと同じ優先度どうしが互いを pending へ追い出す（settings.h の
//   LINK_GROUND_SAY_GAP_MS 参照）。足すときは表に 1 行入れるだけ。
static void link_watch_sender_ground_att() {
    struct Say { uint16_t bit; const char* wav; uint32_t interval; uint32_t last; };
    static Say say[] = {
        { LINK_ST_GROUND_ROLL,  "wav/roll_check.wav",          GROUND_ROLL_WARN_INTERVAL_MS, 0 },
        { LINK_ST_GROUND_PITCH, "wav/pitch_check.wav",         GROUND_ROLL_WARN_INTERVAL_MS, 0 },
        { LINK_ST_CALIB_GND,    "wav/eskf_calib_required.wav", ESKF_APPLY_WARN_INTERVAL_MS,  0 },
    };
    static uint32_t last_say_ms = 0;
    const uint32_t now = millis();
    if (last_say_ms != 0 && now - last_say_ms < LINK_GROUND_SAY_GAP_MS) return;
    const uint16_t st = link_sender_status();
    for (Say &g : say) {
        if ((st & g.bit) == 0) continue;
        if (g.last != 0 && now - g.last < g.interval) continue;
        g.last      = now;
        last_say_ms = now;
        link_alert(g.wav, 370, 250, 3);
        return;
    }
}

// 送信機の GNSS モジュールが抜けた／戻ったことを、送信機と同じ音で知らせる。
// ★ **これは音でしか気づけない。** 画面は「止まった正常な機体」に見える
//   （理由は link_proto.h の LINK_ST_GNSS_LOST）。
// ★ 下降 2 音（784→392）は「何かが失われた」専用の形で、復帰はその逆向き。
//   送信機自身が鳴らすものと同じにしてある（gnss_conn_tick。音を増やさない）。
// ★ ビットが落ちても間隔は落とさない。コネクタが緩んでいると明滅するので、
//   落とすと明滅のたびに鳴り直す（地上チェックで踏んだのと同じ）。
static void link_watch_sender_gnss() {
    static uint32_t last_ms = 0;
    static bool     was_lost = false;
    const bool lost = (link_sender_status() & LINK_ST_GNSS_LOST) != 0;
    const uint32_t now = millis();
    if (lost) {
        if (last_ms == 0 || now - last_ms >= GNSS_LOST_ALERT_MS) {
            last_ms = now;
            enqueueTask(createPlayMultiToneTask(784, 200, 1, 3));
            enqueueTask(createPlayMultiToneTask(392, 300, 1, 3));
            enqueueTask(createLogSdfTask("LINK WARN: sender GNSS not responding"));
        }
    } else if (was_lost && link_is_receiving()) {   // 復帰（受信できているときだけ）
        enqueueTask(createPlayMultiToneTask(392, 120, 1, 2));
        enqueueTask(createPlayMultiToneTask(784, 160, 1, 2));
        enqueueTask(createLogSdTask("LINK: sender GNSS restored"));
    }
    was_lost = lost;
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
            "LINK TX: mod=%s ch=%u prof=%u sent=%u noRf=%lu airmax=%lums to=%lu per=%lums%s",
            mod, link_radio_ch, link_radio_profile,
            (unsigned)s_txCount, (unsigned long)s_txNoRf,
            (unsigned long)s_txAirMaxMs, (unsigned long)s_txAirTimeout,
            (unsigned long)link_tx_period_ms(),
            link_dup_sender() ? " DUP_SENDER" : ""));
    } else if (s_rssiSeen) {
        enqueueTask(createLogSdfTask(
            "LINK RX: mod=%s ch=%u prof=%u rx=%lu miss=%u gaps=%u bad=%lu "
            "bytes=%lu rssi=%d..%d%s",
            mod, link_radio_ch, link_radio_profile,
            (unsigned long)s_rxTotal, (unsigned)s_missTotal,
            (unsigned)s_seqGaps, (unsigned long)e220_bad_frames(),
            (unsigned long)e220_rx_bytes(), s_rssiMin, s_rssiMax,
            link_dup_sender() ? " DUP_SENDER" : ""));
    } else {
        enqueueTask(createLogSdfTask(
            "LINK RX: mod=%s ch=%u prof=%u bad=%lu bytes=%lu no reception yet",
            mod, link_radio_ch, link_radio_profile,
            (unsigned long)e220_bad_frames(), (unsigned long)e220_rx_bytes()));
    }
    // RSSI の幅は「この 60 秒でどうだったか」を見たいので毎回入れ直す。
    // 受信数・取りこぼし・壊れフレーム（bad）は累計のまま（推移が読めるほうが役に立つ）。
    // ※ 以前このコメントは「CRC エラー」を数えている前提で書かれていたが、
    //   実際にはどこにも計数が無かった。e220_bad_frames() を足して実態に合わせた。
    s_rssiSeen = false;
    s_seqGaps  = 0;
}

// ============================================================
//  送信前チェック（プリフライト）
// ============================================================
// 送信モードに入った直後だけ mode 0 に留まり、LINK_PF_WINDOW_MS のあいだ
// **聴くだけ**にする。確かめるのは 2 つ。
//   1. 選んだチャンネルの雑音レベル（e220_read_noise の中央値）
//   2. 同じチャンネルに他の送信機がいないか
//
// ★ 2 が本命。通常運転の送信機は送信の合間 mode 3 で寝ていて何も聴かないので、
//   「同じ CH/SF/Group にもう 1 台 TX がいる」は受信機しか検出できなかった
//   （seq の逆行で判定。link.h 参照）。ここは送信機がそれを自分で確かめられる
//   唯一の機会になる。予備機の設定ミスで一番起こりやすく、一番まずい状態。
//
// ★★ 問題が見つかっても**送信は止めない。** 予備機を緊急で TX に切り替えて
//   載せ替える場面で送信が始まらないほうが危険。E220 は ARIB のキャリアセンスが
//   必須実装なので、混雑していればモジュール側が自動で送信を待つ。
//
// 雑音は**中央値と最大値を分けて**見る。
//   中央値が高い          → ずっとうるさい（Wi-SUN 等）。チャンネルを変える
//   最大値だけ突出している → 断続的に誰かが出ている。別 SF や他方式で
//                            デコードできないので、この差でしか気づけない
static LinkPreflightState s_pfState  = LINK_PF_IDLE;
static uint8_t  s_pfFlags   = LINK_PFF_OK;
static uint32_t s_pfStartMs = 0;
static uint32_t s_pfLastSampleMs = 0;
static uint32_t s_pfDoneMs  = 0;
static uint8_t  s_pfOtherGrp = 0;
// 見つけた他機の RSSI の最大値。**近いのか遠いのかで対処が変わる**ので残す。
// −40dBm なら隣で出ている（すぐ直す）、−110dBm なら遠くの誰か（様子見でよい）。
static int8_t   s_pfPonsRssi = -128;
static int16_t  s_pfNoiseMed = 0;
#define PF_MAX_SAMPLES  ((LINK_PF_WINDOW_MS / LINK_PF_SAMPLE_MS) + 2)
static int16_t  s_pfNoise[PF_MAX_SAMPLES];
static uint8_t  s_pfNoiseN = 0;
// 雑音の取得が連続で失敗した回数。**途中で打ち切るために数える**（下の tick 参照）。
static uint8_t  s_pfNoiseFail = 0;
static bool     s_pfNoiseGiveUp = false;  // 雑音の測定だけ諦めた（聴取は続ける）

// WIRELESS 画面から手で走らせ直す。
// ★ ブリングアップでは「アンテナを動かして測り直す」を何度もやる。
//   従来は CH を変えて戻す、という遠回りしか手段が無かった。
//   TX 以外では走らないので、呼べたかどうかを返して呼び出し側に知らせる。
bool link_preflight_restart() {
    if (link_get_mode() != LINK_MODE_TX) return false;
    link_preflight_start();
    return true;
}

LinkPreflightState link_preflight_state() { return s_pfState; }
uint8_t  link_preflight_flags()       { return (s_pfState == LINK_PF_DONE) ? s_pfFlags : 0; }
uint8_t  link_preflight_other_group() { return s_pfOtherGrp; }
int8_t   link_preflight_pons_rssi()   { return (s_pfPonsRssi > -128) ? s_pfPonsRssi : 0; }
int16_t  link_preflight_noise_med()   { return s_pfNoiseMed; }
uint32_t link_preflight_remain_ms() {
    if (s_pfState != LINK_PF_RUNNING) return 0;
    const uint32_t el = millis() - s_pfStartMs;
    return (el >= LINK_PF_WINDOW_MS) ? 0 : (LINK_PF_WINDOW_MS - el);
}
// 警告を出してからの経過。地図のポップアップを引っ込める判断に使う。
uint32_t link_preflight_since_done_ms() {
    return (s_pfState == LINK_PF_DONE) ? (millis() - s_pfDoneMs) : 0xFFFFFFFFu;
}

// チェックを開始する。TX 以外では何もしない。
// 呼ばれるのは「起動時に TX だった」「TX に切り替えた」「TX 中に CH/SF を変えた」
// 「WIRELESS 画面から手で再実行した（link_preflight_restart）」の 4 つ。
static void link_preflight_start() {
    if (link_mode_setting != LINK_MODE_TX) { s_pfState = LINK_PF_IDLE; return; }

    // ★ モジュールが応答しないなら**実行しない**。e220_read_noise() は応答待ちで
    //   最大 200ms ブロックするので、無応答のまま 10 回呼ぶと Core0 が 2 秒止まる。
    //   生きているときの応答は数 ms なので、ここで弾けば実害が消える。
    //   無応答そのものは link_watch_module() が既に鳴らしている。
    if (!e220_alive()) {
        s_pfFlags = LINK_PFF_SKIPPED;
        s_pfState = LINK_PF_DONE;
        s_pfDoneMs = millis();
        return;
    }
    e220_wake();                 // 監視の間は mode 0 に置く（聴くため）
    s_pfState        = LINK_PF_RUNNING;
    s_pfFlags        = LINK_PFF_OK;
    s_pfStartMs      = millis();
    // ★ **起床直後に即サンプルしない。** 0 にすると最初のループで即座に叩くが、
    //   e220_wake() で mode 3 から戻った直後はモジュールが落ち着いておらず、
    //   実機ではここが必ず失敗していた（AUX が High でも数百 ms 効かない）。
    //   今の時刻を入れておけば 1 回目は LINK_PF_FIRST_SAMPLE_MS 後になる。
    //   窓 3000ms / 1 回目 500ms / 以降 300ms なので、サンプルは 9 個取れる。
    s_pfLastSampleMs = millis();
    s_pfNoiseN       = 0;
    s_pfNoiseFail    = 0;
    s_pfNoiseGiveUp  = false;
    s_pfNoiseMed     = 0;
    s_pfOtherGrp     = 0;
    s_pfPonsRssi     = -128;
}

static void link_preflight_finish() {
    // ---- 雑音の中央値と最大値 ----
    int16_t med = 0, mx = -128;
    if (s_pfNoiseN > 0) {
        int16_t tmp[PF_MAX_SAMPLES];
        for (uint8_t i = 0; i < s_pfNoiseN; i++) tmp[i] = s_pfNoise[i];
        for (uint8_t i = 1; i < s_pfNoiseN; i++) {       // 挿入ソート（最大 12 個）
            const int16_t k = tmp[i];
            int8_t j = (int8_t)i - 1;
            while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
            tmp[j + 1] = k;
        }
        med = tmp[s_pfNoiseN / 2];
        mx  = tmp[s_pfNoiseN - 1];
        s_pfNoiseMed = med;
        if (med > LINK_PF_NOISE_WARN_DBM)      s_pfFlags |= LINK_PFF_NOISY;
        if ((mx - med) > LINK_PF_BURST_DB)     s_pfFlags |= LINK_PFF_BURST;
    } else {
        // ★ モジュールは応答しているのに雑音を 1 つも取れなかった。
        //   「静かだった」ではなく「測れなかった」なので、合格にしてはいけない。
        //   REG3 の RSSI バイトが無効、または固定送信モードになっている疑い
        //   （docs/pons_link.md §6。無線設定画面の Noise が `---` になる症状と同じ）。
        s_pfFlags |= LINK_PFF_SKIPPED;
    }

    e220_sleep();                // 通常運転（送信の合間はスリープ）へ戻す
    s_pfState  = LINK_PF_DONE;
    s_pfDoneMs = millis();

    // ---- 知らせる ----
    if (s_pfFlags == LINK_PFF_OK) {
        // 合格。**点検が走って通ったこと**が分かるよう短い上昇 2 音を鳴らす
        //（較正完了と同じ流儀）。無音にすると「点検が走ったのか」が分からない。
        enqueueTask(createPlayMultiToneTask(1568, 60, 1, 2));
        enqueueTask(createPlayMultiToneTask(2093, 90, 1, 2));
    } else if (s_pfFlags & (LINK_PFF_PONS_SAME | LINK_PFF_PONS_OTHER)) {
        // 同じ CH/SF の PONS を受けた（送信機が 2 台／他チーム）。
        // 対処は「設定を確認する／他チームと調整する」。
        link_alert("wav/link_ch_busy.wav", 294, 350, 3);
    } else if (s_pfFlags & LINK_PFF_BURST) {
        // ★ 解読できない電波が断続的に出ている。**同じチャンネルとは限らない。**
        //   帯域が重なる隣の CH の PONS や、至近距離なら重ならない CH の PONS でも出る
        //   （2026-10-22 の実測。CH9 の送信機で CH10 −28dBm / CH12 −62dBm）。
        //   以前は link_ch_busy.wav（「同じチャンネルに他に送信機がいます」）を鳴らしていて、
        //   隣のチームが別 CH で送っているだけなのに自分の設定を疑わせていた。
        link_alert("wav/wireless_busy.wav", 294, 350, 3);
    } else if (s_pfFlags & LINK_PFF_NOISY) {
        // 雑音が高いだけ。対処は「チャンネルを変える」。音を分けてあるのはこのため。
        link_alert("wav/link_ch_noisy.wav", 440, 250, 3);
    } else {
        // 測れなかった（モジュール無応答、または RSSI バイトが無効）。
        // ★ 無音にしてはいけない。合格の上昇 2 音が鳴らないだけだと
        //   「点検が走ったが通らなかった」のか「そもそも走っていない」のか
        //   区別できない。低い 2 音で「判定できなかった」と分かるようにする。
        enqueueTask(createPlayMultiToneTask(392, 120, 2, 2));
    }
    enqueueTask(createLogSdfTask(
        "LINK PREFLIGHT: ch=%u sf=%u noise med=%d max=%d n=%u pons=%d grp=%u -> 0x%02X",
        link_radio_ch, e220_profile_to_sf(link_radio_profile),
        (int)med, (int)((s_pfNoiseN > 0) ? mx : 0), (unsigned)s_pfNoiseN,
        (int)((s_pfPonsRssi > -128) ? s_pfPonsRssi : 0), (unsigned)s_pfOtherGrp,
        (unsigned)s_pfFlags));
}

static void link_preflight_tick() {
    if (s_pfState != LINK_PF_RUNNING) return;
    // モードを抜けた（RX / OFF にされた）ら中止する。
    // ★ 中止でもモジュールの状態を放置しないこと。点検のために起こしてあるので、
    //   OFF に落とされたのに mode 0 のままだと 8.2mA を食い続ける。
    //   通常は link_set_mode() が先に IDLE にするのでここは通らないが、
    //   通った場合に電力が残る作りにはしない。
    if (link_mode_setting != LINK_MODE_TX) {
        s_pfState = LINK_PF_IDLE;
        if (link_mode_setting != LINK_MODE_RX) e220_sleep();   // 受信機は起きたままが正
        return;
    }

    // ---- 他の送信機を数える ----
    // ★ 受けたフレームを s_rx に入れないこと。入れると **送信機がミラー表示を
    //   始めてしまう**。received/ にも書かない（ここは自機の飛行前点検であって
    //   受信ログではない）。数えるだけにする。
    LinkTelem t;
    int16_t   rssi = 0;
    while (link_recv_telem(&t, &rssi, false, nullptr)) {
        const int8_t r = (int8_t)constrain((int)rssi, -128, 127);
        if (r > s_pfPonsRssi) s_pfPonsRssi = r;
        if (t.group == link_group) {
            s_pfFlags |= LINK_PFF_PONS_SAME;    // 同じ CH/SF/Group に送信機が 2 台
        } else {
            s_pfFlags |= LINK_PFF_PONS_OTHER;
            s_pfOtherGrp = t.group;
        }
    }

    // ---- 雑音のサンプル ----
    const uint32_t now = millis();
    // ★ 1 回目だけ待ちを長くする（理由は settings.h の LINK_PF_FIRST_SAMPLE_MS）。
    const uint32_t sample_due = (s_pfNoiseN == 0 && s_pfNoiseFail == 0)
                                  ? LINK_PF_FIRST_SAMPLE_MS : LINK_PF_SAMPLE_MS;
    if (!s_pfNoiseGiveUp && (now - s_pfLastSampleMs) >= sample_due) {
        s_pfLastSampleMs = now;
        int16_t n = 0;
        if (e220_read_noise(&n, nullptr)) {
            s_pfNoiseFail = 0;
            if (s_pfNoiseN < PF_MAX_SAMPLES) s_pfNoise[s_pfNoiseN++] = n;
        } else if (++s_pfNoiseFail >= LINK_PF_NOISE_FAIL_MAX) {
            // ★★ 連続で返事が無いなら、窓の最後まで回さずにここで打ち切る。
            //   1 回の問い合わせは応答待ちでブロックするので、10 回ぶん繰り返すと
            //   Core0 が長時間止まり、GNSS の受信や描画が巻き添えになって
            //   NAV-PVT を取りこぼす。地図もボタンもバリオも同時に止まる。
            //   ★ e220_alive() の事前チェックだけでは塞げない。あれは起動時の
            //     照合結果なので、**起動後に抜けたコネクタでは true のまま**。
            //     実際に叩いてみて初めて分かるので、叩く回数のほうを縛る。
            //   ★ 打ち切ったときは **合格にしない**（SKIPPED を先に立てる）。
            //     途中まで取れた値から NOISY / BURST を出すのは構わないが、
            //     「静かだった」ではなく「最後まで測れなかった」ので、
            //     OK の上昇 2 音を鳴らしてはいけない。
            //   ★ ただし **点検そのものは打ち切らない。** 雑音が測れないことと、
            //     同じチャンネルに他の送信機がいるかどうかは別の話で、
            //     後者のほうが重い（受信機側の seq 逆行検出では見えなかったもの）。
            //     叩くのをやめるだけにして、窓の最後まで聴き続ける。
            s_pfFlags      |= LINK_PFF_SKIPPED;
            s_pfNoiseGiveUp = true;
        }
    }

    if ((now - s_pfStartMs) >= LINK_PF_WINDOW_MS) link_preflight_finish();
}

// 送信機の刻み（1Hz）。**時計を持つのは RP2350 側**。
// E220 は自分から催促してこないので、こちらが叩きに行く。
//   ・送信直前にテレメトリを組み立てるので、値は常に新しい
//   ・送り終えたら即 mode 3 へ落とす（高速スリープ。AUX を待たなくてよい）
// 1Hz の定期送信。**Core0 を止めない。**
//
// ★ 以前は 1 回の呼び出しで「起こす → 送る → 寝かす」を全部やっていた。
//   そのうち e220_wake() の wait_aux_idle(50) と e220_sleep() の flush() が
//   **毎秒 Core0 を止めていた**（最悪で合わせて 60ms 前後）。待っている間は
//   描画も GNSS もセンサー更新も進まない。
//   3 手に分けて、次の周回で様子を見るようにすれば、どこでも待たずに済む。
//
//   LTX_IDLE     1 秒経ったら e220_wake_begin()（mode 0 にするだけ）→ WAKING
//   LTX_WAKING   起床完了（AUX の Low を見てから High）で送る → TXSTART
//                （上がらないまま LINK_TX_WAKE_GIVEUP_MS 経ったらスロットを捨てる）
//   LTX_TXSTART  AUX が Low へ落ちる＝モジュールが動き出した → TXDONE
//                （落ちないまま LINK_TX_TXSTART_MS 経ったら noRf。捨てられている）
//   LTX_TXDONE   AUX が High へ戻る＝**送信完了**。そこで mode 3 へ → IDLE
//
// ★★ **送信完了まで待つ（0.977 で変更）。** 以前は「送信中なら mode 3 への切替は
//   ハードが先送りする（データシート 5.3）」を根拠に、AUX が落ちた時点で
//   すぐ寝かせていた。ところが実機 2026-09-27 に、送信側 noRf=0（AUX は落ちている）
//   なのに**受信側が 93% 落とす**状態を観測した（bad=0・RSSI -46〜-64dBm、
//   つまり届けば確実に受かる距離）。**送信を始める前**に落としたときの動作は
//   保証が無いので、完了を見るまで待つことにした。
//   副産物として**実測の送信時間**が取れる（60 秒ログの airmax=）。
// ★ 代償: モジュールが mode 0 に居る時間が送信時間ぶん伸びる（SF9 で約 200ms）。
//   受信待機 8.2mA なので平均 +1.6mA 程度。TFT と RP2350 の消費に対しては誤差。
// （LinkTxState と s_txState/s_txStateMs は link_probe_module から参照できるようファイル上部へ配置）

static void link_tx_tick() {
    if (link_mode_setting != LINK_MODE_TX) { s_txState = LTX_IDLE; return; }

    // ★★ **リプレイ中とデモ中は送信しない**（判定は link_rf_suppressed()）。
    //   apply_replay_row()（gnss.cpp）は stored_latitude / stored_fixtype など
    //   **実センサーの格納変数そのものを上書きする**ので、そのまま送ると
    //   **過去の飛行を「いまの飛行」としてボートへ流す**ことになる。
    //   fixflags も立つため、受信側からは正常な現在位置と見分けがつかない。
    //   これは docs/pons_link.md が一貫して避けている壊れ方（画面上は正常なまま
    //   間違った値が出る）そのもの。地上での確認作業で電波を出す理由も無い。
    //   ★ ボート側は 10 秒後に NO SIGNAL になるが、それが「送っていない」という
    //     正しい表示。リプレイをやめれば次のスロットから復帰する。
    //   ★★ **デモも同じ理由で送らない。** デモは stored_* を上書きしないので
    //     フライト CSV には残らないが、link_push_telemetry() は get_gnss_lat() など
    //     **accessor 経由**で詰めるため、仮想機体の位置・速度・方位・衛星数は
    //     そのまま電波に乗る。しかも fixflags だけは実 GNSS 由来なので、
    //     **屋外で fix がある機体がデモを出すとボートには「本物の飛行」として届く**
    //     （機械は陸に置いたまま、画面上は琵琶湖を飛んでいる）。
    //     **記録に残らないものだけが電波に出る**という、一番たちの悪い非対称になる。
    //   ★ 途中状態なら**寝かせてから**畳む。mode 0 に置いたままにすると
    //     8.2mA を食い続ける。
    if (link_rf_suppressed()) {
        if (s_txState != LTX_IDLE) { e220_sleep(); s_txState = LTX_IDLE; }
        return;
    }
    // ★ 点検中は送らない。自分の送信を測ってしまうと雑音の判定が意味を失う。
    //   ★ 状態も畳む。点検は e220_wake() で mode 0 に置くので、
    //     途中状態のまま入ると寝かせ忘れる。
    if (s_pfState == LINK_PF_RUNNING) { s_txState = LTX_IDLE; return; }

    const uint32_t now = millis();

    switch (s_txState) {
    case LTX_WAKING:
        if (e220_wake_ready()) {
            link_push_telemetry();      // 中で e220_send() する
            s_txState   = LTX_TXSTART;
            s_txStateMs = now;
        } else if (now - s_txStateMs >= LINK_TX_WAKE_GIVEUP_MS) {
            // 起きてこない。このスロットは捨てて寝かせ直す。
            // 無応答そのものは link_watch_module() が見ているので、ここでは鳴らさない。
            e220_sleep();
            s_txState = LTX_IDLE;
        }
        return;

    case LTX_TXSTART:
        // ★ AUX が Low に落ちる＝モジュールが動き出した。**まだ寝かせない。**
        if (digitalRead(E220_AUX_PIN) == LOW) {
            s_txState   = LTX_TXDONE;
            s_txStateMs = now;
            return;
        }
        if (now - s_txStateMs >= LINK_TX_TXSTART_MS) {
            // ★ 書いたのに AUX が一度も落ちない＝**モジュールに捨てられている。**
            //   ここを数えていなかったので、Sent だけ見て「送信できている」と
            //   誤解していた（実機 2026-09-27）。
            s_txNoRf++;
            enqueueTask(createLogSdfTask("LINK TX: no RF (AUX stayed HIGH %lums) noRf=%lu",
                                         (unsigned long)(now - s_txStateMs),
                                         (unsigned long)s_txNoRf));
            e220_sleep();
            s_txState = LTX_IDLE;
        }
        return;

    case LTX_TXDONE: {
        // ★★ **AUX が High へ戻る＝送信完了。そこまで待ってから mode 3 へ落とす。**
        //   理由は上の typedef のコメント。
        if (digitalRead(E220_AUX_PIN) != LOW) {
            const uint32_t el = now - s_txStateMs;
            if (el > s_txAirMaxMs) s_txAirMaxMs = el;   // 実測の送信時間（最大）
            { static uint8_t left = 3;                  // 最初の 3 回だけ残す
              if (left) { left--;
                  enqueueTask(createLogSdfTask("LINK TX: air %lums (prof=%u)",
                                               (unsigned long)el, link_radio_profile)); } }
            e220_sleep();
            s_txState = LTX_IDLE;
            return;
        }
        // ★ 送信完了タイムアウト: ローワーは休止50ms固定なので LINK_TX_DONE_MS(1000ms)。
        //   アッパーバンドは電波法(ARIB STD-T108)で送信時間の10倍休止をモジュールが
        //   強制するため AUX Low が 1000ms を超える。送信周期 link_tx_period_ms() まで待つ。
        const uint32_t done_timeout = link_radio_ch_upper(link_radio_ch)
                                    ? link_tx_period_ms()
                                    : LINK_TX_DONE_MS;
        if (now - s_txStateMs >= done_timeout) {
            // 上限。ここに来たら送信時間の想定が違う（または AUX が張り付いている）。
            s_txAirTimeout++;
            enqueueTask(createLogSdfTask("LINK TX: air TIMEOUT (%lums) n=%lu",
                                         (unsigned long)(now - s_txStateMs),
                                         (unsigned long)s_txAirTimeout));
            e220_sleep();
            s_txState = LTX_IDLE;
        }
        return;
    }

    case LTX_IDLE:
    default:
        break;
    }

    static uint32_t lastTx = 0;
    if (now - lastTx < link_tx_period_ms()) return;   // ローワーは 1 秒。アッパーは休止の分だけ長い
    lastTx = now;
    e220_wake_begin();                  // mode 0 にするだけ。**AUX は待たない**
    s_txState   = LTX_WAKING;
    s_txStateMs = now;
}

// ============================================================
//  チャンネル調査（WIRELESS 画面の Ch scan）
// ============================================================
// 選べるチャンネル（link_proto.h の LINK_RADIO_CHS。ローワーの CH0 / 3 / 6 / 9 / 12 と、アッパーの
// CH15 / 21 / 27）を順に聴いて、雑音と他の PONS を一覧にする。
// 見るものは送信前チェックと同じ（雑音の中央値・最大値、解読できた PONS）。違いは
// 「選んだ 1 チャンネル」ではなく「選べる全部」を聴いて、選ぶ材料にすること。
//   SF11 のときはアッパーでは使えないので、一覧で灰色にして「一番静か」の候補から外す（雑音は測る）。
//
// ★ チャンネルは RAM にだけ書く（e220_reconfigure の persist=false。0xC2）。不揮発には
//   書かないので、何周回してもモジュールの書換え寿命は減らない。終わったら元の値を書き戻す。
// ★ 切り替えのたびに Core0 が最大 0.6 秒止まる（e220_reconfigure）。WIRELESS 画面から
//   人が始めるときだけ走るので、飛行中には起きない。
// ★ 調べている間は link_loop() が受信の取り込み・生存確認・送信前チェック・送信を止める。
//   別のチャンネルで送ってしまったり、調査の問い合わせに割り込んだりしないため。
//   受信機は「無信号」の時間（ローワーで 10 秒）受けないと「途切れた」になるので、知らせる音は止めてある
//   （link_watch_mirror_transition）。送信機はこの間ボートへ何も送らない。
static LinkScanState s_scanState = LINK_SCAN_IDLE;
static uint8_t  s_scanIdx    = 0;       // いま測っているチャンネルの番号（0..LINK_SCAN_CH_COUNT-1）
static uint8_t  s_scanSweep  = 0;       // 何周目か（0 始まり）
static uint32_t s_scanChMs   = 0;       // いまのチャンネルへ切り替え終えた時刻
static uint32_t s_scanLastSampleMs = 0;
static uint32_t s_scanEndMs  = 0;       // 終えた（止めた）時刻
static uint8_t  s_scanDwellN = 0;       // いまのチャンネルで取れた雑音の数（1 回目の待ちの判定用）
static uint8_t  s_scanFail   = 0;       // 雑音の問い合わせが続けて失敗した回数
static bool     s_scanGiveUp = false;   // このチャンネルでは雑音を諦めた（聴くのは続ける）
// チャンネルごとの雑音（新しい LINK_SCAN_MAX_SAMPLES 個を輪にして持つ）と、解読できた PONS
static int8_t   s_scanNoise[LINK_SCAN_CH_COUNT][LINK_SCAN_MAX_SAMPLES];
static uint8_t  s_scanN   [LINK_SCAN_CH_COUNT];
static uint8_t  s_scanHead[LINK_SCAN_CH_COUNT];
static uint16_t s_scanPonsSame [LINK_SCAN_CH_COUNT];
static uint16_t s_scanPonsOther[LINK_SCAN_CH_COUNT];
static int8_t   s_scanPonsRssi [LINK_SCAN_CH_COUNT];

// 一覧の idx → チャンネル。選べる CH の小さい順（先にローワー 5 つ、続けてアッパー 3 つ）。
static uint8_t link_chscan_ch(uint8_t idx) { return LINK_RADIO_CHS[idx]; }

static bool link_chscan_holds_radio() {
    if (s_scanState == LINK_SCAN_RUNNING) return true;
    // ★ 待つ時間は送る間隔に比例させる（アッパーの SF9 以上は次のフレームまで 1 秒より長い）
    return (s_scanState != LINK_SCAN_IDLE) &&
           (millis() - s_scanEndMs) < LINK_SCAN_RESUME_GRACE_MS * link_tx_period_ms() / 1000;
}

static void link_chscan_enter(uint8_t idx) {
    s_scanIdx = idx;
    // ★ 調べている間は送信を止めている（link_loop）。
    // ★ アッパーの CH は、SF11 なら SF10 で聴く（使わない組み合わせをモジュールに書かない。
    //   雑音の値は SF に依らず、SF11 のアッパーの PONS はいないので、数える PONS も変わらない）。
    const uint8_t ch   = link_chscan_ch(idx);
    const uint8_t prof = link_radio_profile_allowed(ch, link_radio_profile) ? link_radio_profile
                                                                            : (uint8_t)LINK_UPPER_PROF_MAX;
    e220_reconfigure(ch, prof, false);
    // 切り替えに掛かった時間（最大 0.6 秒）は聴く時間に数えない
    s_scanChMs = s_scanLastSampleMs = millis();
    s_scanDwellN = 0;
    s_scanFail   = 0;
    s_scanGiveUp = false;
}

bool link_chscan_row(uint8_t idx, LinkScanRow* out) {
    if (idx >= LINK_SCAN_CH_COUNT || !out) return false;
    LinkScanRow& r = *out;
    r.ch         = link_chscan_ch(idx);
    r.upper      = link_radio_ch_upper(r.ch);
    r.usable     = link_radio_profile_allowed(r.ch, link_radio_profile);
    r.n          = s_scanN[idx];
    r.pons_same  = s_scanPonsSame[idx];
    r.pons_other = s_scanPonsOther[idx];
    r.pons_rssi  = (s_scanPonsRssi[idx] > -128) ? s_scanPonsRssi[idx] : 0;
    r.measuring  = (s_scanState == LINK_SCAN_RUNNING) && (s_scanIdx == idx);
    r.med = r.max = 0;
    r.burst = false;
    if (r.n == 0) return true;
    int8_t tmp[LINK_SCAN_MAX_SAMPLES];
    for (uint8_t i = 0; i < r.n; i++) tmp[i] = s_scanNoise[idx][i];
    for (uint8_t i = 1; i < r.n; i++) {          // 挿入ソート（最大 LINK_SCAN_MAX_SAMPLES 個）
        const int8_t k = tmp[i];
        int j = (int)i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    r.med   = tmp[r.n / 2];
    r.max   = tmp[r.n - 1];
    r.burst = (r.max - r.med) > LINK_PF_BURST_DB;
    return true;
}

// band: 0 = ローワーだけ、1 = アッパーだけ、2 = 両方
static int8_t link_chscan_best_in(uint8_t band) {
    int8_t best = -1;
    LinkScanRow b = {}, r;
    for (uint8_t i = 0; i < LINK_SCAN_CH_COUNT; i++) {
        link_chscan_row(i, &r);
        if (band != 2 && r.upper != (band == 1)) continue;
        if (!r.usable || r.n == 0 || r.pons_other > 0) continue;
        // ★ 自チームの送信機（同じ Group）がいるチャンネルの断続的な電波は、その送信機のもの。
        //   受信機で調べると自分の CH にいつも出るので、それで外すと今の CH が選べなくなる。
        if (r.burst && r.pons_same == 0) continue;
        if (best < 0 || r.med < b.med || (r.med == b.med && r.max < b.max)) { best = (int8_t)i; b = r; }
    }
    return best;
}
int8_t link_chscan_best()                { return link_chscan_best_in(2); }
int8_t link_chscan_best_band(bool upper) { return link_chscan_best_in(upper ? 1 : 0); }

static void link_chscan_finish(bool complete) {
    // 元のチャンネルと電源状態へ戻す（調査で動かしたのは RAM の設定だけ）。
    // ★ link_apply_power_state() を必ず通す。e220_reconfigure() は mode 0 で返るので、
    //   通さないと送信機・OFF でもモジュールが起きたままになる（同関数のコメント参照）。
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
    link_apply_power_state();
    s_txState   = LTX_IDLE;
    s_scanState = complete ? LINK_SCAN_DONE : LINK_SCAN_STOPPED;
    s_scanEndMs = millis();

    LinkScanRow r;
    for (uint8_t i = 0; i < LINK_SCAN_CH_COUNT; i++) {
        link_chscan_row(i, &r);
        enqueueTask(createLogSdfTask(
            "LINK CHSCAN ch=%u noise med=%d max=%d n=%u pons=%u/%u rssi=%d%s%s",
            r.ch, r.med, r.max, r.n, r.pons_same, r.pons_other, r.pons_rssi,
            r.burst ? " BURST" : "", r.upper ? " UPPER" : ""));
    }
    const int8_t best = link_chscan_best();
    const int8_t bl   = link_chscan_best_band(false);
    const int8_t bu   = link_chscan_best_band(true);
    enqueueTask(createLogSdfTask("LINK CHSCAN %s sweeps=%u best=%d lower=%d upper=%d now=%u sf=%u",
                                 complete ? "done" : "stopped", (unsigned)(s_scanSweep + 1),
                                 best >= 0 ? link_chscan_ch((uint8_t)best) : -1,
                                 bl >= 0 ? link_chscan_ch((uint8_t)bl) : -1,
                                 bu >= 0 ? link_chscan_ch((uint8_t)bu) : -1, link_radio_ch,
                                 e220_profile_to_sf(link_radio_profile)));
    // 終わったことを 2 回の短い音で知らせる（送信前チェックの合格音とは別の音）
    if (complete) enqueueTask(createPlayMultiToneTask(1568, 100, 2, 2));
}

bool link_chscan_start() {
    if (s_scanState == LINK_SCAN_RUNNING || !e220_alive()) return false;
    for (uint8_t i = 0; i < LINK_SCAN_CH_COUNT; i++) {
        s_scanN[i] = s_scanHead[i] = 0;
        s_scanPonsSame[i] = s_scanPonsOther[i] = 0;
        s_scanPonsRssi[i] = -128;
    }
    // 送信前チェックの途中なら打ち切る（調査がモジュールを使う）。送信も最初からやり直す。
    if (s_pfState == LINK_PF_RUNNING) s_pfState = LINK_PF_IDLE;
    s_txState   = LTX_IDLE;
    s_scanSweep = 0;
    s_scanState = LINK_SCAN_RUNNING;
    enqueueTask(createLogSdfTask("LINK CHSCAN start mode=%u ch=%u sf=%u",
                                 link_mode_setting, link_radio_ch,
                                 e220_profile_to_sf(link_radio_profile)));
    link_chscan_enter(0);
    return true;
}

void link_chscan_stop() {
    if (s_scanState == LINK_SCAN_RUNNING) link_chscan_finish(false);
}

bool          link_chscan_running() { return s_scanState == LINK_SCAN_RUNNING; }
LinkScanState link_chscan_state()   { return s_scanState; }
uint8_t       link_chscan_sweep()   { return (uint8_t)(s_scanSweep + 1); }

static void link_chscan_tick() {
    // モジュールが応答しなくなったら止める（無応答は link_watch_module() が知らせる）
    if (!e220_alive()) { link_chscan_finish(false); return; }

    // ---- 他の PONS を数える ----
    // ★ 受けたフレームを s_rx に入れないこと（送信前チェックと同じ理由。ミラーや受信ログに混ぜない）。
    //   核だけ取り出す（link_recv_telem の merge_own=false。group と seq が分かれば足りる）。
    const uint8_t k = s_scanIdx;
    LinkTelem t;
    int16_t   rssi = 0;
    while (link_recv_telem(&t, &rssi, false, nullptr)) {
        const int8_t r = (int8_t)constrain((int)rssi, -128, 127);
        if (r > s_scanPonsRssi[k]) s_scanPonsRssi[k] = r;
        uint16_t& cnt = (t.group == link_group) ? s_scanPonsSame[k] : s_scanPonsOther[k];
        if (cnt < 0xFFFF) cnt++;
    }

    // ---- 雑音のサンプル ----
    // ★ 切り替えの直後は待つ（LINK_PF_FIRST_SAMPLE_MS と同じ理由。早すぎると必ず失敗する）。
    const uint32_t now = millis();
    const uint32_t due = (s_scanDwellN == 0 && s_scanFail == 0)
                           ? LINK_SCAN_FIRST_SAMPLE_MS : LINK_SCAN_SAMPLE_MS;
    if (!s_scanGiveUp && (now - s_scanLastSampleMs) >= due) {
        s_scanLastSampleMs = now;
        int16_t n = 0;
        if (e220_read_noise(&n, nullptr)) {
            s_scanFail = 0;
            s_scanDwellN++;
            s_scanNoise[k][s_scanHead[k]] = (int8_t)constrain((int)n, -128, 127);
            s_scanHead[k] = (uint8_t)((s_scanHead[k] + 1) % LINK_SCAN_MAX_SAMPLES);
            if (s_scanN[k] < LINK_SCAN_MAX_SAMPLES) s_scanN[k]++;
        } else if (++s_scanFail >= LINK_PF_NOISE_FAIL_MAX) {
            s_scanGiveUp = true;      // 叩くのはやめる（送信前チェックと同じ。Core0 を止めすぎない）
        }
    }

    // ---- 次のチャンネルへ ----
    if ((now - s_scanChMs) >= LINK_SCAN_DWELL_MS) {
        if (s_scanIdx + 1 < LINK_SCAN_CH_COUNT) { link_chscan_enter((uint8_t)(s_scanIdx + 1)); return; }
        if (s_scanSweep + 1 < LINK_SCAN_SWEEPS) { s_scanSweep++; link_chscan_enter(0); return; }
        link_chscan_finish(true);
    }
}

void link_loop() {
    // ★ チャンネル調査の間は、モジュールのチャンネルを調査が動かしている。
    //   受信の取り込み・生存確認・送信前チェック・送信はすべて止める（上の節を参照）。
    //   モジュールの無応答だけは見続ける。
    if (s_scanState == LINK_SCAN_RUNNING) {
        link_chscan_tick();
        link_watch_module();
        return;
    }
    link_poll_radio();
    link_watch_mirror_transition();
    link_probe_module();        // ★ watch_module より先。判断の材料をここで更新する
    link_watch_module();
    link_sync_destination();
    link_sync_scale();          // 目的地と同じ「送信側に合わせる」組
    // ★ 下の 3 つと link_watch_module() は **リプレイ中・デモ中も鳴らす**。
    //   「送信機の電池が減っている」「地上チェックに引っかかっている」
    //   「自分のモジュールが応答しない」は、画面に出している数字とは無関係に
    //   **いま現実に起きている事実**なので、再生のために黙らせる理由が無い
    //   （画面の飾りだけを止める。link.h 参照）。
    link_watch_sender_faults();
    link_watch_sender_battery();
    link_watch_sender_power();
    link_watch_sender_ground_att();
    link_watch_sender_gnss();
    link_periodic_report();
    link_preflight_tick();      // ★ tx_tick より先。点検中は送信を止める
    link_tx_tick();
}

// ============================================================
//  設定
// ============================================================
// モードに応じた省電力状態をモジュールへ反映する。
//   受信機 : 常時 mode 0（起きたまま。docs/pons_link.md §3）
//   送信機 : 送信の合間は mode 3（送信直前に link_tx_tick() が起こす）
//   OFF    : mode 3。起こしたままだと 8.2mA を食い続けるうえ、
//            誰も読まない受信データで UART の FIFO を埋め続ける
//
// ★ **e220_reconfigure() は必ず mode 0（起床）で返る。**
//   CH / SF を書き換えたあとは必ずここを通すこと。通さないと
//   「OFF のまま CH を変えたら無線が起きっぱなしになる」という
//   気づきにくい電池消費（本体 72mA に対し 11%・電池で約 1 時間ぶん）が残る。
//   WIRELESS 画面での自然な操作順は「CH を決める → Group を決める →
//   最後に Mode を切り替える」なので、OFF のまま CH を触るのは普通の流れ。
static void link_apply_power_state() {
    if (link_mode_setting == LINK_MODE_RX) e220_wake();
    else                                   e220_sleep();
}

void link_set_mode(uint8_t mode) {
    if (mode > LINK_MODE_RX) mode = LINK_MODE_OFF;
    if (mode == link_mode_setting) return;
    link_mode_setting = mode;
    // ★ 変わった役割をその場で読み上げる（起動時と同じ流儀）。機体を誤って受信モードに
    //   すると送信が止まるが、症状はボート側の「無信号」だけで自機では分からない。
    //   **3 状態すべて読み上げる。OFF を無音に戻さないこと**（docs/pons_link.md §1）。
    // ★ 音量は起動時と揃える（優先度 2・最低音量の強制なし）。
    //   意図して消音している機体を、モードを回すたびに鳴らさないため。
    // ★ WAV_EXCL_LINKMODE を必ず付ける。回転式で RECEIVER へ行くには SENDER を通るので、
    //   付けないと通過した読み上げが pending から復帰して**嘘のモードを喋る**（0.984 の不具合）。
    if      (mode == LINK_MODE_TX) enqueueTask(createPlayWavTask("wav/sender_mode.wav",   2, 0, WAV_EXCL_LINKMODE));
    else if (mode == LINK_MODE_RX) enqueueTask(createPlayWavTask("wav/receiver_mode.wav", 2, 0, WAV_EXCL_LINKMODE));
    else                           enqueueTask(createPlayWavTask("wav/link_off.wav",      2, 0, WAV_EXCL_LINKMODE));
    s_rxValid = false;                 // モードが変わったら受信データは無効
    // ★ MAX G/S は供給元が入れ替わるのでリセットする（ミラー中は機体の値を入れる）。
    //   残すと「ボート自身の最高速」と「機体の最高速」が混ざった値が出続ける。
    reset_maxgs();
    link_apply_power_state();
    // 送信モードに入ったら送り始める前にチャンネルを確かめる。
    // 抜けたときは IDLE に戻す（結果を残すと古い判定が画面に出続ける）。
    if (link_mode_setting == LINK_MODE_TX) link_preflight_start();
    else                                   s_pfState = LINK_PF_IDLE;
}
// その CH で使える SF か。アッパーでは LINK_UPPER_PROF_MAX（SF10）まで。
// ★ SF11 は 32 バイトでも送信時間の見積もり 360ms が 400ms の上限に近く、実際の送信時間が超えると
//   モジュールに捨てられる（noRf=）。危ないので、アッパーでは最初から選べないようにしてある。
bool link_radio_profile_allowed(uint8_t ch, uint8_t prof) {
    if (prof >= LINK_PROFILE_COUNT) return false;
    return !link_radio_ch_upper(ch) || prof <= LINK_UPPER_PROF_MAX;
}

// WIRELESS 画面の長押しで回す次の SF（ローワーは SF7 → … → SF11 → SF7、アッパーは SF10 で SF7 へ戻る）
uint8_t link_next_radio_profile(uint8_t prof) {
    for (uint8_t i = 0; i < LINK_PROFILE_COUNT; i++) {
        prof = (uint8_t)((prof + 1) % LINK_PROFILE_COUNT);
        if (link_radio_profile_allowed(link_radio_ch, prof)) return prof;
    }
    return 0;
}

// 送る間隔 [ms]。ローワーは 1 秒。
// ★ アッパーバンドは 1 回送るごとに送信時間の LINK_UPPER_PAUSE_X 倍の休止が要る
//   （docs/pons_link.md §2）。なので「送信時間 × (1 + 休止の倍率) × 余裕」より詰めない
//   （32 バイトなら SF7 は 1 秒のまま、SF8 で約 1.2 秒、SF9 で約 2.0 秒、SF10 で約 3.5 秒）。
//   モジュールも守るはずだが、こちらでも守る。1 時間 360 秒の上限もこれで必ず下回る。
uint32_t link_tx_period_ms() {
    if (!link_radio_ch_upper(link_radio_ch)) return 1000;
    const uint32_t need = (uint32_t)e220_airtime_ms(link_radio_profile)
                        * (1 + LINK_UPPER_PAUSE_X) * LINK_UPPER_TX_MARGIN_PCT / 100;
    return need > 1000 ? need : 1000;
}

// 値の範囲は E220 の制約（CH0-30 / SF7-11）で決まる。CH はそのうちローワーの 0/3/6/9/12 と、
// アッパーの 15/21/27 だけを使う（link_proto.h の LINK_RADIO_CHS）。アッパーでは SF10 まで。
// チャンネル/プロファイルの変更はモジュールへ書き直しが要る。
// mode 3 へ入って書き、読み返して照合し、mode 0 へ戻る（e220.cpp）。
// ★ この間 Core0 が最大 0.6 秒ほど止まる（9600 での往復とタイムアウト）。
//   設定画面はメニューからしか入れないので飛行中には起きないが、
//   **飛行中に呼んではいけない**関数である点は意識しておくこと。
// ★★ 書込みは **RAM のみ（0xC2）**。設定画面で CH を一周させるだけで
//   5 回（SF も回せばさらに）書くので、不揮発（0xC0）で書くとモジュールの書換え寿命を無駄に削る。
//   設定の正本は SD の settings.txt 側なので、次回起動時に e220_setup() が
//   同じ値を書き直す（そのときだけ不揮発）。
// ★ 電波の設定が変わったら送信前チェックをやり直す。前の CH で測った結果を
//   残しておくと、変えた先が混んでいても「OK」のままになる。
//   link_set_group() からは呼ばない（グループはペイロードの中身で、電波ではない）。
// ★ reconfigure のあとは必ず link_apply_power_state() を通す（理由は同関数のコメント）。
//   TX のときは直後の link_preflight_start() が改めて起こすので、
//   「寝かせる → 起こす」の往復になるが、どちらもピン 1 本の書き換えで済む。
void link_set_radio_ch(uint8_t ch) {
    // 選べる CH（0/3/6/9/12 と、アッパーの 15/21/27。link_proto.h）以外は受け付けない
    if (!link_radio_ch_valid(ch) || ch == link_radio_ch) return;
    link_radio_ch = ch;
    // ★ SF11 のままアッパーの CH へ来たら SF10 へ下げる。SF 行にそのまま出るので、
    //   全台で同じ操作をすれば同じ SF になる（WIRELESS 画面を出るときの「設定を変えた」の知らせにも入る）。
    if (!link_radio_profile_allowed(link_radio_ch, link_radio_profile)) {
        enqueueTask(createLogSdfTask("LINK sf %u -> %u (SF11 is not used on upper ch %u)",
                                     (unsigned)e220_profile_to_sf(link_radio_profile),
                                     (unsigned)e220_profile_to_sf(LINK_UPPER_PROF_MAX), (unsigned)ch));
        link_radio_profile = LINK_UPPER_PROF_MAX;
    }
    // CH が変わったら別のリンク。送る間隔の推定と、32 バイトの組み立て（回転枠の値・8 ビットから
    // 戻した seq の続き）はやり直す。
    s_txPeriodEstMs = link_tx_period_ms();
    link_w2_rx_reset(&s_w2rx);
    s_haveLastSeq = false;
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
    link_apply_power_state();
    link_preflight_start();
}
void link_set_radio_profile(uint8_t p) {
    if (!link_radio_profile_allowed(link_radio_ch, p) || p == link_radio_profile) return;
    link_radio_profile = p;
    s_txPeriodEstMs = link_tx_period_ms();   // アッパーでは SF で送る間隔が変わる
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
    link_apply_power_state();
    link_preflight_start();
}
// 別グループの送信機を直近 10 秒以内に受けたか。
// 「無信号」と「設定違い」を画面で区別するために使う。
bool    link_other_group_seen() {
    return s_otherGroup && (millis() - s_otherGroupMs) < 10000;
}
uint8_t link_other_group_id()    { return s_otherGroupId; }

void    link_set_group(uint8_t g) { link_group = g; }
uint8_t link_get_group()          { return link_group; }
// ============================================================
//  「設定を変えたまま伝え忘れ」対策
//    CH / SF / Group は **3 台すべてで一致していないと通信できない**のに、
//    1 台だけ変えて連絡を忘れると症状は「無信号」になる。
//    現場では距離不足や故障と区別が付かないので、変えた本人に知らせる。
//
//    ★ 送信機・受信機のどちらでも鳴らすこと。ここに mode の分岐を入れてはいけない。
//      受信機側を触って戻し忘れる事故も同じだけ起こるし、受信機は 2 台あるぶん
//      取り違えやすい。
// ============================================================
static uint8_t s_wlEntryCh = 0, s_wlEntryProf = 0, s_wlEntryGroup = 0;

void link_remember_setting() {
    s_wlEntryCh    = link_radio_ch;
    s_wlEntryProf  = link_radio_profile;
    s_wlEntryGroup = link_group;
}

void link_warn_setting_changed() {
    if (link_radio_ch      == s_wlEntryCh &&
        link_radio_profile == s_wlEntryProf &&
        link_group         == s_wlEntryGroup) return;
    // SD が無い機体でも必ず何か鳴る（link_alert の流儀）
    link_alert("wav/link_setting_changed.wav", 880, 200, 3);
    enqueueTask(createLogSdfTask("LINK setting changed: ch=%u prof=%u group=%u",
                                 link_radio_ch, link_radio_profile, link_group));
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
    t.group = link_group;
    // ★ seq は送信のたびに 1 つ進める。**埋め忘れると全パケットが seq=0 になり、
    //   受信側の取りこぼし計数と重複送信機の検出が両方壊れる。**
    t.seq   = s_seqOut++;

    uint16_t have = 0;

    GnssDate d = get_gnss_date();
    GnssTime tm = get_gnss_time();
    if (d.isValid()) { t.year = d.year(); t.month = d.month(); t.day = d.day(); have |= RHAVE_DATE; }
    if (tm.isValid()) { t.hour = tm.hour(); t.minute = tm.minute();
                        t.second = tm.second(); t.centi = tm.centisecond(); }

    t.lat_1e7 = (int32_t)lround(get_gnss_lat() * 1e7);
    t.lon_1e7 = (int32_t)lround(get_gnss_lon() * 1e7);
    t.gs_cms      = link_pack_u16((float)get_gnss_mps(),       100.0f, 65535);
    t.track_cdeg  = link_pack_u16((float)get_gnss_truetrack(), 100.0f, 36000);
    have |= RHAVE_GS | RHAVE_TTRACK;

    t.gnss_alt_dm = link_pack_i16((float)get_gnss_altitude(), 10.0f);
    have |= RHAVE_GNSSALT;

    t.kf_alt_dm = link_pack_i16(get_imu_altitude_msl(),  10.0f);
    t.kf_vs_cms = link_pack_i16(get_imu_vspeed(),      100.0f);
    have |= RHAVE_KFALT | RHAVE_KFVS;

    t.press_dpa = link_pack_u16(get_airdata_pressure(), 10.0f, 65535);
    have |= RHAVE_PRESS;

    // ★ Roll/Pitch/Yaw func（姿勢機能のマスタースイッチ）が OFF なら、姿勢まわりは**送らない**
    //   （2026-10-27）。OFF の機体の画面には姿勢が出ていないので、受信機もそれを再現する
    //   （受信機は自分のスイッチではなく「載っているか」で出すかを決める。display_tft.cpp の
    //   eskf_display_enabled()）。バンク警報も受信機で鳴らなくなり、機体の画面と食い違わない。
    if (attitude_get_rpy_enabled()) {
        float roll, pitch, yaw;
        attitude_get_euler(roll, pitch, yaw);
        if (attitude_ready()) {
            t.roll_cdeg  = link_pack_i16(roll,  100.0f);
            t.pitch_cdeg = link_pack_i16(pitch, 100.0f);
            t.yaw_cdeg   = link_pack_i16(yaw,   100.0f);
            have |= RHAVE_ATT;
            float acc95 = attitude_get_yaw_acc95_deg();
            t.yaw_acc95_deg = link_pack_u8(acc95, 1.0f);
            have |= RHAVE_ATT_YAW;
        }
        t.roll_trim_cdeg = link_pack_i16(attitude_get_roll_trim_deg(), 100.0f);
        have |= RHAVE_ATT_TRIM;

        if (attitude_pitch_avg_valid()) {
            t.pitch_avg_cdeg = link_pack_i16(attitude_get_pitch_avg_deg(), 100.0f);
            have |= RHAVE_ATT_AVG;
        }
        float wspd, wdir;
        if (attitude_get_wind(wspd, wdir)) {
            t.wind_dmps     = link_pack_u8(wspd,   10.0f);
            t.wind_dir_cdeg = link_pack_u16(wdir, 100.0f, 36000);
            have |= RHAVE_ATT_WIND;
        }
    }

    const float batt_v = get_input_voltage();
    t.volt_cv = link_pack_volt(batt_v);
    have |= RHAVE_VOLT;

    t.numsat  = (uint8_t)constrain(get_gnss_numsat(), 0, 255);
    have |= RHAVE_NUMSAT;
    t.hacc_dm = (uint16_t)constrain(get_gnss_hacc_mm() / 100, 0UL, 65535UL);  // mm → 0.1m
    t.fixflags = get_gnss_fixok() ? 0x01 : 0x00;

    // ナビ設定。受信側はこれを自分の設定と突き合わせるためだけに使う。
    t.dest_index = (int8_t)currentdestination;
    t.dest_id    = link_dest_id(currentdestination);   // 受信側は番号ではなくこれで探す
    t.nav_mode   = (uint8_t)destination_mode;
    t.scale_index = (uint8_t)scaleindex;   // 受信側はこれに合わせる（link_sync_scale）

    t.have = have;

    // status は「受信側からは知りようがない送信機自身の健康状態」だけ。
    // 飛行の警告は受信側がミラー値から再計算するので載せない（link_proto.h 参照）。
    uint16_t st = 0;
    if (!good_sd())                                    st |= LINK_ST_SD_ERROR;
    if (!get_imu_ok())                                 st |= LINK_ST_IMU_ERROR;
    if (attitude_needs_apply() && attitude_get_rpy_enabled()) st |= LINK_ST_NEEDS_APPLY;
    // 電源経路の異常のあいだは電池低下を立てない（電圧が低いのは故障のせい。下の PWR_FAULT が知らせる）
    if (batt_v <= BAT_LOW_VOLTAGE && !power_fault_active()) st |= LINK_ST_BAT_LOW;
    // 地上チェック（ロール／ピッチ／較正やり直し）。**送信機しか判定できない**ので
    // ビットで運ぶ（link_proto.h の「再計算できるものは載せない」の例外）。
    if (ground_roll_warn_active)                       st |= LINK_ST_GROUND_ROLL;
    if (ground_pitch_warn_active)                      st |= LINK_ST_GROUND_PITCH;
    if (ground_calib_warn_active)                      st |= LINK_ST_CALIB_GND;
    // ★ GNSS モジュールの断線。**受信側では知りようがない**（理由は link_proto.h）。
    //   条件は送信機自身の警報（gnss_conn_tick）と同じ「一度繋がってから切れた」。
    //   `!get_gnss_connection()` だけだと**起動直後のまだ 1 バイトも来ていない間に
    //   立ち**、受信機が電源投入ごとに誤報する。
    if (gnss_ever_connected() && !get_gnss_connection()) st |= LINK_ST_GNSS_LOST;
    if (power_fault_active())                          st |= LINK_ST_PWR_FAULT;
    t.status = st;

    // ★ ヘッダとペイロードを 1 回の write() にまとめるのは e220_send() の責任。
    //   分割して書くと区切り時間を超えてパケットが分裂する（docs/pons_link.md §4）。
    // ★ 電波には 32 バイトに詰めてから載せる（lora_link/link_wire2.h。中身の組み立ては上のまま。
    //   送信時間が 65 バイトの約 6 割になり、送信電流もそのぶん減る）。
    uint8_t w2[LINK_W2_LEN];
    link_w2_encode(&t, w2);
    if (e220_send(w2, (uint8_t)LINK_W2_LEN)) { s_txCount++; s_txMs = millis(); }
}

// ============================================================
//  受信側：状態
// ============================================================
uint32_t link_age_ms()      { return s_rxValid ? (millis() - s_rxMs) : 0xFFFFFFFF; }
bool     link_is_receiving(){ return s_rxValid && link_age_ms() < link_nosignal_ms(); }

// 「無信号」と受信枠の「切れかけ」の時間を、送る間隔に比例させる（ローワーは 1000ms なので
// 元の 10 秒 / 3 秒のまま）。アッパーの SF9 以上は間隔が 1 秒より長く、元の時間のままだと 1〜2 発
// 落ちただけで NO SIGNAL になるため（「何発落ちたら」をローワーとそろえる）。
uint32_t link_nosignal_ms() { return (uint32_t)NOSIGNAL_MS * link_tx_period_ms() / 1000; }
uint32_t link_stale_ms()    { return LINK_FRAME_STALE_MS * link_tx_period_ms() / 1000; }

// 前回の問い合わせ以降に新しいテレメトリが届いたか。**読むと落ちる**（1 回限り）。
// ★ ミラー中の「新しい位置が来た」の合図。受信モードは「CSV の代わりに電波から
//   リプレイする」実装（このファイル冒頭）なので、リプレイの apply_replay_row() が
//   立てている new_location_arrived / newcourse_arrived と同じ役割をここが担う。
//   使う側は gnss_new_location_arrived()（gnss.cpp）。
bool     link_take_new_frame(){ const bool n = s_rxNewFrame; s_rxNewFrame = false; return n; }
bool     link_take_draw_frame(){ const bool n = s_drawFrame; s_drawFrame = false; return n; }

// ミラー中に TFT を描いてよい時間か。ミラー中でなければ常に true。
// ★ **送信機のフレームを受けている最中に描かないため。** TFT の 16bit バスが動くと
//   E220 の雑音の値が最大で +8〜10dB 上がる（debug/v7-noise-test の雑音試験）。
//   地図の描き直しはフレームを受けた直後にそろえてある（.ino の draw_current_screen）ので、
//   それ以外の描き直し（強制再描画）だけをここで止める。
// 前回受けた時刻から送信周期ごとに次のフレームが来るとみなし（落ちた分も同じ刻みで数える）、
// 「受け終わった直後」から「次の受信が始まる LINK_DRAW_BUDGET_MS + LINK_DRAW_MARGIN_MS 手前」
// までを窓にする。描き始めから最後の転送までの時間を見込んで手前で締め切る。
bool link_draw_window_ok() {
    if (!link_mirror_active()) return true;
    const uint32_t T  = s_txPeriodEstMs;
    const uint32_t ph = (millis() - s_rxMs) % T;
    int32_t last = (int32_t)T - (int32_t)e220_airtime_ms(link_radio_profile)
                 - LINK_DRAW_BUDGET_MS - LINK_DRAW_MARGIN_MS;
    if (last < LINK_DRAW_MIN_WINDOW_MS) last = LINK_DRAW_MIN_WINDOW_MS;   // SF11 は送信時間が長い
    return (int32_t)ph <= last;
}
int8_t   link_rssi()        { return s_rxValid ? s_rx.rssi : 0; }

// ★ ミラー中か。各センサ accessor はこれを見て、自機の値の代わりに
//   受信した機体の値を返す（gnss.cpp の is_demo_active() と同じ流儀）。
//
//   accessor で差し込む方式にした理由:
//     ・グローバル（stored_lat 等）を汚さないので、**自機のフライト CSV は
//       ボート自身の位置を記録し続ける**（try_enque_savecsv は生の変数を読む）
//     ・NO SIGNAL に落ちた瞬間、分岐が外れて即座に自機データへ戻る。
//       上書き方式だと古い機体データが残ってしまう
//     ・received/ は受信データ専用のまま保てる
bool link_mirror_active() {
    // ★ リプレイ中はミラーしない。**再生のほうを勝たせる。**
    //   受信ログ（received/）を再生するのは、たいてい受信モードのボート機。
    //   ここでミラーを優先すると、再生を選んだのに生の受信データが出続けて
    //   「再生できない」ように見える。生データは received/ に残り続けるので、
    //   再生を優先しても失うものは無い。
    // ★★ **デモ中もミラーしない。**「デモは無線 OFF 扱い」（link.h）の一部。
    //   許すと **受信中は機体・切れた瞬間に仮想機体**という混ざり方をする。
    //   get_gnss_lat() はミラー → デモの順に見るので、**黙って入れ替わる**
    //   （NO SIGNAL が「自機に戻った」ではなく「デモに化けた」になる）。
    if (!link_live_ui_ok()) return false;
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

// 直近に送出できてからの経過 [ms]。一度も送っていなければ 0xFFFFFFFF。
// ★ 「電波が出た」瞬間だけ地図のアイコンを膨らませるために使う。
//   モジュールがビジーでスロットを捨てると更新されないので、
//   脈動が止まること自体が「送れていない」の合図になる。
uint32_t link_since_tx_ms() { return s_txMs ? (millis() - s_txMs) : 0xFFFFFFFFu; }

// 直近 10 秒の平均 RSSI [dBm]。1 パケットも入っていなければ 0。
//   1 発ぶんの RSSI は数 dB ふらつくので、アンテナの向きや置き場所を
//   その場で比べるには平均でないと判断できない。
int8_t link_rssi_avg10() {
    const uint32_t now = millis();
    const uint32_t win = link_nosignal_ms();   // RSSI10_WIN_MS を送る間隔に合わせたもの
    int32_t sum = 0; uint8_t n = 0;
    for (uint8_t i = 0; i < RSSI10_SLOTS; i++) {
        if (!s_r10Ms[i]) continue;                       // 未使用の枠
        if (now - s_r10Ms[i] > win) continue;            // 窓より前（ローワーで 10 秒）
        sum += s_r10Val[i]; n++;
    }
    return n ? (int8_t)(sum / (int32_t)n) : 0;
}

// ============================================================
//  電波の強さを 0〜3 本へ（表示用）
// ============================================================
// ★ 絶対値のしきい値は使わない。**限界からの余裕**で切る。
//   到達距離を決めているのは「RSSI − 限界」であって RSSI そのものではないし、
//   SF を上げれば限界も下がるので、固定値だと SF を変えた瞬間に意味を失う。
//
//   限界 RSSI ≒ 環境雑音 + SF の SNR 閾値。
//     環境雑音は実測 −104dBm（琵琶湖畔・docs/pons_link.md §8）。
//     SNR 閾値は SF7 で −7.5dB、SF が 1 上がるごとに 2.5dB 下がる。
//   → SF7 で −111dBm、SF8 で −113dBm … SF11 で −121dBm。
static int16_t link_rssi_floor_dbm() {
    const uint8_t sf = e220_profile_to_sf(link_radio_profile);
    return (int16_t)(-111 - (5 * (int16_t)(sf - E220_SF_MIN)) / 2);
}

// 受信できている電波が限界からどれだけ上にいるか [dB]。
// ★ 本数（下の link_rssi_bars）は**この値から**決める。2 箇所で別々に
//   引き算すると、本数と数字が食い違って現地でどちらを信じるか分からなくなる。
// 本数は 3 段階しかないので、アンテナを少し動かしたときの変化が読めない。
// 置き場所を A/B で比べるには dB の数字が要るので、設定画面にはこちらを出す。
// 受信できていないときは 0（比べる相手が無い）。
// 幅が暴れないよう ±99 に丸める（設定画面の桁が崩れるのを防ぐ）。
int8_t link_rssi_margin_db() {
    if (!link_is_receiving()) return 0;
    const int8_t avg = link_rssi_avg10();
    if (avg == 0) return 0;          // 窓に 1 発も無い（0dBm は実在しない値）
    const int16_t m = (int16_t)avg - link_rssi_floor_dbm();
    return (int8_t)constrain((int)m, -99, 99);
}

// 余裕 [dB] → 本数。SF8（限界 −113dBm）だと
//   3 本 > −83 / 2 本 > −93 / 1 本 > −103 / 0 本 それ以下。
//   要求距離 200m は 2 本、実測でロストし始める 350m 付近が 0〜1 本になる。
#define RSSI_MARGIN_3   30
#define RSSI_MARGIN_2   20
#define RSSI_MARGIN_1   10

uint8_t link_rssi_bars() {
    // ★ 引き算は link_rssi_margin_db() の 1 箇所だけ。ここで再計算しないこと
    //   （受信していない・窓が空 のガードもあちらに入っている）。
    const int16_t margin = link_rssi_margin_db();
    if (margin == 0) return 0;
    if (margin >= RSSI_MARGIN_3) return 3;
    if (margin >= RSSI_MARGIN_2) return 2;
    if (margin >= RSSI_MARGIN_1) return 1;
    return 0;
}

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
// 送信機が 2 台いる状態か。最後に検出してから LINK_DUPSENDER_HOLD_MS の間だけ true。
// ★ 時間で消えることが大事。2 台目を止めたら**再起動せずに直ったことを確認できる**。
bool     link_dup_sender() {
    return s_dupSenderMs != 0 && (millis() - s_dupSenderMs) < LINK_DUPSENDER_HOLD_MS;
}

uint16_t link_sender_status() { return link_is_receiving() ? s_rx.t.status : 0; }

// ★ 目的地／ナビモードが送信側と食い違っていないか。
//   受信側の警告は「ミラーされた値から自分で再計算する」方式なので、
//   目的地が違うと**コース警告だけが静かにずれる**。計算は正しいのに
//   結果が違う、という一番たちの悪い壊れ方をするので、明示的に検出する。
bool link_dest_mismatch() {
    if (!link_is_receiving()) return false;
    // ★ 目的地は回転枠 6（番号）と 7（ID）、ナビモードは 7 で来る。受けるまでは 0 のままなので、
    //   食い違いとは言わない（言うと link_sync_destination が目的地 0 へ書き換えてしまう）。
    // ★ 番号ではなく ID で比べる（link_sender_dest_local）。受信機に無い目的地はいつも「食い違い」。
    const int li = link_sender_dest_local();
    if (li == LINK_DEST_UNKNOWN) return false;
    if (li == LINK_DEST_MISSING) return true;
    return (li != currentdestination) ||
           (s_rx.t.nav_mode != (uint8_t)destination_mode);
}

// ---- リプレイ経路と同じ形の供給関数 ----
// 受信できていない、または have ビットが立っていない項目は false / 0 を返す。
// 呼び出し側は必ず link_has_value() で確かめてから使うこと。
bool link_has_value(uint16_t havebit) {
    return link_is_receiving() && (s_rx.t.have & havebit);
}
// ★ 詰め方を知っているのはここだけ。呼び出し側で割り算を書き直さないこと。
//   s_rx は静的なので未受信でも 0 が返るだけで、参照そのものは安全。
double   link_get_lat()           { return s_rx.t.lat_1e7 / 1e7; }
double   link_get_lon()           { return s_rx.t.lon_1e7 / 1e7; }
double   link_get_gs()            { return s_rx.t.gs_cms      / 100.0; }
// 実 GNSS(gnss.cpp) / リプレイ / デモはどれも 0〜360 を強制している。ここだけ素通しだと
// CRC を通った壊れフレームで 655 度までの値がナビ計算に入るため、同じ形で揃える。
double   link_get_truetrack()     {
  double t = s_rx.t.track_cdeg / 100.0;
  return (t < 0.0 || t > 360.0) ? 0.0 : t;
}
double   link_get_gnss_altitude() { return s_rx.t.gnss_alt_dm /  10.0; }
int      link_get_numsat()        { return s_rx.t.numsat; }
uint32_t link_get_hacc_mm()       { return (uint32_t)s_rx.t.hacc_dm * 100u; }
bool     link_get_fix_ok()        { return (s_rx.t.fixflags & 0x01) != 0; }

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
