// ============================================================
// File    : link.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : PONS Link（機体⇄ボート無線）の RP2350 側実装。
//           UART1 のフレーミング、送信テレメトリの組み立て、
//           受信テレメトリの保持と供給。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/18
// ============================================================
//
// ■ 設計の要点
//   ・**送信の write() はあえてブロックさせる。** 隙間なく 1 回で流し込まないと
//     E220 が途中でパケットを切ってしまうため（docs/pons_link.md §4）。
//     115200bps・65 バイトで 3.1ms なので、1Hz なら Core0 への影響は無視できる。
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
// ★ グループ ID。**既定 0 のまま git に置く。** 実際の値は各機の SD にだけ入れる。
//   同じ会場に別チームの PONS がいても取り違えないための識別子（link_proto.h 参照）。
uint8_t link_group           = LINK_GROUP_DEFAULT;

// ---- 受信状態 ----
static LinkRxTelem   s_rx;                 // 直近に受信したテレメトリ
static bool          s_rxValid   = false;
static uint32_t      s_rxMs      = 0;      // 受信した時刻（RP の millis）
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
#define RSSI10_SLOTS   16
#define RSSI10_WIN_MS  10000
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

static void link_push_telemetry();   // 実体は下（送信側の組み立て）
static void link_preflight_start();  // 実体は下（送信前チェック）
static void link_apply_power_state();// 実体は下（モードに応じた mode 0 / mode 3）

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
        s_r10Val[s_r10Head] = s_rx.rssi;
        s_r10Ms [s_r10Head] = s_rxMs;
        s_r10Head = (uint8_t)((s_r10Head + 1) % RSSI10_SLOTS);

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
void link_setup() {
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

// 起動時に無線モードを読み上げる。**setup() の末尾で呼ぶ。**
// ★ **3 状態すべてを読み上げる**（docs/pons_link.md §1）。TX / RX だけ読み上げて
//   OFF を無音にしていたころ（0.984 まで）は「無音＝OFF」という取り決めだったが、
//   **無音は「音量 0」や「音声が焼けていない」とも区別が付かない**。意図せず OFF に
//   なっている機体で、持ち主が「音量を絞ったままだ」と思い込む余地があった。
//   `link_off.wav` を足して、3 状態とも**必ず何か喋る**ようにしてある。
//   **OFF の分岐を無音に戻さないこと。**
// ★ 範囲外の値は来ない（mysd.cpp の setLinkMode() が OFF へ丸める）。
//   それでも else で受けてあるのは、万一来ても無音にしないため。
// ★ link_setup() から切り離してある理由:
//   link_setup() は起動画面に E220 の結果を出すためスプラッシュより前へ移した。
//   読み上げもそこへ付いていくと、**起動音(opening.wav)より先に喋ってしまう**。
//   元どおり setup() の末尾で呼ぶことで、鳴る順番は移動前と同じになる。
void link_announce_mode() {
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
    // 送信中に叩かない（settings.h の LINK_PROBE_AFTER_TX_MS 参照）。
    // lastProbe を更新せずに戻るので、静かになった時点で改めて実行される。
    if (tx && link_since_tx_ms() < LINK_PROBE_AFTER_TX_MS) return;

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

// 送信機側の異常（SD／IMU／較正未適用）。飛ぶ前に気づきたい項目。
// 音声にはしない ― 内訳は無線ページに出るので「何かある」と分かれば足りる。
// 音声を増やすほど、本当に聞くべきものが埋もれる。
// ★ 電池低下だけは別扱い（下の link_watch_sender_battery）。ここで一緒に見ると
//   「立ち上がりで 1 回鳴るだけ」になるが、電池は減り続けるので繰り返す必要がある。
#define LINK_ST_FAULTS_TONE  (LINK_ST_SD_ERROR | LINK_ST_IMU_ERROR | LINK_ST_NEEDS_APPLY)
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
//   ボートが先に気づいて降ろす判断ができる。画面では S の残量が赤くなるだけなので、
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
            "LINK TX: mod=%s ch=%u prof=%u sent=%u noRf=%lu airmax=%lums to=%lu%s",
            mod, link_radio_ch, link_radio_profile,
            (unsigned)s_txCount, (unsigned long)s_txNoRf,
            (unsigned long)s_txAirMaxMs, (unsigned long)s_txAirTimeout,
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
    } else if (s_pfFlags & (LINK_PFF_PONS_SAME | LINK_PFF_PONS_OTHER | LINK_PFF_BURST)) {
        // 他の送信機がいる。対処は「設定を確認する／他チームと調整する」。
        link_alert("wav/link_ch_busy.wav", 294, 350, 3);
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
    while (e220_recv((uint8_t*)&t, (uint8_t)sizeof(t), &rssi)) {
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
typedef enum { LTX_IDLE = 0, LTX_WAKING, LTX_TXSTART, LTX_TXDONE } LinkTxState;
static LinkTxState s_txState     = LTX_IDLE;
static uint32_t    s_txStateMs   = 0;

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

    case LTX_TXDONE:
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
        if (now - s_txStateMs >= LINK_TX_DONE_MS) {
            // 上限。ここに来たら送信時間の想定が違う（または AUX が張り付いている）。
            s_txAirTimeout++;
            enqueueTask(createLogSdfTask("LINK TX: air TIMEOUT (%lums) n=%lu",
                                         (unsigned long)(now - s_txStateMs),
                                         (unsigned long)s_txAirTimeout));
            e220_sleep();
            s_txState = LTX_IDLE;
        }
        return;

    case LTX_IDLE:
    default:
        break;
    }

    static uint32_t lastTx = 0;
    if (now - lastTx < 1000) return;
    lastTx = now;
    e220_wake_begin();                  // mode 0 にするだけ。**AUX は待たない**
    s_txState   = LTX_WAKING;
    s_txStateMs = now;
}

void link_loop() {
    link_poll_radio();
    link_watch_mirror_transition();
    link_probe_module();        // ★ watch_module より先。判断の材料をここで更新する
    link_watch_module();
    link_sync_destination();
    // ★ この 2 つと link_watch_module() は **リプレイ中・デモ中も鳴らす**。
    //   「送信機の電池が減っている」「自分のモジュールが応答しない」は
    //   画面に出している数字とは無関係に**いま現実に起きている事実**なので、
    //   再生のために黙らせる理由が無い（画面の飾りだけを止める。link.h 参照）。
    link_watch_sender_faults();
    link_watch_sender_battery();
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
    // ★ 変わった役割をその場で読み上げる（起動時の読み上げと同じ流儀）。
    //   機体を誤って受信モードにすると送信が止まり、ボート側の症状は
    //   「無信号」になる。起動時にしか読み上げないと、飛行前の操作ミスに
    //   気づく機会が無い。
    //   ★ **OFF も読み上げる**（0.985 から）。回転式なので目的のモードを通り過ぎて
    //     OFF まで回してしまうことがあり、**OFF を無音にしていると気づけない**。
    //     無線が落ちた症状は相手側の「無信号」だけなので、自機では分からない。
    //   音量は起動時の読み上げと揃える（優先度 2・最低音量の強制なし）。
    //   意図して消音している機体を、モードを回すたびに鳴らさないため。
    //   ★ WAV_EXCL_LINKMODE を必ず付ける。設定画面のモードは回転式で、RECEIVER へ
    //     行くには SENDER を必ず通る。付けないと通過した sender_mode.wav が pending に
    //     残り、**受信モードにした直後の機体が「送信モード」と読み上げる**（0.984 の不具合）。
    //     OFF にも付いているので、**読み上げ中に OFF へ回せばその場で言い替わる**。
    if      (mode == LINK_MODE_TX) enqueueTask(createPlayWavTask("wav/sender_mode.wav",   2, 0, WAV_EXCL_LINKMODE));
    else if (mode == LINK_MODE_RX) enqueueTask(createPlayWavTask("wav/receiver_mode.wav", 2, 0, WAV_EXCL_LINKMODE));
    else                           enqueueTask(createPlayWavTask("wav/link_off.wav",      2, 0, WAV_EXCL_LINKMODE));
    s_rxValid = false;                 // モードが変わったら受信データは無効
    link_apply_power_state();
    // 送信モードに入ったら送り始める前にチャンネルを確かめる。
    // 抜けたときは IDLE に戻す（結果を残すと古い判定が画面に出続ける）。
    if (link_mode_setting == LINK_MODE_TX) link_preflight_start();
    else                                   s_pfState = LINK_PF_IDLE;
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
// ★ 電波の設定が変わったら送信前チェックをやり直す。前の CH で測った結果を
//   残しておくと、変えた先が混んでいても「OK」のままになる。
//   link_set_group() からは呼ばない（グループはペイロードの中身で、電波ではない）。
// ★ reconfigure のあとは必ず link_apply_power_state() を通す（理由は同関数のコメント）。
//   TX のときは直後の link_preflight_start() が改めて起こすので、
//   「寝かせる → 起こす」の往復になるが、どちらもピン 1 本の書き換えで済む。
void link_set_radio_ch(uint8_t ch) {
    if (ch > E220_CH_MAX || ch == link_radio_ch) return;
    link_radio_ch = ch;
    e220_reconfigure(link_radio_ch, link_radio_profile, false);
    link_apply_power_state();
    link_preflight_start();
}
void link_set_radio_profile(uint8_t p) {
    if (p >= LINK_PROFILE_COUNT || p == link_radio_profile) return;
    link_radio_profile = p;
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
    t.group = link_group;       // rsv[] は LinkTelem t = {} で 0 済み
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

    t.volt_cv = link_pack_volt(get_input_voltage());
    have |= RHAVE_VOLT;

    t.numsat  = (uint8_t)constrain(get_gnss_numsat(), 0, 255);
    have |= RHAVE_NUMSAT;
    t.hacc_dm = (uint16_t)constrain(get_gnss_hacc_mm() / 100, 0UL, 65535UL);  // mm → 0.1m
    t.fixflags = get_gnss_fixok() ? 0x01 : 0x00;

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
    if (e220_send((const uint8_t*)&t, (uint8_t)sizeof(t))) { s_txCount++; s_txMs = millis(); }
}

// ============================================================
//  受信側：状態
// ============================================================
uint32_t link_age_ms()      { return s_rxValid ? (millis() - s_rxMs) : 0xFFFFFFFF; }
bool     link_is_receiving(){ return s_rxValid && link_age_ms() < NOSIGNAL_MS; }
int8_t   link_rssi()        { return s_rxValid ? s_rx.rssi : 0; }
const LinkTelem* link_rx_telem() { return s_rxValid ? &s_rx.t : nullptr; }

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
    int32_t sum = 0; uint8_t n = 0;
    for (uint8_t i = 0; i < RSSI10_SLOTS; i++) {
        if (!s_r10Ms[i]) continue;                       // 未使用の枠
        if (now - s_r10Ms[i] > RSSI10_WIN_MS) continue;  // 10 秒より前
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
    return (s_rx.t.dest_index != (int8_t)currentdestination) ||
           (s_rx.t.nav_mode   != (uint8_t)destination_mode);
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
