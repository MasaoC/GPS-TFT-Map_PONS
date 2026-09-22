// ============================================================
// File    : e220.cpp
// Project : PONS v7 — PONS Link
// Role    : E220-900T22S(JP) の下位ドライバ実装
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/18
// ============================================================

#include "e220.h"
#include <string.h>
#include "mysd.h"   // 切り分けログ（enqueueTask / createLogSdfTask）
#include "lora_link/link_proto.h"

#define E220_SERIAL       Serial2      // RP2350 UART1 = GPIO8(TX)/GPIO9(RX)

// モードピンの論理。M0=M1 を結線しているので 1 本で両方を駆動する。
#define MODE_NORMAL       LOW          // mode 0
#define MODE_CONFIG       HIGH         // mode 3 (Config/DeepSleep)

// レジスタ 0x02 のボーレート欄（データシート 表 レジスタ0x02）
//   000:1200 001:2400 010:4800 011:9600(既定) 100:19200 101:38400 110:57600 111:115200
// ★ ここに書くのは **mode 0（通常送受信）で使うボーレート**。
//   mode 3 は設定値に関わらず常に 9600 なので、この値の影響を受けない。
#define BAUD_BITS_115200  0b111

// ---- 状態 ----
static bool    s_alive     = false;    // 設定の読み返しが通ったか
static bool    s_asleep    = false;

// ---- 実行中の生存確認 ----
// ★ s_alive だけでは「起動後に死んだ」を検出できない。書いているのは
//   write_config() だけなので、更新されるのは e220_setup() と e220_reconfigure()
//   を通ったときだけ。**飛行中にコネクタが抜けても true のまま**になる。
//   そうなると e220_send() は書き込むだけで成功し続け（AUX は未実装対策の
//   INPUT_PULLUP なので断線すると High＝アイドルに見える）、送信回数も
//   地図の弧も増え続ける。上りが無いので誰も気づけない。
//
//   そこで「問い合わせて返事があったか」を別に数える。返事＝生きている証拠は
//     ・環境ノイズ取得の応答（こちらから聞けるのはこれだけ。link.cpp が定期的に叩く）
//     ・テレメトリを 1 つ受け取った（受信機はこれで足りる）
//   の 2 つで、どちらかがあれば 0 に戻す。
static uint8_t s_ch      = 0;          // 現在のチャンネル（通常送信の宛先に使う）
static bool    s_strict  = false;      // Strict Mode が有効か（環境ノイズの読み方が変わる）
static uint8_t s_noResp = 0;           // 連続して返事が無かった回数
#define E220_NORESP_LIMIT     3        // これ以上続いたら無応答とみなす

// 環境ノイズ取得の応答待ち [ms]。生きていれば数 ms で返る。
// ★★ 一度でも落ちたら短いほうに切り替える。**無応答のまま 200ms 待ちを
//   繰り返すと Core0 が止まり、GNSS の FIFO（1024B / 38400bps ≒ 267ms）が溢れて
//   NAV-PVT を丸ごと取りこぼす。** 生きていれば数 ms で返るので、短くしても
//   取りこぼさないし、返事があれば 0 に戻って長いほうへ復帰する。
#define E220_NOISE_WAIT_MS      200
#define E220_NOISE_WAIT_FAST_MS  30

// mode 3 へ入ってからコマンドを送るまでの上乗せ [ms]。
// set_mode() が切替後に AUX の立ち上がりを待つようになったので、本来これは要らない。
// 残してあるのは e220_ping() が**生存確認**だからで、ここが誤って失敗すると
// e220_alive() が落ちて送信前チェックごと飛び、警報まで鳴る。10 秒に 1 回の
// 30ms は安いので、保険として置いている。**根拠のある値ではない。**
#define E220_CFG_SETTLE_MS       30

// 返事があった／無かったを 1 か所で数える。
// ---- 無応答の切り分け用ログ（一時的。原因が判明したら消す）----
// モジュールが「黙っている」のか「化けた返事をしている」のかで対処が正反対なので、
// 失敗のたびに **受信バイト数・先頭の中身・AUX の状態** を残す。
//   got=0        → そもそも返事が無い。配線・電源・M0/M1・ボーレートを疑う
//   got>0 で不一致 → 返事はある。ボーレート違いか、別のものが喋っている
//   AUX=0        → モジュールがビジー、または AUX が張り付いている
// log.txt が溢れないよう 1 起動あたりの件数に上限を置く。
static uint16_t s_dbgLeft = 40;
#define E220_DBG(...) do { if (s_dbgLeft) { s_dbgLeft--; \
                           enqueueTask(createLogSdfTask(__VA_ARGS__)); } } while (0)

static inline void note_response()   { s_noResp = 0; }
static inline void note_no_response(){ if (s_noResp < 255) s_noResp++; }

// 受信の組み立てバッファ。
//   1 パケット = ペイロード + RSSI 1 バイト。取りこぼしや異物で位相がずれても
//   magic を探し直せるよう、3 パケットぶんの余裕を持たせる。
//   ★ 環境ノイズ取得（e220_read_noise）も応答待ちの間このバッファを共有する。
//     受信中に測ると「溜まっているテレメトリ + 待っている間に届くテレメトリ +
//     応答 5 バイト」が同居しうるので、2 パケットぶんでは足りない。
#define RXBUF_SIZE       (3 * (sizeof(LinkTelem) + 1) + 16)
static uint8_t  s_rx[RXBUF_SIZE];
static uint16_t s_rxLen = 0;

// ---- 設定値の重複チェック ----
// チャネル数と SF の段数は「無線に依存しない上位」(link_proto.h) と
// 「E220 固有の下位」(e220.h) の両方に書いてある。片方だけ直すと
// 設定画面が選べない値を出すので、ずれたらビルドで止める。
static_assert(LINK_RADIO_CH_MAX == E220_CH_MAX,
              "link_proto.h の CH 上限と e220.h の CH 上限がずれている");
static_assert(LINK_PROFILE_COUNT == (E220_SF_MAX - E220_SF_MIN + 1),
              "link_proto.h のプロファイル数と e220.h の SF 範囲がずれている");

// ============================================================
//  小物
// ============================================================

uint8_t e220_profile_to_sf(uint8_t profile) {
    const uint8_t sf = (uint8_t)(E220_SF_MIN + profile);
    if (sf < E220_SF_MIN || sf > E220_SF_MAX) return E220_SF_MIN + 1;  // 範囲外は SF8
    return sf;
}

// BW500kHz のときの air_data_rate（レジスタ 0x02 の下位 **5bit**）。
//   BW500 は ((SF-5) << 2) | 0b10。SF8 なら 0b01110。
//   ★ 一般の E220 データシートは bit4-3 をパリティ欄としているが、
//     JP 版は 5bit すべてが air_data_rate。動作確認済みの
//     esp32_e220900t22s_jp_lib も REG0 = (baud<<5) | air_data_rate と組んでいる。
static uint8_t air_data_rate_bits(uint8_t sf) {
    if (sf < E220_SF_MIN) sf = E220_SF_MIN;
    if (sf > E220_SF_MAX) sf = E220_SF_MAX;
    return (uint8_t)(((sf - 5) << 2) | 0b10);
}

// AUX が High（アイドル）になるのを待つ。
//   データシート 5.3：モード切替は AUX が High になってから 2ms 後が安全。
//   AUX が来なくても進めるよう上限を切る（配線ミスで固まらないように）。
static void wait_aux_idle(uint16_t timeout_ms) {
    const uint32_t t0 = millis();
    while (digitalRead(E220_AUX_PIN) == LOW) {
        if (millis() - t0 > timeout_ms) {
            // ★ AUX が上がらないまま打ち切った。内部プルアップを入れてあるので
            //   **モジュール不在なら High に見えるはず**。Low のままということは
            //   「居るのにビジー」か「AUX が GND に落ちている」のどちらか。
            E220_DBG("E220 AUX stuck LOW (%ums timeout)", timeout_ms);
            return;
        }
    }
    delay(2);
}

static void set_mode(int level) {
    wait_aux_idle(200);       // 切替は AUX が High のときしか効かない（5.3）
    digitalWrite(E220_MODE_PIN, level);
    delay(2);                 // 切替は 1ms。余裕を見て 2ms
    // ★★ **切替の「あと」も AUX の立ち上がりを待つ。**
    //   データシート 5.4 の 4 番・図 36: モード遷移ではハードウェアのセルフチェックが
    //   走り、その間 AUX は Low。完了で High に戻る。
    //   ここを待たずに喋りかけると **書き込みが黙って捨てられる**。
    //   実機で長く嵌まった（2026-09-22）: mode 0 の状態レジスタ読み出しが
    //   どのコマンド形式・どの送信モードでも「返事ゼロ」になり、書式の問題だと
    //   思い込んだ。実際は **一度も届いていなかった**。
    //   AUX の待ちを入れた途端に 10/10 で読めるようになった。
    wait_aux_idle(50);
}

// mode 3（Config/DeepSleep）は **レジスタ設定に関わらず常に 9600 8N1** なので、
// 設定を読み書きする間だけボーレートを落とす。通常運用は 115200。
// 切替が起きるのは起動時と設定変更時だけで、毎パケットではない。
// ★ arduino-pico の SerialUART には updateBaudRate() が無く、end()+begin()
//   しかない。end() で TX ピンが一瞬浮くのでモジュール側にゴミが入りうる。
//   そのため **reg_xfer() は送信前に受信バッファを捨て、応答は
//   「C1 addr len」の並びを検索して拾う**（先頭決め打ちにしない）。
//   ゴミが乗っても誤判定しないので、この制約は実害にならない。
static void set_baud(uint32_t baud) {
    E220_SERIAL.flush();          // 送りかけを出し切ってから
    E220_SERIAL.end();
    E220_SERIAL.setTX(E220_TX_PIN);
    E220_SERIAL.setRX(E220_RX_PIN);
    E220_SERIAL.setFIFOSize(256);
    E220_SERIAL.begin(baud);
}

// 受信バッファを捨てる。
// ★ 受信データも一緒に消えるので、**受信中に呼んではいけない**。
//   呼んでよいのはモード切替の前後（mode 3 の間）だけ。
static void flush_input() {
    while (E220_SERIAL.available()) E220_SERIAL.read();
    s_rxLen = 0;
}

// UART に来ているバイトを組み立てバッファへ移すだけ。捨てない。
// 受信の取り込みと環境ノイズの応答待ちで共用する。
static void rx_pump() {
    while (E220_SERIAL.available() && s_rxLen < RXBUF_SIZE)
        s_rx[s_rxLen++] = (uint8_t)E220_SERIAL.read();
}

// 組み立てバッファの先頭から n バイト捨てる
static void rx_drop_front(uint16_t n) {
    if (n >= s_rxLen) { s_rxLen = 0; return; }
    s_rxLen = (uint16_t)(s_rxLen - n);
    memmove(s_rx, s_rx + n, s_rxLen);
}

// ============================================================
//  レジスタ操作（mode 3 でのみ有効）
// ============================================================

// C0 = 恒久書込み（不揮発・寿命あり）。C2 = 一時書込み（RAM のみ）。C1 = 読出し。
//   応答はいずれも C1 <開始アドレス> <長さ> <値...>。
//   ★ 受信データが混ざることがあるので、応答の先頭を決め打ちしない。
//   ★ mode 3 でしか使えない。この関数を呼ぶ側がモードを保証すること。
static bool reg_xfer(uint8_t opcode, uint8_t addr, uint8_t len,
                     const uint8_t* in, uint8_t* out) {
    uint8_t cmd[3 + 16];
    if (len > 16) return false;
    cmd[0] = opcode; cmd[1] = addr; cmd[2] = len;
    uint8_t n = 3;
    if (in) { memcpy(cmd + 3, in, len); n = (uint8_t)(3 + len); }

    flush_input();
    E220_SERIAL.write(cmd, n);
    E220_SERIAL.flush();

    uint8_t buf[32];
    uint8_t got = 0;
    bool    found = false;
    uint8_t at = 0;
    bool    errResp = false;
    const uint32_t t0 = millis();
    while (millis() - t0 < 300) {
        while (E220_SERIAL.available() && got < sizeof(buf)) buf[got++] = (uint8_t)E220_SERIAL.read();

        // ★★ **バイト数で打ち切ってはいけない。**
        //   以前は「3+len バイト受け取ったら読むのをやめる」作りだった。ところが
        //   先頭にゴミが 1 バイト乗ることが実際にあり（実機ログ: got=4 [F8 C1 08 01]）、
        //   **応答は正しく返っているのに 1 バイトずれて肝心の値が FIFO に残る**。
        //   ゴミが 1 バイトでも乗ると構造的に必ず失敗していた。
        //   応答の並びが見つかるまで読み続け、見つけた位置から取り出す。
        for (uint8_t i = 0; (uint16_t)i + 3 + len <= got; i++) {
            if (buf[i] == 0xC1 && buf[i + 1] == addr && buf[i + 2] == len) {
                found = true; at = i; break;
            }
        }
        if (found) break;

        // エラー応答 FF FF FF（データシート 6.15 / 図 37）。書式違い、または
        // 無効なレジスタを読んだとき。待ち続けても無駄なので即座に抜ける。
        for (uint8_t i = 0; (uint16_t)i + 3 <= got; i++) {
            if (buf[i] == 0xFF && buf[i + 1] == 0xFF && buf[i + 2] == 0xFF) { errResp = true; break; }
        }
        if (errResp) break;

        if (got >= sizeof(buf)) break;     // これ以上は溜められない
        delay(2);
    }

    if (found) {
        if (out) memcpy(out, buf + at + 3, len);
        return true;
    }
    // ★ ここへ来た＝期待する "C1 addr len" が見つからなかった。
    //   got と先頭バイトで「無言」と「化け」を区別する。
    E220_DBG("E220 regfail op=%02X addr=%02X len=%u got=%u%s [%02X %02X %02X %02X %02X %02X] AUX=%d",
             opcode, addr, len, got, errResp ? " ERR(FFFFFF)" : "",
             got > 0 ? buf[0] : 0, got > 1 ? buf[1] : 0,
             got > 2 ? buf[2] : 0, got > 3 ? buf[3] : 0,
             got > 4 ? buf[4] : 0, got > 5 ? buf[5] : 0,
             (int)digitalRead(E220_AUX_PIN));
    return false;
}

// Strict Mode (v2.0 厳格動作) を有効にする。**必要なときだけ書く。**
//   ★ なぜ要るか: 環境ノイズ(状態レジスタ 0xA3)を読む道が、これ以外に無い。
//     - mode 0 の専用コマンド(6.8 / C0 C1 C2 C3 00 02) … 実機で返事ゼロ。
//       しかも直後に AUX が Low へ落ちる＝6 バイトが電波として出てしまう。
//     - mode 3 で 0xA2〜0xA4 を素の 0xC1 で読む … 実機で FF FF FF（無効レジスタ）。
//       10.1 が「mode 3 では両モードで読める」と言うのは **9.1 の設定レジスタ**の話で、
//       **9.2 の状態レジスタは表 24 に「Strict Mode でのみ使用可」と明記**されている。
//     有効にすると 7.1 のとおり mode 0 のまま FF FF C1 A3 01 で読める。潜らずに済む。
//   ★ 副作用は小さい。1.6 のとおり ver.2.0 の初期値は送信モードビット以外 ver.1.x と同一で、
//     0x09 の他ビットは全部 0（チェックサム無効・パケットサイズ付与無効・宛先出力無効・
//     AUX 待ち 2〜3ms）。bit6 だけ立てれば送受信のバイト列は変わらない。
//     mode 3 の設定レジスタ操作も 10.1 のとおり両モードで使えるので write_config は不変。
//   ★ **0x09 は不揮発。**3 台運用で個体差が出ると最悪なので、毎起動で読んで
//     「無効なら立てる」。既に立っていれば書かないので、書換えは 1 台につき生涯 1 回。
static bool ensure_strict_mode(bool persist) {
    uint8_t ext = 0;
    if (!reg_xfer(0xC1, 0x09, 1, nullptr, &ext)) {
        E220_DBG("E220 strict: 0x09 read FAILED");
        return false;                       // 読めないのに書きに行かない
    }
    if (ext & 0x40) return true;            // 既に有効。**書かない**

    const uint8_t nv = (uint8_t)(ext | 0x40);
    E220_DBG("E220 strict: enabling (0x09 %02X -> %02X, persist=%d)", ext, nv, (int)persist);
    if (!reg_xfer(persist ? 0xC0 : 0xC2, 0x09, 1, &nv, nullptr)) {
        E220_DBG("E220 strict: WRITE failed");
        return false;
    }
    uint8_t rb = 0;
    if (!reg_xfer(0xC1, 0x09, 1, nullptr, &rb) || (rb & 0x40) == 0) {
        E220_DBG("E220 strict: READBACK failed (got %02X)", rb);
        return false;
    }
    E220_DBG("E220 strict: enabled OK (0x09=%02X)", rb);
    return true;
}

// 設定一式を書いて、読み返して照合する。
//   persist = true なら 0xC0（不揮発。書換え寿命あり）、false なら 0xC2（RAM のみ）。
// 直近に書いた（または読み返して一致した）REG1/REG3 の値。切り分けログ用。
static uint8_t v_reg1_cache = 0, v_reg3_cache = 0;

static bool write_config(uint8_t ch, uint8_t profile, bool persist) {
    const uint8_t sf = e220_profile_to_sf(profile);

    uint8_t v[6];
    v[0] = 0x00;                                   // 00H ADDH
    v[1] = 0x00;                                   // 01H ADDL
    v[2] = (uint8_t)((BAUD_BITS_115200 << 5) | air_data_rate_bits(sf));  // 02H REG0
    // 03H REG1: subpacket(00=200B)<<6 | 環境ノイズ有効<<5 | 送信出力(01=13dBm)
    v[3] = (uint8_t)((0b00 << 6) | (1 << 5) | 0b01);
    v[4] = ch;                                     // 04H REG2 チャンネル
    s_ch = ch;                                     // 送信時の宛先チャンネルに使う
    // 05H REG3: RSSIバイト付加<<7 | 送信方法<<6 | WORサイクル(011=2000ms)
    //
    // ★★ 送信方法は **必ず 0（トランスペアレント）** にすること。
    //   1（固定送信）にすると、モジュールは UART に来た先頭 3 バイトを
    //   「宛先ADDH / 宛先ADDL / チャンネル」として解釈する。すると
    //   環境ノイズ取得コマンド `C0 C1 C2 C3 00 02` が
    //   **「宛先 0xC0C1・ch 0xC2 への送信データ」** と誤解され、
    //   コマンドが効かないどころか電波として出てしまう。
    //   症状は「設定は完璧に入っているのに RSSI だけ永久に無応答、
    //   かつ AUX が Low に張り付く」。
    //
    //   我々の通信は「同じチャンネルの全機が受け取るブロードキャスト」なので、
    //   宛先指定は元々不要。トランスペアレントなら
    //     ・先頭 3 バイトのヘッダが要らない（占有時間も短くなる）
    //     ・環境ノイズ取得が使える
    //   と、こちらのほうが素直に合う。工場出荷時の既定もトランスペアレント。
    // ★ bit6 = 1 は **通常(Fixed-block)送信モード**。透過モードではない。
    //   透過モードだと、状態レジスタの読み出しコマンドが
    //   「サブパケットの区切り位置に来たときに限って」しかコマンドと解釈されず、
    //   外れると**そのまま電波として送信される**（6.5 / 6.9）。実機では
    //   ver.1 形式 (C0 C1 C2 C3) と Strict 形式 (FF FF C1) の**両方**が
    //   返事ゼロ＋直後に AUX が Low になり、環境ノイズを一度も取得できなかった。
    //   通常送信モードなら 3 バイト目がチャンネル値として解釈され、
    //   「これに合致するチャンネルは存在しない」ため曖昧さが消える（同節）。
    //   代償は送信時に宛先 3 バイトを前置すること（e220_send を参照）。
    v[5] = (uint8_t)((1 << 7) | (1 << 6) | 0b011);

    // ★ 先に読んで、既に同じなら書かない。
    //   書込みコマンド 0xC0 は **不揮発メモリ（フラッシュ）に保存する**ので
    //   書換え回数に寿命がある。設定は電源を切っても残るため、毎回の起動で
    //   書き直す必要は無い。
    v_reg1_cache = v[3];
    v_reg3_cache = v[5];

    uint8_t rb[6] = {0};
    if (reg_xfer(0xC1, 0x00, 6, nullptr, rb) && memcmp(v, rb, sizeof(v)) == 0)
        return true;                       // 既に正しい。何も書かない

    E220_DBG("E220 cfg: readback differs or unread, writing (persist=%d)", (int)persist);
    if (!reg_xfer(persist ? 0xC0 : 0xC2, 0x00, 6, v, nullptr)) {
        E220_DBG("E220 cfg: WRITE failed");
        return false;
    }

    // 書いたあとは必ず読み返して照合する。
    // 応答のエコーは「受け付けた」ことしか示さないので、実レジスタを読む。
    memset(rb, 0, sizeof(rb));
    if (!reg_xfer(0xC1, 0x00, 6, nullptr, rb)) {
        E220_DBG("E220 cfg: READBACK failed after write");
        return false;
    }
    const bool match = (memcmp(v, rb, sizeof(v)) == 0);
    if (!match)
        E220_DBG("E220 cfg MISMATCH want[%02X %02X %02X %02X %02X %02X] got[%02X %02X %02X %02X %02X %02X]",
                 v[0],v[1],v[2],v[3],v[4],v[5], rb[0],rb[1],rb[2],rb[3],rb[4],rb[5]);
    return match;
}

// ============================================================
//  ライフサイクル
// ============================================================

void e220_setup(uint8_t ch, uint8_t profile) {
    // AUX は E220 の push-pull 出力なので本来プルアップは不要。
    // ★ ただし **モジュール未実装・断線だと入力が浮いて Low に見えることがあり、
    //   それを「ビジー」と誤読すると永久に待つ**。内部プルアップを入れておけば、
    //   居ないときは High（アイドル）に見えて素通りする。
    //   モジュールが居れば push-pull が勝つので動作には影響しない。
    pinMode(E220_AUX_PIN, INPUT_PULLUP);
    pinMode(E220_MODE_PIN, OUTPUT);
    digitalWrite(E220_MODE_PIN, MODE_CONFIG);  // 起動直後は寝かせておく

    E220_SERIAL.setTX(E220_TX_PIN);
    E220_SERIAL.setRX(E220_RX_PIN);
    E220_SERIAL.setFIFOSize(256);              // ★受信側のリングバッファ。送信には効かない
    E220_SERIAL.begin(LINK_UART_BAUD);

    // 電源投入後のセルフチェックを待つ（AUX が Low → High）。
    // データシート 5.2：約 100ms の固定待ちで代用してよい。
    delay(120);

    // 起動時の 1 回だけ不揮発に書く。設定が前回と同じなら書込み自体が起きない。
    e220_reconfigure(ch, profile, true);
}

bool e220_reconfigure(uint8_t ch, uint8_t profile, bool persist) {
    if (ch > E220_CH_MAX) ch = E220_CH_MAX;

    set_mode(MODE_CONFIG);
    set_baud(LINK_UART_BAUD_CFG);      // mode 3 は 9600 固定
    s_alive = write_config(ch, profile, persist);
    if (s_alive) s_strict = ensure_strict_mode(persist);
    E220_DBG("E220 reconfigure ch=%u prof=%u persist=%d -> alive=%d AUX=%d",
             ch, profile, (int)persist, (int)s_alive, (int)digitalRead(E220_AUX_PIN));
    if (s_alive) note_response();      // 読み返しが通った＝確実に生きている

    // ---- 切り分け用: 0x08(VERSION) と 0x09(EXT REG) を読む（一時的）----
    // ★ write_config は 0x00〜0x05 しか触らない。**0x09 は一度も書いていない**が、
    //   ここは不揮発なので、過去に書かれた値がそのまま残っている可能性がある。
    //   0x09 bit6 = Strict Mode 切替フラグ。1 だとデータシート 6.5 のとおり
    //   **ver.1 互換コマンドが使えなくなる** → 6.8 の環境ノイズ取得
    //   (C0 C1 C2 C3 00 02) が無視される。表 24 の
    //   0xFF 0xFF 0xC1 0xA3 0x01 形式でないと読めない。
    //   送信もレジスタ操作も普通に動くので、症状は「雑音だけ取れない」になる。
    //   実機でそれが起きているため、まず値を確かめる。
    if (s_alive) {
        uint8_t ver = 0, ext = 0;
        const bool okv = reg_xfer(0xC1, 0x08, 1, nullptr, &ver);
        const bool oke = reg_xfer(0xC1, 0x09, 1, nullptr, &ext);
        E220_DBG("E220 regs: ver=%s%02X ext=%s%02X strict=%d rssiNoise=%d txmode=%d",
                 okv ? "0x" : "??", ver, oke ? "0x" : "??", ext,
                 oke ? ((ext >> 6) & 1) : -1,      // 1 なら ver.1 互換コマンドが死ぬ
                 (v_reg1_cache >> 5) & 1,          // REG1 bit5: 環境ノイズ有効
                 (v_reg3_cache >> 6) & 1);         // REG3 bit6: 1=通常 0=透過
    }

    set_baud(LINK_UART_BAUD);          // 通常運用へ戻す
    set_mode(MODE_NORMAL);
    s_asleep = false;
    // mode 3 でのやり取りの残りかすが受信の組み立てに混ざらないよう、
    // ここで捨てる。まだ受信していないので消えて困るものは無い。
    flush_input();

    return s_alive;
}

// 起動時の照合が通り、かつ直近の問い合わせに返事があるか。
// ★ 「設定が入っているか」と「今も応答するか」は別物なので両方を見る。
bool e220_alive() { return s_alive && (s_noResp < E220_NORESP_LIMIT); }

// ============================================================
//  送信
// ============================================================

bool e220_send(const uint8_t* payload, uint8_t n) {
    if (!s_alive || !payload || n == 0) return false;
    if (s_asleep) e220_wake();

    // ★ **通常(Fixed-block)送信モード**なので、ペイロードの前に
    //   宛先アドレス 2 バイト + 宛先チャンネル 1 バイトを付ける（6.6 / 図 24）。
    //   宛先 0xFFFF は**ブロードキャスト**で、同じ BW/SF/チャンネル/暗号キーの
    //   機体が自分のデバイス ID に関係なく全部受け取る（4.3）。PONS の
    //   「1 台が送り、複数が受ける」形にそのまま合う。
    //   送信した自モジュールが受信することはない（同節）。
    //
    // ★ 分割して write すると、その間に区切り時間（115200bps で >2ms）以上
    //   空いたときに E220 がそこでパケットを切ってしまう。
    //   1 回の write() で渡すこと。write() はブロックするので隙間はできない。
    if (n > sizeof(LinkTelem)) return false;

    // ★★ 送信前に AUX を見る（docs/pons_link.md §3）。
    //   Low = 前の送信・キャリアセンスの待ち・休止 50ms がまだ終わっていない。
    //   このまま write() すると **モジュールのバッファが空くまで Core0 が止まる**。
    //   テレメトリは最新値だけが意味を持つので、待たずにこのスロットを捨てる。
    //   ただし mode 3 から起こした直後は数 ms Low のことがあるので、そこは待つ。
    wait_aux_idle(10);
    if (digitalRead(E220_AUX_PIN) == LOW) return false;

    // ★ ヘッダと本体を**1 回の write() で**渡す。分けると区切り時間
    //   （115200bps で >2ms、表 3）を跨いだときにそこでパケットが切れる。
    uint8_t buf[3 + sizeof(LinkTelem)];
    buf[0] = 0xFF;          // 宛先 ADDH … 0xFFFF = ブロードキャスト
    buf[1] = 0xFF;          // 宛先 ADDL
    buf[2] = s_ch;          // 宛先チャンネル
    memcpy(buf + 3, payload, (size_t)n);
    E220_SERIAL.write(buf, (size_t)(3 + n));
    return true;
}

// ============================================================
//  受信
//    ペイロード + RSSI 1 バイトが並んで出てくる。
//    位相がずれても立て直せるよう、magic('P','L') を探して CRC で確定する。
// ============================================================

// magic は合ったが ver/type/CRC の検証に落ちたフレームの累計。
// 「届いたが化けた」を「そもそも来ない」と区別するための診断カウンタ。
static uint32_t s_badFrames = 0;
uint32_t e220_bad_frames() { return s_badFrames; }

// 65B ペイロードを 1 回送るのにかかる時間 [ms]（BW500kHz）。
// ★ SF8 の 98ms だけがメーカー公開の「E220-900T22X(JP) LoRa データ送信時間計算表」の実値。
//   他は「SF が 1 段上がるごとに約 1.8 倍」から外挿した概算なので、
//   **同じ表から取り直すこと**（docs/pons_link.md §3 の注意書きを参照）。
// 1Hz 運用なので、電波占有率 [%] は airtime_ms / 10 でそのまま出せる。
uint16_t e220_airtime_ms(uint8_t profile) {
    static const uint16_t tbl[E220_SF_MAX - E220_SF_MIN + 1] = { 55, 98, 180, 340, 600 };
    return (profile < (E220_SF_MAX - E220_SF_MIN + 1)) ? tbl[profile] : 0;
}

bool e220_recv(uint8_t* out, uint8_t frame_len, int16_t* rssi_dbm) {
    // magic + CRC の検査を LinkTelem として行うので、長さが違うと読み過ぎる。
    if (!out || frame_len != sizeof(LinkTelem)) return false;

    rx_pump();

    const uint16_t need = (uint16_t)frame_len + 1;   // ペイロード + RSSI
    for (uint16_t i = 0; i + need <= s_rxLen; i++) {
        if (s_rx[i] != 'P' || s_rx[i + 1] != 'L') continue;

        // ★ s_rx + i は任意オフセットなので、そのまま LinkTelem* にキャストすると
        //   アラインメント違反になる（Cortex-M33 で uint16/int32 の非整列アクセス）。
        //   必ず整列した領域へコピーしてから検査する。
        LinkTelem cand;
        memcpy(&cand, s_rx + i, sizeof(cand));
        if (!link_telem_valid(&cand)) {
            // ★ magic は合ったのに検証に落ちた＝**電波は届いたが化けている**。
            //   「そもそも来ていない」と区別できると、現場の対処がまるで変わる:
            //     壊れフレームが多い → 信号は届いている。SNR 不足。
            //                          アンテナの向き・位置・SF を上げる
            //     何も来ない         → 圏外・CH 違い・送信機が送っていない
            //   マージナルなリンクは全損より先に必ずここに現れるので、
            //   アンテナ調整の一番の指標になる。
            //   （雑音の中にたまたま "PL" が並んだ場合もここに入るが、
            //     それも「その CH に何か出ている」ことの手掛かりになる）
            s_badFrames++;
            continue;
        }

        memcpy(out, s_rx + i, frame_len);
        if (rssi_dbm) *rssi_dbm = (int16_t)s_rx[i + frame_len] - 256;
        note_response();               // 1 発でも受かった＝モジュールは生きている

        rx_drop_front((uint16_t)(i + need));   // 使った分まで詰める
        return true;
    }

    // 溜まりすぎたら古い方を捨てる（magic が見つからないまま埋まった場合）
    if (s_rxLen > RXBUF_SIZE - 8) rx_drop_front((uint16_t)(s_rxLen / 2));
    return false;
}

// ============================================================
//  省電力
// ============================================================

void e220_sleep() {
    if (s_asleep) return;

    // ★★ ここで flush() が要る。
    //   write() は「最後のバイトがハード FIFO に入った」時点で返るので、
    //   まだモジュールへ届いていないバイトが最大 32 バイト残っている
    //   （115200 で約 2.8ms）。その状態で M0/M1 を HIGH にすると、
    //   残りのバイトが **mode 3 のモジュールに届き、コマンドとして
    //   解釈される**（先頭が C0-C3 でなくても設定が化ける危険がある）。
    //   flush() は uart_tx_wait_blocking() なので、送り切るまで待つ。
    E220_SERIAL.flush();

    // ここから先は高速スリープ。**モード切替は AUX が High のときしか効かない**
    // ので、まだ送信中（AUX = Low）なら切替は自動的に先送りされ、
    // 送信を終えてから mode 3 に入る（データシート p.19）。待つ必要は無い。
    digitalWrite(E220_MODE_PIN, MODE_CONFIG);
    s_asleep = true;
}

void e220_wake() {
    if (!s_asleep) return;
    digitalWrite(E220_MODE_PIN, MODE_NORMAL);
    delay(2);          // 切替は 1ms（データシート 5.3）
    // ★ **ここで AUX の立ち上がりを待つ。** データシート 5.4 の 4 番と図 36:
    //   「電源投入時や Config/Deep sleep モードからの復帰時、初期化が完了すると
    //     AUX ピンが High レベルになります」
    //   復帰直後はセルフチェック中で AUX が Low。待たずに書くと、
    //   その書き込みが取りこぼされる。実機で `AUX stuck LOW (10ms timeout)` が
    //   起床直後の送信で毎回出ていたのはこれ。
    wait_aux_idle(50);
    s_asleep = false;
}

// 生存確認。**mode 3 へ潜ってレジスタを 1 バイト読む。**電波は出さない。
// ★ レジスタの汎用読み出し(0xC1)は mode 3 でしか使えない。データシート 6.5:
//   「通常送受信モード(mode 0) / WOR 送信モード(mode 1)において、データ送信、
//     データ受信、コマンド送受信が可能ですが、**レジスタ読み取り操作などはできません**」
//   （Strict Mode を有効にすれば mode 0 でも読めるが、本機は ver.1 互換のまま）。
// ★ 環境ノイズの取得とは**別の経路**。あちらは mode 0 専用（6.8）。混ぜると壊れる。
// 潜っている間（数十 ms）は受信できないので、呼び出し側が「受信できていないとき」に
// 限って呼ぶこと（link_probe_module 参照）。
bool e220_ping() {
    const bool was_asleep = s_asleep;
    set_mode(MODE_CONFIG);
    set_baud(LINK_UART_BAUD_CFG);      // mode 3 は 9600 固定
    delay(E220_CFG_SETTLE_MS);
    uint8_t rb = 0;
    const bool ok = reg_xfer(0xC1, 0x00, 1, nullptr, &rb);
    set_baud(LINK_UART_BAUD);
    if (!was_asleep) set_mode(MODE_NORMAL);
    if (ok) note_response(); else note_no_response();
    return ok;
}
// mode 0 / mode 1 のまま状態レジスタ(0xA0〜0xA4)を読む。Strict Mode 必須（10.1 表 24）。
//   送信: FF FF C1 <reg> <len>
//   応答: FF FF C1 <reg> <len> <値...>
//
// ★★ **返事が無い＝書式が違う。** データシート 6.15:
//   「状態レジスタの取得コマンドは、…コマンドとして解されない場合(単に宛先チャンネル
//     部分が間違っている場合や取得レジスタの範囲指定が正しくない場合)に
//     **エラー応答はありません**」
//   つまり got=0 は「モジュールが死んでいる」ではなく「こちらの投げ方が違う」。
//   実機ではここで長く詰まった。**長さは 7.1 の実例どおり 1 レジスタずつ**読む。
// ★ ver.1 互換の専用コマンド (6.8 / C0 C1 C2 C3 00 02) は**使わない**。
//   実機（v7・firmware v2.0）で返事ゼロ、直後に AUX が Low ＝ 電波として出ていた。
//   送信すべきでない機体が送信するので、s_strict が false なら何もしないこと。
// ★ mode 3 へ潜ってもいけない。mode 3 では 0xC0 が「レジスタ書き込み」なので、
//   ver.1 のコマンド列が長さ 194 の書き込みと解釈されモジュールが固まる（実機で確認）。
static bool stat_reg_read(uint8_t reg, uint8_t len, uint8_t* out) {
    if (s_asleep || !s_strict || len == 0 || len > 8 || !out) return false;

    const uint8_t cmd[5] = { 0xFF, 0xFF, 0xC1, reg, len };

    // ★★ ここで受信バッファを捨ててはいけない。mode 0 のまま、テレメトリと同じ
    //   UART を使って問い合わせるため。捨てるとテレメトリも一緒に消え、
    //   **電波を確かめようとした画面が受信を殺す**。応答の分だけを抜き取る。
    rx_pump();
    if (s_rxLen > RXBUF_SIZE - 96) rx_drop_front((uint16_t)(s_rxLen / 2));
    const uint16_t from = s_rxLen;

    // ★ モジュールがビジーのまま投げない（5.3）。セルフチェック中・送信中の
    //   書き込みは取りこぼされる。空かないならこの回は諦める。
    wait_aux_idle(50);
    if (digitalRead(E220_AUX_PIN) == LOW) return false;

    E220_SERIAL.write(cmd, sizeof(cmd));
    E220_SERIAL.flush();

    const uint32_t wait_ms = (s_noResp == 0) ? E220_NOISE_WAIT_MS
                                             : E220_NOISE_WAIT_FAST_MS;
    const uint16_t need = (uint16_t)(5 + len);
    const uint32_t t0 = millis();
    while (millis() - t0 < wait_ms) {
        rx_pump();
        for (uint16_t i = from; (uint16_t)(i + need) <= s_rxLen; i++) {
            if (s_rx[i]     != 0xFF || s_rx[i + 1] != 0xFF ||
                s_rx[i + 2] != 0xC1 || s_rx[i + 3] != reg ||
                s_rx[i + 4] != len) continue;
            memcpy(out, s_rx + i + 5, len);
            // 応答の分だけを抜く。前後のテレメトリは壊さない。
            memmove(s_rx + i, s_rx + i + need, (size_t)(s_rxLen - (i + need)));
            s_rxLen = (uint16_t)(s_rxLen - need);
            note_response();
            return true;
        }
        delay(2);
    }

    note_no_response();
    const uint16_t g = (uint16_t)(s_rxLen - from);
    E220_DBG("E220 stat %02X len=%u: no reply (waited %ums, got=%u "
             "[%02X %02X %02X %02X %02X %02X] AUX=%d)",
             reg, len, (unsigned)wait_ms, (unsigned)g,
             g > 0 ? s_rx[from + 0] : 0, g > 1 ? s_rx[from + 1] : 0,
             g > 2 ? s_rx[from + 2] : 0, g > 3 ? s_rx[from + 3] : 0,
             g > 4 ? s_rx[from + 4] : 0, g > 5 ? s_rx[from + 5] : 0,
             (int)digitalRead(E220_AUX_PIN));
    // ★ **帳簿(s_asleep)ではなくピンの実レベルを見る。** e220_wake() は
    //   s_asleep が false だと何もせずに返るので、両者がずれていると
    //   「mode 3 のモジュールへ 115200 で話しかける」状態になりうる。
    //   MODE=1(HIGH) は mode 3。そのときは書式ではなくモードが原因。
    E220_DBG("E220 stat %02X: MODE=%d(%s) asleep=%d",
             reg, (int)digitalRead(E220_MODE_PIN),
             digitalRead(E220_MODE_PIN) == MODE_CONFIG ? "cfg/mode3" : "normal/mode0",
             (int)s_asleep);
    return false;
}

// 環境ノイズ(0xA3)と、要求されたときだけ直前の受信 RSSI(0xA4)。表 17。
//   dBm = レジスタ値 - 256。
bool e220_read_noise(int16_t* noise_dbm, int16_t* last_rssi_dbm) {
    uint8_t v = 0;
    if (!stat_reg_read(0xA3, 1, &v)) return false;
    if (noise_dbm) *noise_dbm = (int16_t)v - 256;
    if (last_rssi_dbm) {
        uint8_t l = 0;
        *last_rssi_dbm = stat_reg_read(0xA4, 1, &l) ? (int16_t)l - 256 : -128;
    }
    return true;
}

