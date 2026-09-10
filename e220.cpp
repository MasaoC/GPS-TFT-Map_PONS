// ============================================================
// File    : e220.cpp
// Project : PONS v7 — PONS Link
// Role    : E220-900T22S(JP) の下位ドライバ実装
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/08
// ============================================================

#include "e220.h"
#include <string.h>
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
        if (millis() - t0 > timeout_ms) return;
    }
    delay(2);
}

static void set_mode(int level) {
    wait_aux_idle(200);
    digitalWrite(E220_MODE_PIN, level);
    delay(2);                 // 切替は 1ms。余裕を見て 2ms
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
    const uint32_t t0 = millis();
    while (millis() - t0 < 300) {
        while (E220_SERIAL.available() && got < sizeof(buf)) buf[got++] = (uint8_t)E220_SERIAL.read();
        if (got >= (uint8_t)(3 + len)) break;
        delay(2);
    }

    for (uint8_t i = 0; (uint16_t)i + 3 + len <= got; i++) {
        if (buf[i] == 0xC1 && buf[i + 1] == addr && buf[i + 2] == len) {
            if (out) memcpy(out, buf + i + 3, len);
            return true;
        }
    }
    return false;
}

// 設定一式を書いて、読み返して照合する。
//   persist = true なら 0xC0（不揮発。書換え寿命あり）、false なら 0xC2（RAM のみ）。
static bool write_config(uint8_t ch, uint8_t profile, bool persist) {
    const uint8_t sf = e220_profile_to_sf(profile);

    uint8_t v[6];
    v[0] = 0x00;                                   // 00H ADDH
    v[1] = 0x00;                                   // 01H ADDL
    v[2] = (uint8_t)((BAUD_BITS_115200 << 5) | air_data_rate_bits(sf));  // 02H REG0
    // 03H REG1: subpacket(00=200B)<<6 | 環境ノイズ有効<<5 | 送信出力(01=13dBm)
    v[3] = (uint8_t)((0b00 << 6) | (1 << 5) | 0b01);
    v[4] = ch;                                     // 04H REG2 チャンネル
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
    v[5] = (uint8_t)((1 << 7) | (0 << 6) | 0b011);

    // ★ 先に読んで、既に同じなら書かない。
    //   書込みコマンド 0xC0 は **不揮発メモリ（フラッシュ）に保存する**ので
    //   書換え回数に寿命がある。設定は電源を切っても残るため、毎回の起動で
    //   書き直す必要は無い。
    uint8_t rb[6] = {0};
    if (reg_xfer(0xC1, 0x00, 6, nullptr, rb) && memcmp(v, rb, sizeof(v)) == 0)
        return true;                       // 既に正しい。何も書かない

    if (!reg_xfer(persist ? 0xC0 : 0xC2, 0x00, 6, v, nullptr)) return false;

    // 書いたあとは必ず読み返して照合する。
    // 応答のエコーは「受け付けた」ことしか示さないので、実レジスタを読む。
    memset(rb, 0, sizeof(rb));
    if (!reg_xfer(0xC1, 0x00, 6, nullptr, rb)) return false;
    return memcmp(v, rb, sizeof(v)) == 0;
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
    set_baud(LINK_UART_BAUD);          // 通常運用へ戻す
    set_mode(MODE_NORMAL);
    s_asleep = false;
    // mode 3 でのやり取りの残りかすが受信の組み立てに混ざらないよう、
    // ここで捨てる。まだ受信していないので消えて困るものは無い。
    flush_input();
    return s_alive;
}

bool e220_alive() { return s_alive; }

// ============================================================
//  送信
// ============================================================

bool e220_send(const uint8_t* payload, uint8_t n) {
    if (!s_alive || !payload || n == 0) return false;
    if (s_asleep) e220_wake();

    // トランスペアレント送信なので宛先ヘッダは要らない。
    // 自分の ADDH/ADDL(0x0000) と CH の設定どおりに出て、
    // 同じ設定の機体が全部受け取る＝そのままブロードキャストになる。
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

    E220_SERIAL.write(payload, (size_t)n);
    return true;
}

// ============================================================
//  受信
//    ペイロード + RSSI 1 バイトが並んで出てくる。
//    位相がずれても立て直せるよう、magic('P','L') を探して CRC で確定する。
// ============================================================

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
        if (!link_telem_valid(&cand)) continue;

        memcpy(out, s_rx + i, frame_len);
        if (rssi_dbm) *rssi_dbm = (int16_t)s_rx[i + frame_len] - 256;

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
    s_asleep = false;
}

// ============================================================
//  環境ノイズ（データシート 6.8 / 8.7）
//    送信: C0 C1 C2 C3 00 02
//    応答: C1 00 02 <環境ノイズ> <前回受信時RSSI>   dBm = 値 - 256
//    ★ mode 0 でのみ有効。レジスタ 0x03 bit5 を 1 にしてあること。
// ============================================================

bool e220_read_noise(int16_t* noise_dbm, int16_t* last_rssi_dbm) {
    if (s_asleep) return false;
    const uint8_t cmd[6] = { 0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02 };

    // ★★ ここで受信バッファを捨ててはいけない。
    //   この関数は **mode 0 のまま**、テレメトリと同じ UART を使って問い合わせる。
    //   受信モードで無線設定画面を開いている間は 1Hz で呼ばれるので、
    //   捨てるとテレメトリも 1Hz で消え、**電波を確かめようとした画面が
    //   受信を殺す**という最悪の壊れ方をする。
    //   応答も受信データも同じ組み立てバッファへ入れ、応答の 5 バイトだけを
    //   抜き取る。残りは e220_recv() が今までどおり拾う。
    rx_pump();
    if (s_rxLen > RXBUF_SIZE - 96) rx_drop_front((uint16_t)(s_rxLen / 2));  // 応答の置き場を確保
    const uint16_t from = s_rxLen;   // 応答は必ずこれ以降に来る

    E220_SERIAL.write(cmd, sizeof(cmd));
    E220_SERIAL.flush();

    const uint32_t t0 = millis();
    while (millis() - t0 < 200) {
        rx_pump();
        for (uint16_t i = from; (uint16_t)(i + 5) <= s_rxLen; i++) {
            if (s_rx[i] != 0xC1 || s_rx[i + 1] != 0x00 || s_rx[i + 2] != 0x02) continue;
            if (noise_dbm)     *noise_dbm     = (int16_t)s_rx[i + 3] - 256;
            if (last_rssi_dbm) *last_rssi_dbm = (int16_t)s_rx[i + 4] - 256;
            // 応答の 5 バイトだけを抜く。前後のテレメトリは壊さない。
            memmove(s_rx + i, s_rx + i + 5, (size_t)(s_rxLen - (i + 5)));
            s_rxLen = (uint16_t)(s_rxLen - 5);
            return true;
        }
        delay(2);
    }
    return false;
}
