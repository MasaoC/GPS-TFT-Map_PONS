// ============================================================
// File    : link_proto.h
// Project : PONS v7 — PONS Link (機体⇄ボート テレメトリ)
// Role    : 電波に載せるテレメトリ構造体と、無線モジュールとの
//           UART フレーム形式の定義。
// Author  : MasaoC (@masao_mobile)
// ============================================================
//
// ■ このファイルが持つもの
//     ・LinkTelem  … 電波に載る 65 バイトそのもの
//     ・LinkRxTelem… 受信結果を RP 内部で持ち回す入れ物
//     ・RHAVE_*    … mysd.h からも参照される「値の有無」ビット
//     ・CRC と検査 … 送受で同じ実装を使うため
//   モジュールの叩き方（UART・モード制御・レジスタ）は e220.h / e220.cpp。
//
// ■ ★ 無線方式に依存しない作りにしてある
//   電波の設定は radio_ch / radio_profile という 2 つの数値だけで、
//   意味を決めるのは無線層。LoRa では radio_ch = E220 のチャネル番号
//   （CH0-12 = 920.8-923.2MHz）、radio_profile = SF のプリセット。
//   **無線層の都合をこのファイルより上へ漏らさないこと。**
//   実際、方式を差し替えても上位（link.cpp / display_tft.cpp）は無変更で通った。
//
// ■ 設計の前提（詳細は docs/pons_link.md §4）
//   ・通信は下り一方向のみ。受信側から送信側へ返すものは何も無い。
//   ・ブロードキャスト。ペアリングも宛先指定も不要。
//   ・異物の排除は magic + ver + CRC16 で行う。
//
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

// 版数。構造体レイアウトを変えたら必ず上げる。
//   v3 で src_id を足したが、重複送信機の検出が seq の逆行で足りると分かり
//   （src_id の既定値は 3 台とも同じで、一番ありそうな誤設定では一致してしまう）、
//   表示以外に使い道が無かったので削除した。レイアウトは v2 に戻っている。
#define LINK_PROTO_VER      2

// ============================================================
//  UART のボーレート
//   ★ 115200 を使う。arduino-pico の write() はハード FIFO(32B) が空くまで
//     スピンするので、ボーレートがそのまま Core0 のブロック時間になる。
//       9600  : 68 バイトで 37.5 ms   ← マップ描画 64ms と合わせて許容できない
//     115200  : 68 バイトで  3.1 ms
//
//     Config/DeepSleep モード(mode 3) は **レジスタ設定に関わらず常に 9600 8N1**
//     なので、設定を読み書きする間だけ 9600 へ落とす。切替は起動時と
//     設定変更時だけで、毎パケットではない（e220.cpp が面倒を見る）。
//
//   ★★ E220 は「一定時間データが来ない」ことをパケットの区切りとみなす。
//     115200bps では **区切り検出 >2ms**（データシート 表3）。
//     ただし write() がブロックする＝バイト列に隙間ができないので、
//     1 回の write() にまとめて渡す限り分裂しない。**分割して書かないこと。**
// ============================================================
#define LINK_UART_BAUD      115200   // 通常送受信(mode 0)
#define LINK_UART_BAUD_CFG    9600   // Config/DeepSleep(mode 3)。固定値で変更不可

// ============================================================
//  送信機の重複検出
//    送信機が 2 台同時にいるのは異常事態（機体用とボート用を取り違えた等）。
//    ★ 検出できるのは **受信機だけ**。LoRa の送信機は送信の合間 mode 3 で
//      寝ていて受信できないため。
//    ★ 判定は **seq の逆行**で行う（link.cpp）。送信機が 1 台なら seq は必ず
//      増えるので、減る・止まるのは 2 台いる証拠になる。送信元 ID を持たせて
//      比較する案もあったが、既定値が 3 台とも同じになるため
//      「設定を触っていない 2 台がどちらも TX」という一番ありそうな誤設定で
//      黙って失敗する。運用に依存しない seq のほうが確実。
// ============================================================

// 動作モード。3 択にしてあるので「送受同時 ON」を表現できない。
typedef enum {
    LINK_MODE_OFF = 0,
    LINK_MODE_TX  = 1,          // 機体
    LINK_MODE_RX  = 2,          // ボート／ピット
} LinkMode;

// radio_profile は「無線層が意味を決める数値」。LoRa では拡散率のプリセット。
//   0=SF7 / 1=SF8(既定) / 2=SF9 / 3=SF10 / 4=SF11
//   SF を 1 段上げると 2.5dB 強くなるが、送信時間＝送信電流が倍々に増える。
#define LINK_PROFILE_COUNT   5
// E220 の BW500kHz で使うのは CH0-12（920.8-923.2MHz）。
// ここが休止 50ms 固定の帯域で、400ms 制限も 10 倍休止もかからない（docs/pons_link.md §7）。
#define LINK_RADIO_CH_MAX   12

// ============================================================
//  have ビット（RHAVE_*）
//    「その項目が入っているか」を示すビットマスク。
//    ★ 元は mysd.h にあったが、電波に載せるテレメトリからも参照するので
//      **共有ヘッダであるここへ移した**。mysd.h はこのファイルを
//      include しているので、RP 側のコードは今までどおり使える。
//      定義が 2 か所にあると必ずずれるので、実体はここ 1 つだけにする。
//
//    SD のリプレイ（imu_replaydata/ など）でも同じビットを使っているため、
//    無線で受けたフレームを既存のリプレイ経路にそのまま流せる。
// ============================================================
#define RHAVE_GS       0x0001
#define RHAVE_TTRACK   0x0002
#define RHAVE_GNSSALT  0x0004
#define RHAVE_KFALT    0x0008
#define RHAVE_KFVS     0x0010
#define RHAVE_PRESS    0x0020
#define RHAVE_DATE     0x0040
#define RHAVE_NUMSAT   0x0080
#define RHAVE_VOLT     0x0100
// 姿勢の項目。RHAVE_ATT 以外は ESKF の結果を持つ新形式にしか無い。
#define RHAVE_ATT      0x0200   // ロール・ピッチ
#define RHAVE_ATT_YAW  0x0400   // ヨー + ヨー精度95%値
#define RHAVE_ATT_AVG  0x0800   // 平均ピッチ
#define RHAVE_ATT_TRIM 0x1000   // 自動ロールトリムの累積補正量
#define RHAVE_ATT_WIND 0x2000   // 風速・風向（推定できていた区間のみ）

// type の値。0 = テレメトリ。将来ほかの種類を足すための欄で、
// 受信側は 0 以外を捨てる（link_telem_valid）。
#define LINK_TYPE_TELEM     0

// ============================================================
//  テレメトリ本体
//    UART と電波の両方で、この構造体がそのまま流れる。
//    詰め方は「表示に必要な分解能ぎりぎり」。ペイロードを増やすと
//    受信窓と送信電流の両方に直接効くので、安易に足さないこと（docs/pons_link.md §4）。
//
//    have ビットは上の RHAVE_* をそのまま使う。SD のリプレイと同じ定義なので、
//    受信モードを既存のリプレイ経路に相乗りさせられる（表示側の改造が要らない）。
// ============================================================
typedef struct __attribute__((packed)) {
    // --- ヘッダ（異物の排除用）---
    uint8_t  magic0;            // 'P'
    uint8_t  magic1;            // 'L'
    uint8_t  ver;               // LINK_PROTO_VER
    uint8_t  type;              // 0 = テレメトリ（将来の拡張用）

    uint16_t seq;               // 連番。ロールオーバーしてよい
    uint32_t tx_us;             // 送信側 esp_timer の下位32bit [us]

    // --- 時刻（GPS）---
    uint16_t year;
    uint8_t  month, day, hour, minute, second, centi;

    // --- 位置・対地 ---
    int32_t  lat_1e7;           // 緯度 ×1e-7 度
    int32_t  lon_1e7;           // 経度 ×1e-7 度
    uint16_t gs_cms;            // 対地速度 [0.01 m/s]
    uint16_t track_cdeg;        // 真方位 [0.01 度]

    // --- 高度・気圧 ---
    int16_t  gnss_alt_dm;       // GNSS 高度 [0.1 m]
    int16_t  kf_alt_dm;         // KF 高度   [0.1 m]
    int16_t  kf_vs_cms;         // KF 昇降速度 [0.01 m/s]
    uint16_t press_dpa;         // 気圧 [0.1 hPa]

    // --- 姿勢（ESKF）---
    int16_t  roll_cdeg;         // [0.01 度]
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    int16_t  pitch_avg_cdeg;    // 平均ピッチ
    int16_t  roll_trim_cdeg;    // 自動ロールトリムの累積
    uint8_t  yaw_acc95_deg;     // ヨー精度95% [度]

    // --- 風 ---
    uint8_t  wind_dmps;         // 風速 [0.1 m/s]
    uint16_t wind_dir_cdeg;     // 風向（吹いていく方向）[0.01 度]

    // --- 機体状態 ---
    uint8_t  volt_cv;           // 電圧: (V - 2.5) × 100 → 2.50〜5.05V
    uint8_t  numsat;
    uint16_t hacc_dm;           // 水平精度 [0.1 m]
    uint8_t  fixflags;          // bit0: gnssFixOK

    // --- ナビの設定（受信側は「同じか」を確かめるためだけに使う）---
    // 受信側の警告は自分で再計算する（下の設計方針）。その計算に効くのは
    // 目的地とナビモードだけなので、それだけを載せて突き合わせる。
    int8_t   dest_index;        // 送信側が選んでいる目的地（-1 = 未選択）
    uint8_t  nav_mode;          // 送信側のナビモード（DMODE_*）

    uint16_t have;              // RHAVE_* と同一のビットマスク
    uint16_t status;            // LINK_ST_* のビット

    uint16_t crc16;             // magic0 から status までの CRC16-CCITT
} LinkTelem;

// ★ 65 バイトは仕様書（docs/pons_link.md §4）と、そこから導いた
//   送信時間・送信電流・電波法の検討がすべて前提にしている数値。
//   項目を足して黙って伸びると、その検討が全部ずれる。ビルドで止める。
static_assert(sizeof(LinkTelem) == 65, "LinkTelem が 65 バイトでなくなっている");

// ============================================================
//  status のビット ― 設計方針
//    ★ **受信側で再計算できるものは載せない。**
//
//    飛行の警告（コース警告・バンク角警告・バリオ音）は、すべて
//    ミラーされたセンサ値から受信側が自分で計算する。
//    実際 steer_angle = truec - ttrack で ttrack はミラー済みなので、
//    コース警告とバリオ音は**何もしなくても受信側で再現される**。
//    ビットで送って表示を分岐させるより、計算を 1 本にしておくほうが単純で、
//    送信側と受信側で挙動がずれる余地も無い。
//
//    したがって status に載せるのは
//    **「受信側からは知りようがない、送信機自身の健康状態」だけ**にする。
//    どれも飛ぶ前に気づきたいもので、飛行中の値からは推し量れない。
#define LINK_ST_SD_ERROR    0x0001   // SD が使えない（ログが残らない）
#define LINK_ST_IMU_ERROR   0x0002   // BNO085 が応答しない（姿勢が出ない）
#define LINK_ST_NEEDS_APPLY 0x0004   // ロールトリムが未適用（設定し忘れ）
#define LINK_ST_BAT_LOW     0x0008   // 送信機の電池が少ない

// ============================================================
//  電圧の詰め方
//    2.50〜5.05V を 1 バイトに収める。0.01V 刻みで十分。
// ============================================================
static inline uint8_t link_pack_volt(float v) {
    float x = (v - 2.5f) * 100.0f;
    if (x < 0)   x = 0;
    if (x > 255) x = 255;
    return (uint8_t)(x + 0.5f);
}
static inline float link_unpack_volt(uint8_t c) { return 2.5f + c * 0.01f; }

// ============================================================
//  受信したテレメトリ＋受信時の付帯情報
//    RP2350 の内部で持ち回すための構造体。電波には載らない。
// ============================================================
typedef struct __attribute__((packed)) {
    LinkTelem t;
    int8_t    rssi;             // 受信強度 [dBm]
    uint32_t  age_ms;           // 受信してからの経過時間 [ms]
} LinkRxTelem;

// ============================================================
//  CRC16-CCITT (0x1021, 初期値 0xFFFF)
//  両側で同じ実装を使うため、ここに置いて共有する。
// ============================================================
static inline uint16_t link_crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// テレメトリの CRC を計算する（crc16 フィールド自身は除く）
static inline uint16_t link_telem_crc(const LinkTelem* t) {
    return link_crc16((const uint8_t*)t, (uint16_t)(sizeof(LinkTelem) - 2));
}

// 受け取ったテレメトリが自分たちのものかを判定する。
// magic / ver / CRC の 3 つを通らないものは捨てる。
// これで足りるので、グループ ID のような識別子は持たせていない。
static inline bool link_telem_valid(const LinkTelem* t) {
    if (t->magic0 != 'P' || t->magic1 != 'L') return false;
    if (t->ver != LINK_PROTO_VER)             return false;
    if (t->type != LINK_TYPE_TELEM)           return false;
    return t->crc16 == link_telem_crc(t);
}

#endif // LINK_PROTO_H
