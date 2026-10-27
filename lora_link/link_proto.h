// ============================================================
// File    : link_proto.h
// Project : PONS v7 — PONS Link (機体⇄ボート テレメトリ)
// Role    : テレメトリ構造体（LinkTelem。メモリの上の形）と、無線モジュールとの UART の約束。
//           電波に載せる 32 バイトの形は lora_link/link_wire2.h（2026-10-27〜）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/27
// ============================================================
//
// ■ このファイルが持つもの
//     ・LinkTelem  … テレメトリの 65 バイト（メモリの上の形）。**電波には 32 バイトに詰めて載せる**
//                    （lora_link/link_wire2.h。2026-10-27〜）
//     ・LinkRxTelem… 受信結果を RP 内部で持ち回す入れ物
//     ・RHAVE_*    … mysd.h からも参照される「値の有無」ビット
//     ・CRC16      … 送受で同じ実装を使うため（電波のフレームの検査は link_wire2.h の link_w2_valid）
//   モジュールの叩き方（UART・モード制御・レジスタ）は e220.h / e220.cpp。
//
// ■ ★ 無線方式に依存しない作りにしてある
//   電波の設定は radio_ch / radio_profile という 2 つの数値だけで、
//   意味を決めるのは無線層。LoRa では radio_ch = E220 のチャネル番号
//   （使うのはローワーの 0/3/6/9/12 とアッパーの 15/21/27。下の LINK_RADIO_CHS）、radio_profile = SF のプリセット。
//   **無線層の都合をこのファイルより上へ漏らさないこと。**
//   実際、方式を差し替えても上位（link.cpp / display_tft.cpp）は無変更で通った。
//
// ■ 設計の前提（詳細は docs/pons_link.md §4）
//   ・通信は下り一方向のみ。受信側から送信側へ返すものは何も無い。
//   ・ブロードキャスト。ペアリングも宛先指定も不要。
//   ・異物の排除は、電波に載せる 32 バイトのフレームの印（LINK_W2_MAGIC）と CRC16 で行う（link_wire2.h）。
//
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

// LinkTelem の版数。構造体レイアウトを変えたら上げる。
// ★ **2026-10-27 からこの版数は電波に載らない。** 電波には 32 バイトのフレーム（link_wire2.h）が載り、
//   古い形との食い違いは**そちらの印 LINK_W2_MAGIC** で見分ける（形を変えたら印を変える）。
//   32 バイトの予備ビットに意味を与えるときは、古い機体が送る 0 を「無し」と読めるようにすること
//   （0 を本物の値として読むと黙って壊れる。v4 で縮尺 0 番が最広域に張り付いたのと同じ形）。
//   3: 使われていなかった tx_us（送信側 micros）を group + 予備 3 バイトへ置き換えた。
//   4: scale_index を追加（0.988）
//   5: 予備 2 バイトを dest_id（目的地の ID）にした（2026-10-27）。dest_id は 32 バイトの回転枠 7 の
//      予備（以前は必ず 0）に載せ、**0 =「ID を送っていない」**と決めてある。ID を送らない機体
//      （2026-10-26 までの 32 バイト・debug/v7-noise-test）とは、以前どおり番号で合わせる。
#define LINK_PROTO_VER      5

// ============================================================
//  UART のボーレート
//   ★ 115200 を使う。arduino-pico の write() はハード FIFO(32B) が空くまで
//     スピンするので、ボーレートがそのまま Core0 のブロック時間になる。
//       9600  : 68 バイトで 37.5 ms   ← マップ描画 64ms と合わせて許容できない
//     115200  : 68 バイトで  3.1 ms
//     （65 バイトを送っていた頃の値。2026-10-27 からは 32 バイト + 宛先 3 = 35 バイトで、115200 なら 0.3ms ほど）
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
// E220 の BW500kHz で、ふだん使うのは CH0-12（920.8-923.2MHz、ローワーバンド）。
// ここが休止 50ms 固定の帯域で、400ms 制限も 10 倍休止もかからない（docs/pons_link.md §7）。
// ★ 名前は「CH の上限」のままだが、意味は**ローワーバンドの上限**。アッパーの CH は下の LINK_UPPER_CHS。
#define LINK_RADIO_CH_MAX   12

// ★ 選べるのは **CH0 / 3 / 6 / 9 / 12 の 5 つだけ**（LINK_RADIO_CH_STEP 刻み。2026-10-23〜）。
//   帯域幅 500kHz に対してチャンネルの間隔は 200kHz なので、隣の CH とは 300kHz、
//   2 つ隣とも 100kHz 帯域が重なる。重ならないのは 3 つ（0.6MHz）以上離したときで、
//   CH0〜12 に収まるのがこの 5 つ。間の CH を選んでも隣のチームと帯域を分け合うだけで、
//   得るものが無い（狭い妨害波を避けるのも、この 5 つの中で足りる）。
//   ★ それでも至近距離（同じ格納庫）では 0.6MHz 離しても漏れ込み、送信前チェックは
//     BUSY になる（2026-10-22 の実測。CH9 の送信機から CH12 で −62dBm）。
//   ★ 無線層（e220.cpp）は 0〜12 のどれでも設定できる。絞っているのはこの層の方針。
#define LINK_RADIO_CH_STEP   3
static_assert(LINK_RADIO_CH_MAX % LINK_RADIO_CH_STEP == 0, "最後の CH が刻みに乗っていない");

// ★ アッパーバンド（CH13〜30）から **CH15 / 21 / 27 の 3 つも選べる**（2026-10-27〜）。
//   923.5MHz から上に BW500kHz が丸ごと入る CH15 から、帯域が重ならない 1.2MHz おき
//   （923.8 / 925.0 / 926.2MHz）。大会当日にローワーが混んでいたときの逃げ場。
//   アッパーは 1 回送るごとに休止（送信時間の 10 倍）が要り、1 パケットが 32 バイトに固定されるので、
//   送るのはどの CH でも同じ 32 バイトのフレーム（lora_link/link_wire2.h）で、アッパーでは休止に合わせて
//   送る間隔を延ばす。決まりと送る間隔は docs/pons_link.md §2「アッパーバンド」。
#define LINK_UPPER_CHS      15, 21, 27
#define LINK_UPPER_CH_COUNT  3
#define LINK_LOWER_CH_COUNT  (LINK_RADIO_CH_MAX / LINK_RADIO_CH_STEP + 1)
#define LINK_ALL_CH_COUNT    (LINK_LOWER_CH_COUNT + LINK_UPPER_CH_COUNT)
// 選べる CH を小さい順に（ローワー 5 つ、続けてアッパー 3 つ）。
static const uint8_t LINK_RADIO_CHS[LINK_ALL_CH_COUNT] = { 0, 3, 6, 9, 12, LINK_UPPER_CHS };
static_assert(LINK_LOWER_CH_COUNT == 5, "LINK_RADIO_CHS のローワーの並びを直すこと");

// アッパーバンドの CH か（休止の決まりがかかる CH か）。
static inline bool link_radio_ch_upper(int ch) { return ch > LINK_RADIO_CH_MAX; }
// 選べる CH か。
static inline bool link_radio_ch_valid(int ch) {
    for (uint8_t i = 0; i < LINK_ALL_CH_COUNT; i++) if (LINK_RADIO_CHS[i] == ch) return true;
    return false;
}
// 一番近い選べる CH へ寄せる（以前の settings.txt に残った CH10 → 9、CH11 → 12、CH13 → 12 など）。
// ★ 3 台が同じ版なら必ず同じ値に揃うよう、決まった規則で丸めること。ちょうど中間なら大きいほう（四捨五入）。
static inline uint8_t link_radio_ch_snap(int ch) {
    uint8_t best = LINK_RADIO_CHS[0];
    for (uint8_t i = 1; i < LINK_ALL_CH_COUNT; i++) {
        const int d  = ch - (int)LINK_RADIO_CHS[i];
        const int db = ch - (int)best;
        if ((d < 0 ? -d : d) <= (db < 0 ? -db : db)) best = LINK_RADIO_CHS[i];   // 小さい順なので <= で中間は大きいほう
    }
    return best;
}
// 設定画面で長押しするたびの次の CH（0 → 3 → … → 12 → 15 → 21 → 27 → 0）。
static inline uint8_t link_radio_ch_next(uint8_t ch) {
    for (uint8_t i = 0; i < LINK_ALL_CH_COUNT; i++)
        if (LINK_RADIO_CHS[i] > ch) return LINK_RADIO_CHS[i];
    return LINK_RADIO_CHS[0];
}

// ============================================================
//  グループ ID
//    ★ **同じ会場に別チームの PONS がいたときに取り違えないための識別子。**
//      CH と SF が偶然一致すると、magic も ver も CRC も通ってしまうため、
//      今までは「CH/SF の一致」が事実上のペアリングになっていた。
//      それだけでは、設定を探索する機能（自動スキャン等）を足した瞬間に
//      他チームの機体を掴んでミラーしてしまう。
//
//    ★ **値そのものはソースにも SD の雛形にも書かない。** 既定は 0 のままにして、
//      実際の値は各機の SD カード（settings.txt の link_group）にだけ置く。
//      git に載るのは「仕組み」だけで「値」ではない。
//
//    ★ 受信側は **group 違いを「無信号」ではなく「別グループ」として区別する**こと。
//      SD が飛んで 0 に戻ったとき、黙って無反応になると原因に辿り着けない。
// ============================================================
//    ★ 0〜255。**設定画面の長押しは +1 ずつ**なので、実運用では小さい値
//      （1〜20 程度）を割り当てること。SD が飛んで 0 に戻ったとき、
//      現地で押し戻すのが速い。大きい値は SD からしか実用的に設定できない。
#define LINK_GROUP_DEFAULT  0
#define LINK_GROUP_MAX    255

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

// type の値。0 = テレメトリ。将来ほかの種類を足すための欄。
// ★ LinkTelem の magic / ver / type / crc16 は 65 バイトを送っていた頃の検査用。電波には載らなくなった
//   （2026-10-27）ので、いまは検査していない。形と値はメモリの上の LinkTelem のために残してある。
#define LINK_TYPE_TELEM     0

// ============================================================
//  テレメトリ本体（メモリの上の形）
//    2026-10-27 までは UART と電波の両方でこの構造体がそのまま流れていた。いまは送る直前に
//    link_wire2.h が 32 バイトに詰め、受けたらすぐ戻す。上の層はこの 65 バイトのまま扱う。
//    電波に載る大きさは受信窓と送信電流の両方に直接効くので、項目を安易に足さないこと
//    （足すなら 32 バイトの核か回転枠のどこに載せるかも決める。docs/pons_link.md §4）。
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
    uint8_t  group;             // グループ ID。一致しない機体は表示に使わない
    // 送信側の地図の縮尺（scalelist の添字）。受信側はこれに**合わせる**。
    // ★ 受信機の目的は「パイロットに出ている画面の再現」なので、縮尺が違うと
    //   同じ地図に見えない。送信機が誤った縮尺のまま飛んでいることにも気づける。
    uint8_t  scale_index;
    // 送信側が選んでいる目的地の ID（名前と座標の CRC16。0 = 送っていない・未選択）。
    // ★ 受信側は番号（dest_index）ではなくこれで自分の目的地を探す。3 台の destinations.csv の
    //   中身や並び、パイロンの上書きが違うと、番号では**黙って別の場所**に合ってしまうため
    //   （link.cpp の link_sender_dest_local、docs/pons_link.md §5）。
    uint16_t dest_id;

    // --- 時刻（GNSS）---
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

    uint16_t crc16;             // 65 バイトを送っていた頃の CRC 欄。電波に載らないので、いまは 0 のまま
} LinkTelem;

// ★ 65 バイトは仕様書（docs/pons_link.md §4）の数値。2026-10-27 から電波には 32 バイトに詰めて
//   載せる（lora_link/link_wire2.h）ので、項目を足すなら詰め方（核か回転枠か）も一緒に決めること。
//   黙って伸びると詰め方とずれる。ビルドで止める。
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
#define LINK_ST_IMU_ERROR   0x0002   // IMU が応答しない（姿勢が出ない）
#define LINK_ST_NEEDS_APPLY 0x0004   // ロールトリムが未適用（設定し忘れ）
#define LINK_ST_BAT_LOW     0x0008   // 送信機の電池が少ない
// ★ 以下 3 つは「受信側で再計算できるものは載せない」の**例外**。どれも判定に
//   送信機しか持っていないものが要るため、受信側では原理的に作れない:
//   申告したロール／ピッチ目標（SET ROLL / SET PITCH）、加速度計による静止判定、
//   プラットホーム近傍判定。**ビットは「いま送信機が音声で言っている状態」**で、
//   受信側は同じ音声を同じ間隔で鳴らすだけ（link_watch_sender_ground_att）。
//   ★ 飛行中は立たない（地上・静止のブロックの中でしか立てない）。
//     送信機が飛行中に黙るものを受信機が喋ってはいけない。
#define LINK_ST_GROUND_ROLL  0x0010   // 地上チェック: ロールのズレが続いている
#define LINK_ST_GROUND_PITCH 0x0020   // 地上チェック: ピッチのズレが続いている
#define LINK_ST_CALIB_GND    0x0040   // 地上で較正やり直しが必要（ESKF APPLY）
// ★★ **これは受信側では作れないし、画面にも出ない。** 送信機の GNSS が抜けると
//   `stored_*` が凍ったまま送信は続き、`fixflags` も `numsat` も最後の値のまま、
//   `seq` は進むので `NO SIGNAL` にもならない。つまりボートからは
//   **「止まった正常な機体」**に見える。音で知らせるしかないのでビットで運ぶ。
// ★ ビットを足すだけなら LINK_PROTO_VER も LINK_W2_MAGIC も変えなくてよい（status は 32 バイトの核に
//   16 ビットのまま載り、古い送信機は 0 を送るので誤報も出ない）。
#define LINK_ST_GNSS_LOST    0x0080   // 送信機の GNSS モジュールが応答しない
// 送信機の電源経路の異常（充電 IC の故障。power.cpp）。機体が発熱している恐れがあり、
// ボート側からも知らせたい。受信側では作れない（送信機の電圧と USB 判定が要る）。
#define LINK_ST_PWR_FAULT    0x0100
// 送信機が SW を OFF にしたまま USB で動いている（v7.1。power_sw_off_on_usb()）。
// 電池は切り離されていて電圧は読めないので、このあいだ RHAVE_VOLT は立たない。
// 受信機の電池表示を "s5V" にするためのもので、**故障ではない**（WIRELESS 画面の
// Sender 行も赤にしない）。受信側では作れない（送信機の SW の状態は電波に乗らない）。
#define LINK_ST_USB_SW_OFF   0x0200

// ============================================================
//  数値の詰め方
//    ★ 丸めとクリップを**ここ 1 か所**に置く。送信側に
//      constrain(lround(x * 100.0f), -32768L, 32767L) を 14 回書いていたが、
//      1 か所でも桁を間違えるとその項目だけ静かにずれる。
//      解析ツール側で展開するときも、ここを見れば分解能が分かる。
//    丸めは lround と同じ「0 から遠いほうへ」。Arduino のマクロには依存しない
//    （このヘッダは無線層より上から素で include されるため）。
// ============================================================
static inline int16_t link_pack_i16(float v, float scale) {
    const float x = v * scale;
    if (x != x) return 0;          // NaN（推定が発散したとき）。極値を送るより 0 のほうが無害
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}
static inline uint16_t link_pack_u16(float v, float scale, uint16_t maxv) {
    const float x = v * scale;
    if (x != x) return 0;          // NaN
    if (x <= 0.0f) return 0;
    if (x >= (float)maxv) return maxv;
    return (uint16_t)(x + 0.5f);
}
static inline uint8_t link_pack_u8(float v, float scale) {
    const float x = v * scale;
    if (x != x) return 0;          // NaN
    if (x <= 0.0f) return 0;
    if (x >= 255.0f) return 255;
    return (uint8_t)(x + 0.5f);
}

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


#endif // LINK_PROTO_H
