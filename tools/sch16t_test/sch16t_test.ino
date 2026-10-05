// ============================================================
// File    : tools/sch16t_test/sch16t_test.ino
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : Murata SCH16T-K01-10 の単独立ち上げ・性能評価スケッチ。
//           Raspberry Pi Pico 2 (RP2350) 単体で動く。PONS 本体からは独立。
//
// 目的:
//   1. SPI（SafeSPI2 48bit アウトオブフレーム）で会話できることを確認する
//   2. 初期化シーケンス（リセット → 設定 → EOI → 検証）を通す
//   3. **BNO085 との比較材料になる数値を実測する**
//      ＝ 静止時バイアス・ノイズ・|a|/g のスケール誤差・実 ODR
//      BNO085 の実測値（比較対象。tools/imulog の解析より）:
//        静止時 |f|/g = 0.985〜0.992（約 -1% のスケール誤差）
//        加速度ノイズ床 σ = 0.0008 g（2Hz LPF 後）
//        ジャイロは申告精度が 0 のままで ZRO ドリフトあり
//      SCH16T のデータシート typ 値（同じ土俵での期待値）:
//        ジャイロ オフセット ±0.1 °/s (XY) / ±0.01 °/s (Z)
//        ジャイロ ノイズ密度 0.0004 °/s/√Hz (XY)
//        加速度  オフセット ±0.01 m/s²
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/05
// ============================================================
//
// ---- 配線（Pico 2 ←→ IMU_murata ブレイクアウト 8 ピン）----
//   SCH16T          Pico 2        備考
//   MISO            GP16          SPI0 RX
//   CS              GP17          このスケッチが手動で叩く（SPI ライブラリには任せない）
//   SCK             GP18          SPI0 SCK
//   MOSI            GP19          SPI0 TX
//   EXTRESN         GP20          Low でリセット。内部プルアップあり（既定 High）
//   DRY_SYNC        GP21          **入力として使う。絶対に出力にしないこと**
//                                 （SYNC 入力と DRY 出力の兼用ピン。こちらが
//                                   駆動すると DRY が出せなくなる）
//   3V3             3V3(OUT)      V3P3 と VDDIO の両方。実測 3.3V であること
//   GND             GND
//
//   TA9 / TA8 はブレイクアウト上で GND に落ちている想定（データシート:
//   "Connect to ground for default '0' address"）。**未確認なら 'a' コマンドで
//   TA を走査すること。** 4 通りのうち応答する組み合わせが実際の結線。
//
// ---- 出典（2026/10/01 にフルデータシートで全件照合済み）----
// Murata SCH16T データシート Doc.No.11624 Rev.6（全 65 ページ・CONFIDENTIAL）で確認。
//   §6.2 Table 21  フレームのビット定義（D / TA / SA / RW / FT / DATAI / S1:0 / CE / IDS / CRC8）
//   §6.5 Figure 17 CRC8 の C コード（多項式 0x2F・初期値 0xFF・bit 47〜8 が対象）
//   §6.6 Table 27  **実フレームの 16 進値が載っている。ここで検算した**
//   §5.2           起動シーケンスと待ち時間（NVM 32ms / EN_SENSOR 後 215ms / EOI 後 3ms）
//   §5.5 Table 16  LPF ごとの「最小推奨読み出しレート」
//   §7.2 Table 29  レジスタマップ（アドレス・データ幅・D ビット）
//   §7.4 Table 63-66 感度・レンジ・デシメーションの符号
//   §7.4.4 Table 76 CTRL_USER_IF のビットとリセット値
//   §7.4.7 Table 86 COMP_ID: SCH16T-K01 = 0b0000000000100011 = 0x0023
//
// ★ **検算結果**: このスケッチのフレーム組み立てと CRC8 は、
//   データシート Table 27 の 41 フレームすべてとビット単位で一致した。
//   さらに先駆者の Arduino ライブラリ（github.com/HTY2003/smplboards-SCH16T。
//   Murata の C コード例から移植されたもの）の事前計算フレーム 38 件とも全一致。
//   → フレーム層は 2 つの独立した出典で裏が取れている。
//
// ★ **PX4 のドライバ（PX4-Autopilot src/drivers/imu/murata/sch16t/）には不具合がある。**
//   CTRL_USER_IF に DRY_DRV_EN だけを素で書いており、リセット値で 1 の
//   MISO_SR_CTRL / DRY_SR_CTRL を 0 に落としてしまう（データシートに
//   "This option is not supported by Murata" と明記されている側の設定）。
//   このスケッチは read-modify-write にしてある（下の set_dry() 参照）。
//
// ★ **部品は SCH16T-K01 で確定。品種分岐はしない**（2026/10/05）。
//   ただし COMP_ID が 0x0023 かは毎回見る。**他品種を挿すとジャイロの感度だけが
//   16 倍違い（例: K10 は同じ DYN 値で 100 LSB/(°/s)）、加速度は共通**なので、
//   1g 検証が 1.000 に乗ってもジャイロのバイアスとノイズが 1/16 に小さく出る
//   ＝ このスケッチが出すべき唯一の判断材料が、都合のいい方向へ 16 倍間違う。
//   → check_comp_id() で警告し、'G' の実回転で感度そのものを測れるようにした。
//   出典: Murata 由来ライブラリが SCH16T_K10 クラスで Rate の感度だけを
//   上書きしている（加速度側は上書きが無い）。
//
// ★ **検算結果**: このスケッチのフレーム組み立てと CRC8 は、
//   データシート Table 27 の 41 フレームすべてとビット単位で一致した。
//   さらに Murata 由来の Arduino ライブラリ（github.com/HTY2003/smplboards-SCH16T。
//   ヘッダは Copyright Murata Electronics Oy / BSD）の事前計算フレームと、
//   2026/10/05 にホスト上で全件照合した:
//     読み出しフレーム 38 件（48bit 全ビット）… 全一致
//     書き込みフレームの上位 40bit 9 件（REQ_SET_* と REQ_SOFTRESET）… 全一致
//   → フレーム層は 2 つの独立した出典で裏が取れている。
//   そのうちの 7 件は `selftest_frames()` に焼いて**起動時に自動照合**する。
//
// ★ 軸はチップ native のまま出す。PX4 は Y/Z を反転して自分の FRD 系に直している
//   （チップは FLU）。**PONS に取り込むときは軸変換が別途必要。**
//
#include <SPI.h>

// ------------------------------------------------------------
// 型定義
// ------------------------------------------------------------
// ★ struct は**関数より前**に置くこと。Arduino のプリプロセッサが関数プロトタイプを
//   ファイル先頭（最後の #include の直後）へ挿入するため、後ろで定義すると
//   「'Sample' does not name a type」でコンパイルが通らない。
struct Sample {
  int32_t gx, gy, gz;      // LSB
  int32_t ax, ay, az;      // LSB
  int32_t temp;            // LSB（16bit 相当）
  bool    ok;
};

// Welford で平均・分散を出す（静止測定用）。
struct Stat {
  double n = 0, mean = 0, m2 = 0, mn = 1e30, mx = -1e30;
  void add(double x) {
    n++; const double d = x - mean; mean += d / n; m2 += d * (x - mean);
    if (x < mn) mn = x; if (x > mx) mx = x;
  }
  double sd() const { return (n > 1) ? sqrt(m2 / (n - 1)) : 0.0; }
};

// ------------------------------------------------------------
// ピン定義
// ------------------------------------------------------------
static const int PIN_MISO    = 16;
static const int PIN_CS      = 17;
static const int PIN_SCK     = 18;
static const int PIN_MOSI    = 19;
static const int PIN_EXTRESN = 20;
static const int PIN_DRY     = 21;

// SPI クロック。データシート Table 12 より、既定モード（MISO_HI_SPD=0）で
// 0.095〜10.5MHz。25.5MHz は CTRL_USER_IF の MISO_HI_SPD=1 が必要（非標準）。
// ★ **下限 0.095MHz がある。** 遅すぎても動かない。
// 立ち上げは 1MHz から始める（配線が長いとここで落ちる）。'k' で切り替えられる。
static uint32_t g_spi_hz = 1000000;

// SPI デバイスアドレス（TA9:TA8）。GND 固定なら 0。
static uint8_t g_ta = 0;

// ------------------------------------------------------------
// レジスタ定義（PX4 の Murata_SCH16T_registers.hpp と同じ）
// ------------------------------------------------------------
enum : uint8_t {
  REG_RATE_X1 = 0x01, REG_RATE_Y1 = 0x02, REG_RATE_Z1 = 0x03,
  REG_ACC_X1  = 0x04, REG_ACC_Y1  = 0x05, REG_ACC_Z1  = 0x06,
  REG_ACC_X3  = 0x07, REG_ACC_Y3  = 0x08, REG_ACC_Z3  = 0x09,
  REG_RATE_X2 = 0x0A, REG_RATE_Y2 = 0x0B, REG_RATE_Z2 = 0x0C,
  REG_ACC_X2  = 0x0D, REG_ACC_Y2  = 0x0E, REG_ACC_Z2  = 0x0F,
  REG_TEMP    = 0x10,

  // データカウンタ（データシート Table 29。PX4 のヘッダには無い）
  REG_RATE_DCNT = 0x11,   // 12bit
  REG_ACC_DCNT  = 0x12,   // 14bit
  REG_FREQ_CNTR = 0x13,   // 16bit

  REG_STAT_SUM      = 0x14, REG_STAT_SUM_SAT = 0x15, REG_STAT_COM    = 0x16,
  REG_STAT_RATE_COM = 0x17, REG_STAT_RATE_X  = 0x18, REG_STAT_RATE_Y = 0x19,
  REG_STAT_RATE_Z   = 0x1A, REG_STAT_ACC_X   = 0x1B, REG_STAT_ACC_Y  = 0x1C,
  REG_STAT_ACC_Z    = 0x1D,
  // ★ この 2 本は「1 が正常」ではない情報レジスタ。0xFFFF と比べてはいけない。
  REG_STAT_SYNC_ACTIVE = 0x1E,  // 12bit。SYNC 使用中のみ立つ
  REG_STAT_INFO        = 0x1F,  //  9bit。[2]ACC_LPM [1]RATE_LPM [0]SENSOR_LPM

  REG_CTRL_FILT_RATE  = 0x25, REG_CTRL_FILT_ACC12 = 0x26,
  REG_CTRL_FILT_ACC3  = 0x27, REG_CTRL_RATE       = 0x28,
  REG_CTRL_ACC12      = 0x29, REG_CTRL_ACC3       = 0x2A,
  // 飽和フラグのフィルター設定（既定 0。今回は触らない）
  REG_CTRL_RATE_FLAG_1 = 0x2B, REG_CTRL_RATE_FLAG_2 = 0x2C,
  REG_CTRL_ACC_FLAG_1  = 0x2D, REG_CTRL_ACC_FLAG_2  = 0x2E,
  REG_CTRL_USER_IF    = 0x33, REG_CTRL_ST         = 0x34,
  REG_CTRL_MODE       = 0x35, REG_CTRL_RESET      = 0x36,

  REG_SYS_TEST = 0x37,    // ★ 16bit の read/write 自由レジスタ。疎通確認用
  REG_ASIC_ID = 0x3B, REG_COMP_ID = 0x3C,
  REG_SN_ID1  = 0x3D, REG_SN_ID2  = 0x3E, REG_SN_ID3 = 0x3F,
};

// CTRL_MODE のビット（データシート Table 80）
static const uint16_t BIT_EOI            = (1 << 1);  // EOI_CTRL: End of Initialization
static const uint16_t BIT_EN_SENSOR      = (1 << 0);  // EN_SENSOR: RATE/ACC 測定の開始
// CTRL_RESET（Table 81）。この値を書くとソフトリセット。後 2ms は SPI 禁止。
static const uint16_t VAL_SPI_SOFT_RESET = 0b1010;

// CTRL_USER_IF のビット（データシート Table 76）
// ★★ **このレジスタは read-modify-write すること。** リセット値が 0 でないビットがある:
//   [13:12] FTREE_TDEL  = 2b10（1st level status のクリア遅延 2.5ms）
//   [3]     MISO_SR_CTRL= 1b1 （スルーレート制御あり。**0 は Murata 非サポート**）
//   [2]     DRY_SR_CTRL = 1b1 （同上）
//   → リセット値は 0x200C。素で DRY_DRV_EN(0x0020) だけ書くと上の 3 つを潰す。
//   PX4 のドライバはこれを踏んでいる。Murata の C コード例由来のライブラリは
//   ちゃんと読んでから書いている。
static const uint16_t BIT_DRY_DRV_EN  = (1 << 5);  // DRY 出力を有効化（SYNC は使えなくなる）
static const uint16_t BIT_DRY_POL_LOW = (1 << 6);  // 0 = high active（既定）
static const uint16_t CTRL_USER_IF_RESET_VALUE = 0x200C;  // 参考値（実機からは読んで使う）

// フィルター設定値（CTRL_FILT_* の 3bit フィールドに入れる）
enum : uint8_t {
  FILT_68HZ = 0b000, FILT_30HZ = 0b001, FILT_13HZ  = 0b010,
  FILT_280HZ= 0b011, FILT_370HZ= 0b100, FILT_235HZ = 0b101,
  FILT_BYPASS = 0b111,
};

// デシメーション設定値（データシート Table 66。CTRL_RATE / CTRL_ACC12 の DEC_* に入れる）
// 内部一次レート F_PRIM の分周で決まる。デシメーションは *_2 出力にしか効かない。
enum : uint8_t {
  DEC_NONE = 0b000,  // DEC1: 間引きなし F_PRIM/2  = 11.8kHz（周期 85µs）
  DEC_5900 = 0b001,  // DEC2: 1/2   F_PRIM/4  = 5.9kHz  （169µs）
  DEC_2950 = 0b010,  // DEC3: 1/4   F_PRIM/8  = 2.95kHz （338µs）
  DEC_1475 = 0b011,  // DEC4: 1/8   F_PRIM/16 = 1.475kHz（678µs）
  DEC_738  = 0b100,  // DEC5: 1/16  F_PRIM/32 = 737.5Hz （1355µs）
};

// ダイナミックレンジ設定値（データシート Table 63/64/65 の DYN_* フィールド）
// ★ **'000' は CTRL_RATE / CTRL_ACC12 では "Undefined"。書いてはいけない。**
//   感度は 20bit 出力の値。16bit 出力ではすべて 1/16 になる。
//   RATE : '001'(既定)=±327.68°/s 1600 LSB  '010'=同じ  '011'=±163.84 3200  '100'=±81.92 6400
//   ACC12: '001'(既定)=±163.84m/s² 3200 LSB '010'=±81.92 6400 '011'=±40.96 12800 '100'=±20.48 25600
//   ACC3 : '000'(既定)=±260m/s² 1600 LSB    '001'=±163.84 3200 …
//
// ★ **書くのは '010' にした（'001' ではなく）。** 上の表のとおり '001' と '010' は
//   同じレンジだが、Murata 由来ライブラリは 1600 LSB に対して '010' を書き、
//   読み返しの逆変換（convertBitfieldToRateSens）では **'001' を「感度不明」として
//   0 を返す**。つまり '010' が正式な符号。どちらでも動くが、'010' なら
//   「書いた値 → 感度」の対応が Murata の表と 1 対 1 になり、下の
//   dyn_to_lsb_per_dps() による読み返し検証が意味を持つ。
static const uint8_t RATE_DYN_DEFAULT  = 0b010;  // ±327.68 °/s  → 1600 LSB/(°/s)
static const uint8_t ACC12_DYN_DEFAULT = 0b001;  // ±163.84 m/s² → 3200 LSB/(m/s²)
static const uint8_t ACC3_DYN_DEFAULT  = 0b000;  // ±260 m/s²    → 1600 LSB/(m/s²)

// ------------------------------------------------------------
// 感度（SCH16T-K01 固定）
// ------------------------------------------------------------
// ★ **使うのは K01 で確定。他品種には対応しない。**
//   ただし COMP_ID が 0x0023 かどうかは必ず見る。**他品種を挿すと
//   ジャイロの感度だけが 16 倍違い、加速度は同じ**なので、1g 検証が通っても
//   ジャイロのバイアスとノイズが 1/16 に小さく出る（＝都合のいい方向に外れる）。
//   「届いた部品が本当に K01 か」を確かめる 1 行のチェックとして残してある。
//   出典: Murata 由来ライブラリが SCH16T_K10 クラスで Rate の感度だけを
//   100/200/400 に上書きしている（加速度側は上書きが無い）。
static const float LSB_PER_DPS  = 1600.0f;   // DYN='010'（Table 63、K01）
static const float LSB_PER_MPS2 = 3200.0f;   // DYN='001'（Table 64）
static const float LSB_PER_DEGC =  100.0f;   // Table 8
static const uint16_t COMP_ID_K01 = 0x0023;  // Table 86

// COMP_ID が K01 だったか。false のとき 'm' と 'h' に警告を出す。
static bool g_is_k01 = false;

// DYN ビット値 → 感度。Murata 由来ライブラリの convert*ToSens と同じ表（K01 の値）。
// 書いた設定が本当に入っているかを読み返しから復元するために使う。
// ★ **'001' も '010' と同じ値を返す。** データシート Table 63 では両者が同じレンジで、
//   PX4 は '001' を書いて実機で動いている。Murata 由来ライブラリの逆変換だけが
//   '001' を未定義（0）にしているが、それに合わせて 0 を返すと
//   **RATE_DYN_DEFAULT を '001' に戻した瞬間にゼロ除算になる**ので、ここでは埋める。
//   0 を返すのは本当に未定義な符号（'000' と '101'〜'111'）だけ。
static float dyn_to_lsb_per_dps(uint8_t dyn) {
  switch (dyn & 7) {
    case 0b001: return  1600.0f;           // '010' と同じレンジ（Table 63）
    case 0b010: return  1600.0f;
    case 0b011: return  3200.0f;
    case 0b100: return  6400.0f;
    default:    return 0.0f;               // '000' は "Undefined"。書いてはいけない
  }
}
static float dyn_to_lsb_per_mps2(uint8_t dyn) {
  switch (dyn & 7) {
    case 0b001: return  3200.0f;
    case 0b010: return  6400.0f;
    case 0b011: return 12800.0f;
    case 0b100: return 25600.0f;
    default:    return 0.0f;
  }
}

// LPF ごとの最小推奨読み出しレート [Hz]（データシート §5.5 Table 16）。
// ★ これを下回って読むと、フィルターの阻止域のノイズが折り返して帯域内に入る。
//   ノイズを測るときは achieved rate がこの値以上であることを確認すること。
static uint32_t min_read_rate_hz(uint8_t filt) {
  switch (filt) {
    case 0b010: return  150;   // LPF2 13Hz
    case 0b001: return  200;   // LPF1 30Hz
    case 0b000: return  500;   // LPF0 68Hz（既定）
    case 0b101: return 1000;   // LPF5 235Hz
    case 0b011: return 1000;   // LPF3 280Hz
    case 0b100: return 2000;   // LPF4 370Hz
    default:    return 11800;  // LPF7 バイパス
  }
}

// LPF の公称カットオフ [Hz]。ノイズ密度の帯域に使う。
// ★ **帯域に fs/2 を使ってはいけない。** 68Hz LPF を 500Hz で読むと fs/2=250Hz だが
//   実際にノイズが通っている帯域は 68Hz 前後なので、sd/sqrt(250) は真値の
//   sqrt(107/250)=0.65 倍、つまり**ノイズ密度が 35% 小さく出る**。
//   ここも「都合のいい方向に外れる」ので fc を使う。
static float filt_cutoff_hz(uint8_t filt) {
  switch (filt & 7) {
    case 0b010: return  13.0f;
    case 0b001: return  30.0f;
    case 0b000: return  68.0f;
    case 0b011: return 280.0f;
    case 0b100: return 370.0f;
    case 0b101: return 235.0f;
    default:    return 11800.0f / 2.0f;   // バイパス: 一次レートのナイキスト
  }
}

// PONS で使う設定。HPA の運動帯域は 5Hz 以下なので、
// デシメーションを効かせて 738Hz、フィルターは 68Hz から始める。
static uint8_t g_filt_rate  = FILT_68HZ;
static uint8_t g_filt_acc   = FILT_68HZ;
static uint8_t g_dec        = DEC_738;

// ------------------------------------------------------------
// SPI 48bit フレーム層
// ------------------------------------------------------------
// 48bit フレームのビット配置（データシート §6.2 Table 21。Table 27 の実フレームで検算済み）
//
// 要求（MOSI）
//   [47:38] TA      Target Address。**上位 2bit が TA9:TA8（チップ選択）、
//                   下位 8bit がレジスタアドレス**。合わせて 10bit で 1 つの
//                   フィールドなので、このコードでは ta<<46 と addr<<38 に分けて書く
//   [37]    RW      1 = 書込み、0 = 読出し
//   [35]    FT      Frame Type。**次の MISO フレームの幅**（1 = SPI48BF / 0 = SPI32BF）。
//                   48bit で通すので常に 1。アウトオブフレームなので「次」を指定する
//   [27:8]  DATAI   20bit（書込みで使うのは 16bit なので実質 [23:8]）
//   [7:0]   CRC8
//
// 応答（MISO）
//   [46:37] SA      Source Address。要求の TA と同じ内容が返る（個体の確認に使える）
//   [36]    IDS     Internal Data Status。共通原因エラー（S1:S0 の冗長表示）
//   [35]    CE      Command Error。**意味的に不正なフレームのときだけ立つ**（Table 24:
//                   EOI 後の書込み・未定義アドレス・読み出し専用への書込み）。
//                   SPI プロトコル層のエラーは MISO の Hi-Z で示されるので、
//                   配線ミスは「全部 0 か全部 1」として見える
//   [34:33] S1:S0   00=正常 01=エラー 10=飽和 11=初期化中
//                   ★ **レジスタ書込みの応答では S は常に 00**（Table 22 の注記）
//   [27:8]  SENSOR/INFO  **1 つ前に要求したレジスタの値**（アウトオブフレーム）
//   [7:0]   CRC8

static const uint64_t MASK48_IDS    = (uint64_t)1 << 36;
static const uint64_t MASK48_CE     = (uint64_t)1 << 35;
static const uint64_t MASK48_S1     = (uint64_t)1 << 34;
static const uint64_t MASK48_S0     = (uint64_t)1 << 33;

// CRC8: 多項式 0x2F・初期値 0xFF。上位 40bit（[47:8]）が対象。
static uint8_t crc8_frame(uint64_t frame) {
  uint64_t data = frame & 0xFFFFFFFFFF00ULL;
  uint8_t crc = 0xFF;
  for (int i = 47; i >= 0; i--) {
    uint8_t bit = (uint8_t)((data >> i) & 0x01);
    crc = (crc & 0x80) ? (uint8_t)(((uint8_t)(crc << 1)) ^ 0x2F) ^ bit
                       : (uint8_t)((uint8_t)(crc << 1) | bit);
  }
  return crc;
}

// 20bit 2 の補数 → int32（符号拡張）。フレームの [27:8] を取り出す。
static inline int32_t frame_data_i20(uint64_t f) {
  return (int32_t)(((uint32_t)(f << 4)) & 0xFFFFF000UL) >> 12;
}
static inline uint16_t frame_data_u16(uint64_t f) {
  return (uint16_t)((f >> 8) & 0xFFFFUL);
}

// 統計カウンタ
static uint32_t g_crc_err = 0, g_ids_err = 0, g_ce_err = 0, g_sat_err = 0, g_init_run = 0;
static uint32_t g_frames  = 0;

// 1 フレーム送受信する。戻り値は受信した 48bit。
// ★ **6 バイトを 1 回の transfer() で送る。`SPI.transfer16()` を 3 回呼んではいけない。**
//   arduino-pico 4.5.1 の `transfer16()` は `spi_write16_read16_blocking(..., 1)` を
//   呼ぶので 1 ワードごとに戻ってきて、**ワード間で SCK が止まる**。
//   `transfer(tx, rx, 6)` なら `spi_write_read_blocking()` が 6 バイトを
//   TX FIFO（8 段）へ先に積むので SCK が 48bit 連続する。
//   CS は Low のままなのでどちらでも通るはずだが、連続の方を既定にする。
static uint64_t xfer48(uint64_t frame) {
  uint8_t tx[6], rx[6];
  for (int i = 0; i < 6; i++) tx[i] = (uint8_t)(frame >> (8 * (5 - i)));   // big endian

  SPI.beginTransaction(SPISettings(g_spi_hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);                 // CS 立ち下がり → データ有効まで 40ns。余裕を取る
  SPI.transfer(tx, rx, sizeof(tx));
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);                 // CS の High 期間を必ず作る

  g_frames++;
  uint64_t r = 0;
  for (int i = 0; i < 6; i++) r = (r << 8) | rx[i];
  return r;
}

// 要求フレームの組み立て。**送信とは分けてある**（selftest_frames() が検算に使う）。
static uint64_t build_read_frame(uint8_t addr, uint8_t ta) {
  uint64_t f = 0;
  f |= (uint64_t)(ta & 0x03) << 46;
  f |= (uint64_t)addr << 38;
  f |= (uint64_t)1 << 35;
  return f | (uint64_t)crc8_frame(f);
}
static uint64_t build_write_frame(uint8_t addr, uint16_t value, uint8_t ta) {
  uint64_t f = 0;
  f |= (uint64_t)(ta & 0x03) << 46;
  f |= (uint64_t)addr << 38;
  f |= (uint64_t)1 << 37;                // WRITE
  f |= (uint64_t)1 << 35;
  f |= (uint64_t)value << 8;
  return f | (uint64_t)crc8_frame(f);
}

// 読み出し要求を 1 フレーム投げる。**戻ってくるのは 1 つ前の要求の結果**。
static uint64_t req_read(uint8_t addr) {
  return xfer48(build_read_frame(addr, g_ta));
}

// 書き込み。応答は見ない。
static void reg_write(uint8_t addr, uint16_t value) {
  (void)xfer48(build_write_frame(addr, value, g_ta));
}

// 単発読み出し。アウトオブフレームなので 2 フレーム使う（同じアドレスを 2 回投げる）。
static uint64_t reg_read(uint8_t addr) {
  req_read(addr);
  return req_read(addr);
}

// 応答フレームの健全性を見る。戻り値 true = 使ってよいデータ。
static bool frame_ok(uint64_t f, bool count = true) {
  bool ok = true;
  if ((uint8_t)(f & 0xFF) != crc8_frame(f)) { if (count) g_crc_err++; ok = false; }
  if (f & MASK48_IDS) { if (count) g_ids_err++; ok = false; }
  if (f & MASK48_CE)  { if (count) g_ce_err++;  ok = false; }
  const bool s1 = (f & MASK48_S1) != 0, s0 = (f & MASK48_S0) != 0;
  if (s1 && s0)       { if (count) g_init_run++; ok = false; }   // 11 = 初期化中
  else if (s1)        { if (count) g_sat_err++;  }               // 10 = 飽和。データ自体は使える
  else if (s0)        { ok = false; }                            // 01 = エラー
  return ok;
}

static void print_frame(const char* tag, uint64_t f) {
  Serial.printf("%-10s 0x%04X%04X%04X  IDS=%d CE=%d S=%d%d  data=0x%05lX (%ld)  crc %s\n",
    tag,
    (unsigned)((f >> 32) & 0xFFFF), (unsigned)((f >> 16) & 0xFFFF), (unsigned)(f & 0xFFFF),
    (f & MASK48_IDS) ? 1 : 0, (f & MASK48_CE) ? 1 : 0,
    (f & MASK48_S1) ? 1 : 0, (f & MASK48_S0) ? 1 : 0,
    (unsigned long)((f >> 8) & 0xFFFFF), (long)frame_data_i20(f),
    ((uint8_t)(f & 0xFF) == crc8_frame(f)) ? "OK" : "NG");
}

// ------------------------------------------------------------
// フレーム組み立ての自己テスト（起動時に走る。SPI は叩かない）
// ------------------------------------------------------------
// ★ ここの期待値は **Murata Electronics Oy の BSD コード（smplboards-SCH16T の
//   SCH16T.h の REQ_* 定数）に載っている事前計算フレーム**をそのまま写したもの。
//   ホスト上では 2026/10/05 に 47 件すべて一致している。焼いてあるのは代表 10 件。
// ★ **これが守るのは「将来の編集による退行」だけ。** 元の読み方が間違っていた場合は
//   検出できない（同じ出典から持ってきた値なので）。それでも、CRC や
//   ビット位置をうっかり触ったときに**実機を触る前に止まる**のが大きい。
static bool selftest_frames() {
  struct { uint64_t want; uint8_t addr; uint16_t data; bool write; const char* name; } v[] = {
    { 0x0048000000ACULL, 0x01, 0,      false, "READ RATE_X1"      },
    { 0x03C8000000B5ULL, 0x0F, 0,      false, "READ ACC_Z2"       },
    { 0x0408000000B1ULL, 0x10, 0,      false, "READ TEMP"         },
    { 0x05080000001CULL, 0x14, 0,      false, "READ STAT_SUM"     },
    { 0x0CC80000007CULL, 0x33, 0,      false, "READ CTRL_USER_IF" },
    { 0x0F0800000092ULL, 0x3C, 0,      false, "READ COMP_ID"      },
    { 0x0DA800000AC3ULL, 0x36, 0b1010, true,  "WRITE SOFTRESET"   },
  };
  // 書き込みフレームは Murata 側が「上位 40bit のひな形」だけを持っているので、
  // データ 0 のときの frame>>8 で照合する。
  struct { uint64_t want40; uint8_t addr; const char* name; } w[] = {
    { 0x0968000000ULL, 0x25, "WRITE CTRL_FILT_RATE prefix" },
    { 0x0A28000000ULL, 0x28, "WRITE CTRL_RATE prefix"      },
    { 0x0CE8000000ULL, 0x33, "WRITE CTRL_USER_IF prefix"   },
  };

  int ng = 0;
  for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
    const uint64_t got = v[i].write ? build_write_frame(v[i].addr, v[i].data, 0)
                                    : build_read_frame(v[i].addr, 0);
    if (got != v[i].want) {
      ng++;
      Serial.printf("  NG %-26s want 0x%04X%04X%04X got 0x%04X%04X%04X\n", v[i].name,
        (unsigned)((v[i].want >> 32) & 0xFFFF), (unsigned)((v[i].want >> 16) & 0xFFFF),
        (unsigned)(v[i].want & 0xFFFF),
        (unsigned)((got >> 32) & 0xFFFF), (unsigned)((got >> 16) & 0xFFFF),
        (unsigned)(got & 0xFFFF));
    }
  }
  for (unsigned i = 0; i < sizeof(w) / sizeof(w[0]); i++) {
    const uint64_t got = (build_write_frame(w[i].addr, 0, 0) >> 8) & 0xFFFFFFFFFFULL;
    if (got != w[i].want40) {
      ng++;
      Serial.printf("  NG %-26s want 0x%02X%04X%04X got 0x%02X%04X%04X\n", w[i].name,
        (unsigned)((w[i].want40 >> 32) & 0xFF), (unsigned)((w[i].want40 >> 16) & 0xFFFF),
        (unsigned)(w[i].want40 & 0xFFFF),
        (unsigned)((got >> 32) & 0xFF), (unsigned)((got >> 16) & 0xFFFF),
        (unsigned)(got & 0xFFFF));
    }
  }

  const unsigned total = sizeof(v) / sizeof(v[0]) + sizeof(w) / sizeof(w[0]);
  if (ng == 0)
    Serial.printf("[self] フレーム組み立て %u 件 OK（Murata の事前計算値と一致）\n", total);
  else
    Serial.printf("[self] ★★ フレーム組み立てが %d / %u 件ずれている。"
                  "CRC かビット位置を壊した。実機を触る前に直すこと\n", ng, total);
  return ng == 0;
}

// ------------------------------------------------------------
// 届いた部品が K01 かの確認
// ------------------------------------------------------------
// ★ 使うのは K01 固定なので品種分岐はしない。**ここは「挿してある部品が本当に
//   K01 か」を確かめるだけ。** 他品種だとジャイロの感度だけが 16 倍違い、
//   加速度は同じなので 1g 検証では気づけない。
// 戻り値 true = COMP_ID が K01。
static bool check_comp_id() {
  const uint64_t fa = reg_read(REG_ASIC_ID);
  const uint64_t fc = reg_read(REG_COMP_ID);
  const bool frames_ok = frame_ok(fa, false) && frame_ok(fc, false);
  const uint16_t asic = frame_data_u16(fa);
  const uint16_t comp = frame_data_u16(fc);

  if (!frames_ok) {
    g_is_k01 = false;
    Serial.printf("[id] ★ ID の応答が壊れている（ASIC=0x%04X COMP=0x%04X）。"
                  "'z' と 't' で配線を確認すること\n", asic, comp);
    return false;
  }

  g_is_k01 = (comp == COMP_ID_K01);
  if (g_is_k01) {
    Serial.printf("[id] COMP_ID 0x%04X = SCH16T-K01 → ジャイロ %.0f LSB/(°/s)"
                  " / 加速度 %.0f LSB/(m/s²)\n", comp, LSB_PER_DPS, LSB_PER_MPS2);
  } else {
    Serial.printf("[id] ★★ COMP_ID 0x%04X は K01 の 0x%04X ではない（ASIC=0x%04X）。\n"
                  "     このスケッチは K01 の %.0f LSB/(°/s) 固定なので、"
                  "ジャイロの数値は信用できない。\n"
                  "     **1g 検証では判別できない**（加速度の感度は品種共通）。"
                  "'G' で実回転から直接測ること。\n",
                  comp, COMP_ID_K01, asic, LSB_PER_DPS);
  }
  return g_is_k01;
}

// 読み返した DYN ビットから感度を復元して、スケーリングに使っている定数と照合する。
// ★ CTRL_RATE 全体の want/got 比較とは別物。あれは「書いた値が入ったか」しか見ない。
//   こちらは「LSB_PER_DPS / LSB_PER_MPS2 が、チップに入っているレンジ設定と
//   本当に対応しているか」を見る。DYN1 と DYN2 が食い違っていてもここで出る。
static bool verify_sensitivity_readback() {
  const uint16_t cr = frame_data_u16(reg_read(REG_CTRL_RATE));
  const uint16_t ca = frame_data_u16(reg_read(REG_CTRL_ACC12));
  const uint8_t rd1 = (uint8_t)((cr >> 12) & 7), rd2 = (uint8_t)((cr >> 9) & 7);
  const uint8_t ad1 = (uint8_t)((ca >> 12) & 7), ad2 = (uint8_t)((ca >> 9) & 7);
  const float r1 = dyn_to_lsb_per_dps(rd1),  r2 = dyn_to_lsb_per_dps(rd2);
  const float a1 = dyn_to_lsb_per_mps2(ad1), a2 = dyn_to_lsb_per_mps2(ad2);
  const bool ok = (r1 == LSB_PER_DPS) && (r2 == LSB_PER_DPS)
               && (a1 == LSB_PER_MPS2) && (a2 == LSB_PER_MPS2);

  Serial.printf("[init] DYN 読み返し  RATE1=%u(%.0f) RATE2=%u(%.0f) LSB/(°/s)"
                " | ACC1=%u(%.0f) ACC2=%u(%.0f) LSB/(m/s²)%s\n",
                rd1, r1, rd2, r2, ad1, a1, ad2, a2, ok ? "" : "  <<< 定数と不一致");
  if (!g_is_k01)
    Serial.println("[init] ★ COMP_ID が K01 ではない。'G' で感度を実測すること");
  return ok;
}

// ------------------------------------------------------------
// 初期化
// ------------------------------------------------------------
static bool g_inited = false;

// CTRL_RATE（Table 67）: [2:0] DEC_X2 [5:3] DEC_Y2 [8:6] DEC_Z2 [11:9] DYN_XYZ2 [14:12] DYN_XYZ1
static uint16_t build_ctrl_rate() {
  return (uint16_t)((g_dec & 7) | ((g_dec & 7) << 3) | ((g_dec & 7) << 6)
                    | ((RATE_DYN_DEFAULT & 7) << 9) | ((RATE_DYN_DEFAULT & 7) << 12));
}
// CTRL_ACC12（Table 68）: 同じ並び
static uint16_t build_ctrl_acc12() {
  return (uint16_t)((g_dec & 7) | ((g_dec & 7) << 3) | ((g_dec & 7) << 6)
                    | ((ACC12_DYN_DEFAULT & 7) << 9) | ((ACC12_DYN_DEFAULT & 7) << 12));
}
// CTRL_FILT_*（Table 67 前後）: 3 軸それぞれ 3bit。データシート Table 27 の
// "Select LPFn filter" フレームと一致することを PC 上で検算済み。
static uint16_t build_filt(uint8_t sel) {
  return (uint16_t)((sel & 7) | ((sel & 7) << 3) | ((sel & 7) << 6));
}

// ★★ CTRL_USER_IF は **読んでから必要なビットだけ立てて書き戻す**。
//   素で書くとリセット値で 1 のビット（FTREE_TDEL / MISO_SR_CTRL / DRY_SR_CTRL）を潰す。
//   手順は Murata の C コード例由来のライブラリ（smplboards-SCH16T の setDRY）と同じ。
//   戻り値: 書き戻しが読み返しで一致したか。
static bool set_dry(bool enable, bool active_low = false) {
  const uint16_t before = frame_data_u16(reg_read(REG_CTRL_USER_IF));
  uint16_t v = before;
  if (enable) v |=  BIT_DRY_DRV_EN; else v &= (uint16_t)~BIT_DRY_DRV_EN;
  if (active_low) v |= BIT_DRY_POL_LOW; else v &= (uint16_t)~BIT_DRY_POL_LOW;
  reg_write(REG_CTRL_USER_IF, v);
  const uint16_t after = frame_data_u16(reg_read(REG_CTRL_USER_IF));
  Serial.printf("[init] CTRL_USER_IF 0x%04X -> 0x%04X (読み返し 0x%04X)%s\n",
                before, v, after, (after == v) ? "" : "  <<< MISMATCH");
  if (before == 0x0000 || before == 0xFFFF)
    Serial.println("       ★ 読めた値が 0x0000/0xFFFF。SPI が成立していない疑い");
  return after == v;
}

// ハードリセット（EXTRESN を Low）。
static void hard_reset() {
  digitalWrite(PIN_EXTRESN, LOW);
  delay(2);
  digitalWrite(PIN_EXTRESN, HIGH);
}

// 初期化シーケンス。データシート §5.2 と、Murata の C コード例由来のライブラリに合わせた形。
//   リセット → 250ms → 設定書込み → 250ms → ステータス読み（1 回）→ EOI|EN_SENSOR
//   → 5ms → ステータス読み（2 回）→ 検証
// ★ 250ms の待ちを削らないこと。EOI の前に設定が確定していないと、
//   S1:S0 が 11（初期化中）のまま抜けてこない。
static bool sch_init_once(bool use_hard_reset) {
  g_inited = false;

  // ---- リセット ----
  // ★ Murata の C コード例は **両方**（EXTRESN → 10ms → SPI ソフトリセット）を通す。
  //   ソフトリセットの後は 2ms は SPI 禁止（Table 81）、その後 NVM 読み出しに 32ms。
  //   ここは余裕を見て 50ms 待つ。
  if (use_hard_reset) {
    Serial.println("[init] EXTRESN reset + SPI soft reset");
    hard_reset();
    delay(10);
  } else {
    Serial.println("[init] SPI soft reset");
  }
  reg_write(REG_CTRL_RESET, VAL_SPI_SOFT_RESET);
  delay(50);                    // 2ms（SPI 禁止）＋ 32ms（NVM 読み）に余裕

  // ---- 挿してある部品が K01 かを確認（感度の前提なので設定より先）----
  bool ok = true;
  check_comp_id();              // 違っても止めない。'm' の出力に警告が出る

  // ---- 設定を書く（EN_SENSOR の前に済ませる）----
  Serial.println("[init] write config");
  reg_write(REG_CTRL_FILT_RATE,  build_filt(g_filt_rate));
  reg_write(REG_CTRL_FILT_ACC12, build_filt(g_filt_acc));
  reg_write(REG_CTRL_FILT_ACC3,  build_filt(g_filt_acc));
  reg_write(REG_CTRL_RATE,       build_ctrl_rate());
  reg_write(REG_CTRL_ACC12,      build_ctrl_acc12());
  reg_write(REG_CTRL_ACC3,       ACC3_DYN_DEFAULT);
  // ★ read-modify-write。素で書かないこと。
  //   **戻り値を捨てないこと** — 書き戻しが失敗したまま SUCCESS と出ると、
  //   'd' が「パルスが取れていない。配線を確認」と言って配線を疑わせる。
  if (!set_dry(true)) ok = false;

  // ---- EN_SENSOR → 215ms 待つ ----
  // ★ **この待ちの間、ステータスフラグの値は未定義**（データシート §5.2）。
  //   215ms が規定値。250ms にして余裕を持たせる。
  reg_write(REG_CTRL_MODE,       BIT_EN_SENSOR);
  delay(250);

  // ステータスを 1 周読んでラッチをクリアする
  static const uint8_t stat_regs[] = {
    REG_STAT_SUM, REG_STAT_SUM_SAT, REG_STAT_COM, REG_STAT_RATE_COM,
    REG_STAT_RATE_X, REG_STAT_RATE_Y, REG_STAT_RATE_Z,
    REG_STAT_ACC_X, REG_STAT_ACC_Y, REG_STAT_ACC_Z,
  };
  // ★ 要素数は sizeof(配列)/sizeof(要素) で取る。uint8_t なので sizeof だけでも
  //   たまたま合うが、型を変えた瞬間に静かに回数が狂う。
  const unsigned n_stat = sizeof(stat_regs) / sizeof(stat_regs[0]);
  for (unsigned i = 0; i < n_stat; i++) req_read(stat_regs[i]);

  // ★ EOI を書くと**ソフトリセットと SYS_TEST 以外の R/W レジスタが全部ロックされる**
  //   （Table 80）。以降の設定変更はリセットしない限り効かない。
  //   EOI 後にレジスタへ書くと CE（コマンドエラー）が立つ。
  Serial.println("[init] EOI | EN_SENSOR");
  reg_write(REG_CTRL_MODE, (uint16_t)(BIT_EOI | BIT_EN_SENSOR));
  delay(5);                     // 規定 3ms

  // ★ ステータスは 2 周読む。1 周目は初期化中のラッチが残っている。
  for (int pass = 0; pass < 2; pass++)
    for (unsigned i = 0; i < n_stat; i++) req_read(stat_regs[i]);

  // 設定の読み返し検証。**書いた 6 本すべてを見る**（PX4 も 6 本検証している）。
  // ★ ただしこの検証が言えるのは「書いた値が入った」だけ。
  //   「他のビットを潰していない」は言えない（0 を書いて 0 が読めても同じ）。
  //   潰す心配があるのはリセット値が非ゼロのレジスタだけで、該当するのは
  //   CTRL_USER_IF（set_dry が read-modify-write）と CTRL_ST（触っていない）。
  //   ここで書いている 6 本は幅が 9/15/3bit で、残りは reserved なので安全。
  struct { uint8_t addr; uint16_t want; const char* name; } chk[] = {
    { REG_CTRL_FILT_RATE,  build_filt(g_filt_rate), "CTRL_FILT_RATE"  },
    { REG_CTRL_FILT_ACC12, build_filt(g_filt_acc),  "CTRL_FILT_ACC12" },
    { REG_CTRL_FILT_ACC3,  build_filt(g_filt_acc),  "CTRL_FILT_ACC3"  },
    { REG_CTRL_RATE,       build_ctrl_rate(),       "CTRL_RATE"       },
    { REG_CTRL_ACC12,      build_ctrl_acc12(),      "CTRL_ACC12"      },
    { REG_CTRL_ACC3,       ACC3_DYN_DEFAULT,        "CTRL_ACC3"       },
  };
  for (unsigned i = 0; i < sizeof(chk) / sizeof(chk[0]); i++) {
    const uint16_t got = frame_data_u16(reg_read(chk[i].addr));
    const bool same = (got == chk[i].want);
    if (!same) ok = false;
    Serial.printf("[init] %-16s want 0x%04X got 0x%04X %s\n",
                  chk[i].name, chk[i].want, got, same ? "" : "  <<< MISMATCH");
  }

  // ★ **感度は「書いた値」ではなく「読み返した DYN ビット」から復元して照合する。**
  //   スケーリングに使っている g_lsb_per_* が、いま本当にチップに入っている
  //   レンジ設定と一致しているかを見る唯一の経路。
  if (!verify_sensitivity_readback()) ok = false;

  // センサーステータスの総合判定。正常なら全ビット 1（＝異常なし）。
  const uint16_t sum = frame_data_u16(reg_read(REG_STAT_SUM));
  Serial.printf("[init] STAT_SUM = 0x%04X %s\n", sum,
                (sum == 0xFFFF) ? "(all OK)" : "  <<< 0 のビットが異常箇所");
  if (sum != 0xFFFF) ok = false;

  g_inited = ok;
  Serial.printf("[init] %s\n", ok ? "SUCCESS" : "FAILED");
  return ok;
}

// 初期化は **2 回まで試す**（Murata の begin() も失敗したらリセットして 1 回やり直す）。
// 2 回目は必ず EXTRESN のハードリセットから入る。
static bool sch_init(bool use_hard_reset) {
  for (int attempt = 0; attempt < 2; attempt++) {
    if (attempt > 0)
      Serial.println("[init] 1 回目が失敗したのでハードリセットから再試行する");
    if (sch_init_once(use_hard_reset || attempt > 0)) return true;
  }
  Serial.println("[init] ★ 2 回試して失敗。's' で個別ステータス、'z'/'t' で配線を見ること");
  return false;
}

// ------------------------------------------------------------
// ID 読み出し
// ------------------------------------------------------------
// ★ **ID は CRC を見てから信じること。** README の手順は z → t → i なので実用上は
//   守られるが、'i' を単独で打ったときに化けた値を鵜呑みにしないため。
static uint16_t read_u16_checked(uint8_t addr, bool* all_ok) {
  const uint64_t f = reg_read(addr);
  if (!frame_ok(f, false)) *all_ok = false;       // 統計は汚さない（count=false）
  return frame_data_u16(f);
}

static void cmd_id() {
  bool frames_ok = true;
  const uint16_t asic = read_u16_checked(REG_ASIC_ID, &frames_ok);
  const uint16_t comp = read_u16_checked(REG_COMP_ID, &frames_ok);
  const uint16_t sn1  = read_u16_checked(REG_SN_ID1,  &frames_ok);
  const uint16_t sn2  = read_u16_checked(REG_SN_ID2,  &frames_ok);
  const uint16_t sn3  = read_u16_checked(REG_SN_ID3,  &frames_ok);

  if (!frames_ok)
    Serial.println("★★ 応答フレームが壊れている（CRC/IDS/CE/S のいずれか）。"
                   "以下の値は信用できない。'z' と 't' で配線を見ること");

  // ASIC_ID は 12bit、COMP_ID は 16bit。**%04X で出す** — 0x0023 と比べる話をしながら
  // 0x23 と表示されると読み手が混乱する。
  Serial.printf("ASIC_ID = 0x%04X   COMP_ID = 0x%04X\n", asic, comp);
  Serial.printf("Serial  = %05u%01X%04X\n", sn2, sn1 & 0x000F, sn3);
  // ★ 部品の識別は COMP_ID。データシート Table 86 に
  //   「SCH16T-K01 = 0b0000000000100011」と明記されている（= 0x0023）。
  //   ASIC_ID はシリコンのリビジョン（Table 85: [11:8]型 [7:4]major [3:0]minor）で、
  //   部品名の判定には使えない。PX4 が 0x21 を見ているのは実機のリビジョン値。
  //   **K01 かどうかの確認はここに集約した**（check_comp_id）。
  //   以前ここにあった「'm' の 1g 検証で確かめること」という案内は**誤り**だった:
  //   加速度の感度は品種共通なので、1g 検証ではジャイロの 16 倍違いを
  //   原理的に検出できない。代わりに 'G' の実回転で測る。
  check_comp_id();
  Serial.printf("   ASIC rev: type=%u major=%u minor=%u（部品の判定には使わない）\n",
                (asic >> 8) & 0xF, (asic >> 4) & 0xF, asic & 0xF);

  Serial.printf("CTRL_FILT_RATE =0x%04X  CTRL_FILT_ACC12=0x%04X\n",
                frame_data_u16(reg_read(REG_CTRL_FILT_RATE)),
                frame_data_u16(reg_read(REG_CTRL_FILT_ACC12)));
  Serial.printf("CTRL_RATE      =0x%04X  CTRL_ACC12     =0x%04X  CTRL_ACC3=0x%04X\n",
                frame_data_u16(reg_read(REG_CTRL_RATE)),
                frame_data_u16(reg_read(REG_CTRL_ACC12)),
                frame_data_u16(reg_read(REG_CTRL_ACC3)));
  Serial.printf("CTRL_MODE      =0x%04X  CTRL_USER_IF   =0x%04X\n",
                frame_data_u16(reg_read(REG_CTRL_MODE)),
                frame_data_u16(reg_read(REG_CTRL_USER_IF)));
}

// ------------------------------------------------------------
// TA 走査（TA9:TA8 の結線が未確認なとき用）
// ------------------------------------------------------------
static void cmd_scan_ta() {
  const uint8_t save = g_ta;
  Serial.println("TA を 0..3 で走査する（COMP_ID が読めた組み合わせが正解）");
  Serial.println("  応答フレームの SA[46:37] は要求の TA[47:38] と一致するはず（Murata の例より）");
  for (uint8_t ta = 0; ta < 4; ta++) {
    g_ta = ta;
    const uint64_t f = reg_read(REG_COMP_ID);
    const uint16_t id = frame_data_u16(f);
    const bool crc_ok = ((uint8_t)(f & 0xFF) == crc8_frame(f));
    // SA は TA と同じ内容が返る。合っていなければ別の個体が答えている／配線が違う。
    const uint16_t sa = (uint16_t)((f >> 37) & 0x3FF);
    const uint16_t ta_field = (uint16_t)(((uint32_t)(ta & 3) << 8) | REG_COMP_ID);
    Serial.printf("  TA=%u (TA9=%u TA8=%u) : COMP_ID=0x%04X crc=%s SA=0x%03X(期待 0x%03X)%s\n",
                  ta, (ta >> 1) & 1, ta & 1, id, crc_ok ? "OK" : "NG", sa, ta_field,
                  (crc_ok && sa == ta_field) ? "  <<< 応答あり" : "");
  }
  g_ta = save;
  Serial.printf("TA を %u に戻した\n", g_ta);
}

// ------------------------------------------------------------
// SYS_TEST による疎通確認（データシート Table 84 の推奨手順）
// ------------------------------------------------------------
// SYS_TEST は 16bit の自由な read/write レジスタ。**EOI でロックされない**ので
// 初期化後でも使える。複数個体を同じバスに載せたとき CS が効いているかの確認にも使う。
// データシート指定の手順: 書く → 読む → ダミー読みで前フレームの応答を受け取る。
static void cmd_systest() {
  static const uint16_t pats[] = { 0x0000, 0xFFFF, 0xAAAA, 0x5555, 0x1234 };
  int ng = 0;
  Serial.println("SYS_TEST(0x37) に書いて読み返す（EOI でロックされないレジスタ）");
  for (unsigned i = 0; i < sizeof(pats) / sizeof(pats[0]); i++) {
    reg_write(REG_SYS_TEST, pats[i]);
    const uint16_t got = frame_data_u16(reg_read(REG_SYS_TEST));
    const bool same = (got == pats[i]);
    if (!same) ng++;
    Serial.printf("  書 0x%04X -> 読 0x%04X %s\n", pats[i], got, same ? "OK" : "<<< NG");
  }
  reg_write(REG_SYS_TEST, 0x0000);
  if (ng == 0) Serial.println("-> 疎通 OK。フレーム層・CS・CRC がすべて成立している");
  else         Serial.println("-> ★ 不一致あり。配線・SPI クロック・CS・TA を疑う");
}

// ------------------------------------------------------------
// センサー読み出し
// ------------------------------------------------------------
// デシメーション出力（*_2 = decimated）を 1 サンプル読む。DRY はこちらに紐づく。
// ★ アウトオブフレームなので、先頭に捨てフレームが 1 つ必要。
static Sample read_sample_dec() {
  Sample s = {};
  req_read(REG_RATE_X2);                        // 捨てフレーム
  const uint64_t fgx = req_read(REG_RATE_Y2);
  const uint64_t fgy = req_read(REG_RATE_Z2);
  const uint64_t fgz = req_read(REG_ACC_X2);
  const uint64_t fax = req_read(REG_ACC_Y2);
  const uint64_t fay = req_read(REG_ACC_Z2);
  const uint64_t faz = req_read(REG_TEMP);
  const uint64_t ft  = req_read(REG_TEMP);

  // ★ **`&&` で繋いではいけない。** frame_ok() は g_crc_err 等を増やす副作用がある
  //   関数なので、短絡すると 1 本目で落ちた時点で残りが数えられず、
  //   'm' の「うち CRC n 件」と 's' のフレーム統計が実際より少なく出る。
  //   `&=` は短絡しないので 7 本すべてを必ず通る。
  s.ok = true;
  s.ok &= frame_ok(fgx); s.ok &= frame_ok(fgy); s.ok &= frame_ok(fgz);
  s.ok &= frame_ok(fax); s.ok &= frame_ok(fay); s.ok &= frame_ok(faz);
  s.ok &= frame_ok(ft);
  s.gx = frame_data_i20(fgx); s.gy = frame_data_i20(fgy); s.gz = frame_data_i20(fgz);
  s.ax = frame_data_i20(fax); s.ay = frame_data_i20(fay); s.az = frame_data_i20(faz);
  s.temp = frame_data_i20(ft) >> 4;             // 温度は 16bit 幅。下位 4bit は未使用
  return s;
}

// 補間出力（*_1 = interpolated）を読む。*_2 との比較用。
// ★ *_1 と *_2 はどちらも 20bit。違うのは「補間 vs デシメーション」と、
//   レンジ設定が別フィールド（DYN_*_XYZ1 / XYZ2）であること。
//   *_1 は内部で補間されており実効 ODR が非常に高い（データシート §5.5 では 377kHz 相当）
//   ので、こちらは「好きなレートで読む」使い方。ジッタは常に 2.6µs 未満。
//   データシートが「水平検出・測位用の IMU」に推奨しているのはこちら＋既定設定。
static Sample read_sample_raw() {
  Sample s = {};
  req_read(REG_RATE_X1);
  const uint64_t fgx = req_read(REG_RATE_Y1);
  const uint64_t fgy = req_read(REG_RATE_Z1);
  const uint64_t fgz = req_read(REG_ACC_X1);
  const uint64_t fax = req_read(REG_ACC_Y1);
  const uint64_t fay = req_read(REG_ACC_Z1);
  const uint64_t faz = req_read(REG_TEMP);
  const uint64_t ft  = req_read(REG_TEMP);
  // ★ **`&&` で繋いではいけない。** frame_ok() は g_crc_err 等を増やす副作用がある
  //   関数なので、短絡すると 1 本目で落ちた時点で残りが数えられず、
  //   'm' の「うち CRC n 件」と 's' のフレーム統計が実際より少なく出る。
  //   `&=` は短絡しないので 7 本すべてを必ず通る。
  s.ok = true;
  s.ok &= frame_ok(fgx); s.ok &= frame_ok(fgy); s.ok &= frame_ok(fgz);
  s.ok &= frame_ok(fax); s.ok &= frame_ok(fay); s.ok &= frame_ok(faz);
  s.ok &= frame_ok(ft);
  s.gx = frame_data_i20(fgx); s.gy = frame_data_i20(fgy); s.gz = frame_data_i20(fgz);
  s.ax = frame_data_i20(fax); s.ay = frame_data_i20(fay); s.az = frame_data_i20(faz);
  s.temp = frame_data_i20(ft) >> 4;
  return s;
}

// ------------------------------------------------------------
// ステータスダンプ
// ------------------------------------------------------------
static void cmd_status() {
  struct { uint8_t addr; const char* name; } regs[] = {
    { REG_STAT_SUM,      "STAT_SUM"      }, { REG_STAT_SUM_SAT, "STAT_SUM_SAT" },
    { REG_STAT_COM,      "STAT_COM"      }, { REG_STAT_RATE_COM,"STAT_RATE_COM"},
    { REG_STAT_RATE_X,   "STAT_RATE_X"   }, { REG_STAT_RATE_Y,  "STAT_RATE_Y"  },
    { REG_STAT_RATE_Z,   "STAT_RATE_Z"   }, { REG_STAT_ACC_X,   "STAT_ACC_X"   },
    { REG_STAT_ACC_Y,    "STAT_ACC_Y"    }, { REG_STAT_ACC_Z,   "STAT_ACC_Z"   },
  };
  Serial.println("ステータス（1 = 正常。0 のビットがある行が異常箇所）");
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    const uint16_t v = frame_data_u16(reg_read(regs[i].addr));
    Serial.printf("  %-14s 0x%04X %s\n", regs[i].name, v, (v == 0xFFFF) ? "" : " <<<");
  }
  // ★ この 2 本は**フラグの向きが逆**（1 = その状態にある）。0xFFFF と比べてはいけない。
  const uint16_t info = frame_data_u16(reg_read(REG_STAT_INFO));
  Serial.printf("  %-14s 0x%04X  LPM: sensor=%d rate=%d acc=%d（測定中は全部 0）\n",
                "STAT_INFO", info, info & 1, (info >> 1) & 1, (info >> 2) & 1);
  Serial.printf("  %-14s 0x%04X（SYNC 未使用なら 0）\n", "STAT_SYNC_ACTIVE",
                frame_data_u16(reg_read(REG_STAT_SYNC_ACTIVE)));
  // データカウンタ。連続で読んで増えていればサンプルが更新されている証拠。
  const uint16_t rd1 = frame_data_u16(reg_read(REG_RATE_DCNT));
  const uint16_t ad1 = frame_data_u16(reg_read(REG_ACC_DCNT));
  delay(20);
  const uint16_t rd2 = frame_data_u16(reg_read(REG_RATE_DCNT));
  const uint16_t ad2 = frame_data_u16(reg_read(REG_ACC_DCNT));
  Serial.printf("  DCNT 20ms で RATE %u->%u (12bit ラップ)  ACC %u->%u (14bit)%s\n",
                rd1, rd2, ad1, ad2, (rd1 == rd2 && ad1 == ad2) ? "  <<< 増えていない" : "");
  Serial.printf("フレーム統計: total=%lu crc=%lu ids=%lu ce=%lu sat=%lu init=%lu\n",
                (unsigned long)g_frames, (unsigned long)g_crc_err,
                (unsigned long)g_ids_err, (unsigned long)g_ce_err,
                (unsigned long)g_sat_err, (unsigned long)g_init_run);
}

// ------------------------------------------------------------
// 静止性能の実測（このスケッチの本題）
// ------------------------------------------------------------
// ★ **動かさずに置いた状態で実行すること。**
static void cmd_measure(uint32_t seconds) {
  if (!g_inited) { Serial.println("先に 'r' で初期化すること"); return; }

  // ★ 測定条件を先に全部書き出す。**ログだけ見て後から再現できるようにするため。**
  Serial.printf("静止測定 %lu 秒。動かさないこと…\n", (unsigned long)seconds);
  Serial.printf("  条件: SCH16T-K01%s / ジャイロ %.0f LSB/(°/s) / 加速度 %.0f LSB/(m/s²)\n",
                g_is_k01 ? "" : "（★COMP_ID 不一致）", LSB_PER_DPS, LSB_PER_MPS2);
  Serial.printf("        LPF=%uHz(sel %u) / dec=%u / SPI=%luHz / TA=%u\n",
                (unsigned)filt_cutoff_hz(g_filt_rate), g_filt_rate, g_dec,
                (unsigned long)g_spi_hz, g_ta);
  if (!g_is_k01)
    Serial.println("  ★★ COMP_ID が K01 ではない。下のジャイロの数値は 16 倍ずれている"
                   "可能性がある（加速度は無関係）。'G' で実回転から測ること");

  Stat gx, gy, gz, ax, ay, az, an, tp;
  const uint32_t t0 = millis();
  uint32_t n = 0, bad = 0;
  // 内訳も差分で取る。CRC だけ見ていると IDS/CE/飽和を見落とす。
  const uint32_t crc0 = g_crc_err, ids0 = g_ids_err, ce0 = g_ce_err;
  const uint32_t sat0 = g_sat_err, init0 = g_init_run;

  while (millis() - t0 < seconds * 1000UL) {
    const Sample s = read_sample_dec();
    if (!s.ok) { bad++; continue; }
    const double dgx = s.gx / (double)LSB_PER_DPS;
    const double dgy = s.gy / (double)LSB_PER_DPS;
    const double dgz = s.gz / (double)LSB_PER_DPS;
    const double dax = s.ax / (double)LSB_PER_MPS2;
    const double day = s.ay / (double)LSB_PER_MPS2;
    const double daz = s.az / (double)LSB_PER_MPS2;
    gx.add(dgx); gy.add(dgy); gz.add(dgz);
    ax.add(dax); ay.add(day); az.add(daz);
    an.add(sqrt(dax * dax + day * day + daz * daz));
    tp.add(s.temp / (double)LSB_PER_DEGC);
    n++;
  }
  const double dur = (millis() - t0) / 1000.0;
  const double fs  = n / dur;

  Serial.printf("\n--- %lu サンプル / %.1f s = %.1f Hz（SPI 律速。ODR は 'd' で測る）\n",
                (unsigned long)n, dur, fs);
  Serial.printf("不良フレーム %lu 件（crc=%lu ids=%lu ce=%lu 飽和=%lu 初期化中=%lu）\n",
                (unsigned long)bad,
                (unsigned long)(g_crc_err  - crc0),  (unsigned long)(g_ids_err - ids0),
                (unsigned long)(g_ce_err   - ce0),   (unsigned long)(g_sat_err - sat0),
                (unsigned long)(g_init_run - init0));
  if (n < 100) { Serial.println("サンプルが少なすぎる。配線か初期化を疑う"); return; }

  // ★ エイリアシングの警告。データシート §5.5 Table 16 の最小推奨読み出しレートを
  //   下回って読むと、フィルターの阻止域のノイズが折り返して帯域内に入り、
  //   **ノイズを過大評価する**（バイアスは影響を受けない）。
  const uint32_t need = min_read_rate_hz(g_filt_rate);
  if (fs < (double)need) {
    Serial.printf("★ 読み出し %.0fHz < 最小推奨 %luHz（現在のフィルター設定）。\n"
                  "  ノイズ値は折り返しで過大に出る。'k' で SPI を速くするか、\n"
                  "  'f' でフィルターを下げる（LPF2 13Hz なら 150Hz で足りる）。\n",
                  fs, (unsigned long)need);
  } else {
    Serial.printf("読み出し %.0fHz >= 最小推奨 %luHz。折り返しの心配なし\n",
                  fs, (unsigned long)need);
  }

  Serial.printf("\n[ジャイロ] 単位 deg/s%s\n",
                g_is_k01 ? "" : "  ★COMP_ID 不一致（16 倍ずれの可能性）");
  Serial.printf("           %-12s %-12s %-12s %s\n", "bias", "sd", "p-p", "ノイズ密度*");
  // ★ **帯域に fs/2 を使ってはいけない。** ノイズが通っているのは LPF の帯域で、
  //   fs/2 はそれより広いことが多い（68Hz LPF を 500Hz 読み → fs/2=250Hz）。
  //   広い帯域で割ると**密度が小さく出る**＝都合のいい方向に外れる。
  //   折り返しが起きている（fs/2 < fc）ときだけ fs/2 の方が実態に近いので min を取る。
  const double fc = filt_cutoff_hz(g_filt_rate);
  const double bw = (fs / 2.0 < fc) ? (fs / 2.0) : fc;
  const Stat* gs[3] = { &gx, &gy, &gz };
  const char* gn[3] = { "X", "Y", "Z" };
  for (int i = 0; i < 3; i++)
    Serial.printf("  %-8s %+12.5f %12.5f %12.5f %12.6f\n", gn[i],
                  gs[i]->mean, gs[i]->sd(), gs[i]->mx - gs[i]->mn, gs[i]->sd() / sqrt(bw));
  Serial.printf("  * ノイズ密度は sd/sqrt(%.0fHz) [deg/s/sqrt(Hz)]。帯域は LPF の fc を使った概算。\n", bw);
  Serial.println("    実 ENBW は fc の 1.0〜1.6 倍なので、真値はこの 0.8〜1.0 倍。桁合わせ用。");
  Serial.println("    データシート typ: XY 0.0004。桁が合っていれば妥当。");
  Serial.println("    データシート オフセット typ: XY ±0.1 / Z ±0.01 deg/s");

  Serial.println("\n[加速度] 単位 m/s^2");
  Serial.printf("           %-12s %-12s %-12s\n", "mean", "sd", "p-p");
  const Stat* as[3] = { &ax, &ay, &az };
  for (int i = 0; i < 3; i++)
    Serial.printf("  %-8s %+12.5f %12.5f %12.5f\n", gn[i],
                  as[i]->mean, as[i]->sd(), as[i]->mx - as[i]->mn);
  Serial.println("  データシート オフセット typ: ±0.01 m/s^2");

  // ★ ここが BNO085 との直接比較になる。
  //   BNO085 は静止時 |f|/g = 0.985〜0.992（約 -1% のスケール誤差）だった。
  const double g0 = 9.80665;
  Serial.printf("\n[スケール誤差] |a| = %.5f m/s^2   |a|/g = %.5f  (%.3f %%)\n",
                an.mean, an.mean / g0, (an.mean / g0 - 1.0) * 100.0);
  Serial.printf("               |a| の sd = %.5f m/s^2 = %.6f g\n",
                an.sd(), an.sd() / g0);
  Serial.println("  BNO085 実測: |f|/g = 0.985〜0.992 (-1%), sd = 0.0008 g (2Hz LPF 後)");
  Serial.println("  ※ 傾いて置いてあっても |a| は回転不変なので、この比較は姿勢に依らない");
  Serial.println("  ※★ この 1g 検証が確かめているのは**加速度の感度だけ**。"
                 "加速度の LSB は品種に依らないので、");
  Serial.println("     ここが 1.000 に乗ってもジャイロの感度の裏付けには一切ならない。"
                 "ジャイロは 'G' で測る。");

  Serial.printf("\n[温度] %.2f degC (sd %.3f)\n", tp.mean, tp.sd());
}

// ------------------------------------------------------------
// ジャイロ感度の実測（'G'）
// ------------------------------------------------------------
// ★★ **ジャイロのスケールを端から端まで確かめられる唯一の手段。**
//   1g 検証（'m' の |a|/g）は加速度の感度しか見ていないので、ジャイロ側の
//   LSB_PER_DPS・DYN ビット・20bit の符号拡張のどれが間違っていても通ってしまう。
//   他品種が挿さっている場合もここでしか出ない（感度が 16 倍違うのはジャイロだけで、
//   加速度は品種共通なので |a|/g は 1.000 に乗る）。
//
// 原理: 既知の角度だけゆっくり回し、生の LSB を時間積分する。
//   積分値 [LSB·s] ÷ 回した角度 [°] = 感度 [LSB/(°/s)]
//   角度は実測しなくてよい。**机の角に押し当てて 90°、一周させて 360°** で足りる。
//   ゆっくり回せば（±300°/s 未満）飽和しないので、レンジの心配も要らない。
static void cmd_gyro_scale() {
  if (!g_inited) { Serial.println("先に 'r' で初期化すること"); return; }

  Serial.println();
  Serial.println("=== ジャイロ感度の実測 ===");
  Serial.println("  1. 基板を机に平らに置く（Z 軸が鉛直）");
  Serial.println("  2. 何かキーを押す → 積分が始まる");
  Serial.println("  3. **ゆっくり** きっちり 90°（または 360°）回して止める");
  Serial.println("  4. もう一度キーを押す → 止まる");
  Serial.println("  出てきた LSB·s を回した角度で割った値が LSB/(°/s)。");
  Serial.printf("  期待値: %.0f LSB/(°/s)（K01・DYN=0b%d%d%d）\n", LSB_PER_DPS,
                (RATE_DYN_DEFAULT >> 2) & 1, (RATE_DYN_DEFAULT >> 1) & 1,
                RATE_DYN_DEFAULT & 1);
  Serial.println("  開始を待っています…");

  while (Serial.available()) Serial.read();          // 溜まっている入力を捨てる
  const uint32_t tw = millis();
  while (!Serial.available()) {
    if (millis() - tw > 60000UL) { Serial.println("  60 秒待って入力が無いので中止"); return; }
  }
  while (Serial.available()) Serial.read();
  Serial.println("  積分開始。回してください");

  // バイアス分を引くため、最初の 1 秒は静止しているものとして平均を取る。
  // ★ ここで動かすと感度がずれる。上の手順どおり「押す → 回す」の順で。
  double bias[3] = { 0, 0, 0 };
  uint32_t nb = 0;
  const uint32_t tb = millis();
  while (millis() - tb < 1000) {
    const Sample s = read_sample_dec();
    if (!s.ok) continue;
    bias[0] += s.gx; bias[1] += s.gy; bias[2] += s.gz; nb++;
  }
  if (nb < 10) { Serial.println("  ★ サンプルが取れない。初期化と配線を見ること"); return; }
  for (int i = 0; i < 3; i++) bias[i] /= (double)nb;
  Serial.printf("  静止バイアス（生 LSB）: %+.1f %+.1f %+.1f（%lu サンプル）\n",
                bias[0], bias[1], bias[2], (unsigned long)nb);
  Serial.println("  → いま回してください。終わったらキーを押す");

  double integ[3] = { 0, 0, 0 };
  double peak[3]  = { 0, 0, 0 };
  uint32_t n = 0, bad = 0;
  uint32_t t_prev = micros();
  const uint32_t t0 = millis();
  while (!Serial.available()) {
    // ★ 打ち切り判定はループの先頭で。不良フレームで continue する経路の後ろに
    //   置くと、全フレームが不良のときに永久に抜けない。
    if (millis() - t0 > 120000UL) { Serial.println("  2 分で打ち切り"); break; }
    const Sample s = read_sample_dec();
    // ★ 不良フレームでは t_prev を更新しない。更新してしまうとその区間の dt が
    //   丸ごと落ちて、**積分値が小さく出る**（＝感度が小さく見える）。
    //   更新しなければ次の良サンプルの dt が穴を含むので積分は保存される。
    if (!s.ok) { bad++; continue; }
    const uint32_t t_now = micros();
    const double dt = (double)(t_now - t_prev) * 1e-6;     // ラップしても差は正しい
    t_prev = t_now;
    const double g[3] = { s.gx - bias[0], s.gy - bias[1], s.gz - bias[2] };
    for (int i = 0; i < 3; i++) {
      integ[i] += g[i] * dt;
      const double a = fabs(g[i]);
      if (a > peak[i]) peak[i] = a;
    }
    n++;
  }
  while (Serial.available()) Serial.read();

  Serial.printf("\n  %lu サンプル / 不良 %lu 件\n", (unsigned long)n, (unsigned long)bad);
  const char* ax[3] = { "X", "Y", "Z" };
  Serial.println("  軸  積分[LSB·s]      ÷90°         ÷360°        最大レート[今の感度で °/s]");
  for (int i = 0; i < 3; i++)
    Serial.printf("  %-3s %+14.1f %12.1f %12.1f %12.1f\n", ax[i],
                  integ[i], fabs(integ[i]) / 90.0, fabs(integ[i]) / 360.0,
                  peak[i] / LSB_PER_DPS);
  Serial.println();
  Serial.printf("  回した軸の「÷90°」または「÷360°」の列が %.0f 近辺なら、"
                "いまのスケール %.0f は正しい。\n",
                LSB_PER_DPS, LSB_PER_DPS);
  Serial.println("  16 倍ずれていたら K01 ではない部品が挿さっている。'i' で COMP_ID を見ること。");
  Serial.println("  ※ 最大レートが ±300°/s に近いなら速すぎる。飽和の疑いがあるのでやり直す。");
}

// ------------------------------------------------------------
// DRY_SYNC による実 ODR の測定
// ------------------------------------------------------------
static volatile uint32_t g_dry_count = 0;
static volatile uint32_t g_dry_last_us = 0;
static volatile uint32_t g_dry_min_us = 0xFFFFFFFF, g_dry_max_us = 0;

static void dry_isr() {
  const uint32_t now = micros();
  if (g_dry_last_us) {
    const uint32_t d = now - g_dry_last_us;
    if (d < g_dry_min_us) g_dry_min_us = d;
    if (d > g_dry_max_us) g_dry_max_us = d;
  }
  g_dry_last_us = now;
  g_dry_count++;
}

static void cmd_dry(uint32_t seconds) {
  // ★ 初期化前は DRY_DRV_EN が立っていないので GP21 は誰も駆動していない。
  //   そこで割り込みを張ると雑音を数えて**ありもしない ODR を報告する**。
  if (!g_inited) { Serial.println("先に 'r' で初期化すること（DRY_DRV_EN が必要）"); return; }

  Serial.printf("DRY_SYNC の立ち上がりを %lu 秒数える…\n", (unsigned long)seconds);
  g_dry_count = 0; g_dry_last_us = 0;
  g_dry_min_us = 0xFFFFFFFF; g_dry_max_us = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_DRY), dry_isr, RISING);
  // ★ DRY は「出力レジスタが更新されてから最初の読み出しまで」High なので、
  //   読まずに放置するとパルスが出ない。読みながら数える。
  const uint32_t t0 = millis();
  uint32_t reads = 0;
  while (millis() - t0 < seconds * 1000UL) { read_sample_dec(); reads++; }
  detachInterrupt(digitalPinToInterrupt(PIN_DRY));

  const double dur = (millis() - t0) / 1000.0;
  const double dry_hz = g_dry_count / dur, read_hz = reads / dur;
  Serial.printf("DRY 回数 %lu / %.1f s = %.1f Hz（読み出しは %.1f Hz）\n",
                (unsigned long)g_dry_count, dur, dry_hz, read_hz);
  if (g_dry_count > 2)
    Serial.printf("周期 min %lu us / max %lu us\n",
                  (unsigned long)g_dry_min_us, (unsigned long)g_dry_max_us);
  else
    Serial.println("★ パルスが取れていない。DRY_DRV_EN・配線・ピン番号を確認すること");

  // ★★ **読み出しが ODR より遅いと、この値は ODR ではなく読み出しレートになる。**
  //   DRY は「出力レジスタ更新 → 最初の読み出し」の間だけ High なので、
  //   読む方が遅ければ更新ごとに 1 パルスではなく読むごとに 1 パルスになる。
  //   dec=NONE(11.8kHz) や 5900Hz は 1MHz SPI では原理的に追いつかないので、
  //   下の「期待値」と食い違っても故障ではない。ここを混同すると配線を疑い始める。
  if (g_dry_count > 2 && dry_hz > read_hz * 0.9)
    Serial.println("★ DRY 回数 ≒ 読み出し回数。**これは ODR ではなく読み出し律速の値**。"
                   "'k' で SPI を速くするか 'e' で dec を落として再測定すること");

  Serial.printf("設定上の期待値: dec=%u → %s\n", g_dec,
                g_dec == DEC_NONE ? "11.8kHz(85us)" :
                g_dec == DEC_5900 ? "5.9kHz(169us)" :
                g_dec == DEC_2950 ? "2.95kHz(338us)" :
                g_dec == DEC_1475 ? "1.475kHz(678us)" : "738Hz(1355us)");
}

// ------------------------------------------------------------
// 連続表示 / CSV 出力
// ------------------------------------------------------------
static bool g_stream = false;
static bool g_stream_csv = false;
static uint32_t g_stream_last_ms = 0;

static void stream_tick() {
  if (!g_stream) return;
  if (millis() - g_stream_last_ms < (g_stream_csv ? 10 : 200)) return;
  g_stream_last_ms = millis();
  const Sample s = read_sample_dec();
  if (g_stream_csv) {
    // t_us,gx,gy,gz,ax,ay,az,temp,ok （単位は deg/s, m/s^2, degC。ok は 1/0）
    // ★ 列は 9 つ。'v' で出すヘッダ行と必ず揃えること。
    Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d\n",
      (unsigned long)micros(),
      s.gx / LSB_PER_DPS, s.gy / LSB_PER_DPS, s.gz / LSB_PER_DPS,
      s.ax / LSB_PER_MPS2, s.ay / LSB_PER_MPS2, s.az / LSB_PER_MPS2,
      s.temp / LSB_PER_DEGC, s.ok ? 1 : 0);
  } else {
    const float axf = s.ax / LSB_PER_MPS2, ayf = s.ay / LSB_PER_MPS2,
                azf = s.az / LSB_PER_MPS2;
    Serial.printf("gyro %+8.3f %+8.3f %+8.3f deg/s | acc %+8.3f %+8.3f %+8.3f m/s2 "
                  "| |a|=%6.3f | %5.1fC %s\n",
      s.gx / LSB_PER_DPS, s.gy / LSB_PER_DPS, s.gz / LSB_PER_DPS,
      axf, ayf, azf, sqrtf(axf * axf + ayf * ayf + azf * azf),
      s.temp / LSB_PER_DEGC, s.ok ? "" : "<<BAD");
  }
}

// ------------------------------------------------------------
// *_1 と *_2 の比較
// ------------------------------------------------------------
static void cmd_compare() {
  if (!g_inited) { Serial.println("先に 'r' で初期化すること"); return; }
  Serial.println("補間出力(*_1) と デシメーション出力(*_2) を 10 回ずつ");
  Serial.println("  *_1 は補間（実効 ODR 非常に高・ジッタ<2.6us）、*_2 は DEC 設定で間引いた出力。");
  Serial.println("  レンジ設定は別フィールド（DYN_*_XYZ1 / XYZ2）。同じ設定なら値もほぼ一致する。");
  for (int i = 0; i < 10; i++) {
    const Sample r = read_sample_raw();
    const Sample d = read_sample_dec();
    Serial.printf("  raw g(%+7ld %+7ld %+7ld) a(%+7ld %+7ld %+7ld) | "
                  "dec g(%+7ld %+7ld %+7ld) a(%+7ld %+7ld %+7ld)\n",
      (long)r.gx, (long)r.gy, (long)r.gz, (long)r.ax, (long)r.ay, (long)r.az,
      (long)d.gx, (long)d.gy, (long)d.gz, (long)d.ax, (long)d.ay, (long)d.az);
    delay(50);
  }
}

// ------------------------------------------------------------
// ヘルプ
// ------------------------------------------------------------
static void cmd_help() {
  Serial.println();
  Serial.println("=== SCH16T-K01-10 単独テスト（Pico 2）===");
  Serial.println("  h  このヘルプ");
  Serial.println("  a  TA (TA9:TA8) を走査する。応答しないときまずこれ");
  Serial.println("  i  ID と設定レジスタを読む");
  Serial.println("  t  SYS_TEST に書いて読み返す（データシート推奨の疎通確認）");
  Serial.println("  r  初期化（ソフトリセット）");
  Serial.println("  R  初期化（EXTRESN でハードリセット）");
  Serial.println("  s  ステータス全ダンプ＋フレーム統計");
  Serial.println("  m  静止 10 秒測定（バイアス・ノイズ・1g スケール誤差）");
  Serial.println("  M  静止 60 秒測定");
  Serial.println("  G  ジャイロ感度の実測（90°/360° 回して積分。**ジャイロのスケール検証はこれだけ**）");
  Serial.println("  d  DRY_SYNC で実 ODR を測る（5 秒）");
  Serial.println("  c  連続表示 開始/停止（人が読む形式）");
  Serial.println("  v  CSV 出力 開始/停止（100Hz。ログ取り用）");
  Serial.println("  p  *_1 と *_2 の生値を比べる");
  Serial.println("  k  SPI クロックを 1 → 2 → 5 → 10MHz と切り替える");
  Serial.println("  f  ジャイロ／加速度フィルターを切り替える（68/30/13Hz）");
  Serial.println("  e  デシメーションを切り替える（738/1475/2950/5900/none）");
  Serial.println("  z  1 フレームだけ生で投げて中身を見る（配線確認用）");
  Serial.printf("  現在: TA=%u  SPI=%lu Hz  filt=%u  dec=%u  init=%s\n",
                g_ta, (unsigned long)g_spi_hz, g_filt_rate, g_dec,
                g_inited ? "OK" : "NO");
  Serial.printf("  感度: ジャイロ %.0f LSB/(°/s)  加速度 %.0f LSB/(m/s²)  （K01 固定%s）\n",
                LSB_PER_DPS, LSB_PER_MPS2, g_is_k01 ? "" : " ★COMP_ID 不一致");
  Serial.println();
}

// ------------------------------------------------------------
// setup / loop
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_EXTRESN, OUTPUT);
  digitalWrite(PIN_EXTRESN, HIGH);      // 既定 High（Low でリセット）
  // ★ DRY_SYNC は SYNC 入力との兼用ピン。**出力にしないこと。**
  //   ★ プルダウンを掛ける。DRY_DRV_EN が立つまでこのピンは誰も駆動していないので、
  //     プル無し入力だと RP2350 のパッドが中間電位に張り付いて雑音を拾い、
  //     'd' の割り込みが**ありもしない ODR を数える**。DRY は high-active の
  //     push-pull 出力なので、プルダウンしても動作には影響しない。
  //     （プル無し入力を放置して踏んだ実例が本体側の GPIO42/44 にある。CLAUDE.md 参照）
  pinMode(PIN_DRY, INPUT_PULLDOWN);

  SPI.setRX(PIN_MISO);
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.begin(false);                     // CS はこちらで叩くのでライブラリには任せない

  // データシートのパワーオン待ち。250ms 経つ前に SPI を叩くと初期化中の応答になる。
  const uint32_t t0 = millis();
  while (millis() - t0 < 250) { /* wait */ }

  // USB シリアルが開くまで待つ（開かなくても 5 秒で進む）
  const uint32_t t1 = millis();
  while (!Serial && millis() - t1 < 5000) { /* wait */ }

  Serial.println();
  Serial.println("SCH16T-K01-10 standalone test (RP2350 / Pico 2)");
  Serial.println("配線: MISO=GP16 CS=GP17 SCK=GP18 MOSI=GP19 EXTRESN=GP20 DRY=GP21");

  // ★ **SPI を叩く前にフレーム組み立てを自己テストする。**
  //   ここで落ちたら配線をいくら見ても無駄なので、先に出す。
  selftest_frames();

  cmd_help();

  cmd_id();
  sch_init(false);
}

void loop() {
  stream_tick();

  if (!Serial.available()) return;
  const int c = Serial.read();
  switch (c) {
    case 'h': case '?': cmd_help(); break;
    case 'a': cmd_scan_ta(); break;
    case 'i': cmd_id(); break;
    case 't': cmd_systest(); break;
    case 'r': sch_init(false); break;
    case 'R': sch_init(true);  break;
    case 's': cmd_status(); break;
    case 'm': cmd_measure(10); break;
    case 'M': cmd_measure(60); break;
    case 'G': cmd_gyro_scale(); break;
    case 'd': cmd_dry(5); break;
    case 'p': cmd_compare(); break;
    case 'c':
      g_stream = !g_stream; g_stream_csv = false;
      Serial.printf("stream %s\n", g_stream ? "ON" : "OFF");
      break;
    case 'v':
      g_stream = !g_stream; g_stream_csv = g_stream;
      if (g_stream) Serial.println("t_us,gx_dps,gy_dps,gz_dps,ax_mps2,ay_mps2,az_mps2,temp_c,ok");
      else          Serial.println("# csv off");
      break;
    case 'k': {
      g_spi_hz = (g_spi_hz == 1000000) ? 2000000 :
                 (g_spi_hz == 2000000) ? 5000000 :
                 (g_spi_hz == 5000000) ? 10000000 : 1000000;
      Serial.printf("SPI = %lu Hz（既定モードの上限 10.5MHz）\n", (unsigned long)g_spi_hz);
      break;
    }
    case 'f': {
      g_filt_rate = g_filt_acc =
        (g_filt_rate == FILT_68HZ) ? FILT_30HZ :
        (g_filt_rate == FILT_30HZ) ? FILT_13HZ : FILT_68HZ;
      Serial.printf("filter sel = %u（0=68Hz 1=30Hz 2=13Hz）→ 再初期化する\n", g_filt_rate);
      sch_init(false);
      break;
    }
    case 'e': {
      g_dec = (g_dec == DEC_738)  ? DEC_1475 :
              (g_dec == DEC_1475) ? DEC_2950 :
              (g_dec == DEC_2950) ? DEC_5900 :
              (g_dec == DEC_5900) ? DEC_NONE : DEC_738;
      Serial.printf("decimation = %u → 再初期化する\n", g_dec);
      sch_init(false);
      break;
    }
    case 'z': {
      // 生フレームを 1 つ投げて中身を見る。
      // 全部 0x0000 → MISO が来ていない（配線・電源・CS を疑う）
      // 全部 0xFFFF → MISO が High 固定（Hi-Z のままプルアップに引かれている）
      Serial.println("ASIC_ID を 2 フレーム投げる（1 フレーム目の応答は 1 つ前の要求の結果）");
      print_frame("frame1", req_read(REG_ASIC_ID));
      print_frame("frame2", req_read(REG_ASIC_ID));
      break;
    }
    default: break;   // 改行などは無視
  }
}
