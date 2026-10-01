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
// Updated : 2026/10/01
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
static const uint8_t RATE_DYN_DEFAULT  = 0b001;  // ±327.68 °/s  → 1600 LSB/(°/s)
static const uint8_t ACC12_DYN_DEFAULT = 0b001;  // ±163.84 m/s² → 3200 LSB/(m/s²)
static const uint8_t ACC3_DYN_DEFAULT  = 0b000;  // ±260 m/s²    → 1600 LSB/(m/s²)

// 上の設定に対応する感度（データシート Table 63/64、20bit 出力）。
// ★ レンジ設定を変えたらここも変えること。
static const float LSB_PER_DPS   = 1600.0f;
static const float LSB_PER_MPS2  = 3200.0f;
static const float LSB_PER_DEGC  = 100.0f;   // Table 8

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
static uint64_t xfer48(uint64_t frame) {
  uint16_t buf[3];
  buf[0] = (uint16_t)((frame >> 32) & 0xFFFF);
  buf[1] = (uint16_t)((frame >> 16) & 0xFFFF);
  buf[2] = (uint16_t)( frame        & 0xFFFF);

  SPI.beginTransaction(SPISettings(g_spi_hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);                 // CS 立ち下がり → データ有効まで 40ns。余裕を取る
  for (int i = 0; i < 3; i++) buf[i] = SPI.transfer16(buf[i]);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);                 // CS の High 期間を必ず作る

  g_frames++;
  return ((uint64_t)buf[0] << 32) | ((uint64_t)buf[1] << 16) | (uint64_t)buf[2];
}

// 読み出し要求を 1 フレーム投げる。**戻ってくるのは 1 つ前の要求の結果**。
static uint64_t req_read(uint8_t addr) {
  uint64_t f = 0;
  f |= (uint64_t)(g_ta & 0x03) << 46;
  f |= (uint64_t)addr << 38;
  f |= (uint64_t)1 << 35;
  f |= (uint64_t)crc8_frame(f);
  return xfer48(f);
}

// 書き込み。応答は見ない。
static void reg_write(uint8_t addr, uint16_t value) {
  uint64_t f = 0;
  f |= (uint64_t)(g_ta & 0x03) << 46;
  f |= (uint64_t)addr << 38;
  f |= (uint64_t)1 << 37;                // WRITE
  f |= (uint64_t)1 << 35;
  f |= (uint64_t)value << 8;
  f |= (uint64_t)crc8_frame(f);
  (void)xfer48(f);
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
static bool sch_init(bool use_hard_reset) {
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

  // ---- 設定を書く（EN_SENSOR の前に済ませる）----
  Serial.println("[init] write config");
  reg_write(REG_CTRL_FILT_RATE,  build_filt(g_filt_rate));
  reg_write(REG_CTRL_FILT_ACC12, build_filt(g_filt_acc));
  reg_write(REG_CTRL_FILT_ACC3,  build_filt(g_filt_acc));
  reg_write(REG_CTRL_RATE,       build_ctrl_rate());
  reg_write(REG_CTRL_ACC12,      build_ctrl_acc12());
  reg_write(REG_CTRL_ACC3,       ACC3_DYN_DEFAULT);
  set_dry(true);                // ★ read-modify-write。素で書かないこと

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
  for (unsigned i = 0; i < sizeof(stat_regs); i++) req_read(stat_regs[i]);

  // ★ EOI を書くと**ソフトリセットと SYS_TEST 以外の R/W レジスタが全部ロックされる**
  //   （Table 80）。以降の設定変更はリセットしない限り効かない。
  //   EOI 後にレジスタへ書くと CE（コマンドエラー）が立つ。
  Serial.println("[init] EOI | EN_SENSOR");
  reg_write(REG_CTRL_MODE, (uint16_t)(BIT_EOI | BIT_EN_SENSOR));
  delay(5);                     // 規定 3ms

  // ★ ステータスは 2 周読む。1 周目は初期化中のラッチが残っている。
  for (int pass = 0; pass < 2; pass++)
    for (unsigned i = 0; i < sizeof(stat_regs); i++) req_read(stat_regs[i]);

  // 設定の読み返し検証
  struct { uint8_t addr; uint16_t want; const char* name; } chk[] = {
    { REG_CTRL_FILT_RATE,  build_filt(g_filt_rate), "CTRL_FILT_RATE"  },
    { REG_CTRL_FILT_ACC12, build_filt(g_filt_acc),  "CTRL_FILT_ACC12" },
    { REG_CTRL_RATE,       build_ctrl_rate(),       "CTRL_RATE"       },
    { REG_CTRL_ACC12,      build_ctrl_acc12(),      "CTRL_ACC12"      },
  };
  bool ok = true;
  for (unsigned i = 0; i < sizeof(chk) / sizeof(chk[0]); i++) {
    const uint16_t got = frame_data_u16(reg_read(chk[i].addr));
    const bool same = (got == chk[i].want);
    if (!same) ok = false;
    Serial.printf("[init] %-16s want 0x%04X got 0x%04X %s\n",
                  chk[i].name, chk[i].want, got, same ? "" : "  <<< MISMATCH");
  }

  // センサーステータスの総合判定。正常なら全ビット 1（＝異常なし）。
  const uint16_t sum = frame_data_u16(reg_read(REG_STAT_SUM));
  Serial.printf("[init] STAT_SUM = 0x%04X %s\n", sum,
                (sum == 0xFFFF) ? "(all OK)" : "  <<< 0 のビットが異常箇所");
  if (sum != 0xFFFF) ok = false;

  g_inited = ok;
  Serial.printf("[init] %s\n", ok ? "SUCCESS" : "FAILED");
  return ok;
}

// ------------------------------------------------------------
// ID 読み出し
// ------------------------------------------------------------
static void cmd_id() {
  const uint16_t asic = frame_data_u16(reg_read(REG_ASIC_ID));
  const uint16_t comp = frame_data_u16(reg_read(REG_COMP_ID));
  const uint16_t sn1  = frame_data_u16(reg_read(REG_SN_ID1));
  const uint16_t sn2  = frame_data_u16(reg_read(REG_SN_ID2));
  const uint16_t sn3  = frame_data_u16(reg_read(REG_SN_ID3));

  Serial.printf("ASIC_ID = 0x%02X   COMP_ID = 0x%02X\n", asic, comp);
  Serial.printf("Serial  = %05u%01X%04X\n", sn2, sn1 & 0x000F, sn3);
  // ★ 部品の識別は COMP_ID。データシート Table 86 に
  //   「SCH16T-K01 = 0b0000000000100011」と明記されている（= 0x0023）。
  //   ASIC_ID はシリコンのリビジョン（Table 85: [11:8]型 [7:4]major [3:0]minor）で、
  //   部品名の判定には使えない。PX4 が 0x21 を見ているのは実機のリビジョン値。
  if (comp == 0x0023)
    Serial.println("-> COMP_ID が SCH16T-K01 と一致（データシート Table 86）");
  else
    Serial.printf("-> ★ COMP_ID が K01 の 0x0023 ではない。K10 など別品種の可能性。\n"
                  "   感度が違うかもしれないので 'm' の 1g 検証で必ず確かめること\n");
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

  s.ok = frame_ok(fgx) && frame_ok(fgy) && frame_ok(fgz)
      && frame_ok(fax) && frame_ok(fay) && frame_ok(faz) && frame_ok(ft);
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
  s.ok = frame_ok(fgx) && frame_ok(fgy) && frame_ok(fgz)
      && frame_ok(fax) && frame_ok(fay) && frame_ok(faz) && frame_ok(ft);
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

  Serial.printf("静止測定 %lu 秒。動かさないこと…\n", (unsigned long)seconds);
  Stat gx, gy, gz, ax, ay, az, an, tp;
  const uint32_t t0 = millis();
  uint32_t n = 0, bad = 0;
  const uint32_t crc0 = g_crc_err;

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
  Serial.printf("不良フレーム %lu 件（うち CRC %lu 件）\n",
                (unsigned long)bad, (unsigned long)(g_crc_err - crc0));
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

  Serial.println("\n[ジャイロ] 単位 deg/s");
  Serial.printf("           %-12s %-12s %-12s %s\n", "bias", "sd", "p-p", "ノイズ密度*");
  const double bw = fs / 2.0;   // *片側帯域を fs/2 と仮定した概算。真値はフィルター設定次第
  const Stat* gs[3] = { &gx, &gy, &gz };
  const char* gn[3] = { "X", "Y", "Z" };
  for (int i = 0; i < 3; i++)
    Serial.printf("  %-8s %+12.5f %12.5f %12.5f %12.6f\n", gn[i],
                  gs[i]->mean, gs[i]->sd(), gs[i]->mx - gs[i]->mn, gs[i]->sd() / sqrt(bw));
  Serial.println("  * ノイズ密度は sd/sqrt(fs/2) の概算 [deg/s/sqrt(Hz)]。");
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

  Serial.printf("\n[温度] %.2f degC (sd %.3f)\n", tp.mean, tp.sd());
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
  Serial.printf("DRY 回数 %lu / %.1f s = %.1f Hz（読み出しは %.1f Hz）\n",
                (unsigned long)g_dry_count, dur, g_dry_count / dur, reads / dur);
  if (g_dry_count > 2)
    Serial.printf("周期 min %lu us / max %lu us\n",
                  (unsigned long)g_dry_min_us, (unsigned long)g_dry_max_us);
  else
    Serial.println("★ パルスが取れていない。DRY_DRV_EN・配線・ピン番号を確認すること");
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
    // t_us,gx,gy,gz,ax,ay,az,temp  （単位は deg/s, m/s^2, degC）
    Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d\n",
      (unsigned long)micros(),
      s.gx / LSB_PER_DPS, s.gy / LSB_PER_DPS, s.gz / LSB_PER_DPS,
      s.ax / LSB_PER_MPS2, s.ay / LSB_PER_MPS2, s.az / LSB_PER_MPS2,
      s.temp / LSB_PER_DEGC, s.ok ? 1 : 0);
  } else {
    const float axf = s.ax / LSB_PER_MPS2, ayf = s.ay / LSB_PER_MPS2, azf = s.az / LSB_PER_MPS2;
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
  pinMode(PIN_DRY, INPUT);

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
