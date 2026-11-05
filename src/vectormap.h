// ============================================================
// File    : vectormap.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : フラッシュ内蔵ベクタ地図（OpenStreetMap 由来）のデータ形式定義と
//           描画 API。SD カード上の BMP タイルを置き換える。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/11
//
// 地図データのライセンス:
//   Map data (c) OpenStreetMap contributors, ODbL 1.0
//   https://www.openstreetmap.org/copyright
//   vectormap_data.cpp は Derivative Database に該当するため ODbL で提供する。
//   ファームウェアのソースコード本体は MIT ライセンス（ODbL 4.5(a) Collective
//   Database）。
// ============================================================
#ifndef VECTORMAP_H
#define VECTORMAP_H

#include "../settings.h" // VECTORMAP_HIRES（LOD 段数がこれで変わる）
#include <stdint.h>

// ===== 地物クラス =====
// 描画順は vectormap.cpp の VM_DRAW_ORDER
// で定義（面を先に塗り、線を後から重ねる）。 VM_CLASS_IS_AREA()
// が真のクラスはリングを塗りつぶし、偽なら折れ線として描く。 ★
// 既存の軽量版データ（vectormap_data.cpp）のレコード番号 0..5 を変えないよう、
//   従来未使用だった 2 (旧 VM_URBAN) を VM_PARK に割り当て、6, 7
//   を末尾に追加する。
enum vm_class : uint8_t {
  VM_LANDMASS = 0, // 陸地（面）
  VM_WATER,        // 水面（面）— 琵琶湖・海・河川
  VM_PARK,     // 公園・緑地・大学/学校敷地・競技場（面・HIRES 最大ズーム用）
  VM_MOTORWAY, // 高速道路（線・太）
  VM_TRUNK,    // 国道・主要幹線道路（線）
  VM_RAIL,     // 鉄道（線）
  VM_BUILDING, // 大型建物・空港エプロン等（面・HIRES 最大ズーム用）
  VM_STREET,   // 県道・主要街路 secondary/tertiary（細線・HIRES 最大ズーム用）
  VM_CLASS_COUNT
};
#define VM_CLASS_IS_AREA(c) ((c) <= VM_PARK || (c) == VM_BUILDING)

// ===== LOD（Level of Detail） =====
// 同じ地物をズーム別に間引いた 4 段階で持つ。実行時は scale から 1 段だけ選ぶ。
// LOD ごとに「収録範囲」と「座標の分解能」を変えることで、全国対応を
// フラッシュ容量内に収める（高ズームは狭く細かく、低ズームは広く粗く）。
// ★ 16MB 版は最大ズーム専用の LOD を 1 段足すので 5 になる。
//   データ側（vectormap_data*.cpp）の配列長と必ず一致すること。
#ifdef VECTORMAP_HIRES
// 16MB 版のデータは **git に入れていない**（16MB のテキストになるため
// .gitignore）。 生成せずに VECTORMAP_HIRES を有効にすると vm_tiles
// などが未定義になり、 「undefined
// reference」だけが出て原因が分からない。ここで先に止める。
#if !__has_include("flashdata/vectormap_data_hires.cpp")
#error                                                                         \
    "16MB 版の地図データが未生成です。tools/vectormap/README.md の手順で生成してください（python3 build_vectormap.py --variant hires ...）"
#endif
#define VM_LOD_COUNT 5
#else
#define VM_LOD_COUNT 4
#endif

// ===== レコード形式（vm_blob 内） =====
//   uint8  cls           地物クラス
//   uint8  nrings        リング数（湖本体＋島、など。線クラスでは折れ線の本数）
//   int16  bb_min_lat, bb_min_lon, bb_max_lat, bb_max_lon   レコード全体の bbox
//   uint16 npts[nrings]  各リングの点数
//   int16  d_lat, d_lon  × 全点（リング順に連結）
//
// bbox は画面と交差しないレコードを点変換なしで捨てるための早期リジェクト用。
// 「交差しない場合のみ捨てる」ので、画面全体を覆う巨大な水面ポリゴンは正しく残る。
//
// 座標はタイル原点からのオフセットで、単位は vm_lod_unit_e7[lod]（1e-7
// 度単位）。 原点相対の絶対値なので、前の点からの累積加算は不要。
//   lat_e7 = tile.org_lat_e7 + d_lat * vm_lod_unit_e7[lod]
//
// 面クラスのリングは暗黙に閉じている（最後の点から最初の点へ）。
// 1 レコードに湖本体と島をまとめて入れることで、even-odd
// 塗りで島が穴として抜ける。
struct vm_tile {
  int32_t org_lat_e7; // タイル原点（1e-7 度）
  int32_t org_lon_e7;
  uint32_t blob_off; // vm_blob 内のバイトオフセット
  uint32_t blob_len; // バイト長（LOD0 の都市部タイルで 64KB を超えるため 32bit）
  uint8_t lod;
  uint8_t _pad;
};

// ===== 自動生成データ（tools/vectormap/build_vectormap.py が出力） =====
extern const vm_tile vm_tiles[];
extern const uint16_t vm_tile_count;
extern const uint8_t vm_blob[];
extern const int32_t
    vm_lod_unit_e7[VM_LOD_COUNT]; // 座標 1 単位あたりの 1e-7 度
extern const int32_t vm_lod_tilesize_e7[VM_LOD_COUNT]; // タイル一辺（1e-7 度）
extern const float
    vm_lod_min_scale[VM_LOD_COUNT]; // その LOD を使う最小 scale [px/km]
// その LOD
// が陸地ポリゴン（VM_LANDMASS）を持つか。真なら背景を海色で塗ってから陸を重ねる。
// 偽の LOD（琵琶湖周辺の高精細）では背景は白のまま、水面だけを塗る。
extern const bool vm_lod_sea_background[VM_LOD_COUNT];
extern const uint32_t vm_blob_size;                    // vm_blob の総バイト数
extern const uint16_t vm_lod_tiles[VM_LOD_COUNT];      // LOD 別のタイル数
extern const uint32_t vm_lod_bytes[VM_LOD_COUNT];      // LOD 別のバイト数
extern const uint32_t vm_class_points[VM_CLASS_COUNT]; // 地物クラス別の点数
extern const char *const vm_class_names[VM_CLASS_COUNT];
extern const char vm_data_attribution[]; // 起動時に表示する帰属表示
extern const char vm_data_source_date[]; // 元データのスナップショット日付

// ===== 描画 =====
// backscreen へベクタ地図を直接描画する。回転は latLonToXY と同じ式で座標変換に
// 吸収されるため、TRACKUP でも画面四隅まで隙間なく描ける。
void draw_vectormap(double center_lat, double center_lon, float scale,
                    float up);

// scale から使用する LOD を返す（デバッグ表示用）。
int vm_select_lod(float scale);

// 直近の draw_vectormap で描画したタイル数・点数（性能計測用）。
extern uint16_t vm_last_tiles;
extern uint32_t vm_last_points; // 1 フレームで数千点になるため 32bit

#endif // VECTORMAP_H
