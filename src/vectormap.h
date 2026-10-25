// ============================================================
// File    : vectormap.h
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : フラッシュ内蔵ベクタ地図（OpenStreetMap 由来）のデータ形式と描画 API。
//           v7 の src/vectormap.h と同じデータ形式（データファイルは v7 と同じものを使う）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
//
// 地図データのライセンス:
//   Map data (c) OpenStreetMap contributors, ODbL 1.0
//   https://www.openstreetmap.org/copyright
//   flashdata/vectormap_data.cpp は Derivative Database に該当するため ODbL で提供する（LICENSE.ODbL）。
//   ファームウェアのソースコード本体は MIT ライセンス（ODbL 4.5(a) Collective Database）。
// ============================================================
#ifndef VECTORMAP_H
#define VECTORMAP_H

#include <stdint.h>
#include "../settings.h"

// ===== 地物クラス =====
// 描画順もこの順（面を先に塗り、線を後から重ねる）。
// VM_CLASS_IS_AREA() が真のクラスはリングを塗りつぶし、偽なら折れ線として描く。
enum vm_class : uint8_t {
  VM_LANDMASS = 0,  // 陸地（面）
  VM_WATER,         // 水面（面）— 琵琶湖・海
  VM_URBAN,         // 市街地（面）— 予約枠。現在データは生成していない
  VM_MOTORWAY,      // 高速道路（線・太）
  VM_TRUNK,         // 幹線道路（線）
  VM_RAIL,          // 鉄道（線）
  VM_CLASS_COUNT
};
#define VM_CLASS_IS_AREA(c) ((c) <= VM_URBAN)

// ===== LOD（Level of Detail） =====
// 同じ地物をズーム別に間引いた 4 段階で持つ。実行時は scale から 1 段だけ選ぶ。
// ★ データ側（flashdata/vectormap_data.cpp）の配列長と必ず一致すること。
//   v3 は 2MB の Flash なので、v7 の 16MB 版（VECTORMAP_HIRES、5 段）は使えない。
#define VM_LOD_COUNT 4

// ===== レコード形式（vm_blob 内） =====
//   uint8  cls           地物クラス
//   uint8  nrings        リング数（湖本体＋島、など。線クラスでは折れ線の本数）
//   int16  bb_min_lat, bb_min_lon, bb_max_lat, bb_max_lon   レコード全体の bbox
//   uint16 npts[nrings]  各リングの点数
//   int16  d_lat, d_lon  × 全点（リング順に連結）
//
// 座標はタイル原点からのオフセットで、単位は vm_lod_unit_e7[lod]（1e-7 度単位）。
//   lat_e7 = tile.org_lat_e7 + d_lat * vm_lod_unit_e7[lod]
// 面クラスのリングは暗黙に閉じている。湖本体と島を 1 レコードに入れ、even-odd 塗りで島を穴として抜く。
struct vm_tile {
  int32_t org_lat_e7;  // タイル原点（1e-7 度）
  int32_t org_lon_e7;
  uint32_t blob_off;   // vm_blob 内のバイトオフセット
  uint16_t blob_len;   // バイト長
  uint8_t lod;
  uint8_t _pad;
};

// ===== 自動生成データ（v7 の tools/vectormap/build_vectormap.py が出力） =====
extern const vm_tile vm_tiles[];
extern const uint16_t vm_tile_count;
extern const uint8_t vm_blob[];
extern const int32_t vm_lod_unit_e7[VM_LOD_COUNT];      // 座標 1 単位あたりの 1e-7 度
extern const int32_t vm_lod_tilesize_e7[VM_LOD_COUNT];  // タイル一辺（1e-7 度）
extern const float vm_lod_min_scale[VM_LOD_COUNT];      // その LOD を使う最小 scale [px/km]
extern const bool vm_lod_sea_background[VM_LOD_COUNT];  // 真なら背景を海色で塗ってから陸を重ねる
extern const uint32_t vm_blob_size;
extern const uint16_t vm_lod_tiles[VM_LOD_COUNT];
extern const uint32_t vm_lod_bytes[VM_LOD_COUNT];
extern const uint32_t vm_class_points[VM_CLASS_COUNT];
extern const char* const vm_class_names[VM_CLASS_COUNT];
extern const char vm_data_attribution[];                // 起動時に表示する帰属表示
extern const char vm_data_source_date[];                // 元データのスナップショット日付

// ===== 描画 =====
// 地図のスプライト（map_draw.cpp の backscreen）へベクタ地図を描く。背景の塗りつぶしもここで行う。
// 座標変換は map_draw.cpp の地図の投影（map_begin_frame() で決めたもの）と同じ。
void draw_vectormap();

// scale から使用する LOD を返す（地図一覧の表示用）。
int vm_select_lod(float scale);

// 直近の draw_vectormap で描画したタイル数・点数（地図一覧の表示用）。
extern uint16_t vm_last_tiles;
extern uint32_t vm_last_points;

#endif  // VECTORMAP_H
