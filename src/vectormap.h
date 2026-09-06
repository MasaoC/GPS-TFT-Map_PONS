// ============================================================
// File    : vectormap.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : フラッシュ内蔵ベクタ地図（OpenStreetMap 由来）のデータ形式定義と
//           描画 API。SD カード上の BMP タイルを置き換える。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
//
// 地図データのライセンス:
//   Map data (c) OpenStreetMap contributors, ODbL 1.0
//   https://www.openstreetmap.org/copyright
//   vectormap_data.cpp は Derivative Database に該当するため ODbL で提供する。
//   ファームウェアのソースコード本体は MIT ライセンス（ODbL 4.5(a) Collective Database）。
// ============================================================
#ifndef VECTORMAP_H
#define VECTORMAP_H

#include <stdint.h>

// ===== 地物クラス =====
// 描画順もこの順（面を先に塗り、線を後から重ねる）。
// VM_CLASS_IS_AREA() が真のクラスはリングを塗りつぶし、偽なら折れ線として描く。
enum vm_class : uint8_t {
  VM_LANDMASS = 0,  // 陸地（面）
  VM_WATER,         // 水面（面）— 琵琶湖・海
  VM_URBAN,         // 市街地（面）— 予約枠。現在データは生成していない
                    //   （フラッシュを食う割に「幹線と湖面が分かればよい」用途に寄与しないため）
  VM_MOTORWAY,      // 高速道路（線・太）
  VM_TRUNK,         // 幹線道路（線）
  VM_RAIL,          // 鉄道（線・破線）
  VM_CLASS_COUNT
};
#define VM_CLASS_IS_AREA(c) ((c) <= VM_URBAN)

// ===== LOD（Level of Detail） =====
// 同じ地物をズーム別に間引いた 4 段階で持つ。実行時は scale から 1 段だけ選ぶ。
// LOD ごとに「収録範囲」と「座標の分解能」を変えることで、全国対応を
// フラッシュ容量内に収める（高ズームは狭く細かく、低ズームは広く粗く）。
#define VM_LOD_COUNT 4

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
// 座標はタイル原点からのオフセットで、単位は vm_lod_unit_e7[lod]（1e-7 度単位）。
// 原点相対の絶対値なので、前の点からの累積加算は不要。
//   lat_e7 = tile.org_lat_e7 + d_lat * vm_lod_unit_e7[lod]
//
// 面クラスのリングは暗黙に閉じている（最後の点から最初の点へ）。
// 1 レコードに湖本体と島をまとめて入れることで、even-odd 塗りで島が穴として抜ける。
struct vm_tile {
  int32_t org_lat_e7;  // タイル原点（1e-7 度）
  int32_t org_lon_e7;
  uint32_t blob_off;   // vm_blob 内のバイトオフセット
  uint16_t blob_len;   // バイト長
  uint8_t lod;
  uint8_t _pad;
};

// ===== 自動生成データ（tools/vectormap/build_vectormap.py が出力） =====
extern const vm_tile vm_tiles[];
extern const uint16_t vm_tile_count;
extern const uint8_t vm_blob[];
extern const int32_t vm_lod_unit_e7[VM_LOD_COUNT];      // 座標 1 単位あたりの 1e-7 度
extern const int32_t vm_lod_tilesize_e7[VM_LOD_COUNT];  // タイル一辺（1e-7 度）
extern const float vm_lod_min_scale[VM_LOD_COUNT];      // その LOD を使う最小 scale [px/km]
// その LOD が陸地ポリゴン（VM_LANDMASS）を持つか。真なら背景を海色で塗ってから陸を重ねる。
// 偽の LOD（琵琶湖周辺の高精細）では背景は白のまま、水面だけを塗る。
extern const bool vm_lod_sea_background[VM_LOD_COUNT];
extern const uint32_t vm_blob_size;                     // vm_blob の総バイト数
extern const uint16_t vm_lod_tiles[VM_LOD_COUNT];       // LOD 別のタイル数
extern const uint32_t vm_lod_bytes[VM_LOD_COUNT];       // LOD 別のバイト数
extern const uint32_t vm_class_points[VM_CLASS_COUNT];  // 地物クラス別の点数
extern const char* const vm_class_names[VM_CLASS_COUNT];
extern const char vm_data_attribution[];                // 起動時に表示する帰属表示
extern const char vm_data_source_date[];                // 元データのスナップショット日付

// ===== 描画 =====
// backscreen へベクタ地図を直接描画する。回転は latLonToXY と同じ式で座標変換に
// 吸収されるため、TRACKUP でも画面四隅まで隙間なく描ける。
void draw_vectormap(double center_lat, double center_lon, float scale, float up);

// scale から使用する LOD を返す（デバッグ表示用）。
int vm_select_lod(float scale);

// 直近の draw_vectormap で描画したタイル数・点数（性能計測用）。
extern uint16_t vm_last_tiles;
extern uint32_t vm_last_points;  // 1 フレームで数千点になるため 32bit

#endif  // VECTORMAP_H
