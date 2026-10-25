// ============================================================
// File    : vectormap.cpp
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : フラッシュ内蔵ベクタ地図の描画。地図のスプライト（backscreen）へ直接描くので、
//           TRACK UP の回転も座標変換に吸収され、画面の四隅まで隙間なく塗れる。
//           v7 の src/vectormap.cpp を v3 の地図の大きさ（240x230）に合わせたもの。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
//
// 地図データのライセンス: Map data (c) OpenStreetMap contributors, ODbL 1.0
// ============================================================
// ■ v7 と違うところ
//   点の変換で、タイルの原点＋オフセット（1e-7 度の整数）から地図の中心を整数のまま引いてから
//   float にする。RP2040 には浮動小数点の演算器が無いので、v7 の「double で掛けてから float」を
//   点ごとにやると重い（1 点あたり double の掛け算 2 回ぶん減る）。精度もこの方が良い。
#include <Arduino.h>
#include <math.h>

#include "../settings.h"
#include "../display_tft.h"
#include "../map_draw.h"
#include "vectormap.h"

// ===== 描画色（RGB565。v7 と同じ） =====
#define VMCOL_LAND 0xffff       // 陸地: 白
#define VMCOL_WATER 0xb71f      // 水面: 薄い青
#define VMCOL_URBAN COLOR_BRIGHTGRAY
#define VMCOL_MOTORWAY 0xfeae   // 高速道路: 明るい橙
#define VMCOL_TRUNK TFT_YELLOW
#define VMCOL_RAIL COLOR_BRIGHTGRAY

static const uint8_t VM_LINE_WIDTH[VM_CLASS_COUNT] = { 0, 0, 0, 3, 2, 1 };
static const uint16_t VM_COLOR[VM_CLASS_COUNT] = {
  VMCOL_LAND, VMCOL_WATER, VMCOL_URBAN, VMCOL_MOTORWAY, VMCOL_TRUNK, VMCOL_RAIL
};
// 面を先に塗り、線を後から重ねる
static const uint8_t VM_DRAW_ORDER[VM_CLASS_COUNT] = {
  VM_LANDMASS, VM_WATER, VM_URBAN, VM_RAIL, VM_TRUNK, VM_MOTORWAY
};

// 1 レコード分の全リングをここに展開する（map_draw.cpp の VM_MAX_EDGES と同じ上限）
#define VM_MAX_PTS 2048
#define VM_MAX_RINGS 64
static int16_t vm_px[VM_MAX_PTS];
static int16_t vm_py[VM_MAX_PTS];
static uint16_t vm_ring_start[VM_MAX_RINGS + 1];

uint16_t vm_last_tiles = 0;
uint32_t vm_last_points = 0;

// TRACK UP の自機（下寄り 3/4）から画面の隅までの最遠距離 = sqrt(120^2 + 172^2) ≒ 210px。
// この半径を覆うタイルを集めれば、どの方位を向いても四隅が埋まる。
#define VM_VIEW_RADIUS_PX 215.0f

static inline void vm_project_e7(int32_t lat_e7, int32_t lon_e7, int16_t* ox, int16_t* oy) {
  map_project_d((float)(lat_e7 - map_frame.cLat_e7) * 1e-7f,
                (float)(lon_e7 - map_frame.cLon_e7) * 1e-7f, ox, oy);
}

int vm_select_lod(float scale) {
  for (int lod = 0; lod < VM_LOD_COUNT; lod++) {
    if (scale >= vm_lod_min_scale[lod]) return lod;
  }
  return VM_LOD_COUNT - 1;
}

// 1 レコードを描く。p はレコード先頭。次のレコード先頭を返す。
// cls_filter と違うクラスはヘッダだけ読んで飛ばす（点は変換しない）。
static const uint8_t* vm_draw_record(const uint8_t* p, const vm_tile* t, int lod, uint8_t cls_filter) {
  const uint8_t cls = p[0];
  const uint8_t nrings = p[1];
  const int16_t* bb = (const int16_t*)(p + 2);
  const uint16_t* npts = (const uint16_t*)(p + 10);

  uint32_t total = 0;
  for (uint8_t r = 0; r < nrings; r++) total += npts[r];
  const uint8_t* pts = (const uint8_t*)(npts + nrings);
  const uint8_t* next = pts + total * 4;

  if (cls != cls_filter) return next;
  if (nrings == 0 || nrings > VM_MAX_RINGS) return next;
  if (total == 0 || total > VM_MAX_PTS) return next;

  // bbox の 4 隅を変換し、地図の矩形と交わらなければ点を変換せずに捨てる
  const int32_t unit = vm_lod_unit_e7[lod];
  {
    int16_t bx[4], by[4];
    const int16_t la[4] = { bb[0], bb[0], bb[2], bb[2] };
    const int16_t lo[4] = { bb[1], bb[3], bb[1], bb[3] };
    for (int i = 0; i < 4; i++) {
      vm_project_e7(t->org_lat_e7 + (int32_t)la[i] * unit, t->org_lon_e7 + (int32_t)lo[i] * unit, &bx[i], &by[i]);
    }
    int minx = bx[0], maxx = bx[0], miny = by[0], maxy = by[0];
    for (int i = 1; i < 4; i++) {
      if (bx[i] < minx) minx = bx[i];
      if (bx[i] > maxx) maxx = bx[i];
      if (by[i] < miny) miny = by[i];
      if (by[i] > maxy) maxy = by[i];
    }
    if (maxx < 0 || maxy < 0 || minx >= MAP_W || miny >= MAP_H) return next;
  }

  const int16_t* src = (const int16_t*)pts;
  uint16_t k = 0;
  vm_ring_start[0] = 0;
  for (uint8_t r = 0; r < nrings; r++) {
    for (uint16_t i = 0; i < npts[r]; i++) {
      vm_project_e7(t->org_lat_e7 + (int32_t)src[k * 2 + 0] * unit,
                    t->org_lon_e7 + (int32_t)src[k * 2 + 1] * unit, &vm_px[k], &vm_py[k]);
      k++;
    }
    vm_ring_start[r + 1] = k;
  }
  vm_last_points += k;

  if (VM_CLASS_IS_AREA(cls)) {
    fill_polygon_evenodd(vm_px, vm_py, vm_ring_start, nrings, VM_COLOR[cls]);
  } else {
    const uint8_t w = VM_LINE_WIDTH[cls];
    const uint16_t col = VM_COLOR[cls];
    for (uint8_t r = 0; r < nrings; r++) {
      for (uint16_t i = vm_ring_start[r]; i + 1 < vm_ring_start[r + 1]; i++) {
        draw_map_line(vm_px[i], vm_py[i], vm_px[i + 1], vm_py[i + 1], w, col);
      }
    }
  }
  return next;
}

// 指定 LOD で表示範囲に掛かるタイルが 1 枚でもあるか
static bool vm_has_tiles(int lod, int32_t lat_min_e7, int32_t lat_max_e7, int32_t lon_min_e7, int32_t lon_max_e7) {
  const int32_t tsize = vm_lod_tilesize_e7[lod];
  for (uint16_t i = 0; i < vm_tile_count; i++) {
    const vm_tile* t = &vm_tiles[i];
    if (t->lod != lod) continue;
    if (t->org_lat_e7 > lat_max_e7 || t->org_lat_e7 + tsize < lat_min_e7) continue;
    if (t->org_lon_e7 > lon_max_e7 || t->org_lon_e7 + tsize < lon_min_e7) continue;
    return true;
  }
  return false;
}

void draw_vectormap() {
  vm_last_tiles = 0;
  vm_last_points = 0;
  if (vm_tile_count == 0) {
    backscreen.fillSprite(COLOR_WHITE);
    return;
  }
  const MapFrame& f = map_frame;
  // 表示範囲の緯度経度（回転によらない円で取る）
  const double dlat = VM_VIEW_RADIUS_PX / (double)f.ky;
  const double dlon = VM_VIEW_RADIUS_PX / (double)f.kx;
  const int32_t lat_min_e7 = (int32_t)((f.cLat - dlat) * 1e7);
  const int32_t lat_max_e7 = (int32_t)((f.cLat + dlat) * 1e7);
  const int32_t lon_min_e7 = (int32_t)((f.cLon - dlon) * 1e7);
  const int32_t lon_max_e7 = (int32_t)((f.cLon + dlon) * 1e7);

  // 選んだ LOD にタイルが無ければ、より広く収録している粗い LOD へ落とす
  // （そうしないと収録範囲の外で海の色だけが塗られ、画面全体が真っ青になる。v7 と同じ）
  int lod = vm_select_lod(f.scale);
  while (lod < VM_LOD_COUNT - 1 && !vm_has_tiles(lod, lat_min_e7, lat_max_e7, lon_min_e7, lon_max_e7)) lod++;

  backscreen.fillSprite(vm_lod_sea_background[lod] ? VMCOL_WATER : COLOR_WHITE);
  const int32_t tsize = vm_lod_tilesize_e7[lod];
  for (int ci = 0; ci < VM_CLASS_COUNT; ci++) {
    const uint8_t cls = VM_DRAW_ORDER[ci];
    for (uint16_t i = 0; i < vm_tile_count; i++) {
      const vm_tile* t = &vm_tiles[i];
      if (t->lod != lod) continue;
      if (t->org_lat_e7 > lat_max_e7 || t->org_lat_e7 + tsize < lat_min_e7) continue;
      if (t->org_lon_e7 > lon_max_e7 || t->org_lon_e7 + tsize < lon_min_e7) continue;
      if (ci == 0) vm_last_tiles++;
      const uint8_t* p = vm_blob + t->blob_off;
      const uint8_t* end = p + t->blob_len;
      while (p < end) p = vm_draw_record(p, t, lod, cls);
    }
  }
}
