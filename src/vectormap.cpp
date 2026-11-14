// ============================================================
// File    : vectormap.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : フラッシュ内蔵ベクタ地図の描画。backscreen へ直接描くため、
//           TRACKUP の回転は latLonToXY と同じ座標変換に吸収され、
//           BMP 方式で欠けていた画面四隅まで隙間なく塗れる。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/11
//
// 地図データのライセンス: Map data (c) OpenStreetMap contributors, ODbL 1.0
// ============================================================
#include <Arduino.h>
#include <math.h>

#include "../display_tft.h"
#include "../imu.h" // imu_service_if_due（描画中に IMU を飢えさせない）
#include "../settings.h"
#include "vectormap.h"

// ===== 描画色（RGB565） =====
// https://rgbcolorpicker.com/565
// 陸地: 白。高ズーム側(LOD0/1)は海岸線データを持たず背景の白がそのまま陸地になるため、色を揃えて一貫させる
#define VMCOL_LAND 0xffff
// 水面: 淡い水色
#define VMCOL_WATER 0xb71f
// 公園・大学敷地・緑地: 淡いパステル緑 RGB(208,238,208)
// 飛行軌跡や内蔵ポリゴン（COLOR_GREEN=0x03e0）が埋もれない明度にする
#define VMCOL_PARK 0xd77a
// 大型建物: 暖色系の明るいベージュグレー RGB(222,218,200)
#define VMCOL_BUILDING 0xded9
// 県道・主要街路(secondary/tertiary): 明るいグレー RGB(214,210,214)
#define VMCOL_STREET 0xd69a
// 高速道路: Bright Orange
#define VMCOL_MOTORWAY 0xfeae
// 国道・主要地方道: 黄色
#define VMCOL_TRUNK COLOR_YELLOW
// 鉄道: やや濃いグレー RGB(156,154,156)。街路(VMCOL_STREET)と区別する
#define VMCOL_RAIL 0x9cd3

// 線クラスの太さ [px]（面クラスは 0）
static const uint8_t VM_LINE_WIDTH[VM_CLASS_COUNT] = {
    0, // VM_LANDMASS
    0, // VM_WATER
    0, // VM_PARK
    3, // VM_MOTORWAY
    2, // VM_TRUNK
    1, // VM_RAIL
    0, // VM_BUILDING
    1  // VM_STREET
};
static const uint16_t VM_COLOR[VM_CLASS_COUNT] = {
    VMCOL_LAND,     // VM_LANDMASS
    VMCOL_WATER,    // VM_WATER
    VMCOL_PARK,     // VM_PARK
    VMCOL_MOTORWAY, // VM_MOTORWAY
    VMCOL_TRUNK,    // VM_TRUNK
    VMCOL_RAIL,     // VM_RAIL
    VMCOL_BUILDING, // VM_BUILDING
    VMCOL_STREET    // VM_STREET
};

// クラスの描画順。面を先に塗り（陸地→公園/敷地→水面→建物）、線を後から重ねる（街路→鉄道→国道→高速）。
// 大学キャンパス（VM_PARK）の上に池（VM_WATER）や校舎（VM_BUILDING）が乗る順序にする。
static const uint8_t VM_DRAW_ORDER[VM_CLASS_COUNT] = {
    VM_LANDMASS, VM_PARK, VM_WATER, VM_BUILDING,
    VM_STREET,   VM_RAIL, VM_TRUNK, VM_MOTORWAY};

// ===== 点バッファ =====
// display_tft.cpp の VM_MAX_EDGES と同じ上限。1 レコード分の全リングをここに展開する。
#define VM_MAX_PTS 2048
#define VM_MAX_RINGS 64
static int16_t vm_px[VM_MAX_PTS];
static int16_t vm_py[VM_MAX_PTS];
static uint16_t vm_ring_start[VM_MAX_RINGS + 1];

uint16_t vm_last_tiles = 0;
uint32_t vm_last_points = 0;

// TRACKUP 時に自機から画面隅までの最遠距離 = sqrt(120^2 + 180^2) ≒ 216px。
// この半径を覆う範囲のタイルを集めれば、どの方位を向いても四隅が埋まる。
#define VM_VIEW_RADIUS_PX 216.0f

// ===== 高速座標変換 =====
// 既存 latLonToXY は点ごとに log(tan(...)) を呼ぶため数千点/フレームでは重い。
// メルカトル Y をセンター緯度まわりで 3 次までテイラー展開して置き換える:
//   f(lat)=ln(tan(pi/4+lat/2)),  f'=sec,  f''/f'=tan,  f'''/f'=tan^2+sec^2
//   f(c+d)-f(c) ≒ d*sec(c) * (1 + d*(c2 + d*c3))
// ホスト検証で、全 scale・日本全域の緯度において既存式との差は最大 0.19px。
// 中心の画面 Y を固定する（地図一覧の絵 draw_vectormap_coverage の間だけ。-1 = 自機の位置の決まりどおり）
static int vm_cy_override = -1;

struct vm_proj {
  float cLat, cLon;
  float kx;     // 経度差 1 度あたりの px
  float ky;     // 緯度差 1 度あたりの px（1 次項）
  float c2, c3; // 2 次・3 次補正係数
  float ca, sa; // 回転（up 方向）
  int cy;       // 自機の画面 Y（TRACKUP=180 / NORTHUP=120）
};

static void vm_make_proj(vm_proj *p, double cLat, double cLon, float scale,
                         float up) {
  const double c = cLat * DEG_TO_RAD;
  const double sec = 1.0 / cos(c);
  const double tn = tan(c);
  p->cLat = (float)cLat;
  p->cLon = (float)cLon;
  p->kx = (float)(111.321 * scale);
  p->ky = (float)(6378.22347118 * DEG_TO_RAD * sec * scale);
  p->c2 = (float)(0.5 * tn * DEG_TO_RAD);
  p->c3 = (float)((tn * tn + sec * sec) / 6.0 * DEG_TO_RAD * DEG_TO_RAD);
  const float a = up * DEG_TO_RAD;
  p->ca = cosf(a);
  p->sa = sinf(a);
  p->cy = (vm_cy_override >= 0) ? vm_cy_override
          : is_trackupmode() ? (BACKSCREEN_SIZE * 3 / 4) : (BACKSCREEN_SIZE / 2);
}

// 緯度経度 → スクリーン座標。latLonToXY と同じ結果になる（誤差 < 0.2px）。
static inline void vm_project(const vm_proj *p, float lat, float lon,
                              int16_t *ox, int16_t *oy) {
  const float d = lat - p->cLat;
  const float x = (lon - p->cLon) * p->kx;
  const float y = p->ky * d * (1.0f + d * (p->c2 + d * p->c3));
  float rx = x * p->ca - y * p->sa;
  float ry = x * p->sa + y * p->ca;
  int32_t sx = (int32_t)(BACKSCREEN_SIZE / 2) + (int32_t)rx;
  int32_t sy = (int32_t)p->cy - (int32_t)ry;
  // タイルの大きさは LOD ごとに制限されているので実際には発生しないが、
  // int16 の桁あふれで座標が反転するのを防ぐための保険。
  if (sx < -20000)
    sx = -20000;
  else if (sx > 20000)
    sx = 20000;
  if (sy < -20000)
    sy = -20000;
  else if (sy > 20000)
    sy = 20000;
  *ox = (int16_t)sx;
  *oy = (int16_t)sy;
}

// scale から使用する LOD を選ぶ。vm_lod_min_scale は
// LOD0（最高精細）から降順に並ぶ。
int vm_select_lod(float scale) {
  for (int lod = 0; lod < VM_LOD_COUNT; lod++) {
    if (scale >= vm_lod_min_scale[lod])
      return lod;
  }
  return VM_LOD_COUNT - 1;
}

// 地図の線を 1 本描く。クリップと非アンチエイリアス太線は display_tft.cpp の
// draw_map_line
// に集約してある（トラック描画と共通。理由はそちらのコメント参照）。
static inline void vm_draw_seg(int x0, int y0, int x1, int y1, uint8_t w,
                               uint16_t col) {
  draw_map_line(x0, y0, x1, y1, w, col);
}

// 1 レコードを描画する。blob 内のレコード先頭を指す p
// を受け取り、次のレコード先頭を返す。 cls_filter
// と異なるクラスのレコードはヘッダだけ読んでスキップする（点の変換は行わない）。
static const uint8_t *vm_draw_record(const uint8_t *p, const vm_tile *t,
                                     int lod, const vm_proj *proj,
                                     uint8_t cls_filter) {
  const uint8_t cls = p[0];
  const uint8_t nrings = p[1];
  const int16_t *bb = (const int16_t *)(p + 2);
  const uint16_t *npts = (const uint16_t *)(p + 10);

  uint32_t total = 0;
  for (uint8_t r = 0; r < nrings; r++)
    total += npts[r];
  const uint8_t *pts = (const uint8_t *)(npts + nrings);
  const uint8_t *next = pts + total * 4;

  if (cls != cls_filter)
    return next;
  if (nrings == 0 || nrings > VM_MAX_RINGS)
    return next;
  if (total == 0 || total > VM_MAX_PTS)
    return next;

  // ---- bbox による早期リジェクト ----
  // レコードの bbox 4
  // 隅を投影し、画面矩形と交差しなければ点を変換せずに捨てる。
  const int32_t unit = vm_lod_unit_e7[lod];
  {
    int16_t bx[4], by[4];
    const int16_t la[4] = {bb[0], bb[0], bb[2], bb[2]};
    const int16_t lo[4] = {bb[1], bb[3], bb[1], bb[3]};
    for (int i = 0; i < 4; i++) {
      const float lat = (float)((t->org_lat_e7 + (int32_t)la[i] * unit) * 1e-7);
      const float lon = (float)((t->org_lon_e7 + (int32_t)lo[i] * unit) * 1e-7);
      vm_project(proj, lat, lon, &bx[i], &by[i]);
    }
    int minx = bx[0], maxx = bx[0], miny = by[0], maxy = by[0];
    for (int i = 1; i < 4; i++) {
      if (bx[i] < minx)
        minx = bx[i];
      if (bx[i] > maxx)
        maxx = bx[i];
      if (by[i] < miny)
        miny = by[i];
      if (by[i] > maxy)
        maxy = by[i];
    }
    if (maxx < 0 || maxy < 0 || minx >= BACKSCREEN_SIZE ||
        miny >= BACKSCREEN_SIZE)
      return next;
  }

  // ---- 点をスクリーン座標へ展開 ----
  const int16_t *src = (const int16_t *)pts;
  uint16_t k = 0;
  vm_ring_start[0] = 0;
  for (uint8_t r = 0; r < nrings; r++) {
    for (uint16_t i = 0; i < npts[r]; i++) {
      const float lat =
          (float)((t->org_lat_e7 + (int32_t)src[k * 2 + 0] * unit) * 1e-7);
      const float lon =
          (float)((t->org_lon_e7 + (int32_t)src[k * 2 + 1] * unit) * 1e-7);
      vm_project(proj, lat, lon, &vm_px[k], &vm_py[k]);
      k++;
    }
    vm_ring_start[r + 1] = k;
  }
  vm_last_points += k;

  // ---- 描画 ----
  if (VM_CLASS_IS_AREA(cls)) {
    fill_polygon_evenodd(vm_px, vm_py, vm_ring_start, nrings, VM_COLOR[cls]);
  } else {
    const uint8_t w = VM_LINE_WIDTH[cls];
    const uint16_t col = VM_COLOR[cls];
    for (uint8_t r = 0; r < nrings; r++) {
      for (uint16_t i = vm_ring_start[r]; i + 1 < vm_ring_start[r + 1]; i++) {
        vm_draw_seg(vm_px[i], vm_py[i], vm_px[i + 1], vm_py[i + 1], w, col);
      }
    }
  }
  return next;
}

// 指定 LOD の収録範囲（地図データの vm_cover_e7）の 1 つに、表示範囲が丸ごと入っているか（2026-11-14〜。BUG-115）。
// ★ 「表示範囲にタイルが 1 枚でもあるか」で選んでいたころは、飛行エリアの端が画面に入ると、範囲の外（データの無い所）の
//   陸地が海の色で塗られていた（細かい LOD は飛行エリアの周りだけを持ち、背景は海の色で塗ってから陸を重ねるため）。
//   範囲の中なら、タイルが無い所は本当に水面（陸の無い所）なので、海の色でよい。
static bool vm_view_covered(int lod, int32_t lat_min_e7, int32_t lat_max_e7,
                            int32_t lon_min_e7, int32_t lon_max_e7) {
  const uint16_t first = vm_lod_cover_first[lod];
  for (uint16_t k = 0; k < vm_lod_cover_n[lod]; k++) {
    const int32_t *r = vm_cover_e7[first + k];
    if (lat_min_e7 >= r[0] && lon_min_e7 >= r[1] && lat_max_e7 <= r[2] && lon_max_e7 <= r[3])
      return true;
  }
  return false;
}

// 背景の塗りつぶしもこの関数が担う。呼び出し側で別途クリアすると、
// 直後にここで塗り直すことになり 240x240 の書き込みが二重に走る。
void draw_vectormap(double center_lat, double center_lon, float scale,
                    float up) {
  vm_last_tiles = 0;
  vm_last_points = 0;
  if (vm_tile_count == 0) {
    backscreen.fillSprite(COLOR_WHITE); // データが無くても前フレームは消す
    return;
  }

  vm_proj proj;
  vm_make_proj(&proj, center_lat, center_lon, scale, up);

  // ---- 表示範囲の緯度経度 bbox ----
  // 半径 216px を度に換算。回転に依存しない円で取るので up
  // の値によらず同じ範囲になる。
  const double dlat = VM_VIEW_RADIUS_PX / (double)proj.ky;
  const double dlon = VM_VIEW_RADIUS_PX / (double)proj.kx;
  const int32_t lat_min_e7 = (int32_t)((center_lat - dlat) * 1e7);
  const int32_t lat_max_e7 = (int32_t)((center_lat + dlat) * 1e7);
  const int32_t lon_min_e7 = (int32_t)((center_lon - dlon) * 1e7);
  const int32_t lon_max_e7 = (int32_t)((center_lon + dlon) * 1e7);

  // ---- LOD の選択 ----
  // LOD ごとに収録範囲が違う（高ズームほど狭い）ため、表示範囲が選んだ LOD の収録範囲に
  // 丸ごと入っていなければ、より広範囲を収録している粗い LOD へ落とす（vm_view_covered）。
  // これをやらないと、収録範囲の外に掛かった所が海色の背景のまま陸地が描かれない
  // （範囲の外へ出れば画面全体が真っ青になる）。いちばん粗い LOD は範囲の外でもそれを使う。
  int lod = vm_select_lod(scale);
  while (lod < VM_LOD_COUNT - 1 &&
         !vm_view_covered(lod, lat_min_e7, lat_max_e7, lon_min_e7, lon_max_e7)) {
    lod++;
  }

  // 背景を塗る。陸地ポリゴンを持つ LOD は海色で塗ってから陸を重ね、
  // 持たない LOD は白（＝陸地）にして水面だけを塗る。
  backscreen.fillSprite(vm_lod_sea_background[lod] ? VMCOL_WATER : COLOR_WHITE);

  const int32_t tsize = vm_lod_tilesize_e7[lod];

  // ---- クラス順に 6 パス ----
  // 面を先に塗ってから線を重ねる必要があるため、タイルを跨いでクラス順に描く。
  // クラスが一致しないレコードはヘッダを読んでポインタを進めるだけなので、
  // 再走査のコストは点数に依存せず無視できる。
  for (int ci = 0; ci < VM_CLASS_COUNT; ci++) {
    const uint8_t cls = VM_DRAW_ORDER[ci];
    for (uint16_t i = 0; i < vm_tile_count; i++) {
      const vm_tile *t = &vm_tiles[i];
      if (t->lod != lod)
        continue;
      if (t->org_lat_e7 > lat_max_e7 || t->org_lat_e7 + tsize < lat_min_e7)
        continue;
      if (t->org_lon_e7 > lon_max_e7 || t->org_lon_e7 + tsize < lon_min_e7)
        continue;
      if (ci == 0)
        vm_last_tiles++;

      // ★ タイル 1 枚ごとに IMU のサンプルを引き取る。
      //   地図の再描画は実測で 60ms 以上 Core0
      //   を占有し、その間レポートが溜まると
      //   **次に読んだジャイロが化ける**（2026-09-23 に実機で確定。imu.h
      //   参照）。 デモモードの 2Hz
      //   再描画に同期してバリオが暴れていたのがこれ。
      //   引き取るだけで描画は止めない（imu_service_if_due
      //   は周期内なら即戻る）。
      imu_service_if_due();

      const uint8_t *p = vm_blob + t->blob_off;
      const uint8_t *end = p + t->blob_len;
      while (p < end) {
        p = vm_draw_record(p, t, lod, &proj, cls);
        // ★ レコード 1 本ごとにも呼ぶ（2026-11-11〜）。タイル単位だけだと、道路・建物の多いタイル 1 枚を
        //   描く間は 1 回も呼ばれず、11/11 の生ログでは GNSS の更新（2Hz）ごとに 70〜90ms 止まっていた
        //   （1.03 で公園・建物・街路を足してから倍になった）。周期内なら時刻を見て戻るだけなので軽い。
        //   タイマー割り込みで読んでいるとき（SCH16T_READ_IN_IRQ）は、溜まった分を推定へ渡すだけ。
        imu_service_if_due();
      }
    }
  }
}

// ---- 地図一覧の絵（収録範囲の枠。vectormap.h の説明）----
void draw_vectormap_coverage(bool zoom) {
  const int W = BACKSCREEN_SIZE;
  // 表示の中心と縮尺 [px/km]。zoom は最大ズームの範囲（LOD 0）の全体に余白を足して収める。
  double lat0 = 30.5, lat1 = 45.8, lon0 = 128.5, lon1 = 146.0;   // 日本全体（本州・四国・九州・北海道）
  if (zoom && vm_lod_cover_n[0] > 0) {
    lat0 = lon0 = 1e9; lat1 = lon1 = -1e9;
    for (uint16_t k = 0; k < vm_lod_cover_n[0]; k++) {
      const int32_t *r = vm_cover_e7[vm_lod_cover_first[0] + k];
      lat0 = fmin(lat0, r[0] * 1e-7); lon0 = fmin(lon0, r[1] * 1e-7);
      lat1 = fmax(lat1, r[2] * 1e-7); lon1 = fmax(lon1, r[3] * 1e-7);
    }
    const double mlat = (lat1 - lat0) * 0.15 + 0.2, mlon = (lon1 - lon0) * 0.15 + 0.2;
    lat0 -= mlat; lat1 += mlat; lon0 -= mlon; lon1 += mlon;
  }
  const double clat = (lat0 + lat1) / 2.0, clon = (lon0 + lon1) / 2.0;
  // vm_make_proj と同じ換算: 経度 1 度 = 111.321*scale px、緯度 1 度 ≒ 111.32/cos(緯度)*scale px
  const double sec = 1.0 / cos(clat * DEG_TO_RAD);
  const double s_lon = (W - 10) / ((lon1 - lon0) * 111.321);
  const double s_lat = (W - 10) / ((lat1 - lat0) * 111.321 * sec);
  const float scale = (float)fmin(s_lon, s_lat);

  vm_cy_override = W / 2;
  draw_vectormap(clat, clon, scale, 0.0f);   // いちばん粗い地図（縮尺で LOD が決まる）
  vm_proj proj;
  vm_make_proj(&proj, clat, clon, scale, 0.0f);
  vm_cy_override = -1;

  // 外側の範囲から順に重ねる（全国の範囲＝最後の LOD は描かない）。赤は 2px で上に。
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  for (int lod = VM_LOD_COUNT - 2; lod >= 0; lod--) {
    for (uint16_t k = 0; k < vm_lod_cover_n[lod]; k++) {
      const uint16_t idx = vm_lod_cover_first[lod] + k;
      const int32_t *r = vm_cover_e7[idx];
      const bool site = vm_cover_name[idx][0] != '\0';
      const uint16_t col = (lod == 0) ? COLOR_RED : site ? COLOR_ORANGE : COLOR_BLUE;
      int16_t x0, y0, x1, y1;
      vm_project(&proj, r[2] * 1e-7f, r[1] * 1e-7f, &x0, &y0);   // 北西の角
      vm_project(&proj, r[0] * 1e-7f, r[3] * 1e-7f, &x1, &y1);   // 南東の角
      int w = x1 - x0, h = y1 - y0;
      if (w < 3) { x0 -= (3 - w) / 2; w = 3; }   // 小さすぎて見えない枠は 3px にする
      if (h < 3) { y0 -= (3 - h) / 2; h = 3; }
      backscreen.drawRect(x0, y0, w, h, col);
      if (lod == 0) {
        backscreen.drawRect(x0 - 1, y0 - 1, w + 2, h + 2, col);
        if (zoom) {   // 番号（下の凡例と同じ 1 から）。枠の左上の外側、上に置けないときは下
          backscreen.setTextColor(COLOR_RED);
          backscreen.setCursor(x0, (y0 >= 10) ? y0 - 10 : y0 + h + 2);
          backscreen.print(k + 1);
        }
      }
    }
  }
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
}
