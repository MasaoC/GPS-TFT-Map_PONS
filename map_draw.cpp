// ============================================================
// File    : map_draw.cpp
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 地図画面の地図の部分（ベクタ地図・航跡・誘導線・パイロン・自機・修正矢印）の描画。
//           すべて 240x230 のスプライト（backscreen）に描いてから 1 回で TFT へ送る（v7 と同じ方式）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
// ★ ここは描くだけ。誘導の判断（ズレ角・修正の積算・30 度の範囲の基準）は GPS_TFT_map.ino の
//   nav_tick() が持ち、MapView で渡される（CLAUDE.md「警報を描画関数の中に書かない」と同じ考え）。
#include "map_draw.h"
#include "display_tft.h"
#include "navdata.h"
#include "gps.h"
#include "mysd.h"
#include "src/vectormap.h"

TFT_eSprite backscreen = TFT_eSprite(&tft);
MapFrame map_frame;
uint32_t map_frame_ms_last = 0, map_frame_ms_max = 0;
static bool backscreen_ok = false;

static const uint8_t* s_map_font = nullptr;

void map_setup(const uint8_t* font) {
  s_map_font = font;
  backscreen.setColorDepth(16);
  backscreen_ok = backscreen.createSprite(MAP_W, MAP_H) != nullptr;
  if (!backscreen_ok) {
    Serial.println("ERR backscreen createSprite");
    return;
  }
  backscreen.loadFont(font);   // スプライトの文字は常にこの 1 書体（tft 側とは別に持つ）
  backscreen.setTextWrap(false);
}

int map_self_cy() {
  return is_trackupmode() ? (MAP_H * 3 / 4) : (MAP_H / 2);
}

void map_begin_frame(double cLat, double cLon, float scale, float up) {
  MapFrame& f = map_frame;
  const double c = cLat * DEG_TO_RAD;
  const double sec = 1.0 / cos(c);
  const double tn = tan(c);
  f.cLat = cLat;
  f.cLon = cLon;
  f.cLat_e7 = (int32_t)lround(cLat * 1e7);
  f.cLon_e7 = (int32_t)lround(cLon * 1e7);
  f.scale = scale;
  f.up = up;
  f.kx = (float)(111.321 * scale);
  f.ky = (float)(6378.22347118 * DEG_TO_RAD * sec * scale);
  f.c2 = (float)(0.5 * tn * DEG_TO_RAD);
  f.c3 = (float)((tn * tn + sec * sec) / 6.0 * DEG_TO_RAD * DEG_TO_RAD);
  const float a = up * DEG_TO_RAD;
  f.ca = cosf(a);
  f.sa = sinf(a);
  f.cx = MAP_W / 2;
  f.cy = map_self_cy();
}

void map_project(double lat, double lon, int* x, int* y) {
  int16_t sx, sy;
  map_project_d((float)(lat - map_frame.cLat), (float)(lon - map_frame.cLon), &sx, &sy);
  *x = sx;
  *y = sy;
}

// 地上の 1km が何 px か。メルカトルなので東西・南北とも scale / cos(緯度)。
// ★ cos を掛けない（scale そのもの）と 35 度で 18% 小さくなる（0.501 で直した km の円と同じ間違い）。
static float px_per_km() {
  return map_frame.scale / cosf((float)(map_frame.cLat * DEG_TO_RAD));
}


// ============================================================
// 線・多角形（v7 と同じ実装を地図の大きさに合わせたもの）
// ============================================================
#define CLIP_MARGIN 8

static inline int clip_outcode(double x, double y) {
  int c = 0;
  if (x < -CLIP_MARGIN) c |= 1; else if (x > MAP_W - 1 + CLIP_MARGIN) c |= 2;
  if (y < -CLIP_MARGIN) c |= 4; else if (y > MAP_H - 1 + CLIP_MARGIN) c |= 8;
  return c;
}

// 線分を地図の矩形にクリップする。完全に外なら false。
bool clip_line_to_map(int* px0, int* py0, int* px1, int* py1) {
  const double xlo = -CLIP_MARGIN, xhi = MAP_W - 1 + CLIP_MARGIN;
  const double ylo = -CLIP_MARGIN, yhi = MAP_H - 1 + CLIP_MARGIN;
  double x0 = *px0, y0 = *py0, x1 = *px1, y1 = *py1;
  int c0 = clip_outcode(x0, y0);
  int c1 = clip_outcode(x1, y1);
  for (int guard = 0; guard < 8; guard++) {
    if (!(c0 | c1)) break;        // 両端とも内側
    if (c0 & c1) return false;    // 同じ側に両端がある = 交差しない
    const int c = c0 ? c0 : c1;
    double x = 0, y = 0;
    if (c & 8) { x = x0 + (x1 - x0) * (yhi - y0) / (y1 - y0); y = yhi; }
    else if (c & 4) { x = x0 + (x1 - x0) * (ylo - y0) / (y1 - y0); y = ylo; }
    else if (c & 2) { y = y0 + (y1 - y0) * (xhi - x0) / (x1 - x0); x = xhi; }
    else { y = y0 + (y1 - y0) * (xlo - x0) / (x1 - x0); x = xlo; }
    if (c == c0) { x0 = x; y0 = y; c0 = clip_outcode(x0, y0); }
    else { x1 = x; y1 = y; c1 = clip_outcode(x1, y1); }
  }
  if (c0 | c1) return false;
  *px0 = (int)lround(x0); *py0 = (int)lround(y0);
  *px1 = (int)lround(x1); *py1 = (int)lround(y1);
  return true;
}

// 本数の多い線（地図・航跡）。アンチエイリアスの太線は 1 本ごとに重いので、平行な線を重ねて太らせる（v7 と同じ）。
void draw_map_line(int x0, int y0, int x1, int y1, int w, uint16_t col) {
  if (!clip_line_to_map(&x0, &y0, &x1, &y1)) return;
  backscreen.drawLine(x0, y0, x1, y1, col);
  if (w <= 1) return;
  const bool steep = abs(y1 - y0) > abs(x1 - x0);
  for (int o = 1; o <= (w - 1) / 2; o++) {
    if (steep) {
      backscreen.drawLine(x0 - o, y0, x1 - o, y1, col);
      backscreen.drawLine(x0 + o, y0, x1 + o, y1, col);
    } else {
      backscreen.drawLine(x0, y0 - o, x1, y1 - o, col);
      backscreen.drawLine(x0, y0 + o, x1, y1 + o, col);
    }
  }
  if (w % 2 == 0) {  // 偶数幅は片側にもう 1 本足す
    if (steep) backscreen.drawLine(x0 + w / 2, y0, x1 + w / 2, y1, col);
    else backscreen.drawLine(x0, y0 + w / 2, x1, y1 + w / 2, col);
  }
}

// 本数の少ない目立つ線（誘導線）。アンチエイリアス付き。
static void draw_clipped_wideline(int x0, int y0, int x1, int y1, int w, uint16_t col) {
  if (!clip_line_to_map(&x0, &y0, &x1, &y1)) return;
  backscreen.drawWideLine(x0, y0, x1, y1, w, col);
}

#define VM_MAX_EDGES 2048  // 同時に扱える辺の数（12 バイト/辺 = 24KB）
struct scan_edge {
  float x;        // 現在のスキャンラインにおける交点 x
  float dxdy;     // 1 行進むごとの x の増分
  int16_t y_bot;  // この y に達したら除外する（半開区間）
  int16_t next;   // 連結リストの次（-1 = 終端）
};
static scan_edge s_edges[VM_MAX_EDGES];
static int16_t s_et[MAP_H];                   // Edge Table: 各行から始まる辺のリスト先頭
static int16_t s_aet_buf[VM_MAX_EDGES / 4];   // Active Edge Table

// 複数リングをまとめて even-odd で塗る（湖本体と島を 1 回で塗ると島が穴として抜ける）。
// 辺数が VM_MAX_EDGES を超える場合は中途半端な塗りを避けるため何も描かず false。
bool fill_polygon_evenodd(const int16_t* xs, const int16_t* ys,
                          const uint16_t* ring_start, uint8_t nrings, uint16_t color) {
  const int H = MAP_H, W = MAP_W;
  if (nrings == 0) return true;
  if (ring_start[nrings] > VM_MAX_EDGES) return false;
  for (int i = 0; i < H; i++) s_et[i] = -1;
  int nedges = 0, y_min = H, y_max = 0;
  for (uint8_t r = 0; r < nrings; r++) {
    const uint16_t beg = ring_start[r];
    const int n = ring_start[r + 1] - beg;
    if (n < 3) continue;
    for (int i = 0; i < n; i++) {
      const int j = (i + 1 == n) ? 0 : (i + 1);
      int x0 = xs[beg + i], y0 = ys[beg + i];
      int x1 = xs[beg + j], y1 = ys[beg + j];
      if (y0 == y1) continue;
      if (y0 > y1) { int t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
      if (y1 <= 0 || y0 >= H) continue;
      const float dxdy = (float)(x1 - x0) / (float)(y1 - y0);
      float x_top = (float)x0;
      int y_top = y0;
      if (y_top < 0) { x_top += dxdy * (float)(0 - y_top); y_top = 0; }
      const int y_bot = (y1 > H) ? H : y1;
      if (y_top >= y_bot) continue;
      if (nedges >= VM_MAX_EDGES) return false;
      s_edges[nedges].x = x_top;
      s_edges[nedges].dxdy = dxdy;
      s_edges[nedges].y_bot = (int16_t)y_bot;
      s_edges[nedges].next = s_et[y_top];
      s_et[y_top] = (int16_t)nedges;
      nedges++;
      if (y_top < y_min) y_min = y_top;
      if (y_bot > y_max) y_max = y_bot;
    }
  }
  if (nedges == 0) return true;
  int aet_n = 0;
  for (int y = y_min; y < y_max; y++) {
    for (int e = s_et[y]; e != -1; e = s_edges[e].next) {
      if (aet_n < (int)(sizeof(s_aet_buf) / sizeof(s_aet_buf[0]))) s_aet_buf[aet_n++] = (int16_t)e;
    }
    int w = 0;
    for (int i = 0; i < aet_n; i++) {
      if (s_edges[s_aet_buf[i]].y_bot > y) s_aet_buf[w++] = s_aet_buf[i];
    }
    aet_n = w;
    for (int i = 1; i < aet_n; i++) {   // x で挿入ソート
      const int16_t key = s_aet_buf[i];
      const float kx = s_edges[key].x;
      int j = i - 1;
      while (j >= 0 && s_edges[s_aet_buf[j]].x > kx) { s_aet_buf[j + 1] = s_aet_buf[j]; j--; }
      s_aet_buf[j + 1] = key;
    }
    for (int i = 0; i + 1 < aet_n; i += 2) {
      int xa = (int)ceilf(s_edges[s_aet_buf[i]].x);
      int xb = (int)ceilf(s_edges[s_aet_buf[i + 1]].x) - 1;
      if (xb < 0 || xa >= W || xa > xb) continue;
      if (xa < 0) xa = 0;
      if (xb >= W) xb = W - 1;
      backscreen.drawFastHLine(xa, y, xb - xa + 1, color);
    }
    for (int i = 0; i < aet_n; i++) s_edges[s_aet_buf[i]].x += s_edges[s_aet_buf[i]].dxdy;
  }
  return true;
}


// ============================================================
// 地図の層
// ============================================================

// 内蔵・SD の折れ線地図（cords は [i][0]=経度, [i][1]=緯度）。
static void draw_polyline_map(const mapdata* mp, uint16_t color) {
  if (mp->size < 2) return;
  int px, py;
  map_project(mp->cords[0][1], mp->cords[0][0], &px, &py);
  for (int i = 1; i < mp->size; i++) {
    int x, y;
    map_project(mp->cords[i][1], mp->cords[i][0], &x, &y);
    draw_map_line(px, py, x, y, 1, color);
    px = x;
    py = y;
  }
}

// mapdata.csv の線の色（名前の 1 文字目。README の「地図作成」）
static uint16_t mapdata_color(const char* name) {
  switch (name[0]) {
    case 'r': return COLOR_RED;
    case 'o': return COLOR_ORANGE;
    case 'g': return COLOR_BRIGHTGRAY;
    case 'm': return COLOR_MAGENTA;
    case 'c': return COLOR_CYAN;
    case 'b': return COLOR_BLUE;
    default:  return COLOR_GREEN;
  }
}

// 内蔵の折れ線地図（大阪大学の構内）と、SD の mapdata.csv。
// 琵琶湖・浦安・日本の海岸線の内蔵の折れ線は 0.503 で削除した（ベクタ地図の湖岸・海岸と少しずれて二重に見えるため。v7 と同じ）。
static void draw_line_maps(const MapView& v) {
  if (v.scale <= SCALE_SMALL_GMAP) return;
  if (check_within_latlon(0.6, 0.6, v.lat, OSAKA_LAT, v.lon, OSAKA_LON)) {
    draw_polyline_map(&map_handaioutside, COLOR_CYAN);
    draw_polyline_map(&map_handaihighway, COLOR_MAGENTA);
    draw_polyline_map(&map_handaihighway2, COLOR_MAGENTA);
    draw_polyline_map(&map_handaiinside1, COLOR_BRIGHTGRAY);
    draw_polyline_map(&map_handaiinside2, COLOR_BRIGHTGRAY);
    draw_polyline_map(&map_handaiinside3, COLOR_BRIGHTGRAY);
    draw_polyline_map(&map_handaiinside4, COLOR_BRIGHTGRAY);
    draw_polyline_map(&map_handaiinside5, COLOR_BRIGHTGRAY);
    draw_polyline_map(&map_handairailway, COLOR_ORANGE);
    draw_polyline_map(&map_handaicafe, COLOR_GREEN);
  }
  for (int i = 0; i < mapdata_count; i++) {
    if (extramaps[i].size <= 1) continue;
    const double lon1 = extramaps[i].cords[0][0];
    const double lat1 = extramaps[i].cords[0][1];
    if (check_within_latlon(1, 1, lat1, v.lat, lon1, v.lon)) {
      draw_polyline_map(&extramaps[i], mapdata_color(extramaps[i].name));
    }
  }
}

// 航跡（緑）。各点は 1 回だけ変換し、前の点を持ち回る（以前は 1 点を 2 回ずつ変換していた）。
static void draw_track_layer() {
  const int n = latlon_manager.getCount();
  if (n <= 1) return;
  int px = 0, py = 0;
  bool have_prev = false;
  for (int i = 0; i < n; i++) {
    const Coordinate c = latlon_manager.getData(i);
    if (c.latitude == 0 && c.longitude == 0) break;
    int x, y;
    map_project(c.latitude, c.longitude, &x, &y);
    if (have_prev && (x != px || y != py)) draw_map_line(px, py, x, y, 2, COLOR_GREEN);
    px = x;
    py = y;
    have_prev = true;
  }
}

// パイロンのアイコン（オレンジの三角コーン・黒枠。v7 仕様）
static void draw_pylon_icon(int x, int y) {
  if (x < -10 || x >= MAP_W + 10 || y < -10 || y >= MAP_H + 10) return;
  backscreen.fillTriangle(x - 3, y, x + 3, y, x, y - 9, COLOR_ORANGE);
  backscreen.drawTriangle(x - 3, y, x + 3, y, x, y - 9, COLOR_BLACK);
}

// プラットホーム（PLA）のロゴアイコン（青ベース＋黄色スロープ＋赤デッキ。v7 仕様）
static void draw_plathome_icon(int x, int y) {
  if (x < -10 || x >= MAP_W + 10 || y < -10 || y >= MAP_H + 10) return;
  backscreen.fillRect(x - 3, y - 2, 6, 8, COLOR_BLUE);
  backscreen.fillTriangle(x - 2, y - 1, x + 1, y - 1, x, y + 3, COLOR_YELLOW);
  backscreen.drawLine(x - 2, y - 2, x + 2, y - 2, COLOR_RED);
  backscreen.drawLine(x - 2, y - 3, x + 2, y - 3, COLOR_RED);
}

// 折り返しの目安の円（PLATHOME 中心・半径 PLA_TURN_CIRCLE_KM。v7 と同じ）。72 角形。
static void draw_turn_circle() {
  const int N = 72;
  const double lat0 = course_pla[0][0], lon0 = course_pla[0][1];
  const double dlat = PLA_TURN_CIRCLE_KM / KM_PER_DEG_LAT;
  const double dlon = PLA_TURN_CIRCLE_KM / KM_PER_DEG_LON(lat0);
  int px = 0, py = 0;
  for (int k = 0; k <= N; k++) {
    const double th = 2.0 * PI * k / N;
    int x, y;
    map_project(lat0 + dlat * cos(th), lon0 + dlon * sin(th), &x, &y);
    if (k > 0) draw_map_line(px, py, x, y, 1, COLOR_ORANGE);
    px = x;
    py = y;
  }
}

// 始点 (lat1,lon1) から方位 bearing_rad（北基準・時計回り・ラジアン）へ distance km 進んだ点
static void calculatePointFromBearing(double lat1, double lon1, double bearing_rad, double distance, double& lat2, double& lon2) {
  const double R = 6371.0;
  lat1 = deg2rad(lat1); lon1 = deg2rad(lon1);
  const double d = distance / R;
  lat2 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bearing_rad));
  lon2 = lon1 + atan2(sin(bearing_rad) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat2));
  lat2 = rad2deg(lat2);
  lon2 = rad2deg(lon2);
}

// 琵琶湖のコース基準線。プラットホームから北・南パイロンと竹島への線、センターライン、折り返しの円。
// 座標は navdata.cpp の course_*（SD の override_pilon_coordinate.csv で上書きできる）。
static void draw_course_layer(const MapView& v) {
  if (v.scale <= SCALE_SMALL_GMAP) return;
  if (!check_within_latlon(0.6, 0.6, v.lat, course_pla[0][0], v.lon, course_pla[0][1])) return;
  int pla_x, pla_y, n_x, n_y, s_x, s_y, t_x, t_y;
  map_project(course_pla[0][0], course_pla[0][1], &pla_x, &pla_y);
  map_project(course_npilon[0][0], course_npilon[0][1], &n_x, &n_y);
  map_project(course_spilon[0][0], course_spilon[0][1], &s_x, &s_y);
  map_project(course_takeshima[0][0], course_takeshima[0][1], &t_x, &t_y);
  draw_map_line(pla_x, pla_y, n_x, n_y, 3, COLOR_GREEN);   // パイロンへの基準線は太く（v7 の 0.921）
  draw_map_line(pla_x, pla_y, s_x, s_y, 3, COLOR_GREEN);
  draw_map_line(pla_x, pla_y, t_x, t_y, 1, COLOR_GREEN);   // 竹島は 1px のまま

  // 2本のパイロン基準線の「角度的な中間」を通るセンターライン（オレンジ）。
  // 方位はパイロン座標から実行時に計算するので、パイロンが移動しても自動で追従する（v7 と同じ）。
  // 1km 円〜10.975km 円の間だけ描き、PLA 付近のスタート台・1km パイロンのアイコンを隠さない。
  const double centerline_bearing = pla_centerline_bearing_rad();
  double in_lat, in_lon, out_lat, out_lon;
  calculatePointFromBearing(course_pla[0][0], course_pla[0][1], centerline_bearing, 1.0, in_lat, in_lon);
  calculatePointFromBearing(course_pla[0][0], course_pla[0][1], centerline_bearing, 10.975, out_lat, out_lon);
  int cl_in_x, cl_in_y, cl_out_x, cl_out_y;
  map_project(in_lat, in_lon, &cl_in_x, &cl_in_y);
  map_project(out_lat, out_lon, &cl_out_x, &cl_out_y);
  draw_map_line(cl_in_x, cl_in_y, cl_out_x, cl_out_y, 2, COLOR_ORANGE);

  if (v.scale >= 3.0f) draw_turn_circle();   // 縮尺が小さすぎると点になるので zoom9 以上だけ
}

// 琵琶湖コースのアイコン（PLA スタート台ロゴ、北・南・1km パイロンコーンマーク）。
// マゼンタの誘導ラインより後に呼び、線でアイコンが隠れないようにする（v7 仕様）。
static void draw_course_marks(const MapView& v) {
  if (v.scale <= SCALE_SMALL_GMAP) return;
  if (!check_within_latlon(0.6, 0.6, v.lat, course_pla[0][0], v.lon, course_pla[0][1])) return;
  int pla_x, pla_y, n_x, n_y, s_x, s_y, p1k_x, p1k_y;
  map_project(course_pla[0][0], course_pla[0][1], &pla_x, &pla_y);
  map_project(course_npilon[0][0], course_npilon[0][1], &n_x, &n_y);
  map_project(course_spilon[0][0], course_spilon[0][1], &s_x, &s_y);
  map_project(course_1km[0][0], course_1km[0][1], &p1k_x, &p1k_y);

  draw_pylon_icon(n_x, n_y);
  draw_pylon_icon(s_x, s_y);
  draw_pylon_icon(p1k_x, p1k_y);
  draw_plathome_icon(pla_x, pla_y);
}

// A→B を B の先へ distance km 延ばした点 C（B で見た A→B の方位で）
static void calculatePointC(double lat1, double lon1, double lat2, double lon2, double distance, double& lat3, double& lon3) {
  const double R = 6371.0;
  lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); lat2 = deg2rad(lat2); lon2 = deg2rad(lon2);
  const double d = distance / R;
  // A と B を入れ替えて PI を足すのは、離れているとき A での方位と B での方位が違うため（B での方位が要る）
  const double bearing = calculateTrueCourseRad(lat2, lon2, lat1, lon1) + PI;
  lat3 = asin(sin(lat2) * cos(d) + cos(lat2) * sin(d) * cos(bearing));
  lon3 = lon2 + atan2(sin(bearing) * sin(d) * cos(lat2), cos(d) - sin(lat2) * sin(lat3));
  lat3 = rad2deg(lat3);
  lon3 = rad2deg(lon3);
}

// A から B へ distance km 進んだ点 D。A==B のときは A を返す
// （以前は出力を書かずに戻っていて、呼び出し側の未初期化の値で誘導線が明後日を指しえた。v7 の 0.965）。
static void calculatePointD(double lat1, double lon1, double lat2, double lon2, double distance, double& lat3, double& lon3) {
  if (lat1 == lat2 && lon1 == lon2) { lat3 = lat1; lon3 = lon1; return; }
  const double R = 6371.0;
  lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); lat2 = deg2rad(lat2); lon2 = deg2rad(lon2);
  const double d = distance / R;
  const double bearing = calculateTrueCourseRad(lat1, lon1, lat2, lon2);
  lat3 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(bearing));
  lon3 = lon1 + atan2(sin(bearing) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat3));
  lat3 = rad2deg(lat3);
  lon3 = rad2deg(lon3);
}

// 目的地への誘導線（マゼンタ）。測位していないときは描かない（自機の位置が分からないので。v7 の 0.911）。
static void draw_destination_layer(const MapView& v) {
  if (!v.live || !v.has_destination) return;
  const MapFrame& f = map_frame;
  // 画面の外へ出れば十分な長さ（遠い目的地をそのまま変換すると座標が大きくなりすぎる）
  const double reach_km = 400.0 / px_per_km();
  int gx, gy;
  map_project(v.dest_lat, v.dest_lon, &gx, &gy);
  const bool goal_inside = gx >= 0 && gx < MAP_W && gy >= 0 && gy < MAP_H;
  if (!goal_inside) {
    double la, lo;
    calculatePointD(v.lat, v.lon, v.dest_lat, v.dest_lon, reach_km, la, lo);
    map_project(la, lo, &gx, &gy);
  }
  if (v.fly_into) {
    draw_clipped_wideline(f.cx, f.cy, gx, gy, 4, COLOR_MAGENTA);
  } else {
    // 離れる向き: 目的地への細い線と、自機から反対へ伸びる太い線
    draw_clipped_wideline(f.cx, f.cy, gx, gy, 1, COLOR_MAGENTA);
    double la, lo;
    calculatePointC(v.dest_lat, v.dest_lon, v.lat, v.lon, reach_km, la, lo);
    int ax, ay;
    map_project(la, lo, &ax, &ay);
    draw_clipped_wideline(f.cx, f.cy, ax, ay, 4, COLOR_MAGENTA);
  }
}

// 自機を中心にした距離の円（2 つ）。0.501 で直した半径（cos(緯度) で割る）。
static bool draw_circle_km(float km) {
  const MapFrame& f = map_frame;
  const int radius = (int)(px_per_km() * km);
  const int ypos = f.cy - radius;
  if (ypos <= 2) return false;
  backscreen.drawCircle(f.cx, f.cy, radius, COLOR_PINK);
  char buf[12];
  if (km > 0.999f) snprintf(buf, sizeof(buf), "%dkm", (int)km);
  else snprintf(buf, sizeof(buf), "%.1fkm", km);
  backscreen.setTextColor(COLOR_BLACK);
  backscreen.setCursor(f.cx + 2, ypos + 1);
  backscreen.print(buf);
  return true;
}
static void draw_km_circles() {
  static const float kms[] = { 400, 200, 100, 50, 20, 10, 5, 2, 1, 0.4f, 0.2f };
  int drawn = 0;
  for (unsigned i = 0; i < sizeof(kms) / sizeof(kms[0]) && drawn < 2; i++) {
    if (draw_circle_km(kms[i])) drawn++;
  }
}

// 方位の文字（真方位。画面の上の方位 up に合わせて回す。N は赤、E/S/W は黒。v7 仕様）
static void draw_compass_layer(float up) {
  const MapFrame& f = map_frame;
  const int dist = is_northupmode() ? (MAP_H / 2 - 12) : 73;
  static const char* const names[4] = { "N", "E", "S", "W" };
  for (int i = 0; i < 4; i++) {
    const float a = (i * 90.0f - up) * DEG_TO_RAD;
    const int x = f.cx + (int)(sinf(a) * dist);
    const int y = f.cy - (int)(cosf(a) * dist);
    if (x < 4 || x > MAP_W - 14 || y < 2 || y > MAP_H - 14) continue;
    backscreen.setTextColor(i == 0 ? COLOR_RED : COLOR_BLACK);
    backscreen.setCursor(x - 4, y - 7);
    backscreen.print(names[i]);
  }
}

#define TRIANGLE_HWIDTH 6
#define TRIANGLE_SIZE 20

// 測位していないとき: 自機の位置にグレーの ×（v7 と同じ）
static void draw_nofix_cross() {
  const MapFrame& f = map_frame;
  const int arm = 12;
  if (is_trackupmode()) {
    backscreen.drawFastVLine(f.cx, 16, f.cy - 21, COLOR_BLACK);
    backscreen.drawFastVLine(f.cx + 1, 16, f.cy - 21, COLOR_BLACK);
  }
  backscreen.drawWideLine(f.cx - arm, f.cy - arm, f.cx + arm, f.cy + arm, 3, COLOR_GRAY);
  backscreen.drawWideLine(f.cx + arm, f.cy - arm, f.cx - arm, f.cy + arm, 3, COLOR_GRAY);
}

// 精度が悪いとき: 自機の位置に青い点と、水平精度の半径の円（v7 と同じ。半径は地上の距離に合わせる）
static void draw_hacc_circle(float hacc_m) {
  const MapFrame& f = map_frame;
  if (is_trackupmode()) {
    backscreen.drawFastVLine(f.cx, 16, f.cy - 21, COLOR_BLACK);
    backscreen.drawFastVLine(f.cx + 1, 16, f.cy - 21, COLOR_BLACK);
  }
  backscreen.fillCircle(f.cx, f.cy, 3, COLOR_BLUE);
  const int r = (int)lroundf(hacc_m / 1000.0f * px_per_km());
  if (r >= 4 && r <= MAP_W / 2) {
    backscreen.drawCircle(f.cx, f.cy, r, COLOR_BLUE);
  } else {
    char buf[16];
    if (hacc_m < 1000.0f) snprintf(buf, sizeof(buf), "Acc:%.0fm", hacc_m);
    else snprintf(buf, sizeof(buf), "Acc:%.1fkm", hacc_m / 1000.0f);
    backscreen.setTextColor(COLOR_BLUE);
    backscreen.setCursor(f.cx + 6, f.cy - 7);
    backscreen.print(buf);
  }
}

// 修正の円弧と誘導三角形（v7 の draw_triangle と同じ形）。base_deg は画面上での「進路」の向き [deg]。
// ・円弧: 目的地の方位へのズレに比例。修正しないまま外れ続けるほど長くなる（course_warning_index / 900）。
// ・三角: ズレが 15 度・55 度・100 度を超えると 1 つずつ増える。3 秒に 1 回消えて点滅する。
static void draw_steer(float base_deg, int len, float steer, float index) {
  const MapFrame& f = map_frame;
  if ((millis() / 1000) % 3 == 0) return;
  const float arc_factor = index / 900.0f;
  const int span = (int)(arc_factor * steer);
  // drawArc の角度: 0 = 6 時、時計回り。画面上の方位 base_deg は drawArc の (base_deg + 180)
  const int a0 = ((int)lroundf(base_deg) + 180 + 360) % 360;
  if (span > 0) backscreen.drawArc(f.cx, f.cy, len - 21, len - 23, a0, (a0 + span) % 360, COLOR_RED, COLOR_WHITE);
  else if (span < 0) backscreen.drawArc(f.cx, f.cy, len - 21, len - 23, (a0 + span + 360) % 360, a0, COLOR_RED, COLOR_WHITE);
  const float base = base_deg * DEG_TO_RAD;
  const float sgn = steer < 0 ? -1.0f : 1.0f;
  static const float thr[3] = { 15, 55, 100 };
  static const float st[3] = { 0.1f, 0.7f, 1.3f };
  static const float en[3] = { 0.35f, 0.95f, 1.55f };
  for (int k = 0; k < 3; k++) {
    if (fabsf(steer) <= thr[k]) break;
    const float a1 = base + sgn * st[k], a2 = base + sgn * en[k];
    const float x1 = (len - 12) * sinf(a1) + f.cx, y1 = (len - 12) * -cosf(a1) + f.cy;
    const float x2 = (len - 32) * sinf(a1) + f.cx, y2 = (len - 32) * -cosf(a1) + f.cy;
    const float x3 = (len - 22) * sinf(a2) + f.cx, y3 = (len - 22) * -cosf(a2) + f.cy;
    backscreen.fillTriangle(x1, y1, x2, y2, x3, y3, COLOR_RED);
  }
}

// 30 度の範囲表示（v7 と同じ）。基準の進路 tone_tt から左右 15 度。外れた直後の 8 秒は赤く点滅。
static void draw_tone_triangle(float screen_deg, int len, bool warning) {
  const MapFrame& f = map_frame;
  const float l = (screen_deg - 15) * DEG_TO_RAD, r = (screen_deg + 15) * DEG_TO_RAD, c = screen_deg * DEG_TO_RAD;
  const float x1 = (len - 6) * sinf(l) + f.cx, y1 = (len - 6) * -cosf(l) + f.cy;
  const float x2 = (len - 6) * sinf(r) + f.cx, y2 = (len - 6) * -cosf(r) + f.cy;
  const float x3 = len * sinf(c) + f.cx, y3 = len * -cosf(c) + f.cy;
  if (!warning) backscreen.drawTriangle(x1, y1, x2, y2, x3, y3, COLOR_BLACK);
  else backscreen.fillTriangle(x1, y1, x2, y2, x3, y3, (millis() / 1000) % 2 ? COLOR_RED : COLOR_BLACK);
}

// 自機（三角）と進路の針、30 度の範囲、修正の円弧と三角
static void draw_ownship_layer(const MapView& v) {
  const MapFrame& f = map_frame;
  if (!v.live && !v.demo && !v.replay) { draw_nofix_cross(); return; }
  if (!v.marker_exact) { draw_hacc_circle(v.hacc_m); return; }
  if (is_northupmode()) {
    const int len = MAP_H / 2 - 5;
    draw_tone_triangle(v.tone_tt, len, v.tone_warning);
    draw_steer(v.ttrack, len, v.steer, v.course_warning_index);
    // 進路の針（先の方を赤く。v0.87 仕様の太い針: 幅 5px で視認性を向上）
    const float a = v.ttrack * DEG_TO_RAD, s = sinf(a), c = cosf(a);
    backscreen.drawWideLine(f.cx + s * 9, f.cy - c * 9, f.cx + s * (len - 22), f.cy - c * (len - 22), 5, COLOR_BLACK);
    backscreen.drawWideLine(f.cx + s * (len - 22), f.cy - c * (len - 22), f.cx + s * (len - 10), f.cy - c * (len - 10), 5, COLOR_RED);
    const int rbx = -TRIANGLE_HWIDTH * c - TRIANGLE_SIZE * s, rby = -TRIANGLE_HWIDTH * s + TRIANGLE_SIZE * c;
    const int lbx = TRIANGLE_HWIDTH * c - TRIANGLE_SIZE * s, lby = TRIANGLE_HWIDTH * s + TRIANGLE_SIZE * c;
    backscreen.fillTriangle(f.cx, f.cy, f.cx + rbx, f.cy + rby, f.cx + lbx, f.cy + lby, COLOR_BLACK);
  } else {
    // TRACK UP: 画面の上 = 対地進路。進路の縦線（幅 3px で強調）と、上向きの三角。
    const int len = f.cy - 16;
    backscreen.drawFastVLine(f.cx - 1, 16, f.cy - 21, COLOR_BLACK);
    backscreen.drawFastVLine(f.cx,     16, f.cy - 21, COLOR_BLACK);
    backscreen.drawFastVLine(f.cx + 1, 16, f.cy - 21, COLOR_BLACK);
    draw_tone_triangle(v.tone_tt - v.ttrack, len, v.tone_warning);
    draw_steer(0, len - 10, v.steer, v.course_warning_index);
    backscreen.fillTriangle(f.cx, f.cy, f.cx - TRIANGLE_HWIDTH, f.cy + TRIANGLE_SIZE, f.cx + TRIANGLE_HWIDTH, f.cy + TRIANGLE_SIZE, COLOR_BLACK);
  }
}

// 修正しないまま外れ続けたときの「Turn RIGHT/LEFT」（v7 では音声と一緒に 10 秒出す。v3 は表示だけ）
static void draw_course_warning_box(const MapView& v) {
  if (!v.course_warning_box || (millis() / 500) % 2) return;
  const int y = is_trackupmode() ? (map_frame.cy + 24) : (map_frame.cy + 50);
  backscreen.fillRect(5, y, MAP_W - 10, 24, COLOR_WHITE);
  backscreen.drawRect(5, y, MAP_W - 10, 24, COLOR_RED);
  backscreen.drawRect(6, y + 1, MAP_W - 12, 22, COLOR_RED);
  backscreen.setTextColor(COLOR_RED);
  backscreen.setCursor(v.steer > 0 ? 22 : 28, y + 4);
  backscreen.print(v.steer > 0 ? "Turn RIGHT! Turn RIGHT!" : "Turn LEFT! Turn LEFT!");
}

// 赤枠の小さな札（DEMO / REPLAY）
static void draw_label(int x, int y, const char* text) {
  const int w = backscreen.textWidth(text) + 8;
  backscreen.fillRect(x, y, w, 20, COLOR_WHITE);
  backscreen.drawRect(x, y, w, 20, COLOR_RED);
  backscreen.setTextColor(COLOR_RED);
  backscreen.setCursor(x + 4, y + 3);
  backscreen.print(text);
}

// GPS の状態（測位していないとき）。以前は地図画像があると出なかった。
static void draw_gps_status(const MapView& v) {
  if (v.live || v.demo || v.replay) return;
  backscreen.fillRect(0, 0, MAP_W, 42, COLOR_WHITE);
  if (!v.gps_connected) {
    backscreen.setTextColor(COLOR_RED);
    backscreen.setCursor(3, 3);
    backscreen.print("NO GPS DATA !!");
    backscreen.setTextColor(COLOR_BLACK);
    backscreen.setCursor(3, 22);
    backscreen.print("Check GPS cable. Retrying.");
  } else if (v.fix_age_s >= 0) {
    backscreen.setTextColor(COLOR_RED);
    backscreen.setCursor(3, 3);
    backscreen.printf("GPS FIX LOST %ds ago.", v.fix_age_s);
  } else {
    backscreen.setTextColor(COLOR_GREEN);
    backscreen.setCursor(3, 3);
    backscreen.print("GPS Module connected.");
    char text[20] = "Scanning";
    int i = 8;
    for (int k = 0; k < (int)((millis() / 900) % 10) && i < 19; k++) text[i++] = '.';
    text[i] = '\0';
    backscreen.setTextColor(COLOR_BLACK);
    backscreen.setCursor(3, 22);
    backscreen.print(text);
  }
}

// 旋回角速度（deg/s）のオーバーレイ表示（v7 仕様）。
// 旋回中に旋回方向（右旋回＝右上、左旋回＝左上）に出す。直進中（|dps| < 0.5）は消して地図を広く見せる。
static void draw_degpersec(float dps) {
  if (fabsf(dps) < 0.5f) return;
  uint16_t col = COLOR_GRAY;
  if (get_gps_mps() > 2.0f) {
    if (dps > 1.0f) col = COLOR_GREEN;
    if (dps < -1.0f) col = COLOR_BLUE;
    if (fabsf(dps) > 3.0f) {
      if (dps < 0) backscreen.fillRect(0, 15, 48, 35, col);
      else         backscreen.fillRect(MAP_W - 48, 15, 48, 35, col);
      if (fabsf(dps) > 4.0f) {
        if (dps > 0) {
          backscreen.drawRect(MAP_W - 48, 15, 48, 35, COLOR_RED);
          backscreen.drawRect(MAP_W - 47, 16, 46, 33, COLOR_RED);
        } else {
          backscreen.drawRect(0, 15, 48, 35, COLOR_RED);
          backscreen.drawRect(1, 16, 46, 33, COLOR_RED);
        }
      }
      backscreen.setTextColor(COLOR_WHITE, col);
    } else {
      backscreen.setTextColor(col);
    }
  } else {
    backscreen.setTextColor(col);
  }
  const int posx = (dps > 0) ? (MAP_W - 45) : 5;
  backscreen.setTextWrap(false);
  backscreen.setCursor(posx, 17);
  backscreen.printf("%+.1f", dps);
  backscreen.setCursor(posx - 2, 31);
  backscreen.print("deg/s");
}

// backscreen の最上部に小さなテキスト（GS・TT ラベル、sAcc）を描画する（v7 仕様）。
// trackwarning_until が有効な間は「TT」ラベルを赤/黒で点滅させてコース警告を示す。
extern unsigned long trackwarning_until;
static void draw_gs_track() {
  backscreen.setTextWrap(false);

  // 右上: TT (True Track) ラベル。コース警告中は 1 秒おきに赤/黒点滅
  const int hdr_lx = MAP_W - 40;
  if (trackwarning_until > millis()) {
    const bool blink_red = (millis() / 1000) % 2 == 0;
    backscreen.setTextColor(blink_red ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
    backscreen.drawString("TT", hdr_lx, 1);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  } else {
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    backscreen.drawString("TT", hdr_lx, 1);
  }

  // 左上: GS (Ground Speed) ラベル
  backscreen.drawString("GS", 1, 1);

  // sAcc（速度精度）は GS のすぐ右。常時見るものではないので内蔵フォント最小サイズで控えめに表示
  const int gsw = backscreen.textWidth("GS");
  const int gsh = backscreen.fontHeight();
  const float sacc_mps = get_gps_sacc_mps();
  backscreen.unloadFont();
  backscreen.setTextSize(1);
  backscreen.setTextColor((sacc_mps < 0.0f) ? COLOR_GRAY
                        : (sacc_mps < 1.0f) ? COLOR_GREEN
                        : (sacc_mps < 3.0f) ? COLOR_ORANGE : COLOR_RED);
  backscreen.setCursor(1 + gsw + 4, 1 + (gsh - 8) / 2);
  if (sacc_mps < 0.0f) backscreen.print("sAcc--");
  else                 backscreen.printf("sAcc%.2f", sacc_mps);

  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  if (s_map_font) backscreen.loadFont(s_map_font);
}

void draw_map_screen(const MapView& v) {
  static bool reported = false;
  if (!backscreen_ok) {
    if (!reported) { reported = true; enqueueTask(createLogSdTask("ERR map sprite could not be allocated")); }
    return;
  }
  const uint32_t t0 = millis();
  map_begin_frame(v.lat, v.lon, v.scale, v.up);
  draw_vectormap();             // 背景（海・陸・湖）と道路・鉄道。背景の塗りつぶしもここ
  draw_line_maps(v);
  draw_track_layer();
  draw_course_layer(v);
  draw_destination_layer(v);
  draw_course_marks(v);
  draw_km_circles();
  draw_ownship_layer(v);
  draw_compass_layer(v.up);
  draw_degpersec(v.degpersecond);
  draw_course_warning_box(v);
  if (v.demo) draw_label(4, MAP_H - 24, "DEMO");
  if (v.replay) {
    char buf[16];
    snprintf(buf, sizeof(buf), "REPLAY x%d", v.replay_speed);
    draw_label(v.demo ? 64 : 4, MAP_H - 24, buf);
  }
  draw_gs_track();
  draw_gps_status(v);
  backscreen.pushSprite(0, MAP_Y);
  map_frame_ms_last = millis() - t0;
  if (map_frame_ms_last > map_frame_ms_max) map_frame_ms_max = map_frame_ms_last;
}

void draw_startup_map(float scale) {
  if (!backscreen_ok) return;
  map_begin_frame(35.2334225841915, 136.091056306493, scale, 0);
  map_frame.cy = MAP_H / 2;   // 起動画面は TRACK UP でも中央
  draw_vectormap();
  MapView v = {};
  v.lat = map_frame.cLat; v.lon = map_frame.cLon; v.scale = scale;
  draw_course_layer(v);
  draw_course_marks(v);
}
