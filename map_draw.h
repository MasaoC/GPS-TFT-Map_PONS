// ============================================================
// File    : map_draw.h
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : 地図画面の地図の部分（ベクタ地図・航跡・誘導線・パイロン・自機・修正矢印）の描画。
//           すべて 240x230 のスプライト（backscreen）に描いてから 1 回で TFT へ送る（v7 と同じ方式）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//
// ■ 0.503 で作り替えた理由
//   以前は TFT に直接線を描き、次のフレームで前の線を白で上書きして消していた（StrokeManager）。
//   この方式では、塗りつぶしのある地図（湖面）・自機まわりの円弧・精度の円などを消し残しなく扱えず、
//   描き直すたびに画面がちらつく。v7 と同じく、スプライトに全部描いてから送る方式に変えた。
//   RAM は 240x230x2 = 約 110KB（以前の SD の地図画像用スプライト 240x240 と同じくらい。そちらは廃止）。
#ifndef MAP_DRAW_H
#define MAP_DRAW_H

#include <TFT_eSPI.h>
#include "settings.h"

// 地図のスプライトを置く画面上の位置と大きさ。上はヘッダ（速度・方位・deg/s のバー、〜49）、
// 下はフッタ（最大 G/S・時刻 285〜）と重ならないようにしてある。
#define MAP_Y 50
#define MAP_W 240
#define MAP_H 230

// 地図の縮尺 [px/km]（GPS_TFT_map.ino の縮尺の一覧と同じ値）
#define SCALE_EXLARGE_GMAP 52.32994872   //zoom13
#define SCALE_LARGE_GMAP 13.08248718     //zoom11
#define SCALE_MEDIUM_GMAP 3.2706218      //zoom9
#define SCALE_SMALL_GMAP 0.81765545      //zoom7
#define SCALE_EXSMALL_GMAP 0.2044138625  //zoom5

extern TFT_eSprite backscreen;

// 1 フレームの投影。map_begin_frame() で決め、そのフレームの描画はすべてこれを使う。
// メルカトルの Y を中心の緯度まわりで 3 次までテイラー展開して、点ごとの log(tan()) を省く（v7 と同じ。
// 画面内で厳密な式との差は 0.2px 以下）。中心からの差を double で取ってから float で計算するので、
// 緯度経度の桁落ちも無い。
struct MapFrame {
  double cLat, cLon;     // 地図の中心（自機）
  int32_t cLat_e7, cLon_e7;
  float scale;           // [px/km]
  float up;              // 画面の上の方位 [deg]
  float kx, ky, c2, c3;  // 経度・緯度 1 度あたりの px と補正係数
  float ca, sa;          // 回転
  int cx, cy;            // 自機のスプライト上の位置
};
extern MapFrame map_frame;

void map_setup(const uint8_t* font);       // スプライトを確保し、スプライト用の文字を読み込む（setup_tft から）
int map_self_cy();                          // 自機の y（TRACK UP は下寄り 3/4、NORTH UP は中央）
void map_begin_frame(double cLat, double cLon, float scale, float up);
// 中心からの差 [度] をスプライト座標へ。ベクタ地図の点の変換もこれを使う。
static inline void map_project_d(float dlat, float dlon, int16_t* ox, int16_t* oy) {
  const MapFrame& f = map_frame;
  const float x = dlon * f.kx;
  const float y = f.ky * dlat * (1.0f + dlat * (f.c2 + dlat * f.c3));
  const float rx = x * f.ca - y * f.sa;
  const float ry = x * f.sa + y * f.ca;
  int32_t sx = (int32_t)f.cx + (int32_t)rx;
  int32_t sy = (int32_t)f.cy - (int32_t)ry;
  if (sx < -20000) sx = -20000; else if (sx > 20000) sx = 20000;   // int16 の桁あふれの保険
  if (sy < -20000) sy = -20000; else if (sy > 20000) sy = 20000;
  *ox = (int16_t)sx;
  *oy = (int16_t)sy;
}
void map_project(double lat, double lon, int* x, int* y);

bool clip_line_to_map(int* x0, int* y0, int* x1, int* y1);
void draw_map_line(int x0, int y0, int x1, int y1, int w, uint16_t col);   // 本数の多い線（地図・航跡）
bool fill_polygon_evenodd(const int16_t* xs, const int16_t* ys, const uint16_t* ring_start,
                          uint8_t nrings, uint16_t color);

// 1 フレームぶんの描画に必要な状態。判断は GPS_TFT_map.ino（nav_tick）が持ち、ここは描くだけ。
struct MapView {
  double lat, lon;          // 地図の中心（自機）
  float scale;              // [px/km]
  float up;                 // 画面の上の方位（TRACK UP は対地進路、NORTH UP は 0）
  float ttrack;             // 対地進路 [deg]
  bool live;                // 位置・方位が今の値か（get_gps_live()）
  bool marker_exact;        // 自機の印を三角にしてよいか（デモ・リプレイ・精度が良いとき）
  float hacc_m;             // 水平精度 [m]（精度の円の大きさ）
  bool has_destination;
  double dest_lat, dest_lon;
  bool fly_into;            // 誘導線の向き（Auto 10km はフェーズで置き換え済み）
  float steer;              // 目的地の方位 − 対地進路 [-180..180]（誘導が無いときは 0）
  float course_warning_index;  // 0..900。修正しないまま外れている積算（円弧の長さ）
  bool course_warning_box;  // 「Turn RIGHT/LEFT」の枠を出す
  float tone_tt;            // 30 度の範囲表示の基準の進路
  bool tone_warning;        // 範囲を外れた直後（三角を点滅）
  bool demo, replay;
  int replay_speed;
  bool gps_connected;       // GPS の状態の文字（live でないとき）
  int fix_age_s;            // 測位を失ってからの秒数（-1 = 一度も測位していない）
};
void draw_map_screen(const MapView& v);    // ベクタ地図から自機まで全部描いて TFT へ送る
void draw_startup_map(float scale);         // 起動画面の琵琶湖（ベクタ地図だけ。push はしない）

// 地図 1 フレームの所要時間（地図一覧とログ用）
extern uint32_t map_frame_ms_last, map_frame_ms_max;

#endif
