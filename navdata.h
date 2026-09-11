// ============================================================
// File    : navdata.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : ナビゲーションデータの中核ヘッダー。
//           座標変換（Mercator投影）・距離・真方位計算の宣言、
//           内蔵ポリゴン地図・目的地・飛行コースモードの定義、
//           各飛行地点（琵琶湖・白浜・笠岡・富士川・東京湾など）の座標。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/10
// ============================================================
#include "settings.h"


#ifndef NAVDATA_H
#define NAVDATA_H



  struct Coordinate {
    float latitude;
    float longitude;
  };

  class LatLonManager {
  private:
    Coordinate coords[MAX_TRACK_CORDS];
    int currentIndex;
    int count;

  public:
    LatLonManager();
    void addCoord(Coordinate position);
    int getCount();
    void reset();
    Coordinate getData(int newest_index);
  };
  extern LatLonManager latlon_manager;


  // Function to calculate y-coordinate in Mercator projection
  double latitudeToMercatorY(double latitude);
  bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);





  #define RADIUS_EARTH_KM 6371.0  // Earth's radius in kilometers

  //公式ルール2026.06.21
  #define PLA_LAT 35.294222
  #define PLA_LON 136.254333
  #define PILON_NORTH_LAT 35.377878
  #define PILON_NORTH_LON 136.189907
  // 南パイロン。公式ルールの呼称は「南」。以前は西パイロン(W_PILON)と呼んでいた。
  #define PILON_SOUTH_LAT 35.274220
  #define PILON_SOUTH_LON 136.136183
  #define PILON_1KM_LAT 35.297961
  #define PILON_1KM_LON 136.243624


  #define SHINURA_LAT 35.650433188178
  #define SHINURA_LON 139.913699521520

  // ---- デモ飛行の地点（設定画面の DEMO で選択）----
  // HPA のフライト地点。おおよその中心座標で、厳密な滑走路位置ではない。
  #define SHIRAHAMA_LAT 33.679000    // 南紀白浜（沿岸）
  #define SHIRAHAMA_LON 135.362000
  #define KASAOKA_LAT   34.495000    // 笠岡ふれあい空港・笠岡湾干拓地
  #define KASAOKA_LON   133.495000
  #define FUJIGAWA_LAT  35.155000    // 富士川滑空場（富士川河口）
  #define FUJIGAWA_LON  138.625000
  #define OSAKA_LAT 34.8227376
  #define OSAKA_LON 135.5213544
  #define SAPPORO_LAT 43.05989937316593
  #define SAPPORO_LON 141.37769461180662
  #define SHISHI_LAT 36.4734161//36.44641973
  #define SHISHI_LON 136.9234498//136.64713865


  #define TAKESHIMA_LAT 35.296584352454964
  #define TAKESHIMA_LON 136.1780537684742



  #define SCALE_EXLARGE_GMAP 52.32994872   //zoom13
  #define SCALE_LARGE_GMAP 13.08248718     //zoom11
  #define SCALE_MEDIUM_GMAP 3.2706218      //zoom9
  #define SCALE_SMALL_GMAP 0.81765545      //zoom7
  #define SCALE_EXSMALL_GMAP 0.2044138625  //zoom5


  #define KM_PER_DEG_LAT (111.321)
  #define KM_PER_DEG_LON(LAT_DEG) (cos(LAT_DEG * DEG_TO_RAD) * 111.321)


  struct mapdata {
    int id;
    char* name;
    int size;
    double (*cords)[2]; // Pointer to an array of 2-element arrays
  };

  // 文字列を new[] でコピーする。**解放は必ず delete[]**（navdata.cpp のコメント参照）。
  // libc の strdup() と混ざらないよう、名前を分けてここで宣言しておく。
  char* pons_strdup(const char* str);

  float deg2rad(float degrees);
  double rad2deg(double rad);
  extern int truec;

  // ---- 実行時のコース座標 ----
  // 既定値は上の #define。SD の override_pilon_coordinate.csv があれば起動時に上書きする。
  // 表示・誘導・地図描画はすべてこの変数を見るので、上書きすれば全部が一致して変わる。
  // パイロン座標は毎年変わり得るのに対しファームウェアの再ビルドは大会直前には難しいため。
  extern double pla_lat, pla_lon;                  // プラットホーム
  extern double pilon_north_lat, pilon_north_lon;  // 北パイロン
  extern double pilon_south_lat, pilon_south_lon;  // 南パイロン（旧称 西パイロン）
  extern double pilon_1km_lat, pilon_1km_lon;      // 1km パイロン
  extern double takeshima_lat_v, takeshima_lon_v;  // 竹島
  extern bool   pilon_override_loaded;             // SD から上書きしたか（表示用）
  extern float dest_dist;



  double calculateDistanceKm(double lat1, double lon1, double lat2, double lon2);
  double calculateTrueCourseRad(double lat1, double lon1, double lat2, double lon2);
  // PLA→N パイロンと PLA→W パイロンの中間方位（センターライン）[rad]
  double pla_centerline_bearing_rad();
  void nav_update();

  extern mapdata map_kasaoka;
  extern mapdata map_shirahama;
  extern mapdata map_karasu;

  // フラッシュ内蔵のポリゴン地図一覧（SD 不要で必ず描画される）
  extern mapdata* flashmaps[];
  extern const int flashmap_count;

  #define MAX_MAPDATAS 100
  extern mapdata extramaps[MAX_MAPDATAS];
  extern int current_id;
  extern int mapdata_count;

  #define MAX_DESTINATIONS 100
  extern mapdata extradestinations[MAX_DESTINATIONS];
  extern int destinations_count;
  extern int currentdestination;

  //destination mode
  #define DMODE_FLYINTO 0
  #define DMODE_FLYAWAY 1
  #define DMODE_AUTO10K 2
  extern int destination_mode;


  #define AUTO10K_AWAY 0
  #define AUTO10K_INTO 1
  // 表示・ナビ方位に使う現在のフェーズ。リプレイ再生中は再生データで書き換わる。
  extern int auto10k_status;

  // ---- 折返しフェーズの永続化 ----
  // 飛行中に再起動すると AWAY に戻り、復路ではナビ方位が 180 度逆を指したまま
  // 自動復帰しない（距離だけでは往路と復路を区別できないため）。そこで SD に保存する。
  // リプレイ再生でも状態遷移は起きるので、保存用の値は実飛行の遷移だけで更新する。
  void set_auto10k_status_flight(int st);   // 実飛行の遷移。現在値と保存値の両方を更新
  int  get_auto10k_status_flight();         // SD へ書く値（リプレイの影響を受けない）
  void restore_auto10k_status(int st);      // SD 設定からの復元
  void auto10k_leave_replay();              // リプレイを抜けたとき実飛行の値へ戻す

  void init_destinations();


  extern mapdata map_takeshima;
  extern mapdata map_chikubushima;
  extern mapdata map_okishima;

  extern mapdata map_handaioutside;
  extern mapdata map_handaiinside1;
  extern mapdata map_handaiinside2;
  extern mapdata map_handaiinside3;
  extern mapdata map_handaiinside4;
  extern mapdata map_handaiinside5;
  extern mapdata map_handaicafe;


#endif