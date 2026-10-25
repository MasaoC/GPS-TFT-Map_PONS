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


  // 度 → ラジアン。距離・方位の計算は double で行う
  // （以前は display_tft.cpp に float 版があり、緯度経度が float に丸められていた）。
  double deg2rad(double degrees);
  double rad2deg(double rad);
  // 2 点間の大円距離 [km]（Haversine）
  double calculateDistance(double lat1, double lon1, double lat2, double lon2);
  // 1 から 2 への真方位 [rad]（引数はラジアン。-PI..PI）
  double calculateTrueCourseRad(double lat1, double lon1, double lat2, double lon2);
  // 1 から 2 への真方位 [deg]（引数は度。0 以上 360 未満、四捨五入しない）
  double true_course_deg(double lat1, double lon1, double lat2, double lon2);
  bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2);

  // プラットホームから北・南パイロンへの 2 等分線方位 [rad]（-PI..PI）。
  // パイロン座標が SD 等で上書きされた場合も自動で追従する。
  double pla_centerline_bearing_rad();

  // 文字列を new[] でヒープに複製する。解放は必ず delete[]。
  // libc の strdup()（malloc確保）との衝突・未定義動作を防ぐため名前を分けて宣言。
  char* pons_strdup(const char* str);





    //公式ルール2026.06.21
  #define PLA_LAT 35.294222
  #define PLA_LON 136.254333
  #define PILON_NORTH_LAT 35.377878
  #define PILON_NORTH_LON 136.189907
  // 南パイロン。公式ルールの呼称は「南」。以前は西パイロン(W_PILON)と呼んでいた（座標は同じ）。
  #define PILON_SOUTH_LAT 35.274220
  #define PILON_SOUTH_LON 136.136183
  // 1km パイロン（復路の折り返し。v7 と同じ座標）
  #define PILON_1KM_LAT 35.297961
  #define PILON_1KM_LON 136.243624


  
  #define SHINURA_LAT 35.650433188178
  #define SHINURA_LON 139.913699521520
  #define OSAKA_LAT 34.8227376
  #define OSAKA_LON 135.5213544
  #define SAPPORO_LAT 43.05989937316593
  #define SAPPORO_LON 141.37769461180662

  #define SHISHI_LAT 36.4734161//36.44641973
  #define SHISHI_LON 136.9234498//136.64713865

  #define TAKESHIMA_LAT 35.296584352454964
  #define TAKESHIMA_LON 136.1780537684742



  #define KM_PER_DEG_LAT (111.321)
  #define KM_PER_DEG_LON(LAT_DEG) (cos(LAT_DEG * DEG_TO_RAD) * 111.321)


  struct mapdata {
    int id;
    char* name;
    int size;
    double (*cords)[2]; // Pointer to an array of 2-element arrays
  };



  #define MAX_MAPDATAS 100
  extern mapdata extramaps[MAX_MAPDATAS];
  extern int current_id;
  extern int mapdata_count;

  #define MAX_DESTINATIONS 100
  extern mapdata extradestinations[MAX_DESTINATIONS];
  extern int destinations_count;
  extern int currentdestination;

  // コースの地点の実際に使う座標。既定値は上の #define で、SD の override_pilon_coordinate.csv が
  // あれば起動時に上書きする（mysd.cpp の load_pilon_override()）。内蔵の目的地・地図のパイロンと
  // 基準線・折り返しの円がすべてこれを見るので、1 か所直せば全部が一致して変わる。
  // ★ 目的地の cords と同じ形（[0][0]=緯度, [0][1]=経度）。地図ポリゴンとは順序が逆。
  extern double course_pla[1][2], course_npilon[1][2], course_spilon[1][2], course_1km[1][2], course_takeshima[1][2];
  extern volatile bool course_override_loaded;   // SD から上書きしたか（地図一覧の表示用）

  // Auto 10km（DMODE_AUTO10K）の折り返しのフェーズ
  #define AUTO10K_AWAY 0
  #define AUTO10K_INTO 1
  extern int auto10k_status;          // 表示中のフェーズ（デモ中は仮想の位置で動く）
  extern int auto10k_status_flight;   // 実際の位置でのフェーズ（SD に保存する値）
  // 誘導に使う向き（DMODE_FLYINTO / DMODE_FLYAWAY）。Auto 10km はフェーズで置き換える。
  int nav_effective_mode();
  const char* nav_mode_text();        // 地図の下に出す文字（FLY INTO / FLY AWAY FRM / 10K AWAY / 10K INTO）
  // Auto 10km にしてはいけない目的地（パイロン・竹島。v7 と同じ）。そこから 10km 離れる誘導は意味が無い。
  bool is10k_not_allowed(const char* name);



  // 内蔵の折れ線地図（大阪大学の構内だけ。v7 と同じ）。
  // ★ 琵琶湖・沖島・竹島・竹生島・浦安・日本の海岸線の折れ線は 0.503 で削除した（内蔵のベクタ地図にある）。
  extern mapdata map_handaioutside;
  extern mapdata map_handaihighway;
  extern mapdata map_handaihighway2;
  extern mapdata map_handaiinside1;
  extern mapdata map_handaiinside2;
  extern mapdata map_handaiinside3;
  extern mapdata map_handaiinside4;
  extern mapdata map_handaiinside5;
  extern mapdata map_handairailway;
  extern mapdata map_handaicafe;

#endif