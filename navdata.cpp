// ============================================================
// File    : navdata.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : ナビゲーション計算の実装。
//           Mercator投影による緯度→Y座標変換、距離・真方位計算、
//           フラッシュ内蔵ポリゴン地図の実体と一覧(flashmaps)、目的地リスト、
//           コース・目的地モード管理（FLYINTO / FLYAWAY / AUTO10K）。
//           内蔵ポリゴンは滑走路外周や島など、ベクタ地図では表せない
//           飛行用の注記に限る（SDが無くても必ず描画される）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================
// Geo calculations and navdata.
#include <Arduino.h>

#include "navdata.h"
#include "gps.h"


// LatLonManager コンストラクタ：リングバッファのインデックスとカウントを 0 で初期化
LatLonManager::LatLonManager() : currentIndex(0), count(0) {}


// magc: 現在地→目的地の磁方位（表示用）。nav_update() で毎回更新される。
int magc = 0;
// dest_dist: 現在地→目的地の距離 [km]。nav_update() で毎回更新される。
float dest_dist = 0;

// destination_mode: ナビモード（FLYINTO / FLYAWAY / AUTO10K）。設定画面から変更可。
int destination_mode = DMODE_FLYAWAY;
// auto10k_status: AUTO10K モード時のフェーズ（AWAY=折り返し前 / INTO=折り返し後）。
int auto10k_status = AUTO10K_AWAY;


// extramaps[]: SD カードから読み込んだ地図ポリゴンを格納する配列。
mapdata extramaps[MAX_MAPDATAS];
// current_id: mapdata や destination に割り当てる ID のカウンタ（100 から始まる）。
int current_id = 100;
int mapdata_count = 0;  // SD から読み込んだ地図ポリゴンの数

// extradestinations[]: 目的地リスト。init_destinations() で固定値を登録し、
//                      SD ファイルから追加ロードも可能。
mapdata extradestinations[MAX_DESTINATIONS];
int destinations_count = 0;  // 登録済み目的地の数
int currentdestination = -1; // 現在選択中の目的地インデックス（-1 = 未選択）


// 起動時に固定の目的地を登録する。
// 登録順: PLATHOME（出発地）→ N_PILON（北パイロン）→ W_PILON（西パイロン）→ TAKESHIMA（竹島）
// SHINURA（新浦安）はコメントアウト中（現時点では使用しない）。
void init_destinations(){
  destinations_count = 0;
  currentdestination = 0;
  // PLATHOME: 出発地（プラットフォーム）の緯度経度
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("PLATHOME");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PLA_LAT, PLA_LON} };
  destinations_count++;
  // N_PILON: 北パイロン（10km コース折り返し地点）
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("N_PILON");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PILON_NORTH_LAT, PILON_NORTH_LON} };
  destinations_count++;
  // W_PILON: 西パイロン（10km コース折り返し地点）
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("W_PILON");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {PILON_WEST_LAT, PILON_WEST_LON} };
  destinations_count++;
  // TAKESHIMA: 竹島（10km コース折り返し地点）
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("TAKESHIMA");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {TAKESHIMA_LAT, TAKESHIMA_LON} };
  destinations_count++;
  /*
  extradestinations[destinations_count].id = current_id++;
  extradestinations[destinations_count].name = strdup("SHINURA");
  extradestinations[destinations_count].size = 1;
  extradestinations[destinations_count].cords = new double[][2]{ {SHINURA_LAT, SHINURA_LON} };
  destinations_count++;*/
}


// 度 → ラジアン変換（float 版。三角関数を使う前に角度を変換するユーティリティ）
float deg2rad(float degrees) {
  return degrees * PI / 180.0;
}

// ラジアン → 度 変換（double 版）
double rad2deg(double rad) {
  return rad * (180.0 / PI);
}


// Haversine 式を使って 2 点間の球面距離を計算する [km]。
// 経緯度をラジアンに変換してから Haversine 公式を適用し、
// 地球半径（RADIUS_EARTH_KM ≒ 6371km）を掛けて距離を返す。
double calculateDistanceKm(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = deg2rad(lat1);
  lon1 = deg2rad(lon1);
  lat2 = deg2rad(lat2);
  lon2 = deg2rad(lon2);

  // Haversine formula
  double dlon = lon2 - lon1;
  double dlat = lat2 - lat1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = RADIUS_EARTH_KM * c;

  return distance;
}


// 2 点間の真方位（True Course）をラジアンで返す。
// atan2 を使った球面三角法による計算。返り値の範囲は -π ～ +π（北基準・時計回り）。
double calculateTrueCourseRad(double lat1, double lon1, double lat2, double lon2) {
  double deltaLon = lon2 - lon1;
  return atan2(sin(deltaLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon));
}


// PLA→N パイロン と PLA→W パイロン の「角度的にちょうど中間」の方位を返す [rad]。
// パイロン座標（navdata.h の #define）が変わっても自動追従するよう、
// 中間方位を定数で持たずにここで計算する。初回だけ計算して static にキャッシュする。
// 単純な角度の平均は 0°/360° をまたぐと真逆を向くので、
// 2 方位の単位ベクトルの和の偏角として求める（常に狭い側の二等分線になる）。
double pla_centerline_bearing_rad() {
  static double cached = NAN;
  if (isnan(cached)) {
    // calculateTrueCourseRad の引数はラジアン。deg2rad(double) はこの下で定義されており
    // ここではまだ宣言されていない（float 版に落ちて精度が落ちる）ため、直接変換する。
    const double D2R = PI / 180.0;
    double bn = calculateTrueCourseRad(PLA_LAT * D2R, PLA_LON * D2R,
                                       PILON_NORTH_LAT * D2R, PILON_NORTH_LON * D2R);
    double bw = calculateTrueCourseRad(PLA_LAT * D2R, PLA_LON * D2R,
                                       PILON_WEST_LAT * D2R, PILON_WEST_LON * D2R);
    cached = atan2(sin(bn) + sin(bw), cos(bn) + cos(bw));
  }
  return cached;
}


// ナビゲーション情報を更新する（毎ループ呼ばれる）。
// 処理内容:
//   1. 現在の GPS 位置 → 選択中目的地 の距離（dest_dist）を Haversine 式で計算
//   2. 目的地への真方位を求め、磁気偏角 +8° を加味して magc（表示用磁方位）を算出
//      magc = (true_course_deg + 368) % 360  ← +368 は +8°(偏角) + 360(負値防止)
//   3. FLYAWAY モード、または AUTO10K の AWAY フェーズ（折り返し前）では
//      方位を 180° 反転させ「目的地から離れる方向」を示す
void nav_update(){
  // ====Navigation.==== Distance to plathome.
  if(currentdestination != -1 && currentdestination < destinations_count){
    double destlat = extradestinations[currentdestination].cords[0][0];
    double destlon = extradestinations[currentdestination].cords[0][1];
    dest_dist = calculateDistanceKm(get_gps_lat(), get_gps_lon(), destlat, destlon);

    //Fly into magc
    magc = (int)((rad2deg(calculateTrueCourseRad(deg2rad(get_gps_lat()), deg2rad(get_gps_lon()), deg2rad(destlat), deg2rad(destlon))) + 368)) % 360;

    // FLYAWAY または AUTO10K の AWAY フェーズでは 180° 反転（目的地から離れる方向を示す）
    if(destination_mode == DMODE_FLYAWAY || (destination_mode == DMODE_AUTO10K && auto10k_status == AUTO10K_AWAY)){
      magc = (magc+180)%360;
    }
  }
}



// ======================================================================
// LatLonManager — 飛行軌跡の座標をリングバッファで管理するクラス
//
// 内部構造:
//   coords[]     : Coordinate（緯度・経度ペア）を格納する固定長配列（MAX_TRACK_CORDS 個）
//   currentIndex : 次回書き込み位置（0 ～ MAX_TRACK_CORDS-1 をループ）
//   count        : 実際に格納済みのデータ数（MAX_TRACK_CORDS を超えない）
//
// 最新データは currentIndex-1 の位置にあり、
// getData(0) が最新、getData(1) がその 1 つ前 ... のように「新しい順」で取得できる。
// ======================================================================

// リングバッファに座標を 1 件追加する。
// バッファが満杯になると最古のデータを上書きする（ループ動作）。
void LatLonManager::addCoord(Coordinate position) {
  coords[currentIndex] = position;
  currentIndex = (currentIndex + 1) % MAX_TRACK_CORDS;  // 書き込み位置を次に進める
  if (count < MAX_TRACK_CORDS) {
    count++;  // バッファが満杯になるまでカウントを増やす
  }
}

// 現在格納されているデータ件数を返す
int LatLonManager::getCount() {
  return count;
}

// デバッグ用：全座標をシリアル出力する
void LatLonManager::printData() {
  for (int i = 0; i < count; i++) {
    DEBUG_P(20250508,"Coordinate ");
    DEBUG_P(20250508,i + 1);
    DEBUG_P(20250508,": Latitude = ");
    DEBUG_P(20250508,coords[i].latitude);
    DEBUG_P(20250508,", Longitude = ");
    DEBUG_PLN(20250508,coords[i].longitude);
  }
}

// バッファをクリアする（カウントとインデックスを 0 にリセット）
void LatLonManager::reset(){
  count = 0;
  currentIndex = 0;
}

// newest_index 番目（0=最新）の座標を返す。
// リングバッファを逆順にたどることで「最新から n 個前」のデータを O(1) で取得できる。
// インデックスが範囲外の場合は {0,0} を返す。
Coordinate LatLonManager::getData(int newest_index) {
  if (newest_index >= count || newest_index < 0) {
    DEBUG_PLN(20250508,"Invalid index");
    return {0, 0};
  }
  // リングバッファを逆向きにたどる: (書き込み位置 - 1 - n) を循環インデックスで計算
  int index = (currentIndex - 1 - newest_index + MAX_TRACK_CORDS) % MAX_TRACK_CORDS;
  return coords[index];
}

// グローバルインスタンス（プロジェクト全体で共有する唯一の軌跡バッファ）
LatLonManager latlon_manager;


// 度 → ラジアン変換（double 版。Mercator 投影など高精度計算で使用）
double deg2rad(double degrees) {
  return degrees * PI / 180.0;
}

// Mercator 投影で緯度を Y 座標に変換する。
// 式: Y = ln( tan(π/4 + lat/2) )
// 地図上の Y 方向スケーリングに使用（経度は X として線形に扱う）。
double latitudeToMercatorY(double latitude) {
  double radLatitude = deg2rad(latitude);
  return log(tan(PI / 4.0 + radLatitude / 2.0));
}





// 2 点が指定した緯度差・経度差の範囲内にあるかを判定する。
// 主に「現在地が地図の表示範囲に入っているか」のクイックチェックに使う。
bool check_within_latlon(double latdif,double londif,double lat1,double lat2,double lon1,double lon2){
  return (abs(lat1-lat2) < latdif) && (abs(lon1-lon2) < londif);
}


// Arduino 環境には標準 C の strdup() がないため、独自実装。
// 引数の文字列をヒープ（new）にコピーして返す。
// 呼び出し側は使い終わったら delete[] で解放する責任がある。
char* strdup(const char* str) {
    if (str == nullptr) return nullptr;  // Handle nullptr case
    // Calculate the length of the input string
    int len = 0;
    while (str[len] != '\0') {
        len++;
    }
    // Allocate memory for the duplicate string
    char* dup = new char[len + 1];  // +1 for the null terminator
    // Copy the string into the allocated memory
    for (int i = 0; i <= len; i++) {
        dup[i] = str[i];
    }
    return dup;
}

// フラット配列（経度, 緯度, 経度, 緯度...の繰り返し）から mapdata 構造体を生成するヘルパー。
// 引数 cords[] は "lon, lat" のペアが size 個連続した 1 次元配列を想定。
// 内部で new を使って 2 次元配列を確保し、cords[i][0]=lon, cords[i][1]=lat に変換して格納。
mapdata create_static_mapdata(int id, const char* name, int size, const double cords[]) {
  mapdata new_mapdata;
  new_mapdata.id = id;
  new_mapdata.name = strdup(name); // Duplicate string to allocate memory
  new_mapdata.size = size;
  new_mapdata.cords = new double[size][2];
  for (int i = 0; i < size; i++) {
    new_mapdata.cords[i][0] = cords[i * 2];     // 経度
    new_mapdata.cords[i][1] = cords[i * 2 + 1]; // 緯度
  }
  return new_mapdata;
}






// ============================================================
// 以下、地図ポリゴンの座標データ（経度, 緯度 のペアが連続するフラット配列）
// 各配列は create_static_mapdata() に渡して mapdata 構造体に変換される。
// ============================================================

// 沖島（琵琶湖の島）の輪郭座標
const double map_okishima_coords[] = {
  136.0769088607755, 35.21038374683805,
  136.0708233623226, 35.21241872134723,
  136.0623026873304, 35.20950590839073,
  136.0535721871087, 35.21055415403649,
  136.0552235797892, 35.2038605237371,
  136.0505054301688, 35.19897385483104,
  136.0579600332858, 35.19990514421023,
  136.0633659977478, 35.20558123298269,
  136.0679873597187, 35.20313886524324,
  136.0769088607755, 35.21038374683805
};




// 竹島（琵琶湖の島）の輪郭座標
const double map_takeshima_coords[] = {
  136.1791589861376,35.29716068038715,
  136.1777344333285,35.29669382781091,
  136.1772921019273,35.29600789212441,
  136.1782442605164,35.29614077343304,
  136.1791589861376,35.29716068038715,
};
// 竹生島（琵琶湖の島）の輪郭座標
const double map_chikubushima_coords[] = {
  136.1408267964406,35.41939915652542,
  136.1439237744329,35.41973966541397,
  136.1458430188931,35.42190963520056,
  136.1454323899703,35.42511206237064,
  136.1438652329591,35.42333719768954,
  136.1426500169115,35.4243034801035,
  136.1417556191338,35.42223281537326,
  136.140243732289,35.42034015096833,
  136.1408267964406,35.41939915652542
};

// 阪大（大阪大学・豊中キャンパス）外周の輪郭座標
const double map_handaioutside_coords[] = {
  135.4886236527158,34.83002026530426,
135.4889962188252,34.82469915723961,
135.4934601841001,34.81230132856709,
135.499779740941,34.81363611992104,
135.5042958399623,34.81737630516033,
135.5115598944098,34.81068298493258,
135.5159489073391,34.81089768495469,
135.5134822958925,34.81570935399444,
135.5109306555292,34.8211031076365,
135.5072440443363,34.82608173272482,
135.499549592877,34.83035452559445,
135.4984652269781,34.83413621031295,
135.5023745611614,34.83518698624572,
135.507726098019,34.83564163124254,
135.5153376582334,34.83577757383128,
135.5154947213441,34.83376736856319,
135.5161065000802,34.83024558019009,
135.5158675553951,34.82857293305862,
135.516127523331,34.82727388718998,
135.517360947942,34.82576589758838,
135.517517056979,34.82492222066032,
135.5171017415715,34.8240036248263,
135.5177152914927,34.82353857339502,
135.5200874196052,34.82246049018575,
135.5203974681629,34.82133129776793,
135.51966204979,34.82015863801803,
135.5192968455686,34.81743018154395,
135.5194994667812,34.81572477061772,
135.5187083037661,34.81395055749358,
135.5160788533039,34.81124448537869,
135.5172825182793,34.80937611141903,
135.5180016514732,34.81018355860668,
135.5175692070068,34.81092844616481,
135.5186303347167,34.81199399738044,
135.5209094593572,34.81209769126325,
135.5257586202844,34.81629460407122,
135.520900092197,34.81207809510725,
135.5203977034051,34.81020601011961,
135.5227203181792,34.80745338924606,
135.5280243740033,34.80406407348704,
135.5340987227816,34.80277998792202,
135.5380565584433,34.80450237126545,
135.5396474804874,34.81081376255133,
135.5389612217641,34.81321768357751,
135.5340582780971,34.81469106837893,
135.5295641873243,34.81618838912017,
135.5273975266068,34.81670062156046,
135.5257571207703,34.81628964133991,
135.5273829086462,34.81668487811491,
135.5295884540204,34.81619115171294,
135.529800743125,34.81714058247767,
135.5301425561633,34.81973592956236,
135.5275609159398,34.83102837168807,
135.5266587129085,34.83608740370632,
135.5426255182958,34.8363844864101,
135.5551711588323,34.83635480731414,
135.562334886935,34.82853965901526,
135.5700639325516,34.83149645122305,
135.5671954604985,34.84349573976401,
135.5671748868228,34.8554701292979,
135.5636385192465,34.86499147038762,
135.5545066968347,34.87671003184703,
135.5515170738355,34.88210396688621,
135.5405473290972,34.87960112720334,
135.5329160210004,34.88340504643339,
135.5221145959146,34.87435340392013,
135.5348672737379,34.86390675448425,
135.5277740814594,34.86021537867586,
135.5230418622749,34.85614469497724,
135.5217417304277,34.84996035039993,
135.5218945709854,34.84369502775861,
135.526502000889,34.83885569856469,
135.5266608874051,34.83607505452488,
135.5153546240302,34.83579558844479,
135.5147014017668,34.83838588697532,
135.5162033574176,34.84103567436462,
135.5168082263485,34.84285185816889,
135.5152670828406,34.84338616591315,
135.5133467516317,34.84585248982998,
135.5061597732078,34.84472407973706,
135.5036238907301,34.84385490236347,
135.4968409730373,34.84089484927993,
135.4975592089948,34.83604850985738,
135.4985191637665,34.83412156726729,
135.4886236527158,34.83002026530426
};

// 阪大キャンパス内部 区画1（建物・施設の輪郭）
const double map_handaiinside1_coords[] = {
  135.5192546630597,34.82290122322834,
  135.5202608927341,34.82473714307037,
  135.5206192267198,34.82486547821112,
  135.5217566743228,34.82453841549704,
  135.5244832613544,34.82355410608935,
  135.5250823824439,34.82324933976636,
  135.5267673351993,34.82159632903841,
  135.5267177871875,34.8212531312041,
  135.5261512088265,34.820690594109,
  135.5252942965001,34.82033917705255,
  135.5244373365641,34.81942598165465,
  135.5242214600272,34.81906565250041,
  135.5256380375261,34.81859729141113,
  135.5247158404168,34.81729424042686,
  135.5252446824773,34.81694258980153,
  135.5257522649573,34.81629080476041
};
// 阪大キャンパス内部 区画2
const double map_handaiinside2_coords[] = {
  135.5286365645366,34.82568485125642,
  135.5273128013348,34.82549139911511,
  135.5261273662329,34.82554391784173,
  135.5254161526343,34.82517085459351,
  135.5244468888839,34.82354534706387
};
// 阪大キャンパス内部 区画3
const double map_handaiinside3_coords[] = {
  135.5267331073241,34.82133387379517,
  135.5272059493836,34.82108416686005,
  135.5278977953256,34.82100216147467,
  135.528146269457,34.82106616633,
  135.529093955624,34.82091067930091,
  135.5295367759326,34.82067872443532,
  135.5295844950343,34.82034217934558,
  135.5295209430969,34.81975800482471,
  135.5301028726996,34.81969461270889
};
// 阪大キャンパス内部 区画4
const double map_handaiinside4_coords[] = {
  135.5298121242363,34.81706877893021,
  135.5283966354298,34.8177316121194,
  135.5293706790027,34.81885231412389,
  135.5294961829965,34.81976659237854,
  135.5291018988957,34.81982356034222,
  135.5290421344131,34.81914923603694,
  135.5293880045558,34.81909154410851,
  135.5293796309032,34.81885345398125,
  135.5283935365026,34.81770532516372,
  135.5273655622806,34.81810440892493,
  135.5256027870635,34.81860849241601,
  135.5250884665181,34.81878295185591,
  135.5252858782623,34.81916263538049,
  135.5245482800287,34.81955098896822,
  135.5244482112157,34.81945038187022,
  135.5241533909122,34.81960023635778,
  135.5239282663402,34.81957430691934,
  135.5237028220277,34.8192410143936,
  135.5242211303909,34.81907006999748,
  135.5237046505857,34.81924344665654,
  135.5224634013304,34.81985548296133,
  135.5209487140366,34.82070440413926,
  135.5203786293369,34.82175959067454
};
// 阪大キャンパス内部 区画5
const double map_handaiinside5_coords[] = {
  135.5224490054155,34.81983638295585,
  135.5217425718252,34.81894459807737,
  135.52138305435,34.81849558741609,
  135.5211988415703,34.81759455392782,
  135.5223054627608,34.81753896709042,
  135.5237609911553,34.81672296814051,
  135.5240946255629,34.81665363436559,
  135.5247054321576,34.81729110638855,
  135.5217293474069,34.81895734917615,
};
// 阪大キャンパス内 特定施設（カフェ周辺）の座標
const double map_handaicafe_coords[] = {
  135.5221316636889,34.82304448212256,
  135.5219602001092,34.8231062484271,
  135.5220512052736,34.8232288460149,
  135.5222213470718,34.82322483897624,
  135.5223210581044,34.82336042253564,
  135.5222094422173,34.82346143689536,
  135.5220043159746,34.82350711676857,
  135.5216910472552,34.82295557500845,
  135.5220290308307,34.82283299191694,
  135.5221316636889,34.82304448212256
};


// ============================================================
// 各ポリゴン座標配列から mapdata 構造体を生成する。
// create_static_mapdata(id, name, vertex_count, coords[]) の形式。
// vertex_count は「元ファイルの行番号差 + 1」で計算されている
//（コメントアウトされた元データの行番号が由来）。
// 生成した変数は display_tft.cpp の描画関数から参照される。
// ============================================================
mapdata map_okishima      = create_static_mapdata(1,  "g_Okishima", 10,         map_okishima_coords);
mapdata map_takeshima     = create_static_mapdata(2,  "g_Takeshima",79-75+1,   map_takeshima_coords);
mapdata map_chikubushima  = create_static_mapdata(3,  "g_Chikubushima",94-86+1, map_chikubushima_coords);
mapdata map_handaioutside = create_static_mapdata(5,  "Handai_O",  302-218+1,  map_handaioutside_coords);
mapdata map_handaiinside1 = create_static_mapdata(8,  "Handai_I1", 381-366+1,  map_handaiinside1_coords);
mapdata map_handaiinside2 = create_static_mapdata(9,  "Handai_I2", 391-387+1,  map_handaiinside2_coords);
mapdata map_handaiinside3 = create_static_mapdata(10, "Handai_I3", 406-398+1,  map_handaiinside3_coords);
mapdata map_handaiinside4 = create_static_mapdata(11, "Handai_I4", 435-413+1,  map_handaiinside4_coords);
mapdata map_handaiinside5 = create_static_mapdata(12, "Handai_I5", 452-444+1,  map_handaiinside5_coords);
mapdata map_handaicafe    = create_static_mapdata(14, "Handai_C",  499-490+1,  map_handaicafe_coords);

// g_kasaoka（旧 sd/mapdata.csv）。SD が無くても表示されるようフラッシュへ移した。
const double map_kasaoka_coords[] = {
  133.4869678472837,34.4728147860938,
  133.4871975510390,34.4727095954449,
  133.4881447260519,34.4743043013075,
  133.4884811797833,34.4741629794127,
  133.4884067331514,34.4740146211036,
  133.4887865983976,34.4738247559517,
  133.4890434444286,34.4742654475943,
  133.4886520216932,34.4744178435038,
  133.4885474195947,34.4742781708607,
  133.4882112768476,34.4744025779415,
  133.4910194318497,34.4791993924923,
  133.4907661837915,34.4792970682849,
  133.4901084102685,34.4781831807189,
  133.4903586183876,34.4780797471144,
  133.4903144776204,34.4780061940126,
  133.4900727500214,34.4781080019779,
  133.4901040964150,34.4781797188605,
  133.4869678472837,34.4728147860938
};
mapdata map_kasaoka = create_static_mapdata(19, "g_kasaoka", 18, map_kasaoka_coords);

// g_shirahama（旧 sd/mapdata.csv）。SD が無くても表示されるようフラッシュへ移した。
const double map_shirahama_coords[] = {
  135.3582002449459,33.6598338023568,
  135.3586587994990,33.6599901654786,
  135.3584582231541,33.6603674761473,
  135.3581694965115,33.6605472763670,
  135.3561493018920,33.6646459027923,
  135.3570608022416,33.6649740127228,
  135.3572720697824,33.6645845689551,
  135.3579449528222,33.6648331024128,
  135.3575185577942,33.6656986285271,
  135.3568302160038,33.6654680120788,
  135.3569851791002,33.6651367285996,
  135.3560818566934,33.6648138366743,
  135.3532653520524,33.6703346638383,
  135.3533239492741,33.6705765341298,
  135.3531189724412,33.6709510465436,
  135.3526873770888,33.6708110468319,
  135.3582002449459,33.6598338023568
};
mapdata map_shirahama = create_static_mapdata(20, "g_shirahama", 17, map_shirahama_coords);

// g_karasu（旧 sd/mapdata.csv）。SD が無くても表示されるようフラッシュへ移した。
const double map_karasu_coords[] = {
  136.5442804766340,34.6385872291551,
  136.5442266295943,34.6384673040986,
  136.5468030060870,34.6376303136303,
  136.5468617688231,34.6377524983223,
  136.5442804766340,34.6385872291551
};
mapdata map_karasu = create_static_mapdata(21, "g_karasu", 5, map_karasu_coords);

// ===== フラッシュ内蔵ポリゴン地図の一覧 =====
// SD の mapdata.csv 由来（extramaps）と違い、SD が無くても必ず描画される。
// 飛行に必須の注記はこちらに置く。描画は display_tft.cpp の draw_FlashMaps()、
// 一覧表示は MAPLIST 画面が参照する。
mapdata* flashmaps[] = {
  // 琵琶湖の島（沖島・竹島・竹生島）。
  // ベクタ地図の琵琶湖ポリゴンにも島は穴として含まれており「重複」ではあるが、
  // パイロットが飛行中に位置の目印として視覚的に使うため、緑の輪郭線として
  // 意図的に残している。OSM と重複しているという理由で削除しないこと。
  &map_okishima, &map_takeshima, &map_chikubushima,
  &map_handaioutside,
  &map_handaiinside1, &map_handaiinside2, &map_handaiinside3,
  &map_handaiinside4, &map_handaiinside5, &map_handaicafe,
  &map_kasaoka, &map_shirahama, &map_karasu,
};
const int flashmap_count = sizeof(flashmaps) / sizeof(flashmaps[0]);
