#define ROW_FILLDATA 28
#define COL_FILLDATA 28


#define PLA_LAT 35.29491494
#define PLA_LON 136.255252
#define SHINURA_LAT 35.6343618
#define SHINURA_LON 139.9206059
#define OSAKA_LAT 34.8227376
#define OSAKA_LON 135.5213544


#define PILON_NORTH_LAT 35.41640778478595
#define PILON_NORTH_LON 136.1183001762145
#define PILON_WEST_LAT 35.23295479141404
#define PILON_WEST_LON 136.0493286559818
#define TAKESHIMA_LAT 35.296584352454964
#define TAKESHIMA_LON 136.1780537684742

#ifndef CONSTANTS_H
#define CONSTANTS_H

struct mapdata {
  int id;
  int size; 
  double cords[][2];
};


extern const mapdata map_shinura;
extern const mapdata map_takeshima;
extern const mapdata map_chikubushima;
extern const mapdata map_biwako;
extern const mapdata map_okishima;

extern const mapdata map_handaioutside;
extern const mapdata map_handaihighway;
extern const mapdata map_handaihighway2;
extern const mapdata map_handaiinside1;
extern const mapdata map_handaiinside2;
extern const mapdata map_handaiinside3;
extern const mapdata map_handaiinside4;
extern const mapdata map_handaiinside5;
extern const mapdata map_handairailway;
extern const mapdata map_handaicafe;

extern const mapdata map_japan1;
extern const mapdata map_japan2;
extern const mapdata map_japan3;
extern const mapdata map_japan4;


extern const bool filldata[ROW_FILLDATA][COL_FILLDATA];

#endif  // CONSTANTS_H