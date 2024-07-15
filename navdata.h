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
  char* name;
  int size;
  double (*cords)[2]; // Pointer to an array of 2-element arrays
};



#define MAX_MAPDATAS 20
extern mapdata extramaps[MAX_MAPDATAS];
extern int current_id;
extern int mapdata_count;

extern mapdata map_shinura;
extern mapdata map_takeshima;
extern mapdata map_chikubushima;
extern mapdata map_biwako;
extern mapdata map_okishima;

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

extern mapdata map_japan1;
extern mapdata map_japan2;
extern mapdata map_japan3;
extern mapdata map_japan4;


extern const bool filldata[ROW_FILLDATA][COL_FILLDATA];

#endif  // CONSTANTS_H