
#define SHINURA_DATA 30
#define TAKESHIMA_DATA 5
#define CHIKUBUSHIMA_DATA 9
#define OKISHIMA_DATA 10
#define TOYOSU_DATA 7

#define ROW_FILLDATA 28
#define COL_FILLDATA 28


#define PLA_LAT 35.29491494
#define PLA_LON 136.255252
#define SHINURA_LAT 35.6343618
#define SHINURA_LON 139.9206059
#define OSAKA_LAT 34.8227376
#define OSAKA_LON 135.5213544



#ifndef CONSTANTS_H
#define CONSTANTS_H

struct mapdata{
  int mapsize;
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


extern const bool filldata[ROW_FILLDATA][COL_FILLDATA];

#endif // CONSTANTS_H


