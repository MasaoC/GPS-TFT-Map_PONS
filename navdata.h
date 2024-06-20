
#define SHINURA_DATA 30
#define TAKESHIMA_DATA 5
#define CHIKUBUSHIMA_DATA 9
#define OKISHIMA_DATA 10
#define TOYOSU_DATA 7

#define ROW_FILLDATA 19
#define COL_FILLDATA 19


#ifndef CONSTANTS_H
#define CONSTANTS_H

struct mapdata{
  int mapsize;
  double cords[][2];
};

extern const mapdata map_shinura;
extern const mapdata map_toyosu;
extern const mapdata map_takeshima;
extern const mapdata map_chikubushima;
extern const mapdata map_biwako;
extern const mapdata map_okishima;
extern const bool filldata[ROW_FILLDATA][COL_FILLDATA];

#endif // CONSTANTS_H


