
bool good_sd();
void setup_sd();
void write_sd();
void read_sd();

void log_sd(const char* text);
void log_sdf(const char* format, ...);

void saveCSV(float latitude, float longitude,float gs,int mtrack, int year, int month, int day, int hour, int minute, int second) ;


void bmp_open();
void draw_mapimage(double center_lat, double center_lon,int zoomlevel,double rotation);


extern bool gmap_loaded;