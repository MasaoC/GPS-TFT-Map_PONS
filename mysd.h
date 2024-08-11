
bool good_sd();
void setup_sd();
void write_sd();
void read_sd();

void log_sd(const char* text);

void saveCSV(float latitude, float longitude,float gs,int mtrack, int year, int month, int day, int hour, int minute, int second) ;


void bmp_open();
void display_region3(float center_lat, float center_lon) ;
void display_region5(double center_lat, double center_lon);