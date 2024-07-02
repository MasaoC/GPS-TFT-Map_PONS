
//Stil working on BNO055
//#define COMPASS_BNO055 //not working yet
//#define COMPASS_QMC5883L //tested


void setup_compass();
void compass_loop();
int get_raw_magnetic_heading();
int get_magnetic_heading();