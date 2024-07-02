#include "compass.h"

#ifdef COMPASS_QMC5883L
  #include <QMC5883LCompass.h>
  QMC5883LCompass compass;
  //Deviation Table、コンパスの生データがどれだけずれているかを出している。
  //                     000    030   060   090   120   150   180   210   240   270   300   330
  int deviationTable[] = {+6,   +3,   -1,   -4,   -15,  -20,  -15,  -15,  0,    -5,   -5,   +5};
  const int tableSize = sizeof(deviationTable) / sizeof(deviationTable[0]);
  #define VERTICAL_MAGNETIC_DIP -44
#endif


#ifdef COMPASS_BNO055
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  //Deviation Table、コンパスの生データがどれだけずれているかを出している。
  //                     000    030   060   090   120   150   180   210   240   270   300   330
  int deviationTable[] = {+6,   +3,   -1,   -4,   -15,  -20,  -15,  -15,  0,    -5,   -5,   +5};
  const int tableSize = sizeof(deviationTable) / sizeof(deviationTable[0]);
  Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);
#endif

bool device_orientation_vertical = false;
int raw_compass_heading = 0;
int compass_heading = 0;



void setup_compass(){
  #ifdef COMPASS_QMC5883L
    compass.init();
    compass.setSmoothing(5, true);
    compass.setCalibration(-1380, 1080, -1310, 1320, -1220, 1460);
  #endif

  #ifdef COMPASS_BNO055
    if (!bno.begin()) {
      Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    }else{
      bno.setExtCrystalUse(false);
      adafruit_bno055_offsets_t calData = {-14,-62,-51,30,-280,693,-1,-1,0,1000,820};
      bno.setSensorOffsets(calData);
      bno.setAxisRemap(REMAP_CONFIG_P0);
    }
  #endif
}


int get_raw_magnetic_heading(){
  return raw_compass_heading;
}

int get_magnetic_heading(){
  return compass_heading;
}

unsigned long last_compass_update = 0;
#define COMPASS_INTERVAL 200



// Function to interpolate deviation
int interpolateDeviation(int heading) {
  int index1 = (int)(heading / 30.0);
  int index2 = (index1 + 1) % tableSize;
  float fraction = (heading - (index1 * 30.0)) / 30.0;
  return deviationTable[index1] + fraction * (deviationTable[index2] - deviationTable[index1]);
}

// Function to correct compass heading
float correctHeading(int rawHeading) {
  float deviation = interpolateDeviation(rawHeading);
  float correctedHeading = rawHeading - deviation;
  
  // Ensure the heading is within 0 to 360 degrees
  if (correctedHeading < 0) correctedHeading += 360;
  if (correctedHeading >= 360) correctedHeading -= 360;
  
  return correctedHeading;
}


void compass_loop(){
  unsigned long timenow = millis();
  if(timenow - last_compass_update > COMPASS_INTERVAL){
    last_compass_update = timenow;

    #ifdef COMPASS_QMC5883L
      compass.read();
      int x = compass.getX();
      int y = compass.getY();
      int z = compass.getZ();

      double dipangle = atan2(-y, sqrt(z * z + x * x));
      double direction = atan2(x, z);
      raw_compass_heading = -direction * 180 / 3.1415;
      if (raw_compass_heading < 0) raw_compass_heading += 360;
      compass_heading = correctHeading(raw_compass_heading);
      int dipdif = dipangle*180/3.1415-VERTICAL_MAGNETIC_DIP;
      if((abs(dipdif) > 15)){
        Serial.println("Device vertical?");
        device_orientation_vertical = false;
      }else{
        device_orientation_vertical = true;
      }
    #endif
    #ifdef COMPASS_BNO055
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      float yaw = euler.x();
      float roll = euler.y();
      float pitch = euler.z();
    #endif
  }
}

