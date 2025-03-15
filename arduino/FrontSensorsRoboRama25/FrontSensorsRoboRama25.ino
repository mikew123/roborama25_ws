/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

  mrw 1/30/2024 controls 3 sensors and prints array on 8 lines
  mrw 3/15/2024 Could not control the BNO055 sensor, the libs seem to clash
  mrw 3/15/2024 Try to control the INA228 current sensor for battery

  mrw 3/8/2025 Copied file for RoboRama2025
  mrw 3/8/2025 Connect Wire1(2,3) to I2C connector and INA228 code
  mrw 3/9/2025 Add VL53L4Cx and BNO055 code
*/

#include <Wire.h>

#include <Adafruit_INA228.h>
Adafruit_INA228 ina228 = Adafruit_INA228();

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// This lib also works with VL53L4CX 
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
SFEVL53L1X distanceSensor(Wire1);

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
// Create 3 sensors Left, Front, Right
SparkFun_VL53L5CX myImagerLL;
SparkFun_VL53L5CX myImagerLR;
SparkFun_VL53L5CX myImagerRL;
SparkFun_VL53L5CX myImagerRR;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM


int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
int sharpenerPct = 30; // default 5%
int integrationTimeMsec = 20; // default 5ms


#define pinSDA 0
#define pinSCL 1
#define pinSDA_1 2
#define pinSCL_1 3

#define pinTofLPLL 4 /* sensor enable active low */
#define pinTofLPLR 5
#define pinTofLPRL 6
#define pinTofLPRR 7
#define pinTofLED 8
#define pinTofPwrEn 14 /* 3.3V power down LOW */


//#define i2cImuAddr  0x28 /* BNO055? */
#define i2cTofAddrD 0x29 /* default TOF */
#define i2cTofAddrLL 0x2A /* Left L TOF*/
#define i2cTofAddrLR 0x2B /* Left R TOF */
#define i2cTofAddrRL 0x2C /* Right L TOF */
#define i2cTofAddrRR 0x2D /* Right R TOF */
//#define i2cCurrAddr 0x40 /* INA228?*/

//#define rangingFreqHz 10
//#define resolution (8*8) /*8*8 or 4*4 */

//////////////////////////////////////////////////////////
// enable 3.3V regulator control pin and cycle the power
void init3v3Regulator(void){
  // power cycle to reset default i2c address on VL53L5 devices
  // also it seems like simply enabling the PwrEn pin causes a glitch
  pinMode(pinTofPwrEn, OUTPUT);
  digitalWrite(pinTofPwrEn, LOW);
  delay(100);
  digitalWrite(pinTofPwrEn, HIGH);
  delay(100);
}

///////////////////////////////////////////////////////////
// INA228 current sensor
#define INA228_SAMPLERATE_HZ 1
int batDataPeriodMs = 0;
uint32_t lastInaMillis = 0;
void cfgBatDataRate(int hz){
  if(hz>0 && hz<=10) batDataPeriodMs = 1000/hz;
  else batDataPeriodMs = 0;
}
void initINA228(void) {

  cfgBatDataRate(INA228_SAMPLERATE_HZ);

  if (!ina228.begin(64,&Wire1)) {
    Serial.println("Couldn't find INA228 chip");
    while (1)
      ;
  }
//  Serial.println("Found INA228 chip");
  // the shunt R=0.015 Ohm and max current A=3
  ina228.setShunt(0.015, 3.0);
  ina228.setAveragingCount(INA228_COUNT_16);
  // reading 1/sec
  ina228.setVoltageConversionTime(INA228_TIME_540_us);
  ina228.setCurrentConversionTime(INA228_TIME_540_us);

  Serial.println("INA228 sensor initialized OK");
  
  lastInaMillis = millis();
}

void getInaSensorData(){
  if (batDataPeriodMs==0) return;
  if ((millis() - lastInaMillis) >= batDataPeriodMs) {
    lastInaMillis = millis();
    // convert to integers since in mv and ma
    int vbus_mv = ina228.readBusVoltage();
    int curr_ma = ina228.readCurrent();
    int temp_c = ina228.readDieTemp();

    // send message
    Serial.print("BT ");
    Serial.print(vbus_mv);Serial.print(" ");
    Serial.print(curr_ma);Serial.print(" ");
    Serial.print(temp_c);Serial.println("");
  }
}

///////////////////////////////////////////////////////////////
// BNO055 IMU sensor
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_HZ (50)
#define BNO055_CALRATE_HZ (1)
int imuDataRateMs = 0;
int imuCalRateMs = 0;
uint32_t imu_last_millis = 0;
uint32_t cal_last_millis = 0;

void cfgImuDataRate(int hz) {
  if (hz>0 && hz<=50) imuDataRateMs = 1000/hz;
  else imuDataRateMs = 0;
  imu_last_millis = millis();
}
void cfgImuCalRate(int hz) {
  if (hz>0 && hz<=10) imuCalRateMs = 1000/hz;
  else imuCalRateMs = 0;
  cal_last_millis = millis();
}

void initImuSensor(){
  /* Initialise the sensor */
  if(!bno.begin( OPERATION_MODE_NDOF)) {
    /* There was a problem detecting the BNO055 ... check your connections, freezing!! */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR, freezing!");
    while(1);
  }

  // Write calibration data to jump start IMU operation
  uint8_t imuCalData[22] = {222,255,251,255,238,255,110,0,117,1,51,1,255,255,254,255,255,255,232,3,17,3};
  bno.setSensorOffsets(imuCalData);

  bno.setExtCrystalUse(true);

  cfgImuDataRate(BNO055_SAMPLERATE_HZ);
  cfgImuCalRate(BNO055_CALRATE_HZ);

  imu_last_millis = millis();
  cal_last_millis = millis();
}

void getImuSensorData(void){
  if (imuDataRateMs==0) return;
  if((millis()-imu_last_millis)>=imuDataRateMs) {
    imu_last_millis = millis();

    /* Get a new sensor event */
    sensors_event_t angVelocityData, linearAccelData;
    bool a = bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bool v = bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = bno.getQuat();

    Serial.print("IMU ");
    Serial.print(millis());Serial.print(" ");
    Serial.print(angVelocityData.acceleration.x,4); Serial.print(" ");
    Serial.print(angVelocityData.acceleration.y,4); Serial.print(" ");
    Serial.print(angVelocityData.acceleration.z,4); Serial.print(" ");
    Serial.print(linearAccelData.acceleration.x,4); Serial.print(" ");
    Serial.print(linearAccelData.acceleration.y,4); Serial.print(" ");
    Serial.print(linearAccelData.acceleration.z,4); Serial.print(" ");
    Serial.print(quat.w(), 4); Serial.print(" ");
    Serial.print(quat.x(), 4); Serial.print(" ");
    Serial.print(quat.y(), 4); Serial.print(" ");
    Serial.print(quat.z(), 4); Serial.println(""); // end line
  }
}

void getImuCalStatus(void){
  if (imuCalRateMs==0) return;
  if((millis()-cal_last_millis)>=imuCalRateMs) {
    cal_last_millis = millis();
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.print("CAL ");
    Serial.print(system,DEC);Serial.print(" ");
    Serial.print(gyro  ,DEC);Serial.print(" ");
    Serial.print(accel ,DEC);Serial.print(" ");
    Serial.print(mag   ,DEC);Serial.println();
  }
}

/////////////////////////////////////////////////////////////////////////
// VL53L4CX single point TOF sensor
#define L4_DATA_RATE_HZ 10
int l4DataPeriodMs = 0;
void cfgL4DataRate(int hz){
  if(hz>0 && hz<=10) l4DataPeriodMs = 1000/hz;
  else l4DataPeriodMs = 0;
}
void initL4Sensor(void){
  cfgL4DataRate(L4_DATA_RATE_HZ);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("VL53L4CX sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("VL53L4CX sensor online!");

  // Short mode max distance is limited to 1.3 m but has a better ambient immunity.
  // Above 1.3 meter error 4 is thrown (wrap around).
  //distanceSensor.setDistanceModeShort();
  distanceSensor.setDistanceModeLong(); // default

  /*
     * The minimum timing budget is 20 ms for the short distance mode and 33 ms for the medium and long distance modes.
     * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
     * This function must be called after SetDistanceMode.
     */
  distanceSensor.setTimingBudgetInMs(50); // Set for 10HZ or less

  // measure periodically about 10/sec. 
  // Intermeasurement period must be >/= timing budget.
  distanceSensor.setIntermeasurementPeriod(1000/L4_DATA_RATE_HZ);
  distanceSensor.startRanging(); // Start once
}

void getL4SensorData(void){
  if (l4DataPeriodMs == 0) return; // TODO: How to adjust measurement rate
  if (distanceSensor.checkForDataReady()) {

    byte rangeStatus = distanceSensor.getRangeStatus();
    unsigned int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();

    unsigned int tSignalRate = distanceSensor.getSignalRate();
    unsigned int tAmbientRate = distanceSensor.getAmbientRate();

    // NULL distance when status is not good
    if (rangeStatus != 0) distance = 0;

    Serial.print("L4 ");
    Serial.print(rangeStatus);
    Serial.print(' ');
    Serial.print(distance);
    Serial.print(' ');
    Serial.print(tSignalRate / 100);
    Serial.print(' ');
    Serial.println(tAmbientRate / 100);
  }
}

////////////////////////////////////////////////////
// VL53L5CX 8x8 TOF sensor, set of 4 for obstical and can detection
uint8_t status[4][64];
uint16_t distance_mm[4][64];
uint16_t sigma[4][64];
uint8_t reflect[4][64];
bool dataRdy[4] = {false,false,false,false};
uint8_t reflVal = 25;
uint16_t sigmVal = 10;

enum {
  serialMode_ROS2,
  serialMode_PLOT
};

int serialMode = serialMode_PLOT;
//int serialMode = serialMode_ROS2;

#define resolution (8*8) /*8*8 or 4*4 */
#define L5_DATARATE_HZ 10
int l5DataPeriodMs = 0;
void cfgL5DataRate(int hz){
  if(hz>0 && hz<=30) l5DataPeriodMs = 1000/hz;
  else l5DataPeriodMs = 0;
}

void initL5SensorPins() {
  pinMode(pinTofLPLL, OUTPUT);
  digitalWrite(pinTofLPLL, LOW);
  pinMode(pinTofLPLR, OUTPUT);
  digitalWrite(pinTofLPLR, LOW);
  pinMode(pinTofLPRL, OUTPUT);
  digitalWrite(pinTofLPRL, LOW);
  pinMode(pinTofLPRR, OUTPUT);
  digitalWrite(pinTofLPRR, LOW);
}

void initL5Sensor(SparkFun_VL53L5CX *myImager, int addr, int pinTofLP) {
  // enable I2C on this sensor
  digitalWrite(pinTofLP, HIGH);
  if (myImager->begin() == false) {
    Serial.printf("L5 addr "); Serial.print(addr,HEX);
    Serial.println(F(" Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  myImager->setAddress(addr);
  myImager->setRangingFrequency(L5_DATARATE_HZ);
  myImager->setResolution(resolution); // 8*8 or 4*4
  myImager->setSharpenerPercent(sharpenerPct);
  myImager->setIntegrationTime(integrationTimeMsec);
}

void initL5Sensors() {
  Serial.println("Initializing sensor boards. This can take up to 10s. Please wait.");

  cfgL5DataRate(L5_DATARATE_HZ);

  initL5Sensor(&myImagerLL, i2cTofAddrLL, pinTofLPLL);
  initL5Sensor(&myImagerLR, i2cTofAddrLR, pinTofLPLR);
  initL5Sensor(&myImagerRL, i2cTofAddrRL, pinTofLPRL);
  initL5Sensor(&myImagerRR, i2cTofAddrRR, pinTofLPRR);

  // assume all sensors are configured the same
  imageResolution = myImagerLL.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImagerLL.startRanging();
  myImagerLR.startRanging();
  myImagerRL.startRanging();
  myImagerRR.startRanging();

  Serial.println("TOF8x8 sensors initialized OK");
}


// returns data ready
bool getL5SensorData(SparkFun_VL53L5CX *myImager, int n) {
  bool rdy = false;

  //Read distance data into arrays when ready
  if (myImager->isDataReady() == true) {
    if (myImager->getRangingData(&measurementData)) { 
      for(int i=0; i<imageResolution; i++) {
        status[n][i] =  measurementData.target_status[i];
        distance_mm[n][i] =  measurementData.distance_mm[i];
        sigma[n][i] = measurementData.range_sigma_mm[i];
        reflect[n][i] = measurementData.reflectance[i];
      }
      rdy = true;
    }
  }

  delay(5); //?? Small delay between polling ??
  return(rdy);
}

void getL5SensorsData(void) {
  if (l5DataPeriodMs==0) return;

  if(!dataRdy[0]) dataRdy[0] = getL5SensorData(&myImagerLL, 0);
  if(!dataRdy[1]) dataRdy[1] = getL5SensorData(&myImagerLR, 1);
  if(!dataRdy[2]) dataRdy[2] = getL5SensorData(&myImagerRL, 2);
  if(!dataRdy[3]) dataRdy[3] = getL5SensorData(&myImagerRR, 3);

  // Send frame of sensor data when all 4 sets are ready
  if (dataRdy[0]&dataRdy[1]&dataRdy[2]&dataRdy[3]) {
    //The ST library returns the data transposed from zone mapping shown in datasheet
    int xmax = imageWidth - 1;
    int ymax = imageWidth * xmax;
    uint16_t g,r;
    int s;
    int d3,d4,d;
    int xy;
    int y = 3*8; // sensor row 3
    if(serialMode == serialMode_ROS2) {
      Serial.print("L5 ");
      for(int n=0;n<4;n++) { // 0, 1, 2, 3
        for (int x=0;x<8;x++) {
          // average each distance in rows 3 and 4
          xy=x+y; // row 3
          g = sigma[n][xy];
          r = reflect[n][xy];
          s = status[n][xy];
          if (s==5) d3 = distance_mm[n][xy];
          else d3=-1;
          xy=xy+8; // row 4
          g = sigma[n][xy];
          r = reflect[n][xy];
          s = status[n][xy];
          if (s==5) d4 = distance_mm[n][xy];
          else d4=-1;
          if(d3!=-1 && d4!=-1) d = (d3+d4)/2;
          else if (d3!=0) d = d3;
          else if (d4!=0) d = d4;
          else d = -1;
            Serial.print(d); Serial.print(" ");
        }
      }
      Serial.println(); 
    }
    else { // MODE PLOT
      Serial.print("|");
      for(int n=0;n<4;n++) { // 0,1,2,3
        for(int x=0; x<8; x++) {
          // average each distance in rows 3 and 4
          xy=x+y; // row 3
          g = sigma[n][xy];
          r = reflect[n][xy];
          s = status[n][xy];
          if (s==5) d3 = distance_mm[n][xy];
          else d3=-1;
          xy=xy+8; // row 4
          g = sigma[n][xy];
          r = reflect[n][xy];
          s = status[n][xy];
          if (s==5) d4 = distance_mm[n][xy];
          else d4=-1;
          if(d3!=-1 && d4!=-1) d = (d3+d4)/2;
          else if (d3!=0) d = d3;
          else if (d4!=0) d = d4;
          else d = -1;
          if (d!=-1) {
            char c = '.';
            if(d<=100)       c = '0';
            else if(d<=200)  c = '1';
            else if(d<=500)  c = '2';
            else if(d<=1000) c = '3';
            else if(d<=1500) c = '4';
            else if(d<=2000) c = '5';
            else if(d<=2500) c = '6';
            else if(d<=3000) c = '7';
            else if(d<=3500) c = '8';
            else             c = '.';
            Serial.print(c);
          }
          else Serial.print(" "); 
        }
        Serial.print("|");
      }
      Serial.println();      
    }

    dataRdy[0] = false;
    dataRdy[1] = false;
    dataRdy[2] = false;
    dataRdy[3] = false;
  }
}


void setup()
{
  pinMode(pinTofLED, OUTPUT);
  digitalWrite(pinTofLED, HIGH);

  Serial.begin(2000000);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Front Sensors Module");

  init3v3Regulator();

  // init gpio pins for VL53L5CX sensor management
  initL5SensorPins();

  // Init I2C Wire interface for TOF8x8 sensors
  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(1000000); 

  // Init I2C Wire interface #2 for TOF1X, pwr monitor and IMU
  Wire1.setSDA(pinSDA_1);
  Wire1.setSCL(pinSCL_1);
  Wire1.begin(); //This resets to 100kHz I2C
  Wire1.setClock(400000); //Sensor IMU has max I2C freq of 400kHz 

  initL5Sensors();

  initL4Sensor();

  initImuSensor();

  initINA228();

}

// Process commands from Main Controller (normally ROS2)
void getSerialCommands() {
  if (Serial.available() > 0) {
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');

    char str[100];
    int val;

    if(sscanf(incomingString.c_str(), "MODE %s", &str) == 1){
      if(strncmp("ROS2", str, 4)==0) serialMode = serialMode_ROS2;
      if(strncmp("PLOT", str, 4)==0) serialMode = serialMode_PLOT;
      return;
    }
    if(sscanf(incomingString.c_str(), "REFL %d", &val) == 1){
      reflVal = val;
      return;
    }
    if(sscanf(incomingString.c_str(), "SIGM %d", &val) == 1){
      sigmVal = val;
      return;
    }
    if(sscanf(incomingString.c_str(), "IDHZ %d", &val) == 1){
      cfgImuDataRate(val);
      return;
    }
    if(sscanf(incomingString.c_str(), "ICHZ %d", &val) == 1){
      cfgImuCalRate(val);
      return;
    }
    if(sscanf(incomingString.c_str(), "L4HZ %d", &val) == 1){
      cfgL4DataRate(val);
      return;
    }
    if(sscanf(incomingString.c_str(), "L5HZ %d", &val) == 1){
      cfgL5DataRate(val);
      return;
    }
    if(sscanf(incomingString.c_str(), "BTHZ %d", &val) == 1){
      cfgBatDataRate(val);
      return;
    }
  }
}

void loop()
{

  getSerialCommands();

  getInaSensorData();
  
  getImuSensorData();
  getImuCalStatus();

  getL4SensorData();

  getL5SensorsData();
}
